extern crate alloc;

use esp_idf_svc::hal::sys::*;

const DISPLAY_WIDTH: usize = 320;
const DISPLAY_HEIGHT: usize = 240;
const DRAW_BUFFER_SIZE: usize = DISPLAY_WIDTH * DISPLAY_HEIGHT; // @todo find constants

struct EspPlatform {
    size: slint::PhysicalSize,
    panel_handle: esp_lcd_panel_handle_t,
    touch_handle: Option<*mut esp_lcd_touch_s>,
    window: Rc<MinimalSoftwareWindow>,
}

impl Platform for EspPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        unsafe {
            let ticks = xTaskGetTickCount(); // @fixme Are ticks in microseconds?
            core::time::Duration::from_millis(ticks as u64)
        }
    }
    // optional: You can put the event loop there, or in the main function, see later
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        unsafe {
            if esp_lcd_panel_init(self.panel_handle) != ESP_OK {
                log::error!("Failed to initialize LCD panel");
                return Err(slint::PlatformError::Other(
                    "Failed to initialize LCD panel".into(),
                ));
            }

            esp_lcd_panel_disp_on_off(self.panel_handle, true);
            esp_lcd_panel_mirror(self.panel_handle, true, false);

            let mut last_touch_x = 0;
            let mut last_touch_y = 0;
            let mut touch_down = false;

            use slint::platform::software_renderer::Rgb565Pixel;
            let mut buffer = vec![Rgb565Pixel(0xf800); DRAW_BUFFER_SIZE];

            loop {
                slint::platform::update_timers_and_animations();

                if let Some(touch_handle) = self.touch_handle {
                    let mut touchpad_x = [0];
                    let mut touchpad_y = [0];
                    let mut touchpad_cnt = [0 as u8];

                    esp_lcd_touch_read_data(touch_handle);

                    let touchpad_pressed = esp_lcd_touch_get_coordinates(
                        touch_handle,
                        touchpad_x.as_mut_ptr(),
                        touchpad_y.as_mut_ptr(),
                        std::ptr::null_mut(),
                        touchpad_cnt.as_mut_ptr(),
                        1,
                    );

                    if touchpad_pressed && touchpad_cnt[0] > 0 {
                        last_touch_x = touchpad_x[0] as i32;
                        last_touch_y = touchpad_y[0] as i32;

                        self.window
                            .dispatch_event(slint::platform::WindowEvent::PointerMoved {
                                position: slint::LogicalPosition::new(
                                    last_touch_x as f32,
                                    last_touch_y as f32,
                                ),
                            });

                        if !touch_down {
                            self.window.dispatch_event(
                                slint::platform::WindowEvent::PointerPressed {
                                    position: slint::LogicalPosition::new(
                                        last_touch_x as f32,
                                        last_touch_y as f32,
                                    ),
                                    button: slint::platform::PointerEventButton::Left,
                                },
                            );
                            log::info!("Touchpad pressed: {:?} {:?}", last_touch_x, last_touch_y);
                        }

                        touch_down = true;
                    } else if touch_down {
                        self.window
                            .dispatch_event(slint::platform::WindowEvent::PointerReleased {
                                position: slint::LogicalPosition::new(
                                    last_touch_x as f32,
                                    last_touch_y as f32,
                                ),
                                button: slint::platform::PointerEventButton::Left,
                            });
                        self.window
                            .dispatch_event(slint::platform::WindowEvent::PointerExited);
                        touch_down = false;
                    }

                    // Draw the scene if something needs to be drawn.
                    self.window.draw_if_needed(|renderer| {
                        log::info!("Render frame...");
                        // Do the rendering!
                        let region = renderer.render(&mut buffer, self.size.width as usize);

                        for (o, s) in region.iter() {
                            log::info!("Render region: {:?} {:?}", o, s);
                            for y in o.y..(o.y + s.height as i32) {
                                let flip_y = self.size.height as i32 - y - 1;
                                for x in o.x..(o.x + s.width as i32) {
                                    let offset = (flip_y * self.size.width as i32 +  x) as usize;
                                    let pixel = buffer[offset].0;
                                    // Convert pixel to big endian
                                    let pixel = ((pixel & 0xff) << 8) | ((pixel & 0xff00) >> 8);
                                    buffer[offset].0 = pixel;
                                }

                                use std::ffi::c_void;
                                esp_lcd_panel_draw_bitmap(
                                    self.panel_handle,
                                    o.x,
                                    y,
                                    o.x + s.width as i32,
                                    y + 1,
                                    buffer
                                        .as_ptr()
                                        .add((flip_y * self.size.width as i32 + o.x) as usize)
                                        .cast::<c_void>(),
                                );
                            }
                        }
                    });

                    // Try to put the MCU to sleep
                    if !self.window.has_active_animations() {
                        continue;
                    }
                }
            }
        }
    }
}

use alloc::rc::Rc;

slint::include_modules!();

use slint::platform::{software_renderer::MinimalSoftwareWindow, Platform};

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });

    ui
}

fn main() {
    use esp_idf_svc::hal::sys::*;

    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71

    esp_idf_svc::sys::link_patches();

    /* Initialize I2C (for touch and audio) */
    unsafe {
        bsp_i2c_init();
    }

    let mut io_handle: esp_lcd_panel_io_handle_t = std::ptr::null_mut();
    let mut panel_handle: esp_lcd_panel_handle_t = std::ptr::null_mut();

    let bsp_disp_cfg = bsp_display_config_t {
        max_transfer_sz: (DRAW_BUFFER_SIZE
            * std::mem::size_of::<slint::platform::software_renderer::Rgb565Pixel>())
            as i32,
    };

    let mut touch_handle: *mut esp_lcd_touch_s = std::ptr::null_mut();
    let bsp_touch_cfg = bsp_touch_config_t::default();

    unsafe {
        bsp_display_new(&bsp_disp_cfg, &mut panel_handle, &mut io_handle);
        bsp_touch_new(&bsp_touch_cfg, &mut touch_handle);
        bsp_display_backlight_on();
    }

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Configure platform for Slint
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());
    window.set_size(slint::PhysicalSize::new(
        DISPLAY_WIDTH as u32,
        DISPLAY_HEIGHT as u32,
    ));

    slint::platform::set_platform(alloc::boxed::Box::new(EspPlatform {
        size: slint::PhysicalSize::new(DISPLAY_WIDTH as u32, DISPLAY_HEIGHT as u32),
        panel_handle,
        touch_handle: Some(touch_handle),
        window: window.clone(),
    }))
    .unwrap();
    window.set_fullscreen(true);

    create_slint_app().run().unwrap();
}
