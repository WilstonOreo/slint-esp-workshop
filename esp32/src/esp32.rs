extern crate alloc;

pub struct EspPlatform {
    display_width: usize,
    display_height: usize,
    panel_handle: esp_idf_svc::hal::sys::esp_lcd_panel_handle_t,
    touch_driver: core::cell::RefCell<
        sitronix_touch::TouchIC<
            embedded_hal_bus::i2c::RcDevice<esp_idf_svc::hal::i2c::I2cDriver<'static>>,
        >,
    >,
    window: alloc::rc::Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    timer: esp_idf_svc::timer::EspTimerService<esp_idf_svc::timer::Task>,
}

impl EspPlatform {
    /// Create a new instance of the platform
    pub fn new() -> std::boxed::Box<Self> {
        use esp_idf_svc::hal::prelude::*;
        use esp_idf_svc::hal::sys::*;

        /* Initialize I2C (for touch and audio) */

        let p = Peripherals::take().unwrap();

        let mut touch_i2c = esp_idf_svc::hal::i2c::I2cDriver::new(
            p.i2c0,
            p.pins.gpio1,
            p.pins.gpio41,
            &esp_idf_svc::hal::i2c::config::Config::new().baudrate(400_000.Hz()),
        )
        .unwrap();

        let touch_i2c = alloc::rc::Rc::new(core::cell::RefCell::new(touch_i2c));

        let mut eeprom = eeprom24x::Eeprom24x::new_24x01(
            embedded_hal_bus::i2c::RcDevice::new(touch_i2c.clone()),
            eeprom24x::SlaveAddr::Default,
        );

        let mut eeid = [0u8; 0x1c];
        eeprom.read_data(0x0, &mut eeid).unwrap();

        //log::info!("EEPROM {:x?}", eeid);

        let display_width = u16::from_be_bytes([eeid[8], eeid[9]]) as usize;
        let display_height = u16::from_be_bytes([eeid[10], eeid[11]]) as usize;

        log::info!(
            "Display size: {}x{} /// {}/{}",
            display_width,
            display_height,
            eeid[6],
            eeid[7]
        );

        let pclk_hz = ((eeid[12] as u32) * 1000000 + (eeid[13] as u32) * 100000).min(13600000);
        let hsync_pulse_width = eeid[17] as u32;
        let hsync_back_porch = u16::from_be_bytes([eeid[15], eeid[16]]) as u32;
        let hsync_front_porch = u16::from_be_bytes([eeid[18], eeid[19]]) as u32;
        let vsync_pulse_width = eeid[22] as u32;
        let vsync_back_porch = u16::from_be_bytes([eeid[20], eeid[21]]) as u32;
        let vsync_front_porch = u16::from_be_bytes([eeid[23], eeid[24]]) as u32;

        let hsync_idle_low = (eeid[25] & 0x01) == 0x01;
        let vsync_idle_low = (eeid[25] & 0x02) == 0x02;
        let de_idle_high = (eeid[25] & 0x04) == 0;
        let pclk_active_neg = (eeid[25] & 0x20) == 0x20;
        let pclk_idle_high = false;

        // Initialize LCD panel and touch
        let mut io_handle: esp_lcd_panel_io_handle_t = std::ptr::null_mut();
        let mut panel_handle: esp_lcd_panel_handle_t = std::ptr::null_mut();

        let mut panel_config = esp_lcd_rgb_panel_config_t {
            clk_src: soc_periph_lcd_clk_src_t_LCD_CLK_SRC_PLL240M, //LCD_CLK_SRC_DEFAULT,
            timings: esp_lcd_rgb_timing_t {
                pclk_hz,
                h_res: display_width as u32,
                v_res: display_height as u32,
                hsync_pulse_width,
                hsync_back_porch,
                hsync_front_porch,
                vsync_pulse_width,
                vsync_back_porch,
                vsync_front_porch,
                flags: Default::default(),
            },
            data_width: 16,
            bits_per_pixel: 16,
            num_fbs: 0,
            bounce_buffer_size_px: ((display_width * display_height) * 5) / 100,
            sram_trans_align: 4,
            hsync_gpio_num: 15,
            vsync_gpio_num: 06,
            de_gpio_num: 05,
            pclk_gpio_num: 04,
            disp_gpio_num: 42,
            data_gpio_nums: [9, 17, 46, 16, 7, 8, 21, 3, 11, 18, 10, 14, 20, 13, 19, 12],
            ..Default::default()
        };
        panel_config.__bindgen_anon_1.dma_burst_size = 64;
        panel_config.flags.set_fb_in_psram(1);
        panel_config
            .timings
            .flags
            .set_hsync_idle_low(hsync_idle_low as _);
        panel_config
            .timings
            .flags
            .set_vsync_idle_low(vsync_idle_low as _);
        panel_config
            .timings
            .flags
            .set_de_idle_high(de_idle_high as _);
        panel_config
            .timings
            .flags
            .set_pclk_active_neg(pclk_active_neg as _);
        panel_config
            .timings
            .flags
            .set_pclk_idle_high(pclk_idle_high as _);
        unsafe {
            assert_eq!(
                esp_lcd_new_rgb_panel(&panel_config, &mut panel_handle),
                ESP_OK
            );
            assert_eq!(esp_lcd_panel_init(panel_handle), ESP_OK);

            assert_eq!(
                esp_lcd_rgb_panel_register_event_callbacks(
                    panel_handle,
                    &esp_lcd_rgb_panel_event_callbacks_t {
                        on_color_trans_done: None,
                        on_vsync: Some(vsync_callback),
                        on_bounce_empty: None,
                        ..Default::default()
                    },
                    core::ptr::null_mut()
                ),
                ESP_OK
            );
        }

        let mut touch_driver = sitronix_touch::TouchIC::new_default(
            embedded_hal_bus::i2c::RcDevice::new(touch_i2c.clone()),
        );
        touch_driver.init().unwrap();

        // TODO: enable backlight

        // Setup the window
        let window =
            slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());

        window.set_size(slint::PhysicalSize::new(
            display_width as u32,
            display_height as u32,
        ));

        if display_width > 500 {
            window.dispatch_event(slint::platform::WindowEvent::ScaleFactorChanged {
                scale_factor: 2.0,
            });
            window.dispatch_event(slint::platform::WindowEvent::Resized {
                size: window.size().to_logical(2.0),
            });
        }

        std::boxed::Box::new(Self {
            display_width,
            display_height,
            panel_handle,
            touch_driver: core::cell::RefCell::new(touch_driver),
            window,
            timer: esp_idf_svc::timer::EspTimerService::new().unwrap(),
        })
    }
}

impl slint::platform::Platform for EspPlatform {
    fn create_window_adapter(
        &self,
    ) -> Result<alloc::rc::Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        self.timer.now()
    }
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        use esp_idf_svc::hal::sys::*;

        unsafe {
            // Initialize the LCD panel
            if esp_lcd_panel_init(self.panel_handle) != ESP_OK {
                log::error!("Failed to initialize LCD panel");
                return Err(slint::PlatformError::Other(
                    "Failed to initialize LCD panel".into(),
                ));
            }

            // Turn on the display
            esp_lcd_panel_disp_on_off(self.panel_handle, true);

            // Calling this function rotates the display by 180 degrees
            //esp_lcd_panel_mirror(self.panel_handle, true, true);

            let mut last_position = slint::LogicalPosition::default();
            let mut touch_down = false;

            // Create a buffer to draw the scene
            use slint::platform::software_renderer::Rgb565Pixel;
            let mut buffer = Vec::new();
            buffer.resize(self.display_width * self.display_height, Rgb565Pixel(0x0));

            loop {
                slint::platform::update_timers_and_animations();

                if let Ok(maybe_touch) = self.touch_driver.borrow_mut().get_point0() {
                    if let Some(sitronix_touch::Point {
                        x: touchpad_x,
                        y: touchpad_y,
                    }) = maybe_touch
                    {
                        last_position =
                            slint::LogicalPosition::new(touchpad_x as f32, touchpad_y as f32);

                        // Dispatch the pointer moved event
                        self.window
                            .dispatch_event(slint::platform::WindowEvent::PointerMoved {
                                position: last_position,
                            });

                        if !touch_down {
                            self.window.dispatch_event(
                                slint::platform::WindowEvent::PointerPressed {
                                    position: last_position,
                                    button: slint::platform::PointerEventButton::Left,
                                },
                            );
                        }

                        touch_down = true;
                    } else if touch_down {
                        self.window
                            .dispatch_event(slint::platform::WindowEvent::PointerReleased {
                                position: last_position,
                                button: slint::platform::PointerEventButton::Left,
                            });
                        self.window
                            .dispatch_event(slint::platform::WindowEvent::PointerExited);
                        touch_down = false;
                    }
                }

                // Draw the scene if something needs to be drawn.
                self.window.draw_if_needed(|renderer| {
                    while !VSYNC.load(core::sync::atomic::Ordering::SeqCst) {
                        esp_idf_svc::hal::task::do_yield();
                    }

                    // Do the rendering!
                    let region = renderer.render(&mut buffer, self.display_width);

                    // Iterate each region to be updated
                    for (o, s) in region.iter() {
                        // This is the magic function that sends the buffer to the display
                        esp_lcd_panel_draw_bitmap(
                            self.panel_handle,
                            o.x,
                            o.y,
                            o.x + s.width as i32,
                            o.y + s.height as i32,
                            buffer
                                .as_ptr()
                                .add((o.y * self.display_width as i32 + o.x) as usize)
                                .cast::<core::ffi::c_void>(),
                        );
                    }

                    VSYNC.store(false, core::sync::atomic::Ordering::SeqCst);
                });

                // Try to put the MCU to sleep
                if !self.window.has_active_animations() {
                    continue;
                }
            }
        }
    }
}

/// Set the brightness of the display
pub fn set_brightness(brightness: f32) {
    unsafe {
        use esp_idf_svc::hal::sys::*;
        let brightness = brightness as i32;
        log::info!("Setting brightness to {}", brightness);
        //bsp_display_brightness_set(brightness);
    }
}

static VSYNC: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

unsafe extern "C" fn vsync_callback(
    _panel: esp_idf_svc::hal::sys::esp_lcd_panel_handle_t,
    _edata: *const esp_idf_svc::hal::sys::esp_lcd_rgb_panel_event_data_t,
    _user_ctx: *mut core::ffi::c_void,
) -> bool {
    VSYNC.store(true, core::sync::atomic::Ordering::SeqCst);
    false
}
