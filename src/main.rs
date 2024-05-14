extern crate alloc;

use alloc::rc::Rc;

slint::include_modules!();

fn check_for_touch_event() -> Option<slint::platform::WindowEvent> {
    todo!()
}
mod hal {
    pub struct Timer();
    impl Timer {
        pub fn get_time(&self) -> u64 {
            todo!()
        }
    }

    pub fn wfi() {}
}

use slint::platform::{software_renderer::MinimalSoftwareWindow, Platform, WindowEvent};

fn is_swap_pending() -> bool {
    false
}
fn swap_buffers() {}

fn create_slint_app() -> AppWindow {
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();
    ui.on_request_increase_value(move || {
        let ui = ui_handle.unwrap();
        ui.set_counter(ui.get_counter() + 1);
    });

    ui
}

struct Esp32Platform {
    window: Rc<MinimalSoftwareWindow>,
    // optional: some timer device from your device's HAL crate
    timer: hal::Timer,
    // ... maybe more devices
}

impl Platform for Esp32Platform {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        // Since on MCUs, there can be only one window, just return a clone of self.window.
        // We'll also use the same window in the event loop.
        Ok(self.window.clone())
    }
    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_micros(self.timer.get_time())
    }
    // optional: You can put the event loop there, or in the main function, see later
    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        todo!();
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Configure platform for Slint
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(Default::default());

    slint::platform::set_platform(alloc::boxed::Box::new(Esp32Platform {
        window: window.clone(),
        timer: hal::Timer(),
    }))
    .unwrap();

    const DISPLAY_WIDTH: usize = 320;
    const DISPLAY_HEIGHT: usize = 240;

    use slint::platform::software_renderer::Rgb565Pixel;
    let mut buffer1 = [Rgb565Pixel(0); DISPLAY_WIDTH * DISPLAY_HEIGHT];
    let mut buffer2 = [Rgb565Pixel(0); DISPLAY_WIDTH * DISPLAY_HEIGHT];

    // ... configure the screen driver to use buffer1 or buffer2 ...

    // ... rest of initialization ...

    let mut currently_displayed_buffer: &mut [_] = &mut buffer1;
    let mut work_buffer: &mut [_] = &mut buffer2;

    log::info!("Hello, world!");
    loop {
        // Let Slint run the timer hooks and update animations.
        slint::platform::update_timers_and_animations();

        // Check the touch screen or input device using your driver.
        if let Some(event) = check_for_touch_event(/*...*/) {
            // convert the event from the driver into a `slint::platform::WindowEvent`
            // and pass it to the window.
            window.dispatch_event(event);
        }

        // ... maybe some more application logic ...

        // Draw the scene if something needs to be drawn.
        window.draw_if_needed(|renderer| {
            // The screen driver might be taking some time to do the swap. We need to wait until
            // work_buffer is ready to be written in
            while is_swap_pending() {}

            // Do the rendering!
            renderer.render(work_buffer, DISPLAY_WIDTH);

            // tell the screen driver to display the other buffer.
            swap_buffers();

            // Swap the buffer references for our next iteration
            // (this just swap the reference, not the actual data)
            core::mem::swap::<&mut [_]>(&mut work_buffer, &mut currently_displayed_buffer);
        });

        // Try to put the MCU to sleep
        if !window.has_active_animations() {
            if let Some(duration) = slint::platform::duration_until_next_timer_update() {
                // ... schedule a timer interrupt in `duration` ...
            }
            hal::wfi(); // Wait for interrupt
        }
    }
}
