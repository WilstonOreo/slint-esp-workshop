use core::sync::atomic::Ordering;

use alloc::boxed::Box;
use alloc::rc::Rc;

use embassy_time::{Duration, Ticker};
// Slint platform imports
use slint::platform::software_renderer::Rgb565Pixel;

// Display constants for M5Stack CoreS3 - 320x240 ILI9341
const LCD_H_RES: u16 = 320;
const LCD_V_RES: u16 = 240;
const LCD_H_RES_USIZE: usize = 320;
const LCD_V_RES_USIZE: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

slint::include_modules!();

// Graphics rendering task - handles display output only
#[embassy_executor::task]
pub async fn ui_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    log::info!("=== UI rendering task started ====");

    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    let mut frame_counter = 0u32;

    // Create pixel buffer for Slint rendering
    let mut pixel_buffer: Box<[Rgb565Pixel; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565Pixel(0); LCD_BUFFER_SIZE]);

    log::info!("UI task initialized with {LCD_H_RES}x{LCD_V_RES} buffer");

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for new WiFi scan results and trigger UI refresh if available
        if crate::wifi::WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                log::debug!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Render the frame using the hardware display
        let rendered = window.draw_if_needed(|renderer| {
            // Access the global display instance
            if let Some(()) = crate::display::DISPLAY_COMPONENTS.with_mut(|display_hardware| {
                // Create hardware draw buffer
                let mut hardware_buffer = crate::display::HardwareDrawBuffer::new(
                    &mut display_hardware.display,
                    &mut *pixel_buffer,
                );

                // Render by line to the hardware display
                renderer.render_by_line(&mut hardware_buffer);

                if frame_counter % 60 == 0 {
                    log::debug!("Frame {frame_counter} rendered to hardware display");
                }
            }) {
                // Successfully rendered
            } else {
                log::error!("Display not available in graphics task!");
            }
        });

        // If a frame was rendered, log it
        if rendered && frame_counter % 60 == 0 {
            log::debug!("Frame {frame_counter} rendered and displayed on M5Stack CoreS3");
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            log::info!("UI: Frame {frame_counter}, M5Stack CoreS3 display active");
        }

        ticker.next().await;
    }
}

pub struct EspEmbassyBackend {
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl EspEmbassyBackend {
    pub fn new(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl slint::platform::Platform for EspEmbassyBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        embassy_time::Instant::now()
            .duration_since(embassy_time::Instant::from_secs(0))
            .into()
    }
}
