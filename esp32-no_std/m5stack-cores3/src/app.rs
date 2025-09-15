use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;

use embassy_time::{Duration, Ticker};
// Slint platform imports
use slint::platform::software_renderer::Rgb565Pixel;

use crate::dht22;

slint::include_modules!();

pub struct App {
    pub ui: MainWindow,
}

impl App {
    pub fn new() -> Self {
        let ui = MainWindow::new().unwrap();

        // Create empty WiFi network model with some placeholder data
        let placeholder_networks = vec![
            WifiNetwork {
                ssid: "WiFi Scanning...".into(),
            },
            WifiNetwork {
                ssid: "Please wait".into(),
            },
        ];

        let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(placeholder_networks));
        ui.set_wifi_network_model(wifi_model.clone().into());

        // Set up WiFi refresh handler with real WiFi functionality
        ui.on_wifi_refresh(move || {
            log::info!("WiFi refresh requested - checking for scan results");

            // Check if we have new scan results
            if crate::wifi::WIFI_SCAN_UPDATED.load(core::sync::atomic::Ordering::Relaxed) {
                // Access the scan results
                let scan_results = crate::wifi::WIFI_SCAN_RESULTS.try_lock();
                if let Ok(results) = scan_results {
                    let mut networks = alloc::vec::Vec::new();

                    for ap in results.iter() {
                        networks.push(WifiNetwork {
                            ssid: ap.ssid.as_str().into(),
                        });
                    }

                    if networks.is_empty() {
                        networks.push(WifiNetwork {
                            ssid: "No networks found".into(),
                        });
                    }

                    log::info!("Updated UI with {} real networks", networks.len());
                    wifi_model.set_vec(networks);

                    // Reset the update flag
                    crate::wifi::WIFI_SCAN_UPDATED
                        .store(false, core::sync::atomic::Ordering::Relaxed);
                } else {
                    log::info!("Could not access scan results (locked)");
                }
            } else {
                log::info!("No new scan results available");
            }
        });

        // Trigger initial refresh
        ui.invoke_wifi_refresh();

        Self { ui }
    }

    pub fn run(&self) {}
}

// Graphics rendering task - handles display output only
#[embassy_executor::task]
pub async fn ui_update_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    log::info!("=== UI rendering task started ====");
    use crate::display::*;

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
        if crate::wifi::WIFI_SCAN_UPDATED.load(core::sync::atomic::Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                log::debug!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Check for new WiFi scan results and trigger UI refresh if available
        if let Some(main_window) = ui.upgrade() {
            log::debug!("Triggered UI refresh for new weather record");
            let reading = dht22::DHT22_CHANNEL.receive().await;
            log::info!("Update weather UI");
            main_window.set_weather_record(WeatherRecord {
                temperature_celsius: reading.temperature,
                humidity_percent: reading.relative_humidity,
            });
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
