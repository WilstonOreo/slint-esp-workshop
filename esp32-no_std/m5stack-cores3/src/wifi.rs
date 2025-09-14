use core::sync::atomic::AtomicBool;

use alloc::string::String;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use esp_hal::rng::Rng;
use esp_hal::timer::timg::TimerGroup;
use esp_wifi::EspWifiController;
use esp_wifi::wifi::{AccessPointInfo, ClientConfiguration, Configuration, WifiController};

// Shared state for WiFi scan results
pub static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
pub static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// WiFi scanning task
#[embassy_executor::task]
pub async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    log::info!("=== WiFi scan task started ====");

    // Start WiFi
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::new(),
        password: String::new(),
        ..Default::default()
    });

    match wifi_controller.set_configuration(&client_config) {
        Ok(_) => log::info!("WiFi configuration set successfully"),
        Err(e) => log::info!("Failed to set WiFi configuration: {e:?}"),
    }

    match wifi_controller.start_async().await {
        Ok(_) => log::info!("WiFi started successfully!"),
        Err(e) => log::info!("Failed to start WiFi: {e:?}"),
    }

    // Wait a bit for WiFi to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    loop {
        log::info!("Performing WiFi scan...");

        match wifi_controller.scan_n_async(10).await {
            Ok(results) => {
                log::info!("Found {} networks:", results.len());
                for (i, ap) in results.iter().enumerate() {
                    log::info!(
                        "  {}: SSID: {}, Signal: {:?}, Auth: {:?}, Channel: {}",
                        i + 1,
                        ap.ssid.as_str(),
                        ap.signal_strength,
                        ap.auth_method,
                        ap.channel
                    );
                }

                // Store scan results in shared state
                if let Ok(mut scan_results) = WIFI_SCAN_RESULTS.try_lock() {
                    scan_results.clear();
                    scan_results.extend_from_slice(&results);
                    WIFI_SCAN_UPDATED.store(true, core::sync::atomic::Ordering::Relaxed);
                    log::info!("Stored {} scan results for UI", scan_results.len());
                } else {
                    log::info!("Could not store scan results (mutex locked)");
                }
            }
            Err(e) => {
                log::info!("WiFi scan failed: {e:?}");
            }
        }

        // Wait 10 seconds before next scan
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

pub fn create_wifi_controller<T: esp_hal::timer::timg::TimerGroupInstance>(
    timer: TimerGroup<'static, T>,
    rng: Rng,
) -> &'static EspWifiController<'static> {
    &*mk_static!(
        esp_wifi::EspWifiController<'static>,
        esp_wifi::init(timer.timer0, rng).expect("Failed to initialize WiFi")
    )
}
