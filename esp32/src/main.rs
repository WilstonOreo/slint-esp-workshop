use slint_workshop_common::ValueStore;
use std::cell::RefCell;
use std::time::{Duration, Instant};

mod esp32;

slint::include_modules!();

use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};

use esp_idf_svc::hal::prelude::Peripherals;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::wifi::{BlockingWifi, EspWifi};
use esp_idf_svc::{eventloop::EspSystemEventLoop, nvs::EspDefaultNvsPartition};

use log::info;

const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");

/// Our App struct that holds the UI
struct App {
    ui: MainWindow,
}

impl App {
    /// Create a new App struct.
    ///
    /// The App struct initializes the UI and the weather controller.
    fn new() -> anyhow::Result<Self> {
        // Make a new MainWindow
        let ui = MainWindow::new().map_err(|e| anyhow::anyhow!(e))?;

        // Return the App struct
        Ok(Self { ui })
    }

    /// Run the App
    fn run(&mut self) -> anyhow::Result<()> {
        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| anyhow::anyhow!(e))
    }
}

fn connect_wifi(wifi: &mut BlockingWifi<EspWifi<'static>>) -> anyhow::Result<()> {
    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.try_into().unwrap(),
        channel: None,
        ..Default::default()
    });

    wifi.set_configuration(&wifi_configuration)?;

    wifi.start()?;
    info!("Wifi started");

    //wifi.connect()?;
    //info!("Wifi connected");

    let access_points = wifi.scan()?;
    for access_point in access_points {
        info!("Access point: {}", access_point.ssid);
    }

    wifi.wait_netif_up()?;
    info!("Wifi netif up");

    Ok(())
}

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut platform = esp32::EspPlatform::new();

    //connect_wifi(&mut platform.wifi)?;
    //let ip_info = platform.wifi.wifi().sta_netif().get_ip_info()?;
    //info!("Wifi DHCP info: {:?}", ip_info);

    // Set the platform
    slint::platform::set_platform(platform).unwrap();

    let mut app = App::new()?;

    app.run()
}
