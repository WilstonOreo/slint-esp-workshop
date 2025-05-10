mod esp32;

slint::include_modules!();

use log::{error, info};

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

/*
fn connect_wifi(
    wifi: &mut esp_idf_svc::wifi::BlockingWifi<esp_idf_svc::wifi::EspWifi<'static>>,
) -> anyhow::Result<()> {
    use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};

    const SSID: &str = env!("WIFI_SSID");
    const PASSWORD: &str = env!("WIFI_PASS");

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

    let access_points = wifi.scan()?;
    for access_point in access_points {
        info!("Access point: {}", access_point.ssid);
    }
    match wifi.connect() {
        Ok(_) => {
            info!("Wifi connected");
        }
        Err(e) => {
            error!("Wifi connection error: {e}");
        }
    }

    //wifi.wait_netif_up()?;
    //info!("Wifi netif up");

    Ok(())
}
*/

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let mut platform = esp32::EspPlatform::new();

    //connect_wifi(&mut platform.wifi)?;

    // Set the platform
    slint::platform::set_platform(platform).unwrap();

    let mut app = App::new()?;

    app.run()
}
