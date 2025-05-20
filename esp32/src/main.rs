mod esp32;

slint::include_modules!();

use log::info;

type Wifi = esp_idf_svc::wifi::BlockingWifi<esp_idf_svc::wifi::EspWifi<'static>>;

pub struct Model {
    wifi: std::rc::Rc<std::cell::RefCell<Wifi>>,
}

impl slint_workshop_model::WifiNetworkProvider for Model {
    fn scan_wifi_networks(&self) -> Vec<slint_workshop_model::WifiNetwork> {
        let mut wifi = self.wifi.borrow_mut();

        if let Err(e) = wifi.start() {
            log::error!("Failed to start WiFi: {:?}", e);
            return vec![];
        }

        match wifi.scan() {
            Ok(aps) => aps
                .iter()
                .map(|access_point| slint_workshop_model::WifiNetwork {
                    ssid: access_point.ssid.to_string(),
                })
                .collect(),
            Err(e) => {
                log::error!("WiFi scan failed: {:?}", e);
                vec![]
            }
        }
    }
}

/// Our App struct that holds the UI
struct App {
    ui: MainWindow,
    model: Model,
    wifi_model: std::rc::Rc<slint::VecModel<WifiNetwork>>,
}

impl App {
    /// Create a new App struct.
    ///
    /// The App struct initializes the UI and the Wifi.
    fn new(wifi: std::rc::Rc<std::cell::RefCell<Wifi>>) -> anyhow::Result<Self> {
        // Make a new MainWindow
        let ui = MainWindow::new().map_err(|e| anyhow::anyhow!(e))?;
        let wifi_model = std::rc::Rc::new(slint::VecModel::<WifiNetwork>::from(vec![]));

        // Return the App struct
        Ok(Self {
            ui,
            model: Model { wifi },
            wifi_model,
        })
    }

    /// Run the App
    fn run(self) -> anyhow::Result<()> {
        use slint_workshop_model::WifiNetworkProvider;

        let view_model = self.wifi_model.clone();
        self.ui.set_wifi_network_model(view_model.clone().into());

        self.ui.on_wifi_refresh(move || {
            view_model.set_vec(
                self.model
                    .scan_wifi_networks()
                    .iter()
                    .map(|wifi| {
                        println!("{}", wifi.ssid);
                        WifiNetwork {
                            ssid: wifi.ssid.clone().into(),
                        }
                    })
                    .collect::<Vec<WifiNetwork>>(),
            );
        });

        self.ui.invoke_wifi_refresh();

        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| anyhow::anyhow!(e))
    }
}

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let platform = esp32::EspPlatform::new();

    let wifi = platform.wifi.clone();

    // Set the platform
    slint::platform::set_platform(platform).unwrap();

    let app = App::new(wifi.clone())?;

    app.run()
}
