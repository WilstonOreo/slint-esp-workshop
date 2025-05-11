// Prevent console window in addition to Slint window in Windows release builds when, e.g., starting the app via file manager. Ignored on other platforms.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::rc::Rc;

use slint_workshop_model::WifiNetworkProvider;

slint::include_modules!();

/// Our App struct that holds the UI
struct App {
    ui: MainWindow,
    model: slint_workshop_model::Model,
    wifi_model: std::rc::Rc<slint::VecModel<WifiNetwork>>,
}

impl App {
    /// Create a new App struct.
    ///
    /// The App struct initializes the UI and the models.
    fn new() -> anyhow::Result<Self> {
        // Make a new AppWindow
        let ui = MainWindow::new()?;

        let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(vec![WifiNetwork {
            ssid: "Test".into(),
        }]));

        Ok(Self {
            ui,
            model: slint_workshop_model::Model,
            wifi_model,
        })
    }

    /// Run the App
    fn run(self) -> anyhow::Result<()> {
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

        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| e.into())
    }
}

/// A minimal main function that initializes the App and runs it.
fn main() -> anyhow::Result<()> {
    env_logger::init();

    let app = App::new()?;

    app.run()
}
