slint::include_modules!();

use slint_workshop_model::WifiNetworkProvider;

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
        let wifi_model = std::rc::Rc::new(slint::VecModel::<WifiNetwork>::from(vec![]));

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

        self.ui.invoke_wifi_refresh();

        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| e.into())
    }
}

#[cfg_attr(target_arch = "wasm32", wasm_bindgen::prelude::wasm_bindgen(start))]
pub fn main() {
    let app = App::new().unwrap();

    app.run().unwrap();
}
