// Prevent console window in addition to Slint window in Windows release builds when, e.g., starting the app via file manager. Ignored on other platforms.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use slint_workshop_model::WifiNetworkProvider;

slint::include_modules!();

/// Our App struct that holds the UI
struct App {
    ui: MainWindow,
    model: slint_workshop_model::Model,
}

impl App {
    /// Create a new App struct.
    ///
    /// The App struct initializes the UI and the models.
    fn new() -> anyhow::Result<Self> {
        // Make a new AppWindow
        let ui = MainWindow::new()?;

        Ok(Self {
            ui,
            model: slint_workshop_model::Model,
        })
    }

    /// Run the App
    fn run(self) -> anyhow::Result<()> {
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
