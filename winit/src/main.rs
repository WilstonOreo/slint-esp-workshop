// Prevent console window in addition to Slint window in Windows release builds when, e.g., starting the app via file manager. Ignored on other platforms.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

slint::include_modules!();


/// Our App struct that holds the UI
struct App {
    ui: AppWindow,
}


impl App {
    /// Create a new App struct.
    /// 
    /// The App struct initializes the UI and the weather controller.
    fn new() -> anyhow::Result<Self> {        
        // Make a new AppWindow
        let ui = AppWindow::new()?;

        // Return the App struct
        Ok(Self {
            ui,
        })
    }

    /// Run the App
    fn run(&mut self) -> anyhow::Result<()> {
        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| e.into())
    }
}

/// A minimal main function that initializes the App and runs it.
fn main() -> anyhow::Result<()> {
    env_logger::init();
    
    let mut app = App::new()?;

    app.run()
}

