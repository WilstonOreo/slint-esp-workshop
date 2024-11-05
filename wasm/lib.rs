slint::include_modules!();


/// Our App struct that holds the UI
struct App {
    ui: MainWindow,
}

impl App {
    /// Create a new App struct.
    /// 
    /// The App struct initializes the UI and the weather controller.
    fn new() -> anyhow::Result<Self> {        
        // Make a new AppWindow
        let ui = MainWindow::new()?;

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

#[cfg_attr(target_arch = "wasm32",
           wasm_bindgen::prelude::wasm_bindgen(start))]
pub fn main() {
    let mut app = App::new().unwrap();

    app.run().unwrap();
}

