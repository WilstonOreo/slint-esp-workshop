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
        Ok(self.ui.run()?)
    }
}

#[no_mangle]
fn android_main(app: slint::android::AndroidApp) {
    slint::android::init(app).expect("Slint Android initialization failed");

    let mut app = App::new().expect("Error creating the app");

    app.run().expect("Error running the app");
}
