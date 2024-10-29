// Prevent console window in addition to Slint window in Windows release builds when, e.g., starting the app via file manager. Ignored on other platforms.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::error::Error;

slint::include_modules!();

use slint_workshop_common::weather::WeatherControllerSharedPointer;

struct App {
    ui: AppWindow,
    weather_controller: WeatherControllerSharedPointer,
}

impl App {
    fn new() -> Result<Self, Box<dyn Error>> {
        let ui = AppWindow::new()?;
        let weather_controller = WeatherControllerSharedPointer::new();

        Ok(Self {
            ui,
            weather_controller,
        })
    }

    fn run(&mut self) -> Result<(), Box<dyn Error>> {
        self.weather_controller.load()?;
        self.ui.set_weather_controller(self.weather_controller.clone());

        self.ui.run()?;

        Ok(())
    }
}



fn main() -> Result<(), Box<dyn Error>> {
    let mut app = App::new()?;

    app.run()
}
