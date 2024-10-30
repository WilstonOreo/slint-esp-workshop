// Prevent console window in addition to Slint window in Windows release builds when, e.g., starting the app via file manager. Ignored on other platforms.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::error::Error;

slint::include_modules!();

use slint_workshop_common::weather::{CityData, DummyWeatherController, OpenWeatherController, WeatherControllerPointer, WeatherControllerSharedPointer};


struct App {
    ui: AppWindow,
    weather_controller: WeatherControllerSharedPointer,
    timer: slint::Timer,
    records: std::rc::Rc<slint::VecModel<WeatherRecord>>,
}

impl From<slint_workshop_common::weather::WeatherData> for WeatherRecord {
    fn from(data: slint_workshop_common::weather::WeatherData) -> Self {
        Self {
            temperature_celsius: data.current_temperature as f32,
            humidity_percent: data.current_humidity as f32,
            timestamp: slint::SharedString::new(),
        }
    }
}

impl App {
    fn new() -> Result<Self, Box<dyn Error>> {
        let ui = AppWindow::new()?;
        use std::sync::{Arc, Mutex};
        let data_controller: WeatherControllerPointer = if let Some(api_key) = std::option_env!("OPEN_WEATHER_API_KEY") {
            Box::new(OpenWeatherController::new(
                CityData {
                    city_name: "Florence".into(),
                    lat: 43.77,
                    lon: 11.25,
                },
            api_key.into()))
        } else {
            Box::new(DummyWeatherController::new().unwrap())
        };

        // Create a shared weather controller
        let weather_controller = Arc::new(Mutex::new(data_controller));

        // Create a shared model for the weather records
        let records: std::rc::Rc<slint::VecModel<WeatherRecord>> = std::rc::Rc::default();
        
        // Initialize the view model with the records
        let model = slint::ModelRc::from(records.clone());
        ui.global::<ViewModel>().set_records(model);

        Ok(Self {
            ui,
            weather_controller,
            timer: slint::Timer::default(),
            records,
        })
    }

    fn run(&mut self) -> Result<(), Box<dyn Error>> {
        let ui_handle = self.ui.as_weak();
        let weather_controller = self.weather_controller.clone();
        let records = self.records.clone();

        self.timer.start(
            slint::TimerMode::Repeated,
            std::time::Duration::from_millis(2000),
            move || {
                let ui = ui_handle.unwrap();
                let model = ViewModel::get(&ui);
                
                let current_data = weather_controller.lock().unwrap().current_data().unwrap();
                let record: WeatherRecord = current_data.into();

                records.push(record.clone());

                model.set_current(record);
            },
        );

        self.ui.run().map_err(|e| e.into())
    }
}



fn main() -> Result<(), Box<dyn Error>> {
    env_logger::Builder::default()
        .filter_level(if cfg!(debug_assertions) {
            log::LevelFilter::Debug
        } else {
            log::LevelFilter::Info
        })
        .init();

    let mut app = App::new()?;

    app.run()
}

