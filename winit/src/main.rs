// Prevent console window in addition to Slint window in Windows release builds when, e.g., starting the app via file manager. Ignored on other platforms.
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

slint::include_modules!();

use slint_workshop_common::weather::{CityData, DummyWeatherController, OpenWeatherController, WeatherControllerPointer, WeatherControllerSharedPointer};


/// Our App struct that holds the UI and the weather controller.
/// It also holds a timer that fetches the weather data every 2 seconds.
/// 
/// The App struct is responsible for initializing the UI and the weather controller.
/// It also starts the timer and fetches the weather data.
/// 
/// The weather data is then converted into a WeatherRecord and added to the records.
/// The records are then displayed in the UI.
/// 
/// Generally, I recommend to have an App struct in a Slint application that initializes the UI and the controllers.
struct App {
    ui: AppWindow,
    weather_controller: WeatherControllerSharedPointer,
    timer: slint::Timer,
    records: std::rc::Rc<slint::VecModel<WeatherRecord>>,
}


impl App {
    const TIMER_INTERVAL: std::time::Duration = std::time::Duration::from_secs(5);

    /// Create a new App struct.
    /// 
    /// The App struct initializes the UI and the weather controller.
    fn new() -> anyhow::Result<Self> {        
        // Make a new AppWindow
        let ui = AppWindow::new()?;

        // Create a shared weather controller.
        // If the OPEN_WEATHER_API_KEY environment variable is set, use the OpenWeatherController, otherwise use the DummyWeatherController.
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
            Box::new(DummyWeatherController::new()?)
        };

        // The weather controller is shared between the UI and the timer, so we wrap it in an Arc<Mutex>.
        let weather_controller = Arc::new(Mutex::new(data_controller));

        // Create a shared model for the weather records
        let records: std::rc::Rc<slint::VecModel<WeatherRecord>> = std::rc::Rc::default();
        
        // Initialize the view model with the records
        let model = slint::ModelRc::from(records.clone());
        ui.global::<ViewModel>().set_records(model);

        // Return the App struct
        Ok(Self {
            ui,
            weather_controller,
            timer: slint::Timer::default(),
            records,
        })
    }

    /// Run the App, start the timer and fetch the weather data periodically.
    /// 
    /// The run method starts the timer and fetches the weather data every 2 seconds.
    fn run(&mut self) -> anyhow::Result<()> {
        // Get the handle to the UI as a weak reference.
        let ui_handle = self.ui.as_weak();

        // Get the weather controller and the records, because we need to access them in the timer closure.
        let weather_controller = self.weather_controller.clone();

        // Clone the records, because we need to access them in the timer closure.
        let records = self.records.clone();

        // Start the timer with a 5 second interval.
        self.timer.start(
            slint::TimerMode::Repeated,
            Self::TIMER_INTERVAL,
            move || {
                let ui = ui_handle.unwrap();
                let model = ViewModel::get(&ui);
                
                // Fetch the current weather data from the weather controller.
                let current_data = weather_controller.lock().unwrap().current_data().unwrap();

                // Convert the weather data into a WeatherRecord and add it to the records.
                let record: WeatherRecord = current_data.into();
                records.push(record.clone());

                // Set the current record in the view model.
                model.set_current(record);
            },
        );

        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| e.into())
    }
}


/// Convert the weather data into a weather record.
impl From<slint_workshop_common::weather::WeatherData> for WeatherRecord {
    fn from(data: slint_workshop_common::weather::WeatherData) -> Self {
        Self {
            // Convert the temperature to Celsius
            temperature_celsius: data.current_temperature as f32,
            // Convert the humidity to a percentage
            humidity_percent: data.current_humidity as f32,
            // Set current system time as the timestamp
            timestamp: slint::SharedString::from(chrono::Local::now().format("%Y-%m-%d %H:%M:%S").to_string())
        }
    }
}


/// A minimal main function that initializes the App and runs it.
fn main() -> anyhow::Result<()> {
    env_logger::init();
    
    let mut app = App::new()?;

    app.run()
}

