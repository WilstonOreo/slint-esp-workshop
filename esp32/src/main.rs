use std::time::{Duration, Instant};
use slint_workshop_common::ValueStore;

mod dht22;
mod esp32;

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
        // Make a new MainWindow
        let ui = MainWindow::new().map_err(|e| anyhow::anyhow!(e))?;

        // Return the App struct
        Ok(Self {
            ui,
        })
    }

    /// Run the App
    fn run(&mut self) -> anyhow::Result<()> {
        // Run the UI (and map an error to an anyhow::Error).
        self.ui.run().map_err(|e| anyhow::anyhow!(e))
    }
}


/// The struct that stores the sensor data.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SensorData {
    /// The temperature in Celsius, read from the DHT22 sensor.
    temperature_celsius: f32,

    /// The humidity in percent, read from the DHT22 sensor.
    humidity_percent: f32,

    /// The time when the data was read.
    when: std::time::Duration,

    /// The status of the sensor.
    status: SensorStatus,
}


fn dht_task(last: ValueStore<SensorData>) {
    let pin = 13;
    let dht = dht22::DHT22::new(pin);

    loop {
        // Implement the logic to handle the sensor data
        std::thread::sleep(Duration::from_millis(2000));
    }
}

fn main() -> anyhow::Result<()> {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Set the platform
    slint::platform::set_platform(esp32::EspPlatform::new()).unwrap();

    // Launch the DHT task in a separate thread
    let last_sensor_data = ValueStore::<SensorData>::default();
    let last_for_dht_task = last_sensor_data.clone();
    std::thread::spawn(move || dht_task(last_sensor_data));

    let mut app = App::new()?;

    app.run()
}
