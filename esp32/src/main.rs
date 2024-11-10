use std::time::{Duration, Instant};
use slint_workshop_common::ValueStore;

mod dht22;
mod esp32;

slint::include_modules!();

struct App {
    ui: MainWindow,
    timer: slint::Timer,
    last_sensor_data: ValueStore<SensorData>,
}

impl App {
    /// Create a new App struct.
    fn new() -> anyhow::Result<Self> {
        let ui = MainWindow::new().map_err(|e| anyhow::anyhow!(e))?;
        let timer = slint::Timer::default();
        let last_sensor_data = ValueStore::<SensorData>::default();
        // Clone the last sensor data to pass it to the DHT task
        let last_dht_data = last_sensor_data.clone();

        std::thread::spawn(move || dht_task(last_dht_data));

        Ok(Self {
            ui,
            timer,
            last_sensor_data,
        })
    }

    /// Run the app
    fn run(&mut self) -> anyhow::Result<()> {
        // Get the handle to the UI as a weak reference.
        let ui_handle = self.ui.as_weak();
        let ui = ui_handle.unwrap();
        let model = ViewModel::get(&ui);
        let last_sensor_data = self.last_sensor_data.clone();

        self.timer.start(
            slint::TimerMode::Repeated,
            std::time::Duration::from_millis(2000),
            move || {
                let ui = ui_handle.unwrap();
                let model = ViewModel::get(&ui);
                match last_sensor_data.get() {
                    None => {
                        model.set_sensor_status(SensorStatus::Error);
                    }
                    Some(data) => {
                        model.set_current(data.into());
                        model.set_sensor_status(SensorStatus::Ok);
                    }
                }
            },
        );

        // Set the initial brightness
        model.on_screen_brightness_changed(|value| {
            esp32::set_brightness(value);
        });

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

impl From<SensorData> for WeatherRecord {
    fn from(data: SensorData) -> Self {
        WeatherRecord {
            temperature_celsius: data.temperature_celsius,
            humidity_percent: data.humidity_percent,
            timestamp: slint::SharedString::from(data.when.as_secs().to_string()),
        }
    }
}


fn dht_task(last: ValueStore<SensorData>) {
    let start = Instant::now();
    let pin = 13;
    let dht = dht22::DHT22::new(pin);

    let mut error_count = 0;
    let max_errors = 5;

    loop {
        match dht.read() {
            Ok((temperature_celsius, humidity_percent)) => {
                last.set(SensorData {
                    temperature_celsius,
                    humidity_percent,
                    when: Instant::now().duration_since(start),
                    status: SensorStatus::Ok,
                });
                error_count = 0;
            }
            Err(e) => {                
                error_count += 1;

                if error_count >= max_errors {
                    log::error!("Error reading DHT22: {:?}", e);

                    last.set(SensorData {
                        temperature_celsius: 0.0,
                        humidity_percent: 0.0,
                        when: Instant::now().duration_since(start),
                        status: SensorStatus::Error,
                    });
                }
            }
        }

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

    // Create the app
    let mut app = App::new()?;

    // Run the app
    app.run()
}
