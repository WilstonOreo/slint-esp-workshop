use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

mod dht22;
mod esp32;

slint::include_modules!();


#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SensorData {
    temperature_celsius: f32,
    humidity_percent: f32,
    when: std::time::Duration,
}

/// Convenience helper for passing the last of a value between threads. For example from a thread
/// interfacing with a sensor to another one processing the data.
#[derive(Clone, Default)]
struct ValueStore<T>(Arc<Mutex<Option<T>>>);

impl<T: Clone> ValueStore<T> {
    /// Sets `value` as the last value.
    ///
    /// # Panics
    ///
    /// If the locking the interally used mutex fails.
    fn set(&self, value: T) {
        let mut data = self.0.lock().unwrap();
        let _ = data.insert(value);
    }

    /// Gets the stored value.
    ///
    /// # Panics
    ///
    /// If the locking of the mutex fails
    fn get(&self) -> Option<T> {
        let mut data = self.0.lock().unwrap();
        data.take()
    }
}

fn dht_task(last: ValueStore<SensorData>) {
    let start = Instant::now();
    let dht = dht22::DHT22::new(13);

    loop {
        match dht.read() {
            #[allow(unused_variables)]
            Ok((temperature, humidity)) => {
                let data = SensorData {
                    temperature_celsius: temperature,
                    humidity_percent: humidity,
                    when: Instant::now().duration_since(start),
                };

                last.set(data);
            }
            Err(e) => {
                log::error!("Error reading DHT22: {:?}", e);
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


    let last_sensor_data = ValueStore::<SensorData>::default();
    let last_for_dht_task = last_sensor_data.clone();
    std::thread::spawn(move || dht_task(last_for_dht_task));

    // Finally, run the app!
    let ui = AppWindow::new().expect("Failed to load UI");
    let ui_handle = ui.as_weak();

    ui.on_slider_changed(|value| {
        log::info!("slider changed to {}", value);
    });

    let timer = slint::Timer::default();
    timer.start(
        slint::TimerMode::Repeated,
        std::time::Duration::from_millis(2000),
        move || {
            let ui = ui_handle.unwrap();
            let model = ViewModel::get(&ui);
            if let Some(data) = last_sensor_data.get() {
                let when = data.when.as_secs().to_string();
                let record = WeatherRecord {
                    temperature_celsius: data.temperature_celsius,
                    humidity_percent: data.humidity_percent,
                    timestamp: slint::SharedString::from(when),
                };

                model.set_current(record.clone());
            }
        },
    );

    ui.run().map_err(|e| anyhow::anyhow!(e))
}
