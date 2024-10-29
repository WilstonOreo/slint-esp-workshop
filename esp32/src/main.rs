use std::time::{Duration, Instant};
use slint_workshop_common::ValueStore;

mod dht22;
mod esp32;

slint::include_modules!();

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SensorData {
    temperature_celsius: f32,
    humidity_percent: f32,
    when: std::time::Duration,
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

    // Launch the DHT task in a separate thread
    let last_sensor_data = ValueStore::<SensorData>::default();
    let last_for_dht_task = last_sensor_data.clone();
    std::thread::spawn(move || dht_task(last_sensor_data));

    // Finally, run the app!
    let ui = AppWindow::new().expect("Failed to load UI");
    let ui_handle = ui.as_weak();

    let timer = slint::Timer::default();
    timer.start(
        slint::TimerMode::Repeated,
        std::time::Duration::from_millis(2000),
        move || {
            let ui = ui_handle.unwrap();
            let model = ViewModel::get(&ui);
            match last_for_dht_task.get() {
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
    ui.on_screen_brightness_slider_changed(|value| {
        esp32::set_brightness(value);
    });

    ui.run().map_err(|e| anyhow::anyhow!(e))
}
