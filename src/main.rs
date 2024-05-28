
mod dht22;
mod esp32;

slint::include_modules!();

use core::sync::atomic::Ordering;

static TEMPERATURE: portable_atomic::AtomicF32 = portable_atomic::AtomicF32::new(0.0);
static HUMIDITY: portable_atomic::AtomicF32 = portable_atomic::AtomicF32::new(0.0);
static TIMESTAMP: portable_atomic::AtomicI64 = portable_atomic::AtomicI64::new(0);
static HAVE_DATA: portable_atomic::AtomicBool = portable_atomic::AtomicBool::new(false);

unsafe extern "C" fn dht_task(_: *mut core::ffi::c_void) {
    let dht = dht22::DHT22::new(13);

    loop {
        match dht.read() {
            Ok((temperature, humidity)) => {
                // TODO: Store sensor values somewhere
                TEMPERATURE.store(temperature, Ordering::Relaxed);
                HUMIDITY.store(humidity, Ordering::Relaxed);
                HAVE_DATA.store(true, Ordering::Relaxed);
                log::info!("Temp: {:.2}°C, Humidity: {:.2}%", temperature, humidity);
            }
            Err(e) => {
                HAVE_DATA.store(false, Ordering::Relaxed);

                log::error!("Error reading DHT22: {:?}", e);
            }
        }

        let timestamp = esp_idf_svc::sys::esp_timer_get_time(); // Get microseconds since boot-up
        TIMESTAMP.store(timestamp, Ordering::Relaxed);

        esp_idf_svc::sys::vTaskDelay(2000 / 10);
    }
}

fn main() {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    // Set the platform
    slint::platform::set_platform(esp32::EspPlatform::new()).unwrap();

    // Create DHT task
    unsafe {
        let mut task_handle = std::ptr::null_mut();
        esp_idf_svc::sys::xTaskCreatePinnedToCore(
            Some(dht_task),
            b"sensor_task\0".as_ptr() as *const i8,
            4096,
            std::ptr::null_mut(),
            5,
            &mut task_handle,
            1
        );
    }

    // Finally, run the app!
    let ui = AppWindow::new().expect("Failed to load UI");

    let ui_handle = ui.as_weak();

    let timer = slint::Timer::default();
    timer.start(slint::TimerMode::Repeated, std::time::Duration::from_millis(2000), move || {
        // TODO: Update UI with sensor values
        let ui = ui_handle.unwrap();

        if !HAVE_DATA.load(Ordering::Relaxed) {
            return;
        }

        ui.global::<ViewModel>().set_weather(WeatherRecord {
            temperature: TEMPERATURE.load(Ordering::Relaxed),
            humidity: HUMIDITY.load(Ordering::Relaxed),
            // Convert the i64 timestamp in microseconds to slint::SharedString in seconds
            timestamp: slint::format!("{:?}s", TIMESTAMP.load(Ordering::Relaxed) / 1_000_000)
        });

        ui.global::<ViewModel>().set_have_data(true);
        HAVE_DATA.store(false, Ordering::Relaxed);
    });

    ui.run().unwrap();
}
