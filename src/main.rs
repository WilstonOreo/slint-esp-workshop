
mod dht22;
mod esp32;

use core::sync::atomic::Ordering;

slint::include_modules!();

static TEMPERATURE: portable_atomic::AtomicF32 = portable_atomic::AtomicF32::new(0.0);
static HUMIDITY: portable_atomic::AtomicF32 = portable_atomic::AtomicF32::new(0.0);

unsafe extern "C" fn dht_task(_: *mut core::ffi::c_void) {
    //let dht = slint::platform::dht::Dht::new(4).unwrap();
    let dht = dht22::DHT22::new(13);

    loop {
        match dht.read() {
            Ok((temperature, humidity)) => {
                if temperature != TEMPERATURE.load(Ordering::Relaxed) || humidity != HUMIDITY.load(Ordering::Relaxed) {
                    TEMPERATURE.store(temperature, Ordering::Relaxed);
                    HUMIDITY.store(humidity, Ordering::Relaxed);
                    log::info!("Temperature: {:.2}°C, Humidity: {:.2}%", temperature, humidity);
                }
            }
            Err(e) => {
                log::error!("Error reading DHT22: {:?}", e);
            }
        }

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
        let ui = ui_handle.unwrap();

        ui.set_temperature(TEMPERATURE.load(Ordering::Relaxed));
        ui.set_humidity(HUMIDITY.load(Ordering::Relaxed));    
    });

    ui.run().unwrap();
}
