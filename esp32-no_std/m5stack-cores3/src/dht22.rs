use dht22_sensor::{Dht22, DhtError};
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::gpio::{Flex, Pin};

#[embassy_executor::task]
pub async fn dht22_task(mut dht_pin: esp_hal::gpio::Flex<'static>) {
    let output_config = esp_hal::gpio::OutputConfig::default()
        .with_drive_mode(esp_hal::gpio::DriveMode::OpenDrain)
        .with_pull(esp_hal::gpio::Pull::None);
    dht_pin.apply_output_config(&output_config);
    dht_pin.set_input_enable(true);
    dht_pin.set_output_enable(true);
    dht_pin.set_level(esp_hal::gpio::Level::High);

    let mut delay = esp_hal::delay::Delay::new();
    let delay1 = esp_hal::delay::Delay::new();
    delay1.delay_millis(2000);

    let mut sensor = Dht22::new(&mut dht_pin, &mut delay);
    loop {
        match sensor.read() {
            Ok(reading) => {
                log::info!(
                    "Temperature: {:?}, Humidity: {:?}",
                    reading.temperature,
                    reading.relative_humidity
                );
            }
            Err(err) => match err {
                DhtError::ChecksumMismatch => {
                    log::info!("checksum error");
                }
                DhtError::Timeout => {
                    log::info!("Timeout error");
                }
                DhtError::PinError(e) => {
                    log::info!("Pin error:{}", e);
                }
            },
        }
        delay1.delay_millis(5000);
    }
}
