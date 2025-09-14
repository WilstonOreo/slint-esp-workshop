use dht22_sensor::DhtError;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::gpio::Flex;

use embassy_time::{Duration, Timer};

pub static DHT22_CHANNEL: Channel<CriticalSectionRawMutex, dht22_sensor::Reading, 8> =
    Channel::new();

#[embassy_executor::task]
pub async fn dht22_task(mut dht_pin: Flex<'static>) {
    let mut delay = esp_hal::delay::Delay::new();
    let mut sensor = dht22_sensor::Dht22::new(&mut dht_pin, &mut delay);
    let sender = DHT22_CHANNEL.sender();

    loop {
        match sensor.read() {
            Ok(reading) => {
                log::info!(
                    "Temperature: {:?}, Humidity: {:?}",
                    reading.temperature,
                    reading.relative_humidity
                );
                sender.send(reading).await;
            }
            Err(err) => match err {
                DhtError::ChecksumMismatch => {
                    log::info!("checksum error");
                }
                DhtError::Timeout => {
                    log::info!("Timeout error");
                }
                DhtError::PinError(e) => {
                    log::info!("Pin error:{e}");
                }
            },
        }

        Timer::after(Duration::from_secs(1)).await;
    }
}
