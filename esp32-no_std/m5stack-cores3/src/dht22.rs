#[allow(unused)]
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use esp_hal::gpio::Flex;

#[allow(unused)]
use embassy_time::{Duration, Timer};

#[embassy_executor::task]
pub async fn dht22_task(_dht_pin: Flex<'static>) {
    // TODO
}
