use dht22_sensor::{Dht22, DhtError};
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::gpio::Pin;

#[embassy_executor::task]
pub async fn dht22_task(pin: esp_hal::peripherals::GPIO5<'static>) {
    // TODO
}
