use esp_hal::delay::Delay;
use esp_hal::gpio::{Input, Output};

#[derive(Debug)]
pub enum DhtError {
    ChecksumMismatch,
    Timeout,
}

const TIMEOUT_US: u8 = 100;

fn read_bit(delay: &mut Delay, pin: &mut Input) -> Result<bool, DhtError> {
    wait_until_timeout(delay, || pin.is_high())?;
    delay.delay_micros(35);
    let high = pin.is_high();
    wait_until_timeout(delay, || pin.is_low())?;
    Ok(high)
}

fn read_byte(delay: &mut Delay, pin: &mut Input) -> Result<u8, DhtError> {
    let mut byte: u8 = 0;
    for i in 0..8 {
        let bit_mask = 1 << (7 - i);
        if read_bit(delay, pin)? {
            byte |= bit_mask;
        }
    }
    Ok(byte)
}

pub fn read_raw(
    delay: &mut Delay,
    input: &mut Input,
    //output: &mut Output,
) -> Result<[u8; 4], DhtError> {
    //output.set_high();
    delay.delay_micros(48);

    wait_until_timeout(delay, || input.is_high())?;
    wait_until_timeout(delay, || input.is_low())?;

    let mut data = [0; 4];
    for b in data.iter_mut() {
        *b = read_byte(delay, input)?;
    }
    let checksum = read_byte(delay, input)?;
    if data.iter().fold(0u8, |sum, v| sum.wrapping_add(*v)) != checksum {
        Err(DhtError::ChecksumMismatch)
    } else {
        Ok(data)
    }
}

pub fn read(
    delay: &mut Delay,
    input: &mut Input,
    // output: &mut Output,
) -> Result<Reading, DhtError> {
    // output.set_low();
    delay.delay_millis(1);
    read_raw(delay, input).map(raw_to_reading)
}

/// Wait until the given function returns true or the timeout is reached.
fn wait_until_timeout(delay: &mut Delay, mut func: impl FnMut() -> bool) -> Result<(), DhtError> {
    for _ in 0..TIMEOUT_US {
        if func() {
            return Ok(());
        }
        delay.delay_micros(1);
    }
    Err(DhtError::Timeout)
}
pub struct Reading {
    pub temperature: f32,
    pub relative_humidity: f32,
}

fn raw_to_reading(bytes: [u8; 4]) -> Reading {
    fn convert_signed(signed: u8) -> (bool, u8) {
        let sign = signed & 0x80 != 0;
        let magnitude = signed & 0x7F;
        (sign, magnitude)
    }

    let [rh_h, rh_l, temp_h_signed, temp_l] = bytes;
    let relative_humidity = ((rh_h as u16) << 8 | (rh_l as u16)) as f32 / 10.0;
    let temperature = {
        let (signed, magnitude) = convert_signed(temp_h_signed);
        let temp_sign = if signed { -1.0 } else { 1.0 };
        let temp_magnitude = ((magnitude as u16) << 8) | temp_l as u16;
        temp_sign * temp_magnitude as f32 / 10.0
    };
    Reading {
        temperature,
        relative_humidity,
    }
}
