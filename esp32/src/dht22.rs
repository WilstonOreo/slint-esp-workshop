
pub struct DHT22 {
    pin: i32
}

#[derive(Debug)]
pub enum DHT22Error {
    ChecksumError,
    TimeoutError,
}

impl DHT22 {
    const MAX_DHT_DATA: usize = 5;

    pub fn new(pin: i32) -> Self {
        Self {
            pin
        }
    }

    pub fn get_signal_level(&self, max_wait: i32, state: i32) -> i32 {
        use esp_idf_svc::sys::*;

        let mut u_sec: i32 = 0;
        unsafe {
            while gpio_get_level(self.pin) == state {
                u_sec += 1;
                if u_sec > max_wait {
                    return -1;
                }
                ets_delay_us(1);
            }
        }

        u_sec
    }

    pub fn read(&self) -> Result<(f32, f32), DHT22Error> {
        use esp_idf_svc::sys::*;

        let mut dht_data = [0; Self::MAX_DHT_DATA];
        let mut byte_inx = 0;
        let mut bit_inx = 7;

        unsafe {

            gpio_set_direction(self.pin, GPIO_MODE_DEF_OUTPUT);

            // pull down for 3 ms for a smooth and nice wake up
            gpio_set_level(self.pin, 0);
            ets_delay_us(3000);

            // pull up for 25 us for a gentile asking for data
            gpio_set_level(self.pin, 1);
            ets_delay_us(25);

            gpio_set_direction(self.pin, GPIO_MODE_DEF_INPUT); // change to input mode

            // == DHT will keep the line low for 80 us and then high for 80us ====

            let u_sec = self.get_signal_level(85, 0);
            //	ESP_LOGI( TAG, "Response = %d", uSec );
            if u_sec < 0 {
                return Err(DHT22Error::TimeoutError);
            }

            // -- 80us up ------------------------

            let u_sec = self.get_signal_level(85, 1);
            //	ESP_LOGI( TAG, "Response = %d", uSec );
            if u_sec < 0 {
                return Err(DHT22Error::TimeoutError);
            }

            // == No errors, read the 40 data bits ================
            for _ in 0..40 {
                // -- starts new data transmission with >50us low signal

                let u_sec = self.get_signal_level(56, 0);
                if u_sec < 0 {
                    return Err(DHT22Error::TimeoutError);
                }

                // -- check to see if after >70us rx data is a 0 or a 1

                let u_sec = self.get_signal_level(75, 1);
                if u_sec < 0 {
                    return Err(DHT22Error::TimeoutError);
                }

                // add the current read to the output data
                // since all dhtData array where set to 0 at the start,
                // only look for "1" (>28us us)

                if u_sec > 40 {
                    dht_data[byte_inx] |= 1 << bit_inx;
                }

                // index to next byte

                if bit_inx == 0 {
                    bit_inx = 7;
                    byte_inx += 1;
                } else {
                    bit_inx -= 1;
                }
            }

        }

        // == get humidity from Data[0] and Data[1] ==========================
        let mut humidity = dht_data[0] as f32;
        humidity *= 0x100 as f32; // >> 8
        humidity += dht_data[1] as f32;
        humidity /= 10.0; // get the decimal

        // == get temp from Data[2] and Data[3]

        let mut temperature = (dht_data[2] & 0x7F) as f32;
        temperature *= 0x100 as f32; // >> 8
        temperature += dht_data[3] as f32;
        temperature /= 10.0;

        if dht_data[2] & 0x80 != 0 {
            // negative temp, brrr it's freezing
            temperature *= -1.0;
        }

        // == verify if checksum is ok ===========================================
        // Checksum is the sum of Data 8 bits masked out 0xFF

        if dht_data[4] == ((dht_data[0] + dht_data[1] + dht_data[2] + dht_data[3]) & 0xFF) {
            Ok((temperature, humidity))
        } else {
            Err(DHT22Error::ChecksumError)
        }
    }
}