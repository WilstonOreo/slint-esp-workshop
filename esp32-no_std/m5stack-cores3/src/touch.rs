// Touch controller support for M5Stack CoreS3
// Using FT6336U touch controller via I2C with AW9523 GPIO expander for reset
// Based on the working example from Slint mcu-board-support/m5stack_cores3.rs

use esp_hal::delay::Delay;
use ft3x68_rs::ResetInterface;
use log::info;

// AW9523 I2C address (GPIO expander on M5Stack CoreS3)
pub const AW9523_I2C_ADDRESS: u8 = 0x58;

// FT6336U I2C address (compatible with FT3x68 driver)
pub const FT6336U_DEVICE_ADDRESS: u8 = 0x38;

/// Touch reset implementation via AW9523 GPIO expander using direct I2C commands
/// Based on the AW9523 datasheet and M5Stack CoreS3 schematics
/// The touch reset line is connected to P0_0 on the AW9523 GPIO expander
pub struct TouchResetDriverAW9523<I2C> {
    i2c: I2C,
}

impl<I2C> TouchResetDriverAW9523<I2C> {
    pub fn new(i2c: I2C) -> Self {
        TouchResetDriverAW9523 { i2c }
    }
}

impl<I2C> ResetInterface for TouchResetDriverAW9523<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    type Error = I2C::Error;

    fn reset(&mut self) -> Result<(), Self::Error> {
        let delay = Delay::new();

        info!("Resetting FT6336U touch controller via AW9523 GPIO expander...");

        // AW9523 register addresses:
        // 0x02: Port 0 Configuration (0=output, 1=input)
        // 0x03: Port 1 Configuration (0=output, 1=input)
        // 0x04: Port 0 Output (pin values for outputs)
        // 0x05: Port 1 Output (pin values for outputs)

        // Configure P0_0 (touch reset) as output (bit 0 = 0)
        // Keep other pins as they are - read current config first
        let mut config_p0 = [0u8; 1];
        self.i2c
            .write_read(AW9523_I2C_ADDRESS, &[0x02], &mut config_p0)?;
        let new_config_p0 = config_p0[0] & !0x01; // Clear bit 0 to make P0_0 output
        self.i2c.write(AW9523_I2C_ADDRESS, &[0x02, new_config_p0])?;

        // Pull reset (P0_0) low
        let mut output_p0 = [0u8; 1];
        self.i2c
            .write_read(AW9523_I2C_ADDRESS, &[0x04], &mut output_p0)?;
        let new_output_low = output_p0[0] & !0x01; // Clear bit 0 to pull P0_0 low
        self.i2c
            .write(AW9523_I2C_ADDRESS, &[0x04, new_output_low])?;
        delay.delay_millis(10);

        // Pull reset (P0_0) high
        let new_output_high = output_p0[0] | 0x01; // Set bit 0 to pull P0_0 high
        self.i2c
            .write(AW9523_I2C_ADDRESS, &[0x04, new_output_high])?;
        delay.delay_millis(300);

        info!("FT6336U touch controller reset completed");
        Ok(())
    }
}
