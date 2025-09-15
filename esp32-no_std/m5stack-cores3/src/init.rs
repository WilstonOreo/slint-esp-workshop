//! Initialize hardware

// I2C device addresses for M5Stack CoreS3 power management
const AXP2101_ADDRESS: u8 = 0x34; // AXP2101 power management IC
const AW9523_I2C_ADDRESS: u8 = 0x58; // AW9523 GPIO expander

pub fn init_heap(psram: &esp_hal::peripherals::PSRAM<'_>) {
    let (start, size) = esp_hal::psram::psram_raw_parts(psram);
    log::info!("Initializing PSRAM heap: start: {start:p}, size: {size}");
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}

/// Initialize the AXP2101 power management unit for M5Stack CoreS3
/// This implements the exact same sequence as the working custom implementation
/// Based on M5Stack CoreS3 requirements for proper display power
pub fn init_axp2101_power<I2C>(mut i2c_device: I2C) -> Result<(), ()>
where
    I2C: embedded_hal::i2c::I2c,
{
    log::info!("Initializing AXP2101 power management with M5Stack CoreS3 sequence...");

    // This sequence matches exactly the working custom implementation:
    // 1. CHG_LED register (0x69) <- 0x35 (0b00110101)
    // 2. ALDO_ENABLE register (0x90) <- 0xBF
    // 3. ALDO4 register (0x95) <- 0x1C (0b00011100)

    // Step 1: Configure charge LED (register 0x69 = 105 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x69, 0x35]).is_err() {
        log::error!("Failed to write to CHG_LED register (0x69)");
        return Err(());
    }
    log::info!("AXP2101: CHG_LED configured (0x69 <- 0x35)");

    // Step 2: Enable ALDO outputs (register 0x90 = 144 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x90, 0xBF]).is_err() {
        log::error!("Failed to write to ALDO_ENABLE register (0x90)");
        return Err(());
    }
    log::info!("AXP2101: ALDO outputs enabled (0x90 <- 0xBF)");

    // Step 3: Configure ALDO4 voltage (register 0x95 = 149 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x95, 0x1C]).is_err() {
        log::error!("Failed to write to ALDO4 register (0x95)");
        return Err(());
    }
    log::info!("AXP2101: ALDO4 voltage configured (0x95 <- 0x1C)");

    log::info!("AXP2101 power management initialized successfully with M5Stack CoreS3 sequence");
    Ok(())
}

/// Initialize the AW9523 GPIO expander for M5Stack CoreS3
/// This implements the exact same sequence as the working custom implementation
/// Critical for proper display and touch controller operation
pub fn init_aw9523_gpio_expander<I2C>(mut i2c_device: I2C) -> Result<(), ()>
where
    I2C: embedded_hal::i2c::I2c,
{
    log::info!("Initializing AW9523 GPIO expander with M5Stack CoreS3 sequence...");

    // Step 1: Configure Port 0 Configuration (register 0x02) <- 0b00000101 (0x05)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x02, 0b00000101])
        .is_err()
    {
        log::error!("Failed to write to AW9523 Port 0 Configuration register (0x02)");
        return Err(());
    }
    log::info!("AW9523: Port 0 Configuration set (0x02 <- 0x05)");

    // Step 2: Configure Port 1 Configuration (register 0x03) <- 0b00000011 (0x03)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x03, 0b00000011])
        .is_err()
    {
        log::error!("Failed to write to AW9523 Port 1 Configuration register (0x03)");
        return Err(());
    }
    log::info!("AW9523: Port 1 Configuration set (0x03 <- 0x03)");

    // Step 3: Configure Port 0 Output (register 0x04) <- 0b00011000 (0x18)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x04, 0b00011000])
        .is_err()
    {
        log::error!("Failed to write to AW9523 Port 0 Output register (0x04)");
        return Err(());
    }
    log::info!("AW9523: Port 0 Output set (0x04 <- 0x18)");

    // Step 4: Configure Port 1 Output (register 0x05) <- 0b00001100 (0x0C)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x05, 0b00001100])
        .is_err()
    {
        log::error!("Failed to write to AW9523 Port 1 Output register (0x05)");
        return Err(());
    }
    log::info!("AW9523: Port 1 Output set (0x05 <- 0x0C)");

    // Step 5: Configure register 0x11 <- 0b00010000 (0x10)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x11, 0b00010000])
        .is_err()
    {
        log::error!("Failed to write to AW9523 register (0x11)");
        return Err(());
    }
    log::info!("AW9523: Register 0x11 configured (0x11 <- 0x10)");

    // Step 6: Configure register 0x13 <- 0b11111111 (0xFF)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x13, 0b11111111])
        .is_err()
    {
        log::error!("Failed to write to AW9523 register (0x13)");
        return Err(());
    }
    log::info!("AW9523: Register 0x13 configured (0x13 <- 0xFF)");

    log::info!("AW9523 GPIO expander initialized successfully with M5Stack CoreS3 sequence");
    Ok(())
}
