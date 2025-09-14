#![no_std]
#![no_main]

extern crate alloc;
extern crate esp_bootloader_esp_idf;

//mod display;
//mod touch;

esp_bootloader_esp_idf::esp_app_desc!(
    // Version
    "1.0.0",
    // Project name
    "esp-workshop",
    // Build time
    "12:00:00",
    // Build date
    "2025-01-01",
    // ESP-IDF version
    "4.4",
    // MMU page size
    8 * 1024,
    // Minimal eFuse block revision supported by image. Format: major * 100 + minor
    0,
    // Maximum eFuse block revision supported by image. Format: major * 100 + minor
    u16::MAX
);

use core::cell::RefCell;

use dht22_sensor::{Dht22, DhtError};
use embedded_hal_bus::i2c::RefCellDevice;
use esp_hal::clock::CpuClock;
use esp_hal::delay::Delay;
use esp_hal::gpio::{self, Flex, Level};
use esp_hal::i2c::master::I2c;
use esp_hal::main;
use esp_hal::time::Rate;
use esp_println as _;
use esp_println::logger::init_logger_from_env;

use log::{debug, error, info};
use static_cell::StaticCell;
// I2C device addresses for M5Stack CoreS3 power management
const AXP2101_ADDRESS: u8 = 0x34; // AXP2101 power management IC
const AW9523_I2C_ADDRESS: u8 = 0x58; // AW9523 GPIO expander

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

fn init_heap(psram: &esp_hal::peripherals::PSRAM<'_>) {
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
fn init_axp2101_power<I2C>(mut i2c_device: I2C) -> Result<(), ()>
where
    I2C: embedded_hal::i2c::I2c,
{
    info!("Initializing AXP2101 power management with M5Stack CoreS3 sequence...");

    // This sequence matches exactly the working custom implementation:
    // 1. CHG_LED register (0x69) <- 0x35 (0b00110101)
    // 2. ALDO_ENABLE register (0x90) <- 0xBF
    // 3. ALDO4 register (0x95) <- 0x1C (0b00011100)

    // Step 1: Configure charge LED (register 0x69 = 105 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x69, 0x35]).is_err() {
        error!("Failed to write to CHG_LED register (0x69)");
        return Err(());
    }
    info!("AXP2101: CHG_LED configured (0x69 <- 0x35)");

    // Step 2: Enable ALDO outputs (register 0x90 = 144 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x90, 0xBF]).is_err() {
        error!("Failed to write to ALDO_ENABLE register (0x90)");
        return Err(());
    }
    info!("AXP2101: ALDO outputs enabled (0x90 <- 0xBF)");

    // Step 3: Configure ALDO4 voltage (register 0x95 = 149 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x95, 0x1C]).is_err() {
        error!("Failed to write to ALDO4 register (0x95)");
        return Err(());
    }
    info!("AXP2101: ALDO4 voltage configured (0x95 <- 0x1C)");

    info!("AXP2101 power management initialized successfully with M5Stack CoreS3 sequence");
    Ok(())
}

/// Initialize the AW9523 GPIO expander for M5Stack CoreS3
/// This implements the exact same sequence as the working custom implementation
/// Critical for proper display and touch controller operation
fn init_aw9523_gpio_expander<I2C>(mut i2c_device: I2C) -> Result<(), ()>
where
    I2C: embedded_hal::i2c::I2c,
{
    info!("Initializing AW9523 GPIO expander with M5Stack CoreS3 sequence...");

    // Step 1: Configure Port 0 Configuration (register 0x02) <- 0b00000101 (0x05)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x02, 0b00000101])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 0 Configuration register (0x02)");
        return Err(());
    }
    info!("AW9523: Port 0 Configuration set (0x02 <- 0x05)");

    // Step 2: Configure Port 1 Configuration (register 0x03) <- 0b00000011 (0x03)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x03, 0b00000011])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 1 Configuration register (0x03)");
        return Err(());
    }
    info!("AW9523: Port 1 Configuration set (0x03 <- 0x03)");

    // Step 3: Configure Port 0 Output (register 0x04) <- 0b00011000 (0x18)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x04, 0b00011000])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 0 Output register (0x04)");
        return Err(());
    }
    info!("AW9523: Port 0 Output set (0x04 <- 0x18)");

    // Step 4: Configure Port 1 Output (register 0x05) <- 0b00001100 (0x0C)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x05, 0b00001100])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 1 Output register (0x05)");
        return Err(());
    }
    info!("AW9523: Port 1 Output set (0x05 <- 0x0C)");

    // Step 5: Configure register 0x11 <- 0b00010000 (0x10)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x11, 0b00010000])
        .is_err()
    {
        error!("Failed to write to AW9523 register (0x11)");
        return Err(());
    }
    info!("AW9523: Register 0x11 configured (0x11 <- 0x10)");

    // Step 6: Configure register 0x13 <- 0b11111111 (0xFF)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x13, 0b11111111])
        .is_err()
    {
        error!("Failed to write to AW9523 register (0x13)");
        return Err(());
    }
    info!("AW9523: Register 0x13 configured (0x13 <- 0xFF)");

    info!("AW9523 GPIO expander initialized successfully with M5Stack CoreS3 sequence");
    Ok(())
}

#[main]
fn main() -> ! {
    // Initialize peripherals first
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // Initialize BOTH heap allocators - WiFi first in internal RAM, then PSRAM for GUI
    // Step 1: Initialize internal RAM heap for WiFi (must be first)
    esp_alloc::heap_allocator!(size: 180 * 1024);

    // Step 2: Initialize PSRAM heap for GUI and other data
    init_heap(&peripherals.PSRAM);

    // Initialize logger
    init_logger_from_env();
    info!("Peripherals initialized");

    info!("Starting Slint ESP32 M5Stack CoreS3 Workshop");

    // === Begin M5Stack CoreS3 Power Management and I2C Initialization ===
    // Initialize I2C bus for all I2C devices (AXP2101, AW9523, touch controller)
    info!("Initializing I2C bus for power management and touch controller...");
    let power_i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO12) // AXP2101 SDA
    .with_scl(peripherals.GPIO11); // AXP2101 SCL
    info!("I2C bus initialized for power management and touch");

    // Use StaticCell to create a shared I2C bus for all I2C devices (like the working example)
    static I2C_BUS: StaticCell<RefCell<I2c<'static, esp_hal::Blocking>>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(RefCell::new(power_i2c));

    // Initialize AXP2101 power management using shared I2C - critical for M5Stack CoreS3 display power
    match init_axp2101_power(RefCellDevice::new(i2c_bus)) {
        Ok(_) => {
            info!("AXP2101 power management initialized successfully");
        }
        Err(_) => {
            error!("Failed to initialize AXP2101 power management - display may not work properly");
            // Continue anyway, but warn user
        }
    };

    // Initialize AW9523 GPIO expander using shared I2C - both devices share same I2C lines
    match init_aw9523_gpio_expander(RefCellDevice::new(i2c_bus)) {
        Ok(_) => {
            info!("AW9523 GPIO expander initialized successfully");
        }
        Err(_) => {
            error!("Failed to initialize AW9523 GPIO expander - touch may not work properly");
            // Continue anyway, but warn user
        }
    };

    // Small delay to let power rails stabilize after power management setup
    let mut delay = Delay::new();
    delay.delay_millis(100);

    // let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    // let peripherals = esp_hal::init(config);

    let mut dht_pin = Flex::new(peripherals.GPIO5);

    let output_config = gpio::OutputConfig::default()
        .with_drive_mode(gpio::DriveMode::OpenDrain)
        .with_pull(gpio::Pull::None);
    dht_pin.apply_output_config(&output_config);
    dht_pin.set_input_enable(true);
    dht_pin.set_output_enable(true);
    dht_pin.set_level(Level::High);

    let mut delay = Delay::new();
    let delay1 = Delay::new();
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
                    log::info!("Pin error:{e}");
                }
            },
        }
        delay1.delay_millis(5000);
    }
}
