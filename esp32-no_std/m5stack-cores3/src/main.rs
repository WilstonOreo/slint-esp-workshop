#![no_std]
#![no_main]

extern crate alloc;
extern crate esp_bootloader_esp_idf;

mod app;
mod dht22;
mod display;
mod init;
mod touch;
mod wifi;

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

use core::{cell::RefCell, panic::PanicInfo};

use esp_hal::rng::Rng;

// ESP32 HAL imports - only what we need
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::I2c;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;

// Display imports
use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;

// Touch controller imports
use embedded_hal_bus::i2c::RefCellDevice;
use static_cell::StaticCell;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    log::error!("PANIC: {info}");
    loop {}
}

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize peripherals first
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // Initialize BOTH heap allocators - WiFi first in internal RAM, then PSRAM for GUI
    // Step 1: Initialize internal RAM heap for WiFi (must be first)
    esp_alloc::heap_allocator!(size: 180 * 1024);

    // Step 2: Initialize PSRAM heap for GUI and other data
    crate::init::init_heap(&peripherals.PSRAM);

    // Initialize logger
    esp_println::logger::init_logger_from_env();
    log::info!("Peripherals initialized");

    log::info!("Starting Slint ESP32 M5Stack CoreS3 Workshop");

    // === Begin M5Stack CoreS3 Power Management and I2C Initialization ===
    // Initialize I2C bus for all I2C devices (AXP2101, AW9523, touch controller)
    log::info!("Initializing I2C bus for power management and touch controller...");
    let power_i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO12) // AXP2101 SDA
    .with_scl(peripherals.GPIO11); // AXP2101 SCL
    log::info!("I2C bus initialized for power management and touch");

    // Use StaticCell to create a shared I2C bus for all I2C devices (like the working example)
    static I2C_BUS: StaticCell<RefCell<I2c<'static, esp_hal::Blocking>>> = StaticCell::new();
    let i2c_bus = I2C_BUS.init(RefCell::new(power_i2c));

    // Initialize AXP2101 power management using shared I2C - critical for M5Stack CoreS3 display power
    crate::init::init_axp2101_power(RefCellDevice::new(i2c_bus)).ok();

    // Initialize AW9523 GPIO expander using shared I2C - both devices share same I2C lines
    crate::init::init_aw9523_gpio_expander(RefCellDevice::new(i2c_bus)).ok();

    // Small delay to let power rails stabilize after power management setup
    let mut delay = Delay::new();
    delay.delay_ms(100);
    log::info!("Power management initialization complete, power rails stabilized");
    // === End M5Stack CoreS3 Power Management Initialization ===

    // Initialize WiFi directly in main function
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);

    log::info!("Initializing WiFi...");
    let esp_wifi_ctrl = wifi::create_wifi_controller(timg0, rng);
    log::info!("WiFi controller initialized");

    let (wifi_controller, _interfaces) = esp_wifi::wifi::new(esp_wifi_ctrl, peripherals.WIFI)
        .expect("Failed to create WiFi interface");
    log::info!("WiFi interface created");

    // Initialize embassy timer for task scheduling BEFORE spawning tasks
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    log::info!("Embassy timer initialized");

    // Initialize FT6336U touch driver using shared I2C bus (create fresh delay instance)

    // Create touch reset driver using shared I2C bus

    log::info!("Initializing FT6336U touch controller...");
    let touch_reset = crate::touch::TouchResetDriverAW9523::new(RefCellDevice::new(i2c_bus));
    let touch_delay = esp_hal::delay::Delay::new();
    let mut touch_driver = ft3x68_rs::Ft3x68Driver::new(
        RefCellDevice::new(i2c_bus),
        touch::FT6336U_DEVICE_ADDRESS,
        touch_reset,
        touch_delay,
    );
    touch_driver.initialize().ok();

    log::info!("Initializing M5Stack CoreS3 display hardware using display module...");
    // Initialize display hardware via display module
    display::init_display_hardware(
        peripherals.GPIO3,  // cs
        peripherals.GPIO34, // rst
        peripherals.GPIO35, // dc
        peripherals.GPIO36, // sck
        peripherals.GPIO37, // mosi
        peripherals.GPIO48, // backlight
        peripherals.SPI2,   // SPI peripheral
    )
    .expect("Failed to initialize display hardware");

    // Initialize graphics hardware using display module
    log::info!("=== Starting M5Stack CoreS3 Event Loop ===");

    log::info!("M5Stack CoreS3 display hardware initialized via display module");

    log::info!("Spawning WiFi scan task");
    spawner.spawn(wifi::wifi_scan_task(wifi_controller)).ok();

    log::info!("Spawning DHT22 task");
    use esp_hal::gpio::*;
    let mut dht_pin = Flex::new(peripherals.GPIO5);
    let output_config = OutputConfig::default()
        .with_drive_mode(DriveMode::OpenDrain)
        .with_pull(Pull::None);
    dht_pin.apply_output_config(&output_config);
    dht_pin.set_input_enable(true);
    dht_pin.set_output_enable(true);
    dht_pin.set_level(Level::High);

    spawner.spawn(dht22::dht22_task(dht_pin)).ok();

    use crate::app::*;
    static APP: StaticCell<App> = StaticCell::new();
    let app = APP.init(App::new());

    // Spawn graphics rendering task on same core
    log::info!("Spawning graphics rendering task on Core 0");
    spawner.spawn(ui_update_task(app)).ok();

    app.run(touch_driver).await
}
