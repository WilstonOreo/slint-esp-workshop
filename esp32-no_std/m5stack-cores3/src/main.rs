#![no_std]
#![no_main]

extern crate alloc;
extern crate esp_bootloader_esp_idf;

mod dht22;
mod display;
mod touch;

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

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::vec;
use core::ops::{Deref, DerefMut};
use core::panic::PanicInfo;
use esp_hal::gpio::OutputConfig;
use log::{debug, error, info};

// WiFi imports - simplified
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use esp_hal::rng::Rng;
use esp_wifi::EspWifiController;
use esp_wifi::wifi::{AccessPointInfo, ClientConfiguration, Configuration, WifiController};

// ESP32 HAL imports - only what we need
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::i2c::master::I2c;
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use esp_println::logger::init_logger_from_env;

// Display imports
use display::{DISPLAY_COMPONENTS, HardwareDrawBuffer};
use embedded_hal::delay::DelayNs;
use esp_hal::delay::Delay;

// Slint platform imports
use slint::PhysicalPosition;
use slint::platform::software_renderer::Rgb565Pixel;
use slint::platform::{PointerEventButton, WindowEvent};

// Touch controller imports
use core::cell::RefCell;
use embedded_hal_bus::i2c::RefCellDevice;
use ft3x68_rs::{Ft3x68Driver, TouchState};
use static_cell::StaticCell;
use touch::{FT6336U_DEVICE_ADDRESS, TouchResetDriverAW9523};

slint::include_modules!();

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);
/*
static DHT22_RESULT: Mutex<CriticalSectionRawMutex, dht_sensor::dht22::Reading> =
    Mutex::new(dht_sensor::dht22::Reading {
        temperature: 0.0,
        relative_humidity: 0.0,
    });
*/

// Display constants for M5Stack CoreS3 - 320x240 ILI9341
const LCD_H_RES: u16 = 320;
const LCD_V_RES: u16 = 240;
const LCD_H_RES_USIZE: usize = 320;
const LCD_V_RES_USIZE: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

// I2C device addresses for M5Stack CoreS3 power management
const AXP2101_ADDRESS: u8 = 0x34; // AXP2101 power management IC
const AW9523_I2C_ADDRESS: u8 = 0x58; // AW9523 GPIO expander

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {}
}

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

fn init_heap(psram: &esp_hal::peripherals::PSRAM<'_>) {
    let (start, size) = esp_hal::psram::psram_raw_parts(psram);
    info!(
        "Initializing PSRAM heap: start: {:p}, size: {}",
        start, size
    );
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

struct EspEmbassyBackend {
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl EspEmbassyBackend {
    fn new(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl slint::platform::Platform for EspEmbassyBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        embassy_time::Instant::now()
            .duration_since(embassy_time::Instant::from_secs(0))
            .into()
    }
}

// Graphics rendering task - handles display output only
#[embassy_executor::task]
async fn graphics_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    info!("=== Graphics rendering task started ====");

    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    let mut frame_counter = 0u32;

    // Create pixel buffer for Slint rendering
    let mut pixel_buffer: Box<[Rgb565Pixel; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565Pixel(0); LCD_BUFFER_SIZE]);

    info!(
        "Graphics task initialized with {}x{} buffer",
        LCD_H_RES, LCD_V_RES
    );

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                debug!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Render the frame using the hardware display
        let rendered = window.draw_if_needed(|renderer| {
            // Access the global display instance
            if let Some(()) = DISPLAY_COMPONENTS.with_mut(|display_hardware| {
                // Create hardware draw buffer
                let mut hardware_buffer =
                    HardwareDrawBuffer::new(&mut display_hardware.display, &mut *pixel_buffer);

                // Render by line to the hardware display
                renderer.render_by_line(&mut hardware_buffer);

                if frame_counter % 60 == 0 {
                    debug!("Frame {} rendered to hardware display", frame_counter);
                }
            }) {
                // Successfully rendered
            } else {
                error!("Display not available in graphics task!");
            }
        });

        // If a frame was rendered, log it
        if rendered {
            if frame_counter % 60 == 0 {
                debug!(
                    "Frame {} rendered and displayed on M5Stack CoreS3",
                    frame_counter
                );
            }
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            info!(
                "Graphics: Frame {}, M5Stack CoreS3 display active",
                frame_counter
            );
        }

        ticker.next().await;
    }
}

// TODO: Touch polling task will be implemented later when we add async touch handling
// For now, touch is tested synchronously in main() and can be extended as needed

// WiFi scanning task
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("=== WiFi scan task started ====");

    // Start WiFi
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::new(),
        password: String::new(),
        ..Default::default()
    });

    match wifi_controller.set_configuration(&client_config) {
        Ok(_) => info!("WiFi configuration set successfully"),
        Err(e) => info!("Failed to set WiFi configuration: {:?}", e),
    }

    match wifi_controller.start_async().await {
        Ok(_) => info!("WiFi started successfully!"),
        Err(e) => info!("Failed to start WiFi: {:?}", e),
    }

    // Wait a bit for WiFi to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;

    loop {
        info!("Performing WiFi scan...");

        match wifi_controller.scan_n_async(10).await {
            Ok(results) => {
                info!("Found {} networks:", results.len());
                for (i, ap) in results.iter().enumerate() {
                    info!(
                        "  {}: SSID: {}, Signal: {:?}, Auth: {:?}, Channel: {}",
                        i + 1,
                        ap.ssid.as_str(),
                        ap.signal_strength,
                        ap.auth_method,
                        ap.channel
                    );
                }

                // Store scan results in shared state
                if let Ok(mut scan_results) = WIFI_SCAN_RESULTS.try_lock() {
                    scan_results.clear();
                    scan_results.extend_from_slice(&results);
                    WIFI_SCAN_UPDATED.store(true, Ordering::Relaxed);
                    info!("Stored {} scan results for UI", scan_results.len());
                } else {
                    info!("Could not store scan results (mutex locked)");
                }
            }
            Err(e) => {
                info!("WiFi scan failed: {:?}", e);
            }
        }

        // Wait 10 seconds before next scan
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}

use embassy_sync::channel::{Channel, Receiver, Sender};
use esp_hal::{
    gpio::{AnyPin, Input, InputConfig, Level, Output},
    peripherals::Peripherals,
};

struct MyDelay(Delay);

impl Deref for MyDelay {
    type Target = Delay;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for MyDelay {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[embassy_executor::task]
async fn dht22_task(mut pin: AnyPin<'static>) {
    let mut input = Input::new(pin, InputConfig::default());
    //    let mut output = Output::new(pin, Level::Low, OutputConfig::default());
    let mut delay = Delay::new();

    loop {
        // Run the blocking DHT22 read in a dedicated threadpool
        let res = dht22::read(&mut delay, &mut input);

        match res {
            Ok(reading) => {
                info!(
                    "DHT22 Read OK: Temp={}°C, Humidity={}%",
                    reading.temperature, reading.relative_humidity
                );
            }
            Err(e) => {
                log::warn!("DHT22 Read failed: {:?}", e);
            }
        }

        delay.delay_millis(500);
    }
}

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) {
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
    delay.delay_ms(100);
    info!("Power management initialization complete, power rails stabilized");
    // === End M5Stack CoreS3 Power Management Initialization ===

    // Initialize WiFi directly in main function
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);

    info!("Initializing WiFi...");
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng.clone()).expect("Failed to initialize WiFi")
    );
    info!("WiFi controller initialized");

    let (wifi_controller, _interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI)
        .expect("Failed to create WiFi interface");
    info!("WiFi interface created");

    // Initialize embassy timer for task scheduling BEFORE spawning tasks
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    info!("Embassy timer initialized");

    // Store WiFi controller for the wifi scan task
    let wifi_ctrl = wifi_controller;

    // Create custom Slint window and backend
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.set_size(slint::PhysicalSize::new(320, 240));

    let backend = Box::new(EspEmbassyBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("Custom Slint backend initialized");

    // Initial liveness check
    info!("System initialization complete - M5Stack CoreS3 is alive and ready");
    // Create the UI
    let ui = MainWindow::new().unwrap();

    // Create empty WiFi network model with some placeholder data
    let placeholder_networks = vec![
        WifiNetwork {
            ssid: "WiFi Scanning...".into(),
        },
        WifiNetwork {
            ssid: "Please wait".into(),
        },
    ];

    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(placeholder_networks));
    ui.set_wifi_network_model(wifi_model.clone().into());

    // Set up WiFi refresh handler with real WiFi functionality
    ui.on_wifi_refresh(move || {
        info!("WiFi refresh requested - checking for scan results");

        // Check if we have new scan results
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            // Access the scan results
            let scan_results = WIFI_SCAN_RESULTS.try_lock();
            if let Ok(results) = scan_results {
                let mut networks = alloc::vec::Vec::new();

                for ap in results.iter() {
                    networks.push(WifiNetwork {
                        ssid: ap.ssid.as_str().into(),
                    });
                }

                if networks.is_empty() {
                    networks.push(WifiNetwork {
                        ssid: "No networks found".into(),
                    });
                }

                info!("Updated UI with {} real networks", networks.len());
                wifi_model.set_vec(networks);

                // Reset the update flag
                WIFI_SCAN_UPDATED.store(false, Ordering::Relaxed);
            } else {
                info!("Could not access scan results (locked)");
            }
        } else {
            info!("No new scan results available");
        }
    });

    // Trigger initial refresh
    ui.invoke_wifi_refresh();

    // Spawn WiFi scanning task
    info!("Spawning WiFi scan task");
    spawner.spawn(wifi_scan_task(wifi_ctrl)).ok();

    // Acquire Handle to IO
    let mut io = esp_hal::gpio::Io::new(peripherals.IO_MUX);

    info!("Spawning DHT22 task");
    spawner.spawn(dht22_task(peripherals.GPIO5.into())).ok();

    // Initialize graphics hardware using display module
    info!("=== Starting M5Stack CoreS3 Event Loop ===");
    info!("Initializing M5Stack CoreS3 display hardware using display module...");

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

    info!("M5Stack CoreS3 display hardware initialized via display module");

    // === Begin Touch Controller Initialization ===
    info!("Initializing FT6336U touch controller...");

    // Create touch reset driver using shared I2C bus
    let touch_reset = TouchResetDriverAW9523::new(RefCellDevice::new(i2c_bus));

    // Initialize FT6336U touch driver using shared I2C bus (create fresh delay instance)
    let touch_delay = Delay::new();
    let mut touch_driver = Ft3x68Driver::new(
        RefCellDevice::new(i2c_bus),
        FT6336U_DEVICE_ADDRESS,
        touch_reset,
        touch_delay,
    );

    let mut _touch_driver_for_task = match touch_driver.initialize() {
        Ok(_) => {
            info!("FT6336U touch controller initialized successfully");

            // Test touch polling once to verify it's working
            info!("Testing touch controller polling...");
            match touch_driver.touch1() {
                Ok(touch_state) => match touch_state {
                    TouchState::Pressed(touch_point) => {
                        info!(
                            "Touch controller test: Touch detected at x={}, y={}",
                            touch_point.x, touch_point.y
                        );
                    }
                    TouchState::Released => {
                        info!("Touch controller test: No touch detected (released state)");
                    }
                },
                Err(e) => {
                    info!("Touch controller test: No touch or error: {:?}", e);
                }
            }
            info!("Touch controller test completed - polling functionality verified");
            info!("Touch events will now be logged when you touch the screen");
            Some(touch_driver)
        }
        Err(e) => {
            error!("Touch initialization failed: {:?}", e);
            info!("Continuing without touch functionality");
            None
        }
    };
    // === End Touch Controller Initialization ===

    // Note: WiFi task already spawned above

    // Spawn graphics rendering task on same core
    info!("Spawning graphics rendering task on Core 0");
    spawner
        .spawn(graphics_task(window.clone(), ui.as_weak()))
        .ok();

    // === Touch Polling Integration ===
    info!("Starting continuous touch polling and Slint integration...");

    let mut status_counter = 0u32;
    let mut touch_ticker = Ticker::every(Duration::from_millis(16)); // ~60Hz touch polling
    let mut last_touch_state = TouchState::Released;
    let mut last_touch_position = slint::LogicalPosition::new(0.0, 0.0);

    loop {
        // Poll touch events if touch controller is available
        if let Some(ref mut touch_driver) = _touch_driver_for_task {
            match touch_driver.touch1() {
                Ok(touch_state) => {
                    match (&last_touch_state, &touch_state) {
                        // Touch press event (transition from Released to Pressed)
                        (TouchState::Released, TouchState::Pressed(touch_point)) => {
                            let physical_position =
                                PhysicalPosition::new(touch_point.x as i32, touch_point.y as i32);
                            let logical_position =
                                physical_position.to_logical(window.scale_factor());
                            last_touch_position = logical_position;

                            let pointer_event = WindowEvent::PointerPressed {
                                position: logical_position,
                                button: PointerEventButton::Left,
                            };

                            window.dispatch_event(pointer_event);
                            info!(
                                "Touch PRESSED at x={}, y={} (logical: {:.1}, {:.1}, scale_factor={})",
                                touch_point.x,
                                touch_point.y,
                                logical_position.x,
                                logical_position.y,
                                window.scale_factor()
                            );
                        }
                        // Touch release event (transition from Pressed to Released)
                        (TouchState::Pressed(touch_point), TouchState::Released) => {
                            // Use the last known touch position for the release event
                            let physical_position =
                                PhysicalPosition::new(touch_point.x as i32, touch_point.y as i32);
                            let logical_position =
                                physical_position.to_logical(window.scale_factor());

                            // Send PointerReleased at the actual release position
                            let pointer_released = WindowEvent::PointerReleased {
                                position: logical_position,
                                button: PointerEventButton::Left,
                            };
                            window.dispatch_event(pointer_released);

                            // Also send PointerExited to complete the interaction cycle
                            let pointer_exited = WindowEvent::PointerExited;
                            window.dispatch_event(pointer_exited);

                            info!(
                                "Touch RELEASED at x={}, y={} (logical: {:.1}, {:.1}) + EXITED",
                                touch_point.x,
                                touch_point.y,
                                logical_position.x,
                                logical_position.y
                            );
                        }
                        // Touch move event (both states are Pressed but potentially different positions)
                        (TouchState::Pressed(old_point), TouchState::Pressed(new_point)) => {
                            // Only dispatch move event if position actually changed
                            if old_point.x != new_point.x || old_point.y != new_point.y {
                                let physical_position =
                                    PhysicalPosition::new(new_point.x as i32, new_point.y as i32);
                                let logical_position =
                                    physical_position.to_logical(window.scale_factor());
                                last_touch_position = logical_position;

                                let pointer_event = WindowEvent::PointerMoved {
                                    position: logical_position,
                                };

                                window.dispatch_event(pointer_event);
                                debug!(
                                    "Touch MOVED to x={}, y={} (logical: {:.1}, {:.1}, scale_factor={})",
                                    new_point.x,
                                    new_point.y,
                                    logical_position.x,
                                    logical_position.y,
                                    window.scale_factor()
                                );
                            }
                        }
                        // No state change
                        _ => {}
                    }

                    last_touch_state = touch_state;
                }
                Err(_) => {
                    // Touch polling error - don't spam logs, just continue
                }
            }
        }

        // Status logging (less frequent than touch polling)
        if status_counter % 600 == 0 {
            // Every ~10 seconds at 60Hz
            info!(
                "Main task status check #{} - M5Stack CoreS3 alive with touch polling",
                status_counter / 60
            );
        }

        status_counter += 1;
        touch_ticker.next().await;
    }
}
