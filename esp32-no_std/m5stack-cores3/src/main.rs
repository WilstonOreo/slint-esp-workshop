#![no_std]
#![no_main]

extern crate alloc;
extern crate esp_bootloader_esp_idf;

mod dht22;
mod display;
mod init;
mod touch;
mod ui;
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

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::vec;
use core::{cell::RefCell, panic::PanicInfo};
use log::{debug, error, info};
use slint::{
    PhysicalPosition,
    platform::{PointerEventButton, WindowEvent},
};

// WiFi imports - simplified
use core::sync::atomic::Ordering;
use embassy_time::{Duration, Ticker};
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
use ft3x68_rs::{Ft3x68Driver, TouchState};
use static_cell::StaticCell;
use touch::{FT6336U_DEVICE_ADDRESS, TouchResetDriverAW9523};

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {info}");
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
    crate::init::init_axp2101_power(RefCellDevice::new(i2c_bus)).ok();

    // Initialize AW9523 GPIO expander using shared I2C - both devices share same I2C lines
    crate::init::init_aw9523_gpio_expander(RefCellDevice::new(i2c_bus)).ok();

    // Small delay to let power rails stabilize after power management setup
    let mut delay = Delay::new();
    delay.delay_ms(100);
    info!("Power management initialization complete, power rails stabilized");
    // === End M5Stack CoreS3 Power Management Initialization ===

    // Initialize WiFi directly in main function
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);

    info!("Initializing WiFi...");
    let esp_wifi_ctrl = wifi::create_wifi_controller(timg0, rng);
    info!("WiFi controller initialized");

    let (wifi_controller, _interfaces) = esp_wifi::wifi::new(esp_wifi_ctrl, peripherals.WIFI)
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

    let backend = Box::new(crate::ui::EspEmbassyBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("Custom Slint backend initialized");

    // Initial liveness check
    info!("System initialization complete - M5Stack CoreS3 is alive and ready");
    // Create the UI
    let ui = crate::ui::MainWindow::new().unwrap();

    // Create empty WiFi network model with some placeholder data
    let placeholder_networks = vec![
        crate::ui::WifiNetwork {
            ssid: "WiFi Scanning...".into(),
        },
        crate::ui::WifiNetwork {
            ssid: "Please wait".into(),
        },
    ];

    let wifi_model = Rc::new(slint::VecModel::<crate::ui::WifiNetwork>::from(
        placeholder_networks,
    ));
    ui.set_wifi_network_model(wifi_model.clone().into());

    // Set up WiFi refresh handler with real WiFi functionality
    ui.on_wifi_refresh(move || {
        info!("WiFi refresh requested - checking for scan results");

        // Check if we have new scan results
        if crate::wifi::WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            // Access the scan results
            let scan_results = crate::wifi::WIFI_SCAN_RESULTS.try_lock();
            if let Ok(results) = scan_results {
                let mut networks = alloc::vec::Vec::new();

                for ap in results.iter() {
                    networks.push(crate::ui::WifiNetwork {
                        ssid: ap.ssid.as_str().into(),
                    });
                }

                if networks.is_empty() {
                    networks.push(crate::ui::WifiNetwork {
                        ssid: "No networks found".into(),
                    });
                }

                info!("Updated UI with {} real networks", networks.len());
                wifi_model.set_vec(networks);

                // Reset the update flag
                crate::wifi::WIFI_SCAN_UPDATED.store(false, Ordering::Relaxed);
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
    spawner.spawn(wifi::wifi_scan_task(wifi_ctrl)).ok();

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
                    info!("Touch controller test: No touch or error: {e:?}");
                }
            }
            info!("Touch controller test completed - polling functionality verified");
            info!("Touch events will now be logged when you touch the screen");
            Some(touch_driver)
        }
        Err(e) => {
            error!("Touch initialization failed: {e:?}");
            info!("Continuing without touch functionality");
            None
        }
    };
    // === End Touch Controller Initialization ===

    // Note: WiFi task already spawned above

    use slint::ComponentHandle;

    // Spawn graphics rendering task on same core
    info!("Spawning graphics rendering task on Core 0");
    spawner
        .spawn(crate::ui::ui_update_task(window.clone(), ui.as_weak()))
        .ok();
    info!("Spawning DHT22 task");
    let mut dht_pin = esp_hal::gpio::Flex::new(peripherals.GPIO5);
    let output_config = esp_hal::gpio::OutputConfig::default()
        .with_drive_mode(esp_hal::gpio::DriveMode::OpenDrain)
        .with_pull(esp_hal::gpio::Pull::None);
    dht_pin.apply_output_config(&output_config);
    dht_pin.set_input_enable(true);
    dht_pin.set_output_enable(true);
    dht_pin.set_level(esp_hal::gpio::Level::High);

    spawner.spawn(dht22::dht22_task(dht_pin)).ok();

    // === Touch Polling Integration ===
    info!("Starting continuous touch polling and Slint integration...");

    let mut status_counter = 0u32;
    let mut touch_ticker = Ticker::every(Duration::from_millis(16)); // ~60Hz touch polling
    let mut last_touch_state = TouchState::Released;

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
