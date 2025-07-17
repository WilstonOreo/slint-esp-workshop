#![no_std]
#![no_main]

extern crate alloc;

mod display;

use alloc::rc::Rc;
use alloc::vec;
use core::panic::PanicInfo;
use log::info;

use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::peripherals::Peripherals;
use esp_println::logger::init_logger_from_env;

slint::include_modules!();

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    esp_println::println!("Panic occurred: {:?}", _info);
    loop {}
}

#[esp_hal_embassy::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize peripherals first
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));

    // Initialize logger
    init_logger_from_env();
    info!("Peripherals initialized");

    // Initialize the PSRAM allocator
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    info!("Starting Slint ESP32-S3 Workshop");

    // Initialize display platform
    display::init().expect("Failed to initialize display platform");

    // Initialize display hardware (components are stored globally)
    display::init_display_hardware(peripherals).expect("Failed to initialize display hardware");

    // TODO: Initialize WiFi after display (needs separate peripheral handling)
    // let wifi_controller = init_wifi().await;

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
        info!("WiFi refresh requested - scanning for networks");

        // TODO: Implement actual WiFi scanning here
        // For now, show placeholder networks
        let placeholder_networks = vec![
            WifiNetwork {
                ssid: "Network 1".into(),
            },
            WifiNetwork {
                ssid: "Network 2".into(),
            },
            WifiNetwork {
                ssid: "Network 3".into(),
            },
        ];

        info!("Updated UI with {} networks", placeholder_networks.len());
        wifi_model.set_vec(placeholder_networks);
    });

    // Trigger initial refresh
    ui.invoke_wifi_refresh();

    // For now, we need to integrate the event loop with ui.run()
    // The proper integration will be done after we fix the current approach

    // TODO: Integrate the hardware event loop with Slint's UI event loop
    // For now, just run the UI to show it works
    ui.run().unwrap();
}

// WiFi initialization function
async fn init_wifi(_peripherals: &Peripherals) -> Result<(), &'static str> {
    // TODO: Implement WiFi initialization here
    // This would use peripherals.TIMG0, peripherals.RNG, peripherals.RADIO_CLK, peripherals.WIFI
    // For now, just return Ok
    info!("WiFi initialization placeholder");
    Ok(())
}
