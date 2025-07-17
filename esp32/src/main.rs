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
use esp_hal::{rng::Rng, timer::timg::TimerGroup};
use esp_println::logger::init_logger_from_env;
use esp_wifi::{
    EspWifiController,
    wifi::{ClientConfiguration, Configuration, WifiController},
};
use embassy_time::{Duration, Timer};
use alloc::string::String;

slint::include_modules!();

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
    info!("Initializing PSRAM heap: start: {:p}, size: {}", start, size);
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    esp_println::println!("Panic occurred: {:?}", _info);
    loop {}
}

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize peripherals first
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::_240MHz));
    
    // Initialize BOTH heap allocators - WiFi first in internal RAM, then PSRAM for GUI
    // Step 1: Initialize internal RAM heap for WiFi (must be first)
    esp_alloc::heap_allocator!(size: 120 * 1024);
    
    // Step 2: Initialize PSRAM heap for GUI and other data
    init_heap(&peripherals.PSRAM);
    
    // Initialize logger
    init_logger_from_env();
    info!("Peripherals initialized");
    
    info!("Starting Slint ESP32-S3 Workshop");
    
    // Initialize WiFi directly in main function
    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let rng = Rng::new(peripherals.RNG);
    
    info!("Initializing WiFi...");
    let esp_wifi_ctrl = &*mk_static!(
        EspWifiController<'static>,
        esp_wifi::init(timg0.timer0, rng.clone()).expect("Failed to initialize WiFi")
    );
    info!("WiFi controller initialized");
    
    let (wifi_controller, _interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI).expect("Failed to create WiFi interface");
    info!("WiFi interface created");
    
    // Initialize display platform
    display::init().expect("Failed to initialize display platform");
    
    // Initialize embassy timer for task scheduling
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    
    // Initialize display hardware with specific peripherals
    display::init_display_hardware(
        peripherals.GPIO3,
        peripherals.GPIO4,
        peripherals.GPIO5,
        peripherals.GPIO6,
        peripherals.GPIO7,
        peripherals.GPIO8,
        peripherals.GPIO18,
        peripherals.GPIO47,
        peripherals.GPIO48,
        peripherals.SPI2,
        peripherals.I2C0,
    ).expect("Failed to initialize display hardware");
    
    // Start WiFi scanning task
    spawner.spawn(wifi_scan_task(wifi_controller)).ok();
    
    // Initial liveness check
    info!("System initialization complete - board is alive and ready");
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
    info!("Starting UI event loop - board ready for user interaction");
    ui.run().unwrap();
}

// WiFi scanning task
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("Starting WiFi scan task");
    info!("Device capabilities: {:?}", wifi_controller.capabilities());
    
    // Set up the WiFi controller in station mode
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::new(),
        password: String::new(),
        ..Default::default()
    });
    
    match wifi_controller.set_configuration(&client_config) {
        Ok(_) => info!("WiFi configuration set successfully"),
        Err(e) => {
            info!("Failed to set WiFi configuration: {:?}", e);
            return;
        }
    }
    
    // Start WiFi
    info!("Starting WiFi...");
    match wifi_controller.start_async().await {
        Ok(_) => info!("WiFi started successfully!"),
        Err(e) => {
            info!("Failed to start WiFi: {:?}", e);
            return;
        }
    }
    
    // Wait a bit for WiFi to initialize
    Timer::after(Duration::from_secs(2)).await;
    
    loop {
        info!("Performing WiFi scan...");
        
        // Perform a scan for up to 16 networks
        match wifi_controller.scan_n_async(16).await {
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
                
                // TODO: Update UI model with scan results
                // This requires a channel or shared state mechanism
            }
            Err(e) => {
                info!("WiFi scan failed: {:?}", e);
            }
        }
        
        // Wait 10 seconds before next scan
        info!("Board is alive - next scan in 10 seconds");
        Timer::after(Duration::from_secs(10)).await;
    }
}
