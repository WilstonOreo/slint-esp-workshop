#![no_std]
#![no_main]

extern crate alloc;

use alloc::rc::Rc;
use alloc::vec;
use alloc::vec::Vec;
use core::panic::PanicInfo;
use log::info;

use esp_alloc as _;
use esp_backtrace as _;

slint::include_modules!();

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    esp_println::println!("Panic occurred: {:?}", _info);
    loop {}
}


#[esp_hal_embassy::main]
async fn main(_spawner: embassy_executor::Spawner) {
    // Initialize the MCU board support first (this initializes hal, logger, and heap)
    mcu_board_support::init();
    
    info!("Starting Slint ESP32-S3 Workshop");
    
    // For now, let's create a simple fallback that doesn't require WiFi
    info!("WiFi functionality temporarily disabled due to initialization order");
    
    // Create a simple UI
    let ui = MainWindow::new().unwrap();
    
    // Create empty WiFi network model with some placeholder data
    let placeholder_networks = vec![
        WifiNetwork { ssid: "WiFi Disabled".into() },
        WifiNetwork { ssid: "Please check code".into() },
        WifiNetwork { ssid: "for WiFi integration".into() },
    ];
    
    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(placeholder_networks));
    ui.set_wifi_network_model(wifi_model.clone().into());
    
    // Set up WiFi refresh handler with placeholder functionality
    ui.on_wifi_refresh(move || {
        info!("WiFi refresh requested - showing placeholder data");
        
        // Show placeholder networks
        let placeholder_networks = vec![
            WifiNetwork { ssid: "Network 1".into() },
            WifiNetwork { ssid: "Network 2".into() },
            WifiNetwork { ssid: "Network 3".into() },
        ];
        
        info!("Updated UI with {} placeholder networks", placeholder_networks.len());
        wifi_model.set_vec(placeholder_networks);
    });
    
    // Trigger initial refresh
    ui.invoke_wifi_refresh();
    
    // Run the UI
    ui.run().unwrap();
}
