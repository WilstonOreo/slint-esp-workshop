#![no_std]
#![no_main]

extern crate alloc;

mod wifi_manager;

use alloc::rc::Rc;
use alloc::vec::Vec;
use core::panic::PanicInfo;
use log::info;
use wifi_manager::WifiManager;

slint::include_modules!();

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    esp_println::println!("Panic occurred: {:?}", _info);
    loop {}
}

#[no_mangle]
fn main() {
    // Initialize the MCU board support
    mcu_board_support::init();
    
    info!("Starting Slint ESP32-S3 Workshop");
    
    // Initialize WiFi manager
    WifiManager::init_with_real_controller();
    
    // Create a simple UI
    let ui = MainWindow::new().unwrap();
    
    // Create empty WiFi network model
    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(Vec::new()));
    ui.set_wifi_network_model(wifi_model.clone().into());
    
    // Set up WiFi refresh handler that uses the WiFi manager
    ui.on_wifi_refresh(move || {
        info!("WiFi refresh requested");
        
        // Trigger WiFi scan
        WifiManager::trigger_scan();
        
        // Get current scan results
        let networks = WifiManager::get_scan_results();
        
        // Convert to Slint WiFi network format
        let slint_networks: Vec<WifiNetwork> = networks
            .iter()
            .map(|network| WifiNetwork {
                ssid: network.ssid.clone().into(),
            })
            .collect();
        
        info!("Updated UI with {} networks", slint_networks.len());
        wifi_model.set_vec(slint_networks);
    });
    
    // Trigger initial refresh
    ui.invoke_wifi_refresh();
    
    // Run the UI
    ui.run().unwrap();
}
