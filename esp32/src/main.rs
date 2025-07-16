#![no_std]
#![no_main]

extern crate alloc;

use alloc::rc::Rc;
use alloc::vec::Vec;
use core::panic::PanicInfo;
use log::info;

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
    
    // Create a simple UI without WiFi functionality for now
    let ui = MainWindow::new().unwrap();
    
    // Create empty WiFi network model
    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(Vec::new()));
    ui.set_wifi_network_model(wifi_model.clone().into());
    
    // Set up a simple refresh handler (without actual WiFi scanning)
    ui.on_wifi_refresh(move || {
        // For now, just add a placeholder entry
        let mut networks = Vec::new();
        networks.push(WifiNetwork {
            ssid: "No WiFi Available".into(),
        });
        wifi_model.set_vec(networks);
    });
    
    // Trigger initial refresh
    ui.invoke_wifi_refresh();
    
    // Run the UI
    ui.run().unwrap();
}
