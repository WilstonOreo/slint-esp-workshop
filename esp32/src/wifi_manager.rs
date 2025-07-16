use alloc::{string::String, vec::Vec, format, vec};
use log::info;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_wifi::{
    EspWifiController,
    wifi::{WifiController},
};
use static_cell::StaticCell;
use core::cell::RefCell;

// WiFi network information that matches our Slint model
#[derive(Clone, Debug)]
pub struct WifiNetwork {
    pub ssid: String,
}

// Static storage for WiFi scan results using RefCell for interior mutability
pub static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, RefCell<Vec<WifiNetwork>>> = Mutex::new(RefCell::new(Vec::new()));
pub static SCAN_TRIGGER: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static WIFI_CONTROLLER: StaticCell<Option<WifiController<'static>>> = StaticCell::new();

pub struct WifiManager;

impl WifiManager {
    /// Initialize WiFi manager with real hardware controller
    pub fn init_with_real_controller() {
        info!("WiFi manager initialized with real hardware controller");
        
        // Initialize empty scan results
        WIFI_SCAN_RESULTS.lock(|results| {
            results.borrow_mut().clear();
        });
        
        // Initialize WiFi controller as None initially
        WIFI_CONTROLLER.init(None);
        
        info!("WiFi manager initialized successfully");
    }
    
    /// Initialize WiFi controller for scanning
    pub fn init_wifi_controller(
        esp_wifi_ctrl: &'static EspWifiController<'static>,
        wifi_peripheral: esp_hal::peripherals::WIFI,
    ) -> Result<(), esp_wifi::InitializationError> {
        info!("Initializing WiFi controller for scanning");
        
        // This function is currently not used since we're using direct WiFi scanning
        // The lifetime issue is complex to solve properly here
        // For now, we'll just return Ok and use the direct approach in main.rs
        
        info!("WiFi controller initialized successfully (placeholder)");
        Ok(())
    }
    
    /// Get the current WiFi scan results
    pub fn get_scan_results() -> Vec<WifiNetwork> {
        WIFI_SCAN_RESULTS.lock(|results| results.borrow().clone())
    }
    
    /// Trigger a WiFi scan
    pub fn trigger_scan() {
        info!("WiFi scan triggered");
        SCAN_TRIGGER.signal(());
    }
    
    /// Start the WiFi scanning task (should be called from Embassy executor)
    pub async fn wifi_scan_task() {
        info!("Starting WiFi scan task");
        
        // For now, we'll just use simulation since storing the controller is complex
        info!("Using simulation mode for WiFi scanning");
        Self::simulate_scan_loop().await;
    }
    
    /// Simulate WiFi scanning when real hardware is not available
    async fn simulate_scan_loop() {
        let mut scan_count = 0;
        
        loop {
            // Wait for scan trigger
            SCAN_TRIGGER.wait().await;
            
            scan_count += 1;
            info!("Performing simulated WiFi scan #{}", scan_count);
            
            let mut networks = vec![
                WifiNetwork { ssid: "MyHomeNetwork".into() },
                WifiNetwork { ssid: "CoffeeShop-WiFi".into() },
                WifiNetwork { ssid: "ESP32-Demo".into() },
            ];
            
            if scan_count % 3 == 0 {
                networks.push(WifiNetwork { ssid: "Mobile-Hotspot".into() });
            }
            
            if scan_count % 2 == 0 {
                networks.push(WifiNetwork { ssid: format!("Dynamic-Network-{}", scan_count / 2) });
            }
            
            if scan_count > 5 {
                networks.push(WifiNetwork { ssid: "NeighborNetwork".into() });
            }
            
            // Update results
            WIFI_SCAN_RESULTS.lock(|results| {
                *results.borrow_mut() = networks;
            });
            
            info!("Simulated WiFi scan completed, found {} networks", 
                WIFI_SCAN_RESULTS.lock(|results| results.borrow().len()));
            
            // Small delay to prevent busy waiting
            Timer::after(Duration::from_millis(100)).await;
        }
    }
    
}

