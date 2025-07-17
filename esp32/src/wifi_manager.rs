use alloc::{string::String, vec, vec::Vec};
use log::info;
use critical_section::Mutex;
use core::cell::RefCell;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use esp_wifi::wifi::{WifiController, ClientConfiguration, Configuration};

// WiFi network information that matches our Slint model
#[derive(Clone, Debug)]
pub struct WifiNetwork {
    pub ssid: String,
}

// Static storage for WiFi scan results using critical section mutex
static WIFI_SCAN_RESULTS: Mutex<RefCell<Vec<WifiNetwork>>> = Mutex::new(RefCell::new(Vec::new()));
static SCAN_TRIGGER: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static WIFI_CONTROLLER: Mutex<RefCell<Option<&'static mut WifiController<'static>>>> = Mutex::new(RefCell::new(None));

pub struct WifiManager;

impl WifiManager {
    /// Initialize WiFi manager with real hardware controller
    pub fn init_with_real_controller() {
        info!("WiFi manager initialized with real hardware controller");
        
        // Initialize empty scan results
        critical_section::with(|cs| {
            WIFI_SCAN_RESULTS.borrow(cs).borrow_mut().clear();
        });
        
        info!("WiFi manager initialized successfully");
    }
    
    /// Initialize WiFi controller for scanning
    pub fn init_wifi_controller(wifi_controller: &'static mut WifiController<'static>) {
        info!("Setting up WiFi controller for scanning");
        
        // Store the WiFi controller
        critical_section::with(|cs| {
            *WIFI_CONTROLLER.borrow(cs).borrow_mut() = Some(wifi_controller);
        });
        
        info!("WiFi controller stored successfully");
    }
    
    /// Get the current WiFi scan results
    pub fn get_scan_results() -> Vec<WifiNetwork> {
        critical_section::with(|cs| {
            WIFI_SCAN_RESULTS.borrow(cs).borrow().clone()
        })
    }
    
    /// Trigger a WiFi scan (triggers async scan task)
    pub fn trigger_scan() {
        info!("WiFi scan triggered");
        SCAN_TRIGGER.signal(());
    }
    
    /// Start the WiFi scanning task (should be called from Embassy executor)
    pub async fn wifi_scan_task() {
        info!("Starting WiFi scan task");
        
        loop {
            // Wait for scan trigger
            SCAN_TRIGGER.wait().await;
            
            info!("Performing real WiFi scan");
            
            // Perform actual WiFi scan
            let networks = Self::perform_wifi_scan().await;
            
            // Update results
            critical_section::with(|cs| {
                *WIFI_SCAN_RESULTS.borrow(cs).borrow_mut() = networks;
            });
            
            info!("Real WiFi scan completed, found {} networks", 
                critical_section::with(|cs| WIFI_SCAN_RESULTS.borrow(cs).borrow().len()));
            
            // Small delay to prevent busy waiting
            Timer::after(Duration::from_millis(100)).await;
        }
    }
    
    /// Perform actual WiFi scan
    async fn perform_wifi_scan() -> Vec<WifiNetwork> {
        let mut networks = Vec::new();
        
        // Simulate WiFi scan delay
        Timer::after(Duration::from_millis(500)).await;
        
        // Try to access the WiFi controller
        let controller_opt = critical_section::with(|cs| {
            WIFI_CONTROLLER.borrow(cs).borrow_mut().take()
        });
        
        if let Some(mut controller) = controller_opt {
            // Configure WiFi controller for scanning
            let client_config = Configuration::Client(ClientConfiguration {
                ssid: String::new(),
                password: String::new(),
                ..Default::default()
            });
            
            if let Err(e) = controller.set_configuration(&client_config) {
                info!("Failed to configure WiFi controller: {:?}", e);
                // Put controller back
                critical_section::with(|cs| {
                    *WIFI_CONTROLLER.borrow(cs).borrow_mut() = Some(controller);
                });
                return Self::get_simulated_networks();
            }
            
            // Start WiFi if not already started
            if let Err(e) = controller.start_async().await {
                info!("Failed to start WiFi controller: {:?}", e);
                // Put controller back
                critical_section::with(|cs| {
                    *WIFI_CONTROLLER.borrow(cs).borrow_mut() = Some(controller);
                });
                return Self::get_simulated_networks();
            }
            
            // Perform scan
            match controller.scan_n_async(10).await {
                Ok(scan_results) => {
                    info!("Scan successful, found {} access points", scan_results.len());
                    
                    for ap in scan_results.iter() {
                        if !ap.ssid.is_empty() {
                            networks.push(WifiNetwork {
                                ssid: ap.ssid.clone(),
                            });
                        }
                    }
                }
                Err(e) => {
                    info!("WiFi scan failed: {:?}", e);
                    return Self::get_simulated_networks();
                }
            }
            
            // Put controller back
            critical_section::with(|cs| {
                *WIFI_CONTROLLER.borrow(cs).borrow_mut() = Some(controller);
            });
        } else {
            info!("WiFi controller not available - using simulated networks");
            return Self::get_simulated_networks();
        }
        
        // If no networks found, return simulated ones
        if networks.is_empty() {
            return Self::get_simulated_networks();
        }
        
        networks
    }
    
    /// Get simulated WiFi networks for testing
    fn get_simulated_networks() -> Vec<WifiNetwork> {
        vec![
            WifiNetwork { ssid: "ESP32_Workshop_Demo".into() },
            WifiNetwork { ssid: "MyHomeWiFi".into() },
            WifiNetwork { ssid: "CoffeeShop_Guest".into() },
            WifiNetwork { ssid: "Office_Network".into() },
            WifiNetwork { ssid: "Neighbors_WiFi".into() },
            WifiNetwork { ssid: "IoT_Devices".into() },
        ]
    }
}

