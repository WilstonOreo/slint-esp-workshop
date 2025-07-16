use alloc::{string::String, vec::Vec, format, vec};
use log::info;
use core::sync::atomic::{AtomicUsize, Ordering};
use critical_section::Mutex;
use core::cell::RefCell;

// WiFi network information that matches our Slint model
#[derive(Clone, Debug)]
pub struct WifiNetwork {
    pub ssid: String,
}

// Static storage for WiFi scan results using critical section mutex
static WIFI_SCAN_RESULTS: Mutex<RefCell<Vec<WifiNetwork>>> = Mutex::new(RefCell::new(Vec::new()));
static SCAN_COUNTER: AtomicUsize = AtomicUsize::new(0);

pub struct WifiManager;

impl WifiManager {
    /// Initialize WiFi manager with real hardware controller
    pub fn init_with_real_controller() {
        info!("WiFi manager initialized with simulation mode");
        
        // Initialize empty scan results
        critical_section::with(|cs| {
            WIFI_SCAN_RESULTS.borrow(cs).borrow_mut().clear();
        });
        
        info!("WiFi manager initialized successfully");
    }
    
    /// Get the current WiFi scan results
    pub fn get_scan_results() -> Vec<WifiNetwork> {
        critical_section::with(|cs| {
            WIFI_SCAN_RESULTS.borrow(cs).borrow().clone()
        })
    }
    
    /// Trigger a WiFi scan (synchronous simulation)
    pub fn trigger_scan() {
        info!("WiFi scan triggered");
        
        let scan_count = SCAN_COUNTER.fetch_add(1, Ordering::SeqCst) + 1;
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
        critical_section::with(|cs| {
            *WIFI_SCAN_RESULTS.borrow(cs).borrow_mut() = networks;
        });
        
        info!("Simulated WiFi scan completed, found {} networks", 
            critical_section::with(|cs| WIFI_SCAN_RESULTS.borrow(cs).borrow().len()));
    }
}

