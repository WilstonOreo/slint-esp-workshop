#![no_std]

extern crate alloc;
use alloc::{string::String, vec::Vec};

/// Simple Struct for storing WiFi Network data.
pub struct WifiNetwork {
    pub ssid: String,
}

/// A trait to provide wifi data.
///
/// To be implemented for each app platform.
pub trait WifiNetworkProvider {
    fn scan_wifi_networks(&self) -> Vec<WifiNetwork>;
}

pub struct Model;

#[cfg(target_os = "linux")]
impl WifiNetworkProvider for Model {
    fn scan_wifi_networks(&self) -> Vec<WifiNetwork> {
        // For Linux, we need std for process execution
        extern crate std;
        use alloc::vec;
        
        String::from_utf8(
            std::process::Command::new("nmcli")
                .arg("-t")
                .arg("-f")
                .arg("SSID,BSSID,SIGNAL")
                .arg("dev")
                .arg("wifi")
                .output()
                .unwrap()
                .stdout,
        )
        .unwrap()
        .split('\n')
        .filter_map(|s| {
            s.split_once(":").map(|(ssid, _)| WifiNetwork {
                ssid: ssid.to_string(),
            })
        })
        .collect::<Vec<_>>()
    }
}

// ESP32 or other embedded targets
#[cfg(not(target_os = "linux"))]
impl WifiNetworkProvider for Model {
    fn scan_wifi_networks(&self) -> Vec<WifiNetwork> {
        use alloc::vec;
        
        vec![
            WifiNetwork {
                ssid: "ESP32 Network A".into(),
            },
            WifiNetwork {
                ssid: "ESP32 Network B".into(),
            },
        ]
    }
}
