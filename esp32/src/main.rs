#![no_std]
#![no_main]

extern crate alloc;

mod display;

use alloc::boxed::Box;
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
    wifi::{ClientConfiguration, Configuration, WifiController, AccessPointInfo},
};
use embassy_time::{Duration, Timer};
use alloc::string::String;
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

slint::include_modules!();

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> = Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// Custom Slint backend for Embassy integration
struct EspEmbassyBackend {
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl EspEmbassyBackend {
    fn new(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl slint::platform::Platform for EspEmbassyBackend {
    fn create_window_adapter(&self) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        embassy_time::Instant::now().duration_since(embassy_time::Instant::from_secs(0)).into()
    }
}

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
    esp_alloc::heap_allocator!(size: 180 * 1024);
    
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
    
    // Initialize embassy timer for task scheduling BEFORE spawning tasks
    use esp_hal::timer::systimer::SystemTimer;
    let systimer = SystemTimer::new(peripherals.SYSTIMER);
    esp_hal_embassy::init(systimer.alarm0);
    info!("Embassy timer initialized");
    
    // Don't initialize the standard display platform - we'll use a custom one
    
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
    
    // Store WiFi controller for the render loop task
    let wifi_ctrl = wifi_controller;
    
    // Create custom Slint window and backend
    let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
        slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
    );
    window.set_size(slint::PhysicalSize::new(320, 240));
    
    let backend = Box::new(EspEmbassyBackend::new(window.clone()));
    slint::platform::set_platform(backend).expect("backend already initialized");
    info!("Custom Slint backend initialized");

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
        info!("WiFi refresh requested - checking for scan results");
        
        // Check if we have new scan results
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            // Access the scan results
            let scan_results = WIFI_SCAN_RESULTS.try_lock();
            if let Ok(results) = scan_results {
                let mut networks = alloc::vec::Vec::new();
                
                for ap in results.iter() {
                    networks.push(WifiNetwork {
                        ssid: ap.ssid.as_str().into(),
                    });
                }
                
                if networks.is_empty() {
                    networks.push(WifiNetwork {
                        ssid: "No networks found".into(),
                    });
                }
                
                info!("Updated UI with {} real networks", networks.len());
                wifi_model.set_vec(networks);
                
                // Reset the update flag
                WIFI_SCAN_UPDATED.store(false, Ordering::Relaxed);
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
    spawner.spawn(wifi_scan_task(wifi_ctrl)).ok();
    
    // Spawn render loop task
    info!("Spawning render loop task");
    spawner.spawn(render_loop_task(window, ui.as_weak())).ok();
    
    // Keep the main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_secs(1)).await;
    }
}

// Render loop task for Embassy integration
#[embassy_executor::task]
async fn render_loop_task(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>, ui: slint::Weak<MainWindow>) {
    info!("=== Render loop task started ====");
    
    // Touch state tracking - move outside the loop
    let mut last_touch = None;
    
    loop {
        // Update timers and animations
        slint::platform::update_timers_and_animations();
        
        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                info!("Triggered UI refresh for new WiFi scan results");
            }
        }
        
        // Handle touch input and rendering
        if let Some(display_hardware) = unsafe { display::DISPLAY_COMPONENTS.as_mut() } {
            // Handle touch input
            match display_hardware.touch.get_touch(&mut display_hardware.i2c) {
                Ok(Some(point)) => {
                    let pos = slint::PhysicalPosition::new(point.x as i32, point.y as i32)
                        .to_logical(window.scale_factor());
                    
                    let event = if let Some(previous_pos) = last_touch.replace(pos) {
                        if previous_pos != pos {
                            info!("Touch moved: {:?} -> {:?}", previous_pos, pos);
                            slint::platform::WindowEvent::PointerMoved { position: pos }
                        } else {
                            continue;
                        }
                    } else {
                        info!("Touch pressed at: {:?}", pos);
                        slint::platform::WindowEvent::PointerPressed {
                            position: pos,
                            button: slint::platform::PointerEventButton::Left,
                        }
                    };
                    
                    window.dispatch_event(event);
                }
                Ok(None) => {
                    if let Some(pos) = last_touch.take() {
                        info!("Touch released at: {:?}", pos);
                        window.dispatch_event(slint::platform::WindowEvent::PointerReleased {
                            position: pos,
                            button: slint::platform::PointerEventButton::Left,
                        });
                        window.dispatch_event(slint::platform::WindowEvent::PointerExited);
                    }
                }
                Err(_) => {
                    // Ignore touch errors silently to avoid spam
                }
            }
            
            // Render if needed
            window.draw_if_needed(|renderer| {
                let mut buffer_provider = display::HardwareDrawBuffer {
                    display: &mut display_hardware.display,
                    buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
                };
                renderer.render_by_line(&mut buffer_provider);
            });
        }
        
        // Small delay to prevent busy waiting
        embassy_time::Timer::after(embassy_time::Duration::from_millis(16)).await; // ~60fps
    }
}

// WiFi scanning task
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("=== WiFi scan task started ====");
    
    // Start WiFi
    let client_config = Configuration::Client(ClientConfiguration {
        ssid: String::new(),
        password: String::new(),
        ..Default::default()
    });
    
    match wifi_controller.set_configuration(&client_config) {
        Ok(_) => info!("WiFi configuration set successfully"),
        Err(e) => info!("Failed to set WiFi configuration: {:?}", e),
    }
    
    match wifi_controller.start_async().await {
        Ok(_) => info!("WiFi started successfully!"),
        Err(e) => info!("Failed to start WiFi: {:?}", e),
    }
    
    // Wait a bit for WiFi to initialize
    embassy_time::Timer::after(embassy_time::Duration::from_secs(2)).await;
    
    loop {
        info!("Performing WiFi scan...");
        
        match wifi_controller.scan_n_async(10).await {
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
                
                // Store scan results in shared state
                if let Ok(mut scan_results) = WIFI_SCAN_RESULTS.try_lock() {
                    scan_results.clear();
                    scan_results.extend_from_slice(&results);
                    WIFI_SCAN_UPDATED.store(true, Ordering::Relaxed);
                    info!("Stored {} scan results for UI", scan_results.len());
                } else {
                    info!("Could not store scan results (mutex locked)");
                }
            }
            Err(e) => {
                info!("WiFi scan failed: {:?}", e);
            }
        }
        
        // Wait 10 seconds before next scan
        embassy_time::Timer::after(embassy_time::Duration::from_secs(10)).await;
    }
}

