#![no_std]
#![no_main]

extern crate alloc;

mod display;

use alloc::boxed::Box;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::vec;
use core::panic::PanicInfo;
use log::{debug, error, info};

// WiFi imports - simplified
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Ticker};
use esp_hal::rng::Rng;
use esp_wifi::EspWifiController;
use esp_wifi::wifi::{AccessPointInfo, ClientConfiguration, Configuration, WifiController};

// ESP32 HAL imports - only what we need
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println::logger::init_logger_from_env;

// Display imports
use display::{DISPLAY_COMPONENTS, HardwareDrawBuffer};

// Slint platform imports
use slint::PhysicalPosition;
use slint::platform::software_renderer::Rgb565Pixel;
use slint::platform::{PointerEventButton, WindowEvent};

// Touch controller imports

slint::include_modules!();

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// Display constants for ESP32-S3-Box-3 - 320x240 ILI9486
const LCD_H_RES: u16 = 320;
const LCD_V_RES: u16 = 240;
const LCD_H_RES_USIZE: usize = 320;
const LCD_V_RES_USIZE: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

struct EspEmbassyBackend {
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
}

impl EspEmbassyBackend {
    fn new(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) -> Self {
        Self { window }
    }
}

impl slint::platform::Platform for EspEmbassyBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        Ok(self.window.clone())
    }

    fn duration_since_start(&self) -> core::time::Duration {
        embassy_time::Instant::now()
            .duration_since(embassy_time::Instant::from_secs(0))
            .into()
    }
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    error!("PANIC: {}", info);
    loop {}
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
    info!(
        "Initializing PSRAM heap: start: {:p}, size: {}",
        start, size
    );
    unsafe {
        esp_alloc::HEAP.add_region(esp_alloc::HeapRegion::new(
            start,
            size,
            esp_alloc::MemoryCapability::External.into(),
        ));
    }
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

    let (wifi_controller, _interfaces) = esp_wifi::wifi::new(&esp_wifi_ctrl, peripherals.WIFI)
        .expect("Failed to create WiFi interface");
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
    )
    .expect("Failed to initialize display hardware");

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

    // Spawn graphics rendering task
    info!("Spawning graphics rendering task");
    spawner
        .spawn(graphics_task(window.clone(), ui.as_weak()))
        .ok();

    // === Touch Polling Integration ===
    info!("Starting continuous touch polling and Slint integration...");

    let mut status_counter = 0u32;
    let mut touch_ticker = Ticker::every(Duration::from_millis(16)); // ~60Hz touch polling
    let mut last_touch_state: Option<gt911::Point> = None;
    let mut last_touch_position = slint::LogicalPosition::new(0.0, 0.0);

    loop {
        // Poll touch events if touch controller is available
        DISPLAY_COMPONENTS.with_mut(|display_hardware| {
            if let Ok(touch_data_option) = display_hardware.touch.get_touch(&mut display_hardware.i2c) {
                match (last_touch_state.as_ref(), touch_data_option.as_ref()) {
                    // Touch press event (transition from None to Some)
                    (None, Some(touch_data)) => {
                        let physical_position =
                            PhysicalPosition::new(touch_data.x as i32, touch_data.y as i32);
                        let logical_position =
                            physical_position.to_logical(window.scale_factor());
                        last_touch_position = logical_position;

                        let pointer_event = WindowEvent::PointerPressed {
                            position: logical_position,
                            button: PointerEventButton::Left,
                        };

                        window.dispatch_event(pointer_event);
                        info!(
                            "Touch PRESSED at x={}, y={} (logical: {:.1}, {:.1}, scale_factor={})",
                            touch_data.x,
                            touch_data.y,
                            logical_position.x,
                            logical_position.y,
                            window.scale_factor()
                        );
                    }
                    // Touch release event (transition from Some to None)
                    (Some(_), None) => {
                        // Send PointerReleased at the last known position
                        let pointer_released = WindowEvent::PointerReleased {
                            position: last_touch_position,
                            button: PointerEventButton::Left,
                        };
                        window.dispatch_event(pointer_released);

                        // Also send PointerExited to complete the interaction cycle
                        let pointer_exited = WindowEvent::PointerExited;
                        window.dispatch_event(pointer_exited);

                        info!(
                            "Touch RELEASED at (logical: {:.1}, {:.1}) + EXITED",
                            last_touch_position.x,
                            last_touch_position.y
                        );
                    }
                    // Touch move event (both states are Some but potentially different positions)
                    (Some(old_touch), Some(new_touch)) => {
                        // Only dispatch move event if position actually changed
                        if old_touch.x != new_touch.x || old_touch.y != new_touch.y {
                            let physical_position =
                                PhysicalPosition::new(new_touch.x as i32, new_touch.y as i32);
                            let logical_position =
                                physical_position.to_logical(window.scale_factor());
                            last_touch_position = logical_position;

                            let pointer_event = WindowEvent::PointerMoved {
                                position: logical_position,
                            };

                            window.dispatch_event(pointer_event);
                            debug!(
                                "Touch MOVED to x={}, y={} (logical: {:.1}, {:.1}, scale_factor={})",
                                new_touch.x,
                                new_touch.y,
                                logical_position.x,
                                logical_position.y,
                                window.scale_factor()
                            );
                        }
                    }
                    // No state change
                    _ => {}
                }

                last_touch_state = touch_data_option;
            }
        });

        // Status logging (less frequent than touch polling)
        if status_counter % 600 == 0 {
            // Every ~10 seconds at 60Hz
            info!(
                "Main task status check #{} - ESP32-S3-Box-3 alive with touch polling",
                status_counter / 60
            );
        }

        status_counter += 1;
        touch_ticker.next().await;
    }
}

// Graphics rendering task - handles display output only
#[embassy_executor::task]
async fn graphics_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    info!("=== Graphics rendering task started ====");

    let mut ticker = Ticker::every(Duration::from_millis(16)); // ~60fps
    let mut frame_counter = 0u32;

    // Create pixel buffer for Slint rendering
    let mut pixel_buffer: Box<[Rgb565Pixel; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565Pixel(0); LCD_BUFFER_SIZE]);

    info!(
        "Graphics task initialized with {}x{} buffer",
        LCD_H_RES, LCD_V_RES
    );

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Check for new WiFi scan results and trigger UI refresh if available
        if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
            if let Some(ui_strong) = ui.upgrade() {
                ui_strong.invoke_wifi_refresh();
                debug!("Triggered UI refresh for new WiFi scan results");
            }
        }

        // Render the frame using the hardware display
        let rendered = window.draw_if_needed(|renderer| {
            // Access the global display instance
            if let Some(()) = DISPLAY_COMPONENTS.with_mut(|display_hardware| {
                // Create hardware draw buffer
                let mut hardware_buffer =
                    HardwareDrawBuffer::new(&mut display_hardware.display, &mut *pixel_buffer);

                // Render by line to the hardware display
                renderer.render_by_line(&mut hardware_buffer);

                if frame_counter % 60 == 0 {
                    debug!("Frame {} rendered to hardware display", frame_counter);
                }
            }) {
                // Successfully rendered
            } else {
                error!("Display not available in graphics task!");
            }
        });

        // If a frame was rendered, log it
        if rendered {
            if frame_counter % 60 == 0 {
                debug!(
                    "Frame {} rendered and displayed on ESP32-S3-Box-3",
                    frame_counter
                );
            }
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            // Every ~5 seconds at 60fps
            info!(
                "Graphics: Frame {}, ESP32-S3-Box-3 display active",
                frame_counter
            );
        }

        ticker.next().await;
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
