#![no_std]
#![no_main]

extern crate alloc;

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
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Ticker};
use esp_hal::rng::Rng;
use esp_wifi::EspWifiController;
use esp_wifi::wifi::{AccessPointInfo, ClientConfiguration, Configuration, WifiController};

// ESP32 HAL imports - only what we need
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::gpio::{Level, Output, OutputConfig};
use esp_hal::i2c::master::I2c;
use esp_hal::peripherals::Peripherals;
use esp_hal::spi::Mode as SpiMode;
use esp_hal::spi::master::{Config as SpiConfig, Spi};
// use esp_hal::system::Stack; // Commented out as multicore not currently used
use esp_hal::time::{Instant, Rate};
use esp_hal::timer::{AnyTimer, timg::TimerGroup};
use esp_hal_embassy::Executor;
use esp_println::logger::init_logger_from_env;
use static_cell::StaticCell;

// Display imports
use embedded_graphics_core::draw_target::DrawTarget;
use embedded_graphics_core::pixelcolor::{Rgb565, RgbColor};
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::delay::Delay;
use mipidsi::options::ColorOrder;

// Slint platform imports
use slint::PhysicalSize;
use slint::platform::software_renderer::Rgb565Pixel;

slint::include_modules!();

// Shared state for WiFi scan results
static WIFI_SCAN_RESULTS: Mutex<CriticalSectionRawMutex, alloc::vec::Vec<AccessPointInfo>> =
    Mutex::new(alloc::vec::Vec::new());
static WIFI_SCAN_UPDATED: AtomicBool = AtomicBool::new(false);

// Display constants for M5Stack CoreS3 - 320x240 ILI9341
const LCD_H_RES: u16 = 320;
const LCD_V_RES: u16 = 240;
const LCD_H_RES_USIZE: usize = 320;
const LCD_V_RES_USIZE: usize = 240;
const LCD_BUFFER_SIZE: usize = LCD_H_RES_USIZE * LCD_V_RES_USIZE;

// I2C device addresses for M5Stack CoreS3 power management
const AXP2101_ADDRESS: u8 = 0x34; // AXP2101 power management IC
const AW9523_I2C_ADDRESS: u8 = 0x58; // AW9523 GPIO expander

// Signals for coordination between cores
static PSRAM_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// Note: Multicore support not currently used, but keeping framework for future
// static mut APP_CORE_STACK: Stack<4096> = Stack::new();
// static DMA_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
// static FRAME_READY: Signal<CriticalSectionRawMutex, ()> = Signal::new();
static mut PSRAM_BUF_PTR: *mut u8 = core::ptr::null_mut();
static mut PSRAM_BUF_LEN: usize = 0;

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

/// Initialize the AXP2101 power management unit for M5Stack CoreS3
/// This implements the exact same sequence as the working custom implementation
/// Based on M5Stack CoreS3 requirements for proper display power
fn init_axp2101_power<I2C>(mut i2c_device: I2C) -> Result<(), ()>
where
    I2C: embedded_hal::i2c::I2c,
{
    info!("Initializing AXP2101 power management with M5Stack CoreS3 sequence...");

    // This sequence matches exactly the working custom implementation:
    // 1. CHG_LED register (0x69) <- 0x35 (0b00110101)
    // 2. ALDO_ENABLE register (0x90) <- 0xBF
    // 3. ALDO4 register (0x95) <- 0x1C (0b00011100)

    // Step 1: Configure charge LED (register 0x69 = 105 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x69, 0x35]).is_err() {
        error!("Failed to write to CHG_LED register (0x69)");
        return Err(());
    }
    info!("AXP2101: CHG_LED configured (0x69 <- 0x35)");

    // Step 2: Enable ALDO outputs (register 0x90 = 144 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x90, 0xBF]).is_err() {
        error!("Failed to write to ALDO_ENABLE register (0x90)");
        return Err(());
    }
    info!("AXP2101: ALDO outputs enabled (0x90 <- 0xBF)");

    // Step 3: Configure ALDO4 voltage (register 0x95 = 149 decimal)
    if i2c_device.write(AXP2101_ADDRESS, &[0x95, 0x1C]).is_err() {
        error!("Failed to write to ALDO4 register (0x95)");
        return Err(());
    }
    info!("AXP2101: ALDO4 voltage configured (0x95 <- 0x1C)");

    info!("AXP2101 power management initialized successfully with M5Stack CoreS3 sequence");
    Ok(())
}

/// Initialize the AW9523 GPIO expander for M5Stack CoreS3
/// This implements the exact same sequence as the working custom implementation
/// Critical for proper display and touch controller operation
fn init_aw9523_gpio_expander<I2C>(mut i2c_device: I2C) -> Result<(), ()>
where
    I2C: embedded_hal::i2c::I2c,
{
    info!("Initializing AW9523 GPIO expander with M5Stack CoreS3 sequence...");

    // Step 1: Configure Port 0 Configuration (register 0x02) <- 0b00000101 (0x05)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x02, 0b00000101])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 0 Configuration register (0x02)");
        return Err(());
    }
    info!("AW9523: Port 0 Configuration set (0x02 <- 0x05)");

    // Step 2: Configure Port 1 Configuration (register 0x03) <- 0b00000011 (0x03)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x03, 0b00000011])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 1 Configuration register (0x03)");
        return Err(());
    }
    info!("AW9523: Port 1 Configuration set (0x03 <- 0x03)");

    // Step 3: Configure Port 0 Output (register 0x04) <- 0b00011000 (0x18)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x04, 0b00011000])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 0 Output register (0x04)");
        return Err(());
    }
    info!("AW9523: Port 0 Output set (0x04 <- 0x18)");

    // Step 4: Configure Port 1 Output (register 0x05) <- 0b00001100 (0x0C)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x05, 0b00001100])
        .is_err()
    {
        error!("Failed to write to AW9523 Port 1 Output register (0x05)");
        return Err(());
    }
    info!("AW9523: Port 1 Output set (0x05 <- 0x0C)");

    // Step 5: Configure register 0x11 <- 0b00010000 (0x10)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x11, 0b00010000])
        .is_err()
    {
        error!("Failed to write to AW9523 register (0x11)");
        return Err(());
    }
    info!("AW9523: Register 0x11 configured (0x11 <- 0x10)");

    // Step 6: Configure register 0x13 <- 0b11111111 (0xFF)
    if i2c_device
        .write(AW9523_I2C_ADDRESS, &[0x13, 0b11111111])
        .is_err()
    {
        error!("Failed to write to AW9523 register (0x13)");
        return Err(());
    }
    info!("AW9523: Register 0x13 configured (0x13 <- 0xFF)");

    info!("AW9523 GPIO expander initialized successfully with M5Stack CoreS3 sequence");
    Ok(())
}

struct EspBackend {
    window:
        core::cell::RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
    peripherals: core::cell::RefCell<Option<Peripherals>>,
}

impl Default for EspBackend {
    fn default() -> Self {
        EspBackend {
            window: core::cell::RefCell::new(None),
            peripherals: core::cell::RefCell::new(None),
        }
    }
}

impl slint::platform::Platform for EspBackend {
    fn create_window_adapter(
        &self,
    ) -> Result<Rc<dyn slint::platform::WindowAdapter>, slint::PlatformError> {
        let window = slint::platform::software_renderer::MinimalSoftwareWindow::new(
            slint::platform::software_renderer::RepaintBufferType::ReusedBuffer,
        );
        self.window.replace(Some(window.clone()));
        Ok(window)
    }

    fn duration_since_start(&self) -> core::time::Duration {
        core::time::Duration::from_millis(Instant::now().duration_since_epoch().as_millis())
    }

    fn run_event_loop(&self) -> Result<(), slint::PlatformError> {
        info!("=== Starting M5Stack CoreS3 Event Loop ===");
        let heap_at_start = esp_alloc::HEAP.used();
        info!("Heap usage at event loop start: {} bytes", heap_at_start);

        let peripherals = self
            .peripherals
            .borrow_mut()
            .take()
            .expect("Peripherals already taken");

        // M5Stack CoreS3 Display Setup (much simpler than ESoPE!)
        info!("Setting up M5Stack CoreS3 display...");

        // Enable panel power/backlight (GPIO48 for M5Stack CoreS3)
        let mut backlight = Output::new(peripherals.GPIO48, Level::Low, OutputConfig::default());
        backlight.set_high();
        info!("M5Stack CoreS3 backlight enabled");

        // SPI Display setup for M5Stack CoreS3 (ILI9341)
        let spi = Spi::new(
            peripherals.SPI2,
            SpiConfig::default()
                .with_frequency(Rate::from_mhz(40))
                .with_mode(SpiMode::_0),
        )
        .unwrap()
        .with_sck(peripherals.GPIO36) // M5Stack CoreS3 SPI CLK
        .with_mosi(peripherals.GPIO37); // M5Stack CoreS3 SPI MOSI

        let dc = Output::new(peripherals.GPIO35, Level::Low, OutputConfig::default()); // M5Stack CoreS3 DC
        let cs = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default()); // M5Stack CoreS3 CS  
        let rst = Output::new(peripherals.GPIO34, Level::High, OutputConfig::default()); // M5Stack CoreS3 RST

        let spi_delay = Delay::new();
        let spi_device = ExclusiveDevice::new(spi, cs, spi_delay).unwrap();

        // Create static buffer using heap (much simpler than ESoPE EEPROM setup!)
        let buffer: &'static mut [u8; 512] = Box::leak(Box::new([0_u8; 512]));
        let di = mipidsi::interface::SpiInterface::new(spi_device, dc, buffer);

        let mut display_delay = Delay::new();
        display_delay.delay_ns(500_000u32);

        // Initialize ILI9341 display (M5Stack CoreS3's display controller)
        let mut _display = mipidsi::Builder::new(mipidsi::models::ILI9341Rgb565, di)
            .reset_pin(rst)
            .orientation(
                mipidsi::options::Orientation::new().rotate(mipidsi::options::Rotation::Deg0),
            )
            .color_order(ColorOrder::Rgb)
            .init(&mut display_delay)
            .unwrap();

        info!("M5Stack CoreS3 display initialized");

        // Clear the display to show it's working
        _display.clear(Rgb565::GREEN).unwrap();
        info!("M5Stack CoreS3 display cleared to green");

        // Tell Slint the window dimensions match the display resolution
        let size = PhysicalSize::new(LCD_H_RES.into(), LCD_V_RES.into());
        self.window
            .borrow()
            .as_ref()
            .expect("Window adapter not created")
            .set_size(size);

        // Allocate framebuffer in PSRAM with 64-byte alignment for DMA
        const FRAME_BYTES: usize = LCD_BUFFER_SIZE * 2;

        let layout = alloc::alloc::Layout::from_size_align(FRAME_BYTES, 64)
            .expect("Failed to create layout for framebuffer");
        let fb_ptr = unsafe { alloc::alloc::alloc(layout) };

        if fb_ptr.is_null() {
            alloc::alloc::handle_alloc_error(layout);
        }

        // Initialize the buffer
        let fb_slice = unsafe { core::slice::from_raw_parts_mut(fb_ptr, FRAME_BYTES) };
        let rgb565_slice =
            unsafe { core::slice::from_raw_parts_mut(fb_ptr as *mut Rgb565, LCD_BUFFER_SIZE) };

        // Fill with blue color to show PSRAM is working
        for pixel in rgb565_slice.iter_mut() {
            *pixel = Rgb565::BLUE;
        }

        let psram_buf: &'static mut [u8] = fb_slice;

        // Verify PSRAM buffer allocation and alignment
        let buf_ptr = psram_buf.as_ptr() as usize;
        info!("PSRAM buffer allocated at address: 0x{:08X}", buf_ptr);
        info!("PSRAM buffer length: {}", psram_buf.len());
        info!("PSRAM buffer alignment modulo 64: {}", buf_ptr % 64);
        assert!(
            buf_ptr % 64 == 0,
            "PSRAM buffer must be 64-byte aligned for DMA"
        );

        // Publish PSRAM buffer pointer and len for app core
        unsafe {
            PSRAM_BUF_PTR = psram_buf.as_mut_ptr();
            PSRAM_BUF_LEN = psram_buf.len();
        }

        // Initialize Embassy with both timers for multicore support
        info!("=== Embassy Initialization ===");
        let timg0 = TimerGroup::new(peripherals.TIMG0);
        let timer0: AnyTimer = timg0.timer0.into();
        let timg1 = TimerGroup::new(peripherals.TIMG1);
        let timer1: AnyTimer = timg1.timer0.into();

        info!("Initializing Embassy with dual timers for multicore support...");
        esp_hal_embassy::init([timer0, timer1]);

        // Signal that PSRAM is ready for the app core
        info!("Signaling PSRAM ready for app core...");
        PSRAM_READY.signal(());

        // For now, skip complex multicore DMA setup and just run Slint on main core
        info!("=== Main Core Executor Setup ===");
        static MAIN_EXECUTOR: StaticCell<Executor> = StaticCell::new();
        let executor = MAIN_EXECUTOR.init(Executor::new());
        info!("Main core executor initialized on Core 0");

        let window = self
            .window
            .borrow()
            .as_ref()
            .expect("Window not created")
            .clone();

        executor.run(|spawner| {
            match spawner.spawn(slint_main_task(window)) {
                Ok(_) => info!("Slint main task spawned successfully on Core 0"),
                Err(e) => error!("Failed to spawn Slint main task: {:?}", e),
            }

            info!("=== M5Stack CoreS3 main executor loop starting ===");
        });

        Ok(())
    }
}

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

// WiFi scanning will be added later - focusing on display first
/*
#[embassy_executor::task]
async fn wifi_scan_task(mut wifi_controller: WifiController<'static>) {
    info!("[WiFi] WiFi scan task started");

    let mut ticker = Ticker::every(Duration::from_secs(5));

    loop {
        ticker.next().await;
        info!("[WiFi] Starting WiFi scan...");

        // WiFi scanning implementation will be added here
        // For now, just simulate some networks
        let mock_networks = vec![
            WifiNetwork {
                ssid: "Mock Network 1".into(),
            },
            WifiNetwork {
                ssid: "Mock Network 2".into(),
            },
        ];

        // Store mock results
        WIFI_SCAN_UPDATED.store(true, Ordering::Relaxed);

        embassy_time::Timer::after(Duration::from_secs(1)).await;
    }
}
*/

#[embassy_executor::task]
async fn slint_main_task(window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>) {
    info!("[Slint] Main task starting, waiting for PSRAM ready signal...");

    // Wait for PSRAM to be ready
    PSRAM_READY.wait().await;
    info!("[Slint] PSRAM ready signal received!");

    // Get the PSRAM buffer
    let psram_ptr = unsafe { PSRAM_BUF_PTR };
    let psram_len = unsafe { PSRAM_BUF_LEN };

    if psram_ptr.is_null() || psram_len == 0 {
        error!(
            "[Slint] Invalid PSRAM buffer: ptr=0x{:08X}, len={}",
            psram_ptr as usize, psram_len
        );
        return;
    }

    let _fb_slice: &mut [u8] = unsafe { core::slice::from_raw_parts_mut(psram_ptr, psram_len) };

    info!(
        "[Slint] Slint task started, PSRAM buffer at: 0x{:08X}, len: {}",
        psram_ptr as usize, psram_len
    );

    // Create pixel buffer for Slint rendering in PSRAM
    info!("[Slint] Creating pixel buffer in PSRAM...");
    let mut pixel_box: Box<[Rgb565Pixel; LCD_BUFFER_SIZE]> =
        Box::new([Rgb565Pixel(0); LCD_BUFFER_SIZE]);
    let pixel_buf: &mut [Rgb565Pixel] = &mut *pixel_box;
    info!(
        "[Slint] Pixel buffer created in PSRAM, {} pixels",
        LCD_BUFFER_SIZE
    );

    // Initialize WiFi
    info!("[Slint] Initializing WiFi...");

    // We need to get fresh peripherals for WiFi - this is a simplified approach
    // In a real implementation, we'd pass the WiFi controller from the main thread
    // For now, let's create the UI without WiFi scanning to test the display

    // Create the UI
    let ui = MainWindow::new().unwrap();
    info!("[Slint] UI created");

    // Create empty WiFi network model with placeholder data
    let placeholder_networks = vec![
        WifiNetwork {
            ssid: "M5Stack CoreS3 Ready!".into(),
        },
        WifiNetwork {
            ssid: "Display Working!".into(),
        },
    ];

    let wifi_model = Rc::new(slint::VecModel::<WifiNetwork>::from(placeholder_networks));
    ui.set_wifi_network_model(wifi_model.clone().into());

    // Set up WiFi refresh handler
    ui.on_wifi_refresh({
        let wifi_model = wifi_model.clone();
        move || {
            info!("[Slint] WiFi refresh requested");

            // Check if we have new scan results
            if WIFI_SCAN_UPDATED.load(Ordering::Relaxed) {
                // For now, just show a success message
                let networks = vec![WifiNetwork {
                    ssid: "Scan feature coming soon!".into(),
                }];

                wifi_model.set_vec(networks);
                WIFI_SCAN_UPDATED.store(false, Ordering::Relaxed);
            } else {
                // Show scanning message
                let networks = vec![WifiNetwork {
                    ssid: "Scanning...".into(),
                }];
                wifi_model.set_vec(networks);
            }
        }
    });

    let mut ticker = Ticker::every(Duration::from_millis(200));
    let mut frame_counter = 0u32;

    info!("[Slint] Entering main rendering loop...");

    loop {
        // Update Slint timers and animations
        slint::platform::update_timers_and_animations();

        // Render the frame
        let rendered = window.draw_if_needed(|renderer| {
            renderer.render(pixel_buf, LCD_H_RES as usize);

            if frame_counter % 60 == 0 {
                debug!("[Slint] Frame {} rendered by Slint", frame_counter);
            }
        });

        if rendered && frame_counter % 60 == 0 {
            debug!("[Slint] Frame {} actually rendered", frame_counter);
        }

        frame_counter = frame_counter.wrapping_add(1);

        // Log periodic status
        if frame_counter % 300 == 0 {
            info!("[Slint] Frame {}, M5Stack CoreS3 running...", frame_counter);
        }

        ticker.next().await;
    }
}

// Render loop task for Embassy integration
#[embassy_executor::task]
async fn render_loop_task(
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
    ui: slint::Weak<MainWindow>,
) {
    info!("=== Render loop task started ====");

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

        // Render if needed - simple version for M5Stack CoreS3
        window.draw_if_needed(|renderer| {
            let mut buffer = [slint::platform::software_renderer::Rgb565Pixel(0); 320];
            for line in 0..240 {
                renderer.render_by_line(DisplayLineBuffer {
                    buffer: &mut buffer,
                    line,
                });
            }
        });

        // Small delay to prevent busy waiting
        embassy_time::Timer::after(embassy_time::Duration::from_millis(16)).await;
        // ~60fps
    }
}

// Simple line buffer for M5Stack CoreS3 display
struct DisplayLineBuffer<'a> {
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
    line: usize,
}

impl<'a> slint::platform::software_renderer::LineBufferProvider for DisplayLineBuffer<'a> {
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [Self::TargetPixel]),
    ) {
        if line == self.line {
            render_fn(&mut self.buffer[range]);
        }
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

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) {
    // Initialize peripherals first
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    // Initialize BOTH heap allocators - WiFi first in internal RAM, then PSRAM for GUI
    // Step 1: Initialize internal RAM heap for WiFi (must be first)
    esp_alloc::heap_allocator!(size: 180 * 1024);

    // Step 2: Initialize PSRAM heap for GUI and other data
    init_heap(&peripherals.PSRAM);

    // Initialize logger
    init_logger_from_env();
    info!("Peripherals initialized");

    info!("Starting Slint ESP32 M5Stack CoreS3 Workshop");

    // === Begin M5Stack CoreS3 Power Management Initialization ===
    // Initialize I2C bus for power management (AXP2101, AW9523)
    info!("Initializing I2C bus for power management...");
    let mut power_i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO12) // AXP2101 SDA
    .with_scl(peripherals.GPIO11); // AXP2101 SCL
    info!("I2C bus initialized for power management");

    // Initialize AXP2101 power management - critical for M5Stack CoreS3 display power
    match init_axp2101_power(&mut power_i2c) {
        Ok(_) => {
            info!("AXP2101 power management initialized successfully");
        }
        Err(_) => {
            error!("Failed to initialize AXP2101 power management - display may not work properly");
            // Continue anyway, but warn user
        }
    };

    // Initialize AW9523 GPIO expander using the same I2C bus - both devices share same I2C lines
    match init_aw9523_gpio_expander(&mut power_i2c) {
        Ok(_) => {
            info!("AW9523 GPIO expander initialized successfully");
        }
        Err(_) => {
            error!("Failed to initialize AW9523 GPIO expander - touch may not work properly");
            // Continue anyway, but warn user
        }
    };

    // Small delay to let power rails stabilize after power management setup
    let mut delay = Delay::new();
    delay.delay_ms(100);
    info!("Power management initialization complete, power rails stabilized");
    // === End M5Stack CoreS3 Power Management Initialization ===

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

    // Store WiFi controller for the wifi scan task
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
    info!("System initialization complete - M5Stack CoreS3 is alive and ready");
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
