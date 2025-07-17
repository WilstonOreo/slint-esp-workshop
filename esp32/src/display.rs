// Local display implementation for ESP32-S3-BOX-3
// Based on working examples and mcu-board-support but without WiFi
// Uses the approach from esp32-conways-game-of-life-rs and esp32-spooky-maze-game

use alloc::boxed::Box;
use alloc::rc::Rc;
use core::cell::RefCell;
use embedded_graphics::draw_target::DrawTarget;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::gpio::DriveMode;
use esp_hal::time::Instant;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    i2c::master::I2c,
    spi::master::{Config as SpiConfig, Spi},
    spi::Mode as SpiMode,
    time::Rate,
    Blocking,
};
use gt911::Gt911Blocking;
use log::{error, info, warn};
use mipidsi::options::ColorOrder;
use slint::platform::PointerEventButton;
use slint::platform::WindowEvent;

// Global storage for display components
static mut DISPLAY_COMPONENTS: Option<DisplayHardware> = None;

struct DisplayHardware {
    display: mipidsi::Display<
        mipidsi::interface::SpiInterface<
            'static,
            ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>,
            Output<'static>,
        >,
        mipidsi::models::ILI9486Rgb565,
        Output<'static>,
    >,
    touch: Gt911Blocking<I2c<'static, esp_hal::Blocking>>,
    i2c: I2c<'static, esp_hal::Blocking>,
}

struct EspDisplay {
    window: RefCell<Option<Rc<slint::platform::software_renderer::MinimalSoftwareWindow>>>,
}

impl slint::platform::Platform for EspDisplay {
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
        self.run_event_loop_impl()
    }
}

impl EspDisplay {
    fn run_event_loop_impl(&self) -> Result<(), slint::PlatformError> {
        // Use the actual hardware for rendering and touch input
        let mut last_touch = None;

        // Initialize touch with fallback
        let hardware = unsafe { DISPLAY_COMPONENTS.as_mut().unwrap() };

        const ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP: u8 = 0x5D;
        match hardware.touch.init(&mut hardware.i2c) {
            Ok(_) => info!("Touch initialized"),
            Err(e) => {
                warn!("Touch initialization failed: {:?}", e);
                let touch_fallback = Gt911Blocking::new(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
                match touch_fallback.init(&mut hardware.i2c) {
                    Ok(_) => {
                        info!("Touch initialized with backup address");
                        hardware.touch = touch_fallback;
                    }
                    Err(e) => error!("Touch initialization failed with backup address: {:?}", e),
                }
            }
        }

        // Set window size from display
        if let Some(window) = self.window.borrow().clone() {
            let size = hardware.display.size();
            let size = slint::PhysicalSize::new(size.width, size.height);
            window.set_size(slint::WindowSize::Physical(size));
        }

        loop {
            slint::platform::update_timers_and_animations();

            if let Some(window) = self.window.borrow().clone() {
                // Handle touch input
                match hardware.touch.get_touch(&mut hardware.i2c) {
                    Ok(Some(point)) => {
                        let pos = slint::PhysicalPosition::new(point.x as i32, point.y as i32)
                            .to_logical(window.scale_factor());

                        let event = if let Some(previous_pos) = last_touch.replace(pos) {
                            if previous_pos != pos {
                                WindowEvent::PointerMoved { position: pos }
                            } else {
                                continue;
                            }
                        } else {
                            WindowEvent::PointerPressed {
                                position: pos,
                                button: PointerEventButton::Left,
                            }
                        };

                        let _ = window.dispatch_event(event);
                    }
                    Ok(None) => {
                        if let Some(pos) = last_touch.take() {
                            let _ = window.dispatch_event(WindowEvent::PointerReleased {
                                position: pos,
                                button: PointerEventButton::Left,
                            });
                            let _ = window.dispatch_event(WindowEvent::PointerExited);
                        }
                    }
                    Err(_) => {
                        // Ignore touch errors
                    }
                }

                // Render with actual display hardware
                window.draw_if_needed(|renderer| {
                    let mut buffer_provider = HardwareDrawBuffer {
                        display: &mut hardware.display,
                        buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
                    };
                    renderer.render_by_line(&mut buffer_provider);
                });

                if window.has_active_animations() {
                    continue;
                }
            }

            // Small delay to prevent busy waiting
            let mut delay = Delay::new();
            delay.delay_ms(10);
        }
    }
}

/// Initialize the display platform using the approach from working examples
pub fn init() -> Result<(), &'static str> {
    // Set up Slint platform
    slint::platform::set_platform(Box::new(EspDisplay {
        window: RefCell::new(None),
    }))
    .map_err(|_| "Failed to set Slint platform")?;

    info!("Display platform initialized");
    Ok(())
}

/// Initialize the display hardware directly in main
/// This follows the approach from esp32-conways-game-of-life-rs  
pub fn init_display_hardware(
    peripherals: esp_hal::peripherals::Peripherals,
) -> Result<(), &'static str> {
    let mut delay = Delay::new();

    // Touch initialization sequence for GT911
    let _reset_level = Level::Low;
    let mut int_pin = Output::new(peripherals.GPIO3, Level::High, OutputConfig::default());
    int_pin.set_low();
    delay.delay_ms(10);
    int_pin.set_low();
    delay.delay_ms(1);

    // Reset pin: OpenDrain required for ESP32-S3-BOX-3
    let rst = Output::new(
        peripherals.GPIO48,
        Level::High,
        OutputConfig::default().with_drive_mode(DriveMode::OpenDrain),
    );

    int_pin.set_low();
    delay.delay_ms(10);

    let gpio_level = Level::Low; // For address 0x14
    int_pin.set_level(gpio_level);
    delay.delay_ms(1);

    delay.delay_ms(10);
    delay.delay_ms(50);

    // SPI and Display initialization
    let spi = Spi::<Blocking>::new(
        peripherals.SPI2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(SpiMode::_0),
    )
    .map_err(|_| "Failed to create SPI")?
    .with_sck(peripherals.GPIO7)
    .with_mosi(peripherals.GPIO6);

    let dc = Output::new(peripherals.GPIO4, Level::Low, OutputConfig::default());
    let cs = Output::new(peripherals.GPIO5, Level::High, OutputConfig::default());

    let spi_delay = Delay::new();
    let spi_device =
        ExclusiveDevice::new(spi, cs, spi_delay).map_err(|_| "Failed to create SPI device")?;

    // Create static buffer using Box::leak as in examples
    let buffer: &'static mut [u8; 512] = Box::leak(Box::new([0_u8; 512]));

    let di = mipidsi::interface::SpiInterface::new(spi_device, dc, buffer);

    let mut display_delay = Delay::new();
    display_delay.delay_ns(500_000u32);

    let mut display = mipidsi::Builder::new(mipidsi::models::ILI9486Rgb565, di)
        .reset_pin(rst)
        .display_size(320, 240)
        .color_order(ColorOrder::Bgr)
        .init(&mut display_delay)
        .map_err(|_| "Failed to initialize display")?;

    // Set up the backlight
    let mut backlight = Output::new(peripherals.GPIO47, Level::Low, OutputConfig::default());
    backlight.set_high();

    // Clear the display to show it's working
    display
        .clear(Rgb565::BLUE)
        .map_err(|_| "Failed to clear display")?;

    // I2C initialization for touch
    let i2c = I2c::new(
        peripherals.I2C0,
        esp_hal::i2c::master::Config::default().with_frequency(Rate::from_khz(400)),
    )
    .map_err(|_| "Failed to create I2C")?
    .with_sda(peripherals.GPIO8)
    .with_scl(peripherals.GPIO18);

    let touch = Gt911Blocking::new(0x14);

    // Store components globally for the platform to use
    unsafe {
        DISPLAY_COMPONENTS = Some(DisplayHardware {
            display,
            touch,
            i2c,
        });
    }

    info!("Display hardware initialization complete");
    Ok(())
}

/// Run the complete event loop with hardware display and touch
/// This integrates the display with Slint's rendering system
pub fn run_with_hardware(
    mut display: mipidsi::Display<
        mipidsi::interface::SpiInterface<
            'static,
            ExclusiveDevice<Spi<'static, esp_hal::Blocking>, Output<'static>, Delay>,
            Output<'static>,
        >,
        mipidsi::models::ILI9486Rgb565,
        Output<'static>,
    >,
    mut touch: Gt911Blocking<I2c<'static, esp_hal::Blocking>>,
    mut i2c: I2c<'static, esp_hal::Blocking>,
    window: Rc<slint::platform::software_renderer::MinimalSoftwareWindow>,
) -> Result<(), slint::PlatformError> {
    // Initialize touch with fallback
    const ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS: u8 = 0x14;
    const ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP: u8 = 0x5D;

    match touch.init(&mut i2c) {
        Ok(_) => info!("Touch initialized"),
        Err(e) => {
            warn!("Touch initialization failed: {:?}", e);
            let touch_fallback = Gt911Blocking::new(ESP_LCD_TOUCH_IO_I2C_GT911_ADDRESS_BACKUP);
            match touch_fallback.init(&mut i2c) {
                Ok(_) => {
                    info!("Touch initialized with backup address");
                    touch = touch_fallback;
                }
                Err(e) => error!("Touch initialization failed with backup address: {:?}", e),
            }
        }
    }

    // Update the Slint window size from the display
    let size = display.size();
    let size = slint::PhysicalSize::new(size.width, size.height);
    window.set_size(slint::WindowSize::Physical(size));

    // Prepare a draw buffer for the Slint software renderer
    let mut buffer_provider = DrawBuffer {
        display: &mut display,
        buffer: &mut [slint::platform::software_renderer::Rgb565Pixel(0); 320],
    };

    // Variable to track the last touch position
    let mut last_touch = None;

    // Main event loop
    loop {
        slint::platform::update_timers_and_animations();

        // Poll the GT911 for touch data
        match touch.get_touch(&mut i2c) {
            // Active touch detected: Some(point) means a press or move
            Ok(Some(point)) => {
                // Convert GT911 raw coordinates into a PhysicalPosition
                let pos = slint::PhysicalPosition::new(point.x as i32, point.y as i32)
                    .to_logical(window.scale_factor());

                let event = if let Some(previous_pos) = last_touch.replace(pos) {
                    // If the position changed, send a PointerMoved event
                    if previous_pos != pos {
                        WindowEvent::PointerMoved { position: pos }
                    } else {
                        // If the position is unchanged, skip event generation
                        continue;
                    }
                } else {
                    // No previous touch recorded, generate a PointerPressed event
                    WindowEvent::PointerPressed {
                        position: pos,
                        button: PointerEventButton::Left,
                    }
                };

                // Dispatch the event to Slint
                let _ = window.dispatch_event(event);
            }
            // No active touch: if a previous touch existed, dispatch pointer release
            Ok(None) => {
                if let Some(pos) = last_touch.take() {
                    let _ = window.dispatch_event(WindowEvent::PointerReleased {
                        position: pos,
                        button: PointerEventButton::Left,
                    });
                    let _ = window.dispatch_event(WindowEvent::PointerExited);
                }
            }
            // On errors, ignore them
            Err(_) => {
                // Optionally log errors
            }
        }

        // Render the window if needed
        window.draw_if_needed(|renderer| {
            renderer.render_by_line(&mut buffer_provider);
        });

        if window.has_active_animations() {
            continue;
        }
    }
}

/// Provides a draw buffer for the MinimalSoftwareWindow renderer
struct DrawBuffer<'a, Display> {
    display: &'a mut Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<
        DI: mipidsi::interface::Interface<Word = u8>,
        RST: embedded_hal::digital::OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut DrawBuffer<'_, mipidsi::Display<DI, mipidsi::models::ILI9486Rgb565, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];
        render_fn(buffer);

        // Update the display with the rendered line
        self.display
            .set_pixels(
                range.start as u16,
                line as u16,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}

/// Hardware draw buffer that renders directly to the display
struct HardwareDrawBuffer<'a, Display> {
    display: &'a mut Display,
    buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<
        DI: mipidsi::interface::Interface<Word = u8>,
        RST: embedded_hal::digital::OutputPin<Error = core::convert::Infallible>,
    > slint::platform::software_renderer::LineBufferProvider
    for &mut HardwareDrawBuffer<'_, mipidsi::Display<DI, mipidsi::models::ILI9486Rgb565, RST>>
{
    type TargetPixel = slint::platform::software_renderer::Rgb565Pixel;

    fn process_line(
        &mut self,
        line: usize,
        range: core::ops::Range<usize>,
        render_fn: impl FnOnce(&mut [slint::platform::software_renderer::Rgb565Pixel]),
    ) {
        let buffer = &mut self.buffer[range.clone()];
        render_fn(buffer);

        // Write the rendered line directly to the display hardware
        self.display
            .set_pixels(
                range.start as u16,
                line as u16,
                range.end as u16,
                line as u16,
                buffer
                    .iter()
                    .map(|x| embedded_graphics_core::pixelcolor::raw::RawU16::new(x.0).into()),
            )
            .unwrap();
    }
}
