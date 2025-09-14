// Display implementation for M5Stack CoreS3
// Based on working examples from esp32-s3-box-3 and Slint mcu-board-support

use alloc::boxed::Box;
use embedded_graphics::pixelcolor::Rgb565;
use embedded_graphics::prelude::*;
use embedded_hal::delay::DelayNs;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    spi::Mode as SpiMode,
    spi::master::{Config as SpiConfig, Spi},
    time::Rate,
};
use log::info;
use mipidsi::options::{ColorInversion, ColorOrder};

// Global storage for display components - using a safer approach
use core::cell::UnsafeCell;

// SAFETY: This is only safe because we're in a single-threaded embedded environment
unsafe impl Sync for DisplayComponentsContainer {}

pub static DISPLAY_COMPONENTS: DisplayComponentsContainer = DisplayComponentsContainer::new();

pub struct DisplayComponentsContainer {
    inner: UnsafeCell<Option<DisplayHardware>>,
}

// Safe wrapper for accessing display components
impl DisplayComponentsContainer {
    pub const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(None),
        }
    }

    pub fn set(&self, hardware: DisplayHardware) {
        unsafe {
            *self.inner.get() = Some(hardware);
        }
    }

    pub fn with_mut<R>(&self, f: impl FnOnce(&mut DisplayHardware) -> R) -> Option<R> {
        unsafe { (*self.inner.get()).as_mut().map(f) }
    }
}

type SpiInterface<'a> = mipidsi::interface::SpiInterface<
    'a,
    ExclusiveDevice<Spi<'a, esp_hal::Blocking>, Output<'a>, Delay>,
    Output<'a>,
>;

pub struct DisplayHardware {
    pub display:
        mipidsi::Display<SpiInterface<'static>, mipidsi::models::ILI9342CRgb565, Output<'static>>,
}

/// Initialize the display hardware for M5Stack CoreS3
pub fn init_display_hardware(
    gpio3: esp_hal::peripherals::GPIO3<'static>,
    gpio34: esp_hal::peripherals::GPIO34<'static>,
    gpio35: esp_hal::peripherals::GPIO35<'static>,
    gpio36: esp_hal::peripherals::GPIO36<'static>,
    gpio37: esp_hal::peripherals::GPIO37<'static>,
    gpio48: esp_hal::peripherals::GPIO48<'static>,
    spi2: esp_hal::peripherals::SPI2<'static>,
) -> Result<(), &'static str> {
    let _delay = Delay::new();

    info!("Initializing M5Stack CoreS3 display hardware...");

    // Enable panel power/backlight (GPIO48 for M5Stack CoreS3)
    let mut backlight = Output::new(gpio48, Level::Low, OutputConfig::default());
    backlight.set_high();
    info!("M5Stack CoreS3 backlight enabled");

    // SPI Display setup for M5Stack CoreS3 (ILI9342C)
    let spi = Spi::<esp_hal::Blocking>::new(
        spi2,
        SpiConfig::default()
            .with_frequency(Rate::from_mhz(40))
            .with_mode(SpiMode::_0),
    )
    .map_err(|_| "Failed to create SPI")?
    .with_sck(gpio36) // M5Stack CoreS3 SPI CLK
    .with_mosi(gpio37); // M5Stack CoreS3 SPI MOSI

    let dc = Output::new(gpio35, Level::Low, OutputConfig::default()); // M5Stack CoreS3 DC
    let cs = Output::new(gpio3, Level::High, OutputConfig::default()); // M5Stack CoreS3 CS  
    let rst = Output::new(gpio34, Level::High, OutputConfig::default()); // M5Stack CoreS3 RST

    let spi_delay = Delay::new();
    let spi_device =
        ExclusiveDevice::new(spi, cs, spi_delay).map_err(|_| "Failed to create SPI device")?;

    // Create static buffer using Box::leak as in examples
    let buffer: &'static mut [u8; 512] = Box::leak(Box::new([0_u8; 512]));
    let di = mipidsi::interface::SpiInterface::new(spi_device, dc, buffer);

    let mut display_delay = Delay::new();
    display_delay.delay_ns(500_000u32);

    // Initialize ILI9342C display (M5Stack CoreS3's actual display controller)
    let mut display = mipidsi::Builder::new(mipidsi::models::ILI9342CRgb565, di)
        .reset_pin(rst)
        .display_size(320, 240)
        .color_order(ColorOrder::Bgr) // BGR color order for M5Stack CoreS3
        .invert_colors(ColorInversion::Inverted) // Inverted colors for M5Stack CoreS3
        .init(&mut display_delay)
        .map_err(|_| "Failed to initialize display")?;

    info!("M5Stack CoreS3 display initialized");

    // Clear the display to show it's working
    display
        .clear(Rgb565::BLUE)
        .map_err(|_| "Failed to clear display")?;
    info!("M5Stack CoreS3 display cleared to blue - hardware test passed");

    // Store display hardware globally for tasks to use
    DISPLAY_COMPONENTS.set(DisplayHardware { display });

    info!("M5Stack CoreS3 display hardware initialization complete");
    Ok(())
}

/// Hardware draw buffer that renders directly to the M5Stack CoreS3 display
pub struct HardwareDrawBuffer<'a, Display> {
    pub display: &'a mut Display,
    pub buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
}

impl<'a, Display> HardwareDrawBuffer<'a, Display> {
    pub fn new(
        display: &'a mut Display,
        buffer: &'a mut [slint::platform::software_renderer::Rgb565Pixel],
    ) -> Self {
        Self { display, buffer }
    }
}

impl<
    DI: mipidsi::interface::Interface<Word = u8>,
    RST: embedded_hal::digital::OutputPin<Error = core::convert::Infallible>,
> slint::platform::software_renderer::LineBufferProvider
    for &mut HardwareDrawBuffer<'_, mipidsi::Display<DI, mipidsi::models::ILI9342CRgb565, RST>>
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

        // Write the rendered line directly to the M5Stack CoreS3 display hardware
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
