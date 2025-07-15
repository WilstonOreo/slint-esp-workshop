# Slint Multi-Board Workshop with Bare-Metal Implementation

This is a Rust Slint Workshop template supporting multiple ESP32-S3 boards using a no_std bare-metal implementation.

## Supported Boards

- **ESP32-S3-BOX-3** (default)
- **ESoPE-SLD-C-W-S3**
- **ESP32-S3-LCD-EV-BOARD**

## Switching Boards

By default, the workshop is configured for the ESP32-S3-BOX-3. Use different features to build for other boards:

```sh
# Default (esp32-s3-box-3)
cargo run --release

# ESoPE board
cargo run --release --features esope-sld-c-w-s3 --no-default-features

# LCD-EV board
cargo run --release --features esp32-s3-lcd-ev-board --no-default-features
```

## Pre-requisites

- Rust toolchain (minimum version 1.80)
- An IDE, such as VSCode, with the Slint extension

To use this template, make sure Rust and the necessary components are installed for no_std ESP32-S3 development:

```sh
rustup target add xtensa-esp32s3-none-elf
cargo install espflash 
```

### VSCode setup

Install the Slint extension from the extensions marketplace.

## Repository structure

- `winit` - Application code for `winit` based platforms, e.g. desktop environments.
- `esp32` - **no_std** bare-metal application code for ESP32-S3 boards using esp-hal 1.0.0.beta.1.
- `ui` - Shared Slint code for the UI.
- `model` - Crate with shared Rust code for the ESP32 and desktop applications (no_std compatible).

## Features

- **No standard library (no_std)** - Optimized for embedded systems
- **Multi-board support** - Works with multiple ESP32-S3 development boards
- **Slint UI framework** - Rich graphics and touch interface
- **WiFi ready** - Stub implementation ready for WiFi functionality
- **Embassy async runtime** - Modern async/await support for embedded

## Environment Setup for ESP32-S3 (no_std Bare-Metal)

To build the application for ESP32-S3 boards using no_std bare-metal, switch into the `esp32` directory.

Make sure to install the required Rust components for the ESP32-S3 target:

```sh
rustup target add xtensa-esp32s3-none-elf
```

You should also have `espflash` installed for flashing the device:

```sh
cargo install espflash
```

Once set up, build the project:

```sh
cargo build --release
```

Flash the application using:

```sh
cargo espflash --release --chip esp32s3
```

## No_std Implementation Details

This workshop uses a **no_std** bare-metal implementation, which means:

- **No standard library** - Uses `core` and `alloc` crates only
- **No heap allocation by default** - Uses PSRAM for dynamic allocation
- **Direct hardware access** - Through esp-hal crate
- **No ldproxy required** - Direct compilation to embedded target
- **Optimized for embedded** - Smaller binary size and better performance

### Key Dependencies

- **esp-hal 1.0.0-beta.1** - Hardware abstraction layer for ESP32-S3
- **embassy** - Async runtime for embedded systems
- **slint** - UI framework with no_std support
- **esp-alloc** - Memory allocator for ESP32 with PSRAM support

### Build Configuration

The project uses:
- **Target**: `xtensa-esp32s3-none-elf` (bare-metal, no std)
- **Build std**: `["alloc", "core"]` for no_std with allocation
- **Custom linker scripts** - For proper memory layout
- **DMA and PSRAM** - For framebuffer and graphics performance

### Ubuntu/Debian

#### 1. Install dependencies

Assuming you have installed Rust (minimum version 1.80), please install the following packages via `apt`:

```sh
sudo apt install make gcc g++ libssl-dev pkg-config libudev-dev python3 python3-pip python3-venv
sudo apt install wget flex bison gperf cmake ninja-build ccache libffi-dev dfu-util libusb-1.0-0 usbutils
```

#### 2. Install ESP toolchain for Rust

```sh
cargo install espup # ESP toolchain setup
cargo install espflash # Flashing tool
```

#### 3. Set up ESP toolchain

```sh
espup install # Installs the toolchain. Only has to be done once.
. ${HOME}/export-esp.sh # This step has to be done for each shell session
```

#### 4. Build and run the project

Change directory to the `esp32` folder and build the project:

```sh
cd esp32
cargo build --release
```

Flash the application to the device:

```sh
cargo espflash --release
```

### Windows (WSL2)

If not done already, open a PowerShell as administrator and install WSL:

```ps1
wsl --install
```

Open a new WSL shell and do the following steps:

#### 1. Update dependencies 

```sh
sudo apt update && sudo apt upgrade -y
```

#### 2. Install rust:

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

#### 3. Install packages required for Rust ESP toolchain

```sh
sudo apt install gcc g++ libssl-dev pkg-config libudev-dev make flex bison gperf cmake ccache ninja-build 
sudo apt install git wget libxkbcommon-x11-dev python3 python3-pip python3-venv python3.12-venv libffi-dev dfu-util libusb-1.0-0 usbutils
```

#### 4. Install cargo dependencies

Install these dependencies required to set up the ESP toolchain: 

```sh
cargo install espup # ESP toolchain setup
cargo install espflash # Flashing tool
cargo install cargo-generate # Required to actually check out the template
```

####  5. Set up ESP toolchain

Change the directory to the root of this repository. 
You need to set up the ESP toolchain and initialize the environment:

```sh
espup install # Installs the toolchain. Only has to be done once.
. ${HOME}/export-esp.sh # This step has to be done for each shell session
```

You should now be able to build the project with `cargo build --release`.

#### 6. Install usbipd for flashing the device

* Download the installer at: https://github.com/dorssel/usbipd-win/releases/
* Assuming your ESP32 is connected to your computer, open a PowerShell as administrator and list the USB devices:

```ps1
usbipd list
```

If everything is correctly connected and installed, a list of the  something similar on the console: 

```console
  2-9    303a:1001  USB Serial Device (COM8), USB JTAG/serial debug unit          Not shared
```

The id `2-9` on the left is the bus number. It might be different on your machine.
First, bind the bus to the usbipd service:

```ps1
usbipd bind --busid 2-9 # This only has to be done once
```

After that, attach the bus to WSL:

```ps1
usbipd attach --wsl --busid 2-9 # This has to be done for each WSL session and each time the device is disconnected
```

In your WSL shell, test with `lusb` if you can see the ESP32 in the list.
The list in the console output must contain:

```console
Bus 001 Device 003: ID 303a:1001 Espressif USB JTAG/serial debug unit
```

Now, your device can be flashed and you can run your application via `cargo espflash --release`.

## Current UI Features

The workshop application includes:

- **Tabbed Interface** - WiFi and About tabs
- **WiFi Tab** - Shows network list with refresh functionality (stub implementation)
- **About Tab** - Displays Slint framework information
- **Touch Support** - Works with capacitive touch displays
- **Responsive Design** - Adapts to different screen sizes

## Troubleshooting

### I2C EEPROM Errors

If you encounter `I2C(AcknowledgeCheckFailed(Address))` errors with the ESoPE board:

1. **Switch to ESP32-S3-BOX-3** (default) - This board doesn't require EEPROM
2. **Check hardware connections** - Ensure I2C pins are properly connected
3. **Verify EEPROM address** - The board might use a different I2C address

### Build Errors

- **Missing target**: Run `rustup target add xtensa-esp32s3-none-elf`
- **Linker errors**: Make sure ESP toolchain is properly installed with `espup install`
- **Version conflicts**: Clean build with `cargo clean` and rebuild

### Board Not Detected

- **Check USB connection** - Ensure the board is connected and in download mode
- **Driver issues** - Install proper USB drivers for your board
- **Permission issues** - On Linux, you might need to add your user to the `dialout` group

### Next Steps

This workshop is ready for extending with:

- **Real WiFi scanning** - Replace stub implementation with esp-wifi
- **Network connectivity** - Add TCP/UDP networking
- **IoT features** - MQTT, HTTP clients, etc.
- **Advanced UI** - More complex Slint components
- **Sensors** - Add sensor data display
