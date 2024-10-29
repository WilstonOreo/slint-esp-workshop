# Slint Template for ESP32-S3-BOX-3B and ESP-IDF

This is the Rust Slint Template for ESP32-S3-BOX-3B based on ESP-IDF.

Clone this template with:

```sh
cargo generate https://github.com/WilstonOreo/slint-esp-workshop
```


If required, please install `cargo-generate` beforehand:

```sh
cargo install cargo-generate
```


## Environment setup

Follow the prerequisites for
[esp-idf-template](https://github.com/esp-rs/esp-idf-template?tab=readme-ov-file#prerequisites).

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
cargo install ldproxy # A tool to find a suitable linker on the system
cargo install espflash # Flashing tool
```

#### 3. Build and run the project

Change the directory to the root of this repository. 
You need to set up the ESP toolchain and initialize the environment:

```sh
espup install # Installs the toolchain. Only has to be done once.
. ${HOME}/export-esp.sh # This step has to be done for each shell session
```

You should now be able to build the project with `cargo build` and run it with `cargo run`.



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
cargo install ldproxy # A tool to find a suitable linker on the system
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

You should now be able to build the project with `cargo build`.

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
Now, bind and attach the bus to WSL:

```ps1
usbipd bind --busid 2-9 # This only has to be done once
usbipd attach --wsl --busid 2-9 # This has to be done for each WSL session
```

In your WSL shell, test with `lusb` if you can see the ESP32 in the list.
The list in the console output must contain:

```console
Bus 001 Device 003: ID 303a:1001 Espressif USB JTAG/serial debug unit
```

Now, your device can be flashed and you can run your application via `cargo run`.

## Wifi Configuration

Set the following environment variables to configure the WiFi:

```sh
export WIFI_SSID=SlintWorkshop
export WIFI_PASS=slintworkshop
```