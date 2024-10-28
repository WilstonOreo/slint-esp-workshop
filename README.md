# Slint Template for ESP32-S3-BOX-3B and ESP-IDF

This is the Rust Slint Template for ESP32-S3-BOX-3B based on ESP-IDF.

## Environment setup

Follow the prerequisites for
[esp-idf-template](https://github.com/esp-rs/esp-idf-template?tab=readme-ov-file#prerequisites).

For each new terminal session, you need to set up the environment:

```shell
. ${HOME}/export-esp.sh
```

### Windows (WSL2)

Powershell:

```ps1
wsl --install
```

## In WSL:

1. Update dependencies 

```
sudo apt update && sudo apt upgrade -y

2. Install rust:

```sh
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

3. Install packages required for Rust ESP toolchain

```sh
sudo apt install gcc g++ libssl-dev pkg-config libudev-dev make python3.12-venv
sudo apt install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev dfu-util libusb-1.0-0 usbutils
```

4. Install cargo dependencies


```sh
cargo install espup # ESP toolchain setup
cargo install cargo-generate # To check out the template
cargo install ldproxy # To find the right linker on the system
cargo install espflash # Flashing tool
```

5. Install usbipd

* (Download the installer at)[https://github.com/dorssel/usbipd-win/releases/tag/v4.3.0]

* Assuming your ESP32 is connected to your computer, open a powershell as administrator and list the USB devices:

```ps1
usbipd list
```

If everything is correctly connected and installed, a list of the  something similar on the console: 

```console
  2-9    303a:1001  USB Serial Device (COM8), USB JTAG/serial debug unit          Not shared
```

   
   3 usbipd bind --bus-id 2-9
   4 usbipd bind --busid 2-9
   5 usbipd attach --wsl --busid 2-9



# Set up toolchain
espup install

. /home/micha/export-esp.sh



## Build and run

You can build with:

```
cargo build
```
