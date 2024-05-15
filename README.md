# Slint Template for ESP32-S3-BOX-3B and ESP-IDF

This is the Rust Slint Template for ESP32-S3-BOX-3B based on ESP-IDF.

## Environment setup

Use `espup` to install Espressif Rust ecosystem: 

```shell
cargo install espup
espup install
```

For each new terminal session, you need to set up the environment:

```shell
. ${HOME}/export-esp.sh
```

## Build and run

You can build with:

```
cargo build
```