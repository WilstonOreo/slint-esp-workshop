# Slint Workshop Demo for Android

Install Android Studio with an SDK and NDK: [https://developer.android.com/studio]

Add Android Rust toolchain:

```sh
rustup target add aarch64-linux-android
```

Install `cargo-apk`:

```sh
cargo install cargo-apk
```

Install adb and java:

```
sudo apt install openjdk-21-jre adb
```