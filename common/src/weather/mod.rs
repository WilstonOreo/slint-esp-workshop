// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

mod weathercontroller;
mod dummyweathercontroller;

pub use weathercontroller::*;

pub use dummyweathercontroller::DummyWeatherController;

#[cfg(feature = "weather")]
mod openweathercontroller;

#[cfg(feature = "weather")]
pub use openweathercontroller::OpenWeatherController;
