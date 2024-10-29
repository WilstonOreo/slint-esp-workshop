// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

mod weathercontroller;
mod dummyweathercontroller;

pub use weathercontroller::WeatherControllerPointer;
pub use weathercontroller::WeatherControllerSharedPointer;

pub use dummyweathercontroller::DummyWeatherController;

#[cfg(feature = "weather")]
mod openweathercontroller;

#[cfg(feature = "weather")]
pub use openweathercontroller::OpenWeatherController;

pub mod utils;
