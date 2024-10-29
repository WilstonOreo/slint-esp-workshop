// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use serde::Deserialize;

use crate::weather::weathercontroller::{
    CityData, WeatherData, WeatherController,
};

#[derive(Deserialize, Default)]
pub struct DummyWeatherController {
    city_data: CityData,
    weather_data: WeatherData,
}

impl DummyWeatherController {
    pub fn new() -> Result<Self, serde_json::Error> {
        let json_data = std::include_str!("./dummyweather.json");
        
        serde_json::from_str::<Self>(json_data)
    }
}

impl WeatherController for DummyWeatherController {
    fn current_data(&self) -> Result<WeatherData, Box<dyn std::error::Error>> {
        Ok(self.weather_data.clone())
    }

    fn city_data(&self) -> Result<CityData, Box<dyn std::error::Error>> {
        Ok(self.city_data.clone())
    }
}

#[test]
fn test_dummy_weather_controller() {
    let controller = DummyWeatherController::new().unwrap();
    let city_data = controller.city_data().unwrap();
    let weather_data = controller.current_data().unwrap();

    assert_eq!(city_data.city_name, "Berlin");
    assert_eq!(weather_data.current_temperature, 15.24);
}

