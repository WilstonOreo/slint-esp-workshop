// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use serde::{Deserialize, Serialize};
use std::sync::{Arc, Mutex};

#[derive(Serialize, Deserialize, Clone, Debug, Default, PartialEq)]
pub struct CityData {
    pub lat: f64,
    pub lon: f64,
    pub city_name: String,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub enum WeatherCondition {
    #[default]
    Unknown,
    Sunny,
    PartiallyCloudy,
    MostlyCloudy,
    Cloudy,
    SunnyRainy,
    Rainy,
    Stormy,
    Snowy,
    Foggy,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct TemperatureData {
    pub min: f64,
    pub max: f64,
    pub morning: f64,
    pub day: f64,
    pub evening: f64,
    pub night: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct PrecipitationData {
    pub probability: f64,
    pub rain_volume: f64,
    pub snow_volume: f64,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct WeatherData {
    pub condition: WeatherCondition,
    pub description: String,

    pub current_temperature: f64,
    pub current_humidity: f64,
    pub detailed_temperature: TemperatureData,

    pub precipitation: PrecipitationData,
    pub uv_index: f64,
}


/* 
#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct ForecastWeatherData {
    pub day_name: String,
    pub weather_data: WeatherData,
}
    */


#[cfg(not(target_arch = "wasm32"))]
pub type WeatherControllerPointer = Box<dyn WeatherController + Send>;
#[cfg(target_arch = "wasm32")]
pub type WeatherControllerPointer = Box<dyn WeatherController + Send + 'static>;

pub type WeatherControllerSharedPointer = Arc<Mutex<WeatherControllerPointer>>;


/// The weather controller trait that provides the weather data.
pub trait WeatherController {
    /// Fetches the city data.
    fn city_data(&self) -> Result<CityData, Box<dyn std::error::Error>>;
    
    /// Fetches the current weather data.
    fn current_data(&self) -> Result<WeatherData, Box<dyn std::error::Error>>;

    // Fetches the forecast weather data.
    //fn forecast_data(&self) -> Result<Vec<ForecastWeatherData>, Box<dyn std::error::Error>>;
}

