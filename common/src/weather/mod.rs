// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq)]
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
pub struct DayWeatherData {
    pub condition: WeatherCondition,
    pub description: String,

    pub current_temperature: f64,
    pub detailed_temperature: TemperatureData,

    pub precipitation: PrecipitationData,
}

#[derive(Serialize, Deserialize, Clone, Debug, PartialEq, Default)]
pub struct ForecastWeatherData {
    pub day_name: String,
    pub weather_data: DayWeatherData,
}

#[derive(Serialize, Deserialize, Clone)]
pub struct WeatherData {
    pub city_data: CityData,
    pub current_data: DayWeatherData,
    pub forecast_data: Vec<ForecastWeatherData>,
}

/// DataProvider trait to fetch weather data
pub trait WeatherDataProvider {
    fn fetch_weather_data(&self) -> Result<WeatherData, Box<dyn std::error::Error>>;
}

pub type WeatherDataProviderPointer = Box<dyn WeatherDataProvider + Send>;
pub type WeatherDataProviderSharedPointer = std::sync::Arc<std::sync::Mutex<WeatherDataProviderPointer>>;

mod dummydataprovider;

pub use dummydataprovider::DummyDataProvider;