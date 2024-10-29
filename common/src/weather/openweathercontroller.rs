// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use openweather_sdk::responses::OneCallResponse;
use openweather_sdk::{Language, OpenWeather, Units};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::sync::Mutex;

use crate::weather::weathercontroller::{
    CityData,
    PrecipitationData, TemperatureData, WeatherCondition, WeatherController, WeatherData,
};


const CITIES_STORED_FILE_NAME: &str = "cities_data.json";
const ORGANIZATION_QUALIFIER: &str = "dev"; // have to match android app name in cargo.toml
const ORGANIZATION_NAME: &str = "slint.examples"; // have to match android app name in cargo.toml
const APPLICATION_NAME: &str = "weatherdemo"; // have to match app android name in cargo.toml

#[derive(Serialize, Deserialize, Clone)]
pub struct WeatherClient {
    pub city_data: CityData,
    pub weather_data: Option<OneCallResponse>,
}

pub struct OpenWeatherController {
    weather_api: OpenWeather,
    client: Arc<Mutex<WeatherClient>>,
}


fn weather_condition_from_icon_icon_type(icon_type: &str) -> WeatherCondition {
    match icon_type {
        "01d" | "01n" => WeatherCondition::Sunny,
        "02d" | "02n" => WeatherCondition::PartiallyCloudy,
        "03d" | "03n" => WeatherCondition::MostlyCloudy,
        "04d" | "04n" => WeatherCondition::Cloudy,
        "10d" | "10n" => WeatherCondition::SunnyRainy,
        "09d" | "09n" => WeatherCondition::Rainy,
        "11d" | "11n" => WeatherCondition::Stormy,
        "13d" | "13n" => WeatherCondition::Snowy,
        "50d" | "50n" => WeatherCondition::Foggy,
        _ => WeatherCondition::Unknown,
    }
}


impl From<&OneCallResponse> for WeatherData {
    fn from(response: &OneCallResponse) -> Self {
        if let Some(current) = &response.current {
            let weather_details = &current.weather[0];

            let today_weather_info =
                response.daily.as_ref().and_then(|daily| daily.first());

            let detailed_temp = match today_weather_info {
                Some(info) => {
                    let temp = info.temp;
                    TemperatureData {
                        min: temp.min,
                        max: temp.max,

                        morning: temp.morn,
                        day: temp.day,
                        evening: temp.eve,
                        night: temp.night,
                    }
                }
                
                None => TemperatureData {
                    min: current.temp,
                    max: current.temp,

                    morning: current.temp,
                    day: current.temp,
                    evening: current.temp,
                    night: current.temp,
                },
            };

            return WeatherData {
                description: weather_details.description.clone(),
                condition: weather_condition_from_icon_icon_type(&weather_details.icon),
                current_temperature: current.temp,
                current_humidity: current.humidity as f64,
                detailed_temperature: detailed_temp,
                precipitation: PrecipitationData::default(),
                uv_index: 0.0,
            };
        }

        Self::default()
    }
}

impl OpenWeatherController {
    pub fn new(city_data: CityData, api_key: String) -> Self {
        let mut weather_api = OpenWeather::new(api_key, Units::Metric, Language::English);
        weather_api.one_call.fields.minutely = false;
        weather_api.one_call.fields.hourly = false;
        weather_api.one_call.fields.alerts = false;

        Self {
            weather_api,
            client: Arc::new(Mutex::new(WeatherClient::new(city_data))),
        }
    }



    fn weather_data_from_client(weather_client: &WeatherClient) -> WeatherData {
        weather_client.weather_data.as_ref().unwrap().into()
    }
}

impl WeatherController for OpenWeatherController {
    fn city_data(&self) -> Result<CityData, Box<dyn std::error::Error>> {
        Ok(self.client.lock().unwrap().city_data.clone())
    }

    fn current_data(&self) -> Result<WeatherData, Box<dyn std::error::Error>> {
        let weather_client = self.client.lock().unwrap();
        Ok(Self::weather_data_from_client(&weather_client))
    }
}

impl WeatherClient {
    pub fn new(city_data: CityData) -> Self {
        Self { city_data, weather_data: None }
    }

    pub async fn refresh_weather(
        &mut self,
        weather_api: &OpenWeather,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let res = weather_api.one_call.call(self.city_data.lat, self.city_data.lon).await;
        log::debug!("Weather response: {res:?}");

        match res {
            Ok(response_data) => {
                self.weather_data = Some(response_data);
                log::debug!("Response received at: {:?}", chrono::offset::Local::now().timestamp());

                Ok(())
            }
            Err(e) => Err(e),
        }
    }
}
