// Copyright © SixtyFPS GmbH <info@slint.dev>
// SPDX-License-Identifier: MIT

use crate::weather::*;
pub struct DummyDataProvider;

impl WeatherDataProvider for DummyDataProvider {
    fn fetch_weather_data(&self) -> Result<WeatherData, Box<dyn std::error::Error>> {
        Ok(WeatherData {
            city_data: CityData {
                lat: 52.52,
                lon: 13.405,
                city_name: "Berlin".to_string(),
            },
            current_data: DayWeatherData {
                condition: WeatherCondition::Sunny,
                description: "Sunny".to_string(),
                current_temperature: 25.0,
                detailed_temperature: TemperatureData {
                    min: 20.0,
                    max: 30.0,
                    morning: 22.0,
                    day: 25.0,
                    evening: 23.0,
                    night: 21.0,
                },
                precipitation: PrecipitationData {
                    probability: 0.0,
                    rain_volume: 0.0,
                    snow_volume: 0.0,
                },
            },
            forecast_data: vec![
                ForecastWeatherData {
                    day_name: "Monday".to_string(),
                    weather_data: DayWeatherData {
                        condition: WeatherCondition::PartiallyCloudy,
                        description: "Partially Cloudy".to_string(),
                        current_temperature: 24.0,
                        detailed_temperature: TemperatureData {
                            min: 19.0,
                            max: 29.0,
                            morning: 21.0,
                            day: 24.0,
                            evening: 22.0,
                            night: 20.0,
                        },
                        precipitation: PrecipitationData {
                            probability: 0.0,
                            rain_volume: 0.0,
                            snow_volume: 0.0,
                        },
                    },
                },
                ForecastWeatherData {
                    day_name: "Tuesday".to_string(),
                    weather_data: DayWeatherData {
                        condition: WeatherCondition::MostlyCloudy,
                        description: "Mostly Cloudy".to_string(),
                        current_temperature: 23.0,
                        detailed_temperature: TemperatureData {
                            min: 18.0,
                            max: 28.0,
                            morning: 20.0,
                            day: 23.0,
                            evening: 21.0,
                            night: 19.0,
                        },
                        precipitation: PrecipitationData {
                            probability: 0.0,
                            rain_volume: 0.0,
                            snow_volume: 0.0,
                        },
                    },
                },
                ForecastWeatherData {
                    day_name: "Wednesday".to_string(),
                    weather_data: DayWeatherData {
                        condition: WeatherCondition::Cloudy,
                        description: "Cloudy".to_string(),
                        current_temperature: 22.0,
                        detailed_temperature: TemperatureData {
                            min: 17.0,
                            max: 27.0,
                            morning: 19.0,
                            day: 22.0,
                            evening: 20.0,
                            night: 18.0,
                        },
                        precipitation: PrecipitationData {
                            probability: 0.0,
                            rain_volume: 0.0,
                            snow_volume: 0.0,
                        },
                    },
                },
                ForecastWeatherData {
                    day_name: "Thursday".to_string(),
                    weather_data: DayWeatherData {
                        condition: WeatherCondition::SunnyRainy,
                        description: "Sunny with Rain".to_string(),
                        current_temperature: 21.0,
                        detailed_temperature: TemperatureData {
                            min: 16.0,
                            max: 26.0,
                            morning: 18.0,
                            day: 21.0,
                            evening: 19.0,
                            night: 17.0,
                        },
                        precipitation: PrecipitationData {
                            probability: 0.5,
                            rain_volume: 0.5,
                            snow_volume: 0.0,
                        },
                    },
                },
            ]
        })
    }
}
