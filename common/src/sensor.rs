
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct SensorData {
    temperature_celsius: f32,
    humidity_percent: f32,
    when: std::time::Duration,
}
