
const SSID: &str = env!("WIFI_SSID");
const PASSWORD: &str = env!("WIFI_PASS");

type Wifi = esp_idf_svc::wifi::AsyncWifi<esp_idf_svc::wifi::EspWifi<'static>>;

pub async fn connect(wifi: &mut Wifi) -> anyhow::Result<()> {
    use log::{error, info};
    use embedded_svc::wifi::{AuthMethod, ClientConfiguration, Configuration};
    
    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: SSID.try_into().unwrap(),
        bssid: None,
        auth_method: AuthMethod::WPA2Personal,
        password: PASSWORD.try_into().unwrap(),
        channel: None,
        ..Default::default()
    });

    wifi.set_configuration(&wifi_configuration)?;

    wifi.start().await?;
    info!("Wifi started");

    wifi.connect().await?;
    info!("Wifi connected");

    wifi.wait_netif_up().await?;
    info!("Wifi netif up");

    Ok(())
}