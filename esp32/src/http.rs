use embedded_svc::{
    http::{client::Client as HttpClient, Method} };
use esp_idf_svc::http::client::EspHttpConnection;

pub fn new_client() -> anyhow::Result<HttpClient<EspHttpConnection>> {
    let connection = EspHttpConnection::new(&Default::default())?;
    Ok(HttpClient::wrap(connection))
}

/// Send an HTTP GET request.
pub fn get_request(client: &mut HttpClient<EspHttpConnection>) -> anyhow::Result<()> {
    use log::{error, info};
    use embedded_svc::utils::io;
    
    // Prepare headers and URL
    let headers = [("accept", "text/plain")];
    let url = "http://ifconfig.net/";

    // Send request
    //
    // Note: If you don't want to pass in any headers, you can also use `client.get(url, headers)`.
    let request = client.request(Method::Get, url, &headers)?;
    info!("-> GET {}", url);
    let mut response = request.submit()?;

    // Process response
    let status = response.status();
    info!("<- {}", status);
    let mut buf = [0u8; 1024];
    let bytes_read = io::try_read_full(&mut response, &mut buf).map_err(|e| e.0)?;
    info!("Read {} bytes", bytes_read);
    match std::str::from_utf8(&buf[0..bytes_read]) {
        Ok(body_string) => info!(
            "Response body (truncated to {} bytes): {:?}",
            buf.len(),
            body_string
        ),
        Err(e) => error!("Error decoding response body: {}", e),
    };

    Ok(())
}
