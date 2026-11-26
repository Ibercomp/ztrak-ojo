//! HTTP over TCP, matching the native Espressif firmware. No Arduino dependency.
use anyhow::{Context, Result};
use serde::Deserialize;
use std::{
    io::{Read, Write},
    net::{SocketAddr, TcpStream},
    time::{Duration, Instant},
};

#[derive(Clone, Debug, Deserialize)]
pub struct EyeState {
    pub firmware: String,
    #[serde(default)]
    pub udp_port: Option<u16>,
    #[serde(default)]
    pub gaze_speed_max: Option<f32>,
    #[serde(default)]
    pub udp_session: u32,
    #[serde(default)]
    pub limits: Option<crate::motion_limits::MotionLimits>,
    pub activo: bool,
    pub auto: bool,
    pub pca: bool,
    #[serde(default)]
    pub outputs_ready: Option<bool>,
    #[serde(default)]
    pub seguimiento: bool,
    #[serde(default)]
    pub parpadeando: bool,
    #[serde(default)]
    pub parpado_seguimiento: Option<f32>,
    #[serde(default)]
    pub reposo: Option<bool>,
    #[serde(default = "default_lid")]
    pub parpado_max: f32,
    #[serde(default = "default_yaw")]
    pub giro_max: f32,
    pub centro: [i32; 3],
    pub sentido: [i32; 3],
    pub us: [i32; 3],
    pub actual: Option<[f32; 3]>,
    /// Device diagnostics, absent on older firmware.
    #[serde(default)]
    pub diag: Option<serde_json::Value>,
    /// Approximate device sample time; no physical encoder feedback.
    #[serde(skip)]
    pub sampled_at: Option<Instant>,
}
fn default_yaw() -> f32 {
    35.
}
fn default_lid() -> f32 {
    60.
}
impl EyeState {
    pub fn outputs_ready(&self) -> bool {
        self.outputs_ready.unwrap_or(self.pca)
    }
    pub fn position(&self) -> [f32; 2] {
        if let Some(a) = self.actual {
            [a[0], a[1]]
        } else {
            [
                (self.us[0] - self.centro[0]) as f32 / self.sentido[0] as f32 / 10.,
                (self.us[1] - self.centro[1]) as f32 / self.sentido[1] as f32 / 10.,
            ]
        }
    }
    pub fn supports_tracking(&self) -> bool {
        self.actual.is_some()
    }
}
pub fn parse_address(s: &str) -> Result<SocketAddr> {
    let s = s.trim().trim_start_matches("http://").trim_end_matches('/');
    s.parse()
        .or_else(|_| format!("{s}:80").parse())
        .context("Usa la IP de la placa, por ejemplo 192.168.1.121:80")
}
pub fn request(address: &str, path: &str) -> Result<EyeState> {
    Client::default().request(address, path)
}

/// One sequential persistent connection: no pending command queue and no TCP
/// handshake for every frame. A broken stream is discarded before retrying.
#[derive(Default)]
pub struct Client {
    address: Option<SocketAddr>,
    stream: Option<TcpStream>,
    serial: Option<crate::serial_link::SerialClient>,
    serial_name: Option<String>,
}
impl Client {
    pub fn binary_uart(address:&str)->bool {address.starts_with("uart:")&&address.ends_with("?baud=250000")}
    pub fn send_uart_motion(&mut self,a:[f32;2],lid:f32,rest:bool)->Result<()> {self.serial.as_mut().context("Sin enlace UART")?.raw_send(a,lid,rest)}
    pub fn poll_uart_motion(&mut self)->Result<Option<EyeState>> {match self.serial.as_mut(){Some(s)=>s.raw_poll(),None=>Ok(None)}}
    pub fn uart_paused(&self)->bool {self.serial.as_ref().is_none_or(|s|s.raw_paused())}
    pub fn request(&mut self, address: &str, path: &str) -> Result<EyeState> {
        if let Some(port) = address.strip_prefix("uart:") {
            self.stream = None;
            self.address = None;
            if self.serial_name.as_deref() != Some(port) {
                self.serial = Some(crate::serial_link::SerialClient::default());
                self.serial_name = Some(port.into());
            }
            return self.serial.as_mut().unwrap().request(port, path);
        }
        self.serial = None;
        self.serial_name = None;
        let addr = parse_address(address)?;
        if self.address != Some(addr) {
            self.stream = None;
            self.address = Some(addr);
        }
        let started = Instant::now();
        let result = (|| {
            if self.stream.is_none() {
                self.stream = Some(connect(addr)?);
            }
            exchange(self.stream.as_mut().unwrap(), addr, path)
        })();
        match result {
            Ok((mut state, closed)) => {
                let elapsed = started.elapsed();
                if elapsed <= Duration::from_millis(100) {
                    state.sampled_at = Some(started + elapsed / 2);
                }
                if closed {
                    self.stream = None;
                }
                Ok(state)
            }
            Err(e) => {
                self.stream = None;
                Err(e)
            }
        }
    }
}
fn connect(addr: SocketAddr) -> Result<TcpStream> {
    let stream = TcpStream::connect_timeout(&addr, Duration::from_millis(600))
        .context("No se puede conectar con el ojo")?;
    stream.set_read_timeout(Some(Duration::from_millis(250)))?;
    stream.set_write_timeout(Some(Duration::from_millis(300)))?;
    stream.set_nodelay(true)?;
    Ok(stream)
}
fn write_request(stream: &mut impl Write, addr: SocketAddr, path: &str) -> std::io::Result<()> {
    // Format in memory first. write! on an unbuffered TcpStream emits many
    // tiny writes; TCP_NODELAY turns these into extra traffic/parser wakeups.
    let request = format!("GET {path} HTTP/1.1\r\nHost: {addr}\r\nConnection: keep-alive\r\n\r\n");
    stream.write_all(request.as_bytes())
}
fn exchange(stream: &mut TcpStream, addr: SocketAddr, path: &str) -> Result<(EyeState, bool)> {
    write_request(stream, addr, path)?;
    let mut response = Vec::new();
    let mut part = [0u8; 2048];
    let mut total = None;
    loop {
        let n = stream.read(&mut part)?;
        if n == 0 {
            break;
        }
        response.extend_from_slice(&part[..n]);
        anyhow::ensure!(response.len() < 16384, "Respuesta demasiado larga");
        if let Some(end) = response.windows(4).position(|s| s == b"\r\n\r\n") {
            let header = std::str::from_utf8(&response[..end])?;
            if let Some(len) = header.lines().find_map(|s| {
                s.split_once(':')
                    .filter(|(k, _)| k.eq_ignore_ascii_case("content-length"))
                    .and_then(|(_, v)| v.trim().parse::<usize>().ok())
            }) {
                total = Some(end + 4 + len);
            }
        }
        if total.is_some_and(|len| response.len() >= len) {
            break;
        }
    }
    let end = response
        .windows(4)
        .position(|s| s == b"\r\n\r\n")
        .context("Respuesta HTTP incompleta")?;
    let header = std::str::from_utf8(&response[..end])?;
    anyhow::ensure!(
        header
            .lines()
            .next()
            .is_some_and(|l| l.split_whitespace().nth(1) == Some("200")),
        "La placa rechazó la orden: {}",
        header.lines().next().unwrap_or_default()
    );
    let closed = header.lines().any(|line| {
        line.split_once(':').is_some_and(|(k, v)| {
            k.eq_ignore_ascii_case("connection") && v.trim().eq_ignore_ascii_case("close")
        })
    });
    let length = total.context("Falta Content-Length en la respuesta de la placa")?;
    anyhow::ensure!(response.len() >= length, "Respuesta de la placa truncada");
    Ok((serde_json::from_slice(&response[end + 4..length])?, closed))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn request_is_assembled_before_writing_to_an_unbuffered_socket() {
        #[derive(Default)]
        struct Writes(Vec<Vec<u8>>);
        impl Write for Writes {
            fn write(&mut self, bytes: &[u8]) -> std::io::Result<usize> {
                self.0.push(bytes.to_vec());
                Ok(bytes.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                Ok(())
            }
        }
        let mut w = Writes::default();
        write_request(
            &mut w,
            "192.168.1.121:80".parse().unwrap(),
            "/seguir?giro=12&cabeceo=0",
        )
        .unwrap();
        assert_eq!(
            w.0.len(),
            1,
            "TCP_NODELAY must not send formatted fragments separately"
        );
        let request = std::str::from_utf8(&w.0[0]).unwrap();
        assert!(
            request.starts_with(
                "GET /seguir?giro=12&cabeceo=0 HTTP/1.1\r\nHost: 192.168.1.121:80\r\n"
            )
        );
        assert!(request.ends_with("\r\n\r\n"));
    }
    #[test]
    fn addresses() {
        assert_eq!(parse_address("http://192.168.1.121/").unwrap().port(), 80);
        assert!(parse_address("localhost/ota").is_err());
    }
    #[test]
    fn physical_calibration_is_not_assumed() {
        let s:EyeState=serde_json::from_str(r#"{"firmware":"v8","activo":true,"auto":false,"pca":true,"centro":[1560,1470,1810],"sentido":[1,-1,-1],"us":[1660,1520,1810]}"#).unwrap();
        assert_eq!(s.position(), [10., -5.]);
    }
}
