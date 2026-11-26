//! UART control, configurable baud, 8N1. Same commands/state as HTTP, correlated replies.
use crate::eye::EyeState;
use anyhow::{Context, Result};
use std::{
    io::{Read, Write},
    time::{Duration, Instant},
};
#[derive(Default)]
pub struct SerialClient {
    port: Option<Box<dyn serialport::SerialPort>>,
    sequence: u32,
    raw: Option<crate::raw_uart::RawSerial>,
}
impl SerialClient {
    /// A failed binary link is dropped so the next request reopens the port
    /// (USB re-enumeration leaves the old descriptor dead forever).
    fn raw_checked<T>(&mut self, r: Result<T>) -> Result<T> {
        if let Err(e) = &r {
            // Only a broken device; a protocol timeout must not reset the board.
            if e.chain().any(|c| c.is::<std::io::Error>() || c.is::<serialport::Error>()) {
                self.raw = None;
            }
        }
        r
    }
    pub fn raw_send(&mut self, a:[f32;2], lid:f32, rest:bool)->Result<()> {let r=self.raw.as_mut().context("UART binario sin negociar")?.send(a,lid,rest);self.raw_checked(r)}
    pub fn raw_poll(&mut self)->Result<Option<EyeState>> {let r=match self.raw.as_mut(){Some(s)=>s.poll(),None=>Ok(None)};self.raw_checked(r)}
    pub fn raw_paused(&self)->bool {self.raw.as_ref().is_none_or(|s|s.paused())}
    pub fn request(&mut self, name: &str, path: &str) -> Result<EyeState> {
        let (device, baud) = serial_endpoint(name)?;
        if baud == 250000 {
            if self.raw.is_none() {
                let mut port=serialport::new(device,baud).timeout(Duration::from_millis(5)).preserve_dtr_on_open().open()?;
                if !device.starts_with("/dev/pts/") {port.write_data_terminal_ready(true)?;port.write_request_to_send(true)?;std::thread::sleep(Duration::from_millis(2300));}
                port.clear(serialport::ClearBuffer::All)?;
                self.raw=Some(crate::raw_uart::RawSerial::new(port)?);
            }
            let r = self.raw.as_mut().unwrap().request(path);
            return self.raw_checked(r);
        }
        anyhow::ensure!(
            path.starts_with('/')
                && path.len() <= 240
                && !path.bytes().any(|b| b <= 32 || b >= 127),
            "Orden serie no válida"
        );
        let result = (|| {
            if self.port.is_none() {
                let (device, baud) = serial_endpoint(name)?;
                let port = serialport::new(device, baud)
                    .timeout(Duration::from_millis(20))
                    .preserve_dtr_on_open()
                    .open()
                    .with_context(|| format!("No se puede abrir UART {name}"))?;
                port.clear(serialport::ClearBuffer::All)?;
                self.port = Some(port);
            }
            self.sequence = self.sequence.wrapping_add(1).max(1);
            let port = self.port.as_mut().unwrap();
            write!(port, "@OJO {} {}\n", self.sequence, path)?;
            let start = Instant::now();
            let mut line = Vec::new();
            let mut overflow = false;
            let mut bytes = [0u8; 128];
            while start.elapsed() < Duration::from_millis(350) {
                let n = match port.read(&mut bytes) {
                    Ok(0) => anyhow::bail!("UART desconectada"),
                    Ok(n) => n,
                    Err(e)
                        if e.kind() == std::io::ErrorKind::TimedOut
                            || e.kind() == std::io::ErrorKind::WouldBlock =>
                    {
                        continue;
                    }
                    Err(e) => return Err(e.into()),
                };
                for &b in &bytes[..n] {
                    if b == b'\r' {
                        continue;
                    }
                    if b != b'\n' {
                        if line.len() < 2048 && !overflow {
                            line.push(b);
                        } else {
                            overflow = true;
                        }
                        continue;
                    }
                    if !overflow {
                        if let Ok(s) = std::str::from_utf8(&line) {
                            let mut parts = s.splitn(4, ' ');
                            if parts.next() == Some("@OJO")
                                && parts.next().and_then(|p| p.parse::<u32>().ok())
                                    == Some(self.sequence)
                            {
                                let status = parts.next().unwrap_or("");
                                let body = parts.next().unwrap_or("");
                                anyhow::ensure!(
                                    status == "200",
                                    "La placa rechazó la orden UART ({status}): {body}"
                                );
                                let mut state: EyeState = serde_json::from_str(body)
                                    .context("Respuesta UART incompleta")?;
                                let elapsed = start.elapsed();
                                if elapsed <= Duration::from_millis(100) {
                                    state.sampled_at = Some(start + elapsed / 2);
                                }
                                return Ok(state);
                            }
                        }
                    }
                    line.clear();
                    overflow = false;
                }
            }
            anyhow::bail!("La placa no respondió por UART (350 ms)")
        })();
        if result.is_err() {
            self.port = None;
        }
        result
    }
}

fn serial_endpoint(name: &str) -> Result<(&str, u32)> {
    let (device, baud) = match name.rsplit_once("?baud=") {
        Some((device, baud)) => (device, baud.parse::<u32>().context("Velocidad UART no válida")?),
        None => (name, 230400),
    };
    anyhow::ensure!(!device.is_empty(), "Elige el puerto del ojo en la pestaña Ojo");
    anyhow::ensure!([115200, 230400, 250000].contains(&baud), "Puerto o velocidad UART no válidos");
    Ok((device, baud))
}

#[cfg(test)]
mod endpoint_tests {
    use super::*;
    #[test]
    fn legacy_esp_and_explicit_uno_endpoints() {
        assert_eq!(serial_endpoint("/dev/ttyUSB0").unwrap(), ("/dev/ttyUSB0", 230400));
        assert_eq!(serial_endpoint("COM3?baud=250000").unwrap(), ("COM3", 250000));
        for value in ["COM3?baud=oops", "COM3?baud=0", "?baud=250000"] {
            assert!(serial_endpoint(value).is_err());
        }
    }
}
