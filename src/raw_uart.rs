//! Asynchronous Uno binary transport. Motion writes never wait for a reply.
use crate::{eye::EyeState, motion_limits::MotionLimits, raw_wire::*};
use anyhow::{Context, Result, bail, ensure};
use std::{
    io::{Read, Write},
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};
pub struct RawSerial {
    port: Box<dyn serialport::SerialPort>,
    session: u32,
    seq: u16,
    buffer: Vec<u8>,
    overflow: bool,
    state: Option<EyeState>,
    clock: Option<(u32, Instant)>,
    last_rx: Instant,
    last_stamp: Option<u32>,
    config_wait: Option<(u16, Instant)>,
    probe: Option<(u16, Instant)>,
    last_probe: Instant,
    ack: Option<(u16, u8, u8)>,
    bad: u32,
}
impl RawSerial {
    pub fn new(port: Box<dyn serialport::SerialPort>) -> Result<Self> {
        let now = Instant::now();
        let mut s = Self {
            port,
            session: 0,
            seq: 0,
            buffer: Vec::with_capacity(64),
            overflow: false,
            state: None,
            clock: None,
            last_rx: now,
            last_stamp: None,
            config_wait: None,
            probe: None,
            last_probe: now,
            ack: None,
            bad: 0,
        };
        s.handshake()?;
        Ok(s)
    }
    fn clock_now(&self) -> u32 {
        self.clock
            .map(|(ms, t)| ms.wrapping_add(t.elapsed().as_millis() as u32))
            .unwrap_or(0)
    }
    fn write(&mut self, kind: u8, data: Vec<u8>, motion: bool) -> Result<Option<u16>> {
        if motion && self.port.bytes_to_write()? > 0 {
            return Ok(None);
        }
        self.seq = self.seq.wrapping_add(1);
        let p = Packet {
            kind,
            session: self.session,
            seq: self.seq,
            time: self.clock_now(),
            data,
        };
        if !motion {
            self.port.write_all(&[0])?;
        }
        self.port.write_all(&encode(&p)?)?;
        Ok(Some(self.seq))
    }
    fn handshake(&mut self) -> Result<()> {
        self.session = (SystemTime::now().duration_since(UNIX_EPOCH)?.as_nanos() as u32
            ^ std::process::id())
        .max(1);
        self.seq = 0;
        self.clock = None;
        self.state = None;
        self.last_stamp = None;
        self.buffer.clear();
        self.overflow = false;
        let start = Instant::now();
        let seq = self.write(HELLO, vec![], false)?.unwrap();
        self.config_wait = Some((seq, start));
        while start.elapsed() < Duration::from_millis(250) {
            self.receive()?;
            if self.clock.is_some() && self.state.as_ref().is_some_and(|s| s.actual.is_some()) {
                return Ok(());
            }
            if let Some((q, status, kind)) = self.ack {
                if q == seq && kind == HELLO {
                    bail!("El Uno rechazó HELLO ({status})");
                }
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        bail!("Sin HELLO/CONFIG binario del Uno en 250 ms")
    }
    fn accept(&mut self, p: Packet) -> Result<bool> {
        if p.session != self.session {
            return Ok(false);
        }
        let now = Instant::now();
        if p.kind == ACK && p.data.len() == 2 {
            let age = self.clock_now().wrapping_sub(p.time) as i32;
            if self.clock.is_some() && !(-40..=100).contains(&age) {
                return Ok(false);
            }
            self.ack = Some((p.seq, p.data[0], p.data[1]));
            return Ok(false);
        }
        if p.kind == CONFIG && p.data.len() == 48 {
            let Some((seq, start)) = self.config_wait else {
                return Ok(false);
            };
            if p.seq != seq {
                return Ok(false);
            }
            ensure!(
                start.elapsed()
                    < Duration::from_millis(if self.clock.is_none() { 100 } else { 250 }),
                "CONFIG tardía"
            );
            let b = &p.data;
            ensure!(
                b[0] == 3 && b[2] == 1 && b[3] == 20 && u16at(b, 4) & 15 == 15,
                "Firmware Uno incompatible"
            );
            let limits = MotionLimits {
                up: std::array::from_fn(|i| u16at(b, 20 + i * 2) as i16),
                down: std::array::from_fn(|i| u16at(b, 34 + i * 2) as i16),
            };
            limits.validate()?;
            if self.clock.is_none() {
                self.clock = Some((p.time, start + start.elapsed() / 2));
            }
            self.last_stamp = None;
            let centers = std::array::from_fn(|i| u16at(b, 8 + 2 * i) as i32);
            let signs = std::array::from_fn(|i| (b[14 + i] as i8) as i32);
            ensure!(signs.iter().all(|s| s.abs() == 1), "Sentido inválido");
            self.state = Some(EyeState {
                firmware: format!("uno-avr-{}.{}-raw1", b[0], b[1]),
                udp_port: None,
                gaze_speed_max: Some(u16at(b, 6) as f32 * 0.01),
                udp_session: 0,
                limits: Some(limits),
                activo: false,
                auto: false,
                pca: false,
                outputs_ready: Some(true),
                seguimiento: false,
                parpadeando: false,
                parpado_seguimiento: Some(0.),
                reposo: Some(false),
                parpado_max: 60.,
                giro_max: 55.,
                centro: centers,
                sentido: signs,
                us: centers,
                actual: None,
                diag: Some(serde_json::json!({"uptime_ms":p.time,"uart_binary":true})),
                sampled_at: None,
            });
            self.config_wait = None;
            return Ok(true);
        }
        if p.kind != TELEMETRY || p.data.len() != 24 || self.clock.is_none() || self.state.is_none()
        {
            return Ok(false);
        }
        if self
            .last_stamp
            .is_some_and(|old| (p.time.wrapping_sub(old) as i32) < 0)
        {
            return Ok(false);
        }
        if let Some((seq, sent)) = self.probe {
            if p.seq == seq && sent.elapsed() < Duration::from_millis(80) {
                self.clock = Some((p.time, sent + sent.elapsed() / 2));
                self.probe = None;
            }
        }
        let age = self.clock_now().wrapping_sub(p.time) as i32;
        if !(-40..=100).contains(&age) {
            return Ok(false);
        }
        self.last_rx = now;
        self.last_stamp = Some(p.time);
        let s = self.state.as_mut().unwrap();
        let b = &p.data;
        s.actual = Some(std::array::from_fn(|i| {
            u16at(b, i * 2) as i16 as f32 * 0.01
        }));
        s.us = std::array::from_fn(|i| u16at(b, 6 + i * 2) as i32);
        s.parpado_seguimiento = Some(u16at(b, 12) as f32 * 0.01);
        let f = b[14];
        s.activo = f & 1 != 0;
        s.seguimiento = f & 2 != 0;
        s.parpadeando = f & 4 != 0;
        s.reposo = Some(f & 8 != 0);
        s.outputs_ready = Some(f & 64 != 0);
        s.sampled_at = Some(now - Duration::from_millis(age.max(0) as u64));
        s.diag = Some(
            serde_json::json!({"uptime_ms":p.time,"rx_errors":u16at(b,16),"crc_errors":u16at(b,18),"missed_ticks":u16at(b,20),"target_age_ms":u16at(b,22),"uart_ack":p.seq,"uart_fault":b[15],"uart_host_bad_frames":self.bad,"uart_binary":true}),
        );
        Ok(true)
    }
    fn receive(&mut self) -> Result<bool> {
        let mut changed = false;
        for _ in 0..8 {
            let available = self.port.bytes_to_read()? as usize;
            if available == 0 {
                break;
            }
            let mut bytes = [0u8; 128];
            let count = available.min(bytes.len());
            let n = self.port.read(&mut bytes[..count])?;
            for &b in &bytes[..n] {
                if b != 0 {
                    if self.buffer.len() < 64 && !self.overflow {
                        self.buffer.push(b);
                    } else {
                        self.overflow = true;
                    }
                } else {
                    if !self.buffer.is_empty() || self.overflow {
                        if !self.overflow {
                            match decode(&self.buffer) {
                                Ok(p) => changed |= self.accept(p)?,
                                Err(_) => self.bad += 1,
                            }
                        } else {
                            self.bad += 1;
                        }
                    }
                    self.buffer.clear();
                    self.overflow = false;
                }
            }
        }
        Ok(changed)
    }
    pub fn paused(&self) -> bool {
        self.session == 0
            || self.last_rx.elapsed() > Duration::from_millis(100)
            || self
                .state
                .as_ref()
                .and_then(|s| s.sampled_at)
                .is_none_or(|t| t.elapsed() > Duration::from_millis(100))
    }
    pub fn poll(&mut self) -> Result<Option<EyeState>> {
        if self.session == 0 {
            return Ok(None);
        }
        let changed = self.receive()?;
        if self.last_probe.elapsed() > Duration::from_secs(1) && self.port.bytes_to_write()? == 0 {
            let sent = Instant::now();
            let seq = self.write(STATE, vec![], true)?;
            self.probe = seq.map(|q| (q, sent));
            self.last_probe = sent;
        }
        ensure!(
            self.last_rx.elapsed() < Duration::from_millis(600),
            "Sin telemetría UART binaria en 600 ms"
        );
        Ok(changed.then(|| self.state.clone()).flatten())
    }
    pub fn send(&mut self, a: [f32; 2], lid: f32, rest: bool) -> Result<()> {
        ensure!(
            self.state.as_ref().is_some_and(|s| s.activo),
            "Uno desactivado; requiere encendido explícito"
        );
        ensure!(
            self.last_rx.elapsed() < Duration::from_millis(600),
            "UART sin telemetría"
        );
        ensure!(
            a.iter().chain([lid].iter()).all(|v| v.is_finite())
                && a[0].abs() <= 55.001
                && a[1].abs() <= 35.001
                && (0.0..=if rest { 60. } else { 32. }).contains(&lid),
            "Consigna fuera de límites"
        );
        let mut b = Vec::with_capacity(7);
        for v in [a[0], a[1], lid] {
            b.extend(((v * 100.).round() as i16).to_le_bytes());
        }
        b.push(u8::from(rest));
        self.write(TARGET, b, true)?;
        Ok(())
    }
    pub fn request(&mut self, path: &str) -> Result<EyeState> {
        if path == "/estado" {
            if self.session == 0 || self.last_rx.elapsed() > Duration::from_millis(600) {
                self.handshake()?;
            }
            self.receive()?;
            return self.state.clone().context("Sin estado binario");
        }
        if self.session == 0 && path != "/apagar" {
            self.handshake()?;
        }
        let (kind, data) = match path {
            "/encender" => (ARM, vec![1]),
            "/apagar" => (ARM, vec![0]),
            "/centrar" => (CENTER, vec![]),
            "/parpadear" => (BLINK, vec![]),
            _ if path.starts_with("/limites?") => {
                let mut data = Vec::new();
                for key in ["up", "down"] {
                    let value = path
                        .split_once('?')
                        .unwrap()
                        .1
                        .split('&')
                        .find_map(|part| part.strip_prefix(&format!("{key}=")))
                        .context("Falta tabla de límites")?;
                    let values = value
                        .split(',')
                        .map(str::parse::<u16>)
                        .collect::<std::result::Result<Vec<_>, _>>()?;
                    ensure!(values.len() == 7, "Tabla incompleta");
                    for v in values {
                        data.extend(v.to_le_bytes());
                    }
                }
                (LIMITS, data)
            }
            _ => bail!("Orden no disponible en UART binario: {path}"),
        };
        self.ack = None;
        let start = Instant::now();
        let seq = self.write(kind, data, false)?.unwrap();
        if kind == LIMITS {
            self.config_wait = Some((seq, start));
        }
        while start.elapsed() < Duration::from_millis(250) {
            self.receive()?;
            if kind == LIMITS && self.config_wait.is_none() {
                return self.state.clone().context("Sin CONFIG");
            }
            if let Some((q, status, k)) = self.ack {
                if q == seq && k == kind {
                    ensure!(status == 0, "El Uno rechazó la orden binaria ({status})");
                    let s = self.state.as_mut().context("Sin estado")?;
                    match kind {
                        ARM => {
                            s.activo = path == "/encender";
                            if !s.activo {
                                s.seguimiento = false;
                                s.parpadeando = false;
                                self.session = 0;
                            }
                        }
                        CENTER => s.seguimiento = false,
                        BLINK => s.parpadeando = true,
                        _ => {}
                    }
                    return Ok(s.clone());
                }
            }
            std::thread::sleep(Duration::from_millis(1));
        }
        bail!("Sin confirmación binaria de control en 250 ms")
    }
}
