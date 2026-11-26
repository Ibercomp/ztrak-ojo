//! Latest-value, nonblocking motion channel. HTTP remains for setup/web/OTA.
use crate::eye::{EyeState, parse_address};
use anyhow::{Context, Result, ensure};
use std::{
    collections::VecDeque,
    net::UdpSocket,
    time::{Duration, Instant},
};

pub struct UdpLink {
    socket: UdpSocket,
    token: u32,
    seq: u32,
    ack: Option<u32>,
    sent: VecDeque<(u32, Instant)>,
    clock: (u32, Instant),
    last_rx: Instant,
    started: Instant,
    state: EyeState,
}
impl UdpLink {
    pub fn new(address: &str, state: EyeState) -> Result<Self> {
        let mut address = parse_address(address)?;
        address.set_port(state.udp_port.context("Firmware sin UDP")?);
        ensure!(
            state.udp_session != 0 && state.activo,
            "Sesión UDP no activa"
        );
        let stamp = state
            .sampled_at
            .context("Negociación UDP demasiado lenta; reintentando")?;
        let uptime = state
            .diag
            .as_ref()
            .and_then(|d| d["uptime_ms"].as_u64())
            .context("Falta reloj de la placa")? as u32;
        let socket = UdpSocket::bind(if address.is_ipv4() {
            "0.0.0.0:0"
        } else {
            "[::]:0"
        })?;
        socket.connect(address)?;
        socket.set_nonblocking(true)?;
        let now = Instant::now();
        Ok(Self {
            socket,
            token: state.udp_session,
            seq: 0,
            ack: None,
            sent: VecDeque::new(),
            clock: (uptime, stamp),
            last_rx: now,
            started: now,
            state,
        })
    }
    /// No buffered commands and no retransmission: the next tick supersedes this one.
    pub fn send(&mut self, angles: [f32; 2], lid: f32, rest: bool) -> Result<()> {
        ensure!(
            angles.iter().chain([lid].iter()).all(|v| v.is_finite()),
            "Ángulos no finitos"
        );
        ensure!(
            angles[0].abs() <= 55.001
                && angles[1].abs() <= 35.001
                && (0.0..=if rest { 60. } else { 32. }).contains(&lid),
            "Orden fuera de límites"
        );
        ensure!(
            self.last_rx.elapsed() < Duration::from_millis(600),
            "Sin telemetría UDP durante 600 ms"
        );
        let now = Instant::now();
        self.seq = self.seq.wrapping_add(1);
        let mut p = [0u8; 24];
        p[..4].copy_from_slice(b"OJ19");
        p[4..8].copy_from_slice(&self.token.to_le_bytes());
        p[8..12].copy_from_slice(&self.seq.to_le_bytes());
        let ms = self
            .clock
            .0
            .wrapping_add(now.saturating_duration_since(self.clock.1).as_millis() as u32);
        p[12..16].copy_from_slice(&ms.to_le_bytes());
        for (i, v) in [angles[0], angles[1], lid].iter().enumerate() {
            p[16 + i * 2..18 + i * 2].copy_from_slice(&((*v * 100.).round() as i16).to_le_bytes());
        }
        p[22] = u8::from(rest);
        match self.socket.send(&p) {
            Ok(n) => ensure!(n == 24, "Datagrama incompleto"),
            Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => return Ok(()),
            Err(e) => return Err(e.into()),
        }
        self.sent.push_back((self.seq, now));
        while self.sent.len() > 64 {
            self.sent.pop_front();
        }
        Ok(())
    }
    pub fn paused(&self) -> bool {
        self.ack.is_none()
            || self.last_rx.elapsed() > Duration::from_millis(180)
            || self.state.sampled_at.is_none()
    }
    pub fn poll(&mut self) -> Result<Option<EyeState>> {
        let mut changed = false;
        // Bounded draining prevents a faulty sender from starving vision/control.
        for _ in 0..64 {
            let mut p = [0u8; 64];
            let n = match self.socket.recv(&mut p) {
                Ok(n) => n,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(e.into()),
            };
            if n != 36 || &p[..4] != b"OS19" {
                continue;
            }
            let u32at = |i| u32::from_le_bytes(p[i..i + 4].try_into().unwrap());
            let u16at = |i| u16::from_le_bytes(p[i..i + 2].try_into().unwrap());
            let seq = u32at(8);
            if u32at(4) != self.token || self.ack.is_some_and(|a| seq.wrapping_sub(a) as i32 <= 0) {
                continue;
            }
            let Some((_, sent)) = self.sent.iter().find(|(s, _)| *s == seq).copied() else {
                continue;
            };
            let now = Instant::now();
            let rtt = now.duration_since(sent);
            // Late feedback must not make old telemetry look fresh.
            if rtt > Duration::from_millis(180) {
                continue;
            }
            self.ack = Some(seq);
            let rx_gap = now.duration_since(self.last_rx);
            self.last_rx = now;
            self.state.sampled_at = (rtt <= Duration::from_millis(100)).then_some(sent + rtt / 2);
            if let Some(t) = self.state.sampled_at {
                self.clock = (u32at(12), t);
            }
            self.state.actual = Some(std::array::from_fn(|i| {
                (u16at(16 + i * 2) as i16) as f32 * 0.01
            }));
            self.state.parpado_seguimiento = Some(u16at(22) as f32 * 0.01);
            let f = p[24];
            self.state.activo = f & 1 != 0;
            self.state.seguimiento = f & 2 != 0;
            self.state.parpadeando = f & 4 != 0;
            self.state.reposo = Some(f & 8 != 0);
            self.state.pca = f & 16 != 0;
            self.state.auto = false;
            if let Some(d) = self.state.diag.as_mut() {
                d["udp_rtt_ms"] = (rtt.as_micros() as f64 / 1000.).into();
                d["udp_rx_gap_ms"] = (rx_gap.as_micros() as f64 / 1000.).into();
                d["udp_ack"] = seq.into();
                d["uptime_ms"] = u32at(12).into();
                d["wifi_drops"] = u16at(28).into();
                d["i2c_errors"] = u16at(30).into();
                d["drive_gap_ms"] = u16at(32).into();
            }
            changed = true;
        }
        if self.last_rx.elapsed() > Duration::from_millis(600)
            && self.started.elapsed() > Duration::from_millis(600)
        {
            anyhow::bail!("Sin telemetría UDP durante 600 ms");
        }
        Ok(changed.then(|| self.state.clone()))
    }
}
