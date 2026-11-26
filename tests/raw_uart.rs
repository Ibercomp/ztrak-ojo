#![cfg(unix)]
use serialport::SerialPort;
use std::{
    io::{Read, Write},
    sync::{
        Arc,
        atomic::{AtomicBool, AtomicUsize, Ordering},
    },
    thread,
    time::{Duration, Instant},
};
use ztrak_ojo::{eye, raw_wire::*};
fn config() -> Vec<u8> {
    let mut b = vec![0; 48];
    b[..4].copy_from_slice(&[3, 0, 1, 20]);
    b[4..6].copy_from_slice(&15u16.to_le_bytes());
    b[6..8].copy_from_slice(&8000u16.to_le_bytes());
    for (i, v) in [1560u16, 1470, 1810].iter().enumerate() {
        b[8 + 2 * i..10 + 2 * i].copy_from_slice(&v.to_le_bytes());
    }
    b[14..20].copy_from_slice(&[1, 255, 255, 9, 10, 11]);
    for r in [20, 34] {
        for (i, v) in [0u16, 150, 250, 250, 250, 150, 0].iter().enumerate() {
            b[r + 2 * i..r + 2 * i + 2].copy_from_slice(&v.to_le_bytes());
        }
    }
    b
}
struct Fixture {
    address: String,
    stop: Arc<AtomicBool>,
    silent: Arc<AtomicBool>,
    moves: Arc<AtomicUsize>,
    thread: Option<thread::JoinHandle<()>>,
}
impl Fixture {
    fn new() -> Self {
        let (mut master, slave) = serialport::TTYPort::pair().unwrap();
        master.set_timeout(Duration::from_millis(2)).unwrap();
        let address = format!("uart:{}?baud=250000", slave.name().unwrap());
        let stop = Arc::new(AtomicBool::new(false));
        let silent = Arc::new(AtomicBool::new(false));
        let moves = Arc::new(AtomicUsize::new(0));
        let (st, si, m) = (stop.clone(), silent.clone(), moves.clone());
        let handle = thread::spawn(move || {
            let _keep_slave = slave;
            let start = Instant::now();
            let mut last = Instant::now();
            let mut session = 0;
            let mut seq = 0;
            let mut active = false;
            let mut pose = [0i16; 3];
            let mut buf = Vec::new();
            let send = |master: &mut serialport::TTYPort, kind, session, seq, data: Vec<u8>| {
                let p = Packet {
                    kind,
                    session,
                    seq,
                    time: (start.elapsed().as_millis() / 20 * 20) as u32,
                    data,
                };
                let wire = encode(&p).unwrap();
                for b in wire.chunks(7) {
                    master.write_all(b).unwrap();
                }
            };
            let telemetry = |pose: [i16; 3], active: bool| {
                let mut b = vec![0; 24];
                for (i, v) in pose.iter().enumerate() {
                    b[i * 2..i * 2 + 2].copy_from_slice(&v.to_le_bytes());
                }
                for (i, v) in [1560u16, 1470, 1810].iter().enumerate() {
                    b[6 + 2 * i..8 + 2 * i].copy_from_slice(&v.to_le_bytes());
                }
                b[14] = 64 | u8::from(active);
                b
            };
            while !st.load(Ordering::Relaxed) {
                let mut bytes = [0u8; 128];
                match master.read(&mut bytes) {
                    Ok(n) => {
                        for &b in &bytes[..n] {
                            if b != 0 {
                                buf.push(b);
                                continue;
                            }
                            if buf.is_empty() {
                                continue;
                            }
                            let packet = decode(&buf).unwrap();
                            buf.clear();
                            match packet.kind {
                                HELLO => {
                                    session = packet.session;
                                    seq = packet.seq;
                                    active = false;
                                    send(&mut master, CONFIG, session, seq, config());
                                    send(
                                        &mut master,
                                        TELEMETRY,
                                        session,
                                        seq,
                                        telemetry(pose, active),
                                    );
                                }
                                ARM => {
                                    assert_eq!(packet.session, session);
                                    seq = packet.seq;
                                    active = packet.data[0] != 0;
                                    send(&mut master, ACK, session, seq, vec![0, ARM]);
                                    send(
                                        &mut master,
                                        TELEMETRY,
                                        session,
                                        seq,
                                        telemetry(pose, active),
                                    );
                                }
                                TARGET => {
                                    assert_eq!(packet.session, session);
                                    assert_eq!(packet.data.len(), 7);
                                    m.fetch_add(1, Ordering::Relaxed);
                                    seq = packet.seq;
                                    pose =
                                        std::array::from_fn(|i| u16at(&packet.data, 2 * i) as i16);
                                }
                                STATE => {
                                    seq = packet.seq;
                                    if !si.load(Ordering::Relaxed) {
                                        send(
                                            &mut master,
                                            TELEMETRY,
                                            session,
                                            seq,
                                            telemetry(pose, active),
                                        );
                                    }
                                }
                                _ => panic!("Unexpected binary command"),
                            }
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(e) => panic!("{e}"),
                }
                if session != 0
                    && !si.load(Ordering::Relaxed)
                    && last.elapsed() > Duration::from_millis(20)
                {
                    send(
                        &mut master,
                        TELEMETRY,
                        session,
                        seq,
                        telemetry(pose, active),
                    );
                    last = Instant::now();
                }
            }
        });
        Self {
            address,
            stop,
            silent,
            moves,
            thread: Some(handle),
        }
    }
}
impl Drop for Fixture {
    fn drop(&mut self) {
        self.stop.store(true, Ordering::Relaxed);
        self.thread.take().unwrap().join().unwrap();
    }
}
#[test]
fn binary_handshake_fragmentation_native_outputs_and_no_motion_roundtrip() {
    let f = Fixture::new();
    let mut client = eye::Client::default();
    let s = client.request(&f.address, "/estado").unwrap();
    assert_eq!(s.firmware, "uno-avr-3.0-raw1");
    assert!(s.outputs_ready() && !s.pca && !s.activo);
    assert_eq!(s.centro, [1560, 1470, 1810]);
    assert!(client.send_uart_motion([0., 0.], 0., true).is_err());
    assert!(client.request(&f.address, "/encender").unwrap().activo);
    f.silent.store(true, Ordering::Relaxed);
    for _ in 0..8 {
        client.send_uart_motion([-12.34, 5.67], 32., true).unwrap();
        thread::sleep(Duration::from_millis(1));
    }
    let deadline = Instant::now();
    while f.moves.load(Ordering::Relaxed) < 8 && deadline.elapsed() < Duration::from_millis(200) {
        thread::sleep(Duration::from_millis(1));
    }
    assert_eq!(f.moves.load(Ordering::Relaxed), 8); // No TARGET response was sent by the server.
    f.silent.store(false, Ordering::Relaxed);
    thread::sleep(Duration::from_millis(25));
    let s = client.poll_uart_motion().unwrap().unwrap();
    assert_eq!(s.actual, Some([-12.34, 5.67, 32.]));
    assert!(!client.request(&f.address, "/apagar").unwrap().activo);
    assert!(client.send_uart_motion([0., 0.], 0., true).is_err());
}
#[test]
fn telemetry_pause_then_timeout_has_no_auto_rearm() {
    let f = Fixture::new();
    let mut client = eye::Client::default();
    client.request(&f.address, "/estado").unwrap();
    client.request(&f.address, "/encender").unwrap();
    f.silent.store(true, Ordering::Relaxed);
    thread::sleep(Duration::from_millis(130));
    client.poll_uart_motion().unwrap();
    assert!(client.uart_paused());
    thread::sleep(Duration::from_millis(510));
    assert!(client.poll_uart_motion().is_err());
    assert!(client.send_uart_motion([0., 0.], 0., true).is_err());
}
