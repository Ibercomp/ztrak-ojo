#![cfg(unix)]
use serialport::SerialPort;
use std::{
    io::{Read, Write},
    thread,
    time::Duration,
};
use ztrak_ojo::eye;
const STATE: &str = r#"{"firmware":"rtos-ota-14","activo":true,"auto":false,"pca":true,"seguimiento":true,"reposo":false,"actual":[12,-4,26],"centro":[1560,1470,1810],"sentido":[1,-1,-1],"us":[1680,1510,1700]}"#;
fn pair() -> (serialport::TTYPort, serialport::TTYPort, String) {
    let (mut master, slave) = serialport::TTYPort::pair().unwrap();
    master.set_timeout(Duration::from_secs(3)).unwrap();
    let name = slave.name().unwrap();
    (master, slave, format!("uart:{name}"))
}
fn request(p: &mut serialport::TTYPort) -> String {
    let mut line = Vec::new();
    let mut b = [0u8; 1];
    loop {
        p.read_exact(&mut b).unwrap();
        if b[0] == b'\n' {
            return String::from_utf8(line).unwrap();
        }
        line.push(b[0]);
    }
}
#[test]
fn real_pty_handles_boot_noise_fragmentation_and_matching_sequences() {
    let (mut server, _slave, addr) = pair();
    let (done, wait) = std::sync::mpsc::channel();
    let worker = thread::spawn(move || {
        for seq in 1..=3 {
            let q = request(&mut server);
            assert_eq!(
                q,
                format!("@OJO {seq} /seguir?giro=12&cabeceo=-4&parpado=26")
            );
            write!(server, "boot log\n@OJO 999 200 {{}}\n").unwrap();
            let reply = format!("@OJO {seq} 200 {STATE}\n");
            for part in reply.as_bytes().chunks(17) {
                server.write_all(part).unwrap();
            }
        }
        wait.recv_timeout(Duration::from_secs(2)).unwrap();
    });
    let mut client = eye::Client::default();
    for _ in 0..3 {
        let s = client
            .request(&addr, "/seguir?giro=12&cabeceo=-4&parpado=26")
            .unwrap();
        assert_eq!(s.position(), [12., -4.]);
        assert_eq!(s.reposo, Some(false));
    }
    done.send(()).unwrap();
    worker.join().unwrap();
}
#[test]
fn rejection_is_reported_and_never_treated_as_success() {
    let (mut server, _slave, addr) = pair();
    let worker = thread::spawn(move || {
        request(&mut server);
        write!(server, "@OJO 1 409 {{\"error\":\"apagado\"}}\n").unwrap();
        thread::sleep(Duration::from_millis(100));
    });
    let e = eye::Client::default()
        .request(&addr, "/seguir?giro=1&cabeceo=2")
        .unwrap_err();
    assert!(e.to_string().contains("409"));
    worker.join().unwrap();
}
#[test]
fn timeout_never_uses_an_old_reply() {
    let (mut server, _slave, addr) = pair();
    let worker = thread::spawn(move || {
        request(&mut server);
        write!(server, "@OJO 0 200 {STATE}\n").unwrap();
        thread::sleep(Duration::from_millis(450));
    });
    assert!(
        eye::Client::default()
            .request(&addr, "/estado")
            .unwrap_err()
            .to_string()
            .contains("350 ms")
    );
    worker.join().unwrap();
}
