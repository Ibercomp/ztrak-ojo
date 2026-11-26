use std::{
    io::{Read, Write},
    net::TcpListener,
    thread,
};
use ztrak_ojo::eye;
#[test]
fn tcp_transport_sends_follow_and_reads_position() {
    let server = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = server.local_addr().unwrap();
    let worker = thread::spawn(move || {
        let (mut sock, _) = server.accept().unwrap();
        let mut req = Vec::new();
        let mut b = [0u8; 512];
        while !req.windows(4).any(|s| s == b"\r\n\r\n") {
            let n = sock.read(&mut b).unwrap();
            assert!(n > 0);
            req.extend_from_slice(&b[..n]);
        }
        assert!(
            String::from_utf8(req)
                .unwrap()
                .starts_with("GET /seguir?giro=12.5&cabeceo=-4 HTTP/1.1")
        );
        let body = r#"{"firmware":"rtos-ota-9","activo":true,"auto":false,"pca":true,"seguimiento":true,"parpadeando":true,"actual":[12.5,-4,30],"centro":[1560,1470,1810],"sentido":[1,-1,-1],"us":[1685,1510,1500]}"#;
        write!(
            sock,
            "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            body.len(),
            body
        )
        .unwrap();
    });
    let state = eye::request(&addr.to_string(), "/seguir?giro=12.5&cabeceo=-4").unwrap();
    assert_eq!(state.position(), [12.5, -4.]);
    assert!(state.supports_tracking() && state.parpadeando);
    worker.join().unwrap();
}

const STATE: &str = r#"{"firmware":"rtos-ota-9","activo":false,"auto":false,"pca":true,"seguimiento":false,"parpadeando":false,"actual":[0,0,0],"centro":[1560,1470,1810],"sentido":[1,-1,-1],"us":[1560,1470,1810]}"#;
fn read_request(sock: &mut std::net::TcpStream) {
    let mut buf = Vec::new();
    let mut byte = [0u8; 1];
    while !buf.ends_with(b"\r\n\r\n") {
        assert_eq!(sock.read(&mut byte).unwrap(), 1);
        buf.push(byte[0]);
    }
}
#[test]
fn repeated_requests_reuse_one_connection() {
    let server = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = server.local_addr().unwrap();
    let worker = thread::spawn(move || {
        let (mut sock, _) = server.accept().unwrap();
        sock.set_read_timeout(Some(std::time::Duration::from_secs(2)))
            .unwrap();
        for _ in 0..3 {
            read_request(&mut sock);
            write!(
                sock,
                "HTTP/1.1 200 OK\r\nContent-Length: {}\r\n\r\n{}",
                STATE.len(),
                STATE
            )
            .unwrap();
        }
    });
    let mut client = eye::Client::default();
    for _ in 0..3 {
        assert!(client.request(&addr.to_string(), "/estado").unwrap().pca);
    }
    worker.join().unwrap();
}
#[test]
fn truncated_reply_reconnects_without_panicking() {
    let server = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = server.local_addr().unwrap();
    let worker = thread::spawn(move || {
        let (mut sock, _) = server.accept().unwrap();
        read_request(&mut sock);
        write!(sock, "HTTP/1.1 200 OK\r\nContent-Length: 100\r\n\r\n{{").unwrap();
        drop(sock);
        let (mut sock, _) = server.accept().unwrap();
        read_request(&mut sock);
        write!(
            sock,
            "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            STATE.len(),
            STATE
        )
        .unwrap();
    });
    let mut client = eye::Client::default();
    assert!(client.request(&addr.to_string(), "/estado").is_err());
    assert!(client.request(&addr.to_string(), "/estado").unwrap().pca);
    worker.join().unwrap();
}

#[test]
fn stalled_response_reconnects_before_long_old_timeout() {
    use std::time::{Duration, Instant};
    let server = TcpListener::bind("127.0.0.1:0").unwrap();
    let addr = server.local_addr().unwrap();
    let worker = thread::spawn(move || {
        let (mut old, _) = server.accept().unwrap();
        read_request(&mut old);
        // Stall just beyond the new deadline. No replay of the old command.
        thread::sleep(Duration::from_millis(320));
        drop(old);
        let (mut fresh, _) = server.accept().unwrap();
        read_request(&mut fresh);
        write!(
            fresh,
            "HTTP/1.1 200 OK\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
            STATE.len(),
            STATE
        )
        .unwrap();
    });
    let mut client = eye::Client::default();
    let started = Instant::now();
    assert!(client.request(&addr.to_string(), "/estado").is_err());
    assert!(client.request(&addr.to_string(), "/estado").unwrap().pca);
    assert!(started.elapsed() < Duration::from_millis(650));
    worker.join().unwrap();
}
