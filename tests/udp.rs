use std::{
    net::UdpSocket,
    time::{Duration, Instant},
};
use ztrak_ojo::{eye::EyeState, udp_link::UdpLink};
fn rig() -> (UdpSocket, UdpLink) {
    let server = UdpSocket::bind("127.0.0.1:0").unwrap();
    server
        .set_read_timeout(Some(Duration::from_millis(100)))
        .unwrap();
    let mut s:EyeState=serde_json::from_str(r#"{"firmware":"rtos-ota-19","udp_session":123,"activo":true,"auto":false,"pca":true,"centro":[1560,1470,1810],"sentido":[1,-1,-1],"us":[1560,1470,1810],"actual":[0,0,0],"diag":{"uptime_ms":1000}}"#).unwrap();
    s.udp_port = Some(server.local_addr().unwrap().port());
    s.sampled_at = Some(Instant::now());
    let link = UdpLink::new("127.0.0.1:80", s).unwrap();
    (server, link)
}
fn reply(command: &[u8], yaw: i16) -> [u8; 36] {
    let mut r = [0u8; 36];
    r[..4].copy_from_slice(b"OS19");
    r[4..16].copy_from_slice(&command[4..16]);
    r[16..18].copy_from_slice(&yaw.to_le_bytes());
    r[24] = 19;
    r
}
fn wait_feedback(link: &mut UdpLink) -> EyeState {
    let start = Instant::now();
    loop {
        if let Some(s) = link.poll().unwrap() {
            return s;
        }
        assert!(start.elapsed() < Duration::from_millis(100));
        std::thread::sleep(Duration::from_millis(1));
    }
}
#[test]
fn binary_motion_does_not_wait_for_ack_and_discards_old_feedback() {
    let (server, mut link) = rig();
    let mut b = [0u8; 100];
    let start = Instant::now();
    link.send([-12.34, 5.67], 32., true).unwrap();
    assert!(start.elapsed() < Duration::from_millis(50));
    assert!(link.paused());
    let (n, peer) = server.recv_from(&mut b).unwrap();
    assert_eq!(n, 24);
    assert_eq!(&b[..4], b"OJ19");
    assert_eq!(i16::from_le_bytes(b[16..18].try_into().unwrap()), -1234);
    let old = reply(&b, -1234);
    // Drop this command's reply: next motion must still send immediately.
    link.send([7., 1.], 0., false).unwrap();
    let (n, _) = server.recv_from(&mut b).unwrap();
    assert_eq!(n, 24);
    let fresh = reply(&b, 700);
    server.send_to(&fresh, peer).unwrap();
    assert_eq!(wait_feedback(&mut link).position(), [7., 0.]);
    assert!(!link.paused());
    server.send_to(&old, peer).unwrap();
    server.send_to(&fresh, peer).unwrap();
    std::thread::sleep(Duration::from_millis(3));
    assert!(link.poll().unwrap().is_none());
    assert!(link.send([f32::NAN, 0.], 0., false).is_err());
    assert!(link.send([56., 0.], 0., false).is_err());
    assert!(link.send([0., 0.], 33., false).is_err());
}
#[test]
fn wrong_session_and_missing_feedback_never_look_connected() {
    let (server, mut link) = rig();
    let mut b = [0u8; 100];
    link.send([0.; 2], 0., false).unwrap();
    let (_, peer) = server.recv_from(&mut b).unwrap();
    let mut r = reply(&b, 1000);
    r[4] = 99;
    server.send_to(&r, peer).unwrap();
    std::thread::sleep(Duration::from_millis(5));
    assert!(link.poll().unwrap().is_none());
    assert!(link.paused());
    std::thread::sleep(Duration::from_millis(610));
    assert!(link.poll().is_err());
    assert!(link.send([0.; 2], 0., false).is_err());
}

#[test]
fn delayed_feedback_does_not_resume_motion_just_because_it_arrived_now() {
    let (server, mut link) = rig();
    let mut b = [0u8; 100];
    link.send([0.; 2], 0., false).unwrap();
    let (_, peer) = server.recv_from(&mut b).unwrap();
    std::thread::sleep(Duration::from_millis(120));
    server.send_to(&reply(&b, 0), peer).unwrap();
    wait_feedback(&mut link);
    assert!(
        link.paused(),
        "120 ms old pose must not count as current feedback"
    );
    link.send([0.; 2], 0., false).unwrap();
    server.recv_from(&mut b).unwrap();
    server.send_to(&reply(&b, 0), peer).unwrap();
    wait_feedback(&mut link);
    assert!(!link.paused());
}
