//! Local simulator only: joystick ownership, stale UI input and board limit synchronization.
use std::{
    sync::atomic::Ordering,
    time::{Duration, Instant},
};
use ztrak_ojo::{config::Config, controller, manual::ManualInput, shared::Shared};
fn main() -> anyhow::Result<()> {
    let endpoint = std::env::args().nth(1).unwrap_or("127.0.0.1:8123".into());
    let address: std::net::SocketAddr = endpoint.parse()?;
    anyhow::ensure!(address.ip().is_loopback(), "Simulación local únicamente");
    let mut cfg = Config::load();
    cfg.eye_address = endpoint;
    cfg.uart = false;
    let shared = Shared::new(cfg);
    let worker = controller::spawn(shared.clone());
    shared.set_manual(true);
    let start = Instant::now();
    let mut peak = 0f32;
    let mut held = None;
    let mut saved = false;
    let result = (|| -> anyhow::Result<()> {
        while start.elapsed() < Duration::from_secs(16) {
            let now = Instant::now();
            let t = start.elapsed().as_secs_f32();
            if t < 4. {
                *shared.manual_input.lock() = ManualInput {
                    stick: [1., 0.],
                    speed: 8.,
                    stamp: now,
                    center: false,
                    target: None,
                    lid: 0.,
                };
            }
            // No UI refresh between 4 and 6 s: command must stop safely while camera is absent.
            if (6. ..12.).contains(&t) {
                *shared.manual_input.lock() = ManualInput {
                    stick: [0.; 2],
                    speed: 8.,
                    stamp: now,
                    center: true,
                    target: None,
                    lid: 0.,
                };
            }
            let stats = shared.stats.lock().clone();
            if let Some(s) = &stats.eye {
                if t > 1. && t < 12. {
                    anyhow::ensure!(
                        s.activo && s.reposo == Some(true) && !s.parpadeando,
                        "Manual must own gaze and suppress automatic blinking"
                    );
                }
                peak = peak.max(s.position()[0]);
                if (4.8..5.9).contains(&t) {
                    let p = *held.get_or_insert(s.position()[0]);
                    anyhow::ensure!(
                        (p - s.position()[0]).abs() < 0.15,
                        "Stale input kept moving"
                    );
                }
                if t > 12. && shared.manual() {
                    anyhow::ensure!(s.position()[0].abs() < 0.2, "Return to center incomplete");
                    shared.stop();
                }
                if t > 13. && !s.activo && !saved {
                    let mut lim = s.limits.unwrap();
                    lim.up[6] = 10;
                    *shared.limits_request.lock() = Some(lim);
                    saved = true;
                }
                if t > 14. && s.limits.is_some_and(|l| l.up[6] == 10) {
                    anyhow::ensure!(peak > 20., "Joystick did not move");
                    println!(
                        "OK: manual without camera, release/stale input, center, off and limits confirmed by board"
                    );
                    return Ok(());
                }
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        anyhow::bail!(
            "Manual test timeout: {}",
            shared.stats.lock().calibration_message
        )
    })();
    shared.quit.store(true, Ordering::Relaxed);
    let _ = worker.join();
    result
}
