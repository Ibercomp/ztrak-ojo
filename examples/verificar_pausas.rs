//! Local-only replay: moving observations during a lid gesture + network stalls.
use std::{
    sync::atomic::Ordering,
    time::{Duration, Instant},
};
use ztrak_ojo::{
    config::{Config, MountMode},
    controller,
    shared::{Frame, Kpt, Person, Scene, Shared},
};
fn main() -> anyhow::Result<()> {
    let address = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "127.0.0.1:8123".into());
    let addr: std::net::SocketAddr = address.parse()?;
    anyhow::ensure!(addr.ip().is_loopback(), "Simulador local únicamente");
    let mut cfg = Config::load();
    cfg.eye_address = address;
    cfg.uart = false;
    cfg.mount.mode = MountMode::Pupil;
    cfg.mount.camera_ypr = [0.; 3];
    cfg.sleep_enabled = false;
    cfg.boredom_enabled = false;
    cfg.curiosity = true;
    let shared = Shared::new(cfg);
    let worker = controller::spawn(shared.clone());
    shared.set_tracking(true);
    let start = Instant::now();
    let mut n = 0;
    let mut gesture = None;
    let mut saw_hold = false;
    let mut saw_error = false;
    while start.elapsed() < Duration::from_secs(17) {
        let now = Instant::now();
        n += 1;
        let stats = shared.stats.lock().clone();
        saw_error |= stats.eye_error.is_some()
            || (start.elapsed() > Duration::from_secs(1)
                && stats.status.starts_with("Recuperando conexión"));
        if let Some(s) = stats.eye {
            if s.parpado_seguimiento.unwrap_or(0.) >= 26. && !s.parpadeando {
                if let Some(a) = s.actual {
                    if (a[2] - s.parpado_seguimiento.unwrap()).abs() > 0.1 {
                        gesture.get_or_insert(now);
                        saw_hold = true;
                    }
                }
            }
        }
        let shift = gesture.is_some_and(|t| now.duration_since(t) < Duration::from_millis(600));
        let x = if shift { 500. } else { 640. };
        let mut k = [Kpt::default(); 17];
        k[0] = Kpt {
            x,
            y: 360.,
            c: 0.95,
        };
        k[1] = Kpt {
            x: x - 20.,
            y: 360.,
            c: 0.95,
        };
        k[2] = Kpt {
            x: x + 20.,
            y: 360.,
            c: 0.95,
        };
        shared.camera.put(Frame {
            id: n,
            width: 96,
            height: 54,
            rgb: vec![100; 96 * 54 * 3],
            time: now,
        });
        shared.scene.put(Scene {
            frame_id: n,
            focus_epoch: shared.focus_epoch.load(Ordering::Relaxed),
            width: 1280,
            height: 720,
            people: vec![Person {
                id: 1,
                visible: true,
                bbox: [x - 140., 270., x + 140., 700.],
                score: 0.95,
                kpts: Some(k),
                focus: true,
                quality: 1.,
                age: 10,
            }],
            focus_id: Some(1),
            time: Some(now),
            ..Default::default()
        });
        std::thread::sleep(Duration::from_millis(40));
    }
    shared.quit.store(true, Ordering::Relaxed);
    let _ = worker.join();
    anyhow::ensure!(saw_hold, "No ejercitó la pausa de párpado");
    anyhow::ensure!(saw_error, "Activa --faults en el simulador");
    println!(
        "OK: gestos de párpado con observaciones cambiantes y errores de enlace ejercitados; comprobar registro de órdenes"
    );
    Ok(())
}
