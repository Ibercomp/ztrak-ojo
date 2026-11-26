//! Controller + actual firmware motor in local HTTP simulator; no hardware.
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
    anyhow::ensure!(
        addr.ip().is_loopback(),
        "Esta prueba es exclusivamente local"
    );
    let mut cfg = Config::load();
    cfg.eye_address = address;
    cfg.uart = false;
    cfg.mount.mode = MountMode::Pupil;
    cfg.mount.camera_ypr = [0.; 3];
    cfg.boredom_enabled = true;
    cfg.boredom_after_s = 18.;
    cfg.droop_s = 8.;
    cfg.sleep_slit = 32.;
    let shared = Shared::new(cfg);
    let worker = controller::spawn(shared.clone());
    shared.set_tracking(true);
    let start = Instant::now();
    let result = (|| -> anyhow::Result<()> {
        let mut n = 0;
        let mut last = String::new();
        let mut sleeping = None;
        let mut partial = false;
        let mut awakened = false;
        while start.elapsed() < Duration::from_secs(40) {
            let now = Instant::now();
            n += 1;
            let stats = shared.stats.lock().clone();
            if stats.status != last {
                println!("{:.1}s {}", start.elapsed().as_secs_f32(), stats.status);
                last = stats.status.clone();
            }
            if stats.status.starts_with("Qué sueño") {
                if let Some(s) = &stats.eye {
                    partial |= s.actual.is_some_and(|a| a[2] > 8. && a[2] < 24.);
                }
            }
            if stats.status.starts_with("Zzz") {
                sleeping.get_or_insert(now);
            }
            let trigger =
                sleeping.is_some_and(|t: Instant| now.duration_since(t) > Duration::from_secs(2));
            awakened |= trigger && stats.status.starts_with("¡Eh!");
            if awakened && stats.status == "Mirando a la cara" {
                anyhow::ensure!(partial, "Falta cierre gradual");
                println!(
                    "OK: quietud → cierre gradual → rendija 32° → movimiento de imagen → mirada"
                );
                return Ok(());
            }
            let mut rgb = vec![100; 96 * 54 * 3];
            if trigger {
                for y in 20..30 {
                    for x in 35..55 {
                        for c in 0..3 {
                            rgb[(y * 96 + x) * 3 + c] = 180;
                        }
                    }
                }
            }
            let mut k = [Kpt::default(); 17];
            k[0] = Kpt {
                x: 640.,
                y: 360.,
                c: 0.95,
            };
            k[1] = Kpt {
                x: 620.,
                y: 360.,
                c: 0.95,
            };
            k[2] = Kpt {
                x: 660.,
                y: 360.,
                c: 0.95,
            };
            shared.camera.put(Frame {
                id: n,
                width: 96,
                height: 54,
                rgb,
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
                    bbox: [500., 270., 780., 700.],
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
        anyhow::bail!("No completó aburrimiento/despertar en 40 s")
    })();
    shared.quit.store(true, Ordering::Relaxed);
    let _ = worker.join();
    result
}
