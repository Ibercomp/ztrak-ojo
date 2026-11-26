//! Opt-in physical test. Simulates camera observations; moves the real eye.
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
    let args: Vec<_> = std::env::args().collect();
    let simulator = args.iter().find_map(|s| s.strip_prefix("--simulador="));
    anyhow::ensure!(
        simulator.is_some() || args.iter().any(|a| a == "--mover-ojo"),
        "Usa --simulador=127.0.0.1:puerto o --mover-ojo (mueve el mecanismo real)"
    );
    let mut cfg = Config::load();
    if let Some(address) = simulator {
        let addr: std::net::SocketAddr = address.parse()?;
        anyhow::ensure!(addr.ip().is_loopback(), "El simulador debe ser local");
        cfg.eye_address = address.into();
        cfg.uart = false;
    }
    cfg.mount.mode = MountMode::Pupil;
    cfg.mount.camera_ypr = [0.; 3]; // Synthetic observations use an aligned camera.
    cfg.sleep_enabled = true;
    cfg.sleep_after_s = 30.;
    cfg.sleep_for_s = 5.;
    cfg.reset_pause_s = 2.;
    let shared = Shared::new(cfg);
    let trace = ztrak_ojo::diagnostics::spawn(shared.clone());
    let control = controller::spawn(shared.clone());
    let begin = Instant::now();
    let result = (|| -> anyhow::Result<()> {
        let mut n = 0;
        let mut last_status = String::new();
        let (mut recovered, mut asleep_count, mut left, mut right, mut resumed, mut curious) =
            (false, 0, false, false, false, false);
        let mut was_sleeping = false;
        let mut inject_center = false;
        let mut reached_edge = false;
        let mut search_blinked = false;
        let mut search_squinted = false;
        shared.set_tracking(true);
        while begin.elapsed() < Duration::from_secs(85) {
            let now = Instant::now();
            n += 1;
            let st = shared.stats.lock().clone();
            if st.status != last_status {
                println!("{:.1}s {}", begin.elapsed().as_secs_f32(), st.status);
                last_status = st.status.clone();
            }
            if st.status.contains("Fuera de alcance") {
                recovered = true;
            }
            let sleeping = st.status.starts_with("Durmiendo");
            if sleeping && !was_sleeping {
                asleep_count += 1;
            }
            was_sleeping = sleeping;
            if let Some(s) = st.eye {
                let pose = s.position();
                let cap = ztrak_ojo::geometry::envelope([100., pose[1]])[0];
                reached_edge |= pose[0].abs() > cap - 0.3 || pose[1].abs() > 24.7;
                if st.status.contains("ambos lados") {
                    search_blinked |= s.parpadeando;
                    search_squinted |= !s.parpadeando
                        && s.actual.is_some_and(|a| {
                            a[2] >= 31. && a[2] <= 33. && a[1].abs() < 0.5 && a[0].abs() > 20.
                        });
                    left |= s.position()[0] < -20.;
                    right |= s.position()[0] > 20.;
                    if asleep_count >= 2 {
                        inject_center = true;
                    }
                }
                resumed |= inject_center && st.status == "Mirando a la cara";
                curious |= resumed && s.parpado_seguimiento.unwrap_or(0.) >= 26.;
            }
            if recovered
                && asleep_count >= 2
                && left
                && right
                && resumed
                && curious
                && search_blinked
                && search_squinted
            {
                println!(
                    "OK: reset del foco, pausa, sueño 30/5 s, búsqueda entrecerrada con parpadeos, nueva cara y curiosidad"
                );
                return Ok(());
            }
            let mut people = vec![];
            if ((!recovered && !reached_edge) || inject_center)
                && !shared.focus_paused.load(Ordering::Relaxed)
            {
                let x = if inject_center { 640. } else { 1200. };
                let mut k = [Kpt::default(); 17];
                k[0] = Kpt {
                    x,
                    y: 360.,
                    c: 0.95,
                };
                k[1] = Kpt {
                    x: x - 15.,
                    y: 360.,
                    c: 0.95,
                };
                k[2] = Kpt {
                    x: x + 15.,
                    y: 360.,
                    c: 0.95,
                };
                people.push(Person {
                    id: if inject_center { 2 } else { 1 },
                    bbox: [x - 50., 300., x + 50., 700.],
                    score: 0.95,
                    kpts: Some(k),
                    focus: true,
                    quality: 1.,
                    age: 10,
                    visible: true,
                });
            }
            let focus_id = people.first().map(|p| p.id);
            shared.camera.put(Frame {
                id: n,
                width: 1280,
                height: 720,
                rgb: vec![],
                time: now,
            });
            shared.scene.put(Scene {
                frame_id: n,
                focus_epoch: shared.focus_epoch.load(Ordering::Relaxed),
                width: 1280,
                height: 720,
                people,
                focus_id,
                time: Some(now),
                ..Default::default()
            });
            std::thread::sleep(Duration::from_millis(35));
        }
        anyhow::bail!("No terminó el ciclo completo en 85 segundos")
    })();
    shared.quit.store(true, Ordering::Relaxed);
    let _ = control.join();
    if let Some(trace) = trace {
        let _ = trace.join();
    }
    result
}
