#[cfg(windows)]
mod gpu;
mod ui;
use std::{
    sync::atomic::Ordering,
    time::{Duration, Instant},
};
use ztrak_ojo::{camera, config::Config, controller, shared::Shared, vision};

/// On Windows the log also goes to ztrak-ojo-log.txt, one folder above the program.
fn init_log() {
    let mut logger =
        env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"));
    #[cfg(windows)]
    {
        struct Tee(std::fs::File);
        impl std::io::Write for Tee {
            fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
                let _ = std::io::stderr().write_all(b);
                self.0.write_all(b)?;
                Ok(b.len())
            }
            fn flush(&mut self) -> std::io::Result<()> {
                self.0.flush()
            }
        }
        let path = std::env::current_exe()
            .ok()
            .and_then(|e| e.parent()?.parent().map(|p| p.join("ztrak-ojo-log.txt")));
        if let Some(file) = path.and_then(|p| std::fs::File::create(p).ok()) {
            logger.target(env_logger::Target::Pipe(Box::new(Tee(file))));
        }
        std::panic::set_hook(Box::new(|p| log::error!("Fallo interno: {p}")));
    }
    logger.init();
}

fn main() -> anyhow::Result<()> {
    init_log();
    let args: Vec<String> = std::env::args().collect();
    if args.iter().any(|s| s == "--camaras") {
        for (source, name) in camera::cameras() {
            println!("{source}\t{name}");
        }
        return Ok(());
    }
    let mut cfg = Config::load();
    if let Some(i) = args.iter().position(|s| s == "--modos") {
        let source = args.get(i + 1).cloned().unwrap_or(cfg.camera.source.clone());
        println!("Modos de {source}:");
        for m in camera::modes(&source) {
            println!("  {m}");
        }
        return Ok(());
    }
    // Windows never picks a webcam by itself: the first one may be the bear's,
    // and the eye would move with the wrong image. It waits for the selector.
    let list = camera::cameras();
    if cfg!(windows) {
        if cfg.camera.source.starts_with("/dev/") {
            cfg.camera.source.clear();
        }
    } else if !Config::path().exists() || cfg.camera.source.trim().is_empty() {
        if let Some((source, _)) = list.iter().find(|(_, name)| !name.contains(" IR ")) {
            cfg.camera.source = source.clone();
        }
    }
    if !cfg.camera.source.trim().is_empty() && !camera::available(&cfg.camera.source, &list) {
        log::warn!(
            "La cámara guardada «{}» no está conectada. Cámaras disponibles: {:?}",
            cfg.camera.source,
            list.iter().map(|(_, n)| n).collect::<Vec<_>>()
        );
    }
    if args.iter().any(|s| s == "--prueba-camara") {
        if let Some(i) = args.iter().position(|s| s == "--archivo") {
            cfg.camera.source = args
                .get(i + 1)
                .ok_or_else(|| anyhow::anyhow!("Falta archivo de prueba"))?
                .clone();
            cfg.camera.rotate_180 = false;
        }
    }
    cfg.validate()?;
    if args.iter().any(|s| s == "--prueba-enlace") {
        let mut client = ztrak_ojo::eye::Client::default();
        let mut samples = Vec::new();
        let mut errors = 0;
        for _ in 0..60 {
            let t = Instant::now();
            match client.request(&cfg.endpoint(), "/estado") {
                Ok(_) => samples.push(t.elapsed().as_secs_f64() * 1000.),
                Err(e) => {
                    errors += 1;
                    eprintln!("Enlace: {e:#}");
                }
            }
            std::thread::sleep(Duration::from_millis(50));
        }
        samples.sort_by(f64::total_cmp);
        println!(
            "Lecturas: {}; errores: {errors}; mediana: {:.1} ms; máximo: {:.1} ms",
            samples.len(),
            samples.get(samples.len() / 2).copied().unwrap_or(0.),
            samples.last().copied().unwrap_or(0.)
        );
        anyhow::ensure!(errors == 0, "El enlace ha registrado errores");
        return Ok(());
    }
    let shared = Shared::new(cfg);
    let capture = camera::spawn(shared.clone());
    let vision = vision::spawn(shared.clone());
    if args.iter().any(|s| s == "--prueba-camara") {
        let start = Instant::now();
        let mut images = 0;
        let mut scenes = 0;
        let mut people = 0;
        let mut face_frames = 0;
        let mut body_frames = 0;
        let mut last = None;
        let mut last_scene = None;
        let mut detector_ms = 0.;
        let mut pose_ms = 0.;
        while start.elapsed() < Duration::from_secs(15) {
            if let Some(f) = shared.camera.get() {
                if last != Some(f.time) {
                    images += 1;
                    last = Some(f.time);
                }
            }
            if let Some(s) = shared.scene.get() {
                if last_scene != s.time {
                    scenes += 1;
                    detector_ms += s.detect_ms;
                    pose_ms += s.pose_ms;
                    people = people.max(s.people.len());
                    for p in s.people.iter().filter(|p| p.focus && p.visible) {
                        if let Some(a) =
                            ztrak_ojo::geometry::target(p, s.width, s.height, &shared.cfg().mount)
                        {
                            if a.face {
                                face_frames += 1;
                            } else {
                                body_frames += 1;
                            }
                        }
                    }
                    last_scene = s.time;
                }
            }
            std::thread::sleep(Duration::from_millis(20));
        }
        shared.quit.store(true, Ordering::Relaxed);
        let _ = capture.join();
        let _ = vision.join();
        let st = shared.stats.lock();
        println!(
            "Imágenes: {images}; análisis: {scenes}; máximo personas: {people}; cámara {:.1} FPS; IA {:.1} FPS",
            st.camera_fps, st.vision_fps
        );
        println!(
            "Tiempo medio: detector {:.1} ms; puntos de cabeza {:.1} ms",
            detector_ms / scenes.max(1) as f32,
            pose_ms / scenes.max(1) as f32
        );
        println!("Observaciones del foco: cara {face_frames}, cuerpo {body_frames}");
        println!(
            "Errores: cámara {:?}; visión {:?}",
            st.camera_error, st.vision_error
        );
        anyhow::ensure!(
            images > 20 && scenes > 5,
            "No se pudo validar captura y detección"
        );
        return Ok(());
    }
    let control = controller::spawn(shared.clone());
    let diagnostic = ztrak_ojo::diagnostics::spawn(shared.clone());
    let options = eframe::NativeOptions {
        viewport: eframe::egui::ViewportBuilder::default()
            .with_inner_size([1280., 860.])
            .with_min_inner_size([960., 650.]),
        #[cfg(windows)]
        renderer: eframe::Renderer::Wgpu,
        #[cfg(windows)]
        wgpu_options: gpu::eframe_wgpu(),
        ..Default::default()
    };
    // --seguir: start tracking without pressing the button (stand, remote start).
    // Same rule as the button: never with a missing, ambiguous or bear webcam.
    if args.iter().any(|s| s == "--seguir") {
        let source = shared.cfg().camera.source.clone();
        let problem = if cfg!(windows) {
            camera::eye_camera_problem(&source, camera::bear_source().as_deref(), &list)
        } else {
            None
        };
        match problem {
            None => {
                log::info!("Seguimiento activado al arrancar (--seguir)");
                shared.set_tracking(true);
            }
            Some(p) => log::warn!("--seguir no activa el seguimiento: {p}"),
        }
    }
    let app_shared = shared.clone();
    let result = eframe::run_native(
        "ZTrak Ojo",
        options,
        Box::new(move |cc| {
            // A graphics error (e.g. out of memory for a moment) must not close the app.
            #[cfg(windows)]
            if let Some(rs) = cc.wgpu_render_state.as_ref() {
                rs.device.on_uncaptured_error(std::sync::Arc::new(|e| {
                    log::error!("Ventana: error de la gráfica: {e}")
                }));
            }
            Ok(Box::new(ui::App::new(cc, app_shared)))
        }),
    );
    shared.quit.store(true, Ordering::Relaxed);
    let _ = control.join();
    if let Some(diagnostic) = diagnostic {
        let _ = diagnostic.join();
    }
    let _ = capture.join();
    let _ = vision.join();
    result.map_err(|e| anyhow::anyhow!("Interfaz: {e}"))
}
