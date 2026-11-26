//! Opt-in, bounded diagnostic capture. Independent of the vision/control loops.
use crate::shared::Shared;
use std::{
    io::Write,
    sync::Arc,
    time::{Duration, Instant},
};

pub fn spawn(shared: Arc<Shared>) -> Option<std::thread::JoinHandle<()>> {
    let path = std::env::var_os("ZTRAK_OJO_TRACE")?;
    Some(std::thread::spawn(move || {
        let result = (|| -> anyhow::Result<()> {
            let mut file = std::io::BufWriter::new(
                std::fs::OpenOptions::new()
                    .write(true)
                    .create_new(true)
                    .open(path)?,
            );
            let start = Instant::now();
            let mut previous = None;
            while !shared.quitting() && start.elapsed() < Duration::from_secs(120) {
                let Some(scene) = shared
                    .scene
                    .wait_new(previous.as_ref(), Duration::from_millis(100))
                else {
                    continue;
                };
                let st = shared.stats.lock().clone();
                let people: Vec<_> = scene.people.iter().map(|p| serde_json::json!({
                    "id": p.id, "visible": p.visible, "bbox": p.bbox, "score": p.score,
                    "head": p.kpts.map(|k| k[..7].iter().map(|v| [v.x,v.y,v.c]).collect::<Vec<_>>())
                })).collect();
                let aim = st.aim.as_ref().map(|a| {
                    serde_json::json!({
                        "person":a.person,"face":a.face,"pixel":a.pixel,
                        "raw":a.raw_angles,"command":a.angles,"limited":a.limited
                    })
                });
                serde_json::to_writer(
                    &mut file,
                    &serde_json::json!({
                        "t":start.elapsed().as_secs_f64(), "frame":scene.frame_id,
                        "detect_ms":scene.detect_ms,"pose_ms":scene.pose_ms,
                        "age_ms":scene.time.map(|t| t.elapsed().as_millis()),
                        "focus":scene.focus_id,"epoch":scene.focus_epoch,"people":people,
                        "aim":aim,"actual":st.eye.as_ref().and_then(|s|s.actual),
                        "blink":st.eye.as_ref().map(|s|s.parpadeando),
                        "device":st.eye.as_ref().and_then(|s|s.diag.as_ref()),
                        "status":st.status,"error":st.eye_error,
                        "control_pause":st.control_pause,"sent_command":st.last_command,
                        "command_age_ms":st.command_at.map(|t|t.elapsed().as_millis()),
                        "telemetry_age_ms":st.eye.as_ref().and_then(|s|s.sampled_at).map(|t|t.elapsed().as_millis()),
                        "lid_target":st.eye.as_ref().and_then(|s|s.parpado_seguimiento)
                    }),
                )?;
                file.write_all(b"\n")?;
                file.flush()?;
                previous = Some(scene);
            }
            Ok(())
        })();
        if let Err(e) = result {
            log::warn!("Registro de diagnóstico: {e:#}");
        }
    }))
}
