//! YOLOX y RTMPose: preprocesado y decodificación como en MMDetection/MMPose.
use crate::config::{Config, VisionConfig};
use crate::shared::{Frame, Kpt, NUM_KPTS, Rate, Scene, Shared};
use crate::tracker::Tracker;
use anyhow::{Context, Result};
use ndarray::Array4;
use ort::session::Session;
use ort::value::TensorRef;
use std::sync::Arc;
use std::time::{Duration, Instant};
#[derive(Clone, Copy, Debug, Default)]
pub struct Detection {
    pub bbox: [f32; 4],
    pub score: f32,
    pub track: Option<u64>,
}
fn environment() -> Result<&'static ort::environment::Environment> {
    static ENV: std::sync::OnceLock<std::result::Result<ort::environment::Environment, String>> =
        std::sync::OnceLock::new();
    ENV.get_or_init(|| {
        ort::init()
            .with_name("ztrak-ojo")
            .build()
            .map_err(|e| e.to_string())
    })
    .as_ref()
    .map_err(|e| anyhow::anyhow!("ONNX Runtime: {e}"))
}
/// Windows: only the pose model goes to the graphics card (DirectML, as in ZTrak on the
/// Arc A310). Fixed batch of 1: the variable one was about twice as slow there.
#[cfg(windows)]
fn directml_session(env: &ort::environment::Environment, path: &str, threads: usize) -> Result<Session> {
    let mut b = Session::builder(env).map_err(|e| anyhow::anyhow!("{e}"))?
        .with_intra_threads(threads).map_err(|e| anyhow::anyhow!("{e}"))?
        .with_parallel_execution(false).map_err(|e| anyhow::anyhow!("{e}"))?
        .with_memory_pattern(false).map_err(|e| anyhow::anyhow!("{e}"))?
        .with_dimension_override("batch", 1).map_err(|e| anyhow::anyhow!("{e}"))?;
    let ep = ort::ep::DirectML::default()
        .with_performance_preference(ort::ep::directml::PerformancePreference::HighPerformance);
    b = b.with_execution_providers([ep.build().error_on_failure()]).map_err(|e| anyhow::anyhow!("{e}"))?;
    b.commit_from_file(Config::resolve(path)).map_err(|e| anyhow::anyhow!("{e}"))
}
/// Pose session, its device label and whether it needs one crop per inference.
fn pose_session(env: &ort::environment::Environment, c: &VisionConfig) -> Result<(Session, &'static str, bool)> {
    #[cfg(windows)]
    if c.device != "cpu" {
        match directml_session(env, &c.pose_model, c.threads) {
            Ok(s) => {
                log::info!("Puntos de cara y cuerpo: gráfica (DirectML)");
                return Ok((s, "gráfica (DirectML)", true));
            }
            Err(e) => log::warn!("DirectML no disponible, se usa CPU: {e:#}"),
        }
    }
    Ok((session(env, &c.pose_model, c.threads)?, "CPU", false))
}
/// Valid model output: no NaN or infinity, and not entirely zero.
fn valid_output<'a>(values: impl Iterator<Item = &'a f32>) -> bool {
    let mut nonzero = false;
    for &v in values {
        if !v.is_finite() {
            return false;
        }
        nonzero |= v != 0.;
    }
    nonzero
}
fn note_invalid_output() {
    static N: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = N.fetch_add(1, std::sync::atomic::Ordering::Relaxed) + 1;
    if n == 1 || n % 200 == 0 {
        log::warn!("La IA devolvió una salida inválida (ceros o NaN); se repite ({n} veces en total)");
    }
}
fn session(env: &ort::environment::Environment, path: &str, threads: usize) -> Result<Session> {
    let b = Session::builder(env).map_err(|e| anyhow::anyhow!("{e}"))?;
    let mut b = b
        .with_intra_threads(threads)
        .map_err(|e| anyhow::anyhow!("{e}"))?
        .with_config_entry("session.intra_op.allow_spinning", "0")
        .map_err(|e| anyhow::anyhow!("{e}"))?;
    b.commit_from_file(Config::resolve(path))
        .map_err(|e| anyhow::anyhow!("{e}"))
}
/// Muestreo bilineal de un canal de una imagen RGB8.
#[inline]
fn bilinear(img: &Frame, x: f32, y: f32, out: &mut [f32; 3]) {
    let w = img.width as i32;
    let h = img.height as i32;
    if x < -0.5 || y < -0.5 || x > w as f32 - 0.5 || y > h as f32 - 0.5 {
        *out = [f32::NAN; 3];
        return;
    }
    let x = x.clamp(0.0, (w - 1) as f32);
    let y = y.clamp(0.0, (h - 1) as f32);
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let x1 = (x0 + 1).min(w - 1);
    let y1 = (y0 + 1).min(h - 1);
    let fx = x - x0 as f32;
    let fy = y - y0 as f32;
    let px = |xx: i32, yy: i32, c: usize| img.rgb[((yy * w + xx) * 3) as usize + c] as f32;
    for c in 0..3 {
        let a = px(x0, y0, c) * (1.0 - fx) + px(x1, y0, c) * fx;
        let b = px(x0, y1, c) * (1.0 - fx) + px(x1, y1, c) * fx;
        out[c] = a * (1.0 - fy) + b * fy;
    }
}

pub struct Detector {
    sess: Session,
    size: usize,
    pub threshold: f32,
}

impl Detector {
    pub fn new(env: &ort::environment::Environment, c: &VisionConfig) -> Result<Self> {
        // Reservar parte de la CPU para la localización de cabeza y la interfaz.
        let threads = std::env::var("ZTRAK_OJO_DET_HILOS")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or((c.threads / 2).max(1));
        Ok(Self {
            sess: session(env, &c.detector_model, threads)?,
            size: c.detector_size as usize,
            threshold: c.detector_threshold,
        })
    }

    pub fn detect(&mut self, img: &Frame) -> Result<Vec<Detection>> {
        let s = self.size;
        let k = s as f32 / img.width.max(img.height) as f32;
        let nw = ((img.width as f32 * k) as usize).min(s);
        let nh = ((img.height as f32 * k) as usize).min(s);
        // YOLOX de mmdet: BGR, 0..255, relleno 114, escala manteniendo proporción (arriba-izquierda)
        let mut t = Array4::<f32>::from_elem((1, 3, s, s), 114.0);
        let mut px = [0f32; 3];
        for y in 0..nh {
            let sy = (y as f32 + 0.5) / k - 0.5;
            for x in 0..nw {
                bilinear(img, (x as f32 + 0.5) / k - 0.5, sy, &mut px);
                t[[0, 0, y, x]] = px[2];
                t[[0, 1, y, x]] = px[1];
                t[[0, 2, y, x]] = px[0];
            }
        }
        let out = self
            .sess
            .run(ort::inputs!["input" => TensorRef::from_array_view(&t)?])?;
        let dets = out["dets"].try_extract_array::<f32>()?;
        let labels = out["labels"].try_extract_array::<i64>().ok();
        let n = dets.shape()[1];
        let mut v = Vec::new();
        for i in 0..n {
            let score = dets[[0, i, 4]];
            if !score.is_finite() || score < self.threshold {
                continue;
            }
            if let Some(l) = &labels {
                if l[[0, i]] != 0 {
                    continue;
                }
            }
            let b = [
                dets[[0, i, 0]] / k,
                dets[[0, i, 1]] / k,
                dets[[0, i, 2]] / k,
                dets[[0, i, 3]] / k,
            ];
            let b = [
                b[0].max(0.0),
                b[1].max(0.0),
                b[2].min(img.width as f32),
                b[3].min(img.height as f32),
            ];
            if !b.iter().all(|v| v.is_finite()) || b[2] - b[0] < 8.0 || b[3] - b[1] < 8.0 {
                continue;
            }
            v.push(Detection {
                bbox: b,
                score,
                track: None,
            });
        }
        Ok(v)
    }
}

/// YuNet confirms actual faces; body pose can confidently infer an occluded head.
/// Decoder follows OpenCV FaceDetectorYN (model distributed under MIT).
struct FaceDetector {
    sess: Session,
}
#[derive(Clone, Copy)]
struct Face {
    bbox: [f32; 4],
    score: f32,
    points: [[f32; 2]; 3],
}
impl FaceDetector {
    fn new(env: &ort::environment::Environment, cfg: &VisionConfig) -> Result<Self> {
        Ok(Self {
            sess: session(env, &cfg.face_model, 2)?,
        })
    }
    fn detect(&mut self, img: &Frame, person: [f32; 4]) -> Result<Vec<Face>> {
        let size = 320usize;
        let bw = person[2] - person[0];
        let bh = person[3] - person[1];
        let x0 = (person[0] - bw * 0.1).max(0.);
        let y0 = (person[1] - bh * 0.1).max(0.);
        let cw = (person[2] + bw * 0.1).min(img.width as f32) - x0;
        let ch = (person[1] + bh * 0.7).min(img.height as f32) - y0;
        if cw <= 1. || ch <= 1. {
            return Ok(vec![]);
        }
        let scale = size as f32 / cw.max(ch);
        let mut input = Array4::<f32>::zeros((1, 3, size, size));
        let mut px = [0.; 3];
        for y in 0..(ch * scale) as usize {
            for x in 0..(cw * scale) as usize {
                bilinear(
                    img,
                    x0 + (x as f32 + 0.5) / scale - 0.5,
                    y0 + (y as f32 + 0.5) / scale - 0.5,
                    &mut px,
                );
                for c in 0..3 {
                    input[[0, c, y, x]] = px[2 - c];
                }
            }
        }
        let output = self
            .sess
            .run(ort::inputs!["input"=>TensorRef::from_array_view(&input)?])?;
        let mut faces = vec![];
        for stride in [8usize, 16, 32] {
            let cls = output[format!("cls_{stride}")].try_extract_array::<f32>()?;
            let obj = output[format!("obj_{stride}")].try_extract_array::<f32>()?;
            let bbox = output[format!("bbox_{stride}")].try_extract_array::<f32>()?;
            let kps = output[format!("kps_{stride}")].try_extract_array::<f32>()?;
            let cols = size / stride;
            for i in 0..cols * cols {
                let score = (cls[[0, i, 0]].clamp(0., 1.) * obj[[0, i, 0]].clamp(0., 1.)).sqrt();
                if !score.is_finite() || score < 0.6 {
                    continue;
                }
                let (x, y) = ((i % cols) as f32, (i / cols) as f32);
                let s = stride as f32 / scale;
                let (cx, cy) = (
                    x0 + (x + bbox[[0, i, 0]]) * s,
                    y0 + (y + bbox[[0, i, 1]]) * s,
                );
                let (w, h) = (bbox[[0, i, 2]].exp() * s, bbox[[0, i, 3]].exp() * s);
                let mut points = [[0.; 2]; 3];
                for j in 0..3 {
                    points[j] = [
                        x0 + (x + kps[[0, i, 2 * j]]) * s,
                        y0 + (y + kps[[0, i, 2 * j + 1]]) * s,
                    ];
                }
                faces.push(Face {
                    bbox: [cx - w / 2., cy - h / 2., cx + w / 2., cy + h / 2.],
                    score,
                    points,
                });
            }
        }
        faces.sort_by(|a, b| b.score.total_cmp(&a.score));
        let mut kept: Vec<Face> = vec![];
        for f in faces {
            if kept
                .iter()
                .all(|g| crate::tracker::iou(&f.bbox, &g.bbox) < 0.3)
            {
                kept.push(f);
            }
        }
        Ok(kept)
    }
}
fn confirmed_head(k: &mut [Kpt; NUM_KPTS], person: [f32; 4], faces: &[Face]) -> bool {
    // Do not retain RTMPose's inferred eyes/nose on a hidden face or on the back
    // of a head. Shoulders/body landmarks remain available for depth and identity.
    for point in &mut k[..5] {
        *point = Kpt::default();
    }
    let best = faces
        .iter()
        .filter(|f| {
            let x = (f.bbox[0] + f.bbox[2]) * 0.5;
            let y = (f.bbox[1] + f.bbox[3]) * 0.5;
            x >= person[0]
                && x <= person[2]
                && y >= person[1]
                && y <= person[1] + 0.65 * (person[3] - person[1])
        })
        .max_by(|a, b| a.score.total_cmp(&b.score));
    if let Some(f) = best {
        for (dst, src) in [(0, 2), (1, 1), (2, 0)] {
            k[dst] = Kpt {
                x: f.points[src][0],
                y: f.points[src][1],
                c: f.score,
            };
        }
        return true;
    }
    false
}

pub struct PoseEstimator {
    pub device: &'static str,
    single: bool,
    sess: Session,
    w: usize,
    h: usize,
}

const MEAN: [f32; 3] = [123.675, 116.28, 103.53];
const STD: [f32; 3] = [58.395, 57.12, 57.375];
const SIMCC_SPLIT: f32 = 2.0;

impl PoseEstimator {
    pub fn new(env: &ort::environment::Environment, c: &VisionConfig) -> Result<Self> {
        let (sess, device, single) = pose_session(env, c)?;
        Ok(Self {
            sess,
            device,
            single,
            w: c.pose_width as usize,
            h: c.pose_height as usize,
        })
    }

    /// Esqueletos de varias cajas: en lote, o de una en una con DirectML.
    pub fn estimate(&mut self, img: &Frame, boxes: &[[f32; 4]]) -> Result<Vec<[Kpt; NUM_KPTS]>> {
        if !self.single {
            return self.estimate_batch(img, boxes);
        }
        let mut poses = Vec::with_capacity(boxes.len());
        for b in boxes {
            poses.extend(self.estimate_batch(img, std::slice::from_ref(b))?);
        }
        Ok(poses)
    }

    fn estimate_batch(&mut self, img: &Frame, boxes: &[[f32; 4]]) -> Result<Vec<[Kpt; NUM_KPTS]>> {
        if boxes.is_empty() {
            return Ok(vec![]);
        }
        let (w, h) = (self.w, self.h);
        let aspect = w as f32 / h as f32;
        let mut t = Array4::<f32>::zeros((boxes.len(), 3, h, w));
        let mut crops = Vec::with_capacity(boxes.len());
        let mut px = [0f32; 3];
        for (bi, b) in boxes.iter().enumerate() {
            // centro y escala con margen 1,25 y proporción del modelo (TopDownGetBboxCenterScale)
            let cx = (b[0] + b[2]) / 2.0;
            let cy = (b[1] + b[3]) / 2.0;
            let mut sw = (b[2] - b[0]) * 1.25;
            let mut sh = (b[3] - b[1]) * 1.25;
            if sw / sh > aspect {
                sh = sw / aspect
            } else {
                sw = sh * aspect
            }
            let x0 = cx - sw / 2.0;
            let y0 = cy - sh / 2.0;
            crops.push((x0, y0, sw, sh));
            for v in 0..h {
                let sy = y0 + (v as f32 + 0.5) * sh / h as f32 - 0.5;
                for u in 0..w {
                    bilinear(
                        img,
                        x0 + (u as f32 + 0.5) * sw / w as f32 - 0.5,
                        sy,
                        &mut px,
                    );
                    for c in 0..3 {
                        let val = if px[c].is_nan() {
                            0.0
                        } else {
                            (px[c] - MEAN[c]) / STD[c]
                        };
                        t[[bi, c, v, u]] = val;
                    }
                }
            }
        }
        // An invalid output is repeated and never reaches tracking: after three
        // tries this frame has zero-confidence points, which the tracker ignores.
        let mut attempt = 0;
        let (sx, sy) = loop {
            let out = self
                .sess
                .run(ort::inputs!["input" => TensorRef::from_array_view(&t)?])?;
            let sx = out["simcc_x"].try_extract_array::<f32>()?.to_owned();
            let sy = out["simcc_y"].try_extract_array::<f32>()?.to_owned();
            if valid_output(sx.iter().chain(sy.iter())) {
                break (sx, sy);
            }
            note_invalid_output();
            attempt += 1;
            if attempt >= 3 {
                return Ok(vec![[Kpt::default(); NUM_KPTS]; boxes.len()]);
            }
        };
        let (nx, ny) = (sx.shape()[2], sy.shape()[2]);
        let mut res = Vec::with_capacity(boxes.len());
        for (bi, &(x0, y0, sw, sh)) in crops.iter().enumerate() {
            let mut k = [Kpt::default(); NUM_KPTS];
            for j in 0..NUM_KPTS {
                let (mut bx, mut vx) = (0usize, f32::MIN);
                for i in 0..nx {
                    let val = sx[[bi, j, i]];
                    if val > vx {
                        vx = val;
                        bx = i;
                    }
                }
                let (mut by, mut vy) = (0usize, f32::MIN);
                for i in 0..ny {
                    let val = sy[[bi, j, i]];
                    if val > vy {
                        vy = val;
                        by = i;
                    }
                }
                let px = bx as f32 / SIMCC_SPLIT;
                let py = by as f32 / SIMCC_SPLIT;
                k[j] = Kpt {
                    x: x0 + px * sw / w as f32,
                    y: y0 + py * sh / h as f32,
                    c: vx.min(vy).clamp(0.0, 1.0),
                };
            }
            res.push(k);
        }
        Ok(res)
    }
}

struct DetectedFrame {
    frame: Arc<Frame>,
    boxes: Vec<Detection>,
    ms: f32,
}

pub fn spawn(shared: Arc<Shared>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        while !shared.quitting() {
            let cfg = shared.cfg();
            let key = cfg.vision.clone();
            let result = (|| -> Result<()> {
                let env = environment()?;
                let mut detector = Detector::new(env, &key).context("Detector de personas")?;
                let mut pose = PoseEstimator::new(env, &key).context("Puntos de cara y cuerpo")?;
                let mut face_detector =
                    FaceDetector::new(env, &key).context("Confirmación de caras")?;
                let detections =
                    crate::shared::Slot::<std::result::Result<DetectedFrame, String>>::default();
                let stop = std::sync::atomic::AtomicBool::new(false);
                let mut tracker = Tracker::new(cfg.focus);
                {
                    let mut st = shared.stats.lock();
                    st.vision_error = None;
                    st.pose_device = pose.device.to_string();
                }
                std::thread::scope(|scope| {
                    // Detection and head landmarks overlap. There is only one
                    // latest result, never a queue of old camera frames.
                    let detector_thread = scope.spawn(|| {
                        let mut last = None;
                        while !stop.load(std::sync::atomic::Ordering::Relaxed)
                            && !shared.quitting()
                            && shared.cfg().vision == key
                        {
                            let Some(frame) = shared
                                .camera
                                .wait_new(last.as_ref(), Duration::from_millis(100))
                            else {
                                continue;
                            };
                            last = Some(frame.clone());
                            if frame.time.elapsed() > Duration::from_millis(200) {
                                continue;
                            }
                            let start = Instant::now();
                            match detector.detect(&frame) {
                                Ok(boxes) => detections.put(Ok(DetectedFrame {
                                    frame,
                                    boxes,
                                    ms: start.elapsed().as_secs_f32() * 1000.,
                                })),
                                Err(e) => {
                                    detections.put(Err(format!("Detector: {e:#}")));
                                    break;
                                }
                            }
                        }
                    });
                    let result = (|| -> Result<()> {
                        let mut last = None;
                        let mut last_pose_frame = None;
                        let mut last_detection = None;
                        let mut focus_epoch = shared
                            .focus_epoch
                            .load(std::sync::atomic::Ordering::Relaxed);
                        let mut rate = Rate::default();
                        let mut confirmed_at = std::collections::HashMap::<u64, Instant>::new();
                        while !shared.quitting() && shared.cfg().vision == key {
                            // Like ZTrak: each camera exposure drives pose tracking.
                            // The detector supplies asynchronous confirmations, never a gate.
                            let Some(frame) = shared
                                .camera
                                .wait_new(last.as_ref(), Duration::from_millis(100))
                            else {
                                continue;
                            };
                            last = Some(frame.clone());
                            if frame.time.elapsed() > Duration::from_millis(200) {
                                continue;
                            }
                            let latest = detections.get();
                            let d = latest
                                .as_ref()
                                .map(|d| d.as_ref().as_ref().map_err(|e| anyhow::anyhow!("{e}")))
                                .transpose()?;
                            if last_pose_frame.is_some_and(|t| frame.time <= t) {
                                continue;
                            }
                            last_pose_frame = Some(frame.time);
                            let now = Instant::now();
                            let epoch = shared
                                .focus_epoch
                                .load(std::sync::atomic::Ordering::Relaxed);
                            if epoch != focus_epoch {
                                tracker.tracks.clear();
                                confirmed_at.clear();
                                tracker.focus = None;
                                focus_epoch = epoch;
                            }
                            let paused = shared
                                .focus_paused
                                .load(std::sync::atomic::Ordering::Relaxed);
                            // Keep perception alive during lid gestures. A visible face
                            // remains useful; a genuinely hidden camera produces no valid
                            // face naturally. Motor holds are handled by the controller.
                            let blinking = shared
                                .stats
                                .lock()
                                .eye
                                .as_ref()
                                .is_some_and(|e| e.parpadeando);
                            tracker.set_blink(blinking, frame.time);
                            tracker.cfg = shared.cfg().focus;
                            tracker.cfg.steal_ratio = 0.;
                            let mut observations = tracker
                                .boxes_from_pose(
                                    key.keypoint_threshold,
                                    frame.width,
                                    frame.height,
                                    frame.time,
                                )
                                .unwrap_or_default();
                            if let Some(d) = d.filter(|d| {
                                Some(d.frame.id) != last_detection
                                    && frame.time.saturating_duration_since(d.frame.time)
                                        < Duration::from_millis(200)
                            }) {
                                last_detection = Some(d.frame.id);
                                let covered = observations
                                    .iter()
                                    .filter_map(|b| b.track)
                                    .collect::<Vec<_>>();
                                let (rest, hints) =
                                    tracker.confirm(&d.boxes, &covered, d.frame.time);
                                for (id, hint) in hints {
                                    if let Some(b) =
                                        observations.iter_mut().find(|b| b.track == Some(id))
                                    {
                                        b.bbox = [
                                            b.bbox[0].min(hint[0]),
                                            b.bbox[1].min(hint[1]),
                                            b.bbox[2].max(hint[2]),
                                            b.bbox[3].max(hint[3]),
                                        ];
                                    }
                                }
                                observations.extend(rest);
                            }
                            tracker.update(&observations, frame.width, frame.height, frame.time);
                            if paused {
                                tracker.focus = None;
                            } else {
                                tracker.choose_focus(frame.width, frame.height, now);
                            }
                            let budget = if tracker.focus.is_some() {
                                1
                            } else {
                                key.max_poses
                            };
                            let indices = if paused {
                                vec![]
                            } else {
                                tracker.head_order(budget, frame.time)
                            };
                            let boxes: Vec<_> =
                                indices.iter().map(|&i| tracker.tracks[i].bbox).collect();
                            let start = Instant::now();
                            let (poses, faces) = std::thread::scope(|scope| -> Result<_> {
                                let faces = scope.spawn(|| -> Result<_> {
                                    if indices.is_empty() {
                                        Ok(vec![])
                                    } else {
                                        {
                                            let mut all = vec![];
                                            for &b in &boxes {
                                                all.extend(face_detector.detect(&frame, b)?);
                                            }
                                            Ok(all)
                                        }
                                    }
                                });
                                let poses = pose.estimate(&frame, &boxes)?;
                                Ok((
                                    poses,
                                    faces.join().map_err(|_| {
                                        anyhow::anyhow!("Detector de caras interrumpido")
                                    })??,
                                ))
                            })?;
                            for (i, mut k) in indices.into_iter().zip(poses) {
                                let inferred = k;
                                let id = tracker.tracks[i].id;
                                if confirmed_head(&mut k, tracker.tracks[i].bbox, &faces) {
                                    confirmed_at.insert(id, frame.time);
                                } else if confirmed_at.get(&id).is_some_and(|t| {
                                    frame.time.saturating_duration_since(*t)
                                        < Duration::from_millis(350)
                                }) {
                                    // Short detector flicker: keep following fresh head points,
                                    // never reuse the old pixel location on a moving camera.
                                    for j in 0..5 {
                                        k[j] = inferred[j];
                                    }
                                }
                                confirmed_at.retain(|_, t| {
                                    frame.time.saturating_duration_since(*t)
                                        < Duration::from_secs(2)
                                });
                                tracker.confirm_fresh_head(
                                    i,
                                    &k,
                                    frame.width,
                                    frame.height,
                                    frame.time,
                                );
                                tracker.tracks[i].set_kpts(k, key.keypoint_threshold, frame.time);
                            }
                            if paused {
                                tracker.focus = None;
                            } else {
                                tracker.choose_focus(frame.width, frame.height, now);
                            }
                            rate.tick();
                            shared.scene.put(Scene {
                                frame_id: frame.id,
                                focus_epoch,
                                width: frame.width,
                                height: frame.height,
                                people: tracker.people(now),
                                focus_id: tracker.focus,
                                detect_ms: d.map(|d| d.ms).unwrap_or(0.),
                                pose_ms: start.elapsed().as_secs_f32() * 1000.,
                                time: Some(frame.time),
                            });
                            shared.stats.lock().vision_fps = rate.fps;
                        }
                        Ok(())
                    })();
                    stop.store(true, std::sync::atomic::Ordering::Relaxed);
                    detector_thread
                        .join()
                        .map_err(|_| anyhow::anyhow!("El detector terminó inesperadamente"))?;
                    result
                })
            })();
            if let Err(e) = result {
                shared.stats.lock().vision_error = Some(format!("{e:#}"));
            }
            for _ in 0..20 {
                if shared.quitting() || shared.cfg().vision != key {
                    break;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    })
}

#[cfg(test)]
mod face_tests {
    use super::*;
    #[test]
    fn inferred_head_without_face_confirmation_is_removed() {
        let mut k = [Kpt {
            x: 100.,
            y: 150.,
            c: 0.99,
        }; NUM_KPTS];
        confirmed_head(&mut k, [50., 50., 300., 600.], &[]);
        assert!(k[..5].iter().all(|p| p.c == 0.));
        assert_eq!(k[5].c, 0.99);
    }
    #[test]
    fn unrelated_face_cannot_confirm_the_tracked_body() {
        let mut k = [Kpt::default(); NUM_KPTS];
        let face = Face {
            bbox: [600., 50., 700., 150.],
            score: 0.95,
            points: [[620., 80.], [680., 80.], [650., 100.]],
        };
        confirmed_head(&mut k, [50., 50., 300., 600.], &[face]);
        assert_eq!(k[0].c, 0.);
        confirmed_head(&mut k, [550., 30., 750., 600.], &[face]);
        assert_eq!([k[0].x, k[0].y], [650., 100.]);
        assert_eq!(k[1].c, 0.95);
    }
}

#[cfg(test)]
mod output_tests {
    use super::valid_output;
    #[test]
    fn rejects_nan_infinity_and_all_zero() {
        assert!(valid_output([0., 0.5].iter()));
        assert!(!valid_output([0., 0.].iter()));
        assert!(!valid_output([1., f32::NAN].iter()));
        assert!(!valid_output([f32::INFINITY, 1.].iter()));
    }
}
