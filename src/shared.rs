//! Datos compartidos entre hilos: el último fotograma de cámara, la última escena analizada y
//! el último fotograma de proyección. Cada productor publica un `Arc` nuevo; los consumidores
//! se quedan con el más reciente (nunca se acumula retraso).

use parking_lot::{Condvar, Mutex};
use std::sync::Arc;
use std::time::Instant;

use crate::config::Config;

/// Imagen RGB8 empaquetada.
#[derive(Clone)]
pub struct Frame {
    pub id: u64,
    pub width: u32,
    pub height: u32,
    pub rgb: Vec<u8>,
    pub time: Instant,
}

/// Imagen RGBA8 (proyección).
#[derive(Clone)]
pub struct Canvas {
    pub id: u64,
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
}

pub const NUM_KPTS: usize = 17;

/// Punto del esqueleto en coordenadas de la imagen de cámara (píxeles) y confianza 0..1.
#[derive(Clone, Copy, Debug, Default)]
pub struct Kpt {
    pub x: f32,
    pub y: f32,
    pub c: f32,
}

#[derive(Clone, Debug)]
pub struct Person {
    pub visible: bool,
    pub id: u64,
    pub bbox: [f32; 4],
    pub score: f32,
    pub kpts: Option<[Kpt; NUM_KPTS]>,
    pub focus: bool,
    pub quality: f32,
    pub age: u32,
}

#[derive(Clone, Debug, Default)]
pub struct Scene {
    pub frame_id: u64,
    pub focus_epoch: u64,
    pub width: u32,
    pub height: u32,
    pub people: Vec<Person>,
    pub focus_id: Option<u64>,
    pub detect_ms: f32,
    pub pose_ms: f32,
    pub time: Option<Instant>,
}

/// Ranura con el último valor y aviso a quien espera.
pub struct Slot<T> {
    val: Mutex<Option<Arc<T>>>,
    cv: Condvar,
}

impl<T> Default for Slot<T> {
    fn default() -> Self {
        Self {
            val: Mutex::new(None),
            cv: Condvar::new(),
        }
    }
}

impl<T> Slot<T> {
    pub fn put(&self, v: T) {
        *self.val.lock() = Some(Arc::new(v));
        self.cv.notify_all();
    }
    pub fn get(&self) -> Option<Arc<T>> {
        self.val.lock().clone()
    }
    /// Espera un valor distinto de `after` (por puntero) como mucho `timeout`.
    pub fn wait_new(&self, after: Option<&Arc<T>>, timeout: std::time::Duration) -> Option<Arc<T>> {
        let mut g = self.val.lock();
        let is_new = |v: &Option<Arc<T>>| match (v, after) {
            (Some(n), Some(a)) => !Arc::ptr_eq(n, a),
            (Some(_), None) => true,
            _ => false,
        };
        if !is_new(&g) {
            self.cv.wait_for(&mut g, timeout);
        }
        if is_new(&g) { g.clone() } else { None }
    }
}

/// FPS over an elapsed-time window; USB bursts must not inflate this number.
#[derive(Default)]
pub struct Rate {
    start: Option<Instant>,
    frames: u32,
    pub fps: f32,
}
impl Rate {
    pub fn tick(&mut self) {
        self.tick_at(Instant::now());
    }
    fn tick_at(&mut self, now: Instant) {
        let start = *self.start.get_or_insert(now);
        self.frames += 1;
        let elapsed = now.duration_since(start).as_secs_f32();
        if elapsed >= 1. {
            self.fps = (self.frames - 1) as f32 / elapsed;
            self.frames = 1;
            self.start = Some(now);
        }
    }
}
#[cfg(test)]
mod rate_tests {
    use super::*;
    #[test]
    fn bursts_count_frames_over_time_not_inverse_intervals() {
        let mut rate = Rate::default();
        let t = Instant::now();
        for ms in [0, 1, 2, 500, 501, 502, 1000] {
            rate.tick_at(t + std::time::Duration::from_millis(ms));
        }
        assert_eq!(rate.fps, 6.);
    }
}

#[derive(Default, Clone)]
pub struct Stats {
    pub camera_fps: f32,
    pub vision_fps: f32,
    pub camera_error: Option<String>,
    pub vision_error: Option<String>,
    pub pose_device: String,
    pub eye_error: Option<String>,
    pub eye: Option<crate::eye::EyeState>,
    pub eye_at: Option<Instant>,
    pub aim: Option<crate::geometry::Aim>,
    pub status: String,
    pub calibration_message: String,
    pub control_pause: String,
    pub last_command: Option<[f32; 3]>,
    pub command_at: Option<Instant>,
}
pub struct Shared {
    pub config: Mutex<Config>,
    pub camera: Slot<Frame>,
    pub scene: Slot<Scene>,
    pub stats: Mutex<Stats>,
    pub quit: std::sync::atomic::AtomicBool,
    pub tracking: std::sync::atomic::AtomicBool,
    pub manual: std::sync::atomic::AtomicBool,
    pub manual_input: Mutex<crate::manual::ManualInput>,
    pub limits_request: Mutex<Option<crate::motion_limits::MotionLimits>>,
    pub gesture: Mutex<Option<String>>,
    pub focus_epoch: std::sync::atomic::AtomicU64,
    pub focus_paused: std::sync::atomic::AtomicBool,
}
impl Shared {
    pub fn new(config: Config) -> Arc<Self> {
        Arc::new(Self {
            config: Mutex::new(config),
            camera: Slot::default(),
            scene: Slot::default(),
            stats: Mutex::new(Stats::default()),
            quit: false.into(),
            tracking: false.into(),
            manual: false.into(),
            manual_input: Mutex::new(Default::default()),
            limits_request: Mutex::new(None),
            gesture: Mutex::new(None),
            focus_epoch: 0.into(),
            focus_paused: false.into(),
        })
    }
    pub fn cfg(&self) -> Config {
        self.config.lock().clone()
    }
    pub fn quitting(&self) -> bool {
        self.quit.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn tracking(&self) -> bool {
        self.tracking.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn manual(&self) -> bool {
        self.manual.load(std::sync::atomic::Ordering::Relaxed)
    }
    pub fn driving(&self) -> bool {
        self.manual() || self.tracking()
    }
    pub fn set_manual(&self, on: bool) {
        self.set_tracking(false);
        *self.manual_input.lock() = Default::default();
        self.manual.store(on, std::sync::atomic::Ordering::Relaxed);
    }
    pub fn stop(&self) {
        self.set_manual(false);
        self.set_tracking(false);
    }
    pub fn set_tracking(&self, on: bool) {
        if on {
            self.manual
                .store(false, std::sync::atomic::Ordering::Relaxed);
        }
        self.tracking
            .store(on, std::sync::atomic::Ordering::Relaxed);
    }
}
