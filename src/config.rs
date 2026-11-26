use serde::{Deserialize, Serialize};
use std::path::PathBuf;

const WINDOWS_FIRST_RUN: &str = include_str!("../config/windows-inicial.json");

#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct Config {
    pub camera: CameraConfig,
    pub vision: VisionConfig,
    pub focus: FocusConfig,
    pub mount: MountConfig,
    pub eye_address: String,
    pub uart: bool,
    pub uart_port: String,
    pub uart_baud: u32,
    pub sleep_enabled: bool,
    pub sleep_after_s: f32,
    pub sleep_for_s: f32,
    pub reset_pause_s: f32,
    /// Presentation only. Never changes the camera coordinates used for aiming.
    pub mirror_preview: bool,
    pub curiosity: bool,
    pub boredom_enabled: bool,
    pub boredom_after_s: f32,
    pub droop_s: f32,
    pub sleep_slit: f32,
    pub wake_motion_fraction: f32,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            camera: CameraConfig::default(),
            vision: VisionConfig::default(),
            focus: FocusConfig::default(),
            mount: MountConfig::default(),
            eye_address: "192.168.1.121:80".into(),
            uart: false,
            uart_baud: 230400,
            uart_port: if cfg!(windows) {
                "COM3"
            } else {
                "/dev/ttyUSB0"
            }
            .into(),
            sleep_enabled: true,
            sleep_after_s: 12.,
            sleep_for_s: 5.,
            reset_pause_s: 2.,
            mirror_preview: false,
            curiosity: true,
            boredom_enabled: true,
            boredom_after_s: 10.,
            droop_s: 1.4,
            sleep_slit: 32.,
            wake_motion_fraction: 0.03,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct CameraConfig {
    pub source: String,
    pub width: u32,
    pub height: u32,
    pub fps: u32,
    pub input_format: String,
    /// Correct an upside-down sensor before inference (not a display mirror).
    pub rotate_180: bool,
}
impl Default for CameraConfig {
    fn default() -> Self {
        Self {
            // Windows: empty until the eye's webcam is chosen in the selector.
            source: if cfg!(windows) {
                ""
            } else {
                "/dev/video0"
            }
            .into(),
            width: 1280,
            height: 720,
            fps: 30,
            input_format: "mjpeg".into(),
            rotate_180: false,
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct VisionConfig {
    pub detector_model: String,
    pub pose_model: String,
    pub face_model: String,
    pub detector_size: u32,
    pub pose_width: u32,
    pub pose_height: u32,
    pub detector_threshold: f32,
    pub keypoint_threshold: f32,
    pub threads: usize,
    pub max_poses: usize,
    /// Windows pose model: "auto" (graphics card through DirectML) or "cpu".
    pub device: String,
}
impl Default for VisionConfig {
    fn default() -> Self {
        Self {
            detector_model: "models/yolox_nano_humanart_416.onnx".into(),
            pose_model: "models/rtmpose-m_body7_256x192.onnx".into(),
            face_model: "models/face_detection_yunet_2026may.onnx".into(),
            detector_size: 416,
            pose_width: 192,
            pose_height: 256,
            detector_threshold: 0.35,
            keypoint_threshold: 0.35,
            threads: 4,
            max_poses: 3,
            device: "auto".into(),
        }
    }
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct FocusConfig {
    pub lost_timeout: f32,
    pub steal_ratio: f32,
    pub w_size: f32,
    pub w_center: f32,
    pub w_keypoints: f32,
    pub min_hits: u32,
    /// Seconds to look at one still person before moving to another (min, max).
    pub attention_s: [f32; 2],
    /// Relative image speed (body heights/s) that counts as moving.
    pub motion_threshold: f32,
}
impl Default for FocusConfig {
    fn default() -> Self {
        Self {
            lost_timeout: 0.8,
            steal_ratio: 0.0,
            w_size: 1.0,
            w_center: 0.4,
            w_keypoints: 0.8,
            min_hits: 3,
            attention_s: [2., 4.],
            motion_threshold: 0.35,
        }
    }
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum MountMode {
    #[default]
    External,
    Pupil,
}
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq, Default)]
pub enum DepthMode {
    #[default]
    Manual,
    Shoulders,
}
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
#[serde(default)]
pub struct MountConfig {
    pub limits: crate::motion_limits::MotionLimits,
    pub mode: MountMode,
    /// Camera position relative to eye rest frame: right, up, forward (metres).
    pub camera_xyz: [f32; 3],
    /// Camera orientation relative to eye rest frame (pupil: relative to gaze).
    pub camera_ypr: [f32; 3],
    pub horizontal_fov: f32,
    pub depth_mode: DepthMode,
    /// Optical depth from camera, not radial distance from the eye.
    pub depth_m: f32,
    pub shoulder_width_m: f32,
    pub trim: [f32; 2],
    /// Commanded degrees per degree of image motion (measured): the linkage
    /// turns the camera less than the servo command says.
    pub pupil_scale: [f32; 2],
    /// Mechanical play in commanded degrees, compensated on direction reversal.
    pub backlash_deg: [f32; 2],
    pub invert: [bool; 2],
    pub pupil_gain: f32,
    pub deadband_deg: f32,
    /// Pupil PID: deg/s of gaze speed per degree of image error.
    pub pid_kp: f32,
    pub pid_ki: f32,
    pub pid_kd: f32,
    pub max_speed: f32,
    pub smooth_cutoff: f32,
}
impl Default for MountConfig {
    fn default() -> Self {
        Self {
            limits: Default::default(),
            mode: MountMode::External,
            camera_xyz: [0.; 3],
            camera_ypr: [0.; 3],
            horizontal_fov: 65.,
            depth_mode: DepthMode::Manual,
            depth_m: 2.,
            shoulder_width_m: 0.40,
            trim: [0.; 2],
            pupil_scale: [1.; 2],
            backlash_deg: [0.; 2],
            invert: [false; 2],
            pupil_gain: 0.55,
            deadband_deg: 0.6,
            pid_kp: 4.,
            pid_ki: 0.,
            pid_kd: 0.,
            max_speed: 65.,
            smooth_cutoff: 1.6,
        }
    }
}
impl Config {
    pub fn endpoint(&self) -> String {
        if self.uart {
            if self.uart_baud == 230400 {
                format!("uart:{}", self.uart_port.trim())
            } else {
                format!("uart:{}?baud={}", self.uart_port.trim(), self.uart_baud)
            }
        } else {
            self.eye_address.clone()
        }
    }
    pub fn path() -> PathBuf {
        dirs::config_dir()
            .unwrap_or_else(|| ".".into())
            .join("ztrak-ojo/config.json")
    }
    pub fn load() -> Self {
        std::fs::read_to_string(Self::path())
            .ok()
            // Notepad and PowerShell add a BOM that serde_json rejects.
            .and_then(|s| serde_json::from_str(s.trim_start_matches('\u{feff}')).ok())
            .unwrap_or_else(Self::first_run)
    }
    /// Windows starts from the measured calibration, with webcam and COM port still to choose.
    fn first_run() -> Self {
        if cfg!(windows) {
            serde_json::from_str(WINDOWS_FIRST_RUN).unwrap_or_default()
        } else {
            Self::default()
        }
    }
    pub fn validate(&self) -> anyhow::Result<()> {
        anyhow::ensure!(
            self.camera.width >= 160
                && self.camera.width <= 3840
                && self.camera.height >= 120
                && self.camera.height <= 2160,
            "Resolución fuera de rango"
        );
        anyhow::ensure!((1..=60).contains(&self.camera.fps), "FPS fuera de rango");
        let m = &self.mount;
        anyhow::ensure!(
            m.camera_xyz
                .iter()
                .chain(m.camera_ypr.iter())
                .chain(m.trim.iter())
                .all(|v| v.is_finite()),
            "Calibración no válida"
        );
        anyhow::ensure!(
            m.camera_xyz.iter().all(|v| v.abs() <= 10.)
                && m.camera_ypr.iter().all(|v| v.abs() <= 180.),
            "Posición/orientación fuera de rango"
        );
        for (v, lo, hi) in [
            (m.horizontal_fov, 25., 120.),
            (m.depth_m, 0.3, 20.),
            (m.shoulder_width_m, 0.15, 0.8),
            (m.pupil_gain, 0.05, 1.),
            (m.deadband_deg, 0.1, 5.),
            (m.pid_kp, 0., 20.),
            (m.pupil_scale[0], 0.2, 6.),
            (m.pupil_scale[1], 0.2, 6.),
            (m.backlash_deg[0], 0., 15.),
            (m.backlash_deg[1], 0., 15.),
            (m.pid_ki, 0., 20.),
            (m.pid_kd, 0., 2.),
            (m.max_speed, 5., 160.),
            (m.smooth_cutoff, 0.2, 8.),
        ] {
            anyhow::ensure!(
                v.is_finite() && v >= lo && v <= hi,
                "Valor de calibración fuera de rango"
            );
        }
        anyhow::ensure!(
            self.focus.lost_timeout.is_finite() && (0.1..=3.).contains(&self.focus.lost_timeout),
            "Margen de pérdida no válido"
        );
        anyhow::ensure!(
            self.vision.threads > 0
                && self.vision.threads <= 16
                && self.vision.max_poses > 0
                && self.vision.max_poses <= 8
                && ["auto", "cpu"].contains(&self.vision.device.as_str()),
            "Configuración de visión no válida"
        );
        if self.uart {
            anyhow::ensure!(
                cfg!(windows) || !self.uart_port.trim().is_empty(),
                "Elige un puerto UART"
            );
            anyhow::ensure!([115200, 230400, 250000].contains(&self.uart_baud), "Velocidad UART no válida");
        } else {
            crate::eye::parse_address(&self.eye_address)?;
        }
        for (v, lo, hi) in [
            (self.boredom_after_s, 8., 180.),
            (self.droop_s, 0.6, 5.),
            (self.sleep_slit, 20., 34.),
            (self.wake_motion_fraction, 0.01, 0.15),
            (self.sleep_after_s, 5., 300.),
            (self.sleep_for_s, 1., 60.),
            (self.reset_pause_s, 0.5, 10.),
        ] {
            anyhow::ensure!(
                v.is_finite() && v >= lo && v <= hi,
                "Tiempo de comportamiento fuera de rango"
            );
        }
        Ok(())
    }
    pub fn save(&self) -> anyhow::Result<()> {
        self.validate()?;
        let p = Self::path();
        std::fs::create_dir_all(p.parent().unwrap())?;
        std::fs::write(p, serde_json::to_string_pretty(self)?)?;
        Ok(())
    }
    pub fn resolve(path: &str) -> PathBuf {
        let p = PathBuf::from(path);
        if p.is_absolute() || p.exists() {
            return p;
        }
        if let Ok(exe) = std::env::current_exe() {
            for base in exe.ancestors().skip(1).take(4) {
                let c = base.join(path);
                if c.exists() {
                    return c;
                }
            }
        }
        p
    }
}

#[cfg(test)]
mod windows_first_run {
    use super::*;
    #[test]
    fn starts_calibrated_without_choosing_webcam_or_port() {
        let c: Config = serde_json::from_str(WINDOWS_FIRST_RUN).unwrap();
        assert!(c.camera.source.is_empty() && c.uart_port.is_empty());
        assert!(c.uart && c.uart_baud == 250000);
        assert_eq!(c.mount.backlash_deg, [0., 0.]);
        assert_eq!(c.vision.device, "auto");
    }
}
