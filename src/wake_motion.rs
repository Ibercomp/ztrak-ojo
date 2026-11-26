//! Cheap image motion for waking even when the slit prevents face detection.
use crate::shared::Frame;
use std::time::{Duration, Instant};
const W: usize = 48;
const H: usize = 27;
const N: usize = W * H;
#[derive(Default)]
pub struct WakeMotion {
    last: Option<Instant>,
    reference: Option<([f32; N], Instant)>,
    pose: Option<[f32; 3]>,
    stable_since: Option<Instant>,
    movement_since: Option<Instant>,
}
impl WakeMotion {
    pub fn update(&mut self, f: &Frame, pose: [f32; 3], blocked: bool, fraction: f32) -> bool {
        if self.last.is_some_and(|t| f.time <= t) {
            return false;
        }
        let gap = self
            .last
            .is_some_and(|t| f.time.duration_since(t) > Duration::from_millis(400));
        self.last = Some(f.time);
        if f.width == 0
            || f.height == 0
            || f.rgb.len() != (f.width as usize * f.height as usize * 3)
        {
            self.reference = None;
            self.movement_since = None;
            self.stable_since = Some(f.time);
            return false;
        }
        let own_motion = self.pose.is_none_or(|p| {
            (p[0] - pose[0]).hypot(p[1] - pose[1]) > 0.15 || (p[2] - pose[2]).abs() > 0.05
        });
        if blocked || gap || own_motion {
            self.pose = Some(pose);
            self.reference = None;
            self.movement_since = None;
            self.stable_since = Some(f.time);
            return false;
        }
        let stable = *self.stable_since.get_or_insert(f.time);
        if f.time.duration_since(stable) < Duration::from_millis(450) {
            return false;
        }
        let mut pixels = [0.; N];
        for y in 0..H {
            for x in 0..W {
                // Sample the central viewing band; avoid the upper/lower lid edges.
                let px = ((x as f32 + 0.5) / W as f32 * f.width as f32) as usize;
                let py = ((0.22 + 0.56 * (y as f32 + 0.5) / H as f32) * f.height as f32) as usize;
                let j = (py * f.width as usize + px) * 3;
                pixels[y * W + x] = 0.299 * f.rgb[j] as f32
                    + 0.587 * f.rgb[j + 1] as f32
                    + 0.114 * f.rgb[j + 2] as f32;
            }
        }
        let Some((reference, stamp)) = &self.reference else {
            self.reference = Some((pixels, f.time));
            return false;
        };
        // Remove global brightness change (auto exposure / a light switching).
        let brightness = pixels
            .iter()
            .zip(reference)
            .map(|(a, b)| a - b)
            .sum::<f32>()
            / N as f32;
        let changed = pixels
            .iter()
            .zip(reference)
            .filter(|(a, b)| (*a - *b - brightness).abs() > 20.)
            .count() as f32
            / N as f32;
        if changed >= fraction {
            let since = *self.movement_since.get_or_insert(f.time);
            if f.time.duration_since(since) >= Duration::from_millis(200) {
                self.reference = Some((pixels, f.time));
                self.movement_since = None;
                return true;
            }
        } else if changed < fraction * 0.5 {
            self.movement_since = None;
        }
        if self.movement_since.is_none() && f.time.duration_since(*stamp) > Duration::from_secs(1) {
            self.reference = Some((pixels, f.time));
        }
        false
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn frame(t: Instant, light: u8, patch: bool) -> Frame {
        let mut rgb = vec![light; 96 * 54 * 3];
        if patch {
            for y in 20..30 {
                for x in 35..55 {
                    for c in 0..3 {
                        rgb[(y * 96 + x) * 3 + c] = light + 70;
                    }
                }
            }
        }
        Frame {
            id: 0,
            width: 96,
            height: 54,
            rgb,
            time: t,
        }
    }
    #[test]
    fn exposure_noise_and_one_frame_do_not_wake_but_sustained_local_motion_does() {
        let t = Instant::now();
        let mut m = WakeMotion::default();
        for i in 0..20 {
            assert!(!m.update(
                &frame(t + Duration::from_millis(i * 50), 80 + i as u8, false),
                [0.; 3],
                false,
                0.03
            ));
        }
        assert!(!m.update(
            &frame(t + Duration::from_millis(1000), 100, true),
            [0.; 3],
            false,
            0.03
        ));
        assert!(!m.update(
            &frame(t + Duration::from_millis(1050), 100, false),
            [0.; 3],
            false,
            0.03
        ));
        for ms in [1100, 1150, 1200, 1250] {
            assert!(!m.update(
                &frame(t + Duration::from_millis(ms), 100, true),
                [0.; 3],
                false,
                0.03
            ));
        }
        assert!(m.update(
            &frame(t + Duration::from_millis(1300), 100, true),
            [0.; 3],
            false,
            0.03
        ));
    }
    #[test]
    fn slowly_accumulating_camera_motion_is_also_ignored() {
        let t = Instant::now();
        let mut m = WakeMotion::default();
        for i in 0..100 {
            assert!(!m.update(
                &frame(t + Duration::from_millis(i * 50), 80, i % 5 < 3),
                [i as f32 * 0.05, 0., 0.],
                false,
                0.03
            ));
        }
    }
    #[test]
    fn eye_and_lid_motion_and_stale_camera_do_not_wake() {
        let t = Instant::now();
        let mut m = WakeMotion::default();
        for i in 0..50 {
            assert!(!m.update(
                &frame(t + Duration::from_millis(i * 50), 80, i % 2 == 0),
                [i as f32, 0., i as f32],
                false,
                0.03
            ));
        }
        assert!(!m.update(
            &frame(t + Duration::from_secs(5), 80, true),
            [0.; 3],
            false,
            0.03
        ));
    }
}
