//! One convex mechanical envelope, read from the board. Values in tenths of a degree.
use serde::{Deserialize, Serialize};
pub const YAW: [f32; 7] = [-55., -45., -30., 0., 30., 45., 55.];
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct MotionLimits {
    pub up: [i16; 7],
    pub down: [i16; 7],
}
impl Default for MotionLimits {
    fn default() -> Self {
        Self {
            up: [0, 150, 250, 250, 250, 150, 0],
            down: [0, 150, 250, 250, 250, 150, 0],
        }
    }
}
impl MotionLimits {
    pub fn validate(&self) -> anyhow::Result<()> {
        for row in [self.up, self.down] {
            anyhow::ensure!(
                row.iter().all(|v| (0..=350).contains(v)) && row[3] >= 100,
                "Vertical entre 0° y 35°; en el centro al menos 10°"
            );
            for i in 0..3 {
                anyhow::ensure!(
                    row[i] <= row[i + 1] && row[6 - i] <= row[5 - i],
                    "El margen debe disminuir al acercarse a cada extremo horizontal"
                );
            }
            for i in 0..5 {
                let a = (row[i + 1] - row[i]) as f32 / (YAW[i + 1] - YAW[i]);
                let b = (row[i + 2] - row[i + 1]) as f32 / (YAW[i + 2] - YAW[i + 1]);
                anyhow::ensure!(
                    a + 0.0001 >= b,
                    "La curva debe ser convexa: ajusta también los puntos vecinos para evitar entrantes"
                );
            }
        }
        Ok(())
    }
    pub fn pitch(&self, yaw: f32) -> [f32; 2] {
        let x = yaw.clamp(-55., 55.);
        let i = (0..6).find(|&i| x <= YAW[i + 1]).unwrap_or(5);
        let t = (x - YAW[i]) / (YAW[i + 1] - YAW[i]);
        let interp = |r: [i16; 7]| (r[i] as f32 + (r[i + 1] - r[i]) as f32 * t) * 0.1;
        [-interp(self.down), interp(self.up)]
    }
    pub fn yaw(&self, pitch: f32) -> [f32; 2] {
        let row = if pitch >= 0. { self.up } else { self.down };
        let y = pitch.abs() * 10.;
        let mut lo = 0.;
        let mut hi = 0.;
        for i in 0..6 {
            let (a, b) = (row[i] as f32, row[i + 1] as f32);
            if a >= y {
                lo = f32::min(lo, YAW[i]);
                hi = f32::max(hi, YAW[i]);
            }
            if b >= y {
                lo = f32::min(lo, YAW[i + 1]);
                hi = f32::max(hi, YAW[i + 1]);
            }
            if (a < y && b >= y) || (a >= y && b < y) {
                let x = YAW[i] + (YAW[i + 1] - YAW[i]) * (y - a) / (b - a);
                lo = f32::min(lo, x);
                hi = f32::max(hi, x);
            }
        }
        [lo, hi]
    }
    pub fn envelope(&self, a: [f32; 2]) -> [f32; 2] {
        let y = a[1].clamp(-(self.down[3] as f32) * 0.1, self.up[3] as f32 * 0.1);
        let [lo, hi] = self.yaw(y);
        [a[0].clamp(lo, hi), y]
    }
    pub fn tracking(&self, a: [f32; 2]) -> [f32; 2] {
        let x = a[0].clamp(-55., 55.);
        let [lo, hi] = self.pitch(x);
        [x, a[1].clamp(lo, hi)]
    }
    /// Closest feasible gaze; unlike yaw-priority clipping this can slide
    /// away from a lateral stop to correct a vertical image error.
    pub fn closest(&self, p: [f32; 2]) -> [f32; 2] {
        let clipped = self.tracking(p);
        if (clipped[0] - p[0]).abs() < 0.00001 && (clipped[1] - p[1]).abs() < 0.00001 {
            return p;
        }
        let v: [[f32; 2]; 14] = std::array::from_fn(|i| {
            if i < 7 {
                [YAW[i], self.up[i] as f32 * 0.1]
            } else {
                let j = 13 - i;
                [YAW[j], -self.down[j] as f32 * 0.1]
            }
        });
        let mut best = clipped;
        let mut distance = f32::INFINITY;
        for i in 0..14 {
            let a = v[i];
            let b = v[(i + 1) % 14];
            let d = [b[0] - a[0], b[1] - a[1]];
            let length = d[0] * d[0] + d[1] * d[1];
            if length < 0.00001 {
                continue;
            }
            let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / length).clamp(0., 1.);
            let q = [a[0] + t * d[0], a[1] + t * d[1]];
            let r = (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
            if r < distance {
                distance = r;
                best = q;
            }
        }
        best
    }
    pub fn gaze_step(&self, from: [f32; 2], to: [f32; 2], max: f32) -> [f32; 2] {
        let to = self.closest(to);
        let d = [to[0] - from[0], to[1] - from[1]];
        let extent = d[0].abs().max(d[1].abs());
        let k = if extent > max { max / extent } else { 1. };
        [from[0] + d[0] * k, from[1] + d[1] * k]
    }
    pub fn step(&self, from: [f32; 2], to: [f32; 2], max: f32) -> [f32; 2] {
        let to = self.tracking(to);
        let d = [to[0] - from[0], to[1] - from[1]];
        let extent = d[0].abs().max(d[1].abs());
        let k = if extent > max { max / extent } else { 1. };
        self.tracking([from[0] + d[0] * k, from[1] + d[1] * k])
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn default_matches_existing_mechanism() {
        let l = MotionLimits::default();
        l.validate().unwrap();
        for x in -550..=550 {
            let y = x as f32 / 10.;
            let old = crate::geometry::envelope([55., y]);
            let new = l.envelope([55., y]);
            assert!((old[0] - new[0]).abs() < 0.001 && (old[1] - new[1]).abs() < 0.001);
        }
    }
    #[test]
    fn asymmetric_curve_and_transitions_are_bounded() {
        let mut l = MotionLimits::default();
        l.up[6] = 30;
        l.validate().unwrap();
        assert_eq!(l.tracking([55., 20.]), [55., 3.]);
        assert_eq!(l.tracking([-55., 20.]), [-55., 0.]);
        let mut p = [0., 25.];
        for _ in 0..200 {
            let n = l.step(p, [55., 20.], 0.5);
            assert!((n[0] - p[0]).abs() <= 0.501 && (n[1] - p[1]).abs() <= 0.501);
            p = n;
        }
        assert!((p[0] - 55.).abs() < 0.001 && (p[1] - 3.).abs() < 0.001);
        l.up[5] = 0;
        assert!(l.validate().is_err());
    }
}
