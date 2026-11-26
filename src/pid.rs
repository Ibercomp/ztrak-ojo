//! Simple PID for the pupil-mounted camera.
//!
//! Input: angular error of the face relative to the image centre (degrees).
//! Output: next gaze command (degrees). The PID gives a gaze *velocity*
//! (deg/s), integrated into the command. With only Kp this is a first-order
//! loop: velocity = Kp * error, time constant 1/Kp seconds.
use crate::motion_limits::MotionLimits;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PidGains {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub deadband: f32,
    pub max_speed: f32,
    /// Play to take up on reversal (commanded degrees per axis).
    pub backlash: [f32; 2],
}

#[derive(Default)]
pub struct PupilPid {
    pub limits: MotionLimits,
    person: Option<u64>,
    position: Option<[f32; 2]>,
    integral: [f32; 2],
    previous_error: Option<[f32; 2]>,
    derivative: [f32; 2],
    /// Last direction of travel per axis (-1, 0, +1) for backlash compensation.
    direction: [f32; 2],
}

impl PupilPid {
    /// Blink, link loss or a lost target: restart from the reported pose.
    pub fn pause(&mut self, current: [f32; 2]) {
        *self = Self { limits: self.limits, ..Self::default() };
        self.position = Some(current);
    }

    pub fn step(&mut self, person: u64, error: [f32; 2], current: [f32; 2], dt: f32, g: PidGains) -> [f32; 2] {
        // A NaN would stay in the integrated position: hold the last good command.
        if !error.iter().chain(&current).all(|v| v.is_finite()) || !dt.is_finite() {
            return self.position.unwrap_or(current);
        }
        if self.person != Some(person) || dt >= 0.2 || self.position.is_none() {
            self.pause(current);
            self.person = Some(person);
        }
        let dt = dt.clamp(0.005, 0.1);
        let base = self.position.unwrap();
        let mut out = base;
        let mut velocity = [0.; 2];
        for i in 0..2 {
            // Continuous dead band: no jump when crossing its edge.
            let e = error[i].signum() * (error[i].abs() - g.deadband).max(0.);
            if let Some(p) = self.previous_error {
                let raw = (error[i] - p[i]) / dt;
                self.derivative[i] += (1. - (-dt / 0.05).exp()) * (raw - self.derivative[i]);
            }
            velocity[i] = (g.kp * e + g.ki * self.integral[i] + g.kd * self.derivative[i])
                .clamp(-g.max_speed, g.max_speed);
            out[i] += velocity[i] * dt;
        }
        // Axis-wise clip: sliding along a slanted limit edge couples yaw and
        // pitch and, with camera latency, swings back and forth.
        let bounded = self.limits.tracking(out);
        for i in 0..2 {
            // Anti-windup: integrate only while the axis is free to move.
            if (bounded[i] - out[i]).abs() < 1e-3 && velocity[i].abs() < g.max_speed {
                let e = error[i].signum() * (error[i].abs() - g.deadband).max(0.);
                self.integral[i] = (self.integral[i] + e * dt).clamp(-20., 20.);
            }
        }
        self.previous_error = Some(error);
        // Backlash: keep the command offset by half the play towards the
        // current direction of travel, so a reversal jumps across the gap.
        for i in 0..2 {
            let d = bounded[i] - base[i];
            if d.abs() > 0.02 {
                self.direction[i] = d.signum();
            }
        }
        self.position = Some(bounded);
        let sent = [
            bounded[0] + self.direction[0] * g.backlash[0] / 2.,
            bounded[1] + self.direction[1] * g.backlash[1] / 2.,
        ];
        self.limits.tracking(sent)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn gains(kp: f32) -> PidGains {
        PidGains { kp, ki: 0., kd: 0., deadband: 0., max_speed: 160., backlash: [0.; 2] }
    }
    /// Camera on the eye: error = target - gaze, seen one frame late.
    fn simulate(g: PidGains, target: impl Fn(f32) -> f32, secs: f32) -> Vec<(f32, f32)> {
        let mut pid = PupilPid::default();
        let (mut gaze, mut seen, dt) = (0f32, 0f32, 1. / 25.);
        let mut out = vec![];
        for n in 0..(secs / dt) as usize {
            let t = n as f32 * dt;
            let cmd = pid.step(1, [target(t) - seen, 0.], [gaze, 0.], dt, g)[0];
            seen = gaze; // camera latency: one frame
            gaze = cmd;
            out.push((t, target(t) - gaze));
        }
        out
    }
    #[test]
    fn proportional_converges_to_a_still_face() {
        let r = simulate(gains(4.), |_| 20., 3.);
        assert!(r.last().unwrap().1.abs() < 0.1, "{:?}", r.last());
        let overshoot = r.iter().map(|x| -x.1).fold(0f32, f32::max);
        assert!(overshoot < 0.5, "overshoot {overshoot}");
    }
    #[test]
    fn proportional_lag_on_a_ramp_is_speed_over_kp() {
        // 10 deg/s ramp, Kp 4: steady lag ~ speed/Kp, measured from the latest command.
        let r = simulate(gains(4.), |t| 10. * t, 4.);
        let e = r.last().unwrap().1;
        assert!((1.4..3.2).contains(&e), "{e}");
    }
    #[test]
    fn integral_removes_ramp_lag() {
        let g = PidGains { ki: 4., ..gains(4.) };
        let r = simulate(g, |t| 10. * t, 4.5);
        // Residual = one camera frame of ramp; P alone leaves ~1.7 deg.
        assert!(r.last().unwrap().1.abs() < 1.0, "{:?}", r.last());
    }
    #[test]
    fn backlash_offset_flips_on_reversal() {
        let g = PidGains { backlash: [8., 0.], ..gains(4.) };
        let mut pid = PupilPid::default();
        let a = pid.step(1, [5., 0.], [0., 0.], 0.04, g)[0];
        assert!((a - (0.8 + 4.)).abs() < 1e-3, "{a}");
        let b = pid.step(1, [-5., 0.], [a, 0.], 0.04, g)[0];
        assert!((b - (0.0 - 4.)).abs() < 1e-3, "{b}");
    }
    #[test]
    fn invalid_vision_output_never_reaches_the_command() {
        let g = gains(4.);
        let mut pid = PupilPid::default();
        let good = pid.step(1, [5., 2.], [1., 1.], 0.04, g);
        for bad in [f32::NAN, f32::INFINITY] {
            assert_eq!(pid.step(1, [bad, 0.], [1., 1.], 0.04, g), good);
        }
        let next = pid.step(1, [5., 2.], good, 0.04, g);
        assert!(next[0].is_finite() && next[0] > good[0]);
    }
    #[test]
    fn dead_band_holds_still() {
        let g = PidGains { deadband: 0.5, ..gains(4.) };
        let mut pid = PupilPid::default();
        for _ in 0..50 {
            assert_eq!(pid.step(1, [0.4, -0.4], [3., 1.], 0.04, g), [3., 1.]);
        }
    }
}
