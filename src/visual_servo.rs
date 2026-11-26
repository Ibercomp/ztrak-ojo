//! Image-error feedback: command telemetry is not an encoder on an MF90.
#[cfg(test)]
use crate::geometry::envelope;
use std::time::Instant;
#[derive(Default)]
pub struct VisualServo {
    pub limits: crate::motion_limits::MotionLimits,
    key: Option<(u64, bool)>,
    position: Option<[f32; 2]>,
    previous_error: Option<[f32; 2]>,
    derivative: [f32; 2],
    velocity: [f32; 2],
    pursuit: [f32; 2],
    anchor_offset: [f32; 2],
    anchor_age: f32,
    anchor_duration: f32,
    resuming: bool,
    resume_age: f32,
}
impl VisualServo {
    /// Forget pre-pause velocity, but retain identity for a controlled restart.
    pub fn pause(&mut self, current: [f32; 2]) {
        self.position = Some(current);
        self.velocity = [0.; 2];
        self.pursuit = [0.; 2];
        self.derivative = [0.; 2];
        self.previous_error = None;
        self.anchor_offset = [0.; 2];
        self.resuming = true;
        self.resume_age = 0.;
    }
    pub fn step(
        &mut self,
        key: (u64, bool),
        error: [f32; 2],
        current: [f32; 2],
        dt: f32,
        gain: f32,
        max_speed: f32,
    ) -> [f32; 2] {
        let new_person = self.key.map(|k| k.0) != Some(key.0);
        let restarting = new_person || dt >= 0.2;
        if new_person || dt >= 0.5 {
            *self = Self {
                limits: self.limits,
                resuming: self.resuming,
                ..Self::default()
            };
            self.position = Some(current);
        } else if restarting {
            // Brief detector gaps do not erase the face/body transition anchor.
            // Resume at the current pose, never integrate the missing time.
            self.position = Some(current);
            self.velocity = [0.; 2];
            self.pursuit = [0.; 2];
            self.derivative = [0.; 2];
        }
        if self.key != Some(key) {
            self.anchor_offset = self
                .previous_error
                .map(|p| [p[0] - error[0], p[1] - error[1]])
                .unwrap_or([0.; 2]);
            self.anchor_age = 0.;
            self.anchor_duration = if key.1 { 0.28 } else { 0.45 };
            self.pursuit = [0.; 2];
            self.previous_error = None;
            self.derivative = [0.; 2];
        }
        self.key = Some(key);
        // A missed observation is not time that the eye was moving. Restart
        // with one normal frame, including after blink/lid/network holds.
        let dt = if restarting { dt.min(0.04) } else { dt }.clamp(0.005, 0.1);
        if self.resuming {
            self.resume_age += dt;
            if self.resume_age >= 0.12 {
                self.resuming = false;
            }
        }
        self.anchor_age += dt;
        let phase = (self.anchor_age / self.anchor_duration.max(0.01)).min(1.);
        let blend = 1. - phase * phase * phase * (10. + phase * (-15. + 6. * phase));
        let error = [
            error[0] + self.anchor_offset[0] * blend,
            error[1] + self.anchor_offset[1] * blend,
        ];
        let changing_anchor = self.anchor_offset.iter().any(|v| v.abs() > 0.1) && phase < 1.;
        let mut out = self.position.unwrap_or(current);
        for i in 0..2 {
            let raw_d = self
                .previous_error
                .map(|p| (error[i] - p[i]) / dt)
                .unwrap_or(0.);
            self.derivative[i] +=
                (1. - (-dt / 0.06).exp()) * (raw_d.clamp(-250., 250.) - self.derivative[i]);
            // Derivative of image error brakes camera-induced motion, rather
            // than predicting person velocity from an unmeasured servo angle.
            if error[i] * self.pursuit[i] < 0.
                || error[i].abs() < 1.2
                || (error[i].abs() < 1.
                    && self.derivative[i] * self.pursuit[i] < -2. * self.pursuit[i].abs())
            {
                self.pursuit[i] *= (-dt / 0.12).exp();
            }
            if !changing_anchor && error[i].abs() >= 1.2 {
                self.pursuit[i] = (self.pursuit[i] + 3. * gain * error[i] * dt).clamp(-40., 40.);
            }
            // Fast pursuit away from fixation; lower loop gain near the face
            // avoids chasing PWM quantization, mechanical play and image noise.
            let distance = ((error[i].abs() - 0.6) / 3.).clamp(0., 1.);
            let proportional = 3.2 + 3.5 * distance * distance * (3. - 2. * distance);
            let wanted =
                (proportional * gain * error[i] + self.pursuit[i] + 0.4 * self.derivative[i])
                    .clamp(-max_speed, max_speed);
            let accel = if changing_anchor || self.resuming {
                220.
            } else {
                700.
            };
            self.velocity[i] += (wanted - self.velocity[i]).clamp(-accel * dt, accel * dt);
            out[i] += self.velocity[i] * dt;
        }
        self.previous_error = Some(error);
        let projected = self.limits.closest(out);
        // Bound command lead against fresh firmware telemetry. Without this,
        // repeated visual errors can build a distant destination while the
        // motor is held or lagging; releasing it produces a catch-up burst.
        let lead = (max_speed * 0.05).clamp(2., 8.);
        let limited = self.limits.gaze_step(current, out, lead);
        let bounded =
            self.limits
                .gaze_step(self.position.unwrap_or(current), limited, max_speed * dt);
        for i in 0..2 {
            if projected[i] != out[i] || (limited[i] - out[i]).abs() > 0.001 {
                self.pursuit[i] = 0.;
            }
        }
        // Feed the feasible displacement back into the velocity state. The
        // vertical axis may have moved toward neutral to unlock lateral travel.
        let previous = self.position.unwrap_or(current);
        for i in 0..2 {
            self.velocity[i] = (bounded[i] - previous[i]) / dt;
        }
        self.position = Some(bounded);
        bounded
    }
}
/// Zero speed and acceleration at both ends; only for neutral/search poses.
#[derive(Default)]
pub struct EasePose {
    pub limits: crate::motion_limits::MotionLimits,
    path: Option<(Instant, [f32; 2], [f32; 2], f32)>,
    last_tick: Option<Instant>,
}
impl EasePose {
    pub fn pause(&mut self, now: Instant) {
        if let (Some(last), Some((start, _, _, _))) = (self.last_tick, self.path.as_mut()) {
            *start += now.saturating_duration_since(last);
        }
        self.last_tick = Some(now);
    }
    pub fn step(&mut self, now: Instant, current: [f32; 2], target: [f32; 2]) -> [f32; 2] {
        // A stalled network must not fast-forward an animation on reconnect.
        if self
            .last_tick
            .is_some_and(|last| now.saturating_duration_since(last).as_millis() > 100)
        {
            self.path = None;
        }
        self.last_tick = Some(now);
        if self.path.is_none_or(|(_, _, dst, _)| dst != target) {
            let distance = (target[0] - current[0])
                .abs()
                .max((target[1] - current[1]).abs());
            let (speed, accel) = if target == [0.; 2] {
                (65., 220.)
            } else {
                (95., 350.)
            };
            let duration = (1.875 * distance / speed)
                .max((5.78 * distance / accel).sqrt())
                .max(0.35);
            self.path = Some((now, current, target, duration));
        }
        let (start, from, to, duration) = self.path.unwrap();
        let t = (now.duration_since(start).as_secs_f32() / duration).min(1.);
        let u = t * t * t * (10. + t * (-15. + 6. * t));
        self.limits.envelope([
            from[0] + (to[0] - from[0]) * u,
            from[1] + (to[1] - from[1]) * u,
        ])
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::VecDeque;
    #[test]
    fn resume_discards_old_velocity_and_softens_first_command() {
        let mut f = VisualServo::default();
        let mut p = [0.; 2];
        for _ in 0..10 {
            p = f.step((1, true), [15., 0.], p, 0.04, 0.95, 160.);
        }
        f.pause(p);
        let q = f.step((1, true), [-20., 10.], p, 0.7, 0.95, 160.);
        assert!(
            (q[0] - p[0]).abs() <= 0.353 && (q[1] - p[1]).abs() <= 0.353,
            "restart jumped: {p:?} -> {q:?}"
        );
        assert!(
            q[0] < p[0],
            "old pursuit direction must not survive a pause"
        );
    }
    #[test]
    fn vertical_error_at_lateral_stop_slides_along_boundary() {
        for side in [-1., 1.] {
            for vertical in [-1., 1.] {
                let mut f = VisualServo::default();
                let mut p = [side * 55., 0.];
                for _ in 0..40 {
                    let q = f.step((1, true), [0., vertical * 10.], p, 0.04, 0.95, 160.);
                    assert!((q[0] - p[0]).abs() < 4.01 && (q[1] - p[1]).abs() < 4.01);
                    let b = f.limits.tracking(q);
                    assert!((b[0] - q[0]).abs() < 0.001 && (b[1] - q[1]).abs() < 0.001);
                    p = q;
                }
                assert!(
                    p[0].abs() < 54. && p[1] * vertical > 1.,
                    "must leave the corner to follow vertically: {p:?}"
                );
            }
        }
    }
    #[test]
    fn delayed_motor_cannot_accumulate_a_far_away_destination() {
        let mut f = VisualServo::default();
        for _ in 0..100 {
            let p = f.step((1, true), [20., -15.], [0.; 2], 0.04, 0.95, 160.);
            assert!(
                p[0].abs() <= 8.001 && p[1].abs() <= 8.001,
                "command ran away while motor stayed put: {p:?}"
            );
        }
        let p = f.step((1, true), [-20., 15.], [0.; 2], 0.04, 0.95, 160.);
        assert!(p[0].abs() <= 8.001 && p[1].abs() <= 8.001);
    }
    #[test]
    fn quantized_servo_and_noisy_camera_do_not_hunt_at_fixation() {
        let dt = 0.05f32;
        let mut history = VecDeque::from(vec![0.; 4]);
        let mut f = VisualServo::default();
        let mut actual = 0f32;
        let mut positions = vec![];
        for tick in 0..240 {
            let seen = history.pop_front().unwrap();
            let error = 15. - seen + 0.2 * (tick as f32 * 2.1).sin();
            let cmd = f.step((1, true), [error, 0.], [actual, 0.], dt, 0.95, 160.);
            let pwm = (cmd[0] / 0.488).round() * 0.488;
            if (pwm - actual).abs() > 0.35 {
                actual += (1. - (-dt / 0.08).exp()) * (pwm - actual);
            }
            history.push_back(actual);
            if tick > 180 {
                positions.push(actual);
            }
        }
        let span = positions.iter().copied().fold(f32::NEG_INFINITY, f32::max)
            - positions.iter().copied().fold(f32::INFINITY, f32::min);
        assert!(span < 0.8, "stationary target hunts over {span} degrees");
        assert!(
            (actual - 15.).abs() < 0.8,
            "fixation error {}",
            actual - 15.
        );
    }
    #[test]
    fn face_to_body_after_a_short_detector_gap_still_blends() {
        let mut f = VisualServo::default();
        let current = [10., -5.];
        f.step((1, true), [0.; 2], current, 0.04, 0.95, 160.);
        let p = f.step((1, false), [0., 18.], current, 0.3, 0.95, 160.);
        assert!(
            (p[1] - current[1]).abs() < 0.1,
            "brief loss must not skip the face-to-body blend: {p:?}"
        );
    }
    #[test]
    fn search_restarts_from_reported_pose_after_a_network_stall() {
        let t = Instant::now();
        let mut p = EasePose::default();
        p.step(t, [0.; 2], [50., 0.]);
        let moving = p.step(t + std::time::Duration::from_millis(80), [0.; 2], [50., 0.]);
        let resumed = p.step(t + std::time::Duration::from_millis(430), moving, [50., 0.]);
        assert_eq!(
            resumed, moving,
            "do not catch up 350 ms in a single command"
        );
        let next = p.step(
            t + std::time::Duration::from_millis(450),
            resumed,
            [50., 0.],
        );
        assert!((next[0] - resumed[0]).abs() < 0.1);
    }
    #[test]
    fn reacquisition_after_a_long_gap_never_integrates_the_missing_time() {
        let mut f = VisualServo::default();
        let current = [12., 3.];
        for _ in 0..10 {
            f.step((1, true), [18., -8.], current, 0.04, 0.95, 160.);
        }
        let restarted = f.step((1, true), [25., -15.], current, 0.7, 0.95, 160.);
        for i in 0..2 {
            assert!((restarted[i] - current[i]).abs() <= 1.121);
        }
        let new_person = f.step((2, true), [-25., 15.], current, 0.15, 0.95, 160.);
        for i in 0..2 {
            assert!((new_person[i] - current[i]).abs() <= 1.121);
        }
    }
    #[test]
    fn delayed_cheap_servo_converges_without_sustained_oscillation() {
        for delay in [0.04, 0.08, 0.12, 0.16] {
            for lag in [0.04, 0.08, 0.12] {
                let dt = 0.02;
                let mut history = VecDeque::from(vec![0.; (delay / dt) as usize + 1]);
                let mut f = VisualServo::default();
                let mut actual = 0f32;
                let mut errors = vec![];
                for i in 0..500 {
                    let seen = history.pop_front().unwrap();
                    let cmd = f.step((1, true), [20. - seen, 0.], [actual, 0.], dt, 0.95, 160.);
                    actual += (1. - (-dt / lag as f32).exp()) * (cmd[0] - actual);
                    history.push_back(actual);
                    if i > 400 {
                        errors.push((actual - 20.).abs());
                    }
                }
                assert!(
                    errors.iter().all(|e| *e < 0.15),
                    "delay={delay} lag={lag} error={actual}"
                );
            }
        }
    }
    #[test]
    fn horizontal_only_error_can_reach_full_yaw_from_saturated_pitch() {
        for side in [-1., 1.] {
            for vertical in [-1., 1.] {
                let mut f = VisualServo::default();
                let mut p = [side * 30., vertical * 25.];
                for _ in 0..400 {
                    let next = f.step((1, true), [side * 18., 0.], p, 0.02, 0.95, 100.);
                    assert!(
                        (next[0] - envelope(next)[0]).abs() < 0.001
                            && (next[1] - envelope(next)[1]).abs() < 0.001
                    );
                    assert!((next[0] - p[0]).abs() <= 2.001 && (next[1] - p[1]).abs() <= 2.001);
                    p = next;
                }
                assert!(
                    (p[0] - side * 55.).abs() < 0.01 && p[1].abs() < 0.01,
                    "{p:?}"
                );
                // Reversing the observed horizontal error must leave the stop immediately.
                let next = f.step((1, true), [-side * 10., 0.], p, 0.04, 0.95, 100.);
                assert!(next[0].abs() < p[0].abs());
            }
        }
    }
    #[test]
    fn motion_and_anchor_switch_respect_speed_acceleration_and_limits() {
        let mut f = VisualServo::default();
        let mut current = [0.; 2];
        let mut velocity = [0.; 2];
        for i in 0..100 {
            let next = f.step(
                (1, i < 50),
                [if i < 50 { 15. } else { -15. }, 8.],
                current,
                0.04,
                0.95,
                100.,
            );
            for a in 0..2 {
                let v = (next[a] - current[a]) / 0.04;
                assert!(v.abs() <= 100.001);
                // The piecewise mechanical boundary can redirect velocity.
                // Check acceleration only if neither end touches that boundary.
                let interior = |p: [f32; 2]| {
                    let outward = [p[0] + p[0].signum() * 0.02, p[1] + p[1].signum() * 0.02];
                    envelope(outward) == outward
                };
                if interior(current) && interior(next) {
                    assert!(
                        (v - velocity[a]).abs() <= 28.01,
                        "tick={i} axis={a} previous={current:?} next={next:?} velocity={v} old_v={}",
                        velocity[a]
                    );
                }
                velocity[a] = v;
            }
            assert!(
                (next[0] - envelope(next)[0]).abs() < 0.001
                    && (next[1] - envelope(next)[1]).abs() < 0.001
            );
            current = next;
        }
    }
    #[test]
    fn neutral_return_starts_and_finishes_gently() {
        let t = Instant::now();
        let mut p = EasePose::default();
        let start = [35., 0.];
        assert_eq!(p.step(t, start, [0.; 2]), start);
        let a = p.step(t + std::time::Duration::from_millis(20), start, [0.; 2]);
        assert!((a[0] - start[0]).abs() < 0.05);
        let mut last = start;
        let mut max_speed = 0f32;
        for i in 1..200 {
            let x = p.step(t + std::time::Duration::from_millis(i * 10), last, [0.; 2]);
            max_speed = max_speed.max((x[0] - last[0]).abs() / 0.01);
            assert!(x[0] <= last[0]);
            last = x;
        }
        assert_eq!(last, [0.; 2]);
        assert!(max_speed <= 65.1);
    }
    #[test]
    fn search_trajectory_does_not_jump_ahead_during_blink() {
        let t = Instant::now();
        let mut p = EasePose::default();
        p.step(t, [0.; 2], [35., 0.]);
        let before = p.step(
            t + std::time::Duration::from_millis(100),
            [0.; 2],
            [35., 0.],
        );
        for ms in (120..=600).step_by(20) {
            p.pause(t + std::time::Duration::from_millis(ms));
        }
        let after = p.step(t + std::time::Duration::from_millis(620), before, [35., 0.]);
        let mut reference = EasePose::default();
        reference.step(t, [0.; 2], [35., 0.]);
        reference.step(
            t + std::time::Duration::from_millis(100),
            [0.; 2],
            [35., 0.],
        );
        let expected = reference.step(t + std::time::Duration::from_millis(120), before, [35., 0.]);
        assert_eq!(after, expected);
    }
    #[test]
    fn face_to_body_is_blended_before_accelerating() {
        let mut f = VisualServo::default();
        let mut current = [0.; 2];
        for _ in 0..10 {
            current = f.step((1, true), [0.; 2], current, 0.04, 0.95, 160.);
        }
        let first = f.step((1, false), [0., 18.], current, 0.04, 0.95, 160.);
        assert!(
            first[1] < 0.2,
            "body switch must not start as a hard saccade"
        );
        let mut last = first;
        for _ in 0..15 {
            last = f.step((1, false), [0., 18.], last, 0.04, 0.95, 160.);
        }
        assert!(last[1] > 5.);
    }
    #[test]
    fn moving_face_is_followed_without_large_steady_lag() {
        let mut f = VisualServo::default();
        let mut pos = 0f32;
        let mut hist = std::collections::VecDeque::from(vec![(0., 0.); 5]);
        let dt = 0.02;
        let mut errors = vec![];
        for i in 0..180 {
            let t = i as f32 * dt;
            let target = if t < 1. {
                0.
            } else {
                ((t - 1.) * 20.).min(25.)
            };
            let (seen, observed_target) = hist.pop_front().unwrap();
            let command = f.step(
                (1, true),
                [observed_target - seen, 0.],
                [pos, 0.],
                dt,
                0.95,
                160.,
            );
            pos += (1. - (-dt / 0.08).exp()) * (command[0] - pos);
            hist.push_back((pos, target));
            if t > 1.5 && t < 2.25 {
                errors.push((target - pos).abs());
            }
        }
        let mean = errors.iter().sum::<f32>() / errors.len() as f32;
        assert!(mean < 3., "moving error {mean}");
        assert!(
            (pos - 25.).abs() < 0.8,
            "must settle after person stops: {pos}"
        );
    }
}
