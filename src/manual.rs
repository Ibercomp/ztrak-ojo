use crate::motion_limits::MotionLimits;
use std::time::Instant;
#[derive(Clone, Copy)]
pub struct ManualInput {
    pub stick: [f32; 2],
    pub target: Option<[f32; 2]>,
    pub speed: f32,
    pub stamp: Instant,
    pub center: bool,
    /// Lid command in manual mode (0 open … 60 closed).
    pub lid: f32,
}
impl Default for ManualInput {
    fn default() -> Self {
        Self {
            stick: [0.; 2],
            target: None,
            speed: 8.,
            stamp: Instant::now(),
            center: false,
            lid: 0.,
        }
    }
}
#[derive(Default)]
pub struct ManualDrive {
    position: Option<[f32; 2]>,
    velocity: [f32; 2],
    last: Option<Instant>,
}
impl ManualDrive {
    pub fn step(
        &mut self,
        now: Instant,
        current: [f32; 2],
        input: ManualInput,
        limits: MotionLimits,
        paused: bool,
    ) -> [f32; 2] {
        let gap = self
            .last
            .map(|t| now.saturating_duration_since(t).as_secs_f32())
            .unwrap_or(0.);
        self.last = Some(now);
        if paused || gap > 0.15 || self.position.is_none() {
            self.position = Some(current);
            self.velocity = [0.; 2];
            return current;
        }
        let dt = gap.clamp(0., 0.05);
        let mut p = self.position.unwrap();
        let speed = input.speed.clamp(1., 60.);
        let fresh = now.saturating_duration_since(input.stamp).as_millis() < 150;
        let target = input
            .target
            .or(input.center.then_some([0.; 2]))
            .map(|p| limits.tracking(p));
        for i in 0..2 {
            let wanted = if !fresh {
                0.
            } else if let Some(target) = target {
                ((target[i] - p[i]) * 6.).clamp(-speed, speed)
            } else {
                input.stick[i].clamp(-1., 1.) * speed
            };
            let acceleration = if wanted.abs() < self.velocity[i].abs() {
                300.
            } else {
                180.
            };
            self.velocity[i] +=
                (wanted - self.velocity[i]).clamp(-acceleration * dt, acceleration * dt);
            let next = p[i] + self.velocity[i] * dt;
            p[i] = if let Some(target) = target.filter(|_| fresh) {
                if (target[i] - p[i]) * (target[i] - next) < 0. {
                    self.velocity[i] = 0.;
                    target[i]
                } else {
                    next
                }
            } else {
                next
            };
        }
        let old = self.position.unwrap();
        let out = limits.step(old, p, speed * dt);
        if dt > 0. {
            for i in 0..2 {
                self.velocity[i] = (out[i] - old[i]) / dt;
            }
        }
        self.position = Some(out);
        out
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    #[test]
    fn absolute_target_moves_to_clicked_position_and_stop_cancels_it() {
        let t = Instant::now();
        let mut d = ManualDrive::default();
        let mut p = [-20., 5.];
        let limits = MotionLimits::default();
        for n in 0..200 {
            let now = t + Duration::from_millis(n * 20);
            let q = d.step(
                now,
                p,
                ManualInput {
                    target: Some([12., -8.]),
                    speed: 35.,
                    stamp: now,
                    ..Default::default()
                },
                limits,
                false,
            );
            assert!((q[0] - p[0]).abs() <= 0.701 && (q[1] - p[1]).abs() <= 0.701);
            p = q;
        }
        assert!(
            (p[0] - 12.).abs() < 0.05 && (p[1] + 8.).abs() < 0.05,
            "{p:?}"
        );
        let now = t + Duration::from_secs(4);
        let hold = p;
        for n in 0..30 {
            let now = now + Duration::from_millis(n * 20);
            p = d.step(
                now,
                p,
                ManualInput {
                    target: Some(hold),
                    stamp: now,
                    speed: 35.,
                    ..Default::default()
                },
                limits,
                false,
            );
        }
        assert!((p[0] - hold[0]).abs() < 0.01 && (p[1] - hold[1]).abs() < 0.01);
    }
    #[test]
    fn slow_limited_and_stops_on_release_or_stale_input() {
        let t = Instant::now();
        let mut d = ManualDrive::default();
        let mut p = [0.; 2];
        let lim = MotionLimits::default();
        for n in 0..1000 {
            let now = t + Duration::from_millis(n * 20);
            let q = d.step(
                now,
                p,
                ManualInput {
                    stick: [1., 1.],
                    stamp: now,
                    ..Default::default()
                },
                lim,
                false,
            );
            assert!((q[0] - p[0]).abs() <= 0.161 && (q[1] - p[1]).abs() <= 0.161);
            assert_eq!(q, lim.tracking(q));
            p = q;
        }
        assert!(p[0] > 54.9);
        let fixed = p;
        for n in 1000..1030 {
            p = d.step(
                t + Duration::from_millis(n * 20),
                p,
                ManualInput {
                    stamp: t,
                    ..Default::default()
                },
                lim,
                false,
            );
        }
        assert!((p[0] - fixed[0]).abs() < 1.);
        let held = p;
        p = d.step(
            t + Duration::from_secs(40),
            held,
            ManualInput {
                stick: [-1.; 2],
                stamp: t + Duration::from_secs(40),
                ..Default::default()
            },
            lim,
            false,
        );
        assert_eq!(p, held);
    }
}
