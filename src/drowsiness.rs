//! Exhibition behaviour: still visitors become boring; movement wakes the eye.
use crate::lifecycle::Decision;
use std::time::{Duration, Instant};
#[derive(Clone, Copy)]
pub struct Sample {
    pub time: Instant,
    pub id: u64,
    pub face: bool,
    pub bearing: [f32; 2],
    pub size: f32,
}
#[derive(Clone, Copy)]
enum Phase {
    Awake,
    Center([f32; 2]),
    Fading(Instant, [f32; 2]),
    Sleeping([f32; 2]),
    Waking(Instant, [f32; 2]),
}
pub struct Drowsiness {
    phase: Phase,
    anchor: Option<Sample>,
    filtered: Option<Sample>,
    last_sample: Option<Instant>,
    last_seen: Instant,
    activity: Instant,
    movement_since: Option<Instant>,
}
impl Drowsiness {
    pub fn new(now: Instant) -> Self {
        Self {
            phase: Phase::Awake,
            anchor: None,
            filtered: None,
            last_sample: None,
            last_seen: now,
            activity: now,
            movement_since: None,
        }
    }
    pub fn active(&self) -> bool {
        !matches!(self.phase, Phase::Awake)
    }
    pub fn reset(&mut self, now: Instant) {
        *self = Self::new(now);
    }
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        now: Instant,
        sample: Option<Sample>,
        current: [f32; 2],
        lid: f32,
        after: f32,
        closing: f32,
        slit: f32,
        image_motion: bool,
    ) -> Option<Decision> {
        let mut moved = false;
        if let Some(mut s) = sample.filter(|s| self.last_sample.is_none_or(|t| s.time > t)) {
            self.last_sample = Some(s.time);
            self.last_seen = now;
            let new_anchor = self.anchor.is_none_or(|a| a.id != s.id || a.face != s.face);
            if new_anchor {
                let new_person = self.anchor.is_none_or(|a| a.id != s.id);
                self.anchor = Some(s);
                self.filtered = Some(s);
                self.movement_since = None;
                if new_person && !self.active() {
                    self.activity = now;
                }
            } else {
                let old = self.filtered.unwrap();
                for i in 0..2 {
                    s.bearing[i] = old.bearing[i] + 0.25 * (s.bearing[i] - old.bearing[i]);
                }
                s.size = old.size + 0.25 * (s.size - old.size);
                self.filtered = Some(s);
                let a = self.anchor.unwrap();
                let distance = (s.bearing[0] - a.bearing[0]).hypot(s.bearing[1] - a.bearing[1]);
                let scale_change = (s.size / a.size.max(1.)).ln().abs();
                if distance > 2.0 || scale_change > 0.18 {
                    let since = *self.movement_since.get_or_insert(now);
                    if now.duration_since(since) >= Duration::from_millis(200) {
                        moved = true;
                        self.activity = now;
                        self.anchor = Some(s);
                        self.movement_since = None;
                    }
                } else if distance < 1.0 && scale_change < 0.09 {
                    self.movement_since = None;
                }
            }
        }
        if matches!(self.phase, Phase::Awake) {
            // Optical motion from the moving pupil is not visitor activity.
            // Use compensated target displacement while awake; optical motion
            // wakes a resting eye whose camera is held still.
            if now.duration_since(self.last_seen) > Duration::from_secs(1) {
                self.activity = now;
                self.anchor = None;
                self.filtered = None;
            } else if self.anchor.is_some()
                && now.duration_since(self.activity).as_secs_f32() >= after
            {
                self.phase = Phase::Center([current[0], 0.]);
            }
        } else if (image_motion || moved)
            && !matches!(self.phase, Phase::Center(_) | Phase::Waking(_, _))
        {
            self.phase = Phase::Waking(now, current);
        }
        match self.phase {
            Phase::Center(p) if (current[0] - p[0]).hypot(current[1]) < 0.8 && lid < 1. => {
                self.phase = Phase::Fading(now, p);
                // Re-anchor after the eye's own centering movement.
                self.anchor = None;
                self.filtered = None;
                self.movement_since = None;
            }
            Phase::Fading(t, p) if now.duration_since(t).as_secs_f32() >= closing => {
                self.phase = Phase::Sleeping(p)
            }
            Phase::Sleeping(p) if now.duration_since(self.last_seen) > Duration::from_secs(5) => {
                // If the slit hides the detector, peek instead of remaining blind forever.
                self.phase = Phase::Waking(now, p);
            }
            Phase::Waking(t, _)
                if now.duration_since(t) >= Duration::from_millis(600) && lid < 1. =>
            {
                self.reset(now);
                return None;
            }
            _ => {}
        }
        let (pose, close, status) = match self.phase {
            Phase::Awake => return None,
            Phase::Center(p) => (p, 0., "Me aburro…"),
            Phase::Fading(t, p) => {
                let x = (now.duration_since(t).as_secs_f32() / closing).clamp(0., 1.);
                // Gradual droop, slowing down at both ends.
                (
                    p,
                    slit * x * x * x * (10. + x * (-15. + 6. * x)),
                    "Qué sueño… se me cierra el ojo",
                )
            }
            Phase::Sleeping(p) => (p, slit, "Zzz… vigilando por la rendija"),
            Phase::Waking(_, p) => (p, 0., "¡Eh! ¿Qué se mueve por ahí?"),
        };
        Some(Decision {
            pose: Some(pose),
            lid: close,
            rest: true,
            allow_face: false,
            pause_focus: false,
            status: Some(status),
            ..Default::default()
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample(t: Instant, x: f32) -> Option<Sample> {
        Some(Sample {
            time: t,
            id: 1,
            face: true,
            bearing: [x, 0.],
            size: 200.,
        })
    }
    #[test]
    fn visitor_stillness_droops_gradually_and_motion_wakes_without_losing_vision() {
        let t = Instant::now();
        let mut d = Drowsiness::new(t);
        let mut lid = 0.;
        for i in 0..=300 {
            let now = t + Duration::from_millis(i * 100);
            let r = d.step(
                now,
                sample(now, 0.2 * (i as f32).sin()),
                [8., 0.],
                lid,
                18.,
                8.,
                32.,
                false,
            );
            if i < 180 {
                assert!(r.is_none());
            } else if let Some(r) = r {
                assert!(r.lid >= lid - 0.01);
                assert!(r.lid <= 32.);
                lid = r.lid;
            }
        }
        assert_eq!(lid, 32.);
        let now = t + Duration::from_secs(31);
        let r = d
            .step(now, sample(now, 0.), [8., 0.], 32., 18., 8., 32., true)
            .unwrap();
        assert_eq!(r.lid, 0.);
        assert!(r.rest && !r.pause_focus);
        assert!(
            d.step(
                now + Duration::from_secs(1),
                sample(now + Duration::from_secs(1), 0.),
                [8., 0.],
                0.,
                18.,
                8.,
                32.,
                false
            )
            .is_none()
        );
    }
    #[test]
    fn moving_visitor_does_not_fall_asleep_but_face_body_switch_is_not_activity() {
        let t = Instant::now();
        let mut d = Drowsiness::new(t);
        for i in 0..600 {
            let now = t + Duration::from_millis(i * 100);
            assert!(
                d.step(
                    now,
                    sample(now, (i as f32 * 0.1).sin() * 8.),
                    [0.; 2],
                    0.,
                    18.,
                    8.,
                    32.,
                    false
                )
                .is_none()
            );
        }
        let mut d = Drowsiness::new(t);
        let mut became_drowsy = false;
        for i in 0..200 {
            let now = t + Duration::from_millis(i * 100);
            let mut s = sample(now, 0.).unwrap();
            s.face = i < 150;
            became_drowsy |= d
                .step(now, Some(s), [0.; 2], 0., 18., 8., 32., false)
                .is_some();
        }
        assert!(became_drowsy);
    }
    #[test]
    fn no_person_is_left_to_the_absence_state_machine() {
        let t = Instant::now();
        let mut d = Drowsiness::new(t);
        for i in 0..600 {
            assert!(
                d.step(
                    t + Duration::from_millis(i * 100),
                    None,
                    [0.; 2],
                    0.,
                    18.,
                    8.,
                    32.,
                    false
                )
                .is_none()
            );
        }
    }
    #[test]
    fn duplicate_inference_frames_do_not_confirm_movement() {
        let t = Instant::now();
        let mut d = Drowsiness::new(t);
        d.step(t, sample(t, 0.), [0.; 2], 0., 18., 8., 32., false);
        for i in 1..100 {
            d.step(
                t + Duration::from_millis(i * 5),
                sample(t, 20.),
                [0.; 2],
                0.,
                18.,
                8.,
                32.,
                false,
            );
        }
        assert!(d.movement_since.is_none());
    }
    #[test]
    fn brief_link_or_blink_gaps_and_camera_motion_do_not_reset_stillness() {
        let t = Instant::now();
        let mut d = Drowsiness::new(t);
        let mut entered = false;
        for i in 0..190 {
            let now = t + Duration::from_millis(i * 100);
            // 400 ms of missing telemetry/obscured pupil every five seconds.
            let s = if i % 50 >= 45 && i % 50 < 49 {
                None
            } else {
                sample(now, 0.1 * (i as f32).sin())
            };
            let r = d.step(now, s, [0.; 2], 0., 18., 8., 32., true);
            if i < 180 {
                assert!(r.is_none());
            } else {
                entered |= r.is_some();
            }
        }
        assert!(entered, "own camera motion must not prevent boredom");
    }
}
