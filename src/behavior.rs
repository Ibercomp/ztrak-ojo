//! Short reactions to a stable frontal face. This is theatrical attention,
//! not identity recognition or a claim that the person is making eye contact.
use crate::shared::Person;
use std::time::{Duration, Instant};

pub fn frontal(p: &Person) -> bool {
    let Some(k) = p.kpts else { return false };
    if [0, 1, 2].iter().any(|&i| k[i].c < 0.5) {
        return false;
    }
    let span = (k[1].x - k[2].x).abs();
    span > 8.
        && k[0].x >= k[1].x.min(k[2].x) - span * 0.1
        && k[0].x <= k[1].x.max(k[2].x) + span * 0.1
}
#[derive(Default)]
pub struct Curiosity {
    person: Option<u64>,
    anchor: Option<[f32; 2]>,
    stable_since: Option<Instant>,
    active_until: Option<Instant>,
    cooldown: Option<Instant>,
    reactions: u32,
    motion_sample: Option<(Instant, [f32; 2])>,
    motion_speed: f32,
}
impl Curiosity {
    pub fn reset(&mut self) {
        self.person = None;
        self.anchor = None;
        self.stable_since = None;
        self.active_until = None;
        self.motion_sample = None;
        self.motion_speed = 0.;
    }
    pub fn observe(&mut self, now: Instant, id: u64, bearing: [f32; 2], eligible: bool) {
        if !eligible {
            self.reset();
            return;
        }
        if let Some((t, p)) = self.motion_sample.filter(|_| self.person == Some(id)) {
            let dt = now.saturating_duration_since(t).as_secs_f32();
            if (0.005..=0.5).contains(&dt) {
                let speed = (bearing[0] - p[0]).hypot(bearing[1] - p[1]) / dt;
                self.motion_speed += (1. - (-dt / 0.25).exp()) * (speed - self.motion_speed);
            }
        }
        self.motion_sample = Some((now, bearing));
        if self.motion_speed > 5. {
            self.person = Some(id);
            self.anchor = Some(bearing);
            self.stable_since = Some(now);
            self.active_until = None;
            return;
        }
        let moved = self.person != Some(id)
            || self
                .anchor
                .is_none_or(|a| (a[0] - bearing[0]).hypot(a[1] - bearing[1]) > 3.);
        if moved {
            self.person = Some(id);
            self.anchor = Some(bearing);
            self.stable_since = Some(now);
            self.active_until = None;
            return;
        }
        if self.active_until.is_some_and(|t| now >= t) {
            self.active_until = None;
            self.stable_since = Some(now);
        }
        if self.active_until.is_none()
            && self.cooldown.is_none_or(|t| now >= t)
            && self
                .stable_since
                .is_some_and(|t| now.duration_since(t) >= Duration::from_millis(2500))
        {
            self.active_until = Some(now + Duration::from_millis(3500));
            self.cooldown = Some(now + Duration::from_millis(8500));
            self.reactions += 1;
        }
    }
    pub fn lid(&self, now: Instant) -> f32 {
        if self.active_until.is_some_and(|t| now < t) {
            if self.reactions % 2 == 1 { 26. } else { 30. }
        } else {
            0.
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn motion_inside_anchor_radius_does_not_trigger_curiosity() {
        let t = Instant::now();
        let mut c = Curiosity::default();
        for i in 0..120 {
            let now = t + Duration::from_millis(i * 50);
            let x = (i as f32 * 0.5).sin();
            c.observe(now, 1, [x, 0.], true);
            assert_eq!(c.lid(now), 0.);
        }
        for i in 120..220 {
            let now = t + Duration::from_millis(i * 50);
            c.observe(now, 1, [0.; 2], true);
        }
        assert!(c.lid(t + Duration::from_millis(10950)) > 0.);
    }
    #[test]
    fn curiosity_requires_stillness_and_releases_on_movement() {
        let t = Instant::now();
        let mut c = Curiosity::default();
        c.observe(t, 1, [5., 2.], true);
        c.observe(t + Duration::from_secs(2), 1, [5.2, 2.], true);
        assert_eq!(c.lid(t + Duration::from_secs(2)), 0.);
        c.observe(t + Duration::from_millis(2600), 1, [5.1, 2.], true);
        assert_eq!(c.lid(t + Duration::from_millis(2600)), 26.);
        c.observe(t + Duration::from_millis(2700), 1, [12., 2.], true);
        assert_eq!(c.lid(t + Duration::from_millis(2700)), 0.);
    }
    #[test]
    fn reaction_is_brief_and_profiles_or_new_people_do_not_trigger_it() {
        let t = Instant::now();
        let mut c = Curiosity::default();
        c.observe(t, 1, [0.; 2], true);
        c.observe(t + Duration::from_secs(3), 1, [0.; 2], true);
        assert_eq!(c.lid(t + Duration::from_secs(3)), 26.);
        assert_eq!(c.lid(t + Duration::from_secs(7)), 0.);
        c.observe(t + Duration::from_secs(6), 1, [0.; 2], false);
        assert_eq!(c.lid(t + Duration::from_secs(6)), 0.);
        c.observe(t + Duration::from_secs(10), 2, [0.; 2], true);
        assert_eq!(c.lid(t + Duration::from_secs(10)), 0.);
    }
}
