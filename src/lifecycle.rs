//! Timed eye behaviour, independent of camera/network threads. Test with a virtual clock.
use crate::geometry;
use std::time::{Duration, Instant};
#[derive(Clone, Copy, Debug)]
enum Phase {
    Watch,
    Recover(Option<Instant>),
    CenterSleep,
    Closing,
    Sleep(Instant),
    Opening(Option<Instant>),
    Scan(usize, Option<Instant>),
    LostScan(usize, Option<Instant>),
}
#[derive(Clone, Copy)]
pub struct Observation {
    pub angles: [f32; 2],
    pub time: Instant,
}
#[derive(Clone, Copy, Debug)]
pub struct Decision {
    pub pose: Option<[f32; 2]>,
    pub lid: f32,
    pub rest: bool,
    pub allow_face: bool,
    pub reset_focus: bool,
    pub pause_focus: bool,
    pub status: Option<&'static str>,
}
impl Default for Decision {
    fn default() -> Self {
        Self {
            pose: None,
            lid: 0.,
            rest: false,
            allow_face: true,
            reset_focus: false,
            pause_focus: false,
            status: None,
        }
    }
}
pub struct Lifecycle {
    pub limits: crate::motion_limits::MotionLimits,
    phase: Phase,
    last_seen: Instant,
    blocked_since: Option<Instant>,
    outside_seen: Option<Instant>,
    accept_after: Instant,
    search_anchor: [f32; 2],
    searched: bool,
    had_person: bool,
    search_after_recover: bool,
    observations_blocked: bool,
}
fn near(a: [f32; 2], b: [f32; 2]) -> bool {
    (a[0] - b[0]).abs() < 0.8 && (a[1] - b[1]).abs() < 0.8
}
fn outside(a: [f32; 2], limits: crate::motion_limits::MotionLimits) -> bool {
    let b = limits.envelope(a);
    (a[0] - b[0]).abs() > 1.2 || (a[1] - b[1]).abs() > 1.2
}
fn limit_reached(
    a: [f32; 2],
    requested: Option<[f32; 2]>,
    limits: crate::motion_limits::MotionLimits,
) -> bool {
    let x = limits.yaw(a[1]);
    let y = limits.pitch(a[0]);
    let at = [
        a[0] < x[0] + 0.8 || a[0] > x[1] - 0.8,
        a[1] < y[0] + 0.8 || a[1] > y[1] - 0.8,
    ];
    requested.map_or(at[0] || at[1], |r| {
        (at[0] && (r[0] - a[0]) * a[0].signum() > 1.2)
            || (at[1] && (r[1] - a[1]) * a[1].signum() > 1.2)
    })
}

impl Lifecycle {
    pub fn new(now: Instant) -> Self {
        Self {
            limits: Default::default(),
            phase: Phase::Watch,
            last_seen: now,
            blocked_since: None,
            outside_seen: None,
            accept_after: now - Duration::from_secs(1),
            search_anchor: [0.; 2],
            searched: false,
            had_person: false,
            search_after_recover: false,
            observations_blocked: false,
        }
    }
    pub fn idle_available(&self) -> bool {
        matches!(self.phase, Phase::Watch)
    }
    pub fn sleeping(&self) -> bool {
        matches!(self.phase, Phase::Closing | Phase::Sleep(_))
    }
    pub fn wake(&mut self, now: Instant) {
        *self = Self::new(now);
    }
    pub fn sleep_remaining(&self, now: Instant, after: f32) -> f32 {
        (after - now.saturating_duration_since(self.last_seen).as_secs_f32()).max(0.)
    }
    /// Camera occlusion prevents loss decisions, but must not block lid states.
    pub fn set_obscured(&mut self, obscured: bool) {
        self.observations_blocked = obscured;
    }
    #[allow(clippy::too_many_arguments)]
    pub fn step(
        &mut self,
        now: Instant,
        obs: Option<Observation>,
        current: [f32; 2],
        lid: f32,
        max_lid: f32,
        blinking: bool,
        camera_live: bool,
        sleep_enabled: bool,
        sleep_after: f32,
        sleep_for: f32,
        pause: f32,
    ) -> Decision {
        let mut d = Decision::default();
        if !sleep_enabled
            && !matches!(
                self.phase,
                Phase::Watch | Phase::Recover(_) | Phase::LostScan(_, _)
            )
        {
            self.phase = Phase::Watch;
            self.last_seen = now;
            d.reset_focus = true;
        }
        if !camera_live {
            self.last_seen = now;
            self.blocked_since = None;
            d.allow_face = false;
            match self.phase {
                Phase::Closing | Phase::Sleep(_) => {
                    d.pose = Some([0.; 2]);
                    d.lid = max_lid;
                    d.rest = true;
                    d.pause_focus = true;
                    d.status = Some("Durmiendo · esperando cámara");
                }
                Phase::Recover(_) | Phase::CenterSleep => {
                    d.pose = Some([0.; 2]);
                    d.rest = true;
                    d.pause_focus = true;
                }
                Phase::Opening(_) => {
                    d.pose = Some(current);
                    d.rest = true;
                    d.status = Some("Despertando…");
                }
                Phase::Scan(_, _) | Phase::LostScan(_, _) => {
                    d.pose = Some(current);
                    d.rest = true;
                    d.status = Some("¿Hay alguien? · mirando a ambos lados");
                }
                Phase::Watch => {}
            }
            return d;
        }
        let ready_face =
            obs.filter(|o| !outside(o.angles, self.limits) && o.time >= self.accept_after);
        match self.phase {
            Phase::Watch => {
                let unreachable = obs.is_some_and(|o| outside(o.angles, self.limits));
                if unreachable && limit_reached(current, obs.map(|o| o.angles), self.limits) {
                    self.outside_seen = Some(now);
                } else if obs.is_some() {
                    self.outside_seen = None;
                }
                let lost_at_edge = obs.is_none()
                    && self
                        .outside_seen
                        .is_some_and(|t| now.duration_since(t) < Duration::from_millis(1250));
                if !blinking
                    && !self.observations_blocked
                    && lost_at_edge
                    && limit_reached(current, None, self.limits)
                {
                    let start = *self.blocked_since.get_or_insert(now);
                    if now.duration_since(start) >= Duration::from_millis(750) {
                        self.phase = Phase::Recover(None);
                        self.search_after_recover = true;
                        self.blocked_since = None;
                        self.last_seen = now;
                        d.reset_focus = true;
                    }
                } else {
                    self.blocked_since = None;
                }
                // Saturation is not loss: retain a visible person at the stop.
                // Returning to neutral here caused endless reacquire/reset loops.
                if obs.is_some() {
                    self.last_seen = now;
                    self.had_person = true;
                    self.searched = false;
                } else if matches!(self.phase, Phase::Watch)
                    && !self.searched
                    && self.blocked_since.is_none()
                    && !blinking
                    && !self.observations_blocked
                    && now.duration_since(self.last_seen)
                        >= Duration::from_millis(if self.had_person { 650 } else { 2000 })
                {
                    self.search_anchor = current;
                    self.searched = true;
                    self.search_after_recover = false;
                    self.phase = Phase::LostScan(0, None);
                }
                if matches!(self.phase, Phase::Watch)
                    && sleep_enabled
                    && !blinking
                    && !self.observations_blocked
                    && now.duration_since(self.last_seen).as_secs_f32() >= sleep_after
                {
                    self.phase = Phase::CenterSleep;
                    d.reset_focus = true;
                }
            }
            Phase::Recover(ref mut settled) => {
                if near(current, [0.; 2]) && lid < 1. && !blinking {
                    let start = *settled.get_or_insert(now);
                    if now.duration_since(start).as_secs_f32() >= pause {
                        self.phase = if self.search_after_recover {
                            self.search_anchor = [0.; 2];
                            self.searched = true;
                            Phase::LostScan(0, None)
                        } else {
                            Phase::Watch
                        };
                        self.outside_seen = None;
                        d.reset_focus = true;
                    }
                } else {
                    *settled = None;
                }
            }
            Phase::LostScan(index, ref mut settled) => {
                if obs.is_some_and(|o| o.time >= self.accept_after) && !blinking {
                    self.phase = Phase::Watch;
                    self.last_seen = now;
                    self.searched = false;
                } else if !blinking
                    && near(
                        current,
                        self.limits.envelope(lost_pose(self.search_anchor, index)),
                    )
                    && (lid - search_lid(index, max_lid)).abs() < 1.
                {
                    let start = *settled.get_or_insert(now);
                    if now.duration_since(start) >= Duration::from_millis(400) {
                        self.phase = if index + 1 < SEARCH_STEPS {
                            Phase::LostScan(index + 1, None)
                        } else {
                            d.reset_focus = true;
                            self.search_after_recover = true;
                            Phase::Recover(None)
                        };
                    }
                } else {
                    *settled = None;
                }
            }
            Phase::CenterSleep => {
                if near(current, [0.; 2]) && lid < 1. && !blinking {
                    self.phase = Phase::Closing;
                }
            }
            Phase::Closing => {
                if lid >= max_lid - 1. && !blinking {
                    self.phase = Phase::Sleep(now + Duration::from_secs_f32(sleep_for));
                }
            }
            Phase::Sleep(until) => {
                if now >= until {
                    self.phase = Phase::Opening(None);
                    d.reset_focus = true;
                }
            }
            Phase::Opening(ref mut opened) => {
                if lid < 1. && !blinking {
                    let t = *opened.get_or_insert(now);
                    self.accept_after = t + Duration::from_millis(150);
                    if now.duration_since(t) >= Duration::from_millis(180) {
                        if ready_face.is_some_and(|o| o.time >= t + Duration::from_millis(150)) {
                            self.phase = Phase::Watch;
                            self.last_seen = now;
                        } else {
                            self.phase = Phase::Scan(0, None);
                        }
                    }
                } else {
                    *opened = None;
                }
            }
            Phase::Scan(index, ref mut settled) => {
                if ready_face.is_some() && !blinking {
                    self.phase = Phase::Watch;
                    self.last_seen = now;
                } else if near(current, self.limits.envelope(scan_pose(index)))
                    && !blinking
                    && (lid - search_lid(index, max_lid)).abs() < 1.
                {
                    let start = *settled.get_or_insert(now);
                    if now.duration_since(start) >= Duration::from_millis(700) {
                        self.phase = if index + 1 < SEARCH_STEPS {
                            Phase::Scan(index + 1, None)
                        } else {
                            Phase::CenterSleep
                        };
                        if index + 1 == SEARCH_STEPS {
                            d.reset_focus = true;
                        }
                    }
                } else {
                    *settled = None;
                }
            }
        }
        if sleep_enabled
            && obs.is_none()
            && !blinking
            && !self.observations_blocked
            && now.duration_since(self.last_seen).as_secs_f32() >= sleep_after
            && matches!(
                self.phase,
                Phase::Watch | Phase::Recover(_) | Phase::LostScan(_, _)
            )
        {
            self.phase = Phase::CenterSleep;
            d.reset_focus = true;
        }
        match self.phase {
            Phase::Watch => {
                if d.reset_focus {
                    d.pose = Some([0.; 2]);
                    d.rest = true;
                    d.allow_face = false;
                }
            }
            Phase::Recover(_) => {
                d.pose = Some([0.; 2]);
                d.rest = true;
                d.allow_face = false;
                d.pause_focus = true;
                d.status = Some("Fuera de alcance · al centro y pausa");
            }
            Phase::LostScan(index, _) => {
                d.pose = Some(self.limits.envelope(lost_pose(self.search_anchor, index)));
                d.lid = search_lid(index, max_lid);
                d.rest = false;
                d.allow_face = false;
                d.status = Some("¿Dónde te has metido? · buscando a ambos lados");
            }
            Phase::CenterSleep => {
                d.pose = Some([0.; 2]);
                d.rest = true;
                d.allow_face = false;
                d.pause_focus = true;
                d.status = Some("Me está entrando sueño…");
            }
            Phase::Closing | Phase::Sleep(_) => {
                d.pose = Some([0.; 2]);
                d.lid = max_lid;
                d.rest = true;
                d.allow_face = false;
                d.pause_focus = true;
                d.status = Some("Durmiendo · en un momento vuelvo a mirar");
            }
            Phase::Opening(_) => {
                d.pose = Some([0.; 2]);
                d.rest = true;
                d.allow_face = false;
                d.status = Some("Despertando…");
            }
            Phase::Scan(i, _) => {
                d.pose = Some(self.limits.envelope(scan_pose(i)));
                d.lid = search_lid(i, max_lid);
                d.rest = false;
                d.allow_face = false;
                d.status = Some("¿Hay alguien? · mirando a ambos lados");
            }
        }
        d
    }
}
const SEARCH_STEPS: usize = 8;
fn search_lid(i: usize, max_lid: f32) -> f32 {
    if (1..=3).contains(&i) {
        32_f32.min(max_lid)
    } else {
        0.
    }
}
fn lost_pose(anchor: [f32; 2], i: usize) -> [f32; 2] {
    let side = if anchor[0] < 0. { -1. } else { 1. };
    // First center vertically with the lid open. Then squint sideways;
    // reopen at neutral before looking up/down, keeping the pupil camera usable.
    let p = [
        [anchor[0], 0.],
        [55. * side, 0.],
        [-55. * side, 0.],
        [0., 0.],
        [0., 0.],
        [0., 25.],
        [0., -25.],
        [0., 0.],
    ][i];
    geometry::envelope(p)
}
fn scan_pose(i: usize) -> [f32; 2] {
    lost_pose([-1., 0.], i)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn step(
        l: &mut Lifecycle,
        t: Instant,
        o: Option<Observation>,
        p: [f32; 2],
        lid: f32,
    ) -> Decision {
        l.step(t, o, p, lid, 60., false, true, true, 30., 5., 2.)
    }
    #[test]
    fn edge_reset_waits_two_seconds_after_arrival() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        let o = Some(Observation {
            angles: [60., 0.],
            time: t,
        });
        assert!(step(&mut l, t, o, [55., 0.], 0.).allow_face);
        assert!(step(&mut l, t + Duration::from_millis(50), None, [55., 0.], 0.).allow_face);
        let d = step(&mut l, t + Duration::from_millis(850), None, [55., 0.], 0.);
        assert!(d.reset_focus && d.pause_focus);
        step(&mut l, t + Duration::from_secs(1), o, [0.; 2], 0.);
        assert!(!step(&mut l, t + Duration::from_millis(2999), o, [0.; 2], 0.).allow_face);
        assert!(step(&mut l, t + Duration::from_secs(3), o, [0.; 2], 0.).reset_focus);
    }
    #[test]
    fn brief_face_and_detector_dropout_at_edge_preserves_body_reacquisition() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        step(
            &mut l,
            t,
            Some(Observation {
                angles: [60., 0.],
                time: t,
            }),
            [55., 0.],
            0.,
        );
        for ms in [50, 150, 300, 450] {
            let d = step(&mut l, t + Duration::from_millis(ms), None, [55., 0.], 0.);
            assert!(!d.reset_focus && d.allow_face);
        }
        let now = t + Duration::from_millis(500);
        let d = step(
            &mut l,
            now,
            Some(Observation {
                angles: [50., -10.],
                time: now,
            }),
            [55., 0.],
            0.,
        );
        assert!(d.allow_face && !d.reset_focus && d.pose.is_none());
    }
    #[test]
    fn sleep_wake_scan_and_return_to_sleep() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        assert!(
            step(&mut l, t + Duration::from_secs(29), None, [0.; 2], 0.)
                .pose
                .is_some()
        );
        assert!(step(&mut l, t + Duration::from_secs(30), None, [0.; 2], 0.).rest);
        assert_eq!(
            step(&mut l, t + Duration::from_secs(31), None, [0.; 2], 0.).lid,
            60.
        );
        step(&mut l, t + Duration::from_secs(32), None, [0.; 2], 60.);
        assert_eq!(
            step(&mut l, t + Duration::from_secs(36), None, [0.; 2], 60.).lid,
            60.
        );
        assert_eq!(
            step(&mut l, t + Duration::from_secs(37), None, [0.; 2], 60.).lid,
            0.
        );
        step(&mut l, t + Duration::from_secs(38), None, [0.; 2], 0.);
        assert_eq!(
            step(&mut l, t + Duration::from_secs(39), None, [0.; 2], 0.).pose,
            Some(scan_pose(0))
        );
        for i in 0..SEARCH_STEPS {
            let sec = 40 + 2 * i as u64;
            step(
                &mut l,
                t + Duration::from_secs(sec),
                None,
                scan_pose(i),
                search_lid(i, 60.),
            );
            step(
                &mut l,
                t + Duration::from_secs(sec + 1),
                None,
                scan_pose(i),
                search_lid(i, 60.),
            );
        }
        assert_eq!(
            step(&mut l, t + Duration::from_secs(57), None, [0.; 2], 0.).lid,
            60.
        );
    }
    #[test]
    fn waking_accepts_new_face_but_not_closed_camera_frame() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        l.phase = Phase::Opening(None);
        step(&mut l, t, None, [0.; 2], 0.);
        let old = Some(Observation {
            angles: [5., 0.],
            time: t - Duration::from_secs(1),
        });
        assert!(!step(&mut l, t + Duration::from_millis(200), old, [0.; 2], 0.).allow_face);
        let fresh = Some(Observation {
            angles: [5., 0.],
            time: t + Duration::from_millis(250),
        });
        assert!(step(&mut l, t + Duration::from_millis(300), fresh, [0.; 2], 0.).allow_face);
    }
    #[test]
    fn never_sleep_on_a_camera_outage() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        assert!(
            l.step(
                t + Duration::from_secs(40),
                None,
                [0.; 2],
                0.,
                60.,
                false,
                false,
                true,
                30.,
                5.,
                2.
            )
            .pose
            .is_none()
        );
    }
    #[test]
    fn visible_person_at_limit_is_never_reset_or_put_to_sleep() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        for i in 0..1000 {
            let now = t + Duration::from_millis(i * 50);
            let obs = Some(Observation {
                angles: [45., -35.],
                time: now,
            });
            let d = step(&mut l, now, obs, [30., -25.], 0.);
            assert!(d.allow_face && !d.reset_focus && !d.rest);
        }
    }
    #[test]
    fn brief_dropout_and_blink_at_limit_do_not_reset_identity() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        let obs = Some(Observation {
            angles: [60., 0.],
            time: t,
        });
        step(&mut l, t, obs, [55., 0.], 0.);
        assert!(!step(&mut l, t + Duration::from_millis(50), None, [55., 0.], 0.).reset_focus);
        assert!(!step(&mut l, t + Duration::from_millis(180), obs, [55., 0.], 0.).reset_focus);
        for ms in [200, 400, 600] {
            let d = l.step(
                t + Duration::from_millis(ms),
                None,
                [55., 0.],
                60.,
                60.,
                true,
                true,
                true,
                30.,
                5.,
                2.,
            );
            assert!(!d.reset_focus);
        }
        assert!(step(&mut l, t + Duration::from_millis(650), obs, [55., 0.], 0.).allow_face);
    }
    #[test]
    fn disappearance_searches_then_centers_waits_and_sleeps_on_original_deadline() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        step(
            &mut l,
            t,
            Some(Observation {
                angles: [10., 0.],
                time: t,
            }),
            [10., 0.],
            0.,
        );
        let mut current = [10., 0.];
        let mut scans = 0;
        let mut lid = 0.;
        let mut resets = 0;
        for i in 7..300 {
            let d = step(
                &mut l,
                t + Duration::from_millis(i * 100),
                None,
                current,
                lid,
            );
            if d.status
                .is_some_and(|s| s.contains("buscando a ambos lados"))
            {
                scans += 1;
                assert!(!d.rest, "Search must allow automatic blinks");
            }
            resets += usize::from(d.reset_focus);
            current = d.pose.unwrap_or(current);
            lid = d.lid;
            if lid > 0. {
                assert_eq!(current[1], 0.);
            }
        }
        assert!(scans > 30 && resets >= 2, "must keep searching until sleep");
        assert!(step(&mut l, t + Duration::from_secs(30), None, current, 0.).rest);
        assert_eq!(
            step(&mut l, t + Duration::from_secs(31), None, [0.; 2], 0.).lid,
            60.
        );
    }

    #[test]
    fn local_search_reacquires_body_or_face_immediately() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        step(
            &mut l,
            t,
            Some(Observation {
                angles: [10., 0.],
                time: t,
            }),
            [10., 0.],
            0.,
        );
        step(&mut l, t + Duration::from_secs(1), None, [10., 0.], 0.);
        let now = t + Duration::from_millis(1100);
        let d = step(
            &mut l,
            now,
            Some(Observation {
                angles: [5., 0.],
                time: now,
            }),
            [7., 0.],
            0.,
        );
        assert!(d.allow_face && !d.rest);
    }
    #[test]
    fn empty_start_searches_and_squint_reopens_before_vertical_motion() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        assert!(
            step(&mut l, t + Duration::from_secs(1), None, [12., 20.], 0.)
                .pose
                .is_none()
        );
        let d = step(&mut l, t + Duration::from_secs(2), None, [12., 20.], 0.);
        assert_eq!(d.pose, Some([12., 0.]));
        assert_eq!(d.lid, 0.);
        let mut now = t + Duration::from_secs(3);
        for i in 0..SEARCH_STEPS {
            let pose = lost_pose([12., 20.], i);
            let lid = search_lid(i, 60.);
            let d = step(&mut l, now, None, pose, lid);
            assert_eq!(d.lid, lid);
            assert!(!d.rest);
            if lid > 0. {
                assert_eq!(pose[1], 0.);
            }
            now += Duration::from_millis(450);
            step(&mut l, now, None, pose, lid);
            now += Duration::from_millis(50);
        }
        assert!(matches!(l.phase, Phase::Recover(_)));
        assert_eq!(
            l.sleep_remaining(now, 30.),
            30. - now.duration_since(t).as_secs_f32()
        );
    }
    #[test]
    fn squinted_wake_search_accepts_a_person_without_waiting_for_open_lid() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        l.phase = Phase::Scan(2, None);
        let d = step(
            &mut l,
            t,
            Some(Observation {
                angles: [10., 0.],
                time: t,
            }),
            [20., 0.],
            32.,
        );
        assert!(d.allow_face && !d.rest && d.pose.is_none());
        assert_eq!(d.lid, 0.);
    }
    #[test]
    fn closed_pupil_camera_does_not_block_sleep_timer_or_waking() {
        let t = Instant::now();
        let mut l = Lifecycle::new(t);
        l.phase = Phase::Closing;
        l.set_obscured(true);
        let d = step(&mut l, t, None, [0.; 2], 60.);
        assert_eq!(d.lid, 60.);
        assert!(matches!(l.phase, Phase::Sleep(_)));
        assert_eq!(
            step(&mut l, t + Duration::from_secs(4), None, [0.; 2], 60.).lid,
            60.
        );
        assert_eq!(
            step(&mut l, t + Duration::from_secs(5), None, [0.; 2], 60.).lid,
            0.
        );
        assert!(matches!(l.phase, Phase::Opening(_)));
    }
}
