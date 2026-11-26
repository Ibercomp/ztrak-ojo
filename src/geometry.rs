//! Projection is independent of capture, UI and networking.
use crate::config::{DepthMode, MountConfig, MountMode};
use crate::shared::Person;

#[derive(Clone, Copy, Debug)]
pub struct Target {
    pub pixel: [f32; 2],
    pub depth: f32,
    pub estimated: bool,
    pub face: bool,
}
#[derive(Clone, Debug)]
pub struct Aim {
    pub person: u64,
    pub pixel: [f32; 2],
    pub angles: [f32; 2],
    pub raw_angles: [f32; 2],
    pub depth: f32,
    pub estimated: bool,
    pub uncertainty_deg: f32,
    pub limited: bool,
    pub face: bool,
}
pub fn focal(width: u32, fov: f32) -> f32 {
    width as f32 / (2. * (fov.to_radians() / 2.).tan())
}
pub fn target(p: &Person, w: u32, h: u32, m: &MountConfig) -> Option<Target> {
    if !p.visible {
        return None;
    }
    let mut pixel = [0.; 2];
    let mut face = false;
    let mut depth = m.depth_m;
    let mut estimated = false;
    if let Some(k) = p.kpts {
        let good = |i: usize| {
            k[i].c >= 0.4
                && k[i].x > 1.
                && k[i].x < w as f32 - 1.
                && k[i].y > 1.
                && k[i].y < h as f32 - 1.
        };
        if good(1) && good(2) {
            pixel = [(k[1].x + k[2].x) * 0.5, (k[1].y + k[2].y) * 0.5];
            face = true;
        } else if good(0) {
            pixel = [k[0].x, k[0].y];
            face = true;
        } else {
            // A passer-by need not face us: one eye/ear still locates the head.
            let head: Vec<_> = (1..5).filter(|&i| good(i)).collect();
            if !head.is_empty() {
                pixel = [
                    head.iter().map(|&i| k[i].x).sum::<f32>() / head.len() as f32,
                    head.iter().map(|&i| k[i].y).sum::<f32>() / head.len() as f32,
                ];
                face = true;
            }
        }

        // Shoulder ruler only when facial landmarks provide frontal evidence.
        // With a profile/back view use the configured depth, not a fictitious jump.
        let frontal = good(0)
            && good(1)
            && good(2)
            && (k[1].x - k[2].x).abs() > 8.
            && k[0].x >= k[1].x.min(k[2].x) - 4.
            && k[0].x <= k[1].x.max(k[2].x) + 4.;
        if m.depth_mode == DepthMode::Shoulders && frontal && good(5) && good(6) {
            let span = (k[5].x - k[6].x).hypot(k[5].y - k[6].y);
            // Very narrow/occluded shoulders are not a trustworthy ruler.
            if span > 35. && (k[5].x - k[6].x).abs() > span * 0.7 {
                depth = (focal(w, m.horizontal_fov) * m.shoulder_width_m / span).clamp(0.4, 12.);
                estimated = true;
            }
        }
    }
    if !face {
        // Keep following this same person when their head is occluded or turned
        // away. Use only the visible detector box, never a remembered body.
        if !p.bbox.iter().all(|v| v.is_finite()) {
            return None;
        }
        let [x0, y0, x1, y1] = [
            p.bbox[0].max(0.),
            p.bbox[1].max(0.),
            p.bbox[2].min(w as f32),
            p.bbox[3].min(h as f32),
        ];
        if x1 <= x0 || y1 <= y0 {
            return None;
        }
        pixel = [(x0 + x1) * 0.5, (y0 + y1) * 0.5];
    }
    Some(Target {
        pixel,
        depth,
        estimated,
        face,
    })
}
fn rotate(p: [f32; 3], ypr: [f32; 3]) -> [f32; 3] {
    let (sy, cy) = ypr[0].to_radians().sin_cos();
    let (sp, cp) = ypr[1].to_radians().sin_cos();
    let (sr, cr) = ypr[2].to_radians().sin_cos();
    let (x, y) = (cr * p[0] - sr * p[1], sr * p[0] + cr * p[1]);
    let (y, z) = (cp * y + sp * p[2], -sp * y + cp * p[2]);
    [cy * x + sy * z, y, -sy * x + cy * z]
}
pub fn ray(pixel: [f32; 2], w: u32, h: u32, fov: f32) -> [f32; 3] {
    let f = focal(w, fov);
    [
        (pixel[0] - w as f32 * 0.5) / f,
        (h as f32 * 0.5 - pixel[1]) / f,
        1.,
    ]
}
fn angles(p: [f32; 3]) -> Option<[f32; 2]> {
    if p[2] <= 0.05 || !p.iter().all(|v| v.is_finite()) {
        return None;
    }
    Some([
        p[0].atan2(p[2]).to_degrees(),
        p[1].atan2(p[0].hypot(p[2])).to_degrees(),
    ])
}
pub fn envelope(mut a: [f32; 2]) -> [f32; 2] {
    a[1] = a[1].clamp(-25., 25.);
    let cap = 55. - 2. * a[1].abs().min(15.) / 3. - 1.5 * (a[1].abs() - 15.).clamp(0., 10.);
    a[0] = a[0].clamp(-cap, cap);
    a
}
/// Preserve lateral reach by giving up elevation when the coupled mechanism
/// cannot provide both. Manual/demo poses keep the ordinary safety envelope.
pub fn tracking_envelope(a: [f32; 2]) -> [f32; 2] {
    let yaw = a[0].clamp(-55., 55.);
    let pitch_cap = if yaw.abs() <= 30. {
        25.
    } else if yaw.abs() <= 45. {
        25. - (yaw.abs() - 30.) / 1.5
    } else {
        (55. - yaw.abs()) * 1.5
    };
    envelope([yaw, a[1].clamp(-pitch_cap, pitch_cap)])
}
/// The feasible envelope is convex. Interpolating toward the lateral-priority
/// projection keeps both axes inside it without exceeding the speed budget.
pub fn tracking_step(from: [f32; 2], requested: [f32; 2], max_step: f32) -> [f32; 2] {
    let dst = tracking_envelope(requested);
    let delta = [dst[0] - from[0], dst[1] - from[1]];
    let extent = delta[0].abs().max(delta[1].abs());
    let scale = if extent > max_step {
        max_step / extent
    } else {
        1.
    };
    envelope([from[0] + delta[0] * scale, from[1] + delta[1] * scale])
}
pub fn external(pixel: [f32; 2], w: u32, h: u32, depth: f32, m: &MountConfig) -> Option<[f32; 2]> {
    let r = ray(pixel, w, h, m.horizontal_fov);
    let mut p = rotate([r[0] * depth, r[1] * depth, depth], m.camera_ypr);
    for (i, v) in p.iter_mut().enumerate() {
        *v += m.camera_xyz[i];
    }
    angles(p)
}
pub fn aim(p: &Person, w: u32, h: u32, m: &MountConfig, current: [f32; 2]) -> Option<Aim> {
    let t = target(p, w, h, m)?;
    let (mut a, uncertainty) = match m.mode {
        MountMode::External => {
            let a = external(t.pixel, w, h, t.depth, m)?;
            let mut u = 0f32;
            for scale in [0.65, 1.5] {
                if let Some(b) = external(t.pixel, w, h, t.depth * scale, m) {
                    u = u.max((b[0] - a[0]).abs()).max((b[1] - a[1]).abs());
                }
            }
            (a, u)
        }
        MountMode::Pupil => {
            let e = angles(rotate(ray(t.pixel, w, h, m.horizontal_fov), m.camera_ypr))?;
            let mut a = current;
            for i in 0..2 {
                let error = e[i] + m.trim[i];
                // Continuous at the deadband edge: no one-degree step when
                // a noisy landmark moves across the quiet zone.
                let correction = error.signum() * (error.abs() - m.deadband_deg).max(0.);
                a[i] += correction * m.pupil_gain * if m.invert[i] { -1. } else { 1. };
            }
            (a, 0.)
        }
    };
    if m.mode == MountMode::External {
        for (i, v) in a.iter_mut().enumerate() {
            *v = *v * if m.invert[i] { -1. } else { 1. } + m.trim[i];
        }
    }
    let bounded = m.limits.closest(a);
    Some(Aim {
        person: p.id,
        pixel: t.pixel,
        angles: bounded,
        raw_angles: a,
        depth: t.depth,
        estimated: t.estimated,
        uncertainty_deg: uncertainty,
        limited: bounded != a,
        face: t.face,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    fn face_at(pixel: [f32; 2]) -> Person {
        let mut k = [crate::shared::Kpt::default(); 17];
        k[0] = crate::shared::Kpt {
            x: pixel[0],
            y: pixel[1],
            c: 0.95,
        };
        Person {
            id: 1,
            bbox: [0., 0., 1280., 720.],
            score: 1.,
            kpts: Some(k),
            focus: true,
            quality: 1.,
            age: 4,
            visible: true,
        }
    }
    #[test]
    fn mounted_iris_corrects_both_axes_towards_face() {
        let mut m = MountConfig::default();
        m.mode = MountMode::Pupil;
        m.invert = [true, true];
        m.camera_ypr[2] = -15.;
        // Measured background displacement for +4° commands at 640x360:
        // yaw ~[22,-6], pitch ~[-5,-23]. Both signs must be reversed.
        for p in [[740., 360.], [640., 260.], [540., 360.], [640., 460.]] {
            let a = aim(&face_at(p), 1280, 720, &m, [0., 0.]).unwrap();
            let error = [p[0] - 640., p[1] - 360.];
            let next = [
                error[0] + 11. * a.angles[0] - 2.5 * a.angles[1],
                error[1] - 3. * a.angles[0] - 11.5 * a.angles[1],
            ];
            assert!(next[0].hypot(next[1]) < error[0].hypot(error[1]));
        }
    }
    #[test]
    fn pupil_deadband_has_no_command_jump() {
        let mut m = MountConfig::default();
        m.mode = MountMode::Pupil;
        let pixel = |angle: f32| {
            [
                640. + focal(1280, m.horizontal_fov) * angle.to_radians().tan(),
                360.,
            ]
        };
        let before = aim(
            &face_at(pixel(m.deadband_deg - 0.001)),
            1280,
            720,
            &m,
            [5., 0.],
        )
        .unwrap();
        let after = aim(
            &face_at(pixel(m.deadband_deg + 0.001)),
            1280,
            720,
            &m,
            [5., 0.],
        )
        .unwrap();
        approx(before.angles[0], 5.);
        assert!((after.angles[0] - before.angles[0]).abs() < 0.002);
    }
    #[test]
    fn display_mirror_and_physical_direction_are_independent() {
        let mut cfg = crate::config::Config::default();
        let mut k = [crate::shared::Kpt::default(); 17];
        k[0] = crate::shared::Kpt {
            x: 720.,
            y: 320.,
            c: 0.95,
        };
        let p = Person {
            id: 1,
            bbox: [600., 200., 900., 700.],
            score: 1.,
            kpts: Some(k),
            focus: true,
            quality: 1.,
            age: 4,
            visible: true,
        };
        for mode in [MountMode::External, MountMode::Pupil] {
            cfg.mount.mode = mode;
            cfg.mount.invert = [false, false];
            cfg.mirror_preview = false;
            let current = [10., 0.];
            let normal = aim(&p, 1280, 720, &cfg.mount, current).unwrap();
            cfg.mirror_preview = true;
            let mirrored = aim(&p, 1280, 720, &cfg.mount, current).unwrap();
            assert_eq!(normal.angles, mirrored.angles);
            assert_eq!(normal.pixel, mirrored.pixel);
            cfg.mount.invert[0] = true;
            let reversed = aim(&p, 1280, 720, &cfg.mount, current).unwrap();
            let origin = if mode == MountMode::Pupil {
                current[0]
            } else {
                0.
            };
            approx(normal.angles[0] - origin, -(reversed.angles[0] - origin));
            approx(normal.angles[1], reversed.angles[1]);
        }
    }
    fn approx(a: f32, b: f32) {
        assert!((a - b).abs() < 0.001, "{a} != {b}");
    }
    #[test]
    fn translation_and_depth() {
        let mut m = MountConfig::default();
        m.camera_xyz = [0.5, 0., 0.];
        let a = external([640., 360.], 1280, 720, 2., &m).unwrap();
        approx(a[0], (0.25f32).atan().to_degrees());
        approx(a[1], 0.);
        assert!(external([640., 360.], 1280, 720, 1., &m).unwrap()[0] > a[0]);
    }
    #[test]
    fn orientation() {
        let mut m = MountConfig::default();
        m.camera_ypr = [20., 10., 0.];
        let a = external([640., 360.], 1280, 720, 2., &m).unwrap();
        approx(a[0], 20.);
        approx(a[1], 10.);
    }
    #[test]
    fn image_axes() {
        let m = MountConfig::default();
        assert!(
            external([800., 200.], 1280, 720, 2., &m)
                .unwrap()
                .iter()
                .all(|v| *v > 0.)
        );
    }
    #[test]
    fn lateral_tracking_uses_all_travel_by_lowering_elevation() {
        for side in [-1., 1.] {
            for vertical in [-1., 1.] {
                assert_eq!(
                    tracking_envelope([side * 60., vertical * 25.]),
                    [side * 55., 0.]
                );
                assert_eq!(
                    tracking_envelope([side * 45., vertical * 25.]),
                    [side * 45., vertical * 15.]
                );
                assert_eq!(
                    tracking_envelope([side * 30., vertical * 25.]),
                    [side * 30., vertical * 25.]
                );
                let mut p = [side * 30., vertical * 25.];
                for _ in 0..200 {
                    let next = tracking_step(p, [side * 55., vertical * 25.], 0.5);
                    assert_eq!(next, envelope(next));
                    assert!((next[0] - p[0]).abs() <= 0.501 && (next[1] - p[1]).abs() <= 0.501);
                    p = next;
                }
                assert!((p[0] - side * 55.).abs() < 0.01 && p[1].abs() < 0.01);
            }
        }
    }
    #[test]
    fn limits_and_behind() {
        assert_eq!(envelope([60., 0.]), [55., 0.]);
        assert_eq!(envelope([-60., 0.]), [-55., 0.]);
        assert_eq!(envelope([60., 6.]), [51., 6.]);
        assert_eq!(envelope([-45., 15.]), [-45., 15.]);
        assert_eq!(envelope([45., 20.]), [37.5, 20.]);
        assert_eq!(envelope([45., 25.]), [30., 25.]);
        let mut m = MountConfig::default();
        m.camera_xyz[2] = -3.;
        assert!(external([640., 360.], 1280, 720, 2., &m).is_none());
    }
    #[test]
    fn pupil_centred_does_not_recentre_servo() {
        let mut m = MountConfig::default();
        m.mode = MountMode::Pupil;
        let mut k = [crate::shared::Kpt::default(); 17];
        k[0] = crate::shared::Kpt {
            x: 640.,
            y: 360.,
            c: 0.95,
        };
        let p = Person {
            id: 1,
            bbox: [600., 344., 680., 444.],
            score: 1.,
            kpts: Some(k),
            focus: true,
            quality: 1.,
            age: 4,
            visible: true,
        };
        let a = aim(&p, 1280, 720, &m, [18., -5.]).unwrap();
        approx(a.angles[0], 18.);
        approx(a.angles[1], -5.);
    }
    #[test]
    fn profile_uses_visible_head_and_not_shoulder_depth() {
        let mut m = MountConfig::default();
        m.depth_mode = DepthMode::Shoulders;
        m.depth_m = 3.;
        let mut k = [crate::shared::Kpt::default(); 17];
        k[3] = crate::shared::Kpt {
            x: 700.,
            y: 150.,
            c: 0.9,
        };
        k[5] = crate::shared::Kpt {
            x: 680.,
            y: 240.,
            c: 0.9,
        };
        k[6] = crate::shared::Kpt {
            x: 730.,
            y: 240.,
            c: 0.9,
        };
        let p = Person {
            id: 1,
            bbox: [650., 100., 750., 650.],
            score: 0.9,
            kpts: Some(k),
            focus: true,
            quality: 1.,
            age: 4,
            visible: true,
        };
        let t = target(&p, 1280, 720, &m).unwrap();
        assert_eq!(t.pixel, [700., 150.]);
        assert_eq!(t.depth, 3.);
        assert!(!t.estimated);
    }
    #[test]
    fn no_head_landmarks_tracks_visible_body_center() {
        let p = Person {
            id: 1,
            bbox: [650., 100., 750., 650.],
            score: 0.9,
            kpts: None,
            focus: true,
            quality: 1.,
            age: 4,
            visible: true,
        };
        let a = aim(&p, 1280, 720, &MountConfig::default(), [0.; 2]).unwrap();
        assert_eq!(a.pixel, [700., 375.]);
        assert!(!a.face);
    }
    #[test]
    fn eyes_define_target_regardless_of_body_box() {
        let mut k = [crate::shared::Kpt::default(); 17];
        k[1] = crate::shared::Kpt {
            x: 770.,
            y: 180.,
            c: 0.9,
        };
        k[2] = crate::shared::Kpt {
            x: 810.,
            y: 180.,
            c: 0.9,
        };
        let mut p = Person {
            id: 1,
            bbox: [100., 50., 1200., 700.],
            score: 1.,
            kpts: Some(k),
            focus: true,
            quality: 1.,
            age: 5,
            visible: true,
        };
        let a = aim(&p, 1280, 720, &MountConfig::default(), [0.; 2]).unwrap();
        assert_eq!(a.pixel, [790., 180.]);
        assert!(a.angles[0] > 0. && a.angles[1] > 0.);
        p.bbox = [0., 0., 300., 300.];
        assert_eq!(
            aim(&p, 1280, 720, &MountConfig::default(), [0.; 2])
                .unwrap()
                .pixel,
            a.pixel
        );
    }
    #[test]
    fn shoulders_only_use_body_box_center_and_configured_depth() {
        let mut k = [crate::shared::Kpt::default(); 17];
        k[5] = crate::shared::Kpt {
            x: 300.,
            y: 250.,
            c: 1.,
        };
        k[6] = crate::shared::Kpt {
            x: 500.,
            y: 250.,
            c: 1.,
        };
        let p = Person {
            id: 1,
            bbox: [250., 20., 550., 700.],
            score: 1.,
            kpts: Some(k),
            focus: true,
            quality: 1.,
            age: 5,
            visible: true,
        };
        let m = MountConfig::default();
        let a = aim(&p, 1280, 720, &m, [0.; 2]).unwrap();
        assert_eq!(a.pixel, [400., 360.]);
        assert_eq!(a.depth, m.depth_m);
        assert!(!a.face && !a.estimated);
    }
    #[test]
    fn face_body_face_keeps_identity_and_never_uses_invisible_body() {
        let m = MountConfig::default();
        let mut p = face_at([640., 180.]);
        p.bbox = [500., 100., 780., 700.];
        let head = p.kpts;
        for mode in [MountMode::External, MountMode::Pupil] {
            let m = MountConfig { mode, ..m.clone() };
            p.kpts = head;
            let a = aim(&p, 1280, 720, &m, [0.; 2]).unwrap();
            assert!(a.face);
            assert_eq!(a.pixel, [640., 180.]);
            p.kpts = Some([crate::shared::Kpt::default(); 17]);
            let b = aim(&p, 1280, 720, &m, [0.; 2]).unwrap();
            assert_eq!(a.person, b.person);
            assert_eq!(b.pixel, [640., 400.]);
            assert!(!b.face);
            p.kpts = head;
            assert!(aim(&p, 1280, 720, &m, [0.; 2]).unwrap().face);
        }
        p.visible = false;
        assert!(target(&p, 1280, 720, &m).is_none());
        p.visible = true;
        p.kpts = None;
        for bbox in [
            [0., 0., f32::NAN, 100.],
            [800., 100., 700., 500.],
            [1400., 0., 1500., 200.],
        ] {
            p.bbox = bbox;
            assert!(target(&p, 1280, 720, &m).is_none());
        }
    }
}
