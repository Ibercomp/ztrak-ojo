//! Deterministic comparison to a plain One Euro filter. This measures the software
//! filter with 80 ms observations and a 40 ms actuator model, not the real servo.
use ztrak_ojo::{filter::OneEuro, pupil_filter::PupilAxis};
fn face(t: f32) -> f32 {
    if t < 2. {
        0.
    } else if t < 3.25 {
        (t - 2.) * 20.
    } else if t < 5. {
        25.
    } else if t < 6.25 {
        25. - (t - 5.) * 20.
    } else {
        0.
    }
}
fn simulate(adaptive: bool) -> [f32; 3] {
    let dt = 1. / 24.;
    let mut old = OneEuro::default();
    let mut new = PupilAxis::default();
    let (mut actual, mut command) = (0f32, 0f32);
    let (mut rest, mut moving, mut nr, mut nm) = (0f32, 0f32, 0, 0);
    let mut overshoot = 0f32;
    for i in 0..240 {
        let t = i as f32 * dt;
        let input = face(t - 0.08) + 0.35 * (i as f32 * 2.).sin() + 0.15 * (i as f32 * 0.7).sin();
        let dest = if adaptive {
            new.step(input, dt, 5.5, 0.08)
        } else {
            let x = old.filter(input, dt, 5.5, 0.04);
            x + (old.velocity().clamp(-60., 60.) * 0.08).clamp(-4., 4.)
        };
        // Same gain and limits for both; improvement must come from control,
        // not from simply allowing the new one to run faster.
        let desired = actual + 0.9 * (dest - actual);
        command += (desired - command).clamp(-120. * dt, 120. * dt);
        actual += (command - actual) * (1. - (-dt / 0.04).exp());
        let error = (actual - face(t)).abs();
        if (t > 0.5 && t < 1.9) || (t > 3.8 && t < 4.9) || t > 6.8 {
            rest += error * error;
            nr += 1;
        }
        if (t > 2.2 && t < 3.25) || (t > 5.2 && t < 6.25) {
            moving += error;
            nm += 1;
        }
        if t > 3.25 && t < 4. {
            overshoot = overshoot.max(actual - 25.);
        }
    }
    [(rest / nr as f32).sqrt(), moving / nm as f32, overshoot]
}
#[test]
fn less_rest_error_less_pursuit_lag_and_less_stop_overshoot() {
    let old = simulate(false);
    let new = simulate(true);
    println!("[reposo RMS, seguimiento medio, sobrepaso] grados: OneEuro={old:.4?}, nuevo={new:.4?}");
    assert!(new[0] < old[0] * 0.8, "noise at rest: {new:?} / {old:?}");
    assert!(new[1] < old[1] * 0.8, "moving face: {new:?} / {old:?}");
    assert!(new[2] < old[2] * 0.9, "stop: {new:?} / {old:?}");
}
