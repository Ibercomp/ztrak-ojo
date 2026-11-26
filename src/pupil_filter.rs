//! Adaptive fixation/pursuit in the fixed eye reference frame, in degrees.
//! Velocity is fitted to raw observations, not to lagging filtered positions.
use std::collections::VecDeque;
#[derive(Default)]
pub struct PupilAxis {
    samples: VecDeque<(f32, f32)>,
    time: f32,
    position: Option<f32>,
    fixation: Option<f32>,
    quiet: f32,
}
impl PupilAxis {
    pub fn step(&mut self, raw: f32, dt: f32, response: f32, horizon: f32) -> f32 {
        if dt >= 0.15 {
            *self = Self::default();
        }
        let last = self.samples.back().map(|s| s.1);
        self.time += dt;
        self.samples.push_back((self.time, raw));
        while self.samples.len() > 2 && self.time - self.samples.front().unwrap().0 > 0.14 {
            self.samples.pop_front();
        }
        let n = self.samples.len() as f32;
        let mt = self.samples.iter().map(|s| s.0).sum::<f32>() / n;
        let mx = self.samples.iter().map(|s| s.1).sum::<f32>() / n;
        let variance = self.samples.iter().map(|s| (s.0 - mt).powi(2)).sum::<f32>();
        let velocity = if variance > 1e-6 {
            (self
                .samples
                .iter()
                .map(|s| (s.0 - mt) * (s.1 - mx))
                .sum::<f32>()
                / variance)
                .clamp(-120., 120.)
        } else {
            0.
        };
        let excursion = raw - self.samples.front().unwrap().1;
        let path = self
            .samples
            .iter()
            .zip(self.samples.iter().skip(1))
            .map(|(a, b)| (b.1 - a.1).abs())
            .sum::<f32>();
        let coherent =
            self.samples.len() >= 3 && excursion.abs() > 0.7 && excursion.abs() > 0.8 * path;
        let delta = last.map(|x| raw - x).unwrap_or(0.);
        // Tight filtering for small landmark noise, fast response for pursuit.
        let motion = if coherent { velocity.abs() } else { 0. };
        let cutoff = (response * 0.25 + 0.4 * motion).clamp(0.6, 24.);
        let alpha = 1. - (-std::f32::consts::TAU * cutoff * dt).exp();
        let before = self.position.unwrap_or(raw);
        let smooth = before + alpha * (raw - before);
        self.position = Some(smooth);
        // Stop or reverse: cancel anticipation on this frame, without waiting
        // for a low-pass velocity tail to run down. No lead on noisy zigzags.
        let lead = if coherent && delta.abs() > 0.12 && delta * velocity > 0. {
            (velocity * (horizon + 0.025).min(0.11)).clamp(-3., 3.)
        } else {
            0.
        };
        if self
            .fixation
            .is_some_and(|x| (raw - x).abs() > 0.65 || coherent)
        {
            self.fixation = None;
            self.quiet = 0.;
        }
        if let Some(x) = self.fixation {
            return x;
        }
        if !coherent && (smooth - before).abs() < 0.12 {
            self.quiet += dt;
            if self.quiet >= 0.18 {
                self.fixation = Some(smooth);
            }
        } else {
            self.quiet = 0.;
        }
        smooth + lead
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn fixation_ignores_small_noise_but_releases_for_real_motion() {
        let mut f = PupilAxis::default();
        for i in 0..50 {
            f.step(5. + (i as f32 * 2.).sin() * 0.25, 1. / 24., 5.5, 0.08);
        }
        let held = f.step(5., 1. / 24., 5.5, 0.08);
        for i in 0..24 {
            assert!(
                (f.step(5. + (i as f32 * 2.).sin() * 0.25, 1. / 24., 5.5, 0.08) - held).abs()
                    < 0.01
            );
        }
        for i in 1..=4 {
            f.step(5. + i as f32, 1. / 24., 5.5, 0.08);
        }
        assert!(f.step(10., 1. / 24., 5.5, 0.08) > 9.);
    }
    #[test]
    fn anticipation_stops_on_stop_and_reversal() {
        let mut f = PupilAxis::default();
        for i in 0..12 {
            f.step(i as f32, 1. / 24., 5.5, 0.08);
        }
        assert!(f.step(11., 1. / 24., 5.5, 0.08) <= 11.);
        assert!(f.step(10., 1. / 24., 5.5, 0.08) <= 11.);
        // No leftover lead across a detection gap.
        assert_eq!(f.step(-5., 0.2, 5.5, 0.08), -5.);
    }
}
