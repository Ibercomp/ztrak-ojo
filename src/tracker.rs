//! Seguimiento de personas entre fotogramas (asociación por IoU) y elección del foco.
//!
//! Regla del foco:
//!  * se queda con su persona mientras se la vea (o deje de verse menos de `lost_timeout`);
//!  * si se pierde, pasa a la que «mejor se ve» (tamaño, centrado y puntos fiables);
//!  * opcionalmente otra puede robarlo si es `steal_ratio` veces mejor (0 = nunca).

use std::time::Instant;

use crate::config::FocusConfig;
use crate::shared::{Kpt, NUM_KPTS, Person};
use crate::vision::Detection;

pub struct Track {
    pub id: u64,
    /// Image velocity for association when somebody walks across the camera.
    pub velocity: [f32; 2],
    pub bbox: [f32; 4],
    pub score: f32,
    pub kpts: Option<[Kpt; NUM_KPTS]>,
    /// Últimos puntos buenos y cuándo (si falta un fotograma, se usan un momento).
    pub last_kpts: Option<[Kpt; NUM_KPTS]>,
    pub last_kpts_at: Option<Instant>,
    pub kpt_quality: f32,
    pub hits: u32,
    pub last_seen: Instant,
    /// Última vez que la confirmó el detector (no solo su propio esqueleto).
    pub last_det: Instant,
    pub seen_now: bool,
    pub quality: f32,
}

impl Track {
    fn predicted(&self, now: Instant, horizon: f32) -> [f32; 4] {
        let dt = now
            .saturating_duration_since(self.last_seen)
            .as_secs_f32()
            .min(horizon);
        let [x, y] = self.velocity.map(|v| v * dt);
        [
            self.bbox[0] + x,
            self.bbox[1] + y,
            self.bbox[2] + x,
            self.bbox[3] + y,
        ]
    }
    pub fn set_kpts(&mut self, k: [Kpt; NUM_KPTS], thr: f32, now: Instant) {
        let good = k.iter().filter(|p| p.c >= thr).count() as f32;
        self.kpt_quality = good / NUM_KPTS as f32;
        self.kpts = Some(k);
        if good >= 5.0 {
            self.last_kpts = Some(k);
            self.last_kpts_at = Some(now);
        }
    }
}

/// Intersección sobre la menor de las dos áreas (1 = una dentro de la otra).
pub fn iom(a: &[f32; 4], b: &[f32; 4]) -> f32 {
    let iw = (a[2].min(b[2]) - a[0].max(b[0])).max(0.0);
    let ih = (a[3].min(b[3]) - a[1].max(b[1])).max(0.0);
    let m = ((a[2] - a[0]) * (a[3] - a[1])).min((b[2] - b[0]) * (b[3] - b[1]));
    if m <= 0.0 { 0.0 } else { iw * ih / m }
}

/// Acerca cada lado de `a` al de `b` una fracción `k` (solo los lados en que `a` sobresale).
fn shrink_towards(a: &[f32; 4], b: &[f32; 4], k: f32) -> [f32; 4] {
    [
        if a[0] < b[0] {
            a[0] + (b[0] - a[0]) * k
        } else {
            a[0]
        },
        if a[1] < b[1] {
            a[1] + (b[1] - a[1]) * k
        } else {
            a[1]
        },
        if a[2] > b[2] {
            a[2] + (b[2] - a[2]) * k
        } else {
            a[2]
        },
        if a[3] > b[3] {
            a[3] + (b[3] - a[3]) * k
        } else {
            a[3]
        },
    ]
}

pub fn iou(a: &[f32; 4], b: &[f32; 4]) -> f32 {
    let ix = (a[2].min(b[2]) - a[0].max(b[0])).max(0.0);
    let iy = (a[3].min(b[3]) - a[1].max(b[1])).max(0.0);
    let inter = ix * iy;
    let ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter;
    if ua <= 0.0 { 0.0 } else { inter / ua }
}

pub struct Tracker {
    pub tracks: Vec<Track>,
    pub focus: Option<u64>,
    next_id: u64,
    pub cfg: FocusConfig,
    blink_started: Option<Instant>,
    focus_hold: Option<(u64, Instant)>,
    /// When the current focus started and how long to keep it if everyone is still.
    attention: Option<(u64, Instant, f32)>,
    last_attended: std::collections::HashMap<u64, Instant>,
    motion_history: std::collections::HashMap<u64, std::collections::VecDeque<(Instant, [f32; 2])>>,
    rng: u32,
}

impl Tracker {
    // Preserve identity only, never mark an old observation as visible.
    pub fn set_blink(&mut self, blinking: bool, now: Instant) {
        if blinking {
            let start = *self.blink_started.get_or_insert(now);
            if let Some(id) = self.focus {
                self.focus_hold = Some((id, start + std::time::Duration::from_millis(1200)));
            }
        } else if let Some(start) = self.blink_started.take() {
            if let Some((_, until)) = self.focus_hold.as_mut() {
                *until = (now + std::time::Duration::from_millis(250))
                    .min(start + std::time::Duration::from_millis(1200));
            }
        }
        if self.focus_hold.is_some_and(|(_, until)| now > until) {
            self.focus_hold = None;
        }
    }
    fn held_focus(&self, now: Instant) -> Option<u64> {
        self.focus_hold
            .filter(|(_, until)| now <= *until)
            .map(|(id, _)| id)
    }

    pub fn new(cfg: FocusConfig) -> Self {
        Self {
            tracks: vec![],
            focus: None,
            next_id: 1,
            blink_started: None,
            focus_hold: None,
            attention: None,
            last_attended: Default::default(),
            motion_history: Default::default(),
            rng: 0x6d2b79f5,
            cfg,
        }
    }

    /// Cajas sacadas del esqueleto del fotograma anterior (para no pasar el detector en todos
    /// los fotogramas). `None` si no hay ninguna pista con esqueleto.
    pub fn boxes_from_pose(
        &self,
        thr: f32,
        w: u32,
        h: u32,
        now: Instant,
    ) -> Option<Vec<Detection>> {
        let mut out = vec![];
        for t in &self.tracks {
            // su esqueleto de hace poco (aunque el detector no lo viera en el último fotograma)
            // sin confirmación del detector en 1 s, su esqueleto ya no la mantiene (fantasmas)
            let recent = t
                .last_kpts_at
                .is_some_and(|a| now.saturating_duration_since(a).as_secs_f32() < 0.4)
                && now.saturating_duration_since(t.last_det).as_secs_f32() < 1.0;
            let Some(k) = (if t.seen_now {
                t.kpts.or(t.last_kpts)
            } else {
                t.last_kpts
            })
            .filter(|_| recent) else {
                continue;
            };
            let k = &k;
            let good: Vec<&Kpt> = k.iter().filter(|p| p.c >= thr).collect();
            if good.len() < 5 {
                continue;
            }
            let (mut x0, mut y0, mut x1, mut y1) = (f32::MAX, f32::MAX, f32::MIN, f32::MIN);
            for p in &good {
                x0 = x0.min(p.x);
                y0 = y0.min(p.y);
                x1 = x1.max(p.x);
                y1 = y1.max(p.y);
            }
            let (bw, bh) = (x1 - x0, y1 - y0);
            let side = bw * 0.15 + bh * 0.05;
            let kb = [
                x0 - side,
                y0 - bh * 0.2 - bw * 0.1,
                x1 + side,
                y1 + bh * 0.08,
            ];
            // unión con la caja anterior encogida hacia la del esqueleto: si falta un trozo
            // (brazo, cabeza) un momento no se recorta, pero la caja no crece sin fin (si ocupara
            // toda la imagen, el detector dejaría de reconocerla y el esqueleto perdería resolución)
            let p = shrink_towards(&t.bbox, &kb, 0.25);
            let b = [
                kb[0].min(p[0]).max(0.0),
                kb[1].min(p[1]).max(0.0),
                kb[2].max(p[2]).min(w as f32),
                kb[3].max(p[3]).min(h as f32),
            ];
            out.push(Detection {
                bbox: b,
                score: t.score,
                track: Some(t.id),
            });
        }
        if out.is_empty() { None } else { Some(out) }
    }

    /// Une a cada pista la caja de su esqueleto anterior (misma forma de recorte en todos los
    /// fotogramas: los puntos no «saltan» al alternar detector y esqueleto).
    pub fn merge_pose_boxes(&mut self, boxes: &[Detection]) {
        for t in self.tracks.iter_mut().filter(|t| t.seen_now) {
            if let Some(b) = boxes
                .iter()
                .max_by(|a, b| iou(&a.bbox, &t.bbox).total_cmp(&iou(&b.bbox, &t.bbox)))
            {
                if iou(&b.bbox, &t.bbox) > 0.3 || iom(&b.bbox, &t.bbox) > 0.7 {
                    t.bbox = [
                        t.bbox[0].min(b.bbox[0]),
                        t.bbox[1].min(b.bbox[1]),
                        t.bbox[2].max(b.bbox[2]),
                        t.bbox[3].max(b.bbox[3]),
                    ];
                }
            }
        }
    }

    /// Lo que ha visto el detector: confirma a las personas que ya sigue su esqueleto (`covered`)
    /// y devuelve el resto (gente nueva o pistas sin esqueleto ahora) para `update`.
    /// También devuelve, para cada persona confirmada, la caja del detector: el recorte del
    /// esqueleto se une con ella (si no, en primeros planos el recorte se iba encogiendo hasta
    /// quedarse en la cara).
    pub fn confirm(
        &mut self,
        dets: &[Detection],
        covered: &[u64],
        now: Instant,
    ) -> (Vec<Detection>, Vec<(u64, [f32; 4])>) {
        let mut rest = vec![];
        let mut hints = vec![];
        for d in dets {
            // dentro (sobre todo) de la caja de alguien a quien ya sigue su esqueleto: es esa
            // persona (o un trozo suyo), no otra
            if let Some(t) = self
                .tracks
                .iter_mut()
                .filter(|t| covered.contains(&t.id))
                .find(|t| iom(&t.bbox, &d.bbox) > 0.7 || iou(&t.bbox, &d.bbox) > 0.4)
            {
                t.last_det = now;
                t.score = d.score;
                hints.push((t.id, d.bbox));
                continue;
            }
            let best = self
                .tracks
                .iter_mut()
                .map(|t| {
                    let v = iou(&t.bbox, &d.bbox).max(iom(&t.bbox, &d.bbox) * 0.5);
                    let ok = v > 0.25 || iom(&t.bbox, &d.bbox) > 0.75;
                    (v, ok, t)
                })
                .filter(|x| x.1)
                .max_by(|a, b| a.0.total_cmp(&b.0));
            match best {
                Some((_, _, t)) if covered.contains(&t.id) => {
                    t.last_det = now;
                    t.score = d.score;
                    hints.push((t.id, d.bbox));
                }
                _ => rest.push(Detection { track: None, ..*d }),
            }
        }
        (rest, hints)
    }

    /// Asocia las detecciones a pistas. Devuelve, para cada detección, el índice de su pista.
    pub fn update(&mut self, dets: &[Detection], _w: u32, _h: u32, now: Instant) -> Vec<usize> {
        for t in &mut self.tracks {
            t.seen_now = false;
            t.kpts = None; // los puntos son de este fotograma o no son
        }
        let mut det_track: Vec<Option<u64>> = vec![None; dets.len()];
        let mut used_t = vec![false; self.tracks.len()];
        let mut used_d = vec![false; dets.len()];
        // las cajas sacadas de un esqueleto van directas a su pista
        for (di, d) in dets.iter().enumerate() {
            let Some(tid) = d.track else { continue };
            used_d[di] = true;
            if let Some(ti) = self.tracks.iter().position(|t| t.id == tid) {
                if used_t[ti] {
                    continue;
                }
                used_t[ti] = true;
                let t = &mut self.tracks[ti];
                for k in 0..4 {
                    t.bbox[k] += (d.bbox[k] - t.bbox[k]) * 0.6;
                }
                det_track[di] = Some(t.id);
                t.hits += 1;
                t.last_seen = t.last_seen.max(now);
                t.seen_now = true;
            }
        }
        // emparejamiento voraz por IoU, de mayor a menor (solo detecciones del detector)
        let mut pairs: Vec<(f32, usize, usize)> = vec![];
        for (ti, t) in self.tracks.iter().enumerate() {
            for (di, d) in dets.iter().enumerate() {
                if d.track.is_some() {
                    continue;
                }
                // una caja dentro de la otra también es la misma persona (la de la pista
                // puede ser más grande: incluye brazos extendidos que el detector recorta)
                let predicted = t.predicted(
                    now,
                    if self.held_focus(now) == Some(t.id) {
                        0.8
                    } else {
                        0.3
                    },
                );
                // The pupil camera can stop/reverse in a frame. Prediction must
                // not reject a detection still overlapping the observed box.
                let overlap = |b: &[f32; 4]| iou(b, &d.bbox).max(iom(b, &d.bbox) * 0.5);
                let v = overlap(&predicted).max(overlap(&t.bbox));
                if v > 0.25 || iom(&predicted, &d.bbox) > 0.75 {
                    pairs.push((v, ti, di));
                }
            }
        }
        pairs.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (_, ti, di) in pairs {
            if used_t[ti] || used_d[di] {
                continue;
            }
            used_t[ti] = true;
            used_d[di] = true;
            let t = &mut self.tracks[ti];
            let dt = now
                .saturating_duration_since(t.last_det)
                .as_secs_f32()
                .max(0.01);
            for axis in 0..2 {
                let movement = (dets[di].bbox[axis] + dets[di].bbox[axis + 2]
                    - t.bbox[axis]
                    - t.bbox[axis + 2])
                    * 0.5;
                t.velocity[axis] =
                    t.velocity[axis] * 0.5 + (movement / dt).clamp(-2500., 2500.) * 0.5;
            }
            // suavizado ligero de la caja para que el recorte del esqueleto no tiemble
            let a = 0.6;
            for k in 0..4 {
                t.bbox[k] = t.bbox[k] * (1.0 - a) + dets[di].bbox[k] * a;
            }
            t.score = dets[di].score;
            det_track[di] = Some(t.id);
            t.hits += 1;
            t.last_seen = t.last_seen.max(now);
            t.last_det = now;
            t.seen_now = true;
        }
        for (di, d) in dets.iter().enumerate() {
            // solo el detector crea personas nuevas
            if !used_d[di] && d.track.is_none() {
                det_track[di] = Some(self.next_id);
                self.tracks.push(Track {
                    id: self.next_id,
                    velocity: [0.; 2],
                    bbox: d.bbox,
                    score: d.score,
                    kpts: None,
                    last_kpts: None,
                    last_kpts_at: None,
                    kpt_quality: 0.0,
                    hits: 1,
                    last_seen: now,
                    last_det: now,
                    seen_now: true,
                    quality: 0.0,
                });
                self.next_id += 1;
            }
        }
        // duplicados (dos pistas sobre la misma persona): se queda la más antigua o el foco
        let mut drop = vec![];
        for a in 0..self.tracks.len() {
            for b in a + 1..self.tracks.len() {
                let (ta, tb) = (&self.tracks[a], &self.tracks[b]);
                let partial_duplicate = (!ta.seen_now || !tb.seen_now)
                    && iom(&ta.bbox, &tb.bbox) > 0.85
                    && ((ta.bbox[0] + ta.bbox[2] - tb.bbox[0] - tb.bbox[2]) * 0.5).abs()
                        < (ta.bbox[2] - ta.bbox[0]).max(tb.bbox[2] - tb.bbox[0]) * 0.4;
                if iou(&ta.bbox, &tb.bbox) > 0.6 || partial_duplicate {
                    let keep_a = Some(ta.id) == self.focus
                        || (Some(tb.id) != self.focus && ta.hits >= tb.hits);
                    let (keep, remove) = if keep_a { (a, b) } else { (b, a) };
                    let removed_id = self.tracks[remove].id;
                    // Transfer the fresh BODY observation to the locked identity.
                    // Otherwise the old partial focus would be left blind.
                    if !self.tracks[keep].seen_now && self.tracks[remove].seen_now {
                        let bbox = self.tracks[remove].bbox;
                        let score = self.tracks[remove].score;
                        let det = self.tracks[remove].last_det;
                        let t = &mut self.tracks[keep];
                        t.bbox = bbox;
                        t.score = score;
                        t.last_det = det;
                        t.last_seen = now;
                        t.seen_now = true;
                        t.kpts = None;
                        t.hits += 1;
                    }
                    drop.push(removed_id);
                }
            }
        }
        if !drop.is_empty() {
            self.tracks.retain(|t| !drop.contains(&t.id));
        }
        // olvidar a quien lleva tiempo sin verse (más que el margen del foco)
        let keep = self.cfg.lost_timeout.max(0.3) + 0.5;
        let held = self.held_focus(now);
        self.tracks.retain(|t| {
            Some(t.id) == held || now.saturating_duration_since(t.last_seen).as_secs_f32() < keep
        });
        if let Some(f) = self.focus {
            if !self.tracks.iter().any(|t| t.id == f) {
                self.focus = None;
            }
        }
        det_track
            .iter()
            .map(|id| {
                self.tracks
                    .iter()
                    .position(|t| Some(t.id) == *id)
                    .unwrap_or(0)
            })
            .collect()
    }

    /// A single missed body detection must not interrupt a still-visible head.
    /// Try a new pose in the last focus crop for at most 300 ms; only fresh
    /// confident head OR torso landmarks may confirm visibility in this grace period.
    pub fn head_order(&self, max: usize, now: Instant) -> Vec<usize> {
        let mut out = self.pose_order(max);
        if let Some(i) = self.tracks.iter().position(|t| {
            Some(t.id) == self.focus
                && !t.seen_now
                && (now.saturating_duration_since(t.last_det).as_millis() < 300
                    || self.held_focus(now) == Some(t.id))
        }) {
            out.insert(0, i);
            out.truncate(max.max(1));
        }
        out
    }
    pub fn confirm_fresh_head(
        &mut self,
        i: usize,
        k: &[Kpt; NUM_KPTS],
        w: u32,
        h: u32,
        now: Instant,
    ) {
        let held = self.held_focus(now);
        let t = &mut self.tracks[i];
        if t.seen_now
            || Some(t.id) != self.focus
            || (now.saturating_duration_since(t.last_det).as_millis() >= 300 && held != Some(t.id))
        {
            return;
        }
        let head = k
            .iter()
            .take(5)
            .filter(|p| {
                p.c >= 0.6 && p.x > 1. && p.y > 1. && p.x < w as f32 - 1. && p.y < h as f32 - 1.
            })
            .count();
        let torso = [5, 6, 11, 12].iter().all(|&i| {
            let p = k[i];
            p.c >= 0.6 && p.x > 1. && p.y > 1. && p.x < w as f32 - 1. && p.y < h as f32 - 1.
        }) && (k[5].x - k[6].x).hypot(k[5].y - k[6].y) > 15.
            && ((k[11].y + k[12].y - k[5].y - k[6].y) * 0.5).abs() > 20.;
        if head >= 2 || torso {
            t.seen_now = true;
            t.last_seen = t.last_seen.max(now);
            if head < 2 && torso {
                // A fresh torso is a valid fallback even when the face is hidden.
                // Shift the last detector box with the torso, not a stale head.
                if let Some(old) = t
                    .last_kpts
                    .filter(|old| [5, 6, 11, 12].iter().all(|&i| old[i].c >= 0.6))
                {
                    let dx = [5, 6, 11, 12]
                        .iter()
                        .map(|&i| k[i].x - old[i].x)
                        .sum::<f32>()
                        / 4.;
                    let dy = [5, 6, 11, 12]
                        .iter()
                        .map(|&i| k[i].y - old[i].y)
                        .sum::<f32>()
                        / 4.;
                    if dx.abs() < w as f32 * 0.15 && dy.abs() < h as f32 * 0.15 {
                        t.bbox = [
                            (t.bbox[0] + dx).max(0.),
                            (t.bbox[1] + dy).max(0.),
                            (t.bbox[2] + dx).min(w as f32),
                            (t.bbox[3] + dy).min(h as f32),
                        ];
                    }
                }
            }
        }
    }

    /// Índices de pistas a las que calcular esqueleto: el foco primero, luego las mejores.
    pub fn pose_order(&self, max: usize) -> Vec<usize> {
        let mut idx: Vec<usize> = (0..self.tracks.len())
            .filter(|&i| self.tracks[i].seen_now)
            .collect();
        idx.sort_by(|&a, &b| {
            let fa = Some(self.tracks[a].id) == self.focus;
            let fb = Some(self.tracks[b].id) == self.focus;
            fb.cmp(&fa)
                .then(self.tracks[b].quality.total_cmp(&self.tracks[a].quality))
        });
        idx.truncate(max.max(1));
        idx
    }

    pub fn choose_focus(&mut self, w: u32, h: u32, now: Instant) {
        let (w, h) = (w as f32, h as f32);
        let c = &self.cfg;
        for t in &mut self.tracks {
            let area =
                ((t.bbox[2] - t.bbox[0]) * (t.bbox[3] - t.bbox[1]) / (w * h)).clamp(0.0, 1.0);
            let cx = (t.bbox[0] + t.bbox[2]) / 2.0 / w - 0.5;
            let cy = (t.bbox[1] + t.bbox[3]) / 2.0 / h - 0.5;
            let center = 1.0 - (cx * cx + cy * cy).sqrt() / 0.7071;
            t.quality = c.w_size * area.sqrt()
                + c.w_center * center.max(0.0)
                + c.w_keypoints * t.kpt_quality;
            if !t.seen_now {
                t.quality *= 0.5;
            }
        }
        let best = self
            .tracks
            .iter()
            .filter(|t| t.seen_now && t.hits >= c.min_hits)
            .max_by(|a, b| a.quality.total_cmp(&b.quality))
            .map(|t| (t.id, t.quality));
        let current = self
            .focus
            .and_then(|f| self.tracks.iter().find(|t| t.id == f));
        if let Some(id) = self
            .held_focus(now)
            .filter(|id| self.tracks.iter().any(|t| t.id == *id))
        {
            self.focus = Some(id);
            return;
        }
        self.focus = match (current, best) {
            // la actual sigue dentro del margen: se queda, salvo robo por calidad
            (Some(cur), Some((bid, bq)))
                if now.saturating_duration_since(cur.last_seen).as_secs_f32() <= c.lost_timeout =>
            {
                if c.steal_ratio > 1.0
                    && bid != cur.id
                    && cur.seen_now
                    && bq > cur.quality * c.steal_ratio
                {
                    Some(bid)
                } else {
                    Some(cur.id)
                }
            }
            (Some(cur), None)
                if now.saturating_duration_since(cur.last_seen).as_secs_f32() <= c.lost_timeout =>
            {
                Some(cur.id)
            }
            (_, Some((bid, _))) => Some(bid),
            _ => None,
        };
        self.share_attention(now);
    }

    /// Image speed relative to the group, in body heights per second, over the
    /// last ~0.6 s of box centres (single-frame box jitter is not motion).
    /// Removing the median cancels the camera's own rotation, which shifts
    /// everybody alike.
    fn relative_motion(&mut self, now: Instant) -> Vec<(u64, f32)> {
        const WINDOW: f32 = 0.6;
        let mut seen = vec![];
        for t in self.tracks.iter().filter(|t| t.seen_now && t.hits >= self.cfg.min_hits) {
            let c = [(t.bbox[0] + t.bbox[2]) / 2., (t.bbox[1] + t.bbox[3]) / 2.];
            let h = (t.bbox[3] - t.bbox[1]).max(40.);
            let hist = self.motion_history.entry(t.id).or_default();
            if hist.back().is_none_or(|(at, _)| *at < now) {
                hist.push_back((now, c));
            }
            while hist.len() > 2
                && now.saturating_duration_since(hist[1].0).as_secs_f32() >= WINDOW
            {
                hist.pop_front();
            }
            let (t0, c0) = hist[0];
            let dt = now.saturating_duration_since(t0).as_secs_f32();
            if dt >= WINDOW * 0.5 {
                seen.push((t.id, [(c[0] - c0[0]) / dt, (c[1] - c0[1]) / dt], h));
            }
        }
        let live: Vec<u64> = self.tracks.iter().map(|t| t.id).collect();
        self.motion_history.retain(|id, _| live.contains(id));
        let median = |axis: usize| {
            let mut v: Vec<f32> = seen.iter().map(|s| s.1[axis]).collect();
            v.sort_by(f32::total_cmp);
            if v.is_empty() { 0. } else { v[v.len() / 2] }
        };
        let m = [median(0), median(1)];
        seen.iter()
            .map(|(id, v, h)| (*id, (v[0] - m[0]).hypot(v[1] - m[1]) / h))
            .collect()
    }

    /// Several still people: look at one for a few seconds, then another.
    /// Somebody moving holds (or takes) the attention.
    fn share_attention(&mut self, now: Instant) {
        let Some(focus) = self.focus else {
            self.attention = None;
            return;
        };
        if self.attention.is_none_or(|(id, _, _)| id != focus) {
            self.rng ^= self.rng << 13;
            self.rng ^= self.rng >> 17;
            self.rng ^= self.rng << 5;
            let [lo, hi] = self.cfg.attention_s;
            let dwell = lo + (hi - lo).max(0.) * (self.rng % 1000) as f32 / 1000.;
            self.attention = Some((focus, now, dwell));
        }
        self.last_attended.insert(focus, now);
        let (_, since, dwell) = self.attention.unwrap();
        let looked = now.saturating_duration_since(since).as_secs_f32();
        let motion = self.relative_motion(now);
        if motion.len() < 2 {
            return;
        }
        let moving = |id: u64| motion.iter().any(|&(i, v)| i == id && v > self.cfg.motion_threshold);
        if moving(focus) {
            return; // fix the track while it moves
        }
        // A moving person takes over after a short minimum look at the current one.
        let candidates: Vec<u64> = motion.iter().map(|&(i, _)| i).filter(|&i| i != focus).collect();
        let next = if looked >= 1.5 && candidates.iter().any(|&i| moving(i)) {
            candidates.iter().copied().find(|&i| moving(i))
        } else if looked >= dwell {
            // Round robin: the person not looked at for the longest time.
            candidates.iter().copied().min_by_key(|i| self.last_attended.get(i).copied())
        } else {
            None
        };
        if let Some(id) = next {
            log::info!(
                "Atención: {focus} -> {id} ({}) tras {looked:.1}s; movimiento {:?}",
                if moving(id) { "se mueve" } else { "turno" },
                motion.iter().map(|(i, v)| format!("{i}:{v:.2}")).collect::<Vec<_>>()
            );
            self.focus = Some(id);
            self.attention = None;
            self.share_attention_start(now);
        }
        let live: Vec<u64> = self.tracks.iter().map(|t| t.id).collect();
        self.last_attended.retain(|id, _| live.contains(id));
    }
    fn share_attention_start(&mut self, now: Instant) {
        if let Some(f) = self.focus {
            self.last_attended.insert(f, now);
        }
    }

    pub fn people(&self, now: Instant) -> Vec<crate::shared::Person> {
        self.tracks
            .iter()
            .filter(|t| {
                t.seen_now || now.saturating_duration_since(t.last_seen).as_secs_f32() < 0.3
            })
            .map(|t| Person {
                id: t.id,
                visible: t.seen_now,
                bbox: t.bbox,
                score: t.score,
                // Gaze must not use an old face when only the body is visible.
                // Historical points remain available internally for association.
                kpts: t.kpts,
                focus: Some(t.id) == self.focus,
                quality: t.quality,
                age: t.hits,
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;
    #[test]
    fn blink_preserves_identity_without_claiming_old_body_is_visible() {
        let t = Instant::now();
        let mut tr = Tracker::new(FocusConfig {
            lost_timeout: 0.2,
            ..Default::default()
        });
        tr.update(&[det(100.)], 1280, 720, t);
        let id = tr.tracks[0].id;
        tr.focus = Some(id);
        tr.set_blink(true, t);
        let missing = t + Duration::from_millis(800);
        tr.set_blink(true, missing);
        tr.update(&[], 1280, 720, missing);
        tr.choose_focus(1280, 720, missing);
        assert_eq!(tr.focus, Some(id));
        assert!(!tr.people(missing).iter().any(|p| p.visible));
        tr.set_blink(false, missing);
        let returning = t + Duration::from_millis(900);
        tr.update(&[det(120.)], 1280, 720, returning);
        tr.choose_focus(1280, 720, returning);
        assert_eq!(tr.focus, Some(id));
        assert!(tr.people(returning).iter().any(|p| p.id == id && p.visible));
        let expired = t + Duration::from_secs(3);
        tr.set_blink(false, expired);
        tr.update(&[], 1280, 720, expired);
        tr.choose_focus(1280, 720, expired);
        assert_eq!(tr.focus, None);
    }
    #[test]
    fn blink_retention_has_a_deadline_even_if_closed_flag_sticks() {
        let t = Instant::now();
        let mut tr = Tracker::new(FocusConfig::default());
        tr.update(&[det(100.)], 1280, 720, t);
        tr.focus = Some(tr.tracks[0].id);
        tr.set_blink(true, t);
        let now = t + Duration::from_secs(2);
        tr.set_blink(true, now);
        tr.update(&[], 1280, 720, now);
        tr.choose_focus(1280, 720, now);
        assert_eq!(tr.focus, None);
    }
    #[test]
    fn partial_focus_transfers_to_fresh_body_without_changing_identity() {
        let now = Instant::now();
        let mut tr = Tracker::new(FocusConfig::default());
        let d = |bbox| Detection {
            bbox,
            score: 0.9,
            track: None,
        };
        tr.update(&[d([587., 280., 752., 572.])], 1280, 720, now);
        let id = tr.tracks[0].id;
        tr.focus = Some(id);
        tr.update(
            &[d([559., 454., 898., 715.])],
            1280,
            720,
            now + Duration::from_millis(40),
        );
        assert_eq!(tr.tracks.len(), 2);
        // The full-body observation has already been associated with the
        // second track, while the locked partial box misses this detector frame.
        let body_id = tr.tracks[1].id;
        tr.update(
            &[Detection {
                bbox: [531., 208., 741., 691.],
                score: 0.9,
                track: Some(body_id),
            }],
            1280,
            720,
            now + Duration::from_millis(80),
        );
        let p = tr.people(now + Duration::from_millis(80));
        assert!(p.iter().any(|p| p.id == id && p.visible));
        assert_eq!(
            tr.tracks.len(),
            1,
            "one person must not retain partial and full-body identities"
        );
        assert_eq!(tr.focus, Some(id));
    }
    #[test]
    fn fresh_torso_bridges_body_detector_dropout_without_fabricating_a_face() {
        let now = Instant::now();
        let mut tr = Tracker::new(FocusConfig::default());
        tr.update(&[det(100.)], 1280, 720, now);
        tr.focus = Some(tr.tracks[0].id);
        let mut k = [Kpt::default(); NUM_KPTS];
        for (i, x, y) in [
            (5, 120., 200.),
            (6, 200., 200.),
            (11, 130., 350.),
            (12, 190., 350.),
        ] {
            k[i] = Kpt { x, y, c: 0.9 };
        }
        let t = now + Duration::from_millis(160);
        tr.update(&[], 1280, 720, t);
        tr.confirm_fresh_head(0, &k, 1280, 720, t);
        tr.tracks[0].set_kpts(k, 0.4, t);
        let p = tr.people(t);
        assert!(p[0].visible);
        assert!(
            !crate::geometry::target(&p[0], 1280, 720, &Default::default())
                .unwrap()
                .face
        );
        let t = now + Duration::from_millis(500);
        tr.update(&[], 1280, 720, t);
        tr.confirm_fresh_head(0, &k, 1280, 720, t);
        assert!(
            !tr.tracks[0].seen_now,
            "fresh pose alone cannot keep a ghost forever"
        );
    }
    #[test]
    fn short_body_dropout_requires_fresh_head_and_has_a_deadline() {
        let t = Instant::now();
        let mut tr = Tracker::new(FocusConfig::default());
        tr.update(&[det(100.)], 1280, 720, t);
        tr.focus = Some(tr.tracks[0].id);
        let missing = t + Duration::from_millis(100);
        tr.update(&[], 1280, 720, missing);
        assert_eq!(tr.head_order(1, missing), vec![0]);
        let mut k = [Kpt::default(); NUM_KPTS];
        tr.confirm_fresh_head(0, &k, 1280, 720, missing);
        assert!(!tr.tracks[0].seen_now);
        k[0] = Kpt {
            x: 150.,
            y: 150.,
            c: 0.9,
        };
        k[1] = Kpt {
            x: 140.,
            y: 140.,
            c: 0.9,
        };
        tr.confirm_fresh_head(0, &k, 1280, 720, missing);
        assert!(tr.tracks[0].seen_now);
        let expired = t + Duration::from_millis(350);
        tr.update(&[], 1280, 720, expired);
        assert!(tr.head_order(1, expired).is_empty());
        tr.confirm_fresh_head(0, &k, 1280, 720, expired);
        assert!(!tr.tracks[0].seen_now);
    }
    fn det(x: f32) -> Detection {
        Detection {
            bbox: [x, 100., x + 100., 500.],
            score: 0.9,
            track: None,
        }
    }
    /// Runs `secs` at 20 fps; `xs(t)` gives each person's box x. Returns focus ids.
    fn run(tr: &mut Tracker, start: Instant, secs: f32, xs: impl Fn(f32) -> Vec<f32>) -> Vec<u64> {
        let mut out = vec![];
        for frame in 0..(secs * 20.) as u64 {
            let t = start + Duration::from_millis(frame * 50);
            let d: Vec<Detection> = xs(frame as f32 / 20.).into_iter().map(det).collect();
            tr.update(&d, 1280, 720, t);
            tr.choose_focus(1280, 720, t);
            out.push(tr.focus.unwrap_or(0));
        }
        out
    }
    #[test]
    fn still_people_share_attention() {
        let mut tr = Tracker::new(FocusConfig::default());
        let f = run(&mut tr, Instant::now(), 12., |_| vec![200., 800.]);
        let switches = f.windows(2).filter(|w| w[0] != w[1] && w[0] != 0).count();
        assert!((2..=6).contains(&switches), "{switches}");
        let ids: std::collections::HashSet<u64> = f.iter().copied().filter(|&i| i != 0).collect();
        assert_eq!(ids.len(), 2);
    }
    #[test]
    fn moving_person_keeps_and_takes_attention() {
        let mut tr = Tracker::new(FocusConfig::default());
        // Person at 800 walks at 150 px/s (0.375 heights/s); 200 stays still.
        let f = run(&mut tr, Instant::now(), 10., |t| vec![200., if t < 1. { 800. } else { 800. - 150. * (t - 1.) }]);
        let walker = *f.last().unwrap();
        let tail = &f[f.len() - 120..];
        assert!(tail.iter().all(|&i| i == walker), "{tail:?}");
        let x = tr.tracks.iter().find(|t| t.id == walker).unwrap().bbox[0];
        assert!(x < 700., "the walker should hold attention, x={x}");
    }
    #[test]
    fn passing_without_face_retains_focus() {
        let mut tr = Tracker::new(FocusConfig::default());
        let start = Instant::now();
        for frame in 0..25 {
            let t = start + Duration::from_millis(frame * 50);
            tr.update(&[det(50. + frame as f32 * 20.)], 1280, 720, t);
            tr.choose_focus(1280, 720, t);
            if frame >= 2 {
                assert_eq!(tr.focus, Some(1));
            }
        }
        assert!(
            tr.people(start + Duration::from_millis(1200))[0]
                .kpts
                .is_none()
        );
    }
    #[test]
    fn newcomer_does_not_steal_focus_and_loss_reselects() {
        let mut tr = Tracker::new(FocusConfig::default());
        let start = Instant::now();
        for i in 0..4 {
            let t = start + Duration::from_millis(i * 50);
            tr.update(&[det(100.)], 1280, 720, t);
            tr.choose_focus(1280, 720, t);
        }
        for i in 4..10 {
            let t = start + Duration::from_millis(i * 50);
            tr.update(&[det(100.), det(600.)], 1280, 720, t);
            tr.choose_focus(1280, 720, t);
            assert_eq!(tr.focus, Some(1));
        }
        for i in 10..35 {
            let t = start + Duration::from_millis(i * 50);
            tr.update(&[det(600.)], 1280, 720, t);
            tr.choose_focus(1280, 720, t);
        }
        assert_eq!(tr.focus, Some(2));
    }
    #[test]
    fn losing_face_keeps_person_body_and_stays_awake_then_reacquires_face() {
        let start = Instant::now();
        // Attention sharing is off here: this test is about one person's face.
        let mut tr = Tracker::new(FocusConfig {
            attention_s: [1e6, 1e6],
            ..Default::default()
        });
        let mut life = crate::lifecycle::Lifecycle::new(start);
        let mut curiosity = crate::behavior::Curiosity::default();
        let m = crate::config::MountConfig::default();
        let mut k = [Kpt::default(); NUM_KPTS];
        for (i, x) in [150., 140., 160., 130., 170.].into_iter().enumerate() {
            k[i] = Kpt {
                x,
                y: 150.,
                c: 0.95,
            };
        }
        for frame in 0..950 {
            let t = start + Duration::from_millis(frame * 50);
            let dets = if frame < 5 {
                vec![det(100.)]
            } else {
                vec![det(100.), det(600.)]
            };
            tr.update(&dets, 1280, 720, t);
            // No pose at all in the middle: old facial points must not leak
            // through the person snapshot, even within their historical TTL.
            let face = frame < 5 || frame >= 940;
            if face {
                tr.tracks[0].set_kpts(k, 0.4, t);
            }
            tr.choose_focus(1280, 720, t);
            if frame < 3 {
                continue;
            }
            assert_eq!(tr.focus, Some(1));
            let people = tr.people(t);
            let p = people.iter().find(|p| p.focus).unwrap();
            let target = crate::geometry::target(p, 1280, 720, &m).unwrap();
            assert_eq!(target.face, face);
            assert_eq!(target.pixel, if face { [150., 150.] } else { [150., 300.] });
            curiosity.observe(t, p.id, [0.; 2], target.face && crate::behavior::frontal(p));
            if !face {
                assert_eq!(curiosity.lid(t), 0.);
            }
            // A reachable body is presence, so it must not start the 30 s sleep.
            let d = life.step(
                t,
                Some(crate::lifecycle::Observation {
                    angles: [0.; 2],
                    time: t,
                }),
                [0.; 2],
                0.,
                60.,
                false,
                true,
                true,
                30.,
                5.,
                2.,
            );
            assert!(d.allow_face && !d.rest && !d.reset_focus);
        }
    }
    #[test]
    fn abrupt_camera_reversal_keeps_overlapping_focus() {
        let t = Instant::now();
        let mut tr = Tracker::new(FocusConfig::default());
        tr.update(&[det(400.)], 1280, 720, t);
        tr.focus = Some(1);
        tr.tracks[0].velocity = [2500., 0.];
        tr.update(
            &[det(390.), det(800.)],
            1280,
            720,
            t + Duration::from_millis(80),
        );
        assert!(tr.tracks.iter().any(|p| p.id == 1 && p.seen_now));
        assert!(tr.tracks.iter().find(|p| p.id == 1).unwrap().bbox[0] < 400.);
    }
}
