//! Replays the people boxes of a ZTRAK_OJO_TRACE log through the tracker's
//! focus/attention logic and prints every focus change.
//!   cargo run --release --offline --example simular_atencion -- traza.jsonl
use std::time::{Duration, Instant};
use ztrak_ojo::{config::FocusConfig, tracker::Tracker, vision::Detection};

fn main() -> anyhow::Result<()> {
    let path = std::env::args().nth(1).expect("traza.jsonl");
    let start = Instant::now();
    let mut tr = Tracker::new(FocusConfig::default());
    let mut last: Option<u64> = None;
    let mut since = 0f64;
    let (mut switches, mut longest) = (0, 0f64);
    for line in std::fs::read_to_string(path)?.lines() {
        let v: serde_json::Value = serde_json::from_str(line)?;
        let t = v["t"].as_f64().unwrap_or(0.);
        let now = start + Duration::from_secs_f64(t);
        let dets: Vec<Detection> = v["people"]
            .as_array()
            .into_iter()
            .flatten()
            .filter(|p| p["visible"].as_bool() == Some(true))
            .map(|p| {
                let b: Vec<f32> = p["bbox"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap() as f32).collect();
                Detection { bbox: [b[0], b[1], b[2], b[3]], score: p["score"].as_f64().unwrap_or(0.9) as f32, track: None }
            })
            .collect();
        tr.update(&dets, 1280, 720, now);
        tr.choose_focus(1280, 720, now);
        if tr.focus != last {
            if last.is_some() {
                longest = longest.max(t - since);
                switches += 1;
            }
            println!("{t:6.1}s foco {:?} -> {:?} ({:.1}s) personas={}", last, tr.focus, t - since, dets.len());
            last = tr.focus;
            since = t;
        }
    }
    println!("cambios={switches} mirada más larga={longest:.1}s");
    Ok(())
}
