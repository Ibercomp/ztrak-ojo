//! Latest-frame capture through FFmpeg, as in ZTrak. Platform differences stay here.
use crate::{
    config::CameraConfig,
    shared::{Frame, Rate, Shared},
};
use std::{
    io::{BufRead, BufReader, Read},
    process::{Command, Stdio},
    sync::Arc,
    time::{Duration, Instant},
};

pub fn cameras() -> Vec<(String, String)> {
    let mut list = vec![];
    #[cfg(target_os = "linux")]
    if let Ok(rd) = std::fs::read_dir("/sys/class/video4linux") {
        for e in rd.flatten() {
            let p = e.path();
            if std::fs::read_to_string(p.join("index"))
                .unwrap_or_default()
                .trim()
                != "0"
            {
                continue;
            }
            let name = std::fs::read_to_string(p.join("name"))
                .unwrap_or_default()
                .trim()
                .to_owned();
            list.push((format!("/dev/{}", e.file_name().to_string_lossy()), name));
        }
    }
    #[cfg(target_os = "windows")]
    if let Ok(out) = command()
        .args([
            "-hide_banner",
            "-list_devices",
            "true",
            "-f",
            "dshow",
            "-i",
            "dummy",
        ])
        .output()
    {
        list.extend(parse_dshow_devices(&String::from_utf8_lossy(&out.stderr)));
    }
    list.sort();
    list
}

/// DirectShow video devices from `ffmpeg -list_devices true -f dshow`.
/// A unique camera is addressed by its friendly name (survives changing USB port).
/// Two cameras with the same name use the unique PnP "alternative name" instead,
/// so each one can be chosen (e.g. the game webcam and the eye webcam).
pub fn parse_dshow_devices(text: &str) -> Vec<(String, String)> {
    let mut found: Vec<(String, Option<String>)> = vec![];
    let mut last_was_video = false;
    for line in text.lines() {
        if line.contains("Alternative name") {
            if last_was_video {
                if let (Some(alt), Some(last)) = (line.split('"').nth(1), found.last_mut()) {
                    last.1 = Some(alt.to_owned());
                }
            }
            continue;
        }
        last_was_video = line.contains("(video)");
        if last_was_video {
            if let Some(name) = line.split('"').nth(1) {
                found.push((name.to_owned(), None));
            }
        }
    }
    let mut out = vec![];
    for (i, (name, alt)) in found.iter().enumerate() {
        let same = found.iter().filter(|(n, _)| n == name).count();
        match alt {
            Some(alt) if same > 1 => {
                let n = found[..=i].iter().filter(|(n, _)| n == name).count();
                out.push((format!("video={alt}"), format!("{name} ({n})")));
            }
            _ => out.push((format!("video={name}"), name.clone())),
        }
    }
    out
}

/// Whether a configured local device is currently connected. URLs and files are not checked.
pub fn available(source: &str, list: &[(String, String)]) -> bool {
    if source.starts_with("video=") {
        return list.iter().any(|(s, _)| s == source);
    }
    if source.starts_with("/dev/") {
        return std::path::Path::new(source).exists();
    }
    true
}

/// Webcam the bear game (ZTrak) is configured to use, if any.
pub fn bear_source() -> Option<String> {
    let path = dirs::config_dir()?.join("ztrak").join("config.json");
    let text = std::fs::read_to_string(path).ok()?;
    let v: serde_json::Value = serde_json::from_str(text.trim_start_matches('\u{feff}')).ok()?;
    let source = v["camera"]["source"].as_str()?.trim();
    source.starts_with("video=").then(|| source.to_owned())
}

/// Position in the list of the device a source opens. With identical cameras a
/// plain name opens the first one, listed as "Name (1)".
fn device_index(source: &str, list: &[(String, String)]) -> Option<usize> {
    list.iter().position(|(s, _)| s == source).or_else(|| {
        let first = format!("{} (1)", source.strip_prefix("video=")?);
        list.iter().position(|(_, n)| *n == first)
    })
}

pub fn same_device(a: &str, b: &str, list: &[(String, String)]) -> bool {
    a == b || matches!((device_index(a, list), device_index(b, list)), (Some(x), Some(y)) if x == y)
}

/// Why the eye must not open this source on Windows, if so.
pub fn eye_camera_problem(
    source: &str,
    bear: Option<&str>,
    list: &[(String, String)],
) -> Option<String> {
    let source = source.trim();
    if source.is_empty() {
        return Some("Elige la webcam del ojo en el selector de arriba".into());
    }
    let name = source.strip_prefix("video=")?;
    if bear.is_some_and(|b| same_device(source, b, list)) {
        return Some("Esa webcam es la del oso: elige la del ojo en el selector de arriba".into());
    }
    if !list.iter().any(|(s, _)| s == source)
        && list.iter().any(|(_, n)| *n == format!("{name} (1)"))
    {
        return Some(format!("Hay dos webcams «{name}»: vuelve a elegir la del ojo"));
    }
    None
}

/// Human-readable capture modes reported by the device (formats, sizes, FPS).
pub fn modes(source: &str) -> Vec<String> {
    let mut c = command();
    c.arg("-hide_banner");
    if source.starts_with("video=") {
        c.args(["-f", "dshow", "-list_options", "true", "-i", source]);
    } else if source.starts_with("/dev/") {
        c.args(["-f", "v4l2", "-list_formats", "all", "-i", source]);
    } else {
        return vec!["Solo se pueden consultar modos de webcams locales".into()];
    }
    let Ok(out) = c.output() else {
        return vec!["No se puede ejecutar ffmpeg".into()];
    };
    let mut lines: Vec<String> = String::from_utf8_lossy(&out.stderr)
        .lines()
        .filter(|l| {
            l.contains("vcodec=") || l.contains("pixel_format=") || l.contains("Compressed")
                || l.contains("Raw")
        })
        .map(|l| {
            l.split_once("] ")
                .map_or(l, |(_, r)| r)
                .split(" (")
                .next()
                .unwrap_or("")
                .trim()
                .to_owned()
        })
        .collect();
    lines.dedup();
    if lines.is_empty() {
        lines.push("La cámara no ha informado de modos (¿la usa otro programa?)".into());
    }
    lines
}
fn command() -> Command {
    let local = std::env::current_exe()
        .ok()
        .and_then(|p| {
            p.parent().map(|p| {
                p.join(if cfg!(windows) {
                    "ffmpeg.exe"
                } else {
                    "ffmpeg"
                })
            })
        })
        .filter(|p| p.exists());
    let mut c = Command::new(local.unwrap_or_else(|| "ffmpeg".into()));
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        c.creation_flags(0x08000000);
    }
    c.stdin(Stdio::null());
    c
}
pub fn args(c: &CameraConfig) -> Vec<String> {
    let mut a = vec![
        "-hide_banner".into(),
        "-loglevel".into(),
        "error".into(),
        "-nostdin".into(),
    ];
    if c.source.starts_with("/dev/video") || c.source.starts_with("/dev/v4l/") {
        a.extend(["-f".into(), "v4l2".into()]);
        if !c.input_format.is_empty() {
            a.extend(["-input_format".into(), c.input_format.clone()]);
        }
        a.extend([
            "-video_size".into(),
            format!("{}x{}", c.width, c.height),
            "-framerate".into(),
            c.fps.to_string(),
        ]);
    } else if c.source.starts_with("video=") {
        a.extend([
            "-f".into(),
            "dshow".into(),
            "-rtbufsize".into(),
            "64M".into(),
        ]);
        // DirectShow has no -input_format: compressed formats are chosen with
        // -vcodec, raw ones with -pixel_format. Without this, many webcams fall
        // back to raw YUYV, which is limited to 5-10 FPS at 720p/1080p.
        match c.input_format.trim() {
            "" => {}
            "mjpeg" | "h264" | "hevc" => {
                a.extend(["-vcodec".into(), c.input_format.trim().into()]);
            }
            f => a.extend(["-pixel_format".into(), f.into()]),
        }
        a.extend([
            "-video_size".into(),
            format!("{}x{}", c.width, c.height),
            "-framerate".into(),
            c.fps.to_string(),
        ]);
    } else if c.source.starts_with("rtsp://") {
        a.extend(["-rtsp_transport".into(), "tcp".into()]);
    } else if !c.source.contains("://") {
        a.extend(["-re".into(), "-stream_loop".into(), "-1".into()]);
    }
    a.extend([
        "-i".into(),
        c.source.clone(),
        "-an".into(),
        "-vf".into(),
        format!(
            "scale={}:{}{}",
            c.width,
            c.height,
            if c.rotate_180 { ",hflip,vflip" } else { "" }
        ),
        // Do not duplicate old USB frames to repair capture timestamps.
        "-fps_mode".into(),
        "passthrough".into(),
        "-threads".into(),
        "1".into(),
        "-f".into(),
        "rawvideo".into(),
        "-pix_fmt".into(),
        "rgb24".into(),
        "pipe:1".into(),
    ]);
    a
}
pub fn spawn(shared: Arc<Shared>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        while !shared.quitting() {
            let cfg = shared.cfg().camera;
            if cfg!(windows) {
                if let Some(problem) =
                    eye_camera_problem(&cfg.source, bear_source().as_deref(), &cameras())
                {
                    shared.stats.lock().camera_error = Some(problem);
                    for _ in 0..20 {
                        if shared.quitting() || shared.cfg().camera != cfg {
                            break;
                        }
                        std::thread::sleep(Duration::from_millis(100));
                    }
                    continue;
                }
            }
            let result = (|| -> anyhow::Result<()> {
                let mut child = command()
                    .args(args(&cfg))
                    .stdout(Stdio::piped())
                    .stderr(Stdio::piped())
                    .spawn()?;
                let mut stdout = child.stdout.take().unwrap();
                let stderr = child.stderr.take().unwrap();
                let errors = Arc::new(parking_lot::Mutex::new(String::new()));
                let err = errors.clone();
                let error_reader = std::thread::spawn(move || {
                    for line in BufReader::new(stderr).lines().map_while(Result::ok) {
                        *err.lock() = line.chars().take(600).collect();
                    }
                });
                let s = shared.clone();
                let last_frame = Arc::new(parking_lot::Mutex::new(Instant::now()));
                let lf = last_frame.clone();
                let reader = std::thread::spawn(move || {
                    let mut rate = Rate::default();
                    let mut id = 0;
                    loop {
                        let mut rgb = vec![0u8; (cfg.width * cfg.height * 3) as usize];
                        if stdout.read_exact(&mut rgb).is_err() {
                            break;
                        }
                        let now = Instant::now();
                        *lf.lock() = now;
                        id += 1;
                        rate.tick();
                        s.camera.put(Frame {
                            id,
                            width: cfg.width,
                            height: cfg.height,
                            rgb,
                            time: now,
                        });
                        let mut st = s.stats.lock();
                        st.camera_fps = rate.fps;
                        st.camera_error = None;
                    }
                });
                let mut stalled = false;
                loop {
                    if shared.quitting()
                        || shared.cfg().camera != cfg
                        || child.try_wait()?.is_some()
                    {
                        break;
                    }
                    if last_frame.lock().elapsed() > Duration::from_secs(4) {
                        stalled = true;
                        break;
                    }
                    std::thread::sleep(Duration::from_millis(50));
                }
                let _ = child.kill();
                let _ = child.wait();
                let _ = reader.join();
                let _ = error_reader.join();
                if !shared.quitting() && shared.cfg().camera == cfg {
                    anyhow::bail!(
                        "{}",
                        if stalled {
                            "La cámara no entrega imágenes".into()
                        } else {
                            let e = errors.lock();
                            if e.is_empty() {
                                "Captura interrumpida".into()
                            } else {
                                e.clone()
                            }
                        }
                    );
                }
                Ok(())
            })();
            if let Err(e) = result {
                shared.stats.lock().camera_error = Some(e.to_string());
            }
            for _ in 0..15 {
                if shared.quitting() || shared.cfg().camera != cfg {
                    break;
                }
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn iris_sensor_rotation_and_stable_device_path() {
        let mut c = CameraConfig::default();
        c.source = "/dev/v4l/by-id/usb-5MP-video-index0".into();
        c.rotate_180 = true;
        let a = args(&c);
        assert!(a.iter().any(|s| s == "v4l2"));
        assert!(!a.iter().any(|s| s == "-re"));
        assert!(a.iter().any(|s| s == "scale=1280:720,hflip,vflip"));
    }
    #[test]
    fn windows_device_is_one_argument() {
        let mut c = CameraConfig::default();
        c.source = "video=ASUS FHD webcam".into();
        let a = args(&c);
        assert!(a.iter().any(|s| s == "dshow"));
        assert!(a.iter().any(|s| s == &c.source));
        assert!(!a.iter().any(|s| s == "-re"));
    }
    #[test]
    fn windows_format_selection() {
        let mut c = CameraConfig::default();
        c.source = "video=Creative Live! Cam 4K".into();
        let a = args(&c);
        let i = a.iter().position(|s| s == "-vcodec").expect("mjpeg by default");
        assert_eq!(a[i + 1], "mjpeg");
        assert!(i < a.iter().position(|s| s == "-i").unwrap());
        c.input_format = "yuyv422".into();
        let a = args(&c);
        assert!(a.windows(2).any(|w| w[0] == "-pixel_format" && w[1] == "yuyv422"));
        assert!(!a.iter().any(|s| s == "-input_format" || s == "-vcodec"));
        c.input_format.clear();
        let a = args(&c);
        assert!(!a.iter().any(|s| s == "-pixel_format" || s == "-vcodec"));
    }
    #[test]
    fn dshow_listing_distinguishes_identical_cameras() {
        let text = r#"[in#0 @ 0000] "Creative Live! Cam 4K" (video)
[in#0 @ 0000]   Alternative name "@device_pnp_\\?\usb#vid_041e&pid_40aa#A\global"
[in#0 @ 0000] "Micrófono (Creative Live! Cam 4K Mic)" (audio)
[in#0 @ 0000]   Alternative name "@device_cm_{33D9}\wave_{F02D}"
[in#0 @ 0000] "USB Camera" (video)
[in#0 @ 0000]   Alternative name "@device_pnp_one"
[in#0 @ 0000] "USB Camera" (video)
[in#0 @ 0000]   Alternative name "@device_pnp_two"
Error opening input file dummy."#;
        let l = parse_dshow_devices(text);
        assert_eq!(l.len(), 3);
        assert_eq!(l[0], ("video=Creative Live! Cam 4K".into(), "Creative Live! Cam 4K".into()));
        assert_eq!(l[1], ("video=@device_pnp_one".into(), "USB Camera (1)".into()));
        assert_eq!(l[2], ("video=@device_pnp_two".into(), "USB Camera (2)".into()));
        assert!(available("video=@device_pnp_two", &l));
        assert!(!available("video=Integrated Camera", &l));
        assert!(available("rtsp://camara/stream", &l));
    }
    #[test]
    fn eye_never_opens_the_bear_webcam() {
        let two = vec![
            ("video=@device_pnp_one".to_string(), "USB Camera (1)".to_string()),
            ("video=@device_pnp_two".to_string(), "USB Camera (2)".to_string()),
        ];
        // Nothing chosen: nothing is opened.
        assert!(eye_camera_problem("", None, &two).is_some());
        // The bear's exact device.
        let bear = Some("video=@device_pnp_one");
        assert!(eye_camera_problem("video=@device_pnp_one", bear, &two).is_some());
        assert!(eye_camera_problem("video=@device_pnp_two", bear, &two).is_none());
        // The bear saved a plain name: DirectShow opens the first identical camera.
        let plain = Some("video=USB Camera");
        assert!(eye_camera_problem("video=@device_pnp_one", plain, &two).is_some());
        assert!(eye_camera_problem("video=@device_pnp_two", plain, &two).is_none());
        // A plain name saved before a second identical camera appeared is ambiguous.
        assert!(eye_camera_problem("video=USB Camera", None, &two).is_some());
        // Different models, and files or URLs, are unaffected.
        let one = vec![("video=Eye Cam".to_string(), "Eye Cam".to_string())];
        assert!(eye_camera_problem("video=Eye Cam", Some("video=Creative"), &one).is_none());
        assert!(eye_camera_problem("prueba.mp4", bear, &two).is_none());
    }
    #[test]
    fn preview_mirror_never_enters_capture() {
        assert!(
            !args(&CameraConfig::default())
                .iter()
                .any(|s| s.contains("hflip"))
        );
    }
}
