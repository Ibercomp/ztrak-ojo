use crate::{eye, filter::OneEuro, geometry, shared::Shared};
use std::{
    collections::VecDeque,
    sync::Arc,
    time::{Duration, Instant},
};

#[derive(Default)]
pub struct GazeFilter {
    limits: crate::motion_limits::MotionLimits,
    filters: [OneEuro; 2],
    pupil_filters: [crate::pupil_filter::PupilAxis; 2],
    output: Option<[f32; 2]>,
    person: Option<u64>,
    face_anchor: Option<bool>,
}
impl GazeFilter {
    fn select_anchor(&mut self, face: bool) {
        if self.face_anchor != Some(face) {
            // Face/body offset is not person velocity. Drop prediction history
            // while retaining the command position and its slew limit.
            self.filters = [OneEuro::default(); 2];
            self.pupil_filters = Default::default();
            self.face_anchor = Some(face);
        }
    }

    pub fn step(
        &mut self,
        person: u64,
        target: [f32; 2],
        current: [f32; 2],
        dt: f32,
        cutoff: f32,
        speed: f32,
    ) -> [f32; 2] {
        self.step_tuned(person, target, current, dt, cutoff, speed, 1., 0.)
    }
    #[allow(clippy::too_many_arguments)]
    fn step_tuned(
        &mut self,
        person: u64,
        target: [f32; 2],
        current: [f32; 2],
        dt: f32,
        cutoff: f32,
        speed: f32,
        gain: f32,
        horizon: f32,
    ) -> [f32; 2] {
        let continuing = self.person == Some(person);
        if !continuing {
            self.filters = [OneEuro::default(); 2];
            self.pupil_filters = Default::default();
            self.output = Some(current);
            self.person = Some(person);
        }
        let previous = self.output.unwrap_or(current);
        let mut out = previous;
        for i in 0..2 {
            let destination = if horizon > 0. {
                self.pupil_filters[i].step(
                    target[i],
                    dt,
                    cutoff,
                    if continuing { horizon } else { 0. },
                )
            } else {
                self.filters[i].filter(target[i], dt, cutoff, 0.04)
            };
            let desired = current[i] + gain * (destination - current[i]);
            out[i] = previous[i] + (desired - previous[i]).clamp(-speed * dt, speed * dt);
        }
        out = self.limits.step(previous, out, speed * dt);
        self.output = Some(out);
        out
    }
}
/// Command telemetry at capture time avoids adding an old visual error to
/// a newer eye position. It is an estimate: these servos have no encoders.
#[derive(Default)]
struct PoseHistory(VecDeque<(Instant, [f32; 2])>);
impl PoseHistory {
    fn push(&mut self, stamp: Option<Instant>, pose: [f32; 2]) {
        let Some(t) = stamp else { return };
        if self.0.back().is_some_and(|(last, _)| *last >= t) {
            return;
        }
        self.0.push_back((t, pose));
        while self
            .0
            .front()
            .is_some_and(|(old, _)| t.duration_since(*old) > Duration::from_secs(1))
        {
            self.0.pop_front();
        }
    }
    fn at(&self, t: Instant) -> Option<[f32; 2]> {
        let &(first, p) = self.0.front()?;
        if t <= first {
            return (first.duration_since(t) < Duration::from_millis(200)).then_some(p);
        }
        for (a, b) in self.0.iter().zip(self.0.iter().skip(1)) {
            if t <= b.0 {
                let f = t.duration_since(a.0).as_secs_f32() / b.0.duration_since(a.0).as_secs_f32();
                return Some([
                    a.1[0] + (b.1[0] - a.1[0]) * f,
                    a.1[1] + (b.1[1] - a.1[1]) * f,
                ]);
            }
        }
        let &(last, p) = self.0.back()?;
        (t.duration_since(last) < Duration::from_millis(200)).then_some(p)
    }
}
#[derive(Default)]
pub struct LinkHealth {
    pub failures: u32,
    pub retry_at: Option<Instant>,
}
impl LinkHealth {
    fn failed(&mut self, now: Instant) -> bool {
        self.failures += 1;
        self.retry_at = Some(now + Duration::from_millis(50 * self.failures as u64));
        self.failures < 3
    }
    fn succeeded(&mut self) {
        self.failures = 0;
        self.retry_at = None;
    }
    fn ready(&self, now: Instant) -> bool {
        self.retry_at.is_none_or(|t| now >= t)
    }
}
pub fn spawn(shared: Arc<Shared>) -> std::thread::JoinHandle<()> {
    std::thread::spawn(move || {
        let mut client = eye::Client::default();
        let mut link = LinkHealth::default();
        let mut owned = false;
        let mut owned_address = String::new();
        let mut state = None;
        let mut udp: Option<crate::udp_link::UdpLink> = None;
        let mut last_poll = Instant::now() - Duration::from_secs(2);
        let mut last_send = Instant::now();
        let mut last_scene = None;
        let mut filtered = None;
        let mut filter = GazeFilter::default();
        let mut visual = crate::pid::PupilPid::default();
        let mut ease = crate::visual_servo::EasePose::default();
        let mut last_frame_at = None;
        let mut last_blink = Instant::now() - Duration::from_secs(2);
        let mut last_report = Instant::now();
        let mut history = PoseHistory::default();
        let mut curiosity = crate::behavior::Curiosity::default();
        let mut lifecycle = crate::lifecycle::Lifecycle::new(Instant::now());
        let mut boredom = crate::drowsiness::Drowsiness::new(Instant::now());
        let mut wake_motion = crate::wake_motion::WakeMotion::default();
        let mut behavior_status = None;
        let mut last_mode = shared.cfg().mount.mode;
        let mut last_manual = false;
        let mut manual_drive = crate::manual::ManualDrive::default();
        let mut last_valid_aim = Instant::now();
        while !shared.quitting() {
            let mut cfg = shared.cfg();
            let endpoint = cfg.endpoint();
            let binary_uart = eye::Client::binary_uart(&endpoint);
            if last_mode != cfg.mount.mode {
                last_mode = cfg.mount.mode;
                filter = GazeFilter::default();
                visual = crate::pid::PupilPid::default();
                history = PoseHistory::default();
                lifecycle = crate::lifecycle::Lifecycle::new(Instant::now());
                shared
                    .focus_epoch
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            let now = Instant::now();
            let manual = shared.manual();
            let changed_mode = manual != last_manual;
            last_manual = manual;
            if changed_mode {
                manual_drive = Default::default();
            }
            if owned && (!shared.driving() || endpoint != owned_address || changed_mode) {
                udp = None;
                match client.request(&owned_address, "/apagar") {
                    Ok(s) => {
                        state = Some(s);
                        shared.stats.lock().eye_error = None;
                    }
                    Err(e) => {
                        shared.stats.lock().eye_error = Some(format!(
                            "No se confirmó el apagado; el watchdog detiene la mirada: {e:#}"
                        ))
                    }
                }
                owned = false;
                lifecycle = crate::lifecycle::Lifecycle::new(Instant::now());
                shared
                    .focus_paused
                    .store(false, std::sync::atomic::Ordering::Relaxed);
                link.succeeded();
                if !changed_mode {
                    shared.stop();
                }
                filtered = None;
                filter = GazeFilter::default();
                visual = crate::pid::PupilPid::default();
            }
            if !owned && !shared.driving() {
                if let Some(limits) = shared.limits_request.lock().take() {
                    let result = (|| -> anyhow::Result<eye::EyeState> {
                        limits.validate()?;
                        let up = limits
                            .up
                            .iter()
                            .map(|v| v.to_string())
                            .collect::<Vec<_>>()
                            .join(",");
                        let down = limits
                            .down
                            .iter()
                            .map(|v| v.to_string())
                            .collect::<Vec<_>>()
                            .join(",");
                        let s =
                            client.request(&endpoint, &format!("/limites?up={up}&down={down}"))?;
                        anyhow::ensure!(
                            s.limits == Some(limits),
                            "La placa no confirmó los límites"
                        );
                        Ok(s)
                    })();
                    match result {
                        Ok(s) => {
                            state = Some(s);
                            shared.stats.lock().calibration_message =
                                "Límites guardados y confirmados por la placa".into();
                        }
                        Err(e) => {
                            shared.stats.lock().calibration_message = format!(
                                "No se guardaron: {e:#}. Apaga los servos; para reducir, centra antes el ojo."
                            )
                        }
                    }
                }
            }
            if !owned && last_poll.elapsed() > Duration::from_secs(1) {
                match client.request(&endpoint, "/estado") {
                    Ok(s) => {
                        shared.stats.lock().eye_error = None;
                        state = Some(s);
                        shared.stats.lock().eye_at = Some(Instant::now());
                    }
                    Err(e) => {
                        shared.stats.lock().eye_error = Some(format!("{e:#}"));
                        state = None;
                    }
                }
                last_poll = Instant::now();
            }
            if let Some(channel) = udp.as_mut() {
                match channel.poll() {
                    Ok(Some(s)) => {
                        state = Some(s);
                        shared.stats.lock().eye_at = Some(Instant::now());
                    }
                    Ok(None) => {}
                    Err(e) => {
                        udp = None;
                        link.failed(Instant::now());
                        shared.stats.lock().eye_error = Some(format!("{e:#}"));
                        log::warn!("UDP: {e:#}");
                    }
                }
            }
            if binary_uart {
                match client.poll_uart_motion() {
                    Ok(Some(s))=>{state=Some(s);shared.stats.lock().eye_at=Some(Instant::now());},
                    Ok(None)=>{},
                    Err(e)=>{shared.stats.lock().eye_error=Some(format!("{e:#}"));}
                }
                if owned && state.as_ref().is_some_and(|s|!s.activo) {shared.stop();}
            }
            let transport_paused = udp.as_ref().is_some_and(|u| u.paused()) || (binary_uart && client.uart_paused());
            cfg.mount.limits = state.as_ref().and_then(|s| s.limits).unwrap_or_default();
            cfg.mount.max_speed = cfg.mount.max_speed.min(
                state
                    .as_ref()
                    .and_then(|s| s.gaze_speed_max)
                    .unwrap_or(160.),
            );
            let current = state
                .as_ref()
                .map(|s: &eye::EyeState| s.position())
                .unwrap_or([0.; 2]);
            if let Some(s) = &state {
                history.push(s.sampled_at, current);
            }
            let blinking = state.as_ref().is_some_and(|s| s.parpadeando);
            let lid = state
                .as_ref()
                .and_then(|s| s.actual)
                .map(|a| a[2])
                .unwrap_or(0.);
            let lid_adjusting = state.as_ref().is_some_and(|s| {
                s.parpado_seguimiento
                    .is_some_and(|target| (target - lid).abs() > 0.1)
            });
            let lid_holds_gaze =
                lid_adjusting && state.as_ref().is_some_and(|s| s.reposo == Some(true));
            let control_paused =
                blinking || lid_holds_gaze || link.failures > 0 || transport_paused;
            if control_paused {
                filtered = None;
                filter = GazeFilter::default();
                visual.pause(current);
                last_scene = None;
                last_frame_at = None;
            }
            if blinking || lid > 35. {
                last_blink = now;
            }
            let scene = shared.scene.get();
            let fresh = scene.as_ref().filter(|s| {
                s.focus_epoch
                    == shared
                        .focus_epoch
                        .load(std::sync::atomic::Ordering::Relaxed)
                    && s.time
                        .is_some_and(|t| t.elapsed() < Duration::from_millis(400))
            });
            let person = fresh.and_then(|s| {
                s.people
                    .iter()
                    .find(|p| Some(p.id) == s.focus_id && p.visible)
            });
            let mut aim = person.and_then(|p| {
                let s = fresh.unwrap();
                if cfg.mount.mode == crate::config::MountMode::Pupil {
                    let observed = s.time.and_then(|t| history.at(t)).unwrap_or(current);
                    let mut mount = cfg.mount.clone();
                    mount.pupil_gain = 1.;
                    // Filter the absolute face bearing before applying response.
                    geometry::aim(p, s.width, s.height, &mount, observed)
                } else {
                    geometry::aim(p, s.width, s.height, &cfg.mount, current)
                }
            });
            let camera_fresh = shared
                .camera
                .get()
                .is_some_and(|f| f.time.elapsed() < Duration::from_millis(400));
            // Judge the capture timestamp, not the time inference completed: a
            // frame exposed under a closed lid stays invalid after reopening.
            let pupil_obscured = cfg.mount.mode == crate::config::MountMode::Pupil
                && (blinking
                    || fresh.is_none_or(|s| !pupil_frame_ready(s.time, last_blink))
                    || (person.is_none()
                        && now.saturating_duration_since(last_blink) < Duration::from_millis(250)));
            if !camera_fresh || pupil_obscured || link.failures > 0 {
                aim = None;
            }
            let image_motion = shared.camera.get().is_some_and(|f| {
                let pose = if cfg.mount.mode == crate::config::MountMode::Pupil {
                    [current[0], current[1], lid]
                } else {
                    [0.; 3]
                };
                wake_motion.update(
                    &f,
                    pose,
                    !camera_fresh || pupil_obscured || blinking,
                    cfg.wake_motion_fraction,
                )
            });
            let was_drowsy = boredom.active();
            let drowsy = if shared.tracking()
                && owned
                && camera_fresh
                && cfg.boredom_enabled
                && (lifecycle.idle_available() || was_drowsy)
            {
                let sample = aim.as_ref().and_then(|a| {
                    let p = person?;
                    let size =
                        ((p.bbox[2] - p.bbox[0]).max(1.) * (p.bbox[3] - p.bbox[1]).max(1.)).sqrt();
                    Some(crate::drowsiness::Sample {
                        time: fresh?.time?,
                        id: p.id,
                        face: a.face,
                        bearing: a.raw_angles,
                        size,
                    })
                });
                boredom.step(
                    now,
                    sample,
                    current,
                    lid,
                    cfg.boredom_after_s,
                    cfg.droop_s,
                    cfg.sleep_slit
                        .min(state.as_ref().map(|s| s.parpado_max).unwrap_or(60.)),
                    image_motion,
                )
            } else {
                boredom.reset(now);
                None
            };
            if was_drowsy && !boredom.active() {
                lifecycle.wake(now);
            }
            if image_motion && lifecycle.sleeping() {
                lifecycle.wake(now);
            }
            let behavior = if let Some(d) = drowsy {
                d
            } else if shared.tracking() && owned {
                let observation = aim.as_ref().map(|a| crate::lifecycle::Observation {
                    angles: a.raw_angles,
                    time: fresh.unwrap().time.unwrap(),
                });
                lifecycle.limits = cfg.mount.limits;
                lifecycle.set_obscured(pupil_obscured || lid_adjusting);
                lifecycle.step(
                    now,
                    observation,
                    current,
                    lid,
                    state
                        .as_ref()
                        .map(|s| s.parpado_max)
                        .unwrap_or(60.)
                        .min(cfg.sleep_slit),
                    blinking,
                    // An epoch change invalidates targets, not the camera:
                    // do not restart the absence timer on our own focus reset.
                    camera_fresh
                        && link.failures == 0
                        && scene.as_ref().is_some_and(|s| {
                            s.time
                                .is_some_and(|t| t.elapsed() < Duration::from_millis(400))
                        }),
                    cfg.sleep_enabled,
                    cfg.sleep_after_s,
                    cfg.sleep_for_s,
                    cfg.reset_pause_s,
                )
            } else {
                crate::lifecycle::Decision::default()
            };
            shared
                .focus_paused
                .store(behavior.pause_focus, std::sync::atomic::Ordering::Relaxed);
            if behavior.reset_focus {
                shared
                    .focus_epoch
                    .fetch_add(1, std::sync::atomic::Ordering::Relaxed);
            }
            if manual || !behavior.allow_face {
                aim = None;
                curiosity.reset();
            }
            if behavior.status != behavior_status {
                if let Some(label) = behavior.status {
                    log::info!("Comportamiento: {label}");
                }
                behavior_status = behavior.status;
            }
            if let Some(a) = &mut aim {
                last_valid_aim = now;
                let stamp = fresh.unwrap().time;
                if stamp != last_scene {
                    let dt = stamp
                        .zip(last_frame_at)
                        .map(|(a, b): (Instant, Instant)| {
                            a.saturating_duration_since(b).as_secs_f32()
                        })
                        .unwrap_or(0.04);
                    if shared.tracking()
                        && cfg.curiosity
                        && state
                            .as_ref()
                            .is_some_and(|s| s.parpado_seguimiento.is_some())
                    {
                        let p = person.unwrap();
                        let scene = fresh.unwrap();
                        let centered = (a.pixel[0] - scene.width as f32 / 2.).abs()
                            < scene.width as f32 * 0.055
                            && (a.pixel[1] - scene.height as f32 / 2.).abs()
                                < scene.height as f32 * 0.09;
                        curiosity.observe(
                            now,
                            p.id,
                            a.raw_angles,
                            a.face && centered && crate::behavior::frontal(p),
                        );
                    } else {
                        curiosity.reset();
                    }
                    if !control_paused {
                        let pupil = cfg.mount.mode == crate::config::MountMode::Pupil;
                        filter.limits = cfg.mount.limits;
                        visual.limits = cfg.mount.limits;
                        filter.select_anchor(a.face);
                        filtered = Some(if pupil {
                            // Reproject with a zero origin to get image error,
                            // without treating PWM command position as an encoder.
                            let mut mount = cfg.mount.clone();
                            mount.pupil_gain = 1.;
                            let error = geometry::aim(
                                person.unwrap(),
                                fresh.unwrap().width,
                                fresh.unwrap().height,
                                &mount,
                                [0.; 2],
                            )
                            .unwrap()
                            .raw_angles;
                            let s = cfg.mount.pupil_scale;
                            let error = [error[0] * s[0], error[1] * s[1]];
                            visual.step(
                                a.person,
                                error,
                                current,
                                dt,
                                crate::pid::PidGains {
                                    kp: cfg.mount.pid_kp,
                                    ki: cfg.mount.pid_ki,
                                    kd: cfg.mount.pid_kd,
                                    deadband: cfg.mount.deadband_deg,
                                    max_speed: cfg.mount.max_speed,
                                    backlash: cfg.mount.backlash_deg,
                                },
                            )
                        } else {
                            filter.step_tuned(
                                a.person,
                                a.angles,
                                current,
                                dt.clamp(0.005, 0.1),
                                cfg.mount.smooth_cutoff,
                                cfg.mount.max_speed,
                                1.,
                                0.,
                            )
                        });
                    }
                    if !control_paused {
                        last_scene = stamp;
                        last_frame_at = stamp;
                    }
                }
                if control_paused {
                    a.angles = current;
                } else if let Some(v) = filtered {
                    a.angles = v;
                }
            } else {
                filtered = None;
                if pupil_obscured
                    || behavior.rest
                    || now.duration_since(last_valid_aim) > Duration::from_millis(350)
                {
                    filter = GazeFilter::default();
                    visual.pause(current);
                    last_scene = None;
                    last_frame_at = None;
                }
                if !pupil_obscured {
                    curiosity.reset();
                }
            }
            if !shared.tracking() || !cfg.curiosity {
                curiosity.reset();
            }
            // A blink hides the camera, not the expression: preserve the
            // short curiosity gesture instead of opening/reclosing between it.
            let curious_lid = if camera_fresh && (aim.is_some() || pupil_obscured) {
                curiosity.lid(now)
            } else {
                0.
            };
            if behavior.pose.is_none() {
                ease = crate::visual_servo::EasePose::default();
            }
            if shared.driving() && link.ready(now) {
                let result = (|| -> anyhow::Result<bool> {
                    let mut sent = false;
                    if !owned {
                        let s = state
                            .as_ref()
                            .ok_or_else(|| anyhow::anyhow!("Ojo sin conexión"))?;
                        anyhow::ensure!(
                            s.supports_tracking(),
                            "El ojo necesita el firmware con /seguir"
                        );
                        anyhow::ensure!(
                            s.giro_max >= 55.,
                            "Se necesita firmware OTA 18 para el giro ampliado"
                        );
                        anyhow::ensure!(s.outputs_ready(), "Las salidas de servos no están disponibles");
                        anyhow::ensure!(
                            s.reposo.is_some(),
                            "Se necesita firmware OTA 14 para reposo y búsqueda"
                        );
                        anyhow::ensure!(
                            manual || camera_fresh,
                            "Espera a que llegue imagen de la cámara"
                        );
                        if !manual {
                            state = Some(client.request(&endpoint, "/centrar")?);
                        }
                        owned = true;
                        owned_address = endpoint.clone();
                        state = Some(client.request(&endpoint, "/encender")?);
                        if !cfg.endpoint().starts_with("uart:")
                            && state.as_ref().is_some_and(|s| s.udp_port.is_some())
                        {
                            let s = client.request(&endpoint, "/udp")?;
                            udp = Some(crate::udp_link::UdpLink::new(&endpoint, s.clone())?);
                            log::info!(
                                "Seguimiento UDP: órdenes de 24 bytes, sin espera de respuesta"
                            );
                            state = Some(s);
                        }
                        filter = GazeFilter::default();
                        visual = crate::pid::PupilPid::default();
                        lifecycle = crate::lifecycle::Lifecycle::new(Instant::now());
                        history = PoseHistory::default();
                        filtered = None;
                        last_scene = None;
                        return Ok(true); // Recompute from the newly reported position before moving.
                    }
                    if link.failures > 0 {
                        // Resynchronize after a failed request before sending movement.
                        // In-flight replies and pre-outage velocity are not a new pose.
                        state = Some(client.request(&endpoint, "/estado")?);
                        if binary_uart {anyhow::ensure!(state.as_ref().unwrap().activo,"Uno detenido; vuelve a activar el seguimiento");}
                        if !endpoint.starts_with("uart:")
                            && state.as_ref().is_some_and(|s| s.udp_port.is_some())
                        {
                            anyhow::ensure!(
                                state.as_ref().unwrap().activo
                                    && state.as_ref().unwrap().udp_session != 0,
                                "El control del ojo ha cambiado o caducado; vuelve a iniciar el seguimiento"
                            );
                            let s = client.request(&endpoint, "/udp")?;
                            udp = Some(crate::udp_link::UdpLink::new(&endpoint, s.clone())?);
                            state = Some(s);
                        }
                        filtered = None;
                        filter = GazeFilter::default();
                        visual = crate::pid::PupilPid::default();
                        visual.pause(state.as_ref().unwrap().position());
                        ease = crate::visual_servo::EasePose::default();
                        history = PoseHistory::default();
                        last_scene = None;
                        last_frame_at = None;
                        shared.stats.lock().eye_at = Some(Instant::now());
                        return Ok(true);
                    }
                    if let Some(name) = shared.gesture.lock().take() {
                        if name == "parpadear" && !behavior.rest {
                            state = Some(client.request(&endpoint, "/parpadear")?);
                            sent = true;
                        }
                    }
                    // Hold the gaze and feed the watchdog during an obscured
                    // pupil frame; never queue a face estimate from a closed lid.
                    let command = if manual {
                        Some(manual_drive.step(
                            now,
                            current,
                            *shared.manual_input.lock(),
                            cfg.mount.limits,
                            transport_paused || link.failures > 0,
                        ))
                    } else if behavior.pose.is_some() {
                        if blinking || (lid - behavior.lid).abs() > 0.5 {
                            // Firmware holds gaze while adjusting the lid; keep
                            // the search trajectory clock paused during that hold.
                            ease.pause(now);
                            Some(current)
                        } else {
                            behavior.pose.map(|target| {
                                ease.limits = cfg.mount.limits;
                                ease.step(now, current, target)
                            })
                        }
                    } else if (pupil_obscured || lid_holds_gaze || transport_paused) && camera_fresh
                    {
                        Some(current)
                    } else {
                        // Keep the follow watchdog and automatic blinking alive
                        // while waiting/searching without a target.
                        aim.as_ref()
                            .map(|a| a.angles)
                            .or_else(|| camera_fresh.then_some(current))
                    };
                    if let Some(angles) = command {
                        if last_send.elapsed() >= Duration::from_millis(20) {
                            // 50 Hz is a period between request starts; adding
                            // another idle period after the Wi-Fi response adds lag.
                            last_send = Instant::now();
                            let lid_cmd = if manual {
                                shared.manual_input.lock().lid.clamp(0., 60.)
                            } else if behavior.pose.is_some() {
                                behavior.lid
                            } else {
                                curious_lid
                            };
                            if let Some(channel) = udp.as_mut() {
                                channel.send(
                                    angles,
                                    lid_cmd,
                                    manual || behavior.rest,
                                )?;
                            } else if binary_uart {
                                client.send_uart_motion(angles, lid_cmd, manual || behavior.rest)?;
                            } else {
                                state = Some(client.request(
                                    &endpoint,
                                    &format!(
                                        "/seguir?giro={:.3}&cabeceo={:.3}&parpado={:.0}&reposo={}",
                                        angles[0],
                                        angles[1],
                                        lid_cmd,
                                        u8::from(manual || behavior.rest)
                                    ),
                                )?);
                                shared.stats.lock().eye_at = Some(Instant::now());
                            }
                            {
                                let mut stats = shared.stats.lock();
                                stats.last_command = Some([
                                    angles[0],
                                    angles[1],
                                    lid_cmd,
                                ]);
                                stats.command_at = Some(Instant::now());
                            }
                            sent = true;
                        }
                    } else if udp.is_none() && !binary_uart && last_poll.elapsed() > Duration::from_millis(250) {
                        state = Some(client.request(&endpoint, "/estado")?);
                        sent = true;
                        last_poll = Instant::now();
                    }
                    Ok(sent)
                })();
                match result {
                    Ok(true) => {
                        link.succeeded();
                        shared.stats.lock().eye_error = None;
                    }
                    Ok(false) => {}
                    Err(e) => {
                        let retry = link.failed(Instant::now());
                        log::warn!("Ojo: error de comunicación {}/3: {e:#}", link.failures);
                        shared.stats.lock().eye_error = Some(if retry {
                            format!("Recuperando enlace ({}/3): {e:#}", link.failures)
                        } else {
                            format!("Enlace perdido: {e:#}. Seguimiento detenido.")
                        });
                        if !retry {
                            shared.stop();
                        }
                    }
                }
            }
            if shared.tracking() && last_report.elapsed() >= Duration::from_secs(1) {
                if let Some(a) = &aim {
                    let s = fresh.unwrap();
                    log::info!(
                        "Seguimiento persona={} punto={} error_px=[{:.0},{:.0}] actual=[{:.1},{:.1}] destino=[{:.1},{:.1}] limite={} edad_ms={}",
                        a.person,
                        if a.face { "cara" } else { "cuerpo" },
                        a.pixel[0] - s.width as f32 / 2.,
                        a.pixel[1] - s.height as f32 / 2.,
                        current[0],
                        current[1],
                        a.angles[0],
                        a.angles[1],
                        a.limited,
                        s.time.unwrap().elapsed().as_millis()
                    );
                }
                last_report = Instant::now();
            }
            {
                let mut st = shared.stats.lock();
                st.control_pause = if link.failures > 0 {
                    "link_error"
                } else if transport_paused {
                    "telemetry_wait"
                } else if blinking {
                    "blink"
                } else if lid_holds_gaze {
                    "lid_adjustment"
                } else if pupil_obscured {
                    "pupil_frame_hidden"
                } else if behavior.pause_focus {
                    "behavior_pause"
                } else if !camera_fresh {
                    "camera_stale"
                } else {
                    ""
                }
                .into();
                st.eye = state.clone();
                st.aim = aim.clone();
                st.status = if shared.driving() && (link.failures > 0 || transport_paused) {
                    "Recuperando conexión · mirada en pausa"
                } else if manual {
                    "Control manual · cámara sin controlar la mirada"
                } else if !camera_fresh {
                    "Esperando cámara"
                } else if let Some(status) = behavior.status {
                    status
                } else if shared.tracking() && (blinking || pupil_obscured) {
                    "Parpadeando · mirada en pausa"
                } else if person.is_none() {
                    "Esperando una persona"
                } else if aim.is_none() {
                    "Persona localizada · esperando una posición válida"
                } else if shared.tracking() && curious_lid > 0. {
                    "Mmm… ¿y tú quién eres?"
                } else if shared.tracking() && aim.as_ref().is_some_and(|a| !a.face) {
                    "Siguiendo el cuerpo · buscando la cara"
                } else if shared.tracking() {
                    "Mirando a la cara"
                } else {
                    "Vista previa · sin enviar movimientos"
                }
                .into();
                if shared.tracking()
                    && owned
                    && camera_fresh
                    && cfg.sleep_enabled
                    && person.is_none()
                    && behavior.status.is_none()
                    && !pupil_obscured
                    && !blinking
                    && link.failures == 0
                {
                    st.status = format!(
                        "Sin nadie a la vista · sueño en {:.0} s",
                        lifecycle.sleep_remaining(now, cfg.sleep_after_s).ceil()
                    );
                }
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        if owned {
            let _ = client.request(&owned_address, "/apagar");
        }
    })
}
fn pupil_frame_ready(stamp: Option<Instant>, last_blink: Instant) -> bool {
    stamp.is_some_and(|t| t > last_blink)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn capture_pose_is_interpolated_and_old_samples_expire() {
        let mut h = PoseHistory::default();
        let t = Instant::now();
        h.push(Some(t), [0., 0.]);
        h.push(Some(t + Duration::from_millis(100)), [10., -5.]);
        assert_eq!(h.at(t + Duration::from_millis(50)), Some([5., -2.5]));
        assert!(h.at(t + Duration::from_millis(400)).is_none());
        h.push(Some(t + Duration::from_millis(1200)), [15., 0.]);
        assert!(h.at(t).is_none());
    }
    #[test]
    fn stationary_world_face_does_not_drift_with_camera_motion() {
        // Face bearing 12°, camera was at 2° on exposure, but is now at 8°.
        // Adding the same 10° visual error to the current 8° would chase 18°.
        let mut h = PoseHistory::default();
        let t = Instant::now();
        h.push(Some(t), [2., 0.]);
        h.push(Some(t + Duration::from_millis(100)), [8., 0.]);
        let world = h.at(t).unwrap()[0] + 10.;
        assert_eq!(world, 12.);
        let mut f = GazeFilter::default();
        for _ in 0..20 {
            let result = f.step_tuned(1, [world, 0.], [12., 0.], 0.04, 5.5, 120., 1., 0.08);
            assert!((result[0] - 12.).abs() < 0.001);
        }
    }
    #[test]
    fn prediction_is_bounded_and_reacquisition_never_leads() {
        let mut f = GazeFilter::default();
        f.step_tuned(1, [0., 0.], [0., 0.], 0.04, 5.5, 120., 1., 0.1);
        let a = f.step_tuned(1, [25., 0.], [0., 0.], 0.04, 5.5, 120., 1., 0.1);
        assert!(a[0] <= 4.8);
        let b = f.step_tuned(2, [-25., 0.], a, 0.04, 5.5, 120., 1., 0.1);
        assert!(b[0] >= a[0] - 4.8);
    }
    #[test]
    fn fresh_frame_after_blink_needs_no_extra_delay() {
        let blink = Instant::now();
        assert!(!pupil_frame_ready(
            Some(blink - Duration::from_millis(20)),
            blink
        ));
        assert!(!pupil_frame_ready(Some(blink), blink));
        assert!(pupil_frame_ready(
            Some(blink + Duration::from_millis(1)),
            blink
        ));
        assert!(pupil_frame_ready(
            Some(blink + Duration::from_millis(180)),
            blink
        ));
        assert!(!pupil_frame_ready(None, blink));
    }
    #[test]
    fn one_lost_response_does_not_disable_tracking() {
        let now = Instant::now();
        let mut link = LinkHealth::default();
        assert!(link.failed(now));
        assert!(!link.ready(now));
        assert!(link.ready(now + Duration::from_millis(151)));
        link.succeeded();
        assert_eq!(link.failures, 0);
        assert!(link.failed(now));
        assert!(link.failed(now));
        assert!(!link.failed(now));
    }
    #[test]
    fn reacquisition_is_rate_limited() {
        let mut f = GazeFilter::default();
        let a = f.step(1, [35., 20.], [0., 0.], 0.05, 1.6, 60.);
        assert!(a[0] <= 3. && a[1] <= 3.);
        let b = f.step(2, [-35., -20.], a, 0.05, 1.6, 60.);
        assert!((b[0] - a[0]).abs() <= 3.);
    }
    #[test]
    fn face_body_switch_discards_velocity_but_preserves_slew_limit() {
        let mut f = GazeFilter::default();
        let mut current = [0.; 2];
        f.select_anchor(true);
        for i in 0..12 {
            current = f.step_tuned(1, [i as f32, 0.], current, 0.04, 5.5, 60., 1., 0.1);
        }
        for face in [false, true] {
            f.select_anchor(face);
            let target = [current[0], if face { 15. } else { -15. }];
            let next = f.step_tuned(1, target, current, 0.04, 5.5, 60., 1., 0.1);
            assert!(
                (next[0] - current[0]).abs() < 0.001,
                "stale horizontal prediction"
            );
            assert!((next[1] - current[1]).abs() <= 2.401);
            assert!((next[1] - current[1]) * (target[1] - current[1]) > 0.);
            current = next;
        }
    }
}
