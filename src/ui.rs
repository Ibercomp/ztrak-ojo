use eframe::egui::{
    self, Align2, Color32, ColorImage, FontId, Pos2, Rect, Stroke, StrokeKind, TextureHandle,
    TextureOptions, Vec2,
};
use std::{sync::Arc, time::Duration};
use ztrak_ojo::{
    camera,
    config::{Config, DepthMode, MountMode},
    shared::Shared,
};
const ACCENT: Color32 = Color32::from_rgb(71, 211, 185);
const AMBER: Color32 = Color32::from_rgb(246, 190, 97);

pub struct App {
    shared: Arc<Shared>,
    edit: Config,
    saved: Config,
    texture: Option<TextureHandle>,
    frame_at: Option<std::time::Instant>,
    cameras: Vec<(String, String)>,
    bear_camera: Option<String>,
    modes: Vec<String>,
    message: String,
    tab: usize,
    limits_edit: Option<ztrak_ojo::motion_limits::MotionLimits>,
    manual_speed: f32,
    manual_center: bool,
    manual_target: Option<[f32; 2]>,
    manual_ui_at: Option<std::time::Instant>,
    calibration: bool,
    calibration_lid: f32,
}
impl App {
    pub fn new(cc: &eframe::CreationContext<'_>, shared: Arc<Shared>) -> Self {
        cc.egui_ctx.set_visuals(egui::Visuals::dark());
        let mut style = (*cc.egui_ctx.style_of(egui::Theme::Dark)).clone();
        style.spacing.item_spacing = Vec2::new(10., 9.);
        style.spacing.button_padding = Vec2::new(12., 8.);
        cc.egui_ctx.set_style_of(egui::Theme::Dark, style);
        let edit = shared.cfg();
        Self {
            shared,
            saved: edit.clone(),
            edit,
            texture: None,
            frame_at: None,
            cameras: camera::cameras(),
            bear_camera: camera::bear_source(),
            modes: Vec::new(),
            message: String::new(),
            tab: 0,
            limits_edit: None,
            manual_speed: 35.,
            manual_center: false,
            manual_target: None,
            manual_ui_at: None,
            calibration: false,
            calibration_lid: 0.,
        }
    }
    /// Why the eye cannot use the saved webcam on Windows, from the cached list.
    fn camera_problem(&self) -> Option<String> {
        if !cfg!(windows) {
            return None;
        }
        camera::eye_camera_problem(&self.saved.camera.source, self.bear_camera.as_deref(), &self.cameras)
    }
    fn refresh_cameras(&mut self) {
        self.cameras = camera::cameras();
        self.bear_camera = camera::bear_source();
    }
    /// Applies and saves only the webcam, so the preview shows it at once.
    fn choose_camera(&mut self, source: String) {
        // Never keep the servos following while the image source changes.
        if self.shared.tracking() {
            self.shared.stop();
        }
        let mut next = self.saved.clone();
        next.camera.source = source.clone();
        match next.save() {
            Ok(()) => {
                *self.shared.config.lock() = next.clone();
                self.saved = next;
                self.edit.camera.source = source;
                self.modes.clear();
                self.message = "Webcam del ojo guardada. Comprueba en la imagen que es la suya.".into();
            }
            Err(e) => self.message = format!("No se pudo guardar la webcam: {e:#}"),
        }
    }
    /// Windows webcam selector. The bear's webcam is listed but cannot be chosen.
    fn camera_picker(&mut self, ui: &mut egui::Ui) {
        let source = self.saved.camera.source.clone();
        let current = if source.trim().is_empty() {
            "Elige la webcam del ojo…".to_string()
        } else {
            self.cameras
                .iter()
                .find(|(s, _)| *s == source)
                .map(|(_, n)| n.clone())
                .unwrap_or_else(|| format!("{source} (no conectada)"))
        };
        let mut chosen = None;
        ui.label("Webcam del ojo:");
        egui::ComboBox::from_id_salt("webcam_ojo")
            .selected_text(current)
            .width(260.)
            .show_ui(ui, |ui| {
                for (s, name) in &self.cameras {
                    let bear = self
                        .bear_camera
                        .as_deref()
                        .is_some_and(|b| camera::same_device(s, b, &self.cameras));
                    let label = if bear { format!("{name} (la del oso)") } else { name.clone() };
                    if ui
                        .add_enabled(!bear, egui::Button::selectable(*s == source, label))
                        .clicked()
                    {
                        chosen = Some(s.clone());
                    }
                }
            });
        if ui.button("Actualizar").clicked() {
            self.refresh_cameras();
        }
        if let Some(s) = chosen {
            self.choose_camera(s);
        }
    }
    fn settings(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.tab, 0, "Montaje");
            ui.selectable_value(&mut self.tab, 1, "Cámara");
            ui.selectable_value(&mut self.tab, 2, "Ojo");
            ui.selectable_value(&mut self.tab, 3, "Manual");
        });
        ui.separator();
        egui::ScrollArea::vertical().max_height((ui.available_height()-105.).max(200.)).show(ui,|ui|{
   match self.tab{
    0=>{
     ui.heading("Dónde está la cámara");
     let mut pupil=self.edit.mount.mode==MountMode::Pupil;
     if ui.checkbox(&mut pupil,"Cámara montada en la pupila").changed(){self.edit.mount.mode=if pupil{MountMode::Pupil}else{MountMode::External};}
     ui.small(if pupil{"La cámara gira con el ojo"}else{"Cámara externa: ajustar posición respecto al ojo"});
     ui.add_space(6.);
     if self.edit.mount.mode==MountMode::External{
      ui.label("Posición de la cámara respecto al ojo");
      ui.small("Visto desde detrás del ojo, mirando hacia la gente. Negativo = izquierda / abajo / detrás.");
      for (i,label)in ["Derecha (+) / izquierda (−)","Arriba (+) / abajo (−)","Delante (+) / detrás (−)"].iter().enumerate(){
       let mut cm=self.edit.mount.camera_xyz[i]*100.;
       ui.horizontal(|ui|{ui.label(*label);if ui.add(egui::DragValue::new(&mut cm).speed(1.).range(-1000. ..=1000.).suffix(" cm")).changed(){self.edit.mount.camera_xyz[i]=cm/100.;}});
      }
      ui.separator();ui.label("Distancia frontal cámara → persona");
      ui.selectable_value(&mut self.edit.mount.depth_mode,DepthMode::Manual,"Distancia aproximada fija");
      ui.selectable_value(&mut self.edit.mount.depth_mode,DepthMode::Shoulders,"Estimar por los hombros");
      ui.horizontal(|ui|{ui.label(if self.edit.mount.depth_mode==DepthMode::Manual{"Distancia"}else{"Si no se ven bien"});ui.add(egui::DragValue::new(&mut self.edit.mount.depth_m).speed(0.05).range(0.3..=20.).suffix(" m"));});
      if self.edit.mount.depth_mode==DepthMode::Shoulders{ui.add(egui::Slider::new(&mut self.edit.mount.shoulder_width_m,0.15..=0.8).text("ancho hombros (m)"));ui.colored_label(AMBER,"Es una estimación: ropa, giro y tamaño de la persona cambian el resultado.");}
     }else{
      ui.checkbox(&mut self.edit.curiosity,"Gestos curiosos al mirarnos fijamente");
      ui.label("La cámara se mueve con el ojo. El control corrige la mirada hasta centrar la cabeza.");
      ui.small("No usa el desplazamiento externo ni necesita distancia. Alinea primero la cámara con el eje de la pupila.");
      ui.label("PID de pupila (velocidad de mirada = Kp·error + Ki·∫error + Kd·d(error)/dt)");
      ui.add(egui::Slider::new(&mut self.edit.mount.pid_kp,0. ..=20.).text("Kp (1/s)"));
      ui.add(egui::Slider::new(&mut self.edit.mount.pid_ki,0. ..=20.).text("Ki (1/s²)"));
      ui.add(egui::Slider::new(&mut self.edit.mount.pid_kd,0. ..=2.).text("Kd"));
     }
     ui.separator();ui.label("Óptica y orientación");
     ui.add(egui::Slider::new(&mut self.edit.mount.horizontal_fov,25. ..=120.).text("campo horizontal °"));
     ui.small("65° es un valor inicial, no una calibración de esta webcam.");
     for (i,label)in ["Cámara hacia la derecha °","Cámara hacia arriba °","Rotación de imagen °"].iter().enumerate(){ui.horizontal(|ui|{ui.label(*label);ui.add(egui::DragValue::new(&mut self.edit.mount.camera_ypr[i]).speed(0.2).range(-180. ..=180.));});}
     ui.separator();ui.label("Ajuste fino de la mirada");
     for (i,label)in ["Horizontal °","Vertical °"].iter().enumerate(){ui.horizontal(|ui|{ui.label(*label);ui.add(egui::DragValue::new(&mut self.edit.mount.trim[i]).speed(0.1).range(-30. ..=30.));ui.checkbox(&mut self.edit.mount.invert[i],if i==0{"Invertir horizontal"}else{"Invertir vertical"});});}
     egui::CollapsingHeader::new("Suavidad y seguimiento").show(ui,|ui|{
      ui.add(egui::Slider::new(&mut self.edit.mount.deadband_deg,0.1..=5.).text("zona quieta pupila °"));
      if self.edit.mount.mode==MountMode::External { ui.add(egui::Slider::new(&mut self.edit.mount.smooth_cutoff,0.2..=8.).text("respuesta del filtro")); }
      ui.add(egui::Slider::new(&mut self.edit.mount.max_speed,5. ..=160.).text("velocidad máx. °/s"));
      ui.add(egui::Slider::new(&mut self.edit.focus.lost_timeout,0.1..=3.).text("margen de pérdida (s)"));
     });
    },
    1=>{
     ui.heading("Entrada de vídeo");
     if cfg!(windows){ui.small("La webcam del ojo se elige en el selector de arriba.");}else{
     let current=self.cameras.iter().find(|(s,_)|*s==self.edit.camera.source).map(|(_,n)|n.clone()).unwrap_or_else(||format!("{} (no conectada)",self.edit.camera.source));
     egui::ComboBox::from_label("Webcam").selected_text(current).show_ui(ui,|ui|{for(source,name)in &self.cameras{if ui.selectable_value(&mut self.edit.camera.source,source.clone(),name).clicked(){self.modes.clear();}}});
     }
     ui.horizontal(|ui|{
      if ui.button("Actualizar lista").clicked(){self.refresh_cameras();}
      if ui.button("Ver modos de esta cámara").clicked(){self.modes=camera::modes(&self.edit.camera.source);}
     });
     if self.cameras.is_empty(){ui.colored_label(AMBER,"No se encuentra ninguna webcam. ¿Está conectada? ¿Está ffmpeg junto al programa?");}
     if !self.modes.is_empty(){egui::ScrollArea::vertical().id_salt("modos").max_height(140.).show(ui,|ui|{for m in &self.modes{ui.small(m);}});}
     ui.label("Dispositivo, URL o archivo");ui.text_edit_singleline(&mut self.edit.camera.source);
     ui.horizontal(|ui|{ui.label("Tamaño");ui.add(egui::DragValue::new(&mut self.edit.camera.width).range(160..=3840));ui.label("×");ui.add(egui::DragValue::new(&mut self.edit.camera.height).range(120..=2160));});
     ui.add(egui::Slider::new(&mut self.edit.camera.fps,5..=60).text("FPS"));
     ui.horizontal(|ui|{ui.label("Formato");ui.text_edit_singleline(&mut self.edit.camera.input_format);});
     ui.small("mjpeg recomendado (más FPS a 720p/1080p). Vacío = el que elija la cámara; también yuyv422, nv12, h264…");
     ui.checkbox(&mut self.edit.camera.rotate_180,"Corregir montaje boca abajo (180°)");
     ui.checkbox(&mut self.edit.mirror_preview,"Vista espejo");
     ui.small("Solo cambia la imagen en pantalla. El sentido físico del ojo se ajusta en Montaje, con Invertir horizontal.");
     ui.separator();ui.heading("Detección");
     ui.add(egui::Slider::new(&mut self.edit.vision.detector_threshold,0.15..=0.8).text("confianza persona"));
     ui.add(egui::Slider::new(&mut self.edit.vision.max_poses,1..=8).text("personas con puntos"));
     ui.add(egui::Slider::new(&mut self.edit.vision.threads,1..=16).text("hilos IA"));
     if cfg!(windows){
      ui.horizontal(|ui|{ui.label("Esqueleto en");ui.selectable_value(&mut self.edit.vision.device,"auto".to_string(),"Gráfica");ui.selectable_value(&mut self.edit.vision.device,"cpu".to_string(),"CPU");});
      let device=self.shared.stats.lock().pose_device.clone();
      if !device.is_empty(){ui.small(format!("Ahora: {device}"));}
     }
     ui.small("Se mantiene la persona elegida mientras siga visible. Una persona nueva no le quita el foco.");
    },
    3=>self.manual_controls(ui),
    _=>{
     ui.heading("Conexión con el ojo");
     ui.horizontal(|ui|{ui.selectable_value(&mut self.edit.uart,false,"Wi-Fi / UDP");ui.selectable_value(&mut self.edit.uart,true,"UART / USB-serie");});
     if self.edit.uart {
      ui.label("Puerto serie");ui.text_edit_singleline(&mut self.edit.uart_port);
      egui::ComboBox::from_label("Puertos disponibles").selected_text(&self.edit.uart_port).show_ui(ui,|ui|{for p in serialport::available_ports().unwrap_or_default(){let label=match &p.port_type{serialport::SerialPortType::UsbPort(u)=>format!("{} · {}",p.port_name,u.product.as_deref().unwrap_or("USB")),_=>p.port_name.clone()};ui.selectable_value(&mut self.edit.uart_port,p.port_name.clone(),label);}});
      egui::ComboBox::from_label("Baudios").selected_text(self.edit.uart_baud.to_string()).show_ui(ui,|ui|{
       for baud in [115200,230400,250000] {ui.selectable_value(&mut self.edit.uart_baud,baud,baud.to_string());}
      });
      ui.small("8N1 · Uno R3: USB directo · ESP: UART externo de 3,3 V");
     }else{ui.label("IP y puerto");ui.text_edit_singleline(&mut self.edit.eye_address);}
     ui.separator();ui.checkbox(&mut self.edit.sleep_enabled,"Dormirse cuando no hay nadie");
     ui.add(egui::Slider::new(&mut self.edit.sleep_after_s,5. ..=300.).text("sin nadie antes de dormir (s)"));
     ui.add(egui::Slider::new(&mut self.edit.sleep_for_s,1. ..=60.).text("dormir entre búsquedas (s)"));
     ui.add(egui::Slider::new(&mut self.edit.reset_pause_s,0.5..=10.).text("pausa al volver al centro (s)"));
     ui.separator();ui.checkbox(&mut self.edit.boredom_enabled,"Aburrirse si la persona permanece quieta");
     ui.add(egui::Slider::new(&mut self.edit.boredom_after_s,8. ..=180.).text("quietud antes de aburrirse (s)"));
     ui.add(egui::Slider::new(&mut self.edit.droop_s,0.6..=5.).text("cerrar poco a poco (s)"));
     ui.add(egui::Slider::new(&mut self.edit.sleep_slit,20. ..=34.).text("cierre al dormitar (°)"));
     ui.add(egui::Slider::new(&mut self.edit.wake_motion_fraction,0.01..=0.15).text("umbral de movimiento"));
     ui.small("Más cierre deja menos visión. Menor umbral despierta con movimientos más pequeños; se confirman durante 200 ms.");
     let st=self.shared.stats.lock().clone();
     if let Some(s)=st.eye{
      ui.label(format!("Firmware: {}",s.firmware));
      ui.label(format!("Posición ordenada: {:.1}° / {:.1}°",s.position()[0],s.position()[1]));
      ui.small("La placa informa de su consigna; los servos no devuelven una medición de posición.");
      ui.label(format!("Centros guardados: {:?} µs",s.centro));
     }
     ui.separator();ui.label("Activar seguimiento enciende los servos. Parar los apaga. Al cerrar esta app también se solicita el apagado.");
     ui.label("Si deja de llegar seguimiento, la placa detiene la mirada. Los parpadeos se coordinan dentro del firmware.");
     ui.small("En Manual puedes mover el ojo y calibrar la curva de límites guardada en la placa.");
    }

   }
  });
        ui.add_space(12.);
        ui.separator();
        if ui.button("Aplicar y guardar ajustes").clicked() {
            match self.edit.save() {
                Ok(()) => {
                    *self.shared.config.lock() = self.edit.clone();
                    self.saved = self.edit.clone();
                    self.message = "Ajustes guardados".into();
                }
                Err(e) => self.message = e.to_string(),
            }
        }
        if self.edit != self.saved {
            ui.colored_label(AMBER, "Hay ajustes sin aplicar");
        }
        if !self.message.is_empty() {
            ui.label(&self.message);
        }
    }
    fn manual_controls(&mut self, ui: &mut egui::Ui) {
        use ztrak_ojo::motion_limits::{MotionLimits, YAW};
        let st = self.shared.stats.lock().clone();
        let board = st.eye.as_ref().and_then(|s| s.limits);
        let limits = board.unwrap_or_default();
        let actual = st.eye.as_ref().map(|s| s.position()).unwrap_or([0.; 2]);
        let now = std::time::Instant::now();
        if self
            .manual_ui_at
            .is_none_or(|t| now.duration_since(t) > Duration::from_millis(200))
            || !ui.input(|i| i.viewport().focused.unwrap_or(true))
        {
            self.manual_target = Some(actual);
        }
        self.manual_ui_at = Some(now);
        ui.heading("Posición manual");
        ui.small("Haz clic o arrastra hasta la posición deseada. El círculo vacío es el destino; el punto verde es la posición ordenada por la placa. Detener cancela el desplazamiento.");
        ui.horizontal(|ui| {
            if self.shared.manual() {
                if ui.button("Apagar").clicked() {
                    self.shared.stop();
                    self.manual_center = false;
                    self.calibration = false;
                    self.calibration_lid = 0.;
                }
                if ui.button("Detener").clicked() || ui.input(|i| i.key_pressed(egui::Key::Space)) {
                    self.manual_target = Some(actual);
                    self.manual_center = false;
                }
                if ui.button("Centrar").clicked() {
                    self.manual_target = Some([0.; 2]);
                    self.manual_center = false;
                }
            } else if ui
                .add_enabled(board.is_some(), egui::Button::new("Activar control manual"))
                .clicked()
            {
                self.shared.set_manual(true);
                self.manual_target = Some(actual);
                self.manual_center = false;
                self.calibration = false;
            }
            if !self.shared.manual()
                && ui
                    .add_enabled(board.is_some(), egui::Button::new("Modo calibración"))
                    .clicked()
            {
                self.shared.set_manual(true);
                self.manual_target = Some([0.; 2]);
                self.manual_center = false;
                self.calibration = true;
                self.calibration_lid = 0.;
            }
        });
        if !self.shared.manual() {
            self.calibration = false;
            self.calibration_lid = 0.;
        }
        if self.calibration {
            ui.group(|ui| {
                ui.label("Calibración: ojo centrado. Abre y cierra el párpado.");
                ui.horizontal(|ui| {
                    if ui.button("Centrar ojo").clicked() {
                        self.manual_target = Some([0.; 2]);
                    }
                    if ui.button("Abrir").clicked() {
                        self.calibration_lid = 0.;
                    }
                    if ui.button("Medio").clicked() {
                        self.calibration_lid = 30.;
                    }
                    if ui.button("Cerrar").clicked() {
                        self.calibration_lid = 60.;
                    }
                });
                ui.add(
                    egui::Slider::new(&mut self.calibration_lid, 0. ..=60.)
                        .text("párpado (0 abierto · 60 cerrado)"),
                );
            });
        }
        if board.is_none() {
            ui.colored_label(
                AMBER,
                "Hace falta el firmware OTA 20 para calibrar los límites desde aquí.",
            );
        }
        ui.add(egui::Slider::new(&mut self.manual_speed, 1. ..=60.).text("velocidad °/s"));
        ui.horizontal(|ui| {
            if ui.button("Fino · 4°/s").clicked() {
                self.manual_speed = 4.;
            }
            if ui.button("Normal · 35°/s").clicked() {
                self.manual_speed = 35.;
            }
        });
        let size = ui.available_width().min(320.);
        let (rect, response) =
            ui.allocate_exact_size(Vec2::new(size, size * 0.7), egui::Sense::click_and_drag());
        let center = rect.center();
        let half = rect.size() * 0.46;
        let pos = |a: [f32; 2]| center + Vec2::new(a[0] / 55. * half.x, -a[1] / 35. * half.y);
        let painter = ui.painter();
        painter.rect_filled(rect, 8., Color32::from_rgb(13, 19, 25));
        let mut poly: Vec<Pos2> = YAW
            .iter()
            .enumerate()
            .map(|(i, x)| pos([*x, limits.up[i] as f32 * 0.1]))
            .collect();
        poly.extend(
            YAW.iter()
                .enumerate()
                .rev()
                .map(|(i, x)| pos([*x, -limits.down[i] as f32 * 0.1])),
        );
        painter.add(egui::Shape::convex_polygon(
            poly,
            Color32::from_rgba_unmultiplied(30, 100, 90, 70),
            Stroke::new(1.5, ACCENT),
        ));
        painter.line_segment(
            [
                Pos2::new(rect.left() + 8., center.y),
                Pos2::new(rect.right() - 8., center.y),
            ],
            Stroke::new(1., Color32::DARK_GRAY),
        );
        painter.line_segment(
            [
                Pos2::new(center.x, rect.top() + 8.),
                Pos2::new(center.x, rect.bottom() - 8.),
            ],
            Stroke::new(1., Color32::DARK_GRAY),
        );
        painter.circle_filled(pos(actual), 5., ACCENT);
        if self.shared.manual() && response.is_pointer_button_down_on() {
            if let Some(p) = response.interact_pointer_pos() {
                self.manual_target = Some(limits.tracking([
                    (p.x - center.x) / half.x * 55.,
                    (center.y - p.y) / half.y * 35.,
                ]));
            }
        }
        if let Some(target) = self.manual_target {
            painter.circle_stroke(pos(target), 8., Stroke::new(2., AMBER));
            painter.line_segment([pos(actual), pos(target)], Stroke::new(1., AMBER));
        }
        ui.horizontal(|ui| {
            for (text, axis, sign) in [
                ("G- 0,5°", 0, -1.),
                ("G+ 0,5°", 0, 1.),
                ("V- 0,5°", 1, -1.),
                ("V+ 0,5°", 1, 1.),
            ] {
                if ui
                    .add_enabled(self.shared.manual(), egui::Button::new(text))
                    .clicked()
                {
                    let mut p = self.manual_target.unwrap_or(actual);
                    p[axis] += sign * 0.5;
                    self.manual_target = Some(if axis == 1 {
                        limits.envelope(p)
                    } else {
                        limits.tracking(p)
                    });
                }
            }
        });
        if self.shared.manual() {
            *self.shared.manual_input.lock() = ztrak_ojo::manual::ManualInput {
                stick: [0.; 2],
                target: self.manual_target,
                speed: self.manual_speed,
                stamp: now,
                center: false,
                lid: if self.calibration { self.calibration_lid } else { 0. },
            };
        }
        ui.label(format!(
            "Giro {:+.1}° · Vertical {:+.1}°",
            actual[0], actual[1]
        ));
        let vertical = limits.pitch(actual[0]);
        ui.small(format!(
            "A este giro: vertical {:.1}° … +{:.1}°",
            vertical[0], vertical[1]
        ));
        ui.small("El punto indica la consigna de la placa, no una medida por encoder. La zona verde son sus límites actuales.");
        ui.separator();
        egui::CollapsingHeader::new("Ajustar límites combinados").show(ui,|ui|{
            let edit=self.limits_edit.get_or_insert(limits);
            ui.label("Margen vertical para cada posición horizontal");
            ui.small("Amplía de 0,5° en 0,5° y prueba despacio. Guarda un margen antes del roce. Las dos filas permiten ajustar cada sentido por separado.");
            egui::Grid::new("mechanical_limits").num_columns(3).show(ui,|ui|{
                ui.label("Giro");ui.label("Vertical +");ui.label("Vertical −");ui.end_row();
                for (i,x) in YAW.iter().enumerate(){
                    ui.label(format!("{x:+.0}°"));
                    for row in [&mut edit.up,&mut edit.down]{let mut v=row[i] as f32*0.1;if ui.add(egui::DragValue::new(&mut v).speed(0.5).range(0. ..=35.).suffix("°")).changed(){row[i]=(v*10.).round() as i16;}}
                    ui.end_row();
                }
            });
            let valid=edit.validate();if let Err(e)=&valid{ui.colored_label(AMBER,e.to_string());}
            let off=!self.shared.driving()&&st.eye.as_ref().is_some_and(|s|!s.activo);
            if ui.add_enabled(off&&board.is_some()&&valid.is_ok(),egui::Button::new("Guardar límites en la placa")).clicked(){
                *self.shared.limits_request.lock()=Some(*edit);self.shared.stats.lock().calibration_message="Guardando…".into();
            }
            if !off{ui.small("Apaga los servos para guardar. Antes de reducir una curva, vuelve al centro.");}
            ui.horizontal(|ui|{
                if ui.button("Leer de la placa").clicked(){*edit=limits;}
                if ui.button("Valores iniciales").clicked(){*edit=MotionLimits::default();}
            });
            if !st.calibration_message.is_empty(){ui.label(&st.calibration_message);}
        });
    }
    fn preview(&mut self, ui: &mut egui::Ui) {
        let st = self.shared.stats.lock().clone();
        ui.horizontal(|ui| {
            ui.heading(&st.status);
        });
        ui.label(format!(
            "Cámara {:.0} FPS · Visión {:.0} FPS",
            st.camera_fps, st.vision_fps
        ));
        let available = ui.available_size();
        let (area, _) = ui.allocate_exact_size(
            Vec2::new(available.x, (available.y - 150.).max(240.)),
            egui::Sense::hover(),
        );
        let painter = ui.painter();
        painter.rect_filled(area, 10., Color32::from_rgb(13, 19, 25));
        if let Some(tex) = &self.texture {
            let size = tex.size_vec2();
            let scale = (area.width() / size.x).min(area.height() / size.y);
            let r = Rect::from_center_size(area.center(), size * scale);
            let mirror = self.shared.cfg().mirror_preview;
            let uv = if mirror {
                Rect::from_min_max(Pos2::new(1., 0.), Pos2::new(0., 1.))
            } else {
                Rect::from_min_max(Pos2::ZERO, Pos2::new(1., 1.))
            };
            painter.image(tex.id(), r, uv, Color32::WHITE);
            let pos = |x: f32, y: f32| {
                Pos2::new(
                    r.left() + (if mirror { size.x - x } else { x }) * scale,
                    r.top() + y * scale,
                )
            };
            let center = r.center();
            painter.line_segment(
                [center - Vec2::new(12., 0.), center + Vec2::new(12., 0.)],
                Stroke::new(1., Color32::WHITE),
            );
            painter.line_segment(
                [center - Vec2::new(0., 12.), center + Vec2::new(0., 12.)],
                Stroke::new(1., Color32::WHITE),
            );
            if let Some(s) = self.shared.scene.get().filter(|s| {
                s.time
                    .is_some_and(|t| t.elapsed() < Duration::from_millis(500))
            }) {
                for p in &s.people {
                    let color = if p.focus {
                        ACCENT
                    } else {
                        Color32::from_white_alpha(140)
                    };
                    let b =
                        Rect::from_two_pos(pos(p.bbox[0], p.bbox[1]), pos(p.bbox[2], p.bbox[3]));
                    painter.rect_stroke(
                        b,
                        6.,
                        Stroke::new(1., Color32::from_white_alpha(60)),
                        StrokeKind::Inside,
                    );
                    painter.text(
                        b.left_top() + Vec2::new(7., 8.),
                        Align2::LEFT_TOP,
                        format!(
                            "{} #{}",
                            if p.focus {
                                "Persona elegida"
                            } else {
                                "Persona"
                            },
                            p.id
                        ),
                        FontId::proportional(16.),
                        color,
                    );
                    if let Some(k) = p.kpts {
                        for point in k.iter().take(5).filter(|p| p.c > 0.4) {
                            painter.circle_filled(pos(point.x, point.y), 3., color);
                        }
                    }
                }
            }
            if let Some(a) = &st.aim {
                let q = pos(a.pixel[0], a.pixel[1]);
                painter.circle_stroke(q, 13., Stroke::new(3., ACCENT));
                painter.text(
                    q + Vec2::new(18., -18.),
                    Align2::LEFT_BOTTOM,
                    if a.face { "CARA" } else { "CUERPO" },
                    FontId::proportional(16.),
                    ACCENT,
                );
                painter.line_segment([center, q], Stroke::new(1., ACCENT));
            }
        } else {
            painter.text(
                area.center(),
                Align2::CENTER_CENTER,
                if cfg!(windows) && self.saved.camera.source.trim().is_empty() {
                    "Elige la webcam del ojo en el selector de arriba"
                } else {
                    "Abriendo la cámara…"
                },
                FontId::proportional(20.),
                Color32::GRAY,
            );
        }
        ui.add_space(10.);
        if self.shared.manual() {
            if let Some(s) = &st.eye {
                let p = s.position();
                ui.label(
                    egui::RichText::new(format!(
                        "Manual · Giro {:+.1}°    Vertical {:+.1}°",
                        p[0], p[1]
                    ))
                    .size(25.)
                    .color(ACCENT),
                );
            }
            ui.label(
                "La mirada se controla con el joystick. La detección de cámara no mueve el ojo.",
            );
        } else if let Some(a) = &st.aim {
            ui.horizontal(|ui| {
                ui.label(
                    egui::RichText::new(format!(
                        "Giro {:.1}°    Cabeceo {:.1}°",
                        a.angles[0], a.angles[1]
                    ))
                    .size(25.)
                    .color(ACCENT),
                );
                if a.limited {
                    ui.colored_label(AMBER, "Fuera de alcance · ángulo limitado");
                }
            });
            if self.shared.cfg().mount.mode == MountMode::External {
                ui.label(format!("Distancia {}: {:.2} m · variación angular hasta ±{:.1}° si la distancia cambia −35 % / +50 %",if a.estimated{"estimada por hombros"}else{"configurada"},a.depth,a.uncertainty_deg));
            } else {
                ui.label("Control por centrado de imagen · sin estimar distancia");
            }
        } else {
            ui.label("Cuando entre una persona se seleccionará automáticamente. Los ángulos se pueden revisar antes de activar el ojo.");
        }
        for error in [&st.camera_error, &st.vision_error, &st.eye_error]
            .into_iter()
            .flatten()
        {
            ui.colored_label(AMBER, error);
        }
    }
}
impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        let ctx = ui.ctx().clone();
        ctx.request_repaint_after(Duration::from_millis(30));
        if let Some(f) = self.shared.camera.get() {
            if self.frame_at != Some(f.time) {
                let img = ColorImage::from_rgb([f.width as usize, f.height as usize], &f.rgb);
                match &mut self.texture {
                    Some(t) => t.set(img, TextureOptions::LINEAR),
                    None => {
                        self.texture = Some(ctx.load_texture("camera", img, TextureOptions::LINEAR))
                    }
                };
                self.frame_at = Some(f.time);
            }
        }
        egui::Panel::top("top").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.heading(egui::RichText::new("ZTrak Ojo").color(ACCENT));
                ui.label("Una mirada que te sigue");
                if cfg!(windows) {
                    ui.separator();
                    self.camera_picker(ui);
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if self.shared.driving() {
                        if ui
                            .add(
                                egui::Button::new("Parar y apagar")
                                    .fill(Color32::from_rgb(155, 45, 55)),
                            )
                            .clicked()
                        {
                            self.shared.stop();
                        }
                        if self.shared.tracking() && ui.button("Parpadear").clicked() {
                            *self.shared.gesture.lock() = Some("parpadear".into());
                        }
                    } else if ui
                        .add_enabled(
                            self.camera_problem().is_none(),
                            egui::Button::new("Activar seguimiento")
                                .fill(Color32::from_rgb(25, 104, 88)),
                        )
                        .clicked()
                    {
                        if self.edit != self.saved {
                            self.message =
                                "Aplica los ajustes antes de activar el seguimiento".into();
                        } else {
                            self.shared.set_tracking(true);
                        }
                    }
                });
            });
        });
        egui::Panel::right("settings")
            .default_size(375.)
            .min_size(340.)
            .max_size(480.)
            .resizable(true)
            .show(ui, |ui| self.settings(ui));
        egui::CentralPanel::default().show(ui, |ui| self.preview(ui));
    }
}
