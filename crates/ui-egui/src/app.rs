//! The application state and the panel bodies. The shell around them (menus, toolbar, docking,
//! settings, dialogs) is in [`crate::shell`]; every colour is a theme token and every text a
//! catalogue key.
use crate::camera::Camera;
use crate::i18n::{t, tf};
use crate::settings::UiSettings;
use crate::theme::{Token, Tokens};
use crate::viewport::{self, DrawItem, Frame, ViewportCallback};
use serde_json::{Value, json};
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_geom::{Bvh, Point3};

/// The SurfaceCraft UI.
pub struct SurfaceCraftApp {
    /// The engine session every action goes through.
    pub session: Session,
    /// Viewport camera.
    pub camera: Camera,
    /// Object selected in the list (used to prefill command parameters).
    pub selected_object: Option<ObjectId>,
    /// Command chosen in the palette.
    pub palette_command: Option<&'static str>,
    /// The palette's parameter editor (JSON).
    pub palette_params: String,
    /// Palette filter text.
    pub palette_filter: String,
    /// Last status line.
    pub status: String,
    /// Points picked in the viewport (click: replace, Shift+click: add).
    pub picks: Vec<Pick>,
    /// Theme, density, language and dock layout, saved between runs.
    pub settings: UiSettings,
    /// The dock layout in use (saved into `settings` on exit).
    pub workspace: crate::dock::Workspace,
    /// The theme's tokens resolved for the current preset.
    pub tokens: Tokens,
    /// Where the viewport was drawn last frame (screen points).
    pub viewport_rect: Option<egui::Rect>,
    /// Whether a wgpu render state exists (false in headless tests).
    gpu: bool,
    pub(crate) fit_pending: bool,
    bvh_cache: HashMap<usize, Bvh>,
    highlights: HashMap<ObjectId, Arc<Vec<u32>>>,
    /// The assistant's goal text.
    pub assistant_goal: String,
    /// Align (true) or workflow mode.
    pub assistant_align: bool,
    provider: Option<Box<dyn surfacecraft_ai::Provider>>,
    pub(crate) prepared: bool,
    pub(crate) ctx: Option<egui::Context>,
    pub(crate) applied: Option<(crate::theme::Theme, crate::i18n::Lang)>,
    #[cfg(not(target_arch = "wasm32"))]
    control: Option<std::sync::mpsc::Receiver<crate::control::Request>>,
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) dialogs: craft_dialogs::Dialogs<crate::shell::DialogThen>,
    #[cfg(target_arch = "wasm32")]
    pub(crate) inbox: crate::web::Inbox,
}

/// A point picked on an object in the viewport.
#[derive(Clone, Debug, PartialEq)]
pub struct Pick {
    /// Object hit.
    pub object: ObjectId,
    /// Hit point (model coordinates).
    pub point: Point3,
    /// Triangle hit (of the object's mesh or display mesh).
    pub triangle: u32,
}

impl Default for SurfaceCraftApp {
    fn default() -> Self {
        Self::headless()
    }
}

impl SurfaceCraftApp {
    /// An app without a GPU (tests, or a renderer that failed to start).
    #[must_use]
    pub fn headless() -> Self {
        Self::with_settings(UiSettings::default())
    }

    /// An app without a GPU that starts with `settings`.
    #[must_use]
    pub fn with_settings(settings: UiSettings) -> Self {
        let side = settings.theme.density.metrics().side_dock_width;
        let default = crate::dock::default_workspace(side);
        let mut workspace = settings.workspace.clone().unwrap_or_else(|| default.clone());
        crate::dock::reconcile(&mut workspace, &default);
        Self {
            session: Session::new(),
            camera: Camera::default(),
            selected_object: None,
            palette_command: None,
            palette_params: "{}".into(),
            palette_filter: String::new(),
            status: t("status.ready").to_string(),
            picks: Vec::new(),
            tokens: Tokens::resolve(settings.theme.preset),
            workspace,
            settings,
            viewport_rect: None,
            gpu: false,
            fit_pending: false,
            bvh_cache: HashMap::new(),
            highlights: HashMap::new(),
            assistant_goal: String::new(),
            assistant_align: true,
            provider: default_provider(),
            prepared: false,
            ctx: None,
            applied: None,
            #[cfg(not(target_arch = "wasm32"))]
            control: None,
            #[cfg(not(target_arch = "wasm32"))]
            dialogs: craft_dialogs::Dialogs::new(std::sync::Arc::new(|| {})),
            #[cfg(target_arch = "wasm32")]
            inbox: Default::default(),
        }
    }

    /// Picks the nearest object under viewport pixel `(x, y)` of a `w × h` viewport.
    /// Returns whether something was hit.
    pub fn pick_at(&mut self, x: f64, y: f64, w: f64, h: f64, add: bool) -> bool {
        let (origin, dir) = self.camera.ray(x, y, w, h);
        let items = self.session.display_items();
        let live: Vec<usize> = items.iter().map(|d| d.pick_key).collect();
        self.bvh_cache.retain(|k, _| live.contains(k));
        let mut best: Option<(f64, Pick)> = None;
        for d in &items {
            // pick on the full-resolution mesh: the triangle ids stay the original ones
            let bvh = self.bvh_cache.entry(d.pick_key).or_insert_with(|| Bvh::build(&d.pick_mesh));
            if let Some((tri, t)) = bvh.ray_cast(&origin, &dir)
                && best.as_ref().is_none_or(|(bt, _)| t < *bt)
            {
                best = Some((t, Pick { object: d.id, point: origin + dir * t, triangle: tri as u32 }));
            }
        }
        if !add {
            self.picks.clear();
        }
        let hit = match best {
            Some((_, p)) => {
                self.status = format!("picked {} at ({:.3}, {:.3}, {:.3})", p.object, p.point.x, p.point.y, p.point.z);
                self.selected_object = Some(p.object);
                self.picks.push(p);
                true
            }
            None => false,
        };
        self.refresh_highlights();
        hit
    }

    /// Clears the picks.
    pub fn clear_picks(&mut self) {
        self.picks.clear();
        self.refresh_highlights();
    }

    fn refresh_highlights(&mut self) {
        let mut by: HashMap<ObjectId, Vec<u32>> = HashMap::new();
        for p in &self.picks {
            by.entry(p.object).or_default().push(p.triangle);
        }
        self.highlights = by.into_iter().map(|(k, v)| (k, Arc::new(v))).collect();
    }

    fn picks_panel(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        ui.horizontal(|ui| {
            ui.strong(tf("picks.title", &[("n", &self.picks.len().to_string())]));
            if ui.small_button(t("picks.clear")).clicked() {
                self.clear_picks();
            }
        });
        ui.weak(t("picks.hint"));
        for p in self.picks.iter().rev().take(12) {
            ui.monospace(format!(
                "{} t{} ({:.3}, {:.3}, {:.3})",
                p.object, p.triangle, p.point.x, p.point.y, p.point.z
            ));
        }
    }

    fn pick_context(&self) -> PickContext {
        PickContext {
            selected: self.selected_object,
            points: self.picks.iter().map(|p| [p.point.x, p.point.y, p.point.z]).collect(),
            triangles: self.picks.iter().map(|p| p.triangle).collect(),
            objects: self.session.doc.summaries().iter().map(|o| (o.id, o.kind)).collect(),
        }
    }

    /// The app for an eframe window: restores the saved settings and installs the viewport
    /// renderer when wgpu is available.
    #[must_use]
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let saved = cc
            .storage
            .and_then(|s| s.get_string(crate::settings::STORAGE_KEY))
            .map(|json| UiSettings::from_json(&json));
        let settings =
            saved.unwrap_or_else(|| UiSettings { lang: crate::i18n::Lang::from_system(), ..UiSettings::default() });
        let mut app = Self::with_settings(settings);
        if let Some(rs) = cc.wgpu_render_state.as_ref() {
            viewport::install(rs);
            app.gpu = true;
        }
        #[cfg(not(target_arch = "wasm32"))]
        {
            let ctx = cc.egui_ctx.clone();
            app.control = crate::control::start({
                let ctx = ctx.clone();
                move || ctx.request_repaint()
            })
            .map(|(rx, _)| rx);
            app.dialogs = craft_dialogs::Dialogs::new(std::sync::Arc::new(move || ctx.request_repaint()));
        }
        app
    }

    /// Executes a command the way every UI control does, updating the status line.
    pub fn run_command(&mut self, id: &str, params: Value) -> Result<Value, EngineError> {
        let had = self.session.doc.summaries().len();
        let r = self.session.execute(id, params);
        match &r {
            Ok(_) => {
                self.status = format!("{id}: done");
                if had == 0 && !self.session.doc.summaries().is_empty() {
                    self.fit_pending = true;
                }
            }
            Err(e) => self.status = format!("{id}: {e}"),
        }
        r
    }

    /// Chooses a command in the palette and prefills its parameters.
    pub fn choose_command(&mut self, id: &'static str) {
        self.palette_command = Some(id);
        let schema = surfacecraft_engine::cmd::find(id).map(|c| (c.input)()).unwrap_or(Value::Null);
        self.palette_params =
            serde_json::to_string_pretty(&template(&schema, &self.pick_context())).unwrap_or_else(|_| "{}".into());
    }

    /// Runs the palette's command with its edited parameters.
    pub fn run_palette(&mut self) {
        let Some(id) = self.palette_command else {
            self.status = t("status.choose_command").to_string();
            return;
        };
        match serde_json::from_str::<Value>(&self.palette_params) {
            Ok(v) => {
                let _ = self.run_command(id, v);
            }
            Err(e) => self.status = tf("status.params_not_json", &[("error", &e.to_string())]),
        }
    }

    /// The commands of the log as a runnable script.
    #[must_use]
    pub fn log_as_script(&self) -> Value {
        let cmds: Vec<Value> = self
            .session
            .log
            .iter()
            .filter(|e| e.ok && !e.command.starts_with("app."))
            .map(|e| json!({"command": e.command, "params": e.params}))
            .collect();
        json!({ "commands": cmds })
    }

    /// Answers control-channel requests (bridged MCP) between frames.
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) fn poll_control(&mut self) {
        let Some(rx) = self.control.as_ref() else { return };
        let reqs: Vec<crate::control::Request> = rx.try_iter().collect();
        for r in reqs {
            let reply = match r.method.as_str() {
                "engine.execute" => {
                    let cmd = r.params.get("command").and_then(Value::as_str).unwrap_or("").to_string();
                    let params = r.params.get("params").cloned().unwrap_or(Value::Null);
                    match self.run_command(&cmd, params) {
                        Ok(v) => json!({"ok": v}),
                        Err(e) => json!({"err": e}),
                    }
                }
                "ui.screenshot" => {
                    let dim = |k: &str, d: u64| r.params.get(k).and_then(Value::as_u64).unwrap_or(d) as u32;
                    let cam = self.camera.clone();
                    match crate::shot::screenshot_png(
                        &mut self.session,
                        Some(&cam),
                        dim("width", 800),
                        dim("height", 600),
                    ) {
                        Some(png) => json!({"png": surfacecraft_doc::base64::encode(&png)}),
                        None => json!({"err": "no renderer"}),
                    }
                }
                other => json!({"err": format!("unknown method {other}")}),
            };
            let _ = r.reply.send(reply);
        }
    }

    /// Imports files the browser picked (web build).
    #[cfg(target_arch = "wasm32")]
    pub(crate) fn poll_inbox(&mut self) {
        let files: Vec<(String, Vec<u8>)> = self.inbox.borrow_mut().drain(..).collect();
        for (name, bytes) in files {
            let lower = name.to_lowercase();
            let ext = lower.rsplit('.').next().unwrap_or("").to_string();
            let stem = name.rsplit_once('.').map_or(name.as_str(), |(s, _)| s).to_string();
            let data = surfacecraft_doc::base64::encode(&bytes);
            let r = if ext == "step" || ext == "stp" {
                self.run_command("import.step", json!({"data": data, "name": stem}))
            } else {
                self.run_command("mesh.import", json!({"data": data, "format": ext, "name": stem}))
            };
            if r.is_ok() {
                self.fit_pending = true;
            }
        }
    }

    /// Downloads the selected object: a mesh as STL, a body or surface as STEP (web build).
    #[cfg(target_arch = "wasm32")]
    pub(crate) fn download_selected(&mut self) {
        let Some(id) = self.selected_object else {
            self.status = t("status.select_first").to_string();
            return;
        };
        let (cmd, params, file) = match self.session.doc.kind_of(id) {
            Some(ObjectKind::Mesh) => ("mesh.export", json!({"object": id.0, "format": "stl"}), "mesh.stl"),
            Some(ObjectKind::Body | ObjectKind::Surface) => ("export.step", json!({"objects": [id.0]}), "model.step"),
            _ => {
                self.status = t("status.download_kinds").to_string();
                return;
            }
        };
        if let Ok(v) = self.run_command(cmd, params)
            && let Some(Ok(bytes)) = v.get("data").and_then(Value::as_str).map(surfacecraft_doc::base64::decode)
        {
            crate::web::download(file, &bytes);
        }
    }

    pub(crate) fn shortcuts(&mut self, ui: &mut egui::Ui) {
        let (undo, redo) = ui.input(|i| {
            let z = i.modifiers.command && i.key_pressed(egui::Key::Z);
            (z && !i.modifiers.shift, (z && i.modifiers.shift) || (i.modifiers.command && i.key_pressed(egui::Key::Y)))
        });
        if undo {
            let _ = self.run_command("edit.undo", Value::Null);
        }
        if redo {
            let _ = self.run_command("edit.redo", Value::Null);
        }
    }

    /// A small icon toggle for lists: returns the click.
    fn icon_toggle(&self, ui: &mut egui::Ui, label_key: &str, icon: &str, selected: bool) -> egui::Response {
        let m = self.settings.theme.density.metrics();
        let style = crate::theme::icon_button_style(&self.tokens);
        let size = egui::Vec2::splat(m.icon_button - 6.0);
        let inset = ((size.x - m.icon) / 2.0).max(0.0);
        craft_ui::buttons::IconButton::new(t(label_key), size, &style)
            .selected(selected)
            .show(ui, |ui, rect, tint| crate::icons::paint(ui, rect.shrink(inset), icon, tint))
            .on_hover_text(t(label_key))
    }

    /// The object tree: objects grouped by kind, with visibility and delete controls.
    pub(crate) fn objects_panel(&mut self, ui: &mut egui::Ui) {
        let summaries = self.session.doc.summaries();
        if summaries.is_empty() {
            ui.weak(t("objects.empty"));
        }
        let mut action: Option<(&str, Value)> = None;
        for kind in KIND_ORDER {
            let of_kind: Vec<_> = summaries.iter().filter(|s| s.kind == kind).collect();
            if of_kind.is_empty() {
                continue;
            }
            let title = tf("objects.group", &[("kind", t(kind_plural_key(kind))), ("n", &of_kind.len().to_string())]);
            egui::CollapsingHeader::new(title).id_salt(("objects", kind_key(kind))).default_open(true).show(ui, |ui| {
                for s in of_kind {
                    ui.horizontal(|ui| {
                        if self
                            .icon_toggle(ui, "objects.show_hide", if s.visible { "eye" } else { "eye-off" }, !s.visible)
                            .clicked()
                        {
                            action = Some(("doc.show", json!({"object": s.id.0, "visible": !s.visible})));
                        }
                        let sel = self.selected_object == Some(s.id);
                        if ui
                            .selectable_label(sel, format!("{} {} ({})", t(kind_key(kind)), s.name, s.id))
                            .on_hover_text(&s.detail)
                            .clicked()
                        {
                            self.selected_object = if sel { None } else { Some(s.id) };
                        }
                        if self.icon_toggle(ui, "objects.delete", "trash", false).clicked() {
                            action = Some(("doc.delete", json!({"object": s.id.0})));
                        }
                    });
                }
            });
        }
        if let Some((id, p)) = action {
            let _ = self.run_command(id, p);
        }
        self.picks_panel(ui);
    }

    /// The operation history: what undo would step back through, newest first.
    pub(crate) fn history_panel(&mut self, ui: &mut egui::Ui) {
        let labels = self.session.history.undo_labels();
        if labels.is_empty() {
            ui.weak(t("history.empty"));
        }
        for (i, label) in labels.iter().rev().enumerate() {
            let text = if i == 0 { egui::RichText::new(*label).strong() } else { egui::RichText::new(*label) };
            ui.label(text);
        }
        let redo = self.session.history.redo_len();
        if redo > 0 {
            ui.separator();
            ui.weak(tf("history.redo", &[("n", &redo.to_string())]));
        }
    }

    /// Regions of the document: persistent triangle sets of scans.
    pub(crate) fn regions_panel(&mut self, ui: &mut egui::Ui) {
        let regions: Vec<_> =
            self.session.doc.summaries().into_iter().filter(|s| s.kind == ObjectKind::Region).collect();
        if regions.is_empty() {
            ui.weak(t("regions.empty"));
        }
        for s in &regions {
            let sel = self.selected_object == Some(s.id);
            if ui.selectable_label(sel, format!("{} ({})", s.name, s.id)).on_hover_text(&s.detail).clicked() {
                self.selected_object = if sel { None } else { Some(s.id) };
            }
        }
        ui.separator();
        if ui.button(t("regions.auto_segment")).clicked() {
            self.choose_command("region.auto_segment");
        }
        if ui.button(t("regions.create")).clicked() {
            self.choose_command("region.create");
        }
    }

    /// Deviation maps: the legend (tokens), the scans that carry a map and the commands.
    pub(crate) fn deviation_panel(&mut self, ui: &mut egui::Ui) {
        ui.strong(t("deviation.legend"));
        for (token, key) in [
            (Token::DeviationBelow, "deviation.below"),
            (Token::DeviationInTolerance, "deviation.in_tolerance"),
            (Token::DeviationAbove, "deviation.above"),
            (Token::DeviationOutside, "deviation.outside"),
        ] {
            ui.horizontal(|ui| {
                let (rect, _) = ui.allocate_exact_size(
                    egui::Vec2::splat(self.settings.theme.density.metrics().icon),
                    egui::Sense::hover(),
                );
                ui.painter().rect_filled(rect, 3.0, self.tokens.color(token));
                ui.label(t(key));
            });
        }
        ui.separator();
        let mut mapped: Vec<ObjectId> = self.session.overlays.keys().copied().collect();
        mapped.sort();
        if mapped.is_empty() {
            ui.weak(t("deviation.none"));
        }
        let mut clear = None;
        for id in mapped {
            ui.horizontal(|ui| {
                ui.label(format!("{id}"));
                if ui.small_button(t("deviation.clear")).clicked() {
                    clear = Some(id);
                }
            });
        }
        if let Some(id) = clear {
            let _ = self.run_command("deviation.clear", json!({"object": id.0}));
        }
        ui.separator();
        if ui.button(t("deviation.compute")).clicked() {
            self.choose_command("deviation.compute");
        }
        if ui.button(t("deviation.noise")).clicked() {
            self.choose_command("deviation.noise");
        }
    }

    /// The command palette with its JSON parameter editor and the assistant.
    pub(crate) fn palette_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            let l = ui.label(t("palette.filter"));
            ui.add(egui::TextEdit::singleline(&mut self.palette_filter).hint_text(t("palette.filter_hint")))
                .labelled_by(l.id);
        });
        let filter = self.palette_filter.to_lowercase();
        egui::ScrollArea::vertical().id_salt("cmdlist").max_height(160.0).show(ui, |ui| {
            for c in self.session.commands() {
                if !filter.is_empty() && !c.id.contains(&filter) && !c.label.to_lowercase().contains(&filter) {
                    continue;
                }
                let sel = self.palette_command == Some(c.id);
                if ui.selectable_label(sel, format!("{}  —  {}", c.id, c.label)).on_hover_text(c.summary).clicked() {
                    self.choose_command(c.id);
                }
            }
        });
        ui.separator();
        if let Some(id) = self.palette_command {
            ui.strong(id);
            if let Some(c) = surfacecraft_engine::cmd::find(id) {
                ui.label(c.summary);
            }
            let l = ui.label(t("palette.parameters"));
            ui.add(egui::TextEdit::multiline(&mut self.palette_params).code_editor().desired_rows(6)).labelled_by(l.id);
            if ui.button(t("palette.run")).clicked() {
                self.run_palette();
            }
            if let Some(last) = self.session.log.last()
                && last.command == id
            {
                ui.separator();
                egui::ScrollArea::vertical().id_salt("result").show(ui, |ui| {
                    let text = serde_json::to_string_pretty(&last.outcome).unwrap_or_default();
                    ui.monospace(truncate(&text, 4000));
                });
            }
        } else {
            ui.weak(t("palette.choose"));
        }
        self.assistant_panel(ui);
    }

    /// Uses `p` as the assistant's language model.
    pub fn set_provider(&mut self, p: Box<dyn surfacecraft_ai::Provider>) {
        self.provider = Some(p);
    }

    /// Runs the assistant on the current goal with a picture of the viewport.
    pub fn ask_assistant(&mut self) {
        let Some(mut p) = self.provider.take() else {
            self.status = t("assistant.no_provider_status").to_string();
            return;
        };
        let mode = if self.assistant_align { surfacecraft_ai::Mode::Align } else { surfacecraft_ai::Mode::Workflow };
        let cam = self.camera.clone();
        let png = crate::shot::screenshot_png(&mut self.session, Some(&cam), 512, 384);
        let goal = self.assistant_goal.clone();
        self.status = match surfacecraft_ai::run(&mut self.session, p.as_mut(), mode, &goal, png.as_deref(), 20) {
            Ok(o) => format!("assistant: {} ({} commands)", o.summary, o.steps.len()),
            Err(e) => format!("assistant failed: {e}"),
        };
        self.provider = Some(p);
        self.fit_pending = true;
    }

    fn assistant_panel(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        ui.strong(t("assistant.title"));
        ui.horizontal(|ui| {
            ui.radio_value(&mut self.assistant_align, true, t("assistant.align"));
            ui.radio_value(&mut self.assistant_align, false, t("assistant.workflow"));
        });
        let l = ui.label(t("assistant.goal"));
        ui.add(egui::TextEdit::singleline(&mut self.assistant_goal).hint_text(t("assistant.goal_hint")))
            .labelled_by(l.id);
        if ui.button(t("assistant.ask")).clicked() {
            self.ask_assistant();
        }
        if self.provider.is_none() {
            ui.weak(t("assistant.no_provider"));
        }
    }

    /// The command log: every call as the JSON a script or MCP client would send.
    pub(crate) fn log_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            if ui.small_button(t("log.copy_script")).clicked() {
                let text = serde_json::to_string_pretty(&self.log_as_script()).unwrap_or_default();
                ui.ctx().copy_text(text);
                self.status = t("status.script_copied").to_string();
            }
        });
        egui::ScrollArea::vertical().id_salt("log").auto_shrink([false, false]).stick_to_bottom(true).show(ui, |ui| {
            for e in self.session.log.iter().rev().take(200).rev() {
                let line = format!(
                    "{} {} {}",
                    if e.ok { "ok " } else { "ERR" },
                    e.command,
                    truncate(&e.params.to_string(), 300)
                );
                let color = self.tokens.color(if e.ok { Token::TextMuted } else { Token::StatusError });
                ui.label(egui::RichText::new(line).monospace().color(color));
            }
        });
    }

    /// The 3D viewport: camera gestures, picking and the wgpu draw callback.
    pub(crate) fn viewport_panel(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap();
        self.viewport_rect = Some(rect);
        let resp = ui.allocate_rect(rect, egui::Sense::click_and_drag());
        if resp.dragged_by(egui::PointerButton::Primary) {
            let d = resp.drag_delta();
            self.camera.orbit(f64::from(d.x), f64::from(d.y));
        }
        if resp.dragged_by(egui::PointerButton::Secondary) || resp.dragged_by(egui::PointerButton::Middle) {
            let d = resp.drag_delta();
            self.camera.pan(f64::from(d.x), f64::from(d.y), f64::from(rect.height()));
        }
        if resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom(f64::from(scroll));
            }
        }
        if resp.double_clicked() {
            self.fit_pending = true;
        } else if resp.clicked()
            && let Some(pos) = resp.interact_pointer_pos()
        {
            let local = pos - rect.min;
            let add = ui.input(|i| i.modifiers.shift);
            self.pick_at(
                f64::from(local.x),
                f64::from(local.y),
                f64::from(rect.width()),
                f64::from(rect.height()),
                add,
            );
        }
        if self.fit_pending {
            self.camera.fit(&self.session.doc.bbox());
            self.fit_pending = false;
        }
        if !self.gpu {
            ui.painter().rect_filled(rect, 0.0, self.tokens.color(Token::ViewportBackground));
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                tf("viewport.no_gpu", &[("n", &self.session.doc.summaries().len().to_string())]),
                egui::FontId::proportional(14.0),
                self.tokens.color(Token::TextMuted),
            );
            return;
        }
        let ppp = ui.ctx().pixels_per_point();
        let size = [(rect.width() * ppp).round().max(1.0) as u32, (rect.height() * ppp).round().max(1.0) as u32];
        let tokens = self.tokens.clone();
        let items: Vec<DrawItem> = self
            .session
            .display_items()
            .into_iter()
            .map(|d| DrawItem {
                key: d.key,
                color: tokens.object(d.kind),
                colors: d.colors.clone(),
                colors_key: d.colors_key,
                highlight: self.highlights.get(&d.id).map(|h| Arc::new(d.drawn_triangles(h))),
                mesh: d.mesh,
            })
            .collect();
        let bg = self.tokens.viewport_background().map(f64::from);
        let frame = Frame { size, camera: self.camera.clone(), origin: self.camera.target, items, background: bg };
        ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, ViewportCallback(frame)));
    }
}

/// The order of the kinds in the object tree.
const KIND_ORDER: [ObjectKind; 8] = [
    ObjectKind::Mesh,
    ObjectKind::Region,
    ObjectKind::Primitive,
    ObjectKind::Surface,
    ObjectKind::Quad,
    ObjectKind::Sketch,
    ObjectKind::Body,
    ObjectKind::Feature,
];

/// The catalogue key of a kind's singular name.
fn kind_key(kind: ObjectKind) -> &'static str {
    match kind {
        ObjectKind::Mesh => "kind.mesh",
        ObjectKind::Body => "kind.body",
        ObjectKind::Region => "kind.region",
        ObjectKind::Primitive => "kind.primitive",
        ObjectKind::Surface => "kind.surface",
        ObjectKind::Quad => "kind.quad",
        ObjectKind::Sketch => "kind.sketch",
        ObjectKind::Feature => "kind.feature",
    }
}

/// The catalogue key of a kind's plural (the group header of the object tree).
fn kind_plural_key(kind: ObjectKind) -> &'static str {
    match kind {
        ObjectKind::Mesh => "kind.meshes",
        ObjectKind::Body => "kind.bodies",
        ObjectKind::Region => "kind.regions",
        ObjectKind::Primitive => "kind.primitives",
        ObjectKind::Surface => "kind.surfaces",
        ObjectKind::Quad => "kind.quads",
        ObjectKind::Sketch => "kind.sketches",
        ObjectKind::Feature => "kind.features",
    }
}

fn truncate(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_string() } else { s.chars().take(n).collect::<String>() + "…" }
}

/// What the UI knows when it prefills parameters.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PickContext {
    /// Selected object.
    pub selected: Option<ObjectId>,
    /// Picked points.
    pub points: Vec<[f64; 3]>,
    /// Picked triangles.
    pub triangles: Vec<u32>,
    /// Every object of the document (id, kind), so object references can be prefilled with a
    /// real object instead of a made-up id.
    pub objects: Vec<(ObjectId, ObjectKind)>,
}

impl PickContext {
    /// The object a reference parameter called `name` should point at: the selection when it
    /// fits, else the first object of the needed kind (any object when the kind is open).
    #[must_use]
    pub fn reference_for(&self, name: &str) -> Option<ObjectId> {
        let want = ref_kind(name);
        let fits = |k: ObjectKind| want.is_none_or(|w| w == k);
        if let Some(sel) = self.selected
            && self.objects.iter().any(|(id, k)| *id == sel && fits(*k))
        {
            return Some(sel);
        }
        self.objects.iter().find(|(_, k)| fits(*k)).map(|(id, _)| *id)
    }
}

/// The kind of object a reference parameter names (`None` when any kind goes).
fn ref_kind(name: &str) -> Option<ObjectKind> {
    match name {
        "mesh" | "against" => Some(ObjectKind::Mesh),
        "region" | "regions" => Some(ObjectKind::Region),
        "sketch" | "sketches" | "profile" | "path" => Some(ObjectKind::Sketch),
        "body" | "tool" => Some(ObjectKind::Body),
        "feature" => Some(ObjectKind::Feature),
        "surface" | "surfaces" => Some(ObjectKind::Surface),
        "primitive" | "primitives" => Some(ObjectKind::Primitive),
        _ => None,
    }
}

/// Whether a property schema is an object reference (`ObjRef`: an id or a name).
fn is_obj_ref(p: &Value) -> bool {
    p.get("$ref").and_then(Value::as_str).is_some_and(|r| r.ends_with("/ObjRef"))
}

/// Example parameters from an input schema: required properties with neutral values; `object`
/// gets the selected object, `from`/`points` the picked points, `triangles` the picked triangles.
#[must_use]
pub fn template(schema: &Value, ctx: &PickContext) -> Value {
    let mut out = serde_json::Map::new();
    let props = schema.get("properties").and_then(Value::as_object);
    let required: Vec<&str> = schema
        .get("required")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    if let Some(props) = props {
        for name in required {
            let p = props.get(name).cloned().unwrap_or(Value::Null);
            let v = match name {
                _ if is_obj_ref(&p) || name == "object" || name == "mesh" => {
                    ctx.reference_for(name).map_or(json!(""), |id| json!(id.0))
                }
                _ if p.get("items").is_some_and(is_obj_ref) => {
                    let n = p.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
                    let id = ctx.reference_for(name).map_or(json!(""), |id| json!(id.0));
                    Value::Array(vec![id; n])
                }
                "from" | "points" if !ctx.points.is_empty() => json!(ctx.points),
                "triangles" if !ctx.triangles.is_empty() => json!(ctx.triangles),
                _ => example(&p, schema),
            };
            out.insert(name.to_string(), v);
        }
    }
    Value::Object(out)
}

fn example(p: &Value, root: &Value) -> Value {
    if let Some(r) = p.get("$ref").and_then(Value::as_str) {
        let name = r.rsplit('/').next().unwrap_or("");
        if let Some(def) = root.get("$defs").and_then(|d| d.get(name)) {
            if let Some(e) = def.get("enum").and_then(Value::as_array).and_then(|a| a.first()) {
                return e.clone();
            }
            if let Some(one) = def.get("oneOf").and_then(Value::as_array).and_then(|a| a.first()) {
                if let Some(c) = one.get("const") {
                    return c.clone();
                }
                if let Some(e) = one.get("enum").and_then(Value::as_array).and_then(|a| a.first()) {
                    return e.clone();
                }
            }
            return example(def, root);
        }
    }
    match p.get("type").and_then(Value::as_str) {
        Some("string") => json!(""),
        Some("number") | Some("integer") => json!(0),
        Some("boolean") => json!(false),
        Some("array") => {
            let n = p.get("minItems").and_then(Value::as_u64).unwrap_or(0);
            let item = p.get("items").map_or(json!(0), |i| example(i, root));
            Value::Array((0..n).map(|_| item.clone()).collect())
        }
        Some("object") => template(p, &PickContext::default()),
        _ => json!(null),
    }
}

/// Viewport colour of an object kind (the Light preset's, used for screenshots).
#[must_use]
pub fn kind_color(kind: ObjectKind) -> [f32; 4] {
    Tokens::resolve(crate::theme::Preset::Light).object(kind)
}

fn default_provider() -> Option<Box<dyn surfacecraft_ai::Provider>> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        surfacecraft_ai::CommandProvider::from_env().map(|p| Box::new(p) as Box<dyn surfacecraft_ai::Provider>)
    }
    #[cfg(target_arch = "wasm32")]
    {
        None
    }
}
