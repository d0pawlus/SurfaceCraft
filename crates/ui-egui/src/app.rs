//! The application window: menus, toolbar, object list, command palette, log and viewport.
use crate::camera::Camera;
use crate::viewport::{self, DrawItem, Frame, ViewportCallback};
use serde_json::{Value, json};
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_geom::{Bvh, Point3};

const MESH_COLOR: [f32; 4] = [0.62, 0.66, 0.72, 1.0];
const BODY_COLOR: [f32; 4] = [0.35, 0.55, 0.85, 1.0];

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
    /// Whether a wgpu render state exists (false in headless tests).
    gpu: bool,
    fit_pending: bool,
    bvh_cache: HashMap<usize, Bvh>,
    highlights: HashMap<ObjectId, Arc<Vec<u32>>>,
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
        Self {
            session: Session::new(),
            camera: Camera::default(),
            selected_object: None,
            palette_command: None,
            palette_params: "{}".into(),
            palette_filter: String::new(),
            status: "Ready".into(),
            picks: Vec::new(),
            gpu: false,
            fit_pending: false,
            bvh_cache: HashMap::new(),
            highlights: HashMap::new(),
        }
    }

    /// Picks the nearest object under viewport pixel `(x, y)` of a `w × h` viewport.
    /// Returns whether something was hit.
    pub fn pick_at(&mut self, x: f64, y: f64, w: f64, h: f64, add: bool) -> bool {
        let (origin, dir) = self.camera.ray(x, y, w, h);
        let items = self.session.display_items();
        let live: Vec<usize> = items.iter().map(|d| d.key).collect();
        self.bvh_cache.retain(|k, _| live.contains(k));
        let mut best: Option<(f64, Pick)> = None;
        for d in &items {
            let bvh = self.bvh_cache.entry(d.key).or_insert_with(|| Bvh::build(&d.mesh));
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
            ui.strong(format!("Picks ({})", self.picks.len()));
            if ui.small_button("Clear").clicked() {
                self.clear_picks();
            }
        });
        ui.weak("Click the model to pick, Shift+click to add. Picks prefill `from`/`points`/`triangles`.");
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
        }
    }

    /// The app for an eframe window; installs the viewport renderer when wgpu is available.
    #[must_use]
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut app = Self::headless();
        if let Some(rs) = cc.wgpu_render_state.as_ref() {
            viewport::install(rs);
            app.gpu = true;
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
            self.status = "Choose a command first".into();
            return;
        };
        match serde_json::from_str::<Value>(&self.palette_params) {
            Ok(v) => {
                let _ = self.run_command(id, v);
            }
            Err(e) => self.status = format!("parameters are not JSON: {e}"),
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

    /// Draws the whole UI (the body of `eframe::App::ui`).
    pub fn draw(&mut self, ui: &mut egui::Ui) {
        self.shortcuts(ui);
        egui::Panel::top("menu").show(ui, |ui| self.menu_bar(ui));
        egui::Panel::bottom("status").show(ui, |ui| {
            ui.horizontal(|ui| {
                ui.label(&self.status);
            });
        });
        egui::Panel::bottom("log").resizable(true).default_size(140.0).show(ui, |ui| self.log_panel(ui));
        egui::Panel::left("objects").resizable(true).default_size(230.0).show(ui, |ui| self.objects_panel(ui));
        egui::Panel::right("palette").resizable(true).default_size(320.0).show(ui, |ui| self.palette_panel(ui));
        egui::CentralPanel::default().show(ui, |ui| self.viewport_panel(ui));
    }

    fn shortcuts(&mut self, ui: &mut egui::Ui) {
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

    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::MenuBar::new().ui(ui, |ui| {
            ui.menu_button("File", |ui| {
                if ui.button("New").clicked() {
                    let _ = self.run_command("doc.new", Value::Null);
                    ui.close();
                }
                #[cfg(not(target_arch = "wasm32"))]
                {
                    if ui.button("Open project…").clicked() {
                        if let Some(p) =
                            rfd::FileDialog::new().add_filter("SurfaceCraft project", &["scproj"]).pick_file()
                        {
                            let _ = self.run_command("doc.open", json!({"path": p.to_string_lossy()}));
                            self.fit_pending = true;
                        }
                        ui.close();
                    }
                    if ui.button("Save project…").clicked() {
                        if let Some(p) =
                            rfd::FileDialog::new().add_filter("SurfaceCraft project", &["scproj"]).save_file()
                        {
                            let _ = self.run_command("doc.save", json!({"path": p.to_string_lossy()}));
                        }
                        ui.close();
                    }
                    ui.separator();
                    if ui.button("Import mesh…").clicked() {
                        if let Some(p) = rfd::FileDialog::new().add_filter("Mesh", &["stl", "obj", "ply"]).pick_file() {
                            let _ = self.run_command("mesh.import", json!({"path": p.to_string_lossy()}));
                            self.fit_pending = true;
                        }
                        ui.close();
                    }
                    if ui.button("Export selected mesh…").clicked() {
                        if let Some(id) = self.selected_object
                            && let Some(p) =
                                rfd::FileDialog::new().add_filter("Mesh", &["stl", "obj", "ply"]).save_file()
                        {
                            let _ =
                                self.run_command("mesh.export", json!({"object": id.0, "path": p.to_string_lossy()}));
                        }
                        ui.close();
                    }
                }
            });
            ui.menu_button("Edit", |ui| {
                if ui.button("Undo").clicked() {
                    let _ = self.run_command("edit.undo", Value::Null);
                    ui.close();
                }
                if ui.button("Redo").clicked() {
                    let _ = self.run_command("edit.redo", Value::Null);
                    ui.close();
                }
            });
            ui.menu_button("View", |ui| {
                if ui.button("Fit all").clicked() {
                    self.fit_pending = true;
                    ui.close();
                }
            });
            // one menu per command group: every command is reachable from the menus too
            let mut groups: Vec<&'static str> = self.session.commands().iter().map(|c| c.group).collect();
            groups.sort_unstable();
            groups.dedup();
            for g in groups.into_iter().filter(|g| !matches!(*g, "File" | "Edit" | "View" | "App")) {
                ui.menu_button(g, |ui| {
                    for c in self.session.commands().iter().filter(|c| c.group == g) {
                        if ui.button(c.label).on_hover_text(c.summary).clicked() {
                            self.choose_command(c.id);
                            ui.close();
                        }
                    }
                });
            }
        });
    }

    fn objects_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Objects");
        let summaries = self.session.doc.summaries();
        if summaries.is_empty() {
            ui.weak("No objects. Import a mesh (File) or run mesh.synth.");
        }
        let mut action: Option<(&str, Value)> = None;
        egui::ScrollArea::vertical().id_salt("objects").max_height(320.0).show(ui, |ui| {
            for s in &summaries {
                ui.horizontal(|ui| {
                    let mut vis = s.visible;
                    if ui.checkbox(&mut vis, "").on_hover_text("Show / hide").changed() {
                        action = Some(("doc.show", json!({"object": s.id.0, "visible": vis})));
                    }
                    let kind = match s.kind {
                        ObjectKind::Mesh => "mesh",
                        ObjectKind::Body => "body",
                    };
                    let sel = self.selected_object == Some(s.id);
                    if ui
                        .selectable_label(sel, format!("{kind} {} ({})", s.name, s.id))
                        .on_hover_text(&s.detail)
                        .clicked()
                    {
                        self.selected_object = if sel { None } else { Some(s.id) };
                    }
                    if ui.small_button("Delete").on_hover_text("Delete").clicked() {
                        action = Some(("doc.delete", json!({"object": s.id.0})));
                    }
                });
            }
        });
        if let Some((id, p)) = action {
            let _ = self.run_command(id, p);
        }
        self.picks_panel(ui);
    }

    fn palette_panel(&mut self, ui: &mut egui::Ui) {
        ui.heading("Commands");
        ui.horizontal(|ui| {
            let l = ui.label("Filter");
            ui.add(egui::TextEdit::singleline(&mut self.palette_filter).hint_text("id or label")).labelled_by(l.id);
        });
        let filter = self.palette_filter.to_lowercase();
        egui::ScrollArea::vertical().id_salt("cmdlist").max_height(220.0).show(ui, |ui| {
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
            let l = ui.label("Parameters (JSON)");
            ui.add(egui::TextEdit::multiline(&mut self.palette_params).code_editor().desired_rows(6)).labelled_by(l.id);
            if ui.button("Run").clicked() {
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
            ui.weak("Choose a command to see its parameters.");
        }
    }

    fn log_panel(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.strong("Command log");
            if ui.small_button("Copy as script").clicked() {
                let text = serde_json::to_string_pretty(&self.log_as_script()).unwrap_or_default();
                ui.ctx().copy_text(text);
                self.status = "Script copied to the clipboard".into();
            }
        });
        egui::ScrollArea::vertical().stick_to_bottom(true).show(ui, |ui| {
            for e in self.session.log.iter().rev().take(200).rev() {
                let line = format!(
                    "{} {} {}",
                    if e.ok { "ok " } else { "ERR" },
                    e.command,
                    truncate(&e.params.to_string(), 300)
                );
                ui.monospace(line);
            }
        });
    }

    fn viewport_panel(&mut self, ui: &mut egui::Ui) {
        let rect = ui.available_rect_before_wrap();
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
            ui.painter().text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                format!("viewport (no GPU) — {} objects", self.session.doc.summaries().len()),
                egui::FontId::proportional(14.0),
                ui.visuals().weak_text_color(),
            );
            return;
        }
        let ppp = ui.ctx().pixels_per_point();
        let size = [(rect.width() * ppp).round().max(1.0) as u32, (rect.height() * ppp).round().max(1.0) as u32];
        let items: Vec<DrawItem> = self
            .session
            .display_items()
            .into_iter()
            .map(|d| DrawItem {
                key: d.key,
                color: if d.kind == ObjectKind::Body { BODY_COLOR } else { MESH_COLOR },
                mesh: d.mesh,
                colors: d.colors,
                highlight: self.highlights.get(&d.id).cloned(),
            })
            .collect();
        let bg = if ui.visuals().dark_mode { [0.08, 0.09, 0.11] } else { [0.85, 0.87, 0.9] };
        let frame = Frame { size, camera: self.camera.clone(), origin: self.camera.target, items, background: bg };
        ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, ViewportCallback(frame)));
    }
}

impl eframe::App for SurfaceCraftApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.draw(ui);
    }

    #[cfg(target_arch = "wasm32")]
    fn as_any_mut(&mut self) -> Option<&mut dyn std::any::Any> {
        Some(&mut *self)
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
                "object" | "mesh" => ctx.selected.map_or(json!(0), |id| json!(id.0)),
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
