//! The application state and the panel bodies. The shell around them (menus, ribbon, docking,
//! settings, dialogs) is in [`crate::shell`] and [`crate::ribbon`]; every colour is a theme token and every text a
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
use surfacecraft_engine::Session;
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
    /// The stage whose tools the ribbon shows.
    pub stage: crate::workflow::Stage,
    /// The text of the ribbon's tool search.
    pub tool_search: String,
    /// Put the keyboard in the search field on the next frame.
    pub(crate) focus_search: bool,
    /// Enter and Esc belong to the open tool this frame: when the frame began, no field had the
    /// keyboard, no menu was open and nothing in the viewport was using the keys (a pick, a
    /// sketch, a drag).
    pub(crate) keys_free: bool,
    /// The sketch editing mode (G5).
    pub sketch_editor: crate::sketch_editor::SketchEditor,
    /// The Properties panel's form and live preview.
    pub cmd_panel: crate::command_panel::CommandPanel,
    /// The feature history's editor.
    pub hist_panel: crate::history_panel::HistoryPanel,
    /// The deviation readings in the viewport: pinned ones and the one under the pointer.
    pub deviation: crate::deviation_panel::readout::State,
    /// Whether heavy commands launched from the UI run in the background (see [`crate::jobs`]).
    /// On in a window; off in a headless app, whose callers expect every command to have run
    /// when the call returns.
    pub background_jobs: bool,
    /// The command running now, if one is.
    pub(crate) busy: Option<crate::jobs::Busy>,
    /// A cancelled command whose thread has not finished yet.
    pub(crate) finishing: Option<surfacecraft_engine::Job>,
    /// Last status line.
    pub status: String,
    /// Points picked in the viewport (click: replace, Shift+click: add).
    pub picks: Vec<Pick>,
    /// The form field the viewport is filling by clicks (3-2-1 datums, point lists, body edges).
    pub field_pick: Option<crate::fieldpick::FieldPick>,
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
    pub(crate) bvh_cache: HashMap<usize, Bvh>,
    pub(crate) highlights: HashMap<ObjectId, Arc<Vec<u32>>>,
    /// The viewport tools: selection, face/edge picking, gizmo, section plane, view animation.
    pub tools: crate::viewtools::ViewTools,
    /// The assistant's goal text.
    pub assistant_goal: String,
    /// Align (true) or workflow mode.
    pub assistant_align: bool,
    pub(crate) provider: Option<Box<dyn surfacecraft_ai::Provider>>,
    pub(crate) prepared: bool,
    pub(crate) ctx: Option<egui::Context>,
    pub(crate) applied: Option<(crate::theme::Theme, crate::i18n::Lang)>,
    #[cfg(not(target_arch = "wasm32"))]
    pub(crate) control: Option<std::sync::mpsc::Receiver<crate::control::Request>>,
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
            stage: crate::workflow::Stage::default(),
            tool_search: String::new(),
            focus_search: false,
            keys_free: false,
            sketch_editor: crate::sketch_editor::SketchEditor::default(),
            cmd_panel: Default::default(),
            hist_panel: Default::default(),
            deviation: Default::default(),
            background_jobs: false,
            busy: None,
            finishing: None,
            status: t("status.ready").to_string(),
            picks: Vec::new(),
            field_pick: None,
            tokens: Tokens::resolve(settings.theme.preset),
            workspace,
            settings,
            viewport_rect: None,
            gpu: false,
            fit_pending: false,
            bvh_cache: HashMap::new(),
            highlights: HashMap::new(),
            tools: crate::viewtools::ViewTools::default(),
            assistant_goal: String::new(),
            assistant_align: true,
            provider: crate::assistant::default_provider(),
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

    /// The nearest object hit by the ray through viewport pixel `(x, y)` of a `w × h` viewport.
    pub(crate) fn ray_hit(&mut self, x: f64, y: f64, w: f64, h: f64) -> Option<Pick> {
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
        best.map(|(_, p)| p)
    }

    /// Picks the nearest object under viewport pixel `(x, y)` of a `w × h` viewport.
    /// Returns whether something was hit.
    pub fn pick_at(&mut self, x: f64, y: f64, w: f64, h: f64, add: bool) -> bool {
        let best = self.ray_hit(x, y, w, h);
        if !add {
            self.picks.clear();
        }
        let hit = match best {
            Some(p) => {
                let at = format!("{:.3}, {:.3}, {:.3}", p.point.x, p.point.y, p.point.z);
                self.status = tf("status.picked", &[("object", &p.object.to_string()), ("at", &at)]);
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

    pub(crate) fn refresh_highlights(&mut self) {
        self.tools.touch();
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
            objects: self.session.doc.summaries().iter().map(|o| (o.id, o.kind)).collect(),
            // a brush/lasso/smart selection wins over single picked triangles
            triangles: if self.tools.selection.is_empty() {
                self.picks.iter().map(|p| p.triangle).collect()
            } else {
                self.tools.selection.to_vec()
            },
            faces: self.tools.brep.face_list(),
            edges: self.tools.brep.edge_list(),
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
        app.background_jobs = true;
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

    /// Opens a command in the tool panel and prefills its parameters; the ribbon follows to the
    /// command's stage.
    pub fn choose_command(&mut self, id: &'static str) {
        self.palette_command = Some(id);
        self.field_pick = None;
        if let Some(stage) = crate::workflow::Stage::of(id) {
            self.stage = stage;
        }
        // the tool panel is where the form is: show it (again, if it was closed) with its tab
        // in front; a layout that refuses keeps the panel where it is
        let _ = crate::dock::set_visible(&mut self.workspace, crate::dock::Panel::Properties, true);
        let schema = surfacecraft_engine::cmd::find(id).map(|c| (c.input)()).unwrap_or(Value::Null);
        self.palette_params =
            serde_json::to_string_pretty(&template(&schema, &self.pick_context())).unwrap_or_else(|_| "{}".into());
    }

    /// Closes the open tool without running it: the tool panel shows the guide again.
    pub fn close_tool(&mut self) {
        self.palette_command = None;
        self.field_pick = None;
        self.cmd_panel.cancel_preview();
    }

    /// What the guide reads from the session: the kinds in the document and what ran.
    #[must_use]
    pub fn progress(&self) -> crate::guide::Progress {
        crate::guide::Progress {
            kinds: self.session.doc.summaries().iter().map(|s| s.kind).collect(),
            ran: self.session.log.iter().filter(|e| e.ok).map(|e| e.command.clone()).collect(),
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

    pub(crate) fn shortcuts(&mut self, ui: &mut egui::Ui) {
        if self.is_busy() {
            return;
        }
        let (undo, redo, search) = ui.input(|i| {
            let z = i.modifiers.command && i.key_pressed(egui::Key::Z);
            (
                z && !i.modifiers.shift,
                (z && i.modifiers.shift) || (i.modifiers.command && i.key_pressed(egui::Key::Y)),
                i.modifiers.command && i.key_pressed(egui::Key::K),
            )
        });
        if search {
            self.focus_search = true;
        }
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
            ui.add_space(4.0);
            if ui.button(t(crate::shell::AppAction::ImportMesh.short_key())).clicked() {
                self.perform(crate::shell::AppAction::ImportMesh);
            }
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
                        // the name is what people read; the kind is the group's header and the id
                        // (what scripts use) stays beside it, muted
                        let label = format!("{} ({})", s.name, s.id);
                        if ui
                            .selectable_label(sel, label)
                            .on_hover_text(format!("{}: {}", t(kind_key(kind)), s.detail))
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
        if !self.picks.is_empty() {
            self.picks_panel(ui);
        }
    }

    /// The operation history: what undo would step back through, newest first.
    pub(crate) fn history_panel(&mut self, ui: &mut egui::Ui) {
        crate::history_panel::feature_tree(self, ui);
        let labels = self.session.history.undo_labels();
        if labels.is_empty() {
            ui.weak(t("history.empty"));
        }
        for (i, label) in labels.iter().rev().enumerate() {
            // an undo step is named after its command: show the tool's name, the id on hover
            let name =
                surfacecraft_engine::cmd::find(label).map_or_else(|| (*label).to_string(), crate::workflow::label_of);
            let text = if i == 0 { egui::RichText::new(name).strong() } else { egui::RichText::new(name) };
            ui.label(text).on_hover_text(*label);
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

    /// The Deviation panel: legend, statistics, histogram and the maps (see [`crate::deviation_panel`]).
    pub(crate) fn deviation_panel(&mut self, ui: &mut egui::Ui) {
        crate::deviation_panel::show(self, ui);
    }

    /// The tool panel: the open tool's form, or the guide (and the assistant, when a language
    /// model is configured) while no tool is open.
    pub(crate) fn palette_panel(&mut self, ui: &mut egui::Ui) {
        if let Some(id) = self.palette_command {
            crate::command_panel::command_body(self, ui, id);
            return;
        }
        egui::ScrollArea::vertical().id_salt("dock-guide").auto_shrink([false, false]).show(ui, |ui| {
            // with a language model configured the assistant comes first: it is how that user
            // works, and it stays above the fold
            if self.provider.is_some() {
                self.assistant_panel(ui);
                ui.separator();
            }
            self.guide_panel(ui);
        });
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
        // Sketch mode takes precedence over the viewport tools: the sketch editor owns the
        // primary button and Esc (the innermost gesture); navigation stays (Alt+drag orbits).
        let sketching = self.sketch_editing();
        if sketching {
            self.cancel_interaction();
            self.sketch_input(ui, &resp, rect);
            self.sketch_navigation(ui, &resp, rect);
        } else {
            // navigation, the view cube and the active tool (selection, gizmo, ...): viewtools.rs
            self.view_input(ui, &resp, rect);
        }
        if self.fit_pending {
            self.camera.fit(&self.session.doc.bbox());
            self.fit_pending = false;
        }
        if !self.gpu {
            ui.painter().rect_filled(rect, 0.0, self.tokens.color(Token::ViewportBackground));
            if sketching {
                self.sketch_overlay(ui, rect);
                return;
            }
            let objects = self.session.doc.summaries().len();
            if objects > 0 {
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    tf("viewport.no_gpu", &[("n", &objects.to_string())]),
                    egui::FontId::proportional(14.0),
                    self.tokens.color(Token::TextMuted),
                );
            }
            self.paint_view_overlays(ui, rect);
            self.welcome_card(ui.ctx(), rect);
            return;
        }
        let ppp = ui.ctx().pixels_per_point();
        let size = [(rect.width() * ppp).round().max(1.0) as u32, (rect.height() * ppp).round().max(1.0) as u32];
        let frame = self.viewport_frame(size);
        ui.painter().add(egui_wgpu::Callback::new_paint_callback(rect, ViewportCallback(frame)));
        if sketching {
            self.sketch_overlay(ui, rect);
        } else {
            self.paint_view_overlays(ui, rect);
            self.welcome_card(ui.ctx(), rect);
        }
    }

    /// What the 3D viewport draws now at `size` pixels: the document's objects (with the
    /// selection highlights and the section clip), the live preview of a command form and the
    /// camera. The wgpu callback of the viewport and the offscreen screenshots both render it.
    pub fn viewport_frame(&mut self, size: [u32; 2]) -> Frame {
        let tokens = self.tokens.clone();
        let display = self.session.display_items();
        let overlay = self.frame_overlay(&display);
        let mut items: Vec<DrawItem> = display
            .into_iter()
            .map(|d| DrawItem {
                key: d.key,
                color: tokens.object(d.kind),
                colors: d.colors.clone(),
                colors_key: d.colors_key,
                highlight: self.highlight_for(&d),
                mesh: d.mesh,
            })
            .collect();
        items.extend(self.cmd_panel.draw_items(&tokens));
        let background = tokens.viewport_background().map(f64::from);
        Frame { size, camera: self.camera.clone(), origin: self.camera.target, items, background, overlay }
    }

    /// Lays the viewport out as with a GPU (no "no GPU" notice; the tool overlays are painted
    /// over an empty viewport) while the test renderer has none. Screenshots use it to
    /// composite the offscreen 3D image under the overlays the UI paints.
    pub fn lay_out_for_gpu(&mut self) {
        self.gpu = true;
    }

    /// Camera control while sketching: pan (right/middle drag), zoom (wheel) and Alt+drag
    /// orbit. The primary button belongs to the sketch editor, so orbiting needs Alt.
    fn sketch_navigation(&mut self, ui: &egui::Ui, resp: &egui::Response, rect: egui::Rect) {
        if resp.dragged_by(egui::PointerButton::Secondary) || resp.dragged_by(egui::PointerButton::Middle) {
            let d = resp.drag_delta();
            self.camera.pan(f64::from(d.x), f64::from(d.y), f64::from(rect.height()));
        }
        if resp.dragged_by(egui::PointerButton::Primary) && ui.input(|i| i.modifiers.alt) {
            let d = resp.drag_delta();
            self.camera.orbit(f64::from(d.x), f64::from(d.y));
        }
        if resp.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0.0 {
                self.camera.zoom(f64::from(scroll));
            }
        }
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

pub(crate) fn truncate(s: &str, n: usize) -> String {
    if s.chars().count() <= n { s.to_string() } else { s.chars().take(n).collect::<String>() + "…" }
}

/// What the UI knows when it prefills parameters.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PickContext {
    /// Selected object.
    pub selected: Option<ObjectId>,
    /// Picked points.
    pub points: Vec<[f64; 3]>,
    /// Picked triangles (original ids of the scan).
    pub triangles: Vec<u32>,
    /// Every object of the document (id, kind), so object references can be prefilled with a
    /// real object instead of a made-up id.
    pub objects: Vec<(ObjectId, ObjectKind)>,
    /// Picked B-rep faces (indices in the body record).
    pub faces: Vec<usize>,
    /// Picked B-rep edges (indices in the body record).
    pub edges: Vec<usize>,
}

impl PickContext {
    /// The object a reference parameter called `name` should point at: the selection when it is
    /// of a kind the parameter takes (see [`crate::forms::accepted_kinds`]), else the first
    /// object that is.
    #[must_use]
    pub fn reference_for(&self, name: &str, description: Option<&str>) -> Option<ObjectId> {
        let kinds = crate::forms::accepted_kinds(name, description);
        let fits = |k: ObjectKind| kinds.is_empty() || kinds.contains(&k);
        if let Some(sel) = self.selected
            && self.objects.iter().any(|(id, k)| *id == sel && fits(*k))
        {
            return Some(sel);
        }
        self.objects.iter().find(|(_, k)| fits(*k)).map(|(id, _)| *id)
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
            let described = p.get("description").and_then(Value::as_str);
            let v = match name {
                _ if is_obj_ref(&p) || name == "object" || name == "mesh" => {
                    ctx.reference_for(name, described).map_or(json!(""), |id| json!(id.0))
                }
                _ if p.get("items").is_some_and(is_obj_ref) => {
                    let n = p.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
                    let id = ctx.reference_for(name, described).map_or(json!(""), |id| json!(id.0));
                    Value::Array(vec![id; n])
                }
                "from" | "points" if !ctx.points.is_empty() => json!(ctx.points),
                "triangles" if !ctx.triangles.is_empty() => json!(ctx.triangles),
                "faces" if !ctx.faces.is_empty() => json!(ctx.faces),
                "edges" if !ctx.edges.is_empty() => json!(ctx.edges),
                _ => match crate::fieldpick::PickMode::of(&p).and_then(crate::fieldpick::PickMode::default_value) {
                    Some(datum) => datum,
                    None => example(&p, schema),
                },
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
