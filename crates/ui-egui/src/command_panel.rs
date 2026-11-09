//! The Properties panel body: the chosen command as a form (or JSON for power users), with an
//! optional live preview.
//!
//! The preview runs the command on a copy of the document ([`surfacecraft_engine::Session::preview`]);
//! the result is drawn semi-transparent in the viewport. Apply runs the command for real, Cancel
//! discards the copy, so a preview never changes the document, its history or its log. Commands
//! the engine marks heavy are never previewed, and a preview waits for the parameters to stop
//! changing (debounce) so the UI thread is not blocked on every drag step.
use crate::app::SurfaceCraftApp;
use crate::forms::{self, FormCtx, ObjInfo};
use crate::i18n::{t, tf};
use crate::theme::{Token, Tokens};
use crate::viewport::DrawItem;
use serde_json::Value;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::display::DisplayItem;
use surfacecraft_geom::Mesh;

/// Seconds the parameters must stay unchanged before the preview is recomputed.
const DEBOUNCE: f64 = 0.25;
/// Opacity of the previewed geometry.
const PREVIEW_ALPHA: f32 = 0.45;

/// Why a command has no live preview.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NoPreview {
    /// It does not change the document.
    ReadOnly,
    /// It is too slow to run on every change.
    Heavy,
}

/// What the preview of the current parameters is.
#[derive(Clone, Debug, Default, PartialEq)]
pub enum PreviewStatus {
    /// Preview is off.
    #[default]
    Off,
    /// Waiting for the parameters to settle.
    Pending,
    /// Computed: how many objects it adds or changes.
    Ready(usize),
    /// The command failed on the copy.
    Failed(String),
    /// The parameters are not valid JSON.
    BadJson,
}

/// One object of the previewed result.
#[derive(Clone)]
pub struct PreviewItem {
    /// Object id in the preview copy.
    pub id: ObjectId,
    /// Kind (its colour).
    pub kind: ObjectKind,
    /// Geometry.
    pub mesh: Arc<Mesh>,
    /// Identity of `mesh` for the GPU cache.
    pub key: usize,
}

/// State of the command panel (the form's value, the preview).
#[derive(Default)]
pub struct CommandPanel {
    /// Edit the parameters as JSON text instead of the form.
    pub json_mode: bool,
    /// Preview on?
    pub preview_on: bool,
    /// State of the preview.
    pub status: PreviewStatus,
    /// What the preview adds or changes.
    pub items: Vec<PreviewItem>,
    value: Value,
    synced: String,
    schemas: HashMap<&'static str, Arc<Value>>,
    /// Command and parameters seen last frame.
    seen: Option<(&'static str, String)>,
    /// Command and parameters of the preview in `items`.
    previewed: Option<(&'static str, String)>,
    /// Command the panel showed last frame.
    last_command: Option<&'static str>,
    /// When the parameters last changed (egui time).
    changed_at: f64,
    /// `session.log.len()` when the preview was computed.
    log_len: usize,
}

impl CommandPanel {
    /// Turns the preview off and forgets its result (Cancel).
    pub fn cancel_preview(&mut self) {
        self.preview_on = false;
        self.clear();
    }

    fn clear(&mut self) {
        self.items.clear();
        self.previewed = None;
        self.status = PreviewStatus::Off;
    }

    /// The previewed geometry as viewport items: the kind's colour at a fixed low opacity.
    #[must_use]
    pub fn draw_items(&self, tokens: &Tokens) -> Vec<DrawItem> {
        self.items
            .iter()
            .map(|p| {
                let mut color = tokens.object(p.kind);
                color[3] = PREVIEW_ALPHA;
                DrawItem { key: p.key, mesh: p.mesh.clone(), color, colors: None, colors_key: 0, highlight: None }
            })
            .collect()
    }

    fn schema(&mut self, id: &'static str) -> Arc<Value> {
        self.schemas
            .entry(id)
            .or_insert_with(|| Arc::new(surfacecraft_engine::cmd::find(id).map_or(Value::Null, |c| (c.input)())))
            .clone()
    }
}

/// Why `id` cannot be previewed, `None` when it can.
#[must_use]
pub fn no_preview(id: &str) -> Option<NoPreview> {
    match surfacecraft_engine::cmd::find(id) {
        Some(c) if !c.undoable => Some(NoPreview::ReadOnly),
        Some(_) if surfacecraft_engine::cmd::is_heavy(id) => Some(NoPreview::Heavy),
        Some(_) => None,
        None => Some(NoPreview::ReadOnly),
    }
}

/// The objects of the document for the pickers.
#[must_use]
pub fn objects_of(app: &SurfaceCraftApp) -> Vec<ObjInfo> {
    app.session.doc.summaries().into_iter().map(|s| ObjInfo { id: s.id.0, kind: s.kind, name: s.name }).collect()
}

/// What a form needs from the app, owned so the form can borrow it while the app is edited.
pub struct CtxData {
    objects: Vec<ObjInfo>,
    selected: Option<u64>,
    pick_objects: Vec<u64>,
    pick_points: Vec<[f64; 3]>,
    pick_triangles: Vec<u32>,
    /// Validation messages by path (the first per path).
    pub issues: HashMap<String, String>,
}

impl CtxData {
    /// Snapshots the document's objects and the picks; `validate` produces the (path, message)
    /// pairs from the objects.
    pub fn of(app: &SurfaceCraftApp, validate: impl FnOnce(&[ObjInfo]) -> Vec<(String, String)>) -> Self {
        let objects = objects_of(app);
        let mut issues: HashMap<String, String> = HashMap::new();
        for (p, m) in validate(&objects) {
            issues.entry(p).or_insert(m);
        }
        Self {
            objects,
            selected: app.selected_object.map(|o| o.0),
            pick_objects: app.picks.iter().map(|p| p.object.0).collect(),
            pick_points: app.picks.iter().map(|p| [p.point.x, p.point.y, p.point.z]).collect(),
            pick_triangles: app.picks.iter().map(|p| p.triangle).collect(),
            issues,
        }
    }

    /// The form context for `command`.
    #[must_use]
    pub fn ctx<'a>(&'a self, command: &'a str, tokens: &'a Tokens) -> FormCtx<'a> {
        FormCtx {
            command,
            objects: &self.objects,
            selected: self.selected,
            pick_objects: &self.pick_objects,
            pick_points: &self.pick_points,
            pick_triangles: &self.pick_triangles,
            tokens,
            issues: &self.issues,
        }
    }
}

/// A cheap identity of a display item's geometry (the preview copy tessellates bodies again, so
/// pointers cannot tell what changed).
fn signature(d: &DisplayItem) -> (u64, u64) {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut mix = |x: u64| h = (h ^ x).wrapping_mul(0x0000_0100_0000_01b3);
    mix(d.id.0);
    mix(d.mesh.positions.len() as u64);
    mix(d.mesh.triangles.len() as u64);
    for p in &d.mesh.positions {
        mix(p.x.to_bits());
        mix(p.y.to_bits());
        mix(p.z.to_bits());
    }
    (d.id.0, h)
}

/// Computes the preview of `id` with `params` on a copy of the session.
fn run_preview(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, id: &'static str, params: Value) {
    panel.log_len = app.session.log.len();
    match app.session.preview(id, params) {
        Ok(mut copy) => {
            let base: HashSet<(u64, u64)> = app.session.display_items().iter().map(signature).collect();
            panel.items = copy
                .display_items()
                .into_iter()
                .filter(|d| !base.contains(&signature(d)))
                .map(|d| PreviewItem { id: d.id, kind: d.kind, key: d.key, mesh: d.mesh })
                .collect();
            panel.status = PreviewStatus::Ready(panel.items.len());
        }
        Err(e) => {
            panel.items.clear();
            panel.status = PreviewStatus::Failed(e.to_string());
        }
    }
}

/// Recomputes the preview when the parameters settled, the command changed or the document did.
fn update_preview(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, ui: &egui::Ui, id: &'static str) {
    if !panel.preview_on || no_preview(id).is_some() {
        return;
    }
    let now = ui.input(|i| i.time);
    let key = (id, app.palette_params.clone());
    if panel.seen.as_ref() != Some(&key) {
        // the parameters (or the command) changed: restart the debounce
        panel.seen = Some(key.clone());
        panel.changed_at = now;
    }
    let stale = panel.previewed.as_ref() != Some(&key) || panel.log_len != app.session.log.len();
    if !stale {
        return;
    }
    // the first preview is immediate; later ones wait for the parameters to settle
    if panel.previewed.is_some() && now - panel.changed_at < DEBOUNCE {
        panel.status = PreviewStatus::Pending;
        ui.ctx().request_repaint_after(std::time::Duration::from_secs_f64(DEBOUNCE));
        return;
    }
    match serde_json::from_str::<Value>(&key.1) {
        Ok(v) => run_preview(app, panel, id, v),
        Err(_) => {
            panel.items.clear();
            panel.status = PreviewStatus::BadJson;
        }
    }
    panel.previewed = Some(key);
}

/// The body of the Properties panel for command `id`.
pub fn command_body(app: &mut SurfaceCraftApp, ui: &mut egui::Ui, id: &'static str) {
    let mut panel = std::mem::take(&mut app.cmd_panel);
    body(app, &mut panel, ui, id);
    app.cmd_panel = panel;
}

fn body(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, ui: &mut egui::Ui, id: &'static str) {
    if panel.last_command != Some(id) {
        // another command: a preview of the old one makes no sense
        panel.cancel_preview();
        panel.last_command = Some(id);
    }
    ui.strong(id);
    if let Some(c) = surfacecraft_engine::cmd::find(id) {
        ui.label(c.summary);
    }
    ui.horizontal(|ui| {
        ui.selectable_value(&mut panel.json_mode, false, t("form.mode_form"));
        ui.selectable_value(&mut panel.json_mode, true, t("form.mode_json"));
    });
    let schema = panel.schema(id);
    if panel.json_mode {
        let l = ui.label(t("palette.parameters"));
        ui.add(egui::TextEdit::multiline(&mut app.palette_params).code_editor().desired_rows(6)).labelled_by(l.id);
    } else {
        form_section(app, panel, ui, id, &schema);
    }
    action_buttons(app, panel, ui, id);
    update_preview(app, panel, ui, id);
    preview_status(app, panel, ui);
    if let Some(last) = app.session.log.last()
        && last.command == id
    {
        ui.separator();
        egui::ScrollArea::vertical().id_salt("result").show(ui, |ui| {
            let text = serde_json::to_string_pretty(&last.outcome).unwrap_or_default();
            ui.monospace(crate::app::truncate(&text, 4000));
        });
    }
}

fn form_section(
    app: &mut SurfaceCraftApp,
    panel: &mut CommandPanel,
    ui: &mut egui::Ui,
    id: &'static str,
    schema: &Value,
) {
    // the text is the shared state (the palette, the tests and the JSON mode edit it): re-read it
    // when somebody else changed it
    if panel.synced != app.palette_params {
        match serde_json::from_str::<Value>(&app.palette_params) {
            Ok(v) => panel.value = v,
            Err(_) => {
                ui.colored_label(app.tokens.color(Token::StatusError), t("form.err.json"));
            }
        }
        panel.synced = app.palette_params.clone();
    }
    let data = CtxData::of(app, |objects| {
        forms::validate(schema, &panel.value, objects).into_iter().map(|i| (i.path, i.message)).collect()
    });
    let ctx = data.ctx(id, &app.tokens);
    if forms::form(ui, schema, &mut panel.value, &ctx) {
        app.palette_params = serde_json::to_string_pretty(&panel.value).unwrap_or_else(|_| "{}".into());
        panel.synced = app.palette_params.clone();
    }
    if !data.issues.is_empty() {
        ui.colored_label(
            app.tokens.color(Token::StatusWarning),
            tf("form.problems", &[("n", &data.issues.len().to_string())]),
        );
    }
}

fn action_buttons(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, ui: &mut egui::Ui, id: &'static str) {
    let blocked = no_preview(id);
    ui.horizontal(|ui| {
        if panel.preview_on && blocked.is_none() {
            if ui.button(t("form.apply")).on_hover_text(t("form.apply_hint")).clicked() {
                app.run_palette();
                panel.cancel_preview();
            }
            if ui.button(t("form.cancel")).on_hover_text(t("form.cancel_hint")).clicked() {
                panel.cancel_preview();
            }
        } else if ui.button(t("palette.run")).clicked() {
            app.run_palette();
            panel.clear();
        }
        match blocked {
            None => {
                if ui.checkbox(&mut panel.preview_on, t("form.preview")).changed() && !panel.preview_on {
                    panel.clear();
                }
            }
            Some(NoPreview::Heavy) => {
                ui.add_enabled(false, egui::Checkbox::new(&mut false, t("form.preview")))
                    .on_disabled_hover_text(t("form.preview_heavy"));
            }
            Some(NoPreview::ReadOnly) => {}
        }
    });
}

fn preview_status(app: &SurfaceCraftApp, panel: &CommandPanel, ui: &mut egui::Ui) {
    let err = app.tokens.color(Token::StatusError);
    match &panel.status {
        PreviewStatus::Off => {}
        PreviewStatus::Pending => {
            ui.weak(t("form.preview_pending"));
        }
        PreviewStatus::Ready(n) => {
            ui.weak(tf("form.preview_ready", &[("n", &n.to_string())]));
        }
        PreviewStatus::Failed(e) => {
            ui.colored_label(err, tf("form.preview_failed", &[("error", e)]));
        }
        PreviewStatus::BadJson => {
            ui.colored_label(err, t("form.err.json"));
        }
    }
}
