//! The Properties panel body: the chosen command as a form (or JSON for power users), with an
//! optional live preview.
//!
//! The preview runs the command on a copy of the document ([`surfacecraft_engine::Session::preview`]);
//! the result is drawn semi-transparent in the viewport. Apply runs the command for real, Cancel
//! discards the copy, so a preview never changes the document, its history or its log. Commands
//! the engine marks heavy are never previewed, and a preview waits for the parameters to stop
//! changing (debounce) so the UI thread is not blocked on every drag step.
use crate::app::SurfaceCraftApp;
use crate::fieldpick::PickRequest;
use crate::forms::{self, FormCtx, ObjInfo};
use crate::i18n::{t, tf};
use crate::theme::{Token, Tokens};
use crate::viewport::DrawItem;
use serde_json::Value;
use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_engine::EngineError;
use surfacecraft_engine::display::DisplayItem;
use surfacecraft_geom::Mesh;

/// Seconds the parameters must stay unchanged before the preview is recomputed.
const DEBOUNCE: f64 = 0.25;
/// Opacity of the previewed geometry.
const PREVIEW_ALPHA: f32 = 0.45;
/// Height of the footer before it was measured: OK, Cancel and the preview switch in one row.
const FOOTER_HEIGHT_GUESS: f32 = 52.0;

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
    /// Parameters the form marked as needing attention last frame.
    issues: usize,
    /// Height the footer took last frame (0 before it was drawn).
    footer_height: f32,
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
    /// The parameter the viewport is picking into, with its hint.
    picking: Option<(String, String)>,
    /// What the form asked for this frame (start or stop picking a field).
    request: RefCell<Option<PickRequest>>,
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
            picking: app.field_pick_hint(),
            request: RefCell::new(None),
            issues,
        }
    }

    /// The request a "pick in viewport" button left during the last form pass.
    pub fn take_request(&self) -> Option<PickRequest> {
        self.request.borrow_mut().take()
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
            picking: self.picking.as_ref().map(|(f, h)| (f.as_str(), h.as_str())),
            pick_request: &self.request,
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
    if app.field_pick.as_ref().is_some_and(|f| f.command != id) {
        // picking for another command's form
        app.field_pick = None;
    }
    // The form is drawn before the footer, so Tab reaches the fields and then OK and Cancel. The
    // footer keeps its place at the bottom, in the height it took last frame, and OK and Cancel
    // stay in reach however long the form is.
    let area = ui.max_rect();
    let footer_top = (area.bottom() - footer_reserve(panel)).max(area.top());
    let form_area = egui::Rect::from_min_max(area.min, egui::pos2(area.right(), footer_top));
    let footer_area = egui::Rect::from_min_max(egui::pos2(area.left(), footer_top), area.max);
    ui.scope_builder(egui::UiBuilder::new().max_rect(form_area), |ui| form_scroll(app, panel, ui, id));
    let footer_used = ui.scope_builder(egui::UiBuilder::new().max_rect(footer_area), |ui| {
        egui::Frame::NONE.inner_margin(egui::Margin::symmetric(0, 6)).show(ui, |ui| {
            ui.separator();
            footer(app, panel, ui, id);
        })
    });
    panel.footer_height = footer_used.response.rect.height();
}

/// The space the footer takes: what it took last frame, or a guess before it was drawn once.
fn footer_reserve(panel: &CommandPanel) -> f32 {
    if panel.footer_height > 0.0 { panel.footer_height } else { FOOTER_HEIGHT_GUESS }
}

/// The scrolling part of the tool: its head, the form (or the JSON text) and the preview's result.
fn form_scroll(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, ui: &mut egui::Ui, id: &'static str) {
    egui::ScrollArea::vertical().id_salt("dock-properties").auto_shrink([false, false]).show(ui, |ui| {
        header(app, ui, id);
        ui.separator();
        let schema = panel.schema(id);
        if panel.json_mode {
            panel.issues = 0;
            let l = ui.label(t("palette.parameters"));
            ui.add(egui::TextEdit::multiline(&mut app.palette_params).code_editor().desired_rows(6)).labelled_by(l.id);
        } else {
            form_section(app, panel, ui, id, &schema);
        }
        update_preview(app, panel, ui, id);
        result(app, ui, id);
    });
}

/// The head of the tool: its icon and name, a button that closes it, and what it does in a
/// sentence.
fn header(app: &mut SurfaceCraftApp, ui: &mut egui::Ui, id: &'static str) {
    let spec = surfacecraft_engine::cmd::find(id);
    let name = spec.map_or_else(|| id.to_string(), crate::workflow::label_of);
    let mut close = false;
    ui.horizontal(|ui| {
        let (rect, _) = ui.allocate_exact_size(egui::Vec2::splat(22.0), egui::Sense::hover());
        crate::icons::paint(ui, rect, crate::workflow::icon_of(id), app.tokens.color(Token::AccentHover));
        ui.add(egui::Label::new(egui::RichText::new(name).heading()).truncate());
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let style = crate::theme::icon_button_style(&app.tokens);
            let label = t("tool.close");
            close = craft_ui::buttons::IconButton::new(label, egui::Vec2::splat(22.0), &style)
                .show(ui, |ui, rect, tint| crate::icons::paint(ui, rect.shrink(4.0), "close", tint))
                .on_hover_text(t("form.cancel_hint"))
                .clicked();
        });
    });
    if let Some(c) = spec {
        ui.label(crate::workflow::hint_of(c));
    }
    if close {
        app.close_tool();
    }
}

/// The foot of the tool: what needs attention (parameters to fix, the state of the preview, why
/// the last run failed), then the preview switch, Cancel and OK, and under them the choice between
/// the form and the JSON a script would send. The messages are here, not under the last field,
/// so they are in view however long the form is.
fn footer(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, ui: &mut egui::Ui, id: &'static str) {
    messages(app, panel, ui, id);
    action_buttons(app, panel, ui, id);
    ui.horizontal(|ui| {
        ui.weak(id);
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            ui.selectable_value(&mut panel.json_mode, true, t("form.mode_json"));
            ui.selectable_value(&mut panel.json_mode, false, t("form.mode_form"));
        });
    });
}

/// What the user has to know before pressing OK again.
fn messages(app: &SurfaceCraftApp, panel: &CommandPanel, ui: &mut egui::Ui, id: &'static str) {
    if panel.issues > 0 {
        let text = tf("form.problems", &[("n", &panel.issues.to_string())]);
        ui.colored_label(app.tokens.color(Token::StatusWarning), text);
    }
    preview_status(app, panel, ui);
    if let Some(last) = app.session.log.last().filter(|e| e.command == id && !e.ok) {
        let text = crate::i18n::engine_message(&failure_text(&last.outcome));
        ui.colored_label(app.tokens.color(Token::StatusError), tf("form.failed", &[("error", &text)]));
    }
}

/// The words of a failure kept in the command log. The log keeps the error as the engine
/// serialises it (`{"kind": "invalid_params", "message": "…"}`); this is the sentence the same
/// error prints as.
#[must_use]
pub fn failure_text(outcome: &Value) -> String {
    let message = outcome.get("message").and_then(Value::as_str);
    match (outcome.get("kind").and_then(Value::as_str), message) {
        (Some("invalid_params"), Some(m)) => EngineError::InvalidParams(m.to_string()).to_string(),
        (Some("unavailable"), Some(m)) => EngineError::Unavailable(m.to_string()).to_string(),
        (Some("unknown_command"), Some(m)) => EngineError::UnknownCommand(m.to_string()).to_string(),
        (Some("internal"), Some(m)) => EngineError::Internal(m.to_string()).to_string(),
        (_, Some(m)) => m.to_string(),
        _ => outcome.as_str().map_or_else(|| outcome.to_string(), str::to_string),
    }
}

/// What the last run of this tool gave, as a table of values (a failure is in the footer).
fn result(app: &SurfaceCraftApp, ui: &mut egui::Ui, id: &'static str) {
    let Some(last) = app.session.log.last().filter(|e| e.command == id && e.ok) else { return };
    ui.separator();
    ui.strong(t("form.result"));
    result_rows(ui, &last.outcome);
}

/// A result as rows of name and value; anything nested is shown as the JSON it is.
fn result_rows(ui: &mut egui::Ui, value: &Value) {
    let Some(map) = value.as_object() else {
        let text = serde_json::to_string_pretty(value).unwrap_or_default();
        ui.monospace(crate::app::truncate(&text, 4000));
        return;
    };
    egui::Grid::new("tool-result").num_columns(2).striped(true).show(ui, |ui| {
        for (name, v) in map {
            ui.weak(forms::humanize(name));
            let text = match v {
                Value::Number(n) => n.as_f64().map_or_else(|| n.to_string(), result_number),
                Value::String(s) => s.clone(),
                Value::Bool(b) => t(if *b { "form.yes" } else { "form.no" }).to_string(),
                Value::Null => t("form.none").to_string(),
                other => other.to_string(),
            };
            ui.add(egui::Label::new(egui::RichText::new(crate::app::truncate(&text, 300)).monospace()).wrap());
            ui.end_row();
        }
    });
}

/// A number of a result: whole numbers as they are, others with four decimals.
fn result_number(x: f64) -> String {
    if x.fract() == 0.0 && x.abs() < 1e15 { format!("{x:.0}") } else { format!("{x:.4}") }
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
    // a "pick in viewport" button was pressed: start or stop picking that field
    if let Some(req) = data.take_request() {
        app.handle_pick_request(req);
    }
    panel.issues = data.issues.len();
}

fn action_buttons(app: &mut SurfaceCraftApp, panel: &mut CommandPanel, ui: &mut egui::Ui, id: &'static str) {
    let blocked = no_preview(id);
    // Enter confirms and Esc closes, as in every dialog, while the keys are the tool's
    let (enter, escape) = ui.input(|i| (i.key_pressed(egui::Key::Enter), i.key_pressed(egui::Key::Escape)));
    let (confirm, close) = (app.keys_free && enter, app.keys_free && escape && !enter);
    ui.horizontal(|ui| {
        match blocked {
            None => {
                if ui.checkbox(&mut panel.preview_on, t("form.preview")).on_hover_text(t("form.preview_hint")).changed()
                    && !panel.preview_on
                {
                    panel.clear();
                }
            }
            Some(NoPreview::Heavy) => {
                ui.add_enabled(false, egui::Checkbox::new(&mut false, t("form.preview")))
                    .on_disabled_hover_text(t("form.preview_heavy"));
            }
            Some(NoPreview::ReadOnly) => {}
        }
        // OK is the last button on the right, Cancel beside it
        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
            let ok = crate::theme::primary_button(&app.tokens, t("form.ok"));
            if ui.add(ok).on_hover_text(t("form.apply_hint")).clicked() || confirm {
                app.run_palette();
                panel.cancel_preview();
            }
            if ui.button(t("form.cancel")).on_hover_text(t("form.cancel_hint")).clicked() || close {
                panel.cancel_preview();
                app.close_tool();
            }
        });
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
            let e = crate::i18n::engine_message(e);
            ui.colored_label(err, tf("form.preview_failed", &[("error", &e)]));
        }
        PreviewStatus::BadJson => {
            ui.colored_label(err, t("form.err.json"));
        }
    }
}
