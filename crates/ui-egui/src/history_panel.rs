//! The feature history: every feature of the parametric tree with its state, suppress/restore
//! (`feature.suppress`), the error of a failed feature, and editing its parameters through the
//! same form as the command panel (`feature.edit`). The roll-back marker (`feature.rollback`) sits
//! between the active and the rolled-back features: click it to restore every feature, drag it to
//! another position to roll the body back to that step. Rolled-back features are greyed out.
use crate::app::SurfaceCraftApp;
use crate::command_panel::CtxData;
use crate::forms;
use crate::i18n::{t, tf};
use crate::theme::Token;
use serde_json::{Map, Value, json};
use surfacecraft_doc::{FeatureObject, FeatureOp, ObjectId};

/// State of the history panel.
#[derive(Default)]
pub struct HistoryPanel {
    /// Feature whose parameters are being edited.
    pub editing: Option<ObjectId>,
    original: Map<String, Value>,
    value: Value,
    schema: Value,
    /// Height of each feature row on the last frame, in history order (where a dragged marker
    /// is dropped).
    row_centres: Vec<f32>,
}

/// One feature row.
struct Row {
    id: ObjectId,
    name: String,
    kind: String,
    suppressed: bool,
    error: Option<String>,
    base: bool,
    rolled_back: bool,
}

/// The features in history order: the active ones, then the parked ones behind the marker.
fn rows(app: &SurfaceCraftApp) -> Vec<Row> {
    let active = app.session.doc.features.len();
    app.session
        .doc
        .features
        .iter()
        .chain(app.session.doc.rolled_back.iter())
        .enumerate()
        .map(|(k, f)| row_of(f, k >= active))
        .collect()
}

fn row_of(f: &FeatureObject, rolled_back: bool) -> Row {
    Row {
        id: f.id,
        name: f.name.clone(),
        kind: serde_json::to_value(&f.op)
            .ok()
            .and_then(|v| v.get("kind").and_then(Value::as_str).map(str::to_string))
            .unwrap_or_default(),
        suppressed: f.suppressed,
        error: f.error.clone(),
        base: matches!(f.op, FeatureOp::Base { .. }),
        rolled_back,
    }
}

/// A schema guessed from a value (features without a command of their own).
fn infer(v: &Value) -> Value {
    match v {
        Value::Bool(_) => json!({"type": "boolean"}),
        Value::Number(n) if n.is_i64() || n.is_u64() => json!({"type": "integer"}),
        Value::Number(_) => json!({"type": "number"}),
        Value::String(_) => json!({"type": "string"}),
        Value::Array(a) => json!({"type": "array", "items": a.first().map_or(json!(true), infer)}),
        _ => json!(true),
    }
}

/// The form schema of a feature: the parameters of its creating command that its operation
/// holds, or one guessed from the operation's values.
fn schema_for(kind: &str, op: &Map<String, Value>) -> Value {
    let cmd = surfacecraft_engine::cmd::find(&format!("feature.{kind}")).map(|c| (c.input)());
    let mut root = cmd.unwrap_or_else(|| json!({"type": "object", "properties": {}}));
    let mut required: Vec<Value> = Vec::new();
    if let Some(props) = root.get_mut("properties").and_then(Value::as_object_mut) {
        props.retain(|k, _| op.contains_key(k) && k != "kind");
        for (k, v) in op {
            if k != "kind" && !props.contains_key(k) && !k.ends_with("_refs") {
                props.insert(k.clone(), infer(v));
            }
        }
        // a parameter the feature leaves unset stays optional
        required = props
            .keys()
            .filter(|k| op.get(*k).is_some_and(|v| !v.is_null()))
            .map(|k| Value::String(k.clone()))
            .collect();
    }
    root["required"] = Value::Array(required);
    root["additionalProperties"] = Value::Bool(true);
    root
}

/// What the user asked for in the tree this frame.
#[derive(Default)]
struct Asks {
    suppress: Option<(ObjectId, bool)>,
    edit: Option<ObjectId>,
    /// The `to` of a `feature.rollback`.
    roll: Option<Value>,
}

/// Draws the feature tree above the undo list.
pub fn feature_tree(app: &mut SurfaceCraftApp, ui: &mut egui::Ui) {
    let rows = rows(app);
    if rows.is_empty() {
        return;
    }
    let marker = app.session.doc.features.len();
    ui.strong(tf("history.features", &[("n", &rows.len().to_string())]));
    let mut asks = Asks::default();
    let mut centres = Vec::with_capacity(rows.len());
    for (k, r) in rows.iter().enumerate() {
        if k == marker {
            marker_line(app, ui, &mut asks, rows.len());
        }
        let centre = ui.push_id(r.id.0, |ui| feature_row(app, ui, r, &mut asks)).inner;
        centres.push(centre);
    }
    if marker == rows.len() {
        marker_line(app, ui, &mut asks, rows.len());
    }
    app.hist_panel.row_centres = centres;
    run_asks(app, asks);
    edit_form(app, ui);
    ui.separator();
}

/// One feature: its checkbox, its edit and roll-back buttons and its error. Returns the height
/// of its centre.
fn feature_row(app: &SurfaceCraftApp, ui: &mut egui::Ui, r: &Row, asks: &mut Asks) -> f32 {
    let inner = ui.vertical(|ui| {
        ui.add_enabled_ui(!r.rolled_back, |ui| {
            ui.horizontal(|ui| {
                let mut active = !r.suppressed;
                if ui
                    .checkbox(&mut active, format!("{} ({})", r.name, r.kind))
                    .on_hover_text(t("history.suppress_hint"))
                    .changed()
                {
                    asks.suppress = Some((r.id, !active));
                }
                if !r.base && ui.small_button(t("history.edit")).on_hover_text(t("history.edit_hint")).clicked() {
                    asks.edit = Some(r.id);
                }
                if !r.rolled_back
                    && !r.base
                    && ui.small_button(t("history.roll_here")).on_hover_text(t("history.roll_here_hint")).clicked()
                {
                    asks.roll = Some(json!({"feature": r.id.0}));
                }
            });
        });
        if let Some(e) = &r.error {
            ui.colored_label(
                app.tokens.color(Token::StatusError),
                tf("history.failed", &[("name", &r.name), ("error", &crate::i18n::engine_message(e))]),
            );
        }
    });
    inner.response.rect.center().y
}

/// The roll-back marker between the active and the parked features. A click restores every
/// feature; a drag drops the marker in front of the feature under the pointer.
fn marker_line(app: &SurfaceCraftApp, ui: &mut egui::Ui, asks: &mut Asks, count: usize) {
    let label =
        egui::Label::new(egui::RichText::new(t("history.marker")).color(app.tokens.color(Token::AccentPrimary)))
            .sense(egui::Sense::click_and_drag());
    let resp = ui.add(label).on_hover_text(t("history.marker_hint"));
    if resp.clicked() {
        asks.roll = Some(json!("end"));
    }
    if resp.drag_stopped()
        && let Some(y) = ui.ctx().input(|i| i.pointer.interact_pos()).map(|p| p.y)
    {
        let before = app.hist_panel.row_centres.iter().filter(|c| **c < y).count();
        asks.roll = Some(if before >= count { json!("end") } else { json!({"index": before}) });
    }
}

fn run_asks(app: &mut SurfaceCraftApp, asks: Asks) {
    if let Some((id, suppressed)) = asks.suppress {
        let _ = app.run_command("feature.suppress", json!({"feature": id.0, "suppressed": suppressed}));
    }
    if let Some(to) = asks.roll {
        let _ = app.run_command("feature.rollback", json!({"to": to}));
    }
    if let Some(id) = asks.edit {
        begin_edit(app, id);
    }
}

/// Is the feature greyed out, behind the roll-back marker?
#[must_use]
pub fn is_rolled_back(app: &SurfaceCraftApp, id: ObjectId) -> bool {
    app.session.doc.rolled_back.iter().any(|f| f.id == id)
}

/// Opens the parameter form of feature `id`.
pub fn begin_edit(app: &mut SurfaceCraftApp, id: ObjectId) {
    let Some(f) = app.session.doc.feature(id) else { return };
    let Some(Value::Object(op)) = serde_json::to_value(&f.op).ok() else { return };
    let kind = op.get("kind").and_then(Value::as_str).unwrap_or("").to_string();
    let schema = schema_for(&kind, &op);
    let mut shown = Map::new();
    if let Some(props) = schema.get("properties").and_then(Value::as_object) {
        for k in props.keys() {
            if let Some(v) = op.get(k) {
                shown.insert(k.clone(), v.clone());
            }
        }
    }
    let h = &mut app.hist_panel;
    h.original = shown.clone();
    h.value = Value::Object(shown);
    h.schema = schema;
    h.editing = Some(id);
}

fn edit_form(app: &mut SurfaceCraftApp, ui: &mut egui::Ui) {
    let Some(id) = app.hist_panel.editing else { return };
    let Some(name) = app.session.doc.feature(id).map(|f| f.name.clone()) else {
        app.hist_panel.editing = None;
        return;
    };
    let mut panel = std::mem::take(&mut app.hist_panel);
    ui.group(|ui| {
        ui.strong(tf("history.editing", &[("name", &name)]));
        let data = CtxData::of(app, |objects| {
            forms::validate(&panel.schema, &panel.value, objects).into_iter().map(|i| (i.path, i.message)).collect()
        });
        let ctx = data.ctx("feature.edit", &app.tokens);
        forms::form(ui, &panel.schema, &mut panel.value, &ctx);
        let mut apply = false;
        let mut cancel = false;
        ui.horizontal(|ui| {
            apply = ui.button(t("form.apply")).clicked();
            cancel = ui.button(t("form.cancel")).clicked();
        });
        if apply {
            let set: Map<String, Value> = panel
                .value
                .as_object()
                .map(|m| {
                    m.iter()
                        .filter(|(k, v)| panel.original.get(*k) != Some(v))
                        .map(|(k, v)| (k.clone(), v.clone()))
                        .collect()
                })
                .unwrap_or_default();
            // nothing changed: nothing to run; a failed edit keeps the form open with the error
            if set.is_empty() || app.run_command("feature.edit", json!({"feature": id.0, "set": set})).is_ok() {
                panel.editing = None;
            }
        }
        if cancel {
            panel.editing = None;
        }
    });
    app.hist_panel = panel;
}
