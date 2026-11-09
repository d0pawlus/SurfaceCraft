//! The feature history: every feature of the parametric tree with its state, suppress/restore
//! (`feature.suppress`), the error of a failed feature, and editing its parameters through the
//! same form as the command panel (`feature.edit`).
//!
//! Gap: the engine replays the whole history, there is no roll-back marker (a "replay up to
//! here" position) to drag; suppressing a feature is the closest it offers.
use crate::app::SurfaceCraftApp;
use crate::command_panel::CtxData;
use crate::forms;
use crate::i18n::{t, tf};
use crate::theme::Token;
use serde_json::{Map, Value, json};
use surfacecraft_doc::{FeatureOp, ObjectId};

/// State of the history panel.
#[derive(Default)]
pub struct HistoryPanel {
    /// Feature whose parameters are being edited.
    pub editing: Option<ObjectId>,
    original: Map<String, Value>,
    value: Value,
    schema: Value,
}

/// One feature row.
struct Row {
    id: ObjectId,
    name: String,
    kind: String,
    suppressed: bool,
    error: Option<String>,
    base: bool,
}

fn rows(app: &SurfaceCraftApp) -> Vec<Row> {
    app.session
        .doc
        .features
        .iter()
        .map(|f| Row {
            id: f.id,
            name: f.name.clone(),
            kind: serde_json::to_value(&f.op)
                .ok()
                .and_then(|v| v.get("kind").and_then(Value::as_str).map(str::to_string))
                .unwrap_or_default(),
            suppressed: f.suppressed,
            error: f.error.clone(),
            base: matches!(f.op, FeatureOp::Base { .. }),
        })
        .collect()
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

/// Draws the feature tree above the undo list.
pub fn feature_tree(app: &mut SurfaceCraftApp, ui: &mut egui::Ui) {
    let rows = rows(app);
    if rows.is_empty() {
        return;
    }
    ui.strong(tf("history.features", &[("n", &rows.len().to_string())]));
    ui.weak(t("history.no_rollback"));
    let mut suppress: Option<(ObjectId, bool)> = None;
    let mut edit: Option<ObjectId> = None;
    for r in &rows {
        ui.push_id(r.id.0, |ui| {
            ui.horizontal(|ui| {
                let mut active = !r.suppressed;
                if ui
                    .checkbox(&mut active, format!("{} ({})", r.name, r.kind))
                    .on_hover_text(t("history.suppress_hint"))
                    .changed()
                {
                    suppress = Some((r.id, !active));
                }
                if !r.base && ui.small_button(t("history.edit")).on_hover_text(t("history.edit_hint")).clicked() {
                    edit = Some(r.id);
                }
            });
            if let Some(e) = &r.error {
                ui.colored_label(
                    app.tokens.color(Token::StatusError),
                    tf("history.failed", &[("name", &r.name), ("error", e)]),
                );
            }
        });
    }
    if let Some((id, suppressed)) = suppress {
        let _ = app.run_command("feature.suppress", json!({"feature": id.0, "suppressed": suppressed}));
    }
    if let Some(id) = edit {
        begin_edit(app, id);
    }
    edit_form(app, ui);
    ui.separator();
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
