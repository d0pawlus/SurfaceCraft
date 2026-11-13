//! The form and its rows: one row per parameter, optional parameters behind a check box, and the
//! values a parameter starts with.
use super::names::*;
use super::schema::*;
use super::validate::*;
use super::widgets::*;
use super::*;
use crate::fieldpick::PickMode;
use crate::i18n::t;
use crate::theme::Token;
use serde_json::{Map, Number, Value, json};

pub(super) fn number_value(x: f64) -> Value {
    Number::from_f64(x).map_or(Value::Null, Value::Number)
}

pub(super) fn drag_speed(v: f64) -> f64 {
    (v.abs() * 0.01).max(0.01)
}

pub(super) fn locale_drag<'a>(
    x: &'a mut f64,
    speed: f64,
    unit: &'static str,
    min: f64,
    max: f64,
) -> egui::DragValue<'a> {
    egui::DragValue::new(x)
        .speed(speed)
        .range(min..=max)
        .clamp_existing_to_range(false)
        .suffix(unit)
        .custom_formatter(|n, _| fmt_num(n))
        .custom_parser(|s| s.trim().replace(',', ".").replace('\u{2212}', "-").parse::<f64>().ok())
}

pub(super) fn error_line(ui: &mut egui::Ui, ctx: &FormCtx<'_>, path: &str) {
    if let Some(msg) = ctx.issues.get(path) {
        ui.colored_label(ctx.tokens.color(Token::StatusError), msg);
    }
}

/// The whole form for an object schema; returns whether `value` changed. A `value` that is not
/// an object is replaced by one.
pub fn form(ui: &mut egui::Ui, root: &Value, value: &mut Value, ctx: &FormCtx<'_>) -> bool {
    let mut changed = false;
    if !value.is_object() {
        *value = Value::Object(Map::new());
        changed = true;
    }
    let (_, _, s) = classify(root, root);
    if let Some(map) = value.as_object_mut() {
        changed |= object_rows(ui, root, s, map, "", ctx, None);
    }
    changed
}

pub(super) fn object_rows(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    map: &mut Map<String, Value>,
    path: &str,
    ctx: &FormCtx<'_>,
    skip: Option<&str>,
) -> bool {
    let mut changed = false;
    let Some(props) = schema.get("properties").and_then(Value::as_object) else { return false };
    let required = required_of(schema);
    let mut names: Vec<&String> = props.keys().filter(|n| Some(n.as_str()) != skip).collect();
    names.sort_by_key(|n| !required.contains(&n.as_str()));
    if names.is_empty() {
        ui.weak(t("form.no_parameters"));
    }
    // required parameters come first; the optional ones follow under their own heading, so
    // what must be filled in is never lost among what may be left alone
    let mut optional_headed = !path.is_empty() || required.is_empty();
    for name in names {
        let Some(prop) = props.get(name) else { continue };
        let is_required = required.contains(&name.as_str());
        if !is_required && !optional_headed {
            ui.add_space(4.0);
            ui.weak(t("form.optional"));
            optional_headed = true;
        }
        changed |= property_row(ui, root, name, prop, is_required, map, path, ctx);
    }
    changed
}

#[allow(clippy::too_many_arguments)]
fn property_row(
    ui: &mut egui::Ui,
    root: &Value,
    name: &str,
    prop: &Value,
    required: bool,
    map: &mut Map<String, Value>,
    parent: &str,
    ctx: &FormCtx<'_>,
) -> bool {
    let path = join(parent, name);
    let (kind, _, schema) = classify(root, prop);
    let label = label_of(ctx, name);
    let hover = hover_of(root, prop);
    let mut changed = false;
    let set = map.get(name).is_some_and(|v| !v.is_null());
    let compound = matches!(kind, Kind::Object) || (kind == Kind::Array && !is_tuple(schema, root));
    ui.push_id(&path, |ui| {
        if compound && set {
            let id = ui.make_persistent_id(&path);
            let state = egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), id, true);
            let label_id = std::cell::Cell::new(None);
            let header = state.show_header(ui, |ui| {
                let r = ui.label(marked(&label, required));
                label_id.set(Some(r.id));
                if let Some(h) = &hover {
                    r.on_hover_text(h);
                }
            });
            header.body(|ui| {
                if !required {
                    changed |= presence(ui, map, name, false, prop, root, ctx);
                }
                if let Some(v) = map.get_mut(name) {
                    changed |= value_widget(ui, root, prop, v, name, &path, ctx, label_id.get());
                }
            });
            error_line(ui, ctx, &path);
            return;
        }
        // a narrow tool panel wraps a long row (a datum with its pick button) instead of
        // pushing the button out of sight
        ui.horizontal_wrapped(|ui| {
            let name_resp = if required {
                ui.label(marked(&label, true))
            } else {
                // the check box is the label of an optional parameter
                let mut on = set;
                let r = ui.checkbox(&mut on, &label);
                if r.changed() {
                    if on {
                        map.insert(name.to_string(), start_value(root, prop, name, &kind, ctx));
                    } else {
                        map.remove(name);
                    }
                    changed = true;
                }
                r
            };
            if let Some(h) = &hover {
                name_resp.clone().on_hover_text(h);
            }
            if map.get(name).is_some_and(|v| !v.is_null())
                && let Some(v) = map.get_mut(name)
            {
                let resp = value_widget(ui, root, prop, v, name, &path, ctx, Some(name_resp.id));
                changed |= resp;
            } else if required {
                // a required parameter without a value: give it one (the 3-2-1 datums start as the
                // world planes and axes, so they show their fields and can be edited or picked)
                let d = PickMode::of(prop)
                    .and_then(PickMode::default_value)
                    .filter(|_| kind == Kind::Any && path == name)
                    .unwrap_or_else(|| start_value(root, prop, name, &kind, ctx));
                map.insert(name.to_string(), d);
                changed = true;
            } else {
                ui.weak(t("form.not_set"));
            }
        });
        error_line(ui, ctx, &path);
    });
    changed
}

/// The value a parameter starts with: its default, and for an object reference an object of a
/// kind the parameter takes (the selected one when it fits).
pub(super) fn start_value(root: &Value, prop: &Value, name: &str, kind: &Kind, ctx: &FormCtx<'_>) -> Value {
    let description = description_of(root, prop);
    if *kind == Kind::ObjRef {
        let kinds = accepted_kinds(name, description.as_deref());
        return json!(preferred_object(ctx.objects, ctx.selected, &kinds).unwrap_or(0));
    }
    // A number or a switch the schema gives no default for starts with the default its
    // description states, so ticking an optional parameter on does not change what the tool
    // does until the value is edited (a neutral 0 or "off" could: a target of 0 triangles).
    let has_default = resolve(root, prop).get("default").or_else(|| prop.get("default")).is_some_and(|d| !d.is_null());
    let stated = description.as_deref().filter(|_| !has_default).and_then(described_default);
    match (kind, stated) {
        (Kind::Bool, Some(Value::Bool(on))) => Value::Bool(on),
        (Kind::Num, Some(Value::Number(n))) => Value::Number(n),
        (Kind::Int, Some(Value::Number(n))) => n.as_f64().map_or(Value::Null, |x| json!(x.round() as i64)),
        _ => default_value(root, prop, Some(ctx)),
    }
}

/// The default a parameter's description states: "default 0.5", "Default 3", "default true".
/// `None` when it states none or states it in words ("default: the object being aligned").
#[must_use]
pub fn described_default(description: &str) -> Option<Value> {
    let lower = description.to_ascii_lowercase();
    let (_, after) = lower.split_once("default ")?;
    let word: String = after.chars().take_while(|c| c.is_ascii_alphanumeric() || matches!(c, '.' | '-')).collect();
    match word.trim_end_matches('.') {
        "true" => Some(Value::Bool(true)),
        "false" => Some(Value::Bool(false)),
        number => number.parse::<f64>().ok().filter(|x| x.is_finite()).map(|x| json!(x)),
    }
}

pub(super) fn marked(label: &str, required: bool) -> String {
    if required { format!("{label} *") } else { label.to_string() }
}

/// A "set" check box for a compound optional parameter.
fn presence(
    ui: &mut egui::Ui,
    map: &mut Map<String, Value>,
    name: &str,
    set: bool,
    prop: &Value,
    root: &Value,
    ctx: &FormCtx<'_>,
) -> bool {
    let mut on = set || map.get(name).is_some_and(|v| !v.is_null());
    if ui.checkbox(&mut on, t("form.set")).changed() && !on {
        map.remove(name);
        return true;
    } else if on && !map.contains_key(name) {
        map.insert(name.to_string(), default_value(root, prop, Some(ctx)));
        return true;
    }
    false
}

pub(super) fn is_tuple(schema: &Value, root: &Value) -> bool {
    let (k, _, s) = classify(root, schema);
    if k != Kind::Array {
        return false;
    }
    let (Some(min), Some(max)) = (s.get("minItems").and_then(Value::as_u64), s.get("maxItems").and_then(Value::as_u64))
    else {
        return false;
    };
    let numeric = s.get("items").is_some_and(|i| matches!(classify(root, i).0, Kind::Num | Kind::Int));
    numeric && min == max && (1..=6).contains(&min)
}
