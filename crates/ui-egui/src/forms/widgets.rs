//! The widget of one parameter value, by its kind, and the pick, enum and tuple widgets.
use super::collections::*;
use super::names::*;
use super::rows::*;
use super::schema::*;
use super::validate::*;
use super::*;
use crate::fieldpick::{PickKind, PickMode, PickRequest};
use crate::i18n::{self, find, t};
use crate::theme::Token;
use serde_json::{Value, json};

#[allow(clippy::too_many_arguments)]
pub(super) fn value_widget(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<egui::Id>,
) -> bool {
    let (kind, _, s) = classify(root, schema);
    let mut changed = false;
    let tag = |r: egui::Response| -> egui::Response {
        match label {
            Some(l) => r.labelled_by(l),
            None => r,
        }
    };
    match kind {
        Kind::Bool => {
            let mut b = value.as_bool().unwrap_or(false);
            if tag(ui.checkbox(&mut b, t("form.enabled"))).changed() {
                *value = Value::Bool(b);
                changed = true;
            }
        }
        Kind::Int => {
            let mut x = value.as_f64().unwrap_or(0.0);
            let (min, max) =
                (limit(s, "minimum").unwrap_or(f64::NEG_INFINITY), limit(s, "maximum").unwrap_or(f64::INFINITY));
            if tag(ui.add(locale_drag(&mut x, 1.0, unit_of(name), min, max))).changed() {
                *value = json!(x.round() as i64);
                changed = true;
            }
        }
        Kind::Num => {
            let mut x = value.as_f64().unwrap_or(0.0);
            let (min, max) =
                (limit(s, "minimum").unwrap_or(f64::NEG_INFINITY), limit(s, "maximum").unwrap_or(f64::INFINITY));
            let speed = drag_speed(x);
            if tag(ui.add(locale_drag(&mut x, speed, unit_of(name), min, max))).changed() {
                *value = number_value(x);
                changed = true;
            }
            if min.is_finite() || max.is_finite() {
                ui.weak(limits_text(min, max));
            }
        }
        Kind::Str => {
            let mut text = value.as_str().unwrap_or("").to_string();
            if tag(ui.add(egui::TextEdit::singleline(&mut text).desired_width(160.0))).changed() {
                *value = Value::String(text);
                changed = true;
            }
        }
        Kind::Enum(options) => {
            let shown = enum_label(value);
            let mut chosen: Option<Value> = None;
            let resp = egui::ComboBox::from_id_salt(path).selected_text(shown).show_ui(ui, |ui| {
                for o in &options {
                    let r = ui.selectable_label(o == value, enum_label(o));
                    // what the choice means, when the schema says
                    let r = match option_hint(root, s, o) {
                        Some(h) => r.on_hover_text(h),
                        None => r,
                    };
                    if r.clicked() {
                        chosen = Some(o.clone());
                    }
                }
            });
            tag(resp.response);
            if let Some(c) = chosen
                && &c != value
            {
                *value = c;
                changed = true;
            }
        }
        Kind::ObjRef => {
            let kinds = accepted_kinds(name, description_of(root, schema).as_deref());
            changed |= object_picker(ui, value, &kinds, path, ctx, label);
        }
        Kind::Array => {
            if is_tuple(schema, root) {
                changed |= tuple_row(ui, s, value, name, label);
            } else {
                changed |= list_widget(ui, root, s, value, name, path, ctx, label);
            }
        }
        Kind::Object => {
            if let Some(map) = value.as_object_mut() {
                changed |= object_rows(ui, root, s, map, path, ctx, None);
            }
        }
        Kind::Any => match PickMode::of(schema).map(PickMode::kind).and_then(PickKind::need).filter(|_| path == name) {
            // a 3-2-1 datum: points clicked in the viewport (how most people set a datum), or
            // its JSON text
            Some(need) => {
                // under its label, so the summary and the buttons have the panel's whole width
                ui.end_row();
                ui.vertical(|ui| {
                    // what the datum is, in words; its JSON stays one click away, on the row of
                    // the pick button so three datums fit the panel without scrolling
                    ui.label(crate::fieldpick::datum_summary(value));
                    let open_id = ui.make_persistent_id(("datum-json", path));
                    let mut open = ui.data(|d| d.get_temp::<bool>(open_id)).unwrap_or(false);
                    ui.horizontal(|ui| {
                        pick_button(ui, ctx, name, path, PickKind::Datum { need });
                        if ui.selectable_label(open, t("form.datum.edit")).clicked() {
                            open = !open;
                            ui.data_mut(|d| d.insert_temp(open_id, open));
                        }
                    });
                    pick_hint_line(ui, ctx, name);
                    if open {
                        changed |= json_text(ui, value, path, label);
                    }
                });
            }
            None => match variants(root, s) {
                Some(vars) => changed |= variant_widget(ui, root, &vars, value, path, ctx, label),
                None => changed |= json_text(ui, value, path, label),
            },
        },
    }
    changed
}

/// The name of an alternative (`variant.<tag>` in the catalogue, else the tag in plain words).
fn variant_label(tag: &str) -> String {
    find(&format!("variant.{tag}")).map_or_else(|| humanize(tag), str::to_string)
}

/// A parameter that is one of several shapes: a list to choose the shape, and under it the
/// fields of the chosen one.
fn variant_widget(
    ui: &mut egui::Ui,
    root: &Value,
    vars: &[Variant<'_>],
    value: &mut Value,
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<egui::Id>,
) -> bool {
    let mut changed = false;
    ui.vertical(|ui| {
        let current = vars.iter().position(|v| v.holds(value));
        let shown =
            current.and_then(|i| vars.get(i)).map_or_else(|| t("form.none").to_string(), |v| variant_label(v.tag));
        let mut chosen: Option<usize> = None;
        let combo = egui::ComboBox::from_id_salt(("variant", path)).selected_text(shown).show_ui(ui, |ui| {
            for (i, v) in vars.iter().enumerate() {
                let r = ui.selectable_label(current == Some(i), variant_label(v.tag));
                let r = match v.schema.get("description").and_then(Value::as_str) {
                    Some(d) => r.on_hover_text(i18n::hint(d)),
                    None => r,
                };
                if r.clicked() {
                    chosen = Some(i);
                }
            }
        });
        if let Some(l) = label {
            combo.response.labelled_by(l);
        }
        if let Some(v) = chosen.filter(|i| Some(*i) != current).and_then(|i| vars.get(i)) {
            *value = default_value(root, v.schema, Some(ctx));
            changed = true;
        }
        let Some(v) = vars.iter().find(|v| v.holds(value)) else { return };
        let Some(map) = value.as_object_mut() else { return };
        match v.tag_field {
            Some(field) => changed |= object_rows(ui, root, v.schema, map, path, ctx, Some(field)),
            None => {
                let inner = v.schema.get("properties").and_then(|p| p.get(v.tag));
                if let (Some(schema), Some(fields)) = (inner, map.get_mut(v.tag)) {
                    changed |= value_widget(ui, root, schema, fields, v.tag, &join(path, v.tag), ctx, None);
                }
            }
        }
    });
    changed
}

/// The "pick in viewport" toggle of a top-level parameter the viewport can fill.
pub(super) fn pick_button(ui: &mut egui::Ui, ctx: &FormCtx<'_>, name: &str, path: &str, kind: PickKind) {
    if name != path {
        return;
    }
    let active = ctx.picking.is_some_and(|(field, _)| field == name);
    let hint = match kind {
        PickKind::Datum { .. } => "form.pick_hint.datum",
        PickKind::Points => "form.pick_hint.points",
        PickKind::Edges => "form.pick_hint.edges",
    };
    let label = t(if active { "form.pick_stop" } else { "form.pick" });
    // a real button (not a text toggle): picking in the viewport is the main way to fill the field
    if ui.add(egui::Button::new(label).selected(active)).on_hover_text(t(hint)).clicked() {
        *ctx.pick_request.borrow_mut() = Some(PickRequest { field: name.to_string(), kind, start: !active });
    }
}

/// What to click, under the parameter being picked.
pub(super) fn pick_hint_line(ui: &mut egui::Ui, ctx: &FormCtx<'_>, name: &str) {
    if let Some((field, hint)) = ctx.picking
        && field == name
    {
        ui.colored_label(ctx.tokens.color(Token::AccentPrimary), hint);
    }
}

fn limits_text(min: f64, max: f64) -> String {
    match (min.is_finite(), max.is_finite()) {
        (true, true) => format!("[{} … {}]", fmt_num(min), fmt_num(max)),
        (true, false) => format!("≥ {}", fmt_num(min)),
        (false, true) => format!("≤ {}", fmt_num(max)),
        _ => String::new(),
    }
}

/// The description of one choice of an enum whose choices are described one by one.
pub(super) fn option_hint(root: &Value, schema: &Value, option: &Value) -> Option<String> {
    let branches = schema.get("oneOf").or_else(|| schema.get("anyOf"))?.as_array()?;
    let is_option = |b: &&Value| {
        let single = b.get("enum").and_then(Value::as_array).filter(|e| e.len() == 1).and_then(|e| e.first());
        b.get("const").or(single) == Some(option)
    };
    let branch = branches.iter().map(|b| resolve(root, b)).find(is_option)?;
    branch.get("description").and_then(Value::as_str).map(i18n::hint)
}

fn enum_label(v: &Value) -> String {
    match v {
        Value::String(s) => find(&format!("enum.{s}")).unwrap_or(s.as_str()).to_string(),
        other => other.to_string(),
    }
}

fn tuple_row(ui: &mut egui::Ui, schema: &Value, value: &mut Value, name: &str, label: Option<egui::Id>) -> bool {
    let mut changed = false;
    let n = schema.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
    let Some(items) = value.as_array_mut() else { return false };
    items.resize(n, json!(0.0));
    let int = schema.get("items").is_some_and(|i| {
        let number = i.get("anyOf").and_then(|a| a.get(0)).unwrap_or(i);
        number.get("type").and_then(Value::as_str) == Some("integer")
    });
    for (i, v) in items.iter_mut().enumerate() {
        let mut x = v.as_f64().unwrap_or(0.0);
        let axis = ["x", "y", "z", "w"].get(i).copied();
        let speed = if int { 1.0 } else { drag_speed(x) };
        let r = ui.add(locale_drag(&mut x, speed, unit_of(name), f64::NEG_INFINITY, f64::INFINITY));
        if let Some(l) = label {
            r.clone().labelled_by(l);
        }
        if let (true, Some(a)) = (n <= 4, axis) {
            r.clone().on_hover_text(a);
        }
        if r.changed() {
            *v = if int { json!(x.round() as i64) } else { number_value(x) };
            changed = true;
        }
    }
    changed
}
