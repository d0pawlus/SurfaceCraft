//! Forms generated from a command's input JSON Schema.
//!
//! The form edits a `serde_json::Value` in place and reports whether it changed: components return
//! edit events, they never execute anything. Every colour is a theme token, every text a catalogue
//! key (parameter names and descriptions come from the schema, with optional catalogue overrides
//! `param.<command>.<name>` / `param.<name>`). Numbers use the locale's decimal separator and show
//! their unit and limits; optional parameters have a "set" check box; object parameters are
//! pickers over the document's objects; invalid values get a message from [`validate`].
use crate::i18n::{self, Lang, find, t, tf};
use crate::theme::{Token, Tokens};
use serde_json::{Map, Number, Value, json};
use std::collections::HashMap;
use surfacecraft_doc::ObjectKind;

/// An object the pickers offer.
#[derive(Clone, Debug, PartialEq)]
pub struct ObjInfo {
    /// Id.
    pub id: u64,
    /// Kind.
    pub kind: ObjectKind,
    /// Name.
    pub name: String,
}

/// What a form knows about the document and the viewport picks.
pub struct FormCtx<'a> {
    /// Command the form is for (catalogue overrides of parameter names).
    pub command: &'a str,
    /// Objects of the document.
    pub objects: &'a [ObjInfo],
    /// Object selected in the object list.
    pub selected: Option<u64>,
    /// Objects of the viewport picks, in pick order.
    pub pick_objects: &'a [u64],
    /// Points picked in the viewport.
    pub pick_points: &'a [[f64; 3]],
    /// Triangles picked in the viewport.
    pub pick_triangles: &'a [u32],
    /// Theme tokens (error colour).
    pub tokens: &'a Tokens,
    /// Validation messages by path (see [`validate`]).
    pub issues: &'a HashMap<String, String>,
}

/// What a schema node is, for the widget to use.
#[derive(Clone, Debug, PartialEq)]
pub enum Kind {
    /// `true`/`false`.
    Bool,
    /// Whole number.
    Int,
    /// Real number.
    Num,
    /// Text.
    Str,
    /// One of these values.
    Enum(Vec<Value>),
    /// Number (id) or string (name) of an object.
    ObjRef,
    /// List.
    Array,
    /// Named fields.
    Object,
    /// Anything else: edited as JSON text.
    Any,
}

/// The schema a `$ref` points at (`#/$defs/Name`), or `schema` itself.
#[must_use]
pub fn resolve<'a>(root: &'a Value, schema: &'a Value) -> &'a Value {
    let mut s = schema;
    for _ in 0..8 {
        let Some(r) = s.get("$ref").and_then(Value::as_str) else { return s };
        let Some(path) = r.strip_prefix("#/") else { return s };
        let mut cur = root;
        for part in path.split('/') {
            match cur.get(part) {
                Some(next) => cur = next,
                None => return s,
            }
        }
        s = cur;
    }
    s
}

fn is_null_schema(s: &Value) -> bool {
    s.get("type").and_then(Value::as_str) == Some("null")
}

fn type_names(s: &Value) -> Vec<&str> {
    match s.get("type") {
        Some(Value::String(x)) => vec![x.as_str()],
        Some(Value::Array(a)) => a.iter().filter_map(Value::as_str).collect(),
        _ => Vec::new(),
    }
}

/// Classifies a schema node: its kind, whether `null` is allowed, and the node that defines it.
#[must_use]
pub fn classify<'a>(root: &'a Value, schema: &'a Value) -> (Kind, bool, &'a Value) {
    classify_depth(root, schema, 0)
}

fn classify_depth<'a>(root: &'a Value, schema: &'a Value, depth: u8) -> (Kind, bool, &'a Value) {
    let s = resolve(root, schema);
    if depth > 8 || !s.is_object() {
        return (Kind::Any, false, s);
    }
    if let Some(e) = s.get("enum").and_then(Value::as_array) {
        let nullable = e.iter().any(Value::is_null);
        return (Kind::Enum(e.iter().filter(|v| !v.is_null()).cloned().collect()), nullable, s);
    }
    if let Some(c) = s.get("const") {
        return (Kind::Enum(vec![c.clone()]), false, s);
    }
    for key in ["oneOf", "anyOf"] {
        let Some(branches) = s.get(key).and_then(Value::as_array) else { continue };
        let nullable = branches.iter().any(|b| is_null_schema(resolve(root, b)));
        let rest: Vec<&Value> = branches.iter().filter(|b| !is_null_schema(resolve(root, b))).collect();
        if rest.len() == 1 {
            let (k, n, inner) = classify_depth(root, rest[0], depth + 1);
            return (k, n || nullable, inner);
        }
        let consts: Vec<Value> = rest
            .iter()
            .filter_map(|b| {
                let b = resolve(root, b);
                b.get("const").cloned().or_else(|| match b.get("enum").and_then(Value::as_array) {
                    Some(e) if e.len() == 1 => e.first().cloned(),
                    _ => None,
                })
            })
            .collect();
        if !rest.is_empty() && consts.len() == rest.len() {
            return (Kind::Enum(consts), nullable, s);
        }
        let types: Vec<&str> = rest.iter().flat_map(|b| type_names(resolve(root, b))).collect();
        if types.contains(&"string") && (types.contains(&"integer") || types.contains(&"number")) && rest.len() == 2 {
            return (Kind::ObjRef, nullable, s);
        }
        return (Kind::Any, nullable, s);
    }
    let all = type_names(s);
    let nullable = all.contains(&"null");
    let types: Vec<&str> = all.into_iter().filter(|t| *t != "null").collect();
    let kind = match types.as_slice() {
        ["boolean"] => Kind::Bool,
        ["integer"] => Kind::Int,
        ["number"] => Kind::Num,
        ["string"] => Kind::Str,
        ["array"] => Kind::Array,
        ["object"] if s.get("properties").and_then(Value::as_object).is_some_and(|p| !p.is_empty()) => Kind::Object,
        _ => Kind::Any,
    };
    (kind, nullable, s)
}

fn limit(s: &Value, key: &str) -> Option<f64> {
    s.get(key).and_then(Value::as_f64)
}

/// A neutral value for a schema node (what a newly set optional parameter starts with).
#[must_use]
pub fn default_value(root: &Value, schema: &Value, ctx: Option<&FormCtx<'_>>) -> Value {
    let (kind, _, s) = classify(root, schema);
    if let Some(d) = s.get("default").or_else(|| resolve(root, schema).get("default"))
        && !d.is_null()
    {
        return d.clone();
    }
    match kind {
        Kind::Bool => json!(false),
        Kind::Int => json!(limit(s, "minimum").filter(|m| *m > 0.0).map_or(0, |m| m as i64)),
        Kind::Num => json!(limit(s, "minimum").filter(|m| *m > 0.0).unwrap_or(0.0)),
        Kind::Str => json!(""),
        Kind::Enum(v) => v.first().cloned().unwrap_or(Value::Null),
        Kind::ObjRef => json!(ctx.and_then(|c| c.selected).unwrap_or(0)),
        Kind::Array => {
            let n = s.get("minItems").and_then(Value::as_u64).unwrap_or(0).min(64) as usize;
            let item = s.get("items").map_or(json!(0), |i| default_value(root, i, ctx));
            Value::Array(vec![item; n])
        }
        Kind::Object => {
            let mut m = Map::new();
            let required = required_of(s);
            if let Some(props) = s.get("properties").and_then(Value::as_object) {
                for name in required {
                    if let Some(p) = props.get(name) {
                        m.insert(name.to_string(), default_value(root, p, ctx));
                    }
                }
            }
            Value::Object(m)
        }
        Kind::Any => Value::Null,
    }
}

fn required_of(s: &Value) -> Vec<&str> {
    s.get("required")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default()
}

// ---------------------------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------------------------

/// A validation message for the value at `path` (`a.b[0]`).
#[derive(Clone, Debug, PartialEq)]
pub struct Issue {
    /// Where.
    pub path: String,
    /// What is wrong, in the current language.
    pub message: String,
}

/// Checks `value` against `schema`: types, required parameters, limits, enum members, list
/// lengths and that object references exist. The engine stays the authority (it validates again
/// when the command runs); this gives the messages next to the fields before that.
#[must_use]
pub fn validate(root: &Value, value: &Value, objects: &[ObjInfo]) -> Vec<Issue> {
    let mut out = Vec::new();
    check(root, root, value, "", objects, &mut out);
    out
}

fn type_label(kind: &Kind) -> &'static str {
    match kind {
        Kind::Bool => "form.type.bool",
        Kind::Int => "form.type.int",
        Kind::Num => "form.type.number",
        Kind::Str => "form.type.text",
        Kind::Array => "form.type.list",
        Kind::Object => "form.type.object",
        Kind::ObjRef => "form.type.object_ref",
        Kind::Enum(_) | Kind::Any => "form.type.value",
    }
}

fn fmt_num(n: f64) -> String {
    let s = format!("{n:.6}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    let s = if s.is_empty() || s == "-" { "0" } else { s };
    if i18n::lang() == Lang::Pl { s.replace('.', ",") } else { s.to_string() }
}

fn check(root: &Value, schema: &Value, value: &Value, path: &str, objects: &[ObjInfo], out: &mut Vec<Issue>) {
    let (kind, nullable, s) = classify(root, schema);
    if value.is_null() {
        if !nullable && kind != Kind::Any {
            issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))]));
        }
        return;
    }
    match &kind {
        Kind::Bool => {
            if !value.is_boolean() {
                issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))]));
            }
        }
        Kind::Int | Kind::Num => {
            let ok = if kind == Kind::Int { value.is_i64() || value.is_u64() } else { value.is_number() };
            if !ok {
                issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))]));
            } else if let Some(x) = value.as_f64() {
                if let Some(min) = limit(s, "minimum")
                    && x < min
                {
                    issue(out, path, tf("form.err.min", &[("min", &fmt_num(min))]));
                } else if let Some(max) = limit(s, "maximum")
                    && x > max
                {
                    issue(out, path, tf("form.err.max", &[("max", &fmt_num(max))]));
                } else if let Some(min) = limit(s, "exclusiveMinimum")
                    && x <= min
                {
                    issue(out, path, tf("form.err.above", &[("min", &fmt_num(min))]));
                }
            }
        }
        Kind::Str => {
            if !value.is_string() {
                issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))]));
            }
        }
        Kind::Enum(options) => {
            if !options.contains(value) {
                issue(out, path, t("form.err.not_allowed").to_string());
            }
        }
        Kind::ObjRef => match value {
            Value::Number(n) => {
                if !n.as_u64().is_some_and(|id| objects.iter().any(|o| o.id == id)) {
                    issue(out, path, tf("form.err.no_object", &[("value", &n.to_string())]));
                }
            }
            Value::String(name) => {
                if !objects.iter().any(|o| &o.name == name) {
                    issue(out, path, tf("form.err.no_object", &[("value", name)]));
                }
            }
            _ => issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))])),
        },
        Kind::Array => match value.as_array() {
            None => issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))])),
            Some(items) => {
                if let Some(min) = s.get("minItems").and_then(Value::as_u64)
                    && (items.len() as u64) < min
                {
                    issue(out, path, tf("form.err.items_min", &[("n", &min.to_string())]));
                }
                if let Some(max) = s.get("maxItems").and_then(Value::as_u64)
                    && (items.len() as u64) > max
                {
                    issue(out, path, tf("form.err.items_max", &[("n", &max.to_string())]));
                }
                if let Some(item_schema) = s.get("items") {
                    for (i, v) in items.iter().enumerate() {
                        check(root, item_schema, v, &format!("{path}[{i}]"), objects, out);
                    }
                }
            }
        },
        Kind::Object => match value.as_object() {
            None => issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))])),
            Some(map) => {
                let props = s.get("properties").and_then(Value::as_object);
                for name in required_of(s) {
                    if map.get(name).is_none_or(Value::is_null) {
                        let nullable_ok = props
                            .and_then(|p| p.get(name))
                            .is_some_and(|p| classify(root, p).1 && map.contains_key(name));
                        if !nullable_ok {
                            out.push(Issue { path: join(path, name), message: t("form.err.required").to_string() });
                        }
                    }
                }
                for (k, v) in map {
                    match props.and_then(|p| p.get(k)) {
                        Some(ps) => check(root, ps, v, &join(path, k), objects, out),
                        None if s.get("additionalProperties") == Some(&Value::Bool(false)) => {
                            out.push(Issue { path: join(path, k), message: t("form.err.unknown").to_string() });
                        }
                        None => {}
                    }
                }
            }
        },
        Kind::Any => {}
    }
}

fn issue(out: &mut Vec<Issue>, path: &str, message: String) {
    out.push(Issue { path: path.to_string(), message });
}

fn join(path: &str, name: &str) -> String {
    if path.is_empty() { name.to_string() } else { format!("{path}.{name}") }
}

// ---------------------------------------------------------------------------------------------
// Widgets
// ---------------------------------------------------------------------------------------------

/// The unit suffix a numeric parameter gets from its name.
#[must_use]
pub fn unit_of(name: &str) -> &'static str {
    if name.ends_with("_deg") || name == "angle" {
        " °"
    } else if [
        "radius",
        "distance",
        "size",
        "thickness",
        "tolerance",
        "length",
        "width",
        "height",
        "offset",
        "diameter",
    ]
    .iter()
    .any(|k| name.contains(k))
    {
        " mm"
    } else {
        ""
    }
}

/// The kind of object a parameter name hints at.
fn kind_hint(name: &str) -> Option<ObjectKind> {
    let n = name.to_ascii_lowercase();
    let table: [(&str, ObjectKind); 8] = [
        ("sketch", ObjectKind::Sketch),
        ("mesh", ObjectKind::Mesh),
        ("region", ObjectKind::Region),
        ("primitive", ObjectKind::Primitive),
        ("surface", ObjectKind::Surface),
        ("quad", ObjectKind::Quad),
        ("feature", ObjectKind::Feature),
        ("body", ObjectKind::Body),
    ];
    if n == "target" {
        return Some(ObjectKind::Body);
    }
    table.iter().find(|(p, _)| n.starts_with(p)).map(|(_, k)| *k)
}

fn kind_name_key(kind: ObjectKind) -> &'static str {
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

/// The label of parameter `name`: a catalogue override or the schema's own name.
fn label_of(ctx: &FormCtx<'_>, name: &str) -> String {
    let specific = format!("param.{}.{name}", ctx.command);
    let generic = format!("param.{name}");
    find(&specific).or_else(|| find(&generic)).unwrap_or(name).to_string()
}

fn description_of(root: &Value, schema: &Value) -> Option<String> {
    schema
        .get("description")
        .or_else(|| resolve(root, schema).get("description"))
        .and_then(Value::as_str)
        .map(str::to_string)
}

fn number_value(x: f64) -> Value {
    Number::from_f64(x).map_or(Value::Null, Value::Number)
}

fn drag_speed(v: f64) -> f64 {
    (v.abs() * 0.01).max(0.01)
}

fn locale_drag<'a>(x: &'a mut f64, speed: f64, unit: &'static str, min: f64, max: f64) -> egui::DragValue<'a> {
    egui::DragValue::new(x)
        .speed(speed)
        .range(min..=max)
        .clamp_existing_to_range(false)
        .suffix(unit)
        .custom_formatter(|n, _| fmt_num(n))
        .custom_parser(|s| s.trim().replace(',', ".").replace('\u{2212}', "-").parse::<f64>().ok())
}

fn error_line(ui: &mut egui::Ui, ctx: &FormCtx<'_>, path: &str) {
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
        changed |= object_rows(ui, root, s, map, "", ctx);
    }
    changed
}

fn object_rows(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    map: &mut Map<String, Value>,
    path: &str,
    ctx: &FormCtx<'_>,
) -> bool {
    let mut changed = false;
    let Some(props) = schema.get("properties").and_then(Value::as_object) else { return false };
    let required = required_of(schema);
    let mut names: Vec<&String> = props.keys().collect();
    names.sort_by_key(|n| !required.contains(&n.as_str()));
    if names.is_empty() {
        ui.weak(t("form.no_parameters"));
    }
    for name in names {
        let Some(prop) = props.get(name) else { continue };
        changed |= property_row(ui, root, name, prop, required.contains(&name.as_str()), map, path, ctx);
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
    let hover = description_of(root, prop);
    let mut changed = false;
    let set = map.get(name).is_some_and(|v| !v.is_null());
    let compound = matches!(kind, Kind::Object) || (kind == Kind::Array && !is_tuple(schema, root));
    ui.push_id(&path, |ui| {
        if compound && set {
            let header = egui::CollapsingHeader::new(marked(&label, required)).id_salt(&path).default_open(true);
            let resp = header.show(ui, |ui| {
                if !required {
                    changed |= presence(ui, map, name, false, prop, root, ctx);
                }
                if let Some(v) = map.get_mut(name) {
                    changed |= value_widget(ui, root, prop, v, name, &path, ctx);
                }
            });
            if let Some(h) = &hover {
                resp.header_response.on_hover_text(h);
            }
            error_line(ui, ctx, &path);
            return;
        }
        ui.horizontal(|ui| {
            let name_resp = if required {
                ui.label(marked(&label, true))
            } else {
                // the check box is the label of an optional parameter
                let mut on = set;
                let r = ui.checkbox(&mut on, &label);
                if r.changed() {
                    if on {
                        map.insert(name.to_string(), default_value(root, prop, Some(ctx)));
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
                let resp = value_widget_labelled(ui, root, prop, v, name, &path, ctx, Some(&name_resp));
                changed |= resp;
            } else if required {
                // a required parameter without a value: give it one
                let d = default_value(root, prop, Some(ctx));
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

fn marked(label: &str, required: bool) -> String {
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

fn is_tuple(schema: &Value, root: &Value) -> bool {
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

fn value_widget(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
) -> bool {
    value_widget_labelled(ui, root, schema, value, name, path, ctx, None)
}

#[allow(clippy::too_many_arguments)]
fn value_widget_labelled(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<&egui::Response>,
) -> bool {
    let (kind, _, s) = classify(root, schema);
    let mut changed = false;
    let tag = |r: egui::Response| -> egui::Response {
        match label {
            Some(l) => r.labelled_by(l.id),
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
                    if ui.selectable_label(o == value, enum_label(o)).clicked() {
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
        Kind::ObjRef => changed |= object_picker(ui, value, name, path, ctx, label),
        Kind::Array => {
            if is_tuple(schema, root) {
                changed |= tuple_row(ui, s, value, name, tag_label(label));
            } else {
                changed |= list_widget(ui, root, s, value, name, path, ctx);
            }
        }
        Kind::Object => {
            if let Some(map) = value.as_object_mut() {
                changed |= object_rows(ui, root, s, map, path, ctx);
            }
        }
        Kind::Any => changed |= json_text(ui, value, path),
    }
    changed
}

fn tag_label(l: Option<&egui::Response>) -> Option<egui::Id> {
    l.map(|r| r.id)
}

fn limits_text(min: f64, max: f64) -> String {
    match (min.is_finite(), max.is_finite()) {
        (true, true) => format!("[{} … {}]", fmt_num(min), fmt_num(max)),
        (true, false) => format!("≥ {}", fmt_num(min)),
        (false, true) => format!("≤ {}", fmt_num(max)),
        _ => String::new(),
    }
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
    let int = schema.get("items").and_then(|i| i.get("type")).and_then(Value::as_str) == Some("integer");
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

fn list_widget(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
) -> bool {
    let mut changed = false;
    let item_schema = schema.get("items").cloned().unwrap_or(Value::Bool(true));
    let (item_kind, _, _) = classify(root, &item_schema);
    let max = schema.get("maxItems").and_then(Value::as_u64).unwrap_or(u64::MAX) as usize;
    let min = schema.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
    let Some(items) = value.as_array_mut() else { return false };
    let items_len_over_min = items.len() > min;
    let mut remove = None;
    for (i, v) in items.iter_mut().enumerate() {
        ui.horizontal(|ui| {
            let ipath = format!("{path}[{i}]");
            ui.label(format!("{}", i + 1));
            changed |= value_widget(ui, root, &item_schema, v, name, &ipath, ctx);
            if items_len_over_min && ui.small_button(t("form.remove")).clicked() {
                remove = Some(i);
            }
        });
        error_line(ui, ctx, &format!("{path}[{i}]"));
    }
    if let Some(i) = remove {
        items.remove(i);
        changed = true;
    }
    ui.horizontal(|ui| {
        if items.len() < max && ui.small_button(t("form.add")).clicked() {
            items.push(default_value(root, &item_schema, Some(ctx)));
            changed = true;
        }
        let is_point = is_tuple(&item_schema, root) && item_schema.get("minItems").and_then(Value::as_u64) == Some(3);
        if is_point && !ctx.pick_points.is_empty() && ui.small_button(t("form.use_picked_points")).clicked() {
            *items = ctx.pick_points.iter().map(|p| json!(p)).collect();
            changed = true;
        }
        if item_kind == Kind::Int
            && name.contains("triangle")
            && !ctx.pick_triangles.is_empty()
            && ui.small_button(t("form.use_picked_triangles")).clicked()
        {
            *items = ctx.pick_triangles.iter().map(|x| json!(x)).collect();
            changed = true;
        }
        if item_kind == Kind::ObjRef
            && !ctx.pick_objects.is_empty()
            && ui.small_button(t("form.use_picked_objects")).clicked()
        {
            let mut ids: Vec<u64> = Vec::new();
            for id in ctx.pick_objects {
                if !ids.contains(id) {
                    ids.push(*id);
                }
            }
            *items = ids.into_iter().map(|id| json!(id)).collect();
            changed = true;
        }
    });
    changed
}

fn object_picker(
    ui: &mut egui::Ui,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<&egui::Response>,
) -> bool {
    let mut changed = false;
    let hint = kind_hint(name);
    let current_id = value.as_u64();
    let describe = |o: &ObjInfo| format!("{} {} (#{})", t(kind_name_key(o.kind)), o.name, o.id);
    let shown = match value {
        Value::Number(n) => {
            n.as_u64().and_then(|id| ctx.objects.iter().find(|o| o.id == id)).map_or_else(|| format!("#{n}"), describe)
        }
        Value::String(s) => s.clone(),
        _ => t("form.none").to_string(),
    };
    let candidates: Vec<&ObjInfo> =
        ctx.objects.iter().filter(|o| hint.is_none_or(|h| o.kind == h) || Some(o.id) == current_id).collect();
    let candidates: Vec<&ObjInfo> = if candidates.is_empty() { ctx.objects.iter().collect() } else { candidates };
    let mut chosen: Option<u64> = None;
    let resp = egui::ComboBox::from_id_salt(path).selected_text(shown).width(180.0).show_ui(ui, |ui| {
        if candidates.is_empty() {
            ui.weak(t("form.no_objects"));
        }
        for o in &candidates {
            if ui.selectable_label(current_id == Some(o.id), describe(o)).clicked() {
                chosen = Some(o.id);
            }
        }
    });
    if let Some(l) = label {
        resp.response.labelled_by(l.id);
    }
    if let Some(sel) = ctx.selected
        && ui.small_button(t("form.use_selection")).on_hover_text(t("form.use_selection_hint")).clicked()
    {
        chosen = Some(sel);
    }
    if let Some(p) = ctx.pick_objects.last()
        && ui.small_button(t("form.use_pick")).on_hover_text(t("form.use_pick_hint")).clicked()
    {
        chosen = Some(*p);
    }
    if let Some(id) = chosen
        && current_id != Some(id)
    {
        *value = json!(id);
        changed = true;
    }
    changed
}

/// Free-form JSON of one value (parameters whose schema has no structure).
fn json_text(ui: &mut egui::Ui, value: &mut Value, path: &str) -> bool {
    let id = ui.scope_id().with(("form-json", path));
    let mut text: String = ui.data_mut(|d| d.get_temp(id)).unwrap_or_else(|| value.to_string());
    let mut changed = false;
    let r = ui.add(egui::TextEdit::singleline(&mut text).code_editor().desired_width(160.0));
    if r.changed() {
        if let Ok(v) = serde_json::from_str::<Value>(&text) {
            *value = v;
            changed = true;
        }
        ui.data_mut(|d| d.insert_temp(id, text.clone()));
    } else if !r.has_focus() {
        ui.data_mut(|d| d.remove::<String>(id));
    }
    if serde_json::from_str::<Value>(&text).is_err() {
        ui.label(t("form.err.json"));
    }
    changed
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;
    use surfacecraft_engine::cmd;

    fn schema_of(id: &str) -> Value {
        (cmd::find(id).map(|c| c.input)).map_or(Value::Null, |f| f())
    }

    #[test]
    fn classifies_the_schemas_of_every_command_without_panicking() {
        for c in cmd::registry() {
            let s = (c.input)();
            let (k, _, inner) = classify(&s, &s);
            assert!(matches!(k, Kind::Object | Kind::Any), "{} root is {k:?}", c.id);
            let v = default_value(&s, inner, None);
            // required parameters get a value of the right type
            let issues = validate(&s, &v, &[]);
            let bad: Vec<_> = issues.iter().filter(|i| !i.message.contains('#') && i.path.is_empty()).collect();
            assert!(bad.is_empty(), "{}: {bad:?}", c.id);
        }
    }

    #[test]
    fn extrude_schema_has_an_object_reference_a_number_and_an_enum() {
        let s = schema_of("feature.extrude");
        let props = s["properties"].as_object().unwrap();
        assert_eq!(classify(&s, &props["sketch"]).0, Kind::ObjRef);
        assert_eq!(classify(&s, &props["distance"]).0, Kind::Num);
        assert!(matches!(classify(&s, &props["mode"]).0, Kind::Enum(v) if v.len() == 3));
        let (k, nullable, _) = classify(&s, &props["target"]);
        assert_eq!((k, nullable), (Kind::ObjRef, true));
    }

    #[test]
    fn validation_reports_missing_and_out_of_range_values() {
        let s = schema_of("mesh.synth");
        let issues = validate(&s, &json!({"shape": "sphere", "resolution": -3}), &[]);
        assert!(issues.iter().any(|i| i.path == "resolution"), "{issues:?}");
        let issues = validate(&s, &json!({}), &[]);
        assert!(issues.iter().any(|i| i.path == "shape"), "{issues:?}");
    }
}
