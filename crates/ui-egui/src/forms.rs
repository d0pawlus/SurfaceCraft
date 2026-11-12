//! Forms generated from a command's input JSON Schema.
//!
//! The form edits a `serde_json::Value` in place and reports whether it changed: components return
//! edit events, they never execute anything. Every colour is a theme token, every text a catalogue
//! key (parameter names and descriptions come from the schema, with optional catalogue overrides
//! `param.<command>.<name>` / `param.<name>`). Numbers use the locale's decimal separator and show
//! their unit and limits; optional parameters have a "set" check box; object parameters are
//! pickers over the document's objects; invalid values get a message from [`validate`].
use crate::fieldpick::{PickKind, PickMode, PickRequest};
use crate::i18n::{self, Lang, find, t, tf};
use crate::theme::{Token, Tokens};
use serde_json::{Map, Number, Value, json};
use std::cell::RefCell;
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
    /// The parameter the viewport is picking into right now, with the hint to show next to it.
    pub picking: Option<(&'a str, &'a str)>,
    /// Where a "pick in viewport" button leaves its request (the caller acts on it after the
    /// form: the form itself never starts anything).
    pub pick_request: &'a RefCell<Option<PickRequest>>,
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

/// A number or integer schema marked `x-expr`: the first branch of a number-or-expression union.
fn is_expr_number(s: &Value) -> bool {
    s.get("x-expr").is_some() && matches!(type_names(s).as_slice(), ["number"] | ["integer"])
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
        // a number that may also be given as an expression object: the form edits the number
        if let Some(first) = rest.first().map(|b| resolve(root, b)).filter(|b| is_expr_number(b)) {
            return (if type_names(first) == ["integer"] { Kind::Int } else { Kind::Num }, nullable, first);
        }
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
        // one of several shapes (a sketch entity, a constraint): the first of them
        Kind::Any => variants(root, s)
            .and_then(|v| v.first().map(|first| default_value(root, first.schema, ctx)))
            .unwrap_or(Value::Null),
    }
}

/// One alternative of a parameter that is one of several shapes (`oneOf`): a sketch entity is a
/// line, an arc, a circle or a polygon; a constraint is one of fifteen kinds.
#[derive(Clone, Debug, PartialEq)]
pub struct Variant<'a> {
    /// The name that tells the alternatives apart (`line`, `fix`).
    pub tag: &'a str,
    /// The schema of the alternative.
    pub schema: &'a Value,
    /// The property that holds the tag next to the fields (`{"type": "fix", "point": 0}`), or
    /// `None` when the tag is the one property that wraps them (`{"line": {"from": [0, 0]}}`).
    pub tag_field: Option<&'a str>,
}

impl Variant<'_> {
    /// Whether `value` is of this alternative.
    #[must_use]
    pub fn holds(&self, value: &Value) -> bool {
        match self.tag_field {
            Some(field) => value.get(field).and_then(Value::as_str) == Some(self.tag),
            None => value.get(self.tag).is_some(),
        }
    }
}

/// The alternatives of a `oneOf`/`anyOf` node when every one of them is a tagged object; `None`
/// for anything else (which stays JSON text).
#[must_use]
pub fn variants<'a>(root: &'a Value, schema: &'a Value) -> Option<Vec<Variant<'a>>> {
    let branches = schema.get("oneOf").or_else(|| schema.get("anyOf"))?.as_array()?;
    let found: Vec<Variant<'a>> = branches.iter().filter_map(|b| variant_of(root, b)).collect();
    (found.len() == branches.len() && found.len() > 1).then_some(found)
}

fn variant_of<'a>(root: &'a Value, branch: &'a Value) -> Option<Variant<'a>> {
    let schema = resolve(root, branch);
    let props = schema.get("properties")?.as_object()?;
    let tagged = props.iter().find_map(|(field, p)| Some((field.as_str(), resolve(root, p).get("const")?.as_str()?)));
    if let Some((field, tag)) = tagged {
        return Some(Variant { tag, schema, tag_field: Some(field) });
    }
    let (tag, _) = props.iter().next().filter(|_| props.len() == 1)?;
    Some(Variant { tag, schema, tag_field: None })
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
        Kind::ObjRef => {
            let found = match value {
                Value::Number(n) => n.as_u64().and_then(|id| objects.iter().find(|o| o.id == id)),
                Value::String(name) => objects.iter().find(|o| &o.name == name),
                _ => {
                    issue(out, path, tf("form.err.type", &[("expected", t(type_label(&kind)))]));
                    return;
                }
            };
            let kinds = accepted_kinds(leaf_name(path), description_of(root, schema).as_deref());
            match found {
                None => {
                    let shown = value.as_str().map_or_else(|| value.to_string(), str::to_string);
                    issue(out, path, tf("form.err.no_object", &[("value", &shown)]));
                }
                Some(o) if !kinds.is_empty() && !kinds.contains(&o.kind) => {
                    issue(out, path, tf("form.err.wrong_kind", &[("kinds", &kind_names(&kinds))]));
                }
                Some(_) => {}
            }
        }
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

/// The kinds of object a reference parameter takes: the one its name says (`mesh`, `region`,
/// `target`), else the ones its description names ("Scan mesh.", "Reference: a primitive, a mesh
/// or a body."). Empty when neither says: any object.
#[must_use]
pub fn accepted_kinds(name: &str, description: Option<&str>) -> Vec<ObjectKind> {
    const TABLE: [(&str, &str, ObjectKind); 8] = [
        ("sketch", "sketches", ObjectKind::Sketch),
        ("mesh", "meshes", ObjectKind::Mesh),
        ("region", "regions", ObjectKind::Region),
        ("primitive", "primitives", ObjectKind::Primitive),
        ("surface", "surfaces", ObjectKind::Surface),
        ("quad", "quads", ObjectKind::Quad),
        ("feature", "features", ObjectKind::Feature),
        ("body", "bodies", ObjectKind::Body),
    ];
    let n = name.to_ascii_lowercase();
    if n == "target" {
        return vec![ObjectKind::Body];
    }
    if let Some((_, _, kind)) = TABLE.iter().find(|(one, _, _)| n.starts_with(one)) {
        return vec![*kind];
    }
    let text = description.unwrap_or_default().to_ascii_lowercase();
    let words: Vec<&str> = text.split(|c: char| !c.is_ascii_alphabetic()).filter(|w| !w.is_empty()).collect();
    TABLE.iter().filter(|(one, many, _)| words.contains(one) || words.contains(many)).map(|(_, _, k)| *k).collect()
}

/// The object a reference parameter starts with: the selected one when it is of a kind the
/// parameter takes, else the first object that is.
#[must_use]
pub fn preferred_object(objects: &[ObjInfo], selected: Option<u64>, kinds: &[ObjectKind]) -> Option<u64> {
    let fits = |o: &&ObjInfo| kinds.is_empty() || kinds.contains(&o.kind);
    let selected = selected.and_then(|id| objects.iter().find(|o| o.id == id)).filter(fits);
    selected.or_else(|| objects.iter().find(fits)).map(|o| o.id)
}

/// The kinds of `kinds` as words ("mesh, body"), for a message.
fn kind_names(kinds: &[ObjectKind]) -> String {
    kinds.iter().map(|k| t(kind_name_key(*k))).collect::<Vec<_>>().join(", ")
}

/// The last name of a value path (`a.b[0]` is `b`).
fn leaf_name(path: &str) -> &str {
    let last = path.rsplit('.').next().unwrap_or(path);
    last.split('[').next().unwrap_or(last)
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

/// The label of parameter `name`: the catalogue's (`param.<command>.<name>`, then
/// `param.<name>`), else the schema's name in plain words.
fn label_of(ctx: &FormCtx<'_>, name: &str) -> String {
    let specific = format!("param.{}.{name}", ctx.command);
    let generic = format!("param.{name}");
    find(&specific).or_else(|| find(&generic)).map_or_else(|| humanize(name), str::to_string)
}

/// A schema name in plain words: `max_distance` is "Max distance", `draft_deg` is "Draft" (the
/// field shows the unit).
#[must_use]
pub fn humanize(name: &str) -> String {
    let base = name.strip_suffix("_deg").or_else(|| name.strip_suffix("_mm")).unwrap_or(name);
    let words = base.replace('_', " ");
    let mut chars = words.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
        None => name.to_string(),
    }
}

fn description_of(root: &Value, schema: &Value) -> Option<String> {
    schema
        .get("description")
        .or_else(|| resolve(root, schema).get("description"))
        .and_then(Value::as_str)
        .map(str::to_string)
}

/// What the tooltip of a parameter says: its description in the interface language.
fn hover_of(root: &Value, schema: &Value) -> Option<String> {
    description_of(root, schema).map(|d| i18n::hint(&d))
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
        changed |= object_rows(ui, root, s, map, "", ctx, None);
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
fn start_value(root: &Value, prop: &Value, name: &str, kind: &Kind, ctx: &FormCtx<'_>) -> Value {
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

#[allow(clippy::too_many_arguments)]
fn value_widget(
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
fn pick_button(ui: &mut egui::Ui, ctx: &FormCtx<'_>, name: &str, path: &str, kind: PickKind) {
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
fn pick_hint_line(ui: &mut egui::Ui, ctx: &FormCtx<'_>, name: &str) {
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
fn option_hint(root: &Value, schema: &Value, option: &Value) -> Option<String> {
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

#[allow(clippy::too_many_arguments)]
fn list_widget(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<egui::Id>,
) -> bool {
    let mut changed = false;
    let item_schema = schema.get("items").cloned().unwrap_or(Value::Bool(true));
    let (item_kind, _, _) = classify(root, &item_schema);
    let max = schema.get("maxItems").and_then(Value::as_u64).unwrap_or(u64::MAX) as usize;
    let min = schema.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
    let item_kinds = accepted_kinds(name, schema.get("description").and_then(Value::as_str));
    let Some(items) = value.as_array_mut() else { return false };
    let items_len_over_min = items.len() > min;
    let mut remove = None;
    for (i, v) in items.iter_mut().enumerate() {
        // top-aligned: an item of several rows keeps its number and Remove on its first row
        ui.horizontal_top(|ui| {
            let ipath = format!("{path}[{i}]");
            ui.label(format!("{}", i + 1));
            if item_kind == Kind::ObjRef {
                changed |= object_picker(ui, v, &item_kinds, &ipath, ctx, label);
            } else {
                changed |= value_widget(ui, root, &item_schema, v, name, &ipath, ctx, label);
            }
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
            items.push(start_value(root, &item_schema, name, &item_kind, ctx));
            changed = true;
        }
        let is_point = is_tuple(&item_schema, root) && item_schema.get("minItems").and_then(Value::as_u64) == Some(3);
        // what the viewport fills is said by the schema's marker, not by the parameter's name
        let mode = PickMode::of(schema).map(PickMode::kind);
        if mode == Some(PickKind::Points) && is_point {
            pick_button(ui, ctx, name, path, PickKind::Points);
        }
        if mode == Some(PickKind::Edges) && item_kind == Kind::Int {
            pick_button(ui, ctx, name, path, PickKind::Edges);
        }
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
    pick_hint_line(ui, ctx, name);
    changed
}

fn object_picker(
    ui: &mut egui::Ui,
    value: &mut Value,
    kinds: &[ObjectKind],
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<egui::Id>,
) -> bool {
    let mut changed = false;
    let fits = |id: u64| ctx.objects.iter().any(|o| o.id == id && (kinds.is_empty() || kinds.contains(&o.kind)));
    let current_id = value.as_u64();
    let describe = |o: &ObjInfo| format!("{} {} (#{})", t(kind_name_key(o.kind)), o.name, o.id);
    let shown = match value {
        Value::Number(n) => {
            n.as_u64().and_then(|id| ctx.objects.iter().find(|o| o.id == id)).map_or_else(|| format!("#{n}"), describe)
        }
        Value::String(s) => s.clone(),
        _ => t("form.none").to_string(),
    };
    // only objects of a kind the parameter takes (and the current one, so a wrong value shows)
    let candidates: Vec<&ObjInfo> = ctx.objects.iter().filter(|o| fits(o.id) || Some(o.id) == current_id).collect();
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
        resp.response.labelled_by(l);
    }
    // the shortcuts are offered only when they would change the field to an object that fits
    let offer = |id: u64| current_id != Some(id) && fits(id);
    if let Some(sel) = ctx.selected.filter(|id| offer(*id))
        && ui.small_button(t("form.use_selection")).on_hover_text(t("form.use_selection_hint")).clicked()
    {
        chosen = Some(sel);
    }
    if let Some(p) = ctx.pick_objects.last().copied().filter(|id| offer(*id) && ctx.selected != Some(*id))
        && ui.small_button(t("form.use_pick")).on_hover_text(t("form.use_pick_hint")).clicked()
    {
        chosen = Some(p);
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
fn json_text(ui: &mut egui::Ui, value: &mut Value, path: &str, label: Option<egui::Id>) -> bool {
    let id = ui.scope_id().with(("form-json", path));
    let mut text: String = ui.data_mut(|d| d.get_temp(id)).unwrap_or_else(|| value.to_string());
    let mut changed = false;
    let r = ui.add(egui::TextEdit::singleline(&mut text).code_editor().desired_width(160.0));
    if let Some(l) = label {
        r.clone().labelled_by(l);
    }
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
    fn a_number_or_expression_union_is_edited_as_a_number() {
        let number = json!({"type": "number", "x-expr": true});
        let expr = json!({"type": "object", "properties": {"expr": {"type": "string"}}, "required": ["expr"]});
        let root = json!({});
        let union = json!({"anyOf": [number, expr]});
        assert_eq!(classify(&root, &union).0, Kind::Num);
        let int = json!({"anyOf": [{"type": "integer", "x-expr": true}, expr]});
        assert_eq!(classify(&root, &int).0, Kind::Int);
        let optional = json!({"anyOf": [union, {"type": "null"}]});
        let (kind, nullable, _) = classify(&root, &optional);
        assert!(kind == Kind::Num && nullable);
        // a union of other things is still not a number
        let other = json!({"anyOf": [{"type": "number"}, {"type": "object"}]});
        assert_eq!(classify(&root, &other).0, Kind::Any);
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
    fn an_object_parameter_takes_only_the_kinds_its_name_or_description_names() {
        use ObjectKind::{Body, Mesh, Primitive, Region, Surface};
        assert_eq!(accepted_kinds("mesh", None), vec![Mesh]);
        assert_eq!(accepted_kinds("regions", Some("Regions of the mesh.")), vec![Region]);
        assert_eq!(accepted_kinds("object", Some("Mesh id or name.")), vec![Mesh]);
        assert_eq!(
            accepted_kinds("against", Some("Reference: a primitive, a mesh or a body.")),
            vec![Mesh, Primitive, Body]
        );
        assert_eq!(accepted_kinds("object", Some("Surface (or sheet body) to thicken.")), vec![Surface, Body]);
        assert!(accepted_kinds("object", Some("Object id or name.")).is_empty(), "any object");
        // a mesh tool: a region is refused before the command runs, the mesh is fine
        let objects =
            [ObjInfo { id: 1, kind: Mesh, name: "scan".into() }, ObjInfo { id: 2, kind: Region, name: "top".into() }];
        let s = schema_of("mesh.decimate");
        assert!(validate(&s, &json!({"object": 2}), &objects).iter().any(|i| i.path == "object"));
        assert!(validate(&s, &json!({"object": 1}), &objects).iter().all(|i| i.path != "object"));
        // the field starts with the selection only when it fits, else with the first object that does
        assert_eq!(preferred_object(&objects, Some(2), &[Mesh]), Some(1));
        assert_eq!(preferred_object(&objects, Some(2), &[]), Some(2));
        assert_eq!(preferred_object(&objects, Some(2), &[Body]), None);
    }

    #[test]
    fn a_sketch_entity_and_a_constraint_are_alternatives_with_fields() {
        let s = schema_of("sketch.add");
        let item = &s["properties"]["entities"]["items"];
        let (kind, _, node) = classify(&s, item);
        assert_eq!(kind, Kind::Any);
        let vars = variants(&s, node).expect("entities are alternatives");
        assert_eq!(vars.iter().map(|v| v.tag).collect::<Vec<_>>(), ["line", "arc", "circle", "polygon"]);
        // a new entity is a whole line, ready to edit as fields
        let line = default_value(&s, item, None);
        assert_eq!(line, json!({"line": {"from": [0.0, 0.0], "to": [0.0, 0.0]}}));
        assert!(vars[0].holds(&line) && !vars[2].holds(&line));
        let circle = default_value(&s, vars[2].schema, None);
        assert_eq!(circle, json!({"circle": {"center": [0.0, 0.0], "radius": 0.0}}));
        // constraints carry their kind in a `type` field next to the others
        let s = schema_of("sketch.constrain");
        let item = &s["properties"]["constraints"]["items"];
        let vars = variants(&s, classify(&s, item).2).expect("constraints are alternatives");
        assert_eq!(vars.len(), 15);
        assert!(vars.iter().all(|v| v.tag_field == Some("type")));
        let fix = default_value(&s, item, None);
        assert_eq!(fix["type"], "fix");
        assert!(fix.get("point").is_some() && fix.get("at").is_some(), "{fix}");
        // every alternative has a name in both languages
        for v in &vars {
            for lang in Lang::ALL {
                assert!(i18n::catalog(lang).contains_key(&format!("variant.{}", v.tag)), "{lang:?} variant.{}", v.tag);
            }
        }
        // the choices of an enum say what they mean
        let s = schema_of("feature.boolean");
        let (kind, _, node) = classify(&s, &s["properties"]["op"]);
        assert!(matches!(kind, Kind::Enum(_)), "{kind:?}");
        assert_eq!(option_hint(&s, node, &json!("union")).as_deref(), Some("Union."));
        // a 3-2-1 datum is not such a list: it keeps its own editor
        let s = schema_of("align.datum");
        assert!(variants(&s, classify(&s, &s["properties"]["primary"]).2).is_none());
    }

    #[test]
    fn a_ticked_optional_parameter_starts_with_the_default_its_description_states() {
        assert_eq!(
            described_default("Fraction of triangles to keep, (0, 1] (default 0.5 when `target` is not given)."),
            Some(json!(0.5))
        );
        assert_eq!(
            described_default("Edges sharper than this stay in place (degrees, default 40)."),
            Some(json!(40.0))
        );
        assert_eq!(described_default("Keep the open boundary in place (default true)."), Some(json!(true)));
        assert_eq!(described_default("Default 3."), Some(json!(3.0)));
        assert_eq!(described_default("Mesh they belong to (default: the object being aligned)."), None);
        assert_eq!(described_default("Triangles to keep."), None);
        // ticking the field on must not change what the tool does: it shows the default
        let s = schema_of("mesh.decimate");
        let props = s["properties"].as_object().unwrap();
        let tokens = Tokens::resolve(crate::theme::Preset::Dark);
        let (issues, request) = (HashMap::new(), RefCell::new(None));
        let ctx = FormCtx {
            command: "mesh.decimate",
            objects: &[],
            selected: None,
            pick_objects: &[],
            pick_points: &[],
            pick_triangles: &[],
            picking: None,
            pick_request: &request,
            tokens: &tokens,
            issues: &issues,
        };
        let start = |name: &str| start_value(&s, &props[name], name, &classify(&s, &props[name]).0, &ctx);
        assert_eq!(start("ratio"), json!(0.5));
        assert_eq!(start("feature_angle_deg"), json!(40.0));
        assert_eq!(start("preserve_boundary"), json!(true));
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
