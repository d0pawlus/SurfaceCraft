//! Validation of a value against its parameter schema, with the messages shown next to the fields.
use super::names::*;
use super::schema::*;
use super::*;
use crate::i18n::{self, Lang, t, tf};
use serde_json::Value;

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

pub(super) fn fmt_num(n: f64) -> String {
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

pub(super) fn join(path: &str, name: &str) -> String {
    if path.is_empty() { name.to_string() } else { format!("{path}.{name}") }
}
