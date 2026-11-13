//! Schema navigation: `$ref` resolution, the widget kind of a schema node, its default value and
//! the alternatives of a `oneOf`/`anyOf` node.
use super::*;
use serde_json::{Map, Value, json};

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

pub(super) fn limit(s: &Value, key: &str) -> Option<f64> {
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

pub(super) fn required_of(s: &Value) -> Vec<&str> {
    s.get("required")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default()
}
