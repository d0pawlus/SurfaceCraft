//! Where parameters are used: dimension expressions of sketches and expression-valued fields of
//! features. Evaluating them, listing their users and renaming a parameter in them.
use crate::feature::{FeatureObject, FeatureOp};
use crate::params::{ParamError, ParamTable, ParamUser};
use crate::{Document, SketchObject};
use serde_json::Value;
use std::collections::BTreeMap;
use surfacecraft_sketch::Sketch;
use surfacecraft_sketch::expr::{Expr, ExprError, MAX_LEN, rename_in_text};

/// Feature fields that can be given as an expression: the numeric fields of the operations (a
/// vector field takes one expression per component, as `translate.0`).
pub const EXPR_FIELDS: &[&str] = &[
    "distance",
    "draft_deg",
    "angle_deg",
    "radius",
    "thickness",
    "count",
    "degree",
    "translate",
    "rotate_axis",
    "rotate_origin",
    "rotate_deg",
    "axis_origin",
    "axis_direction",
    "neutral_origin",
    "neutral_normal",
    "pull",
    "plane_origin",
    "plane_normal",
];

/// Fields that count something: an expression must give a whole number.
pub const INT_FIELDS: &[&str] = &["count", "degree"];

/// Largest count (instances, degree) a field may take from an expression or a literal.
pub const MAX_COUNT: f64 = 100_000.0;

/// Largest difference from a whole number that still counts as whole (relative to the value, at
/// least absolute): `0.1 * 30` is 3.
pub const INTEGRAL_TOL: f64 = 1e-9;

/// Is `path` a field that can hold an expression (`distance`, `translate.2`)?
pub fn check_expr_path(path: &str) -> Result<(), String> {
    let mut parts = path.split('.');
    let field = parts.next().unwrap_or("");
    if !EXPR_FIELDS.contains(&field) {
        return Err(format!("`{field}` is not a numeric field that takes an expression"));
    }
    match (parts.next(), parts.next()) {
        (None, _) => Ok(()),
        (Some("0" | "1" | "2"), None) => Ok(()),
        _ => Err(format!("`{path}` is not a field or a component of one")),
    }
}

/// `x` as a count: whole (within [`INTEGRAL_TOL`]) and not above [`MAX_COUNT`].
pub fn whole_number(x: f64, what: &str) -> Result<u64, String> {
    let r = x.round();
    if !x.is_finite() || (x - r).abs() > INTEGRAL_TOL * x.abs().max(1.0) || !(0.0..=MAX_COUNT).contains(&r) {
        return Err(format!("`{what}` must be a whole number from 0 to {MAX_COUNT}, not {x}"));
    }
    Ok(r as u64)
}

/// The sketch with every dimension expression evaluated into its constraint.
pub fn apply_dim_exprs(sk: &Sketch, params: &ParamTable) -> Result<Sketch, String> {
    let mut out = sk.clone();
    for (k, text) in &sk.dim_exprs {
        let v = params.eval(text).map_err(|e| format!("dimension {k} (`{text}`): {e}"))?;
        let c = out
            .constraints
            .get(*k)
            .and_then(|c| c.with_dimension(v))
            .ok_or_else(|| format!("constraint {k} is not a dimension"))?;
        if let Some(slot) = out.constraints.get_mut(*k) {
            *slot = c;
        }
    }
    Ok(out)
}

fn slot<'a>(v: &'a mut Value, path: &str) -> Result<&'a mut Value, String> {
    let mut parts = path.split('.');
    let field = parts.next().unwrap_or("");
    let top = v.get_mut(field).ok_or_else(|| format!("this feature has no field `{field}`"))?;
    let target = match parts.next().and_then(|i| i.parse::<usize>().ok()) {
        Some(i) => top.get_mut(i).ok_or_else(|| format!("`{field}` is not set"))?,
        None => top,
    };
    if target.is_number() { Ok(target) } else { Err(format!("`{path}` is not a number in this feature")) }
}

/// The operation with every expression-valued field evaluated into the field.
pub fn apply_feature_exprs(
    op: &FeatureOp,
    exprs: &BTreeMap<String, String>,
    params: &ParamTable,
) -> Result<FeatureOp, String> {
    if exprs.is_empty() {
        return Ok(op.clone());
    }
    let mut v = serde_json::to_value(op).map_err(|e| e.to_string())?;
    for (path, text) in exprs {
        check_expr_path(path)?;
        let x = params.eval(text).map_err(|e| format!("{path} (`{text}`): {e}"))?;
        let field = path.split('.').next().unwrap_or("");
        let number = if INT_FIELDS.contains(&field) { Value::from(whole_number(x, path)?) } else { Value::from(x) };
        *slot(&mut v, path)? = number;
    }
    serde_json::from_value(v).map_err(|e| e.to_string())
}

/// Every feature, active first, then the parked ones.
fn all_features(doc: &Document) -> impl Iterator<Item = &FeatureObject> {
    doc.features.iter().chain(doc.rolled_back.iter())
}

/// Everything that uses parameter `name`: other parameters, sketch dimensions, features (parked
/// ones included).
#[must_use]
pub fn users_of(doc: &Document, name: &str) -> Vec<ParamUser> {
    let uses = |t: &String| crate::params::text_uses(t, name);
    let mut out = doc.params.users_of(name);
    for s in &doc.sketches {
        let at = s.sketch.dim_exprs.iter().filter(|(_, t)| uses(t)).map(|(k, _)| format!("constraint {k}"));
        out.extend(at.map(|at| ParamUser { kind: "sketch".into(), name: s.name.clone(), at }));
    }
    for f in all_features(doc) {
        let at = f.exprs.iter().filter(|(_, t)| uses(t)).map(|(k, _)| k.clone());
        out.extend(at.map(|at| ParamUser { kind: "feature".into(), name: f.name.clone(), at }));
    }
    out
}

/// A rewritten expression must still be an expression: short enough, parsing, and evaluating
/// against the renamed table. `what` names its owner for the error.
fn checked_text(text: &str, what: &str, table: &ParamTable) -> Result<(), ParamError> {
    if text.len() > MAX_LEN {
        return Err(ParamError(format!(
            "renaming would make the expression of {what} longer than {MAX_LEN} characters"
        )));
    }
    let expr =
        Expr::parse(text).map_err(|e| ParamError(format!("the expression of {what} would no longer parse: {e}")))?;
    let lookup = |n: &str| table.get(n).map(|p| p.value).ok_or_else(|| ExprError(format!("unknown parameter `{n}`")));
    expr.eval(&lookup)
        .map(|_| ())
        .map_err(|e| ParamError(format!("the expression of {what} would no longer evaluate: {e}")))
}

fn renamed_texts<K: Clone + Ord + std::fmt::Display>(
    m: &BTreeMap<K, String>,
    owner: &str,
    (old, new): (&str, &str),
    table: &ParamTable,
) -> Result<BTreeMap<K, String>, ParamError> {
    m.iter()
        .map(|(k, t)| {
            let text = rename_in_text(t, old, new).map_err(|e| ParamError(format!("{owner} `{k}`: {}", e.0)))?;
            checked_text(&text, &format!("{owner} `{k}`"), table)?;
            Ok((k.clone(), text))
        })
        .collect()
}

/// The sketches and features with parameter `old` renamed to `new` in their expressions. Every
/// rewritten text is validated against `table` (the renamed parameter table) first; the first
/// that is not valid refuses the whole rename and names its owner. Nothing is changed.
pub fn renamed_references(
    doc: &Document,
    table: &ParamTable,
    old: &str,
    new: &str,
) -> Result<(Vec<SketchObject>, Vec<FeatureObject>, Vec<FeatureObject>), ParamError> {
    let sketches = doc
        .sketches
        .iter()
        .map(|s| {
            let owner = format!("sketch `{}`, constraint", s.name);
            let dim_exprs = renamed_texts(&s.sketch.dim_exprs, &owner, (old, new), table)?;
            Ok(SketchObject { sketch: Sketch { dim_exprs, ..s.sketch.clone() }, ..s.clone() })
        })
        .collect::<Result<Vec<_>, ParamError>>()?;
    let feats = |list: &[FeatureObject]| {
        list.iter()
            .map(|f| {
                let owner = format!("feature `{}`, field", f.name);
                Ok(FeatureObject { exprs: renamed_texts(&f.exprs, &owner, (old, new), table)?, ..f.clone() })
            })
            .collect::<Result<Vec<_>, ParamError>>()
    };
    Ok((sketches, feats(&doc.features)?, feats(&doc.rolled_back)?))
}

/// Stored expressions that no longer parse (or are over-long): the document is inconsistent, and
/// until that is repaired no parameter can be deleted or renamed, as nothing says what they use.
#[must_use]
pub fn integrity_problems(doc: &Document) -> Vec<String> {
    let bad = |t: &str| t.len() > MAX_LEN || Expr::parse(t).is_err();
    let mut out: Vec<String> =
        doc.params.items().iter().filter(|p| bad(&p.expr)).map(|p| format!("parameter `{}`", p.name)).collect();
    for s in &doc.sketches {
        out.extend(
            s.sketch
                .dim_exprs
                .iter()
                .filter(|(_, t)| bad(t))
                .map(|(k, _)| format!("sketch `{}`, constraint {k}", s.name)),
        );
    }
    for f in all_features(doc) {
        out.extend(f.exprs.iter().filter(|(_, t)| bad(t)).map(|(k, _)| format!("feature `{}`, field `{k}`", f.name)));
    }
    out
}

/// Within a relative `1e-12` (or absolute for tiny values)?
fn close(a: f64, b: f64) -> bool {
    (a - b).abs() <= 1e-12 * a.abs().max(b.abs()).max(1.0)
}

/// Are two JSON values equal, numbers within [`close`]?
fn same_json(a: &Value, b: &Value) -> bool {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => match (x.as_f64(), y.as_f64()) {
            (Some(x), Some(y)) => close(x, y),
            _ => x == y,
        },
        (Value::Array(x), Value::Array(y)) => x.len() == y.len() && x.iter().zip(y).all(|(p, q)| same_json(p, q)),
        (Value::Object(x), Value::Object(y)) => {
            x.len() == y.len() && x.iter().all(|(k, p)| y.get(k).is_some_and(|q| same_json(p, q)))
        }
        _ => a == b,
    }
}

/// Static checks of an expression stored with a feature or a dimension: it parses, uses only
/// defined parameters, and sits where a number does.
fn check_text(text: &str, table: &ParamTable) -> Result<(), String> {
    let e = Expr::parse(text).map_err(|e| e.to_string())?;
    match e.names().into_iter().find(|n| table.get(n).is_none()) {
        Some(n) => Err(format!("it uses `{n}`, which is not a defined parameter")),
        None => Ok(()),
    }
}

/// The feature's expressions refer to numeric fields of its operation and use defined parameters.
fn check_feature(f: &FeatureObject, table: &ParamTable) -> Result<(), String> {
    let mut v = serde_json::to_value(&f.op).map_err(|e| e.to_string())?;
    for (path, text) in &f.exprs {
        let fail = |m: String| format!("feature {}, field `{path}`: {m}", f.name);
        if text.len() > MAX_LEN {
            return Err(fail("the expression is too long".into()));
        }
        check_expr_path(path).map_err(fail)?;
        slot(&mut v, path).map_err(fail)?;
        check_text(text, table).map_err(fail)?;
    }
    Ok(())
}

/// Is the cached value of the feature (its operation) what its expressions give now? An
/// expression that does not evaluate counts as not matching (the replay will report it).
fn feature_current(f: &FeatureObject, table: &ParamTable) -> bool {
    let Ok(now) = apply_feature_exprs(&f.op, &f.exprs, table) else { return false };
    match (serde_json::to_value(&f.op), serde_json::to_value(&now)) {
        (Ok(a), Ok(b)) => same_json(&a, &b),
        _ => false,
    }
}

fn sketch_current(s: &SketchObject, table: &ParamTable) -> bool {
    apply_dim_exprs(&s.sketch, table).is_ok_and(|now| {
        let dims =
            |sk: &Sketch| sk.constraints.iter().map(surfacecraft_sketch::Constraint::dimension).collect::<Vec<_>>();
        let (a, b) = (dims(&s.sketch), dims(&now));
        a.len() == b.len()
            && a.iter().zip(&b).all(|(x, y)| match (x, y) {
                (Some(x), Some(y)) => close(*x, *y),
                (x, y) => x == y,
            })
    })
}

/// Checks the expressions of a document that came from outside (a project file). The table is
/// re-evaluated (stored values are not trusted); every expression must parse, use defined
/// parameters and sit on an existing numeric field or dimension, else the file is refused.
/// Returns the table and whether the cached values of dimensions, fields or parameters differ
/// from what the expressions give (`Document::needs_recompute`): the geometry is then stale.
pub fn validate_document(doc: &Document) -> Result<(ParamTable, bool), String> {
    let table = doc.params.validated().map_err(|e| e.to_string())?;
    let mut stale = table.items().iter().zip(doc.params.items()).any(|(a, b)| !close(a.value, b.value));
    for s in &doc.sketches {
        s.sketch.validate_dim_exprs().map_err(|e| format!("sketch {}: {e}", s.name))?;
        for (k, text) in &s.sketch.dim_exprs {
            check_text(text, &table).map_err(|e| format!("sketch {}, constraint {k}: {e}", s.name))?;
        }
        stale |= !sketch_current(s, &table);
    }
    for f in all_features(doc) {
        check_feature(f, &table)?;
        stale |= !feature_current(f, &table);
    }
    Ok((table, stale))
}

#[cfg(test)]
mod tests;
