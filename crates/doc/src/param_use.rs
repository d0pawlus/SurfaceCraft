//! Where parameters are used: dimension expressions of sketches and expression-valued fields of
//! features. Evaluating them, listing their users and renaming a parameter in them.
use crate::feature::{FeatureObject, FeatureOp};
use crate::params::{ParamError, ParamTable, ParamUser};
use crate::{Document, SketchObject};
use serde_json::Value;
use std::collections::BTreeMap;
use surfacecraft_sketch::Sketch;
use surfacecraft_sketch::expr::{MAX_LEN, rename_in_text};

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

fn renamed_texts<K: Clone + Ord>(
    m: &BTreeMap<K, String>,
    old: &str,
    new: &str,
) -> Result<BTreeMap<K, String>, ParamError> {
    m.iter().map(|(k, t)| rename_in_text(t, old, new).map(|t| (k.clone(), t)).map_err(|e| ParamError(e.0))).collect()
}

/// The sketches and features with parameter `old` renamed to `new` in their expressions. Nothing
/// is changed when any of them fails (they all parse, or a corrupt text is refused).
pub fn renamed_references(
    doc: &Document,
    old: &str,
    new: &str,
) -> Result<(Vec<SketchObject>, Vec<FeatureObject>, Vec<FeatureObject>), ParamError> {
    let sketches = doc
        .sketches
        .iter()
        .map(|s| {
            let dim_exprs = renamed_texts(&s.sketch.dim_exprs, old, new)?;
            Ok(SketchObject { sketch: Sketch { dim_exprs, ..s.sketch.clone() }, ..s.clone() })
        })
        .collect::<Result<Vec<_>, ParamError>>()?;
    let feats = |list: &[FeatureObject]| {
        list.iter()
            .map(|f| Ok(FeatureObject { exprs: renamed_texts(&f.exprs, old, new)?, ..f.clone() }))
            .collect::<Result<Vec<_>, ParamError>>()
    };
    Ok((sketches, feats(&doc.features)?, feats(&doc.rolled_back)?))
}

/// Checks the expressions of a document that came from outside (a project file): the parameter
/// table is re-evaluated, and every expression field is a known field with a text of sane size.
pub fn validate_document(doc: &Document) -> Result<ParamTable, String> {
    let table = doc.params.validated().map_err(|e| e.to_string())?;
    for s in &doc.sketches {
        s.sketch.validate_dim_exprs().map_err(|e| format!("sketch {}: {e}", s.name))?;
    }
    for f in all_features(doc) {
        for (path, text) in &f.exprs {
            check_expr_path(path).map_err(|e| format!("feature {}: {e}", f.name))?;
            if text.len() > MAX_LEN {
                return Err(format!("feature {}: the expression of `{path}` is too long", f.name));
            }
        }
    }
    Ok(table)
}

#[cfg(test)]
mod tests;
