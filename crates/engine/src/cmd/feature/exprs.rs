//! `feature.edit` with expressions: `{"set": {"distance": {"expr": "d * 2"}}}`.
use super::super::param::num::expr_object;
use super::super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Map, Value};
use std::collections::BTreeMap;
use surfacecraft_doc::param_use::{apply_feature_exprs, check_expr_path};
use surfacecraft_doc::{FeatureObject, FeatureOp, ParamTable};

/// An expression a plain number replaced.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ClearedExpr {
    /// The field, `translate.1` for a component.
    pub field: String,
    /// The expression it had.
    pub expression: String,
}

/// Is `path` the field `field` or one of its components (`translate.1`)?
fn belongs_to(path: &str, field: &str) -> bool {
    path == field || path.strip_prefix(field).is_some_and(|rest| rest.starts_with('.'))
}

/// Does the number `v` equal what the old expression `text` gives now?
fn unchanged(v: &Value, text: Option<&String>, params: &ParamTable) -> bool {
    let (Some(x), Some(text)) = (v.as_f64(), text) else { return false };
    params.eval(text).is_ok_and(|now| (now - x).abs() <= 1e-12 * now.abs().max(x.abs()).max(1.0))
}

/// A value of `set` with its expressions taken out (a placeholder number stands in for each) and
/// recorded under their path. A component of an array sent as the number its old expression still
/// gives keeps that expression (a form sends every component, evaluated).
fn extract(
    field: &str,
    v: &Value,
    exprs: &mut BTreeMap<String, String>,
    old: &(&BTreeMap<String, String>, &ParamTable),
) -> CmdResult<Value> {
    let found = expr_object(v).map_err(CmdError::Params)?;
    if let Some(text) = found {
        check_expr_path(field).map_err(CmdError::Params)?;
        exprs.insert(field.to_string(), text);
        return Ok(Value::from(0));
    }
    let Some(items) = v.as_array() else { return Ok(v.clone()) };
    let mut out = Vec::with_capacity(items.len());
    for (k, item) in items.iter().enumerate() {
        let path = format!("{field}.{k}");
        if unchanged(item, old.0.get(&path), old.1)
            && let Some(text) = old.0.get(&path)
        {
            exprs.insert(path, text.clone());
            out.push(Value::from(0));
            continue;
        }
        out.push(extract(&path, item, exprs, old)?);
    }
    Ok(Value::Array(out))
}

/// The operation and expressions of `f` after `set` is applied. A plain number given for a field
/// replaces its expression; an expression object is evaluated and remembered.
pub(super) fn edited(
    f: &FeatureObject,
    set: &Map<String, Value>,
    params: &ParamTable,
) -> CmdResult<(FeatureOp, BTreeMap<String, String>, Vec<(String, String)>)> {
    let mut v = serde_json::to_value(&f.op).map_err(|e| CmdError::Failed(e.to_string()))?;
    let mut exprs = f.exprs.clone();
    let obj = v.as_object_mut().ok_or_else(|| CmdError::Failed("the operation is not an object".into()))?;
    for (k, val) in set {
        if k == "kind" || !obj.contains_key(k) {
            return Err(CmdError::Params(format!("`{k}` is not a parameter of this feature")));
        }
        exprs.retain(|path, _| !belongs_to(path, k));
        let plain = extract(k, val, &mut exprs, &(&f.exprs, params))?;
        obj.insert(k.clone(), plain);
        // new indices name new references: the stored signatures are recaptured from them
        for (indices, refs) in [("edges", "edge_refs"), ("faces", "face_refs")] {
            if k == indices {
                obj.remove(refs);
            }
        }
    }
    let op: FeatureOp = serde_json::from_value(v).map_err(|e| CmdError::Params(e.to_string()))?;
    let op = apply_feature_exprs(&op, &exprs, params).map_err(CmdError::Params)?;
    let cleared =
        f.exprs.iter().filter(|(path, _)| !exprs.contains_key(*path)).map(|(p, t)| (p.clone(), t.clone())).collect();
    Ok((op, exprs, cleared))
}
