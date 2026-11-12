//! A constraint as a command takes it: a dimension's value may be a number or `{"expr": "..."}`.
use super::super::param::num::expr_object;
use super::super::{CmdError, CmdResult};
use schemars::{JsonSchema, Schema, SchemaGenerator};
use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::Value;
use std::borrow::Cow;
use surfacecraft_doc::ParamTable;
use surfacecraft_sketch::{Constraint, Sketch};

/// A constraint, with the expression its dimension was given as, if any. Its schema is that of
/// [`Constraint`]: an expression is written `{"expr": "..."}` in place of the number of a length,
/// distance, radius (`value`) or angle (`degrees`).
#[derive(Clone, Debug, PartialEq)]
pub struct ConstraintIn {
    /// The constraint (its dimension holds a placeholder until the expression is evaluated).
    pub constraint: Constraint,
    /// The expression of the dimension.
    pub expr: Option<String>,
}

/// The JSON field that holds the dimension of the constraint of type `ty`.
fn dimension_key(ty: Option<&str>) -> &'static str {
    if ty == Some("angle") { "degrees" } else { "value" }
}

impl<'de> Deserialize<'de> for ConstraintIn {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        let mut v = Value::deserialize(d)?;
        let key = dimension_key(v.get("type").and_then(Value::as_str));
        let expr = match v.get(key) {
            Some(x) => expr_object(x).map_err(D::Error::custom)?,
            None => None,
        };
        if expr.is_some()
            && let Some(m) = v.as_object_mut()
        {
            m.insert(key.to_string(), Value::from(1.0));
        }
        let constraint = serde_json::from_value(v).map_err(D::Error::custom)?;
        Ok(ConstraintIn { constraint, expr })
    }
}

impl Serialize for ConstraintIn {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        let mut v = serde_json::to_value(&self.constraint).map_err(serde::ser::Error::custom)?;
        if let Some(e) = &self.expr {
            let key = dimension_key(v.get("type").and_then(Value::as_str));
            if let Some(m) = v.as_object_mut() {
                m.insert(key.to_string(), serde_json::json!({ "expr": e }));
            }
        }
        v.serialize(s)
    }
}

impl JsonSchema for ConstraintIn {
    fn schema_name() -> Cow<'static, str> {
        Constraint::schema_name()
    }

    fn schema_id() -> Cow<'static, str> {
        Constraint::schema_id()
    }

    fn inline_schema() -> bool {
        Constraint::inline_schema()
    }

    fn json_schema(g: &mut SchemaGenerator) -> Schema {
        Constraint::json_schema(g)
    }
}

impl ConstraintIn {
    /// The constraint with its expression evaluated, and the expression to keep.
    pub fn evaluated(&self, params: &ParamTable) -> CmdResult<(Constraint, Option<String>)> {
        let Some(text) = &self.expr else { return Ok((self.constraint.clone(), None)) };
        let v = params.eval(text).map_err(|e| CmdError::Params(format!("dimension (`{text}`): {e}")))?;
        let c = self
            .constraint
            .with_dimension(v)
            .ok_or_else(|| CmdError::Params("only a length, distance, radius or angle takes an expression".into()))?;
        Ok((c, Some(text.clone())))
    }
}

/// The sketch with `inputs` appended as constraints (their expressions recorded).
pub fn with_constraints(sk: &Sketch, inputs: &[ConstraintIn], params: &ParamTable) -> CmdResult<Sketch> {
    let mut out = sk.clone();
    for ci in inputs {
        let (c, expr) = ci.evaluated(params)?;
        if let Some(e) = expr {
            out.dim_exprs.insert(out.constraints.len(), e);
        }
        out.constraints.push(c);
    }
    Ok(out)
}

/// The sketch with constraint `index` replaced by `input` (its expression recorded, or cleared
/// for a plain number), or removed when `input` is `None` (later expressions move down).
pub fn with_edit(sk: &Sketch, index: usize, input: Option<&ConstraintIn>, params: &ParamTable) -> CmdResult<Sketch> {
    let mut out = sk.clone();
    match input {
        Some(ci) => {
            let (c, expr) = ci.evaluated(params)?;
            let slot = out
                .constraints
                .get_mut(index)
                .ok_or_else(|| CmdError::Params(format!("constraint {index} does not exist")))?;
            *slot = c;
            match expr {
                Some(e) => out.dim_exprs.insert(index, e),
                None => out.dim_exprs.remove(&index),
            };
        }
        None => {
            if index >= out.constraints.len() {
                return Err(CmdError::Params(format!("constraint {index} does not exist")));
            }
            out.dim_exprs = out.dim_exprs_without(index);
            out.constraints.remove(index);
        }
    }
    Ok(out)
}
