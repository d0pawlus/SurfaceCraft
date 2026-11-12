//! Numeric command inputs that may be an expression over the document's parameters.
//!
//! A plain number stays what it was; `{"expr": "w / 2"}` is evaluated against the parameter
//! table when the command runs, and the text is kept with the thing it set (a sketch dimension, a
//! feature field) so that a later change of a parameter recomputes it. The JSON Schema of these
//! types is a union: the number (an integer for [`Count`]), marked `x-expr` so that forms keep
//! their numeric widgets, or the expression object `{"expr": "text"}`.
use super::super::{CmdError, CmdResult};
use schemars::{JsonSchema, Schema, SchemaGenerator, json_schema};
use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize, Serializer};
use serde_json::{Map, Value};
use std::borrow::Cow;
use std::collections::BTreeMap;
use surfacecraft_doc::ParamTable;
use surfacecraft_doc::param_use::whole_number;

/// The JSON Schema branch of an expression object: `{"expr": "text"}` and nothing else.
#[must_use]
pub fn expr_branch() -> Value {
    serde_json::json!({
        "type": "object",
        "properties": {"expr": {"type": "string"}},
        "required": ["expr"],
        "additionalProperties": false
    })
}

/// A number, or an expression over parameters.
#[derive(Clone, Debug, PartialEq)]
pub enum Num {
    /// A literal.
    Value(f64),
    /// The text of an expression.
    Expr(String),
}

impl Default for Num {
    fn default() -> Self {
        Num::Value(0.0)
    }
}

/// A whole number, or an expression over parameters that gives one.
#[derive(Clone, Debug, PartialEq)]
pub struct Count(pub Num);

/// The expression of `{"expr": "..."}`, if `v` is that.
pub fn expr_object(v: &Value) -> Result<Option<String>, String> {
    let Some(m) = v.as_object() else { return Ok(None) };
    match (m.len(), m.get("expr")) {
        (1, Some(Value::String(s))) => Ok(Some(s.clone())),
        _ => Err("an expression is written {\"expr\": \"text\"}".into()),
    }
}

impl Num {
    /// The JSON this input was read from.
    #[must_use]
    pub fn to_json(&self) -> Value {
        match self {
            Num::Value(v) => Value::from(*v),
            Num::Expr(e) => Value::Object(Map::from_iter([("expr".to_string(), Value::String(e.clone()))])),
        }
    }

    /// Reads a number or an expression object.
    pub fn from_json(v: &Value) -> Result<Num, String> {
        if let Some(e) = expr_object(v)? {
            return Ok(Num::Expr(e));
        }
        match v.as_f64() {
            Some(x) if x.is_finite() => Ok(Num::Value(x)),
            _ => Err("expected a number or {\"expr\": \"text\"}".into()),
        }
    }
}

impl Serialize for Num {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.to_json().serialize(s)
    }
}

impl<'de> Deserialize<'de> for Num {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        Num::from_json(&Value::deserialize(d)?).map_err(D::Error::custom)
    }
}

impl JsonSchema for Num {
    fn schema_name() -> Cow<'static, str> {
        "Num".into()
    }

    fn inline_schema() -> bool {
        true
    }

    fn json_schema(_: &mut SchemaGenerator) -> Schema {
        json_schema!({"anyOf": [{"type": "number", "format": "double", "x-expr": true}, expr_branch()]})
    }
}

impl Serialize for Count {
    fn serialize<S: Serializer>(&self, s: S) -> Result<S::Ok, S::Error> {
        self.0.serialize(s)
    }
}

impl<'de> Deserialize<'de> for Count {
    fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
        Num::deserialize(d).map(Count)
    }
}

impl JsonSchema for Count {
    fn schema_name() -> Cow<'static, str> {
        "Count".into()
    }

    fn inline_schema() -> bool {
        true
    }

    fn json_schema(_: &mut SchemaGenerator) -> Schema {
        json_schema!({"anyOf": [{"type": "integer", "format": "uint", "minimum": 0, "x-expr": true}, expr_branch()]})
    }
}

/// Evaluates inputs against the parameter table and collects the expressions they came from,
/// keyed by field (`distance`, `translate.1`).
pub struct ExprSet<'a> {
    params: &'a ParamTable,
    exprs: BTreeMap<String, String>,
}

impl<'a> ExprSet<'a> {
    /// A collector reading the values of `params`.
    #[must_use]
    pub fn new(params: &'a ParamTable) -> Self {
        Self { params, exprs: BTreeMap::new() }
    }

    /// The value of the input for field `key`; an expression is remembered.
    pub fn num(&mut self, key: &str, n: &Num) -> CmdResult<f64> {
        match n {
            Num::Value(v) => Ok(*v),
            Num::Expr(text) => {
                let v = self.params.eval(text).map_err(|e| CmdError::Params(format!("{key} (`{text}`): {e}")))?;
                self.exprs.insert(key.to_string(), text.clone());
                Ok(v)
            }
        }
    }

    /// An optional input.
    pub fn opt(&mut self, key: &str, n: &Option<Num>) -> CmdResult<Option<f64>> {
        n.as_ref().map(|n| self.num(key, n)).transpose()
    }

    /// A count: a whole number within the limits of [`whole_number`].
    pub fn count(&mut self, key: &str, c: &Count) -> CmdResult<usize> {
        let v = self.num(key, &c.0)?;
        let n = whole_number(v, key).map_err(CmdError::Params)?;
        usize::try_from(n).map_err(|e| CmdError::Params(e.to_string()))
    }

    /// An optional count.
    pub fn opt_count(&mut self, key: &str, c: &Option<Count>) -> CmdResult<Option<u32>> {
        let Some(c) = c else { return Ok(None) };
        let n = self.count(key, c)?;
        u32::try_from(n).map(Some).map_err(|e| CmdError::Params(e.to_string()))
    }

    /// A vector input, one expression per component (`key.0`, `key.1`, `key.2`).
    pub fn vec3(&mut self, key: &str, v: &[Num; 3]) -> CmdResult<[f64; 3]> {
        let mut out = [0.0; 3];
        for (k, (slot, n)) in out.iter_mut().zip(v).enumerate() {
            *slot = self.num(&format!("{key}.{k}"), n)?;
        }
        Ok(out)
    }

    /// An optional vector input.
    pub fn opt_vec3(&mut self, key: &str, v: &Option<[Num; 3]>) -> CmdResult<Option<[f64; 3]>> {
        v.as_ref().map(|v| self.vec3(key, v)).transpose()
    }

    /// The expressions collected.
    #[must_use]
    pub fn finish(self) -> BTreeMap<String, String> {
        self.exprs
    }
}

/// The default of a vector input: three zeros.
#[must_use]
pub fn zero3() -> [Num; 3] {
    [Num::Value(0.0), Num::Value(0.0), Num::Value(0.0)]
}
