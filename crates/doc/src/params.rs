//! The parameter table: named values defined by expressions over each other, which sketch
//! dimensions and feature fields can use instead of a number.
//!
//! The table is always consistent: every name valid and unique, every referenced name defined,
//! no cycle, and every value evaluated from its expression in dependency order. Operations
//! return a new table or an error; nothing is half-changed.
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use surfacecraft_sketch::expr::{Expr, ExprError, check_name, rename_in_text};

/// Most parameters in a document.
pub const MAX_PARAMS: usize = 1000;
/// Longest comment (bytes).
pub const MAX_COMMENT_LEN: usize = 1000;

/// One parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Param {
    /// Name: a letter or underscore, then letters, digits and underscores (case-sensitive).
    pub name: String,
    /// The expression text.
    pub expr: String,
    /// Optional note.
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub comment: String,
    /// The value of the expression, as last evaluated.
    pub value: f64,
}

/// Why a parameter operation was refused.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ParamError(pub String);

impl std::fmt::Display for ParamError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}

impl std::error::Error for ParamError {}

fn fail<T>(m: impl Into<String>) -> Result<T, ParamError> {
    Err(ParamError(m.into()))
}

fn expr_err(param: &str, e: &ExprError) -> ParamError {
    ParamError(format!("parameter `{param}`: {e}"))
}

/// Something that uses a parameter.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct ParamUser {
    /// `parameter`, `sketch` or `feature`.
    pub kind: String,
    /// Name of the parameter, sketch or feature.
    pub name: String,
    /// Where: `expression`, `constraint <index>` or the feature field.
    pub at: String,
}

impl std::fmt::Display for ParamUser {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{} `{}` ({})", self.kind, self.name, self.at)
    }
}

/// The table.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct ParamTable {
    items: Vec<Param>,
}

/// Parsed definitions in table order, with the names each uses.
struct Parsed {
    exprs: Vec<Expr>,
    deps: Vec<Vec<usize>>,
}

fn parse_all(items: &[Param]) -> Result<Parsed, ParamError> {
    if items.len() > MAX_PARAMS {
        return fail(format!("a document holds at most {MAX_PARAMS} parameters"));
    }
    let mut index: BTreeMap<&str, usize> = BTreeMap::new();
    for (k, p) in items.iter().enumerate() {
        check_name(&p.name).map_err(|e| ParamError(e.0))?;
        if p.comment.len() > MAX_COMMENT_LEN {
            return fail(format!("the comment of `{}` is too long", p.name));
        }
        if index.insert(&p.name, k).is_some() {
            return fail(format!("parameter `{}` is defined twice", p.name));
        }
    }
    let mut exprs = Vec::with_capacity(items.len());
    let mut deps = Vec::with_capacity(items.len());
    for p in items {
        let e = Expr::parse(&p.expr).map_err(|e| expr_err(&p.name, &e))?;
        let mut d = Vec::new();
        for n in e.names() {
            match index.get(n.as_str()) {
                Some(k) => d.push(*k),
                None => return fail(format!("parameter `{}` uses `{n}`, which is not defined", p.name)),
            }
        }
        exprs.push(e);
        deps.push(d);
    }
    Ok(Parsed { exprs, deps })
}

/// Indices in dependency order (a parameter after everything it uses), or the names of a cycle.
fn order(items: &[Param], deps: &[Vec<usize>]) -> Result<Vec<usize>, Vec<String>> {
    let n = items.len();
    let mut waiting: Vec<usize> = deps.iter().map(Vec::len).collect();
    let mut users: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (k, d) in deps.iter().enumerate() {
        for dep in d {
            if let Some(u) = users.get_mut(*dep) {
                u.push(k);
            }
        }
    }
    let mut ready: Vec<usize> = (0..n).filter(|k| waiting.get(*k) == Some(&0)).collect();
    let mut out = Vec::with_capacity(n);
    while let Some(k) = ready.pop() {
        out.push(k);
        for u in users.get(k).cloned().unwrap_or_default() {
            if let Some(w) = waiting.get_mut(u) {
                *w -= 1;
                if *w == 0 {
                    ready.push(u);
                }
            }
        }
    }
    if out.len() == n {
        return Ok(out);
    }
    Err(find_cycle(items, deps, &waiting))
}

/// A cycle among the parameters that could not be ordered (`waiting > 0`), named in order and
/// closed (`a -> b -> a`).
fn find_cycle(items: &[Param], deps: &[Vec<usize>], waiting: &[usize]) -> Vec<String> {
    let stuck = |k: usize| waiting.get(k).is_some_and(|w| *w > 0);
    let Some(start) = (0..items.len()).find(|k| stuck(*k)) else { return Vec::new() };
    let mut path: Vec<usize> = vec![start];
    let mut cur = start;
    while let Some(next) = deps.get(cur).and_then(|d| d.iter().copied().find(|k| stuck(*k))) {
        if let Some(at) = path.iter().position(|k| *k == next) {
            let mut cycle: Vec<String> = path
                .get(at..)
                .unwrap_or_default()
                .iter()
                .filter_map(|k| items.get(*k))
                .map(|p| p.name.clone())
                .collect();
            if let Some(first) = cycle.first().cloned() {
                cycle.push(first);
            }
            return cycle;
        }
        path.push(next);
        cur = next;
    }
    Vec::new()
}

/// Evaluates a whole definition list: validates it, orders it and computes every value.
fn rebuilt(items: Vec<Param>) -> Result<ParamTable, ParamError> {
    let parsed = parse_all(&items)?;
    let ord = order(&items, &parsed.deps)
        .map_err(|c| ParamError(format!("the parameters form a cycle: {}", c.join(" -> "))))?;
    let mut values: Vec<f64> = vec![0.0; items.len()];
    for k in ord {
        let (Some(e), Some(p)) = (parsed.exprs.get(k), items.get(k)) else { continue };
        let lookup = |n: &str| -> Result<f64, ExprError> {
            items
                .iter()
                .position(|q| q.name == n)
                .and_then(|i| values.get(i).copied())
                .ok_or_else(|| ExprError(format!("unknown name `{n}`")))
        };
        let v = e.eval(&lookup).map_err(|e| expr_err(&p.name, &e))?;
        if let Some(slot) = values.get_mut(k) {
            *slot = v;
        }
    }
    let items = items.into_iter().zip(values).map(|(p, value)| Param { value, ..p }).collect();
    Ok(ParamTable { items })
}

impl ParamTable {
    /// The parameters in creation order.
    #[must_use]
    pub fn items(&self) -> &[Param] {
        &self.items
    }

    /// Is the table empty?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.items.is_empty()
    }

    /// The parameter called `name`.
    #[must_use]
    pub fn get(&self, name: &str) -> Option<&Param> {
        self.items.iter().find(|p| p.name == name)
    }

    /// Evaluates an expression text against the current values.
    pub fn eval(&self, text: &str) -> Result<f64, ExprError> {
        let lookup = |n: &str| -> Result<f64, ExprError> {
            self.get(n).map(|p| p.value).ok_or_else(|| ExprError(format!("unknown parameter `{n}`")))
        };
        Expr::parse(text)?.eval(&lookup)
    }

    /// Re-checks a table that came from outside (a project file) and re-evaluates every value:
    /// the stored values are not trusted.
    pub fn validated(&self) -> Result<ParamTable, ParamError> {
        rebuilt(self.items.clone())
    }

    /// The table with `name` defined by `expr` (created, or redefined in place; `comment` `None`
    /// keeps the old comment). Refused when the name is invalid, the expression does not parse or
    /// evaluate, it uses an undefined name, or the definitions would form a cycle. A parameter
    /// that depends on this one and would no longer evaluate refuses the change too.
    pub fn with_param(&self, name: &str, expr: &str, comment: Option<&str>) -> Result<ParamTable, ParamError> {
        check_name(name).map_err(|e| ParamError(e.0))?;
        let old = self.get(name);
        let comment = comment.map_or_else(|| old.map(|p| p.comment.clone()).unwrap_or_default(), str::to_string);
        let new = Param { name: name.to_string(), expr: expr.to_string(), comment, value: 0.0 };
        let mut items = self.items.clone();
        match items.iter_mut().find(|p| p.name == name) {
            Some(slot) => *slot = new,
            None => items.push(new),
        }
        rebuilt(items)
    }

    /// The table without `name`; the caller has checked that nothing uses it.
    pub fn without(&self, name: &str) -> Result<ParamTable, ParamError> {
        if self.get(name).is_none() {
            return fail(format!("there is no parameter `{name}`"));
        }
        let items = self.items.iter().filter(|p| p.name != name).cloned().collect();
        rebuilt(items)
    }

    /// The table with `old` renamed to `new` here and in the expressions of the other
    /// parameters.
    pub fn renamed(&self, old: &str, new: &str) -> Result<ParamTable, ParamError> {
        check_name(new).map_err(|e| ParamError(e.0))?;
        if self.get(old).is_none() {
            return fail(format!("there is no parameter `{old}`"));
        }
        if old != new && self.get(new).is_some() {
            return fail(format!("there is already a parameter `{new}`"));
        }
        let items = self
            .items
            .iter()
            .map(|p| {
                let expr = rename_in_text(&p.expr, old, new).map_err(|e| expr_err(&p.name, &e))?;
                let name = if p.name == old { new.to_string() } else { p.name.clone() };
                Ok(Param { name, expr, ..p.clone() })
            })
            .collect::<Result<Vec<_>, ParamError>>()?;
        rebuilt(items)
    }

    /// The parameters whose expression uses `name`.
    #[must_use]
    pub fn users_of(&self, name: &str) -> Vec<ParamUser> {
        self.items
            .iter()
            .filter(|p| Expr::parse(&p.expr).is_ok_and(|e| e.names().contains(name)))
            .map(|p| ParamUser { kind: "parameter".into(), name: p.name.clone(), at: "expression".into() })
            .collect()
    }

    /// Names whose value differs from `before`'s (or that `before` lacks): `(name, old, new)`.
    #[must_use]
    pub fn changes_since(&self, before: &ParamTable) -> Vec<(String, Option<f64>, f64)> {
        self.items
            .iter()
            .filter_map(|p| {
                let old = before.get(&p.name).map(|q| q.value);
                (old != Some(p.value)).then(|| (p.name.clone(), old, p.value))
            })
            .collect()
    }
}

/// Does `text` use the name `name`? (False for text that does not parse.)
#[must_use]
pub fn text_uses(text: &str, name: &str) -> bool {
    Expr::parse(text).is_ok_and(|e| e.names().contains(name))
}

/// The names used by any of `texts`.
#[must_use]
pub fn names_used<'a>(texts: impl IntoIterator<Item = &'a String>) -> BTreeSet<String> {
    texts.into_iter().filter_map(|t| Expr::parse(t).ok()).flat_map(|e| e.names()).collect()
}

#[cfg(test)]
mod tests;
