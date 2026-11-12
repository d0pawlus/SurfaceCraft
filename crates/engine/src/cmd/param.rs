//! `param.*`: named parameters. A parameter is a name with an expression over other parameters;
//! sketch dimensions and the numeric fields of features may use expressions instead of numbers,
//! and changing a parameter recomputes everything that uses it as one undo step.
use super::{CmdError, CmdResult, CommandSpec};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::param_use::{apply_dim_exprs, integrity_problems, renamed_references, users_of};
use surfacecraft_doc::{ObjectId, ParamUser};
use surfacecraft_sketch::solve;

pub mod num;

/// Something that failed in the recompute after a parameter changed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct Failure {
    /// `sketch` or `feature`.
    pub kind: String,
    /// Its name.
    pub name: String,
    /// Why.
    pub message: String,
}

/// A parameter whose value changed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ValueChange {
    /// Parameter name.
    pub name: String,
    /// Value before (absent for a new parameter).
    pub old: Option<f64>,
    /// Value now.
    pub new: f64,
}

/// Result of `param.set`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SetOut {
    /// Parameter name.
    pub name: String,
    /// Its expression.
    pub expression: String,
    /// Its value.
    pub value: f64,
    /// Whether the parameter is new.
    pub created: bool,
    /// Every parameter whose value changed (this one and those that depend on it).
    pub changes: Vec<ValueChange>,
    /// What does not work after the change: sketches that cannot be solved and features that fail.
    pub failures: Vec<Failure>,
    /// Features that failed.
    pub failed_features: usize,
}

/// Set parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetIn {
    /// Parameter name: a letter or underscore, then letters, digits and underscores.
    pub name: String,
    /// Expression: numbers, parameter names, `+ - * / ^`, parentheses, `pi` and the functions `sin cos tan asin acos atan atan2` (degrees), `sqrt abs min max floor ceil round`.
    pub expression: String,
    /// Note kept with the parameter (omitted: unchanged).
    #[serde(default)]
    pub comment: Option<String>,
}

/// The sketches and features that fail at the moment.
pub(crate) fn failures(s: &Session) -> Vec<Failure> {
    let sketches = s.doc.sketches.iter().filter_map(|k| {
        k.error.as_ref().map(|m| Failure { kind: "sketch".into(), name: k.name.clone(), message: m.clone() })
    });
    let features = s.doc.features.iter().filter_map(|f| {
        f.error.as_ref().map(|m| Failure { kind: "feature".into(), name: f.name.clone(), message: m.clone() })
    });
    sketches.chain(features).collect()
}

/// Evaluates the dimension expressions of every sketch against the current parameters and solves
/// the sketches whose dimensions moved (or that were failing). A sketch that cannot be
/// evaluated or solved is marked, and the features reading it fail in the replay.
fn resolve_sketches(s: &mut Session) {
    let ids: Vec<ObjectId> = s.doc.sketches.iter().filter(|k| !k.sketch.dim_exprs.is_empty()).map(|k| k.id).collect();
    for id in ids {
        let Some(o) = s.doc.sketch(id) else { continue };
        let outcome = apply_dim_exprs(&o.sketch, &s.doc.params).and_then(|mut sk| {
            if sk == o.sketch && o.error.is_none() {
                return Ok(None);
            }
            let rep = solve(&mut sk).map_err(|e| e.to_string())?;
            Ok(Some((sk, rep)))
        });
        let Some(o) = s.doc.sketch_mut(id) else { continue };
        match outcome {
            Ok(None) => {}
            Ok(Some((sk, rep))) => {
                o.error =
                    (!rep.is_satisfied()).then(|| format!("its constraints cannot be satisfied ({})", rep.message));
                o.sketch = sk;
                o.report = Some(rep);
            }
            Err(e) => o.error = Some(e),
        }
    }
}

/// Recomputes what depends on the parameters: sketch dimensions, then the whole history (which
/// evaluates the expressions of the features first). Returns the number of failed features.
pub(crate) fn recompute(s: &mut Session) -> usize {
    resolve_sketches(s);
    s.doc.needs_recompute = false;
    crate::replay::replay(s)
}

fn set(s: &mut Session, i: SetIn) -> CmdResult<SetOut> {
    let before = s.doc.params.clone();
    let table = before.with_param(&i.name, &i.expression, i.comment.as_deref()).map_err(|e| CmdError::Params(e.0))?;
    let changes: Vec<ValueChange> =
        table.changes_since(&before).into_iter().map(|(name, old, new)| ValueChange { name, old, new }).collect();
    let created = before.get(&i.name).is_none();
    let value = table.get(&i.name).map_or(0.0, |p| p.value);
    s.doc.params = table;
    // a new parameter has no users; a redefinition that moved no value changes nothing downstream,
    // unless the document was loaded with stale values
    let failed = if s.doc.needs_recompute || changes.iter().any(|c| c.old.is_some()) {
        recompute(s)
    } else {
        s.doc.features.iter().filter(|f| f.error.is_some()).count()
    };
    Ok(SetOut {
        name: i.name,
        expression: i.expression,
        value,
        created,
        changes,
        failures: failures(s),
        failed_features: failed,
    })
}

/// One parameter.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NameIn {
    /// Parameter name.
    pub name: String,
}

/// Result of `param.delete`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct DeleteOut {
    /// The parameter removed.
    pub name: String,
}

fn users_text(users: &[ParamUser]) -> String {
    users.iter().map(ToString::to_string).collect::<Vec<_>>().join(", ")
}

/// Refuses while a stored expression does not parse: it cannot be said what it uses.
fn require_integrity(s: &Session) -> CmdResult<()> {
    let bad = integrity_problems(&s.doc);
    if bad.is_empty() {
        return Ok(());
    }
    Err(CmdError::Params(format!("the document holds expressions that do not parse: {}", bad.join(", "))))
}

fn delete(s: &mut Session, i: NameIn) -> CmdResult<DeleteOut> {
    require_integrity(s)?;
    let users = users_of(&s.doc, &i.name);
    if !users.is_empty() {
        return Err(CmdError::Params(format!("parameter `{}` is used by {}", i.name, users_text(&users))));
    }
    s.doc.params = s.doc.params.without(&i.name).map_err(|e| CmdError::Params(e.0))?;
    Ok(DeleteOut { name: i.name })
}

/// Rename parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RenameIn {
    /// Current name.
    pub name: String,
    /// New name.
    pub new_name: String,
}

/// Result of `param.rename`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RenameOut {
    /// The new name.
    pub name: String,
    /// Where the old name was replaced in an expression.
    pub updated: Vec<ParamUser>,
}

fn rename(s: &mut Session, i: RenameIn) -> CmdResult<RenameOut> {
    require_integrity(s)?;
    let table = s.doc.params.renamed(&i.name, &i.new_name).map_err(|e| CmdError::Params(e.0))?;
    let updated = users_of(&s.doc, &i.name);
    let (sketches, features, parked) =
        renamed_references(&s.doc, &table, &i.name, &i.new_name).map_err(|e| CmdError::Params(e.0))?;
    s.doc.params = table;
    s.doc.sketches = sketches;
    s.doc.features = features;
    s.doc.rolled_back = parked;
    Ok(RenameOut { name: i.new_name, updated })
}

/// A parameter as listed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ParamInfo {
    /// Name.
    pub name: String,
    /// Expression text.
    pub expression: String,
    /// Value.
    pub value: f64,
    /// Note.
    pub comment: String,
    /// What uses it: other parameters, sketch dimensions, features (rolled-back ones too).
    pub users: Vec<ParamUser>,
}

/// Result of `param.list`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ListOut {
    /// The parameters in the order they were created.
    pub parameters: Vec<ParamInfo>,
}

fn list(s: &mut Session, _: super::NoParams) -> CmdResult<ListOut> {
    let parameters = s
        .doc
        .params
        .items()
        .iter()
        .map(|p| ParamInfo {
            name: p.name.clone(),
            expression: p.expr.clone(),
            value: p.value,
            comment: p.comment.clone(),
            users: users_of(&s.doc, &p.name),
        })
        .collect();
    Ok(ListOut { parameters })
}

/// Eval parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EvalIn {
    /// Expression to evaluate against the current parameters (nothing is stored).
    pub expression: String,
}

/// Result of `param.eval`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct EvalOut {
    /// The value.
    pub value: f64,
}

fn eval(s: &mut Session, i: EvalIn) -> CmdResult<EvalOut> {
    let value = s.doc.params.eval(&i.expression).map_err(|e| CmdError::Params(e.0))?;
    Ok(EvalOut { value })
}

/// `param.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "param.set",
        "Set parameter",
        "Feature",
        true,
        "Creates or changes parameter `name` = `expression` and recomputes everything that uses it (dimensions given as {\"expr\": ...}, feature fields given as {\"expr\": ...}) as one undo step; a sketch or feature that fails after the change is listed in `failures`.",
        SetIn,
        SetOut,
        set
    ),
    command!(
        "param.delete",
        "Delete parameter",
        "Feature",
        true,
        "Deletes parameter `name`; refused with the list of its users while anything uses it.",
        NameIn,
        DeleteOut,
        delete
    ),
    command!(
        "param.rename",
        "Rename parameter",
        "Feature",
        true,
        "Renames parameter `name` to `new_name` in every expression that uses it.",
        RenameIn,
        RenameOut,
        rename
    ),
    command!(
        "param.list",
        "List parameters",
        "Feature",
        false,
        "The parameters with their expression, value, comment and users.",
        super::NoParams,
        ListOut,
        list
    ),
    command!(
        "param.eval",
        "Evaluate expression",
        "Feature",
        false,
        "Evaluates `expression` against the current parameters without storing it (angles in degrees).",
        EvalIn,
        EvalOut,
        eval
    ),
];
