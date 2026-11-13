//! The feature tree: its nodes, the dependencies between features, and the tree and replay commands.

use super::*;

/// One node of the history.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TreeNode {
    /// Feature id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// Operation.
    pub op: String,
    /// Body written.
    pub body: ObjectId,
    /// Earlier features this one depends on (through its sketch's or bodies' writers).
    pub depends_on: Vec<ObjectId>,
    /// Sketch read, if any.
    pub sketch: Option<ObjectId>,
    /// Suppressed?
    pub suppressed: bool,
    /// Error of the last replay.
    pub error: Option<String>,
    /// Behind the roll-back marker: not replayed.
    #[serde(default)]
    pub rolled_back: bool,
    /// Fields given as an expression over parameters (their values are in the operation).
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub expressions: BTreeMap<String, String>,
}

/// A feature whose last replay failed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FeatureFailure {
    /// Feature id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// Why it failed.
    pub error: String,
}

/// The history.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TreeOut {
    /// Features in order, rolled-back ones last (see [`TreeNode::rolled_back`]).
    pub features: Vec<TreeNode>,
    /// Features that failed.
    pub failed_features: usize,
    /// Which features failed and why (the active ones; a parked feature has not been replayed).
    #[serde(default)]
    pub failures: Vec<FeatureFailure>,
    /// Roll-back marker: the number of active features (equal to the feature count at the end).
    #[serde(default)]
    pub marker: usize,
}

/// The features that feature `f` depends on, among `earlier` (in history order).
fn dependencies(s: &Session, earlier: &[&FeatureObject], f: &FeatureObject) -> (Vec<ObjectId>, Option<ObjectId>) {
    let mut deps = Vec::new();
    let mut sketch = None;
    for input in f.op.inputs() {
        if s.doc.kind_of(input) == Some(ObjectKind::Sketch) {
            sketch = Some(input);
        }
        // the last earlier feature writing that body
        if let Some(w) = earlier.iter().rev().find(|g| g.body == input) {
            deps.push(w.id);
        }
    }
    // a feature also follows the previous writer of its own body
    if let Some(w) = earlier.iter().rev().find(|g| g.body == f.body)
        && !deps.contains(&w.id)
    {
        deps.push(w.id);
    }
    (deps, sketch)
}

pub(super) fn tree_of(s: &Session, failed: usize) -> TreeOut {
    let all: Vec<&FeatureObject> = s.doc.features.iter().chain(s.doc.rolled_back.iter()).collect();
    let marker = s.doc.features.len();
    let nodes = all
        .iter()
        .enumerate()
        .map(|(k, f)| {
            let earlier = all.get(..k).unwrap_or_default();
            let (depends_on, sketch) = dependencies(s, earlier, f);
            TreeNode {
                id: f.id,
                name: f.name.clone(),
                op: f.op.label().into(),
                body: f.body,
                depends_on,
                sketch,
                suppressed: f.suppressed,
                error: f.error.clone(),
                rolled_back: k >= marker,
                expressions: f.exprs.clone(),
            }
        })
        .collect();
    let failures = s
        .doc
        .features
        .iter()
        .filter_map(|f| f.error.as_ref().map(|e| FeatureFailure { id: f.id, name: f.name.clone(), error: e.clone() }))
        .collect();
    TreeOut { features: nodes, failed_features: failed, failures, marker }
}

pub(super) fn tree(s: &mut Session, _: super::NoParams) -> CmdResult<TreeOut> {
    let failed = s.doc.features.iter().filter(|f| f.error.is_some()).count();
    Ok(tree_of(s, failed))
}

pub(super) fn replay_cmd(s: &mut Session, _: super::NoParams) -> CmdResult<TreeOut> {
    let failed = crate::replay::replay(s);
    Ok(tree_of(s, failed))
}
