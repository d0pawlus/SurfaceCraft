//! `feature.*`: parametric features and the history DAG (design doc §6.4, milestone M5).
use super::{CmdError, CmdResult, CommandSpec, NoParams, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use surfacecraft_doc::{BoolOp, FeatureObject, FeatureOp, Mode, ObjectId, ObjectKind, Side};

mod exprs;
use exprs::ClearedExpr;
mod ops;
mod rollback;
mod shapes;
mod suggest;
mod tree;
use super::param::num::{Count, ExprSet, Num, zero3};
use ops::*;
use shapes::*;
use std::collections::BTreeMap;
use suggest::*;
use surfacecraft_geom::{Point3, Vec3};
use tree::*;

/// Result of a feature command.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FeatureOut {
    /// Feature id.
    pub id: ObjectId,
    /// Body it writes.
    pub body: ObjectId,
    /// Body volume after the replay (mm³).
    pub volume: Option<f64>,
    /// Faces of the body.
    pub faces: usize,
    /// Edges of the body.
    pub edges: usize,
    /// This feature's error, if it failed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Features that failed in the replay.
    pub failed_features: usize,
    /// For `feature.edit`: the expressions the edit replaced with plain numbers.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub expressions_cleared: Vec<ClearedExpr>,
}

fn report(s: &Session, id: ObjectId, failed: usize) -> CmdResult<FeatureOut> {
    let f = s.doc.feature(id).ok_or_else(|| CmdError::Params(format!("no feature {id}")))?;
    let body = f.body;
    let (volume, faces, edges) = s.doc.body(body).map_or((None, 0, 0), |b| {
        (surfacecraft_kernel::solid::volume(&b.brep).ok(), b.brep.faces.len(), b.brep.edges.len())
    });
    Ok(FeatureOut {
        id,
        body,
        volume,
        faces,
        edges,
        error: f.error.clone(),
        failed_features: failed,
        expressions_cleared: Vec::new(),
    })
}

/// Makes sure a body that did not come from a feature enters the history as a base feature.
fn ensure_history(s: &mut Session, body: ObjectId) -> CmdResult<()> {
    if s.doc.features.iter().any(|f| f.body == body) {
        return Ok(());
    }
    let b = s.doc.body(body).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let (name, brep) = (format!("base {}", b.name), b.brep.clone());
    s.doc.add_feature(&name, FeatureOp::Base { brep }, body);
    Ok(())
}

/// Appends a feature and replays; a feature that fails on creation is refused (rolled back).
fn push(
    s: &mut Session,
    name: &str,
    (op, exprs): (FeatureOp, BTreeMap<String, String>),
    body: Option<ObjectId>,
    body_name: Option<&str>,
) -> CmdResult<FeatureOut> {
    for input in op.inputs() {
        if s.doc.kind_of(input) == Some(ObjectKind::Body) {
            ensure_history(s, input)?;
        }
    }
    let body = match body {
        Some(b) => b,
        None => s.doc.fresh_id(),
    };
    let fid = s.doc.add_feature(name, op, body);
    if let Some(f) = s.doc.feature_mut(fid) {
        f.exprs = exprs;
    }
    let failed = crate::replay::replay(s);
    if let Some(e) = s.doc.feature(fid).and_then(|f| f.error.clone()) {
        return Err(CmdError::Failed(format!("{name} failed: {e}")));
    }
    if let Some(n) = body_name
        && s.doc.body(body).is_some()
    {
        s.doc.rename(body, n);
    }
    report(s, fid, failed)
}

fn resolve_body(s: &Session, r: &ObjRef) -> CmdResult<ObjectId> {
    r.resolve(s, ObjectKind::Body)
}

/// `feature.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "feature.loft",
        "Loft",
        "Feature",
        true,
        "Skins a solid through the outer profiles of `sketches` (degree 1 = ruled), new/add/cut.",
        LoftIn,
        FeatureOut,
        loft
    ),
    command!(
        "feature.sweep",
        "Sweep",
        "Feature",
        true,
        "Sweeps the profile of `profile` along the open path of `path` (follow = turns with the path), new/add/cut.",
        SweepIn,
        FeatureOut,
        sweep
    ),
    command!(
        "feature.draft",
        "Draft",
        "Feature",
        true,
        "Tilts `faces` of `body` by `angle_deg` about the neutral plane, leaning in along `pull` (parametric, faces tracked by signature).",
        DraftIn,
        FeatureOut,
        draft
    ),
    command!(
        "feature.mirror",
        "Mirror",
        "Feature",
        true,
        "Reflects `body` in a plane and unites the copy with it (or keeps only the reflection).",
        MirrorIn,
        FeatureOut,
        mirror
    ),
    command!(
        "feature.split",
        "Split / trim",
        "Feature",
        true,
        "Cuts `body` by a plane and keeps the `front` or `back` side.",
        SplitIn,
        FeatureOut,
        split
    ),
    command!(
        "feature.path_pattern",
        "Pattern along path",
        "Feature",
        true,
        "`count` instances of `body` evenly spaced along the open path sketch `path` (optionally turning with it).",
        PathPatternIn,
        FeatureOut,
        path_pattern
    ),
    command!(
        "feature.suggest_axis",
        "Suggest revolve axis",
        "Feature",
        false,
        "Axis (origin and direction) of the turned scan surface of `region`, for `feature.revolve` (redesign assistant).",
        SuggestAxisIn,
        SuggestAxisOut,
        suggest_axis
    ),
    command!(
        "feature.suggest_draft",
        "Suggest draft angle",
        "Feature",
        false,
        "Draft angle of the scanned side face of `region` against the `pull` direction, for `feature.draft` (redesign assistant).",
        SuggestDraftIn,
        SuggestDraftOut,
        suggest_draft
    ),
    command!(
        "feature.rollback",
        "Roll back",
        "Feature",
        true,
        "Moves the roll-back marker of the history to `to` (`\"end\"`, `{\"index\": n}` or `{\"feature\": ref}`): the features from there on are parked and not replayed, so the bodies are as they were at that step; features added now go before the parked ones; `end` restores every feature. Reports the marker and each feature's `rolled_back`.",
        rollback::RollbackIn,
        TreeOut,
        rollback::rollback
    ),
    command!(
        "feature.extrude",
        "Extrude",
        "Feature",
        true,
        "Extrudes the closed profiles of `sketch` by `distance` (draft, new/add/cut on `target`).",
        ExtrudeIn,
        FeatureOut,
        extrude
    ),
    command!(
        "feature.revolve",
        "Revolve",
        "Feature",
        true,
        "Revolves the first profile of `sketch` about an axis by `angle_deg` (new/add/cut).",
        RevolveIn,
        FeatureOut,
        revolve
    ),
    command!(
        "feature.fillet",
        "Fillet",
        "Feature",
        true,
        "Rounds `edges` of `body` with radius `size` (parametric).",
        BlendIn,
        FeatureOut,
        fillet
    ),
    command!(
        "feature.chamfer",
        "Chamfer",
        "Feature",
        true,
        "Bevels `edges` of `body` by `size` (parametric).",
        BlendIn,
        FeatureOut,
        chamfer
    ),
    command!(
        "feature.shell",
        "Shell",
        "Feature",
        true,
        "Hollows `body`, removing `faces`, walls of `thickness`.",
        ShellIn,
        FeatureOut,
        shell
    ),
    command!(
        "feature.pattern",
        "Pattern",
        "Feature",
        true,
        "`count` instances of `body`, each moved by `translate` and/or rotated by `rotate_deg` about `rotate_axis`.",
        PatternIn,
        FeatureOut,
        pattern
    ),
    command!(
        "feature.boolean",
        "Boolean",
        "Feature",
        true,
        "Union/subtract/intersect `tool` with `body`.",
        BooleanIn,
        FeatureOut,
        boolean
    ),
    command!(
        "feature.edit",
        "Edit feature",
        "Feature",
        true,
        "Changes parameters `set` of `feature` and replays the history.",
        EditIn,
        FeatureOut,
        edit
    ),
    command!(
        "feature.suppress",
        "Suppress",
        "Feature",
        true,
        "Suppresses or restores `feature` and replays.",
        SuppressIn,
        FeatureOut,
        suppress
    ),
    command!(
        "feature.tree",
        "History",
        "Feature",
        false,
        "The feature history with dependencies and errors.",
        super::NoParams,
        TreeOut,
        tree
    ),
    command!(
        "feature.replay",
        "Replay history",
        "Feature",
        true,
        "Rebuilds every feature-made body from the history.",
        super::NoParams,
        TreeOut,
        replay_cmd
    ),
    command!(
        "feature.suggest_fillet",
        "Suggest fillet radius",
        "Feature",
        false,
        "Radius of the rounded scan around `edge` of `body`, measured on `mesh` (redesign assistant).",
        SuggestIn,
        SuggestOut,
        suggest_fillet
    ),
];
