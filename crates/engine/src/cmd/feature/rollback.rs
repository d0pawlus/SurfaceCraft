//! `feature.rollback`: the roll-back marker of the history. Features after the marker are parked in
//! `Document::rolled_back`: they are not replayed, so every body is what it was at that step. A
//! feature added while rolled back is appended to the active part, which puts it at the marker and
//! keeps the parked features after it. Moving the marker to the end brings every parked feature back.
use super::{TreeOut, tree_of};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{FeatureObject, ObjectId};

/// Where the marker goes.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum RollbackTo {
    /// Every feature is active again (`"end"`).
    End,
    /// The first rolled-back feature is at this position of the history (`{"index": n}`; `n` equal
    /// to the number of features means the end).
    Index(usize),
    /// The first rolled-back feature, by id or name, active or parked (`{"feature": ref}`).
    Feature(ObjRef),
}

/// Roll-back parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RollbackIn {
    /// The new marker position.
    pub to: RollbackTo,
}

/// The features of the history in order, active first, then the parked ones.
fn all_features(s: &Session) -> Vec<FeatureObject> {
    s.doc.features.iter().chain(s.doc.rolled_back.iter()).cloned().collect()
}

/// The position a `to` names, in the combined list (`all.len()` is the end).
fn target_index(all: &[FeatureObject], to: &RollbackTo) -> CmdResult<usize> {
    match to {
        RollbackTo::End => Ok(all.len()),
        RollbackTo::Index(i) if *i <= all.len() => Ok(*i),
        RollbackTo::Index(i) => Err(CmdError::Params(format!("there is no feature at position {i}"))),
        RollbackTo::Feature(r) => {
            let found = match r {
                ObjRef::Id(n) => all.iter().position(|f| f.id == ObjectId(*n)),
                ObjRef::Name(n) => all.iter().position(|f| f.name == *n),
            };
            found.ok_or_else(|| CmdError::Params("no such feature in the history".into()))
        }
    }
}

/// Splits the combined list at `at`; `None` when `at` is past the end (checked, never panics).
fn split(all: &[FeatureObject], at: usize) -> Option<(Vec<FeatureObject>, Vec<FeatureObject>)> {
    Some((all.get(..at)?.to_vec(), all.get(at..)?.to_vec()))
}

/// Bodies written only by parked features: they did not exist at the marker, so they leave the
/// model (replay would otherwise keep their last geometry, since it only updates written bodies).
fn orphan_bodies(active: &[FeatureObject], parked: &[FeatureObject]) -> Vec<ObjectId> {
    parked.iter().map(|f| f.body).filter(|body| !active.iter().any(|f| f.body == *body)).collect()
}

/// Moves the bodies written only by parked features out of the model (with their name and flags,
/// into `Document::rolled_back_bodies`) and brings back the ones the active features write again.
fn park_bodies(s: &mut Session, orphans: &[ObjectId]) {
    let all: Vec<_> = s.doc.bodies.drain(..).chain(s.doc.rolled_back_bodies.drain(..)).collect();
    let (parked, live): (Vec<_>, Vec<_>) = all.into_iter().partition(|b| orphans.contains(&b.id));
    s.doc.bodies = live;
    s.doc.rolled_back_bodies = parked;
}

/// Moves the marker and replays the active part.
pub(super) fn rollback(s: &mut Session, i: RollbackIn) -> CmdResult<TreeOut> {
    let all = all_features(s);
    let at = target_index(&all, &i.to)?;
    let (active, parked) = split(&all, at).ok_or_else(|| CmdError::Params("the marker is past the end".into()))?;
    let orphans = orphan_bodies(&active, &parked);
    s.doc.features = active;
    s.doc.rolled_back = parked;
    park_bodies(s, &orphans);
    let failed = crate::replay::replay(s);
    Ok(tree_of(s, failed))
}
