//! `align.pair_coarse`: two scans of one part into one pose with no picked points (design doc
//! §6.1, P9-M2).
use super::{AlignOut, apply_to, out, target_mesh};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_align::pair::CoarseMethod;
use surfacecraft_align::{AlignError, PairCandidate, PairOptions, PairResult, PairVerdict, pair_coarse};
use surfacecraft_doc::ObjectKind;
use surfacecraft_geom::Transform;

/// Pair alignment parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PairIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// Mesh or body of the other scan, which stays where it is.
    pub target: ObjRef,
    /// Distance in mm within which a point counts as lying on the other scan (default: found from the residual of the best pose, 0.15 % to 2 % of the target's diagonal).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Move the mesh (default false: the command only proposes the pose). A pose that is not `unique` is not applied unless `choose` names it.
    #[serde(default)]
    pub apply: bool,
    /// With `apply` and an `ambiguous` verdict: the pose to apply, 0 for the best one, 1 and up for the alternatives in the order listed.
    #[serde(default)]
    pub choose: Option<usize>,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
}

/// The proposed (or applied) pose, the evidence and the verdict.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PairOut {
    /// The motion of the best pose (or of the one `choose` named); `rms` is the residual of the part that matched.
    #[serde(flatten)]
    pub align: AlignOut,
    /// `unique`, `ambiguous` (see `alternatives`) or `no_match` (too little overlap, or different parts).
    pub verdict: PairVerdict,
    /// Is the verdict `unique`? When false, check the proposal by eye.
    pub confident: bool,
    /// 0..1: how far the best pose explains the moving scan better than any different pose; certain from 0.5, 0 for no match.
    pub confidence: f64,
    /// Share (0..1) of the moving scan's area that lies on the target within the tolerance.
    pub overlap_moving: f64,
    /// Share (0..1) of the target's area that lies on the moving scan within the tolerance.
    pub overlap_fixed: f64,
    /// Tolerance used, mm.
    pub tolerance: f64,
    /// Which stage proposed the best pose.
    pub method: CoarseMethod,
    /// Different poses that fit about as well as the best one (only for an `ambiguous` verdict), best first.
    pub alternatives: Vec<PairCandidate>,
    /// Candidate poses scored.
    pub candidates: usize,
    /// Iterations of the final ICP.
    pub icp_iterations: usize,
    /// Did the final ICP converge?
    pub icp_converged: bool,
    /// A sentence about the outcome.
    pub note: String,
}

fn note_of(r: &PairResult, applied: bool, refused: bool) -> String {
    match (r.verdict, applied, refused) {
        (PairVerdict::Unique, true, _) => "one pose fits clearly better than any other: applied".into(),
        (PairVerdict::Unique, false, _) => "one pose fits clearly better than any other".into(),
        (PairVerdict::Ambiguous, true, _) => "ambiguous: the pose you chose was applied".into(),
        (PairVerdict::Ambiguous, false, true) => {
            "ambiguous: several poses fit about equally well, nothing was moved; apply one with `choose`".into()
        }
        (PairVerdict::Ambiguous, false, false) => {
            "ambiguous: several poses fit about equally well, check the alternatives".into()
        }
        (PairVerdict::NoMatch, _, true) => "no match: too little overlap or different parts, nothing was moved".into(),
        (PairVerdict::NoMatch, _, false) => "no match: too little overlap or different parts".into(),
    }
}

/// The pose to apply: the best one when it is unique (or `choose` names it), an alternative when
/// `choose` names one; `None` when nothing may move.
fn pose_to_apply(r: &PairResult, choose: Option<usize>) -> CmdResult<Option<Transform>> {
    match (r.verdict, choose) {
        (PairVerdict::NoMatch, _) => Ok(None),
        (PairVerdict::Unique, None | Some(0)) => Ok(Some(r.transform)),
        (PairVerdict::Unique, Some(_)) => {
            Err(CmdError::Params("`choose` picks among alternatives, and this verdict is unique (use 0)".into()))
        }
        (PairVerdict::Ambiguous, None) => Ok(None),
        (PairVerdict::Ambiguous, Some(0)) => Ok(Some(r.transform)),
        (PairVerdict::Ambiguous, Some(k)) => r.alternatives.get(k - 1).map(|a| Some(a.transform)).ok_or_else(|| {
            CmdError::Params(format!("`choose` is {k} but there are {} alternatives", r.alternatives.len()))
        }),
    }
}

pub(super) fn pair(s: &mut Session, i: PairIn) -> CmdResult<PairOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let moving = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let fixed = target_mesh(s, &i.target)?;
    let r = match pair_coarse(&moving, &fixed, &PairOptions { tolerance: i.tolerance }, &s.cancel) {
        Ok(r) => r,
        Err(AlignError::Cancelled) => return Err(crate::cmd::cancelled()),
        Err(e) => return Err(e.into()),
    };
    let chosen = if i.apply { pose_to_apply(&r, i.choose)? } else { None };
    let shown = chosen.unwrap_or(r.transform);
    apply_to(s, id, &i.also, &shown, chosen.is_some())?;
    let refused = i.apply && chosen.is_none();
    Ok(PairOut {
        align: out(id, &shown, Some(r.rms), chosen.is_some()),
        verdict: r.verdict,
        confident: r.verdict == PairVerdict::Unique,
        confidence: r.confidence,
        overlap_moving: r.overlap_moving,
        overlap_fixed: r.overlap_fixed,
        tolerance: r.tolerance,
        method: r.method,
        note: note_of(&r, chosen.is_some(), refused),
        alternatives: r.alternatives,
        candidates: r.candidates,
        icp_iterations: r.icp_iterations,
        icp_converged: r.icp_converged,
    })
}
