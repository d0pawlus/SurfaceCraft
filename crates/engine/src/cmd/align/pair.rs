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
    /// The motion of the best pose (or of the one `choose` named); `rms` is the residual of the part that matched. `confidence`, the overlaps and `method` are those of this pose.
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
    /// Candidates that could be refined and compared; fewer than two never give a `unique` verdict.
    pub refined: usize,
    /// True when `apply` was given but nothing moved (the verdict is `no_match`, or `ambiguous` without `choose`).
    pub refused: bool,
    /// Iterations of the final ICP (0 for an alternative, which gets none).
    pub icp_iterations: usize,
    /// Did the final ICP converge?
    pub icp_converged: bool,
    /// How firmly the matched part pins the pose: below 0.001 it can slide or turn freely (a flat or cylindrical patch; the verdict is then `ambiguous`); up to 0.04 the pose is pushed along its loose directions and refined again, and a different pose that fits as well makes it `ambiguous`.
    pub stability: f64,
    /// Displaced starts tried for a weakly pinned pose (a pinned pose, or one that slides freely, is not tried).
    pub probed: usize,
    /// Of those, the starts that ended in a different pose that fits as well: slides, listed as alternatives.
    pub slides: usize,
    /// Smallest clearance of the pose over the displaced starts that ended elsewhere (the ambiguity margin 0.03 is the bound); absent when all came back.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub probe_margin: Option<f64>,
    /// Largest distance in mm a displaced start ended from the pose, among those that came back.
    pub probe_return_gap: f64,
    /// A sentence about the outcome.
    pub note: String,
}

fn note_of(r: &PairResult, applied: bool, refused: bool) -> String {
    if r.refined < 2 && r.verdict == PairVerdict::Ambiguous {
        return "only one candidate pose could be refined, so nothing was compared: check the pose by eye".into();
    }
    if r.verdict == PairVerdict::Ambiguous && r.alternatives.is_empty() {
        let tail = if refused { ", nothing was moved" } else { "" };
        return format!(
            "ambiguous: the matched part can slide or turn freely on the target (a flat or cylindrical patch){tail}"
        );
    }
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

/// The pose that is shown and, with `apply`, applied: the best one, or the alternative `choose`
/// named, with that pose's own numbers.
struct Shown {
    transform: Transform,
    rms: f64,
    confidence: f64,
    overlap_moving: f64,
    overlap_fixed: f64,
    method: CoarseMethod,
    icp_iterations: usize,
    icp_converged: bool,
}

impl Shown {
    fn best(r: &PairResult) -> Self {
        Shown {
            transform: r.transform,
            rms: r.rms,
            confidence: r.confidence,
            overlap_moving: r.overlap_moving,
            overlap_fixed: r.overlap_fixed,
            method: r.method,
            icp_iterations: r.icp_iterations,
            icp_converged: r.icp_converged,
        }
    }

    /// An alternative: the final ICP only runs for the best pose.
    fn alternative(a: &PairCandidate) -> Self {
        Shown {
            transform: a.transform,
            rms: a.rms,
            confidence: a.confidence,
            overlap_moving: a.overlap_moving,
            overlap_fixed: a.overlap_fixed,
            method: a.method,
            icp_iterations: 0,
            icp_converged: false,
        }
    }
}

/// What `apply` does: the pose to apply (`None`: nothing may move) and the pose shown.
fn pose_to_apply(r: &PairResult, choose: Option<usize>) -> CmdResult<(Shown, bool)> {
    match (r.verdict, choose) {
        (PairVerdict::NoMatch, _) => Ok((Shown::best(r), false)),
        (PairVerdict::Unique, None | Some(0)) => Ok((Shown::best(r), true)),
        (PairVerdict::Unique, Some(_)) => {
            Err(CmdError::Params("`choose` picks among alternatives, and this verdict is unique (use 0)".into()))
        }
        (PairVerdict::Ambiguous, None) => Ok((Shown::best(r), false)),
        (PairVerdict::Ambiguous, Some(0)) => Ok((Shown::best(r), true)),
        (PairVerdict::Ambiguous, Some(k)) => {
            r.alternatives.get(k - 1).map(|a| (Shown::alternative(a), true)).ok_or_else(|| {
                CmdError::Params(format!("`choose` is {k} but there are {} alternatives", r.alternatives.len()))
            })
        }
    }
}

pub(super) fn pair(s: &mut Session, i: PairIn) -> CmdResult<PairOut> {
    if i.choose.is_some() && !i.apply {
        return Err(CmdError::Params("`choose` names the pose to apply: give `apply` too".into()));
    }
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let moving = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let fixed = target_mesh(s, &i.target)?;
    let r = match pair_coarse(&moving, &fixed, &PairOptions { tolerance: i.tolerance }, &s.cancel) {
        Ok(r) => r,
        Err(AlignError::Cancelled) => return Err(crate::cmd::cancelled()),
        Err(e) => return Err(e.into()),
    };
    let (shown, may_move) = if i.apply { pose_to_apply(&r, i.choose)? } else { (Shown::best(&r), false) };
    apply_to(s, id, &i.also, &shown.transform, i.apply && may_move)?;
    let refused = i.apply && !may_move;
    Ok(PairOut {
        align: out(id, &shown.transform, Some(shown.rms), i.apply && may_move),
        verdict: r.verdict,
        confident: r.verdict == PairVerdict::Unique,
        confidence: shown.confidence,
        overlap_moving: shown.overlap_moving,
        overlap_fixed: shown.overlap_fixed,
        tolerance: r.tolerance,
        method: shown.method,
        refused,
        note: note_of(&r, i.apply && may_move, refused),
        alternatives: r.alternatives,
        candidates: r.candidates,
        refined: r.refined,
        stability: r.stability,
        probed: r.probed,
        slides: r.slides,
        probe_margin: r.probe_margin,
        probe_return_gap: r.probe_return_gap,
        icp_iterations: shown.icp_iterations,
        icp_converged: shown.icp_converged,
    })
}
