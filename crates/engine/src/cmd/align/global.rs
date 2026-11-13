//! `align.global`: several scans of one part into one common pose at once (design doc §6.1, P9-M3).
use super::{AlignOut, out};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_align::global::{EdgeReport, GlobalIssue, GlobalOptions, LoopReport, MAX_SCANS, StartMode, Unplaced};
use surfacecraft_align::{AlignError, GlobalVerdict, align_global};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::Mesh;

/// Global alignment parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GlobalIn {
    /// The scans of the part, 2 to 24 meshes, each listed once.
    pub objects: Vec<ObjRef>,
    /// The scan that stays where it is (default: the first of `objects`).
    #[serde(default)]
    pub fixed: Option<ObjRef>,
    /// Move the scans that were placed (default false: the command only proposes the poses). One undo step for all of them; refused when nothing could be connected to the fixed scan.
    #[serde(default)]
    pub apply: bool,
    /// `search` (default): the full pair search for every pair, no assumption about where the scans lie. `current`: the scans are already roughly in place, every pair starts from the current poses.
    #[serde(default)]
    pub start: StartMode,
    /// Distance in mm within which scans count as coincident (default: found for every pair from its own residual).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Most pairs tried (default 36; with `n` scans there are `n (n - 1) / 2`, and when the cap cuts the list the pairs of neighbours in `objects` come first). Raise it when scans are listed in no particular order and the result is `partial`.
    #[serde(default)]
    pub max_pairs: Option<usize>,
}

/// One scan's pose.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct GlobalScan {
    /// The motion that brings the scan into the common frame (identity when `placed` is false).
    #[serde(flatten)]
    pub align: AlignOut,
    /// Position in `objects`.
    pub index: usize,
    /// Is it the fixed scan?
    pub fixed: bool,
    /// Was the scan connected to the fixed one? An unplaced scan is not moved.
    pub placed: bool,
    /// Why it was not placed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unplaced: Option<Unplaced>,
    /// The same motion chained along the spanning tree only, before relaxation (4x4, row-major).
    pub chained_matrix: [[f64; 4]; 4],
    /// Pair poses that tie the scan into the graph.
    pub edges_used: usize,
}

/// The poses, the pair evidence, the loop closure and the verdict.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct GlobalOut {
    /// `aligned` (all scans placed, every pair pose agrees with the final poses within its tolerance), `partial` (some scans not connected, or a loop that does not close) or `failed` (nothing connected to the fixed scan).
    pub verdict: GlobalVerdict,
    /// Reasons for a verdict other than `aligned`, and notes.
    pub issues: Vec<GlobalIssue>,
    /// Were the scans moved?
    pub applied: bool,
    /// One entry per scan, in the order of `objects`.
    pub scans: Vec<GlobalScan>,
    /// Every pair that was tried: verdict, overlap, residual, whether it became an edge of the pose graph, and its misfit before and after relaxation.
    pub edges: Vec<EdgeReport>,
    /// Loop closure error before and after relaxation, over the independent cycles of the graph.
    #[serde(rename = "loop")]
    pub loops: LoopReport,
    /// Pairs tried.
    pub pairs_tried: usize,
    /// Pairs there were to try.
    pub pairs_possible: usize,
    /// Relaxation iterations.
    pub iterations: usize,
    /// A sentence about the outcome.
    pub note: String,
}

fn note_of(verdict: GlobalVerdict, applied: bool, refused: bool, unplaced: usize) -> String {
    match (verdict, applied, refused) {
        (GlobalVerdict::Aligned, true, _) => "every scan is placed and the pair poses agree: applied".into(),
        (GlobalVerdict::Aligned, false, _) => "every scan is placed and the pair poses agree".into(),
        (GlobalVerdict::Partial, true, _) => {
            format!(
                "partial: the placed scans were moved, {unplaced} scan(s) could not be connected and stay where they are"
            )
        }
        (GlobalVerdict::Partial, false, _) => {
            format!("partial: {unplaced} scan(s) could not be connected, or a loop does not close; check the edges")
        }
        (GlobalVerdict::Failed, _, true) => "failed: nothing connects to the fixed scan, nothing was moved".into(),
        (GlobalVerdict::Failed, _, false) => "failed: nothing connects to the fixed scan".into(),
    }
}

fn resolve_all(s: &Session, i: &GlobalIn) -> CmdResult<(Vec<ObjectId>, usize)> {
    if i.objects.len() > MAX_SCANS {
        return Err(CmdError::Params(format!(
            "{} scans given; at most {MAX_SCANS} are taken at once",
            i.objects.len()
        )));
    }
    let mut ids = Vec::with_capacity(i.objects.len());
    for o in &i.objects {
        let id = o.resolve(s, ObjectKind::Mesh)?;
        if ids.contains(&id) {
            return Err(CmdError::Params(format!("mesh {id} is listed twice")));
        }
        ids.push(id);
    }
    let fixed = match &i.fixed {
        None => 0,
        Some(f) => {
            let id = f.resolve(s, ObjectKind::Mesh)?;
            ids.iter()
                .position(|x| *x == id)
                .ok_or_else(|| CmdError::Params(format!("the fixed mesh {id} is not in `objects`")))?
        }
    };
    Ok((ids, fixed))
}

pub(super) fn global(s: &mut Session, i: GlobalIn) -> CmdResult<GlobalOut> {
    let (ids, fixed) = resolve_all(s, &i)?;
    let meshes: Vec<Arc<Mesh>> = ids
        .iter()
        .map(|id| s.doc.mesh(*id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}"))))
        .collect::<CmdResult<_>>()?;
    let refs: Vec<&Mesh> = meshes.iter().map(|m| m.as_ref()).collect();
    let defaults = GlobalOptions::default();
    let opts = GlobalOptions {
        fixed,
        tolerance: i.tolerance,
        start: i.start,
        max_pairs: i.max_pairs.unwrap_or(defaults.max_pairs),
    };
    let r = match align_global(&refs, &opts, &s.cancel) {
        Ok(r) => r,
        Err(AlignError::Cancelled) => return Err(crate::cmd::cancelled()),
        Err(e) => return Err(e.into()),
    };
    let refused = i.apply && r.verdict == GlobalVerdict::Failed;
    let applied = i.apply && !refused;
    if applied {
        for (id, scan) in ids.iter().zip(&r.scans) {
            if scan.placed && !scan.fixed {
                let m = s.doc.mesh_mut(*id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
                m.transform_rigid(&scan.pose);
            }
        }
    } else {
        s.mark_unchanged();
    }
    let unplaced = r.scans.iter().filter(|x| !x.placed).count();
    let scans = ids
        .iter()
        .zip(&r.scans)
        .map(|(id, x)| GlobalScan {
            align: out(*id, &x.pose, None, applied && x.placed && !x.fixed),
            index: x.index,
            fixed: x.fixed,
            placed: x.placed,
            unplaced: x.unplaced,
            chained_matrix: x.chained.to_rows(),
            edges_used: x.edges_used,
        })
        .collect();
    Ok(GlobalOut {
        verdict: r.verdict,
        issues: r.issues,
        applied,
        scans,
        edges: r.edges,
        loops: r.loops,
        pairs_tried: r.pairs_tried,
        pairs_possible: r.pairs_possible,
        iterations: r.iterations,
        note: note_of(r.verdict, applied, refused, unplaced),
    })
}
