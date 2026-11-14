//! `align.global`: several scans of one part into one common pose at once (design doc §6.1, P9-M3).
use super::{AlignOut, out};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_align::global::{
    EdgeReport, EdgeStatus, GlobalIssue, GlobalOptions, GlobalResult, LoopReport, MAX_SCANS, StartMode, Unplaced,
};
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
    /// Is the scan held by a closed loop? False when it hangs on a single pair (two scans, a chain, a leaf) that no other pair confirms: then the pose is only as good as that one pair's own evidence (see its entry in `edges`), and the verdict cannot be `aligned`.
    pub held_by_loop: bool,
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
    /// Pairs used as edges.
    pub used_edges: usize,
    /// Unique pairs left out because they contradicted the others; their misfit at the final poses is in `edges`.
    pub rejected_edges: usize,
    /// How decisive the vote that left pairs out was (inliers of the winning set minus those of the best different one; 0 is a tie, which makes the verdict `partial`); absent when the vote left nothing out.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub consensus_margin: Option<usize>,
    /// A sentence about the outcome.
    pub note: String,
}

/// The sentence about the outcome; for scans held by one pair it quotes that pair's own evidence.
fn note_of(r: &GlobalResult, applied: bool, refused: bool) -> String {
    let unplaced = r.scans.iter().filter(|x| !x.placed).count();
    let weak = weak_note(r);
    let left = if unplaced > 0 {
        format!(", {unplaced} scan(s) could not be connected and stay where they are")
    } else {
        String::new()
    };
    let head = match (r.verdict, applied, refused) {
        (GlobalVerdict::Aligned, true, _) => {
            "every scan is placed, held by a loop of pairs that agree: applied".to_string()
        }
        (GlobalVerdict::Aligned, false, _) => "every scan is placed, held by a loop of pairs that agree".to_string(),
        (GlobalVerdict::Partial, true, _) => format!("partial: the placed scans were moved{left}"),
        (GlobalVerdict::Partial, false, _) => format!("partial: nothing was moved (proposal){left}"),
        (GlobalVerdict::Failed, _, true) => "failed: nothing connects to the fixed scan, nothing was moved".to_string(),
        (GlobalVerdict::Failed, _, false) => "failed: nothing connects to the fixed scan".to_string(),
    };
    format!("{head}{weak}")
}

/// What is known about the scans that hang on a single pair: it is one pair, unconfirmed by any loop.
fn weak_note(r: &GlobalResult) -> String {
    let weak: Vec<usize> = r.scans.iter().filter(|x| x.placed && !x.held_by_loop).map(|x| x.index).collect();
    let Some(&first) = weak.first() else { return String::new() };
    let pair = r.edges.iter().find(|e| e.status == EdgeStatus::Used && (e.fixed == first || e.moving == first));
    let evidence = pair.map_or(String::new(), |e| {
        format!(
            " (scan {first}: pair {}-{}, confidence {:.2}, overlap {:.2}/{:.2}, stability {:.3}, {} of {} probes failed, {} slides)",
            e.fixed, e.moving, e.confidence, e.overlap_moving, e.overlap_fixed, e.stability, e.probes_failed, e.probed, e.slides
        )
    });
    format!(
        "; scan(s) {weak:?} hang on a single pair that no loop confirms{evidence}: check them by eye before relying on the pose"
    )
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

fn finite(m: &[[f64; 4]; 4]) -> bool {
    m.iter().flatten().all(|v| v.is_finite())
}

/// Moves the placed scans (the fixed one stays) by their poses; every pose is checked first, so
/// nothing moves when one is not finite.
fn move_placed(s: &mut Session, ids: &[ObjectId], r: &GlobalResult) -> CmdResult<()> {
    if r.scans.iter().any(|x| !finite(&x.pose.to_rows())) {
        return Err(CmdError::Params("the relaxation gave a pose that is not finite; nothing was moved".into()));
    }
    for (id, scan) in ids.iter().zip(&r.scans).filter(|(_, x)| x.placed && !x.fixed) {
        let m = s.doc.mesh_mut(*id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
        m.transform_rigid(&scan.pose);
    }
    Ok(())
}

fn scan_outs(ids: &[ObjectId], r: &GlobalResult, applied: bool) -> Vec<GlobalScan> {
    ids.iter()
        .zip(&r.scans)
        .map(|(id, x)| GlobalScan {
            align: out(*id, &x.pose, None, applied && x.placed && !x.fixed),
            index: x.index,
            fixed: x.fixed,
            placed: x.placed,
            unplaced: x.unplaced,
            chained_matrix: x.chained.to_rows(),
            edges_used: x.edges_used,
            held_by_loop: x.held_by_loop,
        })
        .collect()
}

pub(super) fn global(s: &mut Session, i: GlobalIn) -> CmdResult<GlobalOut> {
    let (ids, fixed) = resolve_all(s, &i)?;
    let meshes: Vec<Arc<Mesh>> = ids
        .iter()
        .map(|id| s.doc.mesh(*id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}"))))
        .collect::<CmdResult<_>>()?;
    let refs: Vec<&Mesh> = meshes.iter().map(|m| m.as_ref()).collect();
    let max_pairs = i.max_pairs.unwrap_or(GlobalOptions::default().max_pairs);
    let opts = GlobalOptions { fixed, tolerance: i.tolerance, start: i.start, max_pairs };
    let r = match align_global(&refs, &opts, &s.cancel) {
        Ok(r) => r,
        Err(AlignError::Cancelled) => return Err(crate::cmd::cancelled()),
        Err(e) => return Err(e.into()),
    };
    let refused = i.apply && r.verdict == GlobalVerdict::Failed;
    let applied = i.apply && !refused;
    if applied {
        move_placed(s, &ids, &r)?;
    } else {
        s.mark_unchanged();
    }
    Ok(GlobalOut {
        note: note_of(&r, applied, refused),
        scans: scan_outs(&ids, &r, applied),
        verdict: r.verdict,
        issues: r.issues,
        applied,
        edges: r.edges,
        loops: r.loops,
        pairs_tried: r.pairs_tried,
        pairs_possible: r.pairs_possible,
        iterations: r.iterations,
        used_edges: r.edges_used,
        rejected_edges: r.edges_rejected,
        consensus_margin: r.consensus_margin,
    })
}
