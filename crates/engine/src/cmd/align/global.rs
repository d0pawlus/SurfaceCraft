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
    /// Move the scans that were placed (default false: the command only proposes the poses). One undo step for all of them. Only the scans held by a closed loop of strong pairs move; a scan that hangs on a single unconfirmed pair (`held_by_loop` false) moves only when `accept_weak` lists it. Refused when nothing could be connected to the fixed scan.
    #[serde(default)]
    pub apply: bool,
    /// With `apply`: the scans (members of `objects`) that hang on a single pair that no loop confirms and that you accept moving anyway, after checking the pair's evidence in `edges` (the way `align.pair_coarse` moves a non-unique pose only when `choose` names it). Without them nothing weakly held moves.
    #[serde(default)]
    pub accept_weak: Vec<ObjRef>,
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
    /// For a placed scan that is not held: the pair `[fixed, moving]` that nothing confirms.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weak_pair: Option<[usize; 2]>,
}

/// The poses, the pair evidence, the loop closure and the verdict.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct GlobalOut {
    /// `aligned` (all scans placed, every pair pose agrees with the final poses within its tolerance), `partial` (some scans not connected, or a loop that does not close) or `failed` (nothing connected to the fixed scan).
    pub verdict: GlobalVerdict,
    /// Reasons for a verdict other than `aligned`, and notes.
    pub issues: Vec<GlobalIssue>,
    /// Were any scans moved? False for a proposal, and for an `apply` where every placed scan was weakly held and not listed in `accept_weak`.
    pub applied: bool,
    /// Indices (in `objects`) of the scans that were moved.
    pub moved: Vec<usize>,
    /// Indices of the placed scans that were left where they are with `apply`, because they hang on one unconfirmed pair and `accept_weak` does not list them.
    pub left_weakly_held: Vec<usize>,
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
    /// How decisive the vote that left pairs out was (inliers of the winning set minus those of the best different one; 0 is a tie; below 2 with 6 or more pairs, below 1 with fewer, the verdict is `partial`); absent when no vote left anything out.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub consensus_margin: Option<usize>,
    /// Trees the vote tried (the strongest-first tree from each scan, then seeded random ones): the vote is sampled, not exhaustive.
    pub consensus_trials: usize,
    /// A sentence about the outcome.
    pub note: String,
}

/// What `apply` did: the scans moved and the weakly held ones left.
struct Applied {
    moved: Vec<usize>,
    left: Vec<usize>,
}

/// The sentence about the outcome; for scans held by one pair it quotes that pair's own evidence.
fn note_of(r: &GlobalResult, apply: bool, done: &Applied, refused: bool) -> String {
    let unplaced = r.scans.iter().filter(|x| !x.placed).count();
    let left = if unplaced > 0 {
        format!(", {unplaced} scan(s) could not be connected and stay where they are")
    } else {
        String::new()
    };
    let head = match (r.verdict, refused) {
        (_, true) => "failed: nothing connects to the fixed scan, nothing was moved".to_string(),
        (GlobalVerdict::Failed, _) => "failed: nothing connects to the fixed scan".to_string(),
        (GlobalVerdict::Aligned, _) => "every scan is held by a loop of strong pairs that agree with each other and with the final poses (this proves consistency, not truth: on a part with a symmetry a loop of wrong pairs can agree as well)".to_string(),
        (GlobalVerdict::Partial, _) => format!("partial{left}"),
    };
    let moved = match (apply, done.moved.is_empty(), refused) {
        (true, false, _) => format!("; moved scans {:?}", done.moved),
        (true, true, false) => "; nothing was moved".to_string(),
        _ => String::new(),
    };
    let held_back = if done.left.is_empty() {
        String::new()
    } else {
        format!("; left in place: {:?} (weakly held: list them in `accept_weak` to move them)", done.left)
    };
    format!("{head}{moved}{held_back}{}", weak_note(r))
}

/// What is known about the scans that hang on a pair nothing confirms (at most three quoted): the
/// pair is quoted with its own evidence, and said to be one pair, unconfirmed by any loop.
fn weak_note(r: &GlobalResult) -> String {
    let weak: Vec<_> = r.scans.iter().filter(|x| x.placed && !x.held_by_loop).collect();
    if weak.is_empty() {
        return String::new();
    }
    let quote = |scan: usize, pair: Option<[usize; 2]>| {
        let e = pair
            .and_then(|[f, m]| r.edges.iter().find(|e| e.status == EdgeStatus::Used && e.fixed == f && e.moving == m));
        e.map_or(format!("scan {scan}"), |e| {
            format!(
                "scan {scan} (pair {}-{}: confidence {:.2}, overlap {:.2}/{:.2}, residual {:.2} % of the diagonal{}, stability {:.3}, {} of {} probes failed, {} slides)",
                e.fixed, e.moving, e.confidence, e.overlap_moving, e.overlap_fixed, 100.0 * e.rms_per_diagonal,
                if e.strong_evidence { "" } else { ", too large to count as evidence" },
                e.stability, e.probes_failed, e.probed, e.slides
            )
        })
    };
    let shown: Vec<String> = weak.iter().take(3).map(|x| quote(x.index, x.weak_pair)).collect();
    let more = if weak.len() > 3 { format!(" and {} more", weak.len() - 3) } else { String::new() };
    format!(
        "; weakly held, on one pair no loop of strong pairs confirms: {}{more}: check them by eye before relying on the pose",
        shown.join("; ")
    )
}

fn resolve_all(s: &Session, i: &GlobalIn) -> CmdResult<(Vec<ObjectId>, usize, Vec<usize>)> {
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
    let mut accepted = Vec::new();
    for a in &i.accept_weak {
        let id = a.resolve(s, ObjectKind::Mesh)?;
        let k = ids
            .iter()
            .position(|x| *x == id)
            .ok_or_else(|| CmdError::Params(format!("`accept_weak` names mesh {id}, which is not in `objects`")))?;
        accepted.push(k);
    }
    Ok((ids, fixed, accepted))
}

fn finite(m: &[[f64; 4]; 4]) -> bool {
    m.iter().flatten().all(|v| v.is_finite())
}

/// Moves the scans that may move (placed, not the fixed one, and held by a loop or listed in
/// `accepted`); every pose is checked first, so nothing moves when one is not finite.
fn move_placed(s: &mut Session, ids: &[ObjectId], r: &GlobalResult, accepted: &[usize]) -> CmdResult<Applied> {
    if r.scans.iter().any(|x| !finite(&x.pose.to_rows())) {
        return Err(CmdError::Params("the relaxation gave a pose that is not finite; nothing was moved".into()));
    }
    let mut done = Applied { moved: Vec::new(), left: Vec::new() };
    for (k, (id, scan)) in ids.iter().zip(&r.scans).enumerate().filter(|(_, (_, x))| x.placed && !x.fixed) {
        if !(scan.held_by_loop || accepted.contains(&k)) {
            done.left.push(k);
            continue;
        }
        let m = s.doc.mesh_mut(*id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
        m.transform_rigid(&scan.pose);
        done.moved.push(k);
    }
    Ok(done)
}

fn scan_outs(ids: &[ObjectId], r: &GlobalResult, moved: &[usize]) -> Vec<GlobalScan> {
    ids.iter()
        .zip(&r.scans)
        .map(|(id, x)| GlobalScan {
            align: out(*id, &x.pose, None, moved.contains(&x.index)),
            index: x.index,
            fixed: x.fixed,
            placed: x.placed,
            unplaced: x.unplaced,
            chained_matrix: x.chained.to_rows(),
            edges_used: x.edges_used,
            held_by_loop: x.held_by_loop,
            weak_pair: x.weak_pair,
        })
        .collect()
}

pub(super) fn global(s: &mut Session, i: GlobalIn) -> CmdResult<GlobalOut> {
    let (ids, fixed, accepted) = resolve_all(s, &i)?;
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
    let done = if i.apply && !refused {
        move_placed(s, &ids, &r, &accepted)?
    } else {
        Applied { moved: Vec::new(), left: Vec::new() }
    };
    if done.moved.is_empty() {
        s.mark_unchanged();
    }
    Ok(GlobalOut {
        note: note_of(&r, i.apply, &done, refused),
        scans: scan_outs(&ids, &r, &done.moved),
        verdict: r.verdict,
        issues: r.issues,
        applied: !done.moved.is_empty(),
        moved: done.moved,
        left_weakly_held: done.left,
        edges: r.edges,
        loops: r.loops,
        pairs_tried: r.pairs_tried,
        pairs_possible: r.pairs_possible,
        iterations: r.iterations,
        used_edges: r.edges_used,
        rejected_edges: r.edges_rejected,
        consensus_margin: r.consensus_margin,
        consensus_trials: r.consensus_trials,
    })
}
