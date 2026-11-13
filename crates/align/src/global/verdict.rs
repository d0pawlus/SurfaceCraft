//! Per-scan reports and the overall verdict.
use super::report::{EdgeStatus, GlobalIssue, GlobalResult, GlobalVerdict, LoopReport, ScanPose, Unplaced};
use super::solve::{Cand, Solved};
use super::{GlobalOptions, LOOP_BOUND};
use surfacecraft_geom::Transform;

fn unplaced_reason(k: usize, cands: &[Cand]) -> Unplaced {
    let mine = || cands.iter().filter(move |c| c.report.fixed == k || c.report.moving == k);
    if mine().any(|c| c.report.status == EdgeStatus::Unplaced) {
        Unplaced::OtherGroup
    } else if mine().next().is_some() {
        Unplaced::NoUniquePair
    } else {
        Unplaced::PairsNotTried
    }
}

fn scan_poses(n: usize, opts: &GlobalOptions, cands: &[Cand], solved: &Solved) -> Vec<ScanPose> {
    (0..n)
        .map(|k| {
            let placed = solved.tree.reaches(k);
            let pick = |poses: &[Transform]| if placed { poses.get(k).copied().unwrap_or_default() } else { Transform::identity() };
            let edges_used = solved.edges.iter().filter(|e| e.i == k || e.j == k).count();
            ScanPose {
                index: k,
                fixed: k == opts.fixed,
                placed,
                unplaced: (!placed).then(|| unplaced_reason(k, cands)),
                pose: pick(&solved.poses),
                chained: pick(&solved.chained),
                edges_used,
            }
        })
        .collect()
}

/// `aligned` only when every scan is placed and no used edge disagrees with the final poses by more than [`LOOP_BOUND`] tolerances.
fn judge(scans: &[ScanPose], loops: &LoopReport) -> (GlobalVerdict, Vec<GlobalIssue>) {
    let placed_others = scans.iter().filter(|s| s.placed && !s.fixed).count();
    if placed_others == 0 {
        return (GlobalVerdict::Failed, vec![GlobalIssue::NotConnected]);
    }
    if scans.iter().any(|s| !s.placed) {
        return (GlobalVerdict::Partial, vec![GlobalIssue::UnplacedScans]);
    }
    if loops.worst_edge_ratio > LOOP_BOUND {
        return (GlobalVerdict::Partial, vec![GlobalIssue::LoopErrorHigh]);
    }
    (GlobalVerdict::Aligned, Vec::new())
}

pub(super) fn finish(
    n: usize,
    opts: &GlobalOptions,
    cands: Vec<Cand>,
    solved: &Solved,
    loops: LoopReport,
    possible: usize,
    tried: usize,
) -> GlobalResult {
    let scans = scan_poses(n, opts, &cands, solved);
    let (verdict, mut issues) = judge(&scans, &loops);
    if cands.iter().any(|c| c.report.status == EdgeStatus::Rejected) {
        issues.push(GlobalIssue::EdgeRejected);
    }
    if tried < possible && verdict != GlobalVerdict::Aligned {
        issues.push(GlobalIssue::PairsCapped);
    }
    if !solved.converged {
        issues.push(GlobalIssue::NotConverged);
    }
    GlobalResult {
        verdict,
        issues,
        scans,
        edges: cands.into_iter().map(|c| c.report).collect(),
        loops,
        pairs_tried: tried,
        pairs_possible: possible,
        iterations: solved.iterations,
    }
}
