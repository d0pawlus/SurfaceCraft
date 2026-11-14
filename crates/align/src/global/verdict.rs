//! Per-scan reports and the overall verdict.
use super::graph::held_by_loop;
use super::report::{EdgeStatus, GlobalIssue, GlobalResult, GlobalVerdict, LoopReport, ScanPose, Unplaced};
use super::solve::{Cand, Solved};
use super::{GlobalOptions, LOOP_BOUND, MAX_REJECTED_PER_USED, MIN_CONSENSUS_MARGIN};
use surfacecraft_geom::Transform;

fn unplaced_reason(k: usize, cands: &[Cand]) -> Unplaced {
    let mine = || cands.iter().filter(move |c| c.report.fixed == k || c.report.moving == k);
    if mine().any(|c| c.report.status == EdgeStatus::Unplaced) {
        Unplaced::OtherGroup
    } else if mine().any(|c| c.report.status == EdgeStatus::Rejected) {
        Unplaced::Contradicted
    } else if mine().next().is_some() {
        Unplaced::NoUniquePair
    } else {
        Unplaced::PairsNotTried
    }
}

fn scan_poses(n: usize, opts: &GlobalOptions, cands: &[Cand], solved: &Solved) -> Vec<ScanPose> {
    let held = held_by_loop(&solved.tree, &solved.edges, n);
    (0..n)
        .map(|k| {
            let placed = solved.tree.reaches(k);
            let pick = |poses: &[Transform]| {
                if placed { poses.get(k).copied().unwrap_or_default() } else { Transform::identity() }
            };
            let edges_used = solved.edges.iter().filter(|e| e.i == k || e.j == k).count();
            ScanPose {
                held_by_loop: held.get(k).copied().unwrap_or(false),
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

/// `aligned` only when every scan is placed and held by a closed loop, no used edge disagrees with
/// the final poses by more than [`LOOP_BOUND`] tolerances, the vote that left pairs out was clear
/// and the left-out pairs are not the majority.
fn judge(
    scans: &[ScanPose],
    loops: &LoopReport,
    rejected: usize,
    used: usize,
    margin: Option<usize>,
) -> (GlobalVerdict, Vec<GlobalIssue>) {
    let placed_others = scans.iter().filter(|s| s.placed && !s.fixed).count();
    if placed_others == 0 {
        return (GlobalVerdict::Failed, vec![GlobalIssue::NotConnected]);
    }
    let checks = [
        (scans.iter().any(|s| !s.placed), GlobalIssue::UnplacedScans),
        (scans.iter().any(|s| s.placed && !s.held_by_loop), GlobalIssue::WeaklyHeld),
        (loops.worst_edge_ratio > LOOP_BOUND, GlobalIssue::LoopErrorHigh),
        (margin.is_some_and(|m| m < MIN_CONSENSUS_MARGIN), GlobalIssue::ConsensusUnclear),
        (rejected as f64 > MAX_REJECTED_PER_USED * used as f64, GlobalIssue::ManyEdgesRejected),
    ];
    let issues: Vec<GlobalIssue> = checks.into_iter().filter(|(bad, _)| *bad).map(|(_, i)| i).collect();
    let verdict = if issues.is_empty() { GlobalVerdict::Aligned } else { GlobalVerdict::Partial };
    (verdict, issues)
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
    let rejected = cands.iter().filter(|c| c.report.status == EdgeStatus::Rejected).count();
    let (verdict, mut issues) = judge(&scans, &loops, rejected, solved.edges.len(), solved.margin);
    if rejected > 0 {
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
        edges_used: solved.edges.len(),
        edges_rejected: rejected,
        consensus_margin: solved.margin,
    }
}
