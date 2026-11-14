//! From pair results to the placed scans: the edges, the spanning tree, the relaxation and the
//! rejection of an edge that contradicts the others.
use super::graph::{Tree, chain, consensus, spanning_tree};
use super::pairs::Tried;
use super::relax::{Edge, MIN_VIRTUAL, misfit, relax, virtual_points};
use super::report::{EdgeReport, EdgeStatus, GlobalResult, LoopReport};
use super::{GlobalOptions, REJECT_FACTOR, loops, verdict};
use crate::pair::{MIN_OVERLAP, PairVerdict};
use crate::{AlignError, PairResult};
use surfacecraft_geom::{Bvh, Cancel, Mesh, Transform, par};

/// A pair that was tried: its report, and the edge it makes when it is one.
pub(super) struct Cand {
    pub report: EdgeReport,
    pub edge: Option<Edge>,
}

/// Overlap over squared residual, the residual not below a tenth of the tolerance.
fn weight_of(r: &PairResult) -> f64 {
    let floor = 0.1 * r.tolerance;
    r.overlap_moving.max(r.overlap_fixed) / r.rms.max(floor).max(f64::MIN_POSITIVE).powi(2)
}

fn report_of(t: &Tried, status: EdgeStatus, note: Option<String>) -> EdgeReport {
    let base = EdgeReport {
        fixed: t.fixed,
        moving: t.moving,
        verdict: PairVerdict::NoMatch,
        status,
        transform: Transform::identity(),
        overlap_moving: 0.0,
        overlap_fixed: 0.0,
        rms: 0.0,
        tolerance: 0.0,
        confidence: 0.0,
        method: None,
        misfit_before: None,
        misfit_after: None,
        stability: 0.0,
        probed: 0,
        probes_failed: 0,
        slides: 0,
        probe_margin: None,
        probe_return_gap: 0.0,
        refined: 0,
        note,
    };
    let Ok(r) = &t.outcome else { return base };
    EdgeReport {
        verdict: r.verdict,
        transform: r.transform,
        overlap_moving: r.overlap_moving,
        overlap_fixed: r.overlap_fixed,
        rms: r.rms,
        tolerance: r.tolerance,
        confidence: r.confidence,
        method: Some(r.method),
        stability: r.stability,
        probed: r.probed,
        probes_failed: r.probes_failed,
        slides: r.slides,
        probe_margin: r.probe_margin,
        probe_return_gap: r.probe_return_gap,
        refined: r.refined,
        ..base
    }
}

fn candidate(k: usize, t: &Tried, scans: &[&Mesh], bvhs: &[Option<Bvh>]) -> Cand {
    let r = match &t.outcome {
        Ok(r) => r,
        Err(e) => return Cand { report: report_of(t, EdgeStatus::Failed, Some(e.to_string())), edge: None },
    };
    let status = match r.verdict {
        PairVerdict::Ambiguous => EdgeStatus::Ambiguous,
        PairVerdict::NoMatch => EdgeStatus::NoMatch,
        PairVerdict::Unique => EdgeStatus::Used,
    };
    if status != EdgeStatus::Used {
        return Cand { report: report_of(t, status, None), edge: None };
    }
    let points = match (scans.get(t.moving), bvhs.get(t.fixed)) {
        (Some(m), Some(Some(b))) if r.overlap_moving.max(r.overlap_fixed) >= MIN_OVERLAP => {
            virtual_points(m, b, &r.transform, r.tolerance, 0x9E37_79B9 ^ k as u64)
        }
        _ => Vec::new(),
    };
    if points.len() < MIN_VIRTUAL {
        return Cand { report: report_of(t, EdgeStatus::TooLittleOverlap, None), edge: None };
    }
    let edge =
        Edge { i: t.fixed, j: t.moving, measured: r.transform, weight: weight_of(r), tolerance: r.tolerance, points };
    Cand { report: report_of(t, EdgeStatus::Used, None), edge: Some(edge) }
}

/// The pair results as candidates for edges; builds the BVH of every scan some unique pair is solved in.
pub(super) fn candidates(scans: &[&Mesh], tried: &[Tried], cancel: &Cancel) -> Result<Vec<Cand>, AlignError> {
    let wanted = |k: usize| {
        tried.iter().any(|t| t.fixed == k && matches!(&t.outcome, Ok(r) if r.verdict == PairVerdict::Unique))
    };
    let bvhs: Vec<Option<Bvh>> = par::map(scans.len(), |k| match scans.get(k) {
        Some(m) if wanted(k) => Some(Bvh::build(m)),
        _ => None,
    });
    if cancel.is_cancelled() {
        return Err(AlignError::Cancelled);
    }
    Ok(tried.iter().enumerate().map(|(k, t)| candidate(k, t, scans, &bvhs)).collect())
}

/// The relaxed graph.
pub(super) struct Solved {
    /// Candidate index of every edge in the graph, in the order of `edges`.
    pub members: Vec<usize>,
    pub edges: Vec<Edge>,
    pub tree: Tree,
    pub chained: Vec<Transform>,
    pub poses: Vec<Transform>,
    pub iterations: usize,
    pub converged: bool,
    /// Margin of the first vote that left pairs out.
    pub margin: Option<usize>,
}

/// Misfit of the worst edge in tolerances: its position and value.
fn worst_edge(edges: &[Edge], poses: &[Transform]) -> Option<(usize, f64)> {
    edges
        .iter()
        .enumerate()
        .map(|(k, e)| (k, misfit(e, poses).rms_mm / e.tolerance.max(f64::MIN_POSITIVE)))
        .max_by(|a, b| a.1.total_cmp(&b.1).then(b.0.cmp(&a.0)))
}

fn graph_of(cands: &[Cand]) -> (Vec<usize>, Vec<Edge>) {
    cands
        .iter()
        .enumerate()
        .filter_map(|(k, c)| c.edge.clone().filter(|_| c.report.status == EdgeStatus::Used).map(|e| (k, e)))
        .unzip()
}

/// Spanning tree, chain and relaxation over the used edges; an edge that disagrees with the final
/// poses by more than [`REJECT_FACTOR`] tolerances is marked rejected and the graph is solved again.
pub(super) fn solve_graph(n: usize, root: usize, cands: &mut [Cand], cancel: &Cancel) -> Result<Solved, AlignError> {
    let mut margin = None;
    loop {
        let (members, edges) = graph_of(cands);
        let vote = consensus(n, root, &edges, REJECT_FACTOR);
        if vote.inlier.iter().any(|x| !*x) {
            margin = margin.or(vote.margin);
            for (&k, _) in members.iter().zip(&vote.inlier).filter(|(_, ok)| !**ok) {
                if let Some(c) = cands.get_mut(k) {
                    c.report.status = EdgeStatus::Rejected;
                }
            }
            continue;
        }
        let tree = spanning_tree(n, root, &edges);
        let reach = |e: &Edge| tree.reaches(e.i) && tree.reaches(e.j);
        let (members, edges): (Vec<usize>, Vec<Edge>) =
            members.into_iter().zip(edges).filter(|(_, e)| reach(e)).unzip();
        for c in cands.iter_mut().filter(|c| c.report.status == EdgeStatus::Used) {
            if c.edge.as_ref().is_some_and(|e| !reach(e)) {
                c.report.status = EdgeStatus::Unplaced;
            }
        }
        let tree = spanning_tree(n, root, &edges);
        let chained = chain(&tree, &edges, n);
        let relaxed = relax(n, root, &edges, &chained, cancel)?;
        let bad = worst_edge(&edges, &relaxed.poses).filter(|(_, ratio)| *ratio > REJECT_FACTOR);
        let Some((worst, _)) = bad else {
            return Ok(Solved {
                members,
                edges,
                tree,
                chained,
                poses: relaxed.poses,
                iterations: relaxed.iterations,
                converged: relaxed.converged,
                margin,
            });
        };
        if let Some(c) = members.get(worst).and_then(|&k| cands.get_mut(k)) {
            c.report.status = EdgeStatus::Rejected;
        }
    }
}

/// Everything after the pair stage.
pub(super) fn solve(
    n: usize,
    opts: &GlobalOptions,
    mut cands: Vec<Cand>,
    possible: usize,
    tried: usize,
    cancel: &Cancel,
) -> Result<GlobalResult, AlignError> {
    let solved = solve_graph(n, opts.fixed, &mut cands, cancel)?;
    let loops: LoopReport = loops::report(&solved.tree, &solved.edges, &solved.chained, &solved.poses);
    for (e, &k) in solved.edges.iter().zip(&solved.members) {
        if let Some(c) = cands.get_mut(k) {
            c.report.misfit_before = Some(misfit(e, &solved.chained));
            c.report.misfit_after = Some(misfit(e, &solved.poses));
        }
    }
    // a pair that was left out still gets its misfit at the final poses, where both ends are placed:
    // the size of the contradiction must be visible
    for c in cands.iter_mut().filter(|c| c.report.status == EdgeStatus::Rejected) {
        let Some(e) = &c.edge else { continue };
        if solved.tree.reaches(e.i) && solved.tree.reaches(e.j) {
            c.report.misfit_before = Some(misfit(e, &solved.chained));
            c.report.misfit_after = Some(misfit(e, &solved.poses));
        }
    }
    Ok(verdict::finish(n, opts, cands, &solved, loops, possible, tried))
}
