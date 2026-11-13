//! Loop closure before and after relaxation, over the independent cycles of the graph.
use super::graph::{Tree, cycles};
use super::relax::{Edge, misfit};
use super::report::{LoopReport, LoopStats};
use surfacecraft_geom::Transform;

/// Per cycle the largest misfit among its edges; the worst and the mean over cycles.
fn stats(cycles: &[Vec<usize>], edges: &[Edge], poses: &[Transform]) -> LoopStats {
    if cycles.is_empty() {
        return LoopStats::default();
    }
    let per_cycle: Vec<(f64, f64)> = cycles
        .iter()
        .map(|c| {
            c.iter().filter_map(|&k| edges.get(k)).map(|e| misfit(e, poses)).fold((0.0, 0.0), |(mm, deg), m| {
                (f64::max(mm, m.rms_mm), f64::max(deg, m.angle_degrees))
            })
        })
        .collect();
    let n = per_cycle.len() as f64;
    let fold = |f: fn(&(f64, f64)) -> f64| (per_cycle.iter().map(f).fold(0.0, f64::max), per_cycle.iter().map(f).sum::<f64>() / n);
    let ((worst_mm, mean_mm), (worst_degrees, mean_degrees)) = (fold(|c| c.0), fold(|c| c.1));
    LoopStats { worst_mm, mean_mm, worst_degrees, mean_degrees }
}

/// The loop figures of the used `edges` with the poses chained along `tree` and with the relaxed ones.
pub(super) fn report(tree: &Tree, edges: &[Edge], chained: &[Transform], relaxed: &[Transform]) -> LoopReport {
    let cs = cycles(tree, edges);
    let worst_edge_ratio = edges
        .iter()
        .map(|e| misfit(e, relaxed).rms_mm / e.tolerance.max(f64::MIN_POSITIVE))
        .fold(0.0, f64::max);
    LoopReport { cycles: cs.len(), before: stats(&cs, edges, chained), after: stats(&cs, edges, relaxed), worst_edge_ratio }
}
