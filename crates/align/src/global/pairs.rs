//! The pair stage: which pairs of scans are tried, and their poses from [`crate::pair_coarse`].
use super::{GlobalOptions, StartMode};
use crate::{AlignError, PairOptions, PairResult, pair_coarse};
use std::collections::BTreeSet;
use surfacecraft_geom::{Aabb, Cancel, Mesh, Transform, par};

/// Margin (fraction of the larger diagonal) by which two scans' boxes may miss each other and the
/// pair still be tried when the scans start from their current poses.
const BOX_MARGIN: f64 = 0.05;

/// One pair that was tried: `fixed` is the lower index, `moving` the higher.
pub(super) struct Tried {
    pub fixed: usize,
    pub moving: usize,
    pub outcome: Result<PairResult, AlignError>,
}

fn boxes_meet(a: &Aabb, b: &Aabb) -> bool {
    let margin = BOX_MARGIN * a.diagonal().max(b.diagonal());
    (0..3).all(|k| {
        let (amin, amax) = (a.min.coords.get(k).copied().unwrap_or(0.0), a.max.coords.get(k).copied().unwrap_or(0.0));
        let (bmin, bmax) = (b.min.coords.get(k).copied().unwrap_or(0.0), b.max.coords.get(k).copied().unwrap_or(0.0));
        amin - margin <= bmax && bmin - margin <= amax
    })
}

/// The pairs worth trying, best guesses first, and how many there are in all. Scans of one part
/// are usually listed in the order they were taken, so neighbours in the list (cyclically) come
/// first: distance 1, then 2, and so on; the list is cut at `max_pairs`. From the current poses
/// the pairs whose boxes do not meet are left out.
pub(super) fn pair_list(scans: &[&Mesh], opts: &GlobalOptions) -> (Vec<(usize, usize)>, usize) {
    let n = scans.len();
    let boxes: Vec<Aabb> = scans.iter().map(|m| m.bbox()).collect();
    let mut seen = BTreeSet::new();
    let mut all = Vec::new();
    for d in 1..=n / 2 {
        for i in 0..n {
            let (a, b) = (i, (i + d) % n);
            let pair = (a.min(b), a.max(b));
            let near = match (opts.start, boxes.get(pair.0), boxes.get(pair.1)) {
                (StartMode::Current, Some(x), Some(y)) => boxes_meet(x, y),
                _ => true,
            };
            if near && seen.insert(pair) {
                all.push(pair);
            }
        }
    }
    let possible = all.len();
    all.truncate(opts.max_pairs);
    (all, possible)
}

/// Solves every pair in `list` (in parallel; each result is independent of the others and of the
/// thread count). Only a cancellation is an error of the whole stage.
pub(super) fn run_pairs(
    scans: &[&Mesh],
    list: &[(usize, usize)],
    opts: &GlobalOptions,
    cancel: &Cancel,
) -> Result<Vec<Tried>, AlignError> {
    let given = (opts.start == StartMode::Current).then(Transform::identity);
    let options = PairOptions { tolerance: opts.tolerance, given, ..PairOptions::default() };
    let tried = par::map(list.len(), |k| {
        let &(fixed, moving) = list.get(k)?;
        let (f, m) = (scans.get(fixed)?, scans.get(moving)?);
        let outcome = if cancel.is_cancelled() { Err(AlignError::Cancelled) } else { pair_coarse(m, f, &options, cancel) };
        Some(Tried { fixed, moving, outcome })
    });
    let tried: Vec<Tried> = tried.into_iter().flatten().collect();
    if tried.iter().any(|t| matches!(t.outcome, Err(AlignError::Cancelled))) {
        return Err(AlignError::Cancelled);
    }
    Ok(tried)
}
