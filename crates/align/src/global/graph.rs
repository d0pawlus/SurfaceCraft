//! The graph of accepted pair poses: the maximum spanning tree from the fixed scan (strongest
//! edges first), the poses chained along it, and the independent cycles the other edges close.
use super::relax::{Edge, misfit};
use surfacecraft_geom::{Rng, Transform};

/// The spanning tree of the component of the fixed scan.
#[derive(Clone, Debug)]
pub(super) struct Tree {
    /// For every scan the scan it hangs from and the index of the edge between them (`None` for the
    /// root and for scans the tree does not reach).
    pub parent: Vec<Option<(usize, usize)>>,
    /// Scans in the order the tree reached them, the root first.
    pub order: Vec<usize>,
    /// Edges of the tree.
    pub in_tree: Vec<bool>,
}

impl Tree {
    pub fn reaches(&self, k: usize) -> bool {
        self.order.contains(&k)
    }
}

/// Prim's algorithm for the maximum-weight spanning tree from `root`. Ties go to the lower edge
/// index, so the tree does not depend on anything but the input order.
pub(super) fn spanning_tree(n: usize, root: usize, edges: &[Edge]) -> Tree {
    let weights: Vec<f64> = edges.iter().map(|e| e.weight).collect();
    spanning_tree_by(n, root, edges, &weights)
}

/// [`spanning_tree`] with the weights given (one per edge).
fn spanning_tree_by(n: usize, root: usize, edges: &[Edge], weights: &[f64]) -> Tree {
    let mut tree = Tree { parent: vec![None; n], order: vec![root], in_tree: vec![false; edges.len()] };
    let mut reached = vec![false; n];
    if let Some(r) = reached.get_mut(root) {
        *r = true;
    }
    loop {
        let best = edges.iter().enumerate().filter(|(_, e)| is_in(&reached, e.i) != is_in(&reached, e.j)).max_by(
            |(ia, _), (ib, _)| {
                let w = |k: &usize| weights.get(*k).copied().unwrap_or(0.0);
                w(ia).total_cmp(&w(ib)).then(ib.cmp(ia))
            },
        );
        let Some((idx, e)) = best else { break };
        let (from, to) = if is_in(&reached, e.i) { (e.i, e.j) } else { (e.j, e.i) };
        if let Some(r) = reached.get_mut(to) {
            *r = true;
        }
        if let Some(p) = tree.parent.get_mut(to) {
            *p = Some((from, idx));
        }
        if let Some(t) = tree.in_tree.get_mut(idx) {
            *t = true;
        }
        tree.order.push(to);
    }
    tree
}

fn is_in(reached: &[bool], k: usize) -> bool {
    reached.get(k).copied().unwrap_or(false)
}

/// The motion of scan `child` into the frame of its parent, from the measurement on edge `e`.
fn into_parent(e: &Edge, parent: usize) -> Transform {
    if e.i == parent { e.measured } else { e.measured.inverse() }
}

/// Poses (scan -> frame of the root) chained along the tree; identity for scans it does not reach.
pub(super) fn chain(tree: &Tree, edges: &[Edge], n: usize) -> Vec<Transform> {
    let mut poses = vec![Transform::identity(); n];
    for &k in tree.order.iter().skip(1) {
        let Some(&Some((p, idx))) = tree.parent.get(k) else { continue };
        let (Some(e), Some(xp)) = (edges.get(idx), poses.get(p).copied()) else { continue };
        if let Some(x) = poses.get_mut(k) {
            *x = xp.then_after(&into_parent(e, p));
        }
    }
    poses
}

fn path_to_root(tree: &Tree, mut k: usize) -> Vec<(usize, usize)> {
    let mut path = Vec::new();
    while let Some(&Some((p, idx))) = tree.parent.get(k) {
        path.push((k, idx));
        k = p;
    }
    path
}

/// The independent cycles: for every edge between scans the tree reaches that is not in it, the
/// indices of the edges of the cycle it closes (itself first, then the tree path between its ends).
pub(super) fn cycles(tree: &Tree, edges: &[Edge]) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    for (idx, e) in edges.iter().enumerate() {
        if tree.in_tree.get(idx).copied().unwrap_or(true) || !tree.reaches(e.i) || !tree.reaches(e.j) {
            continue;
        }
        let (pi, pj) = (path_to_root(tree, e.i), path_to_root(tree, e.j));
        let common = pi.iter().rev().zip(pj.iter().rev()).take_while(|(a, b)| a.0 == b.0).count();
        let mut cycle = vec![idx];
        cycle.extend(pi.iter().take(pi.len() - common).map(|&(_, edge)| edge));
        cycle.extend(pj.iter().take(pj.len() - common).map(|&(_, edge)| edge));
        out.push(cycle);
    }
    out
}

/// Trials of the consensus search.
const TRIALS_PER_EDGE: usize = 40;
const MAX_TRIALS: usize = 1500;

/// Edges that agree with the poses chained along `tree`, within `bound` tolerances.
fn inliers(tree: &Tree, edges: &[Edge], n: usize, bound: f64) -> Vec<bool> {
    let poses = chain(tree, edges, n);
    edges
        .iter()
        .map(|e| tree.reaches(e.i) && tree.reaches(e.j) && misfit(e, &poses).rms_mm <= bound * e.tolerance)
        .collect()
}

/// The edges to trust: random spanning trees from `root` (the strongest-first tree is the first
/// trial) are chained and every edge is checked against the chain; the tree that agrees with the
/// most edges wins, and the edges that disagree with it are the outliers. A pair pose that is
/// wrong in a way the pair alignment could not tell (a false `unique`) agrees with no consistent
/// set of the others, however many tree edges it sits next to, so it is found where dropping
/// the worst misfit after a relaxation would blame the right poses beside it. Deterministic (the
/// trials are seeded). Returns, per edge, whether it is an inlier of the best tree; edges the
/// root cannot reach count as inliers (they are not judged).
pub(super) fn consensus(n: usize, root: usize, edges: &[Edge], bound: f64) -> Vec<bool> {
    let first = spanning_tree(n, root, edges);
    let mut best = inliers(&first, edges, n, bound);
    let reached: Vec<bool> = edges.iter().map(|e| first.reaches(e.i) && first.reaches(e.j)).collect();
    let score = |v: &[bool]| v.iter().filter(|x| **x).count();
    let mut best_score = score(&best);
    let mut rng = Rng::new(0x51ED_270B);
    for _ in 0..(TRIALS_PER_EDGE * edges.len()).min(MAX_TRIALS) {
        let weights: Vec<f64> = edges.iter().map(|_| rng.uniform()).collect();
        let found = inliers(&spanning_tree_by(n, root, edges, &weights), edges, n, bound);
        let s = score(&found);
        if s > best_score {
            (best, best_score) = (found, s);
        }
    }
    best.iter().zip(&reached).map(|(inlier, reached)| *inlier || !*reached).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn edge(i: usize, j: usize, weight: f64) -> Edge {
        Edge { i, j, measured: Transform::identity(), weight, tolerance: 1.0, points: Vec::new() }
    }

    #[test]
    fn the_tree_takes_the_strongest_edges_first_and_the_ring_has_one_cycle() {
        // a ring 0-1-2-3-0 whose edge 3-0 is the weakest
        let edges = vec![edge(0, 1, 4.0), edge(1, 2, 3.0), edge(2, 3, 2.0), edge(0, 3, 1.0)];
        let tree = spanning_tree(4, 0, &edges);
        assert_eq!(tree.in_tree, vec![true, true, true, false]);
        assert_eq!(tree.order, vec![0, 1, 2, 3]);
        let c = cycles(&tree, &edges);
        assert_eq!(c.len(), 1);
        let mut first = c.first().cloned().unwrap_or_default();
        first.sort_unstable();
        assert_eq!(first, vec![0, 1, 2, 3]);
    }

    #[test]
    fn scans_outside_the_component_of_the_root_are_not_reached() {
        let edges = vec![edge(0, 1, 1.0), edge(2, 3, 5.0)];
        let tree = spanning_tree(4, 0, &edges);
        assert!(tree.reaches(1) && !tree.reaches(2) && !tree.reaches(3));
        assert!(cycles(&tree, &edges).is_empty());
    }
}
