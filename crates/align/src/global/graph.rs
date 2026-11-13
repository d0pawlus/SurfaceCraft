//! The graph of accepted pair poses: the maximum spanning tree from the fixed scan (strongest
//! edges first), the poses chained along it, and the independent cycles the other edges close.
use super::relax::Edge;
use surfacecraft_geom::Transform;

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
    let mut tree = Tree { parent: vec![None; n], order: vec![root], in_tree: vec![false; edges.len()] };
    let mut reached = vec![false; n];
    if let Some(r) = reached.get_mut(root) {
        *r = true;
    }
    loop {
        let best = edges
            .iter()
            .enumerate()
            .filter(|(_, e)| is_in(&reached, e.i) != is_in(&reached, e.j))
            .max_by(|(ia, a), (ib, b)| a.weight.total_cmp(&b.weight).then(ib.cmp(ia)));
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

/// The motion `from` -> `to` an edge between those two scans measured.
fn between(e: &Edge, from: usize, to: usize) -> Option<Transform> {
    if e.j == from && e.i == to {
        Some(e.measured)
    } else if e.i == from && e.j == to {
        Some(e.measured.inverse())
    } else {
        None
    }
}

/// Does the triangle of edges `a`, `b`, `c` (three scans, three edges) close within `bound`
/// tolerances? Each edge is checked against the other two on its own points.
fn triangle_closes(edges: &[Edge], tri: [usize; 3], bound: f64) -> Option<bool> {
    let [a, b, c] = tri.map(|k| edges.get(k));
    let (a, b, c) = (a?, b?, c?);
    let mut worst: f64 = 0.0;
    for (e, f, g) in [(a, b, c), (b, a, c), (c, a, b)] {
        // the scan `e` moves (j) and the one it lands in (i); the third scan is the one `f` and `g` share
        let third = [f.i, f.j].into_iter().find(|k| *k != e.i && *k != e.j)?;
        let to_third = [f, g].into_iter().find_map(|x| between(x, e.j, third))?;
        let from_third = [f, g].into_iter().find_map(|x| between(x, third, e.i))?;
        let via = from_third.then_after(&to_third);
        let sum: f64 = e.points.iter().map(|p| (e.measured.apply(p) - via.apply(p)).norm_squared()).sum();
        let rms = (sum / e.points.len().max(1) as f64).sqrt();
        worst = worst.max(rms / e.tolerance.max(f64::MIN_POSITIVE));
    }
    Some(worst <= bound)
}

/// The edge that most triangles contradict: among the triangles of three scans with all three
/// edges, the edge in more triangles that do not close than in triangles that do (the most
/// contradicted first, the lower index on a tie), if any. A pair pose that is wrong in a way the
/// pair alignment could not tell (a false `unique`) closes no triangle with the right ones, while a
/// right pose next to it still closes the triangles that do not involve it.
pub(super) fn contradicted_edge(edges: &[Edge], bound: f64) -> Option<usize> {
    let mut votes = vec![(0_i64, 0_i64); edges.len()];
    let find = |x: usize, y: usize| edges.iter().position(|e| (e.i, e.j) == (x.min(y), x.max(y)) || (e.j, e.i) == (x.min(y), x.max(y)));
    for (a, e) in edges.iter().enumerate() {
        for (b, f) in edges.iter().enumerate().skip(a + 1) {
            let shared = [e.i, e.j].into_iter().find(|k| *k == f.i || *k == f.j);
            let (Some(s), true) = (shared, (e.i, e.j) != (f.i, f.j)) else { continue };
            let (x, y) = (if e.i == s { e.j } else { e.i }, if f.i == s { f.j } else { f.i });
            let Some(c) = find(x, y).filter(|&c| c > b) else { continue };
            let Some(closes) = triangle_closes(edges, [a, b, c], bound) else { continue };
            for k in [a, b, c] {
                if let Some(v) = votes.get_mut(k) {
                    if closes { v.0 += 1 } else { v.1 += 1 }
                }
            }
        }
    }
    votes
        .iter()
        .enumerate()
        .filter(|(_, v)| v.1 > v.0)
        .max_by(|(ia, a), (ib, b)| (a.1 - a.0).cmp(&(b.1 - b.0)).then(ib.cmp(ia)))
        .map(|(k, _)| k)
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
