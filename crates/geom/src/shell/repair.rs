//! Repairs a real scan needs before it can be offset: triangles on edges that three or more
//! triangles share are removed, and the orientation is made consistent by flooding it over the
//! triangles that share an edge (each connected piece keeps the orientation most of its triangles
//! already had).
use super::topology::Work;
use crate::{Vec3, normalize};

/// What the repair changed.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Repairs {
    /// Triangles removed because they lie on an edge shared by three or more triangles.
    pub removed: usize,
    /// Triangles turned around to make the orientation consistent.
    pub flipped: usize,
}

fn undirected(a: u32, b: u32) -> u64 {
    (u64::from(a.min(b)) << 32) | u64::from(a.max(b))
}

fn edges_of(t: &[u32; 3]) -> [(u32, u32); 3] {
    [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])]
}

/// Sorted `(edge, triangle)` pairs.
fn edge_table(triangles: &[[u32; 3]]) -> Vec<(u64, u32)> {
    let mut table: Vec<(u64, u32)> = triangles
        .iter()
        .enumerate()
        .flat_map(|(i, t)| edges_of(t).map(|(a, b)| (undirected(a, b), i as u32)))
        .collect();
    table.sort_unstable();
    table
}

/// The triangles that use the edge `key`.
fn users(table: &[(u64, u32)], key: u64) -> impl Iterator<Item = u32> + '_ {
    let start = table.partition_point(|&(k, _)| k < key);
    table.get(start..).unwrap_or(&[]).iter().take_while(move |&&(k, _)| k == key).map(|&(_, t)| t)
}

/// Unit normal of triangle `t` (zero when flat).
fn normal_of(w: &Work, t: usize) -> Vec3 {
    let Some(tri) = w.triangles.get(t) else { return Vec3::zeros() };
    let (Some(a), Some(b), Some(c)) =
        (w.positions.get(tri[0] as usize), w.positions.get(tri[1] as usize), w.positions.get(tri[2] as usize))
    else {
        return Vec3::zeros();
    };
    normalize((b - a).cross(&(c - a))).unwrap_or_else(Vec3::zeros)
}

/// How well triangle `t` continues the surface: for each of its edges other than `edge`, the best
/// agreement of its normal (orientation ignored) with a neighbour across that edge. A fin has no
/// neighbours on its other edges, the triangles of the surface have them.
fn continuity(w: &Work, table: &[(u64, u32)], t: usize, edge: u64) -> f64 {
    let Some(tri) = w.triangles.get(t) else { return 0.0 };
    let mine = normal_of(w, t);
    edges_of(tri)
        .iter()
        .map(|&(a, b)| undirected(a, b))
        .filter(|&k| k != edge)
        .map(|k| {
            users(table, k)
                .filter(|&u| u as usize != t)
                .take(2)
                .map(|u| mine.dot(&normal_of(w, u as usize)).abs())
                .fold(0.0, f64::max)
        })
        .sum()
}

/// Of the triangles on an edge that three or more share, keeps the two that continue the surface best
/// and removes the others (fins, doubled sheets); returns how many were removed. One linear pass
/// over the runs of equal edges of the sorted table; a run of `k` triangles costs `k` scorings.
fn remove_non_manifold(w: &mut Work) -> usize {
    let table = edge_table(&w.triangles);
    let mut gone = vec![false; w.triangles.len()];
    let mut start = 0;
    while let Some(&(key, _)) = table.get(start) {
        let run = table.get(start..).map_or(0, |r| r.iter().take_while(|&&(k, _)| k == key).count());
        let members: Vec<usize> =
            table.get(start..start + run).unwrap_or(&[]).iter().map(|&(_, t)| t as usize).collect();
        start += run.max(1);
        let alive: Vec<usize> = members.into_iter().filter(|&t| !gone.get(t).copied().unwrap_or(true)).collect();
        if alive.len() <= 2 {
            continue;
        }
        let mut scored: Vec<(f64, usize)> = alive.iter().map(|&t| (continuity(w, &table, t, key), t)).collect();
        scored.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
        for &(_, t) in scored.iter().skip(2) {
            if let Some(g) = gone.get_mut(t) {
                *g = true;
            }
        }
    }
    let before = w.triangles.len();
    let mut flags = gone.iter();
    w.triangles.retain(|_| !flags.next().copied().unwrap_or(false));
    before - w.triangles.len()
}

/// Removes the triangles that repeat an earlier triangle with the same winding (two such triangles
/// on a boundary edge would close into a flat pillow); returns how many.
fn remove_duplicates(w: &mut Work) -> usize {
    let rotated = |t: &[u32; 3]| {
        let k = (0..3).min_by_key(|&k| t[k]).unwrap_or(0);
        [t[k], t[(k + 1) % 3], t[(k + 2) % 3]]
    };
    let mut keys: Vec<([u32; 3], usize)> = w.triangles.iter().enumerate().map(|(i, t)| (rotated(t), i)).collect();
    keys.sort_unstable();
    let mut gone = vec![false; w.triangles.len()];
    for pair in keys.windows(2) {
        if pair[0].0 == pair[1].0
            && let Some(g) = gone.get_mut(pair[1].1)
        {
            *g = true;
        }
    }
    let before = w.triangles.len();
    let mut flags = gone.iter();
    w.triangles.retain(|_| !flags.next().copied().unwrap_or(false));
    before - w.triangles.len()
}

/// Direction in which triangle `t` (as stored) runs the edge `{a, b}`: true for `a -> b`.
fn runs_forward(t: &[u32; 3], a: u32, b: u32) -> bool {
    edges_of(t).contains(&(a, b))
}

/// Floods the orientation of the triangle `seed` over its piece; `state` is 0 (not reached), 1 (keep)
/// or 2 (turn around). Returns the triangles of the piece.
fn flood(w: &Work, table: &[(u64, u32)], state: &mut [u8], seed: usize) -> Vec<usize> {
    let mut piece = vec![seed];
    if let Some(s) = state.get_mut(seed) {
        *s = 1;
    }
    let mut next = 0;
    while let Some(&t) = piece.get(next) {
        next += 1;
        let (Some(tri), Some(&mine)) = (w.triangles.get(t), state.get(t)) else { continue };
        for (a, b) in edges_of(tri) {
            // the direction this triangle runs the edge once its own state is applied
            let (from, to) = if mine == 2 { (b, a) } else { (a, b) };
            for u in users(table, undirected(a, b)).map(|u| u as usize).filter(|&u| u != t) {
                let (Some(other), Some(slot)) = (w.triangles.get(u), state.get_mut(u)) else { continue };
                if *slot == 0 {
                    *slot = if runs_forward(other, from, to) { 2 } else { 1 };
                    piece.push(u);
                }
            }
        }
    }
    piece
}

/// Makes the orientation of every piece consistent; returns the triangles turned around.
fn orient(w: &mut Work) -> usize {
    let table = edge_table(&w.triangles);
    let mut state = vec![0u8; w.triangles.len()];
    for seed in 0..w.triangles.len() {
        if state.get(seed).copied() != Some(0) {
            continue;
        }
        let piece = flood(w, &table, &mut state, seed);
        let turned = piece.iter().filter(|&&t| state.get(t) == Some(&2)).count();
        if turned * 2 > piece.len() {
            for &t in &piece {
                if let Some(s) = state.get_mut(t) {
                    *s = 3 - *s;
                }
            }
        }
    }
    let mut flipped = 0;
    for (t, s) in w.triangles.iter_mut().zip(&state) {
        if *s == 2 {
            t.swap(1, 2);
            flipped += 1;
        }
    }
    flipped
}

/// Removes non-manifold triangles and orients the rest consistently.
pub(super) fn repair(w: &mut Work) -> Repairs {
    let removed = remove_duplicates(w) + remove_non_manifold(w);
    let flipped = orient(w);
    Repairs { removed, flipped }
}
