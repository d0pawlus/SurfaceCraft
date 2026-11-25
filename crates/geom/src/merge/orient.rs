//! The orientation of a scan, triangle by triangle: exact duplicates, and the triangles wound
//! against the rest of their piece. Across every edge of two triangles the two run the edge in
//! opposite directions on a consistently oriented surface; a flood over the edges splits each piece
//! into two classes, and the smaller one by area is wound the wrong way (a patch turned inside out).
use super::prep::NONE;
use crate::Mesh;
use std::collections::VecDeque;

/// A scan whose triangles wound against the rest cover more than this share of its area is refused;
/// fewer are left out and reported (single flipped triangles a scanner leaves).
pub const FLIPPED_REFUSE_SHARE: f64 = 0.001;

/// Exact duplicate triangles (the same three vertices, in any order) after the first.
pub(super) fn duplicates(mesh: &Mesh) -> Vec<bool> {
    let mut keyed: Vec<([u32; 3], usize)> = mesh
        .triangles
        .iter()
        .enumerate()
        .map(|(i, t)| {
            let mut k = *t;
            k.sort_unstable();
            (k, i)
        })
        .collect();
    keyed.sort_unstable();
    let mut dup = vec![false; mesh.triangles.len()];
    for w in keyed.windows(2) {
        if let [(a, _), (b, j)] = w
            && a == b
            && let Some(d) = dup.get_mut(*j)
        {
            *d = true;
        }
    }
    dup
}

/// Does triangle `n` run the edge `a → b` the same way (`a` then `b` in its order)?
fn same_way(mesh: &Mesh, n: usize, a: u32, b: u32) -> bool {
    mesh.triangles.get(n).is_some_and(|t| (0..3).any(|k| t[k] == a && t[(k + 1) % 3] == b))
}

/// The class of every triangle (0 or 1) in a flood over the edges of two triangles, and the piece
/// it was reached from (the first triangle of the flood).
fn classes(mesh: &Mesh, across: &[[u32; 3]]) -> (Vec<u8>, Vec<u32>) {
    let n = mesh.triangles.len();
    let mut class = vec![u8::MAX; n];
    let mut piece = vec![NONE; n];
    let mut queue = VecDeque::new();
    for seed in 0..n {
        if class.get(seed) != Some(&u8::MAX) {
            continue;
        }
        class[seed] = 0;
        piece[seed] = seed as u32;
        queue.push_back(seed);
        while let Some(t) = queue.pop_front() {
            let (Some(&tri), Some(&adj), Some(&c)) = (mesh.triangles.get(t), across.get(t), class.get(t)) else {
                continue;
            };
            for k in 0..3 {
                let m = adj[k] as usize;
                if adj[k] == NONE || class.get(m) != Some(&u8::MAX) {
                    continue;
                }
                let flip = same_way(mesh, m, tri[k], tri[(k + 1) % 3]);
                class[m] = if flip { 1 - c } else { c };
                piece[m] = seed as u32;
                queue.push_back(m);
            }
        }
    }
    (class, piece)
}

/// The triangles wound against the rest of their piece: per piece, the class with the smaller area
/// (class 1 on a tie), one flag per triangle (all false when every piece is one class).
pub(super) fn flipped(mesh: &Mesh, across: &[[u32; 3]]) -> Vec<bool> {
    let (class, piece) = classes(mesh, across);
    let mut area: std::collections::BTreeMap<u32, [f64; 2]> = std::collections::BTreeMap::new();
    for (t, (&c, &p)) in class.iter().zip(&piece).enumerate() {
        if let Some(slot) = area.entry(p).or_insert([0.0; 2]).get_mut(usize::from(c.min(1))) {
            *slot += mesh.face_area(t);
        }
    }
    class
        .iter()
        .zip(&piece)
        .map(|(&c, p)| {
            let [a0, a1] = area.get(p).copied().unwrap_or([0.0; 2]);
            let minority = if a1 <= a0 { 1 } else { 0 };
            a1 > 0.0 && c == minority
        })
        .collect()
}
