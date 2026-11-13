//! Where the patches of a body are: points on them, and which of them touch.
use super::boundary::{de_boor, pinch_vertices};
use crate::hybrid::{V, cross, norm, sub};
use crate::record::brep::{Brep, Surface};
use std::collections::BTreeMap;

/// The parameter range of a spline's direction, `None` for malformed knots.
fn range(knots: &[f64], count: usize) -> Option<[f64; 2]> {
    let degree = knots.len().checked_sub(count + 1)?;
    Some([*knots.get(degree)?, *knots.get(count)?])
}

/// A grid of `(n + 1) x (n + 1)` points over the whole of patch `face` (its borders included),
/// row by row; `None` when the face is not a spline patch or its data is malformed.
#[must_use]
pub fn face_samples(body: &Brep, face: usize, n: usize) -> Option<Vec<V>> {
    let Surface::Spline { u_knots, v_knots, points } = &body.faces.get(face)?.surface else { return None };
    let cols = points.first()?.len();
    let [u0, u1] = range(u_knots, points.len())?;
    let [v0, v1] = range(v_knots, cols)?;
    let n = n.max(1);
    let mut out = Vec::with_capacity((n + 1) * (n + 1));
    for i in 0..=n {
        let u = u0 + (u1 - u0) * i as f64 / n as f64;
        for j in 0..=n {
            let v = v0 + (v1 - v0) * j as f64 / n as f64;
            let across: Vec<V> = points.iter().map(|row| de_boor(v_knots, row, v)).collect::<Option<Vec<V>>>()?;
            out.push(de_boor(u_knots, &across, u)?);
        }
    }
    Some(out)
}

/// The faces of `faces` that touch along an edge, as groups (each ascending, the groups ordered
/// by their lowest face): the connected pieces of the set.
#[must_use]
pub fn connected_groups(body: &Brep, faces: &[usize]) -> Vec<Vec<usize>> {
    let member: BTreeMap<usize, usize> = faces.iter().enumerate().map(|(k, &f)| (f, k)).collect();
    let mut parent: Vec<usize> = (0..faces.len()).collect();
    fn find(parent: &mut [usize], x: usize) -> usize {
        let mut r = x;
        while parent.get(r).is_some_and(|&p| p != r) {
            r = parent.get(r).copied().unwrap_or(r);
        }
        r
    }
    for e in &body.edges {
        let ks: Vec<usize> = e.uses.iter().filter_map(|u| member.get(&(u.face as usize)).copied()).collect();
        for pair in ks.windows(2) {
            if let [a, b] = pair {
                let (ra, rb) = (find(&mut parent, *a), find(&mut parent, *b));
                let (lo, hi) = (ra.min(rb), ra.max(rb));
                if let Some(slot) = parent.get_mut(hi) {
                    *slot = lo;
                }
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (k, &f) in faces.iter().enumerate() {
        let r = find(&mut parent, k);
        groups.entry(r).or_default().push(f);
    }
    let mut out: Vec<Vec<usize>> = groups.into_values().collect();
    for g in &mut out {
        g.sort_unstable();
    }
    out.sort_by_key(|g| g.first().copied().unwrap_or(usize::MAX));
    out
}

/// The area of a grid of `(n + 1) x (n + 1)` points as [`face_samples`] gives them (two triangles
/// to a cell).
#[must_use]
pub fn grid_area(points: &[V], n: usize) -> f64 {
    let at = |i: usize, j: usize| points.get(i * (n + 1) + j).copied().unwrap_or([0.0; 3]);
    let tri = |a: V, b: V, c: V| {
        let (u, v) = (sub(b, a), sub(c, a));
        norm(cross(u, v)) * 0.5
    };
    let mut area = 0.0;
    for i in 0..n {
        for j in 0..n {
            area += tri(at(i, j), at(i + 1, j), at(i, j + 1)) + tri(at(i + 1, j + 1), at(i, j + 1), at(i + 1, j));
        }
    }
    area
}

/// The area of patch `face` from a grid of `n` cells a side; `None` when it is not a spline patch.
#[must_use]
pub fn face_area(body: &Brep, face: usize, n: usize) -> Option<f64> {
    face_samples(body, face, n).map(|p| grid_area(&p, n.max(1)))
}

/// The most rounds of removing the faces at pinches.
const MAX_PINCH_ROUNDS: usize = 8;

/// `faces` as groups whose boundaries do not touch themselves: the faces of a group that meet its
/// boundary twice at a vertex (a pinch) are taken out, and what is left is split into the connected
/// groups again, until none pinches. The faces taken out stay patches. Groups the topology refuses
/// for another reason (several shells, a face shared with itself) are left out.
#[must_use]
pub fn pinch_free_groups(body: &Brep, faces: &[usize]) -> Vec<Vec<usize>> {
    let mut out = Vec::new();
    let mut pending: Vec<(Vec<usize>, usize)> = connected_groups(body, faces).into_iter().map(|g| (g, 0)).collect();
    while let Some((group, round)) = pending.pop() {
        let Ok(pinches) = pinch_vertices(body, &group) else { continue };
        if pinches.is_empty() {
            out.push(group);
        } else if round < MAX_PINCH_ROUNDS {
            let keep: Vec<usize> = group.iter().copied().filter(|&f| !touches(body, f, &pinches)).collect();
            pending.extend(connected_groups(body, &keep).into_iter().map(|g| (g, round + 1)));
        }
    }
    out.sort_by_key(|g| g.first().copied().unwrap_or(usize::MAX));
    out
}

/// Does face `face` have one of `vertices` as a corner?
fn touches(body: &Brep, face: usize, vertices: &std::collections::BTreeSet<u32>) -> bool {
    let Some(f) = body.faces.get(face) else { return false };
    f.loops.iter().any(|l| match l {
        crate::record::brep::Loop::Edges { edges } => edges.iter().any(|oe| {
            body.edges.get(oe.edge as usize).is_some_and(|e| e.vertices.iter().any(|v| vertices.contains(v)))
        }),
        crate::record::brep::Loop::Vertex { vertex } => vertices.contains(vertex),
    })
}
