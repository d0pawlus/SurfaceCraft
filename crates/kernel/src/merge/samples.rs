//! Where the patches of a body are: points on them, and which of them touch.
use super::boundary::pinch_vertices;
use crate::hybrid::{Shape, V, cross, dot, norm, perpendicular, sub};
use crate::record::brep::{Brep, Surface};
use std::collections::BTreeMap;

/// A grid of `(n + 1) x (n + 1)` points over the whole of patch `face` (its borders included), row by
/// row, evaluated by the kernel; `None` when the face is not a spline patch or cannot be evaluated.
#[must_use]
pub fn face_samples(body: &Brep, face: usize, n: usize) -> Option<Vec<V>> {
    let f = body.faces.get(face)?;
    if !matches!(f.surface, Surface::Spline { .. } | Surface::RationalSpline { .. }) {
        return None;
    }
    let eval = solidcore::eval::SurfaceEval::new(&solidcore_convert::surface(&f.surface).ok()?).ok()?;
    let [Some((u0, u1)), Some((v0, v1))] = eval.domain().range else { return None };
    let n = n.max(1);
    let mut out = Vec::with_capacity((n + 1) * (n + 1));
    for i in 0..=n {
        for j in 0..=n {
            let uv = [u0 + (u1 - u0) * i as f64 / n as f64, v0 + (v1 - v0) * j as f64 / n as f64];
            let p = eval.point(uv)?;
            out.push([p.x, p.y, p.z]);
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

/// Sectors a group that wraps round its axis is cut into (each stays well clear of a half turn, so
/// its face has room for the seam of its surface).
const SECTORS: usize = 3;
/// A group whose patches leave a gap in angle of at least this (radians) does not wrap round its axis.
const WRAP_GAP: f64 = 1.0;

/// The angle (0 to 2 pi) of `p` about the axis through `point` along `axis`, from a fixed direction.
fn angle_about(point: V, axis: V, p: V) -> f64 {
    let x = perpendicular(axis);
    let y = cross(axis, x);
    let q = sub(p, point);
    let a = dot(q, y).atan2(dot(q, x));
    if a < 0.0 { a + std::f64::consts::TAU } else { a }
}

/// A group of patches that goes all the way round the axis of a cylinder or cone cannot be one face
/// (it would need a seam edge, which the merge does not make): it is cut into [`SECTORS`] groups by the
/// angle of the middle of each patch, each as connected as the patches allow. Any other group (one that
/// leaves a gap in angle, a plane, a sphere) is returned as it is.
#[must_use]
pub fn split_round(body: &Brep, faces: &[usize], shape: &Shape) -> Vec<Vec<usize>> {
    let (point, axis) = match shape {
        Shape::Cylinder { point, axis, .. } => (*point, *axis),
        Shape::Cone { apex, axis, .. } => (*apex, *axis),
        _ => return vec![faces.to_vec()],
    };
    let Some(axis) = crate::hybrid::unit(axis) else { return vec![faces.to_vec()] };
    let angles: Vec<(f64, usize)> = faces
        .iter()
        .filter_map(|&f| {
            let mid = face_samples(body, f, 2).and_then(|s| s.get(4).copied())?;
            Some((angle_about(point, axis, mid), f))
        })
        .collect();
    let mut sorted: Vec<f64> = angles.iter().map(|a| a.0).collect();
    sorted.sort_by(f64::total_cmp);
    let tau = std::f64::consts::TAU;
    let widest =
        sorted.iter().zip(sorted.iter().cycle().skip(1)).map(|(a, b)| (b - a).rem_euclid(tau)).fold(0.0_f64, f64::max);
    if angles.len() != faces.len() || widest >= WRAP_GAP {
        return vec![faces.to_vec()];
    }
    let mut sectors: Vec<Vec<usize>> = vec![Vec::new(); SECTORS];
    for (a, f) in &angles {
        let k = ((a / tau) * SECTORS as f64) as usize;
        if let Some(sector) = sectors.get_mut(k.min(SECTORS - 1)) {
            sector.push(*f);
        }
    }
    let mut out: Vec<Vec<usize>> = sectors.iter().flat_map(|g| connected_groups(body, g)).collect();
    out.sort_by_key(|g| g.first().copied().unwrap_or(usize::MAX));
    out
}
