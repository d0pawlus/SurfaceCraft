//! Hole patches that keep sharp edges.
//!
//! A hole that straddles a sharp edge (a spike removed on the edge of a part, missing data on a
//! corner) is first filled and faired like any other, which rounds the edge off into a chamfer.
//! [`sharpen_patch`] then looks at the surface around the hole: when it falls into two or three
//! smooth sides that meet at sharp edges, each side is extended over the hole (a plane, or a
//! quadric when the side is curved) and the patch vertices are moved onto the extended sides, the
//! ones next to an edge onto the edge itself. A hole in a smooth surface has one side and is left
//! as the fairing made it.

use crate::{Mesh, Point3, Vec3, tri};
use nalgebra::{SMatrix, SVector};
use std::collections::{HashMap, HashSet};

/// Two parts of the surface around a hole are different sides when their normals differ by more
/// than this (degrees), and an edge between them is sharp when it turns by more than this.
const FEATURE_DEG: f64 = 35.0;
/// Most sides a hole may touch (three meet at a corner); more is left to the fairing.
const MAX_SIDES: usize = 3;
/// Most groups of normals looked at before the surface is taken to be too rough to read.
const MAX_GROUPS: usize = 8;
/// Smallest share of the area around the hole that makes a group of triangles a side.
const MIN_SIDE_SHARE: f64 = 0.1;
/// Fewest points a quadric (six terms) is fitted to; below it the side is a plane.
const MIN_QUADRIC_POINTS: usize = 12;
/// A quadric replaces the plane only when it leaves less than this share of the plane's RMS.
const QUADRIC_GAIN: f64 = 0.5;
/// A patch vertex this close to a second side (in mean loop edge lengths) goes onto the edge.
const EDGE_SNAP: f64 = 0.5;
/// Projection rounds for one vertex.
const PROJECTIONS: usize = 12;
/// A patch edge is flipped when the other diagonal is at most this share as far from the wedge.
const FLIP_GAIN: f64 = 0.5;
/// Smallest area of a patch triangle after its vertices move, in squared mean loop edge lengths
/// (an equilateral triangle of that edge has 0.43).
const MIN_AREA: f64 = 0.02;
/// Three sides meet in a corner when a point is on all of them within this share of the hole's
/// size.
const CORNER_TOLERANCE: f64 = 1e-6;
/// Passes of edge flips over a patch.
const FLIP_PASSES: usize = 8;

/// Terms of the height polynomial of a side: 1, x, y, x², xy, y².
const TERMS: usize = 6;

/// One smooth side of the surface around a hole, as a height field over its mean plane.
struct Side {
    origin: Point3,
    n: Vec3,
    u: Vec3,
    v: Vec3,
    coef: [f64; TERMS],
}

fn basis(x: f64, y: f64) -> [f64; TERMS] {
    [1.0, x, y, x * x, x * y, y * y]
}

/// Least squares of z over the first `terms` basis functions; the coefficients and the RMS left.
fn fit_height(local: &[(f64, f64, f64)], terms: usize) -> Option<([f64; TERMS], f64)> {
    let mut a = SMatrix::<f64, TERMS, TERMS>::zeros();
    let mut b = SVector::<f64, TERMS>::zeros();
    for &(x, y, z) in local {
        let f = basis(x, y);
        for i in 0..terms.min(TERMS) {
            for j in 0..terms.min(TERMS) {
                a[(i, j)] += f[i] * f[j];
            }
            b[i] += f[i] * z;
        }
    }
    for i in terms.min(TERMS)..TERMS {
        a[(i, i)] = 1.0;
    }
    let c = a.lu().solve(&b)?;
    let coef: [f64; TERMS] = std::array::from_fn(|i| c[i]);
    if coef.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let sq: f64 = local
        .iter()
        .map(|&(x, y, z)| {
            let f = basis(x, y);
            let h: f64 = f.iter().zip(&coef).map(|(f, c)| f * c).sum();
            (z - h).powi(2)
        })
        .sum();
    Some((coef, (sq / local.len().max(1) as f64).sqrt()))
}

impl Side {
    /// The side through `points` with mean normal `n`: a plane, or a quadric when the points
    /// are enough and bend away from the plane.
    fn fit(points: &[Point3], n: Vec3) -> Option<Side> {
        if points.len() < 3 {
            return None;
        }
        let origin = Point3::from(points.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / points.len() as f64);
        let u = crate::any_perpendicular(&n);
        let v = n.cross(&u);
        let local: Vec<(f64, f64, f64)> = points
            .iter()
            .map(|p| {
                let d = p - origin;
                (d.dot(&u), d.dot(&v), d.dot(&n))
            })
            .collect();
        let (plane, plane_rms) = fit_height(&local, 3)?;
        let coef = match fit_height(&local, TERMS) {
            Some((quadric, rms)) if points.len() >= MIN_QUADRIC_POINTS && rms < QUADRIC_GAIN * plane_rms => quadric,
            _ => plane,
        };
        Some(Side { origin, n, u, v, coef })
    }

    /// Height of `p` over the side, along the side's normal.
    fn height(&self, p: &Point3) -> f64 {
        let d = p - self.origin;
        let f = basis(d.dot(&self.u), d.dot(&self.v));
        d.dot(&self.n) - f.iter().zip(&self.coef).map(|(f, c)| f * c).sum::<f64>()
    }
}

/// A triangle next to the hole: its vertices, unit normal, area and side (once known).
struct Near {
    corners: [u32; 3],
    normal: Vec3,
    area: f64,
    side: Option<usize>,
}

/// The triangles before `first_tri` within two rings of the loop, in index order.
fn triangles_near(mesh: &Mesh, first_tri: usize, lp: &[u32]) -> Vec<Near> {
    let mut reach: HashSet<u32> = lp.iter().copied().collect();
    let mut picked: Vec<usize> = Vec::new();
    let mut seen: HashSet<usize> = HashSet::new();
    for _ring in 0..2 {
        let mut next = reach.clone();
        for (i, t) in mesh.triangles.iter().enumerate().take(first_tri) {
            if t.iter().any(|v| reach.contains(v)) && seen.insert(i) {
                picked.push(i);
                next.extend(t.iter().copied());
            }
        }
        reach = next;
    }
    picked.sort_unstable();
    picked
        .into_iter()
        .filter_map(|i| {
            let corners = *mesh.triangles.get(i)?;
            let [a, b, c] = mesh.triangle(i)?;
            Some(Near { corners, normal: tri::normal(&a, &b, &c)?, area: tri::area(&a, &b, &c), side: None })
        })
        .collect()
}

/// Groups the triangles by normal and gives the mean normal of each side; `None` unless there
/// are two or three sides further apart than the feature angle.
fn sides_of(near: &mut [Near]) -> Option<Vec<Vec3>> {
    let cos = FEATURE_DEG.to_radians().cos();
    let mut order: Vec<usize> = (0..near.len()).collect();
    order.sort_by(|&a, &b| near[b].area.total_cmp(&near[a].area).then(a.cmp(&b)));
    // (first normal, area-weighted sum, area)
    let mut groups: Vec<(Vec3, Vec3, f64)> = Vec::new();
    for &i in &order {
        let (n, area) = (near[i].normal, near[i].area);
        if let Some(g) = groups.iter_mut().find(|g| g.0.dot(&n) >= cos) {
            g.1 += n * area;
            g.2 += area;
        } else if groups.len() < MAX_GROUPS {
            groups.push((n, n * area, area));
        } else {
            return None;
        }
    }
    let total: f64 = groups.iter().map(|g| g.2).sum();
    let means: Vec<Vec3> =
        groups.iter().filter(|g| g.2 >= MIN_SIDE_SHARE * total).filter_map(|g| crate::normalize(g.1)).collect();
    if means.len() < 2 || means.len() > MAX_SIDES {
        return None;
    }
    for (i, a) in means.iter().enumerate() {
        if means.iter().skip(i + 1).any(|b| a.dot(b) >= cos) {
            return None;
        }
    }
    for t in near.iter_mut() {
        let best = means
            .iter()
            .enumerate()
            .map(|(k, m)| (m.dot(&t.normal), k))
            .fold(None, |b: Option<(f64, usize)>, c| if b.is_none_or(|b| c.0 > b.0) { Some(c) } else { b });
        t.side = best.filter(|b| b.0 >= cos).map(|b| b.1);
    }
    Some(means)
}

/// Do the sides meet at sharp edges? The median turn across the mesh edges that join triangles
/// of different sides must exceed the feature angle (a smooth surface that merely curves a lot
/// around the hole turns a little at each edge).
fn meets_sharply(near: &[Near]) -> bool {
    let mut by_edge: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in near.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t.corners[k], t.corners[(k + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    let mut turns: Vec<f64> = by_edge
        .values()
        .filter_map(|ts| match ts.as_slice() {
            [i, j] => {
                let (a, b) = (near.get(*i)?, near.get(*j)?);
                (a.side.is_some() && b.side.is_some() && a.side != b.side)
                    .then(|| a.normal.dot(&b.normal).clamp(-1.0, 1.0).acos())
            }
            _ => None,
        })
        .collect();
    turns.sort_by(f64::total_cmp);
    turns.get(turns.len() / 2).is_some_and(|m| *m > FEATURE_DEG.to_radians())
}

/// Is the wedge of the sides convex (`Some(true)`), concave (`Some(false)`) or neither?
fn convexity(sides: &[Side]) -> Option<bool> {
    let mut signs: Vec<bool> = Vec::new();
    for (i, a) in sides.iter().enumerate() {
        for (j, b) in sides.iter().enumerate() {
            if i != j {
                let h = b.height(&a.origin);
                if !(h.abs() > 1e-9) {
                    return None;
                }
                signs.push(h < 0.0);
            }
        }
    }
    let convex = *signs.first()?;
    signs.iter().all(|s| *s == convex).then_some(convex)
}

/// `p` moved onto the surface of the wedge: onto the side it is highest over (lowest under, in
/// a concave wedge), and that point moved on to the edge or corner when further sides are within
/// `snap` of it (`None` when none is).
fn onto_wedge(sides: &[Side], convex: bool, snap: f64, p: Point3) -> (Point3, Option<Point3>) {
    let sign = if convex { 1.0 } else { -1.0 };
    let top = |q: &Point3| {
        sides
            .iter()
            .map(|s| sign * s.height(q))
            .enumerate()
            .fold(None, |b: Option<(usize, f64)>, c| if b.is_none_or(|b| c.1 > b.1) { Some(c) } else { b })
    };
    let mut q = p;
    for _ in 0..PROJECTIONS {
        let Some((k, h)) = top(&q) else { return (p, None) };
        if h.abs() < 1e-12 {
            break;
        }
        if let Some(s) = sides.get(k) {
            q -= s.n * (sign * h);
        }
    }
    let close: Vec<&Side> = sides.iter().filter(|s| sign * s.height(&q) > -snap).collect();
    if close.len() < 2 {
        return (q, None);
    }
    let mut e = q;
    for _ in 0..PROJECTIONS {
        for s in &close {
            e -= s.n * s.height(&e);
        }
    }
    (q, Some(e))
}

/// Area of the smallest patch triangle (from `first_tri` on) at vertex `v`.
fn thinnest_at(mesh: &Mesh, first_tri: usize, v: u32) -> f64 {
    (first_tri..mesh.triangles.len())
        .filter(|&t| mesh.triangles.get(t).is_some_and(|c| c.contains(&v)))
        .map(|t| mesh.face_area(t))
        .fold(f64::INFINITY, f64::min)
}

/// Moves vertex `v` to `to` unless that leaves a patch triangle thinner than `min_area`.
fn move_keeping_shape(mesh: &mut Mesh, first_tri: usize, v: usize, to: Point3, min_area: f64) {
    let Some(from) = mesh.positions.get(v).copied() else { return };
    if let Some(p) = mesh.positions.get_mut(v) {
        *p = to;
    }
    if thinnest_at(mesh, first_tri, v as u32) < min_area
        && let Some(p) = mesh.positions.get_mut(v)
    {
        *p = from;
    }
}

/// Moves the vertices from `first_vertex` on (the patch of the hole bounded by `lp`, whose
/// triangles start at `first_tri`) onto the sides of the surface around the hole when those meet
/// at sharp edges. Returns whether it did; otherwise the patch is untouched.
pub(crate) fn sharpen_patch(mesh: &mut Mesh, first_tri: usize, first_vertex: usize, lp: &[u32]) -> bool {
    if first_vertex >= mesh.positions.len() || lp.len() < 3 {
        return false;
    }
    let mut near = triangles_near(mesh, first_tri, lp);
    let Some(means) = sides_of(&mut near) else { return false };
    if !meets_sharply(&near) {
        return false;
    }
    let mut sides: Vec<Side> = Vec::with_capacity(means.len());
    for (k, n) in means.iter().enumerate() {
        let mut ids: Vec<u32> = near.iter().filter(|t| t.side == Some(k)).flat_map(|t| t.corners).collect();
        ids.sort_unstable();
        ids.dedup();
        let points: Vec<Point3> = ids.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
        let Some(side) = Side::fit(&points, *n) else { return false };
        sides.push(side);
    }
    let Some(convex) = convexity(&sides) else { return false };
    let on_loop: Vec<Point3> = lp.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
    let n = on_loop.len();
    if n < 3 {
        return false;
    }
    let edge: f64 = (0..n).map(|k| (on_loop[k] - on_loop[(k + 1) % n]).norm()).sum::<f64>() / n as f64;
    let reach = on_loop.iter().flat_map(|a| on_loop.iter().map(move |b| (a - b).norm())).fold(0.0, f64::max);
    let moved: Vec<(Point3, Option<Point3>)> =
        mesh.positions.iter().skip(first_vertex).map(|p| onto_wedge(&sides, convex, EDGE_SNAP * edge, *p)).collect();
    let sane = mesh
        .positions
        .iter()
        .skip(first_vertex)
        .zip(&moved)
        .all(|(p, (q, e))| [Some(*q), *e].iter().flatten().all(|to| crate::finite(to) && (to - p).norm() <= reach));
    if !sane {
        return false;
    }
    for (p, (q, _)) in mesh.positions.iter_mut().skip(first_vertex).zip(&moved) {
        *p = *q;
    }
    // onto the edges one vertex at a time, so two vertices never land on each other
    let min_area = MIN_AREA * edge * edge;
    for (k, (_, e)) in moved.iter().enumerate() {
        if let Some(e) = e {
            move_keeping_shape(mesh, first_tri, first_vertex + k, *e, min_area);
        }
    }
    pin_corner(mesh, first_tri, first_vertex, &sides, &on_loop, reach, min_area);
    align_edges(mesh, first_tri, &sides, convex, min_area);
    true
}

/// Where three sides meet there is a corner, and a patch has to have a vertex on it or a
/// triangle cuts it off: the patch vertex nearest the corner is moved there.
fn pin_corner(
    mesh: &mut Mesh,
    first_tri: usize,
    first_vertex: usize,
    sides: &[Side],
    on_loop: &[Point3],
    reach: f64,
    min_area: f64,
) {
    if sides.len() != MAX_SIDES || on_loop.is_empty() {
        return;
    }
    let middle = Point3::from(on_loop.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / on_loop.len() as f64);
    let mut corner = middle;
    for _ in 0..PROJECTIONS {
        for s in sides {
            corner -= s.n * s.height(&corner);
        }
    }
    // three sides need not meet in a point (a chamfer between a face and a wall): no corner then
    let off = sides.iter().map(|s| s.height(&corner).abs()).fold(0.0, f64::max);
    if !crate::finite(&corner) || (corner - middle).norm() > reach || !(off <= CORNER_TOLERANCE * reach) {
        return;
    }
    let nearest = mesh
        .positions
        .iter()
        .enumerate()
        .skip(first_vertex)
        .map(|(i, p)| ((p - corner).norm(), i))
        .fold(None, |b: Option<(f64, usize)>, c| if b.is_none_or(|b| c.0 < b.0) { Some(c) } else { b });
    if let Some((_, i)) = nearest {
        move_keeping_shape(mesh, first_tri, i, corner, min_area);
    }
}

/// Distance of `p` from the surface of the wedge (zero on a side that is the outermost there).
fn off_wedge(sides: &[Side], convex: bool, p: &Point3) -> f64 {
    let sign = if convex { 1.0 } else { -1.0 };
    sides.iter().map(|s| sign * s.height(p)).fold(f64::NEG_INFINITY, f64::max).abs()
}

/// Flips patch edges that cut across a sharp edge of the wedge: an edge between two patch
/// triangles is swapped for the other diagonal when that one's midpoint is at most
/// [`FLIP_GAIN`] × as far from the wedge and the two new triangles keep the orientation.
fn align_edges(mesh: &mut Mesh, first_tri: usize, sides: &[Side], convex: bool, min_area: f64) {
    for _pass in 0..FLIP_PASSES {
        let mut edge: HashMap<(u32, u32), (usize, usize)> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate().skip(first_tri) {
            for k in 0..3 {
                edge.insert((t[k], t[(k + 1) % 3]), (ti, k));
            }
        }
        let mut keys: Vec<(u32, u32)> = edge.keys().copied().filter(|(a, b)| a < b).collect();
        keys.sort_unstable();
        let mut done: HashSet<usize> = HashSet::new();
        let mut flipped = false;
        for (a, b) in keys {
            let (Some(&(t0, k0)), Some(&(t1, k1))) = (edge.get(&(a, b)), edge.get(&(b, a))) else { continue };
            if done.contains(&t0) || done.contains(&t1) {
                continue;
            }
            let (Some(c), Some(d)) =
                (mesh.triangles.get(t0).map(|t| t[(k0 + 2) % 3]), mesh.triangles.get(t1).map(|t| t[(k1 + 2) % 3]))
            else {
                continue;
            };
            if c == d || edge.contains_key(&(c, d)) || edge.contains_key(&(d, c)) {
                continue;
            }
            let at = |v: u32| mesh.positions.get(v as usize).copied();
            let (Some(pa), Some(pb), Some(pc), Some(pd)) = (at(a), at(b), at(c), at(d)) else { continue };
            let mid = |p: &Point3, q: &Point3| Point3::from((p.coords + q.coords) / 2.0);
            let (now, then) = (off_wedge(sides, convex, &mid(&pa, &pb)), off_wedge(sides, convex, &mid(&pc, &pd)));
            if !(then < FLIP_GAIN * now) {
                continue;
            }
            // the new triangles (c, d, b) and (d, c, a) must face the way the old pair did
            let old = tri::cross(&pa, &pb, &pc) + tri::cross(&pb, &pa, &pd);
            let (n1, n2) = (tri::cross(&pc, &pd, &pb), tri::cross(&pd, &pc, &pa));
            if n1.dot(&old) <= 0.0 || n2.dot(&old) <= 0.0 || n1.norm().min(n2.norm()) < 2.0 * min_area {
                continue;
            }
            if let Some(t) = mesh.triangles.get_mut(t0) {
                *t = [c, d, b];
            }
            if let Some(t) = mesh.triangles.get_mut(t1) {
                *t = [d, c, a];
            }
            done.insert(t0);
            done.insert(t1);
            flipped = true;
        }
        if !flipped {
            break;
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::edit::{boundary_loops, fill_holes};
    use crate::{Mesh, Point3, Rng, Vec3, synth};

    const SIDE: f64 = 20.0;

    /// Distance from `p` to the surface of the cube [0, SIDE]³.
    fn to_cube(p: &Point3) -> f64 {
        let q = [
            (p.x - SIDE / 2.0).abs() - SIDE / 2.0,
            (p.y - SIDE / 2.0).abs() - SIDE / 2.0,
            (p.z - SIDE / 2.0).abs() - SIDE / 2.0,
        ];
        let outside = q.iter().map(|c| c.max(0.0).powi(2)).sum::<f64>().sqrt();
        let inside = q.iter().copied().fold(f64::NEG_INFINITY, f64::max).min(0.0);
        (outside + inside).abs()
    }

    /// The cube scan (1.25 mm triangles, noise `sigma`) with a ball of radius `r` around `at`
    /// cut out and filled again; returns the mesh and the worst distance of a vertex and of a
    /// triangle centroid from the cube.
    fn cut_and_fill(at: Point3, r: f64, sigma: f64) -> (Mesh, f64, f64) {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(SIDE, SIDE, SIDE), 16);
        m.weld(1e-9);
        if sigma > 0.0 {
            synth::add_noise(&mut m, sigma, &mut Rng::new(5));
        }
        let keep: Vec<u32> = (0..m.triangle_count() as u32)
            .filter(|&t| m.face_centroid(t as usize).is_some_and(|c| (c - at).norm() > r))
            .collect();
        let mut cut = m.submesh(&keep);
        assert_eq!(boundary_loops(&cut).len(), 1);
        let rep = fill_holes(&mut cut, 1000, true);
        assert_eq!(rep.filled, 1);
        assert!(boundary_loops(&cut).is_empty());
        let thinnest = (0..cut.triangle_count()).map(|t| cut.face_area(t)).fold(f64::INFINITY, f64::min);
        assert!(thinnest > 1e-4, "a triangle of area {thinnest} mm² is left");
        let vertices = cut.positions.iter().map(to_cube).fold(0.0, f64::max);
        let centroids =
            (0..cut.triangle_count()).filter_map(|t| cut.face_centroid(t)).map(|c| to_cube(&c)).fold(0.0, f64::max);
        (cut, vertices, centroids)
    }

    #[test]
    fn a_hole_over_a_sharp_edge_is_filled_with_the_edge() {
        let (_, vertices, centroids) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.0);
        assert!(vertices < 1e-6, "a patch vertex is {vertices} mm off the cube");
        assert!(centroids < 0.05, "a patch triangle is {centroids} mm off the cube");
    }

    #[test]
    fn a_hole_over_a_corner_is_filled_with_the_corner() {
        let (_, vertices, centroids) = cut_and_fill(Point3::new(0.0, 0.0, SIDE), 3.0, 0.0);
        assert!(vertices < 1e-6, "a patch vertex is {vertices} mm off the cube");
        assert!(centroids < 0.1, "a patch triangle is {centroids} mm off the cube");
    }

    #[test]
    fn a_noisy_hole_over_a_sharp_edge_stays_within_the_noise() {
        let (_, vertices, _) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.03);
        assert!(vertices < 0.12, "a vertex is {vertices} mm off the cube at σ = 0.03");
    }

    #[test]
    fn the_fill_is_deterministic() {
        let (a, _, _) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.03);
        let (b, _, _) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.03);
        assert_eq!(a, b);
    }
}
