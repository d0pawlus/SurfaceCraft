//! Hole patches that keep sharp edges.
//!
//! A hole that straddles a sharp edge (a spike removed on the edge of a part, missing data on a
//! corner) is first filled and faired like any other, which rounds the edge off into a chamfer.
//! [`sharpen_patch`] then looks at the surface the hole's border runs through: when it falls
//! into two or three smooth, connected sides that meet at sharp edges, each side is extended over
//! the hole (a plane, or a quadric when the side is curved) and the patch vertices are moved
//! onto the extended sides, the ones next to an edge onto the edge itself. A hole in a smooth
//! surface has one side and is left as the fairing made it, and so is any patch the move would
//! turn over or squash.

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
/// Smallest share of the area next to the hole's border that makes a group of triangles a side.
const MIN_SIDE_SHARE: f64 = 0.1;
/// Rings of triangles around the hole's border a side is fitted to (the first decides the sides).
const SUPPORT_RINGS: usize = 2;
/// Fewest points a quadric (six terms) is fitted to; below it the side is a plane. With fewer,
/// a thin band of a flat side now and then bends by noise alone and the quadric, carried over
/// the hole, curls away from the side.
const MIN_QUADRIC_POINTS: usize = 20;
/// A quadric replaces the plane only when it leaves less than this share of the plane's RMS,
/// each RMS taken over the points left after the terms fitted (so three more terms that only
/// follow the noise gain nothing).
const QUADRIC_GAIN: f64 = 0.8;
/// Triangles of one normal further apart along it than this (in mean loop edge lengths) are on
/// two sides.
const SAME_SIDE_STEP: f64 = 0.5;
/// A side whose fit leaves more than this RMS (in mean loop edge lengths) is not one surface.
const MAX_SIDE_RMS: f64 = 0.25;
/// Sides closer to each other's middle than this (in mean loop edge lengths) make no wedge.
const MIN_WEDGE: f64 = 1e-6;
/// A patch vertex this close to a second side (in mean loop edge lengths) goes onto the edge.
const EDGE_SNAP: f64 = 0.5;
/// Projection rounds for one vertex.
const PROJECTIONS: usize = 12;
/// A point is on a side within this many mean loop edge lengths.
const ON_SIDE: f64 = 1e-12;
/// Three sides meet in a corner when a point is on all of them within this share of the hole's
/// size.
const CORNER_TOLERANCE: f64 = 1e-6;
/// Smallest area of a patch triangle after its vertices move, in squared mean loop edge lengths
/// (an equilateral triangle of that edge has 0.43).
const MIN_AREA: f64 = 0.02;
/// The moved patch is kept only when its thinnest triangle has at least this share of the area
/// of the thinnest one before (or [`MIN_AREA`]).
const MIN_AREA_KEPT: f64 = 0.5;
/// A patch triangle faces the way a side does when the cosine between their normals is above
/// this.
const MIN_FACING: f64 = 0.1;
/// Rounds of untangling a patch whose triangles the move turned over.
const UNTANGLE_ROUNDS: usize = 20;
/// A patch edge is flipped when the other diagonal is at most this share as far from the wedge.
const FLIP_GAIN: f64 = 0.5;
/// Passes of edge flips over a patch.
const FLIP_PASSES: usize = 8;

/// Terms of the height polynomial of a side: 1, x, y, x², xy, y².
const TERMS: usize = 6;
/// Terms of a plane.
const PLANE_TERMS: usize = 3;

/// The triangles around each vertex of a mesh (as it is when this is built).
pub(crate) struct Around {
    start: Vec<u32>,
    tris: Vec<u32>,
}

impl Around {
    /// Indexes `mesh`; triangles with a vertex out of range are left out.
    pub(crate) fn new(mesh: &Mesh) -> Self {
        let n = mesh.positions.len();
        let valid = |t: &[u32; 3]| t.iter().all(|&v| (v as usize) < n);
        let mut start = vec![0u32; n + 1];
        for t in mesh.triangles.iter().filter(|t| valid(t)) {
            for &v in t {
                if let Some(c) = start.get_mut(v as usize + 1) {
                    *c += 1;
                }
            }
        }
        for i in 1..start.len() {
            start[i] += start[i - 1];
        }
        let mut next = start.clone();
        let mut tris = vec![0u32; start.last().copied().unwrap_or(0) as usize];
        for (i, t) in mesh.triangles.iter().enumerate().filter(|(_, t)| valid(t)) {
            for &v in t {
                if let Some(slot) = next.get_mut(v as usize) {
                    if let Some(place) = tris.get_mut(*slot as usize) {
                        *place = i as u32;
                    }
                    *slot += 1;
                }
            }
        }
        Self { start, tris }
    }

    fn of(&self, v: u32) -> &[u32] {
        let (Some(&a), Some(&b)) = (self.start.get(v as usize), self.start.get(v as usize + 1)) else { return &[] };
        self.tris.get(a as usize..b as usize).unwrap_or(&[])
    }
}

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
    let terms = terms.min(TERMS);
    let mut a = SMatrix::<f64, TERMS, TERMS>::zeros();
    let mut b = SVector::<f64, TERMS>::zeros();
    for &(x, y, z) in local {
        let f = basis(x, y);
        for i in 0..terms {
            for j in 0..terms {
                a[(i, j)] += f[i] * f[j];
            }
            b[i] += f[i] * z;
        }
    }
    for i in terms..TERMS {
        a[(i, i)] = 1.0;
    }
    let c = a.lu().solve(&b)?;
    let coef: [f64; TERMS] = std::array::from_fn(|i| c[i]);
    if coef.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let height = |x: f64, y: f64| basis(x, y).iter().zip(&coef).map(|(f, c)| f * c).sum::<f64>();
    let sq: f64 = local.iter().map(|&(x, y, z)| (z - height(x, y)).powi(2)).sum();
    Some((coef, (sq / local.len().max(1) as f64).sqrt()))
}

/// Do `count` points bend away from their plane: does the quadric leave less than
/// [`QUADRIC_GAIN`] of the plane's RMS, per point left free?
fn bends(count: usize, plane_rms: f64, quadric_rms: f64) -> bool {
    let free = |terms: usize| count.saturating_sub(terms).max(1) as f64;
    quadric_rms / free(TERMS).sqrt() < QUADRIC_GAIN * plane_rms / free(PLANE_TERMS).sqrt()
}

impl Side {
    /// The side through `points` with mean normal `n`, and the RMS it leaves: a plane, or a
    /// quadric when the points are enough and bend away from the plane.
    fn fit(points: &[Point3], n: Vec3) -> Option<(Side, f64)> {
        if points.len() < PLANE_TERMS {
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
        let (plane, plane_rms) = fit_height(&local, PLANE_TERMS)?;
        let (coef, rms) = match fit_height(&local, TERMS) {
            Some((quadric, rms)) if points.len() >= MIN_QUADRIC_POINTS && bends(points.len(), plane_rms, rms) => {
                (quadric, rms)
            }
            _ => (plane, plane_rms),
        };
        Some((Side { origin, n, u, v, coef }, rms))
    }

    /// Height of `p` over the side, along the side's normal.
    fn height(&self, p: &Point3) -> f64 {
        let d = p - self.origin;
        let f = basis(d.dot(&self.u), d.dot(&self.v));
        d.dot(&self.n) - f.iter().zip(&self.coef).map(|(f, c)| f * c).sum::<f64>()
    }
}

/// The sides of the surface around a hole and whether they make a convex or a concave wedge.
struct Wedge {
    sides: Vec<Side>,
    convex: bool,
}

impl Wedge {
    fn sign(&self) -> f64 {
        if self.convex { 1.0 } else { -1.0 }
    }

    /// The side `p` is highest over (lowest under, in a concave wedge) and that signed height.
    fn top(&self, p: &Point3) -> Option<(usize, f64)> {
        self.sides
            .iter()
            .map(|s| self.sign() * s.height(p))
            .enumerate()
            .fold(None, |b: Option<(usize, f64)>, c| if b.is_none_or(|b| c.1 > b.1) { Some(c) } else { b })
    }

    /// Distance of `p` from the surface of the wedge.
    fn off(&self, p: &Point3) -> f64 {
        self.top(p).map_or(f64::INFINITY, |(_, h)| h.abs())
    }

    /// `p` moved onto the surface of the wedge, and that point moved on to the edge or corner
    /// when further sides are within `snap` of it (`None` when none is).
    fn onto(&self, snap: f64, on_side: f64, p: Point3) -> (Point3, Option<Point3>) {
        let mut q = p;
        for _ in 0..PROJECTIONS {
            let Some((k, h)) = self.top(&q) else { return (p, None) };
            if h.abs() < on_side {
                break;
            }
            if let Some(s) = self.sides.get(k) {
                q -= s.n * (self.sign() * h);
            }
        }
        let close: Vec<&Side> = self.sides.iter().filter(|s| self.sign() * s.height(&q) > -snap).collect();
        if close.len() < 2 {
            return (q, None);
        }
        (q, Some(onto_all(&close, q)))
    }
}

/// `p` moved onto every one of `sides` at once, by projecting onto them in turn.
fn onto_all(sides: &[&Side], p: Point3) -> Point3 {
    let mut q = p;
    for _ in 0..PROJECTIONS {
        for s in sides {
            q -= s.n * s.height(&q);
        }
    }
    q
}

/// A triangle next to the hole: its vertices, unit normal, area, middle, whether it touches the hole's
/// border, and its group of normals and its side (once known).
struct Near {
    corners: [u32; 3],
    normal: Vec3,
    area: f64,
    centre: Point3,
    at_border: bool,
    group: Option<usize>,
    side: Option<usize>,
}

/// The triangles within [`SUPPORT_RINGS`] rings of the loop, in index order.
fn triangles_near(mesh: &Mesh, around: &Around, lp: &[u32]) -> Vec<Near> {
    let mut ring_of: HashMap<u32, usize> = HashMap::new();
    let mut front: Vec<u32> = lp.to_vec();
    for ring in 0..SUPPORT_RINGS {
        let mut next: Vec<u32> = Vec::new();
        for &v in &front {
            for &t in around.of(v) {
                if let std::collections::hash_map::Entry::Vacant(e) = ring_of.entry(t) {
                    e.insert(ring);
                    next.extend(mesh.triangles.get(t as usize).into_iter().flatten());
                }
            }
        }
        front = next;
    }
    let mut picked: Vec<(u32, usize)> = ring_of.into_iter().collect();
    picked.sort_unstable();
    picked
        .into_iter()
        .filter_map(|(i, ring)| {
            let corners = *mesh.triangles.get(i as usize)?;
            let [a, b, c] = mesh.triangle(i as usize)?;
            let (normal, area) = (tri::normal(&a, &b, &c)?, tri::area(&a, &b, &c));
            let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
            Some(Near { corners, normal, area, centre, at_border: ring == 0, group: None, side: None })
        })
        .collect()
}

/// Groups the triangles by normal (the groups are those of the triangles at the hole's border)
/// and returns the mean normal of each group. `None` unless there are at least two groups
/// further apart than the feature angle.
fn group_by_normal(near: &mut [Near]) -> Option<Vec<Vec3>> {
    let cos = FEATURE_DEG.to_radians().cos();
    let mut order: Vec<usize> = (0..near.len()).filter(|&i| near.get(i).is_some_and(|t| t.at_border)).collect();
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
    if means.len() < 2 {
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
        t.group = best.filter(|b| b.0 >= cos).map(|b| b.1);
    }
    Some(means)
}

/// The triangles on each mesh edge among `near`.
fn edges_of(near: &[Near]) -> HashMap<(u32, u32), Vec<usize>> {
    let mut by_edge: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in near.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t.corners[k], t.corners[(k + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    by_edge
}

/// Splits each group of normals where its triangles step along the group's normal by more
/// than [`SAME_SIDE_STEP`] mean loop edge lengths (the two faces of a step have one normal and
/// are two sides; a curved side has no such step) and makes the pieces that hold enough of the
/// hole's border the sides. Returns the mean normal of each side; `None` unless there are two or
/// three.
fn split_sides(near: &mut [Near], means: &[Vec3], edge: f64) -> Option<Vec<Vec3>> {
    // (area at the border, area-weighted normal, triangles)
    let mut pieces: Vec<(f64, Vec3, Vec<usize>)> = Vec::new();
    for (g, mean) in means.iter().enumerate() {
        let mut members: Vec<(f64, usize)> = near
            .iter()
            .enumerate()
            .filter(|(_, t)| t.group == Some(g))
            .map(|(i, t)| (t.centre.coords.dot(mean), i))
            .collect();
        members.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut last: Option<f64> = None;
        for (d, i) in members {
            if last.is_none_or(|l| d - l > SAME_SIDE_STEP * edge) {
                pieces.push((0.0, Vec3::zeros(), Vec::new()));
            }
            last = Some(d);
            if let (Some(p), Some(t)) = (pieces.last_mut(), near.get(i)) {
                p.0 += if t.at_border { t.area } else { 0.0 };
                p.1 += t.normal * t.area;
                p.2.push(i);
            }
        }
    }
    let total: f64 = pieces.iter().map(|p| p.0).sum();
    pieces.retain(|p| p.0 >= MIN_SIDE_SHARE * total && p.0 > 0.0);
    if pieces.len() < 2 || pieces.len() > MAX_SIDES {
        return None;
    }
    for (k, p) in pieces.iter().enumerate() {
        for &i in &p.2 {
            if let Some(t) = near.get_mut(i) {
                t.side = Some(k);
            }
        }
    }
    pieces.iter().map(|p| crate::normalize(p.1)).collect()
}

/// Do the sides meet at sharp edges? The median turn across the mesh edges that join triangles
/// of different sides must exceed the feature angle (a smooth surface that merely curves a lot
/// around the hole turns a little at each edge).
fn meets_sharply(near: &[Near], by_edge: &HashMap<(u32, u32), Vec<usize>>) -> bool {
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

/// Is the wedge of the sides convex (`Some(true)`), concave (`Some(false)`) or neither? Every
/// side's middle must be under (over) every other side by more than `tiny`.
fn convexity(sides: &[Side], tiny: f64) -> Option<bool> {
    let mut signs: Vec<bool> = Vec::new();
    for (i, a) in sides.iter().enumerate() {
        for (j, b) in sides.iter().enumerate() {
            if i != j {
                let h = b.height(&a.origin);
                if !(h.abs() > tiny) {
                    return None;
                }
                signs.push(h < 0.0);
            }
        }
    }
    let convex = *signs.first()?;
    signs.iter().all(|s| *s == convex).then_some(convex)
}

/// Reads the surface around the hole as a wedge of two or three sides; the reason when it is
/// one smooth surface or cannot be read (`edge` is the mean loop edge length).
fn read_wedge(mesh: &Mesh, around: &Around, lp: &[u32], edge: f64) -> Result<Wedge, &'static str> {
    let mut near = triangles_near(mesh, around, lp);
    let means = group_by_normal(&mut near).ok_or("one group of normals")?;
    let normals = split_sides(&mut near, &means, edge).ok_or("not two or three sides")?;
    let by_edge = edges_of(&near);
    if !meets_sharply(&near, &by_edge) {
        return Err("the sides do not meet sharply");
    }
    let mut sides: Vec<Side> = Vec::with_capacity(normals.len());
    for (k, n) in normals.iter().enumerate() {
        let mut ids: Vec<u32> = near.iter().filter(|t| t.side == Some(k)).flat_map(|t| t.corners).collect();
        ids.sort_unstable();
        ids.dedup();
        let points: Vec<Point3> = ids.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
        let (side, rms) = Side::fit(&points, *n).ok_or("a side has no fit")?;
        if !(rms <= MAX_SIDE_RMS * edge) {
            return Err("a side is not one surface");
        }
        sides.push(side);
    }
    let convex = convexity(&sides, MIN_WEDGE * edge).ok_or("neither convex nor concave")?;
    Ok(Wedge { sides, convex })
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

/// The hole: its patch and the measures of its border.
struct Hole {
    first_tri: usize,
    first_vertex: usize,
    /// Middle of the border.
    middle: Point3,
    /// Largest distance of a border point from the middle.
    radius: f64,
    /// Mean length of a border edge.
    edge: f64,
}

impl Hole {
    fn new(mesh: &Mesh, first_tri: usize, first_vertex: usize, lp: &[u32]) -> Option<Hole> {
        let on: Vec<Point3> = lp.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
        if on.len() < 3 || on.len() != lp.len() || first_vertex >= mesh.positions.len() {
            return None;
        }
        let n = on.len();
        let middle = Point3::from(on.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / n as f64);
        let radius = on.iter().map(|p| (p - middle).norm()).fold(0.0, f64::max);
        let edge = (0..n).filter_map(|k| Some((on.get(k)? - on.get((k + 1) % n)?).norm())).sum::<f64>() / n as f64;
        (edge > 0.0 && radius.is_finite()).then_some(Hole { first_tri, first_vertex, middle, radius, edge })
    }

    fn min_area(&self) -> f64 {
        MIN_AREA * self.edge * self.edge
    }
}

/// Where three sides meet in a point inside the hole there is a corner, and a patch has to have
/// a vertex on it or a triangle cuts it off: the patch vertex nearest the corner is moved there.
fn pin_corner(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge) {
    if wedge.sides.len() != MAX_SIDES {
        return;
    }
    let all: Vec<&Side> = wedge.sides.iter().collect();
    let corner = onto_all(&all, hole.middle);
    // three sides need not meet in a point (a chamfer between a face and a wall), and a corner
    // outside the hole is not the patch's to make
    let off = wedge.sides.iter().map(|s| s.height(&corner).abs()).fold(0.0, f64::max);
    if !crate::finite(&corner)
        || (corner - hole.middle).norm() > hole.radius
        || !(off <= CORNER_TOLERANCE * hole.radius)
    {
        return;
    }
    let nearest = mesh
        .positions
        .iter()
        .enumerate()
        .skip(hole.first_vertex)
        .map(|(i, p)| ((p - corner).norm(), i))
        .fold(None, |b: Option<(f64, usize)>, c| if b.is_none_or(|b| c.0 < b.0) { Some(c) } else { b });
    if let Some((_, i)) = nearest {
        move_keeping_shape(mesh, hole.first_tri, i, corner, hole.min_area());
    }
}

/// The two patch triangles on edge (a, b) and the vertices opposite it, when it is an inner
/// patch edge that may be flipped.
type Flip = (usize, usize, u32, u32);

fn flip_of(mesh: &Mesh, edge: &HashMap<(u32, u32), (usize, usize)>, a: u32, b: u32) -> Option<Flip> {
    let (&(t0, k0), &(t1, k1)) = (edge.get(&(a, b))?, edge.get(&(b, a))?);
    let c = mesh.triangles.get(t0)?[(k0 + 2) % 3];
    let d = mesh.triangles.get(t1)?[(k1 + 2) % 3];
    (c != d && !edge.contains_key(&(c, d)) && !edge.contains_key(&(d, c))).then_some((t0, t1, c, d))
}

/// Should edge (a, b) between triangles (a, b, c) and (b, a, d) become (c, d)? Yes when the
/// middle of (c, d) is at most [`FLIP_GAIN`] × as far from the wedge and the two new triangles
/// keep the orientation and are not slivers.
fn flip_gains(mesh: &Mesh, wedge: &Wedge, min_area: f64, [a, b, c, d]: [u32; 4]) -> bool {
    let at = |v: u32| mesh.positions.get(v as usize).copied();
    let (Some(pa), Some(pb), Some(pc), Some(pd)) = (at(a), at(b), at(c), at(d)) else { return false };
    let mid = |p: &Point3, q: &Point3| Point3::from((p.coords + q.coords) / 2.0);
    if !(wedge.off(&mid(&pc, &pd)) < FLIP_GAIN * wedge.off(&mid(&pa, &pb))) {
        return false;
    }
    // the new triangles (c, d, b) and (d, c, a) must face the way the old pair did
    let old = tri::cross(&pa, &pb, &pc) + tri::cross(&pb, &pa, &pd);
    let (n1, n2) = (tri::cross(&pc, &pd, &pb), tri::cross(&pd, &pc, &pa));
    n1.dot(&old) > 0.0 && n2.dot(&old) > 0.0 && n1.norm().min(n2.norm()) >= 2.0 * min_area
}

/// Flips patch edges that cut across a sharp edge of the wedge (see [`flip_gains`]).
fn align_edges(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge) {
    for _pass in 0..FLIP_PASSES {
        let mut edge: HashMap<(u32, u32), (usize, usize)> = HashMap::new();
        for (ti, t) in mesh.triangles.iter().enumerate().skip(hole.first_tri) {
            for k in 0..3 {
                edge.insert((t[k], t[(k + 1) % 3]), (ti, k));
            }
        }
        let mut keys: Vec<(u32, u32)> = edge.keys().copied().filter(|(a, b)| a < b).collect();
        keys.sort_unstable();
        let mut done: HashSet<usize> = HashSet::new();
        let mut made: HashSet<(u32, u32)> = HashSet::new();
        for (a, b) in keys {
            let Some((t0, t1, c, d)) = flip_of(mesh, &edge, a, b) else { continue };
            let fresh = !done.contains(&t0) && !done.contains(&t1) && !made.contains(&(c.min(d), c.max(d)));
            if !fresh || !flip_gains(mesh, wedge, hole.min_area(), [a, b, c, d]) {
                continue;
            }
            if let Some(t) = mesh.triangles.get_mut(t0) {
                *t = [c, d, b];
            }
            if let Some(t) = mesh.triangles.get_mut(t1) {
                *t = [d, c, a];
            }
            done.extend([t0, t1]);
            made.insert((c.min(d), c.max(d)));
        }
        if done.is_empty() {
            break;
        }
    }
}

/// Area of the thinnest patch triangle.
fn thinnest(mesh: &Mesh, first_tri: usize) -> f64 {
    (first_tri..mesh.triangles.len()).map(|t| mesh.face_area(t)).fold(f64::INFINITY, f64::min)
}

/// Does patch triangle `t` face the way a side at its middle does? At an edge the middle is on
/// two sides at once (within the snap distance), and either will do.
fn faces_out(mesh: &Mesh, hole: &Hole, wedge: &Wedge, t: usize) -> bool {
    let snap = EDGE_SNAP * hole.edge;
    let (Some(c), Some(n)) = (mesh.face_centroid(t), mesh.face_normal(t)) else { return false };
    let Some((_, top)) = wedge.top(&c) else { return false };
    wedge.sides.iter().any(|s| wedge.sign() * s.height(&c) >= top - snap && s.n.dot(&n) > MIN_FACING)
}

/// What a sound patch is measured against: the least area of a triangle, and the triangles
/// between border vertices alone that were turned over as the fairing left them. Those have no
/// vertex that can move, so no move makes or mends them.
struct Sound {
    min_area: f64,
    given: HashSet<[u32; 3]>,
}

impl Sound {
    fn new(mesh: &Mesh, hole: &Hole, wedge: &Wedge, min_area: f64) -> Sound {
        let open = Sound { min_area, given: HashSet::new() };
        let given = tangled(mesh, hole, wedge, &open)
            .into_iter()
            .filter_map(|t| mesh.triangles.get(t))
            .filter(|t| t.iter().all(|&v| (v as usize) < hole.first_vertex))
            .map(same_corners)
            .collect();
        Sound { min_area, given }
    }
}

/// The corners of a triangle in an order that does not depend on where it starts.
fn same_corners(t: &[u32; 3]) -> [u32; 3] {
    let mut c = *t;
    c.sort_unstable();
    c
}

/// The patch triangles that are thinner than the least area or turned over. A triangle thinner
/// than the hole's own least area (the fairing leaves such slivers) has no direction worth the
/// name and is judged by its area alone.
fn tangled(mesh: &Mesh, hole: &Hole, wedge: &Wedge, sound: &Sound) -> Vec<usize> {
    (hole.first_tri..mesh.triangles.len())
        .filter(|&t| {
            let area = mesh.face_area(t);
            let given = mesh.triangles.get(t).is_some_and(|c| sound.given.contains(&same_corners(c)));
            area < sound.min_area || (area >= hole.min_area() && !given && !faces_out(mesh, hole, wedge, t))
        })
        .collect()
}

/// Untangles the patch after its vertices moved: the patch vertices of every turned-over or
/// squashed triangle go to the middle of their neighbours and back onto the wedge, for up to
/// [`UNTANGLE_ROUNDS`] rounds. Returns whether the patch is sound.
fn untangle(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge, sound: &Sound, snap: f64) -> bool {
    let on_side = ON_SIDE * hole.edge;
    for _ in 0..UNTANGLE_ROUNDS {
        let bad = tangled(mesh, hole, wedge, sound);
        if bad.is_empty() {
            return true;
        }
        let mut ring: HashMap<u32, Vec<u32>> = HashMap::new();
        for t in mesh.triangles.iter().skip(hole.first_tri) {
            for k in 0..3 {
                ring.entry(t[k]).or_default().extend([t[(k + 1) % 3], t[(k + 2) % 3]]);
            }
        }
        let mut loose: Vec<u32> = bad.iter().filter_map(|&t| mesh.triangles.get(t)).flatten().copied().collect();
        loose.retain(|&v| v as usize >= hole.first_vertex);
        loose.sort_unstable();
        loose.dedup();
        for v in loose {
            let Some(around) = ring.get_mut(&v) else { continue };
            around.sort_unstable();
            around.dedup();
            let near: Vec<Point3> = around.iter().filter_map(|&w| mesh.positions.get(w as usize).copied()).collect();
            if near.is_empty() {
                continue;
            }
            let middle = Point3::from(near.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / near.len() as f64);
            let (on, at_edge) = wedge.onto(snap, on_side, middle);
            if let Some(p) = mesh.positions.get_mut(v as usize) {
                *p = at_edge.unwrap_or(on);
            }
        }
    }
    tangled(mesh, hole, wedge, sound).is_empty()
}

/// Moves the patch vertices onto the wedge; false when a vertex would leave the hole's reach.
fn move_onto(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge, snap: f64) -> bool {
    let on_side = ON_SIDE * hole.edge;
    let moved: Vec<(Point3, Option<Point3>)> =
        mesh.positions.iter().skip(hole.first_vertex).map(|p| wedge.onto(snap, on_side, *p)).collect();
    let reach = 2.0 * hole.radius;
    let sane = mesh
        .positions
        .iter()
        .skip(hole.first_vertex)
        .zip(&moved)
        .all(|(p, (q, e))| [Some(*q), *e].iter().flatten().all(|to| crate::finite(to) && (to - p).norm() <= reach));
    if !sane {
        return false;
    }
    for (p, (q, _)) in mesh.positions.iter_mut().skip(hole.first_vertex).zip(&moved) {
        *p = *q;
    }
    // onto the edges one vertex at a time, so two vertices never land on each other
    for (k, (_, e)) in moved.iter().enumerate() {
        if let Some(e) = e {
            move_keeping_shape(mesh, hole.first_tri, hole.first_vertex + k, *e, hole.min_area());
        }
    }
    true
}

/// Moves the vertices from `first_vertex` on (the patch of the hole bounded by `lp`, whose
/// triangles start at `first_tri`) onto the sides of the surface around the hole when those meet
/// at sharp edges. `around` indexes the mesh as it was before holes were filled. Returns whether
/// it did; otherwise the patch is as it was.
pub(crate) fn sharpen_patch(
    mesh: &mut Mesh,
    around: &Around,
    first_tri: usize,
    first_vertex: usize,
    lp: &[u32],
) -> bool {
    let done = sharpen(mesh, around, first_tri, first_vertex, lp);
    #[cfg(test)]
    LAST_REASON.with(|r| r.set(done.err()));
    done.is_ok()
}

#[cfg(test)]
thread_local! {
    /// Why the last patch on this thread was left as it was (`None`: it was sharpened).
    static LAST_REASON: std::cell::Cell<Option<&'static str>> = const { std::cell::Cell::new(None) };
}

/// [`sharpen_patch`], with the reason when the patch is left as it was.
fn sharpen(
    mesh: &mut Mesh,
    around: &Around,
    first_tri: usize,
    first_vertex: usize,
    lp: &[u32],
) -> Result<(), &'static str> {
    let hole = Hole::new(mesh, first_tri, first_vertex, lp).ok_or("no patch vertices")?;
    let wedge = read_wedge(mesh, around, lp, hole.edge)?;
    let before = patch_of(mesh, &hole);
    let min_area = (MIN_AREA_KEPT * thinnest(mesh, first_tri)).min(hole.min_area());
    let sound = Sound::new(mesh, &hole, &wedge, min_area);
    let done = shape_patch(mesh, &hole, &wedge, &sound);
    if done.is_err() {
        restore(mesh, &hole, before);
    }
    done
}

/// The patch as it is: its vertices and triangles.
fn patch_of(mesh: &Mesh, hole: &Hole) -> (Vec<Point3>, Vec<[u32; 3]>) {
    (
        mesh.positions.iter().skip(hole.first_vertex).copied().collect(),
        mesh.triangles.iter().skip(hole.first_tri).copied().collect(),
    )
}

/// Puts the patch back as [`patch_of`] saw it.
fn restore(mesh: &mut Mesh, hole: &Hole, (positions, triangles): (Vec<Point3>, Vec<[u32; 3]>)) {
    for (p, q) in mesh.positions.iter_mut().skip(hole.first_vertex).zip(positions) {
        *p = q;
    }
    for (t, u) in mesh.triangles.iter_mut().skip(hole.first_tri).zip(triangles) {
        *t = u;
    }
}

/// Moves the patch onto the wedge and untangles it, with the vertices next to an edge on the
/// edge or, when that tangles the patch, without. Then pins the corner and aligns the edges,
/// which is undone when it tangles the patch again (the vertices stay on the sides).
fn shape_patch(mesh: &mut Mesh, hole: &Hole, wedge: &Wedge, sound: &Sound) -> Result<(), &'static str> {
    let faired = patch_of(mesh, hole);
    let mut snap = EDGE_SNAP * hole.edge;
    if !move_onto(mesh, hole, wedge, snap) {
        return Err("a vertex would leave the hole");
    }
    if !untangle(mesh, hole, wedge, sound, snap) {
        restore(mesh, hole, faired);
        snap = 0.0;
        if !(move_onto(mesh, hole, wedge, snap) && untangle(mesh, hole, wedge, sound, snap)) {
            return Err("tangled after the move");
        }
    }
    let moved = patch_of(mesh, hole);
    pin_corner(mesh, hole, wedge);
    align_edges(mesh, hole, wedge);
    if !untangle(mesh, hole, wedge, sound, snap) {
        restore(mesh, hole, moved);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::edit::{boundary_loops, fill_holes, fill_holes_with};
    use crate::{Mesh, Point3, Rng, Vec3, synth};

    const SIDE: f64 = 20.0;

    /// A cone side read with points of its neighbour (RMS 0.101 as a plane, 0.064 as a quadric)
    /// is a quadric; a flat side whose quadric only follows the noise stays a plane.
    #[test]
    fn a_side_bends_when_the_quadric_gains_beyond_its_extra_terms() {
        assert!(super::bends(79, 0.1007, 0.0640));
        assert!(!super::bends(92, 0.0276, 0.0270));
        assert!(!super::bends(20, 0.0300, 0.0220));
    }

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

    /// A hole in a face, near a corner but not over an edge, is none of the corner's business:
    /// the patch stays in the face and no vertex is dragged to the corner.
    #[test]
    fn a_hole_near_a_corner_stays_in_its_face() {
        let (m, vertices, centroids) = cut_and_fill(Point3::new(3.0, 3.0, SIDE), 2.0, 0.0);
        assert!(vertices < 1e-9 && centroids < 1e-9, "{vertices} {centroids}");
        let top: f64 = (0..m.triangle_count())
            .filter(|&t| m.face_centroid(t).is_some_and(|c| c.z > SIDE - 1e-6))
            .map(|t| m.face_area(t))
            .sum();
        assert!((top - SIDE * SIDE).abs() < 1e-6, "the top has area {top}: the patch overlaps it");
    }

    /// A Z-shaped step (top, wall, floor), 1 mm triangles, as an open sheet over 20 mm in y.
    fn step(height: f64, sigma: f64, seed: u64) -> Mesh {
        let profile = |s: f64| -> (f64, f64) {
            if s <= 10.0 {
                (s, height)
            } else if s <= 10.0 + height {
                (10.0, height - (s - 10.0))
            } else {
                (s - height, 0.0)
            }
        };
        let (ns, ny) = ((20.0 + height).round() as usize, 20usize);
        let mut m = Mesh::default();
        for i in 0..=ns {
            let (x, z) = profile(i as f64);
            for j in 0..=ny {
                m.positions.push(Point3::new(x, j as f64, z));
            }
        }
        let id = |i: usize, j: usize| (i * (ny + 1) + j) as u32;
        for i in 0..ns {
            for j in 0..ny {
                m.triangles.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                m.triangles.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        if sigma > 0.0 {
            synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
        }
        m
    }

    /// Distance from `p` to the step's profile (top, wall and floor as segments in x, z).
    fn to_step(p: &Point3, height: f64) -> f64 {
        let seg = |a: (f64, f64), b: (f64, f64)| {
            let (dx, dz) = (b.0 - a.0, b.1 - a.1);
            let t = (((p.x - a.0) * dx + (p.z - a.1) * dz) / (dx * dx + dz * dz)).clamp(0.0, 1.0);
            (a.0 + t * dx - p.x).hypot(a.1 + t * dz - p.z)
        };
        seg((0.0, height), (10.0, height)).min(seg((10.0, height), (10.0, 0.0))).min(seg((10.0, 0.0), (20.0, 0.0)))
    }

    /// The top and the floor of a step have one normal but are two faces: a hole over the wall
    /// must not be filled worse, or with slivers, because they were read as one tilted side.
    #[test]
    fn a_hole_over_a_step_is_not_made_worse() {
        for (height, r) in [(2.0, 3.0), (3.0, 5.0), (4.0, 4.0)] {
            for seed in 1..=10 {
                let m = step(height, 0.03, seed);
                let at = Point3::new(10.0, 10.0, height / 2.0);
                let keep: Vec<u32> = (0..m.triangle_count() as u32)
                    .filter(|&t| m.face_centroid(t as usize).is_some_and(|c| (c - at).norm() > r))
                    .collect();
                let cut = m.submesh(&keep);
                let worst = |mm: &Mesh| mm.positions.iter().map(|p| to_step(p, height)).fold(0.0, f64::max);
                let (mut faired, mut sharp) = (cut.clone(), cut.clone());
                // the sheet's own border is a longer loop than the hole and stays open
                fill_holes_with(&mut faired, 60, true, false);
                let rep = fill_holes_with(&mut sharp, 60, true, true);
                assert_eq!(rep.filled, 1);
                assert!(
                    worst(&sharp) <= worst(&faired) + 1e-9,
                    "step {height} hole {r} seed {seed}: {} mm, faired {} mm",
                    worst(&sharp),
                    worst(&faired)
                );
                let thinnest =
                    |mm: &Mesh| (0..mm.triangle_count()).map(|t| mm.face_area(t)).fold(f64::INFINITY, f64::min);
                assert!(thinnest(&sharp) >= 0.5 * thinnest(&faired).min(0.02), "a sliver of {} mm²", thinnest(&sharp));
            }
        }
    }

    /// A hundred holes of 0.8 to 2.5 mm over the edges of an irregular scan of a cube (0.7 to
    /// 1.4 mm triangles, σ = 0.03 mm): every patch stays within 0.12 mm of the cube, none is
    /// worse than the faired one, and nine in ten are sharpened.
    #[test]
    fn holes_over_the_edges_of_a_noisy_scan_keep_the_edges() {
        let mut cad = synth::cuboid(Point3::origin(), Vec3::new(SIDE, SIDE, SIDE), 1);
        cad.weld(1e-9);
        let d =
            synth::ScanDefects { spacing: 0.7, noise: 0.03, holes: 0, spikes: 0.0, debris: 0, ..Default::default() };
        let (mut bad, mut worse, mut n) = (0, 0, 0);
        let mut reasons = std::collections::BTreeMap::new();
        for seed in 1..=20u64 {
            let scan = synth::scanify(&cad, &d, &mut Rng::new(seed));
            let mut rng = Rng::new(seed + 1000);
            for _ in 0..5 {
                // a point on one of the edges of the top face, away from the corners
                let t = rng.range(4.0, SIDE - 4.0);
                let at = match rng.index(4) {
                    0 => Point3::new(t, 0.0, SIDE),
                    1 => Point3::new(t, SIDE, SIDE),
                    2 => Point3::new(0.0, t, SIDE),
                    _ => Point3::new(SIDE, t, SIDE),
                };
                let at = at + Vec3::new(rng.range(-0.8, 0.8), rng.range(-0.8, 0.8), 0.0);
                let r = rng.range(0.8, 2.5);
                let keep: Vec<u32> = (0..scan.triangle_count() as u32)
                    .filter(|&t| scan.face_centroid(t as usize).is_some_and(|c| (c - at).norm() > r))
                    .collect();
                let cut = scan.submesh(&keep);
                if boundary_loops(&cut).len() != 1 {
                    continue;
                }
                let first = cut.positions.len();
                let worst = |mm: &Mesh| mm.positions.iter().skip(first).map(to_cube).fold(0.0, f64::max);
                let (mut faired, mut sharp) = (cut.clone(), cut.clone());
                fill_holes_with(&mut faired, 1000, true, false);
                fill_holes_with(&mut sharp, 1000, true, true);
                n += 1;
                let why = super::LAST_REASON.with(std::cell::Cell::get).unwrap_or("sharpened");
                *reasons.entry(why).or_insert(0usize) += 1;
                if worst(&sharp) > 0.12 {
                    bad += 1;
                }
                if worst(&sharp) > worst(&faired) + 1e-9 {
                    worse += 1;
                }
            }
        }
        let sharpened = reasons.get("sharpened").copied().unwrap_or(0);
        assert!(n >= 90, "only {n} holes were cut");
        assert_eq!((bad, worse), (0, 0), "over 0.12 mm, worse than faired, of {n} holes: {reasons:?}");
        assert!(sharpened * 10 >= n * 9, "{sharpened} of {n} sharpened: {reasons:?}");
    }
}
