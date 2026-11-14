//! The boundary of each group of patches put on the group's primitive, before the group is merged
//! (P9-O6b).
//!
//! The patches of a wrap lie on a primitive only within the fit, so the edges a group shares with the
//! patches round it leave the primitive by up to the tolerance. A merged face whose boundary does that
//! is a face a receiving CAD system has to heal. Here the shared boundary is moved onto the primitive
//! first, and both sides of every moved edge move with it, so the body stays watertight:
//!
//! 1. Each vertex of a boundary goes onto the primitives of the groups that meet there (alternating
//!    closest-point projections, which end on the intersection of two primitives at a sharp edge).
//! 2. Each boundary edge, a Bézier curve, is re-fitted: its samples are put on the primitive(s) and the
//!    interior control points are the least-squares fit through them, the end points being the moved
//!    vertices. On a plane this is exact.
//! 3. Every other edge that ends at a moved vertex follows it (its end control point moves).
//! 4. The patches that use a changed edge take the new control points along the matching border of
//!    their net. A patch whose border does not match its edge makes the whole step refuse, and the body
//!    stays as it was.
//!
//! The projection is the kernel's (`SurfaceEval`, in [`Gauge`]); the least-squares fit of a Bézier curve and
//! the edit of the borders of the patches' control nets are made here.
//!
//! APP-GEOMETRY: stands in for a kernel edit of a spline network that moves a shared edge onto a surface
//! (re-fitting the edge's curve and both neighbouring patches consistently); solidcore has no such entry.
use super::boundary::{Boundary, boundary, edge_points};
use super::place::Gauge;
use crate::hybrid::{V, add, norm, scale, sub};
use crate::record::brep::{Brep, Curve, Loop, Surface};
use crate::{KernelError, Result, finite_all};
use solidcore::math::Vec3;
use solidcore::nurbs::crv_fit::p1::ParamMethod;
use solidcore::nurbs::crv_fit::p2::approx_lst_sq;
use std::collections::{BTreeMap, BTreeSet};

/// Alternating projections that put a point on every primitive of its edge or vertex.
const MAX_ALTERNATIONS: usize = 2000;
/// A point lies on a primitive when it is this close (mm).
const ON_SURFACE: f64 = 1e-4;
/// Control points equal within this (mm) are the same point when a patch's border is matched to its edge.
const SAME_POINT: f64 = 1e-7;

/// A group of patches and the evaluator of the primitive they lie on.
pub(super) struct Target<'a> {
    pub faces: &'a [usize],
    pub gauge: &'a Gauge,
}

fn refuse(what: &str) -> KernelError {
    KernelError::Argument(what.to_string())
}

/// `p` on every one of `gauges`: by alternating projections, or, where they do not meet within
/// [`MAX_ALTERNATIONS`], the nearer of the last iterate and the point on the first alone.
fn onto_all(gauges: &[&Gauge], p: V) -> Option<V> {
    let first = gauges.first()?;
    let on_all = |q: V| gauges.iter().all(|g| g.distance(q).is_some_and(|d| d <= ON_SURFACE));
    let mut q = p;
    for _ in 0..MAX_ALTERNATIONS {
        for g in gauges {
            q = g.closest(q)?;
        }
        if on_all(q) {
            return Some(q);
        }
    }
    if gauges.len() == 1 {
        return Some(q);
    }
    // surfaces that meet at a shallow angle converge slowly: the last iterate is taken when it is nearer all of
    // them than the point on the first alone
    let worst = |q: V| gauges.iter().filter_map(|g| g.distance(q)).fold(0.0_f64, f64::max);
    let alone = first.closest(p)?;
    Some(if worst(q) < worst(alone) { q } else { alone })
}

/// The largest distance of the edge samples of a boundary from the primitive (mm).
///
/// # Errors
/// An edge that cannot be evaluated, or a point the primitive has no closest point to.
pub(super) fn boundary_gap(body: &Brep, b: &Boundary, gauge: &Gauge) -> Result<f64> {
    let mut gap = 0.0_f64;
    for le in b.loops.iter().flatten() {
        for p in edge_points(body, le.edge)? {
            let d = gauge.distance(p).ok_or_else(|| refuse("the shape cannot be merged"))?;
            gap = gap.max(d);
        }
    }
    Ok(gap)
}

/// The groups that touch each boundary edge and each end vertex of one: the gauges of their primitives.
struct Touch<'a> {
    edges: BTreeMap<u32, Vec<&'a Gauge>>,
    vertices: BTreeMap<u32, Vec<&'a Gauge>>,
}

fn touching<'a>(body: &Brep, targets: &[Target<'a>]) -> Touch<'a> {
    let mut t = Touch { edges: BTreeMap::new(), vertices: BTreeMap::new() };
    for target in targets {
        let Ok(b) = boundary(body, target.faces) else { continue };
        for le in b.loops.iter().flatten() {
            t.edges.entry(le.edge).or_default().push(target.gauge);
            for v in body.edges.get(le.edge as usize).map(|e| e.vertices).unwrap_or_default() {
                t.vertices.entry(v).or_default().push(target.gauge);
            }
        }
    }
    t
}

/// The control points of an edge that is one Bézier span over its interval.
fn bezier_of(body: &Brep, edge: u32) -> Option<&Vec<V>> {
    let e = body.edges.get(edge as usize)?;
    let Curve::Spline { knots, points } = &e.curve else { return None };
    let n = points.len().checked_sub(1).filter(|&n| n >= 1)?;
    let (lo, hi) = (*knots.first()?, *knots.last()?);
    let clamped = knots.len() == 2 * (n + 1)
        && knots.iter().take(n + 1).all(|k| *k == lo)
        && knots.iter().skip(n + 1).all(|k| *k == hi);
    (clamped && e.nurbs.is_none() && e.interval == [lo, hi]).then_some(points)
}

/// The Bézier control points (degree `n`) of the curve the kernel fits to `targets` (equally spaced in the
/// parameter) by least squares, through the first and the last of them (`nurbs::crv_fit::p2::approx_lst_sq`,
/// Piegl and Tiller A9.7). `None` for a line (the fit needs three control points) or where the kernel
/// refuses.
fn least_squares(n: usize, targets: &[V]) -> Option<Vec<V>> {
    let points: Vec<Vec3> = targets.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
    let curve = approx_lst_sq(&points, n, n, ParamMethod::Uniform).ok()?;
    let single_span = curve.knots().len() == 2 * (n + 1);
    let out: Vec<V> = curve.cpts().iter().map(|c| [c.x, c.y, c.z]).collect();
    (single_span && out.len() == n + 1 && out.iter().all(|p| finite_all(p))).then_some(out)
}

/// `old` with the end points replaced by `ends` and the interior points moved along by the blend of
/// the two end displacements (the fallback of a singular fit, and the edges that only follow a vertex).
fn follow(old: &[V], ends: [V; 2]) -> Vec<V> {
    let n = old.len().saturating_sub(1).max(1);
    let (first, last) = (old.first().copied().unwrap_or(ends[0]), old.last().copied().unwrap_or(ends[1]));
    let (d0, d1) = (sub(ends[0], first), sub(ends[1], last));
    old.iter()
        .enumerate()
        .map(|(j, p)| {
            let s = j as f64 / n as f64;
            add(*p, add(scale(d0, 1.0 - s), scale(d1, s)))
        })
        .collect()
}

/// What changes: the vertices' new points and the edges' new control points (with the old ones, to
/// find the borders of the patches).
#[derive(Default)]
struct Moves {
    vertices: BTreeMap<u32, V>,
    edges: BTreeMap<u32, (Vec<V>, Vec<V>)>,
}

fn moved_vertices(body: &Brep, touch: &Touch) -> BTreeMap<u32, V> {
    let mut out = BTreeMap::new();
    for (&v, gauges) in &touch.vertices {
        let Some(p) = body.vertices.get(v as usize).map(|x| x.point) else { continue };
        if let Some(q) = onto_all(gauges, p) {
            out.insert(v, q);
        }
    }
    out
}

/// The ends of edge `e` after the vertices moved.
fn new_ends(body: &Brep, e: u32, vertices: &BTreeMap<u32, V>) -> Option<[V; 2]> {
    let edge = body.edges.get(e as usize)?;
    let end = |slot: usize| {
        let v = edge.vertices.get(slot).copied()?;
        vertices.get(&v).copied().or_else(|| body.vertices.get(v as usize).map(|x| x.point))
    };
    Some([end(0)?, end(1)?])
}

fn refit_edge(body: &Brep, e: u32, gauges: &[&Gauge], ends: [V; 2], old: &[V]) -> Result<Vec<V>> {
    let samples = edge_points(body, e)?;
    let targets: Vec<V> = samples.iter().map(|p| onto_all(gauges, *p).unwrap_or(*p)).collect();
    let mut fit = least_squares(old.len().saturating_sub(1), &targets).unwrap_or_else(|| follow(old, ends));
    // the ends are the vertices as they moved
    if let Some(first) = fit.first_mut() {
        *first = ends[0];
    }
    if let Some(last) = fit.last_mut() {
        *last = ends[1];
    }
    Ok(fit)
}

fn plan(body: &Brep, touch: &Touch) -> Result<Moves> {
    let vertices = moved_vertices(body, touch);
    let mut edges: BTreeMap<u32, (Vec<V>, Vec<V>)> = BTreeMap::new();
    for (&e, gauges) in &touch.edges {
        let (Some(old), Some(ends)) = (bezier_of(body, e), new_ends(body, e, &vertices)) else {
            return Err(refuse("a boundary edge is not a Bézier curve"));
        };
        edges.insert(e, (old.clone(), refit_edge(body, e, gauges, ends, old)?));
    }
    for (i, _) in body.edges.iter().enumerate() {
        let e = i as u32;
        let touches_moved = body.edges.get(i).is_some_and(|x| x.vertices.iter().any(|v| vertices.contains_key(v)));
        if edges.contains_key(&e) || !touches_moved {
            continue;
        }
        if let (Some(old), Some(ends)) = (bezier_of(body, e), new_ends(body, e, &vertices)) {
            edges.insert(e, (old.clone(), follow(old, ends)));
        } else {
            return Err(refuse("an edge at a moved vertex is not a Bézier curve"));
        }
    }
    Ok(Moves { vertices, edges })
}

fn same(a: V, b: V) -> bool {
    norm(sub(a, b)) <= SAME_POINT * (1.0 + norm(a))
}

/// Does `border` run along `edge` (either way round)? `Some(true)` when it runs the same way.
fn runs_along(border: &[V], edge: &[V]) -> Option<bool> {
    if border.len() != edge.len() {
        return None;
    }
    if border.iter().zip(edge).all(|(a, b)| same(*a, *b)) {
        return Some(true);
    }
    border.iter().zip(edge.iter().rev()).all(|(a, b)| same(*a, *b)).then_some(false)
}

/// The four borders of a net as `(points, positions in the net)`, rows `[i][..]` and columns `[..][j]`.
fn borders(net: &[Vec<V>]) -> Vec<Vec<(usize, usize)>> {
    let (nu, nv) = (net.len(), net.first().map_or(0, Vec::len));
    let (iu, iv) = (nu.saturating_sub(1), nv.saturating_sub(1));
    vec![
        (0..nv).map(|j| (0, j)).collect(),
        (0..nv).map(|j| (iu, j)).collect(),
        (0..nu).map(|i| (i, 0)).collect(),
        (0..nu).map(|i| (i, iv)).collect(),
    ]
}

/// The net of a face with the borders that run along its changed edges replaced. `None` when an edge
/// of the face is changed and no border of the net runs along it.
fn moved_net(net: &[Vec<V>], used: &[&(Vec<V>, Vec<V>)]) -> Option<Vec<Vec<V>>> {
    let at = |net: &[Vec<V>], (i, j): (usize, usize)| net.get(i).and_then(|r| r.get(j)).copied();
    let mut out = net.to_vec();
    for (old, new) in used {
        let mut found = false;
        for border in borders(net) {
            let points: Option<Vec<V>> = border.iter().map(|&c| at(net, c)).collect();
            let Some(same_way) = points.and_then(|p| runs_along(&p, old)) else { continue };
            found = true;
            let order: Vec<&V> = if same_way { new.iter().collect() } else { new.iter().rev().collect() };
            for (&(i, j), p) in border.iter().zip(order) {
                if let Some(slot) = out.get_mut(i).and_then(|r| r.get_mut(j)) {
                    *slot = *p;
                }
            }
        }
        if !found {
            return None;
        }
    }
    Some(out)
}

/// The body with the moves applied to its vertices, edges and patches.
fn applied(body: &Brep, moves: &Moves) -> Result<Brep> {
    let mut out = body.clone();
    for (&v, p) in &moves.vertices {
        if let Some(slot) = out.vertices.get_mut(v as usize) {
            slot.point = *p;
        }
    }
    for (&e, (_, new)) in &moves.edges {
        if let Some(Curve::Spline { points, .. }) = out.edges.get_mut(e as usize).map(|x| &mut x.curve) {
            *points = new.clone();
        }
    }
    for (fi, face) in body.faces.iter().enumerate() {
        let used: BTreeSet<u32> = face
            .loops
            .iter()
            .flat_map(|l| match l {
                Loop::Edges { edges } => edges.iter().map(|u| u.edge).collect::<Vec<_>>(),
                Loop::Vertex { .. } => Vec::new(),
            })
            .filter(|e| moves.edges.contains_key(e))
            .collect();
        if used.is_empty() {
            continue;
        }
        let Surface::Spline { points, .. } = &face.surface else {
            return Err(refuse("a patch at a moved edge is not a spline"));
        };
        let changes: Vec<&(Vec<V>, Vec<V>)> = used.iter().filter_map(|e| moves.edges.get(e)).collect();
        let net = moved_net(points, &changes).ok_or_else(|| refuse("a patch's border does not match its edge"))?;
        if let Some(Surface::Spline { points, .. }) = out.faces.get_mut(fi).map(|f| &mut f.surface) {
            *points = net;
        }
    }
    Ok(out)
}

/// The body with the boundary of every group of `targets` on its primitive, and the patches and edges
/// that share it moved with it; the body as it was where that cannot be done (an error says why).
///
/// # Errors
/// An edge or patch at a boundary that is not a Bézier span, or a patch whose border does not match the
/// edge it is used with.
pub(super) fn snap_boundaries(body: &Brep, targets: &[Target]) -> Result<Brep> {
    let touch = touching(body, targets);
    if touch.edges.is_empty() {
        return Ok(body.clone());
    }
    applied(body, &plan(body, &touch)?)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> V {
        [x, y, z]
    }

    #[test]
    fn a_fit_through_points_on_a_cubic_gives_that_cubic() {
        let cubic = [p(0.0, 0.0, 0.0), p(1.0, 2.0, 0.5), p(3.0, 2.0, -0.5), p(4.0, 0.0, 0.0)];
        let samples: Vec<V> = (0..9)
            .map(|i| {
                let t = f64::from(i) / 8.0;
                let w = [(1.0 - t).powi(3), 3.0 * t * (1.0 - t).powi(2), 3.0 * t * t * (1.0 - t), t.powi(3)];
                (0..3).map(|d| cubic.iter().zip(&w).map(|(q, b)| q[d] * b).sum::<f64>()).collect::<Vec<_>>()
            })
            .map(|v| [v[0], v[1], v[2]])
            .collect();
        let fit = least_squares(3, &samples).unwrap();
        for (a, b) in fit.iter().zip(&cubic) {
            assert!(norm(sub(*a, *b)) < 1e-9, "{a:?} {b:?}");
        }
    }

    #[test]
    fn a_line_has_no_interior_unknowns_and_the_fallback_keeps_its_ends() {
        assert!(least_squares(1, &[p(0.0, 0.0, 0.0), p(0.5, 0.0, 0.0), p(1.0, 0.0, 0.0)]).is_none());
        let moved =
            follow(&[p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(2.0, 0.0, 0.0)], [p(0.0, 1.0, 0.0), p(2.0, 3.0, 0.0)]);
        assert_eq!(moved.first(), Some(&p(0.0, 1.0, 0.0)));
        assert_eq!(moved.last(), Some(&p(2.0, 3.0, 0.0)));
        assert!((moved[1][1] - 2.0).abs() < 1e-12, "the middle moves by the mean of the end displacements");
    }

    #[test]
    fn a_border_is_matched_to_its_edge_either_way_round_and_a_stranger_is_not() {
        let net = vec![vec![p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0)], vec![p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0)]];
        let old = vec![p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0)];
        let new = vec![p(0.0, 0.0, 0.1), p(0.0, 1.0, 0.1)];
        let moved = moved_net(&net, &[&(old.clone(), new.clone())]).unwrap();
        assert_eq!((moved[0][0][2], moved[0][1][2], moved[1][0][2]), (0.1, 0.1, 0.0));
        let backwards = vec![p(0.0, 1.0, 0.0), p(0.0, 0.0, 0.0)];
        assert!(moved_net(&net, &[&(backwards, new.iter().rev().copied().collect())]).is_some());
        assert!(moved_net(&net, &[&(vec![p(5.0, 5.0, 5.0), p(6.0, 5.0, 5.0)], new)]).is_none());
    }
}
