//! Conforming a layout to the outline of its open region (P8-H9). The lattice of the field gives
//! a layout that stops short of a ragged boundary (up to one edge length, and corners are
//! lost); a *ring* of faces between the layout's free boundary and the smoothed, enclosing
//! outline ([`crate::boundary`]) closes that gap.
//!
//! Every boundary loop of the layout is matched to the outline loop most of its vertices are
//! closest to, and the order is checked: the arc positions of the loop's vertices must run once
//! around the outline in the loop's direction (a mismatch leaves the loop as it is and is
//! reported). A boundary vertex within [`SNAP_FRACTION`] of an edge length of the outline (or
//! beyond it) moves onto it; every other one gets a partner vertex on the outline, and each
//! boundary edge becomes a ring face `b[i+1], b[i], u[i], corners…, u[i+1]` with the outline
//! corners between the two partners as extra vertices (so a corner makes a pentagon, not a
//! skewed quad). Closed layouts have no boundary and are not changed.
use super::PolyMesh;
use crate::boundary::{Outline, OutlineLoop};
use std::collections::{BTreeMap, BTreeSet};
use surfacecraft_geom::{Point3, Vec3};

/// A boundary vertex closer than this fraction of the edge length to the outline moves onto it.
const SNAP_FRACTION: f64 = 0.2;
/// An outline corner closer than this fraction of the edge length (along the outline) to a
/// partner vertex becomes that vertex.
const CORNER_FRACTION: f64 = 0.2;
/// Most a vertex beyond the outline may be pulled back, as a fraction of the edge length.
const PULL_BACK_FRACTION: f64 = 0.6;
/// The boundary vertex nearest (along the outline) to an outline corner becomes that corner when
/// it is within this fraction of the edge length of it.
const FORCE_CORNER_FRACTION: f64 = 0.75;
/// Share of a layout loop's vertices that must be closest to one outline loop.
const MATCH_SHARE: f64 = 0.7;

/// What [`conform_to_outline`] did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ConformReport {
    /// Free boundary loops of the layout.
    pub loops: usize,
    /// Loops conformed to an outline loop.
    pub conformed: usize,
    /// Loops left alone (no matching outline loop, or the order did not match).
    pub skipped: usize,
    /// Boundary vertices moved onto the outline.
    pub snapped: usize,
    /// Faces added.
    pub ring_faces: usize,
}

/// Free boundary loops of the faces, region on the left.
fn free_loops(faces: &[Vec<u32>]) -> Vec<Vec<u32>> {
    let mut directed: BTreeSet<(u32, u32)> = BTreeSet::new();
    for f in faces {
        for k in 0..f.len() {
            directed.insert((f[k], f[(k + 1) % f.len()]));
        }
    }
    let mut next: BTreeMap<u32, u32> = BTreeMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            next.insert(a, b);
        }
    }
    let mut loops = Vec::new();
    let mut used: BTreeSet<u32> = BTreeSet::new();
    for &start in next.keys() {
        if used.contains(&start) {
            continue;
        }
        let mut lp = vec![start];
        let mut cur = start;
        while let Some(&n) = next.get(&cur) {
            if n == start {
                loops.push(lp);
                break;
            }
            if used.contains(&n) || lp.len() > next.len() {
                break;
            }
            used.insert(n);
            lp.push(n);
            cur = n;
        }
        used.insert(start);
    }
    loops
}

/// The outline loop most of `points` is closest to.
fn matching_loop(outline: &Outline, points: &[Point3]) -> Option<usize> {
    let mut votes = vec![0usize; outline.loops.len()];
    for p in points {
        if let Some(h) = outline.closest(p)
            && let Some(v) = votes.get_mut(h.loop_index)
        {
            *v += 1;
        }
    }
    let (best, count) = votes.iter().copied().enumerate().max_by_key(|&(_, c)| c)?;
    (count as f64 >= MATCH_SHARE * points.len() as f64).then_some(best)
}

/// Arc positions of the loop's vertices along the outline, made non-decreasing around one
/// revolution, or `None` when they do not run once around in the loop's direction.
fn arc_positions(lp: &OutlineLoop, points: &[Point3], edge: f64) -> Option<Vec<f64>> {
    let (_, total) = lp.arc_lengths();
    let raw: Vec<f64> = points.iter().map(|p| lp.closest(p).map(|h| h.0)).collect::<Option<_>>()?;
    let n = raw.len();
    let mut unwrapped = vec![raw[0]];
    let mut sum = 0.0;
    for k in 0..n {
        let mut d = (raw[(k + 1) % n] - raw[k]).rem_euclid(total);
        if d > 0.5 * total {
            d -= total;
        }
        if d < -0.5 * edge {
            return None;
        }
        sum += d;
        if k + 1 < n {
            unwrapped.push(unwrapped[k] + d.max(0.0));
        }
    }
    ((sum - total).abs() < 0.35 * total).then_some(unwrapped)
}

/// Vertices and faces of one conformed loop.
struct Ring {
    faces: Vec<Vec<u32>>,
    moved: Vec<(u32, Point3)>,
    added: Vec<(Point3, Vec3)>,
}

/// Partner of a boundary vertex on the outline: the vertex itself when it was snapped, else a new one.
struct Partner {
    id: u32,
    arc: f64,
}

/// For every outline corner the loop vertex (index) that becomes it, with the corner's arc and point.
fn forced_corners(lp: &OutlineLoop, arcs: &[f64], edge: f64) -> BTreeMap<usize, (f64, Point3)> {
    let (arc_at, total) = lp.arc_lengths();
    let mut forced: BTreeMap<usize, (f64, Point3)> = BTreeMap::new();
    for &c in &lp.corners {
        let (Some(&ca), Some(&pt)) = (arc_at.get(c), lp.points.get(c)) else { continue };
        let gap = |a: f64| {
            let d = (a - ca).rem_euclid(total);
            d.min(total - d)
        };
        let best = arcs
            .iter()
            .enumerate()
            .filter(|(i, _)| !forced.contains_key(i))
            .min_by(|a, b| gap(*a.1).total_cmp(&gap(*b.1)));
        if let Some((i, &a)) = best
            && gap(a) <= FORCE_CORNER_FRACTION * edge
        {
            forced.insert(i, (ca, pt));
        }
    }
    forced
}

/// Snaps the boundary vertices that are close to (or beyond) the outline and creates a partner
/// vertex on the outline for every other one.
fn partners(
    poly: &PolyMesh,
    lp: &OutlineLoop,
    ids: &[u32],
    arcs: &[f64],
    edge: f64,
    normal_at: &dyn Fn(&Point3) -> Option<Vec3>,
) -> (Vec<Partner>, Ring) {
    let base = poly.vertices.len() as u32;
    let mut ring = Ring { faces: Vec::new(), moved: Vec::new(), added: Vec::new() };
    let mut out = Vec::with_capacity(ids.len());
    let forced = forced_corners(lp, arcs, edge);
    for (i, (&id, &arc)) in ids.iter().zip(arcs).enumerate() {
        if let Some(&(ca, pt)) = forced.get(&i) {
            ring.moved.push((id, pt));
            out.push(Partner { id, arc: ca });
            continue;
        }
        let (Some(p), Some((o, dir))) = (poly.vertices.get(id as usize), lp.at(arc)) else { continue };
        let d = (p - o).norm();
        let beyond = (p - o).dot(&dir) > 0.0;
        if d < SNAP_FRACTION * edge || (beyond && d < PULL_BACK_FRACTION * edge) {
            ring.moved.push((id, o));
            out.push(Partner { id, arc });
        } else {
            let n = normal_at(&o).or_else(|| poly.normals.get(id as usize).copied()).unwrap_or_else(Vec3::z);
            out.push(Partner { id: base + ring.added.len() as u32, arc });
            ring.added.push((o, n));
        }
    }
    (out, ring)
}

/// The ring face over boundary edge `i`: `b[i+1], b[i], u[i], corners…, u[i+1]`.
fn ring_face(
    lp: &OutlineLoop,
    corner_arcs: &[(f64, usize)],
    (b0, b1): (u32, u32),
    (p0, p1): (&Partner, &Partner),
    ctx: (f64, f64, u32),
    ring: &mut Ring,
    normal_at: &dyn Fn(&Point3) -> Option<Vec3>,
) -> Option<Vec<u32>> {
    let (total, edge, base) = ctx;
    let s0 = p0.arc.rem_euclid(total);
    let s1 = s0 + (p1.arc - p0.arc).rem_euclid(total);
    let mut cycle = vec![b1, b0, p0.id];
    for &(arc, c) in corner_arcs {
        let a = s0 + (arc - s0).rem_euclid(total);
        let inside = a > s0 + CORNER_FRACTION * edge && a < s1 - CORNER_FRACTION * edge;
        if let (true, Some(&pt), Some(n)) = (inside, lp.points.get(c), lp.outward.get(c)) {
            cycle.push(base + ring.added.len() as u32);
            ring.added.push((pt, normal_at(&pt).unwrap_or(*n)));
        }
    }
    cycle.push(p1.id);
    cycle.dedup();
    while cycle.len() > 1 && cycle.first() == cycle.last() {
        cycle.pop();
    }
    (cycle.len() >= 3).then_some(cycle)
}

fn build_ring(
    poly: &PolyMesh,
    lp: &OutlineLoop,
    ids: &[u32],
    arcs: &[f64],
    edge: f64,
    normal_at: &dyn Fn(&Point3) -> Option<Vec3>,
) -> Ring {
    let base = poly.vertices.len() as u32;
    let (partner, mut ring) = partners(poly, lp, ids, arcs, edge, normal_at);
    let (arc_at, total) = lp.arc_lengths();
    let corner_arcs: Vec<(f64, usize)> = lp.corners.iter().filter_map(|&c| arc_at.get(c).map(|&a| (a, c))).collect();
    let m = partner.len().min(ids.len());
    for i in 0..m {
        let j = (i + 1) % m;
        let face = ring_face(
            lp,
            &corner_arcs,
            (ids[i], ids[j]),
            (&partner[i], &partner[j]),
            (total, edge, base),
            &mut ring,
            normal_at,
        );
        ring.faces.extend(face);
    }
    ring
}

/// Moves the free-boundary vertices of `layout` that are within `max_move` of `outline` onto it
/// (after a split, whose new boundary vertices sit at chord midpoints).
#[must_use]
pub fn snap_to_outline(layout: &super::QuadMesh, outline: &Outline, max_move: f64) -> super::QuadMesh {
    let mut out = layout.clone();
    if outline.loops.is_empty() {
        return out;
    }
    for (a, b) in layout.boundary_edges() {
        for v in [a, b] {
            let Some(p) = layout.vertices.get(v as usize) else { continue };
            if let (Some(h), Some(slot)) = (outline.closest(p), out.vertices.get_mut(v as usize))
                && h.distance <= max_move
            {
                *slot = h.point;
            }
        }
    }
    out
}

/// Conforms the free boundary of `poly` to `outline` (see the module documentation). `edge` is
/// the edge length of the layout; `normal_at` gives the scan normal near a point.
pub fn conform_to_outline(
    poly: &mut PolyMesh,
    outline: &Outline,
    edge: f64,
    normal_at: &dyn Fn(&Point3) -> Option<Vec3>,
) -> ConformReport {
    let loops = free_loops(&poly.faces);
    let mut report = ConformReport { loops: loops.len(), ..ConformReport::default() };
    for ids in &loops {
        let points: Vec<Point3> = ids.iter().filter_map(|&v| poly.vertices.get(v as usize).copied()).collect();
        let found = matching_loop(outline, &points).and_then(|k| outline.loops.get(k));
        let arcs = found.and_then(|lp| arc_positions(lp, &points, edge));
        let (Some(lp), Some(arcs)) = (found, arcs) else {
            report.skipped += 1;
            continue;
        };
        let ring = build_ring(poly, lp, ids, &arcs, edge, normal_at);
        report.conformed += 1;
        report.snapped += ring.moved.len();
        report.ring_faces += ring.faces.len();
        for (id, p) in ring.moved {
            if let Some(v) = poly.vertices.get_mut(id as usize) {
                *v = p;
            }
        }
        for (p, n) in ring.added {
            poly.vertices.push(p);
            poly.normals.push(n);
        }
        poly.faces.extend(ring.faces);
    }
    report
}
