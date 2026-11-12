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
const SNAP_FRACTION: f64 = 0.3;
/// An outline corner closer than this fraction of the edge length (along the outline) to a
/// partner vertex becomes that vertex.
const CORNER_FRACTION: f64 = 0.2;
/// Most a vertex beyond the outline may be pulled back, as a fraction of the edge length.
const PULL_BACK_FRACTION: f64 = 0.6;
/// The boundary vertex nearest (along the outline) to an outline corner becomes that corner when
/// it is within this fraction of the edge length of it.
const FORCE_CORNER_FRACTION: f64 = 0.75;
/// Two partners closer than this fraction of the edge length are one point.
const MIN_PARTNER_GAP_FRACTION: f64 = 0.05;
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
    (edge, slack): (f64, f64),
    normal_at: &dyn Fn(&Point3) -> Option<Vec3>,
) -> (Vec<Partner>, Ring) {
    let base = poly.vertices.len() as u32;
    let mut ring = Ring { faces: Vec::new(), moved: Vec::new(), added: Vec::new() };
    let mut out = Vec::with_capacity(ids.len());
    let forced = forced_corners(lp, arcs, edge);
    // a partner never lands within `min_gap` of the previous one: it slides along the outline
    let min_gap = MIN_PARTNER_GAP_FRACTION * edge;
    for (i, (&id, &arc)) in ids.iter().zip(arcs).enumerate() {
        if let Some(&(ca, pt)) = forced.get(&i) {
            ring.moved.push((id, pt));
            out.push(Partner { id, arc: ca });
            continue;
        }
        let Some(p) = poly.vertices.get(id as usize) else { continue };
        let mut arc = arc;
        let mut at = lp.at(arc);
        if let (Some(prev), Some(prev_pt)) = (out.last(), out.last().and_then(|q| lp.at(q.arc)))
            && at.is_some_and(|(o, _)| (o - prev_pt.0).norm() < min_gap)
        {
            arc = prev.arc + min_gap;
            at = lp.at(arc);
        }
        let Some((o, dir)) = at else { continue };
        let d = (p - o).norm();
        let beyond = (p - o).dot(&dir) > 0.0;
        if d < SNAP_FRACTION * edge + slack || (beyond && d < PULL_BACK_FRACTION * edge + slack) {
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
    // corners strictly inside the interval, in unwrapped arc order (the stored order is not)
    let mut inside: Vec<(f64, usize)> = corner_arcs
        .iter()
        .map(|&(arc, c)| (s0 + (arc - s0).rem_euclid(total), c))
        .filter(|&(a, _)| a > s0 + CORNER_FRACTION * edge && a < s1 - CORNER_FRACTION * edge)
        .collect();
    inside.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, c) in inside {
        if let (Some(&pt), Some(n)) = (lp.points.get(c), lp.outward.get(c)) {
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
    (edge, slack): (f64, f64),
    normal_at: &dyn Fn(&Point3) -> Option<Vec3>,
) -> Ring {
    let base = poly.vertices.len() as u32;
    let (partner, mut ring) = partners(poly, lp, ids, arcs, (edge, slack), normal_at);
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

/// Does the face have no zero-length edge and a positive area along its vertex normals?
fn face_is_proper(poly: &PolyMesh, f: &[u32]) -> bool {
    let pts: Vec<(Point3, Vec3)> = f
        .iter()
        .filter_map(|&v| poly.vertices.get(v as usize).copied().zip(poly.normals.get(v as usize).copied()))
        .collect();
    if pts.len() != f.len() || pts.len() < 3 {
        return false;
    }
    let n = pts.len();
    if (0..n).any(|k| (pts[(k + 1) % n].0 - pts[k].0).norm() < 1e-9) {
        return false;
    }
    let normal: Vec3 = pts.iter().map(|p| p.1).sum();
    let p0 = pts[0].0;
    let area: f64 = (1..n - 1).map(|k| (pts[k].0 - p0).cross(&(pts[k + 1].0 - p0)).dot(&normal)).sum();
    area > 1e-12
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
        let ring = build_ring(poly, lp, ids, &arcs, (edge, outline.slack), normal_at);
        report.conformed += 1;
        report.snapped += ring.moved.len();
        for (id, p) in ring.moved {
            if let Some(v) = poly.vertices.get_mut(id as usize) {
                *v = p;
            }
        }
        for (p, n) in ring.added {
            poly.vertices.push(p);
            poly.normals.push(n);
        }
        let valid: Vec<Vec<u32>> = ring.faces.into_iter().filter(|f| face_is_proper(poly, f)).collect();
        report.ring_faces += valid.len();
        poly.faces.extend(valid);
    }
    report
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The square [0, 10]², region on the left of the travel direction, four corners.
    fn square() -> Outline {
        let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let outward = [(-h, -h), (h, -h), (h, h), (-h, h)];
        let lp = OutlineLoop {
            points: pts.iter().map(|p| Point3::new(p.0, p.1, 0.0)).collect(),
            corners: vec![0, 1, 2, 3],
            outward: outward.iter().map(|o| Vec3::new(o.0, o.1, 0.0)).collect(),
        };
        Outline { loops: vec![lp], slack: 0.0 }
    }

    fn poly(points: &[(f64, f64)]) -> PolyMesh {
        PolyMesh {
            vertices: points.iter().map(|p| Point3::new(p.0, p.1, 0.0)).collect(),
            normals: vec![Vec3::z(); points.len()],
            faces: vec![(0..points.len() as u32).collect()],
        }
    }

    fn normal_at(_: &Point3) -> Option<Vec3> {
        Some(Vec3::z())
    }

    fn segments_cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
        let o = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
        o(a, b, c) * o(a, b, d) < -1e-12 && o(c, d, a) * o(c, d, b) < -1e-12
    }

    /// Simple polygon with positive (counter-clockwise) area.
    fn assert_simple(poly: &PolyMesh, f: &[u32]) {
        let p: Vec<(f64, f64)> =
            f.iter().map(|&v| (poly.vertices[v as usize].x, poly.vertices[v as usize].y)).collect();
        let n = p.len();
        let area: f64 = (0..n).map(|k| p[k].0 * p[(k + 1) % n].1 - p[(k + 1) % n].0 * p[k].1).sum::<f64>() / 2.0;
        assert!(area > 1e-9, "area {area} of {p:?}");
        for i in 0..n {
            assert!((p[i].0 - p[(i + 1) % n].0).hypot(p[i].1 - p[(i + 1) % n].1) > 1e-9, "zero edge in {p:?}");
            for j in i + 2..n {
                if (j + 1) % n != i {
                    assert!(!segments_cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n]), "self-intersection {p:?}");
                }
            }
        }
    }

    /// Two neighbours that project onto the same outline point are not both snapped onto it.
    #[test]
    fn neighbours_projecting_to_one_point_leave_no_zero_length_edge() {
        let mut m = poly(&[
            (-0.1, 0.0),
            (0.0, -0.1),
            (10.0, -0.1),
            (10.1, 0.0),
            (10.1, 10.0),
            (10.0, 10.1),
            (0.0, 10.1),
            (-0.1, 10.0),
        ]);
        conform_to_outline(&mut m, &square(), 4.0, &normal_at);
        for f in &m.faces {
            assert_simple(&m, f);
        }
    }

    /// An interval that wraps past the start of the loop gets its corners in arc order.
    #[test]
    fn wrapped_ring_intervals_insert_corners_in_arc_order() {
        // perimeter 40; vertices project to arcs 25, 5(+40), 8 and 18(+40)
        let mut m = poly(&[(5.0, 8.0), (5.0, 2.0), (8.0, 2.0), (8.0, 8.0)]);
        let report = conform_to_outline(&mut m, &square(), 2.0, &normal_at);
        assert_eq!(report.conformed, 1, "{report:?}");
        assert!(m.faces.len() > 1);
        for f in &m.faces {
            assert_simple(&m, f);
        }
    }
}
