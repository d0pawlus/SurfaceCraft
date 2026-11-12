//! Conforming a layout to the outline of its open region (P8-H9). The lattice of the field gives
//! a layout that stops short of a ragged boundary (up to one edge length, and corners are
//! lost); a *ring* of faces between the layout's free boundary and the smoothed, enclosing
//! outline ([`crate::boundary`]) closes that gap.
//!
//! Every boundary loop of the layout is matched to the outline loop most of its vertices are
//! closest to, and the order is checked: the arc positions of the loop's vertices must run once
//! around the outline in the loop's direction (a mismatch leaves the loop as it is and is
//! reported). The loop is conformed in one transaction: a single solve gives every boundary
//! vertex its arc (forced corners are anchors; between anchors the arcs keep the cyclic order and
//! a minimum gap, as close to the projections as possible), the snapped vertices and the ring are
//! built on a copy, and every face the change touches is validated; any failure rolls the loop
//! back to the layout as it was and says why. A boundary vertex within [`SNAP_FRACTION`] of an edge length of the outline (or
//! beyond it) moves onto it; every other one gets a partner vertex on the outline, and each
//! boundary edge becomes a ring face `b[i+1], b[i], u[i], corners…, u[i+1]` with the outline
//! corners between the two partners as extra vertices (so a corner makes a pentagon, not a
//! skewed quad). Closed layouts have no boundary and are not changed.
use super::PolyMesh;
use crate::boundary::{Outline, OutlineLoop};
use std::collections::{BTreeMap, BTreeSet};
use surfacecraft_geom::any_perpendicular;
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
/// Neighbouring boundary vertices keep at least this fraction of the edge length of arc between them.
const MIN_PARTNER_GAP_FRACTION: f64 = 0.05;
/// A boundary vertex farther than this fraction of the edge length (plus the slack) from the
/// outline belongs to a dent of the layout and gets no partner.
const DEEP_FRACTION: f64 = 1.0;
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
    /// Loops with nothing to snap (no vertex within reach of the outline): left as they were.
    pub unchanged: usize,
    /// Loops rolled back to the layout as it was (a span too short for its vertices, or an
    /// invalid face after snapping).
    pub rolled_back: usize,
    /// Why the first unconformed loop was left as it is.
    pub reason: Option<&'static str>,
    /// Boundary vertices moved onto the outline.
    pub snapped: usize,
    /// Faces added.
    pub ring_faces: usize,
}

impl ConformReport {
    /// Is every free boundary loop of the layout conformed (at least one vertex on the outline,
    /// everything valid)? False for a layout without a free boundary.
    #[must_use]
    pub fn all_conformed(&self) -> bool {
        self.loops > 0 && self.conformed == self.loops
    }
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
    if n == 0 || !(total.is_finite() && total > 0.0) || raw.iter().any(|a| !a.is_finite()) {
        return None;
    }
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

/// Gives the scan normal near a point on the side `hint` points to.
pub type SideNormal<'a> = &'a dyn Fn(&Point3, &Vec3) -> Option<Vec3>;

/// Everything one loop's conforming changes, built on the side.
#[derive(Default)]
struct Plan {
    moved: Vec<(u32, Point3)>,
    added: Vec<(Point3, Vec3)>,
    faces: Vec<Vec<u32>>,
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
            // the corner's arc on the same revolution as the vertex
            forced.insert(i, (a + (ca - a + 0.5 * total).rem_euclid(total) - 0.5 * total, pt));
        }
    }
    forced
}

/// The arcs of every vertex of a span between the anchors `a` and `b` (exclusive), kept `gap`
/// apart from each other and from both anchors, as close to `targets` as that allows. `None` when
/// the bounds of a vertex do not leave room (after rounding), so the span is not conformed.
fn spread_span(
    targets: &[f64],
    (a, xa): (usize, f64),
    (xb, count): (f64, usize),
    gap: f64,
    total: f64,
) -> Option<Vec<f64>> {
    let n = targets.len();
    let mut out: Vec<f64> = Vec::with_capacity(count);
    for k in 1..=count {
        let j = (a + k) % n;
        // targets are unwrapped from vertex 0: the vertices before the anchor are one revolution on
        let t = targets.get(j).copied()? + if j < a { total } else { 0.0 };
        let (lo, hi) = (xa + k as f64 * gap, xb - (count + 1 - k) as f64 * gap);
        if !(lo <= hi) || !t.is_finite() {
            return None;
        }
        let x = t.max(lo).min(hi);
        out.push(out.last().map_or(x, |&prev| x.max(prev + gap)));
    }
    Some(out)
}

/// One solve for all arcs of the loop: forced corners are anchors, the vertices between two
/// anchors get arcs that keep their order and `gap` between neighbours (also to the anchors and
/// across the closing pair). `None` when a span is too short for its vertices.
fn solve_arcs(
    lp: &OutlineLoop,
    targets: &[f64],
    forced: &BTreeMap<usize, (f64, Point3)>,
    gap: f64,
) -> Option<Vec<f64>> {
    let (_, total) = lp.arc_lengths();
    let n = targets.len();
    let mut anchors: Vec<(usize, f64)> = forced.iter().map(|(&i, &(a, _))| (i, a)).collect();
    if anchors.is_empty() {
        anchors.push((0, *targets.first()?));
    }
    let mut arcs = vec![0.0; n];
    for (k, &(a, xa)) in anchors.iter().enumerate() {
        let (b, xb_raw) = anchors[(k + 1) % anchors.len()];
        let xb = xb_raw + total * (((xa - xb_raw) / total).floor() + 1.0).max(0.0);
        let count = if anchors.len() == 1 { n - 1 } else { (b + n - a - 1) % n };
        arcs[a] = xa;
        for (j, x) in spread_span(targets, (a, xa), (xb, count), gap, total)?.into_iter().enumerate() {
            arcs[(a + 1 + j) % n] = x;
        }
    }
    Some(arcs)
}

/// The plan for one loop: snaps close (or overhanging) vertices onto the outline, gives the other
/// near vertices a partner on it, and builds the ring faces. `near` are the indices of the loop
/// vertices within reach of the outline (`arcs` and `forced` are indexed like `near`); the deep
/// vertices between two near ones (a dent of the layout) end up in one ring face with them.
fn plan_loop(
    poly: &PolyMesh,
    lp: &OutlineLoop,
    (ids, near): (&[u32], &[usize]),
    (arcs, forced): (&[f64], &BTreeMap<usize, (f64, Point3)>),
    (edge, slack): (f64, f64),
    side_normal: SideNormal,
) -> Plan {
    let base = poly.vertices.len() as u32;
    let mut plan = Plan::default();
    let mut partners: Vec<Partner> = Vec::with_capacity(near.len());
    for (k, (&i, &arc)) in near.iter().zip(arcs).enumerate() {
        let id = ids[i];
        if let Some(&(ca, pt)) = forced.get(&k) {
            plan.moved.push((id, pt));
            partners.push(Partner { id, arc: ca });
            continue;
        }
        let (Some(p), Some((o, dir))) = (poly.vertices.get(id as usize), lp.at(arc)) else { continue };
        let (d, beyond) = ((p - o).norm(), (p - o).dot(&dir) > 0.0);
        if d < SNAP_FRACTION * edge + slack || (beyond && d < PULL_BACK_FRACTION * edge + slack) {
            plan.moved.push((id, o));
            partners.push(Partner { id, arc });
        } else {
            let hint = poly.normals.get(id as usize).copied().unwrap_or_else(Vec3::z);
            partners.push(Partner { id: base + plan.added.len() as u32, arc });
            plan.added.push((o, side_normal(&o, &hint).unwrap_or(hint)));
        }
    }
    let (arc_at, total) = lp.arc_lengths();
    let corner_arcs: Vec<(f64, usize)> = lp.corners.iter().filter_map(|&c| arc_at.get(c).map(|&a| (a, c))).collect();
    let n = ids.len();
    for t in 0..partners.len().min(near.len()) {
        let (p, u) = (near[t], (t + 1) % near.len());
        let q = near[u];
        // b[q], the deep vertices back to b[p]
        let mut chain = vec![ids[q]];
        let mut k = (q + n - 1) % n;
        while k != p {
            chain.push(ids[k]);
            k = (k + n - 1) % n;
        }
        chain.push(ids[p]);
        let hint = poly.normals.get(ids[p] as usize).copied().unwrap_or_else(Vec3::z);
        let ctx = RingCtx { lp, corner_arcs: &corner_arcs, total, edge, base, hint };
        let face = ring_face(&ctx, &chain, (&partners[t], &partners[u]), &mut plan, side_normal);
        plan.faces.extend(face);
    }
    plan
}

struct RingCtx<'a> {
    lp: &'a OutlineLoop,
    corner_arcs: &'a [(f64, usize)],
    total: f64,
    edge: f64,
    base: u32,
    hint: Vec3,
}

/// The ring face over a run of boundary vertices: `chain` (the far end back to the near end), then
/// `u0, corners…, u1` on the outline.
fn ring_face(
    ctx: &RingCtx,
    chain: &[u32],
    (p0, p1): (&Partner, &Partner),
    plan: &mut Plan,
    side_normal: SideNormal,
) -> Option<Vec<u32>> {
    let total = ctx.total;
    let s0 = p0.arc.rem_euclid(total);
    let s1 = s0 + (p1.arc - p0.arc).rem_euclid(total);
    let mut cycle = chain.to_vec();
    cycle.push(p0.id);
    // corners strictly inside the interval, in unwrapped arc order (the stored order is not)
    let mut inside: Vec<(f64, usize)> = ctx
        .corner_arcs
        .iter()
        .map(|&(arc, c)| (s0 + (arc - s0).rem_euclid(total), c))
        .filter(|&(a, _)| a > s0 + CORNER_FRACTION * ctx.edge && a < s1 - CORNER_FRACTION * ctx.edge)
        .collect();
    inside.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, c) in inside {
        if let Some(&pt) = ctx.lp.points.get(c) {
            cycle.push(ctx.base + plan.added.len() as u32);
            plan.added.push((pt, side_normal(&pt, &ctx.hint).unwrap_or(ctx.hint)));
        }
    }
    cycle.push(p1.id);
    cycle.dedup();
    while cycle.len() > 1 && cycle.first() == cycle.last() {
        cycle.pop();
    }
    (cycle.len() >= 3).then_some(cycle)
}

fn segments_cross(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> bool {
    let o = |p: (f64, f64), q: (f64, f64), r: (f64, f64)| (q.0 - p.0) * (r.1 - p.1) - (q.1 - p.1) * (r.0 - p.0);
    o(a, b, c) * o(a, b, d) < 0.0 && o(c, d, a) * o(c, d, b) < 0.0
}

/// Is the face a proper polygon in its local surface frame (the plane of its summed vertex
/// normals): no zero-length edge, positive area, no self-intersection?
fn face_is_proper(pts: &[(Point3, Vec3)]) -> bool {
    let n = pts.len();
    let normal: Vec3 = pts.iter().map(|p| p.1).sum();
    let Some(axis) = normal.try_normalize(1e-9) else { return false };
    let e1 = any_perpendicular(&axis);
    let e2 = axis.cross(&e1);
    let flat: Vec<(f64, f64)> = pts.iter().map(|p| (p.0.coords.dot(&e1), p.0.coords.dot(&e2))).collect();
    let area: f64 =
        (0..n).map(|k| flat[k].0 * flat[(k + 1) % n].1 - flat[(k + 1) % n].0 * flat[k].1).sum::<f64>() / 2.0;
    let edges_ok = (0..n).all(|k| (pts[(k + 1) % n].0 - pts[k].0).norm() > 1e-9);
    let simple = (0..n).all(|i| {
        (i + 2..n)
            .filter(|&j| (j + 1) % n != i)
            .all(|j| !segments_cross(flat[i], flat[(i + 1) % n], flat[j], flat[(j + 1) % n]))
    });
    area > 1e-12 && edges_ok && simple
}

/// Splitting levels a polygon is checked through: the conversion of the layout to quads. This is
/// an early exit only; the commands validate every real split and the final layout.
const CHECKED_SPLITS: usize = 1;

/// Is the polygon proper, and are all the quads it converts to (the fan of `split_to_quads`, as
/// `quad.retopo` splits a layout with any non-quad face), and their own splits, proper too?
fn polygon_and_quads_proper(pts: &[(Point3, Vec3)]) -> bool {
    if pts.len() < 3 || !face_is_proper(pts) {
        return false;
    }
    let mut level = PolyMesh {
        vertices: pts.iter().map(|p| p.0).collect(),
        normals: pts.iter().map(|p| p.1).collect(),
        faces: vec![(0..pts.len() as u32).collect()],
    };
    for _ in 0..CHECKED_SPLITS {
        let quads = level.split_to_quads(&|p, n| Some((*p, *n)));
        let proper = quads.faces.iter().all(|f| {
            let q: Vec<(Point3, Vec3)> = f
                .iter()
                .filter_map(|&v| quads.vertices.get(v as usize).copied().zip(quads.normals.get(v as usize).copied()))
                .collect();
            q.len() == 4 && face_is_proper(&q)
        });
        if !proper {
            return false;
        }
        level = quads.to_poly();
    }
    true
}

/// Does every face the plan touches (old faces with a moved vertex, and the ring faces) stay
/// proper, also after conversion to quads?
fn plan_is_valid(poly: &PolyMesh, plan: &Plan) -> bool {
    let mut verts = poly.vertices.clone();
    let mut normals = poly.normals.clone();
    for &(id, p) in &plan.moved {
        if let Some(v) = verts.get_mut(id as usize) {
            *v = p;
        }
    }
    for &(p, n) in &plan.added {
        verts.push(p);
        normals.push(n);
    }
    let moved: BTreeSet<u32> = plan.moved.iter().map(|m| m.0).collect();
    let touched = poly.faces.iter().filter(|f| f.iter().any(|v| moved.contains(v))).chain(plan.faces.iter());
    touched.into_iter().all(|f| {
        let pts: Vec<(Point3, Vec3)> =
            f.iter().filter_map(|&v| verts.get(v as usize).copied().zip(normals.get(v as usize).copied())).collect();
        pts.len() == f.len() && polygon_and_quads_proper(&pts)
    })
}

/// For every quad of `layout`: is it improper (not positive area, a zero edge, or self-intersecting
/// in its local frame)?
#[must_use]
pub fn improper_quads(layout: &super::QuadMesh) -> Vec<bool> {
    layout
        .faces
        .iter()
        .map(|f| {
            let pts: Vec<(Point3, Vec3)> = f
                .iter()
                .filter_map(|&v| layout.vertices.get(v as usize).copied().zip(layout.normals.get(v as usize).copied()))
                .collect();
            pts.len() != 4 || !face_is_proper(&pts)
        })
        .collect()
}

/// How many quads of `layout` are improper.
#[must_use]
pub fn invalid_quads(layout: &super::QuadMesh) -> usize {
    improper_quads(layout).into_iter().filter(|&b| b).count()
}

/// Children of proper polygons that are improper after `poly` was split into `quads` (the faces
/// of `quads` come in the order of the faces of `poly`, one per corner each).
#[must_use]
pub fn split_defects(poly: &PolyMesh, quads: &super::QuadMesh) -> usize {
    let child_flags = improper_quads(quads);
    let mut next = 0usize;
    let mut defects = 0usize;
    for f in &poly.faces {
        let pts: Vec<(Point3, Vec3)> = f
            .iter()
            .filter_map(|&v| poly.vertices.get(v as usize).copied().zip(poly.normals.get(v as usize).copied()))
            .collect();
        let parent_proper = pts.len() == f.len() && pts.len() >= 3 && face_is_proper(&pts);
        let end = (next + f.len()).min(child_flags.len());
        if parent_proper {
            defects += child_flags[next.min(end)..end].iter().filter(|&&b| b).count();
        }
        next = end;
    }
    defects
}

/// The quads that use any vertex of `moved`, with the positions of `verts`: all proper?
fn touched_quads_proper(layout: &super::QuadMesh, verts: &[Point3], moved: &BTreeSet<u32>) -> bool {
    layout.faces.iter().filter(|f| f.iter().any(|v| moved.contains(v))).all(|f| {
        let pts: Vec<(Point3, Vec3)> = f
            .iter()
            .filter_map(|&v| verts.get(v as usize).copied().zip(layout.normals.get(v as usize).copied()))
            .collect();
        pts.len() == 4 && face_is_proper(&pts)
    })
}

/// The new positions of the boundary vertices of one loop of a split layout, solved like the
/// conform of the loop (order, minimum gap, forced corners), or why it cannot be done.
fn resnap_loop(
    layout: &super::QuadMesh,
    lp: &OutlineLoop,
    ids: &[u32],
    (edge, max_move): (f64, f64),
) -> Result<Vec<(u32, Point3)>, &'static str> {
    let points: Vec<Point3> = ids.iter().filter_map(|&v| layout.vertices.get(v as usize).copied()).collect();
    if points.len() != ids.len() {
        return Err("a loop vertex is missing");
    }
    let targets = arc_positions(lp, &points, edge).ok_or("the loop does not run once around the outline")?;
    let near: Vec<usize> =
        (0..points.len()).filter(|&i| lp.closest(&points[i]).is_some_and(|h| h.2 <= max_move)).collect();
    if near.len() < 2 {
        return Ok(Vec::new());
    }
    let near_targets: Vec<f64> = near.iter().map(|&i| targets[i]).collect();
    let forced = forced_corners(lp, &near_targets, edge);
    let arcs = solve_arcs(lp, &near_targets, &forced, MIN_PARTNER_GAP_FRACTION * edge)
        .ok_or("a span between corners is too short for its vertices")?;
    Ok(near
        .iter()
        .zip(&arcs)
        .enumerate()
        .filter_map(|(k, (&i, &arc))| {
            let at = forced.get(&k).map(|f| f.1).or_else(|| lp.at(arc).map(|a| a.0))?;
            Some((ids[i], at))
        })
        .collect())
}

/// Puts the free-boundary vertices of a split `layout` that are within `max_move` of `outline`
/// back onto it: one arc solve per loop (order, minimum gap including the closing pair), the
/// touched quads validated on a copy. A loop that fails keeps its positions and is reported.
#[must_use]
pub fn snap_to_outline(
    layout: &super::QuadMesh,
    outline: &Outline,
    edge: f64,
    max_move: f64,
) -> (super::QuadMesh, ConformReport) {
    let mut out = layout.clone();
    let faces: Vec<Vec<u32>> = layout.faces.iter().map(|f| f.to_vec()).collect();
    let loops = free_loops(&faces);
    let mut report = ConformReport { loops: loops.len(), ..ConformReport::default() };
    for ids in &loops {
        let points: Vec<Point3> = ids.iter().filter_map(|&v| layout.vertices.get(v as usize).copied()).collect();
        let Some(lp) = matching_loop(outline, &points).and_then(|k| outline.loops.get(k)) else {
            report.skipped += 1;
            report.reason.get_or_insert("no outline loop matches the layout loop");
            continue;
        };
        let moves = resnap_loop(layout, lp, ids, (edge, max_move)).and_then(|moves| {
            let mut verts = out.vertices.clone();
            for &(id, p) in &moves {
                if let Some(v) = verts.get_mut(id as usize) {
                    *v = p;
                }
            }
            let moved: BTreeSet<u32> = moves.iter().map(|m| m.0).collect();
            if touched_quads_proper(&out, &verts, &moved) {
                Ok(moves)
            } else {
                Err("re-snapping would leave an invalid quad")
            }
        });
        match moves {
            Ok(moves) if moves.is_empty() => {
                report.unchanged += 1;
                report.reason.get_or_insert("no layout vertex within reach of the outline");
            }
            Ok(moves) => {
                report.conformed += 1;
                report.snapped += moves.len();
                for (id, p) in moves {
                    if let Some(v) = out.vertices.get_mut(id as usize) {
                        *v = p;
                    }
                }
            }
            Err(why) => {
                report.rolled_back += 1;
                report.reason.get_or_insert(why);
            }
        }
    }
    (out, report)
}

/// The plan of one loop, or why it cannot be conformed.
fn try_loop(
    poly: &PolyMesh,
    lp: &OutlineLoop,
    ids: &[u32],
    (edge, slack): (f64, f64),
    side_normal: SideNormal,
) -> Result<Plan, &'static str> {
    let points: Vec<Point3> = ids.iter().filter_map(|&v| poly.vertices.get(v as usize).copied()).collect();
    if points.len() != ids.len() {
        return Err("a loop vertex is missing");
    }
    let targets = arc_positions(lp, &points, edge).ok_or("the loop does not run once around the outline")?;
    // vertices farther from the outline than the reach are a dent of the layout: no partner for them
    let near: Vec<usize> = (0..points.len())
        .filter(|&i| lp.closest(&points[i]).is_some_and(|h| h.2 <= DEEP_FRACTION * edge + slack))
        .collect();
    if near.len() < 2 {
        return Ok(Plan::default()); // nothing within reach: counted as unchanged
    }
    let near_targets: Vec<f64> = near.iter().map(|&i| targets[i]).collect();
    let forced = forced_corners(lp, &near_targets, edge);
    let arcs = solve_arcs(lp, &near_targets, &forced, MIN_PARTNER_GAP_FRACTION * edge)
        .ok_or("a span between corners is too short for its vertices")?;
    let plan = plan_loop(poly, lp, (ids, &near), (&arcs, &forced), (edge, slack), side_normal);
    if plan_is_valid(poly, &plan) { Ok(plan) } else { Err("snapping would leave an invalid face") }
}

/// Conforms the free boundary of `poly` to `outline` (see the module documentation). `edge` is
/// the edge length of the layout; `side_normal` gives the scan normal near a point on the side a
/// hint normal points to. A loop that cannot be conformed is left exactly as it was.
pub fn conform_to_outline(poly: &mut PolyMesh, outline: &Outline, edge: f64, side_normal: SideNormal) -> ConformReport {
    let loops = free_loops(&poly.faces);
    let mut report = ConformReport { loops: loops.len(), ..ConformReport::default() };
    for ids in &loops {
        let points: Vec<Point3> = ids.iter().filter_map(|&v| poly.vertices.get(v as usize).copied()).collect();
        let Some(lp) = matching_loop(outline, &points).and_then(|k| outline.loops.get(k)) else {
            report.skipped += 1;
            report.reason.get_or_insert("no outline loop matches the layout loop");
            continue;
        };
        match try_loop(poly, lp, ids, (edge, outline.slack), side_normal) {
            Ok(plan) if plan.moved.is_empty() && plan.added.is_empty() => {
                report.unchanged += 1;
                report.reason.get_or_insert("no layout vertex within reach of the outline");
            }
            Ok(plan) => {
                report.conformed += 1;
                report.snapped += plan.moved.len();
                report.ring_faces += plan.faces.len();
                for (id, p) in plan.moved {
                    if let Some(v) = poly.vertices.get_mut(id as usize) {
                        *v = p;
                    }
                }
                for (p, n) in plan.added {
                    poly.vertices.push(p);
                    poly.normals.push(n);
                }
                poly.faces.extend(plan.faces);
            }
            Err(why) => {
                report.rolled_back += 1;
                report.reason.get_or_insert(why);
            }
        }
    }
    report
}

#[cfg(test)]
mod tests;
