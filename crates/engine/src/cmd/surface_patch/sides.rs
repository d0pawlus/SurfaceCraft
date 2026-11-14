//! The sides of a `surface.patch` outline: body edges and polylines resolved to points (a body edge
//! densified, a polyline as given), chained into one closed loop, and their corners made to meet.
use super::{ContinuityIn, FaceRefIn, SideIn};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use std::collections::BTreeMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_kernel::patch_fit::{Continuity, G0_TARGET};
use surfacecraft_kernel::record::brep::{Brep, Face};
use surfacecraft_kernel::tess::tessellate_topo;

/// A point in space.
pub(super) type V3 = [f64; 3];
/// A point of the parameter plane.
pub(super) type V2 = [f64; 2];

/// Most sides of an outline.
pub(super) const MAX_SIDES: usize = 64;
/// Most points of a polyline side, as given (a polyline is not densified); the kernel takes 10 000 per
/// side.
const MAX_SIDE_POINTS: usize = 9_900;
/// A body edge is densified so that no step is longer than its length over this many.
const DENSE_SEGMENTS: usize = 32;
/// Chord of the tessellation the edge polylines come from, as a share of the body's diagonal, within
/// [`EDGE_CHORD_RANGE`] (mm).
const EDGE_CHORD_SHARE: f64 = 1e-4;
const EDGE_CHORD_RANGE: (f64, f64) = (0.002, 0.02);

/// A side with its points before densifying: a polyline's are the caller's, a body edge's its tessellation.
pub(super) struct Resolved {
    /// Index in the request's `sides`.
    pub index: usize,
    /// The side's points, at least 2.
    pub points: Vec<V3>,
    /// The side is an edge of a body (its points stay where they are at a corner).
    pub from_edge: bool,
    /// Continuity asked for.
    pub continuity: Continuity,
    /// The face to continue (G1/G2).
    pub reference: Option<Face>,
    /// The side is approximate (traced on a scan): the patch follows the data.
    pub soft: bool,
}

pub(super) fn dist(a: V3, b: V3) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

fn length(points: &[V3]) -> f64 {
    points.iter().zip(points.iter().skip(1)).map(|(a, b)| dist(*a, *b)).sum()
}

fn params(msg: impl Into<String>) -> CmdError {
    CmdError::Params(msg.into())
}

/// The body or sheet an object reference names, with its record.
fn body_of(s: &Session, r: &ObjRef) -> CmdResult<(ObjectId, Arc<Brep>)> {
    let (id, kind) = r.resolve_any(s)?;
    let found = match kind {
        ObjectKind::Body => s.doc.body(id).map(|b| b.brep.clone()),
        ObjectKind::Surface => s.doc.surface(id).map(|o| o.sheet.clone()),
        _ => None,
    };
    found.map(|b| (id, b)).ok_or_else(|| params(format!("{id} is a {kind:?}, not a body or a surface")))
}

/// Chord of the tessellation an edge is sampled from.
fn edge_chord(brep: &Brep) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &brep.vertices {
        for c in 0..3 {
            lo[c] = lo[c].min(v.point[c]);
            hi[c] = hi[c].max(v.point[c]);
        }
    }
    let diag = dist(lo, hi);
    if diag.is_finite() { (diag * EDGE_CHORD_SHARE).clamp(EDGE_CHORD_RANGE.0, EDGE_CHORD_RANGE.1) } else { 0.01 }
}

/// Edge polylines of the bodies the sides refer to, tessellated once per body.
#[derive(Default)]
struct Edges(BTreeMap<u64, Vec<Vec<V3>>>);

impl Edges {
    fn polyline(&mut self, id: ObjectId, brep: &Brep, edge: usize) -> CmdResult<Vec<V3>> {
        if let std::collections::btree_map::Entry::Vacant(slot) = self.0.entry(id.0) {
            let topo = tessellate_topo(brep, Some(edge_chord(brep)))?;
            let lines: Vec<Vec<V3>> = topo.edges.iter().map(|e| e.iter().map(|p| [p.x, p.y, p.z]).collect()).collect();
            slot.insert(lines);
        }
        self.0
            .get(&id.0)
            .and_then(|l| l.get(edge))
            .cloned()
            .ok_or_else(|| CmdError::Failed(format!("edge {edge} of {id} could not be sampled")))
    }
}

/// The face a G1/G2 side continues: `face` when given, else the single face that uses the edge.
fn reference_face(
    s: &Session,
    i: usize,
    face: Option<&FaceRefIn>,
    edge: Option<&(Arc<Brep>, usize)>,
) -> CmdResult<Face> {
    if let Some(f) = face {
        let (_, brep) = body_of(s, &f.body)?;
        return brep.faces.get(f.face).cloned().ok_or_else(|| {
            params(format!("side {i}: the body has {} faces, there is no face {}", brep.faces.len(), f.face))
        });
    }
    let Some((brep, e)) = edge else {
        return Err(params(format!("side {i}: give `face` (a polyline side has no face of its own)")));
    };
    let mut used: Vec<u32> = brep.edges.get(*e).map(|ed| ed.uses.iter().map(|u| u.face).collect()).unwrap_or_default();
    used.sort_unstable();
    used.dedup();
    match used.as_slice() {
        [only] => brep
            .faces
            .get(*only as usize)
            .cloned()
            .ok_or_else(|| params(format!("side {i}: the edge names a face the body does not have"))),
        _ => Err(params(format!("side {i}: give `face` (the edge has {} faces)", used.len()))),
    }
}

fn polyline(i: usize, p: &[V3]) -> CmdResult<Vec<V3>> {
    if p.len() < 2 || p.len() > MAX_SIDE_POINTS {
        return Err(params(format!("side {i} has {} points; 2 to {MAX_SIDE_POINTS} are accepted", p.len())));
    }
    if p.iter().flatten().any(|c| !c.is_finite()) {
        return Err(params(format!("side {i} has a point that is not finite")));
    }
    Ok(p.to_vec())
}

/// Resolves every side of the request to points, a continuity and, for G1/G2, a reference face.
pub(super) fn resolve(s: &Session, sides: &[SideIn]) -> CmdResult<Vec<Resolved>> {
    if sides.is_empty() || sides.len() > MAX_SIDES {
        return Err(params(format!("an outline has 1 to {MAX_SIDES} sides, not {}", sides.len())));
    }
    let mut edges = Edges::default();
    let mut out = Vec::with_capacity(sides.len());
    for (i, side) in sides.iter().enumerate() {
        let continuity = side.continuity.map_or(Continuity::G0, ContinuityIn::kernel);
        let (points, edge_of) = match (&side.body, side.edge, &side.points) {
            (Some(b), Some(e), None) => {
                let (id, brep) = body_of(s, b)?;
                if e >= brep.edges.len() {
                    return Err(params(format!("side {i}: {id} has {} edges, there is no edge {e}", brep.edges.len())));
                }
                let line = edges.polyline(id, &brep, e)?;
                if line.len() < 2 {
                    return Err(CmdError::Failed(format!("side {i}: edge {e} of {id} could not be sampled")));
                }
                (line, Some((brep, e)))
            }
            (None, None, Some(p)) => (polyline(i, p)?, None),
            _ => {
                return Err(params(format!(
                    "side {i}: give `body` and `edge`, or `points` (not both, and not neither)"
                )));
            }
        };
        let len = length(&points);
        if !(len.is_finite() && len > 0.0) {
            return Err(params(format!("side {i} has no length")));
        }
        let reference = if continuity == Continuity::G0 {
            None
        } else {
            Some(reference_face(s, i, side.face.as_ref(), edge_of.as_ref())?)
        };
        out.push(Resolved { index: i, points, from_edge: edge_of.is_some(), continuity, reference, soft: side.soft });
    }
    Ok(out)
}

fn open_loop(a: usize, b: usize, gap: f64, at: V3, limit: f64) -> CmdError {
    CmdError::Failed(format!(
        "the sides do not close: side {a} ends {gap:.4} mm from the nearest end of side {b}, at ({:.3}, {:.3}, {:.3}); \
         gaps up to gap_tolerance = {limit} mm are closed",
        at[0], at[1], at[2]
    ))
}

fn midpoint(a: V3, b: V3) -> V3 {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0]
}

/// Chains the sides into one closed loop (each side ends where the next starts, reversing sides as
/// needed, starting from the first) and closes gaps up to `gap_tol` by giving the two ends of a
/// corner one point: a body edge keeps its end, two polylines or two edges meet in the middle.
/// Returns the loop and the notes on the gaps that were closed.
pub(super) fn chain(sides: Vec<Resolved>, gap_tol: f64) -> CmdResult<(Vec<Resolved>, Vec<String>)> {
    let mut rest = sides;
    if rest.is_empty() {
        return Err(params("`sides` is empty"));
    }
    let mut order = vec![rest.remove(0)];
    while !rest.is_empty() {
        let Some((prev, end)) = order.last().and_then(|r| r.points.last().map(|p| (r.index, *p))) else { break };
        // the nearest free end of the remaining sides
        let mut best: Option<(usize, bool, f64)> = None;
        for (k, r) in rest.iter().enumerate() {
            let (Some(a), Some(b)) = (r.points.first(), r.points.last()) else { continue };
            let (da, db) = (dist(end, *a), dist(end, *b));
            let (rev, d) = if da <= db { (false, da) } else { (true, db) };
            if best.is_none_or(|(_, _, bd)| d < bd) {
                best = Some((k, rev, d));
            }
        }
        let Some((k, rev, d)) = best else { break };
        let mut next = rest.remove(k);
        if d > gap_tol {
            let near = (if rev { next.points.last() } else { next.points.first() }).copied().unwrap_or(end);
            return Err(open_loop(prev, next.index, d, midpoint(end, near), gap_tol));
        }
        if rev {
            next.points.reverse();
        }
        order.push(next);
    }
    // corners: where side k ends and side k + 1 starts
    let n = order.len();
    let mut corners: Vec<V3> = Vec::with_capacity(n);
    let mut notes = Vec::new();
    for k in 0..n {
        let (Some(a), Some(b)) = (order.get(k), order.get((k + 1) % n)) else { continue };
        let (Some(end), Some(start)) = (a.points.last(), b.points.first()) else { continue };
        let (end, start) = (*end, *start);
        let gap = dist(end, start);
        if gap > gap_tol {
            return Err(open_loop(a.index, b.index, gap, midpoint(end, start), gap_tol));
        }
        let corner = match (a.from_edge, b.from_edge) {
            (true, false) => end,
            (false, true) => start,
            _ => midpoint(end, start),
        };
        if gap > G0_TARGET {
            notes.push(format!(
                "corner at ({:.3}, {:.3}, {:.3}): a gap of {gap:.4} mm between side {} and side {} was closed",
                corner[0], corner[1], corner[2], a.index, b.index
            ));
        }
        corners.push(corner);
    }
    for (k, corner) in corners.iter().enumerate() {
        if let Some(last) = order.get_mut(k).and_then(|r| r.points.last_mut()) {
            *last = *corner;
        }
        if let Some(first) = order.get_mut((k + 1) % n).and_then(|r| r.points.first_mut()) {
            *first = *corner;
        }
    }
    Ok((order, notes))
}

/// The points of a body edge with steps inserted (in a straight line) so that none is longer than the
/// edge's length over [`DENSE_SEGMENTS`]; every given point stays. `uv` (one per point) is carried
/// along. A polyline is not densified, so the kernel can smooth it.
pub(super) fn densify(points: &[V3], uv: Option<&[V2]>) -> (Vec<V3>, Option<Vec<V2>>) {
    let step = length(points) / DENSE_SEGMENTS as f64;
    let mut out: Vec<V3> = Vec::with_capacity(points.len() + DENSE_SEGMENTS);
    let mut out_uv: Option<Vec<V2>> = uv.map(|_| Vec::with_capacity(points.len() + DENSE_SEGMENTS));
    for (k, p) in points.iter().enumerate() {
        out.push(*p);
        if let (Some(o), Some(q)) = (out_uv.as_mut(), uv.and_then(|u| u.get(k))) {
            o.push(*q);
        }
        let Some(next) = points.get(k + 1) else { continue };
        let d = dist(*p, *next);
        let pieces = if step > 0.0 && step.is_finite() { (d / step).ceil() as usize } else { 1 };
        let pieces = pieces.clamp(1, 2 * DENSE_SEGMENTS);
        for j in 1..pieces {
            let t = j as f64 / pieces as f64;
            out.push([p[0] + t * (next[0] - p[0]), p[1] + t * (next[1] - p[1]), p[2] + t * (next[2] - p[2])]);
            if let (Some(o), Some(a), Some(b)) =
                (out_uv.as_mut(), uv.and_then(|u| u.get(k)), uv.and_then(|u| u.get(k + 1)))
            {
                o.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
            }
        }
    }
    (out, out_uv)
}
