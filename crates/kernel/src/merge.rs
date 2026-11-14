//! Merging the patches of a freeform body that lie on one primitive into a single analytic face
//! (P9-O6b), by editing the topology of the record: nothing is cut, no sharp edge is needed.
//!
//! The border of a group is first put on its primitive (`snap`: the vertices, the border edges re-fitted,
//! the neighbouring patches following). Then the faces of the group go; their interior edges and vertices
//! go with them; the group's boundary loops (the edges it shares with the patches around it, or the open
//! border of a sheet) become the loops of one new face on the primitive's surface, so the boundary edges
//! stay shared with the neighbours. A plane or cylinder face is built by the kernel (`trim`: its trims, NURBS
//! form and parameter box); a cone or sphere face is placed here (`place`). The result is rebuilt by the
//! kernel and refused, group by group with the reason, when the kernel does not take it, when its volume or
//! topology class changed, when the kernel changed the use of the new face, or when the new face does not
//! tessellate to the area of the patches it replaced or does not agree with them (`check`, `settle`).
//!
//! Findings (solidcore 4f2da85). A face used against its natural normal (the wall of a bore) is kept as
//! such by a closed shell, and by nothing else: an open sheet keeps the `same` use of each face whatever the
//! record says, so the face comes back reversed against its neighbours (measured on a real scan: all the
//! normals of such faces against those of the patches; the group is refused, `prepare`). `canonicalize`
//! walks such a face from its other side, as the record's canonical order says. The boundary edges carry
//! the gap, times 1.1, as their tolerance (the mesher refuses a face whose edge is farther off its surface
//! than 10 chords plus the edge's tolerance), and their end vertices keep theirs. A cone or sphere needs its
//! NURBS form in the record, and that form must be the kernel's own (the full revolution with its arcs, the
//! face being a sub-box of it; any other parameterisation loses the label). The healer is not run on the
//! edited record (it changes the topology of such a body and the record order is lost).
//!
//! APP-GEOMETRY: stands in for the kernel making the pcurves (the face's trims in its own parameters)
//! of the edges of a face on a cone or a sphere: `through` chains cubic Béziers through the parameters
//! of the edge's samples (the parameters themselves are the kernel's `SurfaceEval::project`). Plane and
//! cylinder faces are built by the kernel (`trim`); its builder gives a partial cone or sphere as an
//! unlabelled rational spline, which this application does not take as an analytic face.
use crate::hybrid::{Shape, V, add, cross, norm, scale, sub, unit};
use crate::nurbs::{SurfaceProbe, domain, is_closed};
use crate::record::brep::{Brep, Curve2, Face, Loop, OrientedEdge, Shell, Surface};
use crate::{KernelError, Result, finite_all};

mod boundary;
mod check;
mod place;
mod samples;
mod settle;
mod snap;
mod trim;

use boundary::{Boundary, LoopEdge, boundary, edge_points};
use place::{Gauge, Placement, place};
pub use samples::{connected_groups, face_area, face_samples, grid_area, pinch_free_groups, split_round};
use settle::{Run, settle, trusted_volume};
use snap::{Target, boundary_gap, snap_boundaries};
use std::collections::BTreeSet;

/// The largest edge-to-surface gap a merge accepts (mm); the caller passes its own, smaller one.
pub const MAX_GAP: f64 = 0.5;

/// How far a boundary may be from the primitive once it has been put on it, as a share of the gap the
/// caller allows (the boundary is moved onto the primitive and its curve re-fitted there; what is left
/// is the fit of a cubic to an arc, a few micrometres).
const FINAL_GAP_SHARE: f64 = 0.1;

/// Cells a side of the grid the area of a patch is taken from.
const AREA_GRID: usize = 8;
/// What one merge did.
#[derive(Clone, Debug)]
pub struct Merged {
    /// The body with the group replaced by one analytic face.
    pub brep: Brep,
    /// For each face of the input body, its face in `brep`; `None` for the removed ones. The
    /// new face is the image of the lowest face of the group.
    pub face_map: Vec<Option<usize>>,
    /// The new face in `brep`.
    pub face: usize,
    /// The largest distance of a boundary edge of the new face from its surface (mm).
    pub gap: f64,
}

/// A group of faces to merge and the primitive they lie on.
#[derive(Clone, Copy, Debug)]
pub struct GroupSpec<'a> {
    /// Faces of the input body.
    pub faces: &'a [usize],
    /// The primitive.
    pub shape: &'a Shape,
}

/// The merged groups of one body: for each spec the new face and its gap, or why it stays patches.
#[derive(Clone, Debug)]
pub struct Batch {
    /// The body with every merged group replaced by its face.
    pub brep: Brep,
    /// For each face of the input body, its face in `brep`; `None` for the removed ones.
    pub face_map: Vec<Option<usize>>,
    /// Per spec, in order.
    pub groups: Vec<std::result::Result<GroupResult, String>>,
}

/// A group that was merged.
#[derive(Clone, Copy, Debug)]
pub struct GroupResult {
    /// The new face in the body.
    pub face: usize,
    /// The largest distance of its boundary edges from the primitive (mm).
    pub gap: f64,
    /// The area of the patches it replaced (mm2).
    pub patch_area: f64,
}

/// The outward direction of a patch face at the middle of its domain (its natural normal turned
/// by the shell's use of it).
fn patch_outward(body: &Brep, face: usize) -> Result<V> {
    let f = body.faces.get(face).ok_or_else(|| KernelError::Argument("face index out of range".into()))?;
    if !matches!(f.surface, Surface::Spline { .. } | Surface::RationalSpline { .. }) {
        return Err(KernelError::Argument("only spline patches are merged".into()));
    }
    let [[u0, u1], [v0, v1]] = domain(&f.surface)?;
    let probe = SurfaceProbe::new(&f.surface)?;
    let (um, vm, du, dv) = (0.5 * (u0 + u1), 0.5 * (v0 + v1), 1e-3 * (u1 - u0), 1e-3 * (v1 - v0));
    let a = sub(probe.eval(um + du, vm)?, probe.eval(um - du, vm)?);
    let b = sub(probe.eval(um, vm + dv)?, probe.eval(um, vm - dv)?);
    let n = unit(cross(a, b)).ok_or_else(|| KernelError::Argument("a degenerate patch".into()))?;
    let forward = body.shells.iter().find_map(|s| match s {
        Shell::Faces { faces, .. } => faces.iter().find(|o| o.face as usize == face).map(|o| o.forward),
        _ => None,
    });
    Ok(if forward == Some(false) { scale(n, -1.0) } else { n })
}

/// The pcurve of a boundary edge on the new face, along the edge's own interval.
fn pcurve(body: &Brep, edge: u32, placed: &Placement) -> Result<Curve2> {
    let e = body.edges.get(edge as usize).ok_or_else(|| KernelError::Argument("edge index".into()))?;
    let vertex = |i: u32| body.vertices.get(i as usize).map(|v| v.point);
    let [lo, hi] = e.interval;
    if let (true, crate::record::brep::Curve::Spline { knots, points }) = (placed.is_plane(), &e.curve) {
        let pts = points.iter().map(|p| placed.param(*p)).collect();
        return Ok(Curve2::Spline { knots: knots.clone(), points: pts });
    }
    if placed.is_plane() {
        let (a, b) =
            vertex(e.vertices[0]).zip(vertex(e.vertices[1])).ok_or_else(|| KernelError::Argument("vertex".into()))?;
        return Ok(Curve2::Spline { knots: vec![lo, lo, hi, hi], points: vec![placed.param(a), placed.param(b)] });
    }
    Ok(through(&edge_points(body, edge)?, lo, hi, placed))
}

/// A C0 chain of cubic Béziers through the surface parameters of the samples `points` (equally
/// spaced over `[lo, hi]`), with Catmull-Rom tangents: the pcurve of an edge on a curved surface.
fn through(points: &[V], lo: f64, hi: f64, placed: &Placement) -> Curve2 {
    let q: Vec<[f64; 2]> = points.iter().map(|p| placed.param(*p)).collect();
    let n = q.len().saturating_sub(1).max(1);
    let at = |i: usize| q.get(i.min(n)).copied().unwrap_or([0.0; 2]);
    let tangent = |i: usize| -> [f64; 2] {
        let (a, b) = (at(i.saturating_sub(1)), at(i + 1));
        let k = if i == 0 || i == n { 1.0 } else { 0.5 };
        [k * (b[0] - a[0]) / 3.0, k * (b[1] - a[1]) / 3.0]
    };
    let mut pts = vec![at(0)];
    for i in 0..n {
        let (p0, p1, t0, t1) = (at(i), at(i + 1), tangent(i), tangent(i + 1));
        pts.push([p0[0] + t0[0], p0[1] + t0[1]]);
        pts.push([p1[0] - t1[0], p1[1] - t1[1]]);
        pts.push(p1);
    }
    let mut knots = vec![lo; 4];
    for i in 1..n {
        knots.extend([lo + (hi - lo) * i as f64 / n as f64; 3]);
    }
    knots.extend([hi; 4]);
    Curve2::Spline { knots, points: pts }
}

/// One group ready to be put in the record.
struct Prepared {
    faces: Vec<usize>,
    edge_list: Vec<LoopEdge>,
    /// The area of the patches it replaces.
    area: f64,
    cloud: check::Cloud,
    face: Face,
    forward: bool,
    gap: f64,
    /// The gap the caller allowed: how far the new face may lie from the patches inside them.
    limit: f64,
    shell: usize,
}

/// The parameters of the points along a loop of the face, as the loop runs.
fn loop_points(body: &Brep, edges: &[LoopEdge], placed: &Placement) -> Result<Vec<[f64; 2]>> {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    for le in edges {
        let mut p = edge_points(body, le.edge)?;
        if !le.forward {
            p.reverse();
        }
        pts.extend(p.iter().map(|q| placed.param(*q)));
    }
    if pts.iter().any(|q| !finite_all(q)) {
        return Err(KernelError::Argument("a boundary point has no parameters on the surface".into()));
    }
    Ok(pts)
}

/// The signed area of a polygon (positive when it runs counter-clockwise).
fn polygon_area(pts: &[[f64; 2]]) -> f64 {
    let n = pts.len();
    let at = |i: usize| pts.get(i % n.max(1)).copied().unwrap_or([0.0; 2]);
    0.5 * (0..n).map(|i| at(i)[0] * at(i + 1)[1] - at(i + 1)[0] * at(i)[1]).sum::<f64>()
}

/// Is `p` inside the polygon `poly` (even-odd rule)?
fn inside(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let n = poly.len();
    let at = |i: usize| poly.get(i % n.max(1)).copied().unwrap_or([0.0; 2]);
    let mut crossings = 0;
    for i in 0..n {
        let (a, b) = (at(i), at(i + 1));
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]) {
            crossings += 1;
        }
    }
    crossings % 2 == 1
}

/// The loops as `(signed area, points)`, the outer one first: it runs counter-clockwise, the others
/// (holes) clockwise and inside it.
fn validate_loops(loops: &[(f64, &[[f64; 2]])]) -> Result<()> {
    let Some((outer_area, outer)) = loops.first() else { return Err(KernelError::Argument("no loop".into())) };
    if *outer_area <= 0.0 {
        return Err(KernelError::Argument("the outer boundary of the group does not run counter-clockwise".into()));
    }
    for (area, pts) in loops.iter().skip(1) {
        let first = pts.first().copied().unwrap_or([0.0; 2]);
        if *area >= 0.0 || !inside(outer, first) {
            return Err(KernelError::Argument("a hole of the group winds the wrong way or lies outside it".into()));
        }
    }
    Ok(())
}

/// The loops of the new face, the outer one (the largest) first, each as the face's loop. The outer
/// loop must run counter-clockwise seen from the side the patches face, the others (holes) the other
/// way round and inside the outer one.
fn face_loops(body: &Brep, b: &Boundary, placed: &Placement) -> Result<Vec<Loop>> {
    // seen from the side the patches face, the loops run counter-clockwise; a surface whose natural
    // normal points the other way (a hole, a dimple) sees them clockwise
    let side = if placed.flip { -1.0 } else { 1.0 };
    let mut sized: Vec<(f64, Vec<[f64; 2]>, &Vec<LoopEdge>)> = Vec::new();
    for l in &b.loops {
        let pts = loop_points(body, l, placed)?;
        sized.push((side * polygon_area(&pts), pts, l));
    }
    sized.sort_by(|a, c| c.0.abs().total_cmp(&a.0.abs()));
    validate_loops(&sized.iter().map(|(a, p, _)| (*a, p.as_slice())).collect::<Vec<_>>())?;
    let mut out = Vec::with_capacity(sized.len());
    for (_, _, l) in sized {
        let mut edges: Vec<OrientedEdge> = Vec::with_capacity(l.len());
        for le in l {
            edges.push(OrientedEdge { edge: le.edge, forward: le.forward, pcurve: pcurve(body, le.edge, placed)? });
        }
        if placed.flip {
            edges.reverse();
            for e in &mut edges {
                e.forward = !e.forward;
            }
        }
        out.push(Loop::Edges { edges });
    }
    Ok(out)
}

/// Works out the new face of `spec` (nothing is changed yet): its boundary may be `gap_limit` from the
/// primitive at most, and the face may lie `max_gap` from the patches it replaces.
fn prepare(body: &Brep, spec: &GroupSpec, max_gap: f64, gap_limit: f64) -> Result<Prepared> {
    let b = boundary(body, spec.faces)?;
    let mut outward = [0.0; 3];
    for &f in spec.faces {
        outward = add(outward, patch_outward(body, f)?);
    }
    if !finite_all(&outward) || norm(outward) < 1e-9 {
        return Err(KernelError::Argument("the patches of the group do not face one way".into()));
    }
    let mut samples: Vec<V> = Vec::new();
    for le in b.loops.iter().flatten() {
        samples.extend(edge_points(body, le.edge)?);
    }
    let gap = boundary_gap(body, &b, &Gauge::new(spec.shape)?)?;
    if gap > gap_limit {
        return Err(KernelError::Argument(format!(
            "the group's boundary is {gap:.4} mm from the surface (limit {gap_limit:.4})"
        )));
    }
    let placed = place(spec.shape, &samples, outward)?;
    if placed.flip && !is_closed(body) {
        return Err(KernelError::Argument(
            "a face that looks against the natural normal of its surface (a bore, a dimple) is not merged into an open \
             sheet: the kernel keeps only the same use of a sheet's faces, so the new face would come back reversed"
                .into(),
        ));
    }
    let face = if matches!(spec.shape, Shape::Plane { .. } | Shape::Cylinder { .. }) {
        trim::built_face(body, &b, &placed.surface, !placed.flip)?
    } else {
        // the kernel's builder gives a partial cone or sphere as an unlabelled rational spline: these two
        // are placed here, with their own pcurves (see the header)
        let loops = face_loops(body, &b, &placed)?;
        Face { surface: placed.surface, nurbs: placed.nurbs, domain: placed.domain, loops }
    };
    let area = spec.faces.iter().filter_map(|&f| face_area(body, f, AREA_GRID)).sum();
    let edge_list = b.loops.iter().flatten().copied().collect();
    let cloud = check::cloud(body, spec.faces, samples)?;
    Ok(Prepared {
        faces: spec.faces.to_vec(),
        edge_list,
        area,
        cloud,
        face,
        forward: !placed.flip,
        gap,
        limit: max_gap,
        shell: b.shell,
    })
}

/// Merges each group of `specs` into one face on its shape, all in one rebuild of the body where
/// the kernel takes that, else one group after the other (a group the kernel refuses stays patches
/// and says why). Groups must not share a face. Each group's boundary may leave its shape by
/// `max_gap` at most.
///
/// # Errors
/// An invalid `max_gap` or a cancellation (`cancel` says so between groups). A group that could not be
/// merged says why in its [`Batch::groups`] entry; when none could be, the batch has the body as it was.
pub fn merge_faces_onto_shapes(
    body: &Brep,
    specs: &[GroupSpec],
    max_gap: f64,
    cancel: &dyn Fn() -> bool,
) -> Result<Batch> {
    merge_with(body, specs, max_gap, cancel, true)
}

/// Each spec prepared (or why not), in order. Groups must not share a face; the groups beyond
/// [`MAX_GROUPS`] are not prepared.
fn prepare_all(
    body: &Brep,
    specs: &[GroupSpec],
    limits: (f64, &[bool]),
    cancel: &dyn Fn() -> bool,
) -> Result<Vec<std::result::Result<Prepared, String>>> {
    let (max_gap, moved) = limits;
    let mut claimed: BTreeSet<usize> = BTreeSet::new();
    let mut prepared = Vec::with_capacity(specs.len());
    for (k, spec) in specs.iter().enumerate() {
        if cancel() {
            return Err(KernelError::Argument("cancelled".into()));
        }
        let p = if k >= MAX_GROUPS {
            Err(format!("beyond the {MAX_GROUPS} groups one merge takes"))
        } else if spec.faces.iter().any(|f| claimed.contains(f)) {
            Err("a face belongs to another group".to_string())
        } else {
            let gap_limit = if moved.get(k).copied().unwrap_or(false) { FINAL_GAP_SHARE * max_gap } else { max_gap };
            prepare(body, spec, max_gap, gap_limit).map_err(|e| e.to_string())
        };
        if p.is_ok() {
            claimed.extend(spec.faces.iter().copied());
        }
        prepared.push(p);
    }
    Ok(prepared)
}

/// The batch in which no group was merged: the body as it was and each group's reason.
fn unchanged(body: &Brep, reasons: Vec<String>) -> Batch {
    Batch {
        brep: body.clone(),
        face_map: (0..body.faces.len()).map(Some).collect(),
        groups: reasons.into_iter().map(Err).collect(),
    }
}

/// Can the group's face be placed on its primitive at all (its surface reaches no seam or pole, and in an
/// open sheet it does not look against the natural normal of its surface)? A group that cannot is not moved
/// onto its primitive: its neighbours would be moved for nothing.
fn placeable(body: &Brep, spec: &GroupSpec, closed: bool) -> bool {
    let Ok(b) = boundary(body, spec.faces) else { return false };
    let mut outward = [0.0; 3];
    for &f in spec.faces {
        match patch_outward(body, f) {
            Ok(n) => outward = add(outward, n),
            Err(_) => return false,
        }
    }
    let mut samples: Vec<V> = Vec::new();
    for le in b.loops.iter().flatten() {
        match edge_points(body, le.edge) {
            Ok(p) => samples.extend(p),
            Err(_) => return false,
        }
    }
    place(spec.shape, &samples, outward).is_ok_and(|p| closed || !p.flip)
}

/// The body with the boundary of each group that lies within `max_gap` of its primitive moved onto it,
/// and for each spec whether its boundary was moved (a body the step cannot edit is returned as it was,
/// with no spec moved).
fn snapped(body: &Brep, specs: &[GroupSpec], max_gap: f64) -> (Brep, Vec<bool>) {
    let gauges: Vec<Option<Gauge>> = specs.iter().take(MAX_GROUPS).map(|s| Gauge::new(s.shape).ok()).collect();
    let mut claimed: BTreeSet<usize> = BTreeSet::new();
    let closed = is_closed(body);
    let (mut targets, mut flags) = (Vec::new(), vec![false; specs.len()]);
    for (k, (spec, gauge)) in specs.iter().zip(&gauges).enumerate() {
        let Some(gauge) = gauge.as_ref() else { continue };
        if spec.faces.iter().any(|f| claimed.contains(f)) {
            continue;
        }
        let near = boundary(body, spec.faces).and_then(|b| boundary_gap(body, &b, gauge)).is_ok_and(|g| g <= max_gap);
        if near && placeable(body, spec, closed) {
            claimed.extend(spec.faces.iter().copied());
            targets.push(Target { faces: spec.faces, gauge });
            if let Some(f) = flags.get_mut(k) {
                *f = true;
            }
        }
    }
    let result = snap_boundaries(body, &targets);
    result.map_or_else(|_| (body.clone(), vec![false; specs.len()]), |b| (b, flags))
}

/// [`merge_faces_onto_shapes`], with `all_at_once` false going straight to the group-by-group path
/// (the path the kernel's refusal of the whole takes).
fn merge_with(
    original: &Brep,
    specs: &[GroupSpec],
    max_gap: f64,
    cancel: &dyn Fn() -> bool,
    all_at_once: bool,
) -> Result<Batch> {
    if !(max_gap.is_finite() && max_gap > 0.0 && max_gap <= MAX_GAP) {
        return Err(KernelError::Argument(format!("the gap must be in (0, {MAX_GAP}]")));
    }
    let (body, moved) = snapped(original, specs, max_gap);
    let body = &body;
    let prepared = prepare_all(body, specs, (max_gap, &moved), cancel)?;
    let volume = is_closed(body).then(|| trusted_volume(body)).transpose()?;
    let all: Vec<&Prepared> = prepared.iter().filter_map(|p| p.as_ref().ok()).collect();
    let own =
        |p: &std::result::Result<Prepared, String>, why: &str| p.as_ref().err().cloned().unwrap_or(why.to_string());
    if all.is_empty() {
        return Ok(unchanged(original, prepared.iter().map(|p| own(p, "")).collect()));
    }
    let run = Run { cancel, all_at_once, chord: 2.0 * max_gap };
    let (brep, face_map, mut verdicts) = match settle(body, volume, &all, &run) {
        Ok(settled) => settled,
        Err(_) if cancel() => return Err(KernelError::Argument("cancelled".into())),
        Err(e) => return Ok(unchanged(original, prepared.iter().map(|p| own(p, &e.to_string())).collect())),
    };
    verdicts.reverse();
    let groups = specs
        .iter()
        .zip(&prepared)
        .map(|(spec, p)| {
            let p = p.as_ref().map_err(Clone::clone)?;
            verdicts.pop().unwrap_or_else(|| Err("no verdict".to_string()))?;
            let first = spec.faces.iter().copied().min().unwrap_or(0);
            let face = face_map.get(first).copied().flatten().ok_or("face map")?;
            Ok(GroupResult { face, gap: p.gap, patch_area: p.area })
        })
        .collect();
    Ok(Batch { brep, face_map, groups })
}

/// The most groups one merge takes (the rest stay patches, reported as such): each costs a rebuild
/// where the kernel refuses the whole.
pub const MAX_GROUPS: usize = 64;

/// Merges the faces `group` of `body` into one face on `shape`, within `max_gap` of it.
///
/// # Errors
/// A group that is not one connected set of patches of a face shell, a shape the merge does not
/// place or whose surface the boundary leaves by more than `max_gap`, a face that would wrap round
/// its surface's axis, and a result the kernel does not take or whose volume differs from the
/// body's by more than the gap allows.
pub fn merge_faces_onto_shape(body: &Brep, group: &[usize], shape: &Shape, max_gap: f64) -> Result<Merged> {
    let batch = merge_faces_onto_shapes(body, &[GroupSpec { faces: group, shape }], max_gap, &|| false)?;
    match batch.groups.first() {
        Some(Ok(g)) => Ok(Merged { brep: batch.brep, face_map: batch.face_map, face: g.face, gap: g.gap }),
        Some(Err(why)) => Err(KernelError::Argument(why.clone())),
        None => Err(KernelError::Argument("no group".into())),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const SQUARE: [[f64; 2]; 4] = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]];

    fn reversed(p: &[[f64; 2]]) -> Vec<[f64; 2]> {
        p.iter().rev().copied().collect()
    }

    #[test]
    fn an_outer_loop_and_a_clockwise_hole_inside_it_are_accepted() {
        let hole = reversed(&[[2.0, 2.0], [4.0, 2.0], [4.0, 4.0], [2.0, 4.0]]);
        let loops = [(polygon_area(&SQUARE), &SQUARE[..]), (polygon_area(&hole), &hole[..])];
        assert!(validate_loops(&loops).is_ok());
    }

    #[test]
    fn a_hole_that_winds_like_the_outer_loop_is_refused() {
        let hole = [[2.0, 2.0], [4.0, 2.0], [4.0, 4.0], [2.0, 4.0]];
        let loops = [(polygon_area(&SQUARE), &SQUARE[..]), (polygon_area(&hole), &hole[..])];
        assert!(validate_loops(&loops).is_err());
    }

    #[test]
    fn a_hole_outside_the_outer_loop_and_a_clockwise_outer_loop_are_refused() {
        let far = reversed(&[[20.0, 2.0], [22.0, 2.0], [22.0, 4.0], [20.0, 4.0]]);
        assert!(validate_loops(&[(polygon_area(&SQUARE), &SQUARE[..]), (polygon_area(&far), &far[..])]).is_err());
        let cw = reversed(&SQUARE);
        assert!(validate_loops(&[(polygon_area(&cw), &cw[..])]).is_err());
        assert!(validate_loops(&[]).is_err());
    }

    /// A flat sheet of `n x n` bilinear patches in z = 0, numbered `i * n + j`.
    fn flat_sheet(n: usize) -> Brep {
        let id = |i: usize, j: usize| (i * (n + 1) + j) as u32;
        let pt = |i: usize, j: usize| [i as f64 * 4.0, j as f64 * 4.0, 0.0];
        let mut nets = Vec::new();
        let mut corners = Vec::new();
        for i in 0..n {
            for j in 0..n {
                nets.push(vec![vec![pt(i, j), pt(i, j + 1)], vec![pt(i + 1, j), pt(i + 1, j + 1)]]);
                corners.push([id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        crate::nurbs::sheet_from_bezier_network(&nets, &corners).expect("a flat sheet")
    }

    /// Groups of two neighbouring patches along each row of the sheet.
    fn pairs(n: usize) -> Vec<Vec<usize>> {
        (0..n).flat_map(|i| (0..n / 2).map(move |k| vec![i * n + 2 * k, i * n + 2 * k + 1])).collect()
    }

    fn plane() -> Shape {
        Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] }
    }

    #[test]
    fn the_group_by_group_path_gives_the_same_faces_as_the_whole() {
        let body = flat_sheet(6);
        let groups = pairs(6);
        let shape = plane();
        let specs: Vec<GroupSpec> = groups.iter().map(|g| GroupSpec { faces: g, shape: &shape }).collect();
        let whole = merge_with(&body, &specs, 0.05, &|| false, true).unwrap();
        let each = merge_with(&body, &specs, 0.05, &|| false, false).unwrap();
        assert_eq!(whole.brep.faces.len(), 18, "18 pairs of patches became 18 faces");
        assert_eq!(each.brep.faces.len(), whole.brep.faces.len());
        assert!(each.groups.iter().all(|g| g.is_ok()));
    }

    #[test]
    fn a_merge_is_cancelled_between_groups_and_takes_a_limited_number() {
        let body = flat_sheet(6);
        let groups = pairs(6);
        let shape = plane();
        let specs: Vec<GroupSpec> = groups.iter().map(|g| GroupSpec { faces: g, shape: &shape }).collect();
        assert!(merge_with(&body, &specs, 0.05, &|| true, true).is_err(), "cancelled");
        let polls = std::cell::Cell::new(0);
        let late = || {
            polls.set(polls.get() + 1);
            polls.get() > 3
        };
        assert!(merge_with(&body, &specs, 0.05, &late, false).is_err(), "cancelled part way");
        let big = flat_sheet(16);
        let many = pairs(16);
        let specs: Vec<GroupSpec> = many.iter().map(|g| GroupSpec { faces: g, shape: &shape }).collect();
        assert!(many.len() > MAX_GROUPS);
        let r = merge_with(&big, &specs, 0.05, &|| false, true).unwrap();
        let (ok, refused) =
            (r.groups.iter().filter(|g| g.is_ok()).count(), r.groups.iter().filter(|g| g.is_err()).count());
        assert_eq!((ok, refused), (MAX_GROUPS, many.len() - MAX_GROUPS));
    }
}
