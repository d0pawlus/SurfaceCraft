//! Merging the patches of a freeform body that lie on one primitive into a single analytic face
//! (P9-O6b), by editing the topology of the record: nothing is cut, no sharp edge is needed.
//!
//! The faces of a group go; their interior edges and vertices go with them; the group's
//! boundary loop (the edges it shares with the patches around it, or the open border of a
//! sheet) becomes the outer loop of one new face on the primitive's surface, so the boundary
//! edges stay shared with the neighbours. The result is rebuilt by the kernel
//! (it builds the trims of the analytic face from the edges) and refused when the kernel does not
//! take it, when its volume or topology class changed, or when the new face does not tessellate to
//! the area of the patches it replaced.
//!
//! Findings (solidcore 4f2da85). The boundary edges are splines that lie on the primitive only
//! within the fit (`gap`, reported per face). The edge gets the gap, times 1.1, as its tolerance, and
//! its end vertices keep theirs: the mesher refuses a face whose edge is farther off its surface
//! than 10 chords plus the edge's tolerance (it names the edge), and with the vertices raised too it
//! lost area from a plane of 42 patches (40 % at a chord above 0.1 mm) and took minutes (measured).
//! A wrap of scan-fitted patches already carries gaps of this size on its edges (the kernel's
//! geometry check finds them in every freeform body). A cylinder, cone or sphere needs its NURBS
//! form in the record, and that form must be the kernel's own: the full revolution with its arcs,
//! the face being a sub-box of it; any other parameterisation loses the label. The healer is not
//! run on the edited record (it changes the topology of such a body and the record order is lost).
use crate::hybrid::{Shape, V, add, cross, norm, scale, sub, unit};
use crate::nurbs::{SurfaceProbe, canonicalize, domain, is_closed};
use crate::record::brep::{Brep, Curve2, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface};
use crate::solid::encode;
use crate::{KernelError, Result, finite_all};

mod boundary;
mod place;
mod revolve;
mod samples;

use boundary::{Boundary, LoopEdge, boundary, edge_points};
use place::{Placement, distance, place};
pub use samples::{connected_groups, face_area, face_samples, grid_area, pinch_free_groups};
use std::collections::{BTreeMap, BTreeSet};

/// The largest edge-to-surface gap a merge accepts (mm); the caller passes its own, smaller one.
pub const MAX_GAP: f64 = 0.5;

/// The share of its volume a merge may change a closed body by (patches that lie on the primitive
/// within the gap change it far less; this catches an inside-out or broken result).
const MAX_VOLUME_CHANGE: f64 = 0.02;
/// Relative accuracy of the volumes that are compared.
const VOLUME_ACCURACY: f64 = 1e-4;
/// The tolerance of a boundary edge as a multiple of its gap.
const EDGE_TOLERANCE_PER_GAP: f64 = 1.1;
/// Cells a side of the grid the area of a patch is taken from.
const AREA_GRID: usize = 8;
/// A merged face must tessellate to its patches' area within this share (a face the mesher cannot
/// triangulate, such as one with a neck narrower than the chord, loses area).
const AREA_AGREEMENT: f64 = 0.1;

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
    face: Face,
    forward: bool,
    gap: f64,
    shell: usize,
}

/// The signed area of a loop in the face's parameters (positive when it runs counter-clockwise).
fn loop_area(body: &Brep, edges: &[LoopEdge], placed: &Placement) -> Result<f64> {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    for le in edges {
        let mut p = edge_points(body, le.edge)?;
        if !le.forward {
            p.reverse();
        }
        pts.extend(p.iter().map(|q| placed.param(*q)));
    }
    let n = pts.len();
    let at = |i: usize| pts.get(i % n.max(1)).copied().unwrap_or([0.0; 2]);
    Ok(0.5 * (0..n).map(|i| at(i)[0] * at(i + 1)[1] - at(i + 1)[0] * at(i)[1]).sum::<f64>())
}

/// The loops of the new face, the outer one (the largest) first, each as the face's loop.
fn face_loops(body: &Brep, b: &Boundary, placed: &Placement) -> Result<Vec<Loop>> {
    let mut sized: Vec<(f64, &Vec<LoopEdge>)> = Vec::new();
    for l in &b.loops {
        // seen from the side the patches face, the loops run counter-clockwise; a surface whose natural
        // normal points the other way (a hole, a dimple) sees them clockwise
        let side = if placed.flip { -1.0 } else { 1.0 };
        sized.push((side * loop_area(body, l, placed)?, l));
    }
    sized.sort_by(|a, c| c.0.abs().total_cmp(&a.0.abs()));
    if sized.first().is_some_and(|(area, _)| *area <= 0.0) {
        return Err(KernelError::Argument("the outer boundary of the group does not run counter-clockwise".into()));
    }
    let mut out = Vec::with_capacity(sized.len());
    for (_, l) in sized {
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

/// Works out the new face of `spec` (nothing is changed yet).
fn prepare(body: &Brep, spec: &GroupSpec, max_gap: f64) -> Result<Prepared> {
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
    let gap = samples
        .iter()
        .map(|p| distance(spec.shape, *p).ok_or_else(|| KernelError::Argument("the shape cannot be merged".into())))
        .try_fold(0.0_f64, |m, d| d.map(|d| m.max(d)))?;
    if gap > max_gap {
        return Err(KernelError::Argument(format!(
            "the group's boundary is {gap:.4} mm from the surface (limit {max_gap})"
        )));
    }
    let placed = place(spec.shape, &samples, outward)?;
    let loops = face_loops(body, &b, &placed)?;
    let face = Face { surface: placed.surface, nurbs: placed.nurbs, domain: placed.domain, loops };
    let area = spec.faces.iter().filter_map(|&f| face_area(body, f, AREA_GRID)).sum();
    let edge_list = b.loops.iter().flatten().copied().collect();
    Ok(Prepared { faces: spec.faces.to_vec(), edge_list, area, face, forward: !placed.flip, gap, shell: b.shell })
}

/// Gives the boundary edges the tolerance of the gap (the kernel's mesher accepts an edge no farther
/// from its face than 10 chords plus its own tolerance). Their end vertices keep theirs: a vertex
/// tolerance as large as the gap makes the mesher lose area from the face (measured).
fn loosen(rec: &mut Brep, p: &Prepared) {
    for le in &p.edge_list {
        if let Some(e) = rec.edges.get_mut(le.edge as usize) {
            e.tolerance = e.tolerance.max(EDGE_TOLERANCE_PER_GAP * p.gap + 1e-6);
        }
    }
}

/// The record with every prepared group replaced by its face (not yet rebuilt by the kernel), and
/// the face of the new record each face of `body` became.
fn assembled(body: &Brep, groups: &[&Prepared]) -> (Brep, Vec<Option<usize>>) {
    let mut first_of: BTreeMap<usize, &Prepared> = BTreeMap::new();
    let mut gone: BTreeSet<usize> = BTreeSet::new();
    for p in groups {
        let first = p.faces.iter().copied().min().unwrap_or(0);
        first_of.insert(first, p);
        gone.extend(p.faces.iter().copied().filter(|&f| f != first));
    }
    let mut map = vec![None; body.faces.len()];
    let mut faces = Vec::with_capacity(body.faces.len());
    for (i, f) in body.faces.iter().enumerate() {
        if gone.contains(&i) {
            continue;
        }
        faces.push(first_of.get(&i).map_or_else(|| f.clone(), |p| p.face.clone()));
        if let Some(slot) = map.get_mut(i) {
            *slot = Some(faces.len() - 1);
        }
    }
    let mut out = body.clone();
    out.faces = faces;
    for (si, s) in out.shells.iter_mut().enumerate() {
        let Shell::Faces { faces, .. } = s else { continue };
        *faces = faces
            .iter()
            .filter_map(|o| {
                let merged = first_of.get(&(o.face as usize)).filter(|p| p.shell == si);
                let forward = merged.map_or(o.forward, |p| p.forward);
                map.get(o.face as usize).copied().flatten().map(|n| OrientedFace { face: n as u32, forward })
            })
            .collect();
    }
    for p in groups {
        loosen(&mut out, p);
    }
    (canonicalize(out), map)
}

/// The record as the kernel builds it, taken as it is (the healer is not run: the record is the
/// input body with a topological edit that is consistent by construction, and the healer may
/// renumber what it is told is in order), and read back.
fn rebuild(rec: &Brep) -> Result<(Brep, Option<f64>)> {
    let (body, _) = solidcore_ops::Body::from_record(rec, true).map_err(|e| crate::op_err("from_record", e))?;
    let volume = is_closed(rec).then(|| body.volume(VOLUME_ACCURACY)).and_then(std::result::Result::ok);
    Ok((encode(&body)?, volume))
}

/// The volume of a closed body as the kernel builds it from the record as it is (no healer).
fn trusted_volume(rec: &Brep) -> Result<f64> {
    let (body, _) = solidcore_ops::Body::from_record(rec, true).map_err(|e| crate::op_err("from_record", e))?;
    body.volume(VOLUME_ACCURACY).map_err(|e| crate::op_err("volume", e))
}

/// The rebuilt body must have the faces the edit made, the class (closed or open) of the input and,
/// when closed, nearly its volume `before_volume`.
fn verify(before: &Brep, before_volume: Option<f64>, after: (&Brep, Option<f64>), removed: usize) -> Result<()> {
    let (after, after_volume) = after;
    if after.faces.len() + removed != before.faces.len() {
        return Err(KernelError::Argument("the kernel changed the faces of the merged body".into()));
    }
    if is_closed(before) != is_closed(after) {
        return Err(KernelError::Argument("the merge changed the body from closed to open or back".into()));
    }
    if let Some(v0) = before_volume.filter(|_| is_closed(after)) {
        let v1 = after_volume.ok_or_else(|| KernelError::Argument("the merged body has no volume".into()))?;
        if !(v1 > 0.0 && (v1 - v0).abs() <= MAX_VOLUME_CHANGE * v0) {
            return Err(KernelError::Argument(format!("the merge changed the volume from {v0} to {v1}")));
        }
    }
    Ok(())
}

/// Assembles, rebuilds and verifies the groups; the body and its face map.
fn apply(body: &Brep, volume: Option<f64>, groups: &[&Prepared]) -> Result<(Brep, Vec<Option<usize>>)> {
    let (rec, map) = assembled(body, groups);
    let (rebuilt, volume_after) = rebuild(&rec)?;
    let removed: usize = groups.iter().map(|p| p.faces.len().saturating_sub(1)).sum();
    verify(body, volume, (&rebuilt, volume_after), removed)?;
    Ok((rebuilt, map))
}

/// One face of `body` as a sheet of its own (its edges and vertices, in canonical order).
fn face_sheet(body: &Brep, face: usize) -> Result<Brep> {
    let f = body.faces.get(face).ok_or_else(|| KernelError::Argument("face index".into()))?.clone();
    let sheet = Brep {
        version: body.version,
        tolerance: body.tolerance,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: vec![OrientedFace { face: 0, forward: true }], wires: vec![] }],
        faces: vec![f],
        edges: body.edges.clone(),
        vertices: body.vertices.clone(),
    };
    Ok(canonicalize(sheet))
}

/// For each prepared group whether its new face (`map` gives its place in `brep`) tessellates, alone
/// and at the display chord and at `chord`, to the area of the patches it replaced.
fn areas_agree(brep: &Brep, map: &[Option<usize>], groups: &[&Prepared], chord: f64) -> Result<Vec<bool>> {
    let mut ok = Vec::with_capacity(groups.len());
    for p in groups {
        let first = p.faces.iter().copied().min().unwrap_or(0);
        let Some(face) = map.get(first).copied().flatten() else {
            ok.push(false);
            continue;
        };
        let sheet = face_sheet(brep, face)?;
        let mut good = true;
        for c in [None, Some(chord)] {
            let got = crate::tess::tessellate_topo(&sheet, c).map(|t| t.mesh.area()).unwrap_or(0.0);
            good &= (got - p.area).abs() <= AREA_AGREEMENT * p.area.max(1e-12);
        }
        ok.push(good);
    }
    Ok(ok)
}

/// Merges each group of `specs` into one face on its shape, all in one rebuild of the body where
/// the kernel takes that, else one group after the other (a group the kernel refuses stays patches
/// and says why). Groups must not share a face. Each group's boundary may leave its shape by
/// `max_gap` at most.
///
/// # Errors
/// An invalid `max_gap`, or no group that could be merged at all (see [`GroupResult`] per group
/// for the reasons, in the message of the first).
pub fn merge_faces_onto_shapes(body: &Brep, specs: &[GroupSpec], max_gap: f64) -> Result<Batch> {
    if !(max_gap.is_finite() && max_gap > 0.0 && max_gap <= MAX_GAP) {
        return Err(KernelError::Argument(format!("the gap must be in (0, {MAX_GAP}]")));
    }
    let mut claimed: BTreeSet<usize> = BTreeSet::new();
    let mut prepared: Vec<std::result::Result<Prepared, String>> = Vec::with_capacity(specs.len());
    for spec in specs {
        let overlaps = spec.faces.iter().any(|f| claimed.contains(f));
        let p = if overlaps {
            Err("a face belongs to another group".to_string())
        } else {
            prepare(body, spec, max_gap).map_err(|e| e.to_string())
        };
        if p.is_ok() {
            claimed.extend(spec.faces.iter().copied());
        }
        prepared.push(p);
    }
    let volume = is_closed(body).then(|| trusted_volume(body)).transpose()?;
    let all: Vec<&Prepared> = prepared.iter().filter_map(|p| p.as_ref().ok()).collect();
    if all.is_empty() {
        let why = prepared.iter().find_map(|p| p.as_ref().err()).cloned().unwrap_or_default();
        return Err(KernelError::Argument(format!("no group could be merged: {why}")));
    }
    let (brep, face_map, accepted) = settle(body, volume, &all, 2.0 * max_gap)?;
    let mut accepted = accepted.into_iter();
    let groups = specs
        .iter()
        .zip(&prepared)
        .map(|(spec, p)| match p {
            Err(why) => Err(why.clone()),
            Ok(p) if accepted.next() == Some(true) => {
                let first = spec.faces.iter().copied().min().unwrap_or(0);
                let face = face_map.get(first).copied().flatten().ok_or("face map")?;
                Ok(GroupResult { face, gap: p.gap })
            }
            Ok(_) => Err("the kernel did not take the merged group".to_string()),
        })
        .collect();
    Ok(Batch { brep, face_map, groups })
}

/// All groups in one rebuild; the groups whose face does not hold its area are dropped and the
/// rest rebuilt; a rebuild the kernel refuses goes group by group.
fn settle(
    body: &Brep,
    volume: Option<f64>,
    all: &[&Prepared],
    chord: f64,
) -> Result<(Brep, Vec<Option<usize>>, Vec<bool>)> {
    let Ok((brep, map)) = apply(body, volume, all) else { return one_by_one(body, volume, all, chord) };
    let ok = areas_agree(&brep, &map, all, chord)?;
    if ok.iter().all(|&k| k) {
        return Ok((brep, map, ok));
    }
    let kept: Vec<&Prepared> = all.iter().zip(&ok).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
    if kept.is_empty() {
        return Err(KernelError::Argument("no merged face holds the area of its patches".into()));
    }
    let (brep, map) = apply(body, volume, &kept)?;
    if !areas_agree(&brep, &map, &kept, chord)?.iter().all(|&k| k) {
        return one_by_one(body, volume, &kept, chord).map(|(b, m, f)| (b, m, expand(&ok, &f)));
    }
    Ok((brep, map, ok))
}

/// `inner` (flags of the kept groups) spread over all the groups, `outer` flagging the kept ones.
fn expand(outer: &[bool], inner: &[bool]) -> Vec<bool> {
    let mut it = inner.iter();
    outer.iter().map(|&k| k && it.next().copied().unwrap_or(false)).collect()
}

/// The groups one after the other, each kept when the kernel takes it with the ones before.
fn one_by_one(
    body: &Brep,
    volume: Option<f64>,
    all: &[&Prepared],
    chord: f64,
) -> Result<(Brep, Vec<Option<usize>>, Vec<bool>)> {
    let mut kept: Vec<&Prepared> = Vec::new();
    let mut flags = Vec::with_capacity(all.len());
    for p in all {
        kept.push(p);
        let ok = apply(body, volume, &kept)
            .and_then(|(b, m)| areas_agree(&b, &m, &kept, chord))
            .is_ok_and(|f| f.iter().all(|&k| k));
        if !ok {
            kept.pop();
        }
        flags.push(ok);
    }
    if kept.is_empty() {
        return Err(KernelError::Argument("the kernel took no group".into()));
    }
    let (brep, map) = apply(body, volume, &kept)?;
    Ok((brep, map, flags))
}

/// Merges the faces `group` of `body` into one face on `shape`, within `max_gap` of it.
///
/// # Errors
/// A group that is not one connected set of patches of a face shell, a shape the merge does not
/// place or whose surface the boundary leaves by more than `max_gap`, a face that would wrap round
/// its surface's axis, and a result the kernel does not take or whose volume differs from the
/// body's by more than the gap allows.
pub fn merge_faces_onto_shape(body: &Brep, group: &[usize], shape: &Shape, max_gap: f64) -> Result<Merged> {
    let batch = merge_faces_onto_shapes(body, &[GroupSpec { faces: group, shape }], max_gap)?;
    match batch.groups.first() {
        Some(Ok(g)) => Ok(Merged { brep: batch.brep, face_map: batch.face_map, face: g.face, gap: g.gap }),
        Some(Err(why)) => Err(KernelError::Argument(why.clone())),
        None => Err(KernelError::Argument("no group".into())),
    }
}
