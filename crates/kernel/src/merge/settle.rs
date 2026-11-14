//! Putting prepared groups into the body: the edited record, the kernel's rebuild of it, and the
//! checks of each new face. A group that does not pass stays patches and says why.
use super::{Prepared, check};
use crate::nurbs::{canonicalize, is_closed};
use crate::record::brep::{Brep, OrientedFace, Region, Shell};
use crate::solid::encode;
use crate::{KernelError, Result};
use std::collections::{BTreeMap, BTreeSet};

/// The share of its volume a merge may change a closed body by (patches that lie on the primitive
/// within the gap change it far less; this catches an inside-out or broken result).
const MAX_VOLUME_CHANGE: f64 = 0.02;
/// Relative accuracy of the volumes that are compared.
const VOLUME_ACCURACY: f64 = 1e-4;
/// The tolerance of a boundary edge as a multiple of its gap.
const EDGE_TOLERANCE_PER_GAP: f64 = 1.1;
/// A merged face must tessellate to its patches' area within this share (a face the mesher cannot
/// triangulate, such as one with a neck narrower than the chord, loses area). Measured on the gates and
/// on real scans: the faces that are right differ from their patches' area by at most 0.047 (at the
/// display chord 0.041, at the tolerance's 0.047; on scans 1 and 4 at most 0.027 and 0.013), the faces
/// the mesher loses area from by 0.84 on the gates and 2.0 to 6.0 on the scans; 0.1 is 2.1 times the
/// worst right one.
const AREA_AGREEMENT: f64 = 0.1;

/// Per group: `Ok` when its face was taken, else why not.
pub(super) type Verdicts = Vec<std::result::Result<(), String>>;

/// The body, the face each face of the input became, and the verdict on each group.
pub(super) type Settled = (Brep, Vec<Option<usize>>, Verdicts);

/// How the groups are rebuilt: whether to stop, whether to try all at once first, and the chord of
/// the second tessellation of each face.
pub(super) struct Run<'a> {
    pub cancel: &'a dyn Fn() -> bool,
    pub all_at_once: bool,
    pub chord: f64,
}

fn cancelled() -> KernelError {
    KernelError::Argument("cancelled".into())
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
pub(super) fn trusted_volume(rec: &Brep) -> Result<f64> {
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

/// The verdict on the new face of one group (`face` in `brep`): it tessellates, alone and at the
/// display chord and at `chord`, to the area of the patches it replaced, and the display tessellation
/// passes the direct checks against them.
fn face_verdict(
    body: &Brep,
    brep: &Brep,
    face: usize,
    p: &Prepared,
    chord: f64,
) -> Result<std::result::Result<(), String>> {
    let sheet = face_sheet(brep, face)?;
    for c in [None, Some(chord)] {
        let at = c.map_or_else(|| "display chord".to_string(), |c| format!("chord {c:.3}"));
        let Ok(topo) = crate::tess::tessellate_topo(&sheet, c) else {
            return Ok(Err(format!("the mesher cannot triangulate the new face at the {at}")));
        };
        let off = (topo.mesh.area() - p.area).abs() / p.area.max(1e-12);
        if off > AREA_AGREEMENT {
            return Ok(Err(format!(
                "the new face tessellates to an area {off:.3} (share) off its patches' at the {at}"
            )));
        }
        if c.is_none()
            && let Err(e) = check::check_face(&topo.mesh, p.forward, &p.cloud, body, p.limit)
        {
            return Ok(Err(e.to_string()));
        }
    }
    Ok(Ok(()))
}

/// For each prepared group (its new face is in `brep` as `map` says) the verdict of [`face_verdict`].
fn areas_agree(body: &Brep, brep: &Brep, map: &[Option<usize>], groups: &[&Prepared], chord: f64) -> Result<Verdicts> {
    let mut out = Vec::with_capacity(groups.len());
    for p in groups {
        let first = p.faces.iter().copied().min().unwrap_or(0);
        out.push(match map.get(first).copied().flatten() {
            Some(face) => face_verdict(body, brep, face, p, chord)?,
            None => Err("the new face is missing from the rebuilt body".to_string()),
        });
    }
    Ok(out)
}

fn passed(v: &Verdicts) -> bool {
    v.iter().all(std::result::Result::is_ok)
}

/// The groups of `all` whose verdict in `v` is `Ok`.
fn kept_of<'a>(all: &[&'a Prepared], v: &Verdicts) -> Vec<&'a Prepared> {
    all.iter().zip(v).filter(|(_, k)| k.is_ok()).map(|(p, _)| *p).collect()
}

/// All groups in one rebuild; the groups whose face does not hold its area are dropped and the
/// rest rebuilt; a rebuild the kernel refuses goes group by group.
pub(super) fn settle(body: &Brep, volume: Option<f64>, all: &[&Prepared], run: &Run) -> Result<Settled> {
    if !run.all_at_once {
        return one_by_one(body, volume, all, run);
    }
    let Ok((brep, map)) = apply(body, volume, all) else { return one_by_one(body, volume, all, run) };
    let verdicts = areas_agree(body, &brep, &map, all, run.chord)?;
    if passed(&verdicts) {
        return Ok((brep, map, verdicts));
    }
    let kept = kept_of(all, &verdicts);
    if kept.is_empty() {
        let why = verdicts.iter().find_map(|v| v.as_ref().err()).cloned().unwrap_or_default();
        return Err(KernelError::Argument(format!("no merged face holds up: {why}")));
    }
    let (brep, map) = apply(body, volume, &kept)?;
    if !passed(&areas_agree(body, &brep, &map, &kept, run.chord)?) {
        return one_by_one(body, volume, &kept, run).map(|(b, m, f)| (b, m, expand(&verdicts, f)));
    }
    Ok((brep, map, verdicts))
}

/// `inner` (verdicts of the kept groups) spread over all the groups; `outer` has the dropped ones' reasons.
fn expand(outer: &Verdicts, inner: Verdicts) -> Verdicts {
    let mut it = inner.into_iter();
    outer
        .iter()
        .map(|o| match o {
            Err(why) => Err(why.clone()),
            Ok(()) => it.next().unwrap_or_else(|| Err("not rebuilt".to_string())),
        })
        .collect()
}

/// The verdict on one group rebuilt on its own: an error of the kernel is its reason.
fn alone(body: &Brep, volume: Option<f64>, p: &Prepared, run: &Run) -> std::result::Result<(), String> {
    let one = [p];
    let (b, m) = apply(body, volume, &one).map_err(|e| format!("the kernel did not take the rebuilt body: {e}"))?;
    let verdict = areas_agree(body, &b, &m, &one, run.chord).map_err(|e| e.to_string())?;
    verdict.into_iter().next().unwrap_or_else(|| Err("no verdict".to_string()))
}

/// Each group on its own body (one rebuild and the checks of that one face), then the kept groups
/// together (one rebuild): a number of rebuilds linear in the groups. Groups whose faces the kernel
/// takes alone but not together are dropped from the end until it does.
fn one_by_one(body: &Brep, volume: Option<f64>, all: &[&Prepared], run: &Run) -> Result<Settled> {
    let mut verdicts: Verdicts = Vec::with_capacity(all.len());
    for p in all {
        if (run.cancel)() {
            return Err(cancelled());
        }
        verdicts.push(alone(body, volume, p, run));
    }
    let mut kept = kept_of(all, &verdicts);
    while !kept.is_empty() {
        if (run.cancel)() {
            return Err(cancelled());
        }
        if let Ok((brep, map)) = apply(body, volume, &kept)
            && areas_agree(body, &brep, &map, &kept, run.chord).is_ok_and(|v| passed(&v))
        {
            return Ok((brep, map, verdicts));
        }
        kept.pop();
        if let Some(last) = verdicts.iter_mut().rev().find(|v| v.is_ok()) {
            *last = Err("the kernel took it alone but not together with the other groups".to_string());
        }
    }
    let why = verdicts.iter().find_map(|v| v.as_ref().err()).cloned().unwrap_or_default();
    Err(KernelError::Argument(format!("the kernel took no group: {why}")))
}
