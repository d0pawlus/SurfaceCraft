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
/// A merged face must tessellate to its patches' area: not more than this share smaller (a face the mesher
/// cannot triangulate, such as one with a neck narrower than the chord, loses area, and the scan points over
/// the lost area are then measured against something else: a plane of 159 patches that lost 4.6 % of its
/// area at the display chord lowered the share of measured points of a scan by 4.25 points) ...
const MAX_AREA_LOSS: f64 = 0.025;
/// ... and not more than this share larger (the patches' area is read from a grid of cells, which is
/// always a little under that of a curved surface; measured at most 0.047 on cylinders and cones).
const MAX_AREA_EXCESS: f64 = 0.1;
/// The chord the judge tessellates the body at, as a share of the tolerance (half the chord of the area check).
const JUDGE_CHORD_SHARE: f64 = 0.05;

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
    if check::forward_of(brep, face) != p.forward {
        return Ok(Err(
            "the kernel changed the use of the new face (it was asked to use it against its natural normal)".into(),
        ));
    }
    let sheet = face_sheet(brep, face)?;
    // the display chord, the chord of the area check and the chord of the judge (a twentieth of the gap)
    for c in [None, Some(chord), Some(JUDGE_CHORD_SHARE * (chord / 2.0))] {
        let at = c.map_or_else(|| "display chord".to_string(), |c| format!("chord {c:.3}"));
        let Ok(topo) = crate::tess::tessellate_topo(&sheet, c) else {
            return Ok(Err(format!("the mesher cannot triangulate the new face at the {at}")));
        };
        let ratio = topo.mesh.area() / p.area.max(1e-12);
        if ratio < 1.0 - MAX_AREA_LOSS || ratio > 1.0 + MAX_AREA_EXCESS {
            return Ok(Err(format!("the new face tessellates to {ratio:.3} of its patches' area at the {at}")));
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::sheet_from_bezier_network;

    /// The finding behind the refusal of a face that looks against its natural normal in an open sheet: the
    /// kernel keeps the `same` use of a sheet's face, whatever the record said, so such a face comes back
    /// reversed against its neighbours. (In a closed shell the use is kept; see the gates of
    /// `tests/merge_faces.rs`.)
    #[test]
    fn the_kernel_gives_a_sheet_face_used_backward_back_used_forward() {
        let net = vec![vec![[0.0, 0.0, 0.0], [0.0, 1.0, 0.0]], vec![[1.0, 0.0, 0.0], [1.0, 1.0, 0.0]]];
        let mut sheet = sheet_from_bezier_network(&[net], &[[0, 1, 2, 3]]).unwrap();
        let uses = |b: &Brep| {
            b.shells
                .iter()
                .flat_map(|s| match s {
                    Shell::Faces { faces, .. } => faces.iter().map(|o| o.forward).collect::<Vec<_>>(),
                    _ => Vec::new(),
                })
                .collect::<Vec<_>>()
        };
        assert_eq!(uses(&sheet), vec![true]);
        if let Some(Shell::Faces { faces, .. }) = sheet.shells.first_mut() {
            faces.iter_mut().for_each(|o| o.forward = false);
        }
        let (back, _) = rebuild(&canonicalize(sheet)).unwrap();
        assert_eq!(uses(&back), vec![true], "the use of a sheet's face is the same one");
    }
}
