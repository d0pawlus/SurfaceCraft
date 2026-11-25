//! Putting prepared groups into the body: the edited record, the kernel's rebuild of it, and the
//! checks of each new face. A group that does not pass stays patches and says why.
use super::{Prepared, check};
use crate::nurbs::{canonicalize, is_closed};
use crate::record::brep::{Brep, OrientedFace, Region, Shell, Surface};
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
/// The least chord of the display tessellation of a new face as a multiple of how far its boundary stands off its
/// surface (see `display_chord_of`): the mesher's refinement reaches its cap at a chord of about half the offset and
/// finer (upstream draft 13: offset 0.1 mm at chord 0.05, offset 0.05 at chord 0.02), so twice the offset stays four
/// times clear of it.
const BOUNDARY_CHORD_PER_OFFSET: f64 = 2.0;

/// The most groups settled together: the groups are put into the body a batch at a time, each batch rebuilt with the
/// faces the batches before it gave. Where the kernel refuses a whole batch, its groups go one by one, and those it
/// takes alone but not together are dropped from the end, each drop a rebuild and the checks of the kept faces again:
/// the batch bounds that work (at most `2 BATCH + 2` rebuilds, and the checks of about `BATCH^2 / 2` faces) where a
/// single batch of all the groups would square it. With the judgement of all the faces in the final body
/// ([`MAX_REJUDGE_ROUNDS`] rounds of at most one rebuild and the checks of every taken face), one settle of `n` groups
/// costs at most `n / BATCH (2 BATCH + 2) + MAX_REJUDGE_ROUNDS` rebuilds and the checks of about
/// `n BATCH / 2 + MAX_REJUDGE_ROUNDS n` faces: linear in the groups.
pub(super) const BATCH: usize = 64;

/// The most rounds in which the faces of all the batches are judged in the final body (`rejudged`). Each round drops
/// the faces that do not hold up there and rebuilds the body without them; a body whose faces still do not all hold
/// up after the last round is not returned: the body of the first batch that gave faces is (its faces held up in it).
pub(super) const MAX_REJUDGE_ROUNDS: usize = 2;

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
pub(super) fn apply(body: &Brep, volume: Option<f64>, groups: &[&Prepared]) -> Result<(Brep, Vec<Option<usize>>)> {
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

/// Is the face an analytic one (a plane, cylinder, cone or sphere: what a merge makes)?
fn is_analytic(brep: &Brep, face: usize) -> bool {
    matches!(
        brep.faces.get(face).map(|f| &f.surface),
        Some(
            Surface::Plane { .. }
                | Surface::PlaneThrough { .. }
                | Surface::Cylinder { .. }
                | Surface::Cone { .. }
                | Surface::Sphere { .. }
        )
    )
}

/// The verdict on the new face of one group (`face` in `brep`): the kernel gave it back as an analytic face, it
/// tessellates, alone and at the display chord and at `chord`, to the area of the patches it replaced, the display
/// tessellation passes the direct checks against them, and its border is the group's.
///
/// The kernel's builder gives a plane or cylinder face as a rational spline and the rebuild of the body labels it
/// (it recognises the spline as its surface). Measured on a real scan, the recognition of one exact cylinder face (an
/// arc of 28 degrees) failed where the face was and succeeded once the face was moved by a micrometre, so the face came
/// back a spline. Such a face is no analytic face, and the group stays patches.
pub(super) fn face_verdict(
    body: &Brep,
    brep: &Brep,
    face: usize,
    p: &Prepared,
    chord: f64,
) -> Result<std::result::Result<(), String>> {
    if !is_analytic(brep, face) {
        return Ok(Err("the kernel gave the new face back as a spline, not as its analytic surface".into()));
    }
    if check::forward_of(brep, face) != p.forward {
        return Ok(Err(
            "the kernel changed the use of the new face (it was asked to use it against its natural normal)".into(),
        ));
    }
    let sheet = face_sheet(brep, face)?;
    // the display chord (no finer than the face's boundary allows), the chord of the area check and the chord of the
    // judge (a twentieth of the gap)
    let judge_chord = JUDGE_CHORD_SHARE * (chord / 2.0);
    let display = display_chord_of(&sheet, p.cloud.border_offset(&p.gauge));
    for (c, normals, border) in [(display, true, false), (Some(chord), false, false), (Some(judge_chord), false, true)]
    {
        let at = c.map_or_else(|| "display chord".to_string(), |c| format!("chord {c:.3}"));
        let Ok(topo) = crate::tess::tessellate_topo(&sheet, c) else {
            return Ok(Err(format!("the mesher cannot triangulate the new face at the {at}")));
        };
        let ratio = topo.mesh.area() / p.area.max(1e-12);
        if ratio < 1.0 - MAX_AREA_LOSS || ratio > 1.0 + MAX_AREA_EXCESS {
            return Ok(Err(format!("the new face tessellates to {ratio:.3} of its patches' area at the {at}")));
        }
        if normals && let Err(e) = check::check_face(&topo.mesh, p.forward, &p.cloud, (body, &p.gauge), p.limit) {
            return Ok(Err(e.to_string()));
        }
        // the border is compared with the group's on the finest tessellation, whose chord is the stated slack
        if border && let Err(e) = check::check_border(&p.cloud, &topo.mesh, p.limit, judge_chord) {
            return Ok(Err(e.to_string()));
        }
    }
    Ok(Ok(()))
}

/// The chord of the display tessellation of a new face (`sheet`), on which its area and normals are checked: the display
/// chord (`None`), or, where the face's boundary stands `offset` (mm) off its surface by more than half of that,
/// [`BOUNDARY_CHORD_PER_OFFSET`] times the offset. The rule is the same for every face, snapped or not; in practice it
/// coarsens the faces of groups whose boundary could not be snapped. Such a group keeps the wrap's edges, up
/// to the gap off the primitive, and the kernel's mesher refines such a face to its cap of triangles at any chord finer
/// than about the offset (tangency-labs/solidcraft#17431, `docs/upstream/13-tess-refine-cap-offset-boundary.md`:
/// measured 40 093 to 40 816 triangles at the display chord for faces that give 242 at a chord of 0.4 mm, a wrong area,
/// and slivers that leave the normal check a handful of usable probes). A mesh cannot follow the surface more closely
/// than its boundary does, so the coarser chord hides nothing the checks could see. A snapped boundary (measured 1.1
/// micrometres off) keeps the display chord (a thousandth of the face's size) on every face larger than about 2 mm.
fn display_chord_of(sheet: &Brep, offset: f64) -> Option<f64> {
    let floor = BOUNDARY_CHORD_PER_OFFSET * offset;
    (floor.is_finite() && floor > crate::tess::display_chord(sheet)).then_some(floor)
}

/// For each prepared group (its new face is in `brep` as `map` says) the verdict of [`face_verdict`]. `run.cancel`
/// is polled before each face. An error is the merge's own (a face it cannot read), not a refusal by the kernel.
fn areas_agree(body: &Brep, brep: &Brep, map: &[Option<usize>], groups: &[&Prepared], run: &Run) -> Result<Verdicts> {
    let mut out = Vec::with_capacity(groups.len());
    for p in groups {
        if (run.cancel)() {
            return Err(cancelled());
        }
        let first = p.faces.iter().copied().min().unwrap_or(0);
        out.push(match map.get(first).copied().flatten() {
            Some(face) => face_verdict(body, brep, face, p, run.chord).map_err(|e| {
                KernelError::Argument(format!("an internal error of the merge while judging a new face: {e}"))
            })?,
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

/// The groups of `all`, a [`BATCH`] at a time, each batch rebuilt with the groups taken before it (`settle_batch`).
/// A batch the kernel takes none of leaves its groups as patches with the reason and the next batch goes on.
/// The faces of every batch are judged again in the body that is returned (`rejudged`) when more than one batch gave
/// faces: a later rebuild could change a face taken before it.
pub(super) fn settle(body: &Brep, volume: Option<f64>, all: &[&Prepared], run: &Run) -> Result<Settled> {
    let mut taken = Taken { groups: Vec::with_capacity(all.len()), at: Vec::with_capacity(all.len()) };
    let mut verdicts: Verdicts = Vec::with_capacity(all.len());
    let mut last: Option<(Brep, Vec<Option<usize>>)> = None;
    let mut first_refusal: Option<KernelError> = None;
    let mut batches_with_faces = 0_usize;
    let mut first: Option<Verified> = None;
    for (b, batch) in all.chunks(BATCH).enumerate() {
        if (run.cancel)() {
            return Err(cancelled());
        }
        match settle_batch(body, volume, &taken.groups, batch, run) {
            Ok((brep, map, v)) => {
                for (j, (p, verdict)) in batch.iter().zip(&v).enumerate() {
                    if verdict.is_ok() {
                        taken.groups.push(*p);
                        taken.at.push(b * BATCH + j);
                    }
                }
                batches_with_faces += 1;
                verdicts.extend(v);
                if first.is_none() {
                    first = Some(Verified { brep: brep.clone(), map: map.clone(), at: taken.at.clone() });
                }
                last = Some((brep, map));
            }
            Err(_) if (run.cancel)() => return Err(cancelled()),
            Err(e) => {
                let why = e.to_string();
                verdicts.extend(batch.iter().map(|_| Err(why.clone())));
                first_refusal.get_or_insert(e);
            }
        }
    }
    match (last, first) {
        (Some(rebuilt), Some(fallback)) if batches_with_faces > 1 => {
            rejudged(body, volume, (taken, verdicts), rebuilt, fallback, (MAX_REJUDGE_ROUNDS, run))
        }
        (Some((brep, map)), _) => Ok((brep, map, verdicts)),
        (None, _) => Err(first_refusal.unwrap_or_else(|| KernelError::Argument("no group to merge".into()))),
    }
}

/// The groups taken so far and, for each, its index among all the groups.
pub(super) struct Taken<'a> {
    pub groups: Vec<&'a Prepared>,
    pub at: Vec<usize>,
}

/// A body whose new faces all held up in it, its face map and the indices of its groups among all the groups.
pub(super) struct Verified {
    pub brep: Brep,
    pub map: Vec<Option<usize>>,
    pub at: Vec<usize>,
}

/// `fallback` as the outcome: its groups taken, every other group that was taken given back with the reason.
fn fall_back(fallback: Verified, mut verdicts: Verdicts, why: &str) -> Settled {
    for (i, v) in verdicts.iter_mut().enumerate() {
        if fallback.at.contains(&i) {
            *v = Ok(());
        } else if v.is_ok() {
            *v = Err(format!("its batch was taken back: {why}"));
        }
    }
    (fallback.brep, fallback.map, verdicts)
}

/// The faces of all the batches judged again in the final body `rebuilt`: a face that no longer holds up there is
/// dropped with the reason and the others rebuilt, for at most `rounds` rounds (see [`MAX_REJUDGE_ROUNDS`]). The body
/// is returned once every kept face holds up in it; when that is not so within the rounds, or the kernel does not
/// rebuild the body without the dropped faces, `fallback` (the first batch's body, whose faces held up) is returned and
/// the groups of the later batches are given back with the reason.
pub(super) fn rejudged(
    body: &Brep,
    volume: Option<f64>,
    state: (Taken<'_>, Verdicts),
    rebuilt: (Brep, Vec<Option<usize>>),
    fallback: Verified,
    limits: (usize, &Run),
) -> Result<Settled> {
    let (rounds, run) = limits;
    let (mut taken, mut verdicts) = state;
    let (mut brep, mut map) = rebuilt;
    for round in 0..rounds {
        let v = areas_agree(body, &brep, &map, &taken.groups, run)?;
        if passed(&v) {
            return Ok((brep, map, verdicts));
        }
        let mut kept = Taken { groups: Vec::with_capacity(taken.groups.len()), at: Vec::with_capacity(taken.at.len()) };
        for ((p, &at), verdict) in taken.groups.iter().zip(&taken.at).zip(v) {
            match verdict {
                Ok(()) => {
                    kept.groups.push(*p);
                    kept.at.push(at);
                }
                Err(why) => {
                    if let Some(slot) = verdicts.get_mut(at) {
                        *slot = Err(format!("{why} (in the body with the faces of the later batches)"));
                    }
                }
            }
        }
        if kept.groups.is_empty() {
            return Ok(fall_back(fallback, verdicts, "no face held up in the body with all the batches"));
        }
        if round + 1 == rounds {
            // the body rebuilt without them would not be judged again
            break;
        }
        match apply(body, volume, &kept.groups) {
            Ok(r) => (brep, map) = r,
            Err(e) => {
                let why = format!("the kernel did not rebuild the body without the faces that did not hold up: {e}");
                return Ok(fall_back(fallback, verdicts, &why));
            }
        }
        taken = kept;
    }
    let plural = if rounds == 1 { "" } else { "s" };
    let why = format!("the faces of the batches did not all hold up together within {rounds} round{plural}");
    Ok(fall_back(fallback, verdicts, &why))
}

/// The groups taken before (`taken`) and `batch` as one list for a rebuild.
fn with<'a>(taken: &[&'a Prepared], batch: &[&'a Prepared]) -> Vec<&'a Prepared> {
    taken.iter().chain(batch).copied().collect()
}

/// One batch in one rebuild with the groups taken before it; the groups whose face does not hold its area are
/// dropped and the rest rebuilt; a rebuild the kernel refuses goes group by group. The verdicts are the batch's.
fn settle_batch(
    body: &Brep,
    volume: Option<f64>,
    taken: &[&Prepared],
    batch: &[&Prepared],
    run: &Run,
) -> Result<Settled> {
    if !run.all_at_once {
        return one_by_one(body, volume, taken, batch, run);
    }
    let Ok((brep, map)) = apply(body, volume, &with(taken, batch)) else {
        return one_by_one(body, volume, taken, batch, run);
    };
    let verdicts = areas_agree(body, &brep, &map, batch, run)?;
    if passed(&verdicts) {
        return Ok((brep, map, verdicts));
    }
    let kept = kept_of(batch, &verdicts);
    if kept.is_empty() {
        let why = verdicts.iter().find_map(|v| v.as_ref().err()).cloned().unwrap_or_default();
        return Err(KernelError::Argument(format!("no merged face holds up: {why}")));
    }
    let (brep, map) = apply(body, volume, &with(taken, &kept))?;
    if !passed(&areas_agree(body, &brep, &map, &kept, run)?) {
        return one_by_one(body, volume, taken, &kept, run).map(|(b, m, f)| (b, m, expand(&verdicts, f)));
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

/// The verdict on one group rebuilt with the groups taken before it: an error of the kernel is its reason.
fn alone(
    body: &Brep,
    volume: Option<f64>,
    taken: &[&Prepared],
    p: &Prepared,
    run: &Run,
) -> std::result::Result<(), String> {
    let one = [p];
    let (b, m) = apply(body, volume, &with(taken, &one))
        .map_err(|e| format!("the kernel did not take the rebuilt body: {e}"))?;
    let verdict = areas_agree(body, &b, &m, &one, run).map_err(|e| e.to_string())?;
    verdict.into_iter().next().unwrap_or_else(|| Err("no verdict".to_string()))
}

/// Each group of `batch` on its own with the groups taken before it (one rebuild and the checks of that one face),
/// then the kept groups together (one rebuild): a number of rebuilds linear in the batch. Groups whose faces the
/// kernel takes alone but not together are dropped from the end until it does.
fn one_by_one(
    body: &Brep,
    volume: Option<f64>,
    taken: &[&Prepared],
    batch: &[&Prepared],
    run: &Run,
) -> Result<Settled> {
    let mut verdicts: Verdicts = Vec::with_capacity(batch.len());
    for p in batch {
        if (run.cancel)() {
            return Err(cancelled());
        }
        verdicts.push(alone(body, volume, taken, p, run));
    }
    let mut kept = kept_of(batch, &verdicts);
    while !kept.is_empty() {
        if (run.cancel)() {
            return Err(cancelled());
        }
        if let Ok((brep, map)) = apply(body, volume, &with(taken, &kept))
            && areas_agree(body, &brep, &map, &kept, run).is_ok_and(|v| passed(&v))
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

    /// The display tessellation of a new face is no finer than twice the offset of its boundary: a snapped boundary
    /// (a micrometre off) keeps the display chord (`None`, about 0.014 mm for this 10 mm sheet), one 0.1 mm off is
    /// meshed at 0.2 mm, and an offset that is not finite keeps the display chord.
    #[test]
    fn the_display_chord_of_a_new_face_is_no_finer_than_twice_its_boundary_offset() {
        let net = vec![vec![[0.0, 0.0, 0.0], [0.0, 10.0, 0.0]], vec![[10.0, 0.0, 0.0], [10.0, 10.0, 0.0]]];
        let sheet = sheet_from_bezier_network(&[net], &[[0, 1, 2, 3]]).unwrap();
        assert!(crate::tess::display_chord(&sheet) > 0.01 && crate::tess::display_chord(&sheet) < 0.02);
        assert_eq!(display_chord_of(&sheet, 1e-6), None, "a snapped face keeps the display chord");
        assert_eq!(display_chord_of(&sheet, 1e-3), None, "2e-3 is finer than the display chord");
        assert_eq!(display_chord_of(&sheet, 0.1), Some(0.2));
        assert_eq!(display_chord_of(&sheet, f64::NAN), None);
        assert_eq!(display_chord_of(&sheet, f64::INFINITY), None);
    }

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

    /// A record whose faces are all used forward (what every caller of `canonicalize` but the merge builds) is
    /// canonical already, and `canonicalize` leaves it as it is: the walk of a face used backward is the only
    /// thing it does differently, and no such face is there.
    #[test]
    fn a_record_with_forward_faces_only_is_unchanged_by_canonicalize() {
        let net = |x: f64| vec![vec![[x, 0.0, 0.0], [x, 1.0, 0.0]], vec![[x + 1.0, 0.0, 0.0], [x + 1.0, 1.0, 0.0]]];
        let b = sheet_from_bezier_network(&[net(0.0), net(1.0)], &[[0, 1, 2, 3], [1, 4, 5, 2]]).unwrap();
        assert!(b.shells.iter().all(|s| match s {
            Shell::Faces { faces, .. } => faces.iter().all(|f| f.forward),
            _ => true,
        }));
        b.validate().unwrap();
        assert_eq!(canonicalize(b.clone()), b);
    }

    /// A unit cube of six bilinear patches (a closed, canonical record, every face used forward).
    fn cube() -> Brep {
        let v = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.0, 1.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 1.0],
            [1.0, 1.0, 1.0],
            [0.0, 1.0, 1.0],
        ];
        let corners = [[0, 1, 2, 3], [4, 5, 6, 7], [0, 1, 5, 4], [1, 2, 6, 5], [2, 3, 7, 6], [3, 0, 4, 7]];
        let nets: Vec<Vec<Vec<[f64; 3]>>> =
            corners.iter().map(|c| vec![vec![v[c[0]], v[c[3]]], vec![v[c[1]], v[c[2]]]]).collect();
        let ids: Vec<[u32; 4]> = corners.iter().map(|c| c.map(|i| i as u32)).collect();
        sheet_from_bezier_network(&nets, &ids).unwrap()
    }

    /// `face` of `b` as its other side: the surface's net transposed (its natural normal now points the other
    /// way), the loops walked the other way round and the pcurves' parameters swapped, and the shell using it
    /// backward. The body is the same body.
    fn face_as_used_backward(b: &Brep, face: usize) -> Brep {
        use crate::record::brep::{Curve2, Loop, Surface};
        let mut out = b.clone();
        let f = &mut out.faces[face];
        if let Surface::Spline { points, u_knots, v_knots } = &mut f.surface {
            let t: Vec<Vec<[f64; 3]>> = (0..points[0].len()).map(|j| points.iter().map(|r| r[j]).collect()).collect();
            *points = t;
            std::mem::swap(u_knots, v_knots);
        }
        for l in &mut f.loops {
            if let Loop::Edges { edges } = l {
                edges.reverse();
                for e in edges.iter_mut() {
                    e.forward = !e.forward;
                    if let Curve2::Spline { points, .. } = &mut e.pcurve {
                        points.iter_mut().for_each(|q| q.swap(0, 1));
                    }
                }
            }
        }
        f.domain = [f.domain[1], f.domain[0]];
        for s in &mut out.shells {
            if let Shell::Faces { faces, .. } = s {
                faces.iter_mut().filter(|o| o.face as usize == face).for_each(|o| o.forward = false);
            }
        }
        out
    }

    fn signed_volume(m: &surfacecraft_geom::Mesh) -> f64 {
        (0..m.triangle_count())
            .filter_map(|t| m.triangle(t))
            .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
            .sum()
    }

    /// `canonicalize` of a record with a face used backward: the record is valid and canonical, the kernel
    /// builds the unit cube from it (volume 1, positive), every triangle of the tessellation runs outward and
    /// the STEP round trip keeps the volume.
    #[test]
    fn a_face_used_backward_is_walked_from_its_other_side() {
        let cube = cube();
        assert!(is_closed(&cube));
        for face in 0..6 {
            let backward = face_as_used_backward(&cube, face);
            let c = canonicalize(backward);
            c.validate().unwrap_or_else(|e| panic!("face {face}: {e:?}"));
            assert!(
                c.shells.iter().any(|s| matches!(s, Shell::Faces { faces, .. } if faces.iter().any(|o| !o.forward)))
            );
            let volume = crate::solid::volume(&c).unwrap();
            assert!((volume - 1.0).abs() < 1e-9, "face {face}: volume {volume}");
            let mesh = crate::tess::tessellate_all(&c, None).unwrap();
            assert!((signed_volume(&mesh) - 1.0).abs() < 1e-6, "face {face}: the triangles run outward");
            let text = crate::step::export_solids(&[("cube", &c)]).unwrap();
            let back = crate::step::import(text.as_bytes()).unwrap();
            assert!((crate::solid::volume(&back[0]).unwrap() - 1.0).abs() < 1e-6, "face {face}: STEP");
        }
    }

    /// A body with every face forward is canonical: `canonicalize` is the identity on it (the other callers).
    #[test]
    fn a_closed_record_with_forward_faces_only_is_unchanged_by_canonicalize() {
        let cube = cube();
        assert_eq!(canonicalize(cube.clone()), cube);
    }

    /// The other callers of `canonicalize` give the records they gave before the backward walk existed: the
    /// network of the cube above and its extracted faces are canonical, with the counts and the area they have.
    #[test]
    fn the_extracted_faces_of_a_cube_are_canonical_with_their_counts_and_area() {
        let cube = cube();
        assert_eq!((cube.faces.len(), cube.edges.len(), cube.vertices.len()), (6, 12, 8));
        for face in 0..6 {
            let sheet = crate::hybrid::extract_face(&cube, face).unwrap();
            assert_eq!((sheet.faces.len(), sheet.edges.len(), sheet.vertices.len()), (1, 4, 4), "face {face}");
            assert_eq!(canonicalize(sheet.clone()), sheet, "face {face}");
            let area = crate::tess::tessellate_all(&sheet, None).unwrap().area();
            assert!((area - 1.0).abs() < 1e-9, "face {face}: {area}");
        }
    }
}
