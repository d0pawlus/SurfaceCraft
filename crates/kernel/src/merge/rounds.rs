//! The rounds of a merge: the boundaries of the groups are put on their primitives (`snap`), the groups
//! are prepared and settled on the snapped body, and a group that was snapped but is then refused has its
//! snap taken back: the next round snaps only what is left, from the body as it was, so the neighbours of a
//! refused group are exactly as the wrap had them.
use super::geometry::{Group, group_of};
use super::peel::{MAX_PEELS, Off, Peeled, corners_of, far_patches, peel};
use super::place::{Gauge, place};
use super::settle::{Run, settle, trusted_volume};
use super::snap::{Snapped, Target, snap_boundaries};
use super::{Batch, GroupResult, GroupSpec, MAX_GROUPS, Prepared, prepare_all, unchanged};
use crate::nurbs::is_closed;
use crate::record::brep::Brep;
use crate::{KernelError, Result};
use std::collections::BTreeSet;

/// The farthest a vertex or an edge sample is moved onto its primitives, as a multiple of the gap the caller
/// allows. A point within the gap of each of two primitives that meet at a right angle lies up to
/// 1.414 gaps from their intersection; planes at a shallower angle ask for more and are refused.
const MOVE_PER_GAP: f64 = 1.5;
/// The most rounds: each takes back the snap of at least one refused group, so a merge ends in at most this
/// many. The last one snaps nothing (the groups are merged as they are, under the rules that apply to a boundary
/// that was not moved), which ends it whatever the earlier rounds refused.
const MAX_ROUNDS: usize = 4;

/// A group is snapped when its boundary is within this multiple of the gap of its primitive. The boundary samples the
/// caller checked (a grid of the patches) lie within the gap; the edges are sampled more densely and between the grid
/// points leave it by a little (measured on a plane of 1 342 patches: 0.2016 against the limit 0.2, 1.008 times), so the
/// group would be lost by rounding. The snap brings the boundary onto the primitive (and a group left unsnapped is
/// held to the gap itself), so the slack costs no accuracy; 1.25 leaves 30 times the measured excess.
const NEAR_SLACK: f64 = 1.25;

type Reasons = Vec<Option<String>>;

/// One group as `group_of` sees it, or why not, for each of the first [`MAX_GROUPS`] specs.
fn groups_in(body: &Brep, specs: &[GroupSpec]) -> Vec<std::result::Result<Group, String>> {
    specs.iter().take(MAX_GROUPS).map(|s| group_of(body, s.faces).map_err(|e| e.to_string())).collect()
}

/// The body with the boundaries snapped, which specs were, why the others were not, and the patches of the specs
/// whose border was peeled.
struct Snap {
    body: Brep,
    moved: Vec<bool>,
    why_not: Reasons,
    /// Per spec, its patches after the peels of this round (`None`: as the round was given them).
    faces: Vec<Option<Vec<usize>>>,
}

/// What the snap of a round reads.
struct SnapCtx<'a> {
    body: &'a Brep,
    specs: &'a [GroupSpec<'a>],
    before: &'a [std::result::Result<Group, String>],
    gauges: Vec<Option<Gauge>>,
    closed: bool,
    max_gap: f64,
    /// The groups the snap leaves out (their snap was taken back).
    refused: &'a Reasons,
    cancel: &'a dyn Fn() -> bool,
}

/// The patches and the group of spec `k`: as peeled in this round, else as given.
fn current<'a>(ctx: &'a SnapCtx, peeled: &'a [Option<Peeled>], k: usize) -> Option<(&'a [usize], &'a Group)> {
    if let Some(Some(p)) = peeled.get(k) {
        return Some((&p.faces, &p.group));
    }
    match (ctx.specs.get(k), ctx.before.get(k)) {
        (Some(spec), Some(Ok(group))) => Some((spec.faces, group)),
        _ => None,
    }
}

/// The groups the snap takes, with the spec of each: those within the slack of their primitive that can be placed
/// on it and are not left out; a face belongs to the first group that claims it.
fn admitted<'a>(ctx: &'a SnapCtx, peeled: &'a [Option<Peeled>]) -> (Vec<Target<'a>>, Vec<usize>) {
    let mut claimed: BTreeSet<usize> = BTreeSet::new();
    let (mut targets, mut spec_of) = (Vec::new(), Vec::new());
    for (k, spec) in ctx.specs.iter().enumerate().take(MAX_GROUPS) {
        let (Some(Some(gauge)), Some((faces, group))) = (ctx.gauges.get(k), current(ctx, peeled, k)) else { continue };
        if ctx.refused.get(k).is_some_and(Option::is_some) || faces.iter().any(|f| claimed.contains(f)) {
            continue;
        }
        let near = group.gap(gauge).is_ok_and(|g| g <= NEAR_SLACK * ctx.max_gap);
        if near && place(spec.shape, &group.points(), group.outward).is_ok_and(|p| ctx.closed || !p.flip) {
            claimed.extend(faces.iter().copied());
            targets.push(Target { faces, group, gauge });
            spec_of.push(k);
        }
    }
    (targets, spec_of)
}

/// The peel of a group the snap did not take (`peel`): when it lies too far from its primitive to be merged as it
/// is, the patches at the corners of the neighbours that failed for it and those on its boundary edges that leave
/// the primitive by more than the gap come off. `None` when it can be merged as it is.
fn peel_of(
    ctx: &SnapCtx,
    target: &Target,
    folded: Option<&BTreeSet<usize>>,
) -> Option<std::result::Result<Peeled, String>> {
    if target.group.gap(target.gauge).is_ok_and(|g| g <= ctx.max_gap) {
        return None;
    }
    let off = Off {
        corners: corners_of(ctx.body, folded.into_iter().flatten().copied()),
        patches: far_patches(ctx.body, target.group, target.gauge, ctx.max_gap),
    };
    Some(peel(ctx.body, target.faces, &off, ctx.max_gap, target.group.boundary.loops.len()))
}

/// One snap of the groups admitted with their current patches, the spec of each target, and (when `peeling`) the
/// peel of each group that was not snapped, is too far to be merged as it is and whose peeling has not stopped.
fn snap_once(
    ctx: &SnapCtx,
    peeled: &[Option<Peeled>],
    stopped: &Reasons,
    peeling: bool,
) -> (Snapped, Vec<usize>, Vec<(usize, std::result::Result<Peeled, String>)>) {
    let (targets, spec_of) = admitted(ctx, peeled);
    let snapped = snap_boundaries(ctx.body, &targets, MOVE_PER_GAP * ctx.max_gap, ctx.cancel);
    let mut peels = Vec::new();
    for (t, (target, &k)) in targets.iter().zip(&spec_of).enumerate() {
        let open = peeling && !(ctx.cancel)() && stopped.get(k).is_some_and(Option::is_none);
        if open && !snapped.snapped.get(t).copied().unwrap_or(false) {
            peels.extend(peel_of(ctx, target, snapped.folded.get(t)).map(|p| (k, p)));
        }
    }
    (snapped, spec_of, peels)
}

/// The snap of a round as the round reads it: the flags and notes of the targets put back on their specs.
fn snap_of(ctx: &SnapCtx, snapped: Snapped, spec_of: &[usize], peeled: Vec<Option<Peeled>>, stopped: &Reasons) -> Snap {
    let n = ctx.specs.len();
    let Snapped { body, snapped: flags, why_not, .. } = snapped;
    let (mut moved, mut notes) = (vec![false; n], vec![None; n]);
    for (t, &k) in spec_of.iter().enumerate() {
        let (Some(m), Some(note)) = (moved.get_mut(k), notes.get_mut(k)) else { continue };
        *m = flags.get(t).copied().unwrap_or(false);
        *note = why_not.get(t).cloned().flatten();
        if let (false, Some(Some(stop))) = (*m, stopped.get(k)) {
            let further = if peeled.get(k).is_some_and(Option::is_some) { " further" } else { "" };
            let first = note.take().unwrap_or_else(|| "not snapped".to_string());
            *note = Some(format!("{first}; its border was not peeled{further}: {stop}"));
        }
    }
    let faces = peeled.into_iter().map(|p| p.map(|p| p.faces)).collect();
    Snap { body, moved, why_not: notes, faces }
}

/// Snaps the groups that lie within the slack of their primitive and can be placed on it, except those in
/// `refused`; a group that is not snapped and is too far from its primitive to be merged as it is has its border
/// peeled (`peel`) and is tried again, [`MAX_PEELS`] times at most.
fn snap_round(
    body: &Brep,
    specs: &[GroupSpec],
    before: &[std::result::Result<Group, String>],
    max_gap: f64,
    refused: &Reasons,
    cancel: &dyn Fn() -> bool,
) -> Snap {
    let gauges = specs.iter().take(MAX_GROUPS).map(|s| Gauge::new(s.shape).ok()).collect();
    let ctx = SnapCtx { body, specs, before, gauges, closed: is_closed(body), max_gap, refused, cancel };
    let mut peeled: Vec<Option<Peeled>> = specs.iter().map(|_| None).collect();
    let mut stopped: Reasons = vec![None; specs.len()];
    let mut times = 0;
    loop {
        // a peel removes at least one patch, and peeling stops after MAX_PEELS: the loop ends
        let (snapped, spec_of, peels) = snap_once(&ctx, &peeled, &stopped, times < MAX_PEELS);
        let mut changed = false;
        for (k, p) in peels {
            match (p, peeled.get_mut(k), stopped.get_mut(k)) {
                (Ok(p), Some(slot), _) => {
                    *slot = Some(p);
                    changed = true;
                }
                (Err(why), _, Some(slot)) => *slot = Some(why),
                _ => {}
            }
        }
        if !changed {
            return snap_of(&ctx, snapped, &spec_of, peeled, &stopped);
        }
        times += 1;
    }
}

/// One round: the outcome, the specs that were snapped but not merged, with their reasons, and the patches of the
/// specs whose border was peeled.
struct Round {
    batch: Batch,
    undo: Vec<(usize, String)>,
    faces: Vec<Option<Vec<usize>>>,
}

/// What every round reads.
struct Ctx<'a> {
    original: &'a Brep,
    specs: &'a [GroupSpec<'a>],
    before: &'a [std::result::Result<Group, String>],
    max_gap: f64,
    run: &'a Run<'a>,
}

fn cancelled() -> KernelError {
    KernelError::Argument("cancelled".into())
}

/// The snap of a round: the groups not in `taken_back` snapped (when `snapping`), and for each group of
/// `taken_back` the note that its snap was taken back.
fn snap_for(ctx: &Ctx, taken_back: &Reasons, snapping: bool) -> Snap {
    let mut snap = if snapping {
        snap_round(ctx.original, ctx.specs, ctx.before, ctx.max_gap, taken_back, ctx.run.cancel)
    } else {
        let n = ctx.specs.len();
        Snap { body: ctx.original.clone(), moved: vec![false; n], why_not: vec![None; n], faces: vec![None; n] }
    };
    for (k, why) in taken_back.iter().enumerate() {
        if let (Some(why), Some(slot)) = (why, snap.why_not.get_mut(k)) {
            slot.get_or_insert_with(|| format!("its snap was taken back: {why}"));
        }
    }
    snap
}

/// `why` with the group's own note of why it was not snapped.
fn noted(snap: &Snap, k: usize, why: &str) -> String {
    match snap.why_not.get(k).and_then(Option::as_ref) {
        Some(n) => format!("{why} (not snapped: {n})"),
        None => why.to_string(),
    }
}

/// A round in which no group was merged: the body as it was, each group's reason, and the groups that were snapped
/// to be tried again without their snap.
fn lost(ctx: &Ctx, snap: &Snap, prepared: &[std::result::Result<Prepared, String>], why: &str) -> Round {
    let reasons: Vec<String> = prepared
        .iter()
        .enumerate()
        .map(|(k, p)| noted(snap, k, &p.as_ref().err().cloned().unwrap_or_else(|| why.to_string())))
        .collect();
    let undo = (0..reasons.len())
        .filter(|&k| snap.moved.get(k).copied().unwrap_or(false))
        .filter_map(|k| prepared.get(k).map(|p| (k, p.as_ref().err().cloned().unwrap_or_else(|| why.to_string()))))
        .collect();
    Round { batch: unchanged(ctx.original, reasons), undo, faces: snap.faces.clone() }
}

/// The batch of a settled round and the snapped groups that were refused.
fn finish(
    specs: &[GroupSpec],
    snap: &Snap,
    prepared: &[std::result::Result<Prepared, String>],
    settled: super::settle::Settled,
) -> Round {
    let (brep, face_map, mut verdicts) = settled;
    verdicts.reverse();
    let mut results = Vec::with_capacity(specs.len());
    for (spec, p) in specs.iter().zip(prepared) {
        let outcome = p.as_ref().map_err(Clone::clone).and_then(|p| {
            verdicts.pop().unwrap_or_else(|| Err("no verdict".to_string()))?;
            let first = spec.faces.iter().copied().min().unwrap_or(0);
            let face = face_map.get(first).copied().flatten().ok_or("face map")?;
            Ok((face, p.gap, p.area))
        });
        results.push(outcome);
    }
    let undo = results
        .iter()
        .enumerate()
        .filter(|(k, r)| snap.moved.get(*k).copied().unwrap_or(false) && r.is_err())
        .filter_map(|(k, r)| r.as_ref().err().map(|why| (k, why.clone())))
        .collect();
    let groups = results
        .into_iter()
        .enumerate()
        .map(|(k, r)| match r {
            Ok((face, gap, patch_area)) => {
                let snap_note = snap.why_not.get(k).cloned().flatten();
                Ok(GroupResult { face, gap, patch_area, snap_note, peeled: 0 })
            }
            Err(why) => Err(noted(snap, k, &why)),
        })
        .collect();
    Round { batch: Batch { brep, face_map, groups }, undo, faces: snap.faces.clone() }
}

/// One round: the groups not in `taken_back` snapped (when `snapping`, their borders peeled where that is needed),
/// prepared and settled on the snapped body.
fn round(ctx: &Ctx, taken_back: &Reasons, snapping: bool) -> Result<Round> {
    let snap = snap_for(ctx, taken_back, snapping);
    let specs: Vec<GroupSpec> = ctx
        .specs
        .iter()
        .zip(&snap.faces)
        .map(|(s, f)| GroupSpec { faces: f.as_deref().unwrap_or(s.faces), shape: s.shape })
        .collect();
    let changed = snap.moved.iter().any(|m| *m) || snap.faces.iter().any(Option::is_some);
    let after = if changed { groups_in(&snap.body, &specs) } else { Vec::new() };
    let groups = if changed { &after[..] } else { ctx.before };
    let body = &snap.body;
    let none: Reasons = vec![None; specs.len()];
    let prepared = prepare_all(body, &specs, groups, (ctx.max_gap, &snap.moved, &none), ctx.run.cancel)?;
    let all: Vec<&Prepared> = prepared.iter().filter_map(|p| p.as_ref().ok()).collect();
    if all.is_empty() {
        return Ok(lost(ctx, &snap, &prepared, ""));
    }
    let volume = match is_closed(body).then(|| trusted_volume(body)).transpose() {
        Ok(v) => v,
        // a body the snap made unusable is not the end of the merge: the groups go on without it
        Err(e) if snap.moved.iter().any(|m| *m) => return Ok(lost(ctx, &snap, &prepared, &e.to_string())),
        Err(e) => return Err(e),
    };
    match settle(body, volume, &all, ctx.run) {
        Ok(settled) => Ok(finish(&specs, &snap, &prepared, settled)),
        Err(_) if (ctx.run.cancel)() => Err(cancelled()),
        Err(e) => Ok(lost(ctx, &snap, &prepared, &e.to_string())),
    }
}

/// The batch with the patches peeled off each group (of `given`, now `faces`) counted: in the result of a merged
/// group, in the reason of a refused one.
fn counted(mut batch: Batch, given: &[GroupSpec], faces: &[Vec<usize>]) -> Batch {
    for ((g, spec), now) in batch.groups.iter_mut().zip(given).zip(faces) {
        let peeled = spec.faces.len().saturating_sub(now.len());
        match g {
            Ok(r) => r.peeled = peeled,
            Err(why) if peeled > 0 => *why = format!("{why} (after {peeled} patches were peeled off its border)"),
            Err(_) => {}
        }
    }
    batch
}

/// The merge of `specs` in rounds (see the header): the batch of the first round that has no snapped group
/// refused after its snap. A group whose snap is taken back is tried again as it was (with the patches a peel left
/// it), and says so.
pub(super) fn merge_rounds(original: &Brep, specs: &[GroupSpec], max_gap: f64, run: &Run) -> Result<Batch> {
    let mut faces: Vec<Vec<usize>> = specs.iter().map(|s| s.faces.to_vec()).collect();
    let mut taken_back: Reasons = vec![None; specs.len()];
    for r in 0..MAX_ROUNDS {
        let now: Vec<GroupSpec> =
            specs.iter().zip(&faces).map(|(s, f)| GroupSpec { faces: f, shape: s.shape }).collect();
        let before = groups_in(original, &now);
        let ctx = Ctx { original, specs: &now, before: &before, max_gap, run };
        let Round { batch, undo, faces: peeled } = round(&ctx, &taken_back, r + 1 < MAX_ROUNDS)?;
        for (slot, p) in faces.iter_mut().zip(peeled) {
            if let Some(p) = p {
                *slot = p;
            }
        }
        if undo.is_empty() {
            return Ok(counted(batch, specs, &faces));
        }
        for (k, why) in undo {
            if let Some(slot) = taken_back.get_mut(k) {
                *slot = Some(why);
            }
        }
    }
    // the last round snaps nothing, so it has nothing to take back and the loop ends in it
    Err(KernelError::Argument("the rounds of a merge did not end".into()))
}
