//! The rounds of a merge: the boundaries of the groups are put on their primitives (`snap`), the groups
//! are prepared and settled on the snapped body, and a group that was snapped but is then refused has its
//! snap taken back: the next round snaps only what is left, from the body as it was, so the neighbours of a
//! refused group are exactly as the wrap had them.
use super::geometry::{Group, group_of};
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

/// The body with the boundaries snapped, which specs were, and why the others were not.
struct Snap {
    body: Brep,
    moved: Vec<bool>,
    why_not: Reasons,
}

/// Snaps the groups that lie within `max_gap` of their primitive and can be placed on it, except those in
/// `refused`. The groups are looked at in order, and a face belongs to the first that claims it.
fn snap_round(
    body: &Brep,
    specs: &[GroupSpec],
    before: &[std::result::Result<Group, String>],
    max_gap: f64,
    refused: &Reasons,
    cancel: &dyn Fn() -> bool,
) -> Snap {
    let gauges: Vec<Option<Gauge>> = specs.iter().take(MAX_GROUPS).map(|s| Gauge::new(s.shape).ok()).collect();
    let closed = is_closed(body);
    let mut claimed: BTreeSet<usize> = BTreeSet::new();
    let (mut targets, mut spec_of) = (Vec::new(), Vec::new());
    for (k, spec) in specs.iter().enumerate().take(MAX_GROUPS) {
        let (Some(Some(gauge)), Some(Ok(group))) = (gauges.get(k), before.get(k)) else { continue };
        if refused.get(k).is_some_and(Option::is_some) || spec.faces.iter().any(|f| claimed.contains(f)) {
            continue;
        }
        let near = group.gap(gauge).is_ok_and(|g| g <= NEAR_SLACK * max_gap);
        if near && place(spec.shape, &group.points(), group.outward).is_ok_and(|p| closed || !p.flip) {
            claimed.extend(spec.faces.iter().copied());
            targets.push(Target { faces: spec.faces, group, gauge });
            spec_of.push(k);
        }
    }
    let Snapped { body: snapped, snapped: flags, why_not } =
        snap_boundaries(body, &targets, MOVE_PER_GAP * max_gap, cancel);
    let (mut moved, mut notes) = (vec![false; specs.len()], vec![None; specs.len()]);
    for (t, &k) in spec_of.iter().enumerate() {
        moved[k] = flags.get(t).copied().unwrap_or(false);
        notes[k] = why_not.get(t).cloned().flatten();
    }
    Snap { body: snapped, moved, why_not: notes }
}

/// One round: the outcome, and the specs that were snapped but not merged, with their reasons.
struct Round {
    batch: Batch,
    undo: Vec<(usize, String)>,
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
        Snap { body: ctx.original.clone(), moved: vec![false; ctx.specs.len()], why_not: vec![None; ctx.specs.len()] }
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
    eprintln!(
        "DIAG noted group {k}: moved {:?}; {why}; not snapped because {:?}",
        snap.moved.get(k),
        snap.why_not.get(k).and_then(Option::as_ref)
    );
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
    Round { batch: unchanged(ctx.original, reasons), undo }
}

/// The batch of a settled round and the snapped groups that were refused.
fn finish(
    ctx: &Ctx,
    snap: &Snap,
    prepared: &[std::result::Result<Prepared, String>],
    settled: super::settle::Settled,
) -> Round {
    let (brep, face_map, mut verdicts) = settled;
    verdicts.reverse();
    let mut results = Vec::with_capacity(ctx.specs.len());
    for (spec, p) in ctx.specs.iter().zip(prepared) {
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
                Ok(GroupResult { face, gap, patch_area, snap_note: snap.why_not.get(k).cloned().flatten() })
            }
            Err(why) => Err(noted(snap, k, &why)),
        })
        .collect();
    Round { batch: Batch { brep, face_map, groups }, undo }
}

/// One round: the groups not in `taken_back` snapped (when `snapping`), prepared and settled on the snapped body.
fn round(ctx: &Ctx, taken_back: &Reasons, snapping: bool) -> Result<Round> {
    let snap = snap_for(ctx, taken_back, snapping);
    let any_moved = snap.moved.iter().any(|m| *m);
    let after = if any_moved { groups_in(&snap.body, ctx.specs) } else { Vec::new() };
    let groups = if any_moved { &after[..] } else { ctx.before };
    let body = &snap.body;
    let none: Reasons = vec![None; ctx.specs.len()];
    let prepared = prepare_all(body, ctx.specs, groups, (ctx.max_gap, &snap.moved, &none), ctx.run.cancel)?;
    let all: Vec<&Prepared> = prepared.iter().filter_map(|p| p.as_ref().ok()).collect();
    if all.is_empty() {
        return Ok(lost(ctx, &snap, &prepared, ""));
    }
    let volume = match is_closed(body).then(|| trusted_volume(body)).transpose() {
        Ok(v) => v,
        // a body the snap made unusable is not the end of the merge: the groups go on without it
        Err(e) if any_moved => return Ok(lost(ctx, &snap, &prepared, &e.to_string())),
        Err(e) => return Err(e),
    };
    match settle(body, volume, &all, ctx.run) {
        Ok(settled) => Ok(finish(ctx, &snap, &prepared, settled)),
        Err(_) if (ctx.run.cancel)() => Err(cancelled()),
        Err(e) => Ok(lost(ctx, &snap, &prepared, &e.to_string())),
    }
}

/// The merge of `specs` in rounds (see the header): the batch of the first round that has no snapped group
/// refused after its snap. A group whose snap is taken back is tried again as it was, and says so.
pub(super) fn merge_rounds(original: &Brep, specs: &[GroupSpec], max_gap: f64, run: &Run) -> Result<Batch> {
    let before = groups_in(original, specs);
    let ctx = Ctx { original, specs, before: &before, max_gap, run };
    let mut taken_back: Reasons = vec![None; specs.len()];
    for r in 0..MAX_ROUNDS {
        let Round { batch, undo } = round(&ctx, &taken_back, r + 1 < MAX_ROUNDS)?;
        if undo.is_empty() {
            return Ok(batch);
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
