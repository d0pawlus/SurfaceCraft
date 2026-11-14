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
        let near = group.gap(gauge).is_ok_and(|g| g <= max_gap);
        if near && place(spec.shape, &group.points(), group.outward).is_ok_and(|p| closed || !p.flip) {
            claimed.extend(spec.faces.iter().copied());
            targets.push(Target { faces: spec.faces, group, gauge });
            spec_of.push(k);
        }
    }
    let Snapped { body: snapped, snapped: flags, why_not } = snap_boundaries(body, &targets, MOVE_PER_GAP * max_gap);
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

fn round(
    original: &Brep,
    specs: &[GroupSpec],
    before: &[std::result::Result<Group, String>],
    max_gap: f64,
    refused: Option<&Reasons>,
    run: &Run,
) -> Result<Round> {
    let snap = match refused {
        // the last round: nothing is snapped and nothing is excluded
        None => Snap { body: original.clone(), moved: vec![false; specs.len()], why_not: vec![None; specs.len()] },
        Some(refused) => snap_round(original, specs, before, max_gap, refused),
    };
    let none_refused: Reasons = vec![None; specs.len()];
    let refused = refused.unwrap_or(&none_refused);
    let any_moved = snap.moved.iter().any(|m| *m);
    let after = if any_moved { groups_in(&snap.body, specs) } else { Vec::new() };
    let groups = if any_moved { &after[..] } else { before };
    let body = &snap.body;
    let prepared = prepare_all(body, specs, groups, (max_gap, &snap.moved, refused), run.cancel)?;
    let note = |k: usize, why: &str| match snap.why_not.get(k).and_then(Option::as_ref) {
        Some(n) => format!("{why} (not snapped: {n})"),
        None => why.to_string(),
    };
    let reasons = |why: &str| {
        prepared.iter().enumerate().map(|(k, p)| p.as_ref().err().cloned().unwrap_or_else(|| note(k, why))).collect()
    };
    let all: Vec<&Prepared> = prepared.iter().filter_map(|p| p.as_ref().ok()).collect();
    if all.is_empty() {
        let reasons =
            prepared.iter().enumerate().map(|(k, p)| note(k, &p.as_ref().err().cloned().unwrap_or_default())).collect();
        return Ok(Round { batch: unchanged(original, reasons), undo: Vec::new() });
    }
    let volume = is_closed(body).then(|| trusted_volume(body)).transpose()?;
    let (brep, face_map, mut verdicts) = match settle(body, volume, &all, run) {
        Ok(settled) => settled,
        Err(_) if (run.cancel)() => return Err(KernelError::Argument("cancelled".into())),
        Err(e) => return Ok(Round { batch: unchanged(original, reasons(&e.to_string())), undo: Vec::new() }),
    };
    verdicts.reverse();
    let mut results = Vec::with_capacity(specs.len());
    for (spec, p) in specs.iter().zip(&prepared) {
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
            Err(why) => Err(note(k, &why)),
        })
        .collect();
    Ok(Round { batch: Batch { brep, face_map, groups }, undo })
}

/// The merge of `specs` in rounds (see the header): the batch of the first round that has no snapped group
/// refused after its snap.
pub(super) fn merge_rounds(original: &Brep, specs: &[GroupSpec], max_gap: f64, run: &Run) -> Result<Batch> {
    let before = groups_in(original, specs);
    let mut refused: Reasons = vec![None; specs.len()];
    for r in 0..MAX_ROUNDS {
        let last = r + 1 == MAX_ROUNDS;
        let Round { batch, undo } = round(original, specs, &before, max_gap, (!last).then_some(&refused), run)?;
        if undo.is_empty() {
            return Ok(batch);
        }
        for (k, why) in undo {
            if let Some(slot) = refused.get_mut(k) {
                *slot = Some(why);
            }
        }
    }
    // the last round has nothing to undo, so the loop ends in it
    Err(KernelError::Argument("the rounds of a merge did not end".into()))
}
