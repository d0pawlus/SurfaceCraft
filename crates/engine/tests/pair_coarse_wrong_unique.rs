//! `align.pair_coarse` never gives a wrong pose as `unique` (its design promise), measured against
//! the known truth on every pair of the views of four fixtures (the views of the bracket, of the
//! blob, six wedges of the blob, eight arcs of a lumpy ring: sigma 0.05, every view its own
//! tessellation, noise and motion), three series of noise and motions each. `pair_confusion` has the
//! classes and the table; `PAIR_SEEDS=0,1,2` picks the series of the ignored table test.
//!
//! The defect these gates close (found while building `align.global`): about a quarter of the
//! `unique` answers were wrong by tens of millimetres, with confidence up to 1.00, wherever two
//! scans share little and the surface is self-similar (the faces of a box, a lumpy ellipsoid, the
//! tube of a ring). Measured before the fix over these 219 pairs: 28 wrong `unique`, 109 right.
mod global_common;
mod pair_common;
mod pair_confusion;
use pair_confusion::*;
use surfacecraft_align::pair::MAX_MUTUAL_CONFLICT;

/// Fewest of the pairs that share at least 0.4 that must be found and called `unique` (measured
/// 0.91 over the three series; the bound is 1.5 times below).
const MIN_RECALL: f64 = 0.6;
/// A pose that the command returns must be contradicted by both scans by more than the bound if it
/// is wrong, whatever the verdict; measured at least 1.8 times the bound.
const WRONG_CONFLICT_RATIO: f64 = 1.5;

fn gate(series: u64) {
    let cases = all_cases(&[series]);
    eprintln!("{}", table(&cases));
    let unique: Vec<&Case> = cases.iter().filter(|c| c.is_unique()).collect();
    let wrong = wrong_unique_lines(&cases);
    assert!(wrong.is_empty(), "wrong poses called unique:\n{}", wrong.join("\n"));
    assert!(
        cases.iter().filter(|c| c.overlap() < NOTHING).all(|c| !c.is_unique()),
        "scans that share nothing are never unique"
    );
    let worst_right = unique.iter().map(|c| mutual_conflict(c)).fold(0.0, f64::max);
    let best_wrong = cases.iter().filter(|c| c.is_wrong()).map(mutual_conflict).fold(f64::INFINITY, f64::min);
    eprintln!(
        "series {series}: {} unique, all right; smallest conflict of both scans: right poses at most {worst_right:.4} \
         (bound {MAX_MUTUAL_CONFLICT}, {:.1} times above), wrong poses at least {best_wrong:.4} ({:.1} times the bound)",
        unique.len(),
        MAX_MUTUAL_CONFLICT / worst_right.max(1e-9),
        best_wrong / MAX_MUTUAL_CONFLICT
    );
    assert!(worst_right * WRONG_CONFLICT_RATIO <= MAX_MUTUAL_CONFLICT, "a right pose is near the bound: {worst_right}");
    assert!(best_wrong >= WRONG_CONFLICT_RATIO * MAX_MUTUAL_CONFLICT, "a wrong pose is near the bound: {best_wrong}");
    let sharing: Vec<&Case> = cases.iter().filter(|c| c.overlap() >= SHARING).collect();
    let found = sharing.iter().filter(|c| c.is_unique()).count() as f64 / sharing.len() as f64;
    eprintln!("series {series}: {:.2} of the {} pairs that share at least {SHARING} are found", found, sharing.len());
    assert!(found >= MIN_RECALL, "recall {found}");
}

fn mutual_conflict(c: &Case) -> f64 {
    c.field("conflict_moving").min(c.field("conflict_fixed"))
}

#[test]
fn gate_m2_no_wrong_pose_is_unique_series_0() {
    gate(0);
}

#[test]
fn gate_m2_no_wrong_pose_is_unique_series_1() {
    gate(1);
}

#[test]
fn gate_m2_no_wrong_pose_is_unique_series_2() {
    gate(2);
}

#[test]
#[ignore = "prints every pair with its evidence and the confusion table of the current build; asserts nothing"]
fn print_the_confusion_table() {
    let seeds: Vec<u64> = std::env::var("PAIR_SEEDS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![0, 1, 2]);
    let cases = all_cases(&seeds);
    for l in evidence_lines(&cases) {
        eprintln!("{l}");
    }
    eprintln!("{}", table(&cases));
    for l in wrong_unique_lines(&cases) {
        eprintln!("{l}");
    }
}
