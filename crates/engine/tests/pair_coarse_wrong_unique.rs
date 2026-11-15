//! `align.pair_coarse` never gives a wrong pose as `unique` (its design promise), measured against
//! the known truth on every pair of the views of four fixtures (the views of the bracket, of the
//! blob, six wedges of the blob, eight arcs of a lumpy ring: sigma 0.05, every view its own
//! tessellation, noise and motion), three series of noise and motions each (series 0 runs in CI,
//! the other two are `#[ignore]`d local runs: `cargo test ... -- --ignored gate_m2`). `pair_confusion` has the
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
use surfacecraft_align::pair::UNIQUE_MAX_CONFLICT;

/// Fewest of the pairs that share at least 0.4 that must be found and called `unique` (measured
/// over the three series: 0.65-0.70; the bound is 1.5 times below).
const MIN_RECALL: f64 = 0.4;
/// The margin asserted on both sides of the conflict bound (the numbers are printed).
const MARGIN: f64 = 2.0;

fn worst_conflict(c: &Case) -> f64 {
    c.field("conflict_moving").max(c.field("conflict_fixed"))
}

fn gate(series: u64) {
    check(&all_cases(&[series]), series, MIN_RECALL, true);
}

/// The assertions of the gate over `cases`; `recall` is the share of the pairs that share at least
/// 0.4 that must be found (0 to assert none). With `conflict_decides` every wrong pose the command
/// returns must be contradicted by more than the bound (2x); on coarse meshes the tolerance is
/// capped by the edge length instead and the conflicts cannot resolve the relief, so only the
/// absence of wrong `unique` is asserted (the true poses read up to 0.08 there: the edge cap is below
/// the chord error of such meshes, which is why they are certified less often).
fn check(cases: &[Case], series: u64, recall: f64, conflict_decides: bool) {
    eprintln!("{}", table(cases));
    let wrong = wrong_unique_lines(cases);
    assert!(wrong.is_empty(), "wrong poses called unique:\n{}", wrong.join("\n"));
    assert!(
        cases.iter().filter(|c| c.overlap() < NOTHING).all(|c| !c.is_unique()),
        "scans that share nothing are never unique"
    );
    // the populations the rule separates: every right pose (measured at the truth) and every wrong
    // pose the command returns that is not already a `no_match` by overlap
    let right_at_truth =
        cases.iter().filter_map(|c| c.at_truth).map(|t| t.conflict_moving.max(t.conflict_fixed)).fold(0.0, f64::max);
    let wrong_found: Vec<&Case> = cases.iter().filter(|c| c.is_wrong() && c.verdict != "no_match").collect();
    // a margin assertion over an empty population tests nothing
    let measured_at_truth = cases.iter().filter(|c| c.at_truth.is_some()).count();
    assert!(
        measured_at_truth > 0 && !wrong_found.is_empty(),
        "{measured_at_truth} right poses measured, {} wrong found",
        wrong_found.len()
    );
    let best_wrong = wrong_found.iter().map(|c| worst_conflict(c)).fold(f64::INFINITY, f64::min);
    eprintln!(
        "series {series}: larger conflict of the two scans: right poses at the truth at most {right_at_truth:.4} \
         (bound {UNIQUE_MAX_CONFLICT}: {:.1} times above), {} wrong poses returned at least {best_wrong:.4} ({:.1} times the bound)",
        UNIQUE_MAX_CONFLICT / right_at_truth.max(1e-9),
        wrong_found.len(),
        best_wrong / UNIQUE_MAX_CONFLICT
    );
    assert!(
        !conflict_decides || right_at_truth * MARGIN <= UNIQUE_MAX_CONFLICT,
        "a right pose is near the bound: {right_at_truth}"
    );
    assert!(
        !conflict_decides || best_wrong >= MARGIN * UNIQUE_MAX_CONFLICT,
        "a wrong pose is near the bound: {best_wrong}"
    );
    assert!(
        !conflict_decides
            || wrong_found.iter().all(|c| c.out["verdict"] != "unique" && worst_conflict(c) > UNIQUE_MAX_CONFLICT),
        "every wrong pose the command returns is contradicted"
    );
    let sharing: Vec<&Case> = cases.iter().filter(|c| c.overlap() >= SHARING).collect();
    let found = sharing.iter().filter(|c| c.is_unique()).count() as f64 / sharing.len() as f64;
    eprintln!("series {series}: {found:.2} of the {} pairs that share at least {SHARING} are found", sharing.len());
    assert!(found >= recall, "recall {found}");
}

#[test]
fn gate_m2_no_wrong_pose_is_unique_series_0() {
    gate(0);
}

/// Meshes whose noise estimate is not `smooth` (triangles of 4 to 10 mm): the tolerance has no noise
/// cap there, and the conflicts have to carry the verdict alone.
#[test]
fn gate_m2_no_wrong_pose_is_unique_on_coarse_meshes() {
    let views = coarse_fixtures(0);
    let smooth = views.iter().flat_map(|(_, v)| v).filter(|m| surfacecraft_deviation::noise_estimate(m).smooth).count();
    let total: usize = views.iter().map(|(_, v)| v.len()).sum();
    eprintln!("{smooth} of {total} coarse views have a smooth noise estimate");
    assert!(smooth * 2 <= total, "the fixtures are meant to be coarse: {smooth} of {total} are smooth");
    check(&coarse_cases(&[0]), 0, 0.0, false);
}

#[test]
#[ignore = "series 1 of 3: the same gate, run locally (CI runs series 0, a pair takes seconds)"]
fn gate_m2_no_wrong_pose_is_unique_series_1() {
    gate(1);
}

#[test]
#[ignore = "series 2 of 3: the same gate, run locally (CI runs series 0, a pair takes seconds)"]
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

#[test]
#[ignore = "prints the noise estimate of every view of the fixtures"]
fn print_the_noise_of_the_fixtures() {
    for (name, views) in fixtures(0) {
        for (k, v) in views.iter().enumerate() {
            let e = surfacecraft_deviation::noise_estimate(v);
            let spacing = (v.area() / 3000.0).sqrt();
            eprintln!(
                "{name} view {k}: sigma {:.4} smooth {} spacing {spacing:.2} ({} triangles)",
                e.sigma,
                e.smooth,
                v.triangle_count()
            );
        }
    }
}
