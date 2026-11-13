//! Tests of the coverage issue of `scan.to_solid` (moved out of `pipeline.rs`, which is at the file size limit).
use super::*;

fn summary_of(coverage: f64, not_facing: f64) -> DeviationSummary {
    DeviationSummary { count: 100, coverage, not_facing, within: 1.0, rms: 0.0, max_abs: 0.0, mean: 0.0 }
}

#[test]
fn a_solid_that_covers_the_scan_has_no_coverage_issue() {
    assert!(coverage_issue(&summary_of(0.95, 0.0), "the solid").is_none());
}

fn code(coverage: f64, not_facing: f64) -> String {
    coverage_issue(&summary_of(coverage, not_facing), "the solid").expect("an issue").code
}

#[test]
fn points_that_lie_on_the_solid_but_face_away_are_an_orientation_issue_not_missing_scan() {
    assert_eq!(code(0.71, 0.29), "inconsistent_orientation");
}

#[test]
fn a_few_percent_facing_away_beside_a_real_gap_is_low_coverage_with_both_figures() {
    // 85 % cover, 6 % face away, 9 % are far: the gap is the problem
    let issue = coverage_issue(&summary_of(0.85, 0.06), "the solid").expect("an issue");
    assert_eq!(issue.code, "low_coverage");
    assert!(issue.message.contains("85.0 %") && issue.message.contains("6.0 %") && issue.message.contains("9.0 %"));
}

#[test]
fn the_orientation_issue_starts_at_half_of_the_missing_share() {
    // missing 0.20: 0.10 facing away is the boundary
    assert_eq!(code(0.80, 0.10), "inconsistent_orientation");
    assert_eq!(code(0.80, 0.0999), "low_coverage");
}

#[test]
fn a_scan_whose_points_nearly_all_face_away_is_inside_out() {
    assert_eq!(code(0.05, 0.95), "scan_inside_out");
    assert_eq!(code(0.10, 0.90), "scan_inside_out");
    assert_eq!(code(0.20, 0.80), "inconsistent_orientation");
}

#[test]
fn points_far_from_the_solid_are_low_coverage() {
    assert_eq!(code(0.5, 0.05), "low_coverage");
}
