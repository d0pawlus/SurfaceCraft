//! The heaviest tests of the suite with their measured duration, so `--partition balanced:K/N`
//! can spread them evenly over the parts instead of by count (a few tests hold most of the time).
//!
//! Measured on GitHub Actions on 2026-11-15 (runs 43718645559 and 43727884220, ci profile at
//! opt-level 2, parts of the suite on 4-core Linux and Windows and 3-core macOS runners): every
//! test whose duration, averaged over the three systems, was at least 100 s. The numbers only
//! steer the balance. A stale, renamed or missing entry makes the parts less even; it never drops
//! or doubles a test, because the other parts run everything that is not on this list.
//!
//! To refresh: take the `PASS [ N s] (..) <binary-id> <test>` lines of the `tests (...)` jobs of a
//! recent run, average each test over the systems and list those at or above the threshold.

/// `(nextest binary id, test name, seconds)`, heaviest first.
pub(crate) const HEAVY_TESTS: &[(&str, &str, u32)] = &[
    ("surfacecraft-engine::pair_coarse_wrong_unique", "gate_m2_no_wrong_pose_is_unique_series_0", 553),
    ("surfacecraft-engine::p9_messy_scans", "gate_a_thousand_handles_stall_the_wrap_and_the_rebuild_cures_it", 471),
    ("surfacecraft-engine::hybrid", "p9_block_with_wavy_top_has_one_freeform_top", 406),
    ("surfacecraft-geom", "recon::tests::hostile_clouds_never_panic", 378),
    ("surfacecraft-engine::pair_coarse_wrong_unique", "gate_m2_no_wrong_pose_is_unique_on_coarse_meshes", 368),
    ("surfacecraft-engine::p9_wrap_cap", "a_larger_cap_never_gives_a_worse_share_and_never_exceeds_itself", 270),
    ("surfacecraft-engine::p9_pair_review", "gate_m2_loose_poses_are_decided_by_pushing_them", 195),
    (
        "surfacecraft-engine::scan_to_solid_paths",
        "a_wrap_stopped_by_the_default_patch_limit_is_retried_once_with_a_larger_limit",
        170,
    ),
    ("surfacecraft-engine::close_handles", "gate_m4_a_tunnelled_wall_wraps_after_its_handles_are_closed", 142),
    ("surfacecraft-engine::p9_global", "gate_m3_a_ring_of_views_closes_its_loop", 134),
    ("surfacecraft-engine::p9_wrap_cap", "the_first_round_never_overshoots_a_tiny_cap", 130),
    (
        "surfacecraft-engine::p9_freeform_regions",
        "a_plane_and_a_cone_with_form_error_of_04_tolerance_stay_primitives",
        118,
    ),
    (
        "surfacecraft-engine::p9_noise_calibration",
        "the_tolerance_protects_primitives_with_form_error_and_no_organic_fragment",
        114,
    ),
    (
        "surfacecraft-engine::p9_noise_calibration",
        "the_threshold_keeps_genuine_primitives_and_rejects_most_fragments_of_a_clean_surface",
        114,
    ),
    ("surfacecraft-engine::p9_global", "gate_m3_six_blob_views_are_aligned_within_the_bound", 109),
    (
        "surfacecraft-engine::p9_freeform_regions",
        "large_primitives_with_form_error_stay_and_the_organic_surface_is_consolidated",
        109,
    ),
    ("surfacecraft-engine::p9_global", "gate_m3_a_view_of_another_part_is_left_unplaced", 101),
];
