//! A small patch against a whole part (`align.pair_coarse`): the conflict of the whole part's
//! side must be seen, not only the patch's. A clean side is never enough: a flat corner patch laid
//! on another corner of the bracket lies on surface everywhere (its own conflict is 0), while the
//! boss of the fixed scan stands in the free space above the patch's footprint.
//! (a) corner patch at a wrong corner that has the boss under its footprint: both sides measured,
//! never `unique`; (b) the same patch at its true corner: no conflict on either side; (c) patches
//! with a feature (the boss of the bracket, a lump of the blob) at their true place, 3 series: `unique` and
//! right within the lever-arm bound (0.5 deg, 0.3 mm of the blob patch gate).
mod global_common;
mod pair_common;
mod pair_confusion;
use global_common::visible_bracket_at;
use pair_common::*;
use serde_json::json;
use surfacecraft_align::pair::UNIQUE_MAX_CONFLICT;
use surfacecraft_align::{PairOptions, PairVerdict, pair_coarse, pose_fit};
use surfacecraft_geom::{Cancel, Mesh, Transform, Vec3};

fn fixed_bracket() -> Mesh {
    noisy(visible_bracket_at(12, 48, 16), 1)
}

/// A corner of the bracket's box (top face and the two walls, 22 mm each way), its own tessellation.
fn corner_patch() -> Mesh {
    let whole = noisy(visible_bracket_at(15, 64, 20), 2);
    cut(&whole, |c| c.z > 12.0 && c.x < 22.0 && c.y < 22.0 && (c.z > 19.5 || c.x < 0.9 || c.y < 0.9))
}

/// The corner patch carried onto the corner at (60, 0): the boss (45, 12) stands above its footprint.
fn boss_corner() -> Transform {
    Transform::translation(Vec3::new(60.0, 0.0, 0.0))
        .then_after(&Transform::rotation_axis_angle(&Vec3::z(), std::f64::consts::FRAC_PI_2))
}

#[test]
fn a_patch_on_surface_everywhere_is_contradicted_by_the_feature_above_it() {
    let (fixed, patch) = (fixed_bracket(), corner_patch());
    let tau = Some(0.2);
    let wrong = pose_fit(&patch, &fixed, &boss_corner(), tau, &Cancel::new()).unwrap();
    let right = pose_fit(&patch, &fixed, &Transform::identity(), tau, &Cancel::new()).unwrap();
    eprintln!("wrong corner {wrong:?}\ntrue corner {right:?}");
    assert!(wrong.overlap_moving > 0.6, "the patch lies on the box faces: {}", wrong.overlap_moving);
    assert!(
        wrong.conflict_moving <= UNIQUE_MAX_CONFLICT / 2.0,
        "the patch's own side is clean: {}",
        wrong.conflict_moving
    );
    assert!(wrong.conflict_fixed >= 2.0 * UNIQUE_MAX_CONFLICT, "the boss is seen: {}", wrong.conflict_fixed);
    assert!(right.conflict_moving.max(right.conflict_fixed) <= UNIQUE_MAX_CONFLICT / 2.0, "(b) {right:?}");
    assert!(right.overlap_moving > 0.9);
    let opts = PairOptions { given: Some(boss_corner()), ..PairOptions::default() };
    let r = pair_coarse(&patch, &fixed, &opts, &Cancel::new()).unwrap();
    eprintln!("verdict {:?}, conflicts {:.3}/{:.3}", r.verdict, r.conflict_moving, r.conflict_fixed);
    assert_ne!(r.verdict, PairVerdict::Unique);
}

fn assert_found(name: &str, series: u64, fixed: &Mesh, patch: &Mesh) {
    let pose = random_motion(series);
    let mut s = session(fixed, patch, &pose);
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, patch);
    eprintln!(
        "{name} series {series}: {} conflicts {:.4}/{:.4} overlap {:.2}/{:.2}, {angle:.3} deg {disp:.3} mm",
        out["verdict"], out["conflict_moving"], out["conflict_fixed"], out["overlap_moving"], out["overlap_fixed"]
    );
    assert_eq!(out["verdict"], "unique", "{name} series {series}: {out}");
    assert!(angle <= 0.5 && disp <= 0.3, "{name} series {series}: {angle} deg {disp} mm");
}

#[test]
fn a_bracket_patch_with_the_boss_is_found_in_three_series() {
    for series in 0..3u64 {
        let fixed = noisy(visible_bracket_at(12, 48, 16), 1 + 10 * series);
        let patch = cut(&noisy(visible_bracket_at(15, 64, 20), 2 + 10 * series), |c| c.x > 30.0 && c.z > 15.0);
        assert_found("bracket patch", series, &fixed, &patch);
    }
}

#[test]
fn a_blob_patch_is_found_in_three_series() {
    for series in 0..3u64 {
        let (fixed, moving) = pair_of_trial("blob", series);
        let patch = cut(&moving, |c| c.x > 15.0 && c.z > -4.0);
        assert_found("blob patch", series, &fixed, &patch);
    }
}
