//! Closed bicubic patch networks of thousands of patches (P9-O3). The record of such a network is
//! far past solidcore's 4 000 000 item budget for input records (about 40 000 patches); the kernel
//! builds it as a record of its own and checks it without the budget.
mod common;

use common::{
    cube_sphere, edges_are_consistent, faces_are_as_given, flip_patch, n_for, pillow, record_fingerprint,
    sphere_volume, thin_box, thin_enclosure,
};
use surfacecraft_kernel::nurbs::{
    MAX_NETWORK_PATCHES, is_closed, sheet_from_bezier_network, sheet_from_bezier_network_as_is,
};
use surfacecraft_kernel::solid;

const RADIUS: f64 = 50.0;
/// The control points lie on the sphere, the surface between them slightly inside: the volume
/// is within 1e-3 of the sphere's from about 8 000 patches on (measured: 2.2e-4 at 12 150).
const VOLUME_TOLERANCE: f64 = 1e-3;

/// Builds a cube-sphere network of at least `count` patches and checks the solid.
fn assert_closed_solid(count: usize) {
    let net = cube_sphere(n_for(count), RADIUS);
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("network builds");
    assert_eq!(sheet.faces.len(), net.patches.len());
    assert!(sheet.faces.len() >= count);
    // every edge of a closed network is used by exactly two faces
    assert!(is_closed(&sheet), "{} patches: not closed", sheet.faces.len());
    assert!(sheet.edges.iter().all(|e| e.uses.len() == 2));
    let volume = solid::volume(&sheet).expect("volume of the solid");
    let exact = sphere_volume(RADIUS);
    assert!(
        (volume - exact).abs() <= VOLUME_TOLERANCE * exact,
        "{} patches: volume {volume} vs sphere {exact}",
        sheet.faces.len()
    );
}

#[test]
fn network_of_12000_patches_is_a_closed_solid() {
    assert_closed_solid(12_000);
}

#[test]
fn network_of_16000_patches_is_a_closed_solid() {
    assert_closed_solid(16_000);
}

/// 60 000 patches is the engine's `PATCH_LIMIT`; it is past the 4 000 000 item budget of
/// `Brep::validate` (the failure of the 1 M triangle scan).
#[test]
fn network_of_60000_patches_is_a_closed_solid() {
    assert_closed_solid(60_000);
}

#[test]
fn healer_is_skipped_for_large_networks_behind_the_healing_entry_point() {
    let net = cube_sphere(n_for(12_000), RADIUS);
    let sheet = sheet_from_bezier_network(&net.patches, &net.corners).expect("network builds");
    assert!(is_closed(&sheet));
}

#[test]
fn network_with_a_missing_patch_is_an_open_sheet() {
    let net = cube_sphere(n_for(12_000), RADIUS);
    let sheet = sheet_from_bezier_network_as_is(&net.patches[1..], &net.corners[1..]).expect("open sheet builds");
    assert!(!is_closed(&sheet));
}

#[test]
fn empty_and_oversized_networks_are_refused_by_the_size_guard() {
    let message = |r: surfacecraft_kernel::Result<_>| match r {
        Ok(_) => String::new(),
        Err(e) => e.to_string(),
    };
    assert!(message(sheet_from_bezier_network_as_is(&[], &[])).contains("patches, got 0"));
    // refused before any geometry is looked at: the message names the limit, not a shared edge
    let net = cube_sphere(2, RADIUS);
    let many: Vec<_> = net.patches.iter().cycle().take(MAX_NETWORK_PATCHES + 1).cloned().collect();
    let corners: Vec<_> = net.corners.iter().cycle().take(MAX_NETWORK_PATCHES + 1).copied().collect();
    let text = message(sheet_from_bezier_network_as_is(&many, &corners));
    assert!(text.contains(&format!("patches, got {}", MAX_NETWORK_PATCHES + 1)), "{text}");
}

/// 12 150 patches, built and measured before the orientation pass existed (the same as today).
const VOLUME_12150: f64 = 523_493.698_562_392_9;

#[test]
fn the_normal_network_is_unchanged_by_the_orientation_pass() {
    let net = cube_sphere(n_for(12_000), RADIUS);
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("builds");
    assert_eq!(sheet.faces.len(), 12_150);
    assert!(edges_are_consistent(&sheet));
    let volume = solid::volume(&sheet).expect("volume");
    assert!((volume - VOLUME_12150).abs() <= 1e-9 * VOLUME_12150, "volume {volume}");
}

/// A network of more than 2000 patches goes through the build without the healer.
fn baseline_volume(count: usize) -> f64 {
    let net = cube_sphere(n_for(count), RADIUS);
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("builds");
    solid::volume(&sheet).expect("volume")
}

#[test]
fn one_transposed_patch_is_turned_back() {
    let baseline = baseline_volume(2_100);
    let mut net = cube_sphere(n_for(2_100), RADIUS);
    flip_patch(&mut net, 1_000);
    for sheet in [
        sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("repaired"),
        sheet_from_bezier_network(&net.patches, &net.corners).expect("repaired"),
    ] {
        assert!(is_closed(&sheet) && edges_are_consistent(&sheet));
        let volume = solid::volume(&sheet).expect("volume");
        assert!((volume - baseline).abs() <= 1e-9 * baseline, "volume {volume} vs {baseline}");
    }
}

#[test]
fn an_inside_out_network_is_turned_outward() {
    let baseline = baseline_volume(2_100);
    let mut net = cube_sphere(n_for(2_100), RADIUS);
    (0..net.patches.len()).for_each(|f| flip_patch(&mut net, f));
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("turned outward");
    assert!(is_closed(&sheet) && edges_are_consistent(&sheet));
    let volume = solid::volume(&sheet).expect("volume");
    assert!(volume > 0.0 && (volume - baseline).abs() <= 1e-9 * baseline, "volume {volume} vs {baseline}");
}

#[test]
fn an_inside_out_network_with_a_flipped_patch_is_turned_outward() {
    let baseline = baseline_volume(2_100);
    let mut net = cube_sphere(n_for(2_100), RADIUS);
    (0..net.patches.len()).filter(|f| f % 7 != 0).for_each(|f| flip_patch(&mut net, f));
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("repaired");
    let volume = solid::volume(&sheet).expect("volume");
    assert!(volume > 0.0 && (volume - baseline).abs() <= 1e-9 * baseline, "volume {volume} vs {baseline}");
}

#[test]
fn a_thin_enclosure_keeps_its_outward_orientation() {
    let net = thin_enclosure();
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("builds");
    assert!(is_closed(&sheet) && faces_are_as_given(&sheet, &net));
    let volume = solid::volume(&sheet).expect("volume");
    assert!((volume - (0.01 + 1.0 / 6.0)).abs() < 1e-9, "volume {volume}");
}

#[test]
fn a_thin_box_far_from_the_origin_keeps_its_orientation() {
    let net = thin_box([1.0e6; 3]);
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("builds");
    assert!(is_closed(&sheet) && faces_are_as_given(&sheet, &net));
}

#[test]
fn an_inside_out_thin_enclosure_is_turned_outward() {
    let given = thin_enclosure();
    let mut net = thin_enclosure();
    (0..net.patches.len()).for_each(|f| flip_patch(&mut net, f));
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("turned outward");
    assert!(faces_are_as_given(&sheet, &given));
}

#[test]
fn a_closed_network_without_volume_is_refused() {
    let net = pillow();
    let text = sheet_from_bezier_network_as_is(&net.patches, &net.corners).map(|_| ()).map_err(|e| e.to_string());
    assert!(text.as_ref().is_err_and(|m| m.contains("volume")), "{text:?}");
}

/// Fingerprint of the 12 150 patch record built before the orientation pass existed.
const RECORD_FINGERPRINT_12150: u64 = 7_837_018_526_937_721_461;

#[test]
fn the_record_of_a_consistent_network_is_what_it_was_before_the_pass() {
    let net = cube_sphere(n_for(12_000), RADIUS);
    let sheet = sheet_from_bezier_network_as_is(&net.patches, &net.corners).expect("builds");
    assert!(faces_are_as_given(
        &sheet,
        &common::Network { patches: net.patches.clone(), corners: net.corners.clone(), radius: net.radius }
    ));
    assert_eq!(record_fingerprint(&sheet), RECORD_FINGERPRINT_12150);
}
