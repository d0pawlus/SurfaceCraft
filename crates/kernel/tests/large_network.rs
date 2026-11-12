//! Closed bicubic patch networks of thousands of patches (P9-O3). The record of such a network is
//! far past solidcore's 4 000 000 item budget for input records (about 40 000 patches); the kernel
//! builds it as a record of its own and checks it without the budget.
mod common;

use common::{cube_sphere, n_for, sphere_volume};
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
fn empty_and_oversized_networks_are_refused() {
    assert!(sheet_from_bezier_network_as_is(&[], &[]).is_err());
    let net = cube_sphere(2, RADIUS);
    let many: Vec<_> = net.patches.iter().cycle().take(MAX_NETWORK_PATCHES + 1).cloned().collect();
    let corners: Vec<_> = net.corners.iter().cycle().take(MAX_NETWORK_PATCHES + 1).copied().collect();
    assert!(sheet_from_bezier_network_as_is(&many, &corners).is_err());
}
