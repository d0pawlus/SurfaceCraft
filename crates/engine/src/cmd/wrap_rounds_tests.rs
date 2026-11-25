use super::*;
use surfacecraft_geom::{Rng, synth};

const TOLERANCE: f64 = 0.2;
const BUDGET: f64 = 1.6;

fn options(g1: bool) -> G1Options {
    G1Options { g1, crease_deg: 35.0 }
}

#[test]
fn a_network_asked_to_be_tangent_is_searched_without_the_refit_or_the_stop_margin() {
    // Arrange: a noisy ball and a coarse mesh of it
    let mut scan = synth::sphere(Point3::origin(), 20.0, 120, 60);
    synth::add_noise(&mut scan, 0.03, &mut Rng::new(9));
    let coarse = reduce_scan(&scan, BUDGET, &[]);
    let tangent = Target::new(&scan, TOLERANCE, 0.99, 12_000, options(true));
    let free = Target::new(&scan, TOLERANCE, 0.99, 12_000, options(false));

    // Act: a round of the layout search of each (the search lays unconstrained patches)
    let (searched_tangent, _) = build_round(&tangent, &coarse, BUDGET, false);
    let (searched_free, _) = build_round(&free, &coarse, BUDGET, false);
    let reach = REACH_BUDGETS * BUDGET;
    let plain = wrap_patches_g1_refit(&coarse, &tangent.bvh, &tangent.surface, reach, &options(false), false);

    // Assert: the tangent network's search is the plain construction; the unconstrained one is refitted
    assert_eq!(searched_tangent.set.nets, plain.set.nets);
    assert_ne!(searched_free.set.nets, plain.set.nets, "the refit changes the unconstrained search");
    assert_eq!(tangent.stop_within, 0.99, "no stop margin for a tangent network");
    assert!(free.stop_within > 0.99);
}
