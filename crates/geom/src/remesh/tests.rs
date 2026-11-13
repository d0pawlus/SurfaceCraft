use super::*;
use crate::{Rng, Vec3, synth};

fn noisy(mut m: Mesh, sigma: f64, seed: u64) -> Mesh {
    m.weld(1e-9);
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

fn volume(m: &Mesh) -> f64 {
    signed_volume(m)
}

#[test]
fn a_noisy_sphere_becomes_a_closed_manifold_of_genus_zero_close_to_the_sphere() {
    let scan = noisy(synth::sphere(Point3::origin(), 15.0, 120, 60), 0.03, 3);
    let (out, rep) = remesh(&scan, &Params::default()).unwrap();
    let ps = pieces::pieces(&out);
    assert_eq!(ps.len(), 1);
    assert!(ps[0].closed());
    assert_eq!(ps[0].genus(), Some(0));
    let worst = out.positions.iter().map(|p| (p.coords.norm() - 15.0).abs()).fold(0.0, f64::max);
    assert!(worst < 0.1, "max deviation from the exact sphere {worst}; {rep:?}");
    let exact = 4.0 / 3.0 * std::f64::consts::PI * 15.0f64.powi(3);
    assert!((volume(&out) - exact).abs() / exact < 0.005, "{} vs {exact}", volume(&out));
}

#[test]
fn a_torus_keeps_its_handle() {
    let scan = noisy(synth::torus(Point3::origin(), Vec3::z(), 12.0, 4.0, 120, 48), 0.02, 4);
    let (out, _) = remesh(&scan, &Params::default()).unwrap();
    let ps = pieces::pieces(&out);
    assert_eq!(ps.len(), 1);
    assert_eq!(ps[0].genus(), Some(1));
    assert!(ps[0].closed());
}

#[test]
fn an_open_sheet_is_refused() {
    let sheet = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 30.0, 40, 40);
    let err = remesh(&sheet, &Params::default()).unwrap_err();
    assert!(matches!(err, RemeshError::Open { .. }), "{err}");
}

#[test]
fn a_resolution_beyond_the_limits_is_refused_before_memory_is_spent() {
    let scan = synth::sphere(Point3::origin(), 15.0, 80, 40);
    let tight = Params { limits: Limits { max_columns: 1000, ..Limits::default() }, ..Params::default() };
    assert!(matches!(remesh(&scan, &tight), Err(RemeshError::TooMuchWork { .. })));
    let fine = Params { voxel: Some(0.01), ..Params::default() };
    // refused from the area alone, before a single voxel is built
    match remesh(&scan, &fine) {
        Err(RemeshError::TooMuchWork { what, .. }) => assert!(what.contains("before any voxel is built"), "{what}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn the_result_is_the_same_for_one_and_eight_threads_and_for_two_runs() {
    let mut scan = noisy(synth::torus(Point3::origin(), Vec3::z(), 12.0, 4.0, 90, 40), 0.03, 6);
    let extra = scan.positions.len() as u32;
    scan.positions.extend([Point3::new(0.0, 0.0, 4.2), Point3::new(1.0, 0.0, 4.2), Point3::new(0.0, 1.0, 4.9)]);
    scan.triangles.push([extra, extra + 1, extra + 2]);
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| remesh(&scan, &Params::default()).unwrap().0)
    };
    let (one, eight, again) = (run(1), run(8), run(8));
    assert_eq!(one, eight, "1 and 8 threads");
    assert_eq!(eight, again, "two runs");
}

#[test]
fn the_refusal_does_not_depend_on_the_thread_count() {
    let scan = synth::sphere(Point3::origin(), 15.0, 80, 40);
    let p = Params { limits: Limits { max_crossings: 500, ..Limits::default() }, ..Params::default() };
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
        pool.install(|| remesh(&scan, &p).unwrap_err())
    };
    assert_eq!(run(1), run(8));
}

#[test]
fn absurd_voxels_and_tolerances_are_refused_not_overflowed() {
    let scan = synth::sphere(Point3::origin(), 15.0, 40, 20);
    for p in [
        Params { voxel: Some(1e-18), ..Params::default() },
        Params { tolerance: 1e-300, ..Params::default() },
        Params { voxel: Some(f64::MIN_POSITIVE), ..Params::default() },
    ] {
        assert!(
            matches!(remesh(&scan, &p), Err(RemeshError::TooMuchWork { .. } | RemeshError::ThinWall { .. })),
            "{p:?}"
        );
    }
}

#[test]
fn a_doubled_shell_is_one_shell_and_a_nearly_coincident_one_is_not_irregular() {
    let sphere = {
        let mut m = synth::sphere(Point3::origin(), 15.0, 80, 40);
        m.weld(1e-9);
        m
    };
    let mut doubled = sphere.clone();
    doubled.append(&sphere);
    let (out, rep) = remesh(&doubled, &Params::default()).unwrap();
    assert!(rep.duplicate_triangles > 0 && rep.irregular_columns == 0, "{rep:?}");
    assert_eq!(pieces::pieces(&out)[0].genus(), Some(0));
    let mut near = sphere.clone();
    let mut shifted = sphere.clone();
    shifted.positions.iter_mut().for_each(|p| *p += Vec3::new(1e-3, 0.0, 0.0));
    near.append(&shifted);
    let (_, rep) = remesh(&near, &Params::default()).unwrap();
    assert!((rep.irregular_columns as f64) < 0.05 * rep.columns as f64, "{rep:?}");
}

fn sphere_with_a_hole(radius: f64) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 15.0, 200, 100);
    m.weld(1e-9);
    let pole = Point3::new(0.0, 0.0, 15.0);
    let keep: Vec<bool> =
        (0..m.triangle_count()).map(|i| (m.face_centroid(i).unwrap() - pole).norm() > radius).collect();
    let mut k = keep.iter();
    m.triangles.retain(|_| *k.next().unwrap());
    m
}

#[test]
fn a_small_hole_with_more_edges_than_the_limit_is_refused_not_ignored() {
    let m = sphere_with_a_hole(2.2);
    let loops = pieces::pieces(&m)[0].loops.clone();
    assert!(loops.len() == 1 && loops[0].edges > 20 && loops[0].length / std::f64::consts::PI < 5.0, "{loops:?}");
    let few = Params { max_hole_edges: 20, ..Params::default() };
    assert!(matches!(remesh(&m, &few), Err(RemeshError::Open { .. })));
    let (out, rep) = remesh(&m, &Params::default()).unwrap();
    assert_eq!(rep.holes_filled, 1);
    assert!(pieces::pieces(&out)[0].closed());
}

#[test]
fn a_thin_plate_is_refined_and_an_explicit_coarse_voxel_is_refused() {
    let plate = {
        let mut p = synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, 0.7), 60);
        synth::add_noise(&mut p, 0.005, &mut Rng::new(3));
        p
    };
    let (out, rep) = remesh(&plate, &Params::default()).unwrap();
    assert!(rep.voxel_refined && rep.voxel < 0.3, "{rep:?}");
    let n = out.vertex_normals();
    let mean = |up: bool| {
        let z: Vec<f64> = out
            .positions
            .iter()
            .zip(&n)
            .filter(|(p, v)| {
                (if up { v.z > 0.95 } else { v.z < -0.95 }) && p.x > 5.0 && p.x < 25.0 && p.y > 5.0 && p.y < 25.0
            })
            .map(|(p, _)| p.z)
            .collect();
        z.iter().sum::<f64>() / z.len().max(1) as f64
    };
    assert!((mean(true) - mean(false) - 0.7).abs() < 0.05, "{} {}", mean(true), mean(false));
    let coarse = Params { voxel: Some(0.3), ..Params::default() };
    assert!(matches!(remesh(&plate, &coarse), Err(RemeshError::ThinWall { .. })));
}

#[test]
fn an_axis_aligned_cube_has_no_ray_trouble_and_the_report_says_what_it_measured() {
    let cube = synth::cuboid(Point3::new(0.0, 0.0, 0.0), Vec3::new(20.0, 20.0, 20.0), 10);
    let (out, rep) = remesh(&cube, &Params::default()).unwrap();
    let ps = pieces::pieces(&out);
    assert!(ps[0].closed() && ps[0].genus() == Some(0));
    assert!((volume(&out) - 8000.0).abs() / 8000.0 < 0.02, "{}", volume(&out));
    assert!(rep.original_to_result.exact && rep.result_to_original.exact);
    assert_eq!(rep.self_intersections, 0, "{rep:?}");
    assert!(rep.self_intersection_samples > 0);
}

#[test]
fn a_removed_fin_shows_in_the_removed_share_and_the_exact_maximum() {
    let mut m = synth::sphere(Point3::origin(), 15.0, 80, 40);
    m.weld(1e-9);
    let n = m.positions.len() as u32;
    m.positions.push(Point3::new(0.2, 0.0, 15.9));
    let t = m.triangles[0];
    m.triangles.push([t[1], t[0], n]);
    let (_, rep) = remesh(&m, &Params::default()).unwrap();
    assert!(rep.original_to_result.exact);
    assert!(rep.original_to_result.max > 0.5, "{rep:?}");
    assert!(rep.removed_share > 0.0 && rep.removed_share < 0.01, "{}", rep.removed_share);
}

#[test]
fn even_indices_do_not_wrap_in_32_bits() {
    assert_eq!(crate::even_index(65_535, 100_000, 65_536), 99_998);
    assert_eq!(crate::even_index(0, 0, 0), 0);
}

#[test]
fn a_coarse_voxel_or_a_huge_tolerance_never_panics() {
    let mut big = synth::sphere(Point3::origin(), 150.0, 80, 40);
    big.weld(1e-9);
    for p in [
        Params { voxel: Some(5.0), ..Params::default() },
        Params { tolerance: 3.0, ..Params::default() },
        Params { voxel: Some(50.0), ..Params::default() },
        Params { tolerance: 1e300, ..Params::default() },
        Params { voxel: Some(1e300), ..Params::default() },
    ] {
        let r = std::panic::catch_unwind(|| remesh(&big, &p));
        assert!(r.is_ok(), "{p:?} panicked");
    }
    let (out, rep) = remesh(&big, &Params { voxel: Some(5.0), ..Params::default() }).unwrap();
    assert!(pieces::pieces(&out)[0].closed(), "{rep:?}");
}

#[test]
fn a_missing_triangle_wider_than_the_hole_limit_is_refused_not_drilled() {
    let mut m = synth::sphere(Point3::origin(), 40.0, 30, 15);
    m.weld(1e-9);
    m.triangles.remove(m.triangle_count() / 2);
    let loops = pieces::pieces(&m)[0].loops.clone();
    assert_eq!(loops.len(), 1);
    assert_eq!(loops[0].edges, 3);
    assert!(loops[0].length / std::f64::consts::PI > 5.0, "{loops:?}");
    assert!(matches!(remesh(&m, &Params::default()), Err(RemeshError::Open { .. })));
}

#[test]
fn the_quad_estimate_refuses_only_far_beyond_the_limit_and_the_real_count_decides_after() {
    let mut m = synth::sphere(Point3::origin(), 15.0, 80, 40);
    m.weld(1e-9);
    // area / h² is about 31 000 here: above this limit, below four times it
    let p = Params { limits: Limits { max_quads: 20_000, ..Limits::default() }, ..Params::default() };
    match remesh(&m, &p) {
        Err(RemeshError::TooMuchWork { what, .. }) => assert!(!what.contains("estimated"), "{what}"),
        other => panic!("{other:?}"),
    }
}

#[test]
fn a_double_faced_fin_is_kept_as_a_closed_sheet_and_removed_by_the_closing() {
    let mut m = synth::sphere(Point3::origin(), 15.0, 80, 40);
    m.weld(1e-9);
    let pole = m.positions.iter().position(|p| p.z > 14.99).unwrap() as u32;
    let n = m.positions.len() as u32;
    m.positions.extend([Point3::new(1.0, 0.0, 16.5), Point3::new(0.0, 1.0, 16.5)]);
    m.triangles.extend([[pole, n, n + 1], [pole, n + 1, n]]);
    assert!(pieces::pieces(&m)[0].loops.is_empty(), "both faces kept: no boundary");
    let (out, rep) = remesh(&m, &Params::default()).unwrap();
    assert_eq!(rep.duplicate_triangles, 0);
    assert_eq!(pieces::pieces(&out)[0].genus(), Some(0));
}

#[test]
fn more_columns_than_the_index_can_hold_are_refused_whatever_the_limit() {
    let mut m = synth::sphere(Point3::origin(), 7000.0, 40, 20);
    m.weld(1e-9);
    let p = Params {
        voxel: Some(0.3),
        limits: Limits { max_columns: usize::MAX, ..Limits::default() },
        ..Params::default()
    };
    match remesh(&m, &p) {
        Err(RemeshError::TooMuchWork { what, .. }) => assert!(what.contains("grid columns"), "{what}"),
        other => panic!("{other:?}"),
    }
}
