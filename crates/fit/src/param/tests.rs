use super::*;
use surfacecraft_geom::{Point3, synth};

/// A flat grid of `n × n` cells of 1 mm, only the cells for which `keep` is true.
fn grid_mesh(n: usize, keep: impl Fn(usize, usize) -> bool) -> Mesh {
    let mut positions = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            positions.push(Point3::new(i as f64, j as f64, 0.0));
        }
    }
    let w = (n + 1) as u32;
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            if keep(i, j) {
                let a = j as u32 * w + i as u32;
                triangles.push([a, a + 1, a + w + 1]);
                triangles.push([a, a + w + 1, a + w]);
            }
        }
    }
    Mesh { positions, triangles }
}

fn all_ids(mesh: &Mesh) -> Vec<u32> {
    (0..mesh.triangles.len() as u32).collect()
}

#[test]
fn plane_is_isometric() {
    let mesh = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 30.0, 40, 30);
    let f = flatten(&mesh, &all_ids(&mesh), 200).unwrap();
    assert_eq!(f.flipped, 0);
    assert!(f.distortion_max <= 1e-6, "distortion_max {}", f.distortion_max);
    assert_eq!(f.uv.len(), mesh.positions.len());
    assert!(f.uv.iter().all(Option::is_some));
    assert_eq!(f.boundary.len(), 2 * (40 + 30));
    // counter-clockwise in (u, v)
    let poly: Vec<[f64; 2]> = f.boundary.iter().filter_map(|&v| f.uv[v as usize]).collect();
    let twice: f64 = (0..poly.len())
        .map(|i| {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    assert!((twice / 2.0 - 1200.0).abs() < 1e-3, "boundary area {}", twice / 2.0);
}

#[test]
fn cylinder_patch_unrolls() {
    // 120 degrees of the 36 columns of an open cylinder R = 20, h = 30
    let (nu, nv) = (36usize, 12usize);
    let mesh = synth::cylinder(Point3::origin(), Vec3::z(), 20.0, 30.0, nu, nv);
    let ids: Vec<u32> = (0..nv)
        .flat_map(|j| (0..nu / 3).flat_map(move |i| [(2 * (j * nu + i)) as u32, (2 * (j * nu + i) + 1) as u32]))
        .collect();
    let f = flatten(&mesh, &ids, 100).unwrap();
    assert_eq!(f.flipped, 0);
    assert!(f.distortion_mean <= 0.02, "distortion_mean {}", f.distortion_mean);
}

#[test]
fn sphere_cap() {
    let r = 20.0;
    let mesh = synth::sphere(Point3::origin(), r, 48, 36);
    // the cap of half-angle 40 degrees around the south pole (z = -r)
    let limit = -r * 40f64.to_radians().cos() + 1e-9;
    let ids: Vec<u32> = (0..mesh.triangles.len())
        .filter(|&t| mesh.triangles[t].iter().all(|&v| mesh.positions[v as usize].z <= limit))
        .map(|t| t as u32)
        .collect();
    assert!(!ids.is_empty());
    let f = flatten(&mesh, &ids, 100).unwrap();
    assert_eq!(f.flipped, 0);
    assert!(f.distortion_mean <= 0.08, "distortion_mean {}", f.distortion_mean);
}

#[test]
fn l_shape_is_bijective() {
    // a concave planar L: a 20 × 20 square without its upper right quarter
    let mesh = grid_mesh(20, |i, j| !(i >= 10 && j >= 10));
    let f = flatten(&mesh, &all_ids(&mesh), 500).unwrap();
    assert_eq!(f.flipped, 0);
    assert!(f.distortion_max <= 1e-6, "distortion_max {}", f.distortion_max);
}

#[test]
fn harmonic_map_alone_is_bijective() {
    let mesh = grid_mesh(20, |i, j| !(i >= 10 && j >= 10));
    let f = flatten(&mesh, &all_ids(&mesh), 0).unwrap();
    assert_eq!(f.flipped, 0);
    assert!(f.uv.iter().flatten().all(|p| p[0].is_finite() && p[1].is_finite()));
}

#[test]
fn refuses_two_loops() {
    // a square ring: 12 × 12 cells without the middle 4 × 4
    let mesh = grid_mesh(12, |i, j| !((4..8).contains(&i) && (4..8).contains(&j)));
    let e = flatten(&mesh, &all_ids(&mesh), 10).unwrap_err();
    assert!(e.contains("boundary loops"), "{e}");
}

#[test]
fn refuses_closed_surface() {
    let mesh = synth::sphere(Point3::origin(), 10.0, 24, 12);
    let e = flatten(&mesh, &all_ids(&mesh), 10).unwrap_err();
    assert!(e.contains("closed"), "{e}");
}

#[test]
fn refuses_non_finite() {
    let mut mesh = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 4.0, 4.0, 4, 4);
    mesh.positions[7] = Point3::new(f64::NAN, 0.0, 0.0);
    let e = flatten(&mesh, &all_ids(&mesh), 10).unwrap_err();
    assert!(e.contains("non-finite"), "{e}");
}

#[test]
fn empty_triangles() {
    let mesh = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 4.0, 4.0, 4, 4);
    assert!(flatten(&mesh, &[], 10).is_err());
    assert!(flatten(&Mesh::default(), &[], 10).is_err());
}

#[test]
fn hostile_ids_and_topology_are_errors() {
    let mesh = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 4.0, 4.0, 4, 4);
    assert!(flatten(&mesh, &[u32::MAX], 10).is_err());
    // a vertex index out of range
    let bad = Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 9]] };
    assert!(flatten(&bad, &[0], 10).is_err());
    // a triangle that repeats a vertex, and one of zero area
    let rep = Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 0, 1]] };
    assert!(flatten(&rep, &[0], 10).is_err());
    let flat = Mesh {
        positions: vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)],
        triangles: vec![[0, 1, 2]],
    };
    assert!(flatten(&flat, &[0], 10).is_err());
    // three triangles on one edge
    let fan = Mesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ],
        triangles: vec![[0, 1, 2], [1, 0, 3], [0, 1, 4]],
    };
    assert!(flatten(&fan, &[0, 1, 2], 10).is_err());
    // a single triangle flattens
    let one = Mesh {
        positions: vec![Point3::origin(), Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 4.0, 0.0)],
        triangles: vec![[0, 1, 2]],
    };
    let f = flatten(&one, &[0], 20).unwrap();
    assert_eq!(f.flipped, 0);
    assert!(f.distortion_max <= 1e-9, "{}", f.distortion_max);
}
