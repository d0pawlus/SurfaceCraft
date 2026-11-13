//! Tests of the hybrid model: solids built from cuts and plans, and the faces they leave.

use super::*;

use crate::nurbs::{fit_grid, sheet_from_spline};

fn wave_surface(x0: f64, x1: f64, y0: f64, y1: f64) -> Surface {
    let n = 24;
    let grid: Vec<Vec<[f64; 3]>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let x = x0 + (x1 - x0) * i as f64 / (n - 1) as f64;
                    let y = y0 + (y1 - y0) * j as f64 / (n - 1) as f64;
                    [x, y, 12.0 + 2.0 * (x / 10.0).sin() * (y / 8.0).cos()]
                })
                .collect()
        })
        .collect();
    fit_grid(&grid, 10, 10, 3).unwrap()
}

#[test]
fn plane_fillet_hole_and_freeform_make_one_solid() {
    // a 40 × 30 block (z 0..~12, wavy top) with filleted vertical edges r = 4 and a hole r = 3
    let mut cuts = vec![];
    for (p, n) in [
        ([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
        ([40.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ([0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
        ([0.0, 30.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
    ] {
        cuts.push(Cut { shape: Shape::Plane { point: p, normal: n }, convex: true, full: false });
    }
    for c in [[4.0, 4.0], [36.0, 4.0], [4.0, 26.0], [36.0, 26.0]] {
        cuts.push(Cut {
            shape: Shape::Cylinder { point: [c[0], c[1], 0.0], axis: [0.0, 0.0, 1.0], radius: 4.0 },
            convex: true,
            full: false,
        });
    }
    cuts.push(Cut {
        shape: Shape::Cylinder { point: [20.0, 15.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 3.0 },
        convex: false,
        full: true,
    });
    let top = wave_surface(-3.0, 43.0, -3.0, 33.0);
    let free = [FreeCut { surface: top, outward: [0.0, 0.0, 1.0] }];
    let built = solid_from_cuts([-6.0, -6.0, -3.0], [46.0, 36.0, 20.0], &cuts, &free).unwrap();
    for s in built.primitives.iter().chain(&built.freeform) {
        assert!(matches!(s, CutStatus::Applied(_)), "{s:?}");
    }
    // planes + 4 fillets + hole wall + freeform top
    assert_eq!(built.brep.faces.len(), 5 + 4 + 1 + 1, "faces");
    let v = solid::volume(&built.brep).unwrap();
    // footprint (rounded rectangle) minus the hole, times the mean height ≈ 12
    let area = 40.0 * 30.0 - (4.0 - std::f64::consts::PI) * 16.0 - std::f64::consts::PI * 9.0;
    assert!(v > area * 11.0 && v < area * 13.0, "volume {v}");
    // the faces map back to their shapes
    for c in &cuts {
        assert_eq!(faces_on_shape(&built.brep, &c.shape, 1e-3).len(), 1, "{:?}", c.shape);
    }
    // extract the top as a sheet
    let top_face = faces_on_spline(&built.brep, &free[0].surface, 0.05).unwrap();
    assert_eq!(top_face.len(), 1);
    let sheet = extract_face(&built.brep, top_face[0]).unwrap();
    assert_eq!(sheet.faces.len(), 1);
    // replace the flat top of a block by the same patch gives the same volume as the CAD cut
    let block = solid::make_box([0.0, 0.0, 0.0], [40.0, 30.0, 20.0]).unwrap();
    let top_idx = faces_on_shape(&block, &Shape::Plane { point: [0.0, 0.0, 20.0], normal: [0.0, 0.0, 1.0] }, 1e-6);
    assert_eq!(top_idx.len(), 1);
    let patch = wave_surface(-3.0, 43.0, -3.0, 33.0);
    let r = replace_face(&block, top_idx[0], &patch).unwrap();
    assert_eq!(r.faces.len(), 6);
    let rv = solid::volume(&r).unwrap();
    assert!(rv > 40.0 * 30.0 * 11.0 && rv < 40.0 * 30.0 * 13.0, "replaced volume {rv}");
    let _ = sheet_from_spline;
}

#[test]
fn revolved_part_cone_and_through_hole() {
    // cylinder r = 12, h = 30 with a 3 mm chamfer on the top edge and a through hole r = 4
    let cuts = vec![
        Cut { shape: Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, -1.0] }, convex: true, full: false },
        Cut { shape: Shape::Plane { point: [0.0, 0.0, 30.0], normal: [0.0, 0.0, 1.0] }, convex: true, full: false },
        Cut {
            shape: Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 12.0 },
            convex: true,
            full: true,
        },
        Cut {
            shape: Shape::Cone {
                apex: [0.0, 0.0, 39.0],
                axis: [0.0, 0.0, -1.0],
                half_angle: std::f64::consts::FRAC_PI_4,
            },
            convex: true,
            full: true,
        },
        Cut {
            shape: Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 4.0 },
            convex: false,
            full: true,
        },
    ];
    let built = solid_from_cuts([-20.0, -20.0, -5.0], [20.0, 20.0, 40.0], &cuts, &[]).unwrap();
    assert_eq!(built.brep.faces.len(), 5);
    let text = crate::step::export_solids(&[("p", &built.brep)]).unwrap();
    let back = crate::step::import(text.as_bytes()).unwrap();
    // one body back (a cone record that narrows along z is split into pieces by solidcore's STEP)
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), 5);
    let v = solid::volume(&built.brep).unwrap();
    let pi = std::f64::consts::PI;
    // cylinder minus the hole, minus the chamfer ring (a triangle 3 × 3 revolved at r = 11)
    let exact = pi * (144.0 - 16.0) * 30.0 - 2.0 * pi * 11.0 * 4.5;
    assert!((v - exact).abs() < 1e-3 * exact, "volume {v} vs {exact}");
}

fn box_planes(size: V) -> Vec<Cut> {
    let mut cuts = Vec::new();
    for (p, n) in [
        ([0.0; 3], [-1.0, 0.0, 0.0]),
        ([size[0], 0.0, 0.0], [1.0, 0.0, 0.0]),
        ([0.0; 3], [0.0, -1.0, 0.0]),
        ([0.0, size[1], 0.0], [0.0, 1.0, 0.0]),
        ([0.0; 3], [0.0, 0.0, -1.0]),
        ([0.0, 0.0, size[2]], [0.0, 0.0, 1.0]),
    ] {
        cuts.push(Cut { shape: Shape::Plane { point: p, normal: n }, convex: true, full: false });
    }
    cuts
}

fn circle_edges_at(b: &Brep, z: f64, radius: f64) -> Vec<usize> {
    use crate::record::brep::Curve;
    b.edges
        .iter()
        .enumerate()
        .filter(|(_, e)| match &e.curve {
            Curve::Circle { radius: r, .. } => {
                (r - radius).abs() < 1e-6 && (b.vertices[e.vertices[0] as usize].point[2] - z).abs() < 1e-6
            }
            _ => false,
        })
        .map(|(i, _)| i)
        .collect()
}

#[test]
fn equal_rounds_meet_in_a_corner_patch_and_a_boss_gets_foot_and_chamfer() {
    use crate::local::LocalAdd;
    let size = [60.0, 40.0, 8.0];
    let r = 5.0;
    let mut cuts = box_planes(size);
    let zc = size[2] - r;
    for c in [[r, r], [size[0] - r, r], [r, size[1] - r], [size[0] - r, size[1] - r]] {
        cuts.push(Cut {
            shape: Shape::Cylinder { point: [c[0], c[1], 0.0], axis: [0.0, 0.0, 1.0], radius: r },
            convex: true,
            full: false,
        });
        cuts.push(Cut { shape: Shape::Sphere { center: [c[0], c[1], zc], radius: r }, convex: true, full: false });
    }
    for y in [r, size[1] - r] {
        cuts.push(Cut {
            shape: Shape::Cylinder { point: [0.0, y, zc], axis: [1.0, 0.0, 0.0], radius: r },
            convex: true,
            full: false,
        });
    }
    for x in [r, size[0] - r] {
        cuts.push(Cut {
            shape: Shape::Cylinder { point: [x, 0.0, zc], axis: [0.0, 1.0, 0.0], radius: r },
            convex: true,
            full: false,
        });
    }
    // the CAD truth: one blend call on the eight edges
    let cad = {
        let b = solid::make_box([0.0; 3], size).unwrap();
        let edges: Vec<usize> = (0..b.edges.len())
            .filter(|&i| {
                let e = &b.edges[i];
                let (p, q) = (b.vertices[e.vertices[0] as usize].point, b.vertices[e.vertices[1] as usize].point);
                (p[2] - q[2]).abs() > 1.0 || ((p[2] - size[2]).abs() < 1e-9 && (q[2] - size[2]).abs() < 1e-9)
            })
            .collect();
        solid::fillet(&b, &edges, r).unwrap()
    };
    let built = solid_from_cuts([-5.0, -5.0, -5.0], [65.0, 45.0, 14.0], &cuts, &[]).unwrap();
    for s in &built.primitives {
        assert!(matches!(s, CutStatus::Applied(_)), "{s:?} in {:?}", built.primitives);
    }
    assert_eq!(built.brep.faces.len(), cad.faces.len());
    let (v, vc) = (solid::volume(&built.brep).unwrap(), solid::volume(&cad).unwrap());
    assert!((v - vc).abs() < 1e-6 * vc, "rounded plate volume {v} vs {vc}");
    for c in &cuts {
        if matches!(c.shape, Shape::Sphere { .. }) {
            assert_eq!(faces_on_shape(&built.brep, &c.shape, 1e-3).len(), 1, "corner patch {:?}", c.shape);
        }
    }

    // the boss: plate (sharp), a boss Ø16 up to z = 20, foot fillet r = 2, top chamfer 1.5
    let mut cuts = box_planes(size);
    cuts.push(Cut {
        shape: Shape::Torus {
            center: [30.0, 20.0, 10.0],
            axis: [0.0, 0.0, 1.0],
            major_radius: 10.0,
            minor_radius: 2.0,
        },
        convex: false,
        full: false,
    });
    // the chamfer's cone: radius 6.5 at z = 20 widening downwards, 45 degrees: apex at z = 26.5
    cuts.push(Cut {
        shape: Shape::Cone {
            apex: [30.0, 20.0, 26.5],
            axis: [0.0, 0.0, -1.0],
            half_angle: std::f64::consts::FRAC_PI_4,
        },
        convex: true,
        full: false,
    });
    let boss = LocalAdd::Boss { base: [30.0, 20.0, 8.0], axis: [0.0, 0.0, 1.0], radius: 8.0, height: 12.0, embed: 0.5 };
    let cad = {
        let b = solid::make_box([0.0; 3], size).unwrap();
        let cyl = solid::make_cylinder([30.0, 20.0, 7.0], 8.0, 13.0).unwrap();
        let u = crate::feature::boolean(&b, &cyl, crate::feature::BoolKind::Union).unwrap();
        let u = solid::fillet(&u, &circle_edges_at(&u, 8.0, 8.0), 2.0).unwrap();
        solid::chamfer(&u, &circle_edges_at(&u, 20.0, 8.0), 1.5).unwrap()
    };
    let built = solid_from_plan([-5.0, -5.0, -5.0], [65.0, 45.0, 30.0], &cuts, &[], &[], &[boss]).unwrap();
    assert!(matches!(built.adds[0], CutStatus::Applied(_)), "{:?}", built.adds);
    for s in &built.primitives {
        assert!(matches!(s, CutStatus::Applied(_)), "{s:?} in {:?}", built.primitives);
    }
    assert_eq!(built.brep.faces.len(), cad.faces.len(), "faces");
    let (v, vc) = (solid::volume(&built.brep).unwrap(), solid::volume(&cad).unwrap());
    assert!((v - vc).abs() < 1e-6 * vc, "boss volume {v} vs {vc}");
}

#[test]
fn concave_cylinder_fillets_a_pocket_corner_and_a_dome_stands_on_a_plane() {
    use crate::feature::ProfileCurve;
    use crate::local::{LocalAdd, LocalTool};
    let size = [60.0, 50.0, 20.0];
    let line = |a: [f64; 2], b: [f64; 2]| ProfileCurve::Line { start: [a[0], a[1], 12.0], end: [b[0], b[1], 12.0] };
    // a sharp 30 x 20 pocket, floor at z = 12
    let pocket = LocalTool::Prism {
        profile: vec![
            line([8.0, 8.0], [38.0, 8.0]),
            line([38.0, 8.0], [38.0, 28.0]),
            line([38.0, 28.0], [8.0, 28.0]),
            line([8.0, 28.0], [8.0, 8.0]),
        ],
        direction: [0.0, 0.0, 1.0],
        through: false,
        floor_fillet: None,
        islands: Vec::new(),
    };
    let mut cuts = box_planes(size);
    // a concave round r = 3 in the pocket's corner at (8, 8), tangent to both walls
    cuts.push(Cut {
        shape: Shape::Cylinder { point: [11.0, 11.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 3.0 },
        convex: false,
        full: false,
    });
    let dome = LocalAdd::Dome {
        center: [48.0, 38.0, 17.0],
        radius: 6.0,
        plane_point: [0.0, 0.0, 20.0],
        normal: [0.0, 0.0, 1.0],
    };
    let built =
        solid_from_plan([-5.0, -5.0, -5.0], [65.0, 55.0, 40.0], &cuts, &[], std::slice::from_ref(&pocket), &[dome])
            .unwrap();
    assert!(matches!(built.primitives.last(), Some(CutStatus::Applied("fillet"))), "{:?}", built.primitives);
    assert!(matches!(built.adds[0], CutStatus::Applied(_)), "{:?}", built.adds);
    let sharp =
        solid_from_plan([-5.0, -5.0, -5.0], [65.0, 55.0, 40.0], &box_planes(size), &[], &[pocket], &[]).unwrap();
    let pi = std::f64::consts::PI;
    let (vs, vb) = (solid::volume(&sharp.brep).unwrap(), solid::volume(&built.brep).unwrap());
    // the fillet adds (1 - pi/4) r^2 over the pocket's 8 mm depth; the dome is the cap of the
    // ball above z = 20: height 3, volume pi h^2 (3 R - h) / 3
    let fillet = (1.0 - pi / 4.0) * 9.0 * 8.0;
    let cap = pi * 9.0 * (18.0 - 3.0) / 3.0;
    assert!((vb - vs - fillet - cap).abs() < 1e-3, "fillet + dome: {} vs {}", vb - vs, fillet + cap);
    assert_eq!(built.brep.faces.len(), sharp.brep.faces.len() + 2, "a round and a dome");
}
#[test]
fn bad_input_is_refused() {
    assert!(solid_from_cuts([0.0; 3], [1.0; 3], &[], &[]).is_err());
    assert!(solid_from_cuts([f64::NAN, 0.0, 0.0], [1.0; 3], &[], &[]).is_err());
    let bad = Cut { shape: Shape::Sphere { center: [0.0; 3], radius: -1.0 }, convex: true, full: true };
    assert!(solid_from_cuts([0.0; 3], [1.0; 3], &[bad], &[]).is_err());
    let b = solid::make_box([0.0; 3], [1.0; 3]).unwrap();
    assert!(extract_face(&b, 99).is_err());
    assert!(replace_face(&b, 99, &wave_surface(0.0, 1.0, 0.0, 1.0)).is_err());
}

/// A 40 x 30 block with a wavy freeform top: a stock that is not a box.
fn wavy_block() -> Brep {
    let mut cuts = vec![];
    for (p, n) in [
        ([0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
        ([40.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ([0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
        ([0.0, 30.0, 0.0], [0.0, 1.0, 0.0]),
        ([0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
    ] {
        cuts.push(Cut { shape: Shape::Plane { point: p, normal: n }, convex: true, full: false });
    }
    let free = [FreeCut { surface: wave_surface(-3.0, 43.0, -3.0, 33.0), outward: [0.0, 0.0, 1.0] }];
    solid_from_cuts([-6.0, -6.0, -3.0], [46.0, 36.0, 20.0], &cuts, &free).unwrap().brep
}

#[test]
fn cut_stock_trims_a_given_solid_and_skips_what_the_kernel_cannot_do() {
    let stock = wavy_block();
    assert_eq!(analytic_faces(&stock).len(), stock.faces.len() - 1, "all faces but the wavy top are analytic");
    let plane =
        Cut { shape: Shape::Plane { point: [30.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] }, convex: true, full: false };
    let hole = Cut {
        shape: Shape::Cylinder { point: [12.0, 15.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 3.0 },
        convex: false,
        full: true,
    };
    let fillet = Cut {
        shape: Shape::Cylinder { point: [4.0, 4.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 4.0 },
        convex: true,
        full: false,
    };
    // a plane without a direction cannot be cut with
    let all_gone =
        Cut { shape: Shape::Plane { point: [0.0, 0.0, 5.0], normal: [0.0, 0.0, 0.0] }, convex: true, full: false };
    let built = cut_stock(&stock, &[plane.clone(), hole.clone(), fillet, all_gone], &[], &[]).unwrap();
    assert!(matches!(built.primitives[0], CutStatus::Applied("halfspace")), "{:?}", built.primitives[0]);
    assert!(matches!(built.primitives[1], CutStatus::Applied("subtract")), "{:?}", built.primitives[1]);
    assert!(
        matches!(&built.primitives[2], CutStatus::Skipped(why) if why.contains("blend")),
        "{:?}",
        built.primitives[2]
    );
    assert!(
        matches!(built.primitives[3], CutStatus::Skipped(_)),
        "a cut the kernel cannot make is skipped: {:?}",
        built.primitives[3]
    );
    assert!(built.freeform.is_empty());
    // the plane and the hole are faces of the result, the wavy top is still a spline
    assert_eq!(faces_on_shape(&built.brep, &plane.shape, 1e-3).len(), 1);
    assert_eq!(faces_on_shape(&built.brep, &hole.shape, 1e-3).len(), 1);
    assert_eq!(analytic_faces(&built.brep).len(), built.brep.faces.len() - 1);
    let v = solid::volume(&built.brep).unwrap();
    let before = solid::volume(&stock).unwrap();
    let expected = before * 30.0 / 40.0 - std::f64::consts::PI * 9.0 * 12.0;
    assert!((v - expected).abs() < 0.03 * expected, "volume {v}, expected about {expected}");
}

#[test]
fn cut_stock_with_nothing_applicable_is_an_error_and_bad_input_is_refused() {
    let stock = wavy_block();
    let all_gone =
        Cut { shape: Shape::Plane { point: [0.0, 0.0, 5.0], normal: [0.0, 0.0, 0.0] }, convex: true, full: false };
    assert!(cut_stock(&stock, &[all_gone], &[], &[]).is_err());
    let nan =
        Cut { shape: Shape::Plane { point: [f64::NAN, 0.0, 0.0], normal: [1.0, 0.0, 0.0] }, convex: true, full: false };
    assert!(cut_stock(&stock, &[nan], &[], &[]).is_err());
    assert!(cut_stock(&stock, &[], &[], &[]).is_err());
}
