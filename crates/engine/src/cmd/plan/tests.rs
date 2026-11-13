//! Tests of the planner: plans of synthetic scans and their regions.

use super::*;

use std::f64::consts::PI;

/// Box 100 x 40 x 10 as six plane items; the top plane is tilted by `tilt_deg` about y.
fn slab(tilt_deg: f64) -> (Vec<Item>, Scan) {
    let t = tilt_deg.to_radians();
    let grid = |f: &dyn Fn(f64, f64) -> [f64; 3]| -> Vec<Point3> {
        let mut v = Vec::new();
        for i in 0..=20 {
            for j in 0..=20 {
                let p = f(i as f64 / 20.0, j as f64 / 20.0);
                v.push(Point3::new(p[0], p[1], p[2]));
            }
        }
        v
    };
    let top_n = [-t.sin(), 0.0, t.cos()];
    let faces: Vec<([f64; 3], Vec<Point3>)> = vec![
        ([0.0, 0.0, -1.0], grid(&|u, v| [100.0 * u, 40.0 * v, 0.0])),
        // the tilted top: z = 10 + (x - 50) tan t
        (top_n, grid(&|u, v| [100.0 * u, 40.0 * v, 10.0 + (100.0 * u - 50.0) * t.tan()])),
        ([-1.0, 0.0, 0.0], grid(&|u, v| [0.0, 40.0 * u, (10.0 - 50.0 * t.tan()) * v])),
        ([1.0, 0.0, 0.0], grid(&|u, v| [100.0, 40.0 * u, (10.0 + 50.0 * t.tan()) * v])),
        ([0.0, -1.0, 0.0], grid(&|u, v| [100.0 * u, 0.0, (10.0 + (100.0 * u - 50.0) * t.tan()) * v])),
        ([0.0, 1.0, 0.0], grid(&|u, v| [100.0 * u, 40.0, (10.0 + (100.0 * u - 50.0) * t.tan()) * v])),
    ];
    let scan: Vec<Point3> = faces.iter().flat_map(|f| f.1.iter().copied()).collect();
    let items = faces
        .into_iter()
        .enumerate()
        .map(|(i, (n, pts))| {
            let c: Vec3 = pts.iter().map(|p| p.coords).sum::<Vec3>() / pts.len() as f64;
            Item {
                id: ObjectId(i as u64),
                kind: "plane",
                cut: Cut { shape: Shape::Plane { point: arr(c), normal: n }, convex: true, full: false },
                pts,
            }
        })
        .collect();
    (items, Scan::from_points(scan, 1.0))
}

#[test]
fn a_real_draft_on_a_long_face_is_not_squared() {
    // 0.2 degrees over 50 mm to either side of the centroid: 0.17 mm, above max(2 sigma, 0.05)
    let (items, scan) = slab(0.2);
    let p = plan(&items, &scan, 0.3);
    let top = p.squared.iter().find(|q| q.item == 1).expect("the top plane was considered");
    assert!(!top.applied && top.max_shift > 0.15 && top.limit < 0.06, "{top:?}");
    assert!(p.roles[1].how.contains("not squared"), "{}", p.roles[1].how);
    assert!(matches!(&p.roles[1].shape, Shape::Plane { normal, .. } if normal[0].abs() > 1e-3));
    // the report carries it
    let r = report(&items, &p);
    assert_eq!(r.squared_planes.iter().filter(|q| !q.applied).count(), 1);
    // a hair of tilt is squared; zero disables
    let (items, scan) = slab(0.005);
    let p = plan(&items, &scan, 0.3);
    assert!(p.squared.iter().any(|q| q.item == 1 && q.applied), "{:?}", p.squared);
    assert!(matches!(&p.roles[1].shape, Shape::Plane { normal, .. } if normal[0] == 0.0));
    assert!(plan(&items, &scan, 0.0).squared.is_empty());
}

/// A scan of `pts` with a floor under the walls (which the group of walls does not know).
fn walls_with_a_round_and_a_floor() -> (Vec<Item>, Scan) {
    let grid = |f: &dyn Fn(f64, f64) -> [f64; 3]| -> Vec<Point3> {
        (0..=20)
            .flat_map(|i| (0..=20).map(move |j| (i as f64 / 20.0, j as f64 / 20.0)))
            .map(|(u, v)| {
                let p = f(u, v);
                Point3::new(p[0], p[1], p[2])
            })
            .collect()
    };
    let wall_x = grid(&|u, v| [10.0, 10.0 + 20.0 * u, 4.0 + 4.0 * v]);
    let wall_y = grid(&|u, v| [10.0 + 20.0 * u, 10.0, 4.0 + 4.0 * v]);
    // the convex round of the plate's corner, a little off the walls' corner
    let round: Vec<Point3> = (0..=20)
        .flat_map(|i| (0..=10).map(move |j| (i as f64 / 20.0, j as f64 / 10.0)))
        .map(|(u, v)| {
            let a = PI * (1.0 + 0.5 * u);
            Point3::new(8.0 + 2.0 * a.cos(), 8.0 + 2.0 * a.sin(), 4.0 + 4.0 * v)
        })
        .collect();
    // the floor the walls stand on (not among the regions)
    let floor = grid(&|u, v| [10.0 + 20.0 * u, 10.0 + 20.0 * v, 4.0]);
    let item = |id: u64, kind: &'static str, shape: Shape, convex: bool, pts: Vec<Point3>| Item {
        id: ObjectId(id),
        kind,
        cut: Cut { shape, convex, full: false },
        pts,
    };
    let items = vec![
        item(0, "plane", Shape::Plane { point: [10.0, 20.0, 6.0], normal: [1.0, 0.0, 0.0] }, true, wall_x.clone()),
        item(1, "plane", Shape::Plane { point: [20.0, 10.0, 6.0], normal: [0.0, 1.0, 0.0] }, true, wall_y.clone()),
        item(
            2,
            "cylinder",
            Shape::Cylinder { point: [8.0, 8.0, 6.0], axis: [0.0, 0.0, 1.0], radius: 2.0 },
            true,
            round.clone(),
        ),
    ];
    let scan: Vec<Point3> = [wall_x, wall_y, round, floor].concat();
    (items, Scan::from_points(scan, 1.0))
}

#[test]
fn a_failed_slot_group_leaves_a_convex_round_its_fillet_role() {
    let (items, scan) = walls_with_a_round_and_a_floor();
    let p = plan(&items, &scan, 0.3);
    // the walls have scan inside them (their floor): no slot, no step
    assert!(p.tools.is_empty(), "{:?}", p.tools.iter().map(|t| t.kind).collect::<Vec<_>>());
    assert_eq!(p.roles[0].kind, RoleKind::Skipped, "{}", p.roles[0].how);
    assert_eq!(p.roles[1].kind, RoleKind::Skipped, "{}", p.roles[1].how);
    // the convex round was in the group, but is not lost with it
    assert_eq!(p.roles[2].kind, RoleKind::Blend, "{}", p.roles[2].how);
}
