//! Tests of the relation constraints: found relations and imposed positions on synthetic primitives.

use super::*;

fn plane(z: f64) -> Primitive {
    Primitive::Plane { point: [0.0, 0.0, z], normal: [0.0, 0.0, 1.0] }
}

fn z_of(p: &Primitive) -> f64 {
    match p {
        Primitive::Plane { point, .. } => point[2],
        _ => f64::NAN,
    }
}

#[test]
fn three_planes_become_equally_spaced() {
    let mut prims = vec![plane(0.0), plane(10.02), plane(19.99)];
    let rels = find_relations(&prims, &[], 3.0, 0.2);
    assert_eq!(rels.iter().filter(|r| r.kind == RelKind::EqualDistance).count(), 1);
    apply(&mut prims, &[], &rels).unwrap();
    let z: Vec<f64> = prims.iter().map(z_of).collect();
    assert!(((z[1] - z[0]) - (z[2] - z[1])).abs() < 1e-9, "{z:?}");
    // the smallest change: the total moves are far below the 0.2 mm tolerance
    assert!((z[0]).abs() < 0.05 && (z[2] - 19.99).abs() < 0.05, "{z:?}");
}

#[test]
fn plane_touches_sphere() {
    let mut prims = vec![plane(0.0), Primitive::Sphere { center: [1.0, 2.0, 5.02], radius: 5.0 }];
    let rels = find_relations(&prims, &[], 3.0, 0.2);
    assert_eq!(rels.iter().filter(|r| r.kind == RelKind::Tangent).count(), 1);
    apply(&mut prims, &[], &rels).unwrap();
    let (m, _) = measure(&prims, &[], &rels[0]).unwrap();
    assert!(m < 1e-9, "{m}");
}
