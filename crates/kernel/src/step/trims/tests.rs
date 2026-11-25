//! The kernel's trims: the budget, and the interval of an edge the kernel moved.
use super::*;
use crate::record::brep::{Curve, Surface};
use solidcore_ops::pcurve::TrimUse;

fn line() -> Curve {
    Curve::Line { start: [0.0, 0.0, 0.0], end: [1.0, 0.0, 0.0] }
}

fn plane() -> Surface {
    Surface::Plane { origin: [0.0; 3], u_axis: [1.0, 0.0, 0.0], v_axis: [0.0, 1.0, 0.0] }
}

#[test]
fn past_the_budget_the_kernel_is_not_asked() {
    let (curve, surface) = (line(), plane());
    let edge = TrimEdge { curve: &curve, nurbs: None, interval: [0.0, 1.0], vertices: [0, 1], tolerance: 1e-6 };
    let face = TrimFace {
        surface: &surface,
        nurbs: None,
        domain: [[0.0, 1.0], [0.0, 1.0]],
        edges: vec![edge],
        vertices: vec![[0.0; 3], [1.0, 0.0, 0.0]],
        vertex_tolerances: Vec::new(),
        loops: vec![vec![TrimUse { edge: 0, forward: true, pcurve: None }]],
        tolerance: 1e-6,
    };
    let mut k = TrimKernel::new();
    k.uses = MAX_TRIM_USES;
    let e = k.trims(&face).unwrap_err().to_string();
    assert!(e.contains("edge uses"), "{e}");
}

#[test]
fn an_interval_the_kernel_moved_within_the_tolerance_is_mapped_back_and_farther_is_refused() {
    let curve = line();
    let edge = TrimEdge { curve: &curve, nurbs: None, interval: [0.0, 1.0], vertices: [0, 1], tolerance: 1e-3 };
    let pc = Curve2::Spline { knots: vec![0.0005, 0.0005, 0.9995, 0.9995], points: vec![[0.0, 0.0], [1.0, 0.0]] };
    let Curve2::Spline { knots, .. } = on_our_interval(pc.clone(), &edge, [0.0005, 0.9995]).unwrap() else {
        panic!("a spline")
    };
    assert_eq!(knots, vec![0.0, 0.0, 1.0, 1.0]);
    assert!(on_our_interval(pc, &edge, [0.1, 0.9]).is_err());
}
