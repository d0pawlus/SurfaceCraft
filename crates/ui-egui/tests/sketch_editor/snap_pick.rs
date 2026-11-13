//! Snapping (endpoints, midpoints, centres, grid, inference) and picking of entities and points.
use super::common::*;
use surfacecraft_sketch::{Entity, Sketch};
use surfacecraft_ui_egui::sketch_editor::snap::{self, Hit, Snap, SnapConfig, SnapKind};

#[test]
fn snapping_endpoints_midpoints_centres_grid_and_inference() {
    let sk = rect_sketch();
    let all = SnapConfig::default();
    let s = snap::snap_point(&sk, [10.2, 0.1], None, &[], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([10.0, 0.0], SnapKind::Endpoint));
    let s = snap::snap_point(&sk, [20.1, -0.2], None, &[], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([20.0, 0.0], SnapKind::Centre));
    let s = snap::snap_point(&sk, [5.2, 0.2], None, &[], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([5.0, 0.0], SnapKind::Midpoint));
    // the grid, when nothing else is near
    let grid_only = SnapConfig { points: false, midpoints: false, inference: false, grid: true };
    let s = snap::snap_point(&sk, [3.1, 7.4], None, &[], 0.5, 0.5, &grid_only);
    assert_eq!((s.pos, s.kind), ([3.0, 7.5], SnapKind::Grid));
    // horizontal and vertical inference from the last placed point
    let infer = SnapConfig { points: true, midpoints: false, inference: true, grid: false };
    let s = snap::snap_point(&sk, [30.0, 2.3], Some([2.0, 2.0]), &[], 0.5, 1.0, &infer);
    assert_eq!((s.pos, s.kind), ([30.0, 2.0], SnapKind::Horizontal));
    let s = snap::snap_point(&sk, [2.2, 40.0], Some([2.0, 2.0]), &[], 0.5, 1.0, &infer);
    assert_eq!((s.pos, s.kind), ([2.0, 40.0], SnapKind::Vertical));
    // an existing point beats the inference; the gesture's own points snap too (closing)
    let s = snap::snap_point(&sk, [10.1, 0.3], Some([2.0, 0.0]), &[], 0.5, 1.0, &infer);
    assert_eq!(s.kind, SnapKind::Endpoint);
    let s = snap::snap_point(&sk, [-30.1, -30.1], None, &[[-30.0, -30.0]], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([-30.0, -30.0], SnapKind::Endpoint));
    // out of reach and everything off: the cursor as it is
    let none = SnapConfig { points: false, midpoints: false, inference: false, grid: false };
    assert_eq!(snap::snap_point(&sk, [1.234, 5.678], None, &[], 0.5, 1.0, &none), Snap::free([1.234, 5.678]));
    // the grid step follows the zoom in 1-2-5 steps
    assert!((snap::grid_step(0.1, 14.0) - 2.0).abs() < 1e-12);
    assert!((snap::grid_step(1.0, 14.0) - 20.0).abs() < 1e-12);
    assert!((snap::grid_step(0.02, 14.0) - 0.5).abs() < 1e-12);
}

#[test]
fn picking_finds_lines_arcs_circles_and_points() {
    let sk = rect_sketch();
    assert_eq!(snap::pick(&sk, [5.0, 0.2], 0.5, snap::Want::Line), Some(Hit::Entity(0)));
    assert_eq!(snap::pick(&sk, [5.0, 2.5], 0.5, snap::Want::Line), None);
    assert_eq!(snap::pick(&sk, [23.1, 0.0], 0.5, snap::Want::Circular), Some(Hit::Entity(4)));
    assert_eq!(snap::pick(&sk, [10.1, 0.1], 0.5, snap::Want::Point), Some(Hit::Point(1)));
    // points win over the lines through them
    assert_eq!(snap::pick(&sk, [10.1, 0.1], 0.5, snap::Want::PointOrEntity), Some(Hit::Point(1)));
    // an arc is hit only along its sweep
    let mut a = Sketch::default();
    let (c, s, e) = (a.add_point([0.0, 0.0]), a.add_point([1.0, 0.0]), a.add_point([-1.0, 0.0]));
    a.entities.push(Entity::Arc { center: c, a: s, b: e });
    assert!(snap::entity_distance(&a, 0, [0.0, 1.0]) < 1e-9, "upper half is the arc");
    assert!(snap::entity_distance(&a, 0, [0.0, -1.0]) > 0.9, "lower half is not");
}
