//! Tests of the final net that keeps a conformed layout only when it is proper.
use super::super::{CmdError, CmdResult};
use super::field::*;

use surfacecraft_fit::orient::FlipReport;
use surfacecraft_fit::retopo::{QuadMesh, RetopoReport};
use surfacecraft_geom::{Point3, Vec3};

use surfacecraft_fit::retopo::{ConformReport, Layout};

/// A layout of `good` proper unit quads and `bad` reversed ones.
fn layout(good: usize, bad: usize, conformed: usize) -> FieldLayout {
    let mut quads = QuadMesh::default();
    for k in 0..good + bad {
        let o = 3.0 * k as f64;
        let mut q = [[o, 0.0], [o + 1.0, 0.0], [o + 1.0, 1.0], [o, 1.0]];
        if k >= good {
            q.reverse();
        }
        let base = quads.vertices.len() as u32;
        for p in q {
            quads.vertices.push(Point3::new(p[0], p[1], 0.0));
            quads.normals.push(Vec3::z());
        }
        quads.faces.push([base, base + 1, base + 2, base + 3]);
    }
    let boundary = ConformReport { loops: 1, conformed, ..ConformReport::default() };
    let rep = RetopoReport {
        layout: Layout::Field,
        edge_length: 1.0,
        work_triangles: 0,
        quads: good + bad,
        other_polygons: 0,
        irregular_fraction: 0.0,
        euler_ok: true,
        note: None,
        boundary,
    };
    FieldLayout {
        quads,
        rep,
        field_faces: good + bad,
        field_quads: good + bad,
        folded: 0,
        flips: FlipReport::default(),
        split_defects: 0,
    }
}

/// Conforming left one improper quad and the plain layout has one too (a tie): the plain layout
/// is returned, with the reason; no counts are compared.
#[test]
fn a_tie_of_improper_quads_returns_the_plain_layout() {
    let (out, _, reason) = net_conformed((layout(3, 1, 1), 0), || Ok((layout(3, 1, 0), 0))).unwrap();
    assert_eq!(reason, Some(NET_REASON));
    assert_eq!(out.rep.boundary.conformed, 0);
    assert!(!out.rep.boundary.all_conformed());
}

/// No improper quad: the conformed layout is kept and the plain pipeline is not even run.
#[test]
fn a_conformed_layout_without_improper_quads_is_kept() {
    let plain = || -> CmdResult<(FieldLayout, usize)> { Err(CmdError::Failed("must not run".into())) };
    let (out, _, reason) = net_conformed((layout(4, 0, 1), 0), plain).unwrap();
    assert_eq!(reason, None);
    assert!(out.rep.boundary.all_conformed());
}
