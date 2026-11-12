//! Review fixes of the compact solid writer: closedness, knots, extra shells.
use super::network_fixture::closed_network;
use super::{IoError, write_sheets, write_solids};
use surfacecraft_kernel::record::brep::{Brep, Curve, Shell, Surface};
use surfacecraft_kernel::solid;

fn faces_of(b: &mut Brep) -> &mut Vec<surfacecraft_kernel::record::brep::OrientedFace> {
    match b.shells.first_mut() {
        Some(Shell::Faces { faces, .. }) => faces,
        _ => panic!("no face shell"),
    }
}

fn is_unsupported(r: Result<String, IoError>) -> bool {
    matches!(r, Err(IoError::Unsupported(_)))
}

#[test]
fn a_shell_missing_a_face_is_not_written_as_closed() {
    let mut b = solid::make_box([0.0; 3], [1.0, 2.0, 3.0]).unwrap();
    faces_of(&mut b).pop();
    assert!(is_unsupported(write_solids("x", &[&b])));
}

#[test]
fn a_shell_with_a_flipped_face_is_not_written_as_closed() {
    let mut b = solid::make_box([0.0; 3], [1.0, 2.0, 3.0]).unwrap();
    let f = &mut faces_of(&mut b)[2];
    f.forward = !f.forward;
    assert!(is_unsupported(write_solids("x", &[&b])));
}

/// Two boxes in one record and one shell.
fn two_components() -> Brep {
    let mut a = solid::make_box([0.0; 3], [1.0; 3]).unwrap();
    let b = solid::make_box([5.0, 0.0, 0.0], [1.0; 3]).unwrap();
    let (fo, eo, vo) = (a.faces.len() as u32, a.edges.len() as u32, a.vertices.len() as u32);
    let mut extra = b.clone();
    for f in &mut extra.faces {
        for l in &mut f.loops {
            if let surfacecraft_kernel::record::brep::Loop::Edges { edges } = l {
                edges.iter_mut().for_each(|u| u.edge += eo);
            }
        }
    }
    for e in &mut extra.edges {
        e.vertices.iter_mut().for_each(|v| *v += vo);
        e.uses.iter_mut().for_each(|u| u.face += fo);
    }
    let Some(Shell::Faces { faces: more, .. }) = extra.shells.first().cloned() else { panic!() };
    a.faces.extend(extra.faces);
    a.edges.extend(extra.edges);
    a.vertices.extend(extra.vertices);
    faces_of(&mut a).extend(more.into_iter().map(|mut f| {
        f.face += fo;
        f
    }));
    a
}

#[test]
fn a_shell_of_two_disjoint_components_goes_to_the_fallback() {
    assert!(is_unsupported(write_solids("x", &[&two_components()])));
}

#[test]
fn consistent_closed_shells_still_write() {
    assert!(write_solids("x", &[&solid::make_box([0.0; 3], [1.0; 3]).unwrap()]).is_ok());
    assert!(write_solids("x", &[&closed_network(2)]).is_ok());
}

#[test]
fn a_body_with_an_extra_shell_is_not_written_losing_it() {
    let mut b = solid::make_box([0.0; 3], [1.0; 3]).unwrap();
    b.shells.push(Shell::Vertex { vertex: 0 });
    let extra = (b.shells.len() - 1) as u32;
    b.regions[0].shells.push(extra);
    assert!(is_unsupported(write_solids("x", &[&b])));
}

/// A bicubic patch of the network reparameterised onto `[0, scale]` in u.
fn squeezed(scale: f64) -> Brep {
    let mut b = closed_network(1);
    if let Surface::Spline { u_knots, .. } = &mut b.faces[0].surface {
        u_knots.iter_mut().for_each(|k| *k *= scale);
    }
    b
}

#[test]
fn distinct_knots_are_never_merged() {
    for scale in [1e-15, 1e-13] {
        let b = squeezed(scale);
        match write_solids("x", &[&b]) {
            Ok(text) => {
                let line = text.lines().find(|l| l.contains("B_SPLINE_SURFACE_WITH_KNOTS")).unwrap();
                assert!(line.contains(",(4,4),"), "multiplicities written: {line}");
            }
            Err(e) => assert!(matches!(e, IoError::Export(_)), "{e}"),
        }
    }
}

#[test]
fn near_equal_interior_knots_stay_distinct_and_valid() {
    use super::geom::{checked_knots, knots_of};
    let near = [0.0, 0.0, 0.0, 0.0, 0.5, 0.5 + 1e-16 + f64::EPSILON, 1.0, 1.0, 1.0, 1.0];
    let (k, m) = knots_of(&near);
    assert_eq!((k.len(), m.iter().sum::<usize>()), (4, 10));
    assert!(checked_knots(&near, 3).is_ok());
    let tiny = [0.0, 0.0, 0.0, 0.0, 1e-15, 1e-15, 1e-15, 1e-15];
    assert_eq!(checked_knots(&tiny, 3).unwrap(), (vec![0.0, 1e-15], vec![4, 4]));
    // a knot repeated more often than the degree allows is an error
    // a full break inside is a valid record (the kernel accepts it); one more repeat is not
    assert!(checked_knots(&[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0], 3).is_ok());
    assert!(checked_knots(&[0.0, 0.0, 0.5, 0.5, 1.0, 1.0], 1).is_ok());
    assert!(checked_knots(&[0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5, 0.5, 1.0, 1.0, 1.0, 1.0], 3).is_err());
    assert!(checked_knots(&[0.0; 5], 3).is_err());
}

#[test]
fn sheets_with_squeezed_knots_are_errors_or_valid_files() {
    let mut b = closed_network(1);
    b.regions.truncate(1);
    if let Surface::Spline { u_knots, .. } = &mut b.faces[0].surface {
        u_knots.iter_mut().for_each(|k| *k *= 1e-15);
    }
    if let Ok(t) = write_sheets("x", &[&b]) {
        assert!(!t.contains(",(8),"));
    }
    let _ = Curve::Line { start: [0.0; 3], end: [1.0; 3] };
}
