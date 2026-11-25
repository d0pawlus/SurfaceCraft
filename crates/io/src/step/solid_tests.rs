//! Gates of the compact solid file (P9-O5), now written by solidcore's STEP writer: size per patch,
//! shared entities, round trip through solidcore's importer, hostile records.
use super::network_fixture::closed_network;
use super::{write_sheets, write_solids};
use std::collections::BTreeMap;
use surfacecraft_kernel::record::brep::{Brep, Surface};
use surfacecraft_kernel::solid;

/// Bytes allowed per bicubic patch (P9-O5 design: at most 6 KB; solidcore's old `step-export` needed ~24 KB).
const MAX_BYTES_PER_PATCH: usize = 6 * 1024;
/// Relative volume error allowed after the round trip.
const VOLUME_TOLERANCE: f64 = 1e-6;
/// Patches of the gate network (`6 n²` with n = 7).
const GATE_N: usize = 7;

/// Entity name -> (count, bytes) of a Part 21 file.
fn histogram(text: &str) -> BTreeMap<String, (usize, usize)> {
    let mut h: BTreeMap<String, (usize, usize)> = BTreeMap::new();
    for l in text.lines().filter(|l| l.starts_with('#')) {
        let body = l.split_once('=').map_or("", |x| x.1);
        let name = body.split('(').next().unwrap_or("");
        let name = if name.is_empty() { "(complex)" } else { name };
        let e = h.entry(name.to_string()).or_default();
        e.0 += 1;
        e.1 += l.len() + 1;
    }
    h
}

fn count(h: &BTreeMap<String, (usize, usize)>, name: &str) -> usize {
    h.get(name).map_or(0, |x| x.0)
}

fn relative_volume_error(a: &Brep, b: &Brep) -> f64 {
    let (va, vb) = (solid::volume(a).unwrap(), solid::volume(b).unwrap());
    (va - vb).abs() / va.abs()
}

#[test]
fn closed_network_is_compact_shared_and_round_trips() {
    let b = closed_network(GATE_N);
    let patches = b.faces.len();
    assert_eq!(patches, 6 * GATE_N * GATE_N);
    let text = write_solids("net", &[&b]).unwrap();
    let per_patch = text.len() / patches;
    eprintln!("compact solid: {patches} patches, {} bytes, {per_patch} B/patch", text.len());
    assert!(per_patch <= MAX_BYTES_PER_PATCH, "{per_patch} bytes per patch");

    let h = histogram(&text);
    assert_eq!((count(&h, "MANIFOLD_SOLID_BREP"), count(&h, "CLOSED_SHELL")), (1, 1));
    assert_eq!(count(&h, "OPEN_SHELL"), 0);
    assert_eq!(count(&h, "ADVANCED_FACE"), patches);
    // shared topology: one vertex and one edge curve per record vertex and edge, each written once
    assert_eq!(count(&h, "VERTEX_POINT"), b.vertices.len());
    assert_eq!(count(&h, "EDGE_CURVE"), b.edges.len());
    assert_eq!(count(&h, "ORIENTED_EDGE"), 4 * patches);
    // control points: 16 per patch would be 16 * patches without sharing; each edge's four points
    // are shared with the two patches that use it
    assert!(count(&h, "CARTESIAN_POINT") <= 10 * patches, "{} points", count(&h, "CARTESIAN_POINT"));

    let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
    assert_eq!(back.len(), 1, "one body");
    assert_eq!(back[0].faces.len(), patches, "same face count");
    assert!(surfacecraft_kernel::nurbs::is_closed(&back[0]), "closed shell");
    let err = relative_volume_error(&b, &back[0]);
    eprintln!("compact solid round trip: relative volume error {err:e}");
    assert!(err <= VOLUME_TOLERANCE, "{err}");
}

#[test]
fn analytic_solids_round_trip_through_the_compact_writer() {
    let cases = [
        solid::make_box([0.0; 3], [3.0, 4.0, 5.0]).unwrap(),
        solid::make_cylinder([0.0; 3], 5.0, 10.0).unwrap(),
        solid::make_sphere([1.0, 2.0, 3.0], 4.0).unwrap(),
    ];
    for b in &cases {
        let text = write_solids("part", &[b]).unwrap();
        let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
        assert_eq!((back.len(), back[0].faces.len()), (1, b.faces.len()));
        let err = relative_volume_error(b, &back[0]);
        assert!(err <= VOLUME_TOLERANCE, "{err}");
    }
}

#[test]
fn sheets_and_solids_go_to_separate_files() {
    let mut sheet = closed_network(1);
    // a sheet: no bounded region
    sheet.regions.truncate(1);
    assert!(write_solids("x", &[&sheet]).is_err());
    assert!(write_sheets("x", &[&closed_network(1)]).is_err());
}

#[test]
fn hostile_solids_are_errors_never_panics() {
    assert!(write_solids("x", &[]).is_err());
    let mut mutations: Vec<(&str, Box<dyn Fn(&mut Brep)>)> = Vec::new();
    mutations.push((
        "NaN control point",
        Box::new(|b| {
            if let Surface::Spline { points, .. } = &mut b.faces[2].surface {
                points[1][1][0] = f64::NAN;
            }
        }),
    ));
    mutations.push((
        "infinite control point",
        Box::new(|b| {
            if let Surface::Spline { points, .. } = &mut b.faces[0].surface {
                points[0][0][2] = f64::INFINITY;
            }
        }),
    ));
    mutations.push(("NaN vertex", Box::new(|b| b.vertices[3].point[1] = f64::NAN)));
    mutations.push((
        "NaN edge knot",
        Box::new(|b| {
            if let surfacecraft_kernel::record::brep::Curve::Spline { knots, .. } = &mut b.edges[1].curve {
                knots[2] = f64::NAN;
            }
        }),
    ));
    mutations.push(("empty edge interval", Box::new(|b| b.edges[0].interval = [1.0, 1.0])));
    mutations.push(("NaN edge interval", Box::new(|b| b.edges[0].interval = [0.0, f64::NAN])));
    mutations.push((
        "knots out of order",
        Box::new(|b| {
            if let Surface::Spline { u_knots, .. } = &mut b.faces[1].surface {
                u_knots.reverse();
                u_knots.swap(0, 7);
            }
        }),
    ));
    mutations.push((
        "ragged net",
        Box::new(|b| {
            if let Surface::Spline { points, .. } = &mut b.faces[1].surface {
                points[2].pop();
            }
        }),
    ));
    mutations.push((
        "face index out of range",
        Box::new(|b| {
            if let Some(surfacecraft_kernel::record::brep::Shell::Faces { faces, .. }) = b.shells.first_mut() {
                faces[0].face = 9999;
            }
        }),
    ));
    mutations.push((
        "edge index out of range",
        Box::new(|b| {
            if let surfacecraft_kernel::record::brep::Loop::Edges { edges } = &mut b.faces[0].loops[0] {
                edges[0].edge = 99_999;
            }
        }),
    ));
    for (what, mutate) in &mutations {
        let mut b = closed_network(1);
        mutate(&mut b);
        let r = write_solids("bad", &[&b]);
        assert!(r.is_err(), "{what} must be an error, got a file of {} bytes", r.map_or(0, |t| t.len()));
    }
}

/// Measurement for the pull request: bytes per patch and time of the writer at two sizes.
#[test]
#[ignore = "measurement, run with --ignored --nocapture"]
fn measure_exporters() {
    for n in [7, 18] {
        let b = closed_network(n);
        let patches = b.faces.len();
        let t = std::time::Instant::now();
        let text = write_solids("net", &[&b]).unwrap();
        let written = t.elapsed();
        let t = std::time::Instant::now();
        let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
        eprintln!(
            "solid: {patches} patches, {} bytes, {} B/patch, written in {written:?}, read back ({} faces) in {:?}",
            text.len(),
            text.len() / patches,
            back.first().map_or(0, |r| r.faces.len()),
            t.elapsed()
        );
        let mut h: Vec<_> = histogram(&text).into_iter().collect();
        h.sort_by_key(|x| std::cmp::Reverse(x.1.1));
        for (k, (c, by)) in h.iter().take(8) {
            eprintln!("  {k}: {c} entities, {by} bytes");
        }
    }
}
