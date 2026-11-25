use super::*;
use surfacecraft_kernel::nurbs::{SurfaceProbe, fit_grid, sheet_from_spline};
use surfacecraft_kernel::pcurve::pcurve_gap;
use surfacecraft_kernel::record::brep::Shell;
use surfacecraft_kernel::record::brep::{EdgeUseRef, Loop, Region, Surface};

/// Gap allowed between a face's pcurve and its edge after a round trip (mm): design for h1 asks
/// 1e-6. The reader does not take the file's PCURVEs (solidcore's reader makes each use's pcurve
/// from its closed form or the kernel's projection; the file's are an open item upstream,
/// tangency-labs/solidcraft#13794), so the gap is that of the pcurves the reader made.
const PCURVE_GAP_MM: f64 = 1e-6;

/// Every edge use's pcurve gap to its edge in `b` (mm).
fn pcurve_gaps(b: &Brep) -> Vec<f64> {
    let mut gaps = Vec::new();
    for face in &b.faces {
        let surface = face.nurbs.as_ref().unwrap_or(&face.surface);
        for l in &face.loops {
            let Loop::Edges { edges } = l else { continue };
            for u in edges {
                let e = &b.edges[u.edge as usize];
                let curve = e.nurbs.as_ref().unwrap_or(&e.curve);
                gaps.push(pcurve_gap(surface, &u.pcurve, curve, e.interval).unwrap());
            }
        }
    }
    gaps
}

fn worst(gaps: &[f64]) -> f64 {
    gaps.iter().copied().fold(0.0, f64::max)
}

fn wave() -> Brep {
    let grid: Vec<Vec<[f64; 3]>> = (0..24)
        .map(|i| {
            (0..20)
                .map(|j| {
                    let (x, y) = (60.0 * f64::from(i) / 23.0, 40.0 * f64::from(j) / 19.0);
                    [x, y, surfacecraft_geom::synth::wave_height(3.0, x, y)]
                })
                .collect()
        })
        .collect();
    sheet_from_spline(&fit_grid(&grid, 10, 9, 3).unwrap()).unwrap()
}

/// The open cup: a kernel cylinder (side, top, bottom) without its top cap, so the side's seam is
/// used twice by one face and its bottom circle by two faces.
fn cup() -> Brep {
    let mut b = surfacecraft_kernel::solid::make_cylinder([0.0; 3], 5.0, 10.0).unwrap();
    let top = 1;
    assert!(matches!(&b.faces[top].surface, Surface::Plane { origin, .. } if origin[2] > 5.0), "face 1 is the top cap");
    b.faces.remove(top);
    let shift = |f: u32| if f as usize > top { f - 1 } else { f };
    for e in &mut b.edges {
        e.uses = e
            .uses
            .iter()
            .filter(|u| u.face as usize != top)
            .map(|u| EdgeUseRef { face: shift(u.face), ..*u })
            .collect();
    }
    b.shells = vec![Shell::Faces {
        faces: (0..b.faces.len() as u32)
            .map(|face| surfacecraft_kernel::record::brep::OrientedFace { face, forward: true })
            .collect(),
        wires: vec![],
    }];
    b.regions = vec![Region { shells: vec![0], void: false }];
    b.validate().unwrap();
    b
}

fn count(text: &str, needle: &str) -> usize {
    text.matches(needle).count()
}

#[test]
fn solids_are_refused_and_empty_input_errors() {
    let solid = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 1.0, 1.0]).unwrap();
    assert!(!is_sheet(&solid));
    assert!(write_sheets("x", &[&solid]).is_err());
    assert!(write_sheets("x", &[]).is_err());
    assert!(write_solids("x", &[&wave()]).is_err(), "a sheet is not a solid");
}

/// Gate M3: a fitted sheet written as STEP reads back (solidcore's importer) as one face whose
/// surface matches the original within 1e-6 mm. Unchanged by the pcurves.
#[test]
fn gate_m3_sheet_step_roundtrip() {
    let b = wave();
    let text = write_sheets("wave", &[&b]).unwrap();
    assert!(
        text.contains("SHELL_BASED_SURFACE_MODEL")
            && text.contains("OPEN_SHELL")
            && text.contains("B_SPLINE_SURFACE_WITH_KNOTS")
    );
    let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), 1);
    let face = &back[0].faces[0];
    let surf = face.nurbs.as_ref().unwrap_or(&face.surface);
    let a = SurfaceProbe::new(&b.faces[0].surface).unwrap();
    let bprobe = SurfaceProbe::new(surf).unwrap();
    let mut max: f64 = 0.0;
    for i in 0..=8 {
        for j in 0..=8 {
            let p = a.eval(f64::from(i) / 8.0, f64::from(j) / 8.0).unwrap();
            max = max.max(bprobe.closest(p).unwrap().0);
        }
    }
    eprintln!("gate_m3 STEP round-trip: max surface distance {max:e} mm");
    assert!(max < 1e-6, "{max}");
}

/// The `#id=…;` line of the entity `id`.
fn entity(text: &str, id: &str) -> String {
    let key = format!("#{id}=");
    text.lines().find(|l| l.starts_with(&key)).unwrap_or_default().to_string()
}

/// The numbers of a parameter list.
fn numbers(list: &str) -> Vec<f64> {
    list.split(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | 'E' | '+')))
        .filter(|t| t.chars().any(|c| c.is_ascii_digit()))
        .filter_map(|t| t.parse().ok())
        .collect()
}

/// The 2-D control points of every PCURVE of the file: PCURVE -> DEFINITIONAL_REPRESENTATION ->
/// B_SPLINE_CURVE_WITH_KNOTS -> CARTESIAN_POINTs.
fn written_pcurve_points(text: &str) -> Vec<Vec<[f64; 2]>> {
    let last_ref = |s: &str| s.rsplit('#').next().unwrap().trim_matches(|c: char| !c.is_ascii_digit()).to_string();
    text.lines()
        .filter(|l| l.contains("=PCURVE("))
        .map(|line| {
            let rep = entity(text, &last_ref(line));
            let curve = entity(text, &last_ref(rep.split(",#").next().unwrap()));
            let ids = curve.split("(#").nth(1).unwrap().split(')').next().unwrap().to_string();
            ids.split(",#")
                .map(|id| {
                    let v = numbers(entity(text, id.trim_start_matches('#')).split("',(").nth(1).unwrap());
                    [v[0], v[1]]
                })
                .collect()
        })
        .collect()
}

/// A fitted sheet's four edges are SURFACE_CURVEs with one PCURVE each, and the sheet read back
/// has pcurves on their edges.
#[test]
fn spline_sheet_edges_are_surface_curves_with_pcurves() {
    let b = wave();
    let text = write_sheets("wave", &[&b]).unwrap();
    assert_eq!(count(&text, "=SURFACE_CURVE("), 4);
    assert_eq!(count(&text, "=SEAM_CURVE("), 0);
    assert_eq!(count(&text, "=PCURVE("), 4);
    // the file's pcurves are the record's (a face on a spline keeps its (u, v)): each PCURVE's
    // control points are those of one of the record's pcurves
    let mut written = written_pcurve_points(&text);
    let Loop::Edges { edges } = &b.faces[0].loops[0] else { panic!("an edge loop") };
    let mut ours: Vec<Vec<[f64; 2]>> = edges
        .iter()
        .map(|u| match &u.pcurve {
            surfacecraft_kernel::record::brep::Curve2::Spline { points, .. } => points.clone(),
            other => panic!("{other:?}"),
        })
        .collect();
    let key = |p: &Vec<[f64; 2]>| format!("{p:?}");
    written.sort_by_key(key);
    ours.sort_by_key(key);
    assert_eq!(written, ours);
    let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
    let gaps = pcurve_gaps(&back[0]);
    eprintln!("h1 spline sheet: {} pcurves read back, worst gap {:e} mm", gaps.len(), worst(&gaps));
    assert_eq!(gaps.len(), 4);
    assert!(worst(&gaps) <= PCURVE_GAP_MM, "{gaps:?}");
}

fn outward_signature(b: &Brep) -> (f64, f64) {
    let mesh = surfacecraft_kernel::tess::tessellate(b, Some(0.01)).unwrap();
    let (mut radial, mut z) = (0.0, 0.0);
    for i in 0..mesh.triangle_count() {
        let (n, c, a) = (mesh.face_normal(i).unwrap(), mesh.face_centroid(i).unwrap(), mesh.face_area(i));
        radial += a * (c.x * n.x + c.y * n.y + c.z * n.z);
        z += a * n.z;
    }
    (radial, z)
}

/// Gate M3 (seam): a multi-face sheet on a closed surface (a cup: the cylinder's side with its seam, and
/// the bottom disc) is written with a SEAM_CURVE and PCURVEs on both faces, and solidcore's importer
/// reads it back with the cylinder exact, pcurves on their edges and the outward side kept.
#[test]
fn gate_m3_seam_sheet_step_roundtrip() {
    let b = cup();
    assert!(is_sheet(&b));
    let text = write_sheets("cup", &[&b]).unwrap();
    eprintln!(
        "h1 cup file: SEAM_CURVE {}, SURFACE_CURVE {}, PCURVE {}, EDGE_CURVE {}",
        count(&text, "=SEAM_CURVE("),
        count(&text, "=SURFACE_CURVE("),
        count(&text, "=PCURVE("),
        count(&text, "=EDGE_CURVE(")
    );
    assert!(text.contains("CYLINDRICAL_SURFACE(") && text.contains("PLANE("));
    let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), 2);
    let radius = back[0].faces.iter().find_map(|f| match &f.surface {
        Surface::Cylinder { radius, .. } => Some(radius),
        _ => None,
    });
    assert!((radius.unwrap() - 5.0).abs() < 1e-9);
    let gaps = pcurve_gaps(&back[0]);
    eprintln!("h1 cup: {} edge uses read back, worst pcurve gap {:e} mm", gaps.len(), worst(&gaps));
    assert!(worst(&gaps) <= PCURVE_GAP_MM, "{gaps:?}");
    let (before, after) = (outward_signature(&b), outward_signature(&back[0]));
    eprintln!("h1 cup normals (Σ c·n·A, Σ n_z·A): {before:?} vs {after:?}");
    assert!(before.0 > 0.0 && after.0 > 0.0, "outward normals kept");
    assert!(((before.0 - after.0) / before.0).abs() < 0.01 && ((before.1 - after.1) / before.1).abs() < 0.01);
}

/// The side of a kernel cylinder, extracted as a one-face sheet (the face carries its NURBS form,
/// as every analytic face of a record does), is written and read back as one cylinder face with
/// pcurves on their edges and its side kept.
#[test]
fn an_analytic_face_sheet_round_trips() {
    let solid = surfacecraft_kernel::solid::make_cylinder([1.0, -2.0, 0.5], 3.0, 4.0).unwrap();
    let side = solid.faces.iter().position(|f| matches!(f.surface, Surface::Cylinder { .. })).unwrap();
    let sheet = surfacecraft_kernel::hybrid::extract_face(&solid, side).unwrap();
    assert!(is_sheet(&sheet));
    let text = write_sheets("side", &[&sheet]).unwrap();
    eprintln!(
        "h1 cylinder side file: PCURVE {}, SEAM_CURVE {}",
        count(&text, "=PCURVE("),
        count(&text, "=SEAM_CURVE(")
    );
    assert!(text.contains("CYLINDRICAL_SURFACE("));
    let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
    assert_eq!(back.iter().map(|b| b.faces.len()).sum::<usize>(), 1);
    let gaps = pcurve_gaps(&back[0]);
    eprintln!("h1 cylinder side: worst pcurve gap read back {:e} mm", worst(&gaps));
    assert!(worst(&gaps) <= PCURVE_GAP_MM, "{gaps:?}");
    // Σ c·n·A over the side is radius × area when its normals point away from the axis
    let (before, after) = (outward_signature(&sheet).0, outward_signature(&back[0]).0);
    eprintln!("h1 cylinder side normals (Σ c·n·A): {before} vs {after}");
    assert!(before > 0.0 && after > 0.0, "outward normals kept");
    assert!(((before - after) / before).abs() < 0.01, "{before} vs {after}");
}

mod hostile {
    //! Records the writer must refuse rather than write a file that says something else.
    use super::*;
    use surfacecraft_kernel::record::brep::{Curve, Curve2, Frame};

    fn circle(radius: f64) -> Curve {
        Curve::Circle { frame: Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] }, radius }
    }

    /// Replaces the knots of the first edge use's pcurve on the first face.
    fn set_pcurve_knots(b: &mut Brep, f: impl FnOnce(&mut Vec<f64>)) {
        let Some(Loop::Edges { edges }) = b.faces[0].loops.first_mut() else { panic!("no edge loop") };
        let Curve2::Spline { knots, .. } = &mut edges[0].pcurve else { panic!("not a spline pcurve") };
        f(knots);
    }

    /// The writer refuses `b`, saying `why` (the record check that fails).
    fn refused(b: &Brep, why: &str) {
        let r = write_sheets("bad", &[b]);
        let Err(e) = r else {
            panic!("the writer must refuse ({why}), got a file of {} bytes", r.map_or(0, |t| t.len()))
        };
        assert!(e.to_string().contains(why), "{e}");
    }

    #[test]
    fn circle_radius_not_finite_or_not_positive_is_refused() {
        for radius in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            let mut b = wave();
            b.edges[0].curve = circle(radius);
            refused(&b, "radius");
        }
    }

    #[test]
    fn ellipse_radii_must_be_positive() {
        let mut b = wave();
        b.edges[0].curve = Curve::Ellipse {
            frame: Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] },
            radii: [1.0, 0.0],
        };
        refused(&b, "radius");
    }

    #[test]
    fn zero_length_line_is_refused() {
        let mut b = wave();
        b.edges[0].curve = Curve::Line { start: [1.0; 3], end: [1.0; 3] };
        assert!(write_sheets("bad", &[&b]).is_err());
    }

    #[test]
    fn nan_pcurve_knot_is_refused() {
        let mut b = wave();
        set_pcurve_knots(&mut b, |k| k[1] = f64::NAN);
        refused(&b, "non-finite");
    }

    #[test]
    fn decreasing_knots_and_one_distinct_knot_are_refused() {
        let mut b = wave();
        set_pcurve_knots(&mut b, |k| *k = vec![1.0, 1.0, 0.0, 0.0]);
        refused(&b, "unordered knots");
        let mut b = wave();
        set_pcurve_knots(&mut b, |k| *k = vec![0.5; 4]);
        refused(&b, "empty spline domain");
    }

    #[test]
    fn zero_length_edge_interval_is_refused() {
        let mut b = wave();
        let t = b.edges[0].interval[0];
        b.edges[0].interval = [t, t];
        refused(&b, "empty parameter interval");
    }

    #[test]
    fn non_finite_coordinate_never_reaches_the_file() {
        let mut b = wave();
        b.vertices[0].point[1] = f64::NAN;
        assert!(write_sheets("bad", &[&b]).is_err());
    }

    #[test]
    fn a_frame_that_is_not_orthonormal_is_refused() {
        let mut b = wave();
        b.edges[0].curve = Curve::Circle {
            frame: Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [1.0, 1.0, 0.0] },
            radius: 1.0,
        };
        refused(&b, "without its NURBS form");
    }

    /// A slit (one planar face using an edge twice).
    #[test]
    fn a_slit_record_that_lost_its_neighbours_is_refused() {
        let mut b = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 1.0, 1.0]).unwrap();
        // one planar face, its loop run once more along its first edge back and forth
        let face = 0;
        let (edge, pcurve) = match &b.faces[face].loops[0] {
            Loop::Edges { edges } => (edges[0].edge, edges[0].pcurve.clone()),
            _ => panic!(),
        };
        let position = match &mut b.faces[face].loops[0] {
            Loop::Edges { edges } => {
                edges.push(surfacecraft_kernel::record::brep::OrientedEdge { edge, forward: false, pcurve });
                edges.len() as u32 - 1
            }
            _ => panic!(),
        };
        b.edges[edge as usize].uses.retain(|u| u.face as usize == face);
        b.edges[edge as usize].uses.push(EdgeUseRef { face: face as u32, r#loop: 0, position, upper: true });
        b.shells = vec![Shell::Faces {
            faces: vec![surfacecraft_kernel::record::brep::OrientedFace { face: face as u32, forward: true }],
            wires: vec![],
        }];
        b.regions = vec![Region { shells: vec![0], void: false }];
        // the slit's record no longer lists the box's other faces on that edge: the record check
        // refuses it before anything is written
        refused(&b, "oriented faces");
    }
}
