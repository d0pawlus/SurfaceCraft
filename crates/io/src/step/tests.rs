use super::topology::pcurve_gaps;
use super::*;
use surfacecraft_kernel::nurbs::{SurfaceProbe, fit_grid, sheet_from_spline};
use surfacecraft_kernel::record::brep::{EdgeUseRef, Loop, Region, Surface};

/// Gap allowed between an analytic face's pcurve and its edge (mm): design for h1 asks 1e-6.
const ANALYTIC_GAP_MM: f64 = 1e-6;
/// Gap allowed on a B-spline face: the record's own boundary pcurves are exact up to rounding.
const SPLINE_GAP_MM: f64 = 1e-6;

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

/// The `#id=…;` line of the entity `id`.
fn entity(text: &str, id: &str) -> String {
    let key = format!("#{id}=");
    text.lines().find(|l| l.starts_with(&key)).unwrap_or_default().to_string()
}

fn numbers(list: &str) -> Vec<f64> {
    list.split(|c: char| !(c.is_ascii_digit() || matches!(c, '.' | '-' | 'E' | '+')))
        .filter(|t| t.chars().any(|c| c.is_ascii_digit()))
        .filter_map(|t| t.parse().ok())
        .collect()
}

fn last_ref(s: &str) -> String {
    s.rsplit('#').next().unwrap().trim_matches(|c: char| !c.is_ascii_digit()).to_string()
}

/// The 2-D control points and distinct knots of the PCURVE on `line`.
fn pcurve_of_line(text: &str, line: &str) -> (Vec<[f64; 2]>, Vec<f64>) {
    // PCURVE('',#surface,#rep) -> DEFINITIONAL_REPRESENTATION('',(#curve),#ctx) -> curve
    let rep = entity(text, &last_ref(line));
    let curve = entity(text, &last_ref(rep.split(",#").next().unwrap()));
    let ids = curve.split("(#").nth(1).unwrap().split(')').next().unwrap().to_string();
    let points = ids
        .split(",#")
        .map(|id| {
            let v = numbers(entity(text, id.trim_start_matches('#')).split("',(").nth(1).unwrap());
            [v[0], v[1]]
        })
        .collect();
    (points, numbers(curve.split("),(").last().unwrap()))
}

/// Every PCURVE of the file, in file order.
fn written_pcurves(text: &str) -> Vec<(Vec<[f64; 2]>, Vec<f64>)> {
    text.lines().filter(|l| l.contains("=PCURVE(")).map(|l| pcurve_of_line(text, l)).collect()
}

/// The ids in the first parenthesised list of references on `line`: `(#a,#b)`.
fn listed_refs(line: &str) -> Vec<String> {
    line.split(",(#").nth(1).unwrap().split(')').next().unwrap().split(",#").map(str::to_string).collect()
}

#[test]
fn solids_are_refused_and_empty_input_errors() {
    let solid = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 1.0, 1.0]).unwrap();
    assert!(!is_sheet(&solid));
    assert!(write_sheets("x", &[&solid]).is_err());
    assert!(write_sheets("x", &[]).is_err());
    assert_eq!(geom::knots_of(&[0.0, 0.0, 0.5, 1.0, 1.0]), (vec![0.0, 0.5, 1.0], vec![2, 1, 2]));
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

#[test]
fn spline_sheet_edges_are_surface_curves_with_exact_pcurves() {
    let b = wave();
    let text = write_sheets("wave", &[&b]).unwrap();
    assert_eq!(count(&text, "=SURFACE_CURVE("), 4);
    assert_eq!(count(&text, "=SEAM_CURVE("), 0);
    assert_eq!(count(&text, "=PCURVE("), 4);
    assert!(text.contains(".CURVE_3D."));
    assert!(text.contains("PARAMETRIC_REPRESENTATION_CONTEXT()"));
    // the file's pcurves are the record's
    let file = written_pcurves(&text);
    let Loop::Edges { edges } = &b.faces[0].loops[0] else { panic!() };
    for (i, u) in edges.iter().enumerate() {
        let surfacecraft_kernel::record::brep::Curve2::Spline { knots, points } = &u.pcurve else { panic!() };
        assert_eq!(&file[i].0, points);
        assert_eq!(geom::knots_of(knots).0, file[i].1);
    }
    let gaps = pcurve_gaps(&b).unwrap();
    let worst = gaps.iter().copied().fold(0.0, f64::max);
    eprintln!("h1 spline sheet: {} pcurves, worst gap {worst:e} mm", gaps.len());
    assert_eq!(gaps.len(), 4);
    assert!(worst <= SPLINE_GAP_MM, "{worst}");
}

/// The unit area-weighted mean normal of the display mesh of `b`.
fn mean_normal(b: &Brep) -> [f64; 3] {
    let mesh = surfacecraft_kernel::tess::tessellate(b, Some(0.01)).unwrap();
    let mut sum = [0.0; 3];
    for i in 0..mesh.triangle_count() {
        let (n, a) = (mesh.face_normal(i).unwrap(), mesh.face_area(i));
        sum = [sum[0] + a * n.x, sum[1] + a * n.y, sum[2] + a * n.z];
    }
    let len = sum.iter().map(|c| c * c).sum::<f64>().sqrt();
    sum.map(|c| c / len)
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
/// the bottom disc) is written with a SEAM_CURVE and PCURVEs on both faces, every pcurve lies on
/// its edge, and solidcore's importer reads it back, taking the file's pcurves.
#[test]
fn gate_m3_seam_sheet_step_roundtrip() {
    let b = cup();
    assert!(is_sheet(&b));
    let text = write_sheets("cup", &[&b]).unwrap();
    assert_eq!(count(&text, "=SEAM_CURVE("), 1);
    assert_eq!(count(&text, "=SURFACE_CURVE("), 2);
    assert_eq!(count(&text, "=PCURVE("), 5, "seam 2, shared bottom circle 2, free top circle 1");
    assert!(text.contains("CYLINDRICAL_SURFACE(") && text.contains("PLANE(") && !text.contains("=CIRCLE("));
    // the SEAM_CURVE lists its pcurves in the order of the edge's uses (no rule of ISO 10303-42
    // orders them): the loop's downward use at u = 0 first, the upward one at u = 2π second
    let seam = text.lines().find(|l| l.contains("=SEAM_CURVE(")).unwrap();
    let us: Vec<f64> = listed_refs(seam).iter().map(|id| pcurve_of_line(&text, &entity(&text, id)).0[0][0]).collect();
    assert!(us[0].abs() < 1e-9 && (us[1] - std::f64::consts::TAU).abs() < 1e-9, "{us:?}");
    let gaps = pcurve_gaps(&b).unwrap();
    let worst = gaps.iter().copied().fold(0.0, f64::max);
    eprintln!("h1 cup: {} edge uses, worst pcurve gap {worst:e} mm", gaps.len());
    assert!(worst <= ANALYTIC_GAP_MM, "{worst}");

    let (back, report) = surfacecraft_kernel::step::import_with_pcurves(text.as_bytes()).unwrap();
    eprintln!("h1 cup importer pcurves: {report:?}");
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), 2);
    assert!(report.uses > 0 && report.curved_uses > 0, "{report:?}");
    assert_eq!(report.fallback_faces, 0, "{report:?}");
    let radius = back[0].faces.iter().find_map(|f| match &f.surface {
        Surface::Cylinder { radius, .. } => Some(radius),
        _ => None,
    });
    assert!((radius.unwrap() - 5.0).abs() < 1e-9);
    let (before, after) = (outward_signature(&b), outward_signature(&back[0]));
    eprintln!("h1 cup normals (Σ c·n·A, Σ n_z·A): {before:?} vs {after:?}");
    assert!(before.0 > 0.0 && after.0 > 0.0, "outward normals kept");
    assert!(((before.0 - after.0) / before.0).abs() < 0.01 && ((before.1 - after.1) / before.1).abs() < 0.01);
}

mod patches {
    //! One-face sheets on a cone, sphere and torus (and a cylinder patch): the box `u × v` of the
    //! surface bounded by exact circles and lines.
    use super::*;
    use surfacecraft_kernel::pcurve::surface_point;
    use surfacecraft_kernel::record::brep::{Curve, Edge, Face, Frame, OrientedEdge, OrientedFace, Vertex};

    fn add(a: [f64; 3], b: [f64; 3], k: f64) -> [f64; 3] {
        [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]]
    }

    fn frame() -> Frame {
        Frame { origin: [1.0, -2.0, 0.5], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] }
    }

    fn radial(u: f64) -> [f64; 3] {
        [u.cos(), u.sin(), 0.0]
    }

    fn circle(origin: [f64; 3], x: [f64; 3], y: [f64; 3], radius: f64) -> Curve {
        Curve::Circle { frame: Frame { origin, x_axis: x, y_axis: y }, radius }
    }

    type IsoFn = Box<dyn Fn(f64) -> Curve>;

    /// Surface, u range, v range, shift of an iso-u edge's parameter against v, iso-u and iso-v
    /// edges (an iso-u edge is the curve of constant u, running in v).
    struct Kind {
        surface: Surface,
        u: [f64; 2],
        v: [f64; 2],
        v_shift: f64,
        iso_u: IsoFn,
        iso_v: IsoFn,
    }

    fn kind(name: &str) -> Kind {
        let (f, z) = (frame(), [0.0, 0.0, 1.0]);
        let o = f.origin;
        match name {
            "cylinder" => {
                let s = Surface::Cylinder { frame: f, radius: 3.0 };
                let sa = s.clone();
                Kind {
                    surface: s,
                    u: [0.4, 2.2],
                    v: [0.5, 4.0],
                    v_shift: 0.0,
                    iso_u: Box::new(move |u| Curve::Line {
                        start: surface_point(&sa, [u, 0.0]).unwrap(),
                        end: surface_point(&sa, [u, 1.0]).unwrap(),
                    }),
                    iso_v: Box::new(move |v| circle(add(o, z, v), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 3.0)),
                }
            }
            "cone" | "narrowing cone" => {
                let half_angle: f64 = if name == "cone" { 0.3 } else { -0.2 };
                let s = Surface::Cone { frame: f, radius_at_origin: 2.0, half_angle };
                let sa = s.clone();
                Kind {
                    surface: s,
                    u: [-1.0, 0.9],
                    v: [0.3, 1.7],
                    v_shift: 0.0,
                    iso_u: Box::new(move |u| Curve::Line {
                        start: surface_point(&sa, [u, 0.0]).unwrap(),
                        end: surface_point(&sa, [u, 1.0]).unwrap(),
                    }),
                    iso_v: Box::new(move |v| {
                        circle(add(o, z, v), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 2.0 + v * half_angle.tan())
                    }),
                }
            }
            "sphere" => Kind {
                surface: Surface::Sphere { frame: f, radius: 4.0 },
                u: [0.3, 2.9],
                v: [-0.6, 0.8],
                v_shift: 0.0,
                iso_u: Box::new(move |u| circle(o, radial(u), z, 4.0)),
                iso_v: Box::new(move |v| {
                    circle(add(o, z, 4.0 * v.sin()), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 4.0 * v.cos())
                }),
            },
            _ => {
                let ts = 0.7;
                Kind {
                    surface: Surface::Torus { frame: f, major_radius: 5.0, minor_radius: 1.5, tube_start: ts },
                    u: [0.2, 2.0],
                    v: [0.3, 1.9],
                    v_shift: ts,
                    iso_u: Box::new(move |u| circle(add(o, radial(u), 5.0), radial(u), z, 1.5)),
                    iso_v: Box::new(move |v| {
                        let w = v + ts;
                        circle(add(o, z, 1.5 * w.sin()), [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 5.0 + 1.5 * w.cos())
                    }),
                }
            }
        }
    }

    /// The unit natural normal `S_u × S_v` of the record surface at the middle of the patch.
    pub fn natural_normal(name: &str) -> [f64; 3] {
        let k = kind(name);
        let (u, v) = (0.5 * (k.u[0] + k.u[1]), 0.5 * (k.v[0] + k.v[1]));
        let h = 1e-6;
        let at = |u: f64, v: f64| surface_point(&k.surface, [u, v]).unwrap();
        let (a, b, c, d) = (at(u + h, v), at(u - h, v), at(u, v + h), at(u, v - h));
        let (su, sv) = ([a[0] - b[0], a[1] - b[1], a[2] - b[2]], [c[0] - d[0], c[1] - d[1], c[2] - d[2]]);
        let n = [su[1] * sv[2] - su[2] * sv[1], su[2] * sv[0] - su[0] * sv[2], su[0] * sv[1] - su[1] * sv[0]];
        let len = n.iter().map(|x| x * x).sum::<f64>().sqrt();
        n.map(|x| x / len)
    }

    pub fn sheet(name: &str) -> Brep {
        let k = kind(name);
        let (u, v) = (k.u, k.v);
        let curves = [(k.iso_v)(v[0]), (k.iso_u)(u[1]), (k.iso_v)(v[1]), (k.iso_u)(u[0])];
        let sh = k.v_shift;
        let intervals = [u, [v[0] + sh, v[1] + sh], u, [v[0] + sh, v[1] + sh]];
        // corners (u0,v0) (u1,v0) (u1,v1) (u0,v1); edges bottom, right, top, left
        let corners = [[u[0], v[0]], [u[1], v[0]], [u[1], v[1]], [u[0], v[1]]];
        let ends = [[0, 1], [1, 2], [3, 2], [0, 3]];
        let forward = [true, true, false, false];
        let edges = (0..4)
            .map(|i| Edge {
                curve: curves[i].clone(),
                nurbs: None,
                interval: intervals[i],
                vertices: ends[i],
                tolerance: 1e-9,
                uses: vec![EdgeUseRef { face: 0, r#loop: 0, position: i as u32, upper: !forward[i] }],
            })
            .collect();
        let dummy = surfacecraft_kernel::record::brep::Curve2::Spline {
            knots: vec![0.0, 0.0, 1.0, 1.0],
            points: vec![[0.0; 2], [1.0; 2]],
        };
        let uses =
            (0..4).map(|i| OrientedEdge { edge: i as u32, forward: forward[i], pcurve: dummy.clone() }).collect();
        Brep {
            version: surfacecraft_kernel::record::brep::VERSION,
            tolerance: 1e-6,
            regions: vec![Region { shells: vec![0], void: false }],
            shells: vec![Shell::Faces { faces: vec![OrientedFace { face: 0, forward: true }], wires: vec![] }],
            faces: vec![Face {
                surface: k.surface.clone(),
                nurbs: None,
                domain: [u, v],
                loops: vec![Loop::Edges { edges: uses }],
            }],
            edges,
            vertices: corners
                .iter()
                .map(|c| Vertex { point: surface_point(&k.surface, *c).unwrap(), tolerance: 1e-7 })
                .collect(),
        }
    }
}

/// A patch on each analytic surface kind is written with pcurves on its STEP surface,
/// every pcurve within 1e-6 mm of its edge, and read back by solidcore as one face.
#[test]
fn analytic_patches_write_pcurves_that_lie_on_their_edges() {
    for name in ["cylinder", "cone", "narrowing cone", "sphere", "torus"] {
        let b = patches::sheet(name);
        let text = write_sheets(name, &[&b]).unwrap();
        let gaps = pcurve_gaps(&b).unwrap();
        let worst = gaps.iter().copied().fold(0.0, f64::max);
        assert_eq!(gaps.len(), 4);
        assert_eq!(count(&text, "=PCURVE("), 4, "{name}");
        let (back, report) = surfacecraft_kernel::step::import_with_pcurves(text.as_bytes()).unwrap();
        eprintln!("h1 {name} patch: worst pcurve gap {worst:e} mm; importer {report:?}");
        assert!(worst <= ANALYTIC_GAP_MM, "{name}: {worst}");
        assert_eq!(back.iter().map(|b| b.faces.len()).sum::<usize>(), 1, "{name}");
        assert_eq!(report.fallback_faces, 0, "{name}: {report:?}");
        if matches!(name, "sphere" | "torus") {
            // Not taken, and not a fault of the file: the pcurves pass the importer's own checks
            // (no fallback: each lies on its edge, the loop chains, inside the face's domain) and
            // `analytic_patch_pcurves_in_the_file_agree_with_the_standards_formulas` evaluates them
            // with the standard's closed forms. What leaves them unused is the importer's matching
            // of its converted trim candidates to the face's edge uses (`create_uv_trim_curves`);
            // solidcore's own `exported_cone_torus_and_sphere_pcurves_are_on_their_edges` asserts
            // only that no pcurve is found off its edge, never that a sphere or torus face takes
            // them. Why the matching fails was not established.
            assert_eq!((report.uses, report.uses_projected), (0, 4), "{name}: {report:?}");
        } else {
            assert_eq!((report.uses, report.curved_uses), (4, 4), "{name}: {report:?}");
        }
        let (mean, natural) = (mean_normal(&back[0]), patches::natural_normal(name));
        let dot = mean.iter().zip(natural).map(|(a, b)| a * b).sum::<f64>();
        eprintln!("h1 {name} patch: mean normal after round trip · natural normal = {dot:.4}");
        assert!(dot > 0.9, "{name}: the natural normal is kept ({dot})");
    }
}

#[test]
fn whole_sphere_sheet_has_a_seam_curve_and_roundtrips() {
    let b = surfacecraft_kernel::solid::make_sphere([0.0; 3], 5.0).unwrap();
    // a closed sphere is a solid for the exporter; as a sheet it is its one face
    let mut sheet = b.clone();
    sheet.regions = vec![Region { shells: vec![0], void: false }];
    assert!(is_sheet(&sheet));
    let text = write_sheets("ball", &[&sheet]).unwrap();
    assert_eq!(count(&text, "=SEAM_CURVE("), 1);
    assert_eq!(count(&text, "=PCURVE("), 2);
    let worst = pcurve_gaps(&sheet).unwrap().iter().copied().fold(0.0, f64::max);
    eprintln!("h1 whole sphere: worst pcurve gap {worst:e} mm");
    assert!(worst <= ANALYTIC_GAP_MM);
    let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
    assert_eq!(back[0].faces.len(), 1);
    let (before, after) = (outward_signature(&sheet), outward_signature(&back[0]));
    eprintln!("h1 whole sphere normals (Σ c·n·A): {before:?} vs {after:?}");
    assert!(before.0 > 0.0 && after.0 > 0.0, "outward normals kept");
    assert!(((before.0 - after.0) / before.0).abs() < 0.01);
}

#[test]
fn an_edge_on_a_cylinder_that_is_not_an_iso_curve_makes_the_export_fail_not_lie() {
    let mut b = patches::sheet("cylinder");
    b.edges[0].curve = surfacecraft_kernel::record::brep::Curve::Circle {
        frame: surfacecraft_kernel::record::brep::Frame {
            origin: [1.0, -2.0, 1.0],
            x_axis: [1.0, 0.0, 0.0],
            y_axis: [0.0, 0.6, 0.8],
        },
        radius: 3.0,
    };
    let err = write_sheets("slanted", &[&b]).unwrap_err().to_string();
    assert!(err.contains("not supported yet") || err.contains("does not lie"), "{err}");
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

    fn refused(b: &Brep, entity: &str) {
        let err = write_sheets("bad", &[b]).expect_err("the writer must refuse this record").to_string();
        assert!(err.contains(entity), "{err}");
    }

    #[test]
    fn circle_radius_not_finite_or_not_positive_is_refused() {
        for radius in [f64::NAN, f64::INFINITY, 0.0, -1.0] {
            let mut b = wave();
            b.edges[0].curve = circle(radius);
            refused(&b, "CIRCLE");
        }
    }

    #[test]
    fn ellipse_radii_must_be_positive() {
        let mut b = wave();
        b.edges[0].curve = Curve::Ellipse {
            frame: Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] },
            radii: [1.0, 0.0],
        };
        refused(&b, "ELLIPSE");
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
        refused(&b, "B_SPLINE_CURVE");
    }

    #[test]
    fn decreasing_knots_and_one_distinct_knot_are_refused() {
        let mut b = wave();
        set_pcurve_knots(&mut b, |k| *k = vec![1.0, 1.0, 0.0, 0.0]);
        refused(&b, "B_SPLINE_CURVE");
        let mut b = wave();
        set_pcurve_knots(&mut b, |k| *k = vec![0.5; 4]);
        refused(&b, "B_SPLINE_CURVE");
    }

    #[test]
    fn zero_length_edge_interval_is_refused() {
        let mut b = wave();
        let t = b.edges[0].interval[0];
        b.edges[0].interval = [t, t];
        refused(&b, "edge 0");
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
        refused(&b, "frame");
    }

    /// A slit (one planar face using an edge twice) is not a seam: two pcurves on a SURFACE_CURVE.
    #[test]
    fn an_edge_used_twice_by_a_plane_is_no_seam_curve() {
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
        let text = write_sheets("slit", &[&b]).unwrap();
        assert_eq!(count(&text, "=SEAM_CURVE("), 0);
        let two_pcurves = |l: &str| {
            l.contains("=SURFACE_CURVE(")
                && l.split(",(").nth(1).is_some_and(|t| t.split(')').next().unwrap().contains(','))
        };
        assert!(text.lines().any(two_pcurves), "a SURFACE_CURVE with two pcurves");
    }
}

/// Numbers of the entity `id`'s own parameters after its placement reference.
fn surface_in_file(text: &str, id: &str) -> (String, [[f64; 3]; 4], Vec<f64>) {
    let line = entity(text, id);
    let name = line.split('=').nth(1).unwrap().split('(').next().unwrap().to_string();
    let args = line.split("('',").nth(1).unwrap();
    let placement = entity(text, args.split(',').next().unwrap().trim_start_matches('#'));
    let refs: Vec<String> = placement.split(",#").skip(1).map(|r| r.trim_end_matches(");").to_string()).collect();
    let vec3 = |id: &str| -> [f64; 3] {
        let v = numbers(entity(text, id).split("',(").nth(1).unwrap());
        [v[0], v[1], v[2]]
    };
    let (o, z, x) = (vec3(&refs[0]), vec3(&refs[1]), vec3(&refs[2]));
    let y = [z[1] * x[2] - z[2] * x[1], z[2] * x[0] - z[0] * x[2], z[0] * x[1] - z[1] * x[0]];
    let rest: Vec<f64> = numbers(args.split(',').skip(1).collect::<Vec<_>>().join(",").as_str());
    (name, [o, x, y, z], rest)
}

/// ISO 10303-42 surface points, written here from the standard's formulas.
fn iso_point(name: &str, [o, x, y, z]: [[f64; 3]; 4], p: &[f64], u: f64, v: f64) -> [f64; 3] {
    let radial = |s: f64| -> [f64; 3] { std::array::from_fn(|i| s * (u.cos() * x[i] + u.sin() * y[i])) };
    let at = |a: [f64; 3], b: [f64; 3], k: f64| -> [f64; 3] { std::array::from_fn(|i| a[i] + k * b[i]) };
    match name {
        "CYLINDRICAL_SURFACE" => at(at(o, radial(1.0), p[0]), z, v),
        "CONICAL_SURFACE" => at(at(o, radial(1.0), p[0] + v * p[1].tan()), z, v),
        "SPHERICAL_SURFACE" => at(at(o, radial(1.0), p[0] * v.cos()), z, p[0] * v.sin()),
        "TOROIDAL_SURFACE" => at(at(o, radial(1.0), p[0] + p[1] * v.cos()), z, p[1] * v.sin()),
        other => panic!("{other}"),
    }
}

fn record_point(c: &surfacecraft_kernel::record::brep::Curve, t: f64) -> [f64; 3] {
    use surfacecraft_kernel::record::brep::Curve;
    match c {
        Curve::Line { start, end } => std::array::from_fn(|i| start[i] + t * (end[i] - start[i])),
        Curve::Circle { frame, radius } => {
            std::array::from_fn(|i| frame.origin[i] + radius * (t.cos() * frame.x_axis[i] + t.sin() * frame.y_axis[i]))
        }
        _ => panic!(),
    }
}

/// The pcurves of every analytic patch, read back from the file and evaluated with the standard's
/// closed-form surface formulas (not solidcore's evaluator, which produced them), land on the
/// record's edge at the same parameter.
#[test]
fn analytic_patch_pcurves_in_the_file_agree_with_the_standards_formulas() {
    for name in ["cylinder", "cone", "narrowing cone", "sphere", "torus"] {
        let b = patches::sheet(name);
        let text = write_sheets(name, &[&b]).unwrap();
        let lines: Vec<&str> = text.lines().filter(|l| l.contains("=PCURVE(")).collect();
        assert_eq!(lines.len(), 4);
        let mut worst: f64 = 0.0;
        for (edge, line) in b.edges.iter().zip(&lines) {
            let (pts, knots) = pcurve_of_line(&text, line);
            let surface = line.split(",#").nth(1).unwrap().to_string();
            let (kind, frame, params) = surface_in_file(&text, &surface);
            let (t0, t1) = (knots[0], knots[1]);
            assert!((t0 - edge.interval[0]).abs() < 1e-12 && (t1 - edge.interval[1]).abs() < 1e-12, "{name}");
            for k in 0..=6 {
                let f = f64::from(k) / 6.0;
                let t = t0 + f * (t1 - t0);
                let (u, v) = (pts[0][0] + f * (pts[1][0] - pts[0][0]), pts[0][1] + f * (pts[1][1] - pts[0][1]));
                let (s, c) = (iso_point(&kind, frame, &params, u, v), record_point(&edge.curve, t));
                worst = worst.max(((s[0] - c[0]).powi(2) + (s[1] - c[1]).powi(2) + (s[2] - c[2]).powi(2)).sqrt());
            }
        }
        eprintln!("h1 {name} patch: file pcurves vs closed-form surface, worst distance {worst:e} mm");
        assert!(worst < 1e-9, "{name}: {worst}");
    }
}
