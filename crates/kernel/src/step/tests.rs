//! Tests of the STEP adapter: round trips, hostile files, the record checks and the kernel's trims.
use super::*;

#[test]
fn a_box_round_trips_with_its_volume() {
    let b = crate::solid::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap();
    let text = export_solids(&[("box", &b)]).unwrap();
    assert!(text.contains("MANIFOLD_SOLID_BREP") && text.contains("'SurfaceCraft'"));
    let back = import(text.as_bytes()).unwrap();
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), 6);
    let v = crate::solid::volume(&back[0]).unwrap();
    assert!((v - 24.0).abs() < 1e-6, "{v}");
}

/// A cone narrowing along its axis and a torus, as the kernel makes them, round trip as one
/// body each with their volume (the narrowing cone split into several bodies on the old pin).
#[test]
fn a_narrowing_cone_and_a_torus_round_trip_with_their_volume() {
    let cone = solidcore_ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).unwrap();
    let torus = solidcore_ops::make_torus([1.0, 2.0, 3.0], 6.0, 1.5).unwrap();
    for (name, body) in [("cone", cone), ("torus", torus)] {
        let rec = crate::solid::encode(&body).unwrap();
        let text = export_solids(&[(name, &rec)]).unwrap();
        let back = import(text.as_bytes()).unwrap();
        assert_eq!(back.len(), 1, "{name}");
        let (before, after) = (crate::solid::volume(&rec).unwrap(), crate::solid::volume(&back[0]).unwrap());
        eprintln!("{name}: volume {before} -> {after}");
        assert!((before - after).abs() <= 1e-6 * before, "{name}: {before} vs {after}");
    }
}

/// A box with a filleted edge (the kernel labels the round a cone of half angle 0) round trips
/// with its volume.
#[test]
fn a_filleted_box_round_trips_with_its_volume() {
    let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 8.0]).unwrap();
    let rec = crate::solid::fillet(&b, &[0], 2.0).unwrap();
    let text = export_solids(&[("plate", &rec)]).unwrap();
    let back = import(text.as_bytes()).unwrap();
    let (before, after) = (crate::solid::volume(&rec).unwrap(), crate::solid::volume(&back[0]).unwrap());
    assert!((before - after).abs() <= 1e-6 * before, "{before} vs {after}");
}

#[test]
fn a_cone_of_half_angle_0_is_a_cylinder_and_one_past_its_apex_starts_there() {
    let frame = crate::record::brep::Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] };
    let flat = Surface::Cone { frame: frame.clone(), radius_at_origin: 2.0, half_angle: 0.0 };
    assert!(matches!(step_label(&flat), Some(Surface::Cylinder { radius, .. }) if radius == 2.0));
    let past = Surface::Cone { frame, radius_at_origin: -1.0, half_angle: 0.25 };
    let Some(Surface::Cone { frame, radius_at_origin, .. }) = step_label(&past) else { panic!("moved") };
    assert_eq!(radius_at_origin, 0.0);
    assert!((frame.origin[2] - 1.0 / 0.25f64.tan()).abs() < 1e-12);
    let fine = Surface::Cone { frame: frame.clone(), radius_at_origin: 1.0, half_angle: 0.25 };
    assert!(step_label(&fine).is_none());
}

fn plate_text() -> String {
    let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 8.0]).unwrap();
    let rec = crate::solid::fillet(&b, &[0], 2.0).unwrap();
    export_solids(&[("plate", &rec)]).unwrap()
}

/// Every body a hostile file gives passes the record's checks.
fn sound(r: &Result<Vec<Brep>>) {
    if let Ok(bodies) = r {
        for b in bodies {
            assert!(b.validate_built().is_ok(), "an invalid record was returned");
        }
    }
}

/// A file whose topology does not hold together (an edge's vertex that does not exist, a loop
/// that names another face's edge) is refused or read as a valid record, never returned broken.
#[test]
fn a_file_with_broken_topology_is_refused() {
    let text = plate_text();
    let curve = text.lines().find(|l| l.contains("=EDGE_CURVE(")).unwrap();
    let v = curve.split(',').nth(1).unwrap();
    let dangling = text.replacen(curve, &curve.replacen(v, "#99999999", 1), 1);
    let r = import(dangling.as_bytes());
    assert!(r.is_err(), "a dangling vertex refuses the only body");
    let edges: Vec<&str> = text.lines().filter(|l| l.contains("=ORIENTED_EDGE(")).collect();
    let edge_of = |l: &str| l.rsplit(',').nth(1).unwrap().to_string();
    let (a, b) = (edge_of(edges[0]), edge_of(edges[edges.len() - 1]));
    assert_ne!(a, b);
    let crossed = text.replacen(edges[0], &edges[0].replacen(&a, &b, 1), 1);
    let r = import(crossed.as_bytes());
    assert!(r.is_err(), "a loop crossed onto another edge refuses the only body: {:?}", r.map(|b| b.len()));
}

/// Bytes of a valid file flipped and cut at random: every call returns, and what reads is valid.
#[test]
fn hostile_step_bytes_never_panic() {
    let base = plate_text().into_bytes();
    let mut state = 0x9e37_79b9_7f4a_7c15_u64;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for round in 0..200 {
        let mut bytes = base.clone();
        for _ in 0..1 + next() % 8 {
            let i = (next() % bytes.len() as u64) as usize;
            bytes[i] = next() as u8;
        }
        if round % 3 == 0 {
            bytes.truncate((next() % (bytes.len() as u64 + 1)) as usize);
        }
        sound(&import(&bytes));
    }
}

#[test]
fn nothing_to_export_and_garbage_are_errors() {
    assert!(export("x", &[]).is_err());
    assert!(import(b"").is_err());
    assert!(import(b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=FOO(;\nENDSEC;\nEND-ISO-10303-21;\n").is_err());
    assert!(import(&[0xff, 0xfe, 0x00, 0x80]).is_err());
}

#[test]
fn a_file_over_the_size_limit_is_refused_before_parsing() {
    let big = vec![b' '; MAX_STEP_BYTES + 1];
    assert!(matches!(import(&big), Err(KernelError::Argument(_))));
}

/// A block whose top is cut by a wavy spline surface: the spline face's edges are intersection
/// curves with no closed-form pcurve, so reading the block back needs the kernel's trims.
fn spline_topped_block() -> Brep {
    let grid: Vec<Vec<[f64; 3]>> = (0..12)
        .map(|i| {
            (0..10)
                .map(|j| {
                    let (x, y) = (-5.0 + 50.0 * f64::from(i) / 11.0, -5.0 + 40.0 * f64::from(j) / 9.0);
                    [x, y, 15.0 + 1.5 * (x / 7.0).sin() * (y / 6.0).cos()]
                })
                .collect()
        })
        .collect();
    let top = crate::nurbs::fit_grid(&grid, 6, 5, 3).unwrap();
    let tall = crate::solid::make_box([0.0; 3], [40.0, 30.0, 25.0]).unwrap();
    crate::hybrid::cut_by_spline(&tall, &top, [0.0, 0.0, 1.0]).unwrap()
}

#[test]
fn a_spline_face_cut_by_planes_reads_back_through_the_kernels_trims() {
    let b = spline_topped_block();
    let text = export_solids(&[("block", &b)]).unwrap();
    let ex = solidcore_step::p21::parse(&text).unwrap();
    let trims = KernelTrims::new();
    let read = solidcore_step::topology::read_solids_with(&ex, &trims);
    assert!(read.failures.is_empty(), "{:?}", read.failures.first().map(|f| &f.message));
    assert!(trims.0.borrow().cost() > 0, "the kernel's trims were used");
    let back = import(text.as_bytes()).unwrap();
    let (before, after) = (crate::solid::volume(&b).unwrap(), crate::solid::volume(&back[0]).unwrap());
    assert!((before - after).abs() <= 1e-6 * before, "{before} vs {after}");
}

/// Bytes of a file that needs the kernel's trims, flipped and cut: every call returns, and what
/// reads is a valid record.
#[test]
fn hostile_bytes_of_a_file_that_needs_trims_never_panic() {
    let base = export_solids(&[("block", &spline_topped_block())]).unwrap().into_bytes();
    let mut state = 0x2545_f491_4f6c_dd1d_u64;
    let mut next = move || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        state
    };
    for round in 0..40 {
        let mut bytes = base.clone();
        for _ in 0..1 + next() % 6 {
            let i = (next() % bytes.len() as u64) as usize;
            bytes[i] = next() as u8;
        }
        if round % 4 == 0 {
            bytes.truncate((next() % (bytes.len() as u64 + 1)) as usize);
        }
        sound(&import(&bytes));
    }
}

/// A file whose faces share one spline surface so often that their records would hold more control
/// points than [`check::MAX_FACE_POINTS`] is refused before anything is built.
#[test]
fn a_file_whose_faces_would_hold_too_many_control_points_is_refused_before_reading() {
    let n = 64usize;
    let mut text = String::from("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n");
    let rows: Vec<String> = (0..n)
        .map(|i| format!("({})", (0..n).map(|j| format!("#{}", 10 + i * n + j)).collect::<Vec<_>>().join(",")))
        .collect();
    text += &format!(
        "#2=B_SPLINE_SURFACE_WITH_KNOTS('',3,3,({}),.UNSPECIFIED.,.F.,.F.,.F.,(4,{},4),(4,{},4),({}),({}),.UNSPECIFIED.);\n",
        rows.join(","),
        vec!["1"; n - 4].join(","),
        vec!["1"; n - 4].join(","),
        (0..n - 2).map(|k| format!("{k}.")).collect::<Vec<_>>().join(","),
        (0..n - 2).map(|k| format!("{k}.")).collect::<Vec<_>>().join(","),
    );
    for k in 0..n * n {
        text += &format!("#{}=CARTESIAN_POINT('',({}.,{}.,0.));\n", 10 + k, k / n, k % n);
    }
    let faces = check::MAX_FACE_POINTS / (n * n) + 2;
    for f in 0..faces {
        text += &format!("#{}=ADVANCED_FACE('',(),#2,.T.);\n", 100_000 + f);
    }
    let list: Vec<String> = (0..faces).map(|f| format!("#{}", 100_000 + f)).collect();
    text += &format!("#5=OPEN_SHELL('',({}));\n#6=SHELL_BASED_SURFACE_MODEL('',(#5));\n", list.join(","));
    text += "ENDSEC;\nEND-ISO-10303-21;\n";
    let e = import(text.as_bytes()).unwrap_err().to_string();
    assert!(e.contains("geometry"), "{e}");
}

/// The faces of a narrowing cone and a hemisphere (and of a torus where one is a sheet), each written
/// as a sheet of its own, read back as one face of the same area.
#[test]
fn analytic_face_sheets_round_trip_with_their_area() {
    let cone = crate::solid::encode(&solidcore_ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).unwrap()).unwrap();
    let torus = crate::solid::encode(&solidcore_ops::make_torus([1.0, 2.0, 3.0], 6.0, 1.5).unwrap()).unwrap();
    let ball = crate::solid::make_sphere([0.0; 3], 4.0).unwrap();
    let cut = crate::solid::make_box([-5.0, -5.0, -5.0], [10.0, 10.0, 5.0]).unwrap();
    let hemisphere = crate::feature::boolean(&ball, &cut, crate::feature::BoolKind::Subtract).unwrap();
    let area = |b: &Brep| crate::tess::tessellate(b, Some(0.005)).unwrap().area();
    let mut kinds = Vec::new();
    for body in [&cone, &torus, &hemisphere] {
        for (i, f) in body.faces.iter().enumerate() {
            let kind = match f.surface {
                Surface::Cone { .. } => "cone",
                Surface::Torus { .. } => "torus",
                Surface::Sphere { .. } => "sphere",
                _ => continue,
            };
            // the whole torus is closed on itself: no sheet of its own (its solid round trips above)
            let Ok(sheet) = crate::hybrid::extract_face(body, i) else { continue };
            let text = export(kind, &[(kind, &sheet)]).unwrap();
            let back = import(text.as_bytes()).unwrap();
            assert_eq!(back.iter().map(|b| b.faces.len()).sum::<usize>(), 1, "{kind}");
            let (a, b) = (area(&sheet), area(&back[0]));
            eprintln!("{kind} sheet: area {a} -> {b}");
            assert!((a - b).abs() <= 1e-3 * a, "{kind}: {a} vs {b}");
            kinds.push(kind);
        }
    }
    for kind in ["cone", "sphere"] {
        assert!(kinds.contains(&kind), "a {kind} sheet was written: {kinds:?}");
    }
}

/// A record whose edge has left its face (moved 2 mm off its cylinder) is refused by the writer,
/// not written as a file that says something else.
#[test]
fn a_record_whose_edge_left_its_face_is_not_written() {
    let solid = crate::solid::make_cylinder([0.0; 3], 3.0, 4.0).unwrap();
    let side = solid.faces.iter().position(|f| matches!(f.surface, Surface::Cylinder { .. })).unwrap();
    let mut sheet = crate::hybrid::extract_face(&solid, side).unwrap();
    let shift = |p: &mut [f64; 3]| p[0] += 2.0;
    let e = sheet.edges.iter().position(|e| matches!(e.curve, crate::record::brep::Curve::Circle { .. })).unwrap();
    if let crate::record::brep::Curve::Circle { frame, .. } = &mut sheet.edges[e].curve {
        shift(&mut frame.origin);
    }
    match &mut sheet.edges[e].nurbs {
        Some(crate::record::brep::Curve::RationalSpline { homogeneous, .. }) => {
            for h in homogeneous.iter_mut() {
                h[0] += 2.0 * h[3];
            }
        }
        Some(crate::record::brep::Curve::Spline { points, .. }) => points.iter_mut().for_each(shift),
        _ => {}
    }
    for v in sheet.edges[e].vertices {
        shift(&mut sheet.vertices[v as usize].point);
    }
    let err = export("off", &[("off", &sheet)]).unwrap_err().to_string();
    assert!(err.contains("does not agree"), "{err}");
}

/// The writer refuses a closed shell whose faces do not agree on their sides, and one missing a
/// face, instead of writing a solid that is not one.
#[test]
fn a_closed_shell_that_is_not_one_is_not_written() {
    let b = crate::solid::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap();
    let mut flipped = b.clone();
    if let Some(crate::record::brep::Shell::Faces { faces, .. }) = flipped.shells.first_mut() {
        faces[2].forward = !faces[2].forward;
    }
    // refused by the record's structural check (canonical order) before the sides are compared;
    // `check::shells_agree` alone is tested in `check`
    assert!(export_solids(&[("flipped", &flipped)]).is_err());
    let mut open = b;
    if let Some(crate::record::brep::Shell::Faces { faces, .. }) = open.shells.first_mut() {
        faces.pop();
    }
    assert!(export_solids(&[("open", &open)]).is_err());
}

/// A file of `points` points `#10..`, the shared geometry `#2`, the `extra` entities, then `copies`
/// entities `user` (`#G` names the shared geometry), each with its own id, put in one sheet body:
/// faces in an open shell, edges in one loop of one plane face of an open shell.
fn shared_geometry_file(points: usize, geometry: &str, extra: &str, user: &str, copies: usize) -> String {
    let mut text = String::from("ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n");
    for k in 0..points {
        text += &format!("#{}=CARTESIAN_POINT('',({}.,{}.,0.));\n", 10 + k, k / 64, k % 64);
    }
    text += &format!("#2={geometry};\n");
    text += extra;
    let ids: Vec<String> = (0..copies).map(|c| format!("#{}", 1_000_000 + c)).collect();
    for id in &ids {
        text += &format!("{id}={};\n", user.replace("#G", "#2"));
    }
    let faces = if user.starts_with("EDGE_CURVE") {
        let oriented: Vec<String> = (0..copies).map(|c| format!("#{}", 3_000_000 + c)).collect();
        for (o, e) in oriented.iter().zip(&ids) {
            text += &format!("{o}=ORIENTED_EDGE('',*,*,{e},.T.);\n");
        }
        text += &format!("#900001=EDGE_LOOP('',({}));\n#900002=FACE_BOUND('',#900001,.T.);\n", oriented.join(","));
        text += "#900003=PLANE('',#900004);\n#900004=AXIS2_PLACEMENT_3D('',#900005,$,$);\n#900005=CARTESIAN_POINT('',(0.,0.,0.));\n";
        text += "#900006=ADVANCED_FACE('',(#900002),#900003,.T.);\n";
        "#900006".to_string()
    } else {
        ids.join(",")
    };
    text += &format!("#900010=OPEN_SHELL('',({faces}));\n#900011=SHELL_BASED_SURFACE_MODEL('',(#900010));\n");
    text + "ENDSEC;\nEND-ISO-10303-21;\n"
}

fn point_grid(rows: usize, cols: usize) -> String {
    (0..rows)
        .map(|i| format!("({})", (0..cols).map(|j| format!("#{}", 10 + i * cols + j)).collect::<Vec<_>>().join(",")))
        .collect::<Vec<_>>()
        .join(",")
}

fn point_list(n: usize) -> String {
    (0..n).map(|k| format!("#{}", 10 + k)).collect::<Vec<_>>().join(",")
}

fn assert_refused_before_reading(text: &str, what: &str) {
    let e = import(text.as_bytes()).map(|b| b.len()).expect_err(what).to_string();
    assert!(e.contains("geometry"), "{what}: {e}");
}

/// The estimate counts whatever geometry a face or an edge refers to: a Bézier surface shared by
/// many faces, a deep chain of trimmed surfaces, a large B-spline profile swept and shared, and a
/// large curve shared by many edges are each refused before the reader builds anything.
#[test]
fn shared_or_deep_geometry_is_refused_before_reading() {
    let (n, faces) = (64usize, check::MAX_FACE_POINTS / (64 * 64) + 2);
    let bezier = format!("BEZIER_SURFACE('',3,3,({}),.UNSPECIFIED.,.F.,.F.,.F.)", point_grid(n, n));
    assert_refused_before_reading(
        &shared_geometry_file(n * n, &bezier, "", "ADVANCED_FACE('',(),#G,.T.)", faces),
        "a shared Bezier surface",
    );
    let mut chain = String::new();
    for k in 0..20 {
        let basis = if k == 0 { 2 } else { 500_000 + k - 1 };
        chain += &format!("#{}=RECTANGULAR_TRIMMED_SURFACE('',#{basis},0.,1.,0.,1.,.T.,.T.);\n", 500_000 + k);
    }
    let plane = "PLANE('',#3)";
    let deep = shared_geometry_file(
        0,
        plane,
        &(chain + "#3=AXIS2_PLACEMENT_3D('',#4,$,$);\n#4=CARTESIAN_POINT('',(0.,0.,0.));\n"),
        "ADVANCED_FACE('',(),#500019,.T.)",
        1,
    );
    // the estimate counts the whole chain (it follows deeper than the reader); the reader refuses
    // a chain past its own depth without building it
    let e = import(deep.as_bytes()).map(|b| b.len()).expect_err("a deep chain").to_string();
    assert!(e.contains("nested more than"), "{e}");
    let curve = format!(
        "B_SPLINE_CURVE_WITH_KNOTS('',1,({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
        point_list(n * n),
        std::iter::once("2".to_string())
            .chain(std::iter::repeat_n("1".to_string(), n * n - 2))
            .chain(std::iter::once("2".to_string()))
            .collect::<Vec<_>>()
            .join(","),
        (0..n * n - 1).map(|k| format!("{k}.")).collect::<Vec<_>>().join(",")
    );
    let sweep = "#3=SURFACE_OF_LINEAR_EXTRUSION('',#2,#5);\n#5=VECTOR('',#6,1.);\n#6=DIRECTION('',(0.,0.,1.));\n";
    assert_refused_before_reading(
        &shared_geometry_file(n * n, &curve, sweep, "ADVANCED_FACE('',(),#3,.T.)", faces),
        "a swept profile",
    );
    assert_refused_before_reading(
        &shared_geometry_file(n * n, &curve, "#7=VERTEX_POINT('',#10);\n", "EDGE_CURVE('',#7,#7,#G,.T.)", faces),
        "a curve shared by many edges",
    );
    // one heavy shell (one face on the Bezier surface) that many solids reuse
    let shell = "#3=ADVANCED_FACE('',(),#2,.T.);\n#4=CLOSED_SHELL('',(#3));\n";
    let mut reused = shared_geometry_file(n * n, &bezier, shell, "MANIFOLD_SOLID_BREP('',#4)", faces);
    reused = reused.replace("#900010=OPEN_SHELL('',(", "#900010=OPEN_SHELL('',(#3));\n#900012=X('',(");
    assert_refused_before_reading(&reused, "a shell reused by many solids");
}
