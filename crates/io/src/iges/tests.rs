//! Tests of the IGES writer: the sections written for solids and sheets, and refusals of bad input.

use super::*;

use surfacecraft_kernel::nurbs::{fit_grid, sheet_from_spline};

fn sheet() -> Brep {
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

/// A directory entry: (type, parameter pointer, parameter line count, form).
struct De {
    typ: i64,
    pstart: usize,
    pcount: usize,
    form: i64,
}

/// Checks the fixed layout and returns the directory entries and the parameter lines
/// (data columns 1–64, DE back pointer).
fn parse(text: &str) -> (Vec<De>, Vec<(String, usize)>) {
    let lines: Vec<&str> = text.lines().collect();
    let mut counts: HashMap<char, usize> = HashMap::new();
    let mut last = ' ';
    for l in &lines {
        assert_eq!(l.len(), 80, "line is not 80 columns: {l:?}");
        let sect = l.as_bytes()[72] as char;
        assert!("SGDPT".contains(sect), "{l:?}");
        assert!("SGDPT".find(sect) >= "SGDPT".find(last) || last == ' ', "sections out of order");
        last = sect;
        let seq: usize = l[73..].trim().parse().unwrap();
        let c = counts.entry(sect).or_default();
        *c += 1;
        assert_eq!(seq, *c, "sequence numbers of {sect}");
    }
    let t = lines.last().unwrap();
    assert_eq!(t.as_bytes()[72], b'T');
    assert_eq!(counts[&'T'], 1);
    let field = |i: usize| -> usize { t[i * 8 + 1..i * 8 + 8].trim().parse().unwrap() };
    assert_eq!(field(0), counts[&'S']);
    assert_eq!(field(1), counts[&'G']);
    assert_eq!(field(2), counts[&'D']);
    assert_eq!(field(3), counts[&'P']);
    let d: Vec<&&str> = lines.iter().filter(|l| l.as_bytes()[72] == b'D').collect();
    assert_eq!(d.len() % 2, 0);
    let f = |l: &str, i: usize| -> i64 { l[i * 8..i * 8 + 8].trim().parse().unwrap() };
    let des: Vec<De> = d
        .chunks(2)
        .map(|p| {
            assert_eq!(f(p[0], 0), f(p[1], 0), "both directory lines name one type");
            De { typ: f(p[0], 0), pstart: f(p[0], 1) as usize, pcount: f(p[1], 3) as usize, form: f(p[1], 4) }
        })
        .collect();
    let p: Vec<(String, usize)> = lines
        .iter()
        .filter(|l| l.as_bytes()[72] == b'P')
        .map(|l| (l[..64].to_string(), l[64..72].trim().parse().unwrap()))
        .collect();
    for (k, de) in des.iter().enumerate() {
        let first = &p[de.pstart - 1];
        assert_eq!(first.1, 2 * k + 1, "parameter line points back at its entry");
        let typ: i64 = first.0.split([',', ';']).next().unwrap().trim().parse().unwrap();
        assert_eq!(typ, de.typ, "entry {k}'s parameters start with its type");
        for i in 0..de.pcount {
            assert_eq!(p[de.pstart - 1 + i].1, 2 * k + 1);
        }
    }
    (des, p)
}

/// The numeric parameters of the entity at directory index `k`.
fn params(des: &[De], p: &[(String, usize)], k: usize) -> Vec<f64> {
    let de = &des[k];
    let text: String = p[de.pstart - 1..de.pstart - 1 + de.pcount].iter().map(|l| l.0.as_str()).collect();
    let body = text.split(';').next().unwrap();
    body.split(',').skip(1).map(|s| s.trim().parse::<f64>().unwrap()).collect()
}

#[test]
fn filleted_box_is_one_manifold_solid() {
    let b = surfacecraft_kernel::solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
    let f = surfacecraft_kernel::solid::fillet(&b, &[0], 2.0).unwrap();
    assert_eq!(f.faces.len(), 7);
    let text = write_iges("filleted box", &[&f]).unwrap();
    let (des, p) = parse(&text);
    assert_eq!(des.iter().filter(|d| d.typ == 186).count(), 1);
    assert_eq!(des.iter().filter(|d| d.typ == 510).count(), 7);
    let shells: Vec<usize> = (0..des.len()).filter(|&k| des[k].typ == 514).collect();
    assert_eq!(shells.len(), 1);
    assert_eq!(des[shells[0]].form, 1);
    assert_eq!(params(&des, &p, shells[0])[0], 7.0, "the shell lists 7 faces");
    assert!(text.contains("2HMM"));
    // every face has its outer loop flagged
    for k in (0..des.len()).filter(|&k| des[k].typ == 510) {
        assert_eq!(params(&des, &p, k)[2], 1.0, "face {k} has an outer loop");
    }
    let kinds: std::collections::BTreeSet<i64> = des.iter().map(|d| d.typ).collect();
    eprintln!("filleted box IGES entity types: {kinds:?}");
    // every entity a 510 face names is a surface
    for k in (0..des.len()).filter(|&k| des[k].typ == 510) {
        let sd = params(&des, &p, k)[0] as usize;
        assert_eq!(des[(sd - 1) / 2].typ, 128);
    }
}

#[test]
fn sheet_is_an_open_shell_with_exact_geometry() {
    let b = sheet();
    let text = write_iges("wave", &[&b]).unwrap();
    let (des, p) = parse(&text);
    let shells: Vec<usize> = (0..des.len()).filter(|&k| des[k].typ == 514).collect();
    assert_eq!(shells.len(), 1);
    assert_eq!(des[shells[0]].form, 2);
    assert_eq!(des.iter().filter(|d| d.typ == 186).count(), 0);
    assert_eq!(des.iter().filter(|d| d.typ == 126).count(), 4);
    // geometry round trip of the 128 surface
    let k = (0..des.len()).find(|&k| des[k].typ == 128).unwrap();
    let v = params(&des, &p, k);
    let Surface::Spline { u_knots, v_knots, points } = &b.faces[0].surface else { panic!("spline sheet") };
    let (k1, k2, m1, m2) = (v[0] as usize, v[1] as usize, v[2] as usize, v[3] as usize);
    assert_eq!((k1 + 1, k2 + 1), (points.len(), points[0].len()));
    assert_eq!(v[6], 1.0, "polynomial");
    let (nu, nv) = (k1 + 1, k2 + 1);
    let mut at = 9;
    let ku = &v[at..at + nu + m1 + 1];
    at += nu + m1 + 1;
    let kv = &v[at..at + nv + m2 + 1];
    at += nv + m2 + 1;
    assert!(ku.iter().zip(u_knots).all(|(a, b)| (a - b).abs() < 1e-12));
    assert!(kv.iter().zip(v_knots).all(|(a, b)| (a - b).abs() < 1e-12));
    assert!(v[at..at + nu * nv].iter().all(|w| *w == 1.0));
    at += nu * nv;
    let mut max: f64 = 0.0;
    for j in 0..nv {
        for (i, row) in points.iter().enumerate() {
            let q = &v[at + 3 * (j * nu + i)..at + 3 * (j * nu + i) + 3];
            for c in 0..3 {
                max = max.max((q[c] - row[j][c]).abs());
            }
        }
    }
    assert!(max <= 1e-9, "control points differ by {max}");
    assert_eq!(v.len(), at + 3 * nu * nv + 4);
}

#[test]
fn hostile_input_is_refused() {
    assert!(write_iges("x", &[]).is_err());
    // a face on a cylinder without its NURBS form
    let mut b = sheet();
    b.faces[0].surface = Surface::Cylinder {
        frame: surfacecraft_kernel::record::brep::Frame {
            origin: [0.0; 3],
            x_axis: [1.0, 0.0, 0.0],
            y_axis: [0.0, 1.0, 0.0],
        },
        radius: 1.0,
    };
    match write_iges("x", &[&b]) {
        Err(IoError::Export(m)) => assert!(m.contains("without its NURBS form"), "{m}"),
        other => panic!("{other:?}"),
    }
    // an edge on a procedural intersection curve has no exact IGES form
    let mut b = sheet();
    let plane =
        |z: f64| Box::new(Surface::Plane { origin: [0.0, 0.0, z], u_axis: [1.0, 0.0, 0.0], v_axis: [0.0, 1.0, 0.0] });
    let leader = Box::new(b.edges[0].curve.clone());
    b.edges[0].curve = Curve::SurfaceIntersection { surfaces: [plane(0.0), plane(1.0)], leader, tolerance: 1e-6 };
    match write_iges("x", &[&b]) {
        Err(IoError::Export(m)) => assert!(m.contains("surface intersection"), "{m}"),
        other => panic!("{other:?}"),
    }
    // dangling references and non-finite numbers
    let mut b = sheet();
    if let Loop::Edges { edges } = &mut b.faces[0].loops[0] {
        edges[0].edge = 99;
    }
    assert!(write_iges("x", &[&b]).is_err());
    let mut b = sheet();
    b.vertices[0].point[0] = f64::NAN;
    assert!(write_iges("x", &[&b]).is_err());
    let mut b = sheet();
    b.shells.clear();
    assert!(write_iges("x", &[&b]).is_err());
    // names are sanitised to printable ASCII
    let text = write_iges("ż\u{1}", &[&sheet()]).unwrap();
    assert!(text.contains("2H__"));
}

#[test]
fn helpers() {
    assert_eq!(real(1.0), "1.0");
    assert_eq!(real(1e-20), "1.E-20");
    assert_eq!(holl("MM"), "2HMM");
    let w = wrap(&["a"; 40], ',', ';', 10);
    assert!(w.iter().all(|l| l.len() <= 10));
    assert_eq!(w.concat(), format!("{};", ["a"; 40].join(",")));
}
