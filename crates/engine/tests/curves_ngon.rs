//! P8-H9 gate 3: `surface.from_curves` on regions that are not four-sided. A cube cut by the plane
//! x + y + z = s has three triangular, three pentagonal and one hexagonal face; every face
//! bulges outward by a smooth, analytic dome that vanishes (with its slope) at the face edges,
//! so the faces are curved and meet along the sharp edges of the polyhedron.
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

/// Half edge of the cube (mm) and the position of the cutting plane x + y + z = CUT.
const HALF: f64 = 20.0;
const CUT: f64 = 10.0;
/// Height of the domes (mm).
const BULGE: f64 = 2.0;
/// Subdivision levels of a face for the scan and for the reference integral.
const MESH_LEVELS: usize = 5;
const QUADRATURE_LEVELS: usize = 7;

type Polygon = Vec<Vec3>;

/// Clips `poly` to the half-space `n · p <= d` (Sutherland–Hodgman).
fn clip(poly: &Polygon, n: &Vec3, d: f64) -> Polygon {
    let mut out = Vec::new();
    for k in 0..poly.len() {
        let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
        let (sa, sb) = (n.dot(&a) - d, n.dot(&b) - d);
        if sa <= 0.0 {
            out.push(a);
        }
        if (sa < 0.0 && sb > 0.0) || (sa > 0.0 && sb < 0.0) {
            out.push(a + (b - a) * (sa / (sa - sb)));
        }
    }
    out
}

/// The faces of the cut cube as convex polygons, counter-clockwise seen from outside, with
/// their outward normals.
fn faces() -> Vec<(Polygon, Vec3)> {
    let planes: Vec<(Vec3, f64)> = vec![
        (Vec3::x(), HALF),
        (-Vec3::x(), HALF),
        (Vec3::y(), HALF),
        (-Vec3::y(), HALF),
        (Vec3::z(), HALF),
        (-Vec3::z(), HALF),
        (Vec3::new(1.0, 1.0, 1.0).normalize(), CUT / 3f64.sqrt()),
    ];
    let mut out = Vec::new();
    for (i, (n, d)) in planes.iter().enumerate() {
        let u = if n.x.abs() < 0.9 { Vec3::x() } else { Vec3::y() };
        let e1 = (u - n * u.dot(n)).normalize();
        let e2 = n.cross(&e1);
        let c = n * *d;
        let big = 200.0;
        let mut poly: Polygon =
            vec![c - e1 * big - e2 * big, c + e1 * big - e2 * big, c + e1 * big + e2 * big, c - e1 * big + e2 * big];
        for (j, (m, dm)) in planes.iter().enumerate() {
            if i != j {
                poly = clip(&poly, m, *dm);
            }
        }
        if poly.len() >= 3 {
            out.push((poly, *n));
        }
    }
    out
}

/// Distances of `p` from the edge lines of the convex polygon `poly` (inside: all positive).
fn edge_distances(poly: &Polygon, n: &Vec3, p: &Vec3) -> Vec<f64> {
    (0..poly.len())
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
            let inward = n.cross(&(b - a)).normalize();
            (p - a).dot(&inward)
        })
        .collect()
}

/// Height of the dome over the face at `p`: a product of C¹ ramps of the edge distances.
fn dome(poly: &Polygon, n: &Vec3, scale: f64, p: &Vec3) -> f64 {
    BULGE
        * edge_distances(poly, n, p)
            .iter()
            .map(|d| (std::f64::consts::FRAC_PI_2 * (d / scale).clamp(0.0, 1.0)).sin().powi(2))
            .product::<f64>()
}

/// Midpoint subdivision of the fan of `poly` around its centroid: triangles of about 1 mm.
fn fan_triangles(poly: &Polygon, levels: usize) -> Vec<[Vec3; 3]> {
    let centre = poly.iter().sum::<Vec3>() / poly.len() as f64;
    let mut tris: Vec<[Vec3; 3]> = (0..poly.len()).map(|k| [centre, poly[k], poly[(k + 1) % poly.len()]]).collect();
    for _ in 0..levels {
        tris = tris
            .iter()
            .flat_map(|[a, b, c]| {
                let (ab, bc, ca) = ((a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5);
                [[*a, ab, ca], [ab, *b, bc], [ca, bc, *c], [ab, bc, ca]]
            })
            .collect();
    }
    tris
}

fn inradius(poly: &Polygon, n: &Vec3) -> f64 {
    let c = poly.iter().sum::<Vec3>() / poly.len() as f64;
    edge_distances(poly, n, &c).into_iter().fold(f64::INFINITY, f64::min)
}

/// The part as a triangle mesh and its exact volume.
fn part() -> (Mesh, f64) {
    let mut mesh = Mesh { positions: Vec::new(), triangles: Vec::new() };
    let (mut polyhedron, mut domes) = (0.0, 0.0);
    for (poly, n) in faces() {
        let scale = 0.95 * inradius(&poly, &n);
        let c0 = poly[0];
        let mut area = 0.0;
        for k in 1..poly.len() - 1 {
            area += (poly[k] - c0).cross(&(poly[k + 1] - c0)).norm() / 2.0;
        }
        polyhedron += area * n.dot(&c0) / 3.0;
        for [a, b, c] in fan_triangles(&poly, QUADRATURE_LEVELS) {
            let g = (a + b + c) / 3.0;
            domes += dome(&poly, &n, scale, &g) * (b - a).cross(&(c - a)).norm() / 2.0;
        }
        let mut face = Mesh { positions: Vec::new(), triangles: Vec::new() };
        for [a, b, c] in fan_triangles(&poly, MESH_LEVELS) {
            let base = face.positions.len() as u32;
            for p in [a, b, c] {
                face.positions.push(Point3::from(p + n * dome(&poly, &n, scale, &p)));
            }
            face.triangles.push([base, base + 1, base + 2]);
        }
        // wind outward
        for t in &mut face.triangles {
            let (a, b, c) =
                (face.positions[t[0] as usize], face.positions[t[1] as usize], face.positions[t[2] as usize]);
            if (b - a).cross(&(c - a)).dot(&n) < 0.0 {
                t.swap(1, 2);
            }
        }
        mesh.append(&face);
    }
    mesh.weld(1e-9);
    (mesh, polyhedron + domes)
}

fn mesh_volume(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|i| m.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

/// The creases of 54.7° between the cut face and the triangles are blurred by the resampling of
/// the scan over two triangles of about 27° each, so the sharp-edge angle is lowered to 20°.
/// Gate P8-H9(3): triangular, pentagonal and hexagonal regions become patches; the network is
/// closed, within 0.2 mm of the scan and its volume within 0.1 % of the exact one.
#[test]
fn gate_p8_h9_from_curves_ngon_regions() {
    let (cad, exact) = part();
    eprintln!(
        "faces (corners): {:?}; CAD triangles {}",
        faces().iter().map(|f| f.0.len()).collect::<Vec<_>>(),
        cad.triangle_count()
    );
    let mut s = Session::new();
    let defects = synth::ScanDefects {
        spacing: 0.9,
        density_ramp: 0.6,
        noise: 0.025,
        holes: 2,
        hole_radius: 1.5,
        spikes: 0.0003,
        debris: 3,
    };
    s.doc.add_mesh("scan", synth::scanify(&cad, &defects, &mut Rng::new(21)));
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    eprintln!(
        "CAD mesh volume {:.2} vs exact {exact:.2} ({:.4} %)",
        mesh_volume(&cad),
        100.0 * (mesh_volume(&cad) - exact).abs() / exact
    );
    assert_eq!(c["holes_left"], 0, "{c}");
    let r = s.execute("surface.from_curves", json!({"mesh": "scan", "angle": 20.0})).unwrap();
    eprintln!("from_curves: {r}");
    let v = r["volume"].as_f64().unwrap_or(f64::NAN);
    let verr = 100.0 * (v - exact).abs() / exact;
    eprintln!("volume {v:.2} vs exact {exact:.2} ({verr:.4} %), max {}", r["max"]);
    assert_eq!(r["regions"], 7, "{r}");
    assert!(r["skipped"].as_array().unwrap().is_empty(), "{r}");
    assert_eq!(r["closed"], true, "{r}");
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    assert!(verr <= 0.1, "volume error {verr} %");
    assert!(r["solid"].as_u64().is_some());
}
