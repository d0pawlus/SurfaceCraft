//! P8-H9 gates: layouts that follow the ragged boundary of an open region, control of flipped
//! patches, and patches from feature curves for regions with 3, 5 and 6 corners. The deviation
//! threshold is that of design doc §11.1, unchanged; every metric is printed before the first
//! assertion so one failure does not hide the other numbers.
use serde_json::{Value, json};
use std::collections::{BTreeMap, BTreeSet};
use std::f64::consts::PI;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth};

// ---------------------------------------------------------------------------------------------
// shapes

/// Bumpy half pipe wrapping `sweep` radians around the z axis (the shape of `gate_p4_open_wrapped_patch`).
fn pipe_mesh(noise: f64, seed: u64, sweep: f64) -> Mesh {
    let (nu, nv, height) = (220usize, 160usize, 40.0);
    let point = |theta: f64, z: f64| {
        let u = theta / sweep;
        let r = 15.0 * (1.0 + 0.10 * (2.0 * PI * u * 1.5).sin() * (z / 9.0).cos() + 0.04 * (z / 5.0).sin());
        Point3::new(r * theta.cos(), r * theta.sin(), z)
    };
    let mut positions = Vec::new();
    for j in 0..=nv {
        for i in 0..=nu {
            positions.push(point(sweep * i as f64 / nu as f64, height * j as f64 / nv as f64));
        }
    }
    let mut m = Mesh { positions, triangles: grid_triangles(nu, nv) };
    synth::add_noise(&mut m, noise, &mut Rng::new(seed));
    m
}

/// A smooth open plate (saddle with a bump), a height field over 50 × 40 mm.
fn plate_mesh(noise: f64, seed: u64) -> Mesh {
    let (nu, nv) = (180usize, 140usize);
    let mut positions = Vec::new();
    for j in 0..=nv {
        for i in 0..=nu {
            let (x, y) = (50.0 * i as f64 / nu as f64 - 25.0, 40.0 * j as f64 / nv as f64 - 20.0);
            let z = 4.0 * (x / 9.0).sin() * (y / 7.0).cos() + 0.012 * x * x - 0.01 * y * y;
            positions.push(Point3::new(x, y, z));
        }
    }
    let mut m = Mesh { positions, triangles: grid_triangles(nu, nv) };
    synth::add_noise(&mut m, noise, &mut Rng::new(seed));
    m
}

fn grid_triangles(nu: usize, nv: usize) -> Vec<[u32; 3]> {
    let w = (nu + 1) as u32;
    let mut t = Vec::new();
    for j in 0..nv as u32 {
        for i in 0..nu as u32 {
            let a = j * w + i;
            t.push([a, a + 1, a + w + 1]);
            t.push([a, a + w + 1, a + w]);
        }
    }
    t
}

// ---------------------------------------------------------------------------------------------
// ragged regions

/// Boundary edges of a triangle set (edges used once) per vertex.
fn boundary_degree(mesh: &Mesh, keep: &BTreeSet<u32>) -> (BTreeMap<u32, usize>, BTreeMap<u32, usize>) {
    let mut uses: BTreeMap<(u32, u32), usize> = BTreeMap::new();
    for &t in keep {
        let tri = mesh.triangles[t as usize];
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let mut per_vertex: BTreeMap<u32, usize> = BTreeMap::new();
    let mut per_triangle: BTreeMap<u32, usize> = BTreeMap::new();
    for &t in keep {
        let tri = mesh.triangles[t as usize];
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            if uses[&(a.min(b), a.max(b))] == 1 {
                *per_vertex.entry(a).or_insert(0) += 1;
                *per_vertex.entry(b).or_insert(0) += 1;
                *per_triangle.entry(t).or_insert(0) += 1;
            }
        }
    }
    (per_vertex, per_triangle)
}

/// Removes triangles that would leave a bow-tie vertex (more than two boundary edges) or a
/// triangle held by one edge only, until the region is a manifold disc.
fn make_manifold(mesh: &Mesh, keep: &mut BTreeSet<u32>) {
    loop {
        let (per_vertex, per_triangle) = boundary_degree(mesh, keep);
        let mut remove: Vec<u32> = per_triangle.iter().filter(|&(_, &c)| c == 3).map(|(&t, _)| t).collect();
        for (&v, &c) in &per_vertex {
            if c > 2
                && let Some(&t) = keep.iter().find(|&&t| mesh.triangles[t as usize].contains(&v))
            {
                remove.push(t);
            }
        }
        // fans of triangles joined by a vertex only
        let comps = components(mesh, keep);
        if comps.len() > 1 {
            let biggest = comps.iter().map(Vec::len).max().unwrap_or(0);
            for c in comps.iter().filter(|c| c.len() < biggest) {
                remove.extend(c.iter().copied());
            }
        }
        if remove.is_empty() {
            return;
        }
        for t in remove {
            keep.remove(&t);
        }
    }
}

/// Edge-connected components of a triangle set.
fn components(mesh: &Mesh, keep: &BTreeSet<u32>) -> Vec<Vec<u32>> {
    let mut by_edge: BTreeMap<(u32, u32), Vec<u32>> = BTreeMap::new();
    for &t in keep {
        let tri = mesh.triangles[t as usize];
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(t);
        }
    }
    let mut seen: BTreeSet<u32> = BTreeSet::new();
    let mut out = Vec::new();
    for &s in keep {
        if seen.contains(&s) {
            continue;
        }
        let mut comp = vec![s];
        seen.insert(s);
        let mut i = 0;
        while i < comp.len() {
            let tri = mesh.triangles[comp[i] as usize];
            for k in 0..3 {
                let (a, b) = (tri[k], tri[(k + 1) % 3]);
                for &n in &by_edge[&(a.min(b), a.max(b))] {
                    if seen.insert(n) {
                        comp.push(n);
                    }
                }
            }
            i += 1;
        }
        out.push(comp);
    }
    out
}

/// A ragged region of `mesh`: boundary triangles are removed pseudo-randomly over several
/// rounds (fixed seed), and `notch` (a predicate on the triangle centroid) is cut out.
fn ragged_region(mesh: &Mesh, seed: u64, rounds: usize, notch: &dyn Fn(&Point3) -> bool) -> Vec<u32> {
    let mut keep: BTreeSet<u32> = (0..mesh.triangle_count() as u32)
        .filter(|&t| mesh.face_centroid(t as usize).is_some_and(|c| !notch(&c)))
        .collect();
    let mut rng = Rng::new(seed);
    for _ in 0..rounds {
        let (_, per_triangle) = boundary_degree(mesh, &keep);
        for &t in per_triangle.keys() {
            if rng.uniform() < 0.45 {
                keep.remove(&t);
            }
        }
        make_manifold(mesh, &mut keep);
    }
    keep.into_iter().collect()
}

fn region_vertices(mesh: &Mesh, tris: &[u32]) -> Vec<Point3> {
    let ids: BTreeSet<u32> = tris.iter().flat_map(|&t| mesh.triangles[t as usize]).collect();
    ids.into_iter().map(|v| mesh.positions[v as usize]).collect()
}

// ---------------------------------------------------------------------------------------------
// gate 1

struct Measured {
    outside: f64,
    max_all: f64,
    jump_max: f64,
    flipped: usize,
    ret: Value,
    auto: Value,
}

fn measure(s: &mut Session, mesh: &Mesh, tris: &[u32], target: usize) -> Measured {
    let id = s.doc.add_mesh("scan", mesh.clone());
    let region = s.doc.add_region("ragged", id, tris.to_vec());
    let ret = s.execute("quad.retopo", json!({"region": region.0, "target_quads": target})).unwrap();
    let auto = s.execute("surface.auto", json!({"object": ret["id"], "tolerance": 0.2})).unwrap();
    let sid = auto["blocks"][0]["id"].as_u64().unwrap();
    let surf = s.doc.surfaces.iter().find(|o| o.id.0 == sid).unwrap().clone();
    let tess = surfacecraft_kernel::tess::tessellate(&surf.sheet, Some(0.005)).unwrap();
    let bvh = Bvh::build(&tess);
    let pts = region_vertices(mesh, tris);
    let max_all = pts.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).fold(0.0, f64::max);
    let mut worst: Vec<(f64, Point3)> =
        pts.iter().filter_map(|p| bvh.closest(p, f64::INFINITY).map(|h| (h.distance, *p))).collect();
    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, p) in worst.iter().take(8) {
        eprintln!("  worst {d:.3} mm at ({:.2}, {:.2}, {:.2})", p.x, p.y, p.z);
    }
    Measured {
        outside: auto["uncovered"].as_f64().unwrap(),
        max_all,
        jump_max: auto["normal_jump_max_deg"].as_f64().unwrap(),
        flipped: auto["flipped_patches"].as_u64().unwrap_or(u64::MAX) as usize,
        ret,
        auto,
    }
}

fn check_ragged(name: &str, mesh: &Mesh, tris: &[u32], target: usize) {
    let mut s = Session::new();
    let m = measure(&mut s, mesh, tris, target);
    eprintln!(
        "{name}: {} region triangles; outside the layout {:.3} %; max deviation over ALL region points {:.4} mm; \
         normal jump max {:.2} deg; flipped {}\n  retopo {}\n  auto {}",
        tris.len(),
        100.0 * m.outside,
        m.max_all,
        m.jump_max,
        m.flipped,
        m.ret,
        m.auto
    );
    assert!(m.outside < 0.005, "{name}: {:.3} % of the points are outside the layout", 100.0 * m.outside);
    assert!(m.max_all <= 0.2, "{name}: max deviation including boundary points {}", m.max_all);
    assert!(m.jump_max < 30.0, "{name}: normal jump {}", m.jump_max);
    assert_eq!(m.flipped, 0, "{name}: flipped patches");
}

/// Gate P8-H9(1): a 240° wrapped open surface whose region boundary is ragged (boundary
/// triangles removed with a fixed seed) and has a concave notch gets a layout that covers the
/// region up to its outline.
#[test]
fn gate_p8_h9_ragged_wrapped_region() {
    let sweep = 240f64.to_radians();
    let mesh = pipe_mesh(0.025, 4, sweep);
    let theta0 = 0.3 * sweep;
    let notch = |c: &Point3| {
        let arc = (c.y.atan2(c.x) - theta0) * 15.0;
        c.z > 33.0 && arc.abs() < 3.5
    };
    let tris = ragged_region(&mesh, 17, 5, &notch);
    check_ragged("pipe", &mesh, &tris, 80);
}

/// Gate P8-H9(1b): the same on a smooth open plate.
#[test]
fn gate_p8_h9_ragged_open_plate() {
    let mesh = plate_mesh(0.025, 9);
    let notch = |c: &Point3| c.x > 14.0 && c.y.abs() < 3.0;
    let tris = ragged_region(&mesh, 23, 5, &notch);
    check_ragged("plate", &mesh, &tris, 80);
}

// ---------------------------------------------------------------------------------------------
// gate 2

fn blob_radius(d: &Vec3) -> f64 {
    20.0 * (1.0
        + 0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos()
        + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos()
        + 0.05 * (4.0 * d.z).cos() * d.x)
}

fn blob_scan(noise: f64, seed: u64) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, 200, 100);
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * blob_radius(&d);
    }
    let d = synth::ScanDefects {
        spacing: 0.9,
        density_ramp: 0.6,
        noise,
        holes: 2,
        hole_radius: 1.5,
        spikes: 0.0003,
        debris: 3,
    };
    synth::scanify(&m, &d, &mut Rng::new(seed))
}

/// Gate P8-H9(2): a layout that needed the 4-to-1 split (non-quad field faces) with one quad
/// turned around in the document: `surface.auto` repairs exactly that patch and returns a
/// network without flipped patches.
#[test]
fn gate_p8_h9_flipped_patch_after_split_is_repaired() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", blob_scan(0.025, 11));
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert_eq!(c["holes_left"], 0, "{c}");
    let mut chosen = None;
    for target in [60usize, 40, 80, 100, 50, 70] {
        let r = s.execute("quad.retopo", json!({"mesh": "scan", "target_quads": target})).unwrap();
        eprintln!("target {target}: field faces {} quads {} -> {r}", r["field_faces"], r["field_quads"]);
        if r["field_faces"].as_u64() > r["field_quads"].as_u64() {
            chosen = Some(r);
            break;
        }
    }
    let r = chosen.expect("no target gave a layout with non-quad field faces (the 4-to-1 split never happened)");
    let id = surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap());
    // the undamaged layout needs no repair
    let clean = s.execute("surface.auto", json!({"object": id.0, "tolerance": 0.2})).unwrap();
    eprintln!("undamaged: {clean}");
    // inject: one quad runs against its normals (after the split, in the stored layout)
    let k = s.doc.quad(id).unwrap().faces.len() / 3;
    s.doc.quad_mut(id).unwrap().faces[k].swap(1, 3);
    let a = s.execute("surface.auto", json!({"object": id.0, "tolerance": 0.2})).unwrap();
    eprintln!("damaged: {a}");
    assert_eq!(clean["repaired_patches"], 0, "{clean}");
    assert_eq!(a["repaired_patches"], 1, "{a}");
    assert_eq!(a["flipped_patches"], 0, "{a}");
    assert_eq!(a["closed"], true, "{a}");
    assert!(a["max"].as_f64().unwrap() <= 0.2, "{a}");
    assert!(a["volume"].as_f64().unwrap() > 0.0, "{a}");
}
