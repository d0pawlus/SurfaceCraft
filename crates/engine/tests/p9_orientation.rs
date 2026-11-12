//! P9-O8 exploration.
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_fit::patch::{edge_normal_jumps, wrap_patches};
use surfacecraft_geom::edit::{DecimateOptions, remove_non_manifold};
use surfacecraft_geom::reduce::{ToleranceOptions, reduce_to_tolerance};
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth};

fn blob(noise: f64, seed: u64, nu: usize) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, nu, nu / 2);
    for p in &mut m.positions {
        let d = p.coords;
        let r = 20.0 * (1.0 + 0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos() + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos());
        p.coords = d * r;
    }
    let dd = synth::ScanDefects { spacing: 0.9, density_ramp: 0.6, noise, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 };
    synth::scanify(&m, &dd, &mut Rng::new(seed))
}

fn flat_count(coarse: &Mesh, scan: &Mesh) -> usize {
    let bvh = Bvh::build(scan);
    (0..coarse.triangle_count())
        .filter(|&i| {
            let (Some([a, b, c]), Some(n)) = (coarse.triangle(i), coarse.face_normal(i)) else { return false };
            let cen = Point3::from((a.coords + b.coords + c.coords) / 3.0);
            bvh.closest(&cen, f64::INFINITY).and_then(|h| bvh.triangle_normal(h.triangle)).is_some_and(|m| m.dot(&n) < 0.0)
        })
        .count()
}

fn pct(v: &mut Vec<f64>, q: f64) -> f64 {
    v.sort_by(f64::total_cmp);
    if v.is_empty() { return 0.0; }
    v[((v.len() - 1) as f64 * q) as usize].to_degrees()
}

fn probe(name: &str, scan: Mesh, tol: f64) {
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan.clone());
    let _ = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400}));
    let w = s.execute("surface.wrap", json!({"mesh": "scan", "tolerance": tol})).unwrap();
    let mut cmp = |f: bool| {
        s.execute("body.compare", json!({"body": w["body"]["id"], "scan": "scan", "tolerance": 0.2, "facing": f}))
            .map(|c| (c["totals"]["count"].clone(), c["totals"]["within"].clone()))
    };
    let (cf, cn) = (cmp(true), cmp(false));
    // coarse mesh as the first round
    let mut coarse = scan.clone();
    let opts = DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    let _ = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(8.0 * tol));
    remove_non_manifold(&mut coarse);
    let sc = scan.clone();
    let bvh = Bvh::build(&sc);
    let set = wrap_patches(&coarse, &bvh, 6.0 * 0.32);
    let mut j: Vec<f64> = edge_normal_jumps(&set);
    let mean = j.iter().sum::<f64>() / j.len().max(1) as f64;
    eprintln!("{name}: patches {} closed {} volume {} within {} jumpmean {:.1} p99 {:.1} max {:.1} flipped-coarse {}/{} facing {:?} nofacing {:?}",
        w["patches"], w["closed"], w["body"]["volume"], w["deviation"]["within"], mean.to_degrees(), pct(&mut j.clone(), 0.99), pct(&mut j, 1.0), flat_count(&coarse, &sc), coarse.triangle_count(), cf, cn);
}

#[test]
fn explore() {
    let _ = Vec3::x();
    probe("blob tol0.2", blob(0.03, 1, 200), 0.2);
    probe("blob tol1.0 noise.08", blob(0.08, 2, 200), 1.0);
    probe("blob tol0.5 noise.1", blob(0.1, 3, 160), 0.5);
    probe("sphere r20", synth::scanify(&synth::sphere(Point3::origin(), 20.0, 200, 100), &synth::ScanDefects { spacing: 0.8, density_ramp: 0.0, noise: 0.05, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }, &mut Rng::new(4)), 0.2);
    probe("rounded plate 40x40x1.5", synth::scanify(&synth::rounded_cuboid(Point3::origin(), Vec3::new(40.0, 40.0, 1.5), 0.6, 30), &synth::ScanDefects { spacing: 0.5, density_ramp: 0.0, noise: 0.05, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }, &mut Rng::new(5)), 0.2);
    probe("torus", synth::scanify(&synth::torus(Point3::origin(), Vec3::z(), 20.0, 4.0, 160, 60), &synth::ScanDefects { spacing: 0.7, density_ramp: 0.0, noise: 0.05, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }, &mut Rng::new(6)), 0.2);
    probe("rounded cuboid", synth::scanify(&synth::rounded_cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 3.0, 30), &synth::ScanDefects { spacing: 0.7, density_ramp: 0.5, noise: 0.08, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }, &mut Rng::new(7)), 0.2);
}

#[test]
fn explore_edges() {
    use surfacecraft_fit::patch::{eval, normal};
    let scan = blob(0.03, 1, 200);
    let mut coarse = scan.clone();
    let opts = DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    let _ = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(0.32));
    remove_non_manifold(&mut coarse);
    let bvh = Bvh::build(&scan);
    let set = wrap_patches(&coarse, &bvh, 6.0 * 0.32);
    let nv = coarse.positions.len() as u32;
    let mut by_edge: std::collections::BTreeMap<(u32, u32), Vec<(usize, usize)>> = Default::default();
    for (fi, c) in set.corners.iter().enumerate() {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push((fi, s));
        }
    }
    let mut shown = 0;
    let (mut inner_bad, mut outer_bad, mut inner, mut outer) = (0, 0, 0, 0);
    for ((a, b), uses) in &by_edge {
        let [(f0, s0), (f1, s1)] = uses.as_slice() else { continue };
        let is_inner = *a >= nv + 0 && *b >= nv && (*a >= nv || *b >= nv) && (*b > nv + coarse.triangle_count() as u32 || true);
        let g_involved = set.corners[*f0].iter().any(|&x| x as usize >= (set.vertices.len() - coarse.triangle_count()));
        let _ = (is_inner, g_involved);
        // edge touches a centre g (id >= vertices before centres) => inner
        let first_g = (set.vertices.len() - coarse.triangle_count()) as u32;
        let inner_edge = *b >= first_g;
        let mut worst = 0.0f64; let mut wt = 0.0; let mut info = String::new();
        for k in 1..8 {
            let t = f64::from(k) / 8.0;
            let fw0 = set.corners[*f0][*s0] < set.corners[*f0][(*s0 + 1) % 4];
            let fw1 = set.corners[*f1][*s1] < set.corners[*f1][(*s1 + 1) % 4];
            let uv = |s: usize, fw: bool| { let t = if fw { t } else { 1.0 - t }; match s {0=>(t,0.0),1=>(1.0,t),2=>(1.0-t,1.0),_=>(0.0,1.0-t)} };
            let (u0, v0) = uv(*s0, fw0); let (u1, v1) = uv(*s1, fw1);
            if let (Some(n0), Some(n1)) = (normal(&set.nets[*f0], u0, v0), normal(&set.nets[*f1], u1, v1)) {
                let ang = n0.dot(&n1).clamp(-1.0, 1.0).acos().to_degrees();
                if ang > worst { worst = ang; wt = t;
                    let (p0, a0, b0) = eval(&set.nets[*f0], u0, v0); let (p1, a1, b1) = eval(&set.nets[*f1], u1, v1);
                    info = format!("p0-p1 {:.2e} |pu0| {:.2} |pv0| {:.2} |pu1| {:.2} |pv1| {:.2} n0 {:?} n1 {:?}", (p0 - p1).norm(), a0.norm(), b0.norm(), a1.norm(), b1.norm(), n0.iter().map(|x| (x*100.0).round()/100.0).collect::<Vec<_>>(), n1.iter().map(|x| (x*100.0).round()/100.0).collect::<Vec<_>>()); }
            }
        }
        if inner_edge { inner += 1 } else { outer += 1 }
        if worst > 60.0 { if inner_edge { inner_bad += 1 } else { outer_bad += 1 } if shown < 6 { shown += 1; eprintln!("edge {a}-{b} inner {inner_edge} patches {f0}/{s0} {f1}/{s1} worst {worst:.1} at t {wt}: {info}"); } }
    }
    eprintln!("inner {inner_bad}/{inner} outer {outer_bad}/{outer}");
}

fn big_blob(nu: usize, noise: f64) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, nu, nu / 2);
    for p in &mut m.positions {
        let d = p.coords;
        let r = 20.0 * (1.0 + 0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos() + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos());
        p.coords = d * r;
    }
    synth::add_noise(&mut m, noise, &mut Rng::new(9));
    m
}

#[test]
fn explore_big() {
    probe("big blob 600", big_blob(600, 0.03), 0.2);
}

fn scaled(mut m: Mesh, k: f64) -> Mesh {
    for p in &mut m.positions {
        p.coords *= k;
    }
    m
}

#[test]
fn explore_small() {
    probe("blob in metres", scaled(big_blob(300, 0.03), 0.001), 0.2);
    probe("blob 5mm", scaled(big_blob(300, 0.03), 0.1), 0.2);
    probe("torus in metres", scaled(synth::torus(Point3::origin(), Vec3::z(), 20.0, 4.0, 200, 80), 0.001), 0.2);
    probe("bracket in metres", scaled(synth::bracket(), 0.001), 0.2);
}

#[test]
fn explore_sphere_patches() {
    use surfacecraft_fit::patch::{eval, normal};
    let scan = synth::scanify(&synth::sphere(Point3::origin(), 20.0, 200, 100), &synth::ScanDefects { spacing: 0.8, density_ramp: 0.0, noise: 0.05, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }, &mut Rng::new(4));
    let mut coarse = scan.clone();
    let opts = DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    let _ = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(1.6));
    remove_non_manifold(&mut coarse);
    let bvh = Bvh::build(&scan);
    let set = wrap_patches(&coarse, &bvh, 6.0 * 1.6);
    eprintln!("coarse tris {} patches {}", coarse.triangle_count(), set.nets.len());
    // per patch: fraction of grid samples whose normal is within 60 deg of the scan normal
    let mut bad_patches = 0; let mut bad_samples = 0; let mut total = 0;
    for (pi, net) in set.nets.iter().enumerate() {
        let mut bad = 0;
        for i in 0..=6 { for j in 0..=6 {
            let (u, v) = (i as f64 / 6.0, j as f64 / 6.0);
            let (pt, _, _) = eval(net, u, v);
            let Some(n) = normal(net, u, v) else { bad += 1; continue };
            let sn = bvh.closest(&Point3::from(pt), f64::INFINITY).and_then(|h| bvh.triangle_normal(h.triangle));
            total += 1;
            if sn.is_some_and(|m| m.dot(&n) < 0.5) { bad += 1; bad_samples += 1; }
        }}
        if bad > 0 { bad_patches += 1; if bad_patches <= 5 { eprintln!("patch {pi} corners {:?} bad samples {bad}/49", set.corners[pi]); } }
    }
    eprintln!("bad patches {bad_patches}/{} bad samples {bad_samples}/{total}", set.nets.len());
}

#[test]
fn explore_sphere_edges() {
    use surfacecraft_fit::patch::{eval, normal};
    let scan = synth::scanify(&synth::sphere(Point3::origin(), 20.0, 200, 100), &synth::ScanDefects { spacing: 0.8, density_ramp: 0.0, noise: 0.05, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }, &mut Rng::new(4));
    let mut coarse = scan.clone();
    let opts = DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    let _ = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(1.6));
    remove_non_manifold(&mut coarse);
    let bvh = Bvh::build(&scan);
    let set = wrap_patches(&coarse, &bvh, 6.0 * 1.6);
    let mut by_edge: std::collections::BTreeMap<(u32, u32), Vec<(usize, usize)>> = Default::default();
    for (fi, c) in set.corners.iter().enumerate() {
        for s in 0..4 { let (a, b) = (c[s], c[(s + 1) % 4]); by_edge.entry((a.min(b), a.max(b))).or_default().push((fi, s)); }
    }
    let mut shown = 0;
    for ((a, b), uses) in &by_edge {
        let [(f0, s0), (f1, s1)] = uses.as_slice() else { continue };
        for k in 1..8 {
            let t = f64::from(k) / 8.0;
            let fw0 = set.corners[*f0][*s0] < set.corners[*f0][(*s0 + 1) % 4];
            let fw1 = set.corners[*f1][*s1] < set.corners[*f1][(*s1 + 1) % 4];
            let uv = |s: usize, fw: bool| { let t = if fw { t } else { 1.0 - t }; match s {0=>(t,0.0),1=>(1.0,t),2=>(1.0-t,1.0),_=>(0.0,1.0-t)} };
            let (u0, v0) = uv(*s0, fw0); let (u1, v1) = uv(*s1, fw1);
            if let (Some(n0), Some(n1)) = (normal(&set.nets[*f0], u0, v0), normal(&set.nets[*f1], u1, v1)) {
                let ang = n0.dot(&n1).clamp(-1.0, 1.0).acos().to_degrees();
                if ang > 45.0 && shown < 12 {
                    shown += 1;
                    let (p0, _, _) = eval(&set.nets[*f0], u0, v0);
                    let sn = p0.normalize();
                    eprintln!("edge {a}-{b} t {t}: jump {ang:.0}, n0 vs sphere {:.0}, n1 vs sphere {:.0}, |p| {:.2}", n0.dot(&sn).acos().to_degrees(), n1.dot(&sn).acos().to_degrees(), p0.norm());
                }
            }
        }
    }
}

fn plain(noise: f64) -> synth::ScanDefects {
    synth::ScanDefects { spacing: 0.5, density_ramp: 0.0, noise, holes: 0, hole_radius: 0.0, spikes: 0.0, debris: 0 }
}

#[test]
fn explore_thin() {
    probe("plate 40x40x1.5", synth::scanify(&synth::rounded_cuboid(Point3::origin(), Vec3::new(40.0, 40.0, 1.5), 0.6, 30), &plain(0.05), &mut Rng::new(5)), 0.2);
    probe("plate 40x40x1.0", synth::scanify(&synth::rounded_cuboid(Point3::origin(), Vec3::new(40.0, 40.0, 1.0), 0.4, 30), &plain(0.05), &mut Rng::new(5)), 0.2);
    probe("thin torus r1", synth::scanify(&synth::torus(Point3::origin(), Vec3::z(), 20.0, 1.0, 200, 40), &plain(0.03), &mut Rng::new(6)), 0.2);
    probe("torus r4", synth::scanify(&synth::torus(Point3::origin(), Vec3::z(), 20.0, 4.0, 160, 60), &plain(0.05), &mut Rng::new(6)), 0.2);
}
