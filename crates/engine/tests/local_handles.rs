//! Diagnosis of a real scan with thousands of small handles and an open rim (P9, the one scan of
//! the local corpus that `scan.to_solid` failed): aggregate numbers only, printed per scan NUMBER.
//! The scans stay outside the repository (`SURFACECRAFT_LOCAL_CORPUS`, third-party files); nothing
//! derived from a single file is stored here. `SURFACECRAFT_LOCAL_SCANS` lists the numbers (1-based,
//! in file order; default `3`). `SURFACECRAFT_LOCAL_TO_SOLID=1` also runs `scan.to_solid` and says
//! where the points out of tolerance lie.
//!
//! Run: `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test local_handles -- --ignored --nocapture`
use serde_json::json;
use std::collections::{BTreeMap, BinaryHeap, HashMap};
use surfacecraft_engine::Session;
use surfacecraft_geom::edit::{DecimateOptions, decimate};
use surfacecraft_geom::{Bvh, KdTree, Mesh, Point3, Vec3};

fn corpus() -> Vec<std::path::PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    files
}

fn numbers() -> Vec<usize> {
    let list = std::env::var("SURFACECRAFT_LOCAL_SCANS").unwrap_or_else(|_| "3".into());
    list.split(',').filter_map(|x| x.trim().parse().ok()).collect()
}

fn neighbours(mesh: &Mesh) -> Vec<Vec<u32>> {
    let mut nb: Vec<Vec<u32>> = vec![Vec::new(); mesh.positions.len()];
    for t in &mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            nb[a as usize].push(b);
            nb[b as usize].push(a);
        }
    }
    for n in &mut nb {
        n.sort_unstable();
        n.dedup();
    }
    nb
}

fn edge_faces(mesh: &Mesh) -> HashMap<(u32, u32), u32> {
    let mut m: HashMap<(u32, u32), u32> = HashMap::with_capacity(mesh.triangles.len() * 2);
    for t in &mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *m.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    m
}

fn median_edge(mesh: &Mesh) -> f64 {
    let mut l: Vec<f64> = edge_faces(mesh)
        .keys()
        .map(|&(a, b)| (mesh.positions[a as usize] - mesh.positions[b as usize]).norm())
        .collect();
    let k = l.len() / 2;
    *l.select_nth_unstable_by(k, f64::total_cmp).1
}

#[derive(PartialEq)]
struct Item(f64, u32, u32);
impl Eq for Item {}
impl PartialOrd for Item {
    fn partial_cmp(&self, o: &Self) -> Option<std::cmp::Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Item {
    fn cmp(&self, o: &Self) -> std::cmp::Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}

/// Voronoi cells along the surface around seeds about `r` apart (one per grid cell of size `r`).
fn cells(mesh: &Mesh, nb: &[Vec<u32>], r: f64) -> Vec<u32> {
    let o = mesh.bbox().min;
    let mut seeds: BTreeMap<[i64; 3], u32> = BTreeMap::new();
    for (i, p) in mesh.positions.iter().enumerate() {
        let k = [((p.x - o.x) / r) as i64, ((p.y - o.y) / r) as i64, ((p.z - o.z) / r) as i64];
        seeds.entry(k).or_insert(i as u32);
    }
    let mut label = vec![u32::MAX; mesh.positions.len()];
    let mut dist = vec![f64::INFINITY; mesh.positions.len()];
    let mut heap = BinaryHeap::new();
    for (n, &s) in seeds.values().enumerate() {
        dist[s as usize] = 0.0;
        heap.push(Item(0.0, s, n as u32));
    }
    while let Some(Item(d, v, l)) = heap.pop() {
        if label[v as usize] != u32::MAX {
            continue;
        }
        label[v as usize] = l;
        for &w in &nb[v as usize] {
            let nd = d + (mesh.positions[v as usize] - mesh.positions[w as usize]).norm();
            if nd < dist[w as usize] && label[w as usize] == u32::MAX {
                dist[w as usize] = nd;
                heap.push(Item(nd, w, l));
            }
        }
    }
    label
}

fn find(p: &mut [u32], mut x: u32) -> u32 {
    while p[x as usize] != x {
        p[x as usize] = p[p[x as usize] as usize];
        x = p[x as usize];
    }
    x
}

/// Genus of a set of triangles: `(2c − χ − b) / 2` with `c` components and `b` boundary loops
/// (components of the boundary edge graph).
fn genus_of(mesh: &Mesh, tris: &[u32]) -> i64 {
    let mut vid: HashMap<u32, u32> = HashMap::new();
    let mut edges: HashMap<(u32, u32), u32> = HashMap::new();
    for &t in tris {
        let t = mesh.triangles[t as usize];
        for k in 0..3 {
            let n = vid.len() as u32;
            vid.entry(t[k]).or_insert(n);
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *edges.entry((a.min(b), a.max(b))).or_default() += 1;
        }
    }
    let nv = vid.len();
    let mut comp: Vec<u32> = (0..nv as u32).collect();
    let mut bnd: Vec<u32> = (0..nv as u32).collect();
    let mut on_b = vec![false; nv];
    for (&(a, b), &c) in &edges {
        let (x, y) = (vid[&a], vid[&b]);
        let (rx, ry) = (find(&mut comp, x), find(&mut comp, y));
        comp[rx as usize] = ry;
        if c == 1 {
            on_b[x as usize] = true;
            on_b[y as usize] = true;
            let (bx, by) = (find(&mut bnd, x), find(&mut bnd, y));
            bnd[bx as usize] = by;
        }
    }
    let c = (0..nv as u32).filter(|&x| find(&mut comp, x) == x).count() as i64;
    let b = (0..nv as u32).filter(|&x| on_b[x as usize] && find(&mut bnd, x) == x).count() as i64;
    let chi = nv as i64 - edges.len() as i64 + tris.len() as i64;
    (2 * c - chi - b) / 2
}

/// Handles that fit in cells of size `r`, and the vertices of the cells that hold one.
fn local_genus(mesh: &Mesh, nb: &[Vec<u32>], r: f64) -> (i64, usize, i64, Vec<bool>) {
    let label = cells(mesh, nb, r);
    let mut by: HashMap<u32, Vec<u32>> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        let l = label[t[0] as usize];
        if l == label[t[1] as usize] && l == label[t[2] as usize] {
            by.entry(l).or_default().push(i as u32);
        }
    }
    let mut with: Vec<bool> = vec![false; by.len().max(1) + seeds_bound(&label)];
    let (mut total, mut cells_with, mut max) = (0, 0, 0);
    for (&l, tris) in &by {
        let g = genus_of(mesh, tris).max(0);
        total += g;
        max = max.max(g);
        if g > 0 {
            cells_with += 1;
            if let Some(w) = with.get_mut(l as usize) {
                *w = true;
            }
        }
    }
    let zone = label.iter().map(|&l| with.get(l as usize).copied().unwrap_or(false)).collect();
    (total, cells_with, max, zone)
}

fn seeds_bound(label: &[u32]) -> usize {
    label.iter().copied().filter(|&l| l != u32::MAX).max().map_or(0, |m| m as usize + 1)
}

/// Distance along ± the normal to the next surface, and whether that surface faces the other way.
fn partner(bvh: &Bvh, p: &Point3, n: &Vec3, dir: f64) -> Option<(f64, bool)> {
    let d = *n * dir;
    let eps = 1e-3;
    let (t, dist) = bvh.ray_cast(&(p + d * eps), &d)?;
    let opposite = bvh.triangle_normal(t).is_some_and(|m| m.dot(n) < -0.5);
    Some((dist + eps, opposite))
}

const BINS: [f64; 8] = [0.3, 0.6, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0];

fn bin(d: f64) -> usize {
    BINS.iter().position(|b| d < *b).unwrap_or(BINS.len())
}

/// Wall thickness: the nearest opposite-facing surface behind (`-n`) and in front (`+n`) of sampled
/// vertices, split into handle zone and the rest.
fn walls(mesh: &Mesh, bvh: &Bvh, zone: &[bool]) {
    let normals = mesh.vertex_normals();
    let n = mesh.positions.len();
    let samples = 40_000.min(n);
    for (dir, side) in [(-1.0, "behind (-n)"), (1.0, "in front (+n)")] {
        let mut hist = [[0usize; 9]; 2];
        let mut same = [0usize; 2];
        let mut count = [0usize; 2];
        for k in 0..samples {
            let i = k * n / samples;
            let nv = normals[i].try_normalize(1e-12).unwrap_or(Vec3::zeros());
            if nv.norm() < 0.5 {
                continue;
            }
            let z = usize::from(zone[i]);
            count[z] += 1;
            match partner(bvh, &mesh.positions[i], &nv, dir) {
                Some((d, true)) => hist[z][bin(d)] += 1,
                Some((_, false)) => same[z] += 1,
                None => {}
            }
        }
        for (z, name) in [(1, "handle zone"), (0, "elsewhere")] {
            eprintln!(
                "    {side}, {name} ({} samples): opposite-facing surface within {BINS:?} mm: {:?}; same-facing first hit {}; none {}",
                count[z],
                hist[z],
                same[z],
                count[z] - hist[z].iter().sum::<usize>() - same[z]
            );
        }
    }
}

/// Where the decimation stops: the link condition (an edge on a 3-cycle that bounds no triangle, the
/// last step of a tunnel), the boundary, or the geometry of the fan.
fn stall_census(scan: &Mesh, zone: &[bool]) {
    let mut m = scan.clone();
    let t0 = std::time::Instant::now();
    let opts =
        DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    decimate(&mut m, &opts);
    let secs = t0.elapsed().as_secs_f64();
    let (link, on_bnd, other) = edge_kinds(&m);
    let tree = KdTree::build(&scan.positions);
    let in_zone =
        m.triangles.iter().filter(|t| tree.nearest(&m.face_centroid_of(t)).is_some_and(|(i, _)| zone[i])).count();
    let used = surfacecraft_geom::pieces::pieces(&m);
    eprintln!(
        "    decimation without a distance limit stops at {} triangles in {secs:.1} s (genus {:?}); edges: {} fail the link condition, {} touch the boundary, {} free (refused by the fan geometry); {:.1} % of the triangles left lie in the handle zone",
        m.triangle_count(),
        used.first().and_then(|p| p.genus()),
        link,
        on_bnd,
        other,
        100.0 * in_zone as f64 / m.triangle_count().max(1) as f64
    );
}

/// The edges of a stalled mesh: on a 3-cycle that bounds no triangle (the link condition), at the
/// boundary, or free.
fn edge_kinds(m: &Mesh) -> (usize, usize, usize) {
    let ef = edge_faces(m);
    let nb = neighbours(m);
    let boundary: Vec<bool> = {
        let mut b = vec![false; m.positions.len()];
        for (&(a, c), &n) in &ef {
            if n == 1 {
                b[a as usize] = true;
                b[c as usize] = true;
            }
        }
        b
    };
    let (mut link, mut on_bnd, mut other) = (0usize, 0usize, 0usize);
    for &(a, b) in ef.keys() {
        if boundary[a as usize] || boundary[b as usize] {
            on_bnd += 1;
            continue;
        }
        let common = nb[a as usize].iter().filter(|x| nb[b as usize].binary_search(x).is_ok()).count();
        if common > 2 {
            link += 1;
        } else {
            other += 1;
        }
    }
    (link, on_bnd, other)
}

trait Centroid {
    fn face_centroid_of(&self, t: &[u32; 3]) -> Point3;
}
impl Centroid for Mesh {
    fn face_centroid_of(&self, t: &[u32; 3]) -> Point3 {
        let p = |k: usize| self.positions[t[k] as usize].coords;
        Point3::from((p(0) + p(1) + p(2)) / 3.0)
    }
}

fn area_share(mesh: &Mesh, zone: &[bool]) -> f64 {
    let (mut z, mut all) = (0.0, 0.0);
    for (i, t) in mesh.triangles.iter().enumerate() {
        let a = mesh.face_area(i);
        all += a;
        if zone[t[0] as usize] {
            z += a;
        }
    }
    z / all.max(1e-12)
}

/// The coarse mesh of the first wrap round (budget 8 tolerances): triangles per area in and out of
/// the handle zone.
fn coarse_density(scan: &Mesh, zone: &[bool], budget: f64) {
    use surfacecraft_geom::reduce::{ToleranceOptions, reduce_to_tolerance};
    let mut m = scan.clone();
    let opts =
        DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    let t0 = std::time::Instant::now();
    let rep = reduce_to_tolerance(&mut m, &opts, &ToleranceOptions::new(budget));
    let tree = KdTree::build(&scan.positions);
    let in_zone =
        m.triangles.iter().filter(|t| tree.nearest(&m.face_centroid_of(t)).is_some_and(|(i, _)| zone[i])).count();
    let share = area_share(scan, zone);
    let n = m.triangle_count() as f64;
    let total_area = scan.area();
    let (za, ra) = (share * total_area, (1.0 - share) * total_area);
    eprintln!(
        "    reduction within {budget} mm: {} triangles in {:.1} s (met {:?}); per 1000 mm²: handle zone {:.2}, elsewhere {:.2}",
        m.triangle_count(),
        t0.elapsed().as_secs_f64(),
        rep.map(|r| r.met),
        1000.0 * in_zone as f64 / za.max(1e-9),
        1000.0 * (n - in_zone as f64) / ra.max(1e-9)
    );
}

fn report_handles(scan: &Mesh) -> Vec<bool> {
    let nb = neighbours(scan);
    let h = median_edge(scan);
    let all: Vec<u32> = (0..scan.triangle_count() as u32).collect();
    eprintln!("    median edge {h:.3} mm; genus of the whole scan {}", genus_of(scan, &all));
    let mut zone = vec![false; scan.positions.len()];
    for k in [3.0, 6.0, 12.0, 25.0, 50.0, 100.0] {
        let (total, with, max, z) = local_genus(scan, &nb, k * h);
        let share = area_share(scan, &z);
        eprintln!(
            "    cells about {k} median edges ({:.1} mm) across: {total} handles inside cells; {with} cells hold one (largest {max}); those cells are {:.1} % of the area",
            k * h,
            100.0 * share
        );
        if k == 12.0 {
            zone = z;
        }
    }
    zone
}

/// Vertices of a fold: one of their triangles faces against the average normal around the vertex.
fn folded(mesh: &Mesh) -> Vec<bool> {
    let normals = mesh.vertex_normals();
    let mut f = vec![false; mesh.positions.len()];
    for t in &mesh.triangles {
        let p = |k: usize| mesh.positions[t[k] as usize];
        let n = (p(1) - p(0)).cross(&(p(2) - p(0)));
        for &v in t {
            if n.dot(&normals[v as usize]) < 0.0 {
                f[v as usize] = true;
            }
        }
    }
    f
}

fn rim_distance(mesh: &Mesh, nb: &[Vec<u32>]) -> Vec<f64> {
    let ef = edge_faces(mesh);
    let mut dist = vec![f64::INFINITY; mesh.positions.len()];
    let mut heap = BinaryHeap::new();
    for (&(a, b), &c) in &ef {
        if c == 1 {
            for v in [a, b] {
                dist[v as usize] = 0.0;
                heap.push(Item(0.0, v, 0));
            }
        }
    }
    while let Some(Item(d, v, _)) = heap.pop() {
        if d > dist[v as usize] {
            continue;
        }
        for &w in &nb[v as usize] {
            let nd = d + (mesh.positions[v as usize] - mesh.positions[w as usize]).norm();
            if nd < dist[w as usize] {
                dist[w as usize] = nd;
                heap.push(Item(nd, w, 0));
            }
        }
    }
    dist
}

/// The boundary loops: length, and how far each lies from the others (are two of them the inner and
/// outer edge of one opening of a wall scanned from both sides?).
fn rims(mesh: &Mesh) {
    let ef = edge_faces(mesh);
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    let mut on = vec![false; mesh.positions.len()];
    let mut length: HashMap<u32, f64> = HashMap::new();
    for (&(a, b), &c) in &ef {
        if c == 1 {
            on[a as usize] = true;
            on[b as usize] = true;
            let (x, y) = (find(&mut parent, a), find(&mut parent, b));
            parent[x as usize] = y;
        }
    }
    let mut loops: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for v in 0..mesh.positions.len() as u32 {
        if on[v as usize] {
            let r = find(&mut parent, v);
            loops.entry(r).or_default().push(v);
        }
    }
    for (&(a, b), &c) in &ef {
        if c == 1 {
            *length.entry(find(&mut parent, a)).or_default() +=
                (mesh.positions[a as usize] - mesh.positions[b as usize]).norm();
        }
    }
    let list: Vec<(u32, Vec<u32>)> = loops.into_iter().collect();
    for (i, (ri, vi)) in list.iter().enumerate() {
        let mut parts = Vec::new();
        for (j, (_, vj)) in list.iter().enumerate() {
            if i == j {
                continue;
            }
            let tree = KdTree::build(&vj.iter().map(|&v| mesh.positions[v as usize]).collect::<Vec<_>>());
            let mut d: Vec<f64> =
                vi.iter().filter_map(|&v| tree.nearest(&mesh.positions[v as usize]).map(|x| x.1.sqrt())).collect();
            let k = d.len() / 2;
            let med = *d.select_nth_unstable_by(k, f64::total_cmp).1;
            parts.push(format!("to loop {j}: median {med:.1} mm"));
        }
        eprintln!(
            "    rim loop {i}: {} vertices, {:.0} mm long; {}",
            vi.len(),
            length.get(ri).copied().unwrap_or(0.0),
            parts.join(", ")
        );
    }
}

/// `scan.to_solid`, then where the vertices out of tolerance lie.
fn to_solid_and_misses(s: &mut Session, zone: &[bool]) {
    let t0 = std::time::Instant::now();
    let mut params = json!({"scan": "scan", "method": "freeform"});
    if let Some(extra) = std::env::var("SURFACECRAFT_LOCAL_TO_SOLID_PARAMS")
        .ok()
        .and_then(|x| serde_json::from_str::<serde_json::Value>(&x).ok())
        && let (Some(p), Some(x)) = (params.as_object_mut(), extra.as_object())
    {
        p.extend(x.clone());
    }
    let out = s.execute("scan.to_solid", params).unwrap();
    eprintln!(
        "    scan.to_solid: verdict {} patches {} within {} in {:.0} s; issues {:?}; retry {}; levels {}",
        out["verdict"],
        out["patches"],
        out["deviation"]["within"],
        t0.elapsed().as_secs_f64(),
        out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect::<Vec<_>>(),
        out["retry"],
        out["levels"]
    );
    let Some(body) = out["body"]["id"].as_u64().or_else(|| out["body"].as_u64()) else {
        eprintln!("    no body id in {}", out["body"]);
        return;
    };
    let dev = s.execute("deviation.compute", json!({"object": "scan", "against": body, "tolerance": 0.2})).unwrap();
    eprintln!("    deviation.compute: {}", dev["stats"]);
    let (id, mesh) = s.doc.find_by_name("scan").map(|(id, _)| (id, s.doc.mesh(id).unwrap().mesh.clone())).unwrap();
    let values = s.live_map(id).unwrap().values.clone();
    let recomputed;
    let zone = if zone.len() == mesh.positions.len() {
        zone
    } else {
        recomputed = report_handles(&mesh);
        &recomputed
    };
    classify_misses(&mesh, &values, zone);
}

/// Where the vertices out of tolerance (or not measured) lie: near the rim, on a fold, in the handle
/// zone, elsewhere.
fn classify_misses(mesh: &Mesh, values: &[f32], zone: &[bool]) {
    let nb = neighbours(mesh);
    let h = median_edge(mesh);
    let rim = rim_distance(mesh, &nb);
    let fold = folded(mesh);
    let mut c = BTreeMap::<&str, usize>::new();
    let mut miss = 0usize;
    for (i, v) in values.iter().enumerate() {
        let outside = !v.is_finite() || v.abs() > 0.2;
        if !outside {
            continue;
        }
        miss += 1;
        let class = if !v.is_finite() {
            "not measured"
        } else if rim[i] < 10.0 * h {
            "within 10 edges of the rim"
        } else if fold[i] {
            "on a folded triangle"
        } else if zone[i] {
            "in the handle zone"
        } else {
            "elsewhere"
        };
        *c.entry(class).or_default() += 1;
    }
    let zone_share = zone.iter().filter(|&&z| z).count() as f64 / zone.len().max(1) as f64;
    eprintln!(
        "    {miss} of {} vertices out of 0.2 mm or not measured: {c:?}; the handle zone holds {:.1} % of the vertices",
        values.len(),
        100.0 * zone_share
    );
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn diagnose_handles() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS");
    for n in numbers() {
        let Some(path) = files.get(n - 1) else { continue };
        let mut s = Session::new();
        s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
        s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
        let scan = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| m.mesh.clone()).unwrap();
        eprintln!("C{n:02}: cleaned {} triangles, area {:.0} mm²", scan.triangle_count(), scan.area());
        let ps = surfacecraft_geom::pieces::pieces(&scan);
        let handles: i64 = ps.iter().map(|p| (2 - p.euler - p.loops.len() as i64).max(0) / 2).sum();
        eprintln!(
            "C{n:02}: handles (Euler estimate) {handles}, pieces {}, boundary loops {}",
            ps.len(),
            ps.iter().map(|p| p.loops.len()).sum::<usize>()
        );
        if std::env::var("SURFACECRAFT_LOCAL_COUNT_ONLY").is_ok() {
            continue;
        }
        let zone = report_handles(&scan);
        rims(&scan);
        if std::env::var("SURFACECRAFT_LOCAL_QUICK").is_err() {
            let bvh = Bvh::build(&scan);
            walls(&scan, &bvh, &zone);
            stall_census(&scan, &zone);
            coarse_density(&scan, &zone, 1.6);
        }
        if let Some(size) = std::env::var("SURFACECRAFT_LOCAL_CLOSE").ok().and_then(|v| v.parse::<f64>().ok()) {
            let closed = closed_handles(&scan, size);
            if std::env::var("SURFACECRAFT_LOCAL_WRAP").is_ok_and(|v| v == "1") {
                wrap_closed(&mut s, closed);
            }
        }
        if std::env::var("SURFACECRAFT_LOCAL_TO_SOLID").is_ok_and(|v| v == "1") {
            to_solid_and_misses(&mut s, &zone);
        }
    }
}

/// `scan.to_solid` on the scan with its handles closed, and that solid measured against the scan.
fn wrap_closed(s: &mut Session, closed: Mesh) {
    s.doc.add_mesh("closed", closed);
    let t0 = std::time::Instant::now();
    let out = s.execute("scan.to_solid", json!({"scan": "closed", "method": "freeform"})).unwrap();
    eprintln!(
        "    scan.to_solid on the closed mesh: verdict {} patches {} within {} coverage {} in {:.0} s; issues {:?}; retry {}",
        out["verdict"],
        out["patches"],
        out["deviation"]["within"],
        out["deviation"]["coverage"],
        t0.elapsed().as_secs_f64(),
        out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect::<Vec<_>>(),
        out["retry"]
    );
    let Some(body) = out["body"]["id"].as_u64() else { return };
    let cmp = s.execute("body.compare", json!({"body": body, "scan": "scan", "tolerance": 0.2})).unwrap();
    eprintln!("    against the scan as it was: {}", cmp.to_string().chars().take(600).collect::<String>());
}

/// `handles::close_handles` on the scan: what it closes, by scale, and what the reduction does after.
fn closed_handles(scan: &Mesh, size: f64) -> Mesh {
    use surfacecraft_geom::handles::{Params, close_handles};
    let mut m = scan.clone();
    let t0 = std::time::Instant::now();
    let rep =
        close_handles(&mut m, &Params { max_size: size, tolerance: 0.2 }, &surfacecraft_geom::Cancel::new()).unwrap();
    eprintln!(
        "    close_handles max_size {size} mm in {:.1} s: genus {:?} -> {:?}, loops {} -> {}, {} handles in {} parts, refused {:?}; removed {} triangles, {:.2} % of the area, added {}; removed vertices {} ({} within 0.2 mm, max {:.3} mm)",
        t0.elapsed().as_secs_f64(),
        rep.genus_before,
        rep.genus_after,
        rep.loops_before,
        rep.loops_after,
        rep.handles_closed,
        rep.closed.len(),
        rep.refused,
        rep.removed_triangles,
        100.0 * rep.removed_area_share,
        rep.added_triangles,
        rep.removed_vertices,
        rep.removed_within,
        rep.removed_max_distance
    );
    for sc in &rep.scales {
        let at: Vec<_> = rep.closed.iter().filter(|c| c.scale == *sc).collect();
        eprintln!(
            "      scale {sc:.2} mm: {} parts, {} handles",
            at.len(),
            at.iter().map(|c| c.handles).sum::<usize>()
        );
    }
    let ps = surfacecraft_geom::pieces::pieces(&m);
    eprintln!(
        "      after: pieces {}, euler {}, pinched {}, unoriented {}, non-manifold {}",
        ps.len(),
        ps.iter().map(|p| p.euler).sum::<i64>(),
        ps.iter().map(|p| p.pinched_vertices).sum::<usize>(),
        ps.iter().map(|p| p.unoriented_edges).sum::<usize>(),
        ps.iter().map(|p| p.non_manifold_edges).sum::<usize>()
    );
    let zone = report_handles(&m);
    if std::env::var("SURFACECRAFT_LOCAL_WRAP").is_err() {
        stall_census(&m, &zone);
        coarse_density(&m, &zone, 1.6);
    }
    m
}
