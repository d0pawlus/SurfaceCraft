//! Where does an open real scan come closer to itself than a wall: pairs of places that are near in
//! space but far along the surface ("sheets"), told apart by how they face (numbers only; scans are
//! named by their number and never enter the repository).
//!
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test local_sheets -- --ignored --nocapture`
//! `SURFACECRAFT_LOCAL_SCANS` (default `4,5,6,9`) selects scans, `SURFACECRAFT_LOCAL_THICKNESS` (default 1) is the wall (mm).
use serde_json::json;
use std::collections::{BTreeMap, HashMap, VecDeque};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Aabb, Bvh, Mesh, Vec3, par, tri};

/// Along the surface a triangle is the same sheet when it is within this many times its distance in space.
const SAME_SHEET_RATIO: f64 = 3.0;

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

fn wanted() -> Vec<usize> {
    let list = std::env::var("SURFACECRAFT_LOCAL_SCANS").unwrap_or_else(|_| "4,5,6,9".to_string());
    list.split(',').filter_map(|n| n.trim().parse().ok()).collect()
}

/// The nearest far-sheet triangle of a vertex: distance and the cosine between the two normals.
#[derive(Clone, Copy)]
struct Near {
    d: f64,
    cos: f64,
}

fn vertex_rings(m: &Mesh) -> Vec<Vec<u32>> {
    let mut ring: Vec<Vec<u32>> = vec![Vec::new(); m.vertex_count()];
    for t in &m.triangles {
        for k in 0..3 {
            ring[t[k] as usize].push(t[(k + 1) % 3]);
            ring[t[(k + 1) % 3] as usize].push(t[k]);
        }
    }
    ring.iter_mut().for_each(|r| {
        r.sort_unstable();
        r.dedup();
    });
    ring
}

/// Distances along the surface from vertex `v` to the vertices within `reach`.
fn reached(m: &Mesh, ring: &[Vec<u32>], v: usize, reach: f64) -> HashMap<u32, f64> {
    let mut best: HashMap<u32, f64> = HashMap::from([(v as u32, 0.0)]);
    let mut queue: VecDeque<u32> = VecDeque::from([v as u32]);
    while let Some(a) = queue.pop_front() {
        for &b in &ring[a as usize] {
            let d = best[&a] + (m.positions[b as usize] - m.positions[a as usize]).norm();
            if d <= reach && best.get(&b).is_none_or(|&o| d < o - 1e-12) {
                best.insert(b, d);
                queue.push_back(b);
            }
        }
    }
    best
}

fn nearest_far(m: &Mesh, bvh: &Bvh, ring: &[Vec<u32>], nrm: &[Vec3], v: usize, wall: f64) -> Option<Near> {
    let p = m.positions[v];
    let bx = Aabb { min: p - Vec3::repeat(wall), max: p + Vec3::repeat(wall) };
    let mut close: Vec<(f64, u32)> = bvh
        .candidates(&|b: &Aabb| (0..3).all(|a| b.min[a] <= bx.max[a] && bx.min[a] <= b.max[a]))
        .into_iter()
        .filter_map(|t| {
            let [a, b, c] = m.triangle(t as usize)?;
            Some(((tri::closest_point(&p, &a, &b, &c) - p).norm(), t)).filter(|x| x.0 < wall)
        })
        .collect();
    close.sort_by(|x, y| x.0.total_cmp(&y.0));
    let near = reached(m, ring, v, 3.0 * wall);
    close.into_iter().find_map(|(d, t)| {
        let on_sheet = m.triangles[t as usize].iter().filter_map(|w| near.get(w)).any(|&g| g <= SAME_SHEET_RATIO * d);
        (!on_sheet).then(|| Near { d, cos: nrm[v].dot(&m.face_normal(t as usize).unwrap_or(Vec3::zeros())) })
    })
}

fn find(p: &mut [u32], mut x: u32) -> u32 {
    while p[x as usize] != x {
        p[x as usize] = p[p[x as usize] as usize];
        x = p[x as usize];
    }
    x
}

/// Flagged vertices joined along the mesh edges.
fn clusters(m: &Mesh, near: &[Option<Near>]) -> Vec<Vec<u32>> {
    let mut parent: Vec<u32> = (0..m.vertex_count() as u32).collect();
    for t in &m.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if near[a as usize].is_some() && near[b as usize].is_some() {
                let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
                parent[rb as usize] = ra;
            }
        }
    }
    let mut by_root: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for v in (0..m.vertex_count() as u32).filter(|&v| near[v as usize].is_some()) {
        by_root.entry(find(&mut parent, v)).or_default().push(v);
    }
    let mut all: Vec<Vec<u32>> = by_root.into_values().collect();
    all.sort_by_key(|c| std::cmp::Reverse(c.len()));
    all
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v.get(v.len() / 2).copied().unwrap_or(f64::NAN)
}

fn area_of(m: &Mesh, c: &[u32]) -> f64 {
    let set: std::collections::HashSet<u32> = c.iter().copied().collect();
    (0..m.triangle_count())
        .filter(|&i| m.triangles[i].iter().filter(|v| set.contains(v)).count() >= 2)
        .map(|i| m.face_area(i))
        .sum()
}

fn report(number: usize, m: &Mesh, near: &[Option<Near>]) {
    let all = clusters(m, near);
    let mut kinds: BTreeMap<&str, (usize, usize)> = BTreeMap::new();
    let facing = |c: &Vec<u32>| match median(c.iter().filter_map(|&v| near[v as usize].map(|n| n.cos)).collect()) {
        x if x > 0.5 => "same-facing",
        x if x < -0.5 => "opposite-facing",
        _ => "crease",
    };
    for c in &all {
        let e = kinds.entry(facing(c)).or_default();
        *e = (e.0 + 1, e.1 + c.len());
    }
    eprintln!("S{number:02} {} flagged vertices in {} clusters: {kinds:?}", near.iter().flatten().count(), all.len());
    for kind in ["same-facing", "opposite-facing"] {
        let top = all.iter().filter(|c| facing(c) == kind).take(2);
        for c in top {
            let d = median(c.iter().filter_map(|&v| near[v as usize].map(|n| n.d)).collect());
            eprintln!(
                "S{number:02}   largest {kind}: {} vertices, area {:.2} mm2, median distance {d:.3} mm",
                c.len(),
                area_of(m, c)
            );
        }
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_near_sheets_by_facing() {
    let files = corpus();
    let wall: f64 = std::env::var("SURFACECRAFT_LOCAL_THICKNESS").ok().and_then(|t| t.parse().ok()).unwrap_or(1.0);
    for number in wanted() {
        let Some(path) = number.checked_sub(1).and_then(|k| files.get(k)) else { continue };
        let mut s = Session::new();
        s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
        s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 0, "remove_folds": false})).unwrap();
        let id = s.doc.find_by_name("scan").unwrap().0;
        let m = (*s.doc.mesh(id).unwrap().mesh).clone();
        let (ring, nrm, bvh) = (vertex_rings(&m), m.vertex_normals(), Bvh::build(&m));
        let near: Vec<Option<Near>> = par::map(m.vertex_count(), |v| nearest_far(&m, &bvh, &ring, &nrm, v, wall));
        report(number, &m, &near);
    }
}
