//! Where the handles are: geodesic balls of one scale across around seeds a quarter of the scale apart
//! along the surface, so that every handle at most half the scale across lies wholly inside the ball
//! of the seed nearest to it. Every connected part of a ball that is not a disk is a candidate: it
//! carries a handle inside (genus above zero) or it is a tube whose ends are joined elsewhere (a
//! bridge, or a tunnel through a thin wall). The balls overlap; [`pick`] keeps a set of candidates
//! that are apart from each other.
use crate::{Mesh, par};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BinaryHeap, HashMap};

/// Adjacency of a mesh: neighbours and triangles of every vertex, and the vertices that may not be
/// touched (on the rim of the scan, or on an edge of three or more triangles).
pub(super) struct Topo {
    nbr: Csr,
    tri: Csr,
    pub(super) fixed: Vec<bool>,
    pub(super) used: Vec<bool>,
}

/// Lists per vertex, packed.
struct Csr {
    start: Vec<usize>,
    items: Vec<u32>,
}

impl Csr {
    fn new(n: usize, pairs: &[(u32, u32)]) -> Self {
        let mut count = vec![0usize; n + 1];
        for &(v, _) in pairs {
            if let Some(c) = count.get_mut(v as usize + 1) {
                *c += 1;
            }
        }
        for i in 1..count.len() {
            let prev = count.get(i - 1).copied().unwrap_or(0);
            if let Some(c) = count.get_mut(i) {
                *c += prev;
            }
        }
        let mut fill = count.clone();
        let mut items = vec![0u32; pairs.len()];
        for &(v, x) in pairs {
            if let Some(slot) = fill.get_mut(v as usize)
                && let Some(it) = items.get_mut(*slot)
            {
                *it = x;
                *slot += 1;
            }
        }
        Self { start: count, items }
    }

    fn of(&self, v: u32) -> &[u32] {
        let (Some(&a), Some(&b)) = (self.start.get(v as usize), self.start.get(v as usize + 1)) else { return &[] };
        self.items.get(a..b).unwrap_or(&[])
    }
}

/// Every edge `(low, high)` of the triangles, sorted, with the number of triangles on it.
pub(super) fn edge_counts(mesh: &Mesh, tris: &[u32]) -> Vec<((u32, u32), u32)> {
    let mut e: Vec<(u32, u32)> = Vec::with_capacity(tris.len() * 3);
    for t in tris.iter().filter_map(|&t| mesh.triangles.get(t as usize)) {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            e.push((a.min(b), a.max(b)));
        }
    }
    e.sort_unstable();
    let mut out: Vec<((u32, u32), u32)> = Vec::with_capacity(e.len() / 2 + 1);
    for x in e {
        match out.last_mut() {
            Some((y, c)) if *y == x => *c += 1,
            _ => out.push((x, 1)),
        }
    }
    out
}

impl Topo {
    pub(super) fn new(mesh: &Mesh) -> Self {
        let nv = mesh.positions.len();
        let all: Vec<u32> = (0..mesh.triangles.len() as u32).collect();
        let edges = edge_counts(mesh, &all);
        let mut fixed = vec![false; nv];
        let mut used = vec![false; nv];
        let mut pairs = Vec::with_capacity(edges.len() * 2);
        for &((a, b), c) in &edges {
            pairs.extend([(a, b), (b, a)]);
            for v in [a, b] {
                if let Some(u) = used.get_mut(v as usize) {
                    *u = true;
                }
                if c != 2
                    && let Some(f) = fixed.get_mut(v as usize)
                {
                    *f = true;
                }
            }
        }
        let corners: Vec<(u32, u32)> =
            mesh.triangles.iter().enumerate().flat_map(|(i, t)| t.map(|v| (v, i as u32))).collect();
        Self { nbr: Csr::new(nv, &pairs), tri: Csr::new(nv, &corners), fixed, used }
    }

    pub(super) fn neighbours(&self, v: u32) -> &[u32] {
        self.nbr.of(v)
    }
}

#[derive(PartialEq)]
struct Front(f64, u32);
impl Eq for Front {}
impl PartialOrd for Front {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}
impl Ord for Front {
    /// Nearest first; ties by vertex, so nothing depends on the order of the heap.
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}

/// The vertices within `reach` of `seed` along the edges, with their distances (sorted by vertex).
fn reach_of(mesh: &Mesh, topo: &Topo, seed: u32, reach: f64) -> Vec<(u32, f64)> {
    let mut dist: HashMap<u32, f64> = HashMap::new();
    let mut heap = BinaryHeap::new();
    dist.insert(seed, 0.0);
    heap.push(Front(0.0, seed));
    while let Some(Front(d, x)) = heap.pop() {
        if d > dist.get(&x).copied().unwrap_or(f64::INFINITY) {
            continue;
        }
        let Some(px) = mesh.positions.get(x as usize) else { continue };
        for &w in topo.neighbours(x) {
            let Some(pw) = mesh.positions.get(w as usize) else { continue };
            let nd = d + (px - pw).norm();
            if nd <= reach && nd < dist.get(&w).copied().unwrap_or(f64::INFINITY) {
                dist.insert(w, nd);
                heap.push(Front(nd, w));
            }
        }
    }
    let mut out: Vec<(u32, f64)> = dist.into_iter().collect();
    out.sort_unstable_by_key(|x| x.0);
    out
}

/// Seeds `spacing` apart along the surface: every used vertex, in ascending order from `first` (and
/// round), that no earlier seed reaches within `spacing`.
fn seeds(mesh: &Mesh, topo: &Topo, spacing: f64, first: usize) -> Vec<u32> {
    let n = mesh.positions.len();
    let mut covered = vec![false; n];
    let mut out = Vec::new();
    for v in (first.min(n)..n).chain(0..first.min(n)) {
        if covered.get(v).copied().unwrap_or(true) || !topo.used.get(v).copied().unwrap_or(false) {
            continue;
        }
        out.push(v as u32);
        for (x, _) in reach_of(mesh, topo, v as u32, spacing) {
            if let Some(c) = covered.get_mut(x as usize) {
                *c = true;
            }
        }
    }
    out
}

/// The triangles with all three corners within `radius` of `seed` (ascending).
fn ball(mesh: &Mesh, topo: &Topo, seed: u32, radius: f64) -> Vec<u32> {
    let inside = reach_of(mesh, topo, seed, radius);
    let is_in = |v: u32| inside.binary_search_by_key(&v, |x| x.0).is_ok();
    let mut tris: Vec<u32> = inside
        .iter()
        .flat_map(|&(v, _)| topo.tri.of(v).iter().copied())
        .filter(|&t| mesh.triangles.get(t as usize).is_some_and(|c| c.iter().all(|&v| is_in(v))))
        .collect();
    tris.sort_unstable();
    tris.dedup();
    tris
}

/// A connected part of a ball that is not a disk.
#[derive(Clone, Debug)]
pub(super) struct Candidate {
    /// Its triangles (ascending).
    pub triangles: Vec<u32>,
    /// Its vertices (ascending).
    pub vertices: Vec<u32>,
    /// Genus inside it.
    pub genus: usize,
    /// One vertex of each of its boundary loops (the smallest index of the loop).
    pub loops: Vec<u32>,
}

impl Candidate {
    /// Handles the surface loses when the part is replaced by a disk on each loop and its loops stay
    /// joined outside it.
    pub(super) fn handles(&self) -> usize {
        self.genus + self.loops.len().saturating_sub(1)
    }
}

/// Union-find over `n` items.
pub(super) fn sets(n: usize) -> Vec<u32> {
    (0..n as u32).collect()
}

/// The representative of `x` (with path halving).
pub(super) fn root(p: &mut [u32], mut x: u32) -> u32 {
    for _ in 0..p.len() {
        let Some(&up) = p.get(x as usize) else { break };
        if up == x {
            break;
        }
        let grand = p.get(up as usize).copied().unwrap_or(up);
        if let Some(s) = p.get_mut(x as usize) {
            *s = grand;
        }
        x = up;
    }
    x
}

/// Joins the sets of `a` and `b`.
pub(super) fn join(p: &mut [u32], a: u32, b: u32) {
    let (ra, rb) = (root(p, a), root(p, b));
    if ra != rb
        && let Some(s) = p.get_mut(ra.max(rb) as usize)
    {
        *s = ra.min(rb);
    }
}

/// One connected part of a ball.
struct Part {
    tris: Vec<u32>,
    verts: Vec<u32>,
    edges: usize,
    boundary: Vec<(u32, u32)>,
}

/// The connected parts of the triangles `tris`.
fn parts_of(mesh: &Mesh, tris: &[u32]) -> Vec<Part> {
    let mut verts: Vec<u32> = tris.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flatten().copied().collect();
    verts.sort_unstable();
    verts.dedup();
    let local = |v: u32| verts.binary_search(&v).ok().map(|i| i as u32);
    let edges = edge_counts(mesh, tris);
    let mut comp = sets(verts.len());
    for &((a, b), _) in &edges {
        if let (Some(x), Some(y)) = (local(a), local(b)) {
            join(&mut comp, x, y);
        }
    }
    let mut by_root: BTreeMap<u32, Part> = BTreeMap::new();
    let get = |v: u32, comp: &mut [u32], by_root: &mut BTreeMap<u32, Part>| -> Option<u32> {
        let r = local(v).map(|x| root(comp, x))?;
        by_root.entry(r).or_insert_with(|| Part {
            tris: Vec::new(),
            verts: Vec::new(),
            edges: 0,
            boundary: Vec::new(),
        });
        Some(r)
    };
    for &t in tris {
        let Some(first) = mesh.triangles.get(t as usize).map(|c| c[0]) else { continue };
        if let Some(p) = get(first, &mut comp, &mut by_root).and_then(|r| by_root.get_mut(&r)) {
            p.tris.push(t);
        }
    }
    for &v in &verts {
        if let Some(p) = get(v, &mut comp, &mut by_root).and_then(|r| by_root.get_mut(&r)) {
            p.verts.push(v);
        }
    }
    for &((a, b), c) in &edges {
        if let Some(p) = get(a, &mut comp, &mut by_root).and_then(|r| by_root.get_mut(&r)) {
            p.edges += 1;
            if c == 1 {
                p.boundary.push((a, b));
            }
        }
    }
    by_root.into_values().collect()
}

/// What a part of a ball is.
pub(super) enum Judged {
    /// A part that is not a disk and may be cut out.
    Candidate(Candidate),
    /// A whole closed piece of the mesh with genus (its smallest vertex): cutting it out would delete
    /// the piece, so it is refused.
    WholePiece(u32),
    /// A disk, or a part that may not be touched.
    No,
}

/// A part that is a disk, touches a fixed vertex, meets the rest of the mesh at a vertex that is not on
/// its boundary (a pinch: cutting it out would leave what hangs there loose), or has a count that no
/// oriented surface has, is no candidate; a part without boundary is a whole closed piece.
fn judge(mesh: &Mesh, topo: &Topo, p: Part) -> Judged {
    if p.verts.iter().any(|&v| topo.fixed.get(v as usize).copied().unwrap_or(true)) {
        return Judged::No;
    }
    let mut on_boundary: Vec<u32> = p.boundary.iter().flat_map(|&(a, b)| [a, b]).collect();
    on_boundary.sort_unstable();
    // a vertex on two loops (a pinch on the boundary) would merge them and make the count lie
    let pinched_rim = on_boundary.chunk_by(|a, b| a == b).any(|run| run.len() != 2);
    if pinched_rim {
        return Judged::No;
    }
    on_boundary.dedup();
    let mut inside: Vec<u32> =
        p.tris.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flatten().copied().collect();
    inside.sort_unstable();
    let count_in = |v: u32| inside.partition_point(|&x| x <= v) - inside.partition_point(|&x| x < v);
    let pinched =
        p.verts.iter().any(|&v| on_boundary.binary_search(&v).is_err() && count_in(v) != topo.tri.of(v).len());
    if pinched {
        return Judged::No;
    }
    let loops = loop_seeds(&p.verts, &p.boundary);
    let chi = p.verts.len() as i64 - p.edges as i64 + p.tris.len() as i64;
    let twice = 2 - chi - loops.len() as i64;
    if twice < 0 || twice % 2 != 0 {
        return Judged::No;
    }
    let c = Candidate { triangles: p.tris, vertices: p.verts, genus: (twice / 2) as usize, loops };
    match (c.handles() > 0, c.loops.is_empty()) {
        (false, _) => Judged::No,
        (true, true) => Judged::WholePiece(c.vertices.first().copied().unwrap_or(0)),
        (true, false) => Judged::Candidate(c),
    }
}

/// One vertex (the smallest) of every loop of the boundary edges.
fn loop_seeds(verts: &[u32], boundary: &[(u32, u32)]) -> Vec<u32> {
    let local = |v: u32| verts.binary_search(&v).ok().map(|i| i as u32);
    let mut s = sets(verts.len());
    for &(a, b) in boundary {
        if let (Some(x), Some(y)) = (local(a), local(b)) {
            join(&mut s, x, y);
        }
    }
    let mut seeds: BTreeMap<u32, u32> = BTreeMap::new();
    for &(a, b) in boundary {
        for v in [a, b] {
            if let Some(x) = local(v) {
                let r = root(&mut s, x);
                let e = seeds.entry(r).or_insert(v);
                *e = (*e).min(v);
            }
        }
    }
    seeds.into_values().collect()
}

/// The candidates of the whole mesh at `scale`: the parts of the balls of diameter `scale` around
/// seeds a quarter of the scale apart (taken from vertex `first` on) that are not disks (in the order
/// of the seeds; balls in parallel), and the whole closed pieces with genus that a ball held (their
/// smallest vertices, each once).
pub(super) fn candidates(mesh: &Mesh, topo: &Topo, scale: f64, first: usize) -> (Vec<Candidate>, Vec<u32>) {
    let centres = seeds(mesh, topo, 0.25 * scale, first);
    let judged = par::map_slice(&centres, |&s| {
        let tris = ball(mesh, topo, s, 0.5 * scale);
        parts_of(mesh, &tris).into_iter().map(|p| judge(mesh, topo, p)).collect::<Vec<_>>()
    });
    let (mut cands, mut whole) = (Vec::new(), Vec::new());
    for j in judged.into_iter().flatten() {
        match j {
            Judged::Candidate(c) => cands.push(c),
            Judged::WholePiece(v) => whole.push(v),
            Judged::No => {}
        }
    }
    whole.sort_unstable();
    whole.dedup();
    (cands, whole)
}

/// Candidates that are apart: smallest first (a ball that holds little more than its handle is least
/// likely to cut another one in two), a candidate is kept when none of its vertices is a vertex of a
/// kept one or a neighbour of one, so the holes they leave do not touch.
pub(super) fn pick(topo: &Topo, mut cands: Vec<Candidate>, vertices: usize) -> Vec<Candidate> {
    cands.sort_by_key(|c| (c.triangles.len(), c.triangles.first().copied()));
    let mut claimed = vec![false; vertices];
    let mut out = Vec::new();
    for c in cands {
        if c.vertices.iter().any(|&v| claimed.get(v as usize).copied().unwrap_or(true)) {
            continue;
        }
        for &v in &c.vertices {
            for &w in topo.neighbours(v).iter().chain(std::iter::once(&v)) {
                if let Some(x) = claimed.get_mut(w as usize) {
                    *x = true;
                }
            }
        }
        out.push(c);
    }
    out
}

/// The candidate grown by one ring of triangles (every triangle at one of its vertices), as the part
/// that holds its first triangle; `None` when that part is no candidate any more.
pub(super) fn grow(mesh: &Mesh, topo: &Topo, c: &Candidate) -> Option<Candidate> {
    let first = *c.triangles.first()?;
    let mut tris: Vec<u32> = c.vertices.iter().flat_map(|&v| topo.tri.of(v).iter().copied()).collect();
    tris.extend(c.triangles.iter().copied());
    tris.sort_unstable();
    tris.dedup();
    let part = parts_of(mesh, &tris).into_iter().find(|p| p.tris.binary_search(&first).is_ok())?;
    match judge(mesh, topo, part) {
        Judged::Candidate(c) => Some(c),
        Judged::WholePiece(_) | Judged::No => None,
    }
}

/// For every vertex, the candidate (index) whose vertices or their neighbours hold it, `u32::MAX` for none.
pub(super) fn claims(topo: &Topo, cands: &[Candidate], live: &[bool], vertices: usize) -> Vec<u32> {
    let mut owner = vec![u32::MAX; vertices];
    for (g, c) in cands.iter().enumerate().filter(|(g, _)| live.get(*g).copied().unwrap_or(false)) {
        for &v in &c.vertices {
            for &w in topo.neighbours(v).iter().chain(std::iter::once(&v)) {
                if let Some(o) = owner.get_mut(w as usize) {
                    *o = g as u32;
                }
            }
        }
    }
    owner
}
