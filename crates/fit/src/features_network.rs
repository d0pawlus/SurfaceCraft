//! Network building of the feature-curve strategy ([`crate::features`]): the smooth regions of the
//! mesh, their corners and shared feature curves, handed on to the patch construction.
//!
//! Split out of `features.rs`: this file holds the bookkeeping only (region labels, boundary loops,
//! corner clusters, the register of shared sides). It evaluates and fits nothing itself: the
//! feature curves are fitted by `features::fit_curve` and the patches by `features_ngon`, and the
//! B-rep of the result is built by `kernel::nurbs`.
use crate::features::{FeatureNetwork, FeatureOptions, a3, fit_curve};
use crate::features_ngon::{NgonMode, RegionRec, ngon_patches, ngon_regions, quad_patches};
use std::collections::{BTreeMap, BTreeSet, VecDeque};
use surfacecraft_geom::{KdTree, Mesh, Point3, Vec3};

#[cfg(doc)]
use crate::features::feature_network;

/// The triangles of the network as a mesh of their own, cut into smooth regions by the sharp edges.
struct Smooth {
    /// The triangles `tris` of the input mesh.
    sub: Mesh,
    /// Neighbouring triangle across each edge (`u32::MAX` for none).
    nb: Vec<[u32; 3]>,
    /// Normal of each triangle.
    fnorm: Vec<Vec3>,
    /// An edge is sharp when the dot product of its two normals is below this cosine.
    cos: f64,
    /// Region id of each triangle.
    region: Vec<u32>,
    /// Is the region large enough to count?
    big: Vec<bool>,
}

/// Region membership of the vertices and the edge statistics of the live triangles.
struct EdgeScan {
    /// Regions touching each vertex (`u32::MAX` stands for the mesh boundary).
    vreg: Vec<BTreeSet<u32>>,
    /// Sum of the lengths of all edges of live triangles.
    edge_len_sum: f64,
    /// Number of those edges.
    edge_n: usize,
    /// Total length of the sharp and boundary edges.
    feature_length: f64,
}

/// The corners: clusters of junction vertices.
struct Corners {
    /// Corner id of each junction vertex of the submesh.
    cluster_of_vertex: BTreeMap<u32, u32>,
    /// Point of each corner.
    points: Vec<Vec3>,
}

/// The boundary loop of a region with its corners.
struct Outline {
    /// Boundary vertices of the submesh in loop order.
    lp: Vec<u32>,
    /// Loop index just after which the walk starts (a non-corner vertex).
    origin: usize,
    /// Corner runs `(cluster, first step, last step)` in walk order.
    corners: Vec<(u32, usize, usize)>,
}

/// Collects the regions, their sides and the shared feature curves.
struct Tracer<'a> {
    /// Triangle ids of the network in the input mesh.
    tris: &'a [u32],
    /// The smooth regions.
    sm: &'a Smooth,
    /// The corners.
    corners: Corners,
    /// Vertex of the input mesh behind each vertex of the submesh.
    orig: Vec<u32>,
    /// Vertex normals of the submesh.
    nrm: Vec<Vec3>,
    /// Degree of the feature curves.
    degree: usize,
    /// The regions traced so far.
    recs: Vec<RegionRec>,
    /// Curve id of each side, keyed by its end corners, an interior vertex tag and its point count.
    side_ids: BTreeMap<(u32, u32, u32, usize), usize>,
    /// The fitted feature curves.
    side_curves: Vec<Vec<Vec3>>,
    /// Regions that did not become patches, with the reason.
    skipped: Vec<String>,
}

impl Smooth {
    /// Checks the options and prepares the submesh of `tris` with its adjacency.
    fn new(mesh: &Mesh, tris: &[u32], opts: &FeatureOptions) -> Result<Self, String> {
        if !(2..=8).contains(&opts.degree) {
            return Err("degree must be 2..=8".into());
        }
        if !(opts.angle_deg.is_finite() && opts.angle_deg > 1.0 && opts.angle_deg < 175.0) {
            return Err("angle must be between 1 and 175 degrees".into());
        }
        let sub = mesh.submesh(tris);
        if sub.triangle_count() < 4 {
            return Err("too few triangles".into());
        }
        let nt = sub.triangle_count();
        let nb = sub.triangle_neighbours();
        let fnorm: Vec<Vec3> = (0..nt).map(|i| sub.face_normal(i).unwrap_or_else(Vec3::z)).collect();
        let cos = opts.angle_deg.to_radians().cos();
        Ok(Self { sub, nb, fnorm, cos, region: vec![u32::MAX; nt], big: Vec::new() })
    }

    /// Is edge `k` of triangle `t` a sharp edge?
    fn sharp(&self, t: usize, k: usize) -> bool {
        let n = self.nb[t][k];
        n != u32::MAX && self.fnorm[t].dot(&self.fnorm[n as usize]) < self.cos
    }

    /// Does triangle `t` belong to a large region?
    fn live(&self, t: usize) -> bool {
        self.big.get(self.region[t] as usize).copied().unwrap_or(false)
    }

    /// Is edge `k` of triangle `t` on the boundary of its live region?
    fn is_boundary(&self, t: usize, k: usize) -> bool {
        let n = self.nb[t][k];
        n == u32::MAX || self.region[n as usize] != self.region[t] || !self.live(n as usize)
    }

    /// Flood fill across smooth edges; the number of regions found.
    fn flood_fill(&mut self) -> usize {
        let nt = self.sub.triangle_count();
        let mut sizes: Vec<usize> = Vec::new();
        for s in 0..nt {
            if self.region[s] != u32::MAX {
                continue;
            }
            let id = sizes.len() as u32;
            let mut count = 0;
            let mut queue = VecDeque::from([s]);
            self.region[s] = id;
            while let Some(t) = queue.pop_front() {
                count += 1;
                for k in 0..3 {
                    let n = self.nb[t][k];
                    if n != u32::MAX && self.region[n as usize] == u32::MAX && !self.sharp(t, k) {
                        self.region[n as usize] = id;
                        queue.push_back(n as usize);
                    }
                }
            }
            sizes.push(count);
        }
        sizes.len()
    }

    /// Triangle count of each of the `nregions` regions.
    fn region_sizes(&self, nregions: usize) -> Vec<usize> {
        let mut size = vec![0usize; nregions];
        for &r in &self.region {
            size[r as usize] += 1;
        }
        size
    }

    /// Votes of the edges between a tiny region `a` and a large neighbour `b`.
    fn tiny_votes(&self, size: &[usize], min_triangles: usize) -> BTreeMap<(u32, u32), usize> {
        let mut votes: BTreeMap<(u32, u32), usize> = BTreeMap::new();
        for t in 0..self.sub.triangle_count() {
            for k in 0..3 {
                let n = self.nb[t][k];
                if n == u32::MAX {
                    continue;
                }
                let (a, b) = (self.region[t], self.region[n as usize]);
                if a != b && size[a as usize] < min_triangles && size[b as usize] >= min_triangles {
                    *votes.entry((a, b)).or_insert(0) += 1;
                }
            }
        }
        votes
    }

    /// For each tiny region, the large neighbour it shares most edges with (votes, neighbour).
    fn best_neighbours(&self, nregions: usize, min_triangles: usize) -> BTreeMap<u32, (usize, u32)> {
        let size = self.region_sizes(nregions);
        let votes = self.tiny_votes(&size, min_triangles);
        let mut best: BTreeMap<u32, (usize, u32)> = BTreeMap::new();
        for (&(a, b), &v) in &votes {
            let e = best.entry(a).or_insert((0, b));
            if v > e.0 {
                *e = (v, b);
            }
        }
        best
    }

    /// Tiny regions (spikes, noise) join the large neighbour they share most edges with.
    fn merge_tiny(&mut self, nregions: usize, min_triangles: usize) {
        for _ in 0..4 {
            let best = self.best_neighbours(nregions, min_triangles);
            if best.is_empty() {
                break;
            }
            for r in &mut self.region {
                if let Some(&(_, b)) = best.get(r) {
                    *r = b;
                }
            }
        }
    }

    /// Cuts the triangles into regions and marks the large ones.
    fn label_regions(&mut self, min_triangles: usize) {
        let nregions = self.flood_fill();
        self.merge_tiny(nregions, min_triangles);
        self.big = self.region_sizes(nregions).iter().map(|&c| c >= min_triangles).collect();
    }

    /// Adds the edges of triangle `t` to the vertex regions and the edge statistics of `scan`.
    fn scan_triangle(&self, t: usize, scan: &mut EdgeScan) {
        let tri = self.sub.triangles[t];
        for k in 0..3 {
            let (a, b) = (tri[k] as usize, tri[(k + 1) % 3] as usize);
            scan.vreg[a].insert(self.region[t]);
            let l = (self.sub.positions[b] - self.sub.positions[a]).norm();
            scan.edge_len_sum += l;
            scan.edge_n += 1;
            if self.nb[t][k] == u32::MAX || self.sharp(t, k) {
                if self.nb[t][k] == u32::MAX || (t as u32) < self.nb[t][k] {
                    scan.feature_length += l;
                }
                // the mesh boundary counts as one more region
                if self.nb[t][k] == u32::MAX {
                    scan.vreg[a].insert(u32::MAX);
                    scan.vreg[b].insert(u32::MAX);
                }
            }
        }
    }

    /// Regions touching each vertex and the edge statistics, over the live triangles.
    fn scan_edges(&self) -> EdgeScan {
        let mut scan = EdgeScan {
            vreg: vec![BTreeSet::new(); self.sub.positions.len()],
            edge_len_sum: 0.0,
            edge_n: 0usize,
            feature_length: 0.0,
        };
        for t in 0..self.sub.triangle_count() {
            if !self.live(t) {
                continue;
            }
            self.scan_triangle(t, &mut scan);
        }
        scan
    }

    /// Boundary half-edges (triangle, edge) of each live region, the region on the left of the winding.
    fn boundary_half_edges(&self) -> BTreeMap<u32, BTreeSet<(usize, usize)>> {
        let mut boundary: BTreeMap<u32, BTreeSet<(usize, usize)>> = BTreeMap::new();
        for t in 0..self.sub.triangle_count() {
            if !self.live(t) {
                continue;
            }
            for k in 0..3 {
                if self.is_boundary(t, k) {
                    boundary.entry(self.region[t]).or_default().insert((t, k));
                }
            }
        }
        boundary
    }

    /// The boundary half-edge after (t, k): rotate about its end vertex through the region
    /// (so a vertex the region touches twice is walked as two visits, not as a dead end).
    fn next_half_edge(&self, t: usize, k: usize) -> Option<(usize, usize)> {
        let (mut ct, mut ck) = (t, (k + 1) % 3);
        for _ in 0..4096 {
            if self.is_boundary(ct, ck) {
                return Some((ct, ck));
            }
            let n = self.nb[ct][ck] as usize;
            let (va, vb) = (self.sub.triangles[ct][ck], self.sub.triangles[ct][(ck + 1) % 3]);
            let nk = (0..3).find(|&j| self.sub.triangles[n][j] == vb && self.sub.triangles[n][(j + 1) % 3] == va)?;
            ct = n;
            ck = (nk + 1) % 3;
        }
        None
    }

    /// Walks the boundary loop through `h`; its vertices and whether the walk broke off.
    fn walk_loop(&self, h: (usize, usize), visited: &mut BTreeSet<(usize, usize)>) -> (Vec<u32>, bool) {
        let mut lp = Vec::new();
        let mut cur = h;
        loop {
            visited.insert(cur);
            lp.push(self.sub.triangles[cur.0][cur.1]);
            let Some(n) = self.next_half_edge(cur.0, cur.1) else {
                return (lp, true);
            };
            if n == h {
                return (lp, false);
            }
            if visited.contains(&n) {
                return (lp, true);
            }
            cur = n;
        }
    }

    /// The boundary loops of a region, longest first, and whether any walk broke off.
    fn boundary_loops(&self, set: &BTreeSet<(usize, usize)>) -> (Vec<Vec<u32>>, bool) {
        let mut visited: BTreeSet<(usize, usize)> = BTreeSet::new();
        let mut loops: Vec<Vec<u32>> = Vec::new();
        let mut broken = false;
        for &h in set {
            if visited.contains(&h) {
                continue;
            }
            let (lp, cut) = self.walk_loop(h, &mut visited);
            broken |= cut;
            loops.push(lp);
        }
        // the longest loop is the outline; small ones are holes of a few triangles, which the
        // patch simply covers
        loops.sort_by_key(|l| std::cmp::Reverse(l.len()));
        (loops, broken)
    }
}

/// Root of `x` in the union-find forest `p`, with path halving.
fn find(p: &mut [usize], mut x: usize) -> usize {
    while p[x] != x {
        p[x] = p[p[x]];
        x = p[x];
    }
    x
}

/// Union-find forest over the junction points `jpts`, joining those closer than 2.5 mean edges.
fn merge_junctions(jpts: &[Point3], mean_edge: f64) -> Vec<usize> {
    let tree = KdTree::build(jpts);
    let mut parent: Vec<usize> = (0..jpts.len()).collect();
    for (i, p) in jpts.iter().enumerate() {
        for j in tree.within(p, 2.5 * mean_edge) {
            let (a, b) = (find(&mut parent, i), find(&mut parent, j));
            if a != b {
                parent[a.max(b)] = a.min(b);
            }
        }
    }
    parent
}

impl Corners {
    /// Corners: junction vertices (three or more regions meet) clustered by distance.
    fn cluster(sub: &Mesh, vreg: &[BTreeSet<u32>], mean_edge: f64) -> Self {
        let nv = sub.positions.len();
        let junction: Vec<u32> = (0..nv as u32).filter(|&v| vreg[v as usize].len() >= 3).collect();
        let jpts: Vec<Point3> = junction.iter().map(|&v| sub.positions[v as usize]).collect();
        let mut parent = merge_junctions(&jpts, mean_edge);
        Self::average(sub, &junction, &mut parent)
    }

    /// Numbers the clusters of the forest `parent` and averages the junction vertices of each.
    fn average(sub: &Mesh, junction: &[u32], parent: &mut [usize]) -> Self {
        let mut cluster_of_vertex: BTreeMap<u32, u32> = BTreeMap::new();
        let mut cluster_ids: BTreeMap<usize, u32> = BTreeMap::new();
        let mut sums: Vec<(Vec3, usize)> = Vec::new();
        for (i, &v) in junction.iter().enumerate() {
            let r = find(parent, i);
            let id = *cluster_ids.entry(r).or_insert_with(|| {
                sums.push((Vec3::zeros(), 0));
                (sums.len() - 1) as u32
            });
            cluster_of_vertex.insert(v, id);
            sums[id as usize].0 += sub.positions[v as usize].coords;
            sums[id as usize].1 += 1;
        }
        let points: Vec<Vec3> = sums.iter().map(|(s, n)| s / *n as f64).collect();
        Self { cluster_of_vertex, points }
    }
}

/// Corner runs `(cluster, first step, last step)` met walking a loop of `tag`s from `origin`.
fn corner_runs(tag: &[Option<u32>], origin: usize) -> Vec<(u32, usize, usize)> {
    let m = tag.len();
    let mut corners: Vec<(u32, usize, usize)> = Vec::new(); // (cluster, first index, last index) in walk order
    for step in 0..m {
        let i = (origin + step) % m;
        if let Some(c) = tag[i] {
            match corners.last_mut() {
                Some(l) if l.0 == c && l.2 + 1 == step => l.2 = step,
                _ => corners.push((c, step, step)),
            }
        }
    }
    corners
}

impl Outline {
    /// The corners along the loop `lp`; the reason when the region cannot become a patch.
    fn new(lp: Vec<u32>, cluster_of_vertex: &BTreeMap<u32, u32>, ntri: usize) -> Result<Self, String> {
        // corners along the loop: runs of vertices of one cluster
        let tag: Vec<Option<u32>> = lp.iter().map(|v| cluster_of_vertex.get(v).copied()).collect();
        let m = lp.len();
        // start the walk just after a non-corner vertex so that no run wraps around
        let Some(origin) = (0..m).find(|&i| tag[i].is_none()) else {
            return Err(format!("region of {ntri} triangles: the boundary is all corners"));
        };
        let corners = corner_runs(&tag, origin);
        // the same cluster must not appear twice
        let distinct: BTreeSet<u32> = corners.iter().map(|c| c.0).collect();
        if corners.len() < 3 || distinct.len() != corners.len() {
            return Err(format!(
                "region of {ntri} triangles has {} corners (a patch needs at least 3, all distinct)",
                corners.len()
            ));
        }
        Ok(Self { lp, origin, corners })
    }

    /// Loop vertex at walk position `step`.
    fn at(&self, step: usize) -> u32 {
        self.lp[(self.origin + step) % self.lp.len()]
    }
}

/// Tag of a side that keeps two curves between the same corners apart: its first interior vertex.
fn side_tag(pts: &[Vec3]) -> u32 {
    if pts.len() > 2 {
        let q = pts[1];
        ((q.x * 1e3).round() as i64 as u32) ^ ((q.y * 1e3).round() as i64 as u32).rotate_left(11)
    } else {
        u32::MAX
    }
}

/// Vertex of the input mesh behind each vertex of `sub` (the flattening of n-sided regions works on `mesh`).
fn original_vertices(sub: &Mesh, mesh: &Mesh, tris: &[u32]) -> Vec<u32> {
    let mut orig = vec![u32::MAX; sub.positions.len()];
    if sub.triangle_count() == tris.len() {
        for (t, tri) in sub.triangles.iter().enumerate() {
            let Some(src) = tris.get(t).and_then(|&i| mesh.triangles.get(i as usize)) else { continue };
            for (&v, &o) in tri.iter().zip(src.iter()) {
                if let Some(slot) = orig.get_mut(v as usize) {
                    *slot = o;
                }
            }
        }
    }
    orig
}

impl<'a> Tracer<'a> {
    /// A tracer over the regions `sm` of the triangles `tris` of `mesh`.
    fn new(sm: &'a Smooth, mesh: &Mesh, tris: &'a [u32], corners: Corners, degree: usize) -> Self {
        Self {
            tris,
            sm,
            corners,
            orig: original_vertices(&sm.sub, mesh, tris),
            nrm: sm.sub.vertex_normals(),
            degree,
            recs: Vec::new(),
            side_ids: BTreeMap::new(),
            side_curves: Vec::new(),
            skipped: Vec::new(),
        }
    }

    /// Vertex of the input mesh behind the submesh vertex `v`.
    fn og(&self, v: u32) -> u32 {
        self.orig.get(v as usize).copied().unwrap_or(u32::MAX)
    }

    /// Triangles (ids into the input mesh) of region `rid`.
    fn region_triangles(&self, rid: u32) -> Vec<u32> {
        (0..self.sm.sub.triangle_count())
            .filter(|&t| self.sm.region[t] == rid)
            .map(|t| self.tris.get(t).copied().unwrap_or(0))
            .collect()
    }

    /// The region record of an outline, with the sides still to be registered.
    fn new_rec(&self, o: &Outline, tri_ids: Vec<u32>) -> RegionRec {
        let count = o.corners.len();
        let m = o.lp.len();
        RegionRec {
            corners: o.corners.iter().map(|c| c.0).collect(),
            sides: Vec::with_capacity(count),
            forward: Vec::with_capacity(count),
            points: Vec::new(),
            mean_normal: o.lp.iter().map(|&v| self.nrm[v as usize]).sum(),
            triangles: tri_ids,
            corner_verts: o.corners.iter().map(|c| (c.1..=c.2).map(|s| self.og(o.at(s % m))).collect()).collect(),
            side_verts: Vec::with_capacity(count),
            point_ids: Vec::new(),
        }
    }

    /// Points of side `c` of the outline (corner, vertices between, corner) and the input-mesh
    /// vertices between the corners.
    fn side_points(&self, o: &Outline, c: usize) -> (Vec<Vec3>, Vec<u32>) {
        let count = o.corners.len();
        let m = o.lp.len();
        let (ca, _, last_a) = o.corners[c];
        let (cb, first_b, _) = o.corners[(c + 1) % count];
        let mut pts = vec![self.corners.points[ca as usize]];
        let end = if (c + 1).is_multiple_of(count) { m + first_b } else { first_b };
        let mut between = Vec::new();
        for step in (last_a + 1)..end {
            pts.push(self.sm.sub.positions[o.at(step % m) as usize].coords);
            between.push(self.og(o.at(step % m)));
        }
        pts.push(self.corners.points[cb as usize]);
        (pts, between)
    }

    /// Registers side `c` of the outline (canonical direction: smaller corner id first) and fits its
    /// curve unless the same curve is known already.
    fn register_side(&mut self, o: &Outline, c: usize, rec: &mut RegionRec) {
        let count = o.corners.len();
        let ca = o.corners[c].0;
        let cb = o.corners[(c + 1) % count].0;
        let (mut pts, between) = self.side_points(o, c);
        rec.side_verts.push(between);
        let forward = ca < cb;
        if !forward {
            pts.reverse();
        }
        let (lo, hi) = (ca.min(cb), ca.max(cb));
        let tagv = side_tag(&pts);
        let degree = self.degree;
        let side_curves = &mut self.side_curves;
        let id = *self.side_ids.entry((lo, hi, tagv, pts.len())).or_insert_with(|| {
            side_curves.push(fit_curve(&pts, degree));
            side_curves.len() - 1
        });
        rec.sides.push(id);
        rec.forward.push(forward);
    }

    /// Vertices of region `rid` (submesh points and input-mesh ids) into `rec`.
    fn fill_points(&self, rid: u32, rec: &mut RegionRec) {
        let sub = &self.sm.sub;
        let mut seen: BTreeSet<u32> = BTreeSet::new();
        for t in 0..sub.triangle_count() {
            if self.sm.region[t] == rid {
                seen.extend(sub.triangles[t]);
            }
        }
        rec.point_ids = seen.iter().map(|&v| self.og(v)).collect();
        rec.points = seen.into_iter().map(|v| sub.positions[v as usize].coords).collect();
    }

    /// Traces region `rid` with the boundary half-edges `set`: its outline, corners and sides, or the
    /// reason it is skipped.
    fn trace_region(&mut self, rid: u32, set: &BTreeSet<(usize, usize)>) {
        let tri_ids = self.region_triangles(rid);
        let (mut loops, broken) = self.sm.boundary_loops(set);
        if broken || loops.is_empty() || loops.get(1).is_some_and(|l| l.len() > 12) {
            self.skipped.push(format!(
                "region of {} triangles: {} boundary loops of lengths {:?} (a patch needs one outline)",
                tri_ids.len(),
                loops.len(),
                loops.iter().map(Vec::len).collect::<Vec<_>>()
            ));
            return;
        }
        let lp = loops.swap_remove(0);
        let outline = match Outline::new(lp, &self.corners.cluster_of_vertex, tri_ids.len()) {
            Ok(o) => o,
            Err(reason) => {
                self.skipped.push(reason);
                return;
            }
        };
        let mut rec = self.new_rec(&outline, tri_ids);
        // register the sides (canonical direction: smaller corner id first)
        for c in 0..outline.corners.len() {
            self.register_side(&outline, c, &mut rec);
        }
        self.fill_points(rid, &mut rec);
        self.recs.push(rec);
    }
}

/// Builds the patches of the traced regions into `out` (corner points, nets, corners, region triangles).
fn assemble(
    out: &mut FeatureNetwork,
    mesh: &Mesh,
    recs: &[RegionRec],
    side_curves: &[Vec<Vec3>],
    ngon: NgonMode,
    degree: usize,
) {
    let patches = if ngon == NgonMode::Patch {
        out.ngons = ngon_regions(mesh, recs, side_curves);
        quad_patches(recs, side_curves)
    } else if recs.iter().all(|r| r.corners.len() == 4) {
        quad_patches(recs, side_curves)
    } else {
        let first_new = out.corner_points.len() as u32;
        ngon_patches(recs, side_curves, first_new, degree)
    };
    out.corner_points.extend(patches.extra_points.iter().copied());
    out.nets = patches.nets;
    out.corners = patches.corners;
    out.region_triangles = patches.region_triangles;
}

/// [`feature_network`] with the choice of what to do with regions that are not four-sided: with
/// [`NgonMode::Patch`] only the four-sided ones become Bézier patches and the others are returned
/// in `ngons` (their sides are the same shared feature curves, sampled densely).
///
/// # Errors
/// An error message for an invalid mesh, degree or empty region.
pub fn feature_network_with(
    mesh: &Mesh,
    tris: &[u32],
    opts: &FeatureOptions,
    ngon: NgonMode,
) -> Result<FeatureNetwork, String> {
    let mut sm = Smooth::new(mesh, tris, opts)?;
    sm.label_regions(opts.min_triangles);
    let scan = sm.scan_edges();
    let mean_edge = scan.edge_len_sum / scan.edge_n.max(1) as f64;
    let mut out = FeatureNetwork {
        feature_length: scan.feature_length,
        regions: sm.big.iter().filter(|&&b| b).count(),
        ..FeatureNetwork::default()
    };
    let corners = Corners::cluster(&sm.sub, &scan.vreg, mean_edge);
    out.corner_points = corners.points.iter().map(|p| a3(*p)).collect();
    let boundary = sm.boundary_half_edges();
    let mut tracer = Tracer::new(&sm, mesh, tris, corners, opts.degree);
    for (&rid, set) in &boundary {
        tracer.trace_region(rid, set);
    }
    let Tracer { recs, side_curves, skipped, .. } = tracer;
    out.skipped = skipped;
    out.curves = side_curves.len();
    assemble(&mut out, mesh, &recs, &side_curves, ngon, opts.degree);
    Ok(out)
}
