//! Duplicates, neighbours and normals of the cloud: the k-nearest-neighbour graph, PCA normals and their orientation.

use super::*;

/// Drops exact duplicate points (keeping the first); normals follow when every one of them is
/// non-zero (otherwise they are ignored and re-estimated).
pub(super) fn dedupe(cloud: &PointCloud) -> (Vec<Point3>, Option<Vec<Vec3>>) {
    let mut order: Vec<u32> = (0..cloud.len() as u32).collect();
    order.sort_unstable_by(|&a, &b| {
        let (pa, pb) = (cloud.positions.get(a as usize), cloud.positions.get(b as usize));
        match (pa, pb) {
            (Some(pa), Some(pb)) => {
                pa.x.total_cmp(&pb.x).then(pa.y.total_cmp(&pb.y)).then(pa.z.total_cmp(&pb.z)).then(a.cmp(&b))
            }
            _ => a.cmp(&b),
        }
    });
    let mut keep = vec![true; cloud.len()];
    let mut prev: Option<&Point3> = None;
    for &i in &order {
        let Some(p) = cloud.positions.get(i as usize) else { continue };
        if prev.is_some_and(|q| q == p) {
            if let Some(f) = keep.get_mut(i as usize) {
                *f = false;
            }
        } else {
            prev = Some(p);
        }
    }
    let pts: Vec<Point3> = cloud.positions.iter().zip(&keep).filter_map(|(p, &k)| k.then_some(*p)).collect();
    let normals = cloud.normals.as_ref().and_then(|ns| {
        let unit: Vec<Vec3> = ns.iter().zip(&keep).filter_map(|(n, &k)| k.then_some(*n)).collect();
        let out: Vec<Vec3> = unit.iter().filter_map(|n| crate::normalize(*n)).collect();
        (out.len() == unit.len()).then_some(out)
    });
    (pts, normals)
}

// ------------------------------------------------------------------ parallel helper

/// Runs `f` over `0..n` in fixed-size chunks through [`crate::par`] (all cores natively,
/// sequential in the browser) and concatenates the results in order.
fn par_collect<T: Send, F: Fn(Range<usize>) -> Vec<T> + Sync + Send>(n: usize, f: F) -> Vec<T> {
    const CHUNK: usize = 2048;
    let chunks = n.div_ceil(CHUNK);
    crate::par::map(chunks, |c| f(c * CHUNK..((c + 1) * CHUNK).min(n))).into_iter().flatten().collect()
}

// ------------------------------------------------------------------ neighbours and normals

/// `k` nearest neighbours of every point, nearest first, flat (`k` per point).
pub(super) fn knn_graph(tree: &KdTree, pts: &[Point3], k: usize) -> Vec<u32> {
    par_collect(pts.len(), |r| {
        let mut out = Vec::with_capacity(r.len() * k);
        for i in r {
            let Some(p) = pts.get(i) else { continue };
            let before = out.len();
            for (j, _) in tree.knn(p, k + 1) {
                if j != i && out.len() - before < k {
                    out.push(j as u32);
                }
            }
            let pad = if out.len() > before { out.last().copied().unwrap_or(i as u32) } else { i as u32 };
            while out.len() - before < k {
                out.push(pad);
            }
        }
        out
    })
}

/// Mean distance to the nearest neighbour, ignoring the few isolated outliers.
pub(super) fn mean_spacing(pts: &[Point3], nbr: &[u32], k: usize) -> Option<f64> {
    let mut d: Vec<f64> = (0..pts.len())
        .filter_map(|i| {
            let j = *nbr.get(i * k)? as usize;
            let l = (pts.get(i)? - pts.get(j)?).norm();
            (l.is_finite() && l > 0.0).then_some(l)
        })
        .collect();
    if d.is_empty() {
        return None;
    }
    let mid = d.len() / 2;
    let median = *d.select_nth_unstable_by(mid, f64::total_cmp).1;
    let kept: Vec<f64> = d.into_iter().filter(|&l| l <= 3.0 * median).collect();
    let mean = kept.iter().sum::<f64>() / kept.len().max(1) as f64;
    (mean.is_finite() && mean > 0.0).then_some(mean)
}

/// Unoriented PCA normals.
pub(super) fn estimate_normals(pts: &[Point3], nbr: &[u32], k: usize) -> Vec<Vec3> {
    par_collect(pts.len(), |r| {
        let mut out = Vec::with_capacity(r.len());
        for i in r {
            let Some(&p0) = pts.get(i) else { continue };
            let ids = nbr.get(i * k..(i + 1) * k).unwrap_or(&[]);
            let mut mean = Vec3::zeros();
            let mut count = 1.0;
            for &j in ids {
                if let Some(q) = pts.get(j as usize) {
                    mean += q - p0;
                    count += 1.0;
                }
            }
            mean /= count;
            let mut cov = nalgebra::Matrix3::<f64>::zeros();
            let d0 = -mean;
            cov += d0 * d0.transpose();
            for &j in ids {
                if let Some(q) = pts.get(j as usize) {
                    let d = (q - p0) - mean;
                    cov += d * d.transpose();
                }
            }
            let eig = nalgebra::SymmetricEigen::new(cov);
            let mut best = 0;
            for a in 1..3 {
                if eig.eigenvalues[a] < eig.eigenvalues[best] {
                    best = a;
                }
            }
            let n: Vec3 = eig.eigenvectors.column(best).into_owned();
            out.push(crate::normalize(n).unwrap_or_else(Vec3::z));
        }
        out
    })
}

/// Flips normals so neighbours agree (MST propagation); each connected piece of the neighbour
/// graph is oriented to face away from its extreme point in +x.
pub(super) fn orient_normals(pts: &[Point3], normals: &mut [Vec3], nbr: &[u32], k: usize) {
    let n = pts.len();
    let gk = k.min(10);
    // undirected adjacency in CSR form
    let mut deg = vec![0u32; n + 1];
    for i in 0..n {
        for t in 0..gk {
            if let Some(&j) = nbr.get(i * k + t) {
                if let Some(d) = deg.get_mut(i) {
                    *d += 1;
                }
                if let Some(d) = deg.get_mut(j as usize) {
                    *d += 1;
                }
            }
        }
    }
    let mut start = vec![0usize; n + 1];
    for i in 0..n {
        start[i + 1] = start[i] + deg.get(i).copied().unwrap_or(0) as usize;
    }
    let mut fill = start.clone();
    let mut adj = vec![0u32; start[n]];
    for i in 0..n {
        for t in 0..gk {
            if let Some(&j) = nbr.get(i * k + t) {
                if let Some(f) = fill.get_mut(i) {
                    if let Some(a) = adj.get_mut(*f) {
                        *a = j;
                    }
                    *f += 1;
                }
                if let Some(f) = fill.get_mut(j as usize) {
                    if let Some(a) = adj.get_mut(*f) {
                        *a = i as u32;
                    }
                    *f += 1;
                }
            }
        }
    }
    let mut visited = vec![false; n];
    let mut heap: BinaryHeap<Reverse<(u32, u32, u32)>> = BinaryHeap::new();
    let mut piece: Vec<u32> = Vec::new();
    for s in 0..n {
        if visited.get(s).copied().unwrap_or(true) {
            continue;
        }
        piece.clear();
        heap.clear();
        heap.push(Reverse((0, s as u32, s as u32)));
        while let Some(Reverse((_, to, from))) = heap.pop() {
            let (to_i, from_i) = (to as usize, from as usize);
            if visited.get(to_i).copied().unwrap_or(true) {
                continue;
            }
            let nf = normals.get(from_i).copied();
            if let (Some(nf), Some(nt)) = (nf, normals.get_mut(to_i))
                && nf.dot(nt) < 0.0
            {
                *nt = -*nt;
            }
            if let Some(v) = visited.get_mut(to_i) {
                *v = true;
            }
            piece.push(to);
            let nt = normals.get(to_i).copied().unwrap_or_else(Vec3::z);
            for &j in adj.get(start[to_i]..start[to_i + 1]).unwrap_or(&[]) {
                if visited.get(j as usize).copied().unwrap_or(true) {
                    continue;
                }
                let nj = normals.get(j as usize).copied().unwrap_or_else(Vec3::z);
                let w = ((1.0 - nt.dot(&nj).abs()).clamp(0.0, 1.0) * 1_048_576.0) as u32;
                heap.push(Reverse((w, j, to)));
            }
        }
        // the extreme point in +x must face +x
        let extreme = piece
            .iter()
            .copied()
            .max_by(|&a, &b| {
                let xa = pts.get(a as usize).map_or(0.0, |p| p.x);
                let xb = pts.get(b as usize).map_or(0.0, |p| p.x);
                xa.total_cmp(&xb)
            })
            .unwrap_or(s as u32);
        if normals.get(extreme as usize).is_some_and(|v| v.x < 0.0) {
            for &i in &piece {
                if let Some(v) = normals.get_mut(i as usize) {
                    *v = -*v;
                }
            }
        }
    }
}
