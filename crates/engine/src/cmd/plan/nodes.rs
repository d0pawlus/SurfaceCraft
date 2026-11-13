//! Plane nodes of the planner: the fitted regions as nodes with their normals, contacts between them and their snapped directions.

use super::*;

pub(super) fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}
pub(super) fn arr(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}
pub(super) fn pt(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

/// Distance of a point from the line `origin + t·axis` and its position along it.
pub(super) fn radial_axial(p: &Point3, origin: Vec3, axis: Vec3) -> (f64, f64) {
    let d = p.coords - origin;
    let t = d.dot(&axis);
    ((d - axis * t).norm(), t)
}

/// One surface (a merged group of coplanar fragments is one node).
pub(super) struct Node {
    pub(super) members: Vec<usize>,
    pub(super) base: usize,
    pub(super) shape: Shape,
    pub(super) convex: bool,
    pub(super) full: bool,
    pub(super) pts: Vec<Point3>,
    pub(super) tree: KdTree,
}

pub(super) fn make_nodes(items: &[Item]) -> Vec<Node> {
    // coplanar planes with the same orientation are one surface (segmentation splits big planes)
    let planes: Vec<(usize, Vec3, Vec3)> = items
        .iter()
        .enumerate()
        .filter_map(|(i, it)| match &it.cut.shape {
            Shape::Plane { point, normal } => Some((i, v3(*point), v3(*normal))),
            _ => None,
        })
        .collect();
    let mut parent: Vec<usize> = (0..items.len()).collect();
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent.get(i).is_some_and(|&p| p != i) {
            let p = parent.get(i).copied().unwrap_or(i);
            if let Some(g) = parent.get(p).copied()
                && let Some(x) = parent.get_mut(i)
            {
                *x = g;
            }
            i = p;
        }
        i
    }
    for x in 0..planes.len() {
        for y in (x + 1)..planes.len() {
            let ((i, p, n), (j, q, m)) = (planes[x], planes[y]);
            if n.dot(&m) > 0.998 && (n.dot(&(q - p))).abs() < 0.25 {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj
                    && let Some(x) = parent.get_mut(rj)
                {
                    *x = ri;
                }
            }
        }
    }
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut slot: Vec<Option<usize>> = vec![None; items.len()];
    for i in 0..items.len() {
        let r = find(&mut parent, i);
        match slot.get(r).copied().flatten() {
            Some(g) => {
                if let Some(v) = groups.get_mut(g) {
                    v.push(i);
                }
            }
            None => {
                if let Some(s) = slot.get_mut(r) {
                    *s = Some(groups.len());
                }
                groups.push(vec![i]);
            }
        }
    }
    groups
        .into_iter()
        .filter_map(|members| {
            // the largest fragment gives the plane (the best conditioned fit)
            let base = *members.iter().max_by_key(|&&i| items.get(i).map_or(0, |it| it.pts.len()))?;
            let it = items.get(base)?;
            let pts: Vec<Point3> =
                members.iter().filter_map(|&i| items.get(i)).flat_map(|it| it.pts.iter().copied()).collect();
            let tree = KdTree::build(&pts);
            Some(Node {
                members,
                base,
                shape: it.cut.shape.clone(),
                convex: it.cut.convex,
                full: it.cut.full,
                pts,
                tree,
            })
        })
        .collect()
}

/// The unit direction of the surface at `p`, pointing out of the material.
fn normal_at(nd: &Node, p: &Point3) -> Option<Vec3> {
    let sign = if nd.convex { 1.0 } else { -1.0 };
    match &nd.shape {
        Shape::Plane { normal, .. } => normalize(v3(*normal)),
        Shape::Cylinder { point, axis, .. } => {
            let a = normalize(v3(*axis))?;
            let d = p.coords - v3(*point);
            normalize(d - a * d.dot(&a)).map(|r| r * sign)
        }
        Shape::Cone { apex, axis, half_angle } => {
            let a = normalize(v3(*axis))?;
            let d = p.coords - v3(*apex);
            let r = normalize(d - a * d.dot(&a))?;
            normalize(r * half_angle.cos() - a * half_angle.sin()).map(|n| n * sign)
        }
        Shape::Sphere { center, .. } => normalize(p.coords - v3(*center)).map(|r| r * sign),
        Shape::Torus { center, axis, major_radius, .. } => {
            let a = normalize(v3(*axis))?;
            let d = p.coords - v3(*center);
            let r = normalize(d - a * d.dot(&a))?;
            normalize(d - r * *major_radius).map(|n| n * sign)
        }
    }
}

/// The contact graph: for every pair of neighbouring surfaces, whether the surfaces meet in a
/// convex or a concave edge (the material is behind the surfaces at a convex edge).
pub(super) fn contacts(nodes: &[Node]) -> Vec<Contact> {
    let mut out = Vec::new();
    let bounds: Vec<([f64; 3], [f64; 3])> = nodes
        .iter()
        .map(|n| {
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for p in &n.pts {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
            (lo, hi)
        })
        .collect();
    for x in 0..nodes.len() {
        for y in (x + 1)..nodes.len() {
            let (Some(a), Some(b), Some(ba), Some(bb)) = (nodes.get(x), nodes.get(y), bounds.get(x), bounds.get(y))
            else {
                continue;
            };
            if (0..3).any(|k| ba.0[k] - NEIGHBOUR_GAP > bb.1[k] || bb.0[k] - NEIGHBOUR_GAP > ba.1[k]) {
                continue;
            }
            // closest pair, querying with the smaller region (subsampled)
            let (small, big) = if a.pts.len() <= b.pts.len() { (a, b) } else { (b, a) };
            let stride = (small.pts.len() / 3000).max(1);
            let mut best: Option<(f64, Point3, Point3)> = None;
            for q in small.pts.iter().step_by(stride) {
                if let Some((i, d2)) = big.tree.nearest_within(q, NEIGHBOUR_GAP * NEIGHBOUR_GAP)
                    && best.as_ref().is_none_or(|b| d2 < b.0 * b.0)
                    && let Some(p) = big.tree.point(i)
                {
                    best = Some((d2.sqrt(), *q, *p));
                }
            }
            let Some((dist, p, q)) = best else { continue };
            let at = Point3::from((p.coords + q.coords) * 0.5);
            let local_mean = |n: &Node| -> Option<Vec3> {
                let ids = n.tree.within(&at, 2.5 + dist);
                if ids.is_empty() {
                    return None;
                }
                let s: Vec3 = ids.iter().filter_map(|&i| n.tree.point(i)).map(|p| p.coords).sum();
                Some(s / ids.len() as f64)
            };
            let (Some(ma), Some(mb)) = (local_mean(a), local_mean(b)) else { continue };
            let (Some(na), Some(nb)) = (normal_at(a, &at), normal_at(b, &at)) else { continue };
            // each surface's neighbour lies in front of it at a concave edge and behind at a convex one
            let score = na.dot(&(mb - at.coords)) + nb.dot(&(ma - at.coords));
            let angle = na.dot(&nb).clamp(-1.0, 1.0).acos().to_degrees();
            let kind = if angle < 8.0 || score.abs() < 0.1 {
                "tangent"
            } else if score > 0.0 {
                "concave"
            } else {
                "convex"
            };
            out.push(Contact { a: a.base, b: b.base, kind, angle_deg: angle, distance: dist });
        }
    }
    out
}

pub(super) fn min_dist(tree: &KdTree, q: &Point3) -> f64 {
    tree.nearest(q).map_or(f64::INFINITY, |(_, d2)| d2.sqrt())
}

/// Smallest distance between two regions' points (subsampled), `None` when above `limit`.
pub(super) fn near(a: &Node, b: &Node, limit: f64) -> Option<f64> {
    let (small, big) = if a.pts.len() <= b.pts.len() { (a, b) } else { (b, a) };
    let stride = (small.pts.len() / 3000).max(1);
    small
        .pts
        .iter()
        .step_by(stride)
        .filter_map(|q| big.tree.nearest_within(q, limit * limit).map(|(_, d2)| d2.sqrt()))
        .min_by(f64::total_cmp)
}

/// The first of the `anchors` (up to sign) within `deg` degrees of `d`, else `d`.
pub(super) fn snap_direction(d: Vec3, anchors: &[Vec3], deg: f64) -> Vec3 {
    let cos = deg.to_radians().cos();
    for a in anchors {
        let c = a.dot(&d);
        if c.abs() > cos {
            return if c < 0.0 { -*a } else { *a };
        }
    }
    d
}
