//! Triangles next to a hole, grouped by normal into sides, and the wedge of sides that meets at sharp edges.

use super::sides::*;
use super::*;

/// A triangle next to the hole: its vertices, unit normal, area, middle, whether it touches the hole's
/// border, and its group of normals and its side (once known).
struct Near {
    corners: [u32; 3],
    normal: Vec3,
    area: f64,
    centre: Point3,
    at_border: bool,
    group: Option<usize>,
    side: Option<usize>,
}

/// The triangles within [`SUPPORT_RINGS`] rings of the loop, in index order.
fn triangles_near(mesh: &Mesh, around: &Around, lp: &[u32]) -> Vec<Near> {
    let mut ring_of: HashMap<u32, usize> = HashMap::new();
    let mut front: Vec<u32> = lp.to_vec();
    for ring in 0..SUPPORT_RINGS {
        let mut next: Vec<u32> = Vec::new();
        for &v in &front {
            for &t in around.of(v) {
                if let std::collections::hash_map::Entry::Vacant(e) = ring_of.entry(t) {
                    e.insert(ring);
                    next.extend(mesh.triangles.get(t as usize).into_iter().flatten());
                }
            }
        }
        front = next;
    }
    let mut picked: Vec<(u32, usize)> = ring_of.into_iter().collect();
    picked.sort_unstable();
    picked
        .into_iter()
        .filter_map(|(i, ring)| {
            let corners = *mesh.triangles.get(i as usize)?;
            let [a, b, c] = mesh.triangle(i as usize)?;
            let (normal, area) = (tri::normal(&a, &b, &c)?, tri::area(&a, &b, &c));
            let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
            Some(Near { corners, normal, area, centre, at_border: ring == 0, group: None, side: None })
        })
        .collect()
}

/// Groups the triangles by normal (the groups are those of the triangles at the hole's border)
/// and returns the mean normal of each group. `None` unless there are at least two groups
/// further apart than the feature angle.
fn group_by_normal(near: &mut [Near]) -> Option<Vec<Vec3>> {
    let cos = FEATURE_DEG.to_radians().cos();
    let mut order: Vec<usize> = (0..near.len()).filter(|&i| near.get(i).is_some_and(|t| t.at_border)).collect();
    order.sort_by(|&a, &b| near[b].area.total_cmp(&near[a].area).then(a.cmp(&b)));
    // (first normal, area-weighted sum, area)
    let mut groups: Vec<(Vec3, Vec3, f64)> = Vec::new();
    for &i in &order {
        let (n, area) = (near[i].normal, near[i].area);
        if let Some(g) = groups.iter_mut().find(|g| g.0.dot(&n) >= cos) {
            g.1 += n * area;
            g.2 += area;
        } else if groups.len() < MAX_GROUPS {
            groups.push((n, n * area, area));
        } else {
            return None;
        }
    }
    let total: f64 = groups.iter().map(|g| g.2).sum();
    let means: Vec<Vec3> =
        groups.iter().filter(|g| g.2 >= MIN_SIDE_SHARE * total).filter_map(|g| crate::normalize(g.1)).collect();
    if means.len() < 2 {
        return None;
    }
    for (i, a) in means.iter().enumerate() {
        if means.iter().skip(i + 1).any(|b| a.dot(b) >= cos) {
            return None;
        }
    }
    for t in near.iter_mut() {
        let best = means
            .iter()
            .enumerate()
            .map(|(k, m)| (m.dot(&t.normal), k))
            .fold(None, |b: Option<(f64, usize)>, c| if b.is_none_or(|b| c.0 > b.0) { Some(c) } else { b });
        t.group = best.filter(|b| b.0 >= cos).map(|b| b.1);
    }
    Some(means)
}

/// The triangles on each mesh edge among `near`.
fn edges_of(near: &[Near]) -> HashMap<(u32, u32), Vec<usize>> {
    let mut by_edge: HashMap<(u32, u32), Vec<usize>> = HashMap::new();
    for (i, t) in near.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t.corners[k], t.corners[(k + 1) % 3]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    by_edge
}

/// Splits each group of normals where its triangles step along the group's normal by more
/// than [`SAME_SIDE_STEP`] mean loop edge lengths (the two faces of a step have one normal and
/// are two sides; a curved side has no such step) and makes the pieces that hold enough of the
/// hole's border the sides. Returns the mean normal of each side; `None` unless there are two or
/// three.
fn split_sides(near: &mut [Near], means: &[Vec3], edge: f64) -> Option<Vec<Vec3>> {
    // (area at the border, area-weighted normal, triangles)
    let mut pieces: Vec<(f64, Vec3, Vec<usize>)> = Vec::new();
    for (g, mean) in means.iter().enumerate() {
        let mut members: Vec<(f64, usize)> = near
            .iter()
            .enumerate()
            .filter(|(_, t)| t.group == Some(g))
            .map(|(i, t)| (t.centre.coords.dot(mean), i))
            .collect();
        members.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let mut last: Option<f64> = None;
        for (d, i) in members {
            if last.is_none_or(|l| d - l > SAME_SIDE_STEP * edge) {
                pieces.push((0.0, Vec3::zeros(), Vec::new()));
            }
            last = Some(d);
            if let (Some(p), Some(t)) = (pieces.last_mut(), near.get(i)) {
                p.0 += if t.at_border { t.area } else { 0.0 };
                p.1 += t.normal * t.area;
                p.2.push(i);
            }
        }
    }
    let total: f64 = pieces.iter().map(|p| p.0).sum();
    pieces.retain(|p| p.0 >= MIN_SIDE_SHARE * total && p.0 > 0.0);
    if pieces.len() < 2 || pieces.len() > MAX_SIDES {
        return None;
    }
    for (k, p) in pieces.iter().enumerate() {
        for &i in &p.2 {
            if let Some(t) = near.get_mut(i) {
                t.side = Some(k);
            }
        }
    }
    pieces.iter().map(|p| crate::normalize(p.1)).collect()
}

/// Do the sides meet at sharp edges? The median turn across the mesh edges that join triangles
/// of different sides must exceed the feature angle (a smooth surface that merely curves a lot
/// around the hole turns a little at each edge).
fn meets_sharply(near: &[Near], by_edge: &HashMap<(u32, u32), Vec<usize>>) -> bool {
    let mut turns: Vec<f64> = by_edge
        .values()
        .filter_map(|ts| match ts.as_slice() {
            [i, j] => {
                let (a, b) = (near.get(*i)?, near.get(*j)?);
                (a.side.is_some() && b.side.is_some() && a.side != b.side)
                    .then(|| a.normal.dot(&b.normal).clamp(-1.0, 1.0).acos())
            }
            _ => None,
        })
        .collect();
    turns.sort_by(f64::total_cmp);
    turns.get(turns.len() / 2).is_some_and(|m| *m > FEATURE_DEG.to_radians())
}

/// Is the wedge of the sides convex (`Some(true)`), concave (`Some(false)`) or neither? Every
/// side's middle must be under (over) every other side by more than `tiny`.
fn convexity(sides: &[Side], tiny: f64) -> Option<bool> {
    let mut signs: Vec<bool> = Vec::new();
    for (i, a) in sides.iter().enumerate() {
        for (j, b) in sides.iter().enumerate() {
            if i != j {
                let h = b.height(&a.origin);
                if !(h.abs() > tiny) {
                    return None;
                }
                signs.push(h < 0.0);
            }
        }
    }
    let convex = *signs.first()?;
    signs.iter().all(|s| *s == convex).then_some(convex)
}

/// Reads the surface around the hole as a wedge of two or three sides; the reason when it is
/// one smooth surface or cannot be read (`edge` is the mean loop edge length).
pub(super) fn read_wedge(mesh: &Mesh, around: &Around, lp: &[u32], edge: f64) -> Result<Wedge, &'static str> {
    let mut near = triangles_near(mesh, around, lp);
    let means = group_by_normal(&mut near).ok_or("one group of normals")?;
    let normals = split_sides(&mut near, &means, edge).ok_or("not two or three sides")?;
    let by_edge = edges_of(&near);
    if !meets_sharply(&near, &by_edge) {
        return Err("the sides do not meet sharply");
    }
    let mut sides: Vec<Side> = Vec::with_capacity(normals.len());
    for (k, n) in normals.iter().enumerate() {
        let mut ids: Vec<u32> = near.iter().filter(|t| t.side == Some(k)).flat_map(|t| t.corners).collect();
        ids.sort_unstable();
        ids.dedup();
        let points: Vec<Point3> = ids.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
        let (side, rms) = Side::fit(&points, *n).ok_or("a side has no fit")?;
        if !(rms <= MAX_SIDE_RMS * edge) {
            return Err("a side is not one surface");
        }
        sides.push(side);
    }
    let convex = convexity(&sides, MIN_WEDGE * edge).ok_or("neither convex nor concave")?;
    Ok(Wedge { sides, convex })
}
