//! Direct checks of a new face against the patches it replaced: its normal agrees with theirs, and
//! its surface lies within the gap of theirs. Neither depends on area or volume, so a flipped face,
//! a complementary one and a face on a sheet (which has no volume) are caught.
use super::samples::grid_area;
use crate::hybrid::{V, cross, dot, norm, sub, unit};
use crate::record::brep::{Brep, Shell};
use crate::{KernelError, Result};
use solidcore::eval::SurfaceEval;
use solidcore::math::Vec3;
use std::collections::BTreeMap;
use surfacecraft_geom::Mesh;

/// Cells a side of the grid the replaced patches are sampled on.
const CLOUD_GRID: usize = 4;
/// Corners of the new face's triangles that are checked, spread over all of them by the golden ratio
/// (a stride over the triangle list can alias with the tessellation's rows).
const CHECKS: usize = 256;
/// The golden ratio's fractional part: the step of the sequence that spreads the checked corners.
const GOLDEN: f64 = 0.618_033_988_749_894_9;
/// The least dot product of the new face's normal with the patch normal at the nearest point.
const MIN_DOT: f64 = 0.5;
/// The share of the checked corners that may disagree in normal (a face turned the wrong way disagrees
/// at nearly all of them; the allowance is for the corners of the group, where a patch is folded or
/// collapsed and its normal means nothing).
const DISAGREE_ALLOWANCE: f64 = 0.1;
/// Triangles with less than this share of the mean triangle area are slivers: their normals are noise.
const MIN_TRIANGLE_AREA_SHARE: f64 = 0.3;
/// A patch's normal is read only where its tangents are at least this far from parallel (sine).
const MIN_SIN_BETWEEN_TANGENTS: f64 = 0.1;
/// The patches nearest a corner that are tried.
const NEAREST_PATCHES: usize = 12;
/// A point of the new face may be this much farther from the patches than the gap.
const PROXIMITY_SLACK: f64 = 1.5;
/// ... except within this many gaps of the border of the group.
const BORDER_MARGIN: f64 = 2.0;

/// Points of the replaced patches with their outward normals.
pub(super) struct Cloud {
    points: Vec<V>,
    normals: Vec<V>,
    faces: Vec<usize>,
    /// For each patch: `1` when the shell uses it along its natural normal, else `-1`.
    signs: BTreeMap<usize, f64>,
    /// Points along the border of the group (its boundary edges).
    border: Vec<V>,
}

/// Does the shell of `body` use `face` along its natural normal?
pub(super) fn forward_of(body: &Brep, face: usize) -> bool {
    body.shells
        .iter()
        .find_map(|s| match s {
            Shell::Faces { faces, .. } => faces.iter().find(|o| o.face as usize == face).map(|o| o.forward),
            _ => None,
        })
        .unwrap_or(true)
}

/// The evaluator of a patch of `body`.
fn evaluator(body: &Brep, face: usize) -> Result<SurfaceEval> {
    let f = body.faces.get(face).ok_or_else(|| KernelError::Argument("face index".into()))?;
    let s = solidcore_convert::surface(&f.surface).map_err(|e| KernelError::Argument(e.to_string()))?;
    SurfaceEval::new(&s).map_err(|e| KernelError::Argument(format!("{e:?}")))
}

/// Patches with less than this share of the group's mean patch area are collapsed or slivers: their
/// normals mean nothing, so they are not sampled.
const MIN_PATCH_AREA_SHARE: f64 = 0.05;

/// Samples the patches `faces` of `body` (points and outward normals), leaving out collapsed ones.
pub(super) fn cloud(body: &Brep, faces: &[usize], border: Vec<V>) -> Result<Cloud> {
    let mut blocks: Vec<(usize, Vec<V>, Vec<V>)> = Vec::new();
    for &f in faces {
        let eval = evaluator(body, f)?;
        let [Some((u0, u1)), Some((v0, v1))] = eval.domain().range else {
            return Err(KernelError::Argument("a patch has no parameter box".into()));
        };
        let sign = if forward_of(body, f) { 1.0 } else { -1.0 };
        let (mut points, mut normals) = (Vec::new(), Vec::new());
        for i in 0..=CLOUD_GRID {
            for j in 0..=CLOUD_GRID {
                let (u, v) =
                    (u0 + (u1 - u0) * i as f64 / CLOUD_GRID as f64, v0 + (v1 - v0) * j as f64 / CLOUD_GRID as f64);
                let Some((p, du, dv)) = eval.derivs(u, v) else { continue };
                let n = unit(cross([du.x, du.y, du.z], [dv.x, dv.y, dv.z])).unwrap_or([0.0; 3]);
                points.push([p.x, p.y, p.z]);
                normals.push([sign * n[0], sign * n[1], sign * n[2]]);
            }
        }
        blocks.push((f, points, normals));
    }
    let area = |b: &(usize, Vec<V>, Vec<V>)| {
        if b.1.len() == (CLOUD_GRID + 1).pow(2) { grid_area(&b.1, CLOUD_GRID) } else { 0.0 }
    };
    let mean = blocks.iter().map(area).sum::<f64>() / blocks.len().max(1) as f64;
    let mut c = Cloud { points: Vec::new(), normals: Vec::new(), faces: Vec::new(), signs: BTreeMap::new(), border };
    for b in &blocks {
        c.signs.insert(b.0, if forward_of(body, b.0) { 1.0 } else { -1.0 });
    }
    for b in &blocks {
        if area(b) < MIN_PATCH_AREA_SHARE * mean {
            continue;
        }
        for (p, n) in b.1.iter().zip(&b.2) {
            if norm(*n) > 0.5 {
                c.points.push(*p);
                c.normals.push(*n);
                c.faces.push(b.0);
            }
        }
    }
    Ok(c)
}

impl Cloud {
    /// Is `p` within `reach` of the border of the group? Near it the face and the staircase of patches
    /// differ sideways (a tiny hole the mesher closes, a patch edge across a corner), which says nothing
    /// of whether the face leaves the patches.
    fn near_border(&self, p: V, reach: f64) -> bool {
        self.border.iter().any(|q| norm(sub(*q, p)) <= reach)
    }

    /// The `n` patches nearest `p` by the box of their samples (a corner over a patch has distance 0 to
    /// its box; of the patches that hold it, the smaller come first). The distance to the centre of a box
    /// would put the small patches round a corner before the large one it lies over.
    fn nearest_faces(&self, p: V, n: usize) -> Vec<usize> {
        let mut boxes: BTreeMap<usize, (V, V)> = BTreeMap::new();
        for (q, f) in self.points.iter().zip(&self.faces) {
            let b = boxes.entry(*f).or_insert((*q, *q));
            for k in 0..3 {
                b.0[k] = b.0[k].min(q[k]);
                b.1[k] = b.1[k].max(q[k]);
            }
        }
        let mut ranked: Vec<(f64, f64, usize)> = boxes
            .iter()
            .map(|(f, (lo, hi))| {
                let excess = |k: usize| (lo[k] - p[k]).max(p[k] - hi[k]).max(0.0);
                let d = (excess(0).powi(2) + excess(1).powi(2) + excess(2).powi(2)).sqrt();
                (d, norm(sub(*hi, *lo)), *f)
            })
            .collect();
        ranked.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.total_cmp(&b.1)).then(a.2.cmp(&b.2)));
        ranked.into_iter().take(n).map(|(_, _, f)| f).collect()
    }
}

/// The point of a patch nearest `p`, as `(distance, outward normal there)`; the normal is `None` where the
/// patch is collapsed.
fn nearest_on_patch(cloud: &Cloud, eval: &SurfaceEval, face: usize, p: V, reach: f64) -> Option<(f64, Option<V>)> {
    let q = Vec3::new(p[0], p[1], p[2]);
    let uv = eval.project(q, None, reach)?;
    let (c, du, dv) = eval.derivs(uv[0], uv[1])?;
    let sign = cloud.signs.get(&face).copied().unwrap_or(1.0);
    let (a, b) = ([du.x, du.y, du.z], [dv.x, dv.y, dv.z]);
    // near a collapsed corner the two tangents are nearly parallel and their product is noise
    let regular = norm(cross(a, b)) >= MIN_SIN_BETWEEN_TANGENTS * norm(a) * norm(b);
    let normal = unit(cross(a, b)).filter(|_| regular).map(|n| n.map(|x| sign * x));
    Some((norm(sub([c.x, c.y, c.z], p)), normal))
}

/// The nearest patch point to the corner `a` among the few patches it may lie over, as `(distance,
/// outward normal)`.
///
/// # Errors
/// No patch point within `reach`: the new face leaves the patches, or cannot be measured against them.
fn nearest_over(
    cloud: &Cloud,
    body: &Brep,
    evals: &mut BTreeMap<usize, SurfaceEval>,
    a: V,
    reach: f64,
) -> Result<(f64, Option<V>)> {
    let mut best: Option<(f64, Option<V>)> = None;
    for f in cloud.nearest_faces(a, NEAREST_PATCHES) {
        if let std::collections::btree_map::Entry::Vacant(slot) = evals.entry(f) {
            slot.insert(evaluator(body, f)?);
        }
        let found = evals.get(&f).and_then(|e| nearest_on_patch(cloud, e, f, a, reach));
        if let Some(m) = found.filter(|m| best.is_none_or(|b| m.0 < b.0)) {
            best = Some(m);
        }
    }
    best.ok_or_else(|| {
        KernelError::Argument("the new face leaves the patches it replaced, or cannot be measured against them".into())
    })
}

/// The corner `k` of the triangle list of `n`: spread over all triangles and all three corners.
fn spread(k: usize, n: usize) -> (usize, usize) {
    let t = ((k as f64 * GOLDEN).fract() * n as f64) as usize;
    (t.min(n.saturating_sub(1)), k % 3)
}

/// Checks the corners of the triangles of the new face (`mesh`, whose normals are the face's natural
/// ones; `forward` says whether that is outward) against the replaced patches `body`: the face's normal
/// agrees with the patch's at the nearest point, and the corner (which lies on the surface, unlike the
/// triangle's centre) lies within `gap` (times a slack) of a patch. A corner the patches cannot be
/// measured against is a failure, not a pass.
pub(super) fn check_face(mesh: &Mesh, forward: bool, cloud: &Cloud, body: &Brep, gap: f64) -> Result<()> {
    let n = mesh.triangle_count();
    if n == 0 || cloud.points.is_empty() {
        return Err(KernelError::Argument("the new face has no triangles to check".into()));
    }
    let mean_area = mesh.area() / n as f64;
    let sign = if forward { 1.0 } else { -1.0 };
    let reach = 10.0 * gap + 1.0;
    let (mut checked, mut agree) = (0usize, 0usize);
    let mut evals: BTreeMap<usize, SurfaceEval> = BTreeMap::new();
    for k in 0..CHECKS.min(3 * n) {
        let (t, corner) = spread(k, n);
        let Some(tri) = mesh.triangle(t) else { continue };
        let pts = tri.map(|a| [a.x, a.y, a.z]);
        let (Some(a), Some(b), Some(c)) = (pts.get(corner), pts.get((corner + 1) % 3), pts.get((corner + 2) % 3))
        else {
            continue;
        };
        let twice = norm(cross(sub(*b, *a), sub(*c, *a)));
        let Some(tn) =
            unit(cross(sub(*b, *a), sub(*c, *a))).filter(|_| twice >= MIN_TRIANGLE_AREA_SHARE * 2.0 * mean_area)
        else {
            continue;
        };
        let (distance, normal) = nearest_over(cloud, body, &mut evals, *a, reach)?;
        if distance > PROXIMITY_SLACK * gap + 1e-6 && !cloud.near_border(*a, BORDER_MARGIN * gap) {
            return Err(KernelError::Argument(format!(
                "the new face leaves the patches it replaced by {distance:.3} mm (limit {:.3}) at ({:.1}, {:.1}, {:.1})",
                PROXIMITY_SLACK * gap,
                a[0],
                a[1],
                a[2]
            )));
        }
        if let Some(pn) = normal {
            checked += 1;
            agree += usize::from(sign * dot(tn, pn) >= MIN_DOT);
        }
    }
    if checked == 0 || ((checked - agree) as f64) > DISAGREE_ALLOWANCE * checked as f64 {
        return Err(KernelError::Argument(format!(
            "the new face's normal disagrees with the patches' ({agree} of {checked} checked)"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::sheet_from_bezier_network;

    /// A square sheet of one bilinear patch in z = 0, its normal along +z, and the two triangles of it.
    fn sheet() -> (Brep, Mesh) {
        let net = vec![vec![[0.0, 0.0, 0.0], [0.0, 10.0, 0.0]], vec![[10.0, 0.0, 0.0], [10.0, 10.0, 0.0]]];
        let body = sheet_from_bezier_network(&[net], &[[0, 1, 2, 3]]).unwrap();
        let p = |x: f64, y: f64| surfacecraft_geom::Point3::new(x, y, 0.0);
        let mesh = Mesh {
            positions: vec![p(0.0, 0.0), p(10.0, 0.0), p(10.0, 10.0), p(0.0, 10.0)],
            triangles: vec![[0, 1, 2], [0, 2, 3]],
        };
        (body, mesh)
    }

    #[test]
    fn a_face_that_lies_on_the_patches_with_their_normal_is_accepted_and_a_flipped_one_is_refused() {
        let (body, mesh) = sheet();
        let c = cloud(&body, &[0], Vec::new()).unwrap();
        let outward = c.normals.first().copied().unwrap();
        let along_mesh = outward[2] > 0.0;
        assert!(check_face(&mesh, along_mesh, &c, &body, 0.05).is_ok());
        assert!(check_face(&mesh, !along_mesh, &c, &body, 0.05).is_err(), "a flipped sense");
    }

    #[test]
    fn a_face_off_the_patches_is_refused() {
        let (body, mut mesh) = sheet();
        let c = cloud(&body, &[0], Vec::new()).unwrap();
        for p in &mut mesh.positions {
            p.z = 1.0;
        }
        let along_mesh = c.normals.first().is_some_and(|n| n[2] > 0.0);
        assert!(check_face(&mesh, along_mesh, &c, &body, 0.05).is_err());
    }
}
