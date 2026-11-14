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
/// Triangles of the new face that are checked.
const CHECKS: usize = 64;
/// The least dot product of the new face's normal with the nearest patch normal.
const MIN_DOT: f64 = 0.5;
/// The share of the checked triangles that must agree in normal. A face turned the wrong way agrees at
/// almost none of them and a right one at most (measured 0.4 to 1.0 on real scans: the tessellation of
/// a face and the normals of a collapsed patch are noisy at corners), so this is a vote, not a bound.
const AGREE_SHARE: f64 = 0.6;
/// Triangles with less than this share of the mean triangle area are slivers: their normals are noise.
const MIN_TRIANGLE_AREA_SHARE: f64 = 0.3;
/// The patches nearest a corner that are tried.
const NEAREST_PATCHES: usize = 12;
/// A point of the new face may be this much farther from the patches than the gap.
const PROXIMITY_SLACK: f64 = 1.5;

/// Points of the replaced patches with their outward normals.
pub(super) struct Cloud {
    points: Vec<V>,
    normals: Vec<V>,
    faces: Vec<usize>,
}

fn forward_of(body: &Brep, face: usize) -> bool {
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
pub(super) fn cloud(body: &Brep, faces: &[usize]) -> Result<Cloud> {
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
    let mut c = Cloud { points: Vec::new(), normals: Vec::new(), faces: Vec::new() };
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
    /// The patches a corner `p` may lie over: those whose box of samples (grown by a tenth of its
    /// size and `slack`) holds it, nearest box centre first, at most `n`; when none does, the `n`
    /// patches with a sample nearest `p`.
    fn nearest_faces(&self, p: V, n: usize, slack: f64) -> Vec<usize> {
        let mut boxes: BTreeMap<usize, (V, V)> = BTreeMap::new();
        for (q, f) in self.points.iter().zip(&self.faces) {
            let b = boxes.entry(*f).or_insert((*q, *q));
            for k in 0..3 {
                b.0[k] = b.0[k].min(q[k]);
                b.1[k] = b.1[k].max(q[k]);
            }
        }
        let mut inside: Vec<(f64, usize)> = Vec::new();
        for (f, (lo, hi)) in &boxes {
            let grow = slack + 0.1 * norm(sub(*hi, *lo));
            if (0..3).all(|k| p[k] >= lo[k] - grow && p[k] <= hi[k] + grow) {
                let centre = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])];
                inside.push((norm(sub(centre, p)), *f));
            }
        }
        if inside.is_empty() {
            inside = boxes
                .iter()
                .map(|(f, (lo, hi))| {
                    (norm(sub([0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])], p)), *f)
                })
                .collect();
        }
        inside.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        inside.into_iter().take(n).map(|(_, f)| f).collect()
    }

    fn nearest(&self, p: V) -> Option<usize> {
        let d = |q: &V| norm(sub(*q, p));
        (0..self.points.len()).min_by(|&a, &b| {
            let (da, db) = (self.points.get(a).map_or(f64::MAX, d), self.points.get(b).map_or(f64::MAX, d));
            da.total_cmp(&db)
        })
    }
}

/// Checks the triangles of the new face (`mesh`, whose normals are the face's natural ones; `forward`
/// says whether that is outward) against the cloud of the replaced patches `body`: normals agree and
/// the corners of the triangles (which lie on the surface, unlike their centres) lie within `gap`
/// (times a slack) of the patch nearest to them.
pub(super) fn check_face(mesh: &Mesh, forward: bool, cloud: &Cloud, body: &Brep, gap: f64) -> Result<()> {
    let n = mesh.triangle_count();
    if n == 0 || cloud.points.is_empty() {
        return Err(KernelError::Argument("the new face has no triangles to check".into()));
    }
    let mean_area = mesh.area() / n as f64;
    let step = (n / CHECKS).max(1);
    let (mut checked, mut agree) = (0usize, 0usize);
    let mut evals: BTreeMap<usize, SurfaceEval> = BTreeMap::new();
    for t in (0..n).step_by(step) {
        let Some([a, b, c]) = mesh.triangle(t) else { continue };
        let (a, b, c) = ([a.x, a.y, a.z], [b.x, b.y, b.z], [c.x, c.y, c.z]);
        let Some(k) = cloud.nearest(a) else { continue };
        let Some(pn) = cloud.normals.get(k) else { continue };
        let twice = norm(cross(sub(b, a), sub(c, a)));
        if twice < MIN_TRIANGLE_AREA_SHARE * 2.0 * mean_area {
            continue;
        }
        let Some(tn) = unit(cross(sub(b, a), sub(c, a))) else { continue };
        let sign = if forward { 1.0 } else { -1.0 };
        checked += 1;
        agree += usize::from(sign * dot(tn, *pn) >= MIN_DOT);
        // the nearest sample may belong to a neighbour of the patch the corner is over: the best of the
        // few nearest patches counts
        let mut best = f64::INFINITY;
        for f in cloud.nearest_faces(a, NEAREST_PATCHES, gap) {
            if let std::collections::btree_map::Entry::Vacant(slot) = evals.entry(f) {
                slot.insert(evaluator(body, f)?);
            }
            let reach = evals.get(&f).and_then(|e| {
                let p = Vec3::new(a[0], a[1], a[2]);
                let q = e.point(e.project(p, None, 10.0 * gap + 1.0)?)?;
                Some(norm(sub([q.x, q.y, q.z], a)))
            });
            best = best.min(reach.unwrap_or(f64::INFINITY));
        }
        if best.is_finite() && best > PROXIMITY_SLACK * gap + 1e-6 {
            return Err(KernelError::Argument("the new face leaves the patches it replaced".into()));
        }
    }
    if checked == 0 || (agree as f64) < AGREE_SHARE * checked as f64 {
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
        let c = cloud(&body, &[0]).unwrap();
        let outward = c.normals.first().copied().unwrap();
        let along_mesh = outward[2] > 0.0;
        assert!(check_face(&mesh, along_mesh, &c, &body, 0.05).is_ok());
        assert!(check_face(&mesh, !along_mesh, &c, &body, 0.05).is_err(), "a flipped sense");
    }

    #[test]
    fn a_face_off_the_patches_is_refused() {
        let (body, mut mesh) = sheet();
        let c = cloud(&body, &[0]).unwrap();
        for p in &mut mesh.positions {
            p.z = 1.0;
        }
        let along_mesh = c.normals.first().is_some_and(|n| n[2] > 0.0);
        assert!(check_face(&mesh, along_mesh, &c, &body, 0.05).is_err());
    }
}
