//! Helpers shared by `quad.split`, `quad.snap` and the AutoSurface: the orientation check, the geometry of a layout, its scan, its checks and the split.
use super::super::{CmdError, CmdResult, ObjRef, QuadInfo};
use super::*;
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, QuadObject};
use surfacecraft_fit::boundary::{Outline, cleaned_outline};
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::orient::{FlipReport, repair_orientation, same_side_normal};
use surfacecraft_fit::retopo::{CheckedSplit, QuadMesh, checked_split};
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{KdTree, Point3, Vec3};

/// Detects quads whose patch runs against the scan and turns them around (the change persists in
/// `layout`); `scan_points` are the scan points with their oriented normals.
pub(super) fn orientation_check(
    layout: &mut QuadMesh,
    scan: &ScanSurface,
    scan_points: (&[Point3], &[Vec3]),
    spacing: f64,
) -> FlipReport {
    let (pts, normals) = scan_points;
    let tree = KdTree::build(pts);
    let reference = |p: &Point3, hint: &Vec3| same_side_normal(&tree, normals, 1.0, p, hint);
    let radius = layout_radius(layout, spacing);
    repair_orientation(layout, scan, radius, &reference)
}

/// The unstructured layout of a quad object.
pub(in crate::cmd::quad) fn layout_of(q: &QuadObject) -> QuadMesh {
    let v3 = |p: &[f64; 3]| Point3::new(p[0], p[1], p[2]);
    QuadMesh {
        vertices: q.vertices.iter().map(v3).collect(),
        normals: q.normals.iter().map(|n| Vec3::new(n[0], n[1], n[2])).collect(),
        faces: q.faces.clone(),
    }
}

/// One layout.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct QuadIn {
    /// Quad layout id or name.
    pub object: ObjRef,
    /// Experimental: conform the layout of an open region to a smoothed outline of the region and put split boundary vertices back onto it (default false). Falls back to the plain layout when any quad would be improper.
    #[serde(default)]
    pub conform_boundary: Option<bool>,
}

/// Largest unstructured layout (quads) a split or an automatic refinement produces.
pub(super) const MAX_LAYOUT_QUADS: usize = 12_000;

/// Mean quad edge length of a layout (mm).
pub(super) fn layout_edge(l: &QuadMesh) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for f in &l.faces {
        for k in 0..4 {
            if let (Some(a), Some(b)) = (l.vertices.get(f[k] as usize), l.vertices.get(f[(k + 1) % 4] as usize)) {
                sum += (b - a).norm();
                n += 1;
            }
        }
    }
    sum / n.max(1) as f64
}

/// MLS support for a layout: wide enough to average the scan noise, narrow enough to follow it.
pub(in crate::cmd::quad) fn layout_radius(l: &QuadMesh, spacing: f64) -> f64 {
    (0.8 * layout_edge(l)).max(3.0 * spacing)
}

/// New boundary vertices of a split move onto the outline when within this fraction of the old edge length.
const OUTLINE_SNAP_FRACTION: f64 = 0.4;

/// The outline of the region under layout `q` (empty for a closed mesh).
pub(super) fn outline_for(s: &Session, q: &QuadObject) -> CmdResult<Outline> {
    let m =
        s.doc.mesh(q.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {}", q.mesh)))?;
    let sub = match q.region {
        Some(r) => {
            let tris =
                s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?;
            m.submesh(&tris)
        }
        None => (*m).clone(),
    };
    Ok(cleaned_outline(&sub).0)
}

/// One split of the layout with its new free-boundary vertices put back onto the outline, accepted
/// only if no proper quad turns improper on the real (projected) geometry.
pub(super) fn split_conformed(layout: &QuadMesh, scan: &ScanSurface, outline: &Outline, radius: f64) -> CheckedSplit {
    let edge = layout_edge(layout);
    checked_split(layout, &projector(scan, radius), outline, (edge, OUTLINE_SNAP_FRACTION * edge))
}

/// Checks an unstructured layout read from a document (hostile files).
pub(in crate::cmd::quad) fn check_layout(q: &QuadObject) -> CmdResult<()> {
    let n = q.vertices.len();
    if q.normals.len() != n || q.faces.iter().flatten().any(|&i| i as usize >= n) {
        return Err(CmdError::Failed("the layout is inconsistent (normals or face ids)".into()));
    }
    if q.vertices.iter().flatten().chain(q.normals.iter().flatten()).any(|v| !v.is_finite()) {
        return Err(CmdError::Failed("the layout has non-finite coordinates".into()));
    }
    Ok(())
}

/// The scan points under layout `q`, their normals and the mean edge length of the scan there.
pub(in crate::cmd::quad) fn scan_for(s: &Session, q: &QuadObject) -> CmdResult<(Vec<Point3>, Vec<Vec3>, f64)> {
    let m =
        s.doc.mesh(q.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {}", q.mesh)))?;
    let tris: Vec<u32> = match q.region {
        Some(r) => s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?,
        None => (0..m.triangle_count() as u32).collect(),
    };
    let normals = m.vertex_normals();
    let (pts, point_normals) = region_points(&m, &normals, &tris);
    Ok((pts, point_normals, mean_edge(&m, &tris)))
}

pub(in crate::cmd::quad) fn info_of(q: &QuadObject, snap_max: f64) -> QuadInfo {
    QuadInfo {
        id: q.id,
        nu: q.nu,
        nv: q.nv,
        creases: (q.crease_u.clone(), q.crease_v.clone()),
        snap_max,
        repaired_patches: None,
        flipped_patches: None,
        unverified_patches: None,
        outline_loops_unsnapped: None,
        outline_unsnapped_reason: None,
        split_rejected: None,
        improper_before: None,
        improper_after: None,
        snap_rejected: None,
    }
}

pub(in crate::cmd::quad) fn split_unstructured(s: &mut Session, id: ObjectId, conform: bool) -> CmdResult<QuadInfo> {
    let q = s.doc.quad(id).cloned().ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    check_layout(&q)?;
    if q.faces.len() * 4 > MAX_LAYOUT_QUADS {
        return Err(CmdError::Params(format!("the layout would exceed {MAX_LAYOUT_QUADS} quads")));
    }
    let (pts, point_normals, spacing) = scan_for(s, &q)?;
    let scan = ScanSurface::oriented(&pts, &point_normals);
    let layout = layout_of(&q);
    let outline = if conform { outline_for(s, &q)? } else { Outline::default() };
    let step = split_conformed(&layout, &scan, &outline, layout_radius(&layout, spacing));
    let (resnap, rejected) = (step.report, step.rejected);
    let mut split = step.layout;
    let flips = if rejected {
        FlipReport::default() // the layout is as it was: nothing to check again
    } else {
        orientation_check(&mut split, &scan, (&pts, &point_normals), spacing)
    };
    let snap_max = {
        let bvh_pts = ScanSurface::new(&pts);
        split.vertices.iter().filter_map(|p| bvh_pts.nearest(p)).map(|x| x.1).fold(0.0, f64::max)
    };
    if let (false, Some(m)) = (rejected, s.doc.quad_mut(id)) {
        m.vertices = split.vertices.iter().map(|p| [p.x, p.y, p.z]).collect();
        m.normals = split.normals.iter().map(|n| [n.x, n.y, n.z]).collect();
        m.faces = split.faces;
    }
    let q = s.doc.quad(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    Ok(QuadInfo {
        repaired_patches: Some(flips.repaired),
        flipped_patches: Some(flips.failed),
        unverified_patches: Some(flips.unverified),
        outline_loops_unsnapped: Some(resnap.unsnapped()),
        improper_before: Some(step.improper_before),
        improper_after: Some(step.improper_after),
        outline_unsnapped_reason: resnap.reason.map(str::to_string),
        split_rejected: Some(rejected),
        ..info_of(q, snap_max)
    })
}
