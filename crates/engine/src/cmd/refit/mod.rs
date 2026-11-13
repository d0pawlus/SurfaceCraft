//! `body.refit_features` and `body.refit_faces`: update a CAD body so that it matches a new scan
//! of the changed part (as-built). The scan is already aligned to the body.
//!
//! * `body.refit_features` re-dimensions a body that has a feature history: sketch dimensions,
//!   extrude depths and blend sizes are optimised, within bounds, to minimise the scan-to-body
//!   distance (Huber least squares, finite-difference Jacobian over the replay).
//! * `body.refit_faces` edits a body without history: scan points are associated to faces
//!   (nearest face whose normal faces the point), and each analytic face is moved along its
//!   normal by a regularised amount; the kernel rebuilds the solid.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;
use surfacecraft_deviation::against_mesh;
use surfacecraft_doc::{Document, FeatureOp, ObjectId, ObjectKind};
use surfacecraft_fit::lm::{BoundedOptions, minimize_bounded, robust_residual};
use surfacecraft_geom::{Bvh, Point3, Vec3, par};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::refit::{
    FaceMove, FaceShape, SAME_SURFACE_TOL, face_shapes, offset_face_groups, relation_counts,
};
use surfacecraft_sketch::{Constraint, Sketch, solve};

/// Scan points a refit measures with at most (more are thinned evenly).
const MAX_POINTS: usize = 200_000;

fn positive(v: Option<f64>, default: f64, what: &str) -> CmdResult<f64> {
    let v = v.unwrap_or(default);
    if v.is_finite() && v > 0.0 { Ok(v) } else { Err(CmdError::Params(format!("{what} must be a positive number"))) }
}

/// The scan's vertices and unit vertex normals (zero when unknown), thinned to `max_points`.
fn scan_points(s: &Session, mesh: &ObjRef, max_points: usize) -> CmdResult<(Vec<Point3>, Vec<Vec3>)> {
    let id = mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let normals = m.mesh.vertex_normals();
    let n = m.mesh.positions.len();
    if n == 0 {
        return Err(CmdError::Params("the scan has no points".into()));
    }
    let stride = n.div_ceil(max_points.clamp(1, MAX_POINTS));
    let mut pts = Vec::new();
    let mut nrm = Vec::new();
    for i in (0..n).step_by(stride) {
        if let Some(p) = m.mesh.positions.get(i)
            && p.coords.iter().all(|c| c.is_finite())
        {
            pts.push(*p);
            nrm.push(normals.get(i).copied().unwrap_or_else(Vec3::zeros));
        }
    }
    if pts.is_empty() {
        return Err(CmdError::Params("the scan has no finite points".into()));
    }
    Ok((pts, nrm))
}

fn bbox_diagonal(b: &Brep) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &b.vertices {
        for k in 0..3 {
            lo[k] = lo[k].min(v.point[k]);
            hi[k] = hi[k].max(v.point[k]);
        }
    }
    let d = (0..3).map(|k| (hi[k] - lo[k]).powi(2)).sum::<f64>().sqrt();
    if d.is_finite() { d } else { 1.0 }
}

fn rms(values: &[f64]) -> (f64, usize) {
    let v: Vec<f64> = values.iter().copied().filter(|x| x.is_finite()).collect();
    if v.is_empty() {
        return (f64::NAN, 0);
    }
    ((v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt(), v.len())
}
mod faces;
mod faces_cmd;
mod features;

use faces::*;
use faces_cmd::*;
use features::*;

/// `body.refit_*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "body.refit_features",
        "Refit features to scan",
        "Solid",
        true,
        "Optimises dimensions of the feature history of `body` (sketch lengths/radii, extrude depths, blend sizes; `params` or all by default) within ±`max_change` to minimise the deviation of the aligned scan `mesh`; reports old and new values and the RMS before and after.",
        RefitFeaturesIn,
        RefitFeaturesOut,
        refit_features
    ),
    command!(
        "body.refit_faces",
        "Refit faces to scan",
        "Solid",
        true,
        "Moves the analytic faces of a history-free `body` along their normals to match the aligned scan `mesh`, with per-face stiffness (fixed, tight, loose, free), equal radii kept equal, and rebuilds the solid in the kernel; freeform faces are refused.",
        RefitFacesIn,
        RefitFacesOut,
        refit_faces
    ),
];
