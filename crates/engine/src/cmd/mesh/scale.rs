//! `mesh.scale_from_length`: a scan of unknown scale brought to true size from one known length.
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// Points closer than this (mm) do not define a length.
const MIN_DISTANCE: f64 = 1e-9;
/// Scale factors outside this range are refused (a wrong length, not a scan).
const FACTOR_RANGE: (f64, f64) = (1e-6, 1e6);

/// Scale-from-length parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ScaleIn {
    /// Mesh to scale.
    pub object: ObjRef,
    /// First point on the scan (e.g. picked in the viewport).
    pub from: [f64; 3],
    /// Second point on the scan.
    pub to: [f64; 3],
    /// The true distance between the two points, in mm.
    pub length: f64,
    /// Fixed point of the scaling (default: the centroid of the mesh).
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
}

/// The scaling that was applied.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ScaleOut {
    /// Object scaled.
    pub id: ObjectId,
    /// Factor every coordinate was multiplied by (about `origin`).
    pub factor: f64,
    /// Distance between the two points before scaling, mm.
    pub measured_length: f64,
    /// Fixed point of the scaling.
    pub origin: [f64; 3],
}

fn finite3(v: [f64; 3]) -> CmdResult<Vec3> {
    let v = Vec3::from(v);
    if surfacecraft_geom::finite_vec(&v) {
        Ok(v)
    } else {
        Err(CmdError::Params("the points and the origin must be finite".into()))
    }
}

/// `mesh` scaled by `factor` about `origin`; `None` when a coordinate stops being finite.
fn scaled_about(mesh: &Mesh, origin: &Vec3, factor: f64) -> Option<Mesh> {
    let positions: Vec<Point3> =
        mesh.positions.iter().map(|p| Point3::from(origin + (p.coords - origin) * factor)).collect();
    positions.iter().all(surfacecraft_geom::finite).then(|| Mesh { positions, triangles: mesh.triangles.clone() })
}

pub(super) fn scale_from_length(s: &mut Session, i: ScaleIn) -> CmdResult<ScaleOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let (from, to) = (finite3(i.from)?, finite3(i.to)?);
    let measured = (to - from).norm();
    if !(measured >= MIN_DISTANCE) {
        return Err(CmdError::Params("the two points coincide: they must define a length".into()));
    }
    if !(i.length.is_finite() && i.length > 0.0) {
        return Err(CmdError::Params("length must be a positive number".into()));
    }
    let factor = i.length / measured;
    if !(factor >= FACTOR_RANGE.0 && factor <= FACTOR_RANGE.1) {
        return Err(CmdError::Params(format!(
            "the scale factor {factor:e} is outside {:e}..{:e}: check the length and the points",
            FACTOR_RANGE.0, FACTOR_RANGE.1
        )));
    }
    let obj = s.doc.mesh_mut(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let origin = match i.origin {
        Some(o) => finite3(o)?,
        None => obj.mesh.centroid().coords,
    };
    let too_big = || CmdError::Params("the scaled mesh would not be finite".into());
    let scaled = scaled_about(&obj.mesh, &origin, factor).ok_or_else(too_big)?;
    let working =
        obj.working.as_ref().map(|w| scaled_about(&w.mesh, &origin, factor).ok_or_else(too_big)).transpose()?;
    obj.mesh = Arc::new(scaled);
    if let (Some(w), Some(m)) = (obj.working.as_mut(), working) {
        w.mesh = Arc::new(m);
    }
    Ok(ScaleOut { id, factor, measured_length: measured, origin: origin.into() })
}
