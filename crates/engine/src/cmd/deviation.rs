//! `deviation.*`: how far a scan is from a reference, with a colour map on the mesh.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_deviation::{DeviationStats, against_mesh, against_primitive, colorize, estimate_noise, stats};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::{Bvh, Point3};

/// Deviation parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ComputeIn {
    /// Scan mesh.
    pub object: ObjRef,
    /// Reference: a primitive, a mesh or a body.
    pub against: ObjRef,
    /// Only the vertices of this region.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Colour-map tolerance (mm, default 0.1).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Points farther than this from a mesh/body reference are not measured (mm, default 5).
    #[serde(default)]
    pub max_distance: Option<f64>,
    /// Show the colour map on the mesh (default true).
    #[serde(default = "yes")]
    pub show: bool,
}

fn yes() -> bool {
    true
}

/// Deviation result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ComputeOut {
    /// Scan mesh.
    pub id: ObjectId,
    /// Statistics.
    pub stats: DeviationStats,
    /// Warning when the tolerance is below the estimated scan noise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

fn compute(s: &mut Session, i: ComputeIn) -> CmdResult<ComputeOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let tol = i.tolerance.unwrap_or(0.1);
    if !(tol.is_finite() && tol > 0.0) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    let maxd = i.max_distance.unwrap_or(5.0);
    if !(maxd.is_finite() && maxd > 0.0) {
        return Err(CmdError::Params("max_distance must be positive".into()));
    }
    // which vertices are measured
    let mut mask = vec![true; m.vertex_count()];
    if let Some(r) = &i.region {
        let rid = r.resolve(s, ObjectKind::Region)?;
        let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
        if reg.mesh != id {
            return Err(CmdError::Params("the region belongs to another mesh".into()));
        }
        mask.fill(false);
        for &t in reg.triangles.iter() {
            for &vi in m.triangles.get(t as usize).into_iter().flatten() {
                if let Some(x) = mask.get_mut(vi as usize) {
                    *x = true;
                }
            }
        }
    }
    let pts: Vec<Point3> = m.positions.clone();
    let (rid, kind) = i.against.resolve_any(s)?;
    let mut values = match kind {
        ObjectKind::Primitive => {
            let p = s.doc.primitive(rid).ok_or_else(|| CmdError::Params(format!("no primitive {rid}")))?;
            against_primitive(&pts, &p.primitive)
        }
        ObjectKind::Mesh => {
            let t =
                s.doc.mesh(rid).map(|t| t.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {rid}")))?;
            against_mesh(&pts, &Bvh::build(&t), maxd)
        }
        ObjectKind::Body => {
            let b = s.doc.body(rid).ok_or_else(|| CmdError::Params(format!("no body {rid}")))?;
            let t = surfacecraft_kernel::tess::tessellate(&b.brep, Some(tol * 0.05))?;
            against_mesh(&pts, &Bvh::build(&t), maxd)
        }
        ObjectKind::Surface => {
            let o = s.doc.surface(rid).ok_or_else(|| CmdError::Params(format!("no surface {rid}")))?;
            let face = o.sheet.faces.first().ok_or_else(|| CmdError::Failed("the surface has no face".into()))?;
            let probe = surfacecraft_kernel::nurbs::SurfaceProbe::new(&face.surface)?;
            let mut v = Vec::with_capacity(pts.len());
            for (p, keep) in pts.iter().zip(&mask) {
                v.push(if *keep { probe.closest([p.x, p.y, p.z]).map_or(f64::NAN, |r| r.0) } else { f64::NAN });
            }
            v
        }
        ObjectKind::Region | ObjectKind::Quad => {
            return Err(CmdError::Params("compare against a primitive, mesh, body or surface".into()));
        }
    };
    for (v, keep) in values.iter_mut().zip(&mask) {
        if !keep {
            *v = f64::NAN;
        }
    }
    let st = stats(&values, tol);
    if i.show {
        s.overlays.insert(id, Arc::new(colorize(&values, tol)));
    }
    let noise = estimate_noise(&m);
    let warning =
        (noise > tol).then(|| format!("tolerance {tol} mm is below the estimated scan noise σ ≈ {noise:.3} mm"));
    Ok(ComputeOut { id, stats: st, warning })
}

/// One mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshIn {
    /// Mesh id or name.
    pub object: ObjRef,
}

/// Cleared.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ClearOut {
    /// Mesh.
    pub id: ObjectId,
    /// Was a map shown?
    pub cleared: bool,
}

fn clear(s: &mut Session, i: MeshIn) -> CmdResult<ClearOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    Ok(ClearOut { id, cleared: s.overlays.remove(&id).is_some() })
}

/// Noise estimate.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct NoiseOut {
    /// Mesh.
    pub id: ObjectId,
    /// Estimated σ (mm).
    pub sigma: f64,
    /// Advice relative to the default tolerances (design doc §10).
    pub advice: String,
}

fn noise(s: &mut Session, i: MeshIn) -> CmdResult<NoiseOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let sigma = estimate_noise(&m);
    let advice = if sigma > 0.1 {
        "noise is above the 0.1 mm primitive tolerance: expect RMS above it".to_string()
    } else if sigma > 0.05 {
        "noise is above the 0.05 mm warning level".to_string()
    } else {
        "noise is below the default tolerances".to_string()
    };
    Ok(NoiseOut { id, sigma, advice })
}

/// `deviation.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "deviation.compute",
        "Deviation map",
        "Analyze",
        false,
        "Signed deviation of `object` (or its `region`) from `against` (primitive/mesh/body); colour map ±`tolerance`.",
        ComputeIn,
        ComputeOut,
        compute
    ),
    command!(
        "deviation.clear",
        "Clear deviation map",
        "Analyze",
        false,
        "Removes the colour map of `object`.",
        MeshIn,
        ClearOut,
        clear
    ),
    command!(
        "deviation.noise",
        "Estimate noise",
        "Analyze",
        false,
        "Estimates the scan noise σ of `object`.",
        MeshIn,
        NoiseOut,
        noise
    ),
];
