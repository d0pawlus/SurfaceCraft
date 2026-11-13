//! Suggestions for a feature's parameters (fillet radius, axis, draft) from the selected geometry.

use super::shapes::*;
use super::*;

/// Suggestion parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuggestIn {
    /// Body whose edge would be filleted.
    pub body: ObjRef,
    /// Edge index.
    pub edge: usize,
    /// Scan mesh to measure on.
    pub mesh: ObjRef,
    /// Search radius around the edge (mm, default 10).
    #[serde(default)]
    pub search: Option<f64>,
}

/// A suggested parameter.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SuggestOut {
    /// Suggested fillet radius (mm).
    pub radius: f64,
    /// RMS of the cylinder fitted to the scan around the edge (mm).
    pub rms: f64,
    /// Scan points used.
    pub points: usize,
}

/// Redesign-assistant style suggestion (design doc §6.4): the radius of the rounded scan around a
/// body edge, from a cylinder fitted to the scan points near the edge's middle.
pub(super) fn suggest_fillet(s: &mut Session, i: SuggestIn) -> CmdResult<SuggestOut> {
    let body = resolve_body(s, &i.body)?;
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let b = s.doc.body(body).ok_or_else(|| CmdError::Params(format!("no body {body}")))?;
    let e = b.brep.edges.get(i.edge).ok_or_else(|| CmdError::Params(format!("edge {} does not exist", i.edge)))?;
    let v = |k: u32| b.brep.vertices.get(k as usize).map(|v| Point3::from(v.point));
    let (Some(a), Some(c)) = (v(e.vertices[0]), v(e.vertices[1])) else {
        return Err(CmdError::Failed("edge without vertices".into()));
    };
    let mid = Point3::from((a.coords + c.coords) / 2.0);
    let along = surfacecraft_geom::normalize(c - a).unwrap_or_else(Vec3::z);
    let half = ((c - a).norm() / 2.0) * 0.6;
    let r = i.search.unwrap_or(10.0);
    if !(r.is_finite() && r > 0.0) {
        return Err(CmdError::Params("search must be positive".into()));
    }
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let normals = m.vertex_normals();
    let mut pts = Vec::new();
    let mut nrm = Vec::new();
    for (p, n) in m.positions.iter().zip(&normals) {
        let w = p - mid;
        let t = w.dot(&along);
        if t.abs() <= half && (w - along * t).norm() <= r {
            // only the rounded part: normals turning across the edge
            pts.push(*p);
            nrm.push(*n);
        }
    }
    let f = surfacecraft_fit::primitive::fit_cylinder(&pts, &nrm)
        .ok_or_else(|| CmdError::Failed("no rounded scan near this edge".into()))?;
    let surfacecraft_fit::Primitive::Cylinder { radius, .. } = f.primitive else {
        return Err(CmdError::Failed("no cylinder".into()));
    };
    Ok(SuggestOut { radius, rms: f.rms, points: pts.len() })
}

pub(super) fn new_body_name(i: &Option<String>, target: Option<ObjectId>) -> Option<String> {
    target.is_none().then(|| i.clone().unwrap_or_else(|| "part".to_string()))
}

/// Axis suggestion parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuggestAxisIn {
    /// Region of a turned (revolved) surface of the scan.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
}

/// A suggested revolve axis.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SuggestAxisOut {
    /// A point of the axis (`axis_origin` of `feature.revolve`).
    pub axis_origin: [f64; 3],
    /// Unit axis direction (`axis_direction` of `feature.revolve`).
    pub axis_direction: [f64; 3],
    /// What the scan is: `cylinder`, `cone`, `torus` or `sphere`.
    pub surface: String,
    /// Radius (cylinder, sphere) or major radius (torus); 0 for a cone.
    pub radius: f64,
    /// RMS of the fit (mm).
    pub rms: f64,
    /// Scan points used.
    pub points: usize,
}

/// Redesign assistant (design doc §6.4): the revolve axis of a turned scan surface, from the best
/// cylinder, cone, torus or sphere fitted to it.
pub(super) fn suggest_axis(s: &mut Session, i: SuggestAxisIn) -> CmdResult<SuggestAxisOut> {
    use surfacecraft_fit::{Primitive, PrimitiveKind};
    let (p, n) = selection_points(s, &i.region, &i.mesh, &i.triangles)?;
    let fits: Vec<_> = [PrimitiveKind::Sphere, PrimitiveKind::Cylinder, PrimitiveKind::Cone, PrimitiveKind::Torus]
        .into_iter()
        .filter_map(|k| surfacecraft_fit::primitive::fit(k, &p, &n))
        .filter(|f| f.rms.is_finite())
        .collect();
    let floor = fits.iter().map(|f| f.rms).fold(f64::INFINITY, f64::min);
    // a plane that fits as well as any turned surface means there is no axis to find
    if let Some(pl) = surfacecraft_fit::primitive::fit(PrimitiveKind::Plane, &p, &n)
        && pl.rms <= floor * 1.2 + 1e-9
    {
        return Err(CmdError::Failed("the scan is planar: it has no axis".into()));
    }
    // the simplest surface within 20 % of the best residual wins (a barely tapered cone is a cylinder)
    let best = fits
        .into_iter()
        .find(|f| f.rms <= floor * 1.2 + 1e-9)
        .ok_or_else(|| CmdError::Failed("no surface of revolution fits these scan points".into()))?;
    let (origin, dir, surface, radius) = match best.primitive {
        Primitive::Cylinder { point, axis, radius } => (point, axis, "cylinder", radius),
        Primitive::Cone { apex, axis, .. } => (apex, axis, "cone", 0.0),
        Primitive::Torus { center, axis, major_radius, .. } => (center, axis, "torus", major_radius),
        Primitive::Sphere { center, radius } => (center, [0.0, 0.0, 1.0], "sphere", radius),
        Primitive::Plane { .. } => return Err(CmdError::Failed("the scan is planar: it has no axis".into())),
    };
    Ok(SuggestAxisOut {
        axis_origin: origin,
        axis_direction: dir,
        surface: surface.into(),
        radius,
        rms: best.rms,
        points: best.count,
    })
}

/// Draft suggestion parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SuggestDraftIn {
    /// Region of the side face (a plane or a cone) of the scan.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Pull (mould opening) direction.
    pub pull: [f64; 3],
}

/// A suggested draft.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SuggestDraftOut {
    /// Draft angle, degrees (sign as `feature.draft`: positive leans the face in going along `pull`).
    pub angle_deg: f64,
    /// `plane` or `cone`.
    pub surface: String,
    /// RMS of the fit (mm).
    pub rms: f64,
    /// Scan points used.
    pub points: usize,
}

/// Redesign assistant: the draft angle of a scanned side face against the pull direction, from a
/// plane (angle between the face and the pull) or a cone (half angle) fitted to it.
pub(super) fn suggest_draft(s: &mut Session, i: SuggestDraftIn) -> CmdResult<SuggestDraftOut> {
    use surfacecraft_fit::{Primitive, PrimitiveKind};
    let pull = surfacecraft_geom::normalize(Vec3::from(i.pull))
        .filter(|v| v.iter().all(|c| c.is_finite()))
        .ok_or_else(|| CmdError::Params("`pull` must be a finite non-zero direction".into()))?;
    let (p, n) = selection_points(s, &i.region, &i.mesh, &i.triangles)?;
    let plane = surfacecraft_fit::primitive::fit(PrimitiveKind::Plane, &p, &n);
    let cone = surfacecraft_fit::primitive::fit(PrimitiveKind::Cone, &p, &n);
    // the simpler surface wins unless the cone fits clearly better (as the auto-typing rule)
    let use_cone = match (&plane, &cone) {
        (Some(pl), Some(c)) => c.rms * 1.2 < pl.rms,
        (None, Some(_)) => true,
        _ => false,
    };
    if use_cone && let Some(c) = cone {
        let Primitive::Cone { axis, half_angle, .. } = c.primitive else {
            return Err(CmdError::Failed("no cone".into()));
        };
        // the cone must be turned about the pull for a draft in the mould sense
        if Vec3::from(axis).dot(&pull).abs() < 0.9 {
            return Err(CmdError::Failed("the conical face is not turned about the pull direction".into()));
        }
        // a cone opening along the pull widens along it: its walls lean out, going along the pull
        let sign = if Vec3::from(axis).dot(&pull) > 0.0 { -1.0 } else { 1.0 };
        return Ok(SuggestDraftOut {
            angle_deg: sign * half_angle.to_degrees(),
            surface: "cone".into(),
            rms: c.rms,
            points: c.count,
        });
    }
    let pl = plane.ok_or_else(|| CmdError::Failed("no plane or cone fits these scan points".into()))?;
    let Primitive::Plane { normal, .. } = pl.primitive else {
        return Err(CmdError::Failed("no plane".into()));
    };
    // outward normal of the scanned face: the fitted normal turned to agree with the mesh normals
    let mean = n.iter().fold(Vec3::zeros(), |a, v| a + v);
    let mut nv = Vec3::from(normal);
    if nv.dot(&mean) < 0.0 {
        nv = -nv;
    }
    // positive: the outward normal leans along the pull, i.e. the face leans in going along it
    let angle = nv.dot(&pull).clamp(-1.0, 1.0).asin().to_degrees();
    Ok(SuggestDraftOut { angle_deg: angle, surface: "plane".into(), rms: pl.rms, points: pl.count })
}
