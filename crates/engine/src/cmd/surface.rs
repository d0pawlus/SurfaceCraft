//! `surface.*`: freeform NURBS surfaces fitted to regions (design doc §6.3, milestone M3).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, SurfaceObject};
use surfacecraft_fit::resample::height_grid;
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Mesh, Point3};
use surfacecraft_kernel::nurbs::{SurfaceProbe, fit_grid, sheet_from_spline};

/// Points measured per fit (evenly strided) when choosing the control net.
const PROBE_POINTS: usize = 1500;

/// Fit parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FitIn {
    /// Region to fit.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh` (all when omitted with `mesh`).
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Control net `[u, v]`; omitted → the smallest net meeting `tolerance`.
    #[serde(default)]
    pub control: Option<[usize; 2]>,
    /// Degree (default 3).
    #[serde(default)]
    pub degree: Option<usize>,
    /// Target maximum deviation (mm, default 0.2 — design doc §11.1).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// Fit result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FitOut {
    /// New surface object.
    pub id: ObjectId,
    /// Control net used.
    pub control: [usize; 2],
    /// Degree.
    pub degree: usize,
    /// RMS deviation of the region points from the surface (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Points measured.
    pub measured: usize,
    /// Within the target tolerance?
    pub within_tolerance: bool,
    /// Advice when the target was not met or the region folds.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

fn measure(probe: &SurfaceProbe, pts: &[Point3]) -> CmdResult<(f64, f64, usize)> {
    let mut sq = 0.0;
    let mut max: f64 = 0.0;
    let mut n = 0;
    for p in pts {
        let (d, _) = probe.closest([p.x, p.y, p.z])?;
        sq += d * d;
        max = max.max(d);
        n += 1;
    }
    Ok(((sq / n.max(1) as f64).sqrt(), max, n))
}

fn fit_cmd(s: &mut Session, i: FitIn) -> CmdResult<FitOut> {
    let (mesh, tris, region) = match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            (reg.mesh, reg.triangles.to_vec(), Some(rid))
        }
        (None, Some(m)) => {
            let mid = m.resolve(s, ObjectKind::Mesh)?;
            let all = s.doc.mesh(mid).map_or(0, |m| m.mesh.triangle_count()) as u32;
            (mid, i.triangles.clone().unwrap_or_else(|| (0..all).collect()), None)
        }
        _ => return Err(CmdError::Params("give `region`, or `mesh` (and optionally `triangles`)".into())),
    };
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    if let Some(t) = tris.iter().find(|&&t| t as usize >= m.triangle_count()) {
        return Err(CmdError::Params(format!("triangle {t} out of range")));
    }
    let degree = i.degree.unwrap_or(3);
    if !(1..=5).contains(&degree) {
        return Err(CmdError::Params("degree must be 1..=5".into()));
    }
    let tol = i.tolerance.unwrap_or(0.2);
    if !(tol.is_finite() && tol > 0.0) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    let normals = m.vertex_normals();
    let (pts, nrm) = region_points(&m, &normals, &tris);
    if pts.len() < 16 {
        return Err(CmdError::Params("too few points for a surface (need 16)".into()));
    }
    let max_ctrl = i.control.map_or(40, |c| c[0].max(c[1]));
    let g = (max_ctrl * 2).clamp(30, 120);
    let grid = height_grid(&pts, &nrm, g, g).ok_or_else(|| {
        CmdError::Failed("the region is not a single patch over its plane (it folds or is degenerate); split it".into())
    })?;
    let stride = pts.len().div_ceil(PROBE_POINTS).max(1);
    let probe_pts: Vec<Point3> = pts.iter().step_by(stride).copied().collect();
    let candidates: Vec<[usize; 2]> = match i.control {
        Some(c) => vec![c],
        None => {
            [6usize, 8, 11, 15, 20, 27, 36].iter().map(|&c| [c, c]).filter(|c| c[0] > degree && c[0] <= g).collect()
        }
    };
    let mut best: Option<(surfacecraft_kernel::record::brep::Surface, [usize; 2], f64, f64)> = None;
    for c in candidates {
        let srf = fit_grid(&grid.grid, c[0], c[1], degree)?;
        let probe = SurfaceProbe::new(&srf)?;
        let (rms, max, _) = measure(&probe, &probe_pts)?;
        let better = best.as_ref().is_none_or(|b| max < b.3);
        if better {
            best = Some((srf, c, rms, max));
        }
        if max <= tol {
            break;
        }
    }
    let (srf, control, _, _) = best.ok_or_else(|| CmdError::Failed("no surface could be fitted".into()))?;
    // final statistics over every region point
    let probe = SurfaceProbe::new(&srf)?;
    let (rms, max, measured) = measure(&probe, &pts)?;
    let sheet = sheet_from_spline(&srf)?;
    let display = surfacecraft_kernel::tess::tessellate(&sheet, None).unwrap_or_else(|_| Mesh::default());
    let within = max <= tol;
    let warning = if !within {
        Some(format!(
            "max deviation {max:.4} mm exceeds the {tol} mm target; split the region or raise the control net"
        ))
    } else if grid.fold > tol {
        Some(format!("the region is far from a height field (fold {:.3} mm): check the patch", grid.fold))
    } else {
        None
    };
    let name = i.name.unwrap_or_else(|| "surface".into());
    let id = s.doc.add_surface(SurfaceObject {
        id: ObjectId(0),
        name,
        sheet: Arc::new(sheet),
        region,
        control,
        degree,
        rms,
        max,
        display: Arc::new(display),
        visible: true,
    });
    Ok(FitOut { id, control, degree, rms, max, measured, within_tolerance: within, warning })
}

/// One surface.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SurfIn {
    /// Surface id or name.
    pub object: ObjRef,
}

/// Surface facts.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SurfOut {
    /// Id.
    pub id: ObjectId,
    /// Control net.
    pub control: [usize; 2],
    /// Degree.
    pub degree: usize,
    /// RMS (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Faces / edges of the sheet.
    pub faces: usize,
    /// Edges of the sheet.
    pub edges: usize,
}

fn info(s: &mut Session, i: SurfIn) -> CmdResult<SurfOut> {
    let id = i.object.resolve(s, ObjectKind::Surface)?;
    let o = s.doc.surface(id).ok_or_else(|| CmdError::Params(format!("no surface {id}")))?;
    Ok(SurfOut {
        id,
        control: o.control,
        degree: o.degree,
        rms: o.rms,
        max: o.max,
        faces: o.sheet.faces.len(),
        edges: o.sheet.edges.len(),
    })
}

/// `surface.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "surface.fit",
        "Fit surface",
        "Surface",
        true,
        "Fits one NURBS patch to `region` (or `mesh`/`triangles`); the control net grows until max deviation ≤ `tolerance`.",
        FitIn,
        FitOut,
        fit_cmd
    ),
    command!(
        "surface.info",
        "Surface info",
        "Surface",
        false,
        "Control net, degree and deviation of `object`.",
        SurfIn,
        SurfOut,
        info
    ),
];
