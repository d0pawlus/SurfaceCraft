//! `surface.from_curves`: a patch network bounded by the sharp-edge (feature) curves of the scan
//! (design doc §6.3 second freeform strategy, milestone P4) for housings with clear edges.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, SurfaceObject};
use surfacecraft_fit::features::{FeatureOptions, feature_network, tessellate_nets};
use surfacecraft_geom::{Bvh, Point3};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network};

/// Parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FromCurvesIn {
    /// Region to cover.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh to cover whole (when no region).
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Edges whose dihedral angle exceeds this are sharp (degrees, default 35).
    #[serde(default)]
    pub angle: Option<f64>,
    /// Smooth regions with fewer triangles are ignored (default 30).
    #[serde(default)]
    pub min_triangles: Option<usize>,
    /// Degree of the patches and curves (2 to 8, default 5).
    #[serde(default)]
    pub degree: Option<usize>,
    /// Target maximum deviation (mm, default 0.2).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Also make a solid body when the patch network is closed (default true).
    #[serde(default)]
    pub make_solid: Option<bool>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// Result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FromCurvesOut {
    /// The patch network (one sheet of all patches).
    pub surface: ObjectId,
    /// Smooth regions the sharp edges cut the scan into.
    pub regions: usize,
    /// Patches made (one per four-sided region; with any other region every region is split, see the summary).
    pub patches: usize,
    /// Feature curves fitted (each shared by the patches on both sides).
    pub curves: usize,
    /// Total length of the sharp edges (mm).
    pub feature_length: f64,
    /// Regions that did not become patches, with the reason.
    pub skipped: Vec<String>,
    /// RMS deviation of the region vertices from their patches (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Within the target?
    pub within_tolerance: bool,
    /// Is the network a closed shell?
    pub closed: bool,
    /// Volume of the closed shell (mm³).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume: Option<f64>,
    /// The solid body made from a closed network.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solid: Option<ObjectId>,
    /// Advice when the target was not met or regions were skipped.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

fn from_curves(s: &mut Session, i: FromCurvesIn) -> CmdResult<FromCurvesOut> {
    let (mesh, region) = match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            (s.doc.region(rid).map(|r| r.mesh).ok_or_else(|| CmdError::Params("no region".into()))?, Some(rid))
        }
        (None, Some(m)) => (m.resolve(s, ObjectKind::Mesh)?, None),
        _ => return Err(CmdError::Params("give `region` or `mesh`".into())),
    };
    let tol = i.tolerance.unwrap_or(0.2);
    if !(tol.is_finite() && tol > 0.0) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    let opts = FeatureOptions {
        angle_deg: i.angle.unwrap_or(35.0),
        min_triangles: i.min_triangles.unwrap_or(30).max(4),
        degree: i.degree.unwrap_or(5),
    };
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let tris: Vec<u32> = match region {
        Some(r) => s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?,
        None => (0..m.triangle_count() as u32).collect(),
    };
    if tris.len() < 20 {
        return Err(CmdError::Params("too few triangles (need 20)".into()));
    }
    let net = feature_network(&m, &tris, &opts).map_err(CmdError::Params)?;
    if net.nets.is_empty() {
        return Err(CmdError::Failed(format!(
            "no smooth region with at least three corners found ({} regions; {}): lower `angle` or raise `min_triangles`",
            net.regions,
            net.skipped.first().map_or("none skipped", String::as_str)
        )));
    }
    // compact corner ids
    let mut remap: BTreeMap<u32, u32> = BTreeMap::new();
    let corners: Vec<[u32; 4]> = net
        .corners
        .iter()
        .map(|c| {
            let mut o = [0u32; 4];
            for (k, id) in c.iter().enumerate() {
                let n = remap.len() as u32;
                o[k] = *remap.entry(*id).or_insert(n);
            }
            o
        })
        .collect();
    let sheet = sheet_from_bezier_network(&net.nets, &corners)?;
    // deviation of every region vertex from the patches
    let tess = tessellate_nets(&net.nets, 16);
    let bvh = Bvh::build(&tess);
    let (mut sq, mut max, mut count) = (0.0, 0.0f64, 0usize);
    for tr in &net.region_triangles {
        let mut seen: Vec<u32> = tr.iter().filter_map(|&t| m.triangles.get(t as usize)).flatten().copied().collect();
        seen.sort_unstable();
        seen.dedup();
        for v in seen {
            let Some(p) = m.positions.get(v as usize) else { continue };
            if let Some(h) = bvh.closest(&Point3::new(p.x, p.y, p.z), f64::INFINITY) {
                sq += h.distance * h.distance;
                max = max.max(h.distance);
                count += 1;
            }
        }
    }
    let rms = (sq / count.max(1) as f64).sqrt();
    let closed = is_closed(&sheet);
    let volume = if closed { surfacecraft_kernel::solid::volume(&sheet).ok().map(f64::abs) } else { None };
    let name = i.name.unwrap_or_else(|| "feature_network".into());
    let solid = if closed && i.make_solid.unwrap_or(true) {
        Some(s.doc.add_body(&format!("{name}_solid"), sheet.clone()))
    } else {
        None
    };
    let display = surfacecraft_kernel::tess::tessellate(&sheet, None).unwrap_or_default();
    let patches = sheet.faces.len();
    let surface = s.doc.add_surface(SurfaceObject {
        id: ObjectId(0),
        name,
        sheet: Arc::new(sheet),
        region,
        control: [opts.degree + 1, opts.degree + 1],
        degree: opts.degree,
        rms,
        max,
        display: Arc::new(display),
        parent: None,
        visible: true,
    });
    let within = max <= tol;
    let warning = if !within {
        Some(format!("max deviation {max:.4} mm exceeds {tol} mm: raise `degree` or check the edge angle"))
    } else if !net.skipped.is_empty() {
        Some(format!("{} region(s) skipped (not four-sided)", net.skipped.len()))
    } else {
        None
    };
    Ok(FromCurvesOut {
        surface,
        regions: net.regions,
        patches,
        curves: net.curves,
        feature_length: net.feature_length,
        skipped: net.skipped,
        rms,
        max,
        within_tolerance: within,
        closed,
        volume,
        solid,
        warning,
    })
}

/// `surface.from_curves`.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "surface.from_curves",
    "Patches from feature curves",
    "Freeform",
    true,
    "Cuts `region`/`mesh` at its sharp edges (dihedral angle > `angle`) and fits one Bézier patch to every four-sided smooth region, bounded by the shared feature curves; when a region has 3, 5, 6 or more corners every region is split (n corners: n patches around a centre, curves cut at their midpoints) so the network stays watertight; a closed network also makes a solid.",
    FromCurvesIn,
    FromCurvesOut,
    from_curves
)];
