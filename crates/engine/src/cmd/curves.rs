//! `surface.from_curves`: a patch network bounded by the sharp-edge (feature) curves of the scan
//! (design doc §6.3 second freeform strategy, milestone P4) for housings with clear edges.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use ngon_patch::{NgonFit, Target, fit_regions};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, SurfaceObject};
use surfacecraft_fit::features::{FeatureNetwork, FeatureOptions, NgonMode, feature_network_with, tessellate_nets};
use surfacecraft_geom::{Bvh, Mesh, Point3};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network};

mod ngon_patch;
pub use ngon_patch::TrimmedPatchOut;

/// What to do with smooth regions that do not have four corners.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum NgonIn {
    /// When any region is not four-sided, every region is split into four-sided Bézier patches
    /// (the default).
    #[default]
    Split,
    /// Four-sided regions are Bézier patches of the network; every other region with one outline is
    /// one trimmed patch of its own (a separate surface object, no solid).
    Patch,
}

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
    /// Regions that are not four-sided: `split` (default) cuts every region into four-sided
    /// patches; `patch` keeps four-sided regions as Bézier patches and fits every other region
    /// with one outline as a single trimmed patch (separate surface objects, no solid).
    #[serde(default)]
    pub ngon: NgonIn,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// Result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FromCurvesOut {
    /// The patch network (one sheet of all patches); with `ngon: "patch"` and no four-sided
    /// region, the first trimmed patch.
    pub surface: ObjectId,
    /// Smooth regions the sharp edges cut the scan into.
    pub regions: usize,
    /// Bézier patches made (one per four-sided region; with `ngon: "split"` and any other region
    /// every region is split, see the summary).
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
    /// The regions fitted as single trimmed patches (`ngon: "patch"`), each its own surface object.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub trimmed_patches: Vec<TrimmedPatchOut>,
    /// Largest distance of a shared feature curve from a trimmed patch (mm); absent without one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub side_gap_max_mm: Option<f64>,
    /// Advice when the target was not met or regions were skipped.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

/// The Bézier network of the four-sided regions as one sheet.
struct Bezier {
    surface: ObjectId,
    patches: usize,
    /// Sum of the squared deviations of the region vertices, their number and the largest one.
    sq: f64,
    count: usize,
    max: f64,
    closed: bool,
    volume: Option<f64>,
    solid: Option<ObjectId>,
}

/// Where and how the Bézier network is added to the document.
struct BezierTarget<'a> {
    name: &'a str,
    region: Option<ObjectId>,
    degree: usize,
    make_solid: bool,
    /// Trimmed patches belong to the result: the network is reported open and no solid is made.
    open: bool,
}

/// The sheet of the Bézier patches of `net`, its deviation from the region vertices, its solid.
fn bezier_part(s: &mut Session, m: &Mesh, net: &FeatureNetwork, target: &BezierTarget<'_>) -> CmdResult<Bezier> {
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
    let closed = is_closed(&sheet) && !target.open;
    let volume = if closed { surfacecraft_kernel::solid::volume(&sheet).ok().map(f64::abs) } else { None };
    let solid = if closed && target.make_solid {
        Some(s.doc.add_body(&format!("{}_solid", target.name), sheet.clone()))
    } else {
        None
    };
    let display = surfacecraft_kernel::tess::tessellate(&sheet, None).unwrap_or_default();
    let patches = sheet.faces.len();
    let surface = s.doc.add_surface(SurfaceObject {
        id: ObjectId(0),
        name: target.name.to_string(),
        sheet: Arc::new(sheet),
        region: target.region,
        control: [target.degree + 1, target.degree + 1],
        degree: target.degree,
        rms,
        max,
        display: Arc::new(display),
        parent: None,
        visible: true,
    });
    Ok(Bezier { surface, patches, sq, count, max, closed, volume, solid })
}

/// The sentence added to the warning when regions were fitted as separate trimmed patches.
fn separate_sheets_note(count: usize) -> String {
    format!(
        "{count} region(s) were fitted as single trimmed patches; they are separate sheets (the kernel cannot sew separate sheets yet), so no solid is made"
    )
}

/// The mesh and the region (when given) the command works on.
fn source_mesh(s: &Session, i: &FromCurvesIn) -> CmdResult<(ObjectId, Option<ObjectId>)> {
    match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let mesh = s.doc.region(rid).map(|r| r.mesh).ok_or_else(|| CmdError::Params("no region".into()))?;
            Ok((mesh, Some(rid)))
        }
        (None, Some(m)) => Ok((m.resolve(s, ObjectKind::Mesh)?, None)),
        _ => Err(CmdError::Params("give `region` or `mesh`".into())),
    }
}

/// The notes of the result, joined with `; `: the deviation, skipped regions, unmeasured patches
/// and the separate sheets of trimmed patches.
fn result_warning(
    within: bool,
    max: f64,
    tol: f64,
    skipped: &[String],
    unmeasured: &[String],
    trimmed: usize,
) -> Option<String> {
    let mut notes: Vec<String> = Vec::new();
    if !within {
        notes.push(format!("max deviation {max:.4} mm exceeds {tol} mm: raise `degree` or check the edge angle"));
    } else if !skipped.is_empty() {
        notes.push(format!("{} region(s) skipped (see `skipped` for the reason)", skipped.len()));
    }
    if !unmeasured.is_empty() {
        notes.push(format!("not within tolerance, nothing was measured: {}", unmeasured.join("; ")));
    }
    if trimmed > 0 {
        notes.push(separate_sheets_note(trimmed));
    }
    (!notes.is_empty()).then(|| notes.join("; "))
}

/// What the command works on: the mesh, its region (when given), the options and the feature network.
struct Prepared {
    mesh: std::sync::Arc<Mesh>,
    region: Option<ObjectId>,
    opts: FeatureOptions,
    tol: f64,
    net: FeatureNetwork,
}

fn prepare(s: &Session, i: &FromCurvesIn) -> CmdResult<Prepared> {
    let (mesh, region) = source_mesh(s, i)?;
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
    let mode = match i.ngon {
        NgonIn::Split => NgonMode::Split,
        NgonIn::Patch => NgonMode::Patch,
    };
    let net = feature_network_with(&m, &tris, &opts, mode).map_err(CmdError::Params)?;
    if net.nets.is_empty() && net.ngons.is_empty() {
        return Err(CmdError::Failed(format!(
            "no smooth region with at least three corners found ({} regions; {}): lower `angle` or raise `min_triangles`",
            net.regions,
            net.skipped.first().map_or("none skipped", String::as_str)
        )));
    }
    Ok(Prepared { mesh: m, region, opts, tol, net })
}

fn from_curves(s: &mut Session, i: FromCurvesIn) -> CmdResult<FromCurvesOut> {
    let Prepared { mesh: m, region, opts, tol, net } = prepare(s, &i)?;
    let name = i.name.unwrap_or_else(|| "feature_network".into());
    let bezier = if net.nets.is_empty() {
        None
    } else {
        let target = BezierTarget {
            name: &name,
            region,
            degree: opts.degree,
            make_solid: i.make_solid.unwrap_or(true),
            open: !net.ngons.is_empty(),
        };
        Some(bezier_part(s, &m, &net, &target)?)
    };
    let NgonFit { patches: trimmed, skipped: ngon_skipped, unmeasured, sq: ngon_sq, max: ngon_max, count: ngon_count } =
        fit_regions(s, &net.ngons, &Target { name: &name, region, tolerance: tol })?;
    let Some(surface) = bezier.as_ref().map(|b| b.surface).or_else(|| trimmed.first().map(|p| p.surface)) else {
        let reasons: Vec<&str> = net.skipped.iter().chain(ngon_skipped.iter()).map(String::as_str).collect();
        return Err(CmdError::Failed(format!("no region could be turned into a patch: {}", reasons.join("; "))));
    };
    let (sq, max, count) = match &bezier {
        Some(b) => (b.sq + ngon_sq, b.max.max(ngon_max), b.count + ngon_count),
        None => (ngon_sq, ngon_max, ngon_count),
    };
    let rms = (sq / count.max(1) as f64).sqrt();
    let mut skipped = net.skipped;
    skipped.extend(ngon_skipped);
    let within = max <= tol;
    let warning = result_warning(within, max, tol, &skipped, &unmeasured, trimmed.len());
    let side_gap_max_mm = trimmed.iter().map(|p| p.side_gap_max_mm).reduce(f64::max);
    Ok(FromCurvesOut {
        surface,
        regions: net.regions,
        patches: bezier.as_ref().map_or(0, |b| b.patches),
        curves: net.curves,
        feature_length: net.feature_length,
        skipped,
        rms,
        max,
        within_tolerance: within,
        closed: bezier.as_ref().is_some_and(|b| b.closed),
        volume: bezier.as_ref().and_then(|b| b.volume),
        solid: bezier.as_ref().and_then(|b| b.solid),
        trimmed_patches: trimmed,
        side_gap_max_mm,
        warning,
    })
}

/// `surface.from_curves`.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "surface.from_curves",
    "Patches from feature curves",
    "Freeform",
    true,
    "Cuts `region`/`mesh` at its sharp edges (dihedral angle > `angle`) and fits one Bézier patch to every four-sided smooth region, bounded by the shared feature curves. A region with 3, 5 or more corners: with `ngon: \"split\"` (default) every region is split (n corners: n patches around a centre, curves cut at their midpoints) so the network stays watertight and a closed network also makes a solid; with `ngon: \"patch\"` the four-sided regions stay Bézier patches and every other region is fitted as one trimmed patch to its own feature curves (G0) and vertices, each a separate surface object `<name>_patch_<k>` (`trimmed_patches`), so no solid is made.",
    FromCurvesIn,
    FromCurvesOut,
    from_curves
)];
