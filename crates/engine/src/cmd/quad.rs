//! `quad.*` and `surface.auto`: structured quad layouts over a region and AutoSurface — one
//! NURBS patch per quad, stitched by construction (design doc §6.3, milestone M4).
//!
//! Decision (design doc §10, retopology): neither an Instant Meshes port nor an external
//! binary in M4. A layout is a structured quad grid over a height-field chart of the region
//! (the same parameterisation as Fit Surface), its vertices snapped to the scan. AutoSurface fits
//! one B-spline per crease-free block with one knot span per quad and splits it into its Bézier
//! patches (solidcore `decompose_to_bez`), so neighbouring patches are C² — G1 by construction —
//! and share their edges exactly. Creases split the layout into blocks fitted separately.
//! P4 adds the unstructured case: `quad.retopo` lays a field-aligned quad-dominant layout over a
//! wrapped or closed region (fit::retopo, after Instant Meshes), and `surface.auto` fits one
//! bicubic Bézier patch per quad with shared boundaries and tangent planes (fit::patch), splitting
//! the layout until the deviation is within tolerance. A closed layout becomes one watertight
//! shell, i.e. a solid.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
mod unstructured;
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, QuadObject, SurfaceObject};
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::resample::HeightField;
use surfacecraft_fit::retopo::{QuadMesh, improper_quads};
use unstructured::{
    QuadIn, RetopoIn, RetopoOut, auto_unstructured, check_layout, info_of, layout_of, layout_radius, projector,
    retopo_cmd, scan_for, split_unstructured,
};

use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};
use surfacecraft_kernel::nurbs::{SurfaceProbe, decompose, fit_grid, sheet_from_patches, sheet_from_spline};

/// Samples per quad along each direction when resampling a block for its fit.
const SAMPLES_PER_QUAD: usize = 6;
/// Largest layout (quads per direction).
const MAX_QUADS: usize = 128;

/// A layout as reported.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct QuadInfo {
    /// Id.
    pub id: ObjectId,
    /// Quads along u.
    pub nu: usize,
    /// Quads along v.
    pub nv: usize,
    /// Crease lines (u indices, v indices).
    pub creases: (Vec<usize>, Vec<usize>),
    /// Largest distance of a cage vertex from the scan after snapping (mm).
    pub snap_max: f64,
    /// Unstructured split: quads turned against the scan that were repaired.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub repaired_patches: Option<usize>,
    /// Unstructured split: quads still turned against the scan (should be 0).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flipped_patches: Option<usize>,
    /// Unstructured layouts: patches that could not be checked against the scan orientation (reported, not assumed fine).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unverified_patches: Option<usize>,
    /// Unstructured layouts: boundary loops whose split vertices could not be put back onto the region outline.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_loops_unsnapped: Option<usize>,
    /// Unstructured split: why the first boundary loop was not snapped (nothing within reach, or a rollback).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_unsnapped_reason: Option<String>,
    /// Unstructured split: was the split rejected because it would have broken a proper quad (the layout is then unchanged)?
    #[serde(skip_serializing_if = "Option::is_none")]
    pub split_rejected: Option<bool>,
    /// Unstructured split: improper quads of the layout before the split.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub improper_before: Option<usize>,
    /// Unstructured split: improper quads of the layout after it (a rise means the layout got worse).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub improper_after: Option<usize>,
    /// Unstructured snap: was the projection rejected because it would have turned a proper quad improper (layout unchanged)?
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snap_rejected: Option<bool>,
}

fn points_of(s: &Session, mesh: ObjectId, region: Option<ObjectId>) -> CmdResult<(Arc<Mesh>, Vec<Point3>, Vec<Vec3>)> {
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let tris: Vec<u32> = match region {
        Some(r) => {
            s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params(format!("no region {r}")))?
        }
        None => (0..m.triangle_count() as u32).collect(),
    };
    let normals = m.vertex_normals();
    let (p, n) = region_points(&m, &normals, &tris);
    Ok((m, p, n))
}

fn field_of(s: &Session, q: &QuadObject) -> CmdResult<HeightField> {
    let (_, p, _) = points_of(s, q.mesh, q.region)?;
    let [o, u, v, n] = q.frame;
    HeightField::in_frame(&p, Point3::from(o), Vec3::from(u), Vec3::from(v), Vec3::from(n))
        .ok_or_else(|| CmdError::Failed("the region no longer covers the layout".into()))
}

/// Recomputes the cage vertices on the scan (height field) for the current `nu × nv`.
fn rebuild(s: &mut Session, id: ObjectId) -> CmdResult<QuadInfo> {
    let q = s.doc.quad(id).cloned().ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    let field = field_of(s, &q)?;
    let g = field
        .grid(q.domain, q.nu + 1, q.nv + 1)
        .ok_or_else(|| CmdError::Failed("the height field could not be evaluated".into()))?;
    let verts: Vec<[f64; 3]> = g.into_iter().flatten().collect();
    if let Some(m) = s.doc.quad_mut(id) {
        m.vertices = verts;
    }
    snap(s, id)
}

/// Projects every cage vertex onto the scan surface.
fn snap(s: &mut Session, id: ObjectId) -> CmdResult<QuadInfo> {
    let q = s.doc.quad(id).cloned().ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    let m = s.doc.mesh(q.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params("no mesh".into()))?;
    let bvh = Bvh::build(&m);
    let mut max: f64 = 0.0;
    let verts: Vec<[f64; 3]> = q
        .vertices
        .iter()
        .map(|v| {
            let p = Point3::from(*v);
            match bvh.closest(&p, f64::INFINITY) {
                Some(h) => {
                    max = max.max(h.distance);
                    [h.point.x, h.point.y, h.point.z]
                }
                None => *v,
            }
        })
        .collect();
    if let Some(mq) = s.doc.quad_mut(id) {
        mq.vertices = verts;
    }
    let q = s.doc.quad(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    Ok(QuadInfo {
        id,
        nu: q.nu,
        nv: q.nv,
        creases: (q.crease_u.clone(), q.crease_v.clone()),
        snap_max: max,
        repaired_patches: None,
        flipped_patches: None,
        unverified_patches: None,
        outline_loops_unsnapped: None,
        outline_unsnapped_reason: None,
        split_rejected: None,
        improper_before: None,
        improper_after: None,
        snap_rejected: None,
    })
}

/// Create parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateIn {
    /// Region to cover.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh to cover whole (when no region).
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Target number of quads (default 64).
    #[serde(default)]
    pub target_quads: Option<usize>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn create(s: &mut Session, i: CreateIn) -> CmdResult<QuadInfo> {
    let (mesh, region) = match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            (s.doc.region(rid).map(|r| r.mesh).ok_or_else(|| CmdError::Params("no region".into()))?, Some(rid))
        }
        (None, Some(m)) => (m.resolve(s, ObjectKind::Mesh)?, None),
        _ => return Err(CmdError::Params("give `region` or `mesh`".into())),
    };
    let target = i.target_quads.unwrap_or(64);
    if !(1..=MAX_QUADS * MAX_QUADS).contains(&target) {
        return Err(CmdError::Params(format!("target_quads must be 1..={}", MAX_QUADS * MAX_QUADS)));
    }
    let (_, p, n) = points_of(s, mesh, region)?;
    let field = HeightField::new(&p, &n).ok_or_else(|| {
        CmdError::Failed("the region is not one patch over a plane (it folds or is degenerate); split it".into())
    })?;
    let [[u0, u1], [v0, v1]] = field.domain;
    let aspect = ((u1 - u0) / (v1 - v0)).clamp(1e-3, 1e3);
    let nv = ((target as f64 / aspect).sqrt().round() as usize).clamp(1, MAX_QUADS);
    let nu = ((target as f64 / nv as f64).round() as usize).clamp(1, MAX_QUADS);
    let q = QuadObject {
        id: ObjectId(0),
        name: i.name.unwrap_or_else(|| "quads".into()),
        mesh,
        region,
        frame: [field.origin.into(), field.u.into(), field.v.into(), field.n.into()],
        domain: field.domain,
        nu,
        nv,
        vertices: Vec::new(),
        crease_u: Vec::new(),
        crease_v: Vec::new(),
        faces: Vec::new(),
        normals: Vec::new(),
        visible: true,
    };
    let id = s.doc.add_quad(q);
    rebuild(s, id)
}

fn split(s: &mut Session, i: QuadIn) -> CmdResult<QuadInfo> {
    let id = i.object.resolve(s, ObjectKind::Quad)?;
    if s.doc.quad(id).is_some_and(QuadObject::is_unstructured) {
        return split_unstructured(s, id, i.conform_boundary.unwrap_or(false));
    }
    let q = s.doc.quad_mut(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    if q.nu * 2 > MAX_QUADS || q.nv * 2 > MAX_QUADS {
        return Err(CmdError::Params(format!("the layout would exceed {MAX_QUADS} quads per direction")));
    }
    q.nu *= 2;
    q.nv *= 2;
    q.crease_u.iter_mut().for_each(|c| *c *= 2);
    q.crease_v.iter_mut().for_each(|c| *c *= 2);
    rebuild(s, id)
}

fn merge(s: &mut Session, i: QuadIn) -> CmdResult<QuadInfo> {
    let id = i.object.resolve(s, ObjectKind::Quad)?;
    if s.doc.quad(id).is_some_and(QuadObject::is_unstructured) {
        return Err(CmdError::Params(
            "an unstructured layout cannot be merged; run quad.retopo with fewer quads".into(),
        ));
    }
    let q = s.doc.quad_mut(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    if q.nu < 2 || q.nv < 2 || q.nu % 2 != 0 || q.nv % 2 != 0 {
        return Err(CmdError::Params("merging 2 × 2 quads needs an even number of quads ≥ 2 each way".into()));
    }
    q.nu /= 2;
    q.nv /= 2;
    q.crease_u.retain(|c| c % 2 == 0);
    q.crease_v.retain(|c| c % 2 == 0);
    q.crease_u.iter_mut().for_each(|c| *c /= 2);
    q.crease_v.iter_mut().for_each(|c| *c /= 2);
    rebuild(s, id)
}

fn snap_cmd(s: &mut Session, i: QuadIn) -> CmdResult<QuadInfo> {
    let id = i.object.resolve(s, ObjectKind::Quad)?;
    if let Some(q) = s.doc.quad(id).filter(|q| q.is_unstructured()).cloned() {
        check_layout(&q)?;
        let (pts, point_normals, spacing) = scan_for(s, &q)?;
        let scan = ScanSurface::oriented(&pts, &point_normals);
        let layout = layout_of(&q);
        let project = projector(&scan, layout_radius(&layout, spacing));
        let mut snap_max: f64 = 0.0;
        let (mut vs, mut ns) = (Vec::with_capacity(layout.vertices.len()), Vec::with_capacity(layout.vertices.len()));
        for (p, n) in layout.vertices.iter().zip(&layout.normals) {
            let (np, nn) = project(p, n).unwrap_or((*p, *n));
            snap_max = snap_max.max((np - p).norm());
            vs.push(np);
            ns.push(nn);
        }
        // a projection that would turn a proper quad improper (a collapse) is rejected as a whole
        let candidate = QuadMesh { vertices: vs, normals: ns, faces: layout.faces.clone() };
        let before = improper_quads(&layout);
        let breaks = improper_quads(&candidate).iter().zip(&before).any(|(&after, &was)| after && !was);
        if !breaks && let Some(m) = s.doc.quad_mut(id) {
            m.vertices = candidate.vertices.iter().map(|p| [p.x, p.y, p.z]).collect();
            m.normals = candidate.normals.iter().map(|n| [n.x, n.y, n.z]).collect();
        }
        return Ok(QuadInfo { snap_rejected: Some(breaks), ..info_of(&q, if breaks { 0.0 } else { snap_max }) });
    }
    snap(s, id)
}

/// Crease parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreaseIn {
    /// Quad layout id or name.
    pub object: ObjRef,
    /// Grid lines along v at these u indices (1..nu).
    #[serde(default)]
    pub u_lines: Vec<usize>,
    /// Grid lines along u at these v indices (1..nv).
    #[serde(default)]
    pub v_lines: Vec<usize>,
    /// Remove instead of add.
    #[serde(default)]
    pub remove: bool,
}

fn crease(s: &mut Session, i: CreaseIn) -> CmdResult<QuadInfo> {
    let id = i.object.resolve(s, ObjectKind::Quad)?;
    if s.doc.quad(id).is_some_and(QuadObject::is_unstructured) {
        return Err(CmdError::Params(
            "creases are lines of a structured layout; use surface.from_curves for sharp edges".into(),
        ));
    }
    let q = s.doc.quad_mut(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    if i.u_lines.iter().any(|&c| c == 0 || c >= q.nu) || i.v_lines.iter().any(|&c| c == 0 || c >= q.nv) {
        return Err(CmdError::Params(format!("crease lines must be interior: u in 1..{}, v in 1..{}", q.nu, q.nv)));
    }
    for (list, add) in [(&mut q.crease_u, &i.u_lines), (&mut q.crease_v, &i.v_lines)] {
        if i.remove {
            list.retain(|c| !add.contains(c));
        } else {
            list.extend(add.iter().copied());
        }
        list.sort_unstable();
        list.dedup();
    }
    let q = s.doc.quad(id).ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    Ok(QuadInfo {
        id,
        nu: q.nu,
        nv: q.nv,
        creases: (q.crease_u.clone(), q.crease_v.clone()),
        snap_max: 0.0,
        repaired_patches: None,
        flipped_patches: None,
        unverified_patches: None,
        outline_loops_unsnapped: None,
        outline_unsnapped_reason: None,
        split_rejected: None,
        improper_before: None,
        improper_after: None,
        snap_rejected: None,
    })
}

/// AutoSurface parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AutoIn {
    /// Quad layout id or name.
    pub object: ObjRef,
    /// Target maximum deviation (mm, default 0.2).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
    /// Unstructured layouts: also make a solid body when the patch network is closed (default true).
    #[serde(default)]
    pub make_solid: Option<bool>,
    /// Unstructured layouts: most patches the automatic refinement may reach (default 6000).
    #[serde(default)]
    pub max_patches: Option<usize>,
    /// Experimental: conform the layout of an open region to a smoothed outline of the region and put split boundary vertices back onto it (default false). Falls back to the plain layout when any quad would be improper.
    #[serde(default)]
    pub conform_boundary: Option<bool>,
}

/// One block's surface.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AutoBlock {
    /// Surface object of the block (its faces are the block's quads).
    pub id: ObjectId,
    /// Patches (faces).
    pub patches: usize,
    /// RMS deviation of the block's points (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
}

/// AutoSurface result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AutoOut {
    /// One surface per crease-free block.
    pub blocks: Vec<AutoBlock>,
    /// Largest deviation over every block (mm).
    pub max: f64,
    /// Within the target?
    pub within_tolerance: bool,
    /// Advice when the target was not met.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
    /// Unstructured layouts: largest angle between the normals of two patches across an edge (degrees).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub normal_jump_max_deg: Option<f64>,
    /// Unstructured layouts: mean of the per-edge normal jumps (degrees).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub normal_jump_mean_deg: Option<f64>,
    /// Unstructured layouts: is the patch network a closed shell?
    #[serde(skip_serializing_if = "Option::is_none")]
    pub closed: Option<bool>,
    /// Unstructured layouts: volume of the closed shell (mm³).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub volume: Option<f64>,
    /// Unstructured layouts: the solid body made from a closed network.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub solid: Option<ObjectId>,
    /// Unstructured layouts: quads of the final layout whose orientation is inverted against the scan normals.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub folded: Option<usize>,
    /// Unstructured layouts: flipped patches (normal against the scan) that were repaired.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub repaired_patches: Option<usize>,
    /// Unstructured layouts: flipped patches left in the result (a network with one is an error, so 0).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub flipped_patches: Option<usize>,
    /// Unstructured layouts: patches that could not be checked against the scan orientation (reported, not assumed fine).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unverified_patches: Option<usize>,
    /// Unstructured layouts: boundary loops whose split vertices could not be put back onto the region outline.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_loops_unsnapped: Option<usize>,
    /// Unstructured layouts: splits of the automatic refinement rejected because they would have broken a proper quad.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub splits_rejected: Option<usize>,
    /// Unstructured layouts: how many times the layout was split to reach the tolerance.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub refinements: Option<usize>,
    /// Unstructured layouts: share of scan points beyond the free boundary of the network (not measured).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub uncovered: Option<f64>,
}

fn spans(n: usize, creases: &[usize]) -> Vec<(usize, usize)> {
    let mut cuts: Vec<usize> = vec![0];
    cuts.extend(creases.iter().copied().filter(|&c| c > 0 && c < n));
    cuts.push(n);
    cuts.sort_unstable();
    cuts.dedup();
    cuts.windows(2).filter_map(|w| if let [a, b] = w { Some((*a, *b)) } else { None }).collect()
}

fn auto(s: &mut Session, i: AutoIn) -> CmdResult<AutoOut> {
    let id = i.object.resolve(s, ObjectKind::Quad)?;
    let q = s.doc.quad(id).cloned().ok_or_else(|| CmdError::Params(format!("no quad layout {id}")))?;
    let tol = i.tolerance.unwrap_or(0.2);
    if !(tol.is_finite() && tol > 0.0) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    if q.is_unstructured() {
        return auto_unstructured(s, &q, tol, &i);
    }
    let field = field_of(s, &q)?;
    let (_, pts, _) = points_of(s, q.mesh, q.region)?;
    let [o, uu, vv, _] = q.frame;
    let (o, uu, vv) = (Vec3::from(o), Vec3::from(uu), Vec3::from(vv));
    let degree = 3;
    let [[u0, u1], [v0, v1]] = q.domain;
    let du = (u1 - u0) / q.nu as f64;
    let dv = (v1 - v0) / q.nv as f64;
    let mut blocks = Vec::new();
    let mut worst: f64 = 0.0;
    let name = i.name.unwrap_or_else(|| "autosurface".into());
    for (ia, ib) in spans(q.nu, &q.crease_u) {
        for (ja, jb) in spans(q.nv, &q.crease_v) {
            let (bu, bv) = (ib - ia, jb - ja);
            let d = [[u0 + du * ia as f64, u0 + du * ib as f64], [v0 + dv * ja as f64, v0 + dv * jb as f64]];
            let g = field
                .grid(d, bu * SAMPLES_PER_QUAD + 1, bv * SAMPLES_PER_QUAD + 1)
                .ok_or_else(|| CmdError::Failed("the height field could not be evaluated".into()))?;
            let srf = fit_grid(&g, bu + degree, bv + degree, degree)?;
            let patches = decompose(&srf)?;
            let sheet = sheet_from_patches(&patches)?;
            let parent = sheet_from_spline(&srf)?;
            // deviation of the points inside this block
            let probe = SurfaceProbe::new(&srf)?;
            let mut sq = 0.0;
            let mut max: f64 = 0.0;
            let mut n = 0usize;
            let eps = 1e-9 * (1.0 + du.abs() + dv.abs());
            for p in &pts {
                let w = p.coords - o;
                let (x, y) = (w.dot(&uu), w.dot(&vv));
                if x < d[0][0] - eps || x > d[0][1] + eps || y < d[1][0] - eps || y > d[1][1] + eps {
                    continue;
                }
                let (dist, _) = probe.closest([p.x, p.y, p.z])?;
                sq += dist * dist;
                max = max.max(dist);
                n += 1;
            }
            let rms = (sq / n.max(1) as f64).sqrt();
            worst = worst.max(max);
            let display = surfacecraft_kernel::tess::tessellate(&sheet, None).unwrap_or_default();
            let faces = sheet.faces.len();
            let sid = s.doc.add_surface(SurfaceObject {
                id: ObjectId(0),
                name: name.clone(),
                sheet: Arc::new(sheet),
                region: q.region,
                control: [bu + degree, bv + degree],
                degree,
                rms,
                max,
                display: Arc::new(display),
                parent: Some(Arc::new(parent)),
                visible: true,
            });
            blocks.push(AutoBlock { id: sid, patches: faces, rms, max });
        }
    }
    let within = worst <= tol;
    let warning = (!within)
        .then(|| format!("max deviation {worst:.4} mm exceeds {tol} mm: split the layout (quad.split) and run again"));
    Ok(AutoOut {
        blocks,
        max: worst,
        within_tolerance: within,
        warning,
        normal_jump_max_deg: None,
        normal_jump_mean_deg: None,
        closed: None,
        volume: None,
        solid: None,
        refinements: None,
        uncovered: None,
        folded: None,
        repaired_patches: None,
        flipped_patches: None,
        unverified_patches: None,
        outline_loops_unsnapped: None,
        splits_rejected: None,
    })
}

/// `quad.*` and `surface.auto` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "quad.create",
        "Quad layout",
        "Freeform",
        true,
        "Structured quad layout (≈ `target_quads`) over `region` or `mesh`, snapped to the scan.",
        CreateIn,
        QuadInfo,
        create
    ),
    command!(
        "quad.split",
        "Split quads 4×",
        "Freeform",
        true,
        "Splits every quad of `object` into four.",
        QuadIn,
        QuadInfo,
        split
    ),
    command!(
        "quad.merge",
        "Merge quads 2×2",
        "Freeform",
        true,
        "Merges 2 × 2 quads of `object` into one.",
        QuadIn,
        QuadInfo,
        merge
    ),
    command!(
        "quad.retopo",
        "Retopologise",
        "Freeform",
        true,
        "Field-aligned quad-dominant layout (orientation + position field, Instant Meshes method) over a wrapped or closed `region`/`mesh`; faces other than quads force one 4-to-1 split.",
        RetopoIn,
        RetopoOut,
        retopo_cmd
    ),
    command!(
        "quad.snap",
        "Snap to mesh",
        "Freeform",
        true,
        "Projects the cage vertices of `object` onto the scan.",
        QuadIn,
        QuadInfo,
        snap_cmd
    ),
    command!(
        "quad.crease",
        "Crease",
        "Freeform",
        true,
        "Marks (or with `remove` clears) crease lines `u_lines`/`v_lines` of `object`: patches meet without smoothing there.",
        CreaseIn,
        QuadInfo,
        crease
    ),
    command!(
        "surface.auto",
        "AutoSurface",
        "Freeform",
        true,
        "One NURBS patch per quad of `object` (structured: C2 B-spline blocks; unstructured from quad.retopo: bicubic Bézier patches with shared edges and tangent planes, refined to `tolerance`; a closed network also makes a solid).",
        AutoIn,
        AutoOut,
        auto
    ),
];
