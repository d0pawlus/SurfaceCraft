//! `align.*`: putting a scan into a coordinate system (design doc §6.1, milestone M1).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_align::{Datum, IcpOptions, datum_321, icp_cancellable, kabsch, principal_frame, rms_to_surface};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::{Bvh, Mesh, Point3, Transform, Vec3};

mod auto;
mod global;
mod pair;
mod settle;

/// The motion an alignment applied.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AlignOut {
    /// Object moved.
    pub id: ObjectId,
    /// Row-major 4×4 matrix applied (model → aligned).
    pub matrix: [[f64; 4]; 4],
    /// Rotation angle of the motion, degrees.
    pub angle_degrees: f64,
    /// Translation length, mm.
    pub translation_mm: f64,
    /// Residual (RMS, mm) where the method has one.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub rms: Option<f64>,
    /// Was it applied (false when `apply` was false)?
    pub applied: bool,
}

fn apply_to(s: &mut Session, id: ObjectId, also: &[ObjRef], t: &Transform, apply: bool) -> CmdResult<()> {
    if !apply {
        s.mark_unchanged();
        return Ok(());
    }
    let mut ids = vec![id];
    for o in also {
        ids.push(o.resolve(s, ObjectKind::Mesh)?);
    }
    // a mesh named twice (or the object itself in `also`) moves once
    ids.sort_unstable();
    ids.dedup();
    for i in ids {
        let m = s.doc.mesh_mut(i).ok_or_else(|| CmdError::Params(format!("no mesh {i}")))?;
        m.transform_rigid(t);
    }
    Ok(())
}

fn out(id: ObjectId, t: &Transform, rms: Option<f64>, applied: bool) -> AlignOut {
    AlignOut {
        id,
        matrix: t.to_rows(),
        angle_degrees: t.angle().to_degrees(),
        translation_mm: t.translation.norm(),
        rms,
        applied,
    }
}

fn yes() -> bool {
    true
}

/// N-point alignment parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NPointIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// Points on the mesh (e.g. picked in the viewport).
    #[schemars(extend("x-pick" = "points"))]
    pub from: Vec<[f64; 3]>,
    /// Where each point should go (same count, at least 3).
    #[schemars(extend("x-pick" = "points"))]
    pub to: Vec<[f64; 3]>,
    /// Move the mesh (default true); false only computes.
    #[serde(default = "yes")]
    pub apply: bool,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
}

fn n_point(s: &mut Session, i: NPointIn) -> CmdResult<AlignOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    if i.from.len() != i.to.len() {
        return Err(CmdError::Params(format!("{} `from` points but {} `to` points", i.from.len(), i.to.len())));
    }
    let from: Vec<Point3> = i.from.iter().map(|p| Point3::from(*p)).collect();
    let to: Vec<Point3> = i.to.iter().map(|p| Point3::from(*p)).collect();
    let t = kabsch(&from, &to, None)?;
    let rms = surfacecraft_align::kabsch::rms(&t, &from, &to);
    apply_to(s, id, &i.also, &t, i.apply)?;
    Ok(out(id, &t, Some(rms), i.apply))
}

/// ICP parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct IcpIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// Mesh or body to align to.
    pub target: ObjRef,
    /// Solver settings (defaults: 100 iterations, tolerance 1e-4 mm).
    #[serde(default)]
    pub options: IcpOptions,
    /// Move the mesh (default true).
    #[serde(default = "yes")]
    pub apply: bool,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
    /// Region of `object` that alone drives the fit (a local fine fit); the motion still moves
    /// all of `object`. Give `region` or `triangles`, not both.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Triangles of `object` that alone drive the fit, as an alternative to `region`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
}

/// ICP result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct IcpOut {
    /// The motion.
    pub align: AlignOut,
    /// Iterations run.
    pub iterations: usize,
    /// Converged below the tolerance?
    pub converged: bool,
    /// Last per-iteration change, mm.
    pub last_delta: f64,
    /// Pairs used in the last iteration.
    pub pairs: usize,
    /// Points the fit used: the vertices of the marked part (of the whole object without a
    /// selection), evenly strided down to `options.max_samples`.
    pub points_used: usize,
    /// RMS distance (mm) of the marked part to the target before the motion.
    pub rms_before_marked: f64,
    /// RMS distance (mm) of the marked part to the target after the motion.
    pub rms_after_marked: f64,
    /// RMS distance (mm) of the whole object to the target before the motion.
    pub rms_before_whole: f64,
    /// RMS distance (mm) of the whole object to the target after the motion.
    pub rms_after_whole: f64,
}

fn target_mesh(s: &mut Session, r: &ObjRef) -> CmdResult<Arc<Mesh>> {
    let (id, kind) = r.resolve_any(s)?;
    match kind {
        ObjectKind::Mesh => {
            s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))
        }
        ObjectKind::Body => {
            let b = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
            let m = surfacecraft_kernel::tess::tessellate(&b.brep, None)?;
            Ok(Arc::new(m))
        }
        ObjectKind::Primitive => {
            s.doc.primitive(id).map(|p| p.patch.clone()).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))
        }
        ObjectKind::Surface => {
            s.doc.surface(id).map(|o| o.display.clone()).ok_or_else(|| CmdError::Params(format!("no surface {id}")))
        }
        ObjectKind::Quad => Ok(Arc::new(s.doc.quad(id).map(|q| q.display_mesh()).unwrap_or_default())),
        ObjectKind::Region | ObjectKind::Sketch | ObjectKind::Feature => {
            Err(CmdError::Params(format!("{id} is not geometry to align to; use a mesh, body or primitive")))
        }
    }
}

/// The triangles of `mesh` (`count` triangles) that drive a local fit, from `region` or
/// `triangles`; `None` for a whole-object fit. Invalid selections are parameter errors.
fn selection(
    s: &Session,
    mesh: ObjectId,
    region: Option<&ObjRef>,
    triangles: Option<&[u32]>,
    count: usize,
) -> CmdResult<Option<Vec<u32>>> {
    let tris: Vec<u32> = match (region, triangles) {
        (None, None) => return Ok(None),
        (Some(_), Some(_)) => return Err(CmdError::Params("give either `region` or `triangles`, not both".into())),
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            if reg.mesh != mesh {
                return Err(CmdError::Params(format!("region {rid} is not a region of the object being moved")));
            }
            (*reg.triangles).clone()
        }
        (None, Some(t)) => t.to_vec(),
    };
    if tris.is_empty() {
        return Err(CmdError::Params(
            "the marked part has no triangles; use align.n_point with point pairs instead".into(),
        ));
    }
    if let Some(t) = tris.iter().find(|&&t| t as usize >= count) {
        return Err(CmdError::Params(format!("triangle {t} is out of range (the object has {count})")));
    }
    Ok(Some(tris))
}

fn icp_cmd(s: &mut Session, i: IcpIn) -> CmdResult<IcpOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let target = target_mesh(s, &i.target)?;
    let source = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    if i.options.max_iterations > 10_000 {
        return Err(CmdError::Params("max_iterations must be at most 10000".into()));
    }
    let marked = selection(s, id, i.region.as_ref(), i.triangles.as_deref(), source.triangle_count())?;
    let local = marked.is_some();
    // the part whose vertices drive the fit: the marked triangles, or the whole object
    let part: Option<Mesh> = marked.as_ref().map(|tris| source.submesh(tris));
    let whole: &Mesh = &source;
    let fit_on: &Mesh = part.as_ref().unwrap_or(whole);
    if local && fit_on.vertex_count() < 6 {
        return Err(CmdError::Params(format!(
            "the marked part has {} points, at least 6 are needed for a local fit; use align.n_point with point pairs instead",
            fit_on.vertex_count()
        )));
    }
    let bvh = Bvh::build(&target);
    let rep = match icp_cancellable(fit_on, &bvh, &Transform::identity(), &i.options, &s.cancel) {
        Ok(rep) => rep,
        Err(surfacecraft_align::AlignError::Cancelled) => return Err(super::cancelled()),
        Err(e) if local => {
            return Err(CmdError::Failed(format!(
                "the local fit failed ({e}); use align.n_point with point pairs instead"
            )));
        }
        Err(e) => return Err(e.into()),
    };
    if local && !rep.converged {
        return Err(CmdError::Failed(format!(
            "the local fit did not converge in {} iterations (last change {:.3e} mm); use align.n_point with point pairs instead",
            rep.iterations, rep.last_delta
        )));
    }
    let identity = Transform::identity();
    let measure = |pts: &[Point3], t: &Transform| {
        rms_to_surface(&bvh, pts, t).ok_or_else(|| CmdError::Failed("there is nothing to measure the fit on".into()))
    };
    let rms_before_whole = measure(&whole.positions[..], &identity)?;
    let rms_after_whole = measure(&whole.positions[..], &rep.transform)?;
    let (rms_before_marked, rms_after_marked) = if local {
        (measure(&fit_on.positions[..], &identity)?, measure(&fit_on.positions[..], &rep.transform)?)
    } else {
        (rms_before_whole, rms_after_whole)
    };
    apply_to(s, id, &i.also, &rep.transform, i.apply)?;
    Ok(IcpOut {
        align: out(id, &rep.transform, Some(rep.rms), i.apply),
        iterations: rep.iterations,
        converged: rep.converged,
        last_delta: rep.last_delta,
        pairs: rep.pairs,
        points_used: rep.samples,
        rms_before_marked,
        rms_after_marked,
        rms_before_whole,
        rms_after_whole,
    })
}

/// A datum: given explicitly, or a plane fitted to triangles of a mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum DatumIn {
    /// `{"plane": {...}}`, `{"axis": {...}}` or `{"point": {...}}`.
    Explicit(Datum),
    /// A plane fitted to these triangles (of `mesh`, default the aligned object).
    PlaneFromTriangles {
        /// Triangle indices.
        plane_from_triangles: Vec<u32>,
        /// Mesh they belong to (default: the object being aligned).
        #[serde(default)]
        mesh: Option<ObjRef>,
    },
}

fn resolve_datum(s: &Session, d: &DatumIn, default_mesh: ObjectId) -> CmdResult<Datum> {
    match d {
        DatumIn::Explicit(x) => Ok(x.clone()),
        DatumIn::PlaneFromTriangles { plane_from_triangles, mesh } => {
            let mid = match mesh {
                Some(r) => r.resolve(s, ObjectKind::Mesh)?,
                None => default_mesh,
            };
            let m = s.doc.mesh(mid).ok_or_else(|| CmdError::Params(format!("no mesh {mid}")))?;
            let sub = m.mesh.submesh(plane_from_triangles);
            let orient: Vec3 =
                (0..sub.triangle_count()).filter_map(|t| sub.face_normal(t).map(|n| n * sub.face_area(t))).sum();
            let f = surfacecraft_fit::plane::fit_plane(&sub.positions, Some(&orient))
                .ok_or_else(|| CmdError::Params("the triangles do not define a plane".into()))?;
            Ok(Datum::Plane { point: f.point.into(), normal: f.normal.into() })
        }
    }
}

/// 3-2-1 parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DatumAlignIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// Fixes +Z and the first origin constraint.
    #[schemars(extend("x-pick" = "plane"))]
    pub primary: DatumIn,
    /// Fixes +X.
    #[schemars(extend("x-pick" = "axis"))]
    pub secondary: DatumIn,
    /// Completes the origin.
    #[schemars(extend("x-pick" = "point"))]
    pub tertiary: DatumIn,
    /// Move the mesh (default true).
    #[serde(default = "yes")]
    pub apply: bool,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
}

fn datum_cmd(s: &mut Session, i: DatumAlignIn) -> CmdResult<AlignOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let p = resolve_datum(s, &i.primary, id)?;
    let q = resolve_datum(s, &i.secondary, id)?;
    let r = resolve_datum(s, &i.tertiary, id)?;
    let t = datum_321(&p, &q, &r)?;
    apply_to(s, id, &i.also, &t, i.apply)?;
    Ok(out(id, &t, None, i.apply))
}

/// One mesh.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MeshAlignIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// Move the mesh (default true).
    #[serde(default = "yes")]
    pub apply: bool,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
}

fn principal(s: &mut Session, i: MeshAlignIn) -> CmdResult<AlignOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let t = principal_frame(&m)?;
    apply_to(s, id, &i.also, &t, i.apply)?;
    Ok(out(id, &t, None, i.apply))
}

fn center(s: &mut Session, i: MeshAlignIn) -> CmdResult<AlignOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let t = Transform::translation(-m.bbox().center().coords);
    apply_to(s, id, &i.also, &t, i.apply)?;
    Ok(out(id, &t, None, i.apply))
}

/// `align.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "align.n_point",
        "N-point align",
        "Align",
        true,
        "Moves `object` so points `from` land on `to` (≥ 3 pairs, least squares).",
        NPointIn,
        AlignOut,
        n_point
    ),
    command!(
        "align.icp",
        "Fine align (ICP)",
        "Align",
        true,
        "Point-to-plane ICP of `object` onto `target` (mesh or body); stops below `options.tolerance` mm/iteration. With `region` or `triangles`, only that part of `object` drives the fit and the motion moves all of `object`; a part that does not converge or has under 6 points is an error suggesting align.n_point.",
        IcpIn,
        IcpOut,
        icp_cmd
    ),
    command!(
        "align.datum",
        "3-2-1 align",
        "Align",
        true,
        "Aligns `object` by `primary` (+Z), `secondary` (+X), `tertiary` datums: planes, axes, points or planes fitted to triangles.",
        DatumAlignIn,
        AlignOut,
        datum_cmd
    ),
    command!(
        "align.principal",
        "Align to world axes",
        "Align",
        true,
        "Principal axes of `object` to X/Y/Z, centroid to the origin.",
        MeshAlignIn,
        AlignOut,
        principal
    ),
    command!(
        "align.center",
        "Center at origin",
        "Align",
        true,
        "Moves the bounding-box centre of `object` to the origin (large-coordinate offset).",
        MeshAlignIn,
        AlignOut,
        center
    ),
    command!(
        "align.auto",
        "Auto align",
        "Align",
        true,
        "Proposes a pose from the scan alone: the largest flat face (or an open rim standing for a missing one) on z = 0 with the part above, the longest side of the footprint on x; a turned part gets its axis on z. `apply` false (default) only reports the 4x4 matrix, how far it moves the part, the evidence and a confidence; `mode` all_faces levels all near-axis-aligned faces by area.",
        auto::AutoIn,
        auto::AutoOut,
        auto::auto
    ),
    command!(
        "align.symmetry",
        "Align to symmetry plane",
        "Align",
        true,
        "Puts the mirror plane of `object` on the world plane `plane` (yz = x 0, default; xz; xy) by the shortest turn and a slide along that axis. `apply` false (default) only reports the pose and the symmetry residual (RMS distance mesh to mirrored mesh).",
        settle::SettleIn,
        settle::SettleOut,
        settle::settle
    ),
    command!(
        "align.pair_coarse",
        "Align two scans",
        "Align",
        true,
        "Brings `object` onto the overlapping scan `target` (mesh or body) with no picked points: candidates from the principal axes (24 signed permutations) and from local shape descriptors with RANSAC (partial overlap, down to about half shared), scored robustly, refined by ICP. Reports the pose, the overlap shares, a verdict (`unique`, `ambiguous` with the near-tied alternatives, or `no_match`) and a confidence. `apply` false (default) only proposes; a pose that is not `unique` is applied only when `choose` names it.",
        pair::PairIn,
        pair::PairOut,
        pair::pair
    ),
    command!(
        "align.global",
        "Align many scans",
        "Align",
        true,
        "Brings 2 to 24 scans of one part into one common pose with no picked points: pair poses (as `align.pair_coarse`; only `unique` pairs with enough overlap count, `ambiguous` and `no_match` ones are no edge), a spanning tree from the fixed scan, then a relaxation of all poses together that spreads the error around loops. Reports per scan its pose and whether it was placed, per pair the verdict, overlap and residual, the loop closure error before and after, and a verdict (`aligned`, `partial`, `failed`) with issue codes. A scan held by a single pair that no loop confirms makes the verdict `partial` (`weakly_held`); scans that cannot be connected to the fixed one are listed as unplaced and are not moved. `apply` false (default) only proposes; `apply` moves the placed scans held by a loop in one undo step, and a weakly held one only when `accept_weak` lists it.",
        global::GlobalIn,
        global::GlobalOut,
        global::global
    ),
];
