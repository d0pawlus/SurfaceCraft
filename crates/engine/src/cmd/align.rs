//! `align.*`: putting a scan into a coordinate system (design doc §6.1, milestone M1).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_align::{Datum, IcpOptions, datum_321, icp, kabsch, principal_frame};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::{Bvh, Mesh, Point3, Transform, Vec3};

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
        return Ok(());
    }
    let mut ids = vec![id];
    for o in also {
        ids.push(o.resolve(s, ObjectKind::Mesh)?);
    }
    for i in ids {
        let m = s.doc.mesh_mut(i).ok_or_else(|| CmdError::Params(format!("no mesh {i}")))?;
        m.mesh = Arc::new(m.mesh.transformed(t));
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
    pub from: Vec<[f64; 3]>,
    /// Where each point should go (same count, at least 3).
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
        ObjectKind::Region => Err(CmdError::Params(format!("{id} is a region; align to a mesh, body or primitive"))),
    }
}

fn icp_cmd(s: &mut Session, i: IcpIn) -> CmdResult<IcpOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let target = target_mesh(s, &i.target)?;
    let source = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    if i.options.max_iterations > 10_000 {
        return Err(CmdError::Params("max_iterations must be at most 10000".into()));
    }
    let bvh = Bvh::build(&target);
    let rep = icp(&source, &bvh, &Transform::identity(), &i.options)?;
    apply_to(s, id, &i.also, &rep.transform, i.apply)?;
    Ok(IcpOut {
        align: out(id, &rep.transform, Some(rep.rms), i.apply),
        iterations: rep.iterations,
        converged: rep.converged,
        last_delta: rep.last_delta,
        pairs: rep.pairs,
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
    pub primary: DatumIn,
    /// Fixes +X.
    pub secondary: DatumIn,
    /// Completes the origin.
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
        "Point-to-plane ICP of `object` onto `target` (mesh or body); stops below `options.tolerance` mm/iteration.",
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
];
