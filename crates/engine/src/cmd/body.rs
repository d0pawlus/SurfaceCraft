//! `body.*`: B-rep bodies through the kernel.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_kernel::{BodyInfo, solid};

/// Facts about a body.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct BodyOut {
    /// Object id.
    pub id: ObjectId,
    /// Faces.
    pub faces: usize,
    /// Edges.
    pub edges: usize,
    /// Vertices.
    pub vertices: usize,
    /// Volume, mm³ (absent for sheets).
    pub volume: Option<f64>,
}

pub(crate) fn body_out(s: &Session, id: ObjectId) -> CmdResult<BodyOut> {
    let b = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
    let BodyInfo { counts, volume } = surfacecraft_kernel::solid::info(&b.brep);
    Ok(BodyOut { id, faces: counts.faces, edges: counts.edges, vertices: counts.vertices, volume })
}

/// Box parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BoxIn {
    /// Lower corner (default origin).
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// Size along X, Y, Z.
    pub size: [f64; 3],
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn make_box(s: &mut Session, i: BoxIn) -> CmdResult<BodyOut> {
    let b = solid::make_box(i.origin.unwrap_or([0.0; 3]), i.size)?;
    let id = s.doc.add_body(i.name.as_deref().unwrap_or("box"), b);
    body_out(s, id)
}

/// Cylinder parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CylinderIn {
    /// Centre of the base (default origin); the axis is +Z.
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// Radius.
    pub radius: f64,
    /// Height.
    pub height: f64,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn make_cylinder(s: &mut Session, i: CylinderIn) -> CmdResult<BodyOut> {
    let b = solid::make_cylinder(i.origin.unwrap_or([0.0; 3]), i.radius, i.height)?;
    let id = s.doc.add_body(i.name.as_deref().unwrap_or("cylinder"), b);
    body_out(s, id)
}

/// Sphere parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SphereIn {
    /// Centre (default origin).
    #[serde(default)]
    pub center: Option<[f64; 3]>,
    /// Radius.
    pub radius: f64,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn make_sphere(s: &mut Session, i: SphereIn) -> CmdResult<BodyOut> {
    let b = solid::make_sphere(i.center.unwrap_or([0.0; 3]), i.radius)?;
    let id = s.doc.add_body(i.name.as_deref().unwrap_or("sphere"), b);
    body_out(s, id)
}

/// Blend parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BlendIn {
    /// Body id or name.
    #[schemars(extend("x-pick-body" = true))]
    pub object: ObjRef,
    /// Edge indices (as in the body record).
    #[schemars(extend("x-pick" = "edges"))]
    pub edges: Vec<usize>,
    /// Radius (fillet) or distance (chamfer), mm.
    pub size: f64,
}

fn fillet(s: &mut Session, i: BlendIn) -> CmdResult<BodyOut> {
    let id = i.object.resolve(s, ObjectKind::Body)?;
    let src = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
    let out = solid::fillet(&src.brep, &i.edges, i.size)?;
    if let Some(b) = s.doc.body_mut(id) {
        b.brep = Arc::new(out);
    }
    body_out(s, id)
}

fn chamfer(s: &mut Session, i: BlendIn) -> CmdResult<BodyOut> {
    let id = i.object.resolve(s, ObjectKind::Body)?;
    let src = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
    let out = solid::chamfer(&src.brep, &i.edges, i.size)?;
    if let Some(b) = s.doc.body_mut(id) {
        b.brep = Arc::new(out);
    }
    body_out(s, id)
}

/// One body.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BodyIn {
    /// Body id or name.
    pub object: ObjRef,
}

fn info(s: &mut Session, i: BodyIn) -> CmdResult<BodyOut> {
    let id = i.object.resolve(s, ObjectKind::Body)?;
    body_out(s, id)
}

/// Mesh-from-body parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ToMeshIn {
    /// Body id or name.
    pub object: ObjRef,
    /// Chord tolerance (mm, default from the body size).
    #[serde(default)]
    pub chord: Option<f64>,
    /// Mesh name.
    #[serde(default)]
    pub name: Option<String>,
}

fn to_mesh(s: &mut Session, i: ToMeshIn) -> CmdResult<crate::cmd::mesh::MeshInfo> {
    let id = i.object.resolve(s, ObjectKind::Body)?;
    let b = s.doc.body(id).ok_or_else(|| CmdError::Params(format!("no body {id}")))?;
    if let Some(c) = i.chord
        && !(c.is_finite() && c > 0.0)
    {
        return Err(CmdError::Params("chord must be positive".into()));
    }
    let m = surfacecraft_kernel::tess::tessellate(&b.brep, i.chord)?;
    let name = i.name.unwrap_or_else(|| format!("{} mesh", b.name));
    let mid = s.doc.add_mesh(&name, m);
    crate::cmd::mesh::mesh_info(s, mid)
}

/// `body.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "body.to_mesh",
        "Body to mesh",
        "Solid",
        true,
        "Tessellates body `object` into a new mesh (chord tolerance `chord`).",
        ToMeshIn,
        crate::cmd::mesh::MeshInfo,
        to_mesh
    ),
    command!("body.box", "Box", "Solid", true, "Creates a box body of `size` at `origin`.", BoxIn, BodyOut, make_box),
    command!(
        "body.cylinder",
        "Cylinder",
        "Solid",
        true,
        "Creates a cylinder (`radius`, `height`, +Z) at `origin`.",
        CylinderIn,
        BodyOut,
        make_cylinder
    ),
    command!(
        "body.sphere",
        "Sphere",
        "Solid",
        true,
        "Creates a sphere of `radius` at `center`.",
        SphereIn,
        BodyOut,
        make_sphere
    ),
    command!(
        "body.fillet",
        "Fillet",
        "Solid",
        true,
        "Rounds `edges` of `object` with radius `size`.",
        BlendIn,
        BodyOut,
        fillet
    ),
    command!(
        "body.chamfer",
        "Chamfer",
        "Solid",
        true,
        "Bevels `edges` of `object` by `size`.",
        BlendIn,
        BodyOut,
        chamfer
    ),
    command!("body.info", "Body info", "Solid", false, "Counts and volume of `object`.", BodyIn, BodyOut, info),
];
