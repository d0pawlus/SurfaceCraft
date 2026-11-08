//! `primitive.*`: analytic primitives fitted to regions or triangles, and the symmetry plane.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::segment::region_points;
use surfacecraft_fit::{Primitive, PrimitiveKind, fit_auto, primitive::fit};
use surfacecraft_geom::Point3;

/// Which kind to fit.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema, PartialEq)]
#[serde(rename_all = "snake_case")]
pub enum KindChoice {
    /// Fit every kind and choose by residual.
    #[default]
    Auto,
    /// Plane.
    Plane,
    /// Sphere.
    Sphere,
    /// Cylinder.
    Cylinder,
    /// Cone.
    Cone,
    /// Torus.
    Torus,
}

/// Fit parameters: a region, or a mesh and triangles.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FitIn {
    /// Region to fit.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh`.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Kind (default auto).
    #[serde(default)]
    pub kind: KindChoice,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// A fitted primitive.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FitOut {
    /// New primitive object.
    pub id: ObjectId,
    /// The surface.
    pub primitive: Primitive,
    /// RMS (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Points fitted.
    pub count: usize,
    /// Kinds tried with their RMS (auto only).
    pub candidates: Vec<(PrimitiveKind, f64)>,
    /// Advice, e.g. "more suitable as a cylinder".
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    /// Warning when the RMS exceeds the default tolerance (design doc §11.1: 0.1 mm; warn above 0.05).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

fn fit_cmd(s: &mut Session, i: FitIn) -> CmdResult<FitOut> {
    let (mesh, tris, region) = match (&i.region, &i.mesh, &i.triangles) {
        (Some(r), None, None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            (reg.mesh, reg.triangles.to_vec(), Some(rid))
        }
        (None, Some(m), Some(t)) => (m.resolve(s, ObjectKind::Mesh)?, t.clone(), None),
        _ => return Err(CmdError::Params("give `region`, or `mesh` and `triangles`".into())),
    };
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    if let Some(t) = tris.iter().find(|&&t| t as usize >= m.triangle_count()) {
        return Err(CmdError::Params(format!("triangle {t} out of range")));
    }
    let normals = m.vertex_normals();
    let (p, n) = region_points(&m, &normals, &tris);
    if p.len() < 3 {
        return Err(CmdError::Params("too few points to fit".into()));
    }
    let kind = match i.kind {
        KindChoice::Auto => None,
        KindChoice::Plane => Some(PrimitiveKind::Plane),
        KindChoice::Sphere => Some(PrimitiveKind::Sphere),
        KindChoice::Cylinder => Some(PrimitiveKind::Cylinder),
        KindChoice::Cone => Some(PrimitiveKind::Cone),
        KindChoice::Torus => Some(PrimitiveKind::Torus),
    };
    let (best, candidates, note) = match kind {
        None => {
            let a = fit_auto(&p, &n).ok_or_else(|| CmdError::Failed("no primitive could be fitted".into()))?;
            (a.best, a.candidates, a.note)
        }
        Some(k) => {
            let f = fit(k, &p, &n).ok_or_else(|| CmdError::Failed(format!("failed to fit a {k:?} to these points")))?;
            (f, Vec::new(), None)
        }
    };
    let warning = (best.rms > 0.05).then(|| {
        format!(
            "RMS {:.4} mm is above {} mm",
            best.rms,
            if best.rms > 0.1 { "the 0.1 mm tolerance" } else { "the 0.05 mm warning level" }
        )
    });
    let patch = best.primitive.patch_mesh(&p, 32);
    let name = i.name.unwrap_or_else(|| format!("{:?}", best.primitive.kind()).to_lowercase());
    let id = s.doc.add_primitive(&name, best.primitive.clone(), region, best.rms, best.max, patch);
    if let Some(rid) = region
        && let Some(r) = s.doc.region_mut(rid)
    {
        r.kind = Some(best.primitive.kind());
        r.rms = Some(best.rms);
    }
    Ok(FitOut {
        id,
        primitive: best.primitive,
        rms: best.rms,
        max: best.max,
        count: best.count,
        candidates,
        note,
        warning,
    })
}

/// One primitive.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PrimIn {
    /// Primitive id or name.
    pub object: ObjRef,
}

/// Primitive facts.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PrimOut {
    /// Id.
    pub id: ObjectId,
    /// The surface.
    pub primitive: Primitive,
    /// RMS (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Region it came from.
    pub region: Option<ObjectId>,
}

fn info(s: &mut Session, i: PrimIn) -> CmdResult<PrimOut> {
    let id = i.object.resolve(s, ObjectKind::Primitive)?;
    let p = s.doc.primitive(id).ok_or_else(|| CmdError::Params(format!("no primitive {id}")))?;
    Ok(PrimOut { id, primitive: p.primitive.clone(), rms: p.rms, max: p.max, region: p.region })
}

/// Symmetry parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SymIn {
    /// Mesh id or name.
    pub mesh: ObjRef,
}

fn symmetry(s: &mut Session, i: SymIn) -> CmdResult<FitOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let sym = surfacecraft_align::symmetry_plane(&m)?;
    let prim = Primitive::Plane { point: sym.point, normal: sym.normal };
    // display: the plane clipped to the projection of the mesh's bounding box
    let b = m.bbox();
    let corners: Vec<Point3> = (0..8)
        .map(|k| {
            Point3::new(
                if k & 1 == 0 { b.min.x } else { b.max.x },
                if k & 2 == 0 { b.min.y } else { b.max.y },
                if k & 4 == 0 { b.min.z } else { b.max.z },
            )
        })
        .map(|c| prim.project(&c))
        .collect();
    let patch = prim.patch_mesh(&corners, 4);
    let id = s.doc.add_primitive("symmetry plane", prim.clone(), None, sym.rms, sym.rms, patch);
    Ok(FitOut {
        id,
        primitive: prim,
        rms: sym.rms,
        max: sym.rms,
        count: m.vertex_count(),
        candidates: Vec::new(),
        note: None,
        warning: None,
    })
}

/// `primitive.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "primitive.fit",
        "Fit primitive",
        "Primitive",
        true,
        "Fits a plane/sphere/cylinder/cone/torus (`kind`, default auto) to `region` or `mesh`+`triangles`.",
        FitIn,
        FitOut,
        fit_cmd
    ),
    command!(
        "primitive.info",
        "Primitive info",
        "Primitive",
        false,
        "Parameters and fit quality of `object`.",
        PrimIn,
        PrimOut,
        info
    ),
    command!(
        "primitive.symmetry",
        "Symmetry plane",
        "Primitive",
        true,
        "Fits the plane minimising the distance between `mesh` and its mirror image.",
        SymIn,
        FitOut,
        symmetry
    ),
];
