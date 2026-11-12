//! What depends on a mesh when it is scaled: classified regions, fitted primitives and quad
//! layouts are scaled with it; fitted freeform surfaces cannot be, so their presence refuses
//! the scaling. Bodies are not linked to a mesh and stay as they are.
use crate::cmd::{CmdError, CmdResult};
use surfacecraft_doc::{Document, ObjectId};
use surfacecraft_fit::primitive::Primitive;
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// The objects tied to one mesh: regions of it, primitives and surfaces fitted to those regions,
/// quad layouts over it.
pub(super) struct Dependents {
    regions: Vec<ObjectId>,
    primitives: Vec<ObjectId>,
    surfaces: Vec<ObjectId>,
    quads: Vec<ObjectId>,
}

/// How many of each kind were scaled with the mesh.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Rescaled {
    pub regions: usize,
    pub primitives: usize,
    pub quads: usize,
}

fn point(p: [f64; 3], o: &Vec3, f: f64) -> [f64; 3] {
    (o + (Vec3::from(p) - o) * f).into()
}

pub(super) fn scaled_mesh(mesh: &Mesh, o: &Vec3, f: f64) -> Option<Mesh> {
    let positions: Vec<Point3> = mesh.positions.iter().map(|p| Point3::from(point((*p).into(), o, f))).collect();
    positions.iter().all(surfacecraft_geom::finite).then(|| Mesh { positions, triangles: mesh.triangles.clone() })
}

/// The primitive scaled about `o` by `f`: positions and radii scale, directions and angles do not.
fn scaled_primitive(p: &Primitive, o: &Vec3, f: f64) -> Primitive {
    match p {
        Primitive::Plane { point: q, normal } => Primitive::Plane { point: point(*q, o, f), normal: *normal },
        Primitive::Sphere { center, radius } => Primitive::Sphere { center: point(*center, o, f), radius: radius * f },
        Primitive::Cylinder { point: q, axis, radius } => {
            Primitive::Cylinder { point: point(*q, o, f), axis: *axis, radius: radius * f }
        }
        Primitive::Cone { apex, axis, half_angle } => {
            Primitive::Cone { apex: point(*apex, o, f), axis: *axis, half_angle: *half_angle }
        }
        Primitive::Torus { center, axis, major_radius, minor_radius } => Primitive::Torus {
            center: point(*center, o, f),
            axis: *axis,
            major_radius: major_radius * f,
            minor_radius: minor_radius * f,
        },
    }
}

impl Dependents {
    pub(super) fn of(doc: &Document, mesh: ObjectId) -> Self {
        let regions: Vec<ObjectId> = doc.regions.iter().filter(|r| r.mesh == mesh).map(|r| r.id).collect();
        let linked = |r: Option<ObjectId>| r.is_some_and(|r| regions.contains(&r));
        Self {
            primitives: doc.primitives.iter().filter(|p| linked(p.region)).map(|p| p.id).collect(),
            surfaces: doc.surfaces.iter().filter(|s| linked(s.region)).map(|s| s.id).collect(),
            quads: doc.quads.iter().filter(|q| q.mesh == mesh).map(|q| q.id).collect(),
            regions,
        }
    }

    /// Refuses when a dependent cannot be scaled (fitted surfaces), naming them.
    pub(super) fn check(&self, doc: &Document) -> CmdResult<()> {
        if self.surfaces.is_empty() {
            return Ok(());
        }
        let names: Vec<String> =
            self.surfaces.iter().filter_map(|&id| doc.surface(id).map(|s| format!("{} ({id})", s.name))).collect();
        Err(CmdError::Params(format!(
            "the surfaces {} were fitted to this mesh and cannot be rescaled: delete them and fit again after scaling",
            names.join(", ")
        )))
    }

    /// Scales every dependent about `o` by `f`.
    pub(super) fn rescale(&self, doc: &mut Document, o: &Vec3, f: f64) -> CmdResult<Rescaled> {
        let too_big = || CmdError::Params("the scaled mesh would not be finite".into());
        let mut done = Rescaled::default();
        for r in doc.regions.iter_mut().filter(|r| self.regions.contains(&r.id)) {
            r.rms = r.rms.map(|v| v * f);
            done.regions += 1;
        }
        for p in doc.primitives.iter_mut().filter(|p| self.primitives.contains(&p.id)) {
            p.primitive = scaled_primitive(&p.primitive, o, f);
            (p.rms, p.max) = (p.rms * f, p.max * f);
            p.patch = std::sync::Arc::new(scaled_mesh(&p.patch, o, f).ok_or_else(too_big)?);
            done.primitives += 1;
        }
        for q in doc.quads.iter_mut().filter(|q| self.quads.contains(&q.id)) {
            q.frame[0] = point(q.frame[0], o, f);
            q.domain = q.domain.map(|d| d.map(|v| v * f));
            q.vertices = q.vertices.iter().map(|v| point(*v, o, f)).collect();
            done.quads += 1;
        }
        Ok(done)
    }
}
