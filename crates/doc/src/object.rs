//! Objects of a document.
//!
//! Every kind of object lives in its own list of the [`Document`]; the [`Object`] trait gives
//! them a common face (id, name, visibility, kind, a one-line detail) and the `stores!` macro
//! runs generic operations (find, rename, remove, list) over every list, so adding a kind is
//! one struct, one list and one line in the macro.
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_fit::{Primitive, PrimitiveKind};
use surfacecraft_geom::{Aabb, Mesh, Point3};
use surfacecraft_kernel::record::brep::Brep;

/// A document-unique object id (shared by every kind of object).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct ObjectId(pub u64);

impl std::fmt::Display for ObjectId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#{}", self.0)
    }
}

/// What kind of object an id names.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ObjectKind {
    /// A triangle mesh (scan).
    Mesh,
    /// A B-rep body (solid or sheet).
    Body,
    /// A persistent set of triangles of a mesh (design doc §6.2).
    Region,
    /// A fitted analytic primitive.
    Primitive,
    /// A fitted freeform (NURBS) surface.
    Surface,
    /// A quad layout (retopology cage) over a mesh region.
    Quad,
    /// A 2-D sketch.
    Sketch,
    /// A parametric feature of the history.
    Feature,
}

/// Common face of every object.
pub trait Object {
    /// Id.
    fn id(&self) -> ObjectId;
    /// Name.
    fn name(&self) -> &str;
    /// Mutable name.
    fn name_mut(&mut self) -> &mut String;
    /// Shown?
    fn visible(&self) -> bool;
    /// Mutable visibility.
    fn visible_mut(&mut self) -> &mut bool;
    /// Kind.
    fn kind(&self) -> ObjectKind;
    /// Short facts for lists.
    fn detail(&self) -> String;
}

macro_rules! object_common {
    ($t:ty, $kind:expr) => {
        impl Object for $t {
            fn id(&self) -> ObjectId {
                self.id
            }
            fn name(&self) -> &str {
                &self.name
            }
            fn name_mut(&mut self) -> &mut String {
                &mut self.name
            }
            fn visible(&self) -> bool {
                self.visible
            }
            fn visible_mut(&mut self) -> &mut bool {
                &mut self.visible
            }
            fn kind(&self) -> ObjectKind {
                $kind
            }
            fn detail(&self) -> String {
                self.describe()
            }
        }
    };
}

mod mesh_object;
pub use mesh_object::MeshObject;

/// A B-rep body, held as the kernel's neutral record.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// Geometry.
    pub brep: Arc<Brep>,
    /// Shown in the viewport.
    #[serde(default = "yes")]
    pub visible: bool,
}

impl BodyObject {
    fn describe(&self) -> String {
        format!("{} faces, {} edges", self.brep.faces.len(), self.brep.edges.len())
    }
}
object_common!(BodyObject, ObjectKind::Body);

/// A persistent region: triangles of one mesh, optionally classified by a fitted primitive.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RegionObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// The mesh the triangles belong to.
    pub mesh: ObjectId,
    /// Triangle indices, sorted and unique.
    pub triangles: Arc<Vec<u32>>,
    /// Geometry type after classification.
    #[serde(default)]
    pub kind: Option<PrimitiveKind>,
    /// RMS of the classification fit (mm).
    #[serde(default)]
    pub rms: Option<f64>,
    /// Shown in the viewport (as a colour on its mesh).
    #[serde(default = "yes")]
    pub visible: bool,
}

impl RegionObject {
    fn describe(&self) -> String {
        let k = self.kind.map_or_else(|| "unclassified".to_string(), |k| format!("{k:?}").to_lowercase());
        format!("{} triangles of {}, {k}", self.triangles.len(), self.mesh)
    }
}
object_common!(RegionObject, ObjectKind::Region);

/// A fitted primitive.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrimitiveObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// The surface.
    pub primitive: Primitive,
    /// Region it was fitted to, if any.
    #[serde(default)]
    pub region: Option<ObjectId>,
    /// RMS of the fit (mm).
    pub rms: f64,
    /// Largest deviation of the fit (mm).
    pub max: f64,
    /// Display patch (the part of the surface the fitted points cover).
    #[serde(with = "crate::format::mesh_b64")]
    pub patch: Arc<Mesh>,
    /// Shown in the viewport.
    #[serde(default = "yes")]
    pub visible: bool,
}

impl PrimitiveObject {
    fn describe(&self) -> String {
        format!("{:?}, rms {:.4} mm", self.primitive.kind(), self.rms).to_lowercase()
    }
}
object_common!(PrimitiveObject, ObjectKind::Primitive);

/// A fitted freeform surface: a spline sheet body (record), with its fit statistics.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SurfaceObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// The sheet: one face on the spline surface.
    pub sheet: Arc<Brep>,
    /// Region it was fitted to, if any.
    #[serde(default)]
    pub region: Option<ObjectId>,
    /// Control net size `[u, v]` and degree.
    pub control: [usize; 2],
    /// Degree.
    pub degree: usize,
    /// RMS deviation of the fitted points (mm).
    pub rms: f64,
    /// Largest deviation of the fitted points (mm).
    pub max: f64,
    /// For a patch network fitted from one spline: that spline as a one-face sheet (same
    /// geometry; used where the kernel needs one face, e.g. thicken).
    #[serde(default)]
    pub parent: Option<Arc<Brep>>,
    /// Display mesh.
    #[serde(with = "crate::format::mesh_b64")]
    pub display: Arc<Mesh>,
    /// Shown in the viewport.
    #[serde(default = "yes")]
    pub visible: bool,
}

impl SurfaceObject {
    fn describe(&self) -> String {
        format!("spline {}×{} deg {}, max {:.4} mm", self.control[0], self.control[1], self.degree, self.max)
    }
}
object_common!(SurfaceObject, ObjectKind::Surface);

/// A structured quad layout over a region (retopology cage, design doc §6.3): `(nu + 1) × (nv + 1)`
/// vertices snapped to the scan, on a plane frame the region is a height field over.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct QuadObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// Scan mesh.
    pub mesh: ObjectId,
    /// Region the layout covers (none = the whole mesh).
    #[serde(default)]
    pub region: Option<ObjectId>,
    /// Plane frame: origin, u, v, normal.
    pub frame: [[f64; 3]; 4],
    /// `[[u0, u1], [v0, v1]]` in plane coordinates.
    pub domain: [[f64; 2]; 2],
    /// Quads along u.
    pub nu: usize,
    /// Quads along v.
    pub nv: usize,
    /// Vertices, `vertices[i * (nv + 1) + j]`.
    pub vertices: Vec<[f64; 3]>,
    /// Crease lines along v at these u indices (1..nu): patches are not joined smoothly there.
    #[serde(default)]
    pub crease_u: Vec<usize>,
    /// Crease lines along u at these v indices (1..nv).
    #[serde(default)]
    pub crease_v: Vec<usize>,
    /// Unstructured layout (P4): quads as vertex ids into `vertices`, counter-clockwise seen
    /// from the side `normals` point to. Empty for a structured `nu × nv` layout.
    #[serde(default)]
    pub faces: Vec<[u32; 4]>,
    /// Unit normals of the vertices of an unstructured layout.
    #[serde(default)]
    pub normals: Vec<[f64; 3]>,
    /// Shown in the viewport.
    #[serde(default = "yes")]
    pub visible: bool,
}

impl QuadObject {
    fn describe(&self) -> String {
        if self.is_unstructured() {
            return format!("{} quads (unstructured), {} vertices", self.faces.len(), self.vertices.len());
        }
        format!("{} × {} quads, {} creases", self.nu, self.nv, self.crease_u.len() + self.crease_v.len())
    }

    /// Is this an unstructured (field-aligned) layout rather than a grid?
    #[must_use]
    pub fn is_unstructured(&self) -> bool {
        !self.faces.is_empty()
    }

    /// Display mesh: two triangles per quad.
    #[must_use]
    pub fn display_mesh(&self) -> Mesh {
        if self.is_unstructured() {
            let mut triangles = Vec::with_capacity(self.faces.len() * 2);
            for f in &self.faces {
                triangles.push([f[0], f[1], f[2]]);
                triangles.push([f[0], f[2], f[3]]);
            }
            let positions: Vec<Point3> = self.vertices.iter().map(|p| Point3::from(*p)).collect();
            let n = positions.len() as u32;
            triangles.retain(|t| t.iter().all(|&i| i < n));
            return Mesh { positions, triangles };
        }
        let w = (self.nv + 1) as u32;
        let mut triangles = Vec::with_capacity(self.nu * self.nv * 2);
        for i in 0..self.nu as u32 {
            for j in 0..self.nv as u32 {
                let a = i * w + j;
                triangles.push([a, a + w, a + w + 1]);
                triangles.push([a, a + w + 1, a + 1]);
            }
        }
        Mesh { positions: self.vertices.iter().map(|p| Point3::from(*p)).collect(), triangles }
    }
}
object_common!(QuadObject, ObjectKind::Quad);

impl crate::feature::SketchObject {
    fn describe(&self) -> String {
        let st = self.report.as_ref().map_or_else(|| "not solved".to_string(), |r| r.message.clone());
        format!("{} entities, {} constraints, {st}", self.sketch.entities.len(), self.sketch.constraints.len())
    }
}
object_common!(crate::feature::SketchObject, ObjectKind::Sketch);

impl crate::feature::FeatureObject {
    fn describe(&self) -> String {
        let state = if self.suppressed {
            " (suppressed)".to_string()
        } else {
            self.error.as_ref().map_or_else(String::new, |e| format!(" — error: {e}"))
        };
        format!("{} → {}{state}", self.op.label(), self.body)
    }
}
object_common!(crate::feature::FeatureObject, ObjectKind::Feature);

pub(crate) fn yes_fn() -> bool {
    true
}

fn yes() -> bool {
    true
}

/// A one-line description of an object, for lists, the UI tree and MCP.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ObjectSummary {
    /// Id.
    pub id: ObjectId,
    /// Kind.
    pub kind: ObjectKind,
    /// Name.
    pub name: String,
    /// Shown?
    pub visible: bool,
    /// Short facts (triangle count, face count…).
    pub detail: String,
}

/// The whole model.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Document {
    /// Meshes.
    #[serde(default)]
    pub meshes: Vec<MeshObject>,
    /// B-rep bodies.
    #[serde(default)]
    pub bodies: Vec<BodyObject>,
    /// Regions.
    #[serde(default)]
    pub regions: Vec<RegionObject>,
    /// Primitives.
    #[serde(default)]
    pub primitives: Vec<PrimitiveObject>,
    /// Freeform surfaces (sheets).
    #[serde(default)]
    pub surfaces: Vec<SurfaceObject>,
    /// Quad layouts.
    #[serde(default)]
    pub quads: Vec<QuadObject>,
    /// Sketches.
    #[serde(default)]
    pub sketches: Vec<crate::feature::SketchObject>,
    /// Features, in history order.
    #[serde(default)]
    pub features: Vec<crate::feature::FeatureObject>,
    /// Next free id.
    #[serde(default)]
    pub next_id: u64,
}

/// Runs `$body` with `$s` bound to each object list (immutable or mutable).
macro_rules! stores {
    ($doc:expr, $s:ident => $body:expr) => {{
        {
            let $s = &$doc.meshes;
            $body;
        }
        {
            let $s = &$doc.bodies;
            $body;
        }
        {
            let $s = &$doc.regions;
            $body;
        }
        {
            let $s = &$doc.primitives;
            $body;
        }
        {
            let $s = &$doc.surfaces;
            $body;
        }
        {
            let $s = &$doc.quads;
            $body;
        }
        {
            let $s = &$doc.sketches;
            $body;
        }
        {
            let $s = &$doc.features;
            $body;
        }
    }};
    (mut $doc:expr, $s:ident => $body:expr) => {{
        {
            let $s = &mut $doc.meshes;
            $body;
        }
        {
            let $s = &mut $doc.bodies;
            $body;
        }
        {
            let $s = &mut $doc.regions;
            $body;
        }
        {
            let $s = &mut $doc.primitives;
            $body;
        }
        {
            let $s = &mut $doc.surfaces;
            $body;
        }
        {
            let $s = &mut $doc.quads;
            $body;
        }
        {
            let $s = &mut $doc.sketches;
            $body;
        }
        {
            let $s = &mut $doc.features;
            $body;
        }
    }};
}

impl Document {
    /// An empty document.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserves a fresh id.
    pub fn fresh_id(&mut self) -> ObjectId {
        self.next_id += 1;
        ObjectId(self.next_id)
    }

    /// A name not used by any object, built from `base`.
    #[must_use]
    pub fn unique_name(&self, base: &str) -> String {
        let base = if base.trim().is_empty() { "object" } else { base.trim() };
        if !self.name_taken(base) {
            return base.to_string();
        }
        (2..).map(|i| format!("{base} {i}")).find(|n| !self.name_taken(n)).unwrap_or_else(|| base.to_string())
    }

    fn name_taken(&self, n: &str) -> bool {
        let mut taken = false;
        stores!(self, s => taken |= s.iter().any(|o| o.name() == n));
        taken
    }

    /// Adds a mesh and returns its id.
    pub fn add_mesh(&mut self, name: &str, mesh: Mesh) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.meshes.push(MeshObject { id, name, mesh: Arc::new(mesh), visible: true, working: None });
        id
    }

    /// Adds a body and returns its id.
    pub fn add_body(&mut self, name: &str, brep: Brep) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.bodies.push(BodyObject { id, name, brep: Arc::new(brep), visible: true });
        id
    }

    /// Adds a region (triangles sorted and deduplicated) and returns its id.
    pub fn add_region(&mut self, name: &str, mesh: ObjectId, mut triangles: Vec<u32>) -> ObjectId {
        triangles.sort_unstable();
        triangles.dedup();
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.regions.push(RegionObject {
            id,
            name,
            mesh,
            triangles: Arc::new(triangles),
            kind: None,
            rms: None,
            visible: true,
        });
        id
    }

    /// Adds a primitive and returns its id.
    pub fn add_primitive(
        &mut self,
        name: &str,
        primitive: Primitive,
        region: Option<ObjectId>,
        rms: f64,
        max: f64,
        patch: Mesh,
    ) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.primitives.push(PrimitiveObject {
            id,
            name,
            primitive,
            region,
            rms,
            max,
            patch: Arc::new(patch),
            visible: true,
        });
        id
    }

    /// Adds a sketch and returns its id.
    pub fn add_sketch(&mut self, name: &str, sketch: surfacecraft_sketch::Sketch) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.sketches.push(crate::feature::SketchObject { id, name, sketch, report: None, visible: true });
        id
    }

    /// Appends a feature writing `body` and returns its id.
    pub fn add_feature(&mut self, name: &str, op: crate::feature::FeatureOp, body: ObjectId) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.features.push(crate::feature::FeatureObject {
            id,
            name,
            op,
            body,
            suppressed: false,
            error: None,
            visible: true,
        });
        id
    }

    /// Adds a quad layout and returns its id.
    pub fn add_quad(&mut self, mut q: QuadObject) -> ObjectId {
        q.id = self.fresh_id();
        q.name = self.unique_name(&q.name);
        let id = q.id;
        self.quads.push(q);
        id
    }

    /// Adds a surface and returns its id.
    pub fn add_surface(&mut self, mut s: SurfaceObject) -> ObjectId {
        s.id = self.fresh_id();
        s.name = self.unique_name(&s.name);
        let id = s.id;
        self.surfaces.push(s);
        id
    }

    /// The mesh with `id`.
    #[must_use]
    pub fn mesh(&self, id: ObjectId) -> Option<&MeshObject> {
        self.meshes.iter().find(|m| m.id == id)
    }

    /// Mutable mesh with `id`.
    pub fn mesh_mut(&mut self, id: ObjectId) -> Option<&mut MeshObject> {
        self.meshes.iter_mut().find(|m| m.id == id)
    }

    /// The body with `id`.
    #[must_use]
    pub fn body(&self, id: ObjectId) -> Option<&BodyObject> {
        self.bodies.iter().find(|b| b.id == id)
    }

    /// Mutable body with `id`.
    pub fn body_mut(&mut self, id: ObjectId) -> Option<&mut BodyObject> {
        self.bodies.iter_mut().find(|b| b.id == id)
    }

    /// The region with `id`.
    #[must_use]
    pub fn region(&self, id: ObjectId) -> Option<&RegionObject> {
        self.regions.iter().find(|r| r.id == id)
    }

    /// Mutable region with `id`.
    pub fn region_mut(&mut self, id: ObjectId) -> Option<&mut RegionObject> {
        self.regions.iter_mut().find(|r| r.id == id)
    }

    /// The sketch with `id`.
    #[must_use]
    pub fn sketch(&self, id: ObjectId) -> Option<&crate::feature::SketchObject> {
        self.sketches.iter().find(|s| s.id == id)
    }

    /// Mutable sketch with `id`.
    pub fn sketch_mut(&mut self, id: ObjectId) -> Option<&mut crate::feature::SketchObject> {
        self.sketches.iter_mut().find(|s| s.id == id)
    }

    /// The feature with `id`.
    #[must_use]
    pub fn feature(&self, id: ObjectId) -> Option<&crate::feature::FeatureObject> {
        self.features.iter().find(|f| f.id == id)
    }

    /// Mutable feature with `id`.
    pub fn feature_mut(&mut self, id: ObjectId) -> Option<&mut crate::feature::FeatureObject> {
        self.features.iter_mut().find(|f| f.id == id)
    }

    /// The quad layout with `id`.
    #[must_use]
    pub fn quad(&self, id: ObjectId) -> Option<&QuadObject> {
        self.quads.iter().find(|q| q.id == id)
    }

    /// Mutable quad layout with `id`.
    pub fn quad_mut(&mut self, id: ObjectId) -> Option<&mut QuadObject> {
        self.quads.iter_mut().find(|q| q.id == id)
    }

    /// The surface with `id`.
    #[must_use]
    pub fn surface(&self, id: ObjectId) -> Option<&SurfaceObject> {
        self.surfaces.iter().find(|s| s.id == id)
    }

    /// The primitive with `id`.
    #[must_use]
    pub fn primitive(&self, id: ObjectId) -> Option<&PrimitiveObject> {
        self.primitives.iter().find(|p| p.id == id)
    }

    /// Regions of a mesh.
    pub fn regions_of(&self, mesh: ObjectId) -> impl Iterator<Item = &RegionObject> {
        self.regions.iter().filter(move |r| r.mesh == mesh)
    }

    /// Finds an object by name.
    #[must_use]
    pub fn find_by_name(&self, name: &str) -> Option<(ObjectId, ObjectKind)> {
        let mut found = None;
        stores!(self, s => if found.is_none() { found = s.iter().find(|o| o.name() == name).map(|o| (o.id(), o.kind())); });
        found
    }

    /// The kind of object `id` names.
    #[must_use]
    pub fn kind_of(&self, id: ObjectId) -> Option<ObjectKind> {
        let mut found = None;
        stores!(self, s => if found.is_none() { found = s.iter().find(|o| o.id() == id).map(|o| o.kind()); });
        found
    }

    /// Removes an object of any kind (a mesh takes its regions with it); true when removed.
    pub fn remove(&mut self, id: ObjectId) -> bool {
        let before = self.summaries().len();
        stores!(mut self, s => s.retain(|o| o.id() != id));
        self.regions.retain(|r| r.mesh != id);
        self.quads.retain(|q| q.mesh != id);
        for p in &mut self.primitives {
            if p.region == Some(id) {
                p.region = None;
            }
        }
        for s in &mut self.surfaces {
            if s.region == Some(id) {
                s.region = None;
            }
        }
        before != self.summaries().len()
    }

    /// Replaces a mesh's geometry after an edit that changed its triangles: regions and quad
    /// layouts of the mesh refer to triangle indices, so they are removed (primitives and surfaces
    /// fitted to them keep their geometry). Returns how many regions went; false id → `None`.
    pub fn replace_mesh_topology(&mut self, id: ObjectId, mesh: Mesh) -> Option<usize> {
        let m = self.mesh_mut(id)?;
        m.mesh = Arc::new(mesh);
        let gone: Vec<ObjectId> = self.regions.iter().filter(|r| r.mesh == id).map(|r| r.id).collect();
        self.regions.retain(|r| r.mesh != id);
        self.quads.retain(|q| q.mesh != id);
        for p in &mut self.primitives {
            if p.region.is_some_and(|r| gone.contains(&r)) {
                p.region = None;
            }
        }
        for s in &mut self.surfaces {
            if s.region.is_some_and(|r| gone.contains(&r)) {
                s.region = None;
            }
        }
        Some(gone.len())
    }

    /// Renames an object; false when it does not exist.
    pub fn rename(&mut self, id: ObjectId, name: &str) -> bool {
        let name = self.unique_name(name);
        let mut done = false;
        stores!(mut self, s => if let Some(o) = s.iter_mut().find(|o| o.id() == id) { *o.name_mut() = name.clone(); done = true; });
        done
    }

    /// Shows or hides an object.
    pub fn set_visible(&mut self, id: ObjectId, visible: bool) -> bool {
        let mut done = false;
        stores!(mut self, s => if let Some(o) = s.iter_mut().find(|o| o.id() == id) { *o.visible_mut() = visible; done = true; });
        done
    }

    /// Every object, in creation order.
    #[must_use]
    pub fn summaries(&self) -> Vec<ObjectSummary> {
        let mut v: Vec<ObjectSummary> = Vec::new();
        stores!(self, s => v.extend(s.iter().map(|o| ObjectSummary {
            id: o.id(),
            kind: o.kind(),
            name: o.name().to_string(),
            visible: o.visible(),
            detail: o.detail(),
        })));
        v.sort_by_key(|s| s.id);
        v
    }

    /// Bounding box of every visible mesh, body and primitive patch.
    #[must_use]
    pub fn bbox(&self) -> Aabb {
        let mut b = Aabb::empty();
        let mut add = |mb: Aabb| {
            if !mb.is_empty() {
                b.grow(&mb.min);
                b.grow(&mb.max);
            }
        };
        for m in self.meshes.iter().filter(|m| m.visible) {
            add(m.mesh.bbox());
        }
        for s in self.surfaces.iter().filter(|s| s.visible) {
            add(s.display.bbox());
        }
        for p in self.primitives.iter().filter(|p| p.visible) {
            add(p.patch.bbox());
        }
        for body in self.bodies.iter().filter(|b| b.visible) {
            add(Aabb::from_points(&body.brep.vertices.iter().map(|v| Point3::from(v.point)).collect::<Vec<_>>()));
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_names_and_removal() {
        let mut d = Document::new();
        let a = d.add_mesh("scan", Mesh::default());
        let b = d.add_mesh("scan", Mesh::default());
        assert_ne!(a, b);
        assert_eq!(d.mesh(b).unwrap().name, "scan 2");
        assert_eq!(d.find_by_name("scan 2"), Some((b, ObjectKind::Mesh)));
        let r = d.add_region("region", b, vec![3, 1, 3]);
        assert_eq!(*d.region(r).unwrap().triangles, vec![1, 3]);
        assert!(d.remove(a));
        assert!(!d.remove(a));
        assert_eq!(d.summaries().len(), 2);
        assert!(d.rename(b, "part"));
        assert_eq!(d.kind_of(b), Some(ObjectKind::Mesh));
        assert_eq!(d.kind_of(r), Some(ObjectKind::Region));
        // removing the mesh removes its regions
        assert!(d.remove(b));
        assert!(d.summaries().is_empty());
    }
}
