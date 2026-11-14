//! Merging the patches of the wrapped scan that lie on a protected primitive into one analytic face
//! each (P9-O6b). Nothing is cut: the body keeps the wrap's topology outside the groups, so it is as
//! closed (or as open) as the wrap was.
//!
//! A patch belongs to a primitive when the scan triangle nearest its middle is in the primitive's
//! region and every sample on it lies within a margin of the primitive and of a triangle of the
//! region: a patch that straddles the border over a surface that leaves the primitive has a sample
//! off it, one that follows the border (a crease) is wholly inside. The patches of one primitive that touch along edges form a group, which
//! the kernel replaces by one face on the primitive (`merge_faces_onto_shape`); a group the kernel
//! cannot merge stays patches and is reported with the reason.
use super::hybrid_method::{LeftPatches, MergedFace};
use crate::Session;
use crate::cmd::{CmdError, cancelled};
use surfacecraft_doc::ObjectId;
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Bvh, Mesh, Point3, par};
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::merge::{
    Batch, GroupSpec, face_samples, grid_area, merge_faces_onto_shapes, pinch_free_groups,
};
use surfacecraft_kernel::record::brep::Brep;

/// Samples per side of a patch's grid (so `(n + 1)^2` points are tested).
const SAMPLES_PER_SIDE: usize = 5;
/// A sample must lie within this share of the tolerance of its primitive.
const SAMPLE_SHARE_OF_TOLERANCE: f64 = 1.0;
/// The boundary of a merged group may leave its primitive by this share of the tolerance.
const GAP_SHARE_OF_TOLERANCE: f64 = 1.0;
/// A sample must lie within this share of the tolerance of a triangle of the primitive's region.
const NEAR_REGION_SHARE_OF_TOLERANCE: f64 = 1.0;
/// How far from a sample the scan is looked for, in tolerances.
const SEARCH_TOLERANCES: f64 = 4.0;
/// A group of less area than this many squared tolerances stays patches (collapsed and sliver
/// patches lie on any surface and are no analytic face).
pub(super) const MIN_AREA_IN_TOLERANCES_SQUARED: f64 = 50.0;
/// A group of fewer patches than this stays patches (one patch is no gain).
const MIN_GROUP: usize = 2;
/// Owner of a triangle in no protected region.
const NONE: u32 = u32::MAX;

/// A protected primitive: what the segmentation found and fitted.
pub(super) struct Protected {
    pub primitive: ObjectId,
    pub kind: &'static str,
    pub shape: Shape,
    pub fit: Primitive,
    pub triangles: Vec<u32>,
    /// The area of the region of the scan (mm2).
    pub area: f64,
}

/// What the merging made of the body.
pub(super) struct Merging {
    pub body: Brep,
    pub merged: Vec<MergedFace>,
    pub left: Vec<LeftPatches>,
}

/// The region each scan triangle belongs to (`NONE` for none).
fn region_owners(scan: &Mesh, protected: &[Protected]) -> Vec<u32> {
    let mut owner = vec![NONE; scan.triangles.len()];
    for (k, p) in protected.iter().enumerate() {
        for &t in &p.triangles {
            if let Some(slot) = owner.get_mut(t as usize) {
                *slot = u32::try_from(k).unwrap_or(NONE);
            }
        }
    }
    owner
}

/// The triangles of one region as a mesh of their own (vertices renumbered).
fn region_mesh(scan: &Mesh, triangles: &[u32]) -> Mesh {
    let mut index: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
    let mut out = Mesh::default();
    for &t in triangles {
        let Some(tri) = scan.triangles.get(t as usize) else { continue };
        let mut mapped = [0u32; 3];
        for (slot, &v) in mapped.iter_mut().zip(tri) {
            let Some(p) = scan.positions.get(v as usize) else { continue };
            *slot = *index.entry(v).or_insert_with(|| {
                out.positions.push(*p);
                (out.positions.len() - 1) as u32
            });
        }
        out.triangles.push(mapped);
    }
    out
}

/// What the choice of a patch's primitive looks at.
struct Lookup<'a> {
    all: Bvh,
    owner: Vec<u32>,
    regions: Vec<Bvh>,
    protected: &'a [Protected],
    tol: f64,
}

impl<'a> Lookup<'a> {
    fn new(scan: &Mesh, protected: &'a [Protected], tol: f64) -> Self {
        let regions = protected.iter().map(|p| Bvh::build(&region_mesh(scan, &p.triangles))).collect();
        Self { all: Bvh::build(scan), owner: region_owners(scan, protected), regions, protected, tol }
    }

    /// The protected primitive patch `face` lies on and the area of the patch: the primitive whose
    /// region holds the scan triangle nearest the middle of the patch, if every sample of the patch
    /// lies within a margin of the primitive and of a triangle of its region.
    fn owner_of(&self, body: &Brep, face: usize) -> Option<(usize, f64)> {
        let samples = face_samples(body, face, SAMPLES_PER_SIDE)?;
        let middle = samples.get(samples.len() / 2)?;
        let mid = Point3::new(middle[0], middle[1], middle[2]);
        let hit = self.all.closest(&mid, SEARCH_TOLERANCES * self.tol)?;
        let k = usize::try_from(self.owner.get(hit.triangle).copied().filter(|&o| o != NONE)?).ok()?;
        let (prim, region) = (self.protected.get(k)?, self.regions.get(k)?);
        let near = NEAR_REGION_SHARE_OF_TOLERANCE * self.tol;
        let fits = |p: &[f64; 3]| {
            let q = Point3::new(p[0], p[1], p[2]);
            prim.fit.distance(&q).abs() <= SAMPLE_SHARE_OF_TOLERANCE * self.tol && region.closest(&q, near).is_some()
        };
        samples.iter().all(fits).then(|| (k, grid_area(&samples, SAMPLES_PER_SIDE)))
    }
}

fn left(p: &Protected, patches: usize, reason: impl Into<String>) -> LeftPatches {
    LeftPatches { primitive: p.primitive, kind: p.kind.to_string(), patches, reason: reason.into() }
}

/// The groups of patches that lie on each protected primitive, as `(primitive, faces)`, and the
/// groups too small to merge (recorded in `out`).
fn candidate_groups(
    body: &Brep,
    protected: &[Protected],
    owners: &[Option<(usize, f64)>],
    tol: f64,
    out: &mut Merging,
) -> Vec<(usize, Vec<usize>)> {
    let mut sized: Vec<(f64, usize, Vec<usize>)> = Vec::new();
    for (k, p) in protected.iter().enumerate() {
        let faces: Vec<usize> =
            (0..body.faces.len()).filter(|&f| owners.get(f).copied().flatten().is_some_and(|o| o.0 == k)).collect();
        for group in pinch_free_groups(body, &faces) {
            let area: f64 = group.iter().filter_map(|&f| owners.get(f).copied().flatten().map(|o| o.1)).sum();
            if group.len() < MIN_GROUP {
                out.left.push(left(p, group.len(), "a single patch"));
            } else if area < MIN_AREA_IN_TOLERANCES_SQUARED * tol * tol {
                out.left.push(left(p, group.len(), format!("an area of {area:.2} mm2 is too small to merge")));
            } else {
                sized.push((area, k, group));
            }
        }
    }
    // the largest first: the kernel takes a limited number of groups, and the first are the ones worth most
    sized.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.2.first().cmp(&b.2.first())));
    sized.into_iter().map(|(_, k, g)| (k, g)).collect()
}

/// Merges every group of patches of `body` that lies in a protected region, all groups in one
/// rebuild of the body where the kernel takes that.
pub(super) fn merge_groups(
    s: &Session,
    scan: &Mesh,
    body: &Brep,
    protected: &[Protected],
    tol: f64,
) -> Result<Merging, CmdError> {
    let lookup = Lookup::new(scan, protected, tol);
    let owners: Vec<Option<(usize, f64)>> = par::map(body.faces.len(), |f| lookup.owner_of(body, f));
    let mut out = Merging { body: body.clone(), merged: Vec::new(), left: Vec::new() };
    let candidates = candidate_groups(body, protected, &owners, tol, &mut out);
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    let specs: Vec<GroupSpec> = candidates
        .iter()
        .filter_map(|(k, g)| protected.get(*k).map(|p| GroupSpec { faces: g, shape: &p.shape }))
        .collect();
    if specs.is_empty() {
        return Ok(out);
    }
    match merge_faces_onto_shapes(body, &specs, GAP_SHARE_OF_TOLERANCE * tol, &|| s.cancel.is_cancelled()) {
        Ok(batch) => record(&mut out, protected, &candidates, batch),
        Err(e) => {
            if s.cancel.is_cancelled() {
                return Err(cancelled());
            }
            for (k, g) in &candidates {
                if let Some(p) = protected.get(*k) {
                    out.left.push(left(p, g.len(), e.to_string()));
                }
            }
        }
    }
    Ok(out)
}

/// Puts what the kernel made of each group in the result.
fn record(out: &mut Merging, protected: &[Protected], candidates: &[(usize, Vec<usize>)], batch: Batch) {
    for ((k, g), result) in candidates.iter().zip(batch.groups) {
        let Some(p) = protected.get(*k) else { continue };
        match result {
            Ok(r) => out.merged.push(MergedFace {
                primitive: p.primitive,
                kind: p.kind.to_string(),
                face: r.face,
                patches: g.len(),
                max_edge_gap: r.gap,
                area: 0.0,
            }),
            Err(why) => out.left.push(left(p, g.len(), why)),
        }
    }
    out.body = batch.brep;
}
