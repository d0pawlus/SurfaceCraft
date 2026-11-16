//! Merging the patches of the wrapped scan that lie on a protected primitive into one analytic face
//! each (P9-O6b). Nothing is cut: the body keeps the wrap's topology outside the groups, so it is as
//! closed (or as open) as the wrap was.
//!
//! A patch belongs to a primitive when the scan triangle nearest its middle is in the primitive's
//! region and every sample on it lies within a margin of the primitive and of the region (or of an unassigned
//! triangle on the primitive): a patch that straddles the border over a surface that leaves the primitive has a
//! sample off it, one that follows the border (a crease) is wholly inside. The patches of one primitive that touch along edges form a group, which
//! the kernel replaces by one face on the primitive (`merge_faces_onto_shape`); a group the kernel
//! cannot merge stays patches and is reported with the reason.
use super::hybrid_method::{LeftPatches, MergedFace};
use crate::Session;
use crate::cmd::{CmdError, cancelled};
use pinholes::pinholes;
use surfacecraft_doc::ObjectId;
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Bvh, Mesh, Point3, par};
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::merge::{
    Batch, GroupSpec, MAX_GAP, face_samples, grid_area, merge_faces_onto_shapes, pinch_free_groups, split_round,
};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::tess::TopoMesh;

#[path = "pipeline_pinholes.rs"]
mod pinholes;

/// Samples per side of a patch's grid (so `(n + 1)^2` points are tested) at least.
const SAMPLES_PER_SIDE: usize = 5;
/// The samples of a large patch are no farther apart than this many tolerances (a patch that is the size of
/// a plate can leave the plane between five samples by more than the samples show), up to
/// [`MAX_SAMPLES_PER_SIDE`] a side.
const SAMPLE_SPACING_IN_TOLERANCES: f64 = 2.0;
const MAX_SAMPLES_PER_SIDE: usize = 48;
/// A sample must lie within this share of the tolerance of its primitive.
const SAMPLE_SHARE_OF_TOLERANCE: f64 = 1.0;
/// The boundary of a merged group may leave its primitive by this share of the tolerance.
const GAP_SHARE_OF_TOLERANCE: f64 = 1.0;
/// Every sample must lie within this share of the tolerance of a triangle of the primitive's region, or of an
/// unassigned triangle that lies on the primitive. The pinholes of the region (a few triangles of no region
/// inside it) count as the region (`pinholes`). A share of the samples outside the region cannot be allowed
/// instead, nor the pinholes alone: a patch that lies on a primitive reaches past its region over a tangent round
/// (the part of a round that stays within the tolerance of the wall is as long as the root of twice the radius
/// times the tolerance), so 1 to 18 of 324 to 576 samples lie outside the region of patches that belong to it
/// (measured), and the pinholes were 6 triangles of the 33 018 of that scan.
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
    /// Groups that were merged without their border being moved onto the primitive first, with the reason (one
    /// line each).
    pub notes: Vec<String>,
    /// The body tessellated at the chord the areas are measured on, when it has been (the summary of the result
    /// reads it and does not tessellate again).
    pub mesh: Option<TopoMesh>,
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
    scan: &'a Mesh,
    all: Bvh,
    owner: Vec<u32>,
    regions: Vec<Bvh>,
    protected: &'a [Protected],
    tol: f64,
}

impl<'a> Lookup<'a> {
    fn new(scan: &'a Mesh, protected: &'a [Protected], tol: f64) -> Self {
        let mut owner = region_owners(scan, protected);
        let prims: Vec<&Primitive> = protected.iter().map(|p| &p.fit).collect();
        let holes = pinholes(scan, &owner, &prims, SAMPLE_SHARE_OF_TOLERANCE * tol, NONE);
        let mut triangles: Vec<Vec<u32>> = protected.iter().map(|p| p.triangles.clone()).collect();
        for (t, k) in holes {
            if let (Some(slot), Some(list)) = (owner.get_mut(t as usize), triangles.get_mut(k as usize)) {
                *slot = k;
                list.push(t);
            }
        }
        let regions = triangles.iter().map(|list| Bvh::build(&region_mesh(scan, list))).collect();
        Self { scan, all: Bvh::build(scan), owner, regions, protected, tol }
    }

    /// Is the scan within `near` of `q` a triangle of no region whose corners all lie within the fit's tolerance
    /// of `prim`? (A tangent round that stays within the tolerance of the primitive: the scan there is the
    /// primitive's within the tolerance, though the segmentation gave it to another region.)
    fn unassigned_on(&self, q: &Point3, prim: &Protected, near: f64) -> bool {
        let Some(hit) = self.all.closest(q, near) else { return false };
        let free = self.owner.get(hit.triangle).is_some_and(|&o| o == NONE);
        free && self
            .scan
            .triangle(hit.triangle)
            .is_some_and(|tri| tri.iter().all(|c| prim.fit.distance(c).abs() <= SAMPLE_SHARE_OF_TOLERANCE * self.tol))
    }

    /// The grid of samples of patch `face` and its cells a side: [`SAMPLES_PER_SIDE`], or more for a patch
    /// large against the tolerance.
    fn samples_of(&self, body: &Brep, face: usize) -> Option<(Vec<[f64; 3]>, usize)> {
        let coarse = face_samples(body, face, SAMPLES_PER_SIDE)?;
        let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
        for p in &coarse {
            for (k, c) in p.iter().enumerate() {
                if let (Some(l), Some(h)) = (lo.get_mut(k), hi.get_mut(k)) {
                    (*l, *h) = (l.min(*c), h.max(*c));
                }
            }
        }
        let size = (0..3)
            .map(|k| (hi.get(k).copied().unwrap_or(0.0) - lo.get(k).copied().unwrap_or(0.0)).powi(2))
            .sum::<f64>()
            .sqrt();
        let n = (size / (SAMPLE_SPACING_IN_TOLERANCES * self.tol)).ceil();
        let n =
            if n.is_finite() { (n as usize).clamp(SAMPLES_PER_SIDE, MAX_SAMPLES_PER_SIDE) } else { SAMPLES_PER_SIDE };
        if n == SAMPLES_PER_SIDE { Some((coarse, n)) } else { face_samples(body, face, n).map(|s| (s, n)) }
    }

    /// The protected primitive patch `face` lies on and the area of the patch: the primitive whose
    /// region holds the scan triangle nearest the middle of the patch, if every sample of the patch
    /// lies within a margin of the primitive, and of a triangle of the region (the pinholes of the region
    /// are the region's) or of an unassigned triangle that lies on the primitive.
    fn owner_of(&self, body: &Brep, face: usize) -> Option<(usize, f64)> {
        let (samples, n) = self.samples_of(body, face)?;
        let middle = samples.get(samples.len() / 2)?;
        let mid = Point3::new(middle[0], middle[1], middle[2]);
        let hit = self.all.closest(&mid, SEARCH_TOLERANCES * self.tol)?;
        let k = usize::try_from(self.owner.get(hit.triangle).copied().filter(|&o| o != NONE)?).ok()?;
        let (prim, region) = (self.protected.get(k)?, self.regions.get(k)?);
        let near = NEAR_REGION_SHARE_OF_TOLERANCE * self.tol;
        let (mut off, mut outside) = (false, false);
        for p in &samples {
            let q = Point3::new(p[0], p[1], p[2]);
            off |= prim.fit.distance(&q).abs() > SAMPLE_SHARE_OF_TOLERANCE * self.tol;
            outside |= region.closest(&q, near).is_none() && !self.unassigned_on(&q, prim, near);
        }
        (!off && !outside).then(|| (k, grid_area(&samples, n)))
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
        // a group that goes round its axis is cut into sectors (one face would need a seam)
        let groups: Vec<Vec<usize>> = pinch_free_groups(body, &faces)
            .iter()
            .flat_map(|g| split_round(body, g, &p.shape))
            .flat_map(|g| pinch_free_groups(body, &g))
            .collect();
        for group in groups {
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
    // the largest first: the kernel takes a bounded number of groups (`MAX_GROUPS`), and the first are the ones worth most
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
    let mut out = Merging { body: body.clone(), merged: Vec::new(), left: Vec::new(), notes: Vec::new(), mesh: None };
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
    // a tolerance above MAX_GAP / GAP_SHARE still merges, with the gap the kernel allows at most
    let allowed = (GAP_SHARE_OF_TOLERANCE * tol).min(MAX_GAP);
    match merge_faces_onto_shapes(body, &specs, allowed, &|| s.cancel.is_cancelled()) {
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
            Ok(r) => {
                if let Some(why) = r.snap_note.as_ref() {
                    out.notes.push(format!("{} of {} patches: {why}", p.kind, g.len()));
                }
                out.merged.push(MergedFace {
                    primitive: p.primitive,
                    kind: p.kind.to_string(),
                    face: r.face,
                    patches: g.len(),
                    max_edge_gap: r.gap,
                    area: 0.0,
                    patch_area: r.patch_area,
                });
            }
            Err(why) => out.left.push(left(p, g.len(), why)),
        }
    }
    out.body = batch.brep;
}
