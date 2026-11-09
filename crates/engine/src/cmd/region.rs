//! `region.*`: persistent regions of a mesh (design doc §6.2: Region as a first-class entity).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::{HashSet, VecDeque};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::segment::{SegmentOptions, grow_smooth, region_points, segment};
use surfacecraft_fit::{PrimitiveKind, fit_auto};
use surfacecraft_geom::{Mesh, Point3};

/// A region as reported.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RegionInfo {
    /// Region id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// Mesh it belongs to.
    pub mesh: ObjectId,
    /// Number of triangles.
    pub triangles: usize,
    /// Classified type.
    pub kind: Option<PrimitiveKind>,
    /// RMS of the classification (mm).
    pub rms: Option<f64>,
}

pub(crate) fn info(s: &Session, id: ObjectId) -> CmdResult<RegionInfo> {
    let r = s.doc.region(id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    Ok(RegionInfo { id, name: r.name.clone(), mesh: r.mesh, triangles: r.triangles.len(), kind: r.kind, rms: r.rms })
}

fn check_tris(s: &Session, mesh: ObjectId, tris: &[u32]) -> CmdResult<()> {
    let n = s.doc.mesh(mesh).map_or(0, |m| m.mesh.triangle_count());
    if tris.is_empty() {
        return Err(CmdError::Params("no triangles given".into()));
    }
    if let Some(t) = tris.iter().find(|&&t| t as usize >= n) {
        return Err(CmdError::Params(format!("triangle {t} is out of range (mesh has {n})")));
    }
    Ok(())
}

/// Classifies a region by an automatic fit (stores kind and RMS).
pub(crate) fn classify(s: &mut Session, id: ObjectId) -> CmdResult<()> {
    let r = s.doc.region(id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    let m = s.doc.mesh(r.mesh).ok_or_else(|| CmdError::Params(format!("no mesh {}", r.mesh)))?;
    let normals = m.mesh.vertex_normals();
    let (p, n) = region_points(&m.mesh, &normals, &r.triangles);
    let fit = fit_auto(&p, &n);
    if let Some(r) = s.doc.region_mut(id) {
        r.kind = fit.as_ref().map(|f| f.best.primitive.kind());
        r.rms = fit.as_ref().map(|f| f.best.rms);
    }
    Ok(())
}

/// Create parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateIn {
    /// Mesh id or name.
    pub mesh: ObjRef,
    /// Triangle indices (e.g. picked in the viewport).
    pub triangles: Vec<u32>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn create(s: &mut Session, i: CreateIn) -> CmdResult<RegionInfo> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    check_tris(s, mesh, &i.triangles)?;
    let id = s.doc.add_region(i.name.as_deref().unwrap_or("region"), mesh, i.triangles);
    classify(s, id)?;
    info(s, id)
}

/// Grow parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GrowIn {
    /// Mesh id or name.
    pub mesh: ObjRef,
    /// Seed triangles.
    pub triangles: Vec<u32>,
    /// Largest angle between neighbouring triangles crossed (degrees, default 15).
    #[serde(default)]
    pub max_angle_deg: Option<f64>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn grow(s: &mut Session, i: GrowIn) -> CmdResult<RegionInfo> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    check_tris(s, mesh, &i.triangles)?;
    let ang = i.max_angle_deg.unwrap_or(15.0);
    if !(ang.is_finite() && ang > 0.0 && ang < 180.0) {
        return Err(CmdError::Params("max_angle_deg must be in (0, 180)".into()));
    }
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let tris = grow_smooth(&m, &i.triangles, ang);
    let id = s.doc.add_region(i.name.as_deref().unwrap_or("region"), mesh, tris);
    classify(s, id)?;
    info(s, id)
}

/// Auto-segment parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AutoIn {
    /// Mesh id or name.
    pub mesh: ObjRef,
    /// Segmentation settings (tolerance 0.2 mm, 20°, ≥ 30 triangles by default).
    #[serde(default)]
    pub options: SegmentOptions,
    /// Delete the mesh's existing regions first (default true).
    #[serde(default = "yes")]
    pub replace: bool,
}

fn yes() -> bool {
    true
}

/// Regions created.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RegionsOut {
    /// Regions.
    pub regions: Vec<RegionInfo>,
    /// Triangles not in any region.
    pub unassigned: usize,
}

fn auto_segment(s: &mut Session, i: AutoIn) -> CmdResult<RegionsOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let o = &i.options;
    if !(o.tolerance.is_finite() && o.tolerance > 0.0) || !o.max_angle_deg.is_finite() {
        return Err(CmdError::Params("tolerance must be positive and the angle finite".into()));
    }
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    if i.replace {
        s.doc.regions.retain(|r| r.mesh != mesh);
    }
    let segs = segment(&m, o);
    let mut out = Vec::new();
    let mut assigned = 0;
    for (k, seg) in segs.into_iter().enumerate() {
        assigned += seg.triangles.len();
        let name = format!("{} {}", format!("{:?}", seg.primitive.kind()).to_lowercase(), k + 1);
        let id = s.doc.add_region(&name, mesh, seg.triangles);
        if let Some(r) = s.doc.region_mut(id) {
            r.kind = Some(seg.primitive.kind());
            r.rms = Some(seg.rms);
        }
        out.push(info(s, id)?);
    }
    Ok(RegionsOut { regions: out, unassigned: m.triangle_count().saturating_sub(assigned) })
}

/// Merge parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct MergeIn {
    /// Regions to merge (at least two, same mesh); the first keeps its id.
    pub regions: Vec<ObjRef>,
}

fn merge(s: &mut Session, i: MergeIn) -> CmdResult<RegionInfo> {
    if i.regions.len() < 2 {
        return Err(CmdError::Params("give at least two regions".into()));
    }
    let ids: Vec<ObjectId> = i.regions.iter().map(|r| r.resolve(s, ObjectKind::Region)).collect::<CmdResult<_>>()?;
    let first = ids[0];
    let mesh = s.doc.region(first).map(|r| r.mesh).ok_or_else(|| CmdError::Params(format!("no region {first}")))?;
    let mut all: Vec<u32> = Vec::new();
    for id in &ids {
        let r = s.doc.region(*id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
        if r.mesh != mesh {
            return Err(CmdError::Params("regions of different meshes cannot be merged".into()));
        }
        all.extend(r.triangles.iter().copied());
    }
    all.sort_unstable();
    all.dedup();
    for id in ids.iter().skip(1) {
        s.doc.remove(*id);
    }
    if let Some(r) = s.doc.region_mut(first) {
        r.triangles = Arc::new(all);
    }
    classify(s, first)?;
    info(s, first)
}

/// Split parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SplitIn {
    /// Region to split.
    pub region: ObjRef,
    /// Triangles moved to a new region; omitted → split into connected pieces.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
}

fn components(mesh: &surfacecraft_geom::Mesh, tris: &[u32]) -> Vec<Vec<u32>> {
    let set: HashSet<u32> = tris.iter().copied().collect();
    let nb = mesh.triangle_neighbours();
    let mut seen: HashSet<u32> = HashSet::new();
    let mut out = Vec::new();
    for &t in tris {
        if !seen.insert(t) {
            continue;
        }
        let mut comp = vec![t];
        let mut q = VecDeque::from([t]);
        while let Some(x) = q.pop_front() {
            for &o in nb.get(x as usize).into_iter().flatten() {
                if o != u32::MAX && set.contains(&o) && seen.insert(o) {
                    comp.push(o);
                    q.push_back(o);
                }
            }
        }
        out.push(comp);
    }
    out
}

fn split(s: &mut Session, i: SplitIn) -> CmdResult<RegionsOut> {
    let id = i.region.resolve(s, ObjectKind::Region)?;
    let r = s.doc.region(id).cloned().ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    let pieces: Vec<Vec<u32>> = match i.triangles {
        Some(t) => {
            let moved: HashSet<u32> = t.into_iter().collect();
            let (a, b): (Vec<u32>, Vec<u32>) = r.triangles.iter().partition(|x| !moved.contains(x));
            if a.is_empty() || b.is_empty() {
                return Err(CmdError::Params("the triangles must be a non-empty, proper part of the region".into()));
            }
            vec![a, b]
        }
        None => {
            let m = s.doc.mesh(r.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params("no mesh".into()))?;
            components(&m, &r.triangles)
        }
    };
    let mut out = Vec::new();
    for (k, p) in pieces.into_iter().enumerate() {
        let rid = if k == 0 {
            if let Some(reg) = s.doc.region_mut(id) {
                let mut p = p;
                p.sort_unstable();
                reg.triangles = Arc::new(p);
            }
            id
        } else {
            s.doc.add_region(&r.name, r.mesh, p)
        };
        classify(s, rid)?;
        out.push(info(s, rid)?);
    }
    Ok(RegionsOut { regions: out, unassigned: 0 })
}

/// List parameters.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ListIn {
    /// Only regions of this mesh.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
}

fn list(s: &mut Session, i: ListIn) -> CmdResult<RegionsOut> {
    let mesh = i.mesh.map(|m| m.resolve(s, ObjectKind::Mesh)).transpose()?;
    let ids: Vec<ObjectId> = s.doc.regions.iter().filter(|r| mesh.is_none_or(|m| r.mesh == m)).map(|r| r.id).collect();
    let regions = ids.into_iter().map(|id| info(s, id)).collect::<CmdResult<_>>()?;
    Ok(RegionsOut { regions, unassigned: 0 })
}

/// A region a region operation created, changed or removed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TouchedRegion {
    /// Region id.
    pub id: ObjectId,
    /// Triangles it holds afterwards (0 when removed).
    pub triangles: usize,
    /// Created by the operation.
    pub created: bool,
    /// Removed by the operation (left empty, or replaced by new regions).
    pub removed: bool,
}

/// What a region operation changed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RegionChangeOut {
    /// Regions created, changed or removed.
    pub regions: Vec<TouchedRegion>,
    /// Triangles the operation released that no region holds afterwards.
    pub unassigned: usize,
}

/// Sets the flags of `tris` in `f` (indices out of range are ignored).
fn mark(f: &mut [bool], tris: &[u32]) {
    for &t in tris {
        if let Some(x) = f.get_mut(t as usize) {
            *x = true;
        }
    }
}

/// Membership flags of `tris` over `n` triangles.
fn flags(n: usize, tris: &[u32]) -> Vec<bool> {
    let mut f = vec![false; n];
    mark(&mut f, tris);
    f
}

/// Flags of the triangles of `mesh` that one of its regions holds.
fn held_flags(s: &Session, mesh: ObjectId, n: usize) -> Vec<bool> {
    let mut f = vec![false; n];
    for r in s.doc.regions.iter().filter(|r| r.mesh == mesh) {
        mark(&mut f, &r.triangles);
    }
    f
}

/// How many of `released` no region of `mesh` holds any more.
fn unassigned_of(s: &Session, mesh: ObjectId, released: &[u32]) -> usize {
    let n = s.doc.mesh(mesh).map_or(0, |m| m.mesh.triangle_count());
    let held = held_flags(s, mesh, n);
    released.iter().filter(|&&t| !held.get(t as usize).copied().unwrap_or(false)).count()
}

/// The mesh of region `id`, the region's triangles and the mesh itself (shared, not copied).
fn parts(s: &Session, id: ObjectId) -> CmdResult<(ObjectId, Arc<Vec<u32>>, Arc<Mesh>)> {
    let r = s.doc.region(id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    let m = s.doc.mesh(r.mesh).ok_or_else(|| CmdError::Params(format!("no mesh {}", r.mesh)))?;
    Ok((r.mesh, Arc::clone(&r.triangles), Arc::clone(&m.mesh)))
}

/// Rings to add or remove: at least one, one by default.
fn rings_of(rings: Option<u32>) -> CmdResult<u32> {
    match rings.unwrap_or(1) {
        0 => Err(CmdError::Params("rings must be at least 1".into())),
        n => Ok(n),
    }
}

/// The regions `touched` (id, created) as they are now; a region that is gone counts as removed.
fn report(s: &Session, touched: &[(ObjectId, bool)]) -> Vec<TouchedRegion> {
    touched
        .iter()
        .map(|&(id, created)| match s.doc.region(id) {
            Some(r) => TouchedRegion { id, triangles: r.triangles.len(), created, removed: false },
            None => TouchedRegion { id, triangles: 0, created: false, removed: true },
        })
        .collect()
}

/// Removes the regions `gone`, clearing the links that primitives and surfaces hold to them (what
/// `Document::remove` does for one region, here for many in one pass).
fn drop_regions(s: &mut Session, gone: &[ObjectId]) {
    let gone: HashSet<ObjectId> = gone.iter().copied().collect();
    if gone.is_empty() {
        return;
    }
    s.doc.regions.retain(|r| !gone.contains(&r.id));
    for p in &mut s.doc.primitives {
        if p.region.is_some_and(|r| gone.contains(&r)) {
            p.region = None;
        }
    }
    for x in &mut s.doc.surfaces {
        if x.region.is_some_and(|r| gone.contains(&r)) {
            x.region = None;
        }
    }
}

/// Whether triangle `t` borders a triangle outside `member`; a mesh edge counts as outside.
fn has_outside(nb: &[[u32; 3]], member: &[bool], t: u32) -> bool {
    nb.get(t as usize).is_some_and(|ns| ns.iter().any(|&o| !member.get(o as usize).copied().unwrap_or(false)))
}

/// Enlarge parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EnlargeIn {
    /// Region id or name.
    pub region: ObjRef,
    /// Rings of triangles to add (default 1).
    #[serde(default)]
    pub rings: Option<u32>,
    /// Also take triangles that other regions hold, which they lose (default false: only unassigned
    /// triangles are taken).
    #[serde(default)]
    pub steal: bool,
}

fn enlarge(s: &mut Session, i: EnlargeIn) -> CmdResult<RegionChangeOut> {
    let id = i.region.resolve(s, ObjectKind::Region)?;
    let rings = rings_of(i.rings)?;
    let (mesh, own, m) = parts(s, id)?;
    let n = m.triangle_count();
    let nb = m.triangle_neighbours();
    let held = held_flags(s, mesh, n);
    let mut inside = flags(n, &own);
    // breadth first, ring by ring, across shared edges; stops early when a ring adds nothing
    let mut frontier: Vec<u32> = own.to_vec();
    let mut added: Vec<u32> = Vec::new();
    for _ in 0..rings {
        if frontier.is_empty() {
            break;
        }
        let mut next: Vec<u32> = Vec::new();
        for &t in &frontier {
            for &o in nb.get(t as usize).into_iter().flatten() {
                let oi = o as usize;
                let taken = inside.get(oi).copied().unwrap_or(true);
                let blocked = !i.steal && held.get(oi).copied().unwrap_or(true);
                if taken || blocked {
                    continue;
                }
                if let Some(x) = inside.get_mut(oi) {
                    *x = true;
                }
                added.push(o);
                next.push(o);
            }
        }
        frontier = next;
    }
    let mut touched: Vec<(ObjectId, bool)> = vec![(id, false)];
    let mut emptied: Vec<ObjectId> = Vec::new();
    if i.steal {
        let others: Vec<ObjectId> =
            s.doc.regions.iter().filter(|r| r.mesh == mesh && r.id != id).map(|r| r.id).collect();
        for oid in others {
            let Some(r) = s.doc.region_mut(oid) else { continue };
            let before = r.triangles.len();
            let keep: Vec<u32> =
                r.triangles.iter().copied().filter(|&t| !inside.get(t as usize).copied().unwrap_or(false)).collect();
            if keep.len() != before {
                if keep.is_empty() {
                    emptied.push(oid);
                }
                r.triangles = Arc::new(keep);
                touched.push((oid, false));
            }
        }
    }
    let mut all: Vec<u32> = own.to_vec();
    all.extend_from_slice(&added);
    all.sort_unstable();
    if let Some(r) = s.doc.region_mut(id) {
        r.triangles = Arc::new(all);
    }
    drop_regions(s, &emptied);
    classify(s, id)?;
    Ok(RegionChangeOut { regions: report(s, &touched), unassigned: 0 })
}

/// Shrink parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ShrinkIn {
    /// Region id or name.
    pub region: ObjRef,
    /// Rings of triangles to remove from the edge (default 1).
    #[serde(default)]
    pub rings: Option<u32>,
}

fn shrink(s: &mut Session, i: ShrinkIn) -> CmdResult<RegionChangeOut> {
    let id = i.region.resolve(s, ObjectKind::Region)?;
    let rings = rings_of(i.rings)?;
    let (mesh, own, m) = parts(s, id)?;
    let nb = m.triangle_neighbours();
    let mut member = flags(m.triangle_count(), &own);
    // erosion: a ring goes when it borders a triangle outside the region; the next ring is what
    // borders the ring just removed
    let mut front: Vec<u32> = own.iter().copied().filter(|&t| has_outside(&nb, &member, t)).collect();
    let mut removed: Vec<u32> = Vec::new();
    for _ in 0..rings {
        if front.is_empty() {
            break;
        }
        for &t in &front {
            if let Some(x) = member.get_mut(t as usize) {
                *x = false;
            }
        }
        removed.extend_from_slice(&front);
        let mut next: Vec<u32> = Vec::new();
        for &t in &front {
            next.extend(
                nb.get(t as usize)
                    .into_iter()
                    .flatten()
                    .copied()
                    .filter(|&o| member.get(o as usize).copied().unwrap_or(false)),
            );
        }
        next.sort_unstable();
        next.dedup();
        front = next;
    }
    let keep: Vec<u32> = own.iter().copied().filter(|&t| member.get(t as usize).copied().unwrap_or(false)).collect();
    if keep.is_empty() {
        drop_regions(s, &[id]);
    } else {
        if let Some(r) = s.doc.region_mut(id) {
            r.triangles = Arc::new(keep);
        }
        classify(s, id)?;
    }
    let unassigned = unassigned_of(s, mesh, &removed);
    Ok(RegionChangeOut { regions: report(s, &[(id, false)]), unassigned })
}

/// Separate parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SeparateIn {
    /// Region to separate.
    pub region: ObjRef,
    /// Pieces with fewer triangles become unassigned (default 1: every piece is kept).
    #[serde(default)]
    pub min_triangles: Option<usize>,
}

fn separate(s: &mut Session, i: SeparateIn) -> CmdResult<RegionChangeOut> {
    let id = i.region.resolve(s, ObjectKind::Region)?;
    let r = s.doc.region(id).cloned().ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    let m =
        s.doc.mesh(r.mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {}", r.mesh)))?;
    let min = i.min_triangles.unwrap_or(1).max(1);
    let mut pieces = components(&m, &r.triangles);
    for p in &mut pieces {
        p.sort_unstable();
    }
    // the largest piece keeps the region's id
    pieces.sort_by_key(|p| std::cmp::Reverse(p.len()));
    let kept: Vec<Vec<u32>> = pieces.into_iter().filter(|p| p.len() >= min).collect();
    let mut touched: Vec<(ObjectId, bool)> = Vec::new();
    if kept.is_empty() {
        drop_regions(s, &[id]);
        touched.push((id, false));
    }
    for (k, p) in kept.into_iter().enumerate() {
        let rid = if k == 0 {
            if let Some(reg) = s.doc.region_mut(id) {
                reg.triangles = Arc::new(p);
            }
            id
        } else {
            s.doc.add_region(&r.name, r.mesh, p)
        };
        classify(s, rid)?;
        touched.push((rid, k > 0));
    }
    let unassigned = unassigned_of(s, r.mesh, &r.triangles);
    Ok(RegionChangeOut { regions: report(s, &touched), unassigned })
}

/// A sphere selection: the triangles whose centroid lies inside it.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SphereSel {
    /// Centre (mm).
    pub center: [f64; 3],
    /// Radius (mm, above zero).
    pub radius: f64,
}

/// A box selection: the triangles whose centroid lies inside it.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BoxSel {
    /// Lower corner (mm).
    pub min: [f64; 3],
    /// Upper corner (mm), at least `min` on every axis.
    pub max: [f64; 3],
}

/// Insert parameters: exactly one of `triangles`, `sphere`, `box`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct InsertIn {
    /// Mesh id or name.
    pub mesh: ObjRef,
    /// Triangle indices.
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Triangles whose centroid lies in this sphere.
    #[serde(default)]
    pub sphere: Option<SphereSel>,
    /// Triangles whose centroid lies in this box.
    #[serde(default, rename = "box")]
    pub boxed: Option<BoxSel>,
    /// Name of the new region.
    #[serde(default)]
    pub name: Option<String>,
}

/// Triangles of `m` whose centroid lies in the sphere.
fn sphere_tris(m: &Mesh, sp: &SphereSel) -> CmdResult<Vec<u32>> {
    if !sp.radius.is_finite() || sp.radius <= 0.0 || !sp.center.iter().all(|v| v.is_finite()) {
        return Err(CmdError::Params("a sphere needs a finite centre and a positive radius".into()));
    }
    let [x, y, z] = sp.center;
    let c = Point3::new(x, y, z);
    let r2 = sp.radius * sp.radius;
    Ok((0..m.triangle_count())
        .filter(|&t| m.face_centroid(t).is_some_and(|p| (p - c).norm_squared() <= r2))
        .map(|t| t as u32)
        .collect())
}

/// Triangles of `m` whose centroid lies in the box.
fn box_tris(m: &Mesh, b: &BoxSel) -> CmdResult<Vec<u32>> {
    let finite = b.min.iter().chain(b.max.iter()).all(|v| v.is_finite());
    if !finite || b.min.iter().zip(b.max.iter()).any(|(lo, hi)| lo > hi) {
        return Err(CmdError::Params("a box needs finite corners, with min at most max on every axis".into()));
    }
    let (lo, hi) = (b.min, b.max);
    Ok((0..m.triangle_count())
        .filter(|&t| {
            m.face_centroid(t).is_some_and(|p| {
                let q = [p.x, p.y, p.z];
                (0..3).all(|k| (lo[k]..=hi[k]).contains(&q[k]))
            })
        })
        .map(|t| t as u32)
        .collect())
}

fn insert(s: &mut Session, i: InsertIn) -> CmdResult<RegionChangeOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let mut picked = match (i.triangles, i.sphere, i.boxed) {
        (Some(t), None, None) => {
            check_tris(s, mesh, &t)?;
            t
        }
        (None, Some(sp), None) => sphere_tris(&m, &sp)?,
        (None, None, Some(b)) => box_tris(&m, &b)?,
        _ => return Err(CmdError::Params("give exactly one of triangles, sphere or box".into())),
    };
    picked.sort_unstable();
    picked.dedup();
    if picked.is_empty() {
        return Err(CmdError::Params("the selection holds no triangles".into()));
    }
    let chosen = flags(m.triangle_count(), &picked);
    // the selected triangles leave every region of the mesh that held them
    let others: Vec<ObjectId> = s.doc.regions.iter().filter(|r| r.mesh == mesh).map(|r| r.id).collect();
    let mut emptied: Vec<ObjectId> = Vec::new();
    let mut touched: Vec<(ObjectId, bool)> = Vec::new();
    for oid in others {
        let Some(r) = s.doc.region_mut(oid) else { continue };
        let before = r.triangles.len();
        let keep: Vec<u32> =
            r.triangles.iter().copied().filter(|&t| !chosen.get(t as usize).copied().unwrap_or(false)).collect();
        if keep.len() != before {
            if keep.is_empty() {
                emptied.push(oid);
            }
            r.triangles = Arc::new(keep);
            touched.push((oid, false));
        }
    }
    drop_regions(s, &emptied);
    let name = i.name.as_deref().unwrap_or("region");
    let rid = s.doc.add_region(name, mesh, picked);
    classify(s, rid)?;
    let mut out = vec![(rid, true)];
    out.extend(touched);
    Ok(RegionChangeOut { regions: report(s, &out), unassigned: 0 })
}

/// Resegment parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ResegmentIn {
    /// Regions to segment again; all of one mesh. Their triangles are the only input.
    pub regions: Vec<ObjRef>,
    /// Segmentation settings (as in `region.auto_segment`).
    #[serde(default)]
    pub options: SegmentOptions,
}

fn resegment(s: &mut Session, i: ResegmentIn) -> CmdResult<RegionChangeOut> {
    let o = &i.options;
    if !(o.tolerance.is_finite() && o.tolerance > 0.0) || !o.max_angle_deg.is_finite() {
        return Err(CmdError::Params("tolerance must be positive and the angle finite".into()));
    }
    let mut ids: Vec<ObjectId> =
        i.regions.iter().map(|r| r.resolve(s, ObjectKind::Region)).collect::<CmdResult<_>>()?;
    ids.sort_unstable();
    ids.dedup();
    let Some(&first) = ids.first() else {
        return Err(CmdError::Params("give at least one region".into()));
    };
    let mesh = s.doc.region(first).map(|r| r.mesh).ok_or_else(|| CmdError::Params(format!("no region {first}")))?;
    let mut all: Vec<u32> = Vec::new();
    for id in &ids {
        let r = s.doc.region(*id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
        if r.mesh != mesh {
            return Err(CmdError::Params("regions of different meshes cannot be resegmented together".into()));
        }
        all.extend(r.triangles.iter().copied());
    }
    all.sort_unstable();
    all.dedup();
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let n = m.triangle_count();
    all.retain(|&t| (t as usize) < n);
    if all.is_empty() {
        return Err(CmdError::Params("the regions hold no triangles".into()));
    }
    // the triangles alone form the sub-mesh, so its outer edges bound the growth
    let segs = segment(&m.submesh(&all), o);
    let mut touched: Vec<(ObjectId, bool)> = Vec::new();
    for (k, seg) in segs.into_iter().enumerate() {
        let kind = seg.primitive.kind();
        // a sub-mesh triangle is the original triangle at the same position in `all`
        let tris: Vec<u32> = seg.triangles.iter().filter_map(|&t| all.get(t as usize).copied()).collect();
        let name = format!("{} {}", format!("{kind:?}").to_lowercase(), k + 1);
        let rid = s.doc.add_region(&name, mesh, tris);
        if let Some(r) = s.doc.region_mut(rid) {
            r.kind = Some(kind);
            r.rms = Some(seg.rms);
        }
        touched.push((rid, true));
    }
    drop_regions(s, &ids);
    touched.extend(ids.iter().map(|&id| (id, false)));
    let unassigned = unassigned_of(s, mesh, &all);
    Ok(RegionChangeOut { regions: report(s, &touched), unassigned })
}

/// `region.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "region.create",
        "New region",
        "Region",
        true,
        "Creates a region of `mesh` from `triangles` and classifies it.",
        CreateIn,
        RegionInfo,
        create
    ),
    command!(
        "region.grow",
        "Grow region",
        "Region",
        true,
        "Region grown from seed `triangles` across edges sharper than `max_angle_deg` (smooth selection).",
        GrowIn,
        RegionInfo,
        grow
    ),
    command!(
        "region.auto_segment",
        "Auto segment",
        "Region",
        true,
        "Splits `mesh` into regions that each follow one primitive (fit-and-grow).",
        AutoIn,
        RegionsOut,
        auto_segment
    ),
    command!(
        "region.merge",
        "Merge regions",
        "Region",
        true,
        "Merges `regions` of one mesh into the first.",
        MergeIn,
        RegionInfo,
        merge
    ),
    command!(
        "region.split",
        "Split region",
        "Region",
        true,
        "Moves `triangles` of `region` into a new region, or splits it into connected pieces.",
        SplitIn,
        RegionsOut,
        split
    ),
    command!(
        "region.list",
        "Regions",
        "Region",
        false,
        "Lists regions (optionally of one `mesh`).",
        ListIn,
        RegionsOut,
        list
    ),
    command!(
        "region.enlarge",
        "Enlarge region",
        "Region",
        true,
        "Adds `rings` rings of triangles across mesh edges to `region`: only unassigned triangles unless `steal` is set.",
        EnlargeIn,
        RegionChangeOut,
        enlarge
    ),
    command!(
        "region.shrink",
        "Shrink region",
        "Region",
        true,
        "Removes `rings` rings of triangles from the edge of `region` (triangles on the mesh edge count as edge).",
        ShrinkIn,
        RegionChangeOut,
        shrink
    ),
    command!(
        "region.separate",
        "Separate region",
        "Region",
        true,
        "Splits `region` into its connected pieces; pieces below `min_triangles` become unassigned.",
        SeparateIn,
        RegionChangeOut,
        separate
    ),
    command!(
        "region.insert",
        "Insert region",
        "Region",
        true,
        "New region from `triangles`, or from the triangles whose centroid lies in a `sphere` or `box`; they leave the regions that held them.",
        InsertIn,
        RegionChangeOut,
        insert
    ),
    command!(
        "region.resegment",
        "Resegment regions",
        "Region",
        true,
        "Runs auto segmentation again on the triangles of `regions` only; every other region keeps its id and triangles.",
        ResegmentIn,
        RegionChangeOut,
        resegment
    ),
];
