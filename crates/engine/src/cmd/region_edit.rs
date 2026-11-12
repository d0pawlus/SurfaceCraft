//! `region.enlarge`, `region.shrink`, `region.separate` and `region.insert`: editing the triangles
//! of persistent regions, and the report of what such an operation changed.
use super::*;

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
    /// The scanner noise sigma (mm) a segmentation used; absent for other operations.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_used: Option<f64>,
    /// Where it came from (see `region.auto_segment`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_source: Option<NoiseSource>,
    /// The smallest and largest noise measured on blocks (see `region.auto_segment`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_spread: Option<NoiseSpread>,
    /// The noise is above the tolerance: nothing is consolidated.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub noise_above_tolerance: bool,
}

/// Sets the flags of `tris` in `f` (indices out of range are ignored).
pub(super) fn mark(f: &mut [bool], tris: &[u32]) {
    for &t in tris {
        if let Some(x) = f.get_mut(t as usize) {
            *x = true;
        }
    }
}

/// Membership flags of `tris` over `n` triangles.
pub(super) fn flags(n: usize, tris: &[u32]) -> Vec<bool> {
    let mut f = vec![false; n];
    mark(&mut f, tris);
    f
}

/// Flags of the triangles of `mesh` that one of its regions holds.
pub(super) fn held_flags(s: &Session, mesh: ObjectId, n: usize) -> Vec<bool> {
    let mut f = vec![false; n];
    for r in s.doc.regions.iter().filter(|r| r.mesh == mesh) {
        mark(&mut f, &r.triangles);
    }
    f
}

/// How many of `released` no region of `mesh` holds any more.
pub(super) fn unassigned_of(s: &Session, mesh: ObjectId, released: &[u32]) -> usize {
    let n = s.doc.mesh(mesh).map_or(0, |m| m.mesh.triangle_count());
    let held = held_flags(s, mesh, n);
    released.iter().filter(|&&t| !held.get(t as usize).copied().unwrap_or(false)).count()
}

/// The mesh of region `id`, the region's triangles and the mesh itself (shared, not copied).
pub(super) fn parts(s: &Session, id: ObjectId) -> CmdResult<(ObjectId, Arc<Vec<u32>>, Arc<Mesh>)> {
    let r = s.doc.region(id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    let m = s.doc.mesh(r.mesh).ok_or_else(|| CmdError::Params(format!("no mesh {}", r.mesh)))?;
    Ok((r.mesh, Arc::clone(&r.triangles), Arc::clone(&m.mesh)))
}

/// Rings to add or remove: at least one, one by default.
pub(super) fn rings_of(rings: Option<u32>) -> CmdResult<u32> {
    match rings.unwrap_or(1) {
        0 => Err(CmdError::Params("rings must be at least 1".into())),
        n => Ok(n),
    }
}

/// The regions `touched` (id, created) as they are now; a region that is gone counts as removed.
pub(super) fn report(s: &Session, touched: &[(ObjectId, bool)]) -> Vec<TouchedRegion> {
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
pub(super) fn drop_regions(s: &mut Session, gone: &[ObjectId]) {
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
pub(super) fn has_outside(nb: &[[u32; 3]], member: &[bool], t: u32) -> bool {
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

pub(super) fn enlarge(s: &mut Session, i: EnlargeIn) -> CmdResult<RegionChangeOut> {
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
    Ok(RegionChangeOut {
        regions: report(s, &touched),
        unassigned: 0,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
    })
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

pub(super) fn shrink(s: &mut Session, i: ShrinkIn) -> CmdResult<RegionChangeOut> {
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
    Ok(RegionChangeOut {
        regions: report(s, &[(id, false)]),
        unassigned,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
    })
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

pub(super) fn separate(s: &mut Session, i: SeparateIn) -> CmdResult<RegionChangeOut> {
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
    Ok(RegionChangeOut {
        regions: report(s, &touched),
        unassigned,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
    })
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
pub(super) fn sphere_tris(m: &Mesh, sp: &SphereSel) -> CmdResult<Vec<u32>> {
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
pub(super) fn box_tris(m: &Mesh, b: &BoxSel) -> CmdResult<Vec<u32>> {
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

pub(super) fn insert(s: &mut Session, i: InsertIn) -> CmdResult<RegionChangeOut> {
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
    Ok(RegionChangeOut {
        regions: report(s, &out),
        unassigned: 0,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
    })
}
