//! `region.*`: persistent regions of a mesh (design doc §6.2: Region as a first-class entity).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::{HashSet, VecDeque};
use std::sync::Arc;
#[path = "region_edit.rs"]
mod edit;
#[path = "region_noise.rs"]
mod noise;
use edit::*;
use noise::{NoiseSource, NoiseSpread, with_noise};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::segment::{Segment, SegmentOptions, grow_smooth, region_points, segment_cancellable};
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
    /// An organic area that no primitive describes (found by `region.auto_segment`; `kind` is
    /// then null). Fit a freeform surface to it.
    pub freeform: bool,
}

pub(crate) fn info(s: &Session, id: ObjectId) -> CmdResult<RegionInfo> {
    let r = s.doc.region(id).ok_or_else(|| CmdError::Params(format!("no region {id}")))?;
    Ok(RegionInfo {
        id,
        name: r.name.clone(),
        mesh: r.mesh,
        triangles: r.triangles.len(),
        kind: r.kind,
        rms: r.rms,
        freeform: r.freeform,
    })
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
        r.freeform = false;
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
    /// Segmentation settings (tolerance 0.2 mm, 20°, ≥ 30 triangles by default; `freeform` true: the fragments of an organic surface become a few large regions flagged `freeform`).
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
    /// The scanner noise sigma (mm) the freeform consolidation used; absent when it is off.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_used: Option<f64>,
    /// Where it came from: `estimated` (measured on the scan), `given` (`options.noise`) or
    /// `fallback` (no reliable estimate; only small regions are consolidated).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_source: Option<NoiseSource>,
    /// The smallest and largest noise measured on blocks of the mesh, when two or more were
    /// measured (a spread above a factor of two is why `fallback` is chosen).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub noise_spread: Option<NoiseSpread>,
    /// The noise is above the tolerance: no primitive can be judged and nothing is consolidated.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub noise_above_tolerance: bool,
}

/// Adds the region of one segment found (`number` names it): a freeform area has no kind and no
/// RMS, a primitive region has both.
fn add_segment(s: &mut Session, mesh: ObjectId, seg: Segment, number: usize) -> ObjectId {
    let label =
        if seg.freeform { "freeform".to_string() } else { format!("{:?}", seg.primitive.kind()).to_lowercase() };
    let id = s.doc.add_region(&format!("{label} {number}"), mesh, seg.triangles);
    if let Some(r) = s.doc.region_mut(id) {
        r.freeform = seg.freeform;
        r.kind = (!seg.freeform).then(|| seg.primitive.kind());
        r.rms = (!seg.freeform).then_some(seg.rms);
    }
    id
}

pub(crate) fn auto_segment(s: &mut Session, i: AutoIn) -> CmdResult<RegionsOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let o = &i.options;
    if !(o.tolerance.is_finite() && o.tolerance > 0.0) || !o.max_angle_deg.is_finite() {
        return Err(CmdError::Params("tolerance must be positive and the angle finite".into()));
    }
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let (options, noise) = with_noise(&m, &m, o)?;
    if i.replace {
        s.doc.regions.retain(|r| r.mesh != mesh);
    }
    let segs = segment_cancellable(&m, &options, &s.cancel).ok_or_else(super::cancelled)?;
    let mut out = Vec::new();
    let mut assigned = 0;
    for (k, seg) in segs.into_iter().enumerate() {
        assigned += seg.triangles.len();
        let id = add_segment(s, mesh, seg, k + 1);
        out.push(info(s, id)?);
    }
    Ok(RegionsOut {
        regions: out,
        unassigned: m.triangle_count().saturating_sub(assigned),
        noise_used: noise.map(|n| n.sigma),
        noise_source: noise.map(|n| n.source),
        noise_spread: noise.and_then(|n| n.spread),
        noise_above_tolerance: noise.is_some_and(|n| n.above_tolerance),
    })
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
    Ok(RegionsOut {
        regions: out,
        unassigned: 0,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
    })
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
    Ok(RegionsOut {
        regions,
        unassigned: 0,
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
    })
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
    // the noise is measured on the selected geometry (the part may be noisier or cleaner than the
    // rest of the mesh); only a selection too small to measure uses the whole mesh, if uniform
    let sub = m.submesh(&all);
    let (options, noise) = with_noise(&m, &sub, o)?;
    let segs = segment_cancellable(&sub, &options, &s.cancel).ok_or_else(super::cancelled)?;
    if segs.is_empty() {
        return Err(CmdError::Failed(format!(
            "no surface of at least {} triangles was found in the {} triangles of the regions; they are kept as they were",
            o.min_triangles,
            all.len()
        )));
    }
    let mut touched: Vec<(ObjectId, bool)> = Vec::new();
    for (k, seg) in segs.into_iter().enumerate() {
        // a sub-mesh triangle is the original triangle at the same position in `all`
        let tris: Vec<u32> = seg.triangles.iter().filter_map(|&t| all.get(t as usize).copied()).collect();
        let rid = add_segment(s, mesh, Segment { triangles: tris, ..seg }, k + 1);
        touched.push((rid, true));
    }
    drop_regions(s, &ids);
    touched.extend(ids.iter().map(|&id| (id, false)));
    let unassigned = unassigned_of(s, mesh, &all);
    Ok(RegionChangeOut {
        regions: report(s, &touched),
        unassigned,
        noise_used: noise.map(|n| n.sigma),
        noise_source: noise.map(|n| n.source),
        noise_spread: noise.and_then(|n| n.spread),
        noise_above_tolerance: noise.is_some_and(|n| n.above_tolerance),
    })
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
