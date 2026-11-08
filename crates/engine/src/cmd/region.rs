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
];
