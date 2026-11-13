//! `scan.split`: a scan with several parts or shells in one file, separated into its significant
//! connected pieces (design: P9-O9). A piece is significant when it holds at least `min_share` of
//! the triangles or of the area; the rest is debris, reported (and optionally kept as one mesh).
//! Closed pieces that lie inside another closed piece are reported as nested, with the orientation
//! that tells "the inner skin of one hollow part" (a cavity: its triangles face the void) from
//! "a separate part inside another" (an inclusion).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::pieces::{Piece, pieces};
use surfacecraft_geom::{Bvh, Mesh, nesting};

/// Share of the triangles or the area that makes a piece a part.
pub(crate) const MIN_SHARE: f64 = 0.01;
/// Most parts listed and tested for nesting; the rest of the significant pieces are counted as `parts_beyond_limit`.
pub(crate) const MAX_PARTS: usize = 64;
/// Share of the probes of an inner piece that must lie inside the outer one.
const INSIDE_SHARE: f64 = 0.5;

/// Parameters of `scan.split`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SplitIn {
    /// Scan mesh id or name.
    pub object: ObjRef,
    /// A piece is a part when it holds at least this share of the triangles or of the area (0..1, default 0.01).
    #[serde(default)]
    pub min_share: Option<f64>,
    /// Keep the debris (all pieces below the share) as one mesh named `<name>-debris` (default false).
    #[serde(default)]
    pub keep_debris: bool,
}

/// One part.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct PartOut {
    /// The new mesh.
    pub id: ObjectId,
    /// Its name.
    pub name: String,
    /// Triangles.
    pub triangles: usize,
    /// Share of the scan's triangles (0..1).
    pub share: f64,
    /// Surface area (mm2).
    pub area: f64,
    /// Lower corner of the bounding box.
    pub bbox_min: [f64; 3],
    /// Upper corner of the bounding box.
    pub bbox_max: [f64; 3],
    /// Watertight (no boundary and no non-manifold edge).
    pub closed: bool,
    /// Genus of the surface, when it is defined.
    pub genus: Option<usize>,
    /// Boundary loops.
    pub boundary_loops: usize,
    /// Index (in `parts`) of the closed part this one lies inside.
    pub nested_in: Option<usize>,
    /// `cavity` (its triangles face the void: the inner skin of the part it lies in) or `inclusion` (a separate part inside another); only for a nested part.
    pub relation: Option<String>,
}

/// What was left over.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct DebrisOut {
    /// Pieces below the share.
    pub pieces: usize,
    /// Their triangles.
    pub triangles: usize,
    /// The mesh holding them, when `keep_debris` was set.
    pub id: Option<ObjectId>,
}

/// Result of `scan.split`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct SplitOut {
    /// The parts, largest first.
    pub parts: Vec<PartOut>,
    /// Parts that are not the cavity skin of another: the bodies a model needs.
    pub distinct_parts: usize,
    /// Significant pieces past the limit of listed parts (not made into meshes).
    pub parts_beyond_limit: usize,
    /// Their triangles.
    pub beyond_limit_triangles: usize,
    /// Their surface area (mm2).
    pub beyond_limit_area: f64,
    /// The rest of the scan.
    pub debris: DebrisOut,
}

fn corner(p: &surfacecraft_geom::Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn is_part(p: &Piece, total_tris: f64, total_area: f64, share: f64) -> bool {
    p.triangles.len() as f64 >= share * total_tris || p.area >= share * total_area
}

/// For each part, the closed part it lies inside, with the relation.
fn nesting_of(mesh: &Mesh, parts: &[Piece]) -> Vec<Option<(usize, bool)>> {
    let subs: Vec<Mesh> = parts.iter().map(|p| mesh.submesh(&p.triangles)).collect();
    let volumes: Vec<f64> = subs.iter().map(signed_volume).collect();
    let mut out = vec![None; parts.len()];
    for (i, outer) in parts.iter().enumerate().filter(|(_, p)| p.closed()) {
        let bvh = Bvh::build(&subs[i]);
        let eps = 1e-6 * outer.bbox.diagonal().max(1e-9);
        for (j, inner) in parts.iter().enumerate() {
            let candidate =
                i != j && out[j].is_none_or(|(o, _): (usize, bool)| parts[o].bbox.diagonal() > outer.bbox.diagonal());
            if candidate
                && nesting::contains(&outer.bbox, &inner.bbox, 0.0)
                && nesting::inside_share(&subs[j], &bvh, eps) >= INSIDE_SHARE
            {
                out[j] = Some((i, volumes[j] < 0.0));
            }
        }
    }
    out
}

fn signed_volume(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|i| m.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

pub(crate) fn add_part(s: &mut Session, base: &str, k: usize, p: &Piece, mesh: &Mesh, total: f64) -> PartOut {
    let name = format!("{base}-part-{}", k + 1);
    let id = s.doc.add_mesh(&name, mesh.submesh(&p.triangles));
    let name = s.doc.mesh(id).map_or(name, |m| m.name.clone());
    PartOut {
        id,
        name,
        triangles: p.triangles.len(),
        share: p.triangles.len() as f64 / total.max(1.0),
        area: p.area,
        bbox_min: corner(&p.bbox.min),
        bbox_max: corner(&p.bbox.max),
        closed: p.closed(),
        genus: p.genus(),
        boundary_loops: p.loops.len(),
        nested_in: None,
        relation: None,
    }
}

/// The significant pieces of a scan, the debris, and how the parts nest.
pub(crate) struct Analysis {
    /// Significant pieces, largest first (at most [`MAX_PARTS`]).
    pub parts: Vec<Piece>,
    /// Significant pieces past the limit (kept, so that they can be counted and reported).
    pub extra: Vec<Piece>,
    /// The pieces below the share.
    pub rest: Vec<Piece>,
    /// Per part: the closed part it lies inside and whether it is a cavity skin.
    pub nests: Vec<Option<(usize, bool)>>,
}

impl Analysis {
    /// Parts that are not the cavity skin of another, in order.
    pub(crate) fn distinct(&self) -> Vec<usize> {
        (0..self.parts.len()).filter(|k| !self.nests.get(*k).is_some_and(|n| n.is_some_and(|n| n.1))).collect()
    }
}

/// Splits `mesh` into parts and debris.
pub(crate) fn analyse(mesh: &Mesh, share: f64, with_nesting: bool) -> Analysis {
    let all = pieces(mesh);
    let (total, area): (f64, f64) = (mesh.triangle_count() as f64, all.iter().map(|p| p.area).sum());
    let (mut parts, rest): (Vec<Piece>, Vec<Piece>) = all.into_iter().partition(|p| is_part(p, total, area, share));
    let extra = parts.split_off(parts.len().min(MAX_PARTS));
    let nests = if with_nesting && parts.len() > 1 { nesting_of(mesh, &parts) } else { vec![None; parts.len()] };
    Analysis { parts, extra, rest, nests }
}

pub(crate) fn split(s: &mut Session, i: SplitIn) -> CmdResult<SplitOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let share = i.min_share.unwrap_or(MIN_SHARE);
    if !(share.is_finite() && share > 0.0 && share <= 1.0) {
        return Err(CmdError::Params("min_share must be above 0 and at most 1".into()));
    }
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let (mesh, base): (Arc<Mesh>, String) = (obj.mesh.clone(), obj.name.clone());
    let Analysis { parts: listed, extra, rest, nests } = analyse(&mesh, share, true);
    if listed.is_empty() {
        let msg = if rest.is_empty() {
            "the mesh is empty or invalid".to_string()
        } else {
            format!("no piece reaches min_share {share}: the largest of the {} pieces holds less", rest.len())
        };
        return Err(CmdError::Failed(msg));
    }
    let total = mesh.triangle_count() as f64;
    let mut out: Vec<PartOut> =
        listed.iter().enumerate().map(|(k, p)| add_part(s, &base, k, p, &mesh, total)).collect();
    for (part, nest) in out.iter_mut().zip(&nests) {
        part.nested_in = nest.map(|n| n.0);
        part.relation = nest.map(|n| if n.1 { "cavity" } else { "inclusion" }.to_string());
    }
    let tris: Vec<u32> = rest.iter().flat_map(|p| p.triangles.iter().copied()).collect();
    let debris_id =
        (i.keep_debris && !tris.is_empty()).then(|| s.doc.add_mesh(&format!("{base}-debris"), mesh.submesh(&tris)));
    let distinct = out.iter().filter(|p| p.relation.as_deref() != Some("cavity")).count();
    Ok(SplitOut {
        parts: out,
        distinct_parts: distinct,
        parts_beyond_limit: extra.len(),
        beyond_limit_triangles: extra.iter().map(|p| p.triangles.len()).sum(),
        beyond_limit_area: extra.iter().map(|p| p.area).sum::<f64>() + 0.0,
        debris: DebrisOut { pieces: rest.len(), triangles: tris.len(), id: debris_id },
    })
}

/// The `scan.split` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "scan.split",
    "Split scan into parts",
    "Scan",
    true,
    "Separates the scan `object` into its significant connected pieces, each a new mesh named `<name>-part-N` (largest first): a piece is a part when it holds at least `min_share` (default 0.01) of the triangles or of the area; the rest is debris, reported and with `keep_debris` kept as one mesh. Reports per part the triangles, area, bounding box, whether it is closed and its genus. Closed parts that lie inside another closed part are reported as nested: `cavity` (the inner skin of one hollow part) or `inclusion` (a separate part inside another). The original stays untouched; one undo step removes the new meshes.",
    SplitIn,
    SplitOut,
    split
)];
