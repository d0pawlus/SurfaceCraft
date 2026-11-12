//! Exclusion masks of `surface.fit` (P8-H11): triangles of the region that are left out of the
//! fit and out of its deviation statistics, chosen by index or by volume. The region and the mesh
//! are not changed; the masks only decide which samples the fit sees.
//!
//! A triangle is inside a volume when its centroid is (the boundary is inclusive). A sample is a
//! mesh vertex of the region; it is excluded when every region triangle that uses it is excluded.
use super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// Most triangle indices, spheres or boxes one call may give.
const MAX_INDICES: usize = 10_000_000;
const MAX_VOLUMES: usize = 256;
/// Indices echoed in the result (the count is always given).
const ECHO_INDICES: usize = 100;

/// A sphere of space to leave out.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SphereIn {
    /// Centre of the sphere, mm.
    pub center: [f64; 3],
    /// Radius of the sphere, mm (zero or more).
    pub radius: f64,
}

/// A box of space to leave out.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BoxIn {
    /// Corner of the box with the smallest coordinates, mm.
    pub min: [f64; 3],
    /// Corner of the box with the largest coordinates, mm.
    pub max: [f64; 3],
}

/// The masks that were applied, and what they took out of the region.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ExclusionOut {
    /// Triangles of the region left out of the fit (by index or by volume).
    pub triangles: usize,
    /// Of those, the ones named in `exclude_triangles`.
    pub by_index: usize,
    /// Of those, the ones whose centroid is inside a sphere or box (both may name one triangle).
    pub by_volume: usize,
    /// Indices of `exclude_triangles` that are not triangles of the region: ignored.
    pub indices_ignored: usize,
    /// Points of the region left out: those whose triangles are all excluded.
    pub points: usize,
    /// How many indices `exclude_triangles` had.
    pub indices_given: usize,
    /// The first indices of `exclude_triangles` (at most 100).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub indices: Vec<u32>,
    /// The spheres used.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub spheres: Vec<SphereIn>,
    /// The boxes used.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub boxes: Vec<BoxIn>,
}

/// The triangles of the mesh that are left out.
pub(super) struct Mask {
    /// Per triangle of the mesh; empty for no mask.
    excluded: Vec<bool>,
}

fn finite3(v: &[f64; 3]) -> bool {
    v.iter().all(|x| x.is_finite())
}

fn check_volumes(spheres: &[SphereIn], boxes: &[BoxIn]) -> CmdResult<()> {
    if spheres.len() > MAX_VOLUMES || boxes.len() > MAX_VOLUMES {
        return Err(CmdError::Params(format!("at most {MAX_VOLUMES} spheres and {MAX_VOLUMES} boxes")));
    }
    for s in spheres {
        if !(finite3(&s.center) && s.radius.is_finite() && s.radius >= 0.0) {
            return Err(CmdError::Params("exclude_spheres: centre must be finite and radius zero or more".into()));
        }
    }
    for b in boxes {
        let ordered = b.min.iter().zip(&b.max).all(|(lo, hi)| lo <= hi);
        if !(finite3(&b.min) && finite3(&b.max) && ordered) {
            return Err(CmdError::Params("exclude_boxes: corners must be finite and `min` not above `max`".into()));
        }
    }
    Ok(())
}

fn check_indices(mesh: &Mesh, indices: &[u32]) -> CmdResult<()> {
    if indices.len() > MAX_INDICES {
        return Err(CmdError::Params(format!("exclude_triangles: at most {MAX_INDICES} indices")));
    }
    match indices.iter().find(|&&t| t as usize >= mesh.triangle_count()) {
        Some(t) => Err(CmdError::Params(format!("exclude_triangles: triangle {t} out of range"))),
        None => Ok(()),
    }
}

/// The excluded flags per triangle of the mesh and `[excluded, of them named, of them in a
/// volume]` over the triangles of `region`; `named` flags the triangles of `exclude_triangles`.
fn select(
    mesh: &Mesh,
    region: &[u32],
    named: &[bool],
    spheres: &[SphereIn],
    boxes: &[BoxIn],
) -> (Vec<bool>, [usize; 3]) {
    let mut excluded = vec![false; mesh.triangle_count()];
    let mut tally = [0usize; 3];
    for &t in region {
        let is_named = named.get(t as usize).copied().unwrap_or(false);
        let is_inside = centroid(mesh, t).is_some_and(|c| inside(&c, spheres, boxes));
        let Some(flag) = excluded.get_mut(t as usize) else { continue };
        if (is_named || is_inside) && !*flag {
            *flag = true;
            tally = [tally[0] + 1, tally[1] + usize::from(is_named), tally[2] + usize::from(is_inside)];
        }
    }
    (excluded, tally)
}

fn centroid(mesh: &Mesh, t: u32) -> Option<Point3> {
    let [a, b, c] = mesh.triangle(t as usize)?;
    Some(Point3::from((a.coords + b.coords + c.coords) / 3.0))
}

fn inside(p: &Point3, spheres: &[SphereIn], boxes: &[BoxIn]) -> bool {
    spheres.iter().any(|s| (p - Point3::new(s.center[0], s.center[1], s.center[2])).norm() <= s.radius)
        || boxes.iter().any(|b| (0..3).all(|k| p[k] >= b.min[k] && p[k] <= b.max[k]))
}

/// Sorted, distinct corner ids of `tris` that `keep` accepts.
fn corner_ids(mesh: &Mesh, tris: impl Iterator<Item = u32>, keep: impl Fn(u32) -> bool) -> Vec<u32> {
    let mut ids: Vec<u32> = tris
        .filter(|&t| keep(t))
        .filter_map(|t| mesh.triangles.get(t as usize))
        .flat_map(|t| t.iter().copied())
        .filter(|&v| (v as usize) < mesh.positions.len())
        .collect();
    ids.sort_unstable();
    ids.dedup();
    ids
}

impl Mask {
    /// No triangle is excluded.
    pub fn none() -> Self {
        Self { excluded: Vec::new() }
    }

    /// The mask of `exclude_triangles`, `exclude_spheres` and `exclude_boxes` over the triangles
    /// `region` of `mesh`, with the report of what it took out; `None` for the report when no
    /// mask was asked for. Bad input is an error: an index beyond the mesh, a non-finite or
    /// negative number, an inverted box, too long a list.
    pub fn build(
        mesh: &Mesh,
        region: &[u32],
        indices: Option<&[u32]>,
        spheres: Option<&[SphereIn]>,
        boxes: Option<&[BoxIn]>,
    ) -> CmdResult<(Self, Option<ExclusionOut>)> {
        let (indices, spheres, boxes) = (indices.unwrap_or(&[]), spheres.unwrap_or(&[]), boxes.unwrap_or(&[]));
        if indices.is_empty() && spheres.is_empty() && boxes.is_empty() {
            return Ok((Self::none(), None));
        }
        check_indices(mesh, indices)?;
        check_volumes(spheres, boxes)?;
        let mut named = vec![false; mesh.triangle_count()];
        for &t in indices {
            if let Some(f) = named.get_mut(t as usize) {
                *f = true;
            }
        }
        let (excluded, [total, by_index, by_volume]) = select(mesh, region, &named, spheres, boxes);
        let distinct_named = named.iter().filter(|&&n| n).count();
        let mask = Self { excluded };
        let points = region_vertex_count(mesh, region).saturating_sub(mask.kept_ids(mesh, region).len());
        let report = ExclusionOut {
            triangles: total,
            by_index,
            by_volume,
            indices_ignored: distinct_named.saturating_sub(by_index),
            points,
            indices_given: indices.len(),
            indices: indices.iter().copied().take(ECHO_INDICES).collect(),
            spheres: spheres.to_vec(),
            boxes: boxes.to_vec(),
        };
        Ok((mask, Some(report)))
    }

    fn is_excluded(&self, t: u32) -> bool {
        self.excluded.get(t as usize).copied().unwrap_or(false)
    }

    /// The triangles of `tris` that are not excluded.
    pub fn kept_triangles(&self, tris: &[u32]) -> Vec<u32> {
        tris.iter().copied().filter(|&t| !self.is_excluded(t)).collect()
    }

    /// Sorted, distinct vertices of the triangles of `tris` that are not excluded.
    pub fn kept_ids(&self, mesh: &Mesh, tris: &[u32]) -> Vec<u32> {
        corner_ids(mesh, tris.iter().copied(), |t| !self.is_excluded(t))
    }

    /// Points and normals of the vertices of the triangles of `tris` that are not excluded.
    pub fn points(&self, mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> (Vec<Point3>, Vec<Vec3>) {
        self.kept_ids(mesh, tris)
            .into_iter()
            .filter_map(|v| Some((*mesh.positions.get(v as usize)?, *normals.get(v as usize)?)))
            .unzip()
    }
}

/// Distinct vertices of the triangles of `tris`.
pub(super) fn region_vertex_count(mesh: &Mesh, tris: &[u32]) -> usize {
    corner_ids(mesh, tris.iter().copied(), |_| true).len()
}
