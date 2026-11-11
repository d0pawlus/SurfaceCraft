//! `mesh.working_*`: working copies (non-destructive editing). `begin` starts an editable copy of a
//! mesh; the edit commands act on it until it is promoted (it replaces the original) or discarded.
//! `distance` measures the copy against the original, both ways, with the deviation statistics.
use super::mesh::MeshIn;
use super::{CmdError, CmdResult, CommandSpec, ObjRef, write_file};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::fmt::Write as _;
use std::sync::Arc;
use surfacecraft_deviation::{Report, ReportOptions, Sample, report, samples_mesh};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::{Bvh, Mesh, Point3};

/// Bins of the distance histogram.
const HISTOGRAM_BINS: usize = 20;
/// Header of the per-vertex CSV written by `mesh.working_distance`.
const CSV_HEADER: &str = "index,x,y,z,deviation,distance";

/// Size of a working copy.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct WorkingOut {
    /// Object id.
    pub id: ObjectId,
    /// Triangles of the copy.
    pub triangles: usize,
    /// Vertices of the copy.
    pub vertices: usize,
}

/// What `mesh.working_promote` did.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PromoteOut {
    /// Object id.
    pub id: ObjectId,
    /// Triangles of the original before the promotion.
    pub triangles_before: usize,
    /// Triangles of the original now (the former copy).
    pub triangles: usize,
    /// Regions dropped because the triangles changed.
    pub regions_dropped: usize,
}

/// Distance measurement parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DistanceIn {
    /// Mesh id or name (its working copy is measured against its original).
    pub object: ObjRef,
    /// Tolerance, mm: the report gives the share of the vertices within it.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// CSV file for the per-vertex map (`index,x,y,z,deviation,distance`; the deviation is signed,
    /// positive where the copy lies outside the original). Omitted: the summary only.
    #[serde(default)]
    pub path: Option<String>,
}

/// The distance between a working copy and its original, in mm. Distances are unsigned.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct DistanceOut {
    /// Object id.
    pub id: ObjectId,
    /// Vertices of the copy to the original surface: the distance map (max, RMS, percentiles).
    pub working_to_original: Report,
    /// Vertices of the original to the copy surface: shows material the copy removed.
    pub original_to_working: Report,
    /// The CSV written, if `path` was given.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
}

fn no_working(id: ObjectId) -> CmdError {
    CmdError::Params(format!("mesh {id} has no working copy; start one with mesh.working_begin"))
}

fn copy_of(s: &Session, id: ObjectId) -> CmdResult<Arc<Mesh>> {
    s.doc.mesh(id).and_then(|o| o.working.as_ref()).map(|w| Arc::clone(&w.mesh)).ok_or_else(|| no_working(id))
}

fn working_out(id: ObjectId, m: &Mesh) -> WorkingOut {
    WorkingOut { id, triangles: m.triangle_count(), vertices: m.vertex_count() }
}

fn begin(s: &mut Session, i: MeshIn) -> CmdResult<WorkingOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    if s.doc.mesh(id).is_some_and(|o| o.is_working()) {
        return Err(CmdError::Params(format!("mesh {id} has a working copy already; promote or discard it first")));
    }
    if !s.doc.start_working(id) {
        return Err(CmdError::Params(format!("no mesh {id}")));
    }
    let copy = copy_of(s, id)?;
    Ok(working_out(id, &copy))
}

fn promote(s: &mut Session, i: MeshIn) -> CmdResult<PromoteOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let triangles_before = s.doc.mesh(id).map_or(0, |o| o.mesh.triangle_count());
    let triangles = copy_of(s, id)?.triangle_count();
    let regions_dropped = s.doc.promote_working(id).ok_or_else(|| no_working(id))?;
    s.overlays.remove(&id);
    Ok(PromoteOut { id, triangles_before, triangles, regions_dropped })
}

fn discard(s: &mut Session, i: MeshIn) -> CmdResult<WorkingOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let copy = s.doc.discard_working(id).ok_or_else(|| no_working(id))?;
    Ok(working_out(id, &copy.mesh))
}

/// Vertices a triangle uses, in index order.
fn used_vertices(m: &Mesh) -> Vec<u32> {
    let mut used = vec![false; m.positions.len()];
    for t in &m.triangles {
        for &v in t {
            if let Some(u) = used.get_mut(v as usize) {
                *u = true;
            }
        }
    }
    used.iter().enumerate().filter(|(_, u)| **u).map(|(i, _)| i as u32).collect()
}

/// The vertices of `from` that triangles use, each with its signed distance to the surface of `to`.
struct Measured {
    vertices: Vec<u32>,
    signed: Vec<Sample>,
}

fn measure(from: &Mesh, to: &Mesh) -> Measured {
    let (vertices, points): (Vec<u32>, Vec<Point3>) =
        used_vertices(from).into_iter().filter_map(|v| from.positions.get(v as usize).map(|p| (v, *p))).unzip();
    let signed = samples_mesh(&points, &[], &Bvh::build(to), f64::INFINITY);
    Measured { vertices, signed }
}

/// The unsigned distances summarised: extremes, RMS, percentiles, a histogram from zero to the
/// largest distance, and the share within `tolerance` when one is given.
fn summarise(signed: &[Sample], tolerance: Option<f64>) -> CmdResult<Report> {
    let unsigned: Vec<Sample> = signed.iter().map(|s| Sample { d: s.d.abs(), ..*s }).collect();
    let top = unsigned.iter().filter(|s| s.counted()).fold(0.0_f64, |m, s| m.max(s.d));
    let opts = ReportOptions {
        bins: HISTOGRAM_BINS,
        range: Some([0.0, if top > 0.0 { top } else { 1.0 }]),
        bands: tolerance.into_iter().collect(),
    };
    report(&unsigned, &opts).map_err(CmdError::Params)
}

/// The per-vertex map as CSV: one row per vertex of the copy that triangles use.
fn map_csv(copy: &Mesh, m: &Measured) -> CmdResult<String> {
    let mut out = String::from(CSV_HEADER);
    out.push('\n');
    for (v, s) in m.vertices.iter().zip(&m.signed) {
        let Some(p) = copy.positions.get(*v as usize) else { continue };
        writeln!(out, "{v},{},{},{},{},{}", p.x, p.y, p.z, s.d, s.d.abs())
            .map_err(|e| CmdError::Failed(e.to_string()))?;
    }
    Ok(out)
}

fn positive_tolerance(tolerance: Option<f64>) -> CmdResult<Option<f64>> {
    match tolerance {
        None => Ok(None),
        Some(t) if t.is_finite() && t > 0.0 => Ok(Some(t)),
        Some(_) => Err(CmdError::Params("tolerance must be a positive number".into())),
    }
}

fn distance(s: &mut Session, i: DistanceIn) -> CmdResult<DistanceOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let tolerance = positive_tolerance(i.tolerance)?;
    let copy = copy_of(s, id)?;
    let original = s.doc.mesh(id).map(|o| Arc::clone(&o.mesh)).ok_or_else(|| no_working(id))?;
    let forward = measure(&copy, &original);
    let backward = measure(&original, &copy);
    let out = DistanceOut {
        id,
        working_to_original: summarise(&forward.signed, tolerance)?,
        original_to_working: summarise(&backward.signed, tolerance)?,
        path: i.path.clone(),
    };
    if let Some(p) = &i.path {
        write_file(s, p, map_csv(&copy, &forward)?.as_bytes())?;
    }
    Ok(out)
}

/// `mesh.working_*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "mesh.working_begin",
        "Start working copy",
        "Mesh",
        true,
        "Starts an editable copy of `object` beside the original. While it exists the edit commands (clean, decimate, smooth, fill holes…) act on the copy and the original stays as it was.",
        MeshIn,
        WorkingOut,
        begin
    ),
    command!(
        "mesh.working_distance",
        "Working copy distance",
        "Mesh",
        false,
        "Measures the working copy of `object` against the original, both ways: unsigned distance summary (max, RMS, percentiles, histogram), the share within `tolerance` when given, and a per-vertex CSV at `path`.",
        DistanceIn,
        DistanceOut,
        distance
    ),
    command!(
        "mesh.working_promote",
        "Promote working copy",
        "Mesh",
        true,
        "Replaces the original of `object` with its working copy. Regions of the old triangles are dropped when the triangles changed.",
        MeshIn,
        PromoteOut,
        promote
    ),
    command!(
        "mesh.working_discard",
        "Discard working copy",
        "Mesh",
        true,
        "Drops the working copy of `object`; the original is kept.",
        MeshIn,
        WorkingOut,
        discard
    ),
];
