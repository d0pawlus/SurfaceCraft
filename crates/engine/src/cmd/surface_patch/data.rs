//! The scan data of `surface.patch` and the parameters (u, v) the fit starts from: the vertices of a
//! mesh region, and, when the sides are the region's own boundary, its flattening.
use super::PatchIn;
use super::sides::{Resolved, V2, V3, dist};
use crate::Session;
use crate::cmd::{CmdError, CmdResult};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::param::{CANCELLED as FLATTEN_CANCELLED, flatten_cancellable};
use surfacecraft_geom::{Cancel, KdTree, Mesh, Point3};

/// Local/global steps of the flattening.
const ARAP_ITERATIONS: usize = 20;
/// Most triangles of a region that is flattened: 100 000 take about 3.4 s (20 000: 0.3 s), 400 000
/// about 47 s (aarch64, `ci` profile). A larger region is fitted on the plane of the outline.
const MAX_FLATTEN_TRIANGLES: usize = 100_000;

/// The mesh and the triangles the data come from.
pub(super) struct Scan {
    mesh: Arc<Mesh>,
    triangles: Vec<u32>,
    /// The region, when the data were given as one.
    pub region: Option<ObjectId>,
}

/// What the fit is given of the scan.
pub(super) struct Gathered {
    /// The data points (thinned to the limit).
    pub data: Vec<V3>,
    /// Their parameters, with a flattening.
    pub data_uv: Option<Vec<V2>>,
    /// Parameters of every side's given points, with a flattening.
    pub side_uv: Option<Vec<Vec<V2>>>,
}

impl Gathered {
    fn none() -> Self {
        Self { data: Vec::new(), data_uv: None, side_uv: None }
    }
}

/// The scan the request names: a region, or a mesh with triangles (all when omitted); `None` when
/// it names none.
pub(super) fn resolve_scan(s: &Session, i: &PatchIn) -> CmdResult<Option<Scan>> {
    let (mesh_id, tris, region) = match (&i.region, &i.mesh) {
        (Some(_), Some(_)) => return Err(CmdError::Params("give `region` or `mesh`, not both".into())),
        (Some(r), None) => {
            if i.triangles.is_some() {
                return Err(CmdError::Params("`triangles` goes with `mesh`, not with `region`".into()));
            }
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            (reg.mesh, reg.triangles.to_vec(), Some(rid))
        }
        (None, Some(m)) => {
            let mid = m.resolve(s, ObjectKind::Mesh)?;
            let all = s.doc.mesh(mid).map_or(0, |m| m.mesh.triangle_count());
            let all = u32::try_from(all).map_err(|_| CmdError::Params("the mesh has too many triangles".into()))?;
            (mid, i.triangles.clone().unwrap_or_else(|| (0..all).collect()), None)
        }
        (None, None) => {
            if i.triangles.is_some() {
                return Err(CmdError::Params("`triangles` needs `mesh`".into()));
            }
            return Ok(None);
        }
    };
    let mesh =
        s.doc.mesh(mesh_id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh_id}")))?;
    if let Some(t) = tris.iter().find(|&&t| t as usize >= mesh.triangle_count()) {
        return Err(CmdError::Params(format!("triangle {t} out of range")));
    }
    Ok(Some(Scan { mesh, triangles: tris, region }))
}

/// Vertices of the triangles, ascending and once each.
fn vertex_ids(scan: &Scan) -> Vec<u32> {
    let mut ids: Vec<u32> =
        scan.triangles.iter().filter_map(|&t| scan.mesh.triangles.get(t as usize)).flatten().copied().collect();
    ids.sort_unstable();
    ids.dedup();
    ids
}

/// Vertices of the edges that belong to one triangle only: the boundary of the region.
fn boundary_vertices(scan: &Scan) -> Vec<u32> {
    let mut edges: Vec<(u32, u32)> = Vec::with_capacity(3 * scan.triangles.len());
    for tri in scan.triangles.iter().filter_map(|&t| scan.mesh.triangles.get(t as usize)) {
        let [a, b, c] = *tri;
        for (p, q) in [(a, b), (b, c), (c, a)] {
            edges.push(if p < q { (p, q) } else { (q, p) });
        }
    }
    edges.sort_unstable();
    let mut ids: Vec<u32> = Vec::new();
    for run in edges.chunk_by(|a, b| a == b) {
        if let [(p, q)] = run {
            ids.push(*p);
            ids.push(*q);
        }
    }
    ids.sort_unstable();
    ids.dedup();
    ids
}

fn point(scan: &Scan, v: u32) -> Option<Point3> {
    scan.mesh.positions.get(v as usize).copied()
}

/// At most `cap` indices spread evenly over `0..total`, both ends included.
fn thin(total: usize, cap: usize) -> Vec<usize> {
    if total <= cap || cap < 2 {
        return (0..total).collect();
    }
    (0..cap).map(|k| k * (total - 1) / (cap - 1)).collect()
}

/// The parameters of the given points of every side, when the sides are the boundary of the region
/// (every given point within `gap_tol` of a boundary vertex) and the region flattens without folds:
/// `(flattening, parameters of the sides)`. `None` otherwise, and then the fit projects on the plane
/// of the outline; a flattening that fails says why in `warnings`.
fn flattened(
    scan: &Scan,
    sides: &[Resolved],
    gap_tol: f64,
    warnings: &mut Vec<String>,
    cancel: &Cancel,
) -> Option<(surfacecraft_fit::param::Flattening, Vec<Vec<V2>>)> {
    let boundary = boundary_vertices(scan);
    let positions: Vec<Option<Point3>> = boundary.iter().map(|&v| point(scan, v)).collect();
    if boundary.len() < 3 || positions.iter().any(Option::is_none) {
        return None;
    }
    let cloud: Vec<Point3> = positions.into_iter().flatten().collect();
    let tree = KdTree::build(&cloud);
    // the nearest boundary vertex of every given point of every side
    let mut nearest: Vec<Vec<u32>> = Vec::with_capacity(sides.len());
    for side in sides {
        let mut ids = Vec::with_capacity(side.points.len());
        for p in &side.points {
            let (k, d2) = tree.nearest(&Point3::new(p[0], p[1], p[2]))?;
            let near = cloud.get(k)?;
            if !(d2.is_finite() && dist(*p, [near.x, near.y, near.z]) <= gap_tol) {
                return None;
            }
            ids.push(*boundary.get(k)?);
        }
        nearest.push(ids);
    }
    if scan.triangles.len() > MAX_FLATTEN_TRIANGLES {
        warnings.push(format!(
            "the region has {} triangles (at most {MAX_FLATTEN_TRIANGLES} are flattened): the plane of the outline is used",
            scan.triangles.len()
        ));
        return None;
    }
    let flat = match flatten_cancellable(&scan.mesh, &scan.triangles, ARAP_ITERATIONS, cancel) {
        Ok(f) => f,
        // a cancelled call is ended by the caller, which looks at the flag
        Err(why) if why == FLATTEN_CANCELLED => return None,
        Err(why) => {
            warnings.push(format!("the region could not be flattened ({why}): the plane of the outline is used"));
            return None;
        }
    };
    if flat.flipped > 0 {
        warnings.push(format!(
            "the flattening of the region folds ({} triangles): the plane of the outline is used",
            flat.flipped
        ));
        return None;
    }
    let side_uv: Option<Vec<Vec<V2>>> = nearest
        .iter()
        .map(|ids| ids.iter().map(|&v| flat.uv.get(v as usize).copied().flatten()).collect::<Option<Vec<V2>>>())
        .collect();
    Some((flat, side_uv?))
}

/// The data points of the scan (its region's vertices, thinned evenly to `max_data`), with
/// parameters and the parameters of the sides' given points when the sides are the region's boundary.
pub(super) fn gather(
    scan: Option<&Scan>,
    sides: &[Resolved],
    gap_tol: f64,
    max_data: usize,
    warnings: &mut Vec<String>,
    cancel: &Cancel,
) -> CmdResult<Gathered> {
    let Some(scan) = scan else { return Ok(Gathered::none()) };
    let ids = vertex_ids(scan);
    let flat = flattened(scan, sides, gap_tol, warnings, cancel);
    if cancel.is_cancelled() {
        return Err(crate::cmd::cancelled());
    }
    let mut data: Vec<V3> = Vec::with_capacity(ids.len());
    let mut uv: Vec<V2> = Vec::new();
    for &v in &ids {
        let Some(p) = point(scan, v) else { continue };
        if let Some((f, _)) = &flat {
            let Some(q) = f.uv.get(v as usize).copied().flatten() else { continue };
            uv.push(q);
        }
        data.push([p.x, p.y, p.z]);
    }
    let total = data.len();
    let keep = thin(total, max_data);
    if keep.len() < total {
        warnings.push(format!("the scan data were thinned evenly from {total} to {} points", keep.len()));
        data = keep.iter().filter_map(|&k| data.get(k).copied()).collect();
        uv = keep.iter().filter_map(|&k| uv.get(k).copied()).collect();
    }
    Ok(match flat {
        Some((_, side_uv)) => Gathered { data, data_uv: Some(uv), side_uv: Some(side_uv) },
        None => Gathered { data, data_uv: None, side_uv: None },
    })
}
