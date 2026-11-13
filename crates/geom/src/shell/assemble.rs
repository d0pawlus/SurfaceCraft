//! The shell from the offsets: the two layers and the strip along the rim.
use super::topology::Work;
use super::{ScanLayer, ThickenError, ThickenParams, ThickenReport, Thickened, topology};
use crate::{Bvh, Mesh, MeshError, Point3, Vec3, nesting};

/// Rows of the strip along the rim at most.
const MAX_STRIP_ROWS: usize = 64;
/// Triangles of the strip at most (the rows are cut back for a long rim).
const MAX_STRIP_TRIANGLES: usize = 4_000_000;
/// Triangles of the shell tested for crossing another triangle at most.
const MAX_CROSSING_SAMPLES: usize = 20_000;
/// Vertices at which the wall is measured at most.
const MAX_WALL_SAMPLES: usize = 20_000;

/// Mean length of the boundary edges (mm; 0 without any).
fn mean_rim_edge(positions: &[Point3], boundary: &[(u32, u32)]) -> f64 {
    let total: f64 = boundary
        .iter()
        .filter_map(|&(a, b)| Some((positions.get(a as usize)? - positions.get(b as usize)?).norm()))
        .sum();
    if boundary.is_empty() { 0.0 } else { total / boundary.len() as f64 }
}

/// Rows of the strip: about one edge length each, so no triangle of it is a sliver, within
/// [`MAX_STRIP_ROWS`] and [`MAX_STRIP_TRIANGLES`].
fn strip_rows(thickness: f64, rim_edge: f64, rim_edges: usize) -> usize {
    let by_size = if rim_edge > 0.0 { (thickness.abs() / rim_edge).ceil() as usize } else { 1 };
    let by_count = MAX_STRIP_TRIANGLES / (2 * rim_edges.max(1));
    by_size.clamp(1, MAX_STRIP_ROWS).min(by_count.max(1))
}

/// Ids of the vertices of the strip: layer `k` of rim vertex `v`, `0` the outer layer and `rows` the
/// inner one; the layers between come after the two copies of the mesh's vertices.
struct Rim {
    vertices: u32,
    rows: usize,
    slot: Vec<u32>,
    rim_count: usize,
}

impl Rim {
    fn id(&self, v: u32, k: usize) -> u32 {
        if k == 0 {
            return v;
        }
        if k >= self.rows {
            return v + self.vertices;
        }
        let slot = self.slot.get(v as usize).copied().unwrap_or(0) as usize;
        (2 * self.vertices as usize + (k - 1) * self.rim_count + slot) as u32
    }
}

fn rim_of(vertices: u32, rows: usize, boundary: &[(u32, u32)]) -> (Rim, Vec<u32>) {
    let mut slot = vec![u32::MAX; vertices as usize];
    let mut order: Vec<u32> = Vec::new();
    let mut starts: Vec<u32> = boundary.iter().map(|&(a, _)| a).collect();
    starts.sort_unstable();
    for v in starts {
        if let Some(s) = slot.get_mut(v as usize) {
            *s = order.len() as u32;
            order.push(v);
        }
    }
    (Rim { vertices, rows, slot, rim_count: order.len() }, order)
}

/// The vertices of the layer at `offset` (each moved along its normal by its share of the offset).
fn layer(work: &Work, normals: &[Vec3], scale: &[f64], offset: f64) -> Vec<Point3> {
    (0..work.positions.len())
        .map(|i| match (work.positions.get(i), normals.get(i), scale.get(i)) {
            (Some(p), Some(nm), Some(s)) => p + nm * (offset * s),
            _ => Point3::origin(),
        })
        .collect()
}

/// Wall thickness `(minimum, median)` of the shell, sampled: the distance from vertices of one layer
/// to the surface of the other (mm).
fn measured_wall(outer: &[Point3], inner: &[Point3], triangles: &[[u32; 3]], thickness: f64) -> (f64, f64) {
    let sheet = Bvh::build(&Mesh { positions: inner.to_vec(), triangles: triangles.to_vec() });
    let step = (outer.len() / MAX_WALL_SAMPLES).max(1);
    let reach = 2.0 * thickness.abs();
    let mut walls: Vec<f64> =
        outer.iter().step_by(step).filter_map(|p| sheet.closest(p, reach)).map(|h| h.distance).collect();
    walls.sort_by(f64::total_cmp);
    (walls.first().copied().unwrap_or(0.0), walls.get(walls.len() / 2).copied().unwrap_or(0.0))
}

/// The triangles: the outer layer, the inner layer reversed, the strip.
fn shell_triangles(work: &Work, rim: &Rim, boundary: &[(u32, u32)]) -> Vec<[u32; 3]> {
    let n = rim.vertices;
    let mut triangles = work.triangles.clone();
    triangles.extend(work.triangles.iter().map(|t| [t[0] + n, t[2] + n, t[1] + n]));
    for &(u, v) in boundary {
        for k in 0..rim.rows {
            let (ou, ov, iu, iv) = (rim.id(u, k), rim.id(v, k), rim.id(u, k + 1), rim.id(v, k + 1));
            triangles.push([ov, ou, iu]);
            triangles.push([ov, iu, iv]);
        }
    }
    triangles
}

/// Median of the reductions of the thickness over the vertices that were reduced (0 when none).
fn median_reduction(scale: &[f64], thickness: f64) -> f64 {
    let mut r: Vec<f64> = scale.iter().filter(|&&s| s < 1.0).map(|s| thickness * (1.0 - s)).collect();
    r.sort_by(f64::total_cmp);
    r.get(r.len() / 2).copied().unwrap_or(0.0)
}

pub(super) fn assemble(
    work: &Work,
    normals: &[Vec3],
    scale: &[f64],
    (a, b): (f64, f64),
    boundary: &[(u32, u32)],
    params: &ThickenParams,
) -> Result<Thickened, ThickenError> {
    let n = work.positions.len();
    let rows = strip_rows(params.thickness, mean_rim_edge(&work.positions, boundary), boundary.len());
    let total = 2 * n + (rows - 1) * boundary.len();
    let n32 = u32::try_from(n).ok().filter(|_| u32::try_from(total).is_ok());
    let Some(n32) = n32 else { return Err(ThickenError::Invalid(MeshError::TooManyVertices(total))) };
    let (outer, inner) = (layer(work, normals, scale, a), layer(work, normals, scale, b));
    let (rim, order) = rim_of(n32, rows, boundary);
    let mut positions = outer.clone();
    positions.extend_from_slice(&inner);
    for k in 1..rows {
        let f = k as f64 / rows as f64;
        for &v in &order {
            let (Some(o), Some(i)) = (outer.get(v as usize), inner.get(v as usize)) else { continue };
            positions.push(Point3::from(o.coords * (1.0 - f) + i.coords * f));
        }
    }
    let mesh = Mesh { positions, triangles: shell_triangles(work, &rim, boundary) };
    check_closed(&mesh)?;
    let crossings = {
        let bvh = Bvh::build(&mesh);
        nesting::sampled_self_intersections(&mesh, &bvh, MAX_CROSSING_SAMPLES)
    };
    let (min_wall, median_wall) = measured_wall(&outer, &inner, &work.triangles, params.thickness);
    let loops = topology::loop_lengths(n, boundary);
    let thickness = params.thickness.abs();
    let report = ThickenReport {
        vertices: mesh.positions.len(),
        triangles: mesh.triangles.len(),
        boundary_loops: loops.len(),
        longest_loop_edges: loops.iter().copied().max().unwrap_or(0),
        boundary_edges: boundary.len(),
        strip_rows: rows,
        pinch_splits: work.pinch_splits,
        dropped_triangles: work.dropped,
        removed_non_manifold: work.removed,
        flipped_triangles: work.flipped,
        clamped_vertices: scale.iter().filter(|&&s| s < 1.0).count(),
        largest_reduction: thickness * (1.0 - scale.iter().copied().fold(1.0, f64::min)),
        median_reduction: median_reduction(scale, thickness),
        folded_vertices: work.folded,
        unresolved_vertices: work.unresolved,
        min_wall,
        median_wall,
        crossing_samples: crossings.tested,
        crossing_triangles: crossings.crossing,
    };
    let scan_layer = if !params.symmetric && params.thickness < 0.0 { ScanLayer::Inner } else { ScanLayer::Outer };
    Ok(Thickened {
        mesh,
        report,
        scan_layer,
        scale: scale.to_vec(),
        layer_vertices: n,
        layer_triangles: work.triangles.len(),
    })
}

/// The shell is closed (every edge has its opposite) and encloses a positive volume.
fn check_closed(mesh: &Mesh) -> Result<(), ThickenError> {
    let sorted = topology::directed_edges(&mesh.triangles);
    if topology::repeated_edges(&sorted) > 0 || !topology::boundary_edges(&sorted).is_empty() {
        return Err(ThickenError::NotClosed("an edge has no partner"));
    }
    let origin = mesh.positions.first().copied().unwrap_or_else(Point3::origin);
    let volume: f64 = mesh
        .triangles
        .iter()
        .filter_map(|t| {
            let a = mesh.positions.get(t[0] as usize)? - origin;
            let b = mesh.positions.get(t[1] as usize)? - origin;
            let c = mesh.positions.get(t[2] as usize)? - origin;
            Some(a.dot(&b.cross(&c)) / 6.0)
        })
        .sum();
    if !volume.is_finite() {
        Err(ThickenError::NotClosed("its volume is not a number"))
    } else if volume < 0.0 {
        Err(ThickenError::NotClosed("it is turned inside out"))
    } else if volume == 0.0 {
        Err(ThickenError::NotClosed("it has no volume: the wall is thinner than the coordinates resolve"))
    } else {
        Ok(())
    }
}
