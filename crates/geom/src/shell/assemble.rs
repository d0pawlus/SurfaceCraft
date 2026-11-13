//! The shell from the offsets: the two layers and the strip along the rim.
use super::crossing::{self, Surface};
use super::strip::{self, Rim};
use super::topology::{Csr, Work};
use super::{ScanLayer, ThickenError, ThickenParams, ThickenReport, Thickened, topology};
use crate::{Bvh, Mesh, Point3, Vec3};

/// Triangles of the shell tested for crossing another triangle at most.
const MAX_CROSSING_SAMPLES: usize = 20_000;
/// Vertices at which the wall is measured at most.
const MAX_WALL_SAMPLES: usize = 20_000;

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

/// Median of the reductions of the thickness over the vertices that were reduced (0 when none).
fn median_reduction(scale: &[f64], thickness: f64) -> f64 {
    let mut r: Vec<f64> = scale.iter().filter(|&&s| s < 1.0).map(|s| thickness * (1.0 - s)).collect();
    r.sort_by(f64::total_cmp);
    r.get(r.len() / 2).copied().unwrap_or(0.0)
}

/// What the assembly reads besides the scan.
pub(super) struct Inputs<'a> {
    pub normals: &'a [Vec3],
    pub scale: &'a [f64],
    /// Offsets `(a, b)` of the outer and the inner layer.
    pub offsets: (f64, f64),
    pub boundary: &'a [(u32, u32)],
    pub rim: &'a Rim,
    pub adjacent: &'a Csr,
    /// Median edge length of the scan (mm).
    pub edge: f64,
    /// Diagonal of the bounding box of the scan (mm).
    pub size: f64,
    /// Vertices whose offset the feature size of the scan bounded.
    pub limited: usize,
}

pub(super) fn assemble(work: &Work, inp: &Inputs, params: &ThickenParams) -> Result<Thickened, ThickenError> {
    let n = work.positions.len();
    let rim = inp.rim;
    let (a, b) = inp.offsets;
    let (outer, inner) = (
        strip::layer(&work.positions, inp.normals, inp.scale, a),
        strip::layer(&work.positions, inp.normals, inp.scale, b),
    );
    let mesh = strip::shell_mesh(&work.triangles, &outer, &inner, rim, inp.boundary);
    check_closed(&mesh)?;
    let thickness = params.thickness.abs();
    let crossings = {
        let bvh = Bvh::build(&mesh);
        let surface = Surface::new(&work.positions, inp.adjacent, rim, thickness, inp.edge, inp.size);
        crossing::sample(&mesh, &bvh, &surface, MAX_CROSSING_SAMPLES)
    };
    let (min_wall, median_wall) = measured_wall(&outer, &inner, &work.triangles, params.thickness);
    let loops = topology::loop_lengths(n, inp.boundary);
    let scale = inp.scale;
    let report = ThickenReport {
        vertices: mesh.positions.len(),
        triangles: mesh.triangles.len(),
        boundary_loops: loops.len(),
        longest_loop_edges: loops.iter().copied().max().unwrap_or(0),
        boundary_edges: inp.boundary.len(),
        strip_rows: rim.rows,
        pinch_splits: work.pinch_splits,
        dropped_triangles: work.dropped,
        removed_non_manifold: work.removed,
        flipped_triangles: work.flipped,
        clamped_vertices: scale.iter().filter(|&&s| s < 1.0).count(),
        limited_vertices: inp.limited,
        repaired_vertices: work.repaired,
        largest_reduction: thickness * (1.0 - scale.iter().copied().fold(1.0, f64::min)),
        median_reduction: median_reduction(scale, thickness),
        folded_vertices: work.folded,
        unresolved_vertices: work.unresolved,
        min_wall,
        median_wall,
        crossing_samples: crossings.tested,
        crossing_local: crossings.local,
        crossing_triangles: crossings.nonlocal,
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
