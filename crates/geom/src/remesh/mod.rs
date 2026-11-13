//! `remesh`: a clean, closed, manifold, consistently oriented surface rebuilt from a messy closed
//! scan, at a resolution tied to the tolerance (design: P9-O9).
//!
//! Method, all steps published and deterministic:
//! 1. holes up to `max_hole_edges` are filled ([`crate::edit::fill_holes`]); a scan that still has an
//!    open boundary is refused (a signed field cannot be defined across a large opening, and no skin
//!    is invented there);
//! 2. the solid is scan-converted onto a sparse voxel grid, column by column (`raster`: Nooruddin and
//!    Turk 2003), so that duplicate shells, fins and non-manifold edges cost a column, not the model;
//! 3. closing then opening by a cube (`occupancy`) removes tunnels, bridges and spurs at most
//!    `min_feature` wide, i.e. the topological noise, and keeps every wall of at least `2k + 1` voxels
//!    (a thinner wall in the scan makes the voxel finer, or the command refuse, never a merged sheet);
//! 4. surface nets (`surface`: Gibson 1998) extract the boundary of the solid; vertices that join
//!    several fans are split, so every vertex and edge is manifold;
//! 5. Taubin smoothing and a robust moving-least-squares projection onto the scan vertices (`fit`)
//!    put the vertices on the scan, whatever the voxel size.
//!
//! Memory is bounded and checked before it is spent: eight bytes per ray crossing
//! ([`Limits::max_crossings`]), four bytes per grid column plus eight per run ([`Limits::max_columns`]),
//! and about 60 bytes per surface quad ([`Limits::max_quads`]). A scan of 1 million triangles and
//! 300 mm at 0.3 mm voxels has about 1.1 million columns, 3 million crossings and 1.5 million quads: roughly
//! 24 MB for the crossings plus 4.4 MB of column index, 90 MB for the quads: never a dense grid of 10^9 voxels.
mod fit;
mod occupancy;
mod raster;
mod surface;

use crate::{Aabb, Bvh, KdTree, Mesh, Point3, edit, par, pieces};
pub use occupancy::Occupancy;
pub use surface::split_pinched_vertices;

/// Voxels around the part (room for the closing and the surface).
const MIN_BORDER: usize = 3;
/// Largest half-size of the closing and opening cube (voxels).
const MAX_K: u32 = 4;
/// Grid axes may have at most this many cells (the cell key has 21 bits per axis).
const MAX_AXIS: usize = (1 << 21) - 4;
/// Samples of the surface probed for its thickness.
const PROBE_SAMPLES: usize = 2000;
/// A wall must be this many voxels thick beyond the closing: `2k + 2`.
const THIN_EXTRA_VOXELS: f64 = 2.0;
/// Share of the probes allowed to find a thinner wall (those are detail, not the wall).
const THIN_PERCENTILE: usize = 2;
/// Closing rounds that remove checkerboard voxel blocks.
const DIAGONAL_ROUNDS: usize = 12;
/// Samples of each surface in the deviation report.
const DEVIATION_SAMPLES: usize = 100_000;
/// Median edge lengths of the scan that the projection looks around.
const FIT_RADIUS_EDGES: f64 = 3.2;
/// Largest projection radius (mm).
const FIT_RADIUS_MAX: f64 = 12.0;
/// Vertices of the original that may lie farther than the lost distance before the result is refused.
const MAX_LOST_SHARE: f64 = 0.005;
/// Irregular columns above this share of the occupied ones: the scan is not consistently oriented.
const MAX_IRREGULAR_SHARE: f64 = 0.25;

/// Resource limits. Exceeding one refuses the remesh before the memory is spent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Limits {
    /// Columns of the grid (`nx · ny`).
    pub max_columns: usize,
    /// Ray crossings held at once (8 bytes each).
    pub max_crossings: usize,
    /// Quads of the surface (about 60 bytes each while it is built).
    pub max_quads: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self { max_columns: 16_000_000, max_crossings: 40_000_000, max_quads: 6_000_000 }
    }
}

/// Settings of [`remesh`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    /// Tolerance the result is held to against the scan (mm); sets the default voxel and the
    /// robustness of the projection.
    pub tolerance: f64,
    /// Voxel edge (mm). `None`: `1.5 × tolerance`, made finer when the scan has a thin wall.
    pub voxel: Option<f64>,
    /// Tunnels, bridges and spurs at most about this wide are removed (mm). `None`: two voxels.
    pub min_feature: Option<f64>,
    /// Holes up to this many edges (and `max_hole` wide) are filled first.
    pub max_hole_edges: usize,
    /// Holes whose loop is at most this wide (perimeter over pi, mm) are filled first; wider
    /// openings stay and the scan is refused.
    pub max_hole: f64,
    /// Pieces with fewer triangles than this share of the largest are debris and dropped.
    pub min_component_fraction: f64,
    /// Resource limits.
    pub limits: Limits,
}

impl Default for Params {
    fn default() -> Self {
        Self {
            tolerance: 0.2,
            voxel: None,
            min_feature: None,
            max_hole_edges: 400,
            max_hole: 5.0,
            min_component_fraction: 0.01,
            limits: Limits::default(),
        }
    }
}

/// Why a scan was not remeshed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum RemeshError {
    /// A setting is out of range.
    #[error("{0}")]
    Params(String),
    /// Nothing to remesh.
    #[error("the mesh has no usable triangles")]
    Empty,
    /// The scan has an open boundary after the small holes were filled.
    #[error(
        "the scan is open: {loops} boundary loops remain after filling holes of up to {max_hole_edges} edges and {max_hole} mm, the largest with {largest_edges} edges ({length:.1} mm long); remeshing builds a closed solid and does not close large openings"
    )]
    Open {
        /// Loops left.
        loops: usize,
        /// Edges of the largest.
        largest_edges: usize,
        /// Length of the largest (mm).
        length: f64,
        /// The hole limit used (edges).
        max_hole_edges: usize,
        /// The hole limit used (mm).
        max_hole: f64,
    },
    /// A wall is thinner than the voxel resolves.
    #[error(
        "the scan has walls about {thickness:.3} mm thick, thinner than the {required:.3} mm that voxels of {voxel} mm keep; use a voxel of at most {suggested:.3} mm or leave `voxel` unset"
    )]
    ThinWall {
        /// Thickness found (mm).
        thickness: f64,
        /// Thickness the voxel resolves (mm).
        required: f64,
        /// The voxel asked for (mm).
        voxel: f64,
        /// A voxel that would do (mm).
        suggested: f64,
    },
    /// A resource limit would be exceeded.
    #[error("{what}: {needed} needed, the limit is {limit}; {advice}")]
    TooMuchWork {
        /// What would grow too large.
        what: String,
        /// How much.
        needed: usize,
        /// The limit.
        limit: usize,
        /// What to do.
        advice: String,
    },
    /// The triangles are not consistently oriented (or the scan is not closed in a way the fill
    /// could see): the solid cannot be read from them.
    #[error(
        "{share:.0} % of the columns of the scan do not enter and leave the solid in turn: the triangles are not consistently oriented or the scan is not closed"
    )]
    Irregular {
        /// Percent of the columns.
        share: f64,
    },
    /// No solid came out.
    #[error("the scan encloses no volume at this voxel size")]
    NoSolid,
    /// The surface could not be made manifold.
    #[error("the surface could not be made manifold: {0}")]
    NotManifold(String),
    /// The remesh lost part of the scan (a wall thinner than the closing, or an inconsistent scan).
    #[error(
        "{share:.1} % of the scan lies more than {distance:.2} mm from the remeshed surface (a wall thinner than {width:.2} mm was lost); use a finer voxel"
    )]
    Lost {
        /// Percent of the scan vertices.
        share: f64,
        /// The distance used (mm).
        distance: f64,
        /// The size of feature removed (mm).
        width: f64,
    },
}

/// The grid: voxel `(i, j, k)` has its centre at `origin + (i + ½, j + ½, k + ½) · h`.
#[derive(Clone, Copy, Debug)]
pub struct Grid {
    /// Lower corner.
    pub origin: Point3,
    /// Voxel edge (mm).
    pub h: f64,
    /// Cells along x.
    pub nx: usize,
    /// Cells along y.
    pub ny: usize,
    /// Cells along z.
    pub nz: usize,
}

impl Grid {
    /// A grid of voxels of `h` around `bbox` with `border` empty voxels on every side.
    pub fn new(bbox: &Aabb, h: f64, border: usize) -> Result<Self, RemeshError> {
        let size = bbox.size();
        let cells = |s: f64| (s / h).ceil() as usize + 2 * border + 1;
        let (nx, ny, nz) = (cells(size.x), cells(size.y), cells(size.z));
        let origin = bbox.min - crate::Vec3::new(1.0, 1.0, 1.0) * (border as f64 * h);
        if nx > MAX_AXIS || ny > MAX_AXIS || nz > MAX_AXIS {
            let needed = nx.max(ny).max(nz);
            return Err(RemeshError::TooMuchWork {
                what: "cells along one axis".into(),
                needed,
                limit: MAX_AXIS,
                advice: "use a larger voxel".into(),
            });
        }
        Ok(Self { origin, h, nx, ny, nz })
    }
}

/// Distances between two surfaces in one direction (mm).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Deviation {
    /// Points measured.
    pub samples: usize,
    /// Median distance.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// 99th percentile.
    pub p99: f64,
    /// Largest distance.
    pub max: f64,
}

/// What a remesh did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RemeshReport {
    /// Voxel edge used (mm).
    pub voxel: f64,
    /// The voxel was made finer than the default because of a thin wall.
    pub voxel_refined: bool,
    /// Half-size of the closing and opening cube (voxels): features up to `2k` voxels wide go.
    pub k: u32,
    /// Tunnels, bridges and spurs up to this wide were removed (mm).
    pub feature_removed: f64,
    /// Thickness of the thinnest wall probed (mm), `None` when no probe hit.
    pub thickness: Option<f64>,
    /// Holes filled before the remesh.
    pub holes_filled: usize,
    /// Triangles of debris pieces dropped before the remesh.
    pub debris_triangles: usize,
    /// The scan's triangles were inside out and were read reversed.
    pub flipped: bool,
    /// Grid size.
    pub grid: [usize; 3],
    /// Columns with a crossing.
    pub columns: usize,
    /// Ray crossings.
    pub crossings: usize,
    /// Columns whose crossings did not alternate (fins, coincident shells, unclosed edges).
    pub irregular_columns: usize,
    /// Runs of the solid.
    pub runs: usize,
    /// Voxels of the solid after closing and opening.
    pub voxels: u64,
    /// Voxels added to resolve checkerboard contacts.
    pub diagonal_voxels: u64,
    /// Vertices added by splitting pinched vertices.
    pub pinches_split: usize,
    /// Triangles of islands dropped from the result.
    pub islands_removed: usize,
    /// Projection radius (mm).
    pub fit_radius: f64,
    /// Vertices the projection could not move (too few scan vertices around them).
    pub unfitted: usize,
    /// Triangles of the result.
    pub triangles: usize,
    /// Genus per connected piece of the input after hole filling, `None` where it is not defined.
    pub genus_before: Vec<Option<usize>>,
    /// Genus per connected piece of the result.
    pub genus_after: Vec<Option<usize>>,
    /// Non-manifold edges of the input.
    pub non_manifold_before: usize,
    /// Peak bytes the large buffers needed (estimate: crossings, column index, runs and quads).
    pub peak_bytes: usize,
    /// Original vertices to the remeshed surface.
    pub original_to_result: Deviation,
    /// Remeshed vertices to the original surface.
    pub result_to_original: Deviation,
}

fn check(p: &Params) -> Result<(), RemeshError> {
    let bad = |what: &str| Err(RemeshError::Params(format!("{what} must be a positive number")));
    if !(p.tolerance.is_finite() && p.tolerance > 0.0) {
        return bad("tolerance");
    }
    if p.voxel.is_some_and(|v| !(v.is_finite() && v > 0.0)) {
        return bad("voxel");
    }
    if p.min_feature.is_some_and(|v| !(v.is_finite() && v >= 0.0)) {
        return Err(RemeshError::Params("min_feature must not be negative".into()));
    }
    if !(p.max_hole.is_finite() && p.max_hole >= 0.0) {
        return Err(RemeshError::Params("max_hole must not be negative".into()));
    }
    if !(p.min_component_fraction.is_finite() && (0.0..=1.0).contains(&p.min_component_fraction)) {
        return Err(RemeshError::Params("min_component_fraction must be within 0..=1".into()));
    }
    Ok(())
}

/// Signed volume of the closed surface (positive when the triangles face outward).
fn signed_volume(mesh: &Mesh) -> f64 {
    (0..mesh.triangle_count())
        .filter_map(|i| mesh.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

/// A thickness of the walls: the low percentile of the distances from sampled vertices, along the
/// inward normal, to the surface on the other side.
fn thickness(mesh: &Mesh, bvh: &Bvh, h: f64) -> Option<f64> {
    let normals = mesh.vertex_normals();
    let n = mesh.positions.len();
    let count = PROBE_SAMPLES.min(n);
    let probe = 0.5 * h;
    let mut found: Vec<f64> = par::map(count, |k| {
        let i = k * n / count.max(1);
        let (p, nv) = (mesh.positions.get(i)?, normals.get(i)?);
        let dir = -crate::normalize(*nv)?;
        bvh.ray_cast(&(p + dir * probe), &dir).map(|(_, t)| t + probe)
    })
    .into_iter()
    .flatten()
    .collect();
    if found.is_empty() {
        return None;
    }
    found.sort_by(f64::total_cmp);
    found.get(found.len() * THIN_PERCENTILE / 100).copied()
}

/// Distances from a sample of `points` to the surface `bvh`.
fn deviation<'a>(points: impl Iterator<Item = &'a Point3>, bvh: &Bvh) -> Deviation {
    let all: Vec<&Point3> = points.collect();
    let count = DEVIATION_SAMPLES.min(all.len());
    let mut d: Vec<f64> = par::map(count, |k| {
        all.get(k * all.len() / count.max(1)).and_then(|p| bvh.closest(p, f64::INFINITY)).map_or(0.0, |h| h.distance)
    });
    d.sort_by(f64::total_cmp);
    let at = |q: usize| d.get((d.len().saturating_sub(1) * q) / 100).copied().unwrap_or(0.0);
    Deviation { samples: d.len(), p50: at(50), p95: at(95), p99: at(99), max: d.last().copied().unwrap_or(0.0) }
}

/// Width of a boundary loop: its perimeter over pi (the diameter of a round hole).
fn loop_width(mesh: &Mesh, lp: &[u32]) -> f64 {
    let perimeter: f64 = (0..lp.len())
        .filter_map(|k| {
            Some(
                (mesh.positions.get(*lp.get(k)? as usize)?
                    - mesh.positions.get(*lp.get((k + 1) % lp.len())? as usize)?)
                .norm(),
            )
        })
        .sum();
    perimeter / std::f64::consts::PI
}

/// The scan prepared for the solid: holes filled, debris dropped, open boundaries refused.
struct Prepared {
    work: Mesh,
    holes_filled: usize,
    debris_triangles: usize,
    genus: Vec<Option<usize>>,
    non_manifold: usize,
}

fn prepare(mesh: &Mesh, p: &Params) -> Result<Prepared, RemeshError> {
    mesh.validate().map_err(|e| RemeshError::Params(e.to_string()))?;
    let mut work = mesh.clone();
    work.remove_degenerate(0.0);
    if work.triangle_count() < 4 {
        return Err(RemeshError::Empty);
    }
    let wide = p.max_hole;
    let accept = move |m: &Mesh, lp: &[u32]| loop_width(m, lp) <= wide;
    let holes_filled = edit::fill_holes_where(&mut work, p.max_hole_edges, &accept).filled;
    let debris_triangles = edit::remove_small_components(&mut work, p.min_component_fraction, 8);
    let ps = pieces::pieces(&work);
    let loops: Vec<_> = ps.iter().flat_map(|q| q.loops.iter()).collect();
    if let Some(biggest) = loops.iter().max_by(|a, b| a.edges.cmp(&b.edges)) {
        let (largest_edges, length) = (biggest.edges, biggest.length);
        return Err(RemeshError::Open {
            loops: loops.len(),
            largest_edges,
            length,
            max_hole_edges: p.max_hole_edges,
            max_hole: p.max_hole,
        });
    }
    let genus = ps.iter().map(pieces::Piece::genus).collect();
    Ok(Prepared {
        work,
        holes_filled,
        debris_triangles,
        genus,
        non_manifold: ps.iter().map(|q| q.non_manifold_edges).sum(),
    })
}

/// Voxel and cube size for a scan with walls of `thickness`.
fn resolution(p: &Params, thickness: Option<f64>) -> Result<(f64, u32, bool), RemeshError> {
    let h0 = p.voxel.unwrap_or(1.5 * p.tolerance);
    let k = match p.min_feature {
        None => 1,
        Some(w) => ((w / (2.0 * h0)).round() as u32).min(MAX_K),
    };
    let steps = 2.0 * f64::from(k) + THIN_EXTRA_VOXELS;
    let Some(t) = thickness.filter(|t| *t < steps * h0) else { return Ok((h0, k, false)) };
    let suggested = t / (steps + 0.5);
    if p.voxel.is_some() {
        return Err(RemeshError::ThinWall { thickness: t, required: steps * h0, voxel: h0, suggested });
    }
    Ok((suggested, k, true))
}

/// The surface of the solid as a mesh with consistent triangles, and the facts about it.
fn build_surface(occ: &Occupancy, grid: &Grid, limits: &Limits) -> Result<(Mesh, usize), RemeshError> {
    let (positions, quads) = surface::quads(occ, grid, limits.max_quads)?;
    let triangles = surface::triangulate(&positions, &quads);
    let (mesh, pinches) = split_pinched_vertices(&Mesh { positions, triangles });
    Ok((mesh, pinches))
}

/// Rebuilds a closed, manifold surface from `mesh`; see the module documentation.
pub fn remesh(mesh: &Mesh, p: &Params) -> Result<(Mesh, RemeshReport), RemeshError> {
    check(p)?;
    let prep = prepare(mesh, p)?;
    let bvh = Bvh::build(&prep.work);
    let h_probe = p.voxel.unwrap_or(1.5 * p.tolerance);
    let wall = thickness(&prep.work, &bvh, h_probe);
    let (h, k, refined) = resolution(p, wall)?;
    let grid = Grid::new(&prep.work.bbox(), h, MIN_BORDER + 2 * k as usize)?;
    if grid.nx * grid.ny > p.limits.max_columns {
        let advice = "use a larger voxel".to_string();
        return Err(RemeshError::TooMuchWork {
            what: "grid columns".into(),
            needed: grid.nx * grid.ny,
            limit: p.limits.max_columns,
            advice,
        });
    }
    let flipped = signed_volume(&prep.work) < 0.0;
    let (raw, stats) = raster::solid(&prep.work, &grid, flipped, p.limits.max_crossings)?;
    if stats.columns > 0 && stats.irregular_columns as f64 > MAX_IRREGULAR_SHARE * stats.columns as f64 {
        return Err(RemeshError::Irregular { share: 100.0 * stats.irregular_columns as f64 / stats.columns as f64 });
    }
    if raw.voxel_count() == 0 {
        return Err(RemeshError::NoSolid);
    }
    let closed = raw.close_open(k);
    let (occ, diagonal_voxels, settled) = closed.fill_diagonals(DIAGONAL_ROUNDS);
    if !settled {
        return Err(RemeshError::NotManifold("checkerboard contacts between voxels did not settle".into()));
    }
    if occ.voxel_count() == 0 {
        return Err(RemeshError::Lost { share: 100.0, distance: h, width: 2.0 * f64::from(k) * h });
    }
    let (mut net, pinches_split) = build_surface(&occ, &grid, &p.limits)?;
    let islands_removed = edit::remove_small_components(&mut net, p.min_component_fraction, 8);
    let mut report = RemeshReport {
        voxel: h,
        voxel_refined: refined,
        k,
        feature_removed: 2.0 * f64::from(k) * h,
        thickness: wall,
        holes_filled: prep.holes_filled,
        debris_triangles: prep.debris_triangles,
        flipped,
        grid: [grid.nx, grid.ny, grid.nz],
        columns: stats.columns,
        crossings: stats.crossings,
        irregular_columns: stats.irregular_columns,
        runs: occ.run_count(),
        voxels: occ.voxel_count(),
        diagonal_voxels,
        pinches_split,
        islands_removed,
        genus_before: prep.genus,
        non_manifold_before: prep.non_manifold,
        peak_bytes: 8 * stats.crossings + raw.bytes() + 60 * net.triangle_count() / 2,
        ..Default::default()
    };
    let out = fit_to_scan(net, &prep.work, h, p, &mut report);
    finish(out, &prep.work, &bvh, h, k, &mut report)
}

/// Smooths and projects the surface onto the scan's vertices and triangle centroids.
fn fit_to_scan(net: Mesh, scan: &Mesh, h: f64, p: &Params, report: &mut RemeshReport) -> Mesh {
    let rings = fit::Rings::new(net.positions.len(), &net.triangles);
    let smooth = Mesh { positions: fit::taubin(&net.positions, &rings), triangles: net.triangles };
    let normals = smooth.vertex_normals();
    let mut samples = scan.positions.clone();
    samples.extend((0..scan.triangle_count()).filter_map(|i| scan.face_centroid(i)));
    let radius = (FIT_RADIUS_EDGES * fit::median_edge(&scan.positions, &scan.triangles)).clamp(3.0 * h, FIT_RADIUS_MAX);
    let tree = KdTree::build(&samples);
    let fit = fit::Fit { radius, height: 0.5 * p.tolerance, max_move: 3.0 * h };
    let moved = fit::project(&smooth.positions, &normals, &tree, &fit, h);
    report.fit_radius = radius;
    report.unfitted = moved.iter().zip(&smooth.positions).filter(|(a, b)| a == b).count();
    Mesh { positions: moved, triangles: smooth.triangles }
}

/// Checks the result against the scan, fills in the report.
fn finish(
    out: Mesh,
    scan: &Mesh,
    scan_bvh: &Bvh,
    h: f64,
    k: u32,
    report: &mut RemeshReport,
) -> Result<(Mesh, RemeshReport), RemeshError> {
    let result_bvh = Bvh::build(&out);
    report.original_to_result = deviation(scan.positions.iter(), &result_bvh);
    report.result_to_original = deviation(out.positions.iter(), scan_bvh);
    let lost = lost_share(scan, &result_bvh, 3.0 * h);
    if lost > MAX_LOST_SHARE {
        return Err(RemeshError::Lost { share: 100.0 * lost, distance: 3.0 * h, width: 2.0 * f64::from(k) * h });
    }
    let ps = pieces::pieces(&out);
    if ps.iter().any(|q| !q.closed()) {
        return Err(RemeshError::NotManifold("the surface has boundary or non-manifold edges".into()));
    }
    report.genus_after = ps.iter().map(pieces::Piece::genus).collect();
    report.triangles = out.triangle_count();
    Ok((out, report.clone()))
}

/// Share of a sample of the scan's vertices farther than `distance` from the surface.
fn lost_share(scan: &Mesh, bvh: &Bvh, distance: f64) -> f64 {
    let n = scan.positions.len();
    let count = DEVIATION_SAMPLES.min(n);
    let far: usize = par::map(count, |k| {
        scan.positions.get(k * n / count.max(1)).is_some_and(|p| bvh.closest(p, distance).is_none())
    })
    .into_iter()
    .filter(|f| *f)
    .count();
    far as f64 / count.max(1) as f64
}

#[cfg(test)]
mod tests;
