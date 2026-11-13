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
//! and about 60 bytes per surface quad while it is built ([`Limits::max_quads`]; quads grow with
//! area / voxel², not with the triangles of the scan). Measured on a synthetic scan of 998 000
//! triangles, 300 mm across, 285 000 mm² of surface, at the default voxel of 0.3 mm: grid 1012 x 1014 x
//! 1022 cells (a dense grid would be 10^9 voxels), 1.6 million crossings, 4.7 million quads (9.5 million
//! triangles out), 83 s on a loaded machine, peak resident memory of the process 3.4 GB (the scan, its
//! tree and BVH, the voxel stage of about 0.3 GB and the result); [`RemeshReport::peak_bytes`] estimates the
//! voxel stage only.
mod fit;
mod grid;
mod occupancy;
mod prepare;
mod raster;
mod surface;
mod walls;

use crate::{Bvh, KdTree, Mesh, Point3, edit, nesting, par, pieces};
use grid::Grid;
pub use grid::{MAX_AXIS, cell_product};
pub use occupancy::Occupancy;
use prepare::prepare;
pub use surface::split_pinched_vertices;

/// Voxels around the part (room for the closing and the surface).
const MIN_BORDER: usize = 3;
/// Largest half-size of the closing and opening cube (voxels).
const MAX_K: u32 = 4;
/// Runs held at once never exceed this (the offsets of the column index are 32-bit).
const RUN_BUDGET_MAX: usize = 1 << 30;
/// Closing rounds that remove checkerboard voxel blocks.
const DIAGONAL_ROUNDS: usize = 12;
/// More than this share of the tested triangles crossing another one refuses the result.
const MAX_SELF_INTERSECTION_SHARE: f64 = 0.01;
/// Triangles of the result tested for self-intersection.
const SELF_INTERSECTION_SAMPLES: usize = 3000;
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
        if cfg!(target_pointer_width = "32") {
            Self { max_columns: 4_000_000, max_crossings: 10_000_000, max_quads: 1_500_000 }
        } else {
            Self { max_columns: 16_000_000, max_crossings: 40_000_000, max_quads: 6_000_000 }
        }
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
        "the scan is open: {loops} boundary loops remain after filling holes of up to {max_hole_edges} edges and {max_hole} mm (a loop of more than three edges is not a flap), the largest with {largest_edges} edges ({length:.1} mm long); remeshing builds a closed solid and does not close large openings"
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
    /// The projection left the surface crossing itself.
    #[error(
        "{share:.1} % of a sample of the triangles of the result cross another one: the voxel is too coarse for this scan or the scan too noisy; use a finer voxel"
    )]
    SelfIntersecting {
        /// Percent of the sample.
        share: f64,
    },
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
    /// Largest distance among the points measured: exact when `exact`, else of an even sample, which
    /// can miss a single removed fin.
    pub max: f64,
    /// Every vertex was measured (not a sample).
    pub exact: bool,
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
    /// Bytes of the voxel stage (crossings, column index, runs and quads); not the whole process.
    pub peak_bytes: usize,
    /// Original vertices to the remeshed surface: the error of the result and, in its tail, what the
    /// closing and opening removed.
    pub original_to_result: Deviation,
    /// Remeshed vertices to the original surface: the error of the result alone.
    pub result_to_original: Deviation,
    /// Share of the measured original vertices farther from the result than the tolerance: fins,
    /// bridges and other features removed on purpose (or lost), not error of the surface.
    pub removed_share: f64,
    /// Duplicate triangles dropped before the remesh.
    pub duplicate_triangles: usize,
    /// Milliseconds the smoothing and projection took (0 where there is no clock).
    pub fit_ms: u64,
    /// Triangles of the result that cross another one (of an even sample) after the projection.
    pub self_intersections: usize,
    /// Triangles tested for that.
    pub self_intersection_samples: usize,
    /// What the caller should know (thin features removed).
    pub warnings: Vec<String>,
}

/// Wall-clock time of a stage; zero where the clock is not available (the browser build).
struct Clock(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Clock {
    fn start() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self(std::time::Instant::now())
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self()
        }
    }

    fn ms(&self) -> u64 {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.0.elapsed().as_millis() as u64
        }
        #[cfg(target_arch = "wasm32")]
        {
            0
        }
    }
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

/// Distances from the `points` to the surface `bvh`: all of them up to [`DEVIATION_SAMPLES`] points,
/// else an even sample (the report says which).
fn deviation<'a>(points: impl Iterator<Item = &'a Point3>, bvh: &Bvh) -> (Deviation, Vec<f64>) {
    let all: Vec<&Point3> = points.collect();
    let count = DEVIATION_SAMPLES.min(all.len());
    let mut d: Vec<f64> = par::map(count, |k| {
        all.get(crate::even_index(k, all.len(), count))
            .and_then(|p| bvh.closest(p, f64::INFINITY))
            .map_or(0.0, |h| h.distance)
    });
    d.sort_by(f64::total_cmp);
    let at = |q: usize| d.get((d.len().saturating_sub(1) * q) / 100).copied().unwrap_or(0.0);
    let dev = Deviation {
        samples: d.len(),
        p50: at(50),
        p95: at(95),
        p99: at(99),
        max: d.last().copied().unwrap_or(0.0),
        exact: count == all.len(),
    };
    (dev, d)
}

/// The surface of the solid as a mesh with consistent triangles, and the facts about it.
fn build_surface(occ: &Occupancy, grid: &Grid, limits: &Limits) -> Result<(Mesh, usize), RemeshError> {
    let (positions, quads) = surface::quads(occ, grid, limits.max_quads)?;
    let triangles = surface::triangulate(&positions, &quads);
    let (mesh, pinches) = split_pinched_vertices(&Mesh { positions, triangles });
    Ok((mesh, pinches))
}

/// The solid of the prepared scan after closing and opening, with the grid it lives on.
struct Solid {
    occ: Occupancy,
    grid: Grid,
    raster: raster::RasterStats,
    raw_bytes: usize,
    diagonal_voxels: u64,
    flipped: bool,
}

fn solid_of(work: &Mesh, h: f64, k: u32, limits: &Limits) -> Result<Solid, RemeshError> {
    let grid = Grid::new(&work.bbox(), h, MIN_BORDER + 2 * k as usize)?;
    let columns = grid::cell_product(grid.nx, grid.ny);
    // the column index has 32-bit offsets and keys: never more than 2^31 columns, whatever the limit says
    if columns > (limits.max_columns as u64).min(1 << 31) {
        let advice = "use a larger voxel".to_string();
        let needed = usize::try_from(columns).unwrap_or(usize::MAX);
        return Err(RemeshError::TooMuchWork {
            what: "grid columns".into(),
            needed,
            limit: limits.max_columns,
            advice,
        });
    }
    // a rough bound before any voxel is built: a smooth surface has about area / h² quads and up to
    // 3 times that when it is oblique (a doubled shell or noise add more), so only more than 4 times
    // the limit is refused here; the real count is checked before the quads are allocated
    let quads = work.area() / (h * h);
    if quads > 4.0 * limits.max_quads as f64 {
        let (needed, advice) = (quads as usize, "use a larger voxel".to_string());
        let what = "surface quads (estimated from the area and the voxel, before any voxel is built)".into();
        return Err(RemeshError::TooMuchWork { what, needed, limit: limits.max_quads, advice });
    }
    let flipped = signed_volume(work) < 0.0;
    let (raw, stats) = raster::solid(work, &grid, flipped, limits.max_crossings)?;
    if stats.columns > 0 && stats.irregular_columns as f64 > MAX_IRREGULAR_SHARE * stats.columns as f64 {
        return Err(RemeshError::Irregular { share: 100.0 * stats.irregular_columns as f64 / stats.columns as f64 });
    }
    if raw.voxel_count() == 0 {
        return Err(RemeshError::NoSolid);
    }
    let budget = (4 * limits.max_crossings).min(RUN_BUDGET_MAX);
    let runs = |n: usize| RemeshError::TooMuchWork {
        what: "runs of the solid after dilation or erosion".into(),
        needed: n,
        limit: budget,
        advice: "use a larger voxel or a smaller `min_feature`".into(),
    };
    let closed = raw.close_open(k, budget).map_err(runs)?;
    let (occ, diagonal_voxels, settled) = closed.fill_diagonals(DIAGONAL_ROUNDS, budget).map_err(runs)?;
    if !settled {
        return Err(RemeshError::NotManifold("checkerboard contacts between voxels did not settle".into()));
    }
    if occ.voxel_count() == 0 {
        return Err(RemeshError::Lost { share: 100.0, distance: h, width: 2.0 * f64::from(k) * h });
    }
    Ok(Solid { occ, grid, raster: stats, raw_bytes: raw.bytes(), diagonal_voxels, flipped })
}

/// Rebuilds a closed, manifold surface from `mesh`; see the module documentation.
pub fn remesh(mesh: &Mesh, p: &Params) -> Result<(Mesh, RemeshReport), RemeshError> {
    check(p)?;
    let prep = prepare(mesh, p)?;
    let bvh = Bvh::build(&prep.work);
    let h_probe = p.voxel.unwrap_or(1.5 * p.tolerance);
    let ignore = p.min_feature.unwrap_or(2.0 * h_probe) + 0.5 * h_probe;
    let walls = walls::probe(&prep.work, &bvh, h_probe, ignore);
    let res = walls::resolution(p.voxel, p.min_feature, p.tolerance, &walls)?;
    let (h, k, refined) = (res.h, res.k, res.refined);
    let wall = walls.wall;
    let solid = solid_of(&prep.work, h, k, &p.limits)?;
    let (mut net, pinches_split) = build_surface(&solid.occ, &solid.grid, &p.limits)?;
    let islands_removed = edit::remove_small_components(&mut net, p.min_component_fraction, 8);
    let mut report = RemeshReport {
        voxel: h,
        voxel_refined: refined,
        k,
        feature_removed: 2.0 * f64::from(k) * h,
        thickness: wall,
        holes_filled: prep.holes_filled,
        debris_triangles: prep.debris_triangles,
        flipped: solid.flipped,
        grid: [solid.grid.nx, solid.grid.ny, solid.grid.nz],
        columns: solid.raster.columns,
        crossings: solid.raster.crossings,
        irregular_columns: solid.raster.irregular_columns,
        runs: solid.occ.run_count(),
        voxels: solid.occ.voxel_count(),
        diagonal_voxels: solid.diagonal_voxels,
        pinches_split,
        islands_removed,
        genus_before: prep.genus,
        non_manifold_before: prep.non_manifold,
        duplicate_triangles: prep.duplicate_triangles,
        warnings: res.warnings,
        peak_bytes: 8 * solid.raster.crossings + solid.raw_bytes + 60 * net.triangle_count() / 2,
        ..Default::default()
    };
    let clock = Clock::start();
    let out = fit_to_scan(net, &prep.work, h, p, &mut report);
    report.fit_ms = clock.ms();
    finish(out, &prep.work, &bvh, h, k, p.tolerance, &mut report)
}

/// Smooths and projects the surface onto the scan's vertices and triangle centroids.
fn fit_to_scan(net: Mesh, scan: &Mesh, h: f64, p: &Params, report: &mut RemeshReport) -> Mesh {
    let rings = fit::Rings::new(net.positions.len(), &net.triangles);
    let smooth = Mesh { positions: fit::taubin(&net.positions, &rings), triangles: net.triangles };
    let normals = smooth.vertex_normals();
    let mut samples = scan.positions.clone();
    samples.extend((0..scan.triangle_count()).filter_map(|i| scan.face_centroid(i)));
    let radius = (FIT_RADIUS_EDGES * fit::median_edge(&scan.positions, &scan.triangles))
        .clamp(3.0 * h, FIT_RADIUS_MAX.max(3.0 * h));
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
    tolerance: f64,
    report: &mut RemeshReport,
) -> Result<(Mesh, RemeshReport), RemeshError> {
    let result_bvh = Bvh::build(&out);
    let (back, distances) = deviation(scan.positions.iter(), &result_bvh);
    report.original_to_result = back;
    report.removed_share = distances.iter().filter(|d| **d > tolerance).count() as f64 / distances.len().max(1) as f64;
    report.result_to_original = deviation(out.positions.iter(), scan_bvh).0;
    let lost = lost_share(scan, &result_bvh, 3.0 * h);
    if lost > MAX_LOST_SHARE {
        return Err(RemeshError::Lost { share: 100.0 * lost, distance: 3.0 * h, width: 2.0 * f64::from(k) * h });
    }
    let ps = pieces::pieces(&out);
    if ps.iter().any(|q| !q.closed()) {
        return Err(RemeshError::NotManifold("the surface has boundary or non-manifold edges".into()));
    }
    report.genus_after = ps.iter().map(pieces::Piece::genus).collect();
    let crossing = nesting::sampled_self_intersections(&out, &result_bvh, SELF_INTERSECTION_SAMPLES);
    report.self_intersections = crossing.crossing;
    report.self_intersection_samples = crossing.tested;
    let share = crossing.crossing as f64 / crossing.tested.max(1) as f64;
    if share > MAX_SELF_INTERSECTION_SHARE {
        return Err(RemeshError::SelfIntersecting { share: 100.0 * share });
    }
    if crossing.crossing > 0 {
        let msg =
            format!("{} of {} sampled triangles of the result cross another one", crossing.crossing, crossing.tested);
        report.warnings.push(msg);
    }
    report.triangles = out.triangle_count();
    Ok((out, report.clone()))
}

/// Share of a sample of the scan's vertices farther than `distance` from the surface.
fn lost_share(scan: &Mesh, bvh: &Bvh, distance: f64) -> f64 {
    let n = scan.positions.len();
    let count = DEVIATION_SAMPLES.min(n);
    let far: usize = par::map(count, |k| {
        scan.positions.get(crate::even_index(k, n, count)).is_some_and(|p| bvh.closest(p, distance).is_none())
    })
    .into_iter()
    .filter(|f| *f)
    .count();
    far as f64 / count.max(1) as f64
}

#[cfg(test)]
mod tests;
