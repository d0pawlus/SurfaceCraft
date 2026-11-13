//! Thickening an open triangle mesh into a closed shell of constant thickness, on the mesh side:
//! the scan of a panel, a casting scanned from one side, a sheet. The B-rep kernel refuses
//! multi-face sheets, so a solid of such a scan is made here and wrapped afterwards.
//!
//! Method: every vertex is offset along a robust normal ([`normals`]: angle-weighted, then smoothed),
//! the offset copy is turned around, and every boundary edge of the original is joined to its offset
//! copy by a strip of triangles (rows of about one edge length, so the strip has no slivers). Where
//! the offset would fold, the distance is reduced locally ([`relax`]); when that is needed for more
//! than [`MAX_CLAMPED_SHARE`] of the vertices the call is refused instead, because the thickness is
//! then not a property of the part but a mistake (a wall thicker than the radii of the surface).
//!
//! Layers: the shell occupies `[b, a]` along the normals (`a > b`). `thickness > 0` puts the
//! material behind the scanned face (`a = 0`, `b = -thickness`: against the normals), `thickness < 0`
//! in front of it (`a = |thickness|`, `b = 0`), `symmetric` half on each side. The layer at `a` keeps
//! the orientation of the mesh, the layer at `b` is reversed, so the result is oriented outward.
mod assemble;
mod crossing;
mod feature;
mod normals;
mod relax;
mod repair;
mod selfx;
mod strip;
mod topology;

use crate::{Aabb, Mesh, MeshError, Vec3};
use assemble::{Inputs, assemble};
use strip::Rim;
use topology::{Csr, Work};

/// Share of the vertices that may lie on a folded triangle of the offset (before it is relaxed); more is refused.
pub const MAX_CLAMPED_SHARE: f64 = 0.15;
/// The thickness is at most this share of the diagonal of the mesh's bounding box. (A wall of that
/// size is not a shell; the limit also keeps every coordinate finite.)
pub const MAX_THICKNESS_SHARE: f64 = 0.5;
/// Share of the vertices that may stay folded at the floor of the offset (sharp creases on the offset
/// side); more is refused.
pub const MAX_UNRESOLVED_SHARE: f64 = 0.001;
/// The normals are averaged over this share of the thickness (a distance, at least one edge).
const SMOOTHING_SHARE: f64 = 0.5;
/// How to thicken.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ThickenParams {
    /// mm; positive: the material lies behind the scanned face (against the normals), negative: in
    /// front of it (along them).
    pub thickness: f64,
    /// Half of the thickness on each side of the scanned face (the sign is then ignored).
    pub symmetric: bool,
}

/// Which layer of the result is the scanned face.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ScanLayer {
    /// The layer in the direction of the normals: the scan itself for `thickness > 0`, the scan moved
    /// by half the thickness for `symmetric`.
    Outer,
    /// The layer against the normals: the scan itself for `thickness < 0`.
    Inner,
}

/// What the thickening did.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ThickenReport {
    /// Vertices of the result.
    pub vertices: usize,
    /// Triangles of the result.
    pub triangles: usize,
    /// Boundary loops of the original joined by strips.
    pub boundary_loops: usize,
    /// Boundary edges of the original.
    pub boundary_edges: usize,
    /// Rows of triangles in the strip along the rim.
    pub strip_rows: usize,
    /// Vertices split where boundary loops touched.
    pub pinch_splits: usize,
    /// Degenerate triangles dropped from the input.
    pub dropped_triangles: usize,
    /// Triangles removed because they lay on an edge shared by three or more triangles.
    pub removed_non_manifold: usize,
    /// Triangles turned around to make the orientation consistent.
    pub flipped_triangles: usize,
    /// Vertices whose offset was reduced to avoid a fold.
    pub clamped_vertices: usize,
    /// Vertices of the scan moved (by at most one median edge) to remove crossings of the scan with itself.
    pub repaired_vertices: usize,
    /// Vertices whose offset was bounded before any fold was looked for, by the distance along the
    /// offset direction to another part of the scan (the local feature size).
    pub limited_vertices: usize,
    /// Largest reduction of the thickness at a vertex (mm).
    pub largest_reduction: f64,
    /// Median reduction of the thickness over the vertices that were reduced (mm; 0 when none).
    pub median_reduction: f64,
    /// Vertices of triangles that folded before the relaxation (the others reduced were only tapered).
    pub folded_vertices: usize,
    /// Vertices that stay folded at the floor of the offset (a tenth of the thickness).
    pub unresolved_vertices: usize,
    /// Edges of the longest boundary loop (the rim of a panel).
    pub longest_loop_edges: usize,
    /// Thinnest wall found (mm): the distance from sampled vertices of one layer to the other layer.
    pub min_wall: f64,
    /// Median wall found (mm).
    pub median_wall: f64,
    /// Triangles of the shell tested for crossing a triangle they share no vertex with (an even sample).
    pub crossing_samples: usize,
    /// Of them, triangles that cross only triangles close to them along the scan (a fold of the offset:
    /// neighbouring triangles, the flanks of a crease, the two sheets of a collapsed wall, the strip
    /// along the rim). The relaxation removes these; the count is 0 unless vertices stay unresolved.
    pub crossing_local: usize,
    /// Of them, triangles that cross a triangle far from them along the scan: the offset sheet meets
    /// another part of the shell (a channel or gap narrower than the wall), real geometry.
    pub crossing_triangles: usize,
}

/// A closed shell.
#[derive(Clone, Debug)]
pub struct Thickened {
    /// The shell: closed, manifold, oriented outward.
    pub mesh: Mesh,
    /// What was done.
    pub report: ThickenReport,
    /// Which layer of `mesh` the scan lies on.
    pub scan_layer: ScanLayer,
    /// Share of the offset each vertex of the scan face keeps (1: all of it).
    pub scale: Vec<f64>,
    layer_vertices: usize,
    layer_triangles: usize,
}

impl Thickened {
    /// The layer the scan lies on as a mesh of its own: the scan's triangles over the vertices of
    /// that layer, wound like the shell's outside there (so its normals face the way the shell's do,
    /// which is against the scan's normals for `thickness < 0`).
    #[must_use]
    pub fn scan_face(&self) -> Mesh {
        let (n, t) = (self.layer_vertices, self.layer_triangles);
        let (v0, t0) = match self.scan_layer {
            ScanLayer::Outer => (0, 0),
            ScanLayer::Inner => (n, t),
        };
        let positions = self.mesh.positions.get(v0..v0 + n).unwrap_or(&[]).to_vec();
        let triangles =
            self.mesh.triangles.get(t0..t0 + t).unwrap_or(&[]).iter().map(|tri| tri.map(|v| v - v0 as u32)).collect();
        Mesh { positions, triangles }
    }
}

/// Why a mesh was not thickened.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ThickenError {
    /// The thickness is zero or not finite.
    #[error("thickness must be a finite number other than 0 (got {0})")]
    BadThickness(f64),
    /// The thickness exceeds [`MAX_THICKNESS_SHARE`] of the mesh's size.
    #[error(
        "thickness {thickness} mm is larger than half the size of the mesh ({limit:.3} mm is the most for this mesh)"
    )]
    TooThick {
        /// Requested.
        thickness: f64,
        /// The most for this mesh.
        limit: f64,
    },
    /// The thickness is below what the coordinates resolve.
    #[error(
        "thickness {thickness} mm is below what the coordinates of this mesh resolve ({limit:e} mm is the least): the two sides would coincide"
    )]
    TooThin {
        /// Requested.
        thickness: f64,
        /// The least for this mesh.
        limit: f64,
    },
    /// The mesh is not valid.
    #[error("the mesh is not valid: {0}")]
    Invalid(MeshError),
    /// The mesh has no triangles.
    #[error("the mesh has no triangles")]
    Empty,
    /// The mesh has no boundary.
    #[error("the mesh is closed (it has no boundary): there is no one-sided surface to thicken")]
    Closed,
    /// Edges are shared by three or more triangles, or by two triangles of opposite orientation.
    #[error(
        "{edges} edges are shared by three or more triangles or by triangles of opposite orientation: remove the extra triangles and orient the mesh consistently (mesh.clean) before thickening"
    )]
    NonOrientable {
        /// How many edges.
        edges: usize,
    },
    /// Some vertices have no usable normal.
    #[error(
        "some vertices have no usable normal (all their triangles are degenerate): clean the mesh before thickening"
    )]
    NoNormals,
    /// The offset folds over too much of the surface.
    #[error(
        "a thickness of {thickness} mm folds the offset over {share_percent:.1} % of the surface ({clamped} of {vertices} vertices; the radii of the surface are smaller than the thickness there, largest reduction {largest:.3} mm): use a thickness of at most about {suggested:.3} mm or a smoother scan"
    )]
    Folds {
        /// Requested thickness.
        thickness: f64,
        /// Percentage of vertices that fold.
        share_percent: f64,
        /// Vertices that fold.
        clamped: usize,
        /// Vertices.
        vertices: usize,
        /// Largest reduction of the thickness (mm).
        largest: f64,
        /// A thickness that would fold less (mm).
        suggested: f64,
    },
    /// The folds could not be resolved.
    #[error(
        "the offset folds at {0} vertices that cannot be relaxed to more than a tenth of the thickness: use a smaller thickness"
    )]
    Unresolved(usize),
    /// Too many vertices stay on a crossing of the shell that even the thinnest wall allowed does not remove.
    #[error(
        "{vertices} of {total} vertices ({share_percent:.2} %, more than {limit_percent:.1} % are refused) stay on a crossing of the shell that no wall down to a tenth of the thickness removes (sharp creases, folded or doubled sheets of the scan closer than a tenth of the wall): repair the scan (mesh.clean; isolate the part with scan.split) before thickening; a thinner wall helps only where the sheets are farther apart than a tenth of it"
    )]
    UnresolvedCrossings {
        /// Vertices on a crossing.
        vertices: usize,
        /// Vertices of the scan.
        total: usize,
        /// Their share (percent).
        share_percent: f64,
        /// The share above which the call is refused (percent).
        limit_percent: f64,
    },
    /// One vertex belongs to an absurd number of triangles (a fan around a hub): not a scan.
    #[error(
        "a vertex belongs to {triangles} triangles (more than {limit}): this is not a scan surface; clean the mesh (mesh.clean, mesh.remesh) before thickening"
    )]
    HubVertex {
        /// Triangles at the vertex.
        triangles: usize,
        /// The most accepted.
        limit: usize,
    },
    /// The result is not a closed, outward-oriented surface (never expected).
    #[error("the shell could not be closed consistently ({0}): use a smaller thickness or clean the mesh")]
    NotClosed(&'static str),
}

/// The offsets `(a, b)` of the outer and the inner layer.
fn layer_offsets(p: &ThickenParams) -> (f64, f64) {
    let t = p.thickness.abs();
    if p.symmetric {
        (t / 2.0, -t / 2.0)
    } else if p.thickness > 0.0 {
        (0.0, -t)
    } else {
        (t, 0.0)
    }
}

fn check_params(mesh: &Mesh, p: &ThickenParams) -> Result<(), ThickenError> {
    if !p.thickness.is_finite() || p.thickness == 0.0 {
        return Err(ThickenError::BadThickness(p.thickness));
    }
    mesh.validate().map_err(ThickenError::Invalid)
}

/// Share of the size of the mesh below which the thickness is lost in the coordinates.
pub const MIN_THICKNESS_SHARE: f64 = 1e-9;

/// The thickness must lie between [`MIN_THICKNESS_SHARE`] and [`MAX_THICKNESS_SHARE`] of the size.
fn check_size(work: &Work, p: &ThickenParams) -> Result<(), ThickenError> {
    let size = Aabb::from_points(&work.positions).diagonal();
    let limit = MAX_THICKNESS_SHARE * size;
    if !(p.thickness.abs() <= limit) {
        return Err(ThickenError::TooThick { thickness: p.thickness, limit });
    }
    let least = MIN_THICKNESS_SHARE * size;
    if p.thickness.abs() < least {
        return Err(ThickenError::TooThin { thickness: p.thickness, limit: least });
    }
    Ok(())
}

/// Thickens the open mesh `mesh` as `params` say.
///
/// # Errors
/// The thickness or the mesh is unusable, or the offset folds over too much of the surface; see
/// [`ThickenError`]. Nothing here panics, whatever the mesh holds.
pub fn thicken(mesh: &Mesh, params: &ThickenParams) -> Result<Thickened, ThickenError> {
    check_params(mesh, params)?;
    let mut work = topology::compact(mesh);
    if work.triangles.is_empty() {
        return Err(ThickenError::Empty);
    }
    check_size(&work, params)?;
    let repairs = repair::repair(&mut work);
    (work.removed, work.flipped) = (repairs.removed, repairs.flipped);
    topology::drop_unused_vertices(&mut work);
    if work.triangles.is_empty() {
        return Err(ThickenError::Empty);
    }
    topology::split_pinches(&mut work);
    let sorted = topology::directed_edges(&work.triangles);
    topology::require_oriented(&sorted)?;
    let boundary = topology::boundary_edges(&sorted);
    if boundary.is_empty() {
        return Err(ThickenError::Closed);
    }
    check_hubs(&work)?;
    let adjacent = topology::adjacency(work.positions.len(), &sorted);
    let scan_edge = median_edge(&work);
    work.repaired = selfx::smooth_scan_crossings(&mut work, &adjacent, scan_edge);
    let radius = (SMOOTHING_SHARE * params.thickness.abs()).max(mean_edge(&work));
    let normals =
        normals::smoothed(&work.positions, &work.triangles, &boundary, radius).ok_or(ThickenError::NoNormals)?;
    let (a, b) = layer_offsets(params);
    let rim = make_rim(&work, &boundary, params)?;
    let edge = median_edge(&work);
    let relaxed = fold_free_scale(&work, &normals, &adjacent, &boundary, &rim, &[a, b], params.thickness)?;
    (work.folded, work.unresolved) = (relaxed.folded, relaxed.unresolved);
    let inputs = Inputs {
        normals: &relaxed.normals,
        scale: &relaxed.scale,
        offsets: (a, b),
        boundary: &boundary,
        rim: &rim,
        adjacent: &adjacent,
        edge,
        size: Aabb::from_points(&work.positions).diagonal(),
        limited: relaxed.limited,
    };
    assemble(&work, &inputs, params)
}

/// A vertex belongs to at most this many triangles (a scan has a few dozen; a hub of a hundred thousand
/// would cost the neighbourhood searches the square of that).
pub const MAX_VERTEX_TRIANGLES: usize = 4_096;

/// Refuses a mesh with a vertex on more than [`MAX_VERTEX_TRIANGLES`] triangles.
fn check_hubs(work: &Work) -> Result<(), ThickenError> {
    let mut count = vec![0usize; work.positions.len()];
    for t in &work.triangles {
        for &v in t {
            if let Some(c) = count.get_mut(v as usize) {
                *c += 1;
            }
        }
    }
    match count.into_iter().max() {
        Some(triangles) if triangles > MAX_VERTEX_TRIANGLES => {
            Err(ThickenError::HubVertex { triangles, limit: MAX_VERTEX_TRIANGLES })
        }
        _ => Ok(()),
    }
}

/// The ids of the strip along the rim; the shell must be indexable with 32 bits.
fn make_rim(work: &Work, boundary: &[(u32, u32)], params: &ThickenParams) -> Result<Rim, ThickenError> {
    let n = work.positions.len();
    let rows = strip::strip_rows(params.thickness, strip::mean_rim_edge(&work.positions, boundary), boundary.len());
    let total = 2 * n + rows.saturating_sub(1) * boundary.len();
    match u32::try_from(n).ok().filter(|_| u32::try_from(total).is_ok()) {
        Some(n32) => Ok(Rim::new(n32, rows, boundary)),
        None => Err(ThickenError::Invalid(MeshError::TooManyVertices(total))),
    }
}

/// Median length of the edges of the triangles (mm; robust against a few huge triangles).
fn median_edge(work: &Work) -> f64 {
    let mut lengths: Vec<f64> = work
        .triangles
        .iter()
        .flat_map(|t| [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])])
        .filter_map(|(a, b)| Some((work.positions.get(a as usize)? - work.positions.get(b as usize)?).norm()))
        .collect();
    if lengths.is_empty() {
        return 0.0;
    }
    let mid = lengths.len() / 2;
    let (_, m, _) = lengths.select_nth_unstable_by(mid, f64::total_cmp);
    *m
}

/// Mean length of the edges of the triangles (mm).
fn mean_edge(work: &Work) -> f64 {
    let (mut sum, mut count) = (0.0, 0usize);
    for t in &work.triangles {
        for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
            if let (Some(p), Some(q)) = (work.positions.get(a as usize), work.positions.get(b as usize)) {
                sum += (p - q).norm();
                count += 1;
            }
        }
    }
    if count == 0 { 0.0 } else { sum / count as f64 }
}

/// The share of the offset each vertex keeps; refused when too many vertices need less than all.
fn fold_free_scale(
    work: &Work,
    normals: &[Vec3],
    adjacent: &Csr,
    boundary: &[(u32, u32)],
    rim: &Rim,
    offsets: &[f64; 2],
    thickness: f64,
) -> Result<relax::Relaxed, ThickenError> {
    let pairs: Vec<(u32, u32)> =
        work.triangles.iter().enumerate().flat_map(|(i, t)| t.iter().map(move |&v| (v, i as u32))).collect();
    let around = Csr::from_pairs(work.positions.len(), &pairs);
    let moving: Vec<f64> = offsets.iter().copied().filter(|o| *o != 0.0).collect();
    let field = relax::Field {
        positions: &work.positions,
        normals,
        triangles: &work.triangles,
        offsets: &moving,
        layers: *offsets,
        around: &around,
        adjacent,
        thickness: thickness.abs(),
        edge: median_edge(work),
        size: Aabb::from_points(&work.positions).diagonal(),
        boundary,
        rim,
    };
    let relaxed = relax::relax(&field).map_err(ThickenError::Unresolved)?;
    if relaxed.unresolved as f64 > MAX_UNRESOLVED_SHARE * relaxed.scale.len() as f64 {
        let total = relaxed.scale.len();
        let share = relaxed.unresolved as f64 / total.max(1) as f64;
        return Err(ThickenError::UnresolvedCrossings {
            vertices: relaxed.unresolved,
            total,
            share_percent: 100.0 * share,
            limit_percent: 100.0 * MAX_UNRESOLVED_SHARE,
        });
    }
    let vertices = relaxed.scale.len();
    if relaxed.folded as f64 > MAX_CLAMPED_SHARE * vertices as f64 {
        let smallest = relaxed.scale.iter().copied().fold(1.0, f64::min);
        return Err(ThickenError::Folds {
            thickness,
            share_percent: 100.0 * relaxed.folded as f64 / vertices as f64,
            clamped: relaxed.folded,
            vertices,
            largest: thickness.abs() * (1.0 - smallest),
            suggested: thickness.abs() * quantile_scale(&relaxed.scale),
        });
    }
    Ok(relaxed)
}

/// The scale below which a share [`MAX_CLAMPED_SHARE`] of the vertices lies (1 when empty).
fn quantile_scale(values: &[f64]) -> f64 {
    let mut v = values.to_vec();
    v.sort_by(f64::total_cmp);
    v.get((MAX_CLAMPED_SHARE * v.len() as f64) as usize).copied().unwrap_or(1.0)
}
