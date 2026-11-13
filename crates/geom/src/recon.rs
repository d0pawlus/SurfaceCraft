//! Surface reconstruction from a point cloud (P1: scanners that export points, not faces).
//!
//! Pipeline, all mesh-side computation (design doc §5):
//!
//! 1. exact duplicates are dropped; a k-d tree gives the k nearest neighbours of every point;
//! 2. normals: PCA of the neighbourhood (smallest eigenvector of the covariance), oriented
//!    consistently by propagation along a minimum spanning tree of the neighbour graph with edge
//!    weight `1 − |nᵢ·nⱼ|` (Hoppe et al. 1992), each connected piece facing away from its
//!    extreme point; normals given by the file are trusted as they are;
//! 3. ball pivoting (Bernardini et al. 1999) with radii of a few mean nearest-neighbour
//!    spacings: a ball of radius ρ rolls over the points, every three points it touches without
//!    holding another point form a triangle; passes with growing ρ close the gaps the smaller
//!    balls left. The triangle orientation follows the vertex normals, so the result is
//!    consistently oriented and manifold along edges by construction;
//! 4. leftover specks are dropped and small holes are filled ([`crate::edit`]).
//!
//! Neighbour search and normal estimation use all cores outside the browser; ball pivoting is
//! sequential (its front depends on what was built before).
use crate::{Aabb, KdTree, Mesh, Point3, Vec3, cloud::CloudError, cloud::PointCloud, edit};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::f64::consts::{PI, TAU};
use std::hash::{BuildHasherDefault, Hasher};
use std::ops::Range;
mod grid;
mod normals;
mod pivot;
#[cfg(test)]
mod tests;

use normals::*;
use pivot::*;

/// Reconstruction settings.
#[derive(Clone, Debug, PartialEq)]
pub struct ReconParams {
    /// Neighbours used for normals and orientation (3 – 64).
    pub normals_k: usize,
    /// First ball radius as a multiple of the mean nearest-neighbour spacing (0.5 – 50).
    pub radius_scale: f64,
    /// Number of passes; each doubles the radius (1 – 6).
    pub passes: usize,
    /// Boundary loops of at most this many edges are filled afterwards (0 = none).
    pub fill_holes_max_edges: usize,
    /// Refuse clouds with more points than this.
    pub max_points: usize,
}

impl Default for ReconParams {
    fn default() -> Self {
        Self { normals_k: 12, radius_scale: 2.5, passes: 3, fill_holes_max_edges: 24, max_points: 20_000_000 }
    }
}

/// Why a cloud could not be reconstructed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ReconError {
    /// A setting is out of range.
    #[error("{0}")]
    Params(String),
    /// The cloud itself is invalid.
    #[error(transparent)]
    Cloud(#[from] CloudError),
    /// Too few distinct points.
    #[error("{0} distinct points are too few to reconstruct a surface")]
    TooFew(usize),
    /// Over the configured point limit.
    #[error("{points} points exceed the limit of {limit}")]
    TooMany {
        /// Points in the cloud.
        points: usize,
        /// Configured limit.
        limit: usize,
    },
    /// Coordinates beyond +-1e9 mm (a thousand kilometres) are not a scan.
    #[error("coordinates beyond 1e9 mm cannot be reconstructed")]
    TooLarge,
    /// No triangle could be built (points too sparse, collinear or without a surface).
    #[error("no surface found: no ball of the chosen radii rests on three points; try a larger radius scale")]
    NoSurface,
}

/// What reconstruction did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReconReport {
    /// Points in the input.
    pub points: usize,
    /// Exact duplicates dropped.
    pub duplicates_removed: usize,
    /// Mean nearest-neighbour spacing (mm).
    pub spacing: f64,
    /// Ball radii used, in order (mm).
    pub radii: Vec<f64>,
    /// The file's normals were used (otherwise estimated and oriented here).
    pub normals_given: bool,
    /// Triangles after ball pivoting.
    pub pivoted_triangles: usize,
    /// Points left out of the mesh (no ball rested on them).
    pub unused_points: usize,
    /// Isolated specks removed (triangles).
    pub specks_removed: usize,
    /// Holes filled.
    pub holes_filled: usize,
    /// Boundary loops left in the result.
    pub holes_left: usize,
}

/// Reconstructs a triangle mesh from `cloud`.
pub fn reconstruct(cloud: &PointCloud, p: &ReconParams) -> Result<(Mesh, ReconReport), ReconError> {
    if !(3..=64).contains(&p.normals_k) {
        return Err(ReconError::Params("normals_k must be between 3 and 64".into()));
    }
    if !(p.radius_scale.is_finite() && (0.5..=50.0).contains(&p.radius_scale)) {
        return Err(ReconError::Params("radius_scale must be between 0.5 and 50".into()));
    }
    if !(1..=6).contains(&p.passes) {
        return Err(ReconError::Params("passes must be between 1 and 6".into()));
    }
    cloud.validate()?;
    if cloud.len() > p.max_points {
        return Err(ReconError::TooMany { points: cloud.len(), limit: p.max_points });
    }
    if cloud.positions.iter().any(|q| q.x.abs() > 1e9 || q.y.abs() > 1e9 || q.z.abs() > 1e9) {
        return Err(ReconError::TooLarge);
    }
    let (pts, given) = dedupe(cloud);
    let mut rep =
        ReconReport { points: cloud.len(), duplicates_removed: cloud.len() - pts.len(), ..Default::default() };
    if pts.len() < 8 {
        return Err(ReconError::TooFew(pts.len()));
    }
    let k = p.normals_k.min(pts.len() - 1);
    let tree = KdTree::build(&pts);
    let nbr = knn_graph(&tree, &pts, k);
    drop(tree);

    let spacing = mean_spacing(&pts, &nbr, k).ok_or(ReconError::NoSurface)?;
    rep.spacing = spacing;
    let normals = match given {
        Some(n) => {
            rep.normals_given = true;
            n
        }
        None => {
            let mut n = estimate_normals(&pts, &nbr, k);
            orient_normals(&pts, &mut n, &nbr, k);
            n
        }
    };
    drop(nbr);

    let triangles = {
        let mut bpa = Bpa::new(&pts, &normals);
        for j in 0..p.passes {
            let rho = spacing * p.radius_scale * f64::from(1u32 << j);
            rep.radii.push(rho);
            bpa.pass(rho);
        }
        bpa.tris
    };
    rep.pivoted_triangles = triangles.len();
    if triangles.is_empty() {
        return Err(ReconError::NoSurface);
    }
    let mut mesh = Mesh { positions: pts, triangles };
    edit::compact(&mut mesh);
    rep.unused_points = rep.points - rep.duplicates_removed - mesh.positions.len();
    rep.specks_removed = edit::remove_small_components(&mut mesh, 0.0, 8);
    edit::compact(&mut mesh);
    if p.fill_holes_max_edges >= 3 {
        rep.holes_filled = edit::fill_holes(&mut mesh, p.fill_holes_max_edges, false).filled;
    }
    rep.holes_left = edit::boundary_loops(&mesh).len();
    Ok((mesh, rep))
}
