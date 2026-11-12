//! What a `surface.fit` says about its own measurement (P8-H11): how many samples there were and
//! what became of them, and which parts of each patch's parameter domain have scan data under
//! them (see `fit::coverage`).
use super::surface_mask::{Mask, region_vertex_count};
use super::surface_pieces::Piece;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_fit::coverage::{Coverage, coverage};
use surfacecraft_geom::{Mesh, Point3};

/// What became of the points (vertices) of the region. `eligible = measured + excluded + failed`.
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize, JsonSchema)]
pub struct SampleCounts {
    /// Points of the region: the distinct vertices of its triangles.
    pub eligible: usize,
    /// Points the fit used and measured the deviation on.
    pub measured: usize,
    /// Points left out by an exclusion mask.
    pub excluded: usize,
    /// Points that were not measured although not excluded: the projection onto the spline
    /// failed, or they belong to a part of the region that was left out (see `skipped`).
    pub failed: usize,
}

/// Shares of a patch's parameter domain (area of the rectangle of the fit) by what lies under it.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, JsonSchema)]
pub struct CoverageShares {
    /// Share with scan data under it.
    pub measured: f64,
    /// Share inside the outline of the data but without any (a hole the patch bridges).
    pub hole: f64,
    /// Share outside the outline of the data (the patch extrapolates).
    pub extrapolated: f64,
}

/// Measurement coverage of the (first) sheet, and the shares of all sheets.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct CoverageOut {
    /// Cells along the first axis (u) of the fit plane, at most 32.
    pub cols: usize,
    /// Cells along the second axis (v) of the fit plane, at most 32.
    pub rows: usize,
    /// One string of `cols` letters per row, row 0 at the lowest v: `M` measured, `H` hole (inside
    /// the outline of the data, no data under it), `E` extrapolated (outside the outline). Of the
    /// first sheet.
    pub grid: Vec<String>,
    /// Origin of the plane frame of the first sheet's fit, mm.
    pub origin: [f64; 3],
    /// Unit axis of the first coordinate (u) of that frame; cell `(i, j)` spans u from `u_range[0]` by `(u_range[1] - u_range[0]) / cols` per column.
    pub u_axis: [f64; 3],
    /// Unit axis of the second coordinate (v) of that frame; rows step along it from `v_range[0]`.
    pub v_axis: [f64; 3],
    /// Extent of the domain along `u_axis` from `origin`, mm.
    pub u_range: [f64; 2],
    /// Extent of the domain along `v_axis` from `origin`, mm.
    pub v_range: [f64; 2],
    /// Shares of the area of the parameter domain: of the first sheet when the region is one
    /// piece, else of all sheets weighted by the area of their domain.
    pub shares: CoverageShares,
}

/// The coverage of one piece, with the area of the domain rectangle it was measured on.
pub(super) struct PieceCoverage {
    pub coverage: Coverage,
    pub area: f64,
    frame: ([f64; 3], [f64; 3], [f64; 3]),
    domain: [[f64; 2]; 2],
}

impl From<&Coverage> for CoverageShares {
    fn from(c: &Coverage) -> Self {
        Self { measured: c.shares.measured, hole: c.shares.hole, extrapolated: c.shares.extrapolated }
    }
}

/// The coverage of `piece` by the triangles of its chart that are not masked out, in the plane
/// frame of its fit; `None` when it cannot be told (no data, degenerate domain).
pub(super) fn piece_coverage(mesh: &Mesh, mask: &Mask, piece: &Piece) -> Option<PieceCoverage> {
    let g = &piece.grid;
    let project = |p: &Point3| {
        let d = p - g.origin;
        [d.dot(&g.u), d.dot(&g.v)]
    };
    let tris: Vec<[[f64; 2]; 3]> = mask
        .kept_triangles(&piece.triangles)
        .into_iter()
        .filter_map(|t| mesh.triangle(t as usize))
        .map(|[a, b, c]| [project(&a), project(&b), project(&c)])
        .collect();
    let domain = g.domain()?;
    let (w, h) = ((domain[0][1] - domain[0][0]).abs(), (domain[1][1] - domain[1][0]).abs());
    let (o, u, v) = (g.origin, g.u, g.v);
    let frame = ([o.x, o.y, o.z], [u.x, u.y, u.z], [v.x, v.y, v.z]);
    Some(PieceCoverage { coverage: coverage(&tris, domain)?, area: w * h, frame, domain })
}

/// The report of all pieces: the grid of the first, the shares weighted by domain area.
pub(super) fn summarise_coverage(pieces: &[Option<PieceCoverage>]) -> Option<CoverageOut> {
    let first = pieces.first()?.as_ref()?;
    let known: Vec<&PieceCoverage> = pieces.iter().flatten().collect();
    let total: f64 = known.iter().map(|p| p.area).sum();
    let weighted = |f: fn(&Coverage) -> f64| {
        if known.len() == 1 || total <= 0.0 {
            return f(&first.coverage);
        }
        known.iter().map(|p| f(&p.coverage) * p.area).sum::<f64>() / total
    };
    Some(CoverageOut {
        cols: first.coverage.cols,
        rows: first.coverage.rows,
        grid: first.coverage.rows_as_text(),
        origin: first.frame.0,
        u_axis: first.frame.1,
        v_axis: first.frame.2,
        u_range: first.domain[0],
        v_range: first.domain[1],
        shares: CoverageShares {
            measured: weighted(|c| c.shares.measured),
            hole: weighted(|c| c.shares.hole),
            extrapolated: weighted(|c| c.shares.extrapolated),
        },
    })
}

/// The counters of a fit of `tris` (the region) into `pieces`. By construction
/// `eligible = measured + excluded + failed`: `measured` counts the distinct points that at least
/// one piece measured (a vertex shared by two pieces once, and measured when either projects it),
/// `failed` is what is left of the kept points.
pub(super) fn sample_counts(mesh: &Mesh, mask: &Mask, tris: &[u32], pieces: &[Piece]) -> SampleCounts {
    let eligible = region_vertex_count(mesh, tris);
    let kept = mask.kept_ids(mesh, tris).len();
    let sets: Vec<&[u32]> = pieces.iter().map(|p| p.measured_ids.as_slice()).collect();
    count_samples(eligible, kept, &sets)
}

/// The counters from the number of points of the region, the number not excluded, and the ids of
/// the points each piece measured.
fn count_samples(eligible: usize, kept: usize, measured_by_piece: &[&[u32]]) -> SampleCounts {
    let mut union: Vec<u32> = measured_by_piece.iter().flat_map(|ids| ids.iter().copied()).collect();
    union.sort_unstable();
    union.dedup();
    let measured = union.len().min(kept);
    SampleCounts { eligible, measured, excluded: eligible.saturating_sub(kept), failed: kept - measured }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_shared_point_that_one_piece_cannot_project_is_measured_once() {
        // points 0..=9; 4 is shared: piece A fails on it, piece B measures it
        let a: Vec<u32> = vec![0, 1, 2, 3];
        let b: Vec<u32> = vec![4, 5, 6, 7, 8];
        let c = count_samples(10, 10, &[&a, &b]);
        assert_eq!((c.eligible, c.measured, c.excluded, c.failed), (10, 9, 0, 1), "9 is on no piece");
    }

    #[test]
    fn a_shared_point_that_fails_on_both_pieces_is_failed_once() {
        let a: Vec<u32> = vec![0, 1, 2];
        let b: Vec<u32> = vec![4, 5];
        let c = count_samples(6, 6, &[&a, &b]);
        assert_eq!((c.measured, c.failed), (5, 1), "{c:?}");
    }

    #[test]
    fn a_shared_point_measured_by_both_counts_once_and_exclusions_stay_apart() {
        let a: Vec<u32> = vec![0, 1, 2];
        let b: Vec<u32> = vec![2, 3];
        let c = count_samples(6, 5, &[&a, &b]);
        assert_eq!((c.eligible, c.measured, c.excluded, c.failed), (6, 4, 1, 1));
    }
}
