//! Which pieces of a scan with several parts `scan.to_solid` models (P9-O9, scans of many parts).
//!
//! Two kinds of part are modelled. A *co-equal* part has at least `part_share` (default 10 %) of the
//! area of the largest piece that is not a cavity skin; it is modelled whatever its quality, and its
//! verdict counts in full (a failed co-equal part fails the call). A *small part* is any other piece
//! that is a solid by what it is: watertight (no boundary edge), consistently oriented, enclosing a
//! positive volume, and large enough not to be scan debris ([`MIN_PART_TRIANGLES`] triangles and
//! [`MIN_PART_SPAN`] tolerances across). A small part gets its own verdict; when it does not pass,
//! its area counts with the pieces left out against [`super::prepare`]'s review bound, so it never
//! drags the call to `fail`. Every other piece is left out with its reason.
//!
//! The constants come from the corpus (numbers only, scans by number): after cleaning, the smallest
//! real part of the scan of many parts has 294 triangles and 32.9 mm across (the parts cleaning used
//! to drop there: 236 to 248 triangles, 69 to 82 mm); the closed debris on the other scans that faces
//! outward has at most 50 triangles and 1.22 mm across (8.0 mm for a flat closed sliver of no volume).
use crate::cmd::mesh_edit::{CLEAN_COMPONENT_FRACTION, CLEAN_COMPONENT_TRIANGLES};
use crate::cmd::scan_split::{Analysis, MIN_SHARE};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::Mesh;
use surfacecraft_geom::pieces::{Piece, pieces};

/// Parts modelled in one run at most (co-equal first, then small parts, each by area); the rest are
/// reported. The scan of many parts holds 81; small parts take about 3 s each.
pub(super) const MAX_COMPONENTS: usize = 128;
/// A part is co-equal when its area is at least this share of the largest piece's.
pub(super) const PART_SHARE: f64 = 0.1;
/// A small part has at least this many triangles (debris on the corpus: at most 50; parts: 236 and more).
pub(super) const MIN_PART_TRIANGLES: usize = 100;
/// ... and a bounding-box diagonal of at least this many tolerances (debris: at most 1.22 mm, parts:
/// 32.9 mm and more; 10 mm at the default tolerance of 0.2 mm).
pub(super) const MIN_PART_SPAN: f64 = 50.0;

/// Why a piece of the scan is not modelled.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum LeftOutReason {
    /// Too few triangles or too small against the tolerance to be a part.
    Debris,
    /// It has a boundary: a fragment, not a solid.
    Open,
    /// Two of its triangles run the same way over a shared edge: no consistent inside.
    NotOriented,
    /// Its triangles face inward (no positive volume): an inner skin or a piece turned inside out.
    InsideOut,
}

impl LeftOutReason {
    /// The word used in messages and in the counts of `preparation`.
    pub(super) fn word(self) -> &'static str {
        match self {
            Self::Debris => "debris",
            Self::Open => "open",
            Self::NotOriented => "not_oriented",
            Self::InsideOut => "inside_out",
        }
    }
}

/// Every piece of the cleaned scan, in the order of the split analysis (listed parts, parts past the
/// listing limit, debris), with what is known of each.
pub(super) struct Pieces {
    pub all: Vec<Piece>,
    /// Signed enclosed volume of each piece (mm³; positive when it faces outward).
    pub volumes: Vec<f64>,
    /// Per piece: the cavity skin of another (known for the listed parts).
    pub cavity: Vec<bool>,
}

impl Pieces {
    pub(super) fn of(mesh: &Mesh, a: Analysis) -> Self {
        let cavity: Vec<bool> = (0..a.parts.len() + a.extra.len() + a.rest.len())
            .map(|k| a.nests.get(k).is_some_and(|n| n.is_some_and(|n| n.1)))
            .collect();
        let all: Vec<Piece> = a.parts.into_iter().chain(a.extra).chain(a.rest).collect();
        let volumes = all.iter().map(|p| signed_volume(mesh, p)).collect();
        Self { all, volumes, cavity }
    }

    pub(super) fn area(&self, k: usize) -> f64 {
        self.all.get(k).map_or(0.0, |p| p.area)
    }

    pub(super) fn total_area(&self) -> f64 {
        self.all.iter().map(|p| p.area).sum::<f64>().max(1e-12)
    }
}

/// Signed volume enclosed by the triangles of `p`.
pub(super) fn signed_volume(mesh: &Mesh, p: &Piece) -> f64 {
    p.triangles
        .iter()
        .filter_map(|t| mesh.triangle(*t as usize))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

/// Why `p` is no small part, or `None` when it is one.
pub(super) fn reason_of(p: &Piece, volume: f64, tolerance: f64) -> Option<LeftOutReason> {
    if p.triangles.len() < MIN_PART_TRIANGLES || p.bbox.diagonal() < MIN_PART_SPAN * tolerance {
        Some(LeftOutReason::Debris)
    } else if p.boundary_edges > 0 {
        Some(LeftOutReason::Open)
    } else if p.unoriented_edges > 0 {
        Some(LeftOutReason::NotOriented)
    } else if volume.is_nan() || volume <= 0.0 {
        Some(LeftOutReason::InsideOut)
    } else {
        None
    }
}

/// The pieces of the scan by what becomes of them (indices into [`Pieces::all`]).
pub(super) struct Plan {
    /// Parts to model: the co-equal ones, then the small ones, each by area (largest first).
    pub chosen: Vec<usize>,
    /// How many of `chosen` (the first ones) are co-equal.
    pub co_equal: usize,
    /// Parts left out because the caller asked for fewer.
    pub not_requested: Vec<usize>,
    /// Parts left out by the cap of [`MAX_COMPONENTS`].
    pub beyond_cap: Vec<usize>,
    /// Pieces that are no part, with the reason.
    pub left_out: Vec<(usize, LeftOutReason)>,
    /// Inner skins of hollow parts.
    pub cavities: Vec<usize>,
}

/// Indices sorted by area, largest first (ties by index).
fn by_area(p: &Pieces, mut ks: Vec<usize>) -> Vec<usize> {
    ks.sort_by(|x, y| p.area(*y).total_cmp(&p.area(*x)).then(x.cmp(y)));
    ks
}

/// The co-equal parts (largest first) and the rest of the pieces that are not cavity skins.
fn co_equal_of(p: &Pieces, listed: usize, part_share: f64) -> (Vec<usize>, Vec<usize>) {
    let outer: Vec<usize> = (0..p.all.len()).filter(|k| !p.cavity.get(*k).copied().unwrap_or(false)).collect();
    let largest = outer.iter().map(|k| p.area(*k)).fold(0.0, f64::max);
    let (equal, rest): (Vec<usize>, Vec<usize>) =
        outer.iter().copied().partition(|k| *k < listed && p.area(*k) >= part_share * largest);
    let mut equal = by_area(p, equal);
    if equal.is_empty() {
        // never a result without a body: fall back to the largest outer piece (or the largest piece)
        let pool = if outer.is_empty() { (0..p.all.len()).collect() } else { outer };
        equal.extend(pool.into_iter().max_by(|x, y| p.area(*x).total_cmp(&p.area(*y))));
    }
    let rest = rest.into_iter().filter(|k| !equal.contains(k)).collect();
    (equal, rest)
}

/// Sorts the pieces that are not co-equal into small parts (by area) and pieces left out.
fn small_parts_of(p: &Pieces, rest: Vec<usize>, tolerance: f64) -> (Vec<usize>, Vec<(usize, LeftOutReason)>) {
    let mut small = Vec::new();
    let mut left = Vec::new();
    for k in rest {
        let (Some(piece), Some(v)) = (p.all.get(k), p.volumes.get(k)) else { continue };
        match reason_of(piece, *v, tolerance) {
            None => small.push(k),
            Some(r) => left.push((k, r)),
        }
    }
    (by_area(p, small), left)
}

/// The plan of a split run: what is modelled, in which order, and what is left out and why. A
/// request for a number of parts takes the largest co-equal ones and nothing else.
pub(super) fn plan_of(
    p: &Pieces,
    listed: usize,
    (part_share, tolerance): (f64, f64),
    requested: Option<usize>,
) -> Plan {
    let cavities: Vec<usize> = (0..p.all.len()).filter(|k| p.cavity.get(*k).copied().unwrap_or(false)).collect();
    let (mut equal, rest) = co_equal_of(p, listed, part_share);
    let (small, left_out) = small_parts_of(p, rest, tolerance);
    if let Some(n) = requested {
        let take = n.min(MAX_COMPONENTS).min(equal.len());
        let mut not_requested = equal.split_off(take);
        not_requested.extend(small);
        let co_equal = equal.len();
        return Plan { chosen: equal, co_equal, not_requested, beyond_cap: Vec::new(), left_out, cavities };
    }
    let co_equal = equal.len().min(MAX_COMPONENTS);
    let mut chosen = equal;
    chosen.extend(small);
    let beyond_cap = chosen.split_off(MAX_COMPONENTS.min(chosen.len()));
    Plan { chosen, co_equal, not_requested: Vec::new(), beyond_cap, left_out, cavities }
}

/// Does the scan hold more than one piece worth a run of its own (a co-equal part beside the
/// largest, or a small part)? A scan of one part and debris is modelled as it is.
pub(super) fn worth_splitting(plan: &Plan) -> bool {
    plan.chosen.len() + plan.not_requested.len() + plan.beyond_cap.len() >= 2
}

/// Number of co-equal parts of `mesh` (those a split run would model as co-equal; 1 for a single piece).
pub(super) fn coequal_parts(mesh: &Mesh) -> usize {
    let a = crate::cmd::scan_split::analyse(mesh, MIN_SHARE, true);
    let listed = a.parts.len();
    if listed < 2 {
        return listed;
    }
    co_equal_of(&Pieces::of(mesh, a), listed, PART_SHARE).0.len()
}

/// The smallest component the cleaning of `scan.to_solid` must keep: `Some` with a triangle count below
/// the default of `mesh.clean` when a piece under that default is a small part by [`reason_of`] (a
/// scan of many parts, where the default is a share of the largest piece); `None` otherwise, and the
/// cleaning keeps its defaults.
pub(super) fn keep_components_from(mesh: &Mesh, tolerance: f64) -> Option<usize> {
    let ps = pieces(mesh);
    let largest = ps.iter().map(|p| p.triangles.len()).max()?;
    let floor = ((largest as f64 * CLEAN_COMPONENT_FRACTION).ceil() as usize).max(CLEAN_COMPONENT_TRIANGLES);
    ps.iter()
        .filter(|p| p.triangles.len() < floor)
        .filter(|p| reason_of(p, signed_volume(mesh, p), tolerance).is_none())
        .map(|p| p.triangles.len())
        .min()
        .map(|n| n.max(CLEAN_COMPONENT_TRIANGLES))
}

#[cfg(test)]
#[path = "pipeline_parts_tests.rs"]
mod tests;
