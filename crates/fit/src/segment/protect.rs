//! Protection of large primitives from the freeform consolidation, by what the primitive does for
//! a model rather than by how well it fits the scanner noise.
//!
//! A real manufactured face has form error (waviness, draft, wear, warp) far above the noise of a
//! good scan: a plane that holds a tenth of a part's surface within a few hundredths of a
//! millimetre is a design plane even though its residual is many times the noise. A fragment of a
//! freeform surface is small. So a region is *protected* when it is large (a share of the scan's
//! AREA, never fewer than [`MIN_TRIANGLES`] triangles), spans enough of its primitive, and either
//! - its form error, the spread of its residuals with the scanner noise taken out in quadrature,
//!   is within [`FORM_ERROR_SHARE`] of the tolerance; or
//! - it bows little: the smooth deviation from the primitive that a second-order fit of small
//!   patches explains (see `bow`) is within [`BOW_TOLERANCE_SHARE`] of the tolerance, and small
//!   against the region's own size: at most [`BOW_SLOPE`] of its extent, or at most
//!   [`DOMINANT_BOW_SLOPE`] when the region is [`DOMINANT_SHARE`] of the scan or more.
//!
//! The constants are a calibrated heuristic, not a separator: the two populations overlap. Bow
//! over extent of genuine large faces on three real scans ran 2e-4 to 1.2e-3 (the large sphere of
//! one: 1.2e-3); that of the large fragments of a synthetic wavy surface had minimum 6.8e-4, 5th
//! percentile 1.0 to 1.5e-3, median 2e-3. So by construction (a) a genuine face under 5 % of the
//! scan with a bow over 5e-4 of its extent is lost to freeform; (b) a freeform fragment of 5 % of the
//! scan or more with a bow under 2e-3 of its extent is protected; (c) the 5 % tier rests on three
//! real scans and one synthetic family (no fragment there reached 3.2 %). The scan-wide `tolerance`
//! moves the bow limit; there is no per-region override: a region wrongly merged can be recreated
//! with `region.create` and fitted with `primitive.fit`, or the whole step skipped with
//! `options.freeform: false`.
use super::Segment;
use super::bow::region_bow;
use super::freeform::{curvature_support, extent, residual_spread, sample_corners};
use crate::primitive::PrimitiveKind;
use surfacecraft_geom::{Aabb, Mesh};

/// A primitive is protected from this share of the scan's area, and never from fewer triangles than
/// [`MIN_TRIANGLES`]. (The large fragments of a wavy surface hold up to 3.2 % of a small scene and
/// those of three organic scans at most 1.0 %; what keeps them out is the form-error, bow and span
/// tests below, not the share.)
const MIN_SHARE: f64 = 0.01;
const MIN_TRIANGLES: usize = 500;
/// The form error of a protected region is at most this share of the tolerance. Calibrated
/// (`p9_noise_calibration`, `print_tolerance_table`, and the wavy-top scenes): planes with form
/// error read 0.02 to 0.135 of a tolerance of 0.2 mm and cylinders 0.02 to 0.05, but the large
/// organic fragments read 0.13 and above before the scanner noise is taken out, and as little as
/// 0.096 after it (a cone fragment of the wavy-top scene at noise 0.03). A plane with more form
/// error than this share is protected by its bow instead (see below).
pub const FORM_ERROR_SHARE: f64 = 0.08;
/// The RMS of a protected region is at most this share of the tolerance (a shoulder where it grew
/// into a tangent neighbour inflates the RMS, not the spread).
const RMS_CAP_SHARE: f64 = 0.5;
/// The bow of a protected region is at most this share of the tolerance ...
const BOW_TOLERANCE_SHARE: f64 = 0.5;
/// ... and at most this fraction of its extent. Calibrated (`print_bow_table` and the real scans,
/// numbers in the pull request): genuine large faces read 2e-4 to 1.2e-3; the large fragments of a
/// wavy surface 1.0 to 1.5e-3 at the 5th percentile (median 2e-3, smallest 6.8e-4).
const BOW_SLOPE: f64 = 5e-4;
/// From this share of the scan's area (no organic fragment reached it) a curved region needs no
/// span of a radius and the bow may be this large.
const DOMINANT_SHARE: f64 = 0.05;
const DOMINANT_BOW_SLOPE: f64 = 2e-3;
/// A plane spans enough when its extent is at least this many tolerances, or ...
const PLANE_EXTENT_TOLERANCES: f64 = 50.0;
/// ... this fraction of the scan's diagonal, whichever is less.
const PLANE_EXTENT_DIAGONAL: f64 = 0.2;

/// The scan as the protection sees it: its area and the diagonal of its box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Scene {
    pub area: f64,
    pub diagonal: f64,
}

impl Scene {
    pub(super) fn of(mesh: &Mesh) -> Self {
        let area: f64 = surfacecraft_geom::par::map(mesh.triangle_count(), |t| mesh.face_area(t)).iter().sum();
        Self { area, diagonal: Aabb::from_points(&mesh.positions).diagonal() }
    }
}

fn region_area(mesh: &Mesh, seg: &Segment) -> f64 {
    seg.triangles.iter().map(|&t| mesh.face_area(t as usize)).sum()
}

/// Does `seg` (a `share` of the scan's area) span enough of its primitive: a curved one at least a
/// radius, or [`DOMINANT_SHARE`] of the scan (a large gentle patch of a big cylinder or sphere); a
/// plane [`PLANE_EXTENT_TOLERANCES`] tolerances or [`PLANE_EXTENT_DIAGONAL`] of the scan, whichever
/// is less.
fn spans(mesh: &Mesh, seg: &Segment, tolerance: f64, scene: &Scene, share: f64) -> bool {
    match seg.primitive.kind() {
        PrimitiveKind::Plane => {
            let least = (PLANE_EXTENT_TOLERANCES * tolerance).min(PLANE_EXTENT_DIAGONAL * scene.diagonal);
            extent(&sample_corners(mesh, &seg.triangles), None) >= least
        }
        _ => share >= DOMINANT_SHARE || curvature_support(mesh, seg) >= 1.0,
    }
}

/// Is `seg` protected by the tolerance (see the module documentation); `noise` is the scanner noise
/// sigma (mm) taken out of the residual spread, 0 when it is not known.
pub(super) fn protects(mesh: &Mesh, seg: &Segment, tolerance: f64, noise: f64, scene: &Scene) -> bool {
    if seg.triangles.len() < MIN_TRIANGLES || seg.rms > RMS_CAP_SHARE * tolerance || scene.area <= 0.0 {
        return false;
    }
    let share = region_area(mesh, seg) / scene.area;
    if share < MIN_SHARE || !spans(mesh, seg, tolerance, scene, share) {
        return false;
    }
    let spread = residual_spread(mesh, &seg.triangles, &seg.primitive);
    if (spread * spread - noise * noise).max(0.0).sqrt() <= FORM_ERROR_SHARE * tolerance {
        return true;
    }
    let Some(bow) = region_bow(mesh, &seg.triangles, &seg.primitive) else { return false };
    let slope = if share >= DOMINANT_SHARE { DOMINANT_BOW_SLOPE } else { BOW_SLOPE };
    bow.bow() <= BOW_TOLERANCE_SHARE * tolerance && bow.bow() <= slope * bow.extent
}

/// [`protects`] for the region `seg` of the scan `mesh`: the tolerance (and the bow) protects it
/// from consolidation into a freeform area. `noise` is the scanner noise sigma (mm), 0 if unknown.
#[must_use]
pub fn protected_by_tolerance(mesh: &Mesh, seg: &Segment, tolerance: f64, noise: f64) -> bool {
    protects(mesh, seg, tolerance, noise, &Scene::of(mesh))
}
