//! A layout split that is validated on the real geometry (rule: a split is accepted only if it does
//! not turn proper quads into improper ones). Used by `quad.split` and by the refinements of
//! `surface.auto`; independent of the conforming of the boundary.
use super::{ConformReport, QuadMesh, improper_quads, snap_to_outline};
use crate::boundary::Outline;
use surfacecraft_geom::{Point3, Vec3};

/// The outcome of [`checked_split`].
#[derive(Clone, Debug)]
pub struct CheckedSplit {
    /// The layout after the split, or the layout as it was when the split was rejected.
    pub layout: QuadMesh,
    /// How the boundary was put back onto the outline (default when it was not tried or dropped).
    pub report: ConformReport,
    /// Was the whole split rejected (a proper quad would have broken)?
    pub rejected: bool,
    /// Was the split kept without re-snapping because re-snapping would have broken a quad?
    pub resnap_dropped: bool,
}

/// Does any child of a proper parent come out improper? (`split_to_quads` makes four children per
/// quad, in parent order.)
fn breaks_a_proper_parent(parents: &[bool], children: &QuadMesh) -> bool {
    improper_quads(children).iter().enumerate().any(|(c, &bad)| bad && !parents.get(c / 4).copied().unwrap_or(false))
}

/// Splits every quad of `layout` (new points projected by `project`) and puts the free boundary
/// back onto `outline` (`edge` is the layout's edge length, `max_move` how far a vertex may move).
/// The split is accepted only if no child of a proper quad is improper on the resulting geometry;
/// if re-snapping is what breaks one, the split is tried without it; if the plain split breaks one
/// too, the layout is returned as it was and `rejected` is set.
#[must_use]
pub fn checked_split(
    layout: &QuadMesh,
    project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>,
    outline: &Outline,
    (edge, max_move): (f64, f64),
) -> CheckedSplit {
    let parents = improper_quads(layout);
    let plain = layout.split(project);
    let (snapped, report) = snap_to_outline(&plain, outline, 0.5 * edge, max_move);
    if !breaks_a_proper_parent(&parents, &snapped) {
        return CheckedSplit { layout: snapped, report, rejected: false, resnap_dropped: false };
    }
    if !breaks_a_proper_parent(&parents, &plain) {
        return CheckedSplit { layout: plain, report: ConformReport::default(), rejected: false, resnap_dropped: true };
    }
    CheckedSplit { layout: layout.clone(), report: ConformReport::default(), rejected: true, resnap_dropped: false }
}
