//! The issues of `mesh.merge` in the form of the command: code, severity, scans and an English
//! message with the numbers (its Polish template is in `errors.pl.json`).
use super::mesh_merge::MergeIssueOut;
use surfacecraft_geom::merge::Issue;

fn issue(code: &str, severity: &str, scans: Vec<usize>, message: String) -> MergeIssueOut {
    MergeIssueOut { code: code.into(), severity: severity.into(), scans, message }
}

/// The issue in the form of the command, under the alignment mode `warn` (false: refuse).
pub(super) fn issue_out(i: &Issue, warn: bool) -> MergeIssueOut {
    alignment_out(i, warn).or_else(|| refusal_out(i)).or_else(|| scan_notice(i)).unwrap_or_else(|| notice_out(i))
}

/// A finding about the alignment of a pair: a refusal by default, a warning with `alignment` "warn".
fn alignment_out(i: &Issue, warn: bool) -> Option<MergeIssueOut> {
    let severity = if warn { "warning" } else { "refusal" };
    let reach = surfacecraft_geom::merge::REACH_TOLERANCES;
    Some(match i {
        Issue::Misaligned { a, b, near, over, beyond_share, rms, p95 } => {
            let share = 100.0 * beyond_share;
            let text = if warn {
                format!(
                    "scans {a} and {b} disagree where they overlap: {share:.1} % of {near} shared points lie farther apart than the tolerance (rms {rms:.4} mm, 95th percentile {p95:.4} mm), and {over} points lie over the other scan farther away than {reach} tolerances; merged anyway (alignment \"warn\"): check the result"
                )
            } else {
                format!(
                    "scans {a} and {b} disagree where they overlap: {share:.1} % of {near} shared points lie farther apart than the tolerance (rms {rms:.4} mm, 95th percentile {p95:.4} mm), and {over} points lie over the other scan farther away than {reach} tolerances; they are not averaged or stacked: align them better (align.icp, align.global), give a larger tolerance, or merge anyway with alignment \"warn\""
                )
            };
            issue("scans_misaligned", severity, vec![*a, *b], text)
        }
        Issue::Displaced { a, b, samples, over, within, offset } => {
            let text = if warn {
                format!(
                    "scan {a} lies over scan {b} displaced along its normals: {over} of {samples} points of scan {a} lie over scan {b} farther than {reach} tolerances (median {offset:.4} mm along the normal), {within} within the tolerance, and no scan that shares surface with both ties them; merged anyway (alignment \"warn\"): check the result"
                )
            } else {
                format!(
                    "scan {a} lies over scan {b} displaced along its normals: {over} of {samples} points of scan {a} lie over scan {b} farther than {reach} tolerances (median {offset:.4} mm along the normal), {within} within the tolerance, and no scan that shares surface with both ties them; they are not stacked: align them better (align.icp, align.global), add a scan that sees both, or merge anyway with alignment \"warn\""
                )
            };
            issue("scans_misaligned", severity, vec![*a, *b], text)
        }
        _ => return None,
    })
}

/// An issue that refuses, in the form of the command.
fn refusal_out(i: &Issue) -> Option<MergeIssueOut> {
    Some(match i {
        Issue::Opposed { a, b, opposed, facing } => issue(
            "opposed_normals",
            "refusal",
            vec![*a, *b],
            format!(
                "the normals of scans {a} and {b} oppose where they meet ({opposed} points on opposed surface, {facing} on facing surface): one of them is turned inside out; orient it outward and merge again"
            ),
        ),
        Issue::Winding { scan, flipped, share } => issue(
            "inconsistent_winding",
            "refusal",
            vec![*scan],
            format!(
                "{flipped} triangles of scan {scan} ({:.2} % of its area) are wound against the rest of their piece: a patch is turned inside out; repair its orientation first",
                100.0 * share
            ),
        ),
        Issue::Inverted { scan, pieces } => issue(
            "inverted_scan",
            "refusal",
            vec![*scan],
            format!(
                "{pieces} closed piece(s) of scan {scan} have a negative volume: turned inside out; orient it outward and merge again"
            ),
        ),
        Issue::InvertedUnjudged { scan, pieces } => issue(
            "inverted_scan",
            "refusal",
            vec![*scan],
            format!(
                "{pieces} closed piece(s) of scan {scan} have a negative volume and the test whether they lie inside another piece ran out of its budget (a pile of pieces or many nested shells); together they are at least {:.0} % of its area, so they may be turned inside out: repair the scan (mesh.clean) and merge again",
                100.0 * surfacecraft_geom::merge::INVERTED_MIN_SHARE
            ),
        ),
        _ => return None,
    })
}

/// An issue about the scans given that does not refuse, in the form of the command.
fn scan_notice(i: &Issue) -> Option<MergeIssueOut> {
    if let Some(out) = pair_notice(i) {
        return Some(out);
    }
    Some(match i {
        Issue::NonManifold { scan, dropped } => issue(
            "non_manifold_input",
            "warning",
            vec![*scan],
            format!(
                "{dropped} triangles of scan {scan} are exact duplicates or lie on edges of three or more triangles and were left out (they count as removed in the checks)"
            ),
        ),
        Issue::InnerSkins { scan, skins, debris, unjudged: 0 } => issue(
            "inner_skins",
            "info",
            vec![*scan],
            format!(
                "scan {scan} has {skins} closed inner skin(s) (inside another piece, facing inward as the inside of a hollow part does) and {debris} closed debris piece(s) below {:.0} % of its area together with a negative volume; they are not taken as turned inside out",
                100.0 * surfacecraft_geom::merge::INVERTED_MIN_SHARE
            ),
        ),
        Issue::InnerSkins { scan, skins, debris, unjudged } => issue(
            "inner_skins",
            "warning",
            vec![*scan],
            format!(
                "scan {scan} has {skins} closed inner skin(s) and {debris} closed debris piece(s) with a negative volume; for {unjudged} of the inner skins the test whether they lie inside another piece ran out of its budget (a pile of pieces or many nested shells): they are taken as inner skins, not as turned inside out"
            ),
        ),
        Issue::Disjoint { groups } => issue(
            "scans_disjoint",
            "info",
            groups.iter().flatten().copied().collect(),
            format!(
                "the scans fall into {} groups that share no surface ({groups:?}): the result is a union of separate pieces",
                groups.len()
            ),
        ),
        _ => return None,
    })
}

/// A notice about a pair of scans that does not refuse, in the form of the command.
fn pair_notice(i: &Issue) -> Option<MergeIssueOut> {
    Some(match i {
        Issue::LayersApart { a, b, over, related, offset, spread, one_offset } => issue(
            "layers_apart",
            "warning",
            vec![*a, *b],
            format!(
                "scans {a} and {b} meet in places but also lie over each other: {over} of the {related} points of scan {a} that relate to scan {b} lie over it farther than {} tolerances, median {offset:.4} mm along the normals, {spread:.4} mm between the quartiles ({}); either the part has a face there (a shelf over a floor) or one scan is displaced: check the alignment",
                surfacecraft_geom::merge::REACH_TOLERANCES,
                if *one_offset {
                    "one offset: a shift as a whole"
                } else {
                    "not one offset: a tilt or a curved surface"
                }
            ),
        ),
        Issue::SmallOverlap { a, b, near, over } => issue(
            "overlap_too_small",
            "info",
            vec![*a, *b],
            format!(
                "scans {a} and {b} touch by too little to judge their alignment: {} and {} points on the other, {} and {} over it (fewer than {} each)",
                near[0],
                near[1],
                over[0],
                over[1],
                surfacecraft_geom::merge::MIN_SHARED_SAMPLES
            ),
        ),
        Issue::Unjudged { a, b, samples } => issue(
            "overlap_unjudged",
            "warning",
            vec![*a, *b],
            format!(
                "{samples} points of scan {a} could not be judged against scan {b}: the search along their normal ran out of its budget (a pile of triangles)"
            ),
        ),
        _ => return None,
    })
}

/// An issue about the result, in the form of the command.
fn notice_out(i: &Issue) -> MergeIssueOut {
    match i {
        Issue::FullyCovered { scan } => issue(
            "scan_fully_covered",
            "info",
            vec![*scan],
            format!("scan {scan} lies wholly on the others: nothing of it is in the result"),
        ),
        Issue::OverBudget { refusals } => issue(
            "seam_over_budget",
            "warning",
            Vec::new(),
            format!(
                "{refusals} times a triangle of the seam or the restoration, or the fill of a hole, was left out because testing it for crossings would have compared too many triangles (a pile of triangles, or a very long one among many new ones)"
            ),
        ),
        Issue::SeamOpen { edges, length } => issue(
            "seam_open",
            "warning",
            Vec::new(),
            format!(
                "{edges} cut edges ({length:.1} mm) stayed open: no triangle there lay on scanned surface without crossing another"
            ),
        ),
        Issue::RemovedUncovered { share, worst } => issue(
            "removed_not_covered",
            "warning",
            Vec::new(),
            format!(
                "only {:.2} % of the removed area lies within the tolerance of the result (worst {worst:.3} mm)",
                100.0 * share
            ),
        ),
        Issue::Pinched { before, after } => issue(
            "pinched_vertices",
            "warning",
            Vec::new(),
            format!("the result has {after} pinched vertices (the scans had {before})"),
        ),
        other => check_notice(other),
    }
}

/// An issue of the result's own checks (and the scans' flipped triangles), in the form of the command.
fn check_notice(i: &Issue) -> MergeIssueOut {
    match i {
        Issue::FlippedDropped { scan, flipped, share } => issue(
            "flipped_triangles_dropped",
            "warning",
            vec![*scan],
            format!(
                "{flipped} triangles of scan {scan} ({:.3} % of its area) were wound against the rest of their piece and were left out (they count as removed in the checks)",
                100.0 * share
            ),
        ),
        Issue::Unoriented { edges } => issue(
            "unoriented_edges",
            "warning",
            Vec::new(),
            format!("{edges} edges of the result have two triangles that run the same way"),
        ),
        Issue::Unwelded { vertices } => issue(
            "unwelded_vertices",
            "warning",
            Vec::new(),
            format!(
                "{vertices} vertices of the result lie at (nearly) the point of a vertex of another scan without an edge between them"
            ),
        ),
        Issue::Stacked { share, triangles } => stacked_notice(*share, *triangles),
        // refusals, scan notices and notices are written by the functions above
        other @ (Issue::Misaligned { .. }
        | Issue::Displaced { .. }
        | Issue::LayersApart { .. }
        | Issue::Unjudged { .. }
        | Issue::SmallOverlap { .. }
        | Issue::Opposed { .. }
        | Issue::Winding { .. }
        | Issue::Inverted { .. }
        | Issue::InvertedUnjudged { .. }
        | Issue::InnerSkins { .. }
        | Issue::NonManifold { .. }
        | Issue::Disjoint { .. }
        | Issue::FullyCovered { .. }
        | Issue::SeamOpen { .. }
        | Issue::OverBudget { .. }
        | Issue::RemovedUncovered { .. }
        | Issue::Pinched { .. }) => issue("other", "warning", Vec::new(), format!("{other:?}")),
    }
}

/// The notice of two layers left in the result.
fn stacked_notice(share: f64, triangles: usize) -> MergeIssueOut {
    issue(
        "layers_stacked",
        "warning",
        Vec::new(),
        format!(
            "{triangles} triangles ({:.2} % of the scans' area in the result) lie on another scan's surface: two layers were left there",
            100.0 * share
        ),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unjudged_inner_skins_are_a_warning_that_names_them() {
        let o = issue_out(&Issue::InnerSkins { scan: 0, skins: 3, debris: 1, unjudged: 2 }, false);
        assert_eq!((o.code.as_str(), o.severity.as_str()), ("inner_skins", "warning"));
        assert!(o.message.contains("for 2 of the inner skins"), "{}", o.message);
        let plain = issue_out(&Issue::InnerSkins { scan: 0, skins: 3, debris: 1, unjudged: 0 }, false);
        assert_eq!(plain.severity, "info");
    }

    #[test]
    fn alignment_findings_are_refusals_by_default_and_warnings_under_warn() {
        let d = Issue::Displaced { a: 0, b: 1, samples: 100, over: 80, within: 0, offset: 2.0 };
        let refused = issue_out(&d, false);
        assert_eq!(refused.severity, "refusal");
        assert!(refused.message.contains("alignment \"warn\""), "{}", refused.message);
        let warned = issue_out(&d, true);
        assert_eq!(warned.severity, "warning");
        assert!(warned.message.contains("merged anyway"), "{}", warned.message);
    }
}
