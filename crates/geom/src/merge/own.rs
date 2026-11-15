//! Which scan keeps the surface where several cover it.
//!
//! The rule: of the scans that cover a point (facing surface within the tolerance, not past its
//! border), the one in which the point lies farthest from its own border keeps it; distances within
//! one median edge of the largest (two copies of one scan, two closed scans) go to the scan listed
//! first. Each vertex applies the rule to the scans that cover it, with their distances interpolated
//! at its projection onto them; two neighbouring vertices of different scans can therefore decide
//! differently near a tie (a gap or a narrow double strip), and what that leaves is caught by the
//! eating back below and measured by `RemovedUncovered` and `Stacked`. A scan is least reliable
//! at its border (the scanner sees the surface at a grazing angle there, and the border is ragged),
//! so this puts the seam midway between the borders, as far from both as the overlap allows. A
//! vertex another scan keeps is lost, and a triangle with a lost corner is removed, so each side
//! stops short of the seam line and leaves a narrow band for the seam.
//!
//! What is left after that are the triangles on the edge of a kept region that still lie wholly on
//! another scan's kept surface (where the comparison came out differently on the two sides). They
//! are eaten back from the border inward, one scan after the other, as Turk and Levoy (1994) remove
//! redundant triangles: a scan only eats what is covered by the triangles the scans before it kept.
use super::overlap::COS_FACING;
use super::prep::{NONE, Scan};
use crate::{Cancel, Point3, Vec3, par};

/// A point closer than this share of the tolerance to a kept triangle lies on it (also on its edge).
const ON_SURFACE_SHARE: f64 = 0.01;
/// A closest point within this barycentric share of an edge of the kept region lies on it.
const KEPT_EDGE_SNAP: f64 = 1e-9;
/// Vertices handled between two looks at the cancellation flag.
const CHUNK: usize = 1 << 15;
/// Rounds of eating back per scan at most.
const EAT_ROUNDS: usize = 6;
/// Two border distances closer than this many median edges are equal (the scan listed first keeps
/// the surface): where two scans see the same surface with the same border (a boss standing on a
/// face, seen whole by both), their distances differ by the tessellation only, and a sharper
/// comparison would hand the surface back and forth between them.
pub const TIE_EDGES: f64 = 1.0;

/// The scan that keeps a point, from the border distances `(d, scan)` of every scan covering it:
/// the largest distance, and of the scans within `tie` of it the one listed first. One rule for the
/// whole set (not scan against scan), so three scans with nearly equal distances never hand the
/// point round in a circle with each losing it.
fn winner(cands: &[(f64, usize)], tie: f64) -> Option<usize> {
    let best = cands.iter().map(|c| c.0).fold(f64::NEG_INFINITY, f64::max);
    let near = |d: f64| if best.is_infinite() { d.is_infinite() } else { d >= best - tie };
    cands.iter().filter(|c| near(c.0)).map(|c| c.1).min()
}

/// Is vertex `v` of scan `a` kept by another scan?
fn lost(scans: &[Scan], a: usize, v: usize, tol: f64, tie: f64) -> bool {
    let Some(scan) = scans.get(a) else { return false };
    let (Some(p), Some(n), Some(&da)) = (scan.mesh.positions.get(v), scan.normals.get(v), scan.border_distance.get(v))
    else {
        return false;
    };
    if n.norm_squared() == 0.0 {
        return false;
    }
    let mut cands = vec![(da, a)];
    for (x, o) in scans.iter().enumerate().filter(|(x, o)| *x != a && o.bbox.dist2(p) <= tol * tol) {
        let hit = o.bvh.closest_facing(p, tol, &[*n], COS_FACING).filter(|h| o.interior(h.triangle, &h.point));
        if let Some(h) = hit {
            cands.push((o.distance_at(h.triangle, &h.point), x));
        }
    }
    winner(&cands, tie).is_some_and(|w| w != a)
}

/// Per scan and triangle: removed because another scan keeps one of its corners; and per scan and
/// vertex: kept by another scan (lost). `None` when cancelled.
pub(super) fn removed_by_owner(
    scans: &[Scan],
    tol: f64,
    scale: f64,
    cancel: &Cancel,
) -> Option<(Vec<Vec<bool>>, Vec<Vec<bool>>)> {
    let tie = TIE_EDGES * scale;
    let mut out = Vec::with_capacity(scans.len());
    let mut lost_all = Vec::with_capacity(scans.len());
    for (a, scan) in scans.iter().enumerate() {
        let n = scan.mesh.positions.len();
        let mut lost_v = Vec::with_capacity(n);
        for start in (0..n).step_by(CHUNK) {
            if cancel.is_cancelled() {
                return None;
            }
            let end = (start + CHUNK).min(n);
            lost_v.extend(par::map(end - start, |i| lost(scans, a, start + i, tol, tie)));
        }
        let removed =
            scan.mesh.triangles.iter().map(|t| t.iter().any(|&v| lost_v.get(v as usize).copied().unwrap_or(false)));
        out.push(removed.collect());
        lost_all.push(lost_v);
    }
    Some((out, lost_all))
}

/// Is `q` within `tol` of facing surface that a scan other than `a` keeps?
fn on_kept(scans: &[Scan], removed: &[Vec<bool>], a: usize, q: &Point3, n: &Vec3, tol: f64) -> bool {
    scans.iter().enumerate().filter(|(x, o)| *x != a && o.bbox.dist2(q) <= tol * tol).any(|(x, o)| {
        let gone = removed.get(x).map_or(&[][..], Vec::as_slice);
        let keep = |t: usize| !gone.get(t).copied().unwrap_or(true) && o.face_normal(t).dot(n) >= COS_FACING;
        // on the surface (its edge included) or over its inside; not beyond its edge
        o.bvh
            .closest_filtered(q, tol, &keep)
            .is_some_and(|h| h.distance <= ON_SURFACE_SHARE * tol || inside_kept(o, gone, h.triangle, &h.point))
    })
}

/// Is the closest point `hit` on kept triangle `t` of scan `o` inside its kept surface: not on an edge
/// of the kept region (the scan's border, or an edge to a removed triangle)? A point beyond the kept
/// region finds its closest point on such an edge and is not covered by it.
fn inside_kept(o: &Scan, gone: &[bool], t: usize, hit: &Point3) -> bool {
    let (Some(tri), Some(across)) = (o.mesh.triangle(t), o.across.get(t)) else { return false };
    let w = super::prep::barycentric(&tri, hit);
    (0..3).all(|k| {
        let edge = across[k] == NONE || gone.get(across[k] as usize).copied().unwrap_or(true);
        // the barycentric coordinate opposite edge k is that of corner k + 2
        !(edge && w[(k + 2) % 3] <= KEPT_EDGE_SNAP)
    })
}

/// Triangles of scan `a` on the edge of its kept region (a neighbour across an interior edge removed).
fn kept_edge(scan: &Scan, gone: &[bool]) -> Vec<usize> {
    (0..scan.mesh.triangles.len())
        .filter(|&t| !gone.get(t).copied().unwrap_or(true))
        .filter(|&t| {
            scan.across
                .get(t)
                .is_some_and(|a| a.iter().any(|&x| x != NONE && gone.get(x as usize).copied().unwrap_or(false)))
        })
        .collect()
}

/// Is kept triangle `t` of scan `a` wholly (corners and centre) on another scan's kept surface?
fn redundant(scans: &[Scan], removed: &[Vec<bool>], a: usize, t: usize, tol: f64) -> bool {
    let Some(scan) = scans.get(a) else { return false };
    let Some([p, q, r]) = scan.mesh.triangle(t) else { return false };
    let n = scan.face_normal(t);
    if n.norm_squared() == 0.0 {
        return false;
    }
    let centre = Point3::from((p.coords + q.coords + r.coords) / 3.0);
    [p, q, r, centre].iter().all(|x| on_kept(scans, removed, a, x, &n, tol))
}

/// Eats the redundant triangles back from the edges of the kept regions, scan after scan.
/// Returns the number eaten; `None` when cancelled.
pub(super) fn eat_back(scans: &[Scan], removed: &mut [Vec<bool>], tol: f64, cancel: &Cancel) -> Option<usize> {
    let mut eaten = 0;
    for a in 0..scans.len() {
        for _ in 0..EAT_ROUNDS {
            if cancel.is_cancelled() {
                return None;
            }
            let Some(scan) = scans.get(a) else { break };
            let edge = kept_edge(scan, removed.get(a).map_or(&[][..], Vec::as_slice));
            let snapshot: &[Vec<bool>] = removed;
            let flags = par::map_slice(&edge, |&t| redundant(scans, snapshot, a, t, tol));
            let go: Vec<usize> = edge.iter().zip(flags).filter(|(_, f)| *f).map(|(t, _)| *t).collect();
            if go.is_empty() {
                break;
            }
            eaten += go.len();
            if let Some(g) = removed.get_mut(a) {
                for t in go {
                    if let Some(s) = g.get_mut(t) {
                        *s = true;
                    }
                }
            }
        }
    }
    Some(eaten)
}
