//! The boundary of each group of patches put on the group's primitive, before the group is merged
//! (P9-O6b).
//!
//! The patches of a wrap lie on a primitive only within the fit, so the edges a group shares with the
//! patches round it leave the primitive by up to the tolerance. A merged face whose boundary does that
//! is a face a receiving CAD system has to heal. Here the shared boundary is moved onto the primitive
//! first, and both sides of every moved edge move with it, so the body stays watertight:
//!
//! 1. Each vertex of a boundary goes onto the primitives of the groups that meet there (alternating
//!    closest-point projections, which end on the intersection of two primitives at a sharp edge).
//! 2. Each boundary edge, a Bézier curve, is re-fitted: its samples are put on the primitive(s) and the
//!    interior control points are the least-squares fit through them, the end points being the moved
//!    vertices. On a plane this is exact.
//! 3. Every other edge that ends at a moved vertex follows it (its end control point moves).
//! 4. The patches that use a changed edge take the new control points along the matching border of
//!    their net.
//!
//! A vertex never moves farther than `max_move` (an eighth of a tolerance away from a group's own
//! primitive is the usual size of a move; two primitives that meet at a shallow angle can ask for several
//! tolerances, and a patch pulled that far folds or leaves the scan). What cannot be done is refused per
//! group: a group with a vertex that would move too far, an edge that is not a Bézier span, or a neighbour
//! that does not take the move is left as it was and says why; the others are snapped, and the step is
//! repeated without the refused groups until nothing is refused. The result is checked: the topology is
//! the one it had, the borders of the patches that moved follow their edges, and none moved farther than
//! the limit or turned over.
//!
//! The projection is the kernel's (`SurfaceEval`, in [`Gauge`]); the least-squares fit of a Bézier curve and
//! the edit of the borders of the patches' control nets are made here.
//!
//! APP-GEOMETRY: stands in for a kernel edit of a spline network that moves a shared edge onto a surface
//! (re-fitting the edge's curve and both neighbouring patches consistently); solidcore has no such entry.
use super::boundary::edge_points;
use super::geometry::Group;
use super::place::Gauge;
use crate::hybrid::{V, add, cross, dot, norm, scale, sub, unit};
use crate::nurbs::{SurfaceProbe, domain};
use crate::record::brep::{Brep, Curve, Loop, Surface};
use crate::{KernelError, Result, finite_all};
use solidcore::math::Vec3;
use solidcore::nurbs::crv_fit::p1::ParamMethod;
use solidcore::nurbs::crv_fit::p2::approx_lst_sq;
use std::collections::{BTreeMap, BTreeSet};

/// Alternating projections that put a point on every primitive of its edge or vertex. Two primitives that meet at an
/// angle `t` converge by a factor of `cos(t)^2` a round: from 0.2 mm to [`ON_SURFACE`] in 200 rounds takes `t` of about
/// 8 degrees or more. A set of primitives that does not converge by then (shallower than that, or with no common
/// point near the point) is reported as not converged by [`put_on_all`]; [`placed`] (and so the snap) then takes the
/// point nearest to all of them and says it is not on them all, which the 0.1-gap check of a snapped group and the
/// face checks judge. Parallel or offset primitives place nothing.
const MAX_ALTERNATIONS: usize = 200;
/// A point lies on a primitive when it is this close (mm): a nanometre. A looser bound (3 micrometres) stops the
/// alternation of a shallow crease early, where the points of one crease then scatter along it by that over the sine
/// of half the angle between the primitives (0.07 mm at 5 degrees): measured on a prismatic part, the border of its
/// largest planes then no longer matched and half of the area stayed patches.
const ON_SURFACE: f64 = 1e-6;
/// A round that moves the point by less than this (mm) without having reached every primitive has found a cycle
/// (parallel primitives): there is nothing to wait for.
const STALLED: f64 = 1e-9;
/// Control points equal within this (mm) are the same point when a patch's border is matched to its edge.
const SAME_POINT: f64 = 1e-7;
/// The least dot product of the normals of a moved patch before and after, at the points of the grid where both are
/// well defined: below it the patch has turned over. (A patch that is only tilted by a border that moved keeps
/// the sign of its normal everywhere; the measured tilt of the small patches next to a moved group reached 60
/// degrees at one corner of a patch.)
const MIN_TURN_DOT: f64 = 0.0;
/// A patch's normal is read only where its tangents are at least this far from parallel (sine): a sliver or a
/// collapsed side has a normal that flips for a displacement of hundredths of a millimetre (measured: dot -0.98
/// after a move of 0.013 mm at a sine of 0.1).
const MIN_SIN_BETWEEN_TANGENTS: f64 = 0.4;
/// The fewest points of the grid at which a moved patch's normal must be readable before and after to say it has not
/// turned over (of 25).
const MIN_JUDGED: usize = 4;
/// A patch with fewer judged points passes if no point moved more than this share of the patch's width: a patch
/// turns over only when it moves by about its own size.
const MAX_UNJUDGED_MOVE: f64 = 0.25;
/// Cells a side of the grid a moved patch is compared with itself on.
const MOVED_GRID: usize = 4;

/// A group of patches and the evaluator of the primitive they lie on.
pub(super) struct Target<'a> {
    pub faces: &'a [usize],
    pub group: &'a Group,
    pub gauge: &'a Gauge,
}

/// `p` on every one of `gauges` (the points of the surfaces they evaluate), by alternating projections. `None` when
/// they do not meet within [`MAX_ALTERNATIONS`] (see there), when the point would move more than `max_move`, or when
/// it is not finite.
#[must_use]
pub fn put_on_all(gauges: &[&Gauge], p: V, max_move: f64) -> Option<V> {
    put_on_all_with(&|i, q| gauges.get(i).and_then(|g| g.closest(q)), gauges.len(), p, max_move)
}

/// [`placed`] for the gauges, without the flag: where they do not all meet, the point on the nearest one.
fn put_near(gauges: &[&Gauge], p: V, max_move: f64) -> Option<V> {
    placed(&|i, q| gauges.get(i).and_then(|g| g.closest(q)), gauges.len(), p, max_move).map(|(q, _)| q)
}

/// [`put_on_all`] for `count` surfaces whose closest point to `q` is `closest(i, q)`.
#[must_use]
pub fn put_on_all_with(closest: &dyn Fn(usize, V) -> Option<V>, count: usize, p: V, max_move: f64) -> Option<V> {
    placed(closest, count, p, max_move).and_then(|(q, on_all)| on_all.then_some(q))
}

/// Where `p` goes onto `count` surfaces, and whether that is on all of them: `(q, true)` when the alternating
/// projections meet; where they do not within the rounds (surfaces that meet at a shallow angle) the point nearest to
/// all of them among the last iterate and the projections of `p` onto each, `(q, false)`: the point is not on them
/// all, and the groups are judged by their border checks. `None` when the point would move more than `max_move`, or is not finite.
#[must_use]
pub fn placed(closest: &dyn Fn(usize, V) -> Option<V>, count: usize, p: V, max_move: f64) -> Option<(V, bool)> {
    let (q, on_all) = onto_all(closest, count, p)?;
    (count > 0 && finite_all(&q) && max_move >= 0.0 && norm(sub(q, p)) <= max_move).then_some((q, on_all))
}

fn onto_all(closest: &dyn Fn(usize, V) -> Option<V>, count: usize, p: V) -> Option<(V, bool)> {
    let distance = |i: usize, q: V| closest(i, q).map(|c| norm(sub(c, q))).filter(|d| d.is_finite());
    let on_all = |q: V| (0..count).all(|i| distance(i, q).is_some_and(|d| d <= ON_SURFACE));
    let mut q = p;
    for _ in 0..MAX_ALTERNATIONS {
        let before = q;
        for i in 0..count {
            q = closest(i, q)?;
        }
        if on_all(q) {
            return Some((q, true));
        }
        if count == 1 {
            break;
        }
        if norm(sub(q, before)) < STALLED {
            // a round that changes nothing without reaching them all: parallel (or offset) surfaces, which have no
            // common point; nothing is placed
            return None;
        }
    }
    if count == 1 {
        return Some((q, true));
    }
    // not on all of them: where the worst distance from them is least, the last iterate or the point on one alone
    // (surfaces that meet at a shallow angle come close to all of them long before they meet)
    let worst = |q: V| (0..count).filter_map(|i| distance(i, q)).fold(0.0_f64, f64::max);
    let alone = closest(0, p).into_iter().chain(std::iter::once(q));
    let best = alone.map(|c| (worst(c), c)).min_by(|a, b| a.0.total_cmp(&b.0))?;
    Some((best.1, false))
}

/// The groups that touch each boundary edge and each end vertex of one, with the gauges of their primitives.
struct Touch<'a> {
    edges: BTreeMap<u32, Vec<(usize, &'a Gauge)>>,
    vertices: BTreeMap<u32, Vec<(usize, &'a Gauge)>>,
    /// The targets (by index) that have a boundary.
    bounded: BTreeSet<usize>,
}

fn touching<'a>(body: &Brep, targets: &[Target<'a>], live: &[usize]) -> Touch<'a> {
    let mut t = Touch { edges: BTreeMap::new(), vertices: BTreeMap::new(), bounded: BTreeSet::new() };
    for &k in live {
        let Some(target) = targets.get(k) else { continue };
        t.bounded.insert(k);
        for le in target.group.boundary.loops.iter().flatten() {
            t.edges.entry(le.edge).or_default().push((k, target.gauge));
            for v in body.edges.get(le.edge as usize).map(|e| e.vertices).unwrap_or_default() {
                let list = t.vertices.entry(v).or_default();
                if !list.iter().any(|(g, _)| *g == k) {
                    list.push((k, target.gauge));
                }
            }
        }
    }
    for list in t.edges.values_mut() {
        list.dedup_by_key(|(g, _)| *g);
    }
    t
}

/// The control points of an edge that is one Bézier span over its interval.
fn bezier_of(body: &Brep, edge: u32) -> Option<&Vec<V>> {
    let e = body.edges.get(edge as usize)?;
    let Curve::Spline { knots, points } = &e.curve else { return None };
    let n = points.len().checked_sub(1).filter(|&n| n >= 1)?;
    let (lo, hi) = (*knots.first()?, *knots.last()?);
    let clamped = knots.len() == 2 * (n + 1)
        && knots.iter().take(n + 1).all(|k| *k == lo)
        && knots.iter().skip(n + 1).all(|k| *k == hi);
    (clamped && e.nurbs.is_none() && e.interval == [lo, hi]).then_some(points)
}

/// The Bézier control points (degree `n`) of the curve the kernel fits to `targets` (equally spaced in the
/// parameter) by least squares, through the first and the last of them (`nurbs::crv_fit::p2::approx_lst_sq`,
/// Piegl and Tiller A9.7). `None` for a line (the fit needs three control points) or where the kernel
/// refuses.
fn least_squares(n: usize, targets: &[V]) -> Option<Vec<V>> {
    if n + 1 > targets.len() {
        return None;
    }
    let points: Vec<Vec3> = targets.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
    let curve = approx_lst_sq(&points, n, n, ParamMethod::Uniform).ok()?;
    let single_span = curve.knots().len() == 2 * (n + 1);
    let out: Vec<V> = curve.cpts().iter().map(|c| [c.x, c.y, c.z]).collect();
    (single_span && out.len() == n + 1 && out.iter().all(|p| finite_all(p))).then_some(out)
}

/// `old` with the end points replaced by `ends` and the interior points moved along by the blend of
/// the two end displacements (the fallback of a singular fit, and the edges that only follow a vertex).
fn follow(old: &[V], ends: [V; 2]) -> Vec<V> {
    let n = old.len().saturating_sub(1).max(1);
    let (first, last) = (old.first().copied().unwrap_or(ends[0]), old.last().copied().unwrap_or(ends[1]));
    let (d0, d1) = (sub(ends[0], first), sub(ends[1], last));
    old.iter()
        .enumerate()
        .map(|(j, p)| {
            let s = j as f64 / n as f64;
            add(*p, add(scale(d0, 1.0 - s), scale(d1, s)))
        })
        .collect()
}

type Groups = BTreeSet<usize>;

/// Groups that could not be snapped, each with the first reason.
type Refused = BTreeMap<usize, String>;

fn blame(refused: &mut Refused, who: impl IntoIterator<Item = usize>, why: &str) {
    for g in who {
        refused.entry(g).or_insert_with(|| why.to_string());
    }
}

fn groups_of(who: &[(usize, &Gauge)]) -> Groups {
    who.iter().map(|(g, _)| *g).collect()
}

/// What changes: the vertices' new points, the edges' new control points (with the old ones, to find the
/// borders of the patches) and the groups that asked for each changed edge.
#[derive(Default)]
struct Moves {
    vertices: BTreeMap<u32, V>,
    edges: BTreeMap<u32, (Vec<V>, Vec<V>)>,
    asked: BTreeMap<u32, Groups>,
}

fn moved_vertices(body: &Brep, touch: &Touch, max_move: f64, refused: &mut Refused) -> BTreeMap<u32, V> {
    let mut out = BTreeMap::new();
    for (&v, who) in &touch.vertices {
        let Some(p) = body.vertices.get(v as usize).map(|x| x.point) else { continue };
        let gauges: Vec<&Gauge> = who.iter().map(|w| w.1).collect();
        match put_near(&gauges, p, max_move) {
            Some(q) => {
                out.insert(v, q);
            }
            None => blame(
                refused,
                groups_of(who),
                &format!("a vertex would have to move more than {max_move:.4} mm to lie on its primitive"),
            ),
        }
    }
    out
}

/// The ends of edge `e` after the vertices moved.
fn new_ends(body: &Brep, e: u32, vertices: &BTreeMap<u32, V>) -> Option<[V; 2]> {
    let edge = body.edges.get(e as usize)?;
    let end = |slot: usize| {
        let v = edge.vertices.get(slot).copied()?;
        vertices.get(&v).copied().or_else(|| body.vertices.get(v as usize).map(|x| x.point))
    };
    Some([end(0)?, end(1)?])
}

/// The control points of edge `e` re-fitted onto the primitives of `gauges`; `None` where a sample would
/// have to move more than `max_move`.
fn refit_edge(
    body: &Brep,
    e: u32,
    gauges: &[&Gauge],
    ends: [V; 2],
    old: &[V],
    max_move: f64,
) -> Result<Option<Vec<V>>> {
    let samples = edge_points(body, e)?;
    if old.len() > samples.len() {
        // the fit needs at least as many samples as control points (a record from outside can have any number)
        return Err(KernelError::Argument("a boundary edge has more control points than samples".into()));
    }
    let Some(targets) = samples.iter().map(|p| put_near(gauges, *p, max_move)).collect::<Option<Vec<V>>>() else {
        return Ok(None);
    };
    let mut fit = least_squares(old.len().saturating_sub(1), &targets).unwrap_or_else(|| follow(old, ends));
    // the ends are the vertices as they moved
    if let Some(first) = fit.first_mut() {
        *first = ends[0];
    }
    if let Some(last) = fit.last_mut() {
        *last = ends[1];
    }
    Ok(Some(fit))
}

/// The edges of the boundaries re-fitted, and the edges that end at a moved vertex following it.
fn plan(body: &Brep, touch: &Touch, max_move: f64) -> std::result::Result<Moves, Refused> {
    let mut refused = Refused::new();
    let vertices = moved_vertices(body, touch, max_move, &mut refused);
    if !refused.is_empty() {
        return Err(refused);
    }
    let mut moves = Moves { vertices, ..Moves::default() };
    for (&e, who) in &touch.edges {
        let (Some(old), Some(ends)) = (bezier_of(body, e), new_ends(body, e, &moves.vertices)) else {
            blame(&mut refused, groups_of(who), "a boundary edge is not a Bézier curve");
            continue;
        };
        let gauges: Vec<&Gauge> = who.iter().map(|w| w.1).collect();
        match refit_edge(body, e, &gauges, ends, old, max_move) {
            Ok(Some(new)) => {
                moves.edges.insert(e, (old.clone(), new));
                moves.asked.insert(e, groups_of(who));
            }
            Ok(None) => {
                blame(&mut refused, groups_of(who), "a boundary edge would have to move too far onto its primitive")
            }
            Err(why) => blame(&mut refused, groups_of(who), &why.to_string()),
        }
    }
    for (i, edge) in body.edges.iter().enumerate() {
        let e = i as u32;
        let moved: Vec<u32> = edge.vertices.iter().copied().filter(|v| moves.vertices.contains_key(v)).collect();
        if moves.edges.contains_key(&e) || moved.is_empty() {
            continue;
        }
        let asking: Groups = moved.iter().filter_map(|v| touch.vertices.get(v)).flat_map(|w| groups_of(w)).collect();
        match (bezier_of(body, e), new_ends(body, e, &moves.vertices)) {
            (Some(old), Some(ends)) => {
                moves.edges.insert(e, (old.clone(), follow(old, ends)));
                moves.asked.insert(e, asking);
            }
            _ => blame(&mut refused, asking, "an edge at a vertex that has to move is not a Bézier curve"),
        }
    }
    if refused.is_empty() { Ok(moves) } else { Err(refused) }
}

fn same(a: V, b: V) -> bool {
    norm(sub(a, b)) <= SAME_POINT * (1.0 + norm(a))
}

/// Does `border` run along `edge` (either way round)? `Some(true)` when it runs the same way.
fn runs_along(border: &[V], edge: &[V]) -> Option<bool> {
    if border.len() != edge.len() {
        return None;
    }
    if border.iter().zip(edge).all(|(a, b)| same(*a, *b)) {
        return Some(true);
    }
    border.iter().zip(edge.iter().rev()).all(|(a, b)| same(*a, *b)).then_some(false)
}

/// The four borders of a net as positions in the net, rows `[i][..]` and columns `[..][j]`.
fn borders(net: &[Vec<V>]) -> Vec<Vec<(usize, usize)>> {
    let (nu, nv) = (net.len(), net.first().map_or(0, Vec::len));
    let (iu, iv) = (nu.saturating_sub(1), nv.saturating_sub(1));
    vec![
        (0..nv).map(|j| (0, j)).collect(),
        (0..nv).map(|j| (iu, j)).collect(),
        (0..nu).map(|i| (i, 0)).collect(),
        (0..nu).map(|i| (i, iv)).collect(),
    ]
}

fn at(net: &[Vec<V>], (i, j): (usize, usize)) -> Option<V> {
    net.get(i).and_then(|r| r.get(j)).copied()
}

/// The points of one border of a net.
fn border_points(net: &[Vec<V>], border: &[(usize, usize)]) -> Option<Vec<V>> {
    border.iter().map(|&c| at(net, c)).collect()
}

/// A changed edge as the net of its face has it: the control points before and after, and, from the face's loop
/// (`side_of`), the border of the net it is the side of and whether the border runs the way the edge does. The loop
/// settles a collapsed side, whose border (all control points equal) runs along its edge either way round.
struct Edit<'a> {
    old: &'a [V],
    new: &'a [V],
    side: Option<(usize, bool)>,
}

/// The border (index into [`borders`]) of the net that side `s` of a four-sided loop is, and whether the border
/// runs the way the side does: the borders are net[0][..] (the fourth side, backwards), net[n][..] (the second),
/// net[..][0] (the first) and net[..][n] (the third, backwards). `forward` is whether the loop walks the edge
/// from its first vertex to its last.
fn side_of(s: usize, forward: bool) -> Option<(usize, bool)> {
    Some(match s {
        0 => (2, forward),
        1 => (1, forward),
        2 => (3, !forward),
        3 => (0, !forward),
        _ => return None,
    })
}

fn is_collapsed(points: &[V]) -> bool {
    points.first().is_some_and(|f| points.iter().all(|p| same(*p, *f)))
}

/// The net of a face with the borders that run along its changed edges replaced. A border that has collapsed to a
/// point (all its control points equal) runs along any collapsed edge at that point either way round, so it is
/// moved by the side the loop says it is. `None` when an edge of the face is changed and no border of the net runs
/// along it.
fn moved_net(net: &[Vec<V>], edits: &[Edit]) -> Option<Vec<Vec<V>>> {
    let mut out = net.to_vec();
    let mut taken: BTreeSet<usize> = BTreeSet::new();
    let candidates = borders(net);
    for e in edits {
        // a collapsed border runs along its edge either way round: only the loop can say which end is which, and
        // when the loop does not lead to a collapsed border (it starts elsewhere than the net was laid out) the
        // face is refused rather than guessed
        let found = if is_collapsed(e.old) {
            e.side.filter(|(k, _)| {
                !taken.contains(k)
                    && candidates
                        .get(*k)
                        .and_then(|b| border_points(net, b))
                        .is_some_and(|p| runs_along(&p, e.old).is_some())
            })
        } else {
            candidates.iter().enumerate().find_map(|(k, border)| {
                let same_way = border_points(net, border).and_then(|p| runs_along(&p, e.old))?;
                (!taken.contains(&k)).then_some((k, same_way))
            })
        };
        let (k, same_way) = found?;
        taken.insert(k);
        let order: Vec<&V> = if same_way { e.new.iter().collect() } else { e.new.iter().rev().collect() };
        for (&(i, j), p) in candidates.get(k)?.iter().zip(order) {
            if let Some(slot) = out.get_mut(i).and_then(|r| r.get_mut(j)) {
                *slot = *p;
            }
        }
    }
    Some(out)
}

fn edges_of(face: &crate::record::brep::Face) -> BTreeSet<u32> {
    face.loops
        .iter()
        .flat_map(|l| match l {
            Loop::Edges { edges } => edges.iter().map(|u| u.edge).collect::<Vec<_>>(),
            Loop::Vertex { .. } => Vec::new(),
        })
        .collect()
}

/// The changed edges `used` of `face` as edits of its net (with the side of its loop each is, when the face has one
/// four-sided loop).
fn edits_of<'a>(face: &crate::record::brep::Face, used: &[u32], moves: &'a Moves) -> Vec<Edit<'a>> {
    let sides: Vec<(u32, bool)> = match face.loops.as_slice() {
        [Loop::Edges { edges }] if edges.len() == 4 => edges.iter().map(|u| (u.edge, u.forward)).collect(),
        _ => Vec::new(),
    };
    used.iter()
        .filter_map(|e| {
            let (old, new) = moves.edges.get(e)?;
            let side = sides.iter().position(|(id, _)| id == e).and_then(|s| side_of(s, sides.get(s)?.1));
            Some(Edit { old, new, side })
        })
        .collect()
}

/// A patch whose net changed: the changed edges it uses and the groups that asked for the change.
struct Changed {
    face: usize,
    edges: Vec<u32>,
    asked: Groups,
}

/// The body with the moves applied to its vertices, edges and patches, and the patches that changed.
fn applied(body: &Brep, moves: &Moves) -> std::result::Result<(Brep, Vec<Changed>), Refused> {
    let mut out = body.clone();
    for (&v, p) in &moves.vertices {
        if let Some(slot) = out.vertices.get_mut(v as usize) {
            slot.point = *p;
        }
    }
    for (&e, (_, new)) in &moves.edges {
        if let Some(Curve::Spline { points, .. }) = out.edges.get_mut(e as usize).map(|x| &mut x.curve) {
            *points = new.clone();
        }
    }
    let (mut changed, mut refused) = (Vec::new(), Refused::new());
    for (fi, face) in body.faces.iter().enumerate() {
        let used: Vec<u32> = edges_of(face).into_iter().filter(|e| moves.edges.contains_key(e)).collect();
        if used.is_empty() {
            continue;
        }
        let asked: Groups = used.iter().filter_map(|e| moves.asked.get(e)).flatten().copied().collect();
        let Surface::Spline { points, .. } = &face.surface else {
            blame(&mut refused, asked, "a patch at a moved edge is not a spline");
            continue;
        };
        let edits = edits_of(face, &used, moves);
        let Some(net) = moved_net(points, &edits) else {
            blame(&mut refused, asked, "a patch's border does not match its edge");
            continue;
        };
        if let Some(Surface::Spline { points, .. }) = out.faces.get_mut(fi).map(|f| &mut f.surface) {
            *points = net;
        }
        changed.push(Changed { face: fi, edges: used, asked });
    }
    if refused.is_empty() { Ok((out, changed)) } else { Err(refused) }
}

/// Does patch `face` of `after` have a border along the control points of each of `edges` (either way round),
/// each border for one edge?
fn borders_follow_edges(after: &Brep, face: usize, edges: &[u32]) -> bool {
    let Some(Surface::Spline { points, .. }) = after.faces.get(face).map(|f| &f.surface) else { return false };
    let mut taken = BTreeSet::new();
    edges.iter().all(|&e| {
        let Some(edge) = bezier_of(after, e) else { return false };
        let found = borders(points).iter().enumerate().find_map(|(k, b)| {
            let along = border_points(points, b).and_then(|p| runs_along(&p, edge));
            (!taken.contains(&k) && along.is_some()).then_some(k)
        });
        found.is_some_and(|k| taken.insert(k))
    })
}

/// The unit normals of patch `face` at the `(n + 1) x (n + 1)` grid of its domain, `None` where the patch is
/// degenerate (a collapsed side, a fold).
fn normals_at_grid(body: &Brep, face: usize, n: usize) -> Option<Vec<Option<V>>> {
    let f = body.faces.get(face)?;
    let [[u0, u1], [v0, v1]] = domain(&f.surface).ok()?;
    let probe = SurfaceProbe::new(&f.surface).ok()?;
    let (du, dv) = (1e-3 * (u1 - u0), 1e-3 * (v1 - v0));
    let mut out = Vec::with_capacity((n + 1) * (n + 1));
    for i in 0..=n {
        for j in 0..=n {
            let u = (u0 + (u1 - u0) * i as f64 / n as f64).clamp(u0 + du, u1 - du);
            let v = (v0 + (v1 - v0) * j as f64 / n as f64).clamp(v0 + dv, v1 - dv);
            let a = sub(probe.eval(u + du, v).ok()?, probe.eval(u - du, v).ok()?);
            let b = sub(probe.eval(u, v + dv).ok()?, probe.eval(u, v - dv).ok()?);
            let c = cross(a, b);
            out.push((norm(c) >= MIN_SIN_BETWEEN_TANGENTS * norm(a) * norm(b)).then(|| unit(c)).flatten());
        }
    }
    Some(out)
}

/// The width of the box of `points` (mm): its second largest side (a long thin patch is as wide as it is thin).
fn width(points: &[V]) -> f64 {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in points {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let mut sides = [hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]];
    sides.sort_by(f64::total_cmp);
    sides[1]
}

/// A patch that moved: no point of it farther than `max_move` from where it was, still facing the way it did,
/// and its borders along its changed edges.
fn moved_patch_ok(before: &Brep, after: &Brep, changed: &Changed, max_move: f64) -> bool {
    use super::samples::face_samples;
    let face = changed.face;
    let (Some(a), Some(b)) = (face_samples(before, face, MOVED_GRID), face_samples(after, face, MOVED_GRID)) else {
        return false;
    };
    let far = a.iter().zip(&b).map(|(p, q)| norm(sub(*p, *q))).fold(0.0_f64, f64::max);
    // (a NaN displacement is no displacement `max` would see: every one is checked)
    let near = a.len() == b.len() && a.iter().zip(&b).all(|(p, q)| norm(sub(*p, *q)) <= max_move * (1.0 + 1e-9));
    let facing = match (normals_at_grid(before, face, MOVED_GRID), normals_at_grid(after, face, MOVED_GRID)) {
        (Some(n0), Some(n1)) => {
            let pairs: Vec<(V, V)> = n0.iter().zip(&n1).filter_map(|(a, b)| Some(((*a)?, (*b)?))).collect();
            // a patch whose normal can be read at too few points of the grid (a sliver, a collapsed side) cannot be
            // shown not to have turned over: it passes only when it moved too little to turn
            let judged = pairs.len() >= MIN_JUDGED || far <= MAX_UNJUDGED_MOVE * width(&a);
            judged && pairs.iter().all(|(a, b)| dot(*a, *b) >= MIN_TURN_DOT)
        }
        _ => false,
    };
    near && facing && borders_follow_edges(after, face, &changed.edges)
}

/// Every edge of `after` is used as often as it was (once on an open border, twice inside), nothing was added
/// or removed.
fn topology_kept(before: &Brep, after: &Brep) -> bool {
    before.faces.len() == after.faces.len()
        && before.edges.len() == after.edges.len()
        && before.vertices.len() == after.vertices.len()
        && before.edges.iter().zip(&after.edges).all(|(a, b)| a.uses == b.uses && matches!(b.uses.len(), 1 | 2))
}

/// The patches that moved but are not part of a snapped group (those are replaced by their face) must be
/// sound; the groups that asked for a patch that is not are refused. An unsound topology refuses every group.
fn check_moved(
    before: &Brep,
    after: &Brep,
    changed: &[Changed],
    own: &BTreeSet<usize>,
    live: &[usize],
    max_move: f64,
) -> Refused {
    let mut refused = Refused::new();
    if !topology_kept(before, after) {
        blame(&mut refused, live.iter().copied(), "the snap changed the topology of the body");
        return refused;
    }
    for c in changed {
        if !own.contains(&c.face) && !moved_patch_ok(before, after, c, max_move) {
            blame(
                &mut refused,
                c.asked.iter().copied(),
                "a neighbouring patch would fold, turn over or come off its border",
            );
        }
    }
    refused
}

/// What [`snap_boundaries`] made: the body, which targets (by index) have their boundary on the primitive and,
/// of those that have not, why.
pub(super) struct Snapped {
    pub body: Brep,
    pub snapped: Vec<bool>,
    pub why_not: Vec<Option<String>>,
}

/// The body with the boundary of every group of `targets` on its primitive, and the patches and edges that
/// share it moved with it. A group that cannot be snapped (see the header) is left as it was and its reason
/// is in [`Snapped::why_not`]; the others are snapped without it. No vertex moves farther than `max_move`. A
/// cancellation (`cancel` is polled once a round) ends the step with nothing snapped.
pub(super) fn snap_boundaries(body: &Brep, targets: &[Target], max_move: f64, cancel: &dyn Fn() -> bool) -> Snapped {
    let mut why_not: Vec<Option<String>> = vec![None; targets.len()];
    let mut live: Vec<usize> = (0..targets.len()).collect();
    // each round refuses at least one group, or ends
    for _ in 0..=targets.len() {
        if cancel() {
            break;
        }
        let touch = touching(body, targets, &live);
        let flags = |touch: &Touch| (0..targets.len()).map(|k| touch.bounded.contains(&k)).collect::<Vec<_>>();
        if touch.edges.is_empty() {
            return Snapped { body: body.clone(), snapped: flags(&touch), why_not };
        }
        let own: BTreeSet<usize> =
            live.iter().filter_map(|&k| targets.get(k)).flat_map(|t| t.faces.iter().copied()).collect();
        let attempt = plan(body, &touch, max_move).and_then(|m| {
            let (after, changed) = applied(body, &m)?;
            let refused = check_moved(body, &after, &changed, &own, &live, max_move);
            if refused.is_empty() { Ok(after) } else { Err(refused) }
        });
        match attempt {
            Ok(after) => return Snapped { body: after, snapped: flags(&touch), why_not },
            Err(refused) if !refused.is_empty() => {
                for (k, why) in refused {
                    if let Some(slot) = why_not.get_mut(k) {
                        *slot = Some(why);
                    }
                    live.retain(|&g| g != k);
                }
            }
            Err(_) => break,
        }
    }
    Snapped {
        body: body.clone(),
        snapped: vec![false; targets.len()],
        why_not: vec![Some("the boundaries could not be moved consistently".to_string()); targets.len()],
    }
}

#[cfg(test)]
#[path = "snap_tests.rs"]
mod tests;
