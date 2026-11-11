//! Joins the walls and rounds of a floor into loops of lines and arcs.
//!
//! Every element is travelled with the void on its left, so a loop that goes round a pocket runs
//! counter-clockwise (positive area) and a loop that goes round an island or a rib runs clockwise
//! (negative area). An element leads into the one whose start is nearest its end; where they meet
//! is a joint (a corner of two lines, the tangent point of a line and an arc, the point two arcs
//! share). A rounded corner between two walls is solved from the walls, so that its arc is exactly
//! tangent to both.
use super::NEIGHBOUR_GAP;
use super::extent::{arc_ends, wall_ends};
use super::joint::{Bounds, Part, join};
use super::profile::{
    Corner, Seg, Wall, check_input, crosses_itself, d2, polygon_of, seg_is_sound, signed_area, solve, tangent,
};
use super::scan::Scale;
use std::f64::consts::{PI, TAU};

/// Shortest piece of a profile (mm at the reference spacing; so are the lengths below).
const MIN_LENGTH: f64 = 0.05;
/// Smallest area of a loop (mm² at the reference spacing).
const MIN_AREA: f64 = 0.5;
/// Smallest distance between a round's data and its fit's centre that is always allowed (mm).
const ROUND_CENTRE_FLOOR: f64 = 1.0;
/// How far a half round may differ from half the distance between its walls (mm).
const HALF_ROUND_SLACK: f64 = 0.6;
/// Largest sweep of a rounded corner between two walls, radians (a half round plus slack).
const MAX_ROUND_SWEEP: f64 = PI + 0.2;
/// Smallest sweep of a rounded corner between two walls, radians.
const MIN_ROUND_SWEEP: f64 = 0.05;
/// Angular slack when asking whether an arc covers a point, radians.
const COVER_SLACK: f64 = 0.05;

/// A loop of lines and arcs in the plane of a floor.
#[derive(Debug)]
pub(super) struct Loop {
    pub(super) segs: Vec<Seg>,
    /// The loop's outline (arcs sampled).
    pub(super) polygon: Vec<[f64; 2]>,
    /// Closed by the elements themselves (an open chain is closed far outside).
    pub(super) closed: bool,
    /// Signed area: positive when the void is inside (a pocket), negative when it is outside (an
    /// island, a rib).
    pub(super) area: f64,
    /// The planner's nodes of the walls and rounds that form it.
    pub(super) nodes: Vec<usize>,
    /// The arcs as built: (node, centre, radius).
    pub(super) arcs: Vec<(usize, [f64; 2], f64)>,
    /// A point inside the part that belongs to the loop (the middle of its first element).
    pub(super) probe: [f64; 2],
}

#[derive(Clone, Copy)]
enum Kind {
    Wall(usize),
    Arc(usize),
}

/// An element with the ends it has in travel order.
struct Elem {
    kind: Kind,
    begin: [f64; 2],
    end: [f64; 2],
}

struct Link {
    from: usize,
    to: usize,
    gap: f64,
    vertex: [f64; 2],
}

struct Round {
    center: [f64; 2],
    r: f64,
    t1: [f64; 2],
    t2: [f64; 2],
}

struct Path {
    nodes: Vec<usize>,
    closed: bool,
}

struct Ctx<'a> {
    walls: &'a [Wall],
    corners: &'a [Corner],
    scale: Scale,
}

impl Ctx<'_> {
    fn wall(&self, i: usize) -> Option<&Wall> {
        self.walls.get(i)
    }
    fn corner(&self, i: usize) -> Option<&Corner> {
        self.corners.get(i)
    }
}

fn dist_to(pts: &[[f64; 2]], p: [f64; 2]) -> f64 {
    pts.iter().map(|q| d2(*q, p)).fold(f64::INFINITY, f64::min)
}

fn along(a: [f64; 2], b: [f64; 2], t: [f64; 2]) -> f64 {
    (a[0] - b[0]) * t[0] + (a[1] - b[1]) * t[1]
}

fn angle_of(p: [f64; 2], c: [f64; 2]) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}

// ---------------------------------------------------------------------------------------------
// elements and links

fn elements(ctx: &Ctx) -> Vec<Elem> {
    let walls = ctx.walls.iter().enumerate().map(|(i, w)| {
        let (begin, end) = wall_ends(w, ctx.scale);
        Elem { kind: Kind::Wall(i), begin, end }
    });
    let arcs = ctx.corners.iter().enumerate().filter(|(_, k)| !k.full).map(|(i, k)| {
        let (begin, end) = arc_ends(k, ctx.scale);
        Elem { kind: Kind::Arc(i), begin, end }
    });
    walls.chain(arcs).collect()
}

/// Does the element `i` lead into `j`, and where do they meet?
fn link_between(ctx: &Ctx, elems: &[Elem], i: usize, j: usize) -> Option<Link> {
    let (a, b) = (part_of(ctx, elems.get(i)?.kind)?, part_of(ctx, elems.get(j)?.kind)?);
    let joint = join(a, b, ctx.scale)?;
    Some(Link { from: i, to: j, gap: joint.score, vertex: joint.vertex })
}

fn part_of<'a>(ctx: &'a Ctx<'a>, k: Kind) -> Option<Part<'a>> {
    match k {
        Kind::Wall(w) => ctx.wall(w).map(Part::Wall),
        Kind::Arc(c) => ctx.corner(c).map(Part::Arc),
    }
}

/// Each element leads into at most one other, the nearest first. The pairs worth a look are those
/// whose boxes come near each other (each box is computed once).
fn match_links(ctx: &Ctx, elems: &[Elem]) -> Vec<Option<Link>> {
    let n = elems.len();
    let boxes: Vec<Option<Bounds>> = elems.iter().map(|e| part_of(ctx, e.kind).map(|p| Bounds::of(p.pts()))).collect();
    let gap = ctx.scale.mm(NEIGHBOUR_GAP);
    let mut links: Vec<Link> = (0..n)
        .flat_map(|i| (0..n).filter(move |&j| j != i).map(move |j| (i, j)))
        .filter(|&(i, j)| match (boxes.get(i).copied().flatten(), boxes.get(j).copied().flatten()) {
            (Some(a), Some(b)) => a.near(&b, gap),
            _ => false,
        })
        .filter_map(|(i, j)| link_between(ctx, elems, i, j))
        .collect();
    links.sort_by(|a, b| a.gap.total_cmp(&b.gap));
    let mut has_pred = vec![false; n];
    let mut succ: Vec<Option<Link>> = (0..n).map(|_| None).collect();
    for l in links {
        let free = succ.get(l.from).is_some_and(Option::is_none) && !has_pred.get(l.to).copied().unwrap_or(true);
        if free {
            if let Some(p) = has_pred.get_mut(l.to) {
                *p = true;
            }
            let from = l.from;
            if let Some(s) = succ.get_mut(from) {
                *s = Some(l);
            }
        }
    }
    succ
}

/// The chains (from an element nothing leads into) and the cycles of the links.
fn paths(succ: &[Option<Link>]) -> Vec<Path> {
    let n = succ.len();
    let mut has_pred = vec![false; n];
    for l in succ.iter().flatten() {
        if let Some(p) = has_pred.get_mut(l.to) {
            *p = true;
        }
    }
    let mut seen = vec![false; n];
    let mut out = Vec::new();
    let starts = (0..n).filter(|&i| has_pred.get(i).is_some_and(|p| !p)).chain(0..n);
    for s in starts {
        if seen.get(s).copied().unwrap_or(true) {
            continue;
        }
        let mut nodes = vec![s];
        let mut closed = false;
        if let Some(x) = seen.get_mut(s) {
            *x = true;
        }
        let mut cur = s;
        while let Some(l) = succ.get(cur).and_then(Option::as_ref) {
            if l.to == s {
                closed = true;
                break;
            }
            if seen.get(l.to).copied().unwrap_or(true) {
                break;
            }
            if let Some(x) = seen.get_mut(l.to) {
                *x = true;
            }
            nodes.push(l.to);
            cur = l.to;
        }
        out.push(Path { nodes, closed });
    }
    out
}

// ---------------------------------------------------------------------------------------------
// rounded corners and arcs

/// A half round between two walls that face each other (a slot's end, a rib's end): the centre
/// moves to the middle line and the radius is half the distance. `s` is +1 for a round of the void.
fn half_round(a: &Wall, b: &Wall, k: &Corner, s: f64, sc: Scale) -> Option<([f64; 2], f64)> {
    if a.n[0] * b.n[0] + a.n[1] * b.n[1] > -0.9 {
        return None;
    }
    let width = -s * (a.c + b.c);
    if width < sc.mm(0.2) || (k.r - 0.5 * width).abs() > sc.mm(HALF_ROUND_SLACK) {
        return None;
    }
    let mid = 0.5 * (a.c - b.c);
    let shift = mid - (a.n[0] * k.center[0] + a.n[1] * k.center[1]);
    Some(([k.center[0] + a.n[0] * shift, k.center[1] + a.n[1] * shift], 0.5 * width))
}

/// The round `k` between the walls `a` and `b`, solved from the walls (exactly tangent to both),
/// if the data agrees.
fn fillet(a: &Wall, b: &Wall, k: &Corner, sc: Scale) -> Option<Round> {
    let s = if k.concave { 1.0 } else { -1.0 };
    let parallel = (a.n[0] * b.n[1] - a.n[1] * b.n[0]).abs() < 0.1;
    let (center, r) = if parallel {
        half_round(a, b, k, s, sc)?
    } else {
        let center = solve(a.n, a.c + s * k.r, b.n, b.c + s * k.r)?;
        (d2(center, k.center) <= (0.3 * k.r).max(sc.mm(ROUND_CENTRE_FLOOR))).then_some((center, k.r))?
    };
    let t1 = [center[0] - s * a.n[0] * r, center[1] - s * a.n[1] * r];
    let t2 = [center[0] - s * b.n[0] * r, center[1] - s * b.n[1] * r];
    if !parallel {
        let gap = sc.mm(NEIGHBOUR_GAP);
        let touches = dist_to(&a.pts, t1) <= gap
            && dist_to(&b.pts, t2) <= gap
            && dist_to(&k.pts, t1) <= gap
            && dist_to(&k.pts, t2) <= gap;
        let (p, q) = (angle_of(t1, center), angle_of(t2, center));
        let sweep = if k.concave { (q - p).rem_euclid(TAU) } else { (p - q).rem_euclid(TAU) };
        if !touches || !(MIN_ROUND_SWEEP..=MAX_ROUND_SWEEP).contains(&sweep) {
            return None;
        }
    }
    Some(Round { center, r, t1, t2 })
}

/// The arc of a round from `b` to `e` about `center`, travelled the way the round's kind says.
fn arc_seg(center: [f64; 2], r: f64, b: [f64; 2], e: [f64; 2], concave: bool) -> Seg {
    let (a0, a1) = (angle_of(b, center), angle_of(e, center));
    let sweep = if concave { (a1 - a0).rem_euclid(TAU) } else { -(a0 - a1).rem_euclid(TAU) };
    Seg::Arc { c: center, r, a0, a1: a0 + sweep }
}

fn arc_covers(seg: &Seg, p: [f64; 2]) -> bool {
    let Seg::Arc { c, a0, a1, .. } = seg else { return false };
    let th = angle_of(p, *c);
    if a1 >= a0 {
        (th - a0).rem_euclid(TAU) <= a1 - a0 + COVER_SLACK
    } else {
        (a0 - th).rem_euclid(TAU) <= a0 - a1 + COVER_SLACK
    }
}

/// The circle of radius ~`k.r` through `b` and `e` that the round's points lie on.
fn arc_through(k: &Corner, b: [f64; 2], e: [f64; 2], sc: Scale) -> Result<(Seg, [f64; 2], f64), String> {
    let chord = d2(b, e);
    if chord < sc.mm(MIN_LENGTH) {
        return Err("a round has no length between its neighbours".into());
    }
    let half = 0.5 * chord;
    if half - k.r > sc.mm(HALF_ROUND_SLACK) {
        return Err(format!("the round r = {:.2} is shorter than the {chord:.2} mm between its neighbours", k.r));
    }
    let r = k.r.max(half);
    let h = (r * r - half * half).max(0.0).sqrt();
    let mid = [0.5 * (b[0] + e[0]), 0.5 * (b[1] + e[1])];
    let perp = [-(e[1] - b[1]) / chord, (e[0] - b[0]) / chord];
    let score = |c: [f64; 2]| {
        let seg = arc_seg(c, r, b, e, k.concave);
        let covered = k.pts.iter().filter(|p| arc_covers(&seg, **p)).count() as f64 / k.pts.len().max(1) as f64;
        covered - 0.01 * d2(c, k.center)
    };
    let (c1, c2) = ([mid[0] + perp[0] * h, mid[1] + perp[1] * h], [mid[0] - perp[0] * h, mid[1] - perp[1] * h]);
    let center = if score(c1) >= score(c2) { c1 } else { c2 };
    Ok((arc_seg(center, r, b, e, k.concave), center, r))
}

/// A post (a full round) as a loop of its own.
fn circle_loop(k: &Corner) -> Loop {
    let half = if k.concave { PI } else { -PI };
    let segs = vec![
        Seg::Arc { c: k.center, r: k.r, a0: 0.0, a1: half },
        Seg::Arc { c: k.center, r: k.r, a0: half, a1: 2.0 * half },
    ];
    let polygon = polygon_of(&segs);
    let area = signed_area(&polygon);
    let probe = k.center;
    Loop { segs, polygon, closed: true, area, nodes: vec![k.id], arcs: vec![(k.id, k.center, k.r)], probe }
}

// ---------------------------------------------------------------------------------------------
// paths into loops

/// Start and end vertex of every element of a path, and the exactly-solved arcs.
struct Joints {
    vin: Vec<Option<[f64; 2]>>,
    vout: Vec<Option<[f64; 2]>>,
    round: Vec<Option<Round>>,
}

fn joints(ctx: &Ctx, elems: &[Elem], succ: &[Option<Link>], p: &Path) -> Joints {
    let m = p.nodes.len();
    let mut j = Joints { vin: vec![None; m], vout: vec![None; m], round: (0..m).map(|_| None).collect() };
    for k in 0..m {
        let Some(l) = p.nodes.get(k).and_then(|&n| succ.get(n)).and_then(Option::as_ref) else { continue };
        if k + 1 == m && !p.closed {
            continue;
        }
        if let (Some(o), Some(i)) = (j.vout.get_mut(k), j.vin.get_mut((k + 1) % m)) {
            *o = Some(l.vertex);
            *i = Some(l.vertex);
        }
    }
    for kc in 0..m {
        let (prev, next) = if p.closed { ((kc + m - 1) % m, (kc + 1) % m) } else { (kc.wrapping_sub(1), kc + 1) };
        let kinds = [prev, kc, next].map(|i| p.nodes.get(i).and_then(|&n| elems.get(n)).map(|e| e.kind));
        let [Some(Kind::Wall(a)), Some(Kind::Arc(c)), Some(Kind::Wall(b))] = kinds else { continue };
        let (Some(wa), Some(wb), Some(k)) = (ctx.wall(a), ctx.wall(b), ctx.corner(c)) else { continue };
        let Some(rd) = fillet(wa, wb, k, ctx.scale) else { continue };
        if let Some(v) = j.vout.get_mut(prev) {
            *v = Some(rd.t1);
        }
        if let Some(v) = j.vin.get_mut(kc) {
            *v = Some(rd.t1);
        }
        if let Some(v) = j.vout.get_mut(kc) {
            *v = Some(rd.t2);
        }
        if let Some(v) = j.vin.get_mut(next) {
            *v = Some(rd.t2);
        }
        if let Some(slot) = j.round.get_mut(kc) {
            *slot = Some(rd);
        }
    }
    j
}

/// The line of a wall between its joints; a free end runs out by `ext`.
fn wall_seg(w: &Wall, vin: Option<[f64; 2]>, vout: Option<[f64; 2]>, ext: f64, sc: Scale) -> Result<Seg, String> {
    let t = tangent(w.n);
    let back = |e: [f64; 2], by: f64| [e[0] - t[0] * by, e[1] - t[1] * by];
    let (b, e) = match (vin, vout) {
        (Some(b), Some(e)) => (b, e),
        (None, Some(e)) => (back(e, ext), e),
        (Some(b), None) => (b, back(b, -ext)),
        (None, None) => {
            let (lo, hi) = wall_ends(w, sc);
            let mid = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])];
            (back(mid, ext), back(mid, -ext))
        }
    };
    if !(along(e, b, t) >= sc.mm(MIN_LENGTH)) {
        return Err("a wall has no length between its neighbours".into());
    }
    Ok(Seg::Line(b, e))
}

/// Closes an open chain far outside: a lone wall with a box on its void side, longer chains with
/// the line between their free ends.
fn close_chain(segs: &mut Vec<Seg>, lone: Option<&Wall>, ext: f64) -> Result<(), String> {
    let (Some(Seg::Line(first, _)), Some(Seg::Line(_, last))) = (segs.first().cloned(), segs.last().cloned()) else {
        return Err("an open profile must start and end with a wall".into());
    };
    if let Some(w) = lone {
        let up = |p: [f64; 2]| [p[0] + w.n[0] * ext, p[1] + w.n[1] * ext];
        segs.extend([Seg::Line(last, up(last)), Seg::Line(up(last), up(first)), Seg::Line(up(first), first)]);
    } else {
        segs.push(Seg::Line(last, first));
    }
    Ok(())
}

/// The middle of the first element of a path (on the loop, inside the part).
fn first_point(elems: &[Elem], p: &Path) -> Option<[f64; 2]> {
    let e = elems.get(*p.nodes.first()?)?;
    Some([0.5 * (e.begin[0] + e.end[0]), 0.5 * (e.begin[1] + e.end[1])])
}

fn build_path(ctx: &Ctx, elems: &[Elem], succ: &[Option<Link>], p: &Path, ext: f64) -> Result<Loop, String> {
    let j = joints(ctx, elems, succ, p);
    let mut segs: Vec<Seg> = Vec::new();
    let mut nodes = Vec::new();
    let mut arcs = Vec::new();
    for (k, &n) in p.nodes.iter().enumerate() {
        let (vin, vout) = (j.vin.get(k).copied().flatten(), j.vout.get(k).copied().flatten());
        match elems.get(n).map(|e| e.kind) {
            Some(Kind::Wall(w)) => {
                let wall = ctx.wall(w).ok_or("index")?;
                segs.push(wall_seg(wall, vin, vout, ext, ctx.scale)?);
                nodes.push(wall.id);
            }
            Some(Kind::Arc(c)) => {
                let k_ = ctx.corner(c).ok_or("index")?;
                let (Some(b), Some(e)) = (vin, vout) else {
                    return Err(format!(
                        "the round r = {:.2} at ({:.1}, {:.1}) does not join its neighbours (an open profile that ends in a round is not built)",
                        k_.r, k_.center[0], k_.center[1]
                    ));
                };
                let solved = j.round.get(k).and_then(Option::as_ref);
                let (seg, center, r) = match solved {
                    Some(rd) => (arc_seg(rd.center, rd.r, b, e, k_.concave), rd.center, rd.r),
                    None => arc_through(k_, b, e, ctx.scale)?,
                };
                segs.push(seg);
                nodes.push(k_.id);
                arcs.push((k_.id, center, r));
            }
            None => return Err("index".into()),
        }
    }
    if !p.closed {
        let lone = match (p.nodes.as_slice(), elems.get(p.nodes.first().copied().unwrap_or(0)).map(|e| e.kind)) {
            ([_], Some(Kind::Wall(w))) => ctx.wall(w),
            _ => None,
        };
        close_chain(&mut segs, lone, ext)?;
    }
    if !segs.iter().all(seg_is_sound) {
        return Err("the profile has a coordinate that is not a finite number in range".into());
    }
    let polygon = polygon_of(&segs);
    let area = signed_area(&polygon);
    if crosses_itself(&polygon) {
        return Err("the walls cross each other".into());
    }
    if !(area.abs() >= ctx.scale.mm2(MIN_AREA)) {
        return Err("the walls enclose no area".into());
    }
    let probe = first_point(elems, p).unwrap_or([0.0; 2]);
    Ok(Loop { segs, polygon, closed: p.closed, area, nodes, arcs, probe })
}

/// The loops of `walls` and `corners`, and the open chains (closed far outside, `ext` beyond the
/// part). Posts (full rounds) are loops of their own. `scale` carries the scan's spacing.
pub(super) fn build_loops(walls: &[Wall], corners: &[Corner], ext: f64, scale: Scale) -> Result<Vec<Loop>, String> {
    check_input(walls, corners, ext)?;
    let ctx = Ctx { walls, corners, scale };
    let mut loops: Vec<Loop> = corners.iter().filter(|k| k.full).map(circle_loop).collect();
    let elems = elements(&ctx);
    let succ = match_links(&ctx, &elems);
    for p in paths(&succ) {
        loops.push(build_path(&ctx, &elems, &succ, &p, ext)?);
    }
    if loops.is_empty() {
        return Err("a profile needs at least one wall".into());
    }
    Ok(loops)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    const EXT: f64 = 300.0;
    const SC: Scale = Scale { spacing: 0.9 };

    /// A wall along `a`..`b` (points trimmed by `trim` at both ends) with the void on the side of
    /// `void`.
    fn wall(id: usize, a: [f64; 2], b: [f64; 2], void: [f64; 2], trim: f64) -> Wall {
        let l = d2(a, b);
        let t = [(b[0] - a[0]) / l, (b[1] - a[1]) / l];
        let mut n = [-t[1], t[0]];
        if n[0] * (void[0] - a[0]) + n[1] * (void[1] - a[1]) < 0.0 {
            n = [-n[0], -n[1]];
        }
        let pts = (0..=20)
            .map(|k| {
                let s = trim + (l - 2.0 * trim) * k as f64 / 20.0;
                [a[0] + t[0] * s, a[1] + t[1] * s]
            })
            .collect();
        Wall { id, n, c: n[0] * a[0] + n[1] * a[1], pts }
    }

    /// A round about `c` covering the angles `from`..`to` (degrees, counter-clockwise).
    fn round(id: usize, c: [f64; 2], r: f64, from: f64, to: f64, concave: bool) -> Corner {
        let pts = (0..=12)
            .map(|k| {
                let a = (from + (to - from) * k as f64 / 12.0).to_radians();
                [c[0] + r * a.cos(), c[1] + r * a.sin()]
            })
            .collect();
        Corner { id, center: c, r, concave, full: false, pts }
    }

    fn assert_continuous(l: &Loop) {
        let end = |s: &Seg| match s {
            Seg::Line(_, b) => *b,
            Seg::Arc { c, r, a1, .. } => [c[0] + r * a1.cos(), c[1] + r * a1.sin()],
        };
        let start = |s: &Seg| match s {
            Seg::Line(a, _) => *a,
            Seg::Arc { c, r, a0, .. } => [c[0] + r * a0.cos(), c[1] + r * a0.sin()],
        };
        for (i, s) in l.segs.iter().enumerate() {
            let next = l.segs.get((i + 1) % l.segs.len()).unwrap();
            assert!(
                d2(end(s), start(next)) < 1e-9,
                "piece {i} does not meet the next: {:?} -> {:?}",
                end(s),
                start(next)
            );
        }
    }

    fn one(walls: &[Wall], corners: &[Corner]) -> Loop {
        let mut loops = build_loops(walls, corners, EXT, SC).unwrap();
        assert_eq!(loops.len(), 1, "{loops:?}");
        let l = loops.remove(0);
        assert_continuous(&l);
        l
    }

    #[test]
    fn a_rounded_rectangle_closes_into_eight_pieces() {
        let (x0, y0, x1, y1, r) = (8.0, 8.0, 38.0, 28.0, 4.0);
        let inside = [20.0, 18.0];
        let walls = [
            wall(0, [x0 + r, y0], [x1 - r, y0], inside, 0.3),
            wall(1, [x1, y0 + r], [x1, y1 - r], inside, 0.3),
            wall(2, [x1 - r, y1], [x0 + r, y1], inside, 0.3),
            wall(3, [x0, y1 - r], [x0, y0 + r], inside, 0.3),
        ];
        let corners = [
            round(4, [x1 - r, y0 + r], r, -85.0, -5.0, true),
            round(5, [x1 - r, y1 - r], r, 5.0, 85.0, true),
            round(6, [x0 + r, y1 - r], r, 95.0, 175.0, true),
            round(7, [x0 + r, y0 + r], r, 185.0, 265.0, true),
        ];
        let l = one(&walls, &corners);
        assert!(l.closed && l.segs.len() == 8);
        let area = 30.0 * 20.0 - (4.0 - PI) * 16.0;
        assert!((l.area - area).abs() < 0.5, "area {} vs {area}", l.area);
        assert_eq!(l.arcs.len(), 4);
    }

    fn l_walls(inside_of_l: bool) -> Vec<Wall> {
        // the L: (8,8) (38,8) (38,20) (20,20) (20,34) (8,34); void on the inside of the L
        let v = [14.0, 14.0];
        let _ = inside_of_l;
        vec![
            wall(0, [8.0, 8.0], [38.0, 8.0], v, 0.5),
            wall(1, [38.0, 8.0], [38.0, 20.0], v, 0.5),
            wall(2, [38.0, 20.0], [20.0, 20.0], v, 0.5),
            wall(3, [20.0, 20.0], [20.0, 34.0], v, 0.5),
            wall(4, [20.0, 34.0], [8.0, 34.0], v, 0.5),
            wall(5, [8.0, 34.0], [8.0, 8.0], v, 0.5),
        ]
    }

    #[test]
    fn an_l_shaped_profile_with_a_reflex_corner_closes_the_right_way_round() {
        let l = one(&l_walls(true), &[]);
        assert!(l.closed && l.segs.len() == 6);
        assert!((l.area - 528.0).abs() < 1e-6, "area {}", l.area);
    }

    #[test]
    fn a_rounded_reflex_corner_is_a_clockwise_arc() {
        // the reflex corner (20, 20) rounded r = 2: centre (22, 22) in the material
        let mut walls = l_walls(true);
        walls.remove(3);
        walls.remove(2);
        let v = [14.0, 14.0];
        walls.push(wall(2, [38.0, 20.0], [22.0, 20.0], v, 0.3));
        walls.push(wall(3, [20.0, 22.0], [20.0, 34.0], v, 0.3));
        let corner = round(9, [22.0, 22.0], 2.0, 185.0, 265.0, false);
        let l = one(&walls, &[corner]);
        let arc = l.segs.iter().find(|s| matches!(s, Seg::Arc { .. })).unwrap();
        let Seg::Arc { a0, a1, .. } = arc else { panic!("not an arc") };
        assert!(a1 < a0, "a reflex round runs clockwise");
        assert!((a0 - a1 - FRAC_PI_2).abs() < 1e-6);
        assert!((l.area - (528.0 + 4.0 - PI)).abs() < 0.05, "area {}", l.area);
    }

    #[test]
    fn two_arcs_next_to_each_other_form_a_lens() {
        let (a, b) = (8.0_f64.atan2(6.0).to_degrees(), 8.0_f64.atan2(-6.0).to_degrees());
        let corners = [
            round(0, [-6.0, 0.0], 10.0, -a + 3.0, a - 3.0, true),
            round(1, [6.0, 0.0], 10.0, b + 3.0, 360.0 - b - 3.0, true),
        ];
        let l = one(&[], &corners);
        assert!(l.closed && l.segs.len() == 2);
        let area = 2.0 * (100.0 * 0.6_f64.acos() - 48.0);
        assert!((l.area - area).abs() < 0.5, "area {} vs {area}", l.area);
    }

    #[test]
    fn an_arc_tangent_to_two_rounds_is_chained_with_them() {
        // wall y = 8 .. round r 3 .. big arc R 18 about (22, 20) .. round r 3 .. wall y = 32 .. left side
        let inside = [20.0, 20.0];
        let (x, d) = (36.0_f64.to_radians().sin(), 36.87_f64);
        let _ = x;
        let walls = [
            wall(0, [11.0, 8.0], [34.0, 8.0], inside, 0.3),
            wall(1, [34.0, 32.0], [11.0, 32.0], inside, 0.3),
            wall(2, [8.0, 29.0], [8.0, 11.0], inside, 0.3),
        ];
        let corners = [
            round(3, [34.0, 11.0], 3.0, -88.0, -d + 2.0, true),
            round(4, [22.0, 20.0], 18.0, -d + 4.0, d - 4.0, true),
            round(5, [34.0, 29.0], 3.0, d - 2.0, 88.0, true),
            round(6, [11.0, 29.0], 3.0, 92.0, 178.0, true),
            round(7, [11.0, 11.0], 3.0, 182.0, 268.0, true),
        ];
        let l = one(&walls, &corners);
        assert!(l.closed && l.segs.len() == 8, "{} pieces", l.segs.len());
        assert!(l.area > 0.0);
    }

    #[test]
    fn walls_round_an_island_run_clockwise() {
        // a rectangle 10 x 8 with the void outside it
        let walls = [
            wall(0, [16.0, 14.0], [26.0, 14.0], [21.0, 0.0], 0.3),
            wall(1, [26.0, 14.0], [26.0, 22.0], [40.0, 18.0], 0.3),
            wall(2, [26.0, 22.0], [16.0, 22.0], [21.0, 40.0], 0.3),
            wall(3, [16.0, 22.0], [16.0, 14.0], [0.0, 18.0], 0.3),
        ];
        let l = one(&walls, &[]);
        assert!(l.closed && (l.area + 80.0).abs() < 1e-6, "area {}", l.area);
    }

    #[test]
    fn a_post_is_a_clockwise_circle() {
        let mut k = round(0, [30.0, 20.0], 4.0, 0.0, 360.0, false);
        k.full = true;
        let l = one(&[], &[k]);
        assert!(l.closed && (l.area + PI * 16.0).abs() < 0.2, "area {}", l.area);
    }

    #[test]
    fn an_open_slot_is_extended_and_closed_outside() {
        // walls y = 26 and y = 34 running to x = 40, a half round between them, open to +x
        let inside = [45.0, 30.0];
        let walls =
            [wall(0, [40.0, 26.0], [50.0, 26.0], inside, 0.3), wall(1, [50.0, 34.0], [40.0, 34.0], inside, 0.3)];
        let end = round(2, [40.0, 30.0], 4.0, 92.0, 268.0, true);
        let l = one(&walls, &[end]);
        assert!(!l.closed && l.area > 0.0);
        let far = l.polygon.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!(far > 100.0, "the free walls run out of the part ({far})");
    }

    #[test]
    fn a_lone_wall_is_closed_with_a_box_on_its_void_side() {
        let wall = wall(0, [40.0, 0.0], [40.0, 50.0], [20.0, 25.0], 0.3);
        let l = one(&[wall], &[]);
        assert!(!l.closed && l.area > 1000.0);
        assert!(l.polygon.iter().all(|p| p[0] <= 40.0 + 1e-9), "the box lies on the void side");
    }

    #[test]
    fn walls_that_do_not_meet_stay_apart() {
        let walls =
            [wall(0, [0.0, 0.0], [10.0, 0.0], [5.0, 5.0], 0.3), wall(1, [100.0, 0.0], [100.0, 10.0], [50.0, 5.0], 0.3)];
        let loops = build_loops(&walls, &[], EXT, SC).unwrap();
        assert_eq!(loops.len(), 2);
        assert!(loops.iter().all(|l| !l.closed));
    }

    #[test]
    fn hostile_regions_are_refused_not_built() {
        let good = || wall(0, [0.0, 0.0], [10.0, 0.0], [5.0, 5.0], 0.3);
        let mut nan_point = good();
        nan_point.pts.push([f64::NAN, 1.0]);
        let mut nan_c = good();
        nan_c.c = f64::NAN;
        let mut no_direction = good();
        no_direction.n = [0.0, 0.0];
        let mut far = good();
        far.pts.push([1e300, 0.0]);
        for w in [nan_point, nan_c, no_direction, far] {
            assert!(build_loops(&[w], &[], EXT, SC).is_err());
        }
        let round_of = |centre: [f64; 2], r: f64| round(1, centre, r, 0.0, 90.0, true);
        for k in [
            round_of([f64::NAN, 0.0], 3.0),
            round_of([0.0, 0.0], 0.0),
            round_of([0.0, 0.0], -3.0),
            round_of([0.0, 0.0], 1e300),
        ] {
            assert!(build_loops(&[good()], &[k], EXT, SC).is_err());
        }
        assert!(build_loops(&[good()], &[], f64::NAN, SC).is_err());
        assert!(build_loops(&[good()], &[], -1.0, SC).is_err());
    }

    #[test]
    fn a_short_segment_between_two_rounds_links_at_any_scale() {
        // a pocket 5 x 3 with r = 1 corners: its straight pieces are 3 and 1 long, well under the
        // fixed 6 mm the joints used to need; the same shape at twice and a fifth of the size
        for k in [0.2, 1.0, 2.0, 5.0] {
            let sc = Scale { spacing: 0.9 * k };
            let (w, h, r, trim) = (5.0 * k, 3.0 * k, 1.0 * k, 0.1 * k);
            let inside = [w / 2.0, h / 2.0];
            let walls = [
                wall(0, [r, 0.0], [w - r, 0.0], inside, trim),
                wall(1, [w, r], [w, h - r], inside, trim),
                wall(2, [w - r, h], [r, h], inside, trim),
                wall(3, [0.0, h - r], [0.0, r], inside, trim),
            ];
            let corners = [
                round(4, [w - r, r], r, -85.0, -5.0, true),
                round(5, [w - r, h - r], r, 5.0, 85.0, true),
                round(6, [r, h - r], r, 95.0, 175.0, true),
                round(7, [r, r], r, 185.0, 265.0, true),
            ];
            let mut loops = build_loops(&walls, &corners, 100.0 * k, sc).unwrap_or_else(|e| panic!("scale {k}: {e}"));
            assert_eq!(loops.len(), 1);
            let l = loops.remove(0);
            assert!(l.closed && l.segs.len() == 8, "scale {k}: {} pieces", l.segs.len());
        }
    }

    #[test]
    fn a_long_wall_links_to_its_round() {
        // a wall 2000 mm long: its 400 sampled points are 5 mm apart, so only a few lie near an end
        let r = 20.0;
        let mut long = wall(0, [r, 0.0], [2000.0, 0.0], [1000.0, 500.0], 1.0);
        long.pts = (0..400).map(|k| [r + 1.0 + (2000.0 - r - 2.0) * k as f64 / 399.0, 0.0]).collect();
        let side = wall(1, [2000.0, r], [2000.0, 500.0 - r], [1000.0, 250.0], 1.0);
        let round_ = round(2, [2000.0 - r, r], r, -85.0, -5.0, true);
        let sc = Scale { spacing: 0.9 * 20.0 };
        let loops = build_loops(&[long, side], &[round_], 10000.0, sc).unwrap();
        assert!(loops.iter().any(|l| l.arcs.len() == 1), "{loops:?}");
    }

    #[test]
    fn empty_regions_and_lone_rounds_are_refused() {
        assert!(build_loops(&[], &[], EXT, SC).is_err());
        let k = round(0, [0.0, 0.0], 3.0, 0.0, 90.0, true);
        assert!(build_loops(&[], &[k], EXT, SC).is_err());
    }
}
