//! The planner of the hybrid model (plan P3b, Z1 and Z2): which fitted regions are *global* cuts of
//! the stock (the outer envelope of the part) and which are *local* (a pocket floor and walls, a
//! blind hole's wall and bottom, a slot's walls), and the local tools the latter form.
//!
//! The test for a plane is the one a person would make: is there scan in front of it? A global
//! plane has none (the part lies wholly behind it); a pocket floor has the rim of the pocket in
//! front of it. The same logic drives `body.from_regions` (which builds the solid from the plan)
//! and `body.plan_regions` (which only reports it), so the diagnostic never disagrees with the
//! build. Everything here is mesh-side geometry on the points of the regions; the solids of the
//! tools are the kernel's (`surfacecraft_kernel::local`).
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::f64::consts::{PI, TAU};
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::{KdTree, Point3, Vec3, any_perpendicular, normalize};
use surfacecraft_kernel::feature::ProfileCurve;
use surfacecraft_kernel::hybrid::{Cut, Shape};
use surfacecraft_kernel::local::{HoleBottom, LocalAdd, LocalTool};

/// Fraction of the scan allowed in front of a plane (or outside a cylinder) that is still the
/// part's envelope: outliers, not geometry.
const ENVELOPE_FRACTION: f64 = 0.005;
/// Largest gap between two regions that are neighbours (mm): the width of an unmodelled fillet.
const NEIGHBOUR_GAP: f64 = 3.0;

/// A fitted region the planner looks at.
pub struct Item {
    /// The primitive object.
    pub id: ObjectId,
    /// Its kind of surface, for reports.
    pub kind: &'static str,
    /// Cut (shape, material side, full revolution) from the scan normals.
    pub cut: Cut,
    /// The points of its region.
    pub pts: Vec<Point3>,
}

/// The role of a region in the solid.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoleKind {
    /// A cut of the stock's envelope (half space, full revolution).
    Global,
    /// A fillet of two global planes.
    Blend,
    /// Wall, floor or bottom of a tool that is subtracted (hole, pocket, slot).
    LocalSubtract,
    /// A boss or dome that adds material (a step is not built yet).
    LocalAdd,
    /// Not used, with the reason.
    Skipped,
}

impl RoleKind {
    /// Name in reports.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            RoleKind::Global => "global",
            RoleKind::Blend => "blend",
            RoleKind::LocalSubtract => "local-subtract",
            RoleKind::LocalAdd => "local-add",
            RoleKind::Skipped => "skipped",
        }
    }
}

/// What the planner decided for one item.
#[derive(Clone, Debug)]
pub struct Role {
    /// The role.
    pub kind: RoleKind,
    /// Why.
    pub how: String,
    /// Tools this region belongs to (indices into [`Plan::tools`]).
    pub tools: Vec<usize>,
    /// Local additions this region belongs to (indices into [`Plan::adds`]).
    pub adds: Vec<usize>,
    /// The surface as planned (snapped axes, refitted cone).
    pub shape: Shape,
}

/// A contact of two neighbouring regions.
#[derive(Clone, Debug)]
pub struct Contact {
    /// First item.
    pub a: usize,
    /// Second item.
    pub b: usize,
    /// `convex`, `concave` or `tangent`.
    pub kind: &'static str,
    /// Angle between the surface normals at the contact (degrees; 0 is tangent).
    pub angle_deg: f64,
    /// Smallest distance between the regions' points (mm).
    pub distance: f64,
}

/// A local tool and the regions that form it.
#[derive(Clone, Debug)]
pub struct PlannedTool {
    /// The tool for the kernel.
    pub tool: LocalTool,
    /// `blind_hole`, `pocket` or `through_slot`.
    pub kind: &'static str,
    /// Items that form it.
    pub members: Vec<usize>,
    /// Numbers for the report.
    pub detail: String,
}

/// A local addition (boss, dome) and the regions that form it.
#[derive(Clone, Debug)]
pub struct PlannedAdd {
    /// The solid for the kernel.
    pub add: LocalAdd,
    /// `boss` or `dome`.
    pub kind: &'static str,
    /// Items that form it.
    pub members: Vec<usize>,
    /// Numbers for the report.
    pub detail: String,
}

/// A global plane the planner tried to square to a model axis.
#[derive(Clone, Debug)]
pub struct SquaredPlane {
    /// The item (base of its node).
    pub item: usize,
    /// Angle between the fitted and the squared normal, degrees.
    pub angle_deg: f64,
    /// Largest distance the squaring moves the plane over its region, mm.
    pub max_shift: f64,
    /// The largest shift allowed, mm.
    pub limit: f64,
    /// Squared (false: left as fitted because the shift exceeds the limit).
    pub applied: bool,
}

/// The plan.
#[derive(Clone, Debug)]
pub struct Plan {
    /// Planes the planner tried to square.
    pub squared: Vec<SquaredPlane>,
    /// Cuts for the kernel's global steps.
    pub cuts: Vec<Cut>,
    /// Items per cut (merged coplanar planes share one).
    pub cut_members: Vec<Vec<usize>>,
    /// Local tools.
    pub tools: Vec<PlannedTool>,
    /// Local additions (bosses, domes).
    pub adds: Vec<PlannedAdd>,
    /// Role per item.
    pub roles: Vec<Role>,
    /// Neighbour graph with the kind of each contact.
    pub contacts: Vec<Contact>,
}

fn v3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}
fn arr(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}
fn pt(a: [f64; 3]) -> Point3 {
    Point3::new(a[0], a[1], a[2])
}

/// Distance of a point from the line `origin + t·axis` and its position along it.
fn radial_axial(p: &Point3, origin: Vec3, axis: Vec3) -> (f64, f64) {
    let d = p.coords - origin;
    let t = d.dot(&axis);
    ((d - axis * t).norm(), t)
}

/// One surface (a merged group of coplanar fragments is one node).
struct Node {
    members: Vec<usize>,
    base: usize,
    shape: Shape,
    convex: bool,
    full: bool,
    pts: Vec<Point3>,
    tree: KdTree,
}

fn make_nodes(items: &[Item]) -> Vec<Node> {
    // coplanar planes with the same orientation are one surface (segmentation splits big planes)
    let planes: Vec<(usize, Vec3, Vec3)> = items
        .iter()
        .enumerate()
        .filter_map(|(i, it)| match &it.cut.shape {
            Shape::Plane { point, normal } => Some((i, v3(*point), v3(*normal))),
            _ => None,
        })
        .collect();
    let mut parent: Vec<usize> = (0..items.len()).collect();
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent.get(i).is_some_and(|&p| p != i) {
            let p = parent.get(i).copied().unwrap_or(i);
            if let Some(g) = parent.get(p).copied()
                && let Some(x) = parent.get_mut(i)
            {
                *x = g;
            }
            i = p;
        }
        i
    }
    for x in 0..planes.len() {
        for y in (x + 1)..planes.len() {
            let ((i, p, n), (j, q, m)) = (planes[x], planes[y]);
            if n.dot(&m) > 0.998 && (n.dot(&(q - p))).abs() < 0.25 {
                let (ri, rj) = (find(&mut parent, i), find(&mut parent, j));
                if ri != rj
                    && let Some(x) = parent.get_mut(rj)
                {
                    *x = ri;
                }
            }
        }
    }
    let mut groups: Vec<Vec<usize>> = Vec::new();
    let mut slot: Vec<Option<usize>> = vec![None; items.len()];
    for i in 0..items.len() {
        let r = find(&mut parent, i);
        match slot.get(r).copied().flatten() {
            Some(g) => {
                if let Some(v) = groups.get_mut(g) {
                    v.push(i);
                }
            }
            None => {
                if let Some(s) = slot.get_mut(r) {
                    *s = Some(groups.len());
                }
                groups.push(vec![i]);
            }
        }
    }
    groups
        .into_iter()
        .filter_map(|members| {
            // the largest fragment gives the plane (the best conditioned fit)
            let base = *members.iter().max_by_key(|&&i| items.get(i).map_or(0, |it| it.pts.len()))?;
            let it = items.get(base)?;
            let pts: Vec<Point3> =
                members.iter().filter_map(|&i| items.get(i)).flat_map(|it| it.pts.iter().copied()).collect();
            let tree = KdTree::build(&pts);
            Some(Node {
                members,
                base,
                shape: it.cut.shape.clone(),
                convex: it.cut.convex,
                full: it.cut.full,
                pts,
                tree,
            })
        })
        .collect()
}

/// The unit direction of the surface at `p`, pointing out of the material.
fn normal_at(nd: &Node, p: &Point3) -> Option<Vec3> {
    let sign = if nd.convex { 1.0 } else { -1.0 };
    match &nd.shape {
        Shape::Plane { normal, .. } => normalize(v3(*normal)),
        Shape::Cylinder { point, axis, .. } => {
            let a = normalize(v3(*axis))?;
            let d = p.coords - v3(*point);
            normalize(d - a * d.dot(&a)).map(|r| r * sign)
        }
        Shape::Cone { apex, axis, half_angle } => {
            let a = normalize(v3(*axis))?;
            let d = p.coords - v3(*apex);
            let r = normalize(d - a * d.dot(&a))?;
            normalize(r * half_angle.cos() - a * half_angle.sin()).map(|n| n * sign)
        }
        Shape::Sphere { center, .. } => normalize(p.coords - v3(*center)).map(|r| r * sign),
        Shape::Torus { center, axis, major_radius, .. } => {
            let a = normalize(v3(*axis))?;
            let d = p.coords - v3(*center);
            let r = normalize(d - a * d.dot(&a))?;
            normalize(d - r * *major_radius).map(|n| n * sign)
        }
    }
}

/// The contact graph: for every pair of neighbouring surfaces, whether the surfaces meet in a
/// convex or a concave edge (the material is behind the surfaces at a convex edge).
fn contacts(nodes: &[Node]) -> Vec<Contact> {
    let mut out = Vec::new();
    let bounds: Vec<([f64; 3], [f64; 3])> = nodes
        .iter()
        .map(|n| {
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            for p in &n.pts {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
            (lo, hi)
        })
        .collect();
    for x in 0..nodes.len() {
        for y in (x + 1)..nodes.len() {
            let (Some(a), Some(b), Some(ba), Some(bb)) = (nodes.get(x), nodes.get(y), bounds.get(x), bounds.get(y))
            else {
                continue;
            };
            if (0..3).any(|k| ba.0[k] - NEIGHBOUR_GAP > bb.1[k] || bb.0[k] - NEIGHBOUR_GAP > ba.1[k]) {
                continue;
            }
            // closest pair, querying with the smaller region (subsampled)
            let (small, big) = if a.pts.len() <= b.pts.len() { (a, b) } else { (b, a) };
            let stride = (small.pts.len() / 3000).max(1);
            let mut best: Option<(f64, Point3, Point3)> = None;
            for q in small.pts.iter().step_by(stride) {
                if let Some((i, d2)) = big.tree.nearest_within(q, NEIGHBOUR_GAP * NEIGHBOUR_GAP)
                    && best.as_ref().is_none_or(|b| d2 < b.0 * b.0)
                    && let Some(p) = big.tree.point(i)
                {
                    best = Some((d2.sqrt(), *q, *p));
                }
            }
            let Some((dist, p, q)) = best else { continue };
            let at = Point3::from((p.coords + q.coords) * 0.5);
            let local_mean = |n: &Node| -> Option<Vec3> {
                let ids = n.tree.within(&at, 2.5 + dist);
                if ids.is_empty() {
                    return None;
                }
                let s: Vec3 = ids.iter().filter_map(|&i| n.tree.point(i)).map(|p| p.coords).sum();
                Some(s / ids.len() as f64)
            };
            let (Some(ma), Some(mb)) = (local_mean(a), local_mean(b)) else { continue };
            let (Some(na), Some(nb)) = (normal_at(a, &at), normal_at(b, &at)) else { continue };
            // each surface's neighbour lies in front of it at a concave edge and behind at a convex one
            let score = na.dot(&(mb - at.coords)) + nb.dot(&(ma - at.coords));
            let angle = na.dot(&nb).clamp(-1.0, 1.0).acos().to_degrees();
            let kind = if angle < 8.0 || score.abs() < 0.1 {
                "tangent"
            } else if score > 0.0 {
                "concave"
            } else {
                "convex"
            };
            out.push(Contact { a: a.base, b: b.base, kind, angle_deg: angle, distance: dist });
        }
    }
    out
}

fn min_dist(tree: &KdTree, q: &Point3) -> f64 {
    tree.nearest(q).map_or(f64::INFINITY, |(_, d2)| d2.sqrt())
}

/// Smallest distance between two regions' points (subsampled), `None` when above `limit`.
fn near(a: &Node, b: &Node, limit: f64) -> Option<f64> {
    let (small, big) = if a.pts.len() <= b.pts.len() { (a, b) } else { (b, a) };
    let stride = (small.pts.len() / 3000).max(1);
    small
        .pts
        .iter()
        .step_by(stride)
        .filter_map(|q| big.tree.nearest_within(q, limit * limit).map(|(_, d2)| d2.sqrt()))
        .min_by(f64::total_cmp)
}

/// The first of the `anchors` (up to sign) within `deg` degrees of `d`, else `d`.
fn snap_direction(d: Vec3, anchors: &[Vec3], deg: f64) -> Vec3 {
    let cos = deg.to_radians().cos();
    for a in anchors {
        let c = a.dot(&d);
        if c.abs() > cos {
            return if c < 0.0 { -*a } else { *a };
        }
    }
    d
}

// ---------------------------------------------------------------------------------------------
// profiles

struct Wall {
    /// Unit normal in the plane, towards the void.
    n: [f64; 2],
    /// `n · x = c` on the wall.
    c: f64,
    mean: [f64; 2],
    pts: Vec<[f64; 2]>,
}

struct Corner {
    center: [f64; 2],
    r: f64,
    mean: [f64; 2],
    pts: Vec<[f64; 2]>,
}

enum Seg {
    Line([f64; 2], [f64; 2]),
    Arc { c: [f64; 2], r: f64, a0: f64, a1: f64 },
}

struct Profile {
    segs: Vec<Seg>,
    polygon: Vec<[f64; 2]>,
    closed: bool,
}

fn d2(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn dist_to(pts: &[[f64; 2]], p: [f64; 2]) -> f64 {
    pts.iter().map(|q| d2(*q, p)).fold(f64::INFINITY, f64::min)
}

/// Tangent direction of a wall in counter-clockwise travel (the void on the left).
fn tangent(n: [f64; 2]) -> [f64; 2] {
    [n[1], -n[0]]
}

fn solve(n1: [f64; 2], c1: f64, n2: [f64; 2], c2: f64) -> Option<[f64; 2]> {
    let det = n1[0] * n2[1] - n1[1] * n2[0];
    if det.abs() < 0.1 {
        return None;
    }
    Some([(c1 * n2[1] - c2 * n1[1]) / det, (n1[0] * c2 - n2[0] * c1) / det])
}

enum Junction {
    /// The two walls meet in a sharp corner at the point.
    Direct([f64; 2]),
    /// A rounded corner: arc from `t1` (on the first wall) to `t2`.
    Round { center: [f64; 2], r: f64, t1: [f64; 2], t2: [f64; 2] },
}

impl Junction {
    fn enter(&self) -> [f64; 2] {
        match self {
            Junction::Direct(p) => *p,
            Junction::Round { t1, .. } => *t1,
        }
    }
    fn exit(&self) -> [f64; 2] {
        match self {
            Junction::Direct(p) => *p,
            Junction::Round { t2, .. } => *t2,
        }
    }
}

fn junction(a: &Wall, b: &Wall, corner: Option<&Corner>) -> Result<Junction, String> {
    let gap = NEIGHBOUR_GAP;
    let cross = a.n[0] * b.n[1] - a.n[1] * b.n[0];
    match corner {
        None => {
            let p = solve(a.n, a.c, b.n, b.c).ok_or("two parallel walls with no corner between them")?;
            if dist_to(&a.pts, p) > gap || dist_to(&b.pts, p) > gap {
                return Err("the walls do not meet".into());
            }
            Ok(Junction::Direct(p))
        }
        Some(k) => {
            if cross.abs() < 0.1 {
                // a half round between two parallel walls facing each other (a slot's end)
                if a.n[0] * b.n[0] + a.n[1] * b.n[1] > -0.9 {
                    return Err("a rounded corner between parallel walls that do not face each other".into());
                }
                let width = -b.c - a.c;
                if width < 0.2 || (k.r - 0.5 * width).abs() > 0.6 {
                    return Err(format!("the end round r = {:.2} does not span the walls ({width:.2} apart)", k.r));
                }
                let r = 0.5 * width;
                let mid = 0.5 * (a.c - b.c);
                let s = mid - (a.n[0] * k.center[0] + a.n[1] * k.center[1]);
                let center = [k.center[0] + a.n[0] * s, k.center[1] + a.n[1] * s];
                let t1 = [center[0] - a.n[0] * r, center[1] - a.n[1] * r];
                let t2 = [center[0] - b.n[0] * r, center[1] - b.n[1] * r];
                return Ok(Junction::Round { center, r, t1, t2 });
            }
            let r = k.r;
            let center = solve(a.n, a.c + r, b.n, b.c + r).ok_or("the corner's walls are parallel")?;
            if d2(center, k.center) > (0.3 * r).max(1.0) {
                return Err(format!("the corner round r = {r:.2} is not tangent to its two walls"));
            }
            let t1 = [center[0] - a.n[0] * r, center[1] - a.n[1] * r];
            let t2 = [center[0] - b.n[0] * r, center[1] - b.n[1] * r];
            if dist_to(&a.pts, t1) > gap
                || dist_to(&b.pts, t2) > gap
                || dist_to(&k.pts, t1) > gap
                || dist_to(&k.pts, t2) > gap
            {
                return Err("the corner round does not touch its walls".into());
            }
            let sweep = ((t2[1] - center[1]).atan2(t2[0] - center[0]) - (t1[1] - center[1]).atan2(t1[0] - center[0]))
                .rem_euclid(TAU);
            if !(0.05..=PI + 0.2).contains(&sweep) {
                return Err("the corner round turns the wrong way (a reflex corner)".into());
            }
            Ok(Junction::Round { center, r, t1, t2 })
        }
    }
}

/// The closed profile of walls (and rounded corners) in the plane, in counter-clockwise order.
/// When the walls do not close (a slot open to the outside) the two free walls are extended by
/// `ext` and joined by a closing line far outside.
fn build_profile(walls: &[Wall], corners: &[Corner], ext: f64) -> Result<Profile, String> {
    if walls.len() < 2 {
        return Err("a profile needs at least two walls".into());
    }
    // sort by angle around the centroid of the walls
    let mut all: Vec<(bool, usize, [f64; 2])> = walls
        .iter()
        .enumerate()
        .map(|(i, w)| (false, i, w.mean))
        .chain(corners.iter().enumerate().map(|(i, c)| (true, i, c.mean)))
        .collect();
    let g = {
        let n = all.len() as f64;
        [all.iter().map(|a| a.2[0]).sum::<f64>() / n, all.iter().map(|a| a.2[1]).sum::<f64>() / n]
    };
    all.sort_by(|a, b| (a.2[1] - g[1]).atan2(a.2[0] - g[0]).total_cmp(&(b.2[1] - g[1]).atan2(b.2[0] - g[0])));
    let wall_pos: Vec<usize> = all.iter().enumerate().filter(|(_, a)| !a.0).map(|(p, _)| p).collect();
    let m = wall_pos.len();
    // the item between consecutive walls (cyclically): nothing or one corner
    let mut links: Vec<(usize, usize, Option<usize>)> = Vec::new();
    for k in 0..m {
        let (p, q) = (wall_pos[k], wall_pos[(k + 1) % m]);
        let between: Vec<usize> = if q > p { (p + 1..q).collect() } else { (p + 1..all.len()).chain(0..q).collect() };
        let corner = match between.as_slice() {
            [] => None,
            [one] => Some(all[*one].1),
            _ => return Err("two corners follow each other with no wall between them".into()),
        };
        links.push((all[p].1, all[q].1, corner));
    }
    let mut js: Vec<Result<Junction, String>> = links
        .iter()
        .map(|&(a, b, c)| match (walls.get(a), walls.get(b)) {
            (Some(a), Some(b)) => junction(a, b, c.and_then(|c| corners.get(c))),
            _ => Err("index".into()),
        })
        .collect();
    let bad: Vec<usize> = js.iter().enumerate().filter(|(_, j)| j.is_err()).map(|(i, _)| i).collect();
    if bad.len() > 1 {
        return Err(js.swap_remove(bad[0]).err().unwrap_or_default());
    }
    let gap_at = bad.first().copied();
    // walls in travel order; for an open chain start after the gap
    let start = gap_at.map_or(0, |g| (g + 1) % m);
    let order: Vec<usize> = (0..m).map(|k| (start + k) % m).collect();
    let mut segs: Vec<Seg> = Vec::new();
    let mut closing: Option<([f64; 2], [f64; 2])> = None;
    let mut first_start = [0.0; 2];
    let mut last_end = [0.0; 2];
    for (pos, &k) in order.iter().enumerate() {
        let wall = links.get(k).and_then(|l| walls.get(l.0)).ok_or("index")?;
        let prev = (k + m - 1) % m;
        let t = tangent(wall.n);
        let out_j = js.get(k).and_then(|j| j.as_ref().ok());
        let in_j = js.get(prev).and_then(|j| j.as_ref().ok());
        let open_first = pos == 0 && gap_at.is_some();
        let open_last = pos + 1 == m && gap_at.is_some();
        let end = if open_last { None } else { out_j.map(Junction::enter) };
        let beg = if open_first { None } else { in_j.map(Junction::exit) };
        let (b, e) = match (beg, end) {
            (Some(b), Some(e)) => (b, e),
            (None, Some(e)) => ([e[0] - t[0] * ext, e[1] - t[1] * ext], e),
            (Some(b), None) => (b, [b[0] + t[0] * ext, b[1] + t[1] * ext]),
            (None, None) => return Err("a wall with no neighbour".into()),
        };
        if (e[0] - b[0]) * t[0] + (e[1] - b[1]) * t[1] < 0.05 {
            return Err("a wall has no length between its corners".into());
        }
        if pos == 0 {
            first_start = b;
        }
        last_end = e;
        segs.push(Seg::Line(b, e));
        if let Some(Junction::Round { center, r, t1, t2 }) = out_j
            && !open_last
        {
            let a0 = (t1[1] - center[1]).atan2(t1[0] - center[0]);
            let a1 = (t2[1] - center[1]).atan2(t2[0] - center[0]);
            let sweep = (a1 - a0).rem_euclid(TAU);
            segs.push(Seg::Arc { c: *center, r: *r, a0, a1: a0 + sweep });
        }
    }
    if gap_at.is_some() {
        closing = Some((last_end, first_start));
    }
    if let Some((a, b)) = closing {
        segs.push(Seg::Line(a, b));
    }
    let mut polygon: Vec<[f64; 2]> = Vec::new();
    for s in &segs {
        match s {
            Seg::Line(a, _) => polygon.push(*a),
            Seg::Arc { c, r, a0, a1 } => {
                for k in 0..=8 {
                    let th = a0 + (a1 - a0) * k as f64 / 8.0;
                    polygon.push([c[0] + r * th.cos(), c[1] + r * th.sin()]);
                }
            }
        }
    }
    Ok(Profile { segs, polygon, closed: gap_at.is_none() })
}

fn inside_polygon(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

fn polygon_distance(poly: &[[f64; 2]], p: [f64; 2]) -> f64 {
    let n = poly.len();
    let mut best = f64::INFINITY;
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        let ab = [b[0] - a[0], b[1] - a[1]];
        let l2 = ab[0] * ab[0] + ab[1] * ab[1];
        let t = if l2 > 0.0 { (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min(d2(p, [a[0] + ab[0] * t, a[1] + ab[1] * t]));
    }
    best
}

/// A frame in the profile's plane: origin, two in-plane axes, normal `d` (e1 × e2 = d).
struct Frame {
    o: Vec3,
    e1: Vec3,
    e2: Vec3,
    d: Vec3,
}

impl Frame {
    fn new(o: Vec3, d: Vec3) -> Frame {
        let e1 = any_perpendicular(&d);
        Frame { o, e1, e2: d.cross(&e1), d }
    }
    fn to2(&self, p: &Point3) -> [f64; 2] {
        let v = p.coords - self.o;
        [v.dot(&self.e1), v.dot(&self.e2)]
    }
    fn to3(&self, p: [f64; 2]) -> Vec3 {
        self.o + self.e1 * p[0] + self.e2 * p[1]
    }
    fn curves(&self, segs: &[Seg]) -> Vec<ProfileCurve> {
        segs.iter()
            .map(|s| match s {
                Seg::Line(a, b) => ProfileCurve::Line { start: arr(self.to3(*a)), end: arr(self.to3(*b)) },
                Seg::Arc { c, r, a0, a1 } => ProfileCurve::Arc {
                    center: arr(self.to3(*c)),
                    axis: arr(self.d),
                    reference: arr(self.e1),
                    radius: *r,
                    start: *a0,
                    end: *a1,
                },
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------------------------
// bosses

/// Scan that is in front of a plane only because a boss stands on it.
struct Explained {
    o: Vec3,
    u: Vec3,
    reach: f64,
    s_base: f64,
}

impl Explained {
    /// Does this boss explain the point `p` in front of a plane with normal `n`?
    fn covers(&self, p: &Point3, n: &Vec3) -> bool {
        n.dot(&self.u).abs() > 0.99
            && radial_axial(p, self.o, self.u).0 <= self.reach
            && (p.coords - self.o).dot(&self.u) >= self.s_base - 0.3
    }
}

/// The axial position of the plane `pn` along the axis `(o, a)`, and its unit normal.
fn plane_t(pn: &Node, o: Vec3, a: Vec3) -> Option<(f64, Vec3)> {
    let Shape::Plane { point, normal } = &pn.shape else { return None };
    let n = normalize(v3(*normal))?;
    let c = n.dot(&a);
    (c.abs() > 0.99).then(|| ((v3(*point) - o).dot(&n) / c, n))
}

/// A boss: the convex full cylinder `ci`, the plane across its far end (its top) and the plane it
/// stands on, with a chamfer or round on the top edge and a fillet at the base when the scan has
/// them (a coaxial cone, a coaxial torus). On success the regions that form the boss are given
/// their roles in `st`; the base plane stays for the global test (the boss explains the scan in
/// front of it).
fn plan_boss(
    nodes: &[Node],
    ci: usize,
    st: &mut State,
    anchors: &[Vec3],
    add_index: usize,
    outside: f64,
) -> Result<(PlannedAdd, Explained), String> {
    let nd = nodes.get(ci).ok_or("index")?;
    let Shape::Cylinder { point, axis, radius } = &nd.shape else { return Err("not a cylinder".into()) };
    let (o, a0, r) = (v3(*point), normalize(v3(*axis)).ok_or("degenerate axis")?, *radius);
    let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &nd.pts {
        let t = (p.coords - o).dot(&a0);
        t0 = t0.min(t);
        t1 = t1.max(t);
    }
    // the two ends: the top is a disc, the base a plane with the boss's foot in it
    let mut found: Option<(Vec3, f64, f64, usize)> = None;
    for sign in [1.0, -1.0] {
        let u = a0 * sign;
        let (s0, s1) = if sign > 0.0 { (t0, t1) } else { (-t1, -t0) };
        let mut top: Option<(usize, f64)> = None;
        let mut base: Option<f64> = None;
        for (pi, pn) in nodes.iter().enumerate() {
            if pi == ci || !st.pending(pi) {
                continue;
            }
            let Some((t, n)) = plane_t(pn, o, a0) else { continue };
            let s = sign * t;
            if n.dot(&u) < 0.9 {
                continue;
            }
            let inside = pn.pts.iter().filter(|p| radial_axial(p, o, a0).0 <= r + 0.6).count() as f64
                / pn.pts.len().max(1) as f64;
            let ring = pn
                .pts
                .iter()
                .filter(|p| {
                    let d = radial_axial(p, o, a0).0;
                    d > r + 0.2 && d < r + 6.0
                })
                .count();
            if inside >= 0.9
                && (s1 - 0.4..=s1 + 4.0).contains(&s)
                && top.is_none_or(|b| (s - s1).abs() < (b.1 - s1).abs())
            {
                top = Some((pi, s));
            }
            if ring >= 8 && (s0 - 4.0..=s0 + 0.4).contains(&s) && base.is_none_or(|b| (s - s0).abs() < (b - s0).abs()) {
                base = Some(s);
            }
        }
        if let (Some((ti, s_top)), Some(s_base)) = (top, base)
            && s_top > s_base + 0.5
        {
            found = Some((u, s_base, s_top, ti));
            break;
        }
    }
    let Some((u0, s_base, s_top, ti)) = found else {
        return Err("no plane across its end for a top, or no plane it stands on".into());
    };
    let u = snap_direction(u0, anchors, 0.5);
    let h = s_top - s_base;
    // the top edge: a chamfer (a coaxial cone) or a round (a coaxial convex torus); the foot: a
    // coaxial concave torus
    let mut members: Vec<usize> = vec![nd.base, nodes.get(ti).map_or(nd.base, |n| n.base)];
    let mut parts: Vec<String> = Vec::new();
    let mut m_base = 0.0;
    for (ni, nn) in nodes.iter().enumerate() {
        if ni == ci || ni == ti || !st.pending(ni) {
            continue;
        }
        match &nn.shape {
            Shape::Cone { .. } if nn.convex => {
                let near_top = nn
                    .pts
                    .iter()
                    .filter(|p| {
                        let (rho, _) = radial_axial(p, o, u);
                        let s = (p.coords - o).dot(&u);
                        rho <= r + 0.5 && s >= s_top - 8.0 && s <= s_top + 0.5
                    })
                    .count();
                if near_top * 10 < nn.pts.len() * 9 {
                    continue;
                }
                // radius against height along the boss axis: rho = mr + k (s - ms)
                let pairs: Vec<(f64, f64)> = nn
                    .pts
                    .iter()
                    .map(|p| {
                        let (rho, _) = radial_axial(p, o, u);
                        ((p.coords - o).dot(&u), rho)
                    })
                    .collect();
                let n = pairs.len() as f64;
                if n < 10.0 {
                    continue;
                }
                let (ms, mr) = (pairs.iter().map(|p| p.0).sum::<f64>() / n, pairs.iter().map(|p| p.1).sum::<f64>() / n);
                let sxx: f64 = pairs.iter().map(|p| (p.0 - ms).powi(2)).sum();
                let sxy: f64 = pairs.iter().map(|p| (p.0 - ms) * (p.1 - mr)).sum();
                if sxx < 1e-9 {
                    continue;
                }
                let k = sxy / sxx;
                if !(-20.0..=-0.1).contains(&k) {
                    continue;
                }
                let rho_top = mr + k * (s_top - ms);
                let s_r = ms + (r - mr) / k;
                let (c_rad, c_ax) = (r - rho_top, s_top - s_r);
                if !(0.1..r).contains(&c_rad) || !(0.1..h).contains(&c_ax) {
                    continue;
                }
                let half = (-k).atan();
                let s_apex = ms - mr / k;
                st.set(
                    ni,
                    RoleKind::Blend,
                    format!("chamfer of the top edge of the boss: {c_rad:.2} mm on the top, {c_ax:.2} mm on the wall"),
                );
                if let Some(sh) = st.shape.get_mut(ni) {
                    *sh = Shape::Cone { apex: arr(o + u * s_apex), axis: arr(-u), half_angle: half };
                }
                members.push(nn.base);
                parts.push(format!("chamfer {c_rad:.2} x {c_ax:.2}"));
            }
            Shape::Torus { center, axis, major_radius, minor_radius } => {
                let m = *minor_radius;
                let Some(ta) = normalize(v3(*axis)) else { continue };
                let tc = v3(*center);
                let s_c = (tc - o).dot(&u);
                let tol = 0.6 + 0.2 * m;
                if ta.dot(&u).abs() < 0.99 || radial_axial(&Point3::from(tc), o, u).0 > 1.0 {
                    continue;
                }
                let (name, center_s, major) = if nn.convex {
                    ("round of the top edge of the boss", s_top - m, r - m)
                } else {
                    ("fillet at the foot of the boss", s_base + m, r + m)
                };
                if (major_radius - major).abs() > tol || (s_c - center_s).abs() > tol {
                    continue;
                }
                st.set(ni, RoleKind::Blend, format!("{name}: torus, minor radius {m:.2}"));
                if let Some(sh) = st.shape.get_mut(ni) {
                    *sh = Shape::Torus {
                        center: arr(o + u * center_s),
                        axis: arr(u),
                        major_radius: major,
                        minor_radius: m,
                    };
                }
                if !nn.convex {
                    m_base = m;
                }
                members.push(nn.base);
                parts.push(format!("{} r = {m:.2}", if nn.convex { "round" } else { "foot fillet" }));
            }
            _ => {}
        }
    }
    let base_pt = o + u * s_base;
    st.set(
        ci,
        RoleKind::LocalAdd,
        format!(
            "boss Ø{:.3}, {h:.2} mm tall ({:.1} % of the scan lies outside this cylinder)",
            2.0 * r,
            outside * 100.0
        ),
    );
    st.set(ti, RoleKind::LocalAdd, format!("top of the boss Ø{:.3}", 2.0 * r));
    for k in [ci, ti] {
        if let Some(v) = st.adds.get_mut(k) {
            v.push(add_index);
        }
    }
    if let Some(sh) = st.shape.get_mut(ci) {
        *sh = Shape::Cylinder { point: arr(base_pt), axis: arr(u), radius: r };
    }
    if let Some(sh) = st.shape.get_mut(ti) {
        *sh = Shape::Plane { point: arr(o + u * s_top), normal: arr(u) };
    }
    Ok((
        PlannedAdd {
            add: LocalAdd::Boss { base: arr(base_pt), axis: arr(u), radius: r, height: h, embed: 0.5 },
            kind: "boss",
            members,
            detail: format!(
                "Ø{:.3}, {h:.2} mm tall{}",
                2.0 * r,
                if parts.is_empty() { String::new() } else { format!(", {}", parts.join(", ")) }
            ),
        },
        Explained { o, u, reach: r + m_base + 0.6, s_base },
    ))
}

// ---------------------------------------------------------------------------------------------
// the planner

struct State {
    kind: Vec<Option<RoleKind>>,
    how: Vec<String>,
    /// Why an attempt to use a still-unassigned region failed (the first one is kept).
    notes: Vec<String>,
    tools: Vec<Vec<usize>>,
    adds: Vec<Vec<usize>>,
    shape: Vec<Shape>,
}

impl State {
    fn set(&mut self, n: usize, kind: RoleKind, how: impl Into<String>) {
        if let (Some(k), Some(h)) = (self.kind.get_mut(n), self.how.get_mut(n)) {
            *k = Some(kind);
            *h = how.into();
        }
    }
    fn note(&mut self, n: usize, text: impl Into<String>) {
        if let Some(t) = self.notes.get_mut(n)
            && t.is_empty()
        {
            *t = text.into();
        }
    }
    fn pending(&self, n: usize) -> bool {
        self.kind.get(n).is_some_and(Option::is_none)
    }
}

/// Plans the build: classifies every region and forms the local tools. `scan` is the whole
/// scanned cloud the regions come from (the "is there scan in front of it?" test).
#[must_use]
pub fn plan(items: &[Item], scan: &[Point3], square_deg: f64) -> Plan {
    let nodes = make_nodes(items);
    let contacts = contacts(&nodes);
    let mut st = State {
        kind: vec![None; nodes.len()],
        how: vec![String::new(); nodes.len()],
        notes: vec![String::new(); nodes.len()],
        tools: vec![Vec::new(); nodes.len()],
        adds: vec![Vec::new(); nodes.len()],
        shape: nodes.iter().map(|n| n.shape.clone()).collect(),
    };
    let mut tools: Vec<PlannedTool> = Vec::new();
    let diag = {
        let mut lo = Vec3::repeat(f64::INFINITY);
        let mut hi = Vec3::repeat(f64::NEG_INFINITY);
        for p in scan {
            lo = lo.inf(&p.coords);
            hi = hi.sup(&p.coords);
        }
        if lo.iter().all(|v| v.is_finite()) { (hi - lo).norm() } else { 100.0 }
    };
    let total = scan.len().max(1) as f64;
    let mut front_of: Vec<Option<f64>> = vec![None; nodes.len()];

    // Auto-constraint of directions (design doc §6.2): the tools' directions (a pocket's, a hole's
    // axis, a slot's) snap to the model axes when within half a degree. The kernel's booleans fail
    // on a tool whose direction is, to the last digit, the normal of one of the stock's fitted
    // planes (observed on a window in a plate); a model axis is not.
    let global_normals: Vec<Vec3> = vec![Vec3::x(), Vec3::y(), Vec3::z()];

    // --- 0. bosses: a convex full cylinder with scan outside it that stands on a plane. What is in
    // front of that plane because of the boss is not an obstacle to the plane being global.
    let mut adds: Vec<PlannedAdd> = Vec::new();
    let mut explained: Vec<Explained> = Vec::new();
    for ci in 0..nodes.len() {
        let Some(nd) = nodes.get(ci) else { continue };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { continue };
        if !(nd.full && nd.convex) || !st.pending(ci) {
            continue;
        }
        let (o, a) = (v3(*point), v3(*axis));
        let tol = 0.1 + 0.05 * radius.min(2.0);
        let out = scan.iter().filter(|p| radial_axial(p, o, a).0 > radius + tol).count() as f64 / total;
        front_of[ci] = Some(out);
        if out <= ENVELOPE_FRACTION {
            continue;
        }
        let ai = adds.len();
        match plan_boss(&nodes, ci, &mut st, &global_normals, ai, out) {
            Ok((add, ex)) => {
                adds.push(add);
                explained.push(ex);
            }
            Err(why) => st.note(ci, why),
        }
    }

    // --- 1. global or local, from the scan
    for (ni, nd) in nodes.iter().enumerate() {
        if !st.pending(ni) {
            continue;
        }
        match &nd.shape {
            Shape::Plane { point, normal } => {
                let (p0, n) = (v3(*point), v3(*normal));
                let rms = (nd.pts.iter().map(|p| (p.coords - p0).dot(&n).powi(2)).sum::<f64>()
                    / nd.pts.len().max(1) as f64)
                    .sqrt();
                let tol = (0.1 + 3.0 * rms).min(0.4);
                let (mut in_front, mut by_boss) = (0usize, 0usize);
                for p in scan {
                    if (p.coords - p0).dot(&n) > tol {
                        if explained.iter().any(|e| e.covers(p, &n)) {
                            by_boss += 1;
                        } else {
                            in_front += 1;
                        }
                    }
                }
                let front = in_front as f64 / total;
                front_of[ni] = Some(front);
                if front <= ENVELOPE_FRACTION {
                    let boss = if by_boss > 0 {
                        format!(
                            "; {:.1} % of the scan in front of it is the boss standing on it",
                            by_boss as f64 / total * 100.0
                        )
                    } else {
                        String::new()
                    };
                    st.set(
                        ni,
                        RoleKind::Global,
                        format!("global plane: {:.3} % of the scan lies in front of it{boss}", front * 100.0),
                    );
                }
            }
            Shape::Cylinder { point, axis, radius } if nd.full && nd.convex => {
                let (o, a) = (v3(*point), v3(*axis));
                let tol = 0.1 + 0.05 * radius.min(2.0);
                let out = scan.iter().filter(|p| radial_axial(p, o, a).0 > radius + tol).count() as f64 / total;
                front_of[ni] = Some(out);
                if out <= ENVELOPE_FRACTION {
                    st.set(
                        ni,
                        RoleKind::Global,
                        format!("global cylinder: {:.3} % of the scan lies outside it", out * 100.0),
                    );
                } else {
                    let note = st.notes.get(ni).filter(|n| !n.is_empty()).map_or(String::new(), |n| format!(": {n}"));
                    st.set(
                        ni,
                        RoleKind::Skipped,
                        format!(
                            "boss or step: {:.1} % of the scan lies outside this convex cylinder and it is no boss on a plane{note}",
                            out * 100.0
                        ),
                    );
                }
            }
            Shape::Cylinder { .. } if !nd.full && nd.convex => {
                st.set(ni, RoleKind::Blend, "partial convex cylinder: a fillet if it is tangent to two global planes");
            }
            Shape::Cylinder { .. } | Shape::Cone { .. } if !nd.full && !nd.convex => {} // pending: pocket corner or floor fillet
            Shape::Cylinder { .. } => {} // full concave: hole analysis below
            Shape::Cone { .. } if nd.full && !nd.convex => {} // pending: drill point or countersink
            Shape::Cone { .. } if nd.full => st.set(ni, RoleKind::Global, "global cone (full revolution, convex)"),
            Shape::Cone { .. } => {}  // pending: a chamfer of a circular edge (step 6)
            Shape::Torus { .. } => {} // pending: a floor fillet's corner, or a circular fillet (step 6)
            Shape::Sphere { .. } if nd.full => {
                st.set(ni, RoleKind::Global, "global revolved surface (full revolution)")
            }
            Shape::Sphere { .. } => {} // pending: a corner patch of equal fillets, or a dome (step 6)
        }
    }
    // --- 2. holes: a concave full cylinder is through (global) or blind (a tool with a bottom)
    for hi in 0..nodes.len() {
        let Some(nd) = nodes.get(hi) else { continue };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { continue };
        if !st.pending(hi) || !nd.full || nd.convex {
            continue;
        }
        let (o, a0, r) = (v3(*point), v3(*axis), *radius);
        let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in &nd.pts {
            let t = (p.coords - o).dot(&a0);
            t0 = t0.min(t);
            t1 = t1.max(t);
        }
        let window = 0.5 * r + 1.5;
        let (mut low, mut high) = (0usize, 0usize);
        for p in scan {
            let (rho, t) = radial_axial(p, o, a0);
            if rho < 0.6 * r {
                low += usize::from(t > t0 - window && t < t0 + 0.3);
                high += usize::from(t > t1 - 0.3 && t < t1 + window);
            }
        }
        let closed_low = low >= 3;
        let closed_high = high >= 3;
        match (closed_low, closed_high) {
            (false, false) => {
                st.set(hi, RoleKind::Global, "through hole: nothing of the scan closes the cylinder at either end");
                continue;
            }
            (true, true) => {
                st.set(hi, RoleKind::Skipped, "a concave cylinder closed at both ends (a cavity) is not built");
                continue;
            }
            _ => {}
        }
        // u points out of the material, towards the hole's opening
        let u = snap_direction(if closed_low { a0 } else { -a0 }, &global_normals, 0.5);
        // the wall's closed end: the smallest position along u
        let wall_end = nd.pts.iter().map(|p| (p.coords - o).dot(&u)).fold(f64::INFINITY, f64::min);
        // the bottom: a local plane across the axis, or a coaxial cone (a drill point)
        let mut bottom: Option<(usize, HoleBottom, f64, String)> = None;
        for bi in 0..nodes.len() {
            let Some(bn) = nodes.get(bi) else { continue };
            if !st.pending(bi) {
                continue;
            }
            match &bn.shape {
                Shape::Plane { point: bp, normal } => {
                    let n = v3(*normal);
                    if n.dot(&u) < 0.99 {
                        continue;
                    }
                    let near = bn.pts.iter().filter(|p| radial_axial(p, o, u).0 <= r + 1.0).count();
                    if near < 8 {
                        continue;
                    }
                    let s_f = (v3(*bp) - o).dot(&n) / n.dot(&u);
                    if (s_f - wall_end).abs() <= 2.0 {
                        bottom = Some((
                            bi,
                            HoleBottom::Flat,
                            s_f,
                            format!("flat bottom {:.2} mm from the wall's end", s_f - wall_end),
                        ));
                        break;
                    }
                }
                Shape::Cone { axis, .. } if bn.full && !bn.convex => {
                    if v3(*axis).dot(&u).abs() < 0.99 {
                        continue;
                    }
                    // regress the cone's radius on the position along the hole's axis: radius = slope * (s - apex)
                    let pairs: Vec<(f64, f64)> =
                        bn.pts.iter().map(|p| ((p.coords - o).dot(&u), radial_axial(p, o, u).0)).collect();
                    let n = pairs.len() as f64;
                    if n < 10.0 {
                        continue;
                    }
                    let (ms, mr) =
                        (pairs.iter().map(|p| p.0).sum::<f64>() / n, pairs.iter().map(|p| p.1).sum::<f64>() / n);
                    let sxx: f64 = pairs.iter().map(|p| (p.0 - ms).powi(2)).sum();
                    let sxy: f64 = pairs.iter().map(|p| (p.0 - ms) * (p.1 - mr)).sum();
                    if sxx < 1e-9 || sxy <= 0.0 {
                        continue;
                    }
                    let slope = sxy / sxx;
                    let half = slope.atan();
                    let apex_s = ms - mr / slope;
                    let rim_s = apex_s + r / slope;
                    if !(0.3..=1.4).contains(&half) || (rim_s - wall_end).abs() > 1.5 {
                        continue;
                    }
                    bottom = Some((
                        bi,
                        HoleBottom::Cone { half_angle: half },
                        apex_s,
                        format!(
                            "drill point: {:.1} deg included angle, apex {:.2} mm from the wall's end",
                            2.0 * half.to_degrees(),
                            apex_s - wall_end
                        ),
                    ));
                    break;
                }
                _ => {}
            }
        }
        let Some((bi, kind, s, how)) = bottom else {
            st.set(
                hi,
                RoleKind::Skipped,
                "blind hole: its bottom (a plane across the axis or a coaxial cone) was not given",
            );
            continue;
        };
        let base = o + u * s;
        let ti = tools.len();
        tools.push(PlannedTool {
            tool: LocalTool::BlindHole { base: arr(base), axis: arr(u), radius: r, bottom: kind },
            kind: "blind_hole",
            members: vec![nd.base, nodes.get(bi).map_or(nd.base, |b| b.base)],
            detail: format!("Ø{:.3}, {how}", 2.0 * r),
        });
        st.set(
            hi,
            RoleKind::LocalSubtract,
            format!("wall of a blind hole Ø{:.3} (the scan closes it at one end)", 2.0 * r),
        );
        if let Some(t) = st.tools.get_mut(hi) {
            t.push(ti);
        }
        st.set(bi, RoleKind::LocalSubtract, format!("bottom of the blind hole Ø{:.3}: {how}", 2.0 * r));
        if let Some(t) = st.tools.get_mut(bi) {
            t.push(ti);
        }
        // the planned geometry: snapped axis through the tool's base
        if let Some(sh) = st.shape.get_mut(hi) {
            *sh = Shape::Cylinder { point: arr(base), axis: arr(u), radius: r };
        }
        if let (Some(sh), HoleBottom::Cone { half_angle }) = (st.shape.get_mut(bi), kind) {
            *sh = Shape::Cone { apex: arr(base), axis: arr(u), half_angle };
        }
        if let (Some(sh), HoleBottom::Flat) = (st.shape.get_mut(bi), kind) {
            *sh = Shape::Plane { point: arr(base), normal: arr(u) };
        }
    }

    // --- 3. pockets: a local floor plane with walls (planes) and rounded corners (cylinders)
    for fi in 0..nodes.len() {
        let Some(fl) = nodes.get(fi) else { continue };
        let Shape::Plane { point, normal } = &fl.shape else { continue };
        if !st.pending(fi) {
            continue;
        }
        let d = snap_direction(v3(*normal), &global_normals, 0.3);
        let (p0, nf) = (v3(*point), d);
        // wall and corner candidates touching the floor
        let height = |nd: &Node| nd.pts.iter().map(|p| (p.coords - p0).dot(&nf)).fold(f64::INFINITY, f64::min);
        let mut cand: Vec<usize> = Vec::new();
        for ci in 0..nodes.len() {
            let Some(c) = nodes.get(ci) else { continue };
            if ci == fi || !st.pending(ci) {
                continue;
            }
            let ok = match &c.shape {
                Shape::Plane { normal, .. } => normalize(v3(*normal)).is_some_and(|n| n.dot(&nf).abs() < 0.05),
                Shape::Cylinder { axis, .. } if !c.full && !c.convex => {
                    normalize(v3(*axis)).is_some_and(|a| a.dot(&nf).abs() > 0.99 || a.dot(&nf).abs() < 0.1)
                }
                Shape::Torus { .. } if !c.full && !c.convex => true,
                _ => false,
            };
            if ok && (-0.6..=3.0).contains(&height(c)) && near(fl, c, NEIGHBOUR_GAP).is_some() {
                cand.push(ci);
            }
        }
        // floor fillet (cylinders along the walls, tori round the corners)
        let is_fillet = |c: &Node| match &c.shape {
            Shape::Cylinder { axis, .. } => normalize(v3(*axis)).is_some_and(|a| a.dot(&nf).abs() < 0.1),
            Shape::Torus { axis, .. } => normalize(v3(*axis)).is_some_and(|a| a.dot(&nf).abs() > 0.99),
            _ => false,
        };
        let wall_like: Vec<usize> =
            cand.iter().copied().filter(|&c| nodes.get(c).is_some_and(|c| !is_fillet(c))).collect();
        let fillets: Vec<usize> = cand.iter().copied().filter(|c| !wall_like.contains(c)).collect();
        // connected components of the walls by proximity
        let mut comp: Vec<usize> = (0..wall_like.len()).collect();
        for x in 0..wall_like.len() {
            for y in (x + 1)..wall_like.len() {
                if let (Some(a), Some(b)) = (nodes.get(wall_like[x]), nodes.get(wall_like[y]))
                    && near(a, b, NEIGHBOUR_GAP).is_some()
                {
                    let (cx, cy) = (comp[x], comp[y]);
                    for c in comp.iter_mut() {
                        if *c == cy {
                            *c = cx;
                        }
                    }
                }
            }
        }
        let mut seen: Vec<usize> = Vec::new();
        for &c in &comp {
            if !seen.contains(&c) {
                seen.push(c);
            }
        }
        let mut pockets_made = 0;
        for cid in seen {
            let members: Vec<usize> = (0..wall_like.len()).filter(|&k| comp[k] == cid).map(|k| wall_like[k]).collect();
            if !members.iter().any(|&m| nodes.get(m).is_some_and(|n| matches!(n.shape, Shape::Plane { .. }))) {
                continue;
            }
            let frame = Frame::new(p0, nf);
            match walls_and_corners(&nodes, &members, &frame, &global_normals) {
                Ok((walls, corners)) => match build_profile(&walls, &corners, 2.0 * diag + 10.0) {
                    Ok(pr) if pr.closed => {
                        let used: Vec<usize> = members.clone();
                        // fillets of this pocket: radius from the cylinders tangent to the floor
                        let mut fr: Vec<f64> = Vec::new();
                        let mut fmembers: Vec<usize> = Vec::new();
                        for &f in &fillets {
                            let Some(c) = nodes.get(f) else { continue };
                            if !used.iter().any(|&u| nodes.get(u).is_some_and(|w| near(w, c, NEIGHBOUR_GAP).is_some()))
                            {
                                continue;
                            }
                            match &c.shape {
                                Shape::Cylinder { point, radius, .. } => {
                                    let h = (v3(*point) - p0).dot(&nf);
                                    if (h - radius).abs() < 0.4 && *radius < 6.0 {
                                        fr.push(*radius);
                                        fmembers.push(f);
                                    }
                                }
                                Shape::Torus { minor_radius, .. } if *minor_radius < 6.0 => fmembers.push(f),
                                _ => {}
                            }
                        }
                        fr.sort_by(f64::total_cmp);
                        let fillet = fr.get(fr.len() / 2).copied();
                        let ti = tools.len();
                        let mut all: Vec<usize> = vec![fl.base];
                        all.extend(used.iter().filter_map(|&u| nodes.get(u).map(|n| n.base)));
                        all.extend(fmembers.iter().filter_map(|&u| nodes.get(u).map(|n| n.base)));
                        tools.push(PlannedTool {
                            tool: LocalTool::Prism {
                                profile: frame.curves(&pr.segs),
                                direction: arr(nf),
                                through: false,
                                floor_fillet: fillet,
                            },
                            kind: "pocket",
                            members: all,
                            detail: format!(
                                "{} walls, {} corners{}",
                                walls.len(),
                                corners.len(),
                                fillet.map_or(String::new(), |r| format!(", floor fillet r = {r:.2}"))
                            ),
                        });
                        pockets_made += 1;
                        for &u in used.iter().chain(&fmembers) {
                            st.set(
                                u,
                                RoleKind::LocalSubtract,
                                "wall or corner of a pocket (concave contact with its floor)",
                            );
                            if let Some(t) = st.tools.get_mut(u) {
                                t.push(ti);
                            }
                            if let (Some(Shape::Plane { .. }), Some(nd)) =
                                (nodes.get(u).map(|n| &n.shape), nodes.get(u))
                            {
                                // walls are exactly perpendicular to the floor in the planned shape
                                if let (Shape::Plane { point, normal }, Some(sh)) = (&nd.shape, st.shape.get_mut(u))
                                    && let Some(m) = wall_normal(v3(*normal), nf, &global_normals)
                                {
                                    *sh = Shape::Plane { point: *point, normal: arr(m) };
                                }
                            }
                        }
                        st.set(
                            fi,
                            RoleKind::LocalSubtract,
                            format!(
                                "floor of a pocket ({} walls), {:.1} % of the scan lies in front of it",
                                walls.len(),
                                front_of.get(fi).copied().flatten().unwrap_or(0.0) * 100.0
                            ),
                        );
                        if let Some(t) = st.tools.get_mut(fi) {
                            t.push(ti);
                        }
                    }
                    Ok(_) => {
                        for &u in &members {
                            st.note(u, "its pocket's walls do not close into a profile (an open chain with a floor is not built)");
                        }
                    }
                    Err(why) => {
                        for &u in &members {
                            st.note(u, format!("pocket profile: {why}"));
                        }
                    }
                },
                Err(why) => {
                    for &u in &members {
                        st.note(u, format!("pocket profile: {why}"));
                    }
                }
            }
        }
        if pockets_made == 0 {
            st.note(fi, "no pocket walls close around it (a step or an island: not built yet)");
        }
    }

    // --- 4. through slots: walls (and rounds) with no floor, all parallel to one direction
    let direction = |st: &State| -> Option<Vec3> {
        let pend: Vec<usize> = (0..nodes.len()).filter(|&i| st.pending(i)).collect();
        for &i in &pend {
            if let Some(Node { shape: Shape::Cylinder { axis, .. }, full: false, convex: false, .. }) = nodes.get(i) {
                return normalize(v3(*axis));
            }
        }
        for &i in &pend {
            for &j in &pend {
                if let (
                    Some(Node { shape: Shape::Plane { normal: n1, .. }, .. }),
                    Some(Node { shape: Shape::Plane { normal: n2, .. }, .. }),
                ) = (nodes.get(i), nodes.get(j))
                {
                    let c = v3(*n1).cross(&v3(*n2));
                    if c.norm() > 0.3 {
                        return normalize(c);
                    }
                }
            }
        }
        None
    };
    for _ in 0..8 {
        let Some(d0) = direction(&st) else { break };
        let d = snap_direction(d0, &global_normals, 0.5);
        let members: Vec<usize> = (0..nodes.len())
            .filter(|&i| {
                st.pending(i)
                    && nodes.get(i).is_some_and(|n| match &n.shape {
                        Shape::Plane { normal, .. } => normalize(v3(*normal)).is_some_and(|m| m.dot(&d).abs() < 0.05),
                        Shape::Cylinder { axis, .. } => {
                            !n.full && !n.convex && normalize(v3(*axis)).is_some_and(|a| a.dot(&d).abs() > 0.99)
                        }
                        _ => false,
                    })
            })
            .collect();
        if members.len() < 2 {
            for &i in &members {
                st.set(i, RoleKind::Skipped, "a lone local wall: no slot or pocket around it");
            }
            if members.is_empty() {
                break;
            }
            continue;
        }
        // connected components by proximity; take the component of the first member
        let mut comp_set = vec![members[0]];
        let mut grew = true;
        while grew {
            grew = false;
            for &m in &members {
                if comp_set.contains(&m) {
                    continue;
                }
                let touches = comp_set.iter().any(|&c| match (nodes.get(c), nodes.get(m)) {
                    (Some(a), Some(b)) => near(a, b, NEIGHBOUR_GAP).is_some(),
                    _ => false,
                });
                if touches {
                    comp_set.push(m);
                    grew = true;
                }
            }
        }
        let mut sum = Vec3::zeros();
        let mut cnt: f64 = 0.0;
        for &c in &comp_set {
            if let Some(n) = nodes.get(c) {
                for p in n.pts.iter().step_by((n.pts.len() / 200).max(1)) {
                    sum += p.coords;
                    cnt += 1.0;
                }
            }
        }
        let frame = Frame::new(sum / cnt.max(1.0), d);
        let mut reason: Option<String> = None;
        match walls_and_corners(&nodes, &comp_set, &frame, &global_normals)
            .and_then(|(w, c)| build_profile(&w, &c, 2.0 * diag + 10.0).map(|p| (w.len(), c.len(), p)))
        {
            Ok((nw, nc, pr)) => {
                // through: the interior of the profile holds no scan
                let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
                for q in &pr.polygon {
                    for k in 0..2 {
                        lo[k] = lo[k].min(q[k]);
                        hi[k] = hi[k].max(q[k]);
                    }
                }
                let mut inside = 0usize;
                for p in scan {
                    let q = frame.to2(p);
                    if q[0] < lo[0] || q[0] > hi[0] || q[1] < lo[1] || q[1] > hi[1] {
                        continue;
                    }
                    if inside_polygon(&pr.polygon, q) && polygon_distance(&pr.polygon, q) > 0.5 {
                        inside += 1;
                    }
                }
                if inside > 3 {
                    reason = Some(format!(
                        "slot walls with scan inside them ({inside} points): a floor that was not given, or an island"
                    ));
                } else {
                    let ti = tools.len();
                    tools.push(PlannedTool {
                        tool: LocalTool::Prism {
                            profile: frame.curves(&pr.segs),
                            direction: arr(d),
                            through: true,
                            floor_fillet: None,
                        },
                        kind: "through_slot",
                        members: comp_set.iter().filter_map(|&c| nodes.get(c).map(|n| n.base)).collect(),
                        detail: format!(
                            "{nw} walls, {nc} rounded corners, {}",
                            if pr.closed { "closed window" } else { "open to the outside" }
                        ),
                    });
                    for &c in &comp_set {
                        st.set(
                            c,
                            RoleKind::LocalSubtract,
                            format!("wall of a through {} ({nw} walls)", if pr.closed { "window" } else { "slot" }),
                        );
                        if let Some(t) = st.tools.get_mut(c) {
                            t.push(ti);
                        }
                    }
                }
            }
            Err(why) => reason = Some(format!("slot profile: {why}")),
        }
        if let Some(why) = reason {
            for &c in &comp_set {
                st.set(c, RoleKind::Skipped, why.clone());
            }
        }
    }

    // --- 6. rounds, chamfers and corner patches: concave partial cylinders between planes, tori,
    // partial cones and spheres (a corner of equal fillets, or a dome)
    let kind_of = |st: &State, i: usize| st.kind.get(i).copied().flatten();
    // (a) a concave partial cylinder tangent to two planes (its centre in front of both by r)
    for ci in 0..nodes.len() {
        let Some(nd) = nodes.get(ci) else { continue };
        let Shape::Cylinder { point, axis, radius } = &nd.shape else { continue };
        if nd.full || nd.convex || !st.pending(ci) {
            continue;
        }
        let Some(a) = normalize(v3(*axis)) else { continue };
        let (c0, r) = (v3(*point), *radius);
        let normals: Vec<Vec3> = nodes
            .iter()
            .enumerate()
            .filter(|(pi, _)| {
                *pi != ci
                    && matches!(
                        kind_of(&st, *pi),
                        Some(RoleKind::Global | RoleKind::LocalSubtract | RoleKind::LocalAdd)
                    )
            })
            .filter_map(|(_, pn)| match &pn.shape {
                Shape::Plane { point: pp, normal } => {
                    let n = normalize(v3(*normal))?;
                    (n.dot(&a).abs() < 0.05 && ((c0 - v3(*pp)).dot(&n) - r).abs() < 0.08 * r + 0.15).then_some(n)
                }
                _ => None,
            })
            .collect();
        if normals.iter().enumerate().any(|(x, n)| normals.iter().skip(x + 1).any(|m| n.dot(m).abs() < 0.95)) {
            st.set(ci, RoleKind::Blend, format!("fillet of a concave edge: r = {r:.2}, tangent to two planes"));
        }
    }
    // (b) a torus tangent to a plane across its axis and to a coaxial cylinder: the circular
    // fillet of the foot of a boss (concave) or of the rim of a disc (convex)
    for ti in 0..nodes.len() {
        let Some(nd) = nodes.get(ti) else { continue };
        let Shape::Torus { center, axis, major_radius, minor_radius } = &nd.shape else { continue };
        if !st.pending(ti) {
            continue;
        }
        let (Some(a), c, mj, m) = (normalize(v3(*axis)), v3(*center), *major_radius, *minor_radius) else { continue };
        let sgn = if nd.convex { -1.0 } else { 1.0 };
        let want = if nd.convex { mj + m } else { mj - m };
        let plane = nodes.iter().enumerate().any(|(pi, pn)| {
            kind_of(&st, pi) == Some(RoleKind::Global)
                && match &pn.shape {
                    Shape::Plane { point: pp, normal } => normalize(v3(*normal)).is_some_and(|n| {
                        n.dot(&a).abs() > 0.99 && ((c - v3(*pp)).dot(&n) - sgn * m).abs() < 0.4 + 0.15 * m
                    }),
                    _ => false,
                }
        });
        let cyl = nodes.iter().enumerate().find_map(|(pi, pn)| {
            if !matches!(kind_of(&st, pi), Some(RoleKind::Global | RoleKind::LocalAdd)) {
                return None;
            }
            match &pn.shape {
                Shape::Cylinder { point, axis, radius } if pn.full => {
                    let ca = normalize(v3(*axis))?;
                    let ok = ca.dot(&a).abs() > 0.99
                        && radial_axial(&Point3::from(c), v3(*point), ca).0 < 1.0
                        && (radius - want).abs() < 0.5 + 0.2 * m;
                    ok.then(|| {
                        // the torus on the cylinder's own axis, with its sign
                        let ca = if ca.dot(&a) < 0.0 { -ca } else { ca };
                        let on = v3(*point) + ca * (c - v3(*point)).dot(&ca);
                        (on, ca, *radius)
                    })
                }
                _ => None,
            }
        });
        match (plane, cyl) {
            (true, Some((on, ca, rc))) => {
                st.set(
                    ti,
                    RoleKind::Blend,
                    format!(
                        "{} circular fillet, minor radius {m:.2}, tangent to a plane and a cylinder of radius {rc:.2}",
                        if nd.convex { "convex (rim)" } else { "concave (foot)" }
                    ),
                );
                if let Some(sh) = st.shape.get_mut(ti) {
                    *sh = Shape::Torus {
                        center: arr(on),
                        axis: arr(ca),
                        major_radius: if nd.convex { rc - m } else { rc + m },
                        minor_radius: m,
                    };
                }
            }
            _ if nd.full => st.set(ti, RoleKind::Global, "global revolved surface (full revolution)"),
            _ => st.note(ti, "no plane tangent to its tube and coaxial cylinder was found"),
        }
    }
    // (c) a partial convex cone: the chamfer of a circular edge between a plane and a coaxial cylinder
    for ci in 0..nodes.len() {
        let Some(nd) = nodes.get(ci) else { continue };
        let Shape::Cone { apex, axis, .. } = &nd.shape else { continue };
        if nd.full || !nd.convex || !st.pending(ci) {
            continue;
        }
        let (Some(a), ap) = (normalize(v3(*axis)), v3(*apex)) else { continue };
        let plane = nodes.iter().enumerate().any(|(pi, pn)| {
            kind_of(&st, pi) == Some(RoleKind::Global)
                && matches!(&pn.shape, Shape::Plane { normal, .. } if normalize(v3(*normal)).is_some_and(|n| n.dot(&a).abs() > 0.99))
        });
        let cyl = nodes.iter().enumerate().any(|(pi, pn)| {
            matches!(kind_of(&st, pi), Some(RoleKind::Global | RoleKind::LocalAdd))
                && matches!(&pn.shape, Shape::Cylinder { point, axis, .. } if pn.full
                    && normalize(v3(*axis)).is_some_and(|ca| ca.dot(&a).abs() > 0.99)
                    && radial_axial(&Point3::from(ap), v3(*point), a).0 < 1.0)
        });
        if plane && cyl {
            st.set(ci, RoleKind::Blend, "chamfer of a circular edge between a plane and a coaxial cylinder");
        } else {
            st.note(ci, "no plane across its axis and coaxial cylinder for it to chamfer");
        }
    }
    // (d) a partial sphere: the corner patch of equal fillets, or a dome standing on a plane
    for si in 0..nodes.len() {
        let Some(nd) = nodes.get(si) else { continue };
        let Shape::Sphere { center, radius } = &nd.shape else { continue };
        if !st.pending(si) {
            continue;
        }
        let (cs, rs) = (v3(*center), *radius);
        if !nd.convex {
            st.set(si, RoleKind::Skipped, "a concave partial sphere (a dimple) is not built");
            continue;
        }
        let fillets = nodes
            .iter()
            .enumerate()
            .filter(|(pi, pn)| {
                *pi != si
                    && kind_of(&st, *pi) == Some(RoleKind::Blend)
                    && matches!(&pn.shape, Shape::Cylinder { point, axis, radius: rr }
                        if (rr - rs).abs() <= 0.1 * rs + 0.15
                            && normalize(v3(*axis)).is_some_and(|ca| radial_axial(&Point3::from(cs), v3(*point), ca).0 <= 0.25 * rs + 0.3))
            })
            .count();
        if fillets >= 2 {
            st.set(
                si,
                RoleKind::Blend,
                format!("corner patch of {fillets} fillets of radius {rs:.2} (the kernel makes it when they meet)"),
            );
            continue;
        }
        // a dome: the plane it stands on has the sphere's lowest points in it and cuts the ball
        let mut best: Option<(f64, Vec3, Vec3)> = None;
        for (pi, pn) in nodes.iter().enumerate() {
            if kind_of(&st, pi) != Some(RoleKind::Global) {
                continue;
            }
            let Shape::Plane { point: pp, normal } = &pn.shape else { continue };
            let (Some(n), p0) = (normalize(v3(*normal)), v3(*pp)) else { continue };
            let hs: Vec<f64> = nd.pts.iter().map(|p| (p.coords - p0).dot(&n)).collect();
            let low = hs.iter().copied().fold(f64::INFINITY, f64::min);
            let below = hs.iter().filter(|&&h| h < -0.5).count();
            if low > 0.8 || below * 50 > hs.len() || ((cs - p0).dot(&n)).abs() > rs - 0.2 {
                continue;
            }
            if best.as_ref().is_none_or(|b| low.abs() < b.0) {
                best = Some((low.abs(), p0, n));
            }
        }
        match best {
            Some((_, p0, n)) => {
                let ai = adds.len();
                adds.push(PlannedAdd {
                    add: LocalAdd::Dome { center: arr(cs), radius: rs, plane_point: arr(p0), normal: arr(n) },
                    kind: "dome",
                    members: vec![nd.base],
                    detail: format!("ball of radius {rs:.3} above a plane"),
                });
                st.set(
                    si,
                    RoleKind::LocalAdd,
                    format!("dome: the cap of a ball of radius {rs:.3} standing on a plane"),
                );
                if let Some(v) = st.adds.get_mut(si) {
                    v.push(ai);
                }
            }
            None => st.set(
                si,
                RoleKind::Skipped,
                "a convex partial sphere that is not the corner of equal fillets and stands on no global plane",
            ),
        }
    }

    // --- 5. what is left
    for ni in 0..nodes.len() {
        if !st.pending(ni) {
            continue;
        }
        let Some(nd) = nodes.get(ni) else { continue };
        let note = st.notes.get(ni).filter(|n| !n.is_empty()).map_or(String::new(), |n| format!("; {n}"));
        match &nd.shape {
            Shape::Cone { .. } if nd.convex => {
                st.set(ni, RoleKind::Skipped, format!("a partial convex cone that is no chamfer of a circular edge{note}"));
            }
            Shape::Cone { .. } => st.set(ni, RoleKind::Global, "concave cone (countersink): subtracted from the stock"),
            Shape::Torus { .. } | Shape::Sphere { .. } => {
                st.set(ni, RoleKind::Skipped, format!("a partial torus or sphere that is no fillet, corner patch or dome{note}"));
            }
            Shape::Plane { .. } => st.set(
                ni,
                RoleKind::Skipped,
                format!(
                    "local plane ({:.1} % of the scan lies in front of it) that is not the floor or a wall of a recognised pocket, hole or slot{note}",
                    front_of.get(ni).copied().flatten().unwrap_or(0.0) * 100.0
                ),
            ),
            _ => st.set(ni, RoleKind::Skipped, format!("a partial concave surface that is not the wall of a recognised pocket or slot{note}")),
        }
    }

    // --- the plan for the kernel and the report
    let mut cuts: Vec<Cut> = Vec::new();
    let mut cut_members: Vec<Vec<usize>> = Vec::new();
    let mut squared: Vec<SquaredPlane> = Vec::new();
    let mut roles: Vec<Option<Role>> = vec![None; items.len()];
    for (ni, nd) in nodes.iter().enumerate() {
        let kind = st.kind.get(ni).copied().flatten().unwrap_or(RoleKind::Skipped);
        let mut how = st.how.get(ni).cloned().unwrap_or_default();
        let tl = st.tools.get(ni).cloned().unwrap_or_default();
        let al = st.adds.get(ni).cloned().unwrap_or_default();
        let mut shape = st.shape.get(ni).cloned().unwrap_or_else(|| nd.shape.clone());
        // Auto-constraint (design doc §6.2), switchable by `square_deg` (0 disables): a global plane
        // within `square_deg` of a model axis is made square to it, about the fit's own point. A
        // plane off by fit noise would make the circle where a boss's cylinder meets it an
        // ellipse, and the kernel's blend would then see no circular edge. It is not squared when
        // that would move it over its region by more than max(2 sigma, 0.05 mm): a real draft.
        if square_deg > 0.0
            && kind == RoleKind::Global
            && let Shape::Plane { point, normal } = &shape
            && let Some(n) = normalize(v3(*normal))
        {
            let snapped = snap_direction(n, &global_normals, square_deg);
            let angle = n.dot(&snapped).clamp(-1.0, 1.0).acos().to_degrees();
            if angle > 1e-9 {
                let p0 = v3(*point);
                let rms = (nd.pts.iter().map(|p| (p.coords - p0).dot(&n).powi(2)).sum::<f64>()
                    / nd.pts.len().max(1) as f64)
                    .sqrt();
                let limit = (2.0 * rms).max(0.05);
                let shift = nd.pts.iter().map(|p| (p.coords - p0).dot(&(snapped - n)).abs()).fold(0.0, f64::max);
                let applied = shift <= limit;
                squared.push(SquaredPlane { item: nd.base, angle_deg: angle, max_shift: shift, limit, applied });
                if applied {
                    shape = Shape::Plane { point: *point, normal: arr(snapped) };
                    how.push_str(&format!("; squared to a model axis ({angle:.3} deg, up to {shift:.3} mm)"));
                } else {
                    how.push_str(&format!(
                        "; not squared: {angle:.3} deg would move it up to {shift:.3} mm (limit {limit:.3} mm)"
                    ));
                }
            }
        }
        if matches!(kind, RoleKind::Global | RoleKind::Blend) {
            // a blend is a partial surface for the kernel whatever the region's extent round its axis
            cuts.push(Cut { shape: shape.clone(), convex: nd.convex, full: nd.full && kind == RoleKind::Global });
            cut_members.push(nd.members.clone());
        }
        for &m in &nd.members {
            let merged = if nd.members.len() > 1 {
                format!(" (merged with {} coplanar fragments)", nd.members.len() - 1)
            } else {
                String::new()
            };
            if let Some(r) = roles.get_mut(m) {
                *r = Some(Role {
                    kind,
                    how: format!("{how}{merged}"),
                    tools: tl.clone(),
                    adds: al.clone(),
                    shape: shape.clone(),
                });
            }
        }
    }
    let roles = roles
        .into_iter()
        .zip(items)
        .map(|(r, it)| {
            r.unwrap_or(Role {
                kind: RoleKind::Skipped,
                how: "not planned".into(),
                tools: Vec::new(),
                adds: Vec::new(),
                shape: it.cut.shape.clone(),
            })
        })
        .collect();
    // tool members are node bases; every fragment of a merged plane shares the node's tools already
    let _ = (min_dist, pt as fn([f64; 3]) -> Point3);
    Plan { cuts, cut_members, tools, adds, roles, contacts, squared }
}

/// A wall's normal made exactly square to the profile's direction `d` and, within 0.3°, exactly
/// along one of the part's axes.
fn wall_normal(n: Vec3, d: Vec3, anchors: &[Vec3]) -> Option<Vec3> {
    let flat = normalize(n - d * n.dot(&d))?;
    let snapped = snap_direction(flat, anchors, 0.3);
    normalize(snapped - d * snapped.dot(&d))
}

/// The walls (planes) and rounded corners (cylinders along `frame.d`) of `members` in 2-D.
fn walls_and_corners(
    nodes: &[Node],
    members: &[usize],
    frame: &Frame,
    anchors: &[Vec3],
) -> Result<(Vec<Wall>, Vec<Corner>), String> {
    let mut walls = Vec::new();
    let mut corners = Vec::new();
    for &m in members {
        let nd = nodes.get(m).ok_or("index")?;
        let pts: Vec<[f64; 2]> = nd.pts.iter().step_by((nd.pts.len() / 400).max(1)).map(|p| frame.to2(p)).collect();
        let mean = {
            let n = pts.len().max(1) as f64;
            [pts.iter().map(|p| p[0]).sum::<f64>() / n, pts.iter().map(|p| p[1]).sum::<f64>() / n]
        };
        match &nd.shape {
            Shape::Plane { point, normal } => {
                let flat = wall_normal(v3(*normal), frame.d, anchors).ok_or("a wall parallel to the floor")?;
                let n = [flat.dot(&frame.e1), flat.dot(&frame.e2)];
                let q = frame.to2(&pt(*point));
                // a wall plane's own point may be anywhere on it: the line is n · x = n · q
                walls.push(Wall { n, c: n[0] * q[0] + n[1] * q[1], mean, pts });
            }
            Shape::Cylinder { point, radius, .. } => {
                corners.push(Corner { center: frame.to2(&pt(*point)), r: *radius, mean, pts });
            }
            _ => return Err("only planes and cylinders form a profile".into()),
        }
    }
    Ok((walls, corners))
}

// ---------------------------------------------------------------------------------------------
// reports

/// One region's role, for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PlannedRegion {
    /// The primitive object.
    pub object: ObjectId,
    /// Surface kind.
    pub surface: String,
    /// `global`, `blend`, `local-subtract`, `local-add` or `skipped`.
    pub role: String,
    /// Why.
    pub reason: String,
    /// Tools (indices into `tools`) this region belongs to.
    pub tools: Vec<usize>,
    /// Local additions (indices into `adds`) this region belongs to.
    #[serde(default)]
    pub adds: Vec<usize>,
}

/// One local tool, for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PlannedToolOut {
    /// Index.
    pub index: usize,
    /// `blind_hole`, `pocket` or `through_slot`.
    pub kind: String,
    /// The primitive objects that form it.
    pub members: Vec<ObjectId>,
    /// Numbers.
    pub detail: String,
}

/// One contact of neighbouring regions, for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ContactOut {
    /// First primitive object.
    pub a: ObjectId,
    /// Second primitive object.
    pub b: ObjectId,
    /// `convex`, `concave` or `tangent`.
    pub kind: String,
    /// Angle between the normals at the contact, degrees.
    pub angle_deg: f64,
    /// Distance between the regions, mm.
    pub distance: f64,
}

/// One plane squared to a model axis (or left alone), for `body.plan_regions`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SquaredOut {
    /// The primitive object.
    pub object: ObjectId,
    /// Angle between the fitted and the squared normal, degrees.
    pub angle_deg: f64,
    /// Largest distance the squaring moves the plane over its region, mm.
    pub max_shift_mm: f64,
    /// The largest shift allowed, mm.
    pub limit_mm: f64,
    /// Whether the plane was squared (false: the shift exceeds the limit).
    pub applied: bool,
}

/// The report of a plan.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PlanOut {
    /// Planes squared to a model axis, or left as fitted (see `applied`).
    #[serde(default)]
    pub squared_planes: Vec<SquaredOut>,
    /// Role of every input region.
    pub regions: Vec<PlannedRegion>,
    /// Local tools (blind holes, pockets, through slots) the regions form.
    pub tools: Vec<PlannedToolOut>,
    /// Local additions (bosses, domes) the regions form.
    #[serde(default)]
    pub adds: Vec<PlannedToolOut>,
    /// Neighbour graph.
    pub contacts: Vec<ContactOut>,
}

/// The report of `plan` for `items`.
#[must_use]
pub fn report(items: &[Item], plan: &Plan) -> PlanOut {
    let id = |i: usize| items.get(i).map_or(ObjectId(0), |it| it.id);
    PlanOut {
        squared_planes: plan
            .squared
            .iter()
            .map(|q| SquaredOut {
                object: id(q.item),
                angle_deg: q.angle_deg,
                max_shift_mm: q.max_shift,
                limit_mm: q.limit,
                applied: q.applied,
            })
            .collect(),
        regions: items
            .iter()
            .zip(&plan.roles)
            .map(|(it, r)| PlannedRegion {
                object: it.id,
                surface: it.kind.to_string(),
                role: r.kind.name().to_string(),
                reason: r.how.clone(),
                tools: r.tools.clone(),
                adds: r.adds.clone(),
            })
            .collect(),
        adds: plan
            .adds
            .iter()
            .enumerate()
            .map(|(index, t)| PlannedToolOut {
                index,
                kind: t.kind.to_string(),
                members: t.members.iter().map(|&m| id(m)).collect(),
                detail: t.detail.clone(),
            })
            .collect(),
        tools: plan
            .tools
            .iter()
            .enumerate()
            .map(|(index, t)| PlannedToolOut {
                index,
                kind: t.kind.to_string(),
                members: t.members.iter().map(|&m| id(m)).collect(),
                detail: t.detail.clone(),
            })
            .collect(),
        contacts: plan
            .contacts
            .iter()
            .map(|c| ContactOut {
                a: id(c.a),
                b: id(c.b),
                kind: c.kind.to_string(),
                angle_deg: c.angle_deg,
                distance: c.distance,
            })
            .collect(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wall(x0: [f64; 2], x1: [f64; 2], toward: [f64; 2]) -> Wall {
        // line through x0, x1 with the void on the side of `toward`
        let t = [x1[0] - x0[0], x1[1] - x0[1]];
        let l = t[0].hypot(t[1]);
        let mut n = [-t[1] / l, t[0] / l];
        if n[0] * (toward[0] - x0[0]) + n[1] * (toward[1] - x0[1]) < 0.0 {
            n = [-n[0], -n[1]];
        }
        let pts: Vec<[f64; 2]> =
            (0..=20).map(|k| [x0[0] + t[0] * k as f64 / 20.0, x0[1] + t[1] * k as f64 / 20.0]).collect();
        Wall { n, c: n[0] * x0[0] + n[1] * x0[1], mean: [0.5 * (x0[0] + x1[0]), 0.5 * (x0[1] + x1[1])], pts }
    }

    fn arc_corner(c: [f64; 2], r: f64, a0: f64) -> Corner {
        let pts: Vec<[f64; 2]> = (0..=10)
            .map(|k| {
                let a = a0 + 0.5 * PI * k as f64 / 10.0;
                [c[0] + r * a.cos(), c[1] + r * a.sin()]
            })
            .collect();
        let mid = a0 + 0.25 * PI;
        Corner { center: c, r, mean: [c[0] + r * mid.cos(), c[1] + r * mid.sin()], pts }
    }

    #[test]
    fn rounded_rectangle_closes_into_eight_segments() {
        let (w, h, r) = (30.0, 20.0, 4.0);
        let c = [15.0, 10.0];
        let walls = vec![
            wall([r, 0.0], [w - r, 0.0], c),
            wall([w, r], [w, h - r], c),
            wall([w - r, h], [r, h], c),
            wall([0.0, h - r], [0.0, r], c),
        ];
        let corners = vec![
            arc_corner([w - r, r], r, -0.5 * PI),
            arc_corner([w - r, h - r], r, 0.0),
            arc_corner([r, h - r], r, 0.5 * PI),
            arc_corner([r, r], r, PI),
        ];
        let p = build_profile(&walls, &corners, 100.0).unwrap();
        assert!(p.closed);
        assert_eq!(p.segs.len(), 8);
        // the polygon encloses the rounded rectangle's area
        let a: f64 = (0..p.polygon.len())
            .map(|i| {
                let (q, s) = (p.polygon[i], p.polygon[(i + 1) % p.polygon.len()]);
                q[0] * s[1] - s[0] * q[1]
            })
            .sum::<f64>()
            * 0.5;
        let exact = w * h - (4.0 - PI) * r * r;
        assert!((a - exact).abs() < 0.5, "area {a} vs {exact}");
    }

    #[test]
    fn open_slot_is_extended_and_closed_outside() {
        // a U slot open towards -x: walls y = 0 and y = 8 from x = 0.., a half round at x = 0..4 side
        let walls = vec![wall([4.0, 0.0], [30.0, 0.0], [10.0, 4.0]), wall([30.0, 8.0], [4.0, 8.0], [10.0, 4.0])];
        // half round at the closed end x = 4 (centre (4, 4), r = 4), mean on the arc
        let pts: Vec<[f64; 2]> = (0..=10)
            .map(|k| {
                let a = 0.5 * PI + PI * k as f64 / 10.0;
                [4.0 + 4.0 * a.cos(), 4.0 + 4.0 * a.sin()]
            })
            .collect();
        let corners = vec![Corner { center: [4.0, 4.0], r: 4.0, mean: [0.0, 4.0], pts }];
        // the slot opens towards +x here: walls end at x = 30 with nothing there
        let p = build_profile(&walls, &corners, 100.0).unwrap();
        assert!(!p.closed);
        assert_eq!(p.segs.len(), 4, "two walls, the round, the closing line");
    }

    #[test]
    fn walls_that_do_not_meet_are_refused() {
        let walls = vec![
            wall([0.0, 0.0], [10.0, 0.0], [5.0, 5.0]),
            wall([30.0, 30.0], [30.0, 40.0], [5.0, 5.0]),
            wall([0.0, 50.0], [10.0, 50.0], [5.0, 25.0]),
        ];
        assert!(build_profile(&walls, &[], 100.0).is_err());
    }

    /// Box 100 x 40 x 10 as six plane items; the top plane is tilted by `tilt_deg` about y.
    fn slab(tilt_deg: f64) -> (Vec<Item>, Vec<Point3>) {
        let t = tilt_deg.to_radians();
        let grid = |f: &dyn Fn(f64, f64) -> [f64; 3]| -> Vec<Point3> {
            let mut v = Vec::new();
            for i in 0..=20 {
                for j in 0..=20 {
                    let p = f(i as f64 / 20.0, j as f64 / 20.0);
                    v.push(Point3::new(p[0], p[1], p[2]));
                }
            }
            v
        };
        let top_n = [-t.sin(), 0.0, t.cos()];
        let faces: Vec<([f64; 3], Vec<Point3>)> = vec![
            ([0.0, 0.0, -1.0], grid(&|u, v| [100.0 * u, 40.0 * v, 0.0])),
            // the tilted top: z = 10 + (x - 50) tan t
            (top_n, grid(&|u, v| [100.0 * u, 40.0 * v, 10.0 + (100.0 * u - 50.0) * t.tan()])),
            ([-1.0, 0.0, 0.0], grid(&|u, v| [0.0, 40.0 * u, (10.0 - 50.0 * t.tan()) * v])),
            ([1.0, 0.0, 0.0], grid(&|u, v| [100.0, 40.0 * u, (10.0 + 50.0 * t.tan()) * v])),
            ([0.0, -1.0, 0.0], grid(&|u, v| [100.0 * u, 0.0, (10.0 + (100.0 * u - 50.0) * t.tan()) * v])),
            ([0.0, 1.0, 0.0], grid(&|u, v| [100.0 * u, 40.0, (10.0 + (100.0 * u - 50.0) * t.tan()) * v])),
        ];
        let scan: Vec<Point3> = faces.iter().flat_map(|f| f.1.iter().copied()).collect();
        let items = faces
            .into_iter()
            .enumerate()
            .map(|(i, (n, pts))| {
                let c: Vec3 = pts.iter().map(|p| p.coords).sum::<Vec3>() / pts.len() as f64;
                Item {
                    id: ObjectId(i as u64),
                    kind: "plane",
                    cut: Cut { shape: Shape::Plane { point: arr(c), normal: n }, convex: true, full: false },
                    pts,
                }
            })
            .collect();
        (items, scan)
    }

    #[test]
    fn a_real_draft_on_a_long_face_is_not_squared() {
        // 0.2 degrees over 50 mm to either side of the centroid: 0.17 mm, above max(2 sigma, 0.05)
        let (items, scan) = slab(0.2);
        let p = plan(&items, &scan, 0.3);
        let top = p.squared.iter().find(|q| q.item == 1).expect("the top plane was considered");
        assert!(!top.applied && top.max_shift > 0.15 && top.limit < 0.06, "{top:?}");
        assert!(p.roles[1].how.contains("not squared"), "{}", p.roles[1].how);
        assert!(matches!(&p.roles[1].shape, Shape::Plane { normal, .. } if normal[0].abs() > 1e-3));
        // the report carries it
        let r = report(&items, &p);
        assert_eq!(r.squared_planes.iter().filter(|q| !q.applied).count(), 1);
        // a hair of tilt is squared; zero disables
        let (items, scan) = slab(0.005);
        let p = plan(&items, &scan, 0.3);
        assert!(p.squared.iter().any(|q| q.item == 1 && q.applied), "{:?}", p.squared);
        assert!(matches!(&p.roles[1].shape, Shape::Plane { normal, .. } if normal[0] == 0.0));
        assert!(plan(&items, &scan, 0.0).squared.is_empty());
    }
}
