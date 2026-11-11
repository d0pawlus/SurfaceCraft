//! Walls and rounds of a floor's tools in the floor's plane (the inputs of [`super::loops`]), the
//! polygon helpers they share and the frame that maps them back to 3-D.
use super::{Node, arr, pt, snap_direction, v3};
use surfacecraft_geom::{Point3, Vec3, any_perpendicular, normalize};
use surfacecraft_kernel::feature::ProfileCurve;
use surfacecraft_kernel::hybrid::Shape;

/// Points of a region kept for the 2-D profile (the regions are subsampled evenly).
const PROFILE_POINTS: usize = 400;
/// Largest angle between two samples of an arc in the polygon of a loop, radians.
const ARC_STEP: f64 = 0.1;
/// Fewest samples of an arc.
const MIN_ARC_SAMPLES: usize = 4;

/// Largest radius, and largest coordinate, of a profile (mm): anything beyond is not a part.
const MAX_EXTENT: f64 = 1e7;

/// A wall: a plane square to the floor, seen as a line.
pub(super) struct Wall {
    /// The planner's node of the region.
    pub(super) id: usize,
    /// Unit normal in the plane, towards the void.
    pub(super) n: [f64; 2],
    /// `n · x = c` on the wall.
    pub(super) c: f64,
    pub(super) pts: Vec<[f64; 2]>,
}

/// A round: a cylinder along the floor's normal, seen as a circle.
pub(super) struct Corner {
    /// The planner's node of the region.
    pub(super) id: usize,
    pub(super) center: [f64; 2],
    pub(super) r: f64,
    /// The void is inside the cylinder (a rounded corner of the void); else the material is (a
    /// reflex round, or a post).
    pub(super) concave: bool,
    /// The region goes all the way round the axis (a post).
    pub(super) full: bool,
    pub(super) pts: Vec<[f64; 2]>,
}

/// A piece of a profile. An arc runs from angle `a0` to `a1` about its centre; `a1 < a0` is a
/// clockwise arc (the round of a reflex corner).
#[derive(Clone, Debug, PartialEq)]
pub(super) enum Seg {
    Line([f64; 2], [f64; 2]),
    Arc { c: [f64; 2], r: f64, a0: f64, a1: f64 },
}

impl Seg {
    /// The same piece travelled the other way.
    pub(super) fn reversed(&self) -> Seg {
        match self {
            Seg::Line(a, b) => Seg::Line(*b, *a),
            Seg::Arc { c, r, a0, a1 } => Seg::Arc { c: *c, r: *r, a0: *a1, a1: *a0 },
        }
    }
}

/// The loop with every piece (and the order of the pieces) reversed.
pub(super) fn reverse_loop(segs: &[Seg]) -> Vec<Seg> {
    segs.iter().rev().map(Seg::reversed).collect()
}

pub(super) fn d2(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// Tangent direction of a wall in counter-clockwise travel (the void on the left).
pub(super) fn tangent(n: [f64; 2]) -> [f64; 2] {
    [n[1], -n[0]]
}

/// The meeting point of the lines `n1 · x = c1` and `n2 · x = c2`, `None` when they are (nearly)
/// parallel.
pub(super) fn solve(n1: [f64; 2], c1: f64, n2: [f64; 2], c2: f64) -> Option<[f64; 2]> {
    let det = n1[0] * n2[1] - n1[1] * n2[0];
    if det.abs() < 0.1 {
        return None;
    }
    Some([(c1 * n2[1] - c2 * n1[1]) / det, (n1[0] * c2 - n2[0] * c1) / det])
}

/// The outline of the pieces as a polygon (arcs sampled).
pub(super) fn polygon_of(segs: &[Seg]) -> Vec<[f64; 2]> {
    let mut polygon: Vec<[f64; 2]> = Vec::new();
    for s in segs {
        match s {
            Seg::Line(a, _) => polygon.push(*a),
            Seg::Arc { c, r, a0, a1 } => {
                let n = (((a1 - a0).abs() / ARC_STEP).ceil() as usize).max(MIN_ARC_SAMPLES);
                for k in 0..=n {
                    let th = a0 + (a1 - a0) * k as f64 / n as f64;
                    polygon.push([c[0] + r * th.cos(), c[1] + r * th.sin()]);
                }
            }
        }
    }
    polygon
}

/// Signed area of a polygon, positive when it runs counter-clockwise.
pub(super) fn signed_area(poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    let twice: f64 = (0..n)
        .filter_map(|i| Some((poly.get(i)?, poly.get((i + 1) % n)?)))
        .map(|(a, b)| a[0] * b[1] - b[0] * a[1])
        .sum();
    0.5 * twice
}

pub(super) fn inside_polygon(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let mut inside = false;
    let n = poly.len();
    for i in 0..n {
        let (Some(a), Some(b)) = (poly.get(i), poly.get((i + 1) % n)) else { continue };
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

pub(super) fn polygon_distance(poly: &[[f64; 2]], p: [f64; 2]) -> f64 {
    let n = poly.len();
    let mut best = f64::INFINITY;
    for i in 0..n {
        let (Some(a), Some(b)) = (poly.get(i), poly.get((i + 1) % n)) else { continue };
        let ab = [b[0] - a[0], b[1] - a[1]];
        let l2 = ab[0] * ab[0] + ab[1] * ab[1];
        let t = if l2 > 0.0 { (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / l2).clamp(0.0, 1.0) } else { 0.0 };
        best = best.min(d2(p, [a[0] + ab[0] * t, a[1] + ab[1] * t]));
    }
    best
}

/// The part of a polygon inside the rectangle `lo`..`hi` (Sutherland-Hodgman; the result of a
/// concave polygon may have edges along the rectangle, which does not change its area).
pub(super) fn clip_to_rect(poly: &[[f64; 2]], lo: [f64; 2], hi: [f64; 2]) -> Vec<[f64; 2]> {
    let mut out: Vec<[f64; 2]> = poly.to_vec();
    for axis in 0..2 {
        for upper in [false, true] {
            let bound = if upper { hi[axis] } else { lo[axis] };
            let inside = |p: &[f64; 2]| if upper { p[axis] <= bound } else { p[axis] >= bound };
            let input = std::mem::take(&mut out);
            for (i, a) in input.iter().enumerate() {
                let Some(b) = input.get((i + 1) % input.len()) else { continue };
                let cut = |a: &[f64; 2], b: &[f64; 2]| {
                    let t = (bound - a[axis]) / (b[axis] - a[axis]);
                    let mut p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
                    p[axis] = bound;
                    p
                };
                match (inside(a), inside(b)) {
                    (true, true) => out.push(*b),
                    (true, false) => out.push(cut(a, b)),
                    (false, true) => {
                        out.push(cut(a, b));
                        out.push(*b);
                    }
                    (false, false) => {}
                }
            }
        }
    }
    out
}

/// Do the segments `p1 p2` and `q1 q2` cross (a shared end point does not count)?
fn segments_cross(p1: [f64; 2], p2: [f64; 2], q1: [f64; 2], q2: [f64; 2]) -> bool {
    let side = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    let eps = 1e-9;
    let (d1, d2_, d3, d4) = (side(q1, q2, p1), side(q1, q2, p2), side(p1, p2, q1), side(p1, p2, q2));
    ((d1 > eps && d2_ < -eps) || (d1 < -eps && d2_ > eps)) && ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps))
}

/// Does the closed polygon cross itself?
pub(super) fn crosses_itself(poly: &[[f64; 2]]) -> bool {
    let n = poly.len();
    for i in 0..n {
        for j in (i + 2)..n {
            if i == 0 && j + 1 == n {
                continue;
            }
            if let (Some(a), Some(b), Some(c), Some(d)) =
                (poly.get(i), poly.get((i + 1) % n), poly.get(j), poly.get((j + 1) % n))
                && segments_cross(*a, *b, *c, *d)
            {
                return true;
            }
        }
    }
    false
}

/// A frame in the profile's plane: origin, two in-plane axes, normal `d` (e1 × e2 = d).
pub(super) struct Frame {
    pub(super) o: Vec3,
    pub(super) e1: Vec3,
    pub(super) e2: Vec3,
    pub(super) d: Vec3,
}

impl Frame {
    pub(super) fn new(o: Vec3, d: Vec3) -> Frame {
        let e1 = any_perpendicular(&d);
        Frame { o, e1, e2: d.cross(&e1), d }
    }
    pub(super) fn to2(&self, p: &Point3) -> [f64; 2] {
        let v = p.coords - self.o;
        [v.dot(&self.e1), v.dot(&self.e2)]
    }
    pub(super) fn to3(&self, p: [f64; 2]) -> Vec3 {
        self.o + self.e1 * p[0] + self.e2 * p[1]
    }
    /// The kernel's curves of a loop. A clockwise arc is written counter-clockwise about the
    /// reversed axis, whose angles run the other way.
    pub(super) fn curves(&self, segs: &[Seg]) -> Vec<ProfileCurve> {
        segs.iter()
            .map(|s| match s {
                Seg::Line(a, b) => ProfileCurve::Line { start: arr(self.to3(*a)), end: arr(self.to3(*b)) },
                Seg::Arc { c, r, a0, a1 } => {
                    let (axis, start, end) = if a1 >= a0 { (self.d, *a0, *a1) } else { (-self.d, -*a0, -*a1) };
                    ProfileCurve::Arc {
                        center: arr(self.to3(*c)),
                        axis: arr(axis),
                        reference: arr(self.e1),
                        radius: *r,
                        start,
                        end,
                    }
                }
            })
            .collect()
    }
}

/// A wall's normal made exactly square to the profile's direction `d` and, within 0.3°, exactly
/// along one of the part's axes.
pub(super) fn wall_normal(n: Vec3, d: Vec3, anchors: &[Vec3]) -> Option<Vec3> {
    let flat = normalize(n - d * n.dot(&d))?;
    let snapped = snap_direction(flat, anchors, 0.3);
    normalize(snapped - d * snapped.dot(&d))
}

/// The walls (planes) and rounds (cylinders along `frame.d`) of `members` in 2-D.
pub(super) fn walls_and_corners(
    nodes: &[Node],
    members: &[usize],
    frame: &Frame,
    anchors: &[Vec3],
) -> Result<(Vec<Wall>, Vec<Corner>), String> {
    let mut walls = Vec::new();
    let mut corners = Vec::new();
    for &id in members {
        let nd = nodes.get(id).ok_or("index")?;
        let step = (nd.pts.len() / PROFILE_POINTS).max(1);
        let pts: Vec<[f64; 2]> = nd.pts.iter().step_by(step).map(|p| frame.to2(p)).collect();
        match &nd.shape {
            Shape::Plane { point, normal } => {
                let flat = wall_normal(v3(*normal), frame.d, anchors).ok_or("a wall parallel to the floor")?;
                let n = [flat.dot(&frame.e1), flat.dot(&frame.e2)];
                let q = frame.to2(&pt(*point));
                // a wall plane's own point may be anywhere on it: the line is n · x = n · q
                walls.push(Wall { id, n, c: n[0] * q[0] + n[1] * q[1], pts });
            }
            Shape::Cylinder { point, radius, .. } => {
                let center = frame.to2(&pt(*point));
                corners.push(Corner { id, center, r: *radius, concave: !nd.convex, full: nd.full, pts });
            }
            _ => return Err("only planes and cylinders form a profile".into()),
        }
    }
    Ok((walls, corners))
}

fn point_is_sound(p: [f64; 2]) -> bool {
    p.iter().all(|c| c.is_finite() && c.abs() <= MAX_EXTENT)
}

/// A piece of a profile with finite coordinates in range and a positive radius.
pub(super) fn seg_is_sound(s: &Seg) -> bool {
    match s {
        Seg::Line(a, b) => point_is_sound(*a) && point_is_sound(*b),
        Seg::Arc { c, r, a0, a1 } => {
            point_is_sound(*c) && r.is_finite() && *r > 0.0 && *r <= MAX_EXTENT && a0.is_finite() && a1.is_finite()
        }
    }
}

/// Input is hostile: refuse numbers that are not finite, radii that are not positive and walls
/// that have no direction before anything is computed from them.
pub(super) fn check_input(walls: &[Wall], corners: &[Corner], ext: f64) -> Result<(), String> {
    let walls_ok = walls.iter().all(|w| {
        let len = w.n[0].hypot(w.n[1]);
        point_is_sound(w.n)
            && (0.5..=2.0).contains(&len)
            && w.c.is_finite()
            && w.c.abs() <= MAX_EXTENT
            && w.pts.iter().all(|p| point_is_sound(*p))
    });
    let corners_ok = corners.iter().all(|k| {
        point_is_sound(k.center)
            && k.r.is_finite()
            && k.r > 0.0
            && k.r <= MAX_EXTENT
            && k.pts.iter().all(|p| point_is_sound(*p))
    });
    if !(walls_ok && corners_ok && ext.is_finite() && ext > 0.0 && ext <= MAX_EXTENT) {
        return Err(
            "a wall or round has a coordinate that is not a finite number in range, or a radius that is not positive"
                .into(),
        );
    }
    if walls.iter().any(|w| w.pts.is_empty()) || corners.iter().any(|k| k.pts.is_empty()) {
        return Err("a region has no points".into());
    }
    Ok(())
}
