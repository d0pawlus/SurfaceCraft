//! Plane sections of a mesh: the polylines where the plane cuts the triangles, and the analysis
//! of a stack of parallel planes (chains, areas, fitted circles, draft and taper trends).
use crate::{Mesh, Point3, Vec3};
use std::collections::HashMap;
use std::f64::consts::{FRAC_PI_6, TAU};

/// One section polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct Polyline {
    /// Points in order.
    pub points: Vec<Point3>,
    /// Does it close on itself?
    pub closed: bool,
}

/// The section of `mesh` by the plane through `origin` with normal `normal`, chained into
/// polylines (each crossing point is identified by the mesh edge it lies on).
#[must_use]
pub fn section(mesh: &Mesh, origin: &Point3, normal: &Vec3) -> Vec<Polyline> {
    let Some(n) = crate::normalize(*normal) else { return Vec::new() };
    let dist: Vec<f64> = mesh.positions.iter().map(|p| (p - origin).dot(&n)).collect();
    // nudge exact zeros so every crossing lies strictly inside an edge
    let eps = 1e-12 * (1.0 + mesh.bbox().diagonal());
    let d = |i: u32| {
        let v = dist.get(i as usize).copied().unwrap_or(0.0);
        if v.abs() < eps { eps } else { v }
    };
    let key = |a: u32, b: u32| if a < b { (a, b) } else { (b, a) };
    let point = |a: u32, b: u32| -> Option<Point3> {
        let (pa, pb) = (mesh.positions.get(a as usize)?, mesh.positions.get(b as usize)?);
        let (da, db) = (d(a), d(b));
        let t = da / (da - db);
        Some(pa + (pb - pa) * t)
    };
    // segments between edge keys
    let mut adj: HashMap<(u32, u32), Vec<(u32, u32)>> = HashMap::new();
    for t in &mesh.triangles {
        let mut cut = Vec::with_capacity(2);
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if (d(a) > 0.0) != (d(b) > 0.0) {
                cut.push(key(a, b));
            }
        }
        if let [e0, e1] = cut.as_slice() {
            adj.entry(*e0).or_default().push(*e1);
            adj.entry(*e1).or_default().push(*e0);
        }
    }
    let mut seen: HashMap<(u32, u32), bool> = HashMap::new();
    let mut out = Vec::new();
    // start open chains at their ends first, then the closed loops
    let mut starts: Vec<(u32, u32)> = adj.iter().filter(|(_, v)| v.len() == 1).map(|(k, _)| *k).collect();
    let mut rest: Vec<(u32, u32)> = adj.keys().copied().collect();
    starts.sort_unstable();
    rest.sort_unstable();
    starts.extend(rest);
    for s in starts {
        if seen.contains_key(&s) {
            continue;
        }
        let mut chain = vec![s];
        seen.insert(s, true);
        let mut cur = s;
        loop {
            let next = adj.get(&cur).and_then(|v| v.iter().copied().find(|k| !seen.contains_key(k)));
            match next {
                Some(k) => {
                    seen.insert(k, true);
                    chain.push(k);
                    cur = k;
                }
                None => break,
            }
        }
        let closed = chain.len() > 2 && adj.get(&cur).is_some_and(|v| v.contains(&s));
        let points: Vec<Point3> = chain.iter().filter_map(|&(a, b)| point(a, b)).collect();
        if points.len() >= 2 {
            out.push(Polyline { points, closed });
        }
    }
    out
}

/// A plane with axes: an `origin`, a unit `normal` and two in-plane unit axes `u`, `v` with
/// `u × v = normal`. The first axis is the world axis least aligned with the normal, so a plane
/// normal to Z has `u = X` and `v = Y`.
#[derive(Clone, Debug, PartialEq)]
pub struct PlaneFrame {
    /// A point of the reference plane.
    pub origin: Point3,
    /// Unit normal.
    pub normal: Vec3,
    /// First in-plane unit axis.
    pub u: Vec3,
    /// Second in-plane unit axis.
    pub v: Vec3,
}

impl PlaneFrame {
    /// The frame of the plane through `origin` with normal `normal`; `None` when the normal is zero
    /// or not finite, or the origin is not finite.
    #[must_use]
    pub fn new(origin: Point3, normal: Vec3) -> Option<Self> {
        if !crate::finite(&origin) {
            return None;
        }
        let n = crate::normalize(normal)?;
        let (ax, ay, az) = (n.x.abs(), n.y.abs(), n.z.abs());
        let seed = if ax <= ay && ax <= az {
            Vec3::x()
        } else if ay <= az {
            Vec3::y()
        } else {
            Vec3::z()
        };
        let u = crate::normalize(seed - n * seed.dot(&n))?;
        let v = n.cross(&u);
        Some(Self { origin, normal: n, u, v })
    }

    /// The point at distance `offset` from the origin along the normal.
    #[must_use]
    pub fn at(&self, offset: f64) -> Point3 {
        self.origin + self.normal * offset
    }

    /// The point with plane coordinates `p` (mm) at distance `offset` along the normal.
    #[must_use]
    pub fn point(&self, p: [f64; 2], offset: f64) -> Point3 {
        self.origin + self.u * p[0] + self.v * p[1] + self.normal * offset
    }

    /// Plane coordinates `[u, v]` (mm) of a point.
    #[must_use]
    pub fn to_2d(&self, p: &Point3) -> [f64; 2] {
        let d = *p - self.origin;
        [d.dot(&self.u), d.dot(&self.v)]
    }
}

/// A near-circular closed chain and its least-squares circle.
#[derive(Clone, Debug, PartialEq)]
pub struct CircleFit {
    /// Centre in plane coordinates (mm).
    pub center: [f64; 2],
    /// Radius (mm).
    pub radius: f64,
    /// Largest distance of a chain point from the circle (mm).
    pub max_dev: f64,
}

/// The least-squares circle through the plane points `pts`, when they are near-circular: at least
/// eight points, every point within 2 % of the radius, and no angular gap above 30 degrees. Corners
/// of a square are concyclic but leave 90 degree gaps, so a square is not near-circular.
#[must_use]
pub fn fit_circle(pts: &[[f64; 2]]) -> Option<CircleFit> {
    let n = pts.len();
    if n < 8 {
        return None;
    }
    let nf = n as f64;
    let mx = pts.iter().map(|p| p[0]).sum::<f64>() / nf;
    let my = pts.iter().map(|p| p[1]).sum::<f64>() / nf;
    let (mut suu, mut svv, mut suv) = (0.0, 0.0, 0.0);
    let (mut suuu, mut svvv, mut suvv, mut svuu) = (0.0, 0.0, 0.0, 0.0);
    for p in pts {
        let (u, v) = (p[0] - mx, p[1] - my);
        suu += u * u;
        svv += v * v;
        suv += u * v;
        suuu += u * u * u;
        svvv += v * v * v;
        suvv += u * v * v;
        svuu += v * u * u;
    }
    // normal equations of z = u² + v² against 2a·u + 2b·v + c, with centred coordinates
    let det = suu * svv - suv * suv;
    if !(det.abs() > 1e-10 * suu * svv) {
        return None;
    }
    let (ru, rv) = (0.5 * (suuu + suvv), 0.5 * (svvv + svuu));
    let a = (ru * svv - suv * rv) / det;
    let b = (suu * rv - suv * ru) / det;
    let r2 = a * a + b * b + (suu + svv) / nf;
    if !(r2.is_finite() && r2 > 0.0) {
        return None;
    }
    let radius = r2.sqrt();
    let center = [mx + a, my + b];
    let mut max_dev = 0.0_f64;
    let mut angles = Vec::with_capacity(n);
    for p in pts {
        let (dx, dy) = (p[0] - center[0], p[1] - center[1]);
        max_dev = max_dev.max((dx.hypot(dy) - radius).abs());
        angles.push(dy.atan2(dx));
    }
    if !(max_dev <= 0.02 * radius) {
        return None;
    }
    angles.sort_by(f64::total_cmp);
    let (first, last) = (angles.first().copied()?, angles.last().copied()?);
    let wrap = first + TAU - last;
    let gap = angles.windows(2).filter_map(|w| match w {
        [x, y] => Some(y - x),
        _ => None,
    });
    let largest = gap.fold(wrap, f64::max);
    if largest > FRAC_PI_6 { None } else { Some(CircleFit { center, radius, max_dev }) }
}

/// A straight line `y = intercept + slope · x`, fitted by least squares.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearFit {
    /// Slope.
    pub slope: f64,
    /// Value at `x = 0`.
    pub intercept: f64,
    /// Root-mean-square residual.
    pub rms: f64,
}

/// The least-squares line through the `(x, y)` pairs; `None` with fewer than two pairs or with a
/// constant `x`.
#[must_use]
pub fn fit_line(xy: &[(f64, f64)]) -> Option<LinearFit> {
    let n = xy.len();
    if n < 2 {
        return None;
    }
    let nf = n as f64;
    let mx = xy.iter().map(|&(x, _)| x).sum::<f64>() / nf;
    let my = xy.iter().map(|&(_, y)| y).sum::<f64>() / nf;
    let sxx: f64 = xy.iter().map(|&(x, _)| (x - mx) * (x - mx)).sum();
    let sxy: f64 = xy.iter().map(|&(x, y)| (x - mx) * (y - my)).sum();
    if !(sxx > 0.0) {
        return None;
    }
    let slope = sxy / sxx;
    let intercept = my - slope * mx;
    let ss: f64 = xy
        .iter()
        .map(|&(x, y)| {
            let r = y - (intercept + slope * x);
            r * r
        })
        .sum();
    Some(LinearFit { slope, intercept, rms: (ss / nf).sqrt() })
}

/// The sections of one plane, analysed in a [`PlaneFrame`].
#[derive(Clone, Debug, PartialEq)]
pub struct LevelStats {
    /// Distance of the plane from the frame origin along the normal (mm).
    pub offset: f64,
    /// Closed chains.
    pub closed: usize,
    /// Open chains.
    pub open: usize,
    /// Length of every chain, the closing edge of a closed chain included (mm).
    pub perimeter: f64,
    /// Area enclosed by the closed chains, holes subtracted (mm²).
    pub area: f64,
    /// Centroid of that area, or the length-weighted centroid of the open chains when there is no
    /// area (model coordinates, mm).
    pub centroid: Option<Point3>,
    /// Extent of every chain along `u` and `v` (mm).
    pub size: [f64; 2],
    /// The near-circular closed chains, largest radius first.
    pub circles: Vec<CircleFit>,
    /// The closed chain of largest area, in plane coordinates (the outline).
    pub outline: Option<Vec<[f64; 2]>>,
}

/// Closed chains beyond this count are not nested: their holes are not subtracted.
const NEST_LIMIT: usize = 256;

/// Analyses the sections `polys` of one plane (`offset` along the frame normal).
#[must_use]
pub fn analyse_level(frame: &PlaneFrame, offset: f64, polys: &[Polyline]) -> LevelStats {
    let mut loops: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut open = 0;
    let mut perimeter = 0.0;
    let mut open_len = 0.0;
    let mut open_mid = [0.0; 2];
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for pl in polys {
        let pts: Vec<[f64; 2]> = pl.points.iter().map(|p| frame.to_2d(p)).collect();
        if pts.len() < 2 {
            continue;
        }
        for p in &pts {
            lo = [lo[0].min(p[0]), lo[1].min(p[1])];
            hi = [hi[0].max(p[0]), hi[1].max(p[1])];
        }
        let mut edges: Vec<([f64; 2], [f64; 2])> = pts
            .windows(2)
            .filter_map(|w| match w {
                [a, b] => Some((*a, *b)),
                _ => None,
            })
            .collect();
        if let (true, Some(&last), Some(&first)) = (pl.closed, pts.last(), pts.first()) {
            edges.push((last, first));
        }
        for &(a, b) in &edges {
            let len = (b[0] - a[0]).hypot(b[1] - a[1]);
            perimeter += len;
            if !pl.closed {
                open_len += len;
                open_mid = [open_mid[0] + 0.5 * (a[0] + b[0]) * len, open_mid[1] + 0.5 * (a[1] + b[1]) * len];
            }
        }
        if pl.closed {
            loops.push(pts);
        } else {
            open += 1;
        }
    }
    // even-odd region: a loop nested in an odd number of other loops is a hole
    let nested = loops.len() <= NEST_LIMIT;
    let mut area = 0.0;
    let mut moment = [0.0; 2];
    let mut best: Option<(f64, usize)> = None;
    for (i, lp) in loops.iter().enumerate() {
        let (signed, c) = polygon_area_centroid(lp);
        let a = signed.abs();
        let depth = match lp.first() {
            Some(p0) if nested => loops.iter().enumerate().filter(|&(j, other)| j != i && contains(other, *p0)).count(),
            _ => 0,
        };
        let sign = if depth.is_multiple_of(2) { 1.0 } else { -1.0 };
        area += sign * a;
        moment = [moment[0] + sign * a * c[0], moment[1] + sign * a * c[1]];
        if best.is_none_or(|(b, _)| a > b) {
            best = Some((a, i));
        }
    }
    let centroid = if area.abs() > 1e-12 {
        Some([moment[0] / area, moment[1] / area])
    } else if open_len > 0.0 {
        Some([open_mid[0] / open_len, open_mid[1] / open_len])
    } else {
        None
    };
    let size = if lo[0] <= hi[0] && lo[1] <= hi[1] { [hi[0] - lo[0], hi[1] - lo[1]] } else { [0.0; 2] };
    let mut circles: Vec<CircleFit> = loops.iter().filter_map(|lp| fit_circle(lp)).collect();
    circles.sort_by(|a, b| b.radius.total_cmp(&a.radius));
    let outline = best.and_then(|(_, i)| loops.get(i).cloned());
    LevelStats {
        offset,
        closed: loops.len(),
        open,
        perimeter,
        area,
        centroid: centroid.map(|c| frame.point(c, offset)),
        size,
        circles,
        outline,
    }
}

/// Signed area and centroid of the polygon `pts` (implicitly closed).
fn polygon_area_centroid(pts: &[[f64; 2]]) -> (f64, [f64; 2]) {
    let n = pts.len();
    if n == 0 {
        return (0.0, [0.0; 2]);
    }
    let (mut twice, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for (i, p) in pts.iter().enumerate() {
        let q = pts.get((i + 1) % n).copied().unwrap_or(*p);
        let cross = p[0] * q[1] - q[0] * p[1];
        twice += cross;
        cx += (p[0] + q[0]) * cross;
        cy += (p[1] + q[1]) * cross;
    }
    if twice.abs() <= 1e-18 {
        let nf = n as f64;
        let mean = pts.iter().fold([0.0; 2], |m, p| [m[0] + p[0] / nf, m[1] + p[1] / nf]);
        return (0.0, mean);
    }
    (twice / 2.0, [cx / (3.0 * twice), cy / (3.0 * twice)])
}

/// Is `p` inside the polygon `poly` (even-odd rule)?
fn contains(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    let Some(&last) = poly.last() else { return false };
    let mut inside = false;
    let mut a = last;
    for &b in poly {
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < a[0] + (p[1] - a[1]) * (b[0] - a[0]) / (b[1] - a[1]) {
            inside = !inside;
        }
        a = b;
    }
    inside
}

/// One side of the outline, measured by how its extent along a plane axis moves with height.
#[derive(Clone, Debug, PartialEq)]
pub struct SideDraft {
    /// `+u`, `-u`, `+v` or `-v`.
    pub side: &'static str,
    /// Draft in degrees, from the least-squares slope of the extent against the offset: positive
    /// when the side moves inward as the stack rises (the outline narrows).
    pub draft_deg: f64,
    /// Root-mean-square residual of the straight-line fit (mm).
    pub rms: f64,
}

/// The taper of the outline radii over the stack.
#[derive(Clone, Debug, PartialEq)]
pub struct RadiusTrend {
    /// Radius against offset (slope in mm per mm, residual in mm).
    pub fit: LinearFit,
    /// Taper in degrees: positive when the radius shrinks as the stack rises.
    pub taper_deg: f64,
    /// Levels whose outline is near-circular.
    pub levels: usize,
}

/// The trends over a stack.
#[derive(Clone, Debug, PartialEq)]
pub struct Trend {
    /// `prismatic` (constant chains, area and zero draft), `tapered` (linear draft, not constant),
    /// `varying`, or `insufficient` (fewer than two levels with an outline).
    pub profile: &'static str,
    /// Draft per side; empty when insufficient.
    pub sides: Vec<SideDraft>,
    /// Taper of the outline radii, when at least two outlines are near-circular.
    pub radius: Option<RadiusTrend>,
}

/// A stack of parallel sections with its trends.
#[derive(Clone, Debug, PartialEq)]
pub struct StackReport {
    /// The frame the levels are measured in.
    pub frame: PlaneFrame,
    /// One entry per plane, in order.
    pub levels: Vec<LevelStats>,
    /// The trends over the levels.
    pub trend: Trend,
}

/// The four sides of the outline: name and in-plane direction.
const SIDES: [(&str, [f64; 2]); 4] = [("+u", [1.0, 0.0]), ("-u", [-1.0, 0.0]), ("+v", [0.0, 1.0]), ("-v", [0.0, -1.0])];
/// A draft up to this many degrees, on every side, counts as vertical.
const PRISMATIC_DRAFT_DEG: f64 = 0.05;
/// Relative spread of the outline area that still counts as constant.
const PRISMATIC_AREA_REL: f64 = 1e-3;
/// A residual up to this many mm means a linear draft.
const LINEAR_RMS: f64 = 0.01;

/// Largest projection of a plane polygon on the in-plane direction `d`.
fn support(poly: &[[f64; 2]], d: [f64; 2]) -> f64 {
    poly.iter().map(|p| p[0] * d[0] + p[1] * d[1]).fold(f64::NEG_INFINITY, f64::max)
}

fn trend_of(levels: &[LevelStats]) -> Trend {
    let insufficient = || Trend { profile: "insufficient", sides: Vec::new(), radius: None };
    let outlined: Vec<(f64, &[[f64; 2]])> =
        levels.iter().filter_map(|l| l.outline.as_deref().map(|o| (l.offset, o))).collect();
    if outlined.len() < 2 {
        return insufficient();
    }
    let mut sides = Vec::new();
    for (side, dir) in SIDES {
        let xy: Vec<(f64, f64)> = outlined.iter().map(|&(z, o)| (z, support(o, dir))).collect();
        if let Some(fit) = fit_line(&xy) {
            sides.push(SideDraft { side, draft_deg: (-fit.slope).atan().to_degrees(), rms: fit.rms });
        }
    }
    if sides.is_empty() {
        return insufficient();
    }
    let radii: Vec<(f64, f64)> = outlined.iter().filter_map(|&(z, o)| fit_circle(o).map(|c| (z, c.radius))).collect();
    let radius = if radii.len() >= 2 {
        fit_line(&radii).map(|fit| RadiusTrend {
            taper_deg: (-fit.slope).atan().to_degrees(),
            levels: radii.len(),
            fit,
        })
    } else {
        None
    };
    let all_outlined = outlined.len() == levels.len();
    let (closed0, open0) = levels.first().map_or((0, 0), |l| (l.closed, l.open));
    let same_chains = levels.iter().all(|l| l.closed == closed0 && l.open == open0);
    let amax = levels.iter().map(|l| l.area).fold(f64::NEG_INFINITY, f64::max);
    let amin = levels.iter().map(|l| l.area).fold(f64::INFINITY, f64::min);
    let same_area = amax - amin <= PRISMATIC_AREA_REL * amax.abs();
    let flat = sides.iter().all(|s| s.draft_deg.abs() <= PRISMATIC_DRAFT_DEG);
    let linear = sides.iter().all(|s| s.rms <= LINEAR_RMS);
    let profile = if !all_outlined || !same_chains {
        "varying"
    } else if same_area && flat {
        "prismatic"
    } else if linear {
        "tapered"
    } else {
        "varying"
    };
    Trend { profile, sides, radius }
}

/// Cuts `mesh` with `count` planes parallel to `frame`, plane `k` lying at offset `start + k · step`
/// along the normal, and analyses each section and the trends over the stack.
#[must_use]
pub fn stack(mesh: &Mesh, frame: &PlaneFrame, start: f64, step: f64, count: usize) -> StackReport {
    let levels: Vec<LevelStats> = (0..count)
        .map(|k| {
            let offset = start + step * k as f64;
            let polys = section(mesh, &frame.at(offset), &frame.normal);
            analyse_level(frame, offset, &polys)
        })
        .collect();
    let trend = trend_of(&levels);
    StackReport { frame: frame.clone(), levels, trend }
}

/// Distance from `p` to the segment `a`–`b`.
fn segment_distance(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (a[0] + t * dx - p[0]).hypot(a[1] + t * dy - p[1])
}

/// Marks the points of the run `first..=last` that a polyline within `tol` of every point of the
/// run has to keep (Ramer–Douglas–Peucker, without recursion). The ends are the caller's.
fn keep_run(points: &[[f64; 2]], first: usize, last: usize, tol: f64, keep: &mut [bool]) {
    let mut runs = vec![(first, last)];
    while let Some((i, j)) = runs.pop() {
        let (Some(&a), Some(&b)) = (points.get(i), points.get(j)) else { continue };
        let mut worst = (tol, None);
        for (k, &p) in points.iter().enumerate().take(j).skip(i + 1) {
            let d = segment_distance(p, a, b);
            if d > worst.0 {
                worst = (d, Some(k));
            }
        }
        if let Some(k) = worst.1 {
            if let Some(flag) = keep.get_mut(k) {
                *flag = true;
            }
            runs.push((i, k));
            runs.push((k, j));
        }
    }
}

/// The point of `points` farthest from `from` (the first one on a tie or without a finite
/// distance).
fn farthest(points: &[[f64; 2]], from: [f64; 2]) -> usize {
    let mut best = (0.0_f64, 0_usize);
    for (k, p) in points.iter().enumerate() {
        let d = (p[0] - from[0]).hypot(p[1] - from[1]);
        if d > best.0 {
            best = (d, k);
        }
    }
    best.1
}

/// Drops vertices of a plane chain so that every dropped vertex stays within `tol` (mm) of the
/// simplified chain: the collinear runs a section of a planar face gives become single edges and
/// a dense curve keeps its shape. A closed chain is split at two extreme points, so a start that
/// lies inside a straight run is dropped too; the remaining points keep their order.
#[must_use]
pub fn simplify(points: &[[f64; 2]], closed: bool, tol: f64) -> Vec<[f64; 2]> {
    let n = points.len();
    let Some(&start) = points.first() else { return Vec::new() };
    if n < 3 {
        return points.to_vec();
    }
    if !closed {
        let mut keep = vec![false; n];
        for k in [0, n - 1] {
            if let Some(flag) = keep.get_mut(k) {
                *flag = true;
            }
        }
        keep_run(points, 0, n - 1, tol, &mut keep);
        return points.iter().zip(&keep).filter(|(_, k)| **k).map(|(p, _)| *p).collect();
    }
    // a point farthest from any point is a corner of the outline: start the loop there
    let anchor = farthest(points, start);
    let turned: Vec<[f64; 2]> = points.iter().cycle().skip(anchor).take(n + 1).copied().collect();
    let opposite = turned.first().map_or(0, |&a| farthest(&turned, a));
    let mut keep = vec![false; n + 1];
    for k in [0, opposite] {
        if let Some(flag) = keep.get_mut(k) {
            *flag = true;
        }
    }
    keep_run(&turned, 0, opposite, tol, &mut keep);
    keep_run(&turned, opposite, n, tol, &mut keep);
    // back to the order of `points`: `turned[k]` is `points[(k + anchor) % n]`
    (0..n)
        .filter(|i| keep.get((i + n - anchor) % n).copied().unwrap_or(false))
        .filter_map(|i| points.get(i).copied())
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;
    use std::f64::consts::TAU;

    #[test]
    fn cylinder_section_is_a_circle() {
        let m = synth::cylinder(Point3::origin(), Vec3::z(), 5.0, 10.0, 64, 4);
        let s = section(&m, &Point3::new(0.0, 0.0, 3.3), &Vec3::z());
        assert_eq!(s.len(), 1);
        assert!(s[0].closed);
        assert!(s[0].points.iter().all(|p| (p.x.hypot(p.y) - 5.0).abs() < 0.02 && (p.z - 3.3).abs() < 1e-9));
        let b = synth::cuboid(Point3::origin(), Vec3::new(10.0, 6.0, 4.0), 4);
        let s = section(&b, &Point3::new(0.0, 0.0, 1.7), &Vec3::z());
        assert_eq!(s.len(), 1);
        assert!(s[0].closed);
        assert!(section(&b, &Point3::new(0.0, 0.0, 99.0), &Vec3::z()).is_empty());
    }

    #[test]
    fn frame_is_orthonormal_and_circle_fit_is_exact() {
        let f = PlaneFrame::new(Point3::origin(), Vec3::z()).unwrap();
        assert!((f.u - Vec3::x()).norm() < 1e-12 && (f.v - Vec3::y()).norm() < 1e-12);
        let g = PlaneFrame::new(Point3::origin(), Vec3::new(1.0, 2.0, 3.0)).unwrap();
        assert!(g.u.dot(&g.normal).abs() < 1e-12 && g.v.dot(&g.normal).abs() < 1e-12);
        let ring: Vec<[f64; 2]> = (0..64)
            .map(|i| {
                let t = TAU * i as f64 / 64.0;
                [3.0 + 5.0 * t.cos(), -2.0 + 5.0 * t.sin()]
            })
            .collect();
        let c = fit_circle(&ring).unwrap();
        assert!((c.radius - 5.0).abs() < 1e-9 && (c.center[0] - 3.0).abs() < 1e-9);
        let square =
            [[0.0, 0.0], [5.0, 0.0], [10.0, 0.0], [10.0, 5.0], [10.0, 10.0], [5.0, 10.0], [0.0, 10.0], [0.0, 5.0]];
        assert!(fit_circle(&square).is_none());
        assert_eq!(simplify(&square, true, 1e-9).len(), 4);
    }

    /// Distance from `p` to the nearest edge of the chain `c`.
    fn to_chain(p: [f64; 2], c: &[[f64; 2]], closed: bool) -> f64 {
        let n = c.len();
        let edges = if closed { n } else { n.saturating_sub(1) };
        (0..edges).map(|i| segment_distance(p, c[i], c[(i + 1) % n])).fold(f64::INFINITY, f64::min)
    }

    /// A dense curve (point spacing below the tolerance) must not drift: every dropped point
    /// stays within the tolerance of the simplified chain.
    #[test]
    fn simplify_keeps_a_dense_curve_within_tolerance() {
        let (r, n, tol) = (10.0, 2000, 0.05);
        let circle: Vec<[f64; 2]> =
            (0..n).map(|i| TAU * f64::from(i) / f64::from(n)).map(|a| [r * a.cos(), r * a.sin()]).collect();
        for closed in [true, false] {
            let s = simplify(&circle, closed, tol);
            let worst = circle.iter().map(|&p| to_chain(p, &s, closed)).fold(0.0, f64::max);
            assert!(worst <= tol + 1e-12, "closed {closed}: {} points, worst {worst} mm", s.len());
            assert!(s.len() >= 30 && s.len() < 200, "closed {closed}: {} points", s.len());
        }
        // the start of a closed chain inside a straight run is not kept as a corner
        let square = [[5.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [0.0, 0.0], [2.0, 0.0]];
        assert_eq!(simplify(&square, true, 1e-9), vec![[10.0, 0.0], [10.0, 10.0], [0.0, 10.0], [0.0, 0.0]]);
        // degenerate and hostile input: no panic
        assert_eq!(simplify(&[], true, 0.1).len(), 0);
        assert_eq!(simplify(&[[1.0, 2.0]], true, 0.1).len(), 1);
        assert!(simplify(&[[f64::NAN, 0.0], [0.0, f64::INFINITY], [1.0, 1.0], [2.0, 0.0]], true, f64::NAN).len() <= 4);
    }
}
