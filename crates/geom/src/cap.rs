//! The cap of a plane section: the cut face of a closed mesh, filled where the plane is inside the
//! solid. The section loops of [`section::section`] bound the face by the even-odd rule (a loop
//! inside another one is a hole of it), so the cap is the region that an odd number of loops
//! enclose. The region is cut into trapezoids by a sweep over the heights of the loops' corners,
//! and each trapezoid gives two triangles; no hole has to be bridged, and any nesting works.
//!
//! The sweep keeps only the sides that cross the current band, so its cost follows the sides and the
//! triangles, not the square of the section. Nothing is filled that the parity cannot settle: a loop
//! whose sides cross is not filled and is counted; an open chain turns the cap off for the whole
//! section, since the parity of the loops around it is unknown; a section beyond [`Budget`] is not
//! filled, and says so.
use crate::section::{PlaneFrame, Polyline, section};
use crate::{Mesh, Point3, Vec3};

/// A closed loop whose twice-signed area is at most this (mm²) encloses nothing.
const DEGENERATE_AREA2: f64 = 1e-12;

/// Two neighbouring sides are out of order (they cross inside a band) when they differ by more than
/// this (mm) at the band's bottom or top.
const ORDER_TOL: f64 = 1e-9;

/// The sweep runs again without the loops it found crossing, at most this many times.
const MAX_PASSES: usize = 3;

/// How large a section may be and still be filled.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Budget {
    /// Non-horizontal sides of the closed loops.
    pub max_sides: usize,
    /// Triangles of the cut face.
    pub max_triangles: usize,
}

/// The budget of the viewport: a section beyond it is not filled, and the viewport says so.
pub const DEFAULT_BUDGET: Budget = Budget { max_sides: 250_000, max_triangles: 500_000 };

/// A plane section of a mesh and the cap filling it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Cut {
    /// The section polylines, open and closed, as [`section::section`] gives them.
    pub lines: Vec<Polyline>,
    /// The filled cut face.
    pub cap: Cap,
}

/// The filled cut face of a plane section.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Cap {
    /// The triangles of the cut face, on the plane.
    pub triangles: Vec<[Point3; 3]>,
    /// The area of the cap (mm²).
    pub area: f64,
    /// Section chains that do not close: with any of them, the section gets no cap at all.
    pub open_loops: usize,
    /// Closed loops that are not filled: fewer than three corners, no area, sides that cross, or
    /// every closed loop of a section that could not be filled.
    pub failed_loops: usize,
    /// The section is beyond the budget: nothing is filled.
    pub over_limit: bool,
}

/// The section of `mesh` by the plane through `origin` with normal `normal`, and its cap. An empty
/// cut when the normal is zero or not finite.
#[must_use]
pub fn cut(mesh: &Mesh, origin: &Point3, normal: &Vec3) -> Cut {
    let lines = section(mesh, origin, normal);
    let cap = PlaneFrame::new(*origin, *normal).map(|frame| cap_of(&lines, &frame)).unwrap_or_default();
    Cut { lines, cap }
}

/// A non-horizontal side of a closed loop, `lo` below `hi` (plane coordinates), and its loop.
#[derive(Clone, Copy, Debug)]
struct Side {
    lo: [f64; 2],
    hi: [f64; 2],
    ring: usize,
}

impl Side {
    /// The abscissa of the side at height `y` (the side spans it).
    fn x_at(&self, y: f64) -> f64 {
        self.lo[0] + (y - self.lo[1]) * (self.hi[0] - self.lo[0]) / (self.hi[1] - self.lo[1])
    }
}

/// Why the sweep stopped before the whole face was built.
#[derive(Debug)]
enum Defect {
    /// These loops cross, or an odd number of sides crosses a band: they are not filled.
    Rings(Vec<usize>),
    /// The section is beyond the budget.
    TooLarge,
}

/// The cap of the section polylines `lines` on the plane `frame`, within [`DEFAULT_BUDGET`].
#[must_use]
pub fn cap_of(lines: &[Polyline], frame: &PlaneFrame) -> Cap {
    cap_of_with_budget(lines, frame, DEFAULT_BUDGET)
}

/// The cap of the section polylines `lines` on the plane `frame`, filled only within `budget`.
#[must_use]
pub fn cap_of_with_budget(lines: &[Polyline], frame: &PlaneFrame, budget: Budget) -> Cap {
    let mut cap = Cap::default();
    let mut rings: Vec<Vec<[f64; 2]>> = Vec::new();
    for line in lines {
        if line.closed {
            rings.push(line.points.iter().map(|p| frame.to_2d(p)).collect());
        } else {
            cap.open_loops += 1;
        }
    }
    // an open chain (the mesh has a hole there) leaves the parity of the loops around it unknown
    if cap.open_loops > 0 {
        cap.failed_loops = rings.len();
        return cap;
    }
    let mut bad: Vec<bool> = rings.iter().map(|r| !is_fillable(r)).collect();
    cap.failed_loops = bad.iter().filter(|b| **b).count();
    for _ in 0..MAX_PASSES {
        match sweep(&sides_of(&rings, &bad), budget) {
            Ok(tris) => {
                for t in tris {
                    add_triangle(&mut cap, frame, t);
                }
                return cap;
            }
            Err(Defect::TooLarge) => {
                cap.over_limit = true;
                return cap;
            }
            Err(Defect::Rings(found)) => {
                let before = cap.failed_loops;
                for r in found {
                    if let Some(b) = bad.get_mut(r)
                        && !*b
                    {
                        *b = true;
                        cap.failed_loops += 1;
                    }
                }
                if cap.failed_loops == before {
                    break;
                }
            }
        }
    }
    // the defects could not be pinned on single loops: nothing is filled
    cap.failed_loops = rings.len();
    cap
}

/// Whether a closed loop encloses an area: three or more corners, all finite, and not flat.
fn is_fillable(ring: &[[f64; 2]]) -> bool {
    ring.len() >= 3
        && ring.iter().all(|p| p[0].is_finite() && p[1].is_finite())
        && signed_area2(ring).abs() > DEGENERATE_AREA2
}

/// The sides of the loops that are not `bad`.
fn sides_of(rings: &[Vec<[f64; 2]>], bad: &[bool]) -> Vec<Side> {
    let mut sides = Vec::new();
    for (i, ring) in rings.iter().enumerate() {
        if !bad.get(i).copied().unwrap_or(true) {
            sides.extend(ring_sides(ring, i));
        }
    }
    sides
}

/// Sweeps the bands between the heights of the sides' corners and returns the triangles of the cut
/// face (plane coordinates). Within a band the sides that cross it are kept sorted by abscissa, and
/// every pair of neighbours is inside the face. Sides that cross inside a band, and a band with an
/// odd number of sides, are refused, with the loops they belong to.
fn sweep(sides: &[Side], budget: Budget) -> Result<Vec<[[f64; 2]; 3]>, Defect> {
    if sides.len() > budget.max_sides {
        return Err(Defect::TooLarge);
    }
    let mut levels: Vec<f64> = sides.iter().flat_map(|s| [s.lo[1], s.hi[1]]).collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup();
    let (mut flats, mut slopes): (Vec<Side>, Vec<Side>) = sides.iter().copied().partition(|s| s.lo[1] == s.hi[1]);
    flats.sort_by(|a, b| a.lo[1].total_cmp(&b.lo[1]));
    slopes.sort_by(|a, b| a.lo[1].total_cmp(&b.lo[1]));
    let (mut next_slope, mut next_flat) = (0, 0);
    let mut active: Vec<Side> = Vec::new();
    let mut out: Vec<[[f64; 2]; 3]> = Vec::new();
    for band in levels.windows(2) {
        let (y0, y1) = (band[0], band[1]);
        active.retain(|s| s.hi[1] > y0);
        while let Some(s) = slopes.get(next_slope)
            && s.lo[1] <= y0
        {
            active.push(*s);
            next_slope += 1;
        }
        let mut at_bottom = Vec::new();
        while let Some(f) = flats.get(next_flat)
            && f.lo[1] <= y0
        {
            if f.lo[1] == y0 {
                at_bottom.push(*f);
            }
            next_flat += 1;
        }
        let mid = 0.5 * (y0 + y1);
        active.sort_by(|a, b| a.x_at(mid).total_cmp(&b.x_at(mid)));
        if let Some(found) = crossings(&at_bottom, &active, y0, y1) {
            return Err(Defect::Rings(found));
        }
        if active.len() % 2 == 1 {
            return Err(Defect::Rings(active.iter().map(|s| s.ring).collect()));
        }
        for [left, right] in active.as_chunks::<2>().0 {
            if out.len() + 2 > budget.max_triangles {
                return Err(Defect::TooLarge);
            }
            let quad = [[left.x_at(y0), y0], [right.x_at(y0), y0], [right.x_at(y1), y1], [left.x_at(y1), y1]];
            out.push([quad[0], quad[1], quad[2]]);
            out.push([quad[0], quad[2], quad[3]]);
        }
    }
    Ok(out)
}

/// The loops that cross inside the band `y0 .. y1`, if there are any. Two neighbouring sides out of
/// order at the bottom or the top of the band cross inside it. A horizontal side at the bottom of
/// the band (`flats`) crosses any side that passes through that height inside its span.
fn crossings(flats: &[Side], active: &[Side], y0: f64, y1: f64) -> Option<Vec<usize>> {
    let mut found = Vec::new();
    for pair in active.windows(2) {
        if let &[a, b] = pair
            && (a.x_at(y0) > b.x_at(y0) + ORDER_TOL || a.x_at(y1) > b.x_at(y1) + ORDER_TOL)
        {
            found.extend([a.ring, b.ring]);
        }
    }
    for f in flats {
        let (xa, xb) = (f.lo[0].min(f.hi[0]), f.lo[0].max(f.hi[0]));
        for s in active.iter().filter(|s| s.lo[1] < y0) {
            let x = s.x_at(y0);
            if x > xa + ORDER_TOL && x < xb - ORDER_TOL {
                found.extend([f.ring, s.ring]);
            }
        }
    }
    (!found.is_empty()).then_some(found)
}

/// Twice the signed area of a closed ring.
fn signed_area2(ring: &[[f64; 2]]) -> f64 {
    ring.iter().zip(ring.iter().cycle().skip(1)).map(|(a, b)| a[0] * b[1] - b[0] * a[1]).sum()
}

/// The sides of the closed ring `id`, horizontal ones included (they are kept apart by the sweep).
fn ring_sides(ring: &[[f64; 2]], id: usize) -> Vec<Side> {
    ring.iter()
        .zip(ring.iter().cycle().skip(1))
        .map(|(a, b)| if a[1] <= b[1] { Side { lo: *a, hi: *b, ring: id } } else { Side { lo: *b, hi: *a, ring: id } })
        .collect()
}

/// Adds the planar triangle `t` (plane coordinates) to the cap, unless it has no area.
fn add_triangle(cap: &mut Cap, frame: &PlaneFrame, t: [[f64; 2]; 3]) {
    let area2 = (t[1][0] - t[0][0]) * (t[2][1] - t[0][1]) - (t[2][0] - t[0][0]) * (t[1][1] - t[0][1]);
    if area2 == 0.0 || !area2.is_finite() {
        return;
    }
    cap.area += 0.5 * area2.abs();
    cap.triangles.push(t.map(|p| frame.point(p, 0.0)));
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;

    /// The plane through the origin with normal `normal`.
    fn frame(normal: Vec3) -> PlaneFrame {
        PlaneFrame::new(Point3::origin(), normal).expect("a valid plane")
    }

    /// A closed loop of plane points on the XY plane.
    fn loop_of(corners: &[[f64; 2]]) -> Polyline {
        Polyline { points: corners.iter().map(|p| Point3::new(p[0], p[1], 0.0)).collect(), closed: true }
    }

    fn square(x0: f64, y0: f64, s: f64) -> Polyline {
        loop_of(&[[x0, y0], [x0 + s, y0], [x0 + s, y0 + s], [x0, y0 + s]])
    }

    /// A closed tube: an outer cylinder of radius `outer` and a coaxial inner one of radius `inner`
    /// turned inside out, both along Z from z = -10 to 10.
    fn tube(outer: f64, inner: f64, n: usize) -> Mesh {
        let base = Point3::new(0.0, 0.0, -10.0);
        let mut m = synth::cylinder(base, Vec3::z(), outer, 20.0, n, 2);
        let mut hole = synth::cylinder(base, Vec3::z(), inner, 20.0, n, 2);
        for t in &mut hole.triangles {
            t.swap(1, 2);
        }
        m.append(&hole);
        m
    }

    /// The area of the regular polygon with `n` corners on a circle of radius `r`.
    fn polygon_area(n: usize, r: f64) -> f64 {
        0.5 * n as f64 * r * r * (std::f64::consts::TAU / n as f64).sin()
    }

    #[test]
    fn a_sphere_caps_to_a_disc_of_its_radius() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let c = cut(&mesh, &Point3::origin(), &Vec3::z());
        assert_eq!(c.lines.len(), 1);
        assert_eq!((c.cap.open_loops, c.cap.failed_loops), (0, 0));
        let disc = std::f64::consts::PI * 100.0;
        assert!((c.cap.area - disc).abs() / disc < 0.01, "cap {} vs disc {disc}", c.cap.area);
        assert!((c.cap.area - polygon_area(48, 10.0)).abs() < 1e-6 * disc, "the cap is the chain's polygon");
        for t in &c.cap.triangles {
            for p in t {
                assert!(p.z.abs() < 1e-9, "on the plane: {p:?}");
            }
        }
    }

    #[test]
    fn a_hollow_tube_caps_to_an_annulus() {
        let mesh = tube(10.0, 6.0, 48);
        let c = cut(&mesh, &Point3::origin(), &Vec3::z());
        assert_eq!(c.lines.len(), 2, "the outer and the inner loop");
        let annulus = std::f64::consts::PI * (100.0 - 36.0);
        assert!((c.cap.area - (polygon_area(48, 10.0) - polygon_area(48, 6.0))).abs() < 1e-6 * annulus);
        assert!((c.cap.area - annulus).abs() / annulus < 0.01, "cap {} vs annulus {annulus}", c.cap.area);
        // nothing is drawn inside the bore
        for t in &c.cap.triangles {
            let (x, y) = ((t[0].x + t[1].x + t[2].x) / 3.0, (t[0].y + t[1].y + t[2].y) / 3.0);
            assert!(x.hypot(y) > 6.0 - 0.05, "centroid ({x}, {y}) inside the bore");
        }
    }

    /// A plate 20 mm square with three holes at different heights: the area is what is left.
    #[test]
    fn a_plate_with_several_holes_keeps_the_material_between_them() {
        let lines = vec![square(0.0, 0.0, 20.0), square(2.0, 2.0, 4.0), square(12.0, 3.0, 5.0), square(4.0, 12.0, 3.0)];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert!((cap.area - (400.0 - 16.0 - 25.0 - 9.0)).abs() < 1e-9, "area {}", cap.area);
        assert_eq!((cap.open_loops, cap.failed_loops), (0, 0));
        let holes = [(2.0, 2.0, 6.0, 6.0), (12.0, 3.0, 17.0, 8.0), (4.0, 12.0, 7.0, 15.0)];
        for t in &cap.triangles {
            let (x, y) = ((t[0].x + t[1].x + t[2].x) / 3.0, (t[0].y + t[1].y + t[2].y) / 3.0);
            for (x0, y0, x1, y1) in holes {
                assert!(!(x > x0 && x < x1 && y > y0 && y < y1), "centroid ({x}, {y}) inside a hole");
            }
        }
    }

    /// Nested loops alternate: a square, a hole in it, and an island in the hole.
    #[test]
    fn an_island_inside_a_hole_is_material_again() {
        let lines = vec![square(0.0, 0.0, 10.0), square(2.0, 2.0, 6.0), square(4.0, 4.0, 2.0)];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert!((cap.area - (100.0 - 36.0 + 4.0)).abs() < 1e-9, "area {}", cap.area);
    }

    /// A sphere without its front half: the cut is an arc that ends on the mesh's edge, so no cap.
    #[test]
    fn an_open_section_gets_no_cap_and_is_reported() {
        let mut mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let keep: Vec<[u32; 3]> = mesh
            .triangles
            .iter()
            .copied()
            .filter(|t| t.iter().all(|&v| mesh.positions.get(v as usize).is_some_and(|p| p.x <= 0.0)))
            .collect();
        mesh.triangles = keep;
        let c = cut(&mesh, &Point3::origin(), &Vec3::z());
        assert!(c.lines.iter().any(|l| !l.closed), "the chain is open");
        assert_eq!(c.cap.open_loops, c.lines.iter().filter(|l| !l.closed).count());
        assert!(c.cap.triangles.is_empty());
        assert_eq!(c.cap.area, 0.0);
    }

    #[test]
    fn a_plane_off_the_solid_has_no_cut() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let c = cut(&mesh, &Point3::new(0.0, 0.0, 50.0), &Vec3::z());
        assert!(c.lines.is_empty() && c.cap == Cap::default());
        let none = cut(&mesh, &Point3::origin(), &Vec3::zeros());
        assert!(none.lines.is_empty() && none.cap == Cap::default());
    }

    /// Degenerate loops (two corners, a bow tie, one corner repeated, a non-finite corner) are
    /// counted, never a crash or a NaN area.
    #[test]
    fn degenerate_loops_are_counted_and_do_not_panic() {
        let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
        let lines = vec![
            Polyline { points: vec![p(0.0, 0.0), p(1.0, 0.0)], closed: true },
            Polyline { points: vec![p(0.0, 0.0), p(4.0, 4.0), p(4.0, 0.0), p(0.0, 4.0)], closed: true },
            Polyline { points: vec![p(1.0, 1.0), p(1.0, 1.0), p(1.0, 1.0)], closed: true },
            Polyline { points: vec![p(0.0, 0.0), p(f64::NAN, 1.0), p(1.0, 1.0)], closed: true },
            Polyline { points: vec![p(0.0, 0.0), p(1.0, 0.0)], closed: false },
        ];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert_eq!((cap.open_loops, cap.failed_loops), (1, 4));
        assert_eq!((cap.area, cap.triangles.len()), (0.0, 0));
    }

    /// A bow tie whose lobes are unequal: its edges cross, so it bounds no region and is not filled.
    #[test]
    fn a_self_crossing_loop_is_not_filled_and_is_counted() {
        let cap = cap_of(&[loop_of(&[[0.0, 0.0], [4.0, 4.0], [4.0, 0.0], [0.0, 2.0]])], &frame(Vec3::z()));
        assert_eq!((cap.failed_loops, cap.open_loops), (1, 0));
        assert!(cap.triangles.is_empty() && cap.area == 0.0);
    }

    /// Two squares that overlap cross each other: neither is filled.
    #[test]
    fn two_overlapping_loops_are_not_filled_and_are_counted() {
        let lines = vec![square(0.0, 0.0, 4.0), square(2.0, 2.0, 4.0)];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert_eq!(cap.failed_loops, 2);
        assert!(cap.triangles.is_empty() && cap.area == 0.0);
    }

    /// A crossing loop beside a good one: the good one is filled, the crossing one is counted.
    #[test]
    fn a_crossing_loop_does_not_stop_its_neighbours_being_filled() {
        let lines = vec![square(20.0, 20.0, 4.0), loop_of(&[[0.0, 0.0], [4.0, 4.0], [4.0, 0.0], [0.0, 2.0]])];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert_eq!(cap.failed_loops, 1);
        assert!((cap.area - 16.0).abs() < 1e-9, "area {}", cap.area);
    }

    /// An edge only 1e-9 mm from horizontal is a real side: the square keeps its area.
    #[test]
    fn a_nearly_horizontal_edge_keeps_the_loop_filled() {
        let lines = vec![loop_of(&[[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0 + 1e-9]])];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert_eq!(cap.failed_loops, 0);
        assert!((cap.area - 100.0).abs() < 1e-6, "area {}", cap.area);
    }

    /// An L-shaped (non-convex) loop: the 10 × 10 square less its 5 × 5 corner.
    #[test]
    fn an_l_shaped_loop_is_filled_by_its_area() {
        let lines = vec![loop_of(&[[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [5.0, 5.0], [5.0, 10.0], [0.0, 10.0]])];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert!((cap.area - 75.0).abs() < 1e-9, "area {}", cap.area);
        assert_eq!(cap.failed_loops, 0);
    }

    /// Four levels of nesting: a square, a hole, an island in the hole and a hole in the island.
    #[test]
    fn a_hole_inside_a_hole_inside_a_square_is_counted_by_parity() {
        let lines = vec![square(0.0, 0.0, 10.0), square(2.0, 2.0, 6.0), square(3.0, 3.0, 4.0), square(4.0, 4.0, 2.0)];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert!((cap.area - (100.0 - 36.0 + 16.0 - 4.0)).abs() < 1e-9, "area {}", cap.area);
    }

    /// An open chain next to closed loops leaves their parity unknown: nothing is filled.
    #[test]
    fn an_open_chain_turns_the_cap_off_for_the_whole_section() {
        let open = Polyline { points: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)], closed: false };
        let lines = vec![square(0.0, 0.0, 10.0), square(2.0, 2.0, 6.0), open];
        let cap = cap_of(&lines, &frame(Vec3::z()));
        assert_eq!((cap.open_loops, cap.failed_loops), (1, 2), "the closed loops are not filled");
        assert!(cap.triangles.is_empty() && cap.area == 0.0);
    }

    /// A section with thousands of loops is swept in time: each band looks only at the sides that
    /// cross it, not at every side (that was quadratic). Here 20,000 small squares, each with its own
    /// two levels, so 40,000 bands.
    #[test]
    fn twenty_thousand_loops_are_filled_within_two_seconds() {
        let lines: Vec<Polyline> = (0..20_000).map(|k| square((k % 50) as f64, k as f64 * 0.4, 0.4)).collect();
        let t0 = std::time::Instant::now();
        let cap = cap_of(&lines, &frame(Vec3::z()));
        let elapsed = t0.elapsed();
        assert!(elapsed < std::time::Duration::from_secs(2), "took {elapsed:?}");
        assert_eq!(cap.failed_loops, 0);
        assert!((cap.area - 20_000.0 * 0.16).abs() < 1e-6, "area {}", cap.area);
        assert!(!cap.over_limit);
    }

    /// A section beyond the triangle budget is not filled, and says so; within it, it is filled.
    #[test]
    fn a_section_beyond_the_triangle_budget_is_not_filled_and_says_so() {
        let circle: Vec<[f64; 2]> = (0..40)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 40.0;
                [10.0 * a.cos(), 10.0 * a.sin()]
            })
            .collect();
        let lines = vec![loop_of(&circle)];
        let small = Budget { max_sides: 1000, max_triangles: 10 };
        let cap = cap_of_with_budget(&lines, &frame(Vec3::z()), small);
        assert!(cap.over_limit && cap.triangles.is_empty() && cap.area == 0.0);
        let roomy = Budget { max_sides: 1000, max_triangles: 1000 };
        let fine = cap_of_with_budget(&lines, &frame(Vec3::z()), roomy);
        assert!(!fine.over_limit && (fine.area - polygon_area(40, 10.0)).abs() < 1e-9);
    }

    /// Too many sides is refused before any work is done.
    #[test]
    fn too_many_sides_is_refused_as_over_the_limit() {
        // a square has two non-horizontal sides: a budget of one side refuses it
        let cap = cap_of_with_budget(
            &[square(0.0, 0.0, 1.0)],
            &frame(Vec3::z()),
            Budget { max_sides: 1, max_triangles: 100 },
        );
        assert!(cap.over_limit && cap.triangles.is_empty());
    }

    /// A band where an odd number of sides crosses is not a closed section: the sweep refuses it
    /// (it cannot happen for closed loops; the check guards the parity of the fill).
    #[test]
    fn an_odd_number_of_sides_across_a_band_is_refused() {
        let side = |x: f64| Side { lo: [x, 0.0], hi: [x, 10.0], ring: 0 };
        let sides = vec![side(0.0), side(5.0), side(9.0)];
        assert!(matches!(sweep(&sides, DEFAULT_BUDGET), Err(Defect::Rings(_))));
    }

    /// Timing of the section and its cap on about a million triangles. Run with
    /// `cargo test -p surfacecraft-geom --profile ci cut_of_a_million -- --ignored --nocapture`.
    #[test]
    #[ignore = "timing on about 1 M triangles"]
    fn cut_of_a_million_triangles_is_timed() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 720, 720);
        let t0 = std::time::Instant::now();
        let c = cut(&mesh, &Point3::origin(), &Vec3::z());
        let elapsed = t0.elapsed();
        eprintln!(
            "{} triangles: section and cap {:.1} ms, {} loops, {} cap triangles, area {:.2}",
            mesh.triangle_count(),
            elapsed.as_secs_f64() * 1e3,
            c.lines.len(),
            c.cap.triangles.len(),
            c.cap.area
        );
        let disc = std::f64::consts::PI * 100.0;
        assert!((c.cap.area - disc).abs() / disc < 0.01);
    }
}
