//! The cap of a plane section: the cut face of a closed mesh, filled where the plane is inside the
//! solid. The section loops of [`section::section`] bound the face by the even-odd rule (a loop
//! inside another one is a hole of it), so the cap is the region that an odd number of loops
//! enclose. The region is cut into trapezoids by horizontal lines through every corner of the
//! loops, and each trapezoid gives two triangles; no hole has to be bridged, and any nesting works.
//! An open chain (the mesh has a hole there) gets no cap and is counted, and so is a closed loop
//! that encloses nothing.
use crate::section::{PlaneFrame, Polyline, section};
use crate::{Mesh, Point3, Vec3};

/// A closed loop whose twice-signed area is at most this (mm²) encloses nothing.
const DEGENERATE_AREA2: f64 = 1e-12;

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
    /// Section chains that do not close: no cap is made for them.
    pub open_loops: usize,
    /// Closed loops that enclose nothing (fewer than three corners, or no area).
    pub failed_loops: usize,
}

/// The section of `mesh` by the plane through `origin` with normal `normal`, and its cap. An empty
/// cut when the normal is zero or not finite.
#[must_use]
pub fn cut(mesh: &Mesh, origin: &Point3, normal: &Vec3) -> Cut {
    let lines = section(mesh, origin, normal);
    let cap = PlaneFrame::new(*origin, *normal).map(|frame| cap_of(&lines, &frame)).unwrap_or_default();
    Cut { lines, cap }
}

/// A non-horizontal side of a loop, `lo` below `hi` (plane coordinates).
#[derive(Clone, Copy, Debug)]
struct Side {
    lo: [f64; 2],
    hi: [f64; 2],
}

impl Side {
    /// The abscissa of the side at height `y` (the side spans it).
    fn x_at(&self, y: f64) -> f64 {
        self.lo[0] + (y - self.lo[1]) * (self.hi[0] - self.lo[0]) / (self.hi[1] - self.lo[1])
    }
}

/// The cap of the section polylines `lines` on the plane `frame`.
#[must_use]
pub fn cap_of(lines: &[Polyline], frame: &PlaneFrame) -> Cap {
    let mut cap = Cap::default();
    let mut sides: Vec<Side> = Vec::new();
    for line in lines {
        if !line.closed {
            cap.open_loops += 1;
            continue;
        }
        let ring: Vec<[f64; 2]> = line.points.iter().map(|p| frame.to_2d(p)).collect();
        let finite = ring.iter().all(|p| p[0].is_finite() && p[1].is_finite());
        if ring.len() < 3 || !finite || signed_area2(&ring).abs() <= DEGENERATE_AREA2 {
            cap.failed_loops += 1;
            continue;
        }
        sides.extend(ring_sides(&ring));
    }
    let mut levels: Vec<f64> = sides.iter().flat_map(|s| [s.lo[1], s.hi[1]]).collect();
    levels.sort_by(f64::total_cmp);
    levels.dedup();
    for band in levels.windows(2) {
        add_band(&sides, band[0], band[1], frame, &mut cap);
    }
    cap
}

/// Twice the signed area of a closed ring.
fn signed_area2(ring: &[[f64; 2]]) -> f64 {
    ring.iter().zip(ring.iter().cycle().skip(1)).map(|(a, b)| a[0] * b[1] - b[0] * a[1]).sum()
}

/// The non-horizontal sides of a closed ring.
fn ring_sides(ring: &[[f64; 2]]) -> Vec<Side> {
    ring.iter()
        .zip(ring.iter().cycle().skip(1))
        .filter(|(a, b)| a[1] != b[1])
        .map(|(a, b)| if a[1] < b[1] { Side { lo: *a, hi: *b } } else { Side { lo: *b, hi: *a } })
        .collect()
}

/// Fills the band between the heights `y0 < y1`, where no loop has a corner in between: the sides
/// that span the band are sorted across it, and every pair of neighbours is the inside of the cap.
fn add_band(sides: &[Side], y0: f64, y1: f64, frame: &PlaneFrame, cap: &mut Cap) {
    let mid = 0.5 * (y0 + y1);
    let mut across: Vec<(f64, &Side)> =
        sides.iter().filter(|s| s.lo[1] <= y0 && s.hi[1] >= y1).map(|s| (s.x_at(mid), s)).collect();
    across.sort_by(|a, b| a.0.total_cmp(&b.0));
    for [(_, left), (_, right)] in across.as_chunks::<2>().0 {
        let quad = [[left.x_at(y0), y0], [right.x_at(y0), y0], [right.x_at(y1), y1], [left.x_at(y1), y1]];
        add_triangle(cap, frame, [quad[0], quad[1], quad[2]]);
        add_triangle(cap, frame, [quad[0], quad[2], quad[3]]);
    }
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
