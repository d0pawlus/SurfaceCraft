//! How much scan surface lies in the void of a tool. A tool is only sound when the part has no
//! surface in it; the surface is measured as area (the area each scan vertex stands for), so the
//! decision does not depend on how densely the part was scanned, and relative to the footprint of
//! the tool on the part.
use super::profile::{Frame, clip_to_rect, inside_polygon, polygon_distance, signed_area};
use super::scan::{Scale, Scan};
use surfacecraft_geom::Point3;

/// Largest share of a tool's footprint that scan surface in its void may cover (noise and
/// floaters, not geometry).
pub(super) const VOID_SHARE: f64 = 0.02;
/// A scan vertex this far inside the outline of a tool (mm at the reference spacing) is in its void.
const VOID_DEPTH: f64 = 0.5;

/// What is measured: the scan against the void above a floor inside an outline.
pub(super) struct Void<'a> {
    pub(super) scan: &'a Scan,
    pub(super) frame: &'a Frame,
    pub(super) scale: Scale,
    /// Height above the frame's origin plane from which scan counts; `None`: any height (a tool
    /// that runs through the part).
    pub(super) above: Option<f64>,
    pub(super) outline: &'a [[f64; 2]],
    /// Outlines of material that stands in the tool (islands).
    pub(super) islands: &'a [&'a [[f64; 2]]],
    /// Width of the band along the floor and the walls that is not counted (fillets nobody
    /// modelled); 0 for none.
    pub(super) band: f64,
}

/// Scan area in the void, and the footprint of the tool on the part, both in mm².
pub(super) struct Measure {
    pub(super) area: f64,
    pub(super) footprint: f64,
}

impl Measure {
    /// The scan area as a share of the footprint.
    pub(super) fn share(&self) -> f64 {
        if self.area <= 0.0 {
            0.0
        } else if self.footprint > 0.0 {
            self.area / self.footprint
        } else {
            f64::INFINITY
        }
    }
}

/// The scan's box in the frame: 2-D corners and heights.
fn scan_box(scan: &Scan, frame: &Frame) -> Option<([f64; 2], [f64; 2], f64, f64)> {
    let (lo, hi) = scan.bounds()?;
    let mut b = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2], f64::INFINITY, f64::NEG_INFINITY);
    for k in 0..8 {
        let c = Point3::new(
            if k & 1 == 0 { lo.x } else { hi.x },
            if k & 2 == 0 { lo.y } else { hi.y },
            if k & 4 == 0 { lo.z } else { hi.z },
        );
        let (q, h) = (frame.to2(&c), (c.coords - frame.o).dot(&frame.d));
        b = ([b.0[0].min(q[0]), b.0[1].min(q[1])], [b.1[0].max(q[0]), b.1[1].max(q[1])], b.2.min(h), b.3.max(h));
    }
    Some(b)
}

/// The scan area in the void of the tool and the footprint of the tool. `skip` names the vertices
/// that are not the part's surface in the void (the tool's own walls, a boss standing on the floor).
pub(super) fn measure(v: &Void, skip: impl Fn(usize) -> bool) -> Measure {
    let Some((lo, hi, hmin, hmax)) = scan_box(v.scan, v.frame) else { return Measure { area: 0.0, footprint: 0.0 } };
    let clipped = clip_to_rect(v.outline, lo, hi);
    let island_area: f64 = v.islands.iter().map(|i| signed_area(i).abs()).sum();
    let footprint = (signed_area(&clipped).abs() - island_area).max(0.0);
    let (blo, bhi) = clipped.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(l, h), q| {
        ([l[0].min(q[0]), l[1].min(q[1])], [h[0].max(q[0]), h[1].max(q[1])])
    });
    if blo.iter().chain(bhi.iter()).any(|c| !c.is_finite()) {
        return Measure { area: 0.0, footprint };
    }
    // the vertices near the box of the tool on the part
    let mid = [0.5 * (blo[0] + bhi[0]), 0.5 * (blo[1] + bhi[1])];
    let centre = v.frame.to3(mid) + v.frame.d * (0.5 * (hmin + hmax));
    let radius = 0.5 * ((bhi[0] - blo[0]).hypot(bhi[1] - blo[1]).hypot(hmax - hmin));
    let depth = v.scale.mm(VOID_DEPTH);
    let mut area = 0.0;
    for i in v.scan.within(&Point3::from(centre), radius + depth) {
        let Some(p) = v.scan.points.get(i) else { continue };
        let h = (p.coords - v.frame.o).dot(&v.frame.d);
        if v.above.is_some_and(|a| h <= a) {
            continue;
        }
        let q = v.frame.to2(p);
        let d_out = polygon_distance(v.outline, q);
        if !inside_polygon(v.outline, q) || d_out <= depth {
            continue;
        }
        let near_island = |poly: &&[[f64; 2]]| inside_polygon(poly, q) || polygon_distance(poly, q) <= depth;
        let in_band = v.band > 0.0 && v.above.is_some_and(|a| h - a <= v.band) && d_out <= v.band;
        if in_band || v.islands.iter().any(near_island) || skip(i) {
            continue;
        }
        area += v.scan.areas.get(i).copied().unwrap_or(0.0);
    }
    Measure { area, footprint }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::Vec3;

    /// A scan of a plate 60 × 40 (z = 0) and a block on it, as a regular grid of `spacing`: the plate
    /// (area weights) and, if `block`, a top face at z = 10 over x 10..30, y 10..30.
    fn scan(spacing: f64, block: bool, floaters: usize) -> Scan {
        let mut pts = Vec::new();
        let n = |len: f64| (len / spacing).round() as usize;
        for i in 0..=n(60.0) {
            for j in 0..=n(40.0) {
                pts.push(Point3::new(i as f64 * 60.0 / n(60.0) as f64, j as f64 * 40.0 / n(40.0) as f64, 0.0));
            }
        }
        if block {
            for i in 0..=n(20.0) {
                for j in 0..=n(20.0) {
                    pts.push(Point3::new(
                        10.0 + i as f64 * 20.0 / n(20.0) as f64,
                        10.0 + j as f64 * 20.0 / n(20.0) as f64,
                        10.0,
                    ));
                }
            }
        }
        let mut areas = vec![0.866 * spacing * spacing; pts.len()];
        // floaters are specks: a tiny triangle each
        for k in 0..floaters {
            pts.push(Point3::new(12.0 + 3.0 * k as f64, 15.0, 6.0));
            areas.push(0.05);
        }
        Scan::from_points_with_areas(pts, areas, spacing)
    }

    fn share(scan: &Scan) -> f64 {
        let frame = Frame::new(Vec3::zeros(), Vec3::z());
        let corner = |x: f64, y: f64| frame.to2(&Point3::new(x, y, 0.0));
        let outline = [corner(8.0, 8.0), corner(32.0, 8.0), corner(32.0, 32.0), corner(8.0, 32.0)];
        let v = Void {
            scan,
            frame: &frame,
            scale: scan.scale(),
            above: Some(0.6),
            outline: &outline,
            islands: &[],
            band: 0.0,
        };
        let m = measure(&v, |_| false);
        m.share()
    }

    #[test]
    fn the_decision_does_not_depend_on_the_scan_density() {
        for spacing in [0.35, 0.7, 2.0] {
            let empty = share(&scan(spacing, false, 0));
            let solid = share(&scan(spacing, true, 0));
            assert!(empty <= VOID_SHARE, "void at {spacing}: {empty}");
            assert!((0.5..=1.5).contains(&solid), "material at {spacing}: share {solid}");
        }
    }

    #[test]
    fn a_few_floaters_above_a_void_do_not_reject_it() {
        for spacing in [0.35, 2.0] {
            let s = share(&scan(spacing, false, 4));
            assert!(s <= VOID_SHARE, "floaters at {spacing}: {s}");
        }
    }
}
