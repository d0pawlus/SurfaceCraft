//! A sheet face trimmed to an outline (design doc §6.3, P8-H4): one clamped B-spline surface
//! bounded by an outer loop and any number of holes given as closed polygons in the surface's
//! own `(u, v)`. The loops become edges whose pcurves are the polygon itself (degree-1 splines in
//! `(u, v)`) and whose 3-D curves are degree-1 splines through the surface points of the same
//! parameters; the polygon is densified until the chord of every segment stays within
//! [`MAX_SAG`] of the surface, so the pcurve lifts onto its edge within the edge tolerance.
use super::{SurfaceProbe, canonicalize, domain};
use crate::record::brep::{
    Brep, Curve, Curve2, Edge, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface, Vertex,
};
use crate::{KernelError, Result, op_err};

/// Most corners accepted over all loops of one outline (the crossing test is quadratic).
pub const MAX_OUTLINE_CORNERS: usize = 8000;
/// Largest distance (mm) between the chord of a loop segment and the surface it lies on.
const MAX_SAG: f64 = 0.004;
/// The measured sag may exceed [`MAX_SAG`] by this factor (the bisection tests fewer fractions
/// than the final measurement) before the outline is refused; an edge's tolerance is twice its
/// measured sag, at least [`MAX_SAG`].
const SAG_SLACK: f64 = 2.0;
/// Fractions of a segment at which the sag is tested while bisecting, and when it is measured.
const SPLIT_FRACTIONS: [f64; 3] = [0.25, 0.5, 0.75];
const MEASURE_FRACTIONS: [f64; 7] = [0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875];
/// Segments closer than this share of the parameter range are taken to touch.
const TOUCH_EPS: f64 = 1e-9;
/// Vertex tolerance, mm (the vertices are surface points evaluated at their parameters).
const VERTEX_TOLERANCE: f64 = 1e-6;
/// Bisection depth limit of the densification, and the points one loop may reach.
const MAX_SPLIT_DEPTH: u32 = 12;
const MAX_LOOP_POINTS: usize = 60_000;

type Uv = [f64; 2];

fn bad(m: impl Into<String>) -> KernelError {
    KernelError::Argument(m.into())
}

/// Twice the signed area of a closed polygon (positive: counter-clockwise).
fn signed_area2(p: &[Uv]) -> f64 {
    let n = p.len();
    (0..n)
        .map(|i| {
            let (a, b) = (p[i], p[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum()
}

/// Corners clamped into `dom`, consecutive repeats (and a repeated first corner) dropped.
fn clean_loop(raw: &[Uv], dom: [[f64; 2]; 2]) -> Result<Vec<Uv>> {
    if raw.iter().flatten().any(|v| !v.is_finite()) {
        return Err(bad("outline corners must be finite"));
    }
    let eps = 1e-12 * (dom[0][1] - dom[0][0]).max(dom[1][1] - dom[1][0]);
    let mut out: Vec<Uv> = Vec::with_capacity(raw.len());
    for p in raw {
        let q = [p[0].clamp(dom[0][0], dom[0][1]), p[1].clamp(dom[1][0], dom[1][1])];
        if out.last().is_none_or(|l| (l[0] - q[0]).abs() > eps || (l[1] - q[1]).abs() > eps) {
            out.push(q);
        }
    }
    while out.len() > 1
        && out.first().zip(out.last()).is_some_and(|(f, l)| (f[0] - l[0]).abs() <= eps && (f[1] - l[1]).abs() <= eps)
    {
        out.pop();
    }
    if out.len() < 3 {
        return Err(bad("an outline loop needs at least 3 distinct corners"));
    }
    Ok(out)
}

fn cross(o: Uv, a: Uv, b: Uv) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

fn point_segment_distance(p: Uv, a: Uv, b: Uv) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

/// Distance between the closed segments `ab` and `cd` (zero when they cross or touch).
fn segment_gap(a: Uv, b: Uv, c: Uv, d: Uv) -> f64 {
    let (d1, d2, d3, d4) = (cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return 0.0;
    }
    [
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min)
}

fn point_in_polygon(p: Uv, poly: &[Uv]) -> bool {
    let n = poly.len();
    let mut inside = false;
    for i in 0..n {
        let (a, b) = (poly[i], poly[(i + 1) % n]);
        if (a[1] > p[1]) != (b[1] > p[1]) && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0] {
            inside = !inside;
        }
    }
    inside
}

/// Two consecutive segments `s→p` and `s→q` fold back over each other (a spike).
fn folds_back(s: Uv, p: Uv, q: Uv, eps: f64) -> bool {
    let (u, v) = ([p[0] - s[0], p[1] - s[1]], [q[0] - s[0], q[1] - s[1]]);
    let dot = u[0] * v[0] + u[1] * v[1];
    dot > 0.0 && (point_segment_distance(q, s, p) <= eps || point_segment_distance(p, s, q) <= eps)
}

/// A segment of a loop: loop index, position in the loop, ends.
#[derive(Clone, Copy)]
struct Seg {
    lp: usize,
    at: usize,
    a: Uv,
    b: Uv,
}

fn segments_of(loops: &[Vec<Uv>]) -> Vec<Seg> {
    loops
        .iter()
        .enumerate()
        .flat_map(|(lp, p)| (0..p.len()).map(move |at| Seg { lp, at, a: p[at], b: p[(at + 1) % p.len()] }))
        .collect()
}

/// No two edges of the outline cross, touch or overlap (also a hole touching the outer loop at a
/// corner, and a spike folding back over itself), and every hole lies inside the outer loop and
/// outside the other holes. Segments are swept in order of their smallest `u`, so honest outlines
/// cost little; the cap [`MAX_OUTLINE_CORNERS`] bounds the worst case.
fn check_outline(loops: &[Vec<Uv>], eps: f64) -> Result<()> {
    let mut segs = segments_of(loops);
    let lo = |s: &Seg| s.a[0].min(s.b[0]);
    let hi = |s: &Seg| s.a[0].max(s.b[0]);
    segs.sort_by(|x, y| lo(x).total_cmp(&lo(y)));
    for (i, s) in segs.iter().enumerate() {
        for t in segs.iter().skip(i + 1).take_while(|t| lo(t) <= hi(s) + eps) {
            let len = loops.get(s.lp).map_or(0, Vec::len);
            let adjacent = s.lp == t.lp && (s.at + 1) % len == t.at || s.lp == t.lp && (t.at + 1) % len == s.at;
            if adjacent {
                // they meet at a corner by construction; only an overlap is wrong
                let (corner, p, q) = if (s.at + 1) % len == t.at { (s.b, s.a, t.b) } else { (t.b, t.a, s.b) };
                if folds_back(corner, p, q, eps) {
                    return Err(bad("the outline folds back over itself"));
                }
            } else if segment_gap(s.a, s.b, t.a, t.b) <= eps {
                return Err(bad("the outline crosses or touches itself, or a hole touches another loop"));
            }
        }
    }
    let Some((outer, holes)) = loops.split_first() else { return Err(bad("an outline needs an outer loop")) };
    for (k, h) in holes.iter().enumerate() {
        let all_inside = h.iter().all(|&p| point_in_polygon(p, outer));
        let in_other = holes.iter().enumerate().any(|(j, o)| j != k && h.iter().any(|&p| point_in_polygon(p, o)));
        if !all_inside || in_other {
            return Err(bad("a hole must lie inside the outer loop and outside the other holes"));
        }
    }
    Ok(())
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

/// Largest distance, over `fractions` of the segment, between the surface point at the
/// parameters `a + t (b - a)` and the point `sa + t (sb - sa)` of the 3-D edge: how far the
/// degree-1 pcurve, lifted onto the surface, strays from the degree-1 3-D edge.
fn chord_sag(probe: &SurfaceProbe, (a, sa): (Uv, [f64; 3]), (b, sb): (Uv, [f64; 3]), fractions: &[f64]) -> Result<f64> {
    let mut worst: f64 = 0.0;
    for &t in fractions {
        let on_surface = probe.eval(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))?;
        let on_edge = [sa[0] + t * (sb[0] - sa[0]), sa[1] + t * (sb[1] - sa[1]), sa[2] + t * (sb[2] - sa[2])];
        worst = worst.max(dist3(on_surface, on_edge));
    }
    Ok(worst)
}

/// Bisects `a → b` until each piece's chord is within [`MAX_SAG`] of the surface; the points
/// strictly between `a` and `b` are pushed to `out`.
fn subdivide(
    probe: &SurfaceProbe,
    (a, sa): (Uv, [f64; 3]),
    (b, sb): (Uv, [f64; 3]),
    depth: u32,
    out: &mut Vec<Uv>,
) -> Result<()> {
    if chord_sag(probe, (a, sa), (b, sb), &SPLIT_FRACTIONS)? <= MAX_SAG {
        return Ok(());
    }
    if depth == 0 || out.len() > MAX_LOOP_POINTS {
        return Err(bad("the surface bends too sharply under the outline to follow it"));
    }
    let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
    let sm = probe.eval(m[0], m[1])?;
    subdivide(probe, (a, sa), (m, sm), depth - 1, out)?;
    out.push(m);
    subdivide(probe, (m, sm), (b, sb), depth - 1, out)
}

/// A loop with every segment densified.
struct Dense {
    uv: Vec<Uv>,
    xyz: Vec<[f64; 3]>,
    /// Index in `uv` of each corner of the input loop.
    corners: Vec<usize>,
    /// Measured sag of the segment from point `i` to the next (at seven fractions, at least
    /// those the bisection tested and four it did not).
    sag: Vec<f64>,
}

fn densify(probe: &SurfaceProbe, corners: &[Uv]) -> Result<Dense> {
    let n = corners.len();
    let xyz: Vec<[f64; 3]> = corners.iter().map(|c| probe.eval(c[0], c[1])).collect::<Result<_>>()?;
    let mut uv = Vec::with_capacity(n);
    let mut at = Vec::with_capacity(n);
    for i in 0..n {
        let (j, a, b) = ((i + 1) % n, corners[i], corners[(i + 1) % n]);
        at.push(uv.len());
        uv.push(a);
        subdivide(probe, (a, xyz[i]), (b, xyz[j]), MAX_SPLIT_DEPTH, &mut uv)?;
    }
    let pts: Vec<[f64; 3]> = uv.iter().map(|c| probe.eval(c[0], c[1])).collect::<Result<_>>()?;
    let m = uv.len();
    let mut sag = Vec::with_capacity(m);
    for i in 0..m {
        let j = (i + 1) % m;
        let measured = chord_sag(probe, (uv[i], pts[i]), (uv[j], pts[j]), &MEASURE_FRACTIONS)?;
        if measured > SAG_SLACK * MAX_SAG {
            return Err(bad("the surface bends too sharply under the outline to follow it"));
        }
        sag.push(measured);
    }
    Ok(Dense { uv, xyz: pts, corners: at, sag })
}

/// Clamped degree-1 knots for `segments` segments: `0, 0, 1, …, segments, segments`.
fn linear_knots(segments: usize) -> Vec<f64> {
    let mut k = vec![0.0];
    k.extend((0..=segments).map(|i| i as f64));
    k.push(segments as f64);
    k
}

/// Tolerance of the edge over segments `start..end` of a loop whose segment sags are `sag`.
fn edge_tolerance(sag: &[f64], start: usize, end: usize) -> f64 {
    let n = sag.len().max(1);
    let worst = (start..end).filter_map(|i| sag.get(i % n)).fold(0.0f64, |m, s| m.max(*s));
    (2.0 * worst).max(MAX_SAG)
}

/// The edges of one closed, densified loop and its oriented uses. Vertices and edges are
/// appended to the shared lists.
fn loop_edges(d: &Dense, vertices: &mut Vec<Vertex>, edges: &mut Vec<Edge>) -> Vec<OrientedEdge> {
    let (uv, xyz) = (&d.uv, &d.xyz);
    let n = uv.len();
    let cuts = &d.corners;
    let k = cuts.len();
    let first_vertex = vertices.len() as u32;
    for &c in cuts {
        vertices.push(Vertex { point: xyz[c], tolerance: VERTEX_TOLERANCE });
    }
    let mut uses = Vec::with_capacity(k);
    for (j, &start) in cuts.iter().enumerate() {
        let end = cuts.get(j + 1).copied().unwrap_or(n);
        let range = start..=end;
        let (pts, pc): (Vec<[f64; 3]>, Vec<Uv>) = range.map(|i| (xyz[i % n], uv[i % n])).unzip();
        let segments = pts.len() - 1;
        let knots = linear_knots(segments);
        let id = edges.len() as u32;
        edges.push(Edge {
            curve: Curve::Spline { knots: knots.clone(), points: pts },
            nurbs: None,
            interval: [0.0, segments as f64],
            vertices: [first_vertex + j as u32, first_vertex + ((j + 1) % k) as u32],
            tolerance: edge_tolerance(&d.sag, start, end),
            uses: Vec::new(),
        });
        uses.push(OrientedEdge { edge: id, forward: true, pcurve: Curve2::Spline { knots, points: pc } });
    }
    uses
}

/// A sheet body of one face on the clamped spline `s`, trimmed to `outer` with `holes` removed.
///
/// The loops are closed polygons of corners in the surface's `(u, v)` (the first corner is not
/// repeated; either winding is accepted and fixed). Corners outside the surface's parameter
/// rectangle are clamped onto it. Refused: fewer than 3 corners per loop, non-finite values,
/// more than [`MAX_OUTLINE_CORNERS`] corners in all, loops that cross each other or themselves,
/// holes outside the outer loop. The result is checked by the kernel like every sheet.
pub fn trimmed_sheet(s: &Surface, outer: &[Uv], holes: &[Vec<Uv>]) -> Result<Brep> {
    let dom = domain(s)?;
    let total = outer.len() + holes.iter().map(Vec::len).sum::<usize>();
    if total > MAX_OUTLINE_CORNERS {
        return Err(bad(format!("the outline has {total} corners; at most {MAX_OUTLINE_CORNERS}")));
    }
    let mut loops = Vec::with_capacity(1 + holes.len());
    for (i, raw) in std::iter::once(outer).chain(holes.iter().map(Vec::as_slice)).enumerate() {
        let mut p = clean_loop(raw, dom)?;
        let area2 = signed_area2(&p);
        if !(area2.abs() > 0.0) {
            return Err(bad("an outline loop has no area"));
        }
        // seen from the natural normal the outer loop runs counter-clockwise, holes clockwise
        if (area2 > 0.0) != (i == 0) {
            p.reverse();
        }
        loops.push(p);
    }
    let span = (dom[0][1] - dom[0][0]).max(dom[1][1] - dom[1][0]);
    check_outline(&loops, TOUCH_EPS * span)?;
    let probe = SurfaceProbe::new(s)?;
    let (mut vertices, mut edges, mut face_loops) = (Vec::new(), Vec::new(), Vec::new());
    for p in &loops {
        let dense = densify(&probe, p)?;
        face_loops.push(Loop::Edges { edges: loop_edges(&dense, &mut vertices, &mut edges) });
    }
    let brep = Brep {
        version: crate::record::brep::VERSION,
        tolerance: 1e-6,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: vec![OrientedFace { face: 0, forward: true }], wires: vec![] }],
        faces: vec![Face { surface: s.clone(), nurbs: None, domain: dom, loops: face_loops }],
        edges,
        vertices,
    };
    let brep = canonicalize(brep);
    brep.validate().map_err(|e| op_err("record validate", e))?;
    solidcore_ops::Body::from_record(&brep, false).map_err(|e| op_err("from_record", e))?;
    Ok(brep)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::{fit_grid, sheet_from_spline};

    fn wave_surface() -> Surface {
        let grid: Vec<Vec<[f64; 3]>> = (0..30)
            .map(|i| {
                (0..20)
                    .map(|j| {
                        let (x, y) = (60.0 * f64::from(i) / 29.0, 40.0 * f64::from(j) / 19.0);
                        [x, y, 3.0 * (x / 12.0).sin() * (y / 9.0).cos()]
                    })
                    .collect()
            })
            .collect();
        fit_grid(&grid, 12, 10, 3).unwrap()
    }

    /// Outline in `(u, v)` of the unit square `[0, 1]²`: an L-shaped outer loop and a square hole.
    fn l_outline() -> (Vec<Uv>, Vec<Uv>) {
        let outer = vec![[0.05, 0.05], [0.95, 0.05], [0.95, 0.45], [0.5, 0.45], [0.5, 0.95], [0.05, 0.95]];
        let hole = vec![[0.2, 0.2], [0.2, 0.35], [0.35, 0.35], [0.35, 0.2]];
        (outer, hole)
    }

    #[test]
    fn linear_knots_are_clamped() {
        assert_eq!(linear_knots(1), vec![0.0, 0.0, 1.0, 1.0]);
        assert_eq!(linear_knots(3), vec![0.0, 0.0, 1.0, 2.0, 3.0, 3.0]);
    }

    #[test]
    fn trims_to_an_l_shape_with_a_hole() {
        let s = wave_surface();
        let (outer, hole) = l_outline();
        let sheet = trimmed_sheet(&s, &outer, &[hole]).unwrap();
        assert_eq!(sheet.faces.len(), 1);
        assert_eq!(sheet.faces[0].loops.len(), 2);
        sheet.validate().unwrap();
        // the tessellation honours the outline: its area is that of the L minus the hole
        // (parameter rectangle 60 × 40 mm is almost flat: the wave adds a few percent)
        let full = crate::tess::tessellate(&sheet_from_spline(&s).unwrap(), Some(0.01)).unwrap().area();
        let trimmed = crate::tess::tessellate(&sheet, Some(0.01)).unwrap().area();
        let l_area = 0.9 * 0.4 + 0.45 * 0.5; // the outer L as a share of the unit parameter square
        let expected = l_area - 0.15 * 0.15;
        let ratio = trimmed / full;
        assert!((ratio - expected).abs() < 0.03, "trimmed/full {ratio}, expected about {expected}");
        // loops of either winding are accepted
        let (mut o2, h2) = l_outline();
        o2.reverse();
        let again = trimmed_sheet(&s, &o2, &[h2.into_iter().rev().collect()]).unwrap();
        assert_eq!(again.faces[0].loops.len(), 2);
    }

    #[test]
    fn refuses_bad_outlines() {
        let s = wave_surface();
        let (outer, hole) = l_outline();
        assert!(trimmed_sheet(&s, &outer[..2], &[]).is_err());
        assert!(trimmed_sheet(&s, &[[0.0, 0.0], [f64::NAN, 0.5], [1.0, 1.0]], &[]).is_err());
        // a bow tie crosses itself
        assert!(trimmed_sheet(&s, &[[0.1, 0.1], [0.9, 0.9], [0.9, 0.1], [0.1, 0.9]], &[]).is_err());
        // a hole outside the outer loop
        let outside = vec![[0.7, 0.7], [0.7, 0.8], [0.8, 0.8], [0.8, 0.7]];
        assert!(trimmed_sheet(&s, &outer, &[outside]).is_err());
        // a hole across the outer boundary
        let across = vec![[0.9, 0.3], [1.0, 0.3], [1.0, 0.6], [0.9, 0.6]];
        assert!(trimmed_sheet(&s, &outer, &[across]).is_err());
        assert!(trimmed_sheet(&s, &outer, &[hole]).is_ok());
    }

    #[test]
    fn refuses_touching_overlapping_and_nested_outlines() {
        let s = wave_surface();
        let (outer, _) = l_outline();
        // a hole sharing a corner with the outer loop, and one with a corner on its edge
        let at_corner = vec![[0.05, 0.05], [0.2, 0.1], [0.1, 0.2]];
        assert!(trimmed_sheet(&s, &outer, &[at_corner]).is_err());
        let on_edge = vec![[0.5, 0.05], [0.6, 0.2], [0.4, 0.2]];
        assert!(trimmed_sheet(&s, &outer, &[on_edge]).is_err());
        // a spike folding back over its own edge
        let spike = vec![[0.1, 0.1], [0.9, 0.1], [0.9, 0.5], [0.9, 0.3], [0.1, 0.9]];
        assert!(trimmed_sheet(&s, &spike, &[]).is_err());
        // a hole with its first corner inside the outer loop and another outside it
        let leaves = vec![[0.2, 0.2], [0.3, 0.2], [0.7, 0.8]];
        assert!(trimmed_sheet(&s, &outer, &[leaves]).is_err());
        // overlapping holes, and a hole inside a hole
        let a = vec![[0.15, 0.15], [0.35, 0.15], [0.35, 0.35], [0.15, 0.35]];
        let b = vec![[0.25, 0.25], [0.45, 0.25], [0.45, 0.4], [0.25, 0.4]];
        assert!(trimmed_sheet(&s, &outer, &[a.clone(), b]).is_err());
        let nested = vec![[0.2, 0.2], [0.3, 0.2], [0.3, 0.3], [0.2, 0.3]];
        assert!(trimmed_sheet(&s, &outer, &[a, nested]).is_err());
        // two corners at one place: the loop touches itself
        let pinched = vec![[0.1, 0.1], [0.9, 0.1], [0.5, 0.5], [0.9, 0.9], [0.1, 0.9], [0.5, 0.5]];
        assert!(trimmed_sheet(&s, &pinched, &[]).is_err());
        // too many corners are refused before the crossing test runs
        let many: Vec<Uv> = (0..MAX_OUTLINE_CORNERS + 1)
            .map(|i| {
                let a = std::f64::consts::TAU * i as f64 / (MAX_OUTLINE_CORNERS + 1) as f64;
                [0.5 + 0.4 * a.cos(), 0.5 + 0.4 * a.sin()]
            })
            .collect();
        assert!(trimmed_sheet(&s, &many, &[]).is_err());
        // a legitimate outline with thousands of corners is fast
        let round: Vec<Uv> = many[..2000]
            .iter()
            .enumerate()
            .map(|(i, _)| {
                let a = std::f64::consts::TAU * i as f64 / 2000.0;
                [0.5 + 0.4 * a.cos(), 0.5 + 0.4 * a.sin()]
            })
            .collect();
        let started = std::time::Instant::now();
        assert!(trimmed_sheet(&s, &round, &[]).is_ok());
        assert!(started.elapsed().as_secs_f64() < 20.0, "{:?}", started.elapsed());
    }

    /// The degree-1 pcurve lifted onto the surface stays within the edge tolerance of the 3-D edge
    /// at fractions the construction did not test.
    #[test]
    fn pcurves_lift_onto_their_edges_within_the_tolerance() {
        let s = wave_surface();
        let (outer, hole) = l_outline();
        let sheet = trimmed_sheet(&s, &outer, &[hole]).unwrap();
        let probe = SurfaceProbe::new(&s).unwrap();
        let at = |pts: &[[f64; 3]], t: f64| {
            let i = (t.floor() as usize).min(pts.len() - 2);
            let f = t - i as f64;
            [0, 1, 2].map(|k| pts[i][k] + f * (pts[i + 1][k] - pts[i][k]))
        };
        let mut worst: f64 = 0.0;
        for l in &sheet.faces[0].loops {
            let Loop::Edges { edges } = l else { continue };
            for u in edges {
                let e = &sheet.edges[u.edge as usize];
                let (Curve::Spline { points: p3, .. }, Curve2::Spline { points: p2, .. }) = (&e.curve, &u.pcurve)
                else {
                    panic!()
                };
                let segs = p3.len() - 1;
                for k in 0..segs * 9 {
                    let t = (k as f64 + 0.37) / 9.0;
                    let uv = {
                        let i = (t.floor() as usize).min(p2.len() - 2);
                        let f = t - i as f64;
                        [p2[i][0] + f * (p2[i + 1][0] - p2[i][0]), p2[i][1] + f * (p2[i + 1][1] - p2[i][1])]
                    };
                    let lifted = probe.eval(uv[0], uv[1]).unwrap();
                    let d = dist3(lifted, at(p3, t));
                    assert!(d <= e.tolerance, "lift {d} over the edge tolerance {}", e.tolerance);
                    worst = worst.max(d);
                }
            }
        }
        eprintln!("trim: pcurve lift off the edge by at most {worst:.5} mm");
    }
}
