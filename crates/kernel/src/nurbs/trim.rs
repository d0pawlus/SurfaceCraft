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
pub const MAX_OUTLINE_CORNERS: usize = 4000;
/// Largest distance (mm) between the chord of a loop segment and the surface it lies on.
const MAX_SAG: f64 = 0.004;
/// Edge tolerance (mm) given to the loop edges: twice the sag the densification allows.
const EDGE_TOLERANCE: f64 = 2.0 * MAX_SAG;
/// Vertex tolerance, mm (the vertices are surface points evaluated at their parameters).
const VERTEX_TOLERANCE: f64 = 1e-6;
/// Edges a loop is cut into (the proven 4-edge shape of the untrimmed sheet).
const EDGES_PER_LOOP: usize = 4;
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

/// Do the open segments `ab` and `cd` cross at a single interior point?
fn segments_cross(a: Uv, b: Uv, c: Uv, d: Uv) -> bool {
    let (d1, d2, d3, d4) = (cross(a, b, c), cross(a, b, d), cross(c, d, a), cross(c, d, b));
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
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

/// No two edges of the outline cross, and every hole lies inside the outer loop and outside the
/// other holes.
fn check_outline(loops: &[Vec<Uv>]) -> Result<()> {
    let segs: Vec<(usize, Uv, Uv)> = loops
        .iter()
        .enumerate()
        .flat_map(|(l, p)| (0..p.len()).map(move |i| (l, p[i], p[(i + 1) % p.len()])))
        .collect();
    for (i, s) in segs.iter().enumerate() {
        if segs.iter().skip(i + 1).any(|t| segments_cross(s.1, s.2, t.1, t.2)) {
            return Err(bad("the outline crosses itself"));
        }
    }
    let Some((outer, holes)) = loops.split_first() else { return Err(bad("an outline needs an outer loop")) };
    for (k, h) in holes.iter().enumerate() {
        let inside_outer = point_in_polygon(h[0], outer);
        let inside_other = holes.iter().enumerate().any(|(j, o)| j != k && point_in_polygon(h[0], o));
        if !inside_outer || inside_other {
            return Err(bad("a hole must lie inside the outer loop and outside the other holes"));
        }
    }
    Ok(())
}

fn mid3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0]
}

fn dist3(a: [f64; 3], b: [f64; 3]) -> f64 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
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
    let m = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
    let sm = probe.eval(m[0], m[1])?;
    if dist3(sm, mid3(sa, sb)) <= MAX_SAG {
        return Ok(());
    }
    if depth == 0 || out.len() > MAX_LOOP_POINTS {
        return Err(bad("the surface bends too sharply under the outline to follow it"));
    }
    subdivide(probe, (a, sa), (m, sm), depth - 1, out)?;
    out.push(m);
    subdivide(probe, (m, sm), (b, sb), depth - 1, out)
}

/// The loop with every segment densified, as `(parameters, surface points)`.
fn densify(probe: &SurfaceProbe, corners: &[Uv]) -> Result<(Vec<Uv>, Vec<[f64; 3]>)> {
    let n = corners.len();
    let xyz: Vec<[f64; 3]> = corners.iter().map(|c| probe.eval(c[0], c[1])).collect::<Result<_>>()?;
    let mut uv = Vec::with_capacity(n);
    for i in 0..n {
        let (j, a, b) = ((i + 1) % n, corners[i], corners[(i + 1) % n]);
        uv.push(a);
        subdivide(probe, (a, xyz[i]), (b, xyz[j]), MAX_SPLIT_DEPTH, &mut uv)?;
    }
    let pts = uv.iter().map(|c| probe.eval(c[0], c[1])).collect::<Result<_>>()?;
    Ok((uv, pts))
}

/// Clamped degree-1 knots for `segments` segments: `0, 0, 1, …, segments, segments`.
fn linear_knots(segments: usize) -> Vec<f64> {
    let mut k = vec![0.0];
    k.extend((0..=segments).map(|i| i as f64));
    k.push(segments as f64);
    k
}

/// The edges of one closed, densified loop and its oriented uses. Vertices and edges are
/// appended to the shared lists.
fn loop_edges(uv: &[Uv], xyz: &[[f64; 3]], vertices: &mut Vec<Vertex>, edges: &mut Vec<Edge>) -> Vec<OrientedEdge> {
    let n = uv.len();
    let k = EDGES_PER_LOOP.min(n);
    let first_vertex = vertices.len() as u32;
    let cuts: Vec<usize> = (0..k).map(|j| j * n / k).collect();
    for &c in &cuts {
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
            tolerance: EDGE_TOLERANCE,
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
    check_outline(&loops)?;
    let probe = SurfaceProbe::new(s)?;
    let (mut vertices, mut edges, mut face_loops) = (Vec::new(), Vec::new(), Vec::new());
    for p in &loops {
        let (uv, xyz) = densify(&probe, p)?;
        face_loops.push(Loop::Edges { edges: loop_edges(&uv, &xyz, &mut vertices, &mut edges) });
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
}
