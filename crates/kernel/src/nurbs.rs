//! NURBS surfaces through solidcore (design doc §6.3): least-squares fit of a regular grid
//! (`srf_fit::lst_sq_approx`), point projection (`surface_closest_point`), and a sheet body
//! built as a record with one spline face (spike §1.6: `ops` has no "sheet from surface", the
//! kernel's `face_on` fails, so the record is assembled here and checked by the kernel).
use crate::record::brep::{
    Brep, Curve, Curve2, Edge, EdgeUseRef, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface, Vertex,
};
use crate::{KernelError, Result, op_err};
use solidcore::kernel::{Kernel, KernelOptions};
use solidcore::math::{Point3 as KPoint, Vec3 as KVec};
use solidcore::nurbs::b_spline_surface::BSplineSurface;
use solidcore::nurbs::crv_fit::p1::ParamMethod;
use solidcore::nurbs::math::{CPoint, NOW};
use solidcore::nurbs::srf_basic::Surface as Nurb;
use solidcore::query::api_queries::surface_closest_point;
use solidcore::surface::surface::{Surface as KSurface, SurfaceKind};

/// Largest control net accepted (per direction).
pub const MAX_CONTROL: usize = 200;

/// Least-squares B-spline surface through a regular grid of points (`grid[i][j]`, i along u),
/// with `ctrl_u × ctrl_v` control points of the given degree. Chord-length parameters.
pub fn fit_grid(grid: &[Vec<[f64; 3]>], ctrl_u: usize, ctrl_v: usize, degree: usize) -> Result<Surface> {
    let rows = grid.len();
    let cols = grid.first().map_or(0, Vec::len);
    if rows < 2 || cols < 2 || grid.iter().any(|r| r.len() != cols) {
        return Err(KernelError::Argument("the grid must be rectangular and at least 2 × 2".into()));
    }
    if grid.iter().flatten().flatten().any(|v| !v.is_finite()) {
        return Err(KernelError::Argument("grid points must be finite".into()));
    }
    let degree = degree.clamp(1, 5);
    if ctrl_u <= degree
        || ctrl_v <= degree
        || ctrl_u > rows
        || ctrl_v > cols
        || ctrl_u > MAX_CONTROL
        || ctrl_v > MAX_CONTROL
    {
        return Err(KernelError::Argument(format!(
            "control net {ctrl_u} × {ctrl_v} must exceed the degree {degree} and not exceed the grid {rows} × {cols}"
        )));
    }
    let pts: Vec<Vec<KVec>> = grid.iter().map(|r| r.iter().map(|p| KVec::new(p[0], p[1], p[2])).collect()).collect();
    let nurb = solidcore::nurbs::srf_fit::lst_sq_approx(
        &pts,
        ctrl_u - 1,
        ctrl_v - 1,
        degree,
        degree,
        ParamMethod::ChordLength,
    )
    .map_err(|e| op_err("srf_fit::lst_sq_approx", format!("{e:?}")))?;
    to_record(&nurb)
}

fn to_record(n: &Nurb) -> Result<Surface> {
    let net = n.cpts();
    let (nn, mm) = (n.n(), n.m());
    let mut points = Vec::with_capacity(nn + 1);
    for i in 0..=nn {
        let row = net.get(i).ok_or_else(|| KernelError::Argument("control net row missing".into()))?;
        let mut r = Vec::with_capacity(mm + 1);
        for j in 0..=mm {
            let c = row.get(j).ok_or_else(|| KernelError::Argument("control point missing".into()))?;
            r.push([c.x, c.y, c.z]);
        }
        points.push(r);
    }
    Ok(Surface::Spline { u_knots: n.u_knots().to_vec(), v_knots: n.v_knots().to_vec(), points })
}

fn spline_parts(s: &Surface) -> Result<(&[f64], &[f64], &Vec<Vec<[f64; 3]>>)> {
    match s {
        Surface::Spline { u_knots, v_knots, points } => {
            let nu = points.len();
            let nv = points.first().map_or(0, Vec::len);
            if nu < 2 || nv < 2 || points.iter().any(|r| r.len() != nv) {
                return Err(KernelError::Argument("spline control net must be rectangular".into()));
            }
            let du =
                u_knots.len().checked_sub(nu + 1).ok_or_else(|| KernelError::Argument("u knots too short".into()))?;
            let dv =
                v_knots.len().checked_sub(nv + 1).ok_or_else(|| KernelError::Argument("v knots too short".into()))?;
            if du == 0 || dv == 0 || points.iter().flatten().flatten().any(|x| !x.is_finite()) {
                return Err(KernelError::Argument("invalid spline surface".into()));
            }
            Ok((u_knots, v_knots, points))
        }
        _ => Err(KernelError::Argument("expected a spline surface".into())),
    }
}

fn to_nurb(s: &Surface) -> Result<Nurb> {
    let (u, v, points) = spline_parts(s)?;
    let p = u.len() - points.len() - 1;
    let q = v.len() - points.first().map_or(0, Vec::len) - 1;
    let net: Vec<Vec<CPoint>> =
        points.iter().map(|r| r.iter().map(|c| CPoint { x: c[0], y: c[1], z: c[2], w: NOW }).collect()).collect();
    Nurb::new(net, p, q, u.to_vec(), v.to_vec()).map_err(|e| op_err("srf_basic::Surface::new", format!("{e:?}")))
}

/// Evaluates and projects onto one spline surface.
pub struct SurfaceProbe {
    kernel: Kernel,
    surface: KSurface,
    nurb: Nurb,
}

impl SurfaceProbe {
    /// Prepares a record spline surface for queries.
    pub fn new(s: &Surface) -> Result<Self> {
        let nurb = to_nurb(s)?;
        let surface = KSurface::new(SurfaceKind::BSpline(BSplineSurface::from_nurb(nurb.clone(), false)));
        Ok(Self { kernel: Kernel::new(&KernelOptions::default()), surface, nurb })
    }

    /// Closest point on the surface: `(distance, point)`.
    pub fn closest(&self, p: [f64; 3]) -> Result<(f64, [f64; 3])> {
        if p.iter().any(|v| !v.is_finite()) {
            return Err(KernelError::Argument("point must be finite".into()));
        }
        let r = surface_closest_point(&self.kernel, &self.surface, KPoint::new(p[0], p[1], p[2]))
            .map_err(|e| op_err("surface_closest_point", format!("{e:?}")))?;
        let q = r.closest.point;
        Ok((r.closest.distance, [q.x, q.y, q.z]))
    }

    /// The point at `(u, v)`.
    pub fn eval(&self, u: f64, v: f64) -> Result<[f64; 3]> {
        let c = self.nurb.eval(u, v, solidcore::nurbs::knots::Side::Left, solidcore::nurbs::knots::Side::Left);
        let c = c.map_err(|e| op_err("nurbs eval", format!("{e:?}")))?;
        Ok([c.x, c.y, c.z])
    }
}

/// `[[u0, u1], [v0, v1]]` of a clamped spline.
pub fn domain(s: &Surface) -> Result<[[f64; 2]; 2]> {
    let (u, v, points) = spline_parts(s)?;
    let p = u.len() - points.len() - 1;
    let q = v.len() - points.first().map_or(0, Vec::len) - 1;
    let g = |k: &[f64], i: usize| k.get(i).copied().ok_or_else(|| KernelError::Argument("knot index".into()));
    Ok([[g(u, p)?, g(u, points.len())?], [g(v, q)?, g(v, points.first().map_or(0, Vec::len))?]])
}

fn line2(a: [f64; 2], b: [f64; 2], t0: f64, t1: f64) -> Curve2 {
    Curve2::Spline { knots: vec![t0, t0, t1, t1], points: vec![a, b] }
}

/// A sheet body: one face on the clamped spline `s`, bounded by its four boundary
/// iso-curves (exact: a clamped B-spline's boundary curves are its outer control rows).
pub fn sheet_from_spline(s: &Surface) -> Result<Brep> {
    let (u, v, points) = spline_parts(s)?;
    let [[u0, u1], [v0, v1]] = domain(s)?;
    let nu = points.len();
    let nv = points.first().map_or(0, Vec::len);
    let row = |j: usize| -> Vec<[f64; 3]> { points.iter().filter_map(|r| r.get(j).copied()).collect() };
    let col = |i: usize| -> Vec<[f64; 3]> { points.get(i).cloned().unwrap_or_default() };
    // edges: bottom v=v0 (u), right u=u1 (v), top v=v1 (u), left u=u0 (v)
    let curves = [
        Curve::Spline { knots: u.to_vec(), points: row(0) },
        Curve::Spline { knots: v.to_vec(), points: col(nu - 1) },
        Curve::Spline { knots: u.to_vec(), points: row(nv - 1) },
        Curve::Spline { knots: v.to_vec(), points: col(0) },
    ];
    let probe = SurfaceProbe::new(s)?;
    let corners = [probe.eval(u0, v0)?, probe.eval(u1, v0)?, probe.eval(u1, v1)?, probe.eval(u0, v1)?];
    let vertices: Vec<Vertex> = corners.iter().map(|c| Vertex { point: *c, tolerance: 1e-7 }).collect();
    // vertex indices [start, end] of each edge on its own parameter
    let ends: [[u32; 2]; 4] = [[0, 1], [1, 2], [3, 2], [0, 3]];
    let intervals = [[u0, u1], [v0, v1], [u0, u1], [v0, v1]];
    // the outer loop runs counter-clockwise in (u, v): bottom →, right ↑, top ←, left ↓
    let forward = [true, true, false, false];
    let pcurves = [
        line2([u0, v0], [u1, v0], u0, u1),
        line2([u1, v0], [u1, v1], v0, v1),
        line2([u0, v1], [u1, v1], u0, u1),
        line2([u0, v0], [u0, v1], v0, v1),
    ];
    let mut edges = Vec::with_capacity(4);
    let mut loop_edges = Vec::with_capacity(4);
    for k in 0..4 {
        edges.push(Edge {
            curve: curves[k].clone(),
            nurbs: None,
            interval: intervals[k],
            vertices: ends[k],
            tolerance: 1e-7,
            uses: vec![EdgeUseRef { face: 0, r#loop: 0, position: k as u32, upper: !forward[k] }],
        });
        loop_edges.push(OrientedEdge { edge: k as u32, forward: forward[k], pcurve: pcurves[k].clone() });
    }
    let brep = Brep {
        version: crate::record::brep::VERSION,
        tolerance: 1e-6,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: vec![OrientedFace { face: 0, forward: true }], wires: vec![] }],
        faces: vec![Face {
            surface: s.clone(),
            nurbs: None,
            domain: [[u0, u1], [v0, v1]],
            loops: vec![Loop::Edges { edges: loop_edges }],
        }],
        edges,
        vertices,
    };
    brep.validate().map_err(|e| op_err("record validate", e))?;
    // the kernel must accept it as a body
    solidcore_ops::Body::from_record(&brep, false).map_err(|e| op_err("from_record", e))?;
    Ok(brep)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn wave_grid(n: usize) -> Vec<Vec<[f64; 3]>> {
        (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| {
                        let (x, y) = (60.0 * i as f64 / (n - 1) as f64, 40.0 * j as f64 / (n - 1) as f64);
                        [x, y, 3.0 * (x / 12.0).sin() * (y / 9.0).cos()]
                    })
                    .collect()
            })
            .collect()
    }

    #[test]
    fn fit_project_and_sheet() {
        let grid = wave_grid(30);
        let s = fit_grid(&grid, 12, 12, 3).unwrap();
        let probe = SurfaceProbe::new(&s).unwrap();
        let mut max: f64 = 0.0;
        for r in grid.iter().step_by(3) {
            for p in r.iter().step_by(3) {
                max = max.max(probe.closest(*p).unwrap().0);
            }
        }
        assert!(max < 0.01, "grid deviation {max}");
        let sheet = sheet_from_spline(&s).unwrap();
        assert_eq!(sheet.faces.len(), 1);
        assert_eq!(sheet.edges.len(), 4);
        let mesh = crate::tess::tessellate(&sheet, None).unwrap();
        assert!(mesh.triangle_count() > 10);
        assert!(fit_grid(&grid, 2, 12, 3).is_err());
        assert!(fit_grid(&[], 4, 4, 3).is_err());
    }
}

/// Splits a spline surface into its Bézier patches, `patches[a][b]` (a along u), each a
/// record spline on its own span of the original parameters.
pub fn decompose(s: &Surface) -> Result<Vec<Vec<Surface>>> {
    let n = to_nurb(s)?;
    let bez = solidcore::nurbs::srf_tool::p1::decompose_to_bez(&n)
        .map_err(|e| op_err("decompose_to_bez", format!("{e:?}")))?;
    bez.iter().map(|row| row.iter().map(to_record).collect::<Result<Vec<_>>>()).collect()
}

/// A sheet of `patches[a][b]` (a along u, b along v) that meet edge to edge, as one shell:
/// neighbouring faces share their edge, so the patch network is one connected sheet.
pub fn sheet_from_patches(patches: &[Vec<Surface>]) -> Result<Brep> {
    let na = patches.len();
    let nb = patches.first().map_or(0, Vec::len);
    if na == 0 || nb == 0 || patches.iter().any(|r| r.len() != nb) {
        return Err(KernelError::Argument("the patch grid must be rectangular and non-empty".into()));
    }
    let get = |a: usize, b: usize| -> Result<&Surface> {
        patches.get(a).and_then(|r| r.get(b)).ok_or_else(|| KernelError::Argument("patch index".into()))
    };
    let mut doms = vec![vec![[[0.0; 2]; 2]; nb]; na];
    for a in 0..na {
        for b in 0..nb {
            doms[a][b] = domain(get(a, b)?)?;
        }
    }
    // vertices (na+1) × (nb+1)
    let vid = |a: usize, b: usize| (a * (nb + 1) + b) as u32;
    let mut vertices = Vec::with_capacity((na + 1) * (nb + 1));
    for a in 0..=na {
        for b in 0..=nb {
            let (pa, pb) = (a.min(na - 1), b.min(nb - 1));
            let d = doms[pa][pb];
            let u = if a == na { d[0][1] } else { d[0][0] };
            let v = if b == nb { d[1][1] } else { d[1][0] };
            let p = SurfaceProbe::new(get(pa, pb)?)?.eval(u, v)?;
            vertices.push(Vertex { point: p, tolerance: 1e-6 });
        }
    }
    let rows = |s: &Surface, last: bool| -> Result<(Vec<f64>, Vec<[f64; 3]>)> {
        let (u, _, pts) = spline_parts(s)?;
        let j = if last { pts.first().map_or(0, Vec::len) - 1 } else { 0 };
        Ok((u.to_vec(), pts.iter().filter_map(|r| r.get(j).copied()).collect()))
    };
    let cols = |s: &Surface, last: bool| -> Result<(Vec<f64>, Vec<[f64; 3]>)> {
        let (_, v, pts) = spline_parts(s)?;
        let i = if last { pts.len() - 1 } else { 0 };
        Ok((v.to_vec(), pts.get(i).cloned().unwrap_or_default()))
    };
    // edges along u: H[a][b], a < na, b ≤ nb; along v: V[a][b], a ≤ na, b < nb
    let mut edges: Vec<Edge> = Vec::new();
    let mut h_id = vec![vec![0u32; nb + 1]; na];
    for a in 0..na {
        for b in 0..=nb {
            let (k, pts) = if b < nb { rows(get(a, b)?, false)? } else { rows(get(a, nb - 1)?, true)? };
            let d = doms[a][b.min(nb - 1)];
            h_id[a][b] = edges.len() as u32;
            edges.push(Edge {
                curve: Curve::Spline { knots: k, points: pts },
                nurbs: None,
                interval: d[0],
                vertices: [vid(a, b), vid(a + 1, b)],
                tolerance: 1e-6,
                uses: Vec::new(),
            });
        }
    }
    let mut v_id = vec![vec![0u32; nb]; na + 1];
    for a in 0..=na {
        for b in 0..nb {
            let (k, pts) = if a < na { cols(get(a, b)?, false)? } else { cols(get(na - 1, b)?, true)? };
            let d = doms[a.min(na - 1)][b];
            v_id[a][b] = edges.len() as u32;
            edges.push(Edge {
                curve: Curve::Spline { knots: k, points: pts },
                nurbs: None,
                interval: d[1],
                vertices: [vid(a, b), vid(a, b + 1)],
                tolerance: 1e-6,
                uses: Vec::new(),
            });
        }
    }
    let mut faces = Vec::with_capacity(na * nb);
    let mut oriented = Vec::with_capacity(na * nb);
    for a in 0..na {
        for b in 0..nb {
            let f = faces.len() as u32;
            let [[u0, u1], [v0, v1]] = doms[a][b];
            let uses = [
                (h_id[a][b], true, line2([u0, v0], [u1, v0], u0, u1)),
                (v_id[a + 1][b], true, line2([u1, v0], [u1, v1], v0, v1)),
                (h_id[a][b + 1], false, line2([u0, v1], [u1, v1], u0, u1)),
                (v_id[a][b], false, line2([u0, v0], [u0, v1], v0, v1)),
            ];
            let mut le = Vec::with_capacity(4);
            for (pos, (e, fw, pc)) in uses.into_iter().enumerate() {
                if let Some(edge) = edges.get_mut(e as usize) {
                    edge.uses.push(EdgeUseRef { face: f, r#loop: 0, position: pos as u32, upper: !fw });
                }
                le.push(OrientedEdge { edge: e, forward: fw, pcurve: pc });
            }
            faces.push(Face {
                surface: get(a, b)?.clone(),
                nurbs: None,
                domain: doms[a][b],
                loops: vec![Loop::Edges { edges: le }],
            });
            oriented.push(OrientedFace { face: f, forward: true });
        }
    }
    let brep = Brep {
        version: crate::record::brep::VERSION,
        tolerance: 1e-6,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: oriented, wires: vec![] }],
        faces,
        edges,
        vertices,
    };
    let brep = canonicalize(brep);
    brep.validate().map_err(|e| op_err("record validate", e))?;
    let (body, _) = solidcore_ops::Body::from_record(&brep, false).map_err(|e| op_err("from_record", e))?;
    body.to_record().map(|(r, _)| r).map_err(|e| op_err("to_record", e))
}

/// Sews faces of a sheet whose edges meet within `tolerance` (solidcore `stitch`). Returns the
/// sheet and whether everything was sewn.
pub fn stitch(b: &Brep, tolerance: f64) -> Result<(Brep, bool)> {
    let body = crate::solid::decode(b)?;
    let (out, all) = solidcore_ops::stitch::stitch(body, tolerance).map_err(|e| op_err("stitch", e))?;
    Ok((crate::solid::encode(&out)?, all))
}

#[cfg(test)]
mod patch_tests {
    use super::*;

    #[test]
    fn bezier_patch_network_and_thicken() {
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
        let s = fit_grid(&grid, 7, 6, 3).unwrap();
        let patches = decompose(&s).unwrap();
        assert_eq!((patches.len(), patches[0].len()), (4, 3));
        let sheet = sheet_from_patches(&patches).unwrap();
        assert_eq!(sheet.faces.len(), 12);
        // interior edges are shared by two faces
        let shared = sheet.edges.iter().filter(|e| e.uses.len() == 2).count();
        assert_eq!(shared, 3 * 3 + 4 * 2);
        // the multi-face network does not thicken in solidcore yet (offset units partial); the
        // patches come from one spline, so its single-face sheet is the same geometry
        assert!(crate::solid::thicken(&sheet, 2.0, false).is_err());
        let solid = crate::solid::thicken(&sheet_from_spline(&s).unwrap(), 2.0, false).unwrap();
        let v = crate::solid::volume(&solid).unwrap();
        // ≈ area × thickness (the wave adds a few percent of area)
        assert!(v > 60.0 * 40.0 * 2.0 * 0.99 && v < 60.0 * 40.0 * 2.0 * 1.2, "volume {v}");
    }
}

/// Renumbers edges and vertices into the record's canonical order (first appearance walking
/// faces → loops → oriented edges, then edges' start/end vertices) and rebuilds the uses in
/// walk order. Faces must all be used forward.
fn canonicalize(mut b: Brep) -> Brep {
    let ne = b.edges.len();
    let mut new_of_edge = vec![u32::MAX; ne];
    let mut order: Vec<usize> = Vec::with_capacity(ne);
    let mut uses: Vec<Vec<EdgeUseRef>> = vec![Vec::new(); ne];
    for (fi, f) in b.faces.iter().enumerate() {
        for (li, l) in f.loops.iter().enumerate() {
            if let Loop::Edges { edges } = l {
                for (p, u) in edges.iter().enumerate() {
                    let e = u.edge as usize;
                    if let Some(slot) = new_of_edge.get_mut(e)
                        && *slot == u32::MAX
                    {
                        *slot = order.len() as u32;
                        order.push(e);
                    }
                    if let Some(list) = uses.get_mut(e) {
                        list.push(EdgeUseRef {
                            face: fi as u32,
                            r#loop: li as u32,
                            position: p as u32,
                            upper: !u.forward,
                        });
                    }
                }
            }
        }
    }
    let old_edges = std::mem::take(&mut b.edges);
    let mut edges: Vec<Edge> = order
        .iter()
        .filter_map(|&e| {
            old_edges.get(e).map(|ed| {
                let mut ed = ed.clone();
                ed.uses = uses.get(e).cloned().unwrap_or_default();
                ed
            })
        })
        .collect();
    for f in &mut b.faces {
        for l in &mut f.loops {
            if let Loop::Edges { edges: le } = l {
                for u in le {
                    u.edge = new_of_edge.get(u.edge as usize).copied().unwrap_or(u.edge);
                }
            }
        }
    }
    let nv = b.vertices.len();
    let mut new_of_vertex = vec![u32::MAX; nv];
    let mut vorder: Vec<usize> = Vec::with_capacity(nv);
    for e in &edges {
        for &v in &e.vertices {
            if let Some(slot) = new_of_vertex.get_mut(v as usize)
                && *slot == u32::MAX
            {
                *slot = vorder.len() as u32;
                vorder.push(v as usize);
            }
        }
    }
    for e in &mut edges {
        for v in &mut e.vertices {
            *v = new_of_vertex.get(*v as usize).copied().unwrap_or(*v);
        }
    }
    b.vertices = vorder.iter().filter_map(|&v| b.vertices.get(v).cloned()).collect();
    b.edges = edges;
    b
}
