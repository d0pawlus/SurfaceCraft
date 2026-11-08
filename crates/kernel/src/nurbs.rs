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
