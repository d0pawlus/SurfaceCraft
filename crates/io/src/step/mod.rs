// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/step/src/write/topology.rs (open shells as SHELL_BASED_SURFACE_MODEL / OPEN_SHELL of
// ADVANCED_FACEs bounded by EDGE_LOOPs of ORIENTED_EDGEs, side handling by face use).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): works directly on solidcore-record without
// solidcraft-geom; supports the surfaces and curves SurfaceCraft's sheets use (B-spline,
// rational B-spline, plane; line, B-spline, rational B-spline edges); edges are written with
// their 3-D curve only (no PCURVEs); own AP214 product structure and units.
// Technical debt (design doc §10): to be replaced by solidcore's `step-export` once it writes
// `ShellModel` bodies (upstream PR optional, only with the user's consent).

//! STEP export of sheet (open shell) bodies, design doc §10 milestone M3.

pub mod p21;

use crate::IoError;
use p21::{Param, Writer};
use surfacecraft_kernel::record::brep::{Brep, Curve, Loop, Shell, Surface};

fn s() -> Param {
    Param::Str(String::new())
}

fn r(id: u64) -> Param {
    Param::Ref(id)
}

fn e(name: &str) -> Param {
    Param::Enum(name.into())
}

fn reals(v: &[f64]) -> Param {
    Param::List(v.iter().map(|x| Param::Real(*x)).collect())
}

fn ints(v: &[usize]) -> Param {
    Param::List(v.iter().map(|x| Param::Int(*x as i64)).collect())
}

fn bad(m: impl Into<String>) -> IoError {
    IoError::Export(m.into())
}

/// Distinct knots and their multiplicities.
fn knots_of(full: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut k: Vec<f64> = Vec::new();
    let mut m: Vec<usize> = Vec::new();
    for &x in full {
        match (k.last(), m.last_mut()) {
            (Some(&last), Some(c)) if (x - last).abs() <= 1e-14 * (1.0 + last.abs()) => *c += 1,
            _ => {
                k.push(x);
                m.push(1);
            }
        }
    }
    (k, m)
}

fn degree(knots: &[f64], count: usize) -> Result<usize, IoError> {
    knots
        .len()
        .checked_sub(count + 1)
        .filter(|d| *d >= 1)
        .ok_or_else(|| bad(format!("{} knots for {count} control points", knots.len())))
}

fn finite3(p: &[f64; 3]) -> Result<[f64; 3], IoError> {
    if p.iter().all(|v| v.is_finite()) { Ok(*p) } else { Err(bad("non-finite coordinate")) }
}

struct ShellWriter<'a> {
    w: Writer,
    b: &'a Brep,
    vertices: Vec<Option<u64>>,
    edges: Vec<Option<u64>>,
    surfaces: Vec<Option<u64>>,
}

impl ShellWriter<'_> {
    fn point(&mut self, p: [f64; 3]) -> Result<u64, IoError> {
        let p = finite3(&p)?;
        Ok(self.w.add("CARTESIAN_POINT", &[s(), reals(&p)]))
    }

    fn direction(&mut self, d: [f64; 3]) -> Result<u64, IoError> {
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if !(n > 1e-300) || !n.is_finite() {
            return Err(bad("zero direction"));
        }
        Ok(self.w.add("DIRECTION", &[s(), reals(&[d[0] / n, d[1] / n, d[2] / n])]))
    }

    fn rational_split(h: &[[f64; 4]]) -> Result<(Vec<[f64; 3]>, Vec<f64>), IoError> {
        let mut pts = Vec::with_capacity(h.len());
        let mut ws = Vec::with_capacity(h.len());
        for c in h {
            if !(c[3] > 0.0) || !c[3].is_finite() {
                return Err(bad("rational weight must be positive"));
            }
            pts.push([c[0] / c[3], c[1] / c[3], c[2] / c[3]]);
            ws.push(c[3]);
        }
        Ok((pts, ws))
    }

    fn curve(&mut self, c: &Curve) -> Result<u64, IoError> {
        match c {
            Curve::Line { start, end } => {
                let p = self.point(*start)?;
                let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
                let len = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
                let dir = self.direction(d)?;
                let v = self.w.add("VECTOR", &[s(), r(dir), Param::Real(len)]);
                Ok(self.w.add("LINE", &[s(), r(p), r(v)]))
            }
            Curve::Spline { knots, points } => {
                let deg = degree(knots, points.len())?;
                let ids: Vec<Param> = points.iter().map(|p| self.point(*p).map(r)).collect::<Result<_, _>>()?;
                let (k, m) = knots_of(knots);
                Ok(self.w.add(
                    "B_SPLINE_CURVE_WITH_KNOTS",
                    &[
                        s(),
                        Param::Int(deg as i64),
                        Param::List(ids),
                        e("UNSPECIFIED"),
                        e("F"),
                        e("F"),
                        ints(&m),
                        reals(&k),
                        e("UNSPECIFIED"),
                    ],
                ))
            }
            Curve::RationalSpline { knots, homogeneous } => {
                let deg = degree(knots, homogeneous.len())?;
                let (pts, ws) = Self::rational_split(homogeneous)?;
                let ids: Vec<Param> = pts.iter().map(|p| self.point(*p).map(r)).collect::<Result<_, _>>()?;
                let (k, m) = knots_of(knots);
                Ok(self.w.add_complex(&[
                    ("BOUNDED_CURVE", vec![]),
                    (
                        "B_SPLINE_CURVE",
                        vec![Param::Int(deg as i64), Param::List(ids), e("UNSPECIFIED"), e("F"), e("F")],
                    ),
                    ("B_SPLINE_CURVE_WITH_KNOTS", vec![ints(&m), reals(&k), e("UNSPECIFIED")]),
                    ("CURVE", vec![]),
                    ("GEOMETRIC_REPRESENTATION_ITEM", vec![]),
                    ("RATIONAL_B_SPLINE_CURVE", vec![reals(&ws)]),
                    ("REPRESENTATION_ITEM", vec![s()]),
                ]))
            }
            other => Err(bad(format!("not supported yet: STEP export of a {} edge in a sheet", curve_name(other)))),
        }
    }

    fn surface(&mut self, f: usize) -> Result<u64, IoError> {
        if let Some(Some(id)) = self.surfaces.get(f) {
            return Ok(*id);
        }
        let face = self.b.faces.get(f).ok_or_else(|| bad(format!("face {f} missing")))?;
        if face.nurbs.is_some() {
            return Err(bad(format!("not supported yet: STEP export of face {f} on a NURBS form in a sheet")));
        }
        let id = match &face.surface {
            Surface::Spline { u_knots, v_knots, points } => {
                let nu = points.len();
                let nv = points.first().map_or(0, Vec::len);
                let (p, q) = (degree(u_knots, nu)?, degree(v_knots, nv)?);
                let mut rows = Vec::with_capacity(nu);
                for row in points {
                    if row.len() != nv {
                        return Err(bad("control net is not rectangular"));
                    }
                    rows.push(Param::List(row.iter().map(|c| self.point(*c).map(r)).collect::<Result<_, _>>()?));
                }
                let (uk, um) = knots_of(u_knots);
                let (vk, vm) = knots_of(v_knots);
                self.w.add(
                    "B_SPLINE_SURFACE_WITH_KNOTS",
                    &[
                        s(),
                        Param::Int(p as i64),
                        Param::Int(q as i64),
                        Param::List(rows),
                        e("UNSPECIFIED"),
                        e("F"),
                        e("F"),
                        e("F"),
                        ints(&um),
                        ints(&vm),
                        reals(&uk),
                        reals(&vk),
                        e("UNSPECIFIED"),
                    ],
                )
            }
            Surface::RationalSpline { u_knots, v_knots, homogeneous } => {
                let nu = homogeneous.len();
                let nv = homogeneous.first().map_or(0, Vec::len);
                let (p, q) = (degree(u_knots, nu)?, degree(v_knots, nv)?);
                let mut rows = Vec::with_capacity(nu);
                let mut wrows = Vec::with_capacity(nu);
                for row in homogeneous {
                    if row.len() != nv {
                        return Err(bad("control net is not rectangular"));
                    }
                    let (pts, ws) = Self::rational_split(row)?;
                    rows.push(Param::List(pts.iter().map(|c| self.point(*c).map(r)).collect::<Result<_, _>>()?));
                    wrows.push(reals(&ws));
                }
                let (uk, um) = knots_of(u_knots);
                let (vk, vm) = knots_of(v_knots);
                self.w.add_complex(&[
                    ("BOUNDED_SURFACE", vec![]),
                    (
                        "B_SPLINE_SURFACE",
                        vec![
                            Param::Int(p as i64),
                            Param::Int(q as i64),
                            Param::List(rows),
                            e("UNSPECIFIED"),
                            e("F"),
                            e("F"),
                            e("F"),
                        ],
                    ),
                    (
                        "B_SPLINE_SURFACE_WITH_KNOTS",
                        vec![ints(&um), ints(&vm), reals(&uk), reals(&vk), e("UNSPECIFIED")],
                    ),
                    ("GEOMETRIC_REPRESENTATION_ITEM", vec![]),
                    ("RATIONAL_B_SPLINE_SURFACE", vec![Param::List(wrows)]),
                    ("REPRESENTATION_ITEM", vec![s()]),
                    ("SURFACE", vec![]),
                ])
            }
            Surface::Plane { origin, u_axis, v_axis } => {
                let z = [
                    u_axis[1] * v_axis[2] - u_axis[2] * v_axis[1],
                    u_axis[2] * v_axis[0] - u_axis[0] * v_axis[2],
                    u_axis[0] * v_axis[1] - u_axis[1] * v_axis[0],
                ];
                let o = self.point(*origin)?;
                let zd = self.direction(z)?;
                let xd = self.direction(*u_axis)?;
                let ax = self.w.add("AXIS2_PLACEMENT_3D", &[s(), r(o), r(zd), r(xd)]);
                self.w.add("PLANE", &[s(), r(ax)])
            }
            _ => return Err(bad(format!("not supported yet: STEP export of face {f}'s surface kind in a sheet"))),
        };
        if let Some(slot) = self.surfaces.get_mut(f) {
            *slot = Some(id);
        }
        Ok(id)
    }

    fn vertex(&mut self, v: u32) -> Result<u64, IoError> {
        if let Some(Some(id)) = self.vertices.get(v as usize) {
            return Ok(*id);
        }
        let p = self.b.vertices.get(v as usize).ok_or_else(|| bad(format!("vertex {v} missing")))?.point;
        let pt = self.point(p)?;
        let id = self.w.add_distinct("VERTEX_POINT", &[s(), r(pt)]);
        if let Some(slot) = self.vertices.get_mut(v as usize) {
            *slot = Some(id);
        }
        Ok(id)
    }

    fn edge(&mut self, e_idx: u32) -> Result<u64, IoError> {
        if let Some(Some(id)) = self.edges.get(e_idx as usize) {
            return Ok(*id);
        }
        let edge = self.b.edges.get(e_idx as usize).ok_or_else(|| bad(format!("edge {e_idx} missing")))?;
        let curve = edge.nurbs.as_ref().unwrap_or(&edge.curve).clone();
        let (a, b) = (self.vertex(edge.vertices[0])?, self.vertex(edge.vertices[1])?);
        let g = self.curve(&curve)?;
        let id = self.w.add_distinct("EDGE_CURVE", &[s(), r(a), r(b), r(g), e("T")]);
        if let Some(slot) = self.edges.get_mut(e_idx as usize) {
            *slot = Some(id);
        }
        Ok(id)
    }

    /// The face as a shell uses it; `side` true where its normal is the surface's natural one.
    fn face(&mut self, f: u32, side: bool) -> Result<u64, IoError> {
        let face = self.b.faces.get(f as usize).ok_or_else(|| bad(format!("face {f} missing")))?;
        let loops = face.loops.clone();
        let geometry = self.surface(f as usize)?;
        let mut bounds = Vec::with_capacity(loops.len());
        for (li, l) in loops.iter().enumerate() {
            let lp = match l {
                Loop::Edges { edges } => {
                    let mut oriented = Vec::with_capacity(edges.len());
                    let order: Box<dyn Iterator<Item = _>> =
                        if side { Box::new(edges.iter()) } else { Box::new(edges.iter().rev()) };
                    for u in order {
                        let ec = self.edge(u.edge)?;
                        let along = if u.forward == side { "T" } else { "F" };
                        oriented.push(r(self
                            .w
                            .add_distinct("ORIENTED_EDGE", &[s(), Param::Derived, Param::Derived, r(ec), e(along)])));
                    }
                    self.w.add_distinct("EDGE_LOOP", &[s(), Param::List(oriented)])
                }
                Loop::Vertex { vertex } => {
                    let v = self.vertex(*vertex)?;
                    self.w.add_distinct("VERTEX_LOOP", &[s(), r(v)])
                }
            };
            let kind = if li == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
            bounds.push(r(self.w.add_distinct(kind, &[s(), r(lp), e("T")])));
        }
        Ok(self
            .w
            .add_distinct("ADVANCED_FACE", &[s(), Param::List(bounds), r(geometry), e(if side { "T" } else { "F" })]))
    }
}

fn curve_name(c: &Curve) -> &'static str {
    match c {
        Curve::Line { .. } => "line",
        Curve::Circle { .. } => "circle",
        Curve::Ellipse { .. } => "ellipse",
        Curve::Polyline { .. } => "polyline",
        Curve::Spline { .. } => "spline",
        Curve::RationalSpline { .. } => "rational spline",
        Curve::SurfaceIntersection { .. } => "intersection curve",
    }
}

/// Is the record a sheet (no bounded material region)?
#[must_use]
pub fn is_sheet(b: &Brep) -> bool {
    b.regions.iter().skip(1).all(|reg| reg.void || reg.shells.is_empty()) && !b.faces.is_empty()
}

/// Writes sheet bodies as one AP214 file: each body's face shells become OPEN_SHELLs of one
/// SHELL_BASED_SURFACE_MODEL, all in a MANIFOLD_SURFACE_SHAPE_REPRESENTATION, millimetres.
pub fn write_sheets(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if bodies.is_empty() {
        return Err(bad("nothing to export"));
    }
    let mut w = Writer::new();
    let mut models = Vec::new();
    for (bi, b) in bodies.iter().enumerate() {
        if !is_sheet(b) {
            return Err(bad(format!(
                "body {bi} is a solid; sheets only here (solids are written by the solid exporter)"
            )));
        }
        let mut sw = ShellWriter {
            w,
            b,
            vertices: vec![None; b.vertices.len()],
            edges: vec![None; b.edges.len()],
            surfaces: vec![None; b.faces.len()],
        };
        let mut shells = Vec::new();
        for sh in &b.shells {
            if let Shell::Faces { faces, .. } = sh {
                let mut entries = Vec::with_capacity(faces.len());
                for of in faces {
                    entries.push(r(sw.face(of.face, of.forward)?));
                }
                shells.push(r(sw.w.add_distinct("OPEN_SHELL", &[s(), Param::List(entries)])));
            }
        }
        w = sw.w;
        if !shells.is_empty() {
            models.push(r(w.add_distinct(
                "SHELL_BASED_SURFACE_MODEL",
                &[Param::Str(format!("sheet {}", bi + 1)), Param::List(shells)],
            )));
        }
    }
    // units and context
    let len = w.add_complex(&[
        ("LENGTH_UNIT", vec![]),
        ("NAMED_UNIT", vec![Param::Derived]),
        ("SI_UNIT", vec![e("MILLI"), e("METRE")]),
    ]);
    let ang = w.add_complex(&[
        ("NAMED_UNIT", vec![Param::Derived]),
        ("PLANE_ANGLE_UNIT", vec![]),
        ("SI_UNIT", vec![Param::Unset, e("RADIAN")]),
    ]);
    let sol = w.add_complex(&[
        ("NAMED_UNIT", vec![Param::Derived]),
        ("SI_UNIT", vec![Param::Unset, e("STERADIAN")]),
        ("SOLID_ANGLE_UNIT", vec![]),
    ]);
    let unc = w.add(
        "UNCERTAINTY_MEASURE_WITH_UNIT",
        &[
            Param::Typed("LENGTH_MEASURE".into(), Box::new(Param::Real(1e-6))),
            r(len),
            Param::Str("distance_accuracy_value".into()),
            s(),
        ],
    );
    let ctx = w.add_complex(&[
        ("GEOMETRIC_REPRESENTATION_CONTEXT", vec![Param::Int(3)]),
        ("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT", vec![Param::List(vec![r(unc)])]),
        ("GLOBAL_UNIT_ASSIGNED_CONTEXT", vec![Param::List(vec![r(len), r(ang), r(sol)])]),
        ("REPRESENTATION_CONTEXT", vec![s(), s()]),
    ]);
    let o = w.add("CARTESIAN_POINT", &[s(), reals(&[0.0, 0.0, 0.0])]);
    let z = w.add("DIRECTION", &[s(), reals(&[0.0, 0.0, 1.0])]);
    let x = w.add("DIRECTION", &[s(), reals(&[1.0, 0.0, 0.0])]);
    let axis = w.add("AXIS2_PLACEMENT_3D", &[s(), r(o), r(z), r(x)]);
    let mut items = models;
    items.push(r(axis));
    let rep =
        w.add_distinct("MANIFOLD_SURFACE_SHAPE_REPRESENTATION", &[Param::Str(name.into()), Param::List(items), r(ctx)]);
    // product structure (AP214)
    let app =
        w.add("APPLICATION_CONTEXT", &[Param::Str("core data for automotive mechanical design processes".into())]);
    w.add(
        "APPLICATION_PROTOCOL_DEFINITION",
        &[
            Param::Str("international standard".into()),
            Param::Str("automotive_design".into()),
            Param::Int(2000),
            r(app),
        ],
    );
    let pc = w.add("PRODUCT_CONTEXT", &[s(), r(app), Param::Str("mechanical".into())]);
    let prod =
        w.add_distinct("PRODUCT", &[Param::Str(name.into()), Param::Str(name.into()), s(), Param::List(vec![r(pc)])]);
    let pdf = w.add_distinct("PRODUCT_DEFINITION_FORMATION", &[s(), s(), r(prod)]);
    let pdc = w.add(
        "PRODUCT_DEFINITION_CONTEXT",
        &[Param::Str("part definition".into()), r(app), Param::Str("design".into())],
    );
    let pd = w.add_distinct("PRODUCT_DEFINITION", &[Param::Str("design".into()), s(), r(pdf), r(pdc)]);
    let pds = w.add_distinct("PRODUCT_DEFINITION_SHAPE", &[s(), s(), r(pd)]);
    w.add_distinct("SHAPE_DEFINITION_REPRESENTATION", &[r(pds), r(rep)]);
    Ok(w.file(name, "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_kernel::nurbs::{SurfaceProbe, fit_grid, sheet_from_spline};

    fn sheet() -> Brep {
        let grid: Vec<Vec<[f64; 3]>> = (0..24)
            .map(|i| {
                (0..20)
                    .map(|j| {
                        let (x, y) = (60.0 * f64::from(i) / 23.0, 40.0 * f64::from(j) / 19.0);
                        [x, y, surfacecraft_geom::synth::wave_height(3.0, x, y)]
                    })
                    .collect()
            })
            .collect();
        sheet_from_spline(&fit_grid(&grid, 10, 9, 3).unwrap()).unwrap()
    }

    /// Gate M3: a fitted sheet written as STEP reads back (solidcore's importer) as one face whose
    /// surface matches the original within 1e-6 mm.
    #[test]
    fn gate_m3_sheet_step_roundtrip() {
        let b = sheet();
        let text = write_sheets("wave", &[&b]).unwrap();
        assert!(
            text.contains("SHELL_BASED_SURFACE_MODEL")
                && text.contains("OPEN_SHELL")
                && text.contains("B_SPLINE_SURFACE_WITH_KNOTS")
        );
        let back = surfacecraft_kernel::step::import(text.as_bytes()).unwrap();
        assert_eq!(back.len(), 1);
        assert_eq!(back[0].faces.len(), 1);
        let face = &back[0].faces[0];
        let surf = face.nurbs.as_ref().unwrap_or(&face.surface);
        let a = SurfaceProbe::new(&b.faces[0].surface).unwrap();
        let bprobe = SurfaceProbe::new(surf).unwrap();
        let mut max: f64 = 0.0;
        for i in 0..=8 {
            for j in 0..=8 {
                let p = a.eval(f64::from(i) / 8.0, f64::from(j) / 8.0).unwrap();
                max = max.max(bprobe.closest(p).unwrap().0);
            }
        }
        eprintln!("gate_m3 STEP round-trip: max surface distance {max:e} mm");
        assert!(max < 1e-6, "{max}");
    }

    #[test]
    fn solids_are_refused_and_empty_input_errors() {
        let solid = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 1.0, 1.0]).unwrap();
        assert!(!is_sheet(&solid));
        assert!(write_sheets("x", &[&solid]).is_err());
        assert!(write_sheets("x", &[]).is_err());
        assert_eq!(knots_of(&[0.0, 0.0, 0.5, 1.0, 1.0]), (vec![0.0, 0.5, 1.0], vec![2, 1, 2]));
    }
}
