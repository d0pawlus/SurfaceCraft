// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/step/src/write/topology.rs (open shells as SHELL_BASED_SURFACE_MODEL / OPEN_SHELL of
// ADVANCED_FACEs bounded by EDGE_LOOPs of ORIENTED_EDGEs, side handling by face use) and
// crates/step/src/write.rs (product structure, units and context).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): works directly on solidcore-record without
// solidcraft-geom; supports planes, cylinders, cones, spheres, tori, B-splines and rational
// B-splines, with lines, circles, ellipses and splines as edges; every edge carries the PCURVE of
// each face that uses it (computed by the kernel crate where the record's parameters are not
// STEP's); own AP214 product structure and units. Split over `geom`, `topology`, `pcurve`.
// Technical debt (design doc §10): the pinned `step-export` writes closed MANIFOLD_SOLID_BREPs
// only (it refuses an open shell and every body kind but a solid), so sheets are written here;
// it also costs ~24 KB per spline patch, so closed solids of spline patches are written here too
// (`solid.rs`, our own code on the same topology writer).

//! STEP export of sheet (open shell) bodies, design doc §10 milestone M3, and of closed solids
//! made of many spline patches (compact, shared topology).

mod geom;
pub mod p21;
mod pcurve;
mod solid;
mod solid_check;
mod topology;

#[cfg(test)]
mod network_fixture;
#[cfg(test)]
mod solid_hardening_tests;
#[cfg(test)]
mod solid_tests;
#[cfg(test)]
mod tests;

use crate::IoError;
use p21::{Param, Writer};
use surfacecraft_kernel::record::brep::{Brep, Shell};
use topology::ShellWriter;

pub use solid::write_solids;

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

/// A record the writer has no entity for (the caller may use another exporter).
fn unsupported(m: impl Into<String>) -> IoError {
    IoError::Unsupported(m.into())
}

/// Is the record a sheet (no bounded material region)?
#[must_use]
pub fn is_sheet(b: &Brep) -> bool {
    b.regions.iter().skip(1).all(|reg| reg.void || reg.shells.is_empty()) && !b.faces.is_empty()
}

/// The OPEN_SHELLs of one body, as one SHELL_BASED_SURFACE_MODEL (`None` for a body without face
/// shells); the writer is handed back with them.
fn write_body(w: Writer, b: &Brep, index: usize) -> Result<(Writer, Option<Param>), IoError> {
    let mut sw = ShellWriter::new(w, b);
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
    let mut w = sw.w;
    if shells.is_empty() {
        return Ok((w, None));
    }
    let model =
        w.add_distinct("SHELL_BASED_SURFACE_MODEL", &[Param::Str(format!("sheet {}", index + 1)), Param::List(shells)]);
    Ok((w, Some(r(model))))
}

/// Millimetres, radians and steradians, the uncertainty, and the context of the representation.
fn units_and_context(w: &mut Writer) -> u64 {
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
    w.add_complex(&[
        ("GEOMETRIC_REPRESENTATION_CONTEXT", vec![Param::Int(3)]),
        ("GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT", vec![Param::List(vec![r(unc)])]),
        ("GLOBAL_UNIT_ASSIGNED_CONTEXT", vec![Param::List(vec![r(len), r(ang), r(sol)])]),
        ("REPRESENTATION_CONTEXT", vec![s(), s()]),
    ])
}

/// The AP214 product structure around the shape representation `rep`.
fn product_structure(w: &mut Writer, name: &str, rep: u64) {
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
}

/// Writes sheet bodies as one AP214 file: each body's face shells become OPEN_SHELLs of one
/// SHELL_BASED_SURFACE_MODEL, all in a MANIFOLD_SURFACE_SHAPE_REPRESENTATION, millimetres. A
/// record the file cannot say faithfully (a number that is not finite, an empty interval, a
/// non-positive radius, knots out of order) is an error, never a file.
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
        let (next, model) = write_body(w, b, bi)?;
        w = next;
        models.extend(model);
    }
    finish(w, name, models, "MANIFOLD_SURFACE_SHAPE_REPRESENTATION")
}

/// The units, the origin placement, the shape representation `rep_kind` of `models` and the product
/// structure, then the file.
fn finish(mut w: Writer, name: &str, mut models: Vec<Param>, rep_kind: &str) -> Result<String, IoError> {
    let ctx = units_and_context(&mut w);
    let o = w.add("CARTESIAN_POINT", &[s(), reals(&[0.0, 0.0, 0.0])]);
    let z = w.add("DIRECTION", &[s(), reals(&[0.0, 0.0, 1.0])]);
    let x = w.add("DIRECTION", &[s(), reals(&[1.0, 0.0, 0.0])]);
    models.push(r(w.add("AXIS2_PLACEMENT_3D", &[s(), r(o), r(z), r(x)])));
    let rep = w.add_distinct(rep_kind, &[Param::Str(name.into()), Param::List(models), r(ctx)]);
    product_structure(&mut w, name, rep);
    w.file(name, "AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }").map_err(bad)
}
