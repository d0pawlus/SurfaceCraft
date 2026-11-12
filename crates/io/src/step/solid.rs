//! Closed solids as MANIFOLD_SOLID_BREPs: one CLOSED_SHELL of ADVANCED_FACEs whose vertices, edge
//! curves and control points are written once and shared by every face that uses them. This is
//! the compact path for bodies of thousands of spline patches; the solidcore exporter spends
//! about 24 KB on each patch, this one a few KB.

use super::p21::{Param, Writer};
use super::topology::ShellWriter;
use super::{bad, finish, r, s, unsupported};
use crate::IoError;
use surfacecraft_kernel::pcurve as kp;
use surfacecraft_kernel::record::brep::{Brep, OrientedFace, Shell, Surface};

/// The outer shell's faces when the body is one solid without voids: a single material region
/// bounded by a single closed face shell.
fn solid_faces(b: &Brep) -> Result<&[OrientedFace], IoError> {
    let mut material = b.regions.iter().skip(1).filter(|reg| !reg.void);
    let (Some(region), None) = (material.next(), material.next()) else {
        return Err(unsupported("STEP solid export of a body that is not exactly one solid region"));
    };
    let [shell] = region.shells.as_slice() else {
        return Err(unsupported("STEP solid export of a solid with inner shells (voids)"));
    };
    match b.shells.get(*shell as usize) {
        Some(Shell::Faces { faces, wires }) if wires.is_empty() && !faces.is_empty() => Ok(faces),
        _ => Err(unsupported("STEP solid export of a body whose shell is not a plain face shell")),
    }
}

/// Can the writer say every face of the shell: a plane, a (rational) spline, or an analytic
/// surface whose edges are isoparametric?
fn check_faces(b: &Brep, faces: &[OrientedFace]) -> Result<(), IoError> {
    for of in faces {
        let face = b.faces.get(of.face as usize).ok_or_else(|| bad(format!("face {} missing", of.face)))?;
        let supported = if kp::is_analytic(&face.surface) {
            kp::analytic_face(face, &b.edges).is_ok()
        } else {
            face.nurbs.is_none()
                && matches!(
                    face.surface,
                    Surface::Plane { .. } | Surface::Spline { .. } | Surface::RationalSpline { .. }
                )
        };
        if !supported {
            return Err(unsupported(format!("STEP solid export of face {}'s surface", of.face)));
        }
    }
    Ok(())
}

/// The MANIFOLD_SOLID_BREP of one solid body.
fn write_solid(w: Writer, b: &Brep, index: usize) -> Result<(Writer, Param), IoError> {
    let faces = solid_faces(b)?;
    check_faces(b, faces)?;
    let mut sw = ShellWriter::new(w, b);
    let mut entries = Vec::with_capacity(faces.len());
    for of in faces {
        entries.push(r(sw.face(of.face, of.forward)?));
    }
    let shell = sw.w.add_distinct("CLOSED_SHELL", &[s(), Param::List(entries)]);
    let solid = sw.w.add_distinct("MANIFOLD_SOLID_BREP", &[Param::Str(format!("solid {}", index + 1)), r(shell)]);
    Ok((sw.w, r(solid)))
}

/// Writes solid bodies as one AP214 file: each body's closed shell becomes one MANIFOLD_SOLID_BREP
/// in an ADVANCED_BREP_SHAPE_REPRESENTATION, millimetres. A body that is not one plain solid, or
/// has a face the writer does not support, is [`IoError::Unsupported`] (the caller may use
/// another exporter); a record the file cannot say faithfully (a number that is not finite, an
/// empty interval, knots out of order) is [`IoError::Export`], never a file.
pub fn write_solids(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if bodies.is_empty() {
        return Err(bad("nothing to export"));
    }
    let mut w = Writer::new();
    let mut models = Vec::with_capacity(bodies.len() + 1);
    for (bi, b) in bodies.iter().enumerate() {
        let (next, model) = write_solid(w, b, bi)?;
        w = next;
        models.push(model);
    }
    finish(w, name, models, "ADVANCED_BREP_SHAPE_REPRESENTATION")
}
