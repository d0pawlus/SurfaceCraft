//! STEP through solidcore's `solidcore-step`: import of any STEP file into records (the file's
//! topology read as it is, `topology::read_solids_with`) and export of solid and sheet records as
//! one AP242 file (`write::product::bodies`), design doc §10 milestones M3 and M5.
//!
//! The reader builds every body from the vertices, edges and faces the file states; it does not
//! stitch faces by geometry, and it does not use the file's PCURVE entities (each use's pcurve is
//! its closed form on the surface, or the kernel's, [`KernelTrims`]). The writer writes each edge
//! as a SURFACE_CURVE (SEAM_CURVE where a face uses it twice) with the PCURVEs of its uses on
//! planes and splines; a face on a cylinder, cone, sphere or torus is written as that surface with
//! no PCURVEs on this pin (STEP's are in the angle; upstream: tangency-labs/solidcraft#13797).
use crate::record::brep::{Brep, Curve2, Face, Surface};
use crate::{KernelError, Result, op_err};
use solidcore::kernel::{Kernel, KernelOptions};
use solidcore::surface::face_surface::analytic_label;
use solidcore_ops::pcurve::{TrimFace, Trims};
use solidcore_step::write::product::{ExportBody, Header};
use std::borrow::Cow;
use std::cell::RefCell;

mod check;
mod trims;

/// Largest STEP text accepted (bytes).
pub const MAX_STEP_BYTES: usize = 256 * 1024 * 1024;

/// What this application writes in the originating-system and preprocessor fields of a header.
const SYSTEM: &str = "SurfaceCraft";

fn check_size(text: &[u8]) -> Result<()> {
    if text.len() > MAX_STEP_BYTES {
        return Err(KernelError::Argument(format!("STEP file over {MAX_STEP_BYTES} bytes")));
    }
    Ok(())
}

/// The pcurves a face's uses have no closed form for, made by the kernel ([`trims`]); the closed
/// forms the reader found are kept as they are. One kernel serves every face of an import, and the
/// work it may do is bounded ([`trims::MAX_TRIM_USES`]).
pub(crate) struct KernelTrims(RefCell<trims::TrimKernel>);

impl KernelTrims {
    pub(crate) fn new() -> Self {
        Self(RefCell::new(trims::TrimKernel::new()))
    }
}

impl Trims for KernelTrims {
    fn trims(&self, face: &TrimFace<'_>) -> std::result::Result<Vec<Vec<Curve2>>, String> {
        let mut kernel =
            self.0.try_borrow_mut().map_err(|_| "the trims were asked for from inside a trim".to_string())?;
        kernel.trims(face).map_err(|e| e.to_string())
    }
}

/// What a STEP file gave: the bodies read, and why each other body of the file was refused.
#[derive(Clone, Debug, Default)]
pub struct Imported {
    /// The bodies read, each a record that passed the record's checks.
    pub bodies: Vec<Brep>,
    /// One line per body refused: the file entity and the reason.
    pub refused: Vec<String>,
}

/// Every body of a STEP file, as records: each solid and surface model the file states, built
/// from the file's own topology ([`import_report`]). A file of which no body reads is an error
/// naming why.
pub fn import(text: &[u8]) -> Result<Vec<Brep>> {
    import_report(text).map(|i| i.bodies)
}

/// [`import`] with the bodies the file holds that were refused, and why.
///
/// The file is hostile input: the parser bounds its size and entity counts; the control points its
/// faces would hold are estimated before anything is built ([`check::MAX_FACE_POINTS`]); the reader
/// bounds its faces (`solidcore_step::topology`'s caps), refuses a body whose references, loops or
/// geometry do not hold together, and the kernel's trims are bounded too; every record it builds is
/// checked again here ([`check::record`]: an item cap, the record's structural and geometric checks,
/// and faces that agree on their sides). A record that fails is refused with its reason, never
/// returned.
pub fn import_report(text: &[u8]) -> Result<Imported> {
    check_size(text)?;
    // Part 21 is ASCII (other characters are escaped); a stray byte of another code page can only
    // sit in a string, and is replaced there rather than refusing the file.
    let text = String::from_utf8_lossy(text);
    let exchange = solidcore_step::p21::parse(&text).map_err(|e| op_err("STEP parse", e.to_string()))?;
    let points = check::face_points(&exchange);
    if points > check::MAX_FACE_POINTS {
        return Err(op_err(
            "STEP import",
            format!("the file's faces would hold {points} control points, more than {}", check::MAX_FACE_POINTS),
        ));
    }
    let read = solidcore_step::topology::read_solids_with(&exchange, &KernelTrims::new());
    let mut out = Imported::default();
    out.refused.extend(read.failures.iter().map(|f| format!("#{}: {}", f.item, f.message)));
    for solid in read.solids {
        match check::record(&solid.brep) {
            Ok(()) => out.bodies.push(solid.brep),
            Err(e) => out.refused.push(format!("#{}: {e}", solid.item)),
        }
    }
    if out.bodies.is_empty()
        && let Some(first) = out.refused.first()
    {
        return Err(op_err(
            "STEP import",
            format!("no body could be read ({} refused; first, {first})", out.refused.len()),
        ));
    }
    Ok(out)
}

/// Writes solid and sheet records as one STEP file (AP242): each bounded material region as a
/// MANIFOLD_SOLID_BREP (BREP_WITH_VOIDS with voids), the open shells of each body as a
/// SHELL_BASED_SURFACE_MODEL of OPEN_SHELLs, all in millimetres, named `name`.
pub fn export(name: &str, bodies: &[(&str, &Brep)]) -> Result<String> {
    if bodies.is_empty() {
        return Err(KernelError::Argument("nothing to export".into()));
    }
    // the writer writes what the record says; a record that breaks the schema is refused here
    // rather than written as a file that says something else
    for (n, record) in bodies {
        check::record(record)
            .and_then(|()| check::agreement(record))
            .map_err(|e| KernelError::Argument(format!("STEP export of '{n}': {e}")))?;
    }
    let ready: Vec<Cow<'_, Brep>> = bodies.iter().map(|(_, record)| cones_from_their_apex(record)).collect();
    let list = bodies
        .iter()
        .zip(&ready)
        .map(|((n, _), record)| ExportBody {
            name: (*n).to_string(),
            record: record.as_ref(),
            color: None,
            faces: Vec::new(),
        })
        .collect();
    let header = Header {
        file_name: name.to_string(),
        preprocessor_version: SYSTEM.into(),
        originating_system: SYSTEM.into(),
        ..Header::default()
    };
    solidcore_step::write::product::bodies(list, &header).map_err(|e| op_err("STEP export", e.to_string()))
}

/// The cone the kernel proves a rational face without a label to be (as the writer labels such faces
/// itself, `solidcore::surface::face_surface::analytic_label`), when it is a cone.
fn proved_cone(k: &Kernel, f: &Face) -> Option<Surface> {
    if f.nurbs.is_some() || !matches!(f.surface, Surface::RationalSpline { .. }) {
        return None;
    }
    let net = solidcore_convert::surface(&f.surface).ok()?;
    let label = analytic_label(k, &net, f.domain).ok()??;
    let label = solidcore_convert::record_surface(label).ok()?;
    matches!(label, Surface::Cone { .. }).then_some(label)
}

/// The label STEP can hold for a cone label it cannot: a cone of half angle 0 (the kernel's fillet
/// gives a round of a box edge so, `docs/upstream/02-blend-cylinder-as-cone.md`) is the cylinder it
/// is, and a cone whose radius at its frame's origin is negative is moved along its axis to its apex
/// (radius 0): the same surface either way. `None` when the label needs no change.
fn step_label(s: &Surface) -> Option<Surface> {
    let Surface::Cone { frame, radius_at_origin, half_angle } = s else { return None };
    if *half_angle == 0.0 {
        return (*radius_at_origin > 0.0)
            .then(|| Surface::Cylinder { frame: frame.clone(), radius: *radius_at_origin });
    }
    let slope = half_angle.tan();
    if !(*radius_at_origin < 0.0 && slope.is_finite() && slope != 0.0) {
        return None;
    }
    let [x, y] = [frame.x_axis, frame.y_axis];
    let z = [x[1] * y[2] - x[2] * y[1], x[2] * y[0] - x[0] * y[2], x[0] * y[1] - x[1] * y[0]];
    let t = -*radius_at_origin / slope;
    let mut frame = frame.clone();
    frame.origin = std::array::from_fn(|k| frame.origin[k] + t * z[k]);
    Some(Surface::Cone { frame, radius_at_origin: 0.0, half_angle: *half_angle })
}

/// `b` with every cone face STEP cannot hold relabelled ([`step_label`]): the writer writes a
/// cone's radius and half angle as they are, and STEP's cone has neither a radius below 0 nor a
/// half angle of 0, so the reader refuses such a face (`docs/upstream/19-step-cone-labels.md`).
/// This covers the cones the record labels and the rational faces the writer would label as cones;
/// a face with a NURBS form has its parameters on the form, so its label is free to change.
fn cones_from_their_apex(b: &Brep) -> Cow<'_, Brep> {
    // the kernel the writer labels such faces with: the fixes the session runs with, and no others
    // (`session::fixes` names those the application turned on, none by default; this turns none on
    // by itself), so the labels here are the writer's own
    let mut k = Kernel::new(&KernelOptions::default());
    for name in solidcore_ops::session::fixes() {
        if let Some(on) = k.fixes.flag_mut(name) {
            *on = true;
        }
    }
    let mut changed: Vec<(usize, Surface, Option<Surface>)> = Vec::new();
    for (i, f) in b.faces.iter().enumerate() {
        if f.nurbs.is_some()
            && let Some(label) = step_label(&f.surface)
        {
            changed.push((i, label, None));
        } else if let Some(cone) = proved_cone(&k, f)
            && let Some(label) = step_label(&cone)
        {
            changed.push((i, label, Some(f.surface.clone())));
        }
    }
    if changed.is_empty() {
        return Cow::Borrowed(b);
    }
    let mut out = b.clone();
    for (i, label, form) in changed {
        if let Some(f) = out.faces.get_mut(i) {
            f.surface = label;
            if form.is_some() {
                f.nurbs = form;
            }
        }
    }
    Cow::Owned(out)
}

/// [`export`] of solid bodies, the file named after the first body.
pub fn export_solids(bodies: &[(&str, &Brep)]) -> Result<String> {
    let name = bodies.first().map_or("surfacecraft", |(n, _)| *n);
    export(name, bodies)
}

#[cfg(test)]
mod tests;
