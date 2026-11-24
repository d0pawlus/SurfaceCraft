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
/// forms the reader found are kept as they are.
pub(crate) struct KernelTrims;

impl Trims for KernelTrims {
    fn trims(&self, face: &TrimFace<'_>) -> std::result::Result<Vec<Vec<Curve2>>, String> {
        trims::kernel_trims(face).map_err(|e| e.to_string())
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
/// The file is hostile input: the parser bounds its size and entity counts, the reader its faces
/// (`solidcore_step::topology`'s caps) and refuses a body whose references, loops or geometry do
/// not hold together, and every record it builds is checked again here (`Brep::validate_built`:
/// every structural check of the record; the item budget of `Brep::validate` is left to the
/// reader's caps, since it refuses real closed shells of tens of thousands of patches,
/// `docs/upstream/08-validate-item-budget-large-bspline-shell.md`). A record that fails is refused
/// with its reason, never returned.
pub fn import_report(text: &[u8]) -> Result<Imported> {
    check_size(text)?;
    // Part 21 is ASCII (other characters are escaped); a stray byte of another code page can only
    // sit in a string, and is replaced there rather than refusing the file.
    let text = String::from_utf8_lossy(text);
    let exchange = solidcore_step::p21::parse(&text).map_err(|e| op_err("STEP parse", e.to_string()))?;
    let read = solidcore_step::topology::read_solids_with(&exchange, &KernelTrims);
    let mut out = Imported::default();
    out.refused.extend(read.failures.iter().map(|f| format!("#{}: {}", f.item, f.message)));
    for solid in read.solids {
        match solid.brep.validate_built() {
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
        record.validate_built().map_err(|e| KernelError::Argument(format!("STEP export of '{n}': {e}")))?;
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
mod tests {
    use super::*;

    #[test]
    fn a_box_round_trips_with_its_volume() {
        let b = crate::solid::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap();
        let text = export_solids(&[("box", &b)]).unwrap();
        assert!(text.contains("MANIFOLD_SOLID_BREP") && text.contains("'SurfaceCraft'"));
        let back = import(text.as_bytes()).unwrap();
        assert_eq!(back.len(), 1);
        assert_eq!(back[0].faces.len(), 6);
        let v = crate::solid::volume(&back[0]).unwrap();
        assert!((v - 24.0).abs() < 1e-6, "{v}");
    }

    /// A cone narrowing along its axis and a torus, as the kernel makes them, round trip as one
    /// body each with their volume (the narrowing cone split into several bodies on the old pin).
    #[test]
    fn a_narrowing_cone_and_a_torus_round_trip_with_their_volume() {
        let cone = solidcore_ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).unwrap();
        let torus = solidcore_ops::make_torus([1.0, 2.0, 3.0], 6.0, 1.5).unwrap();
        for (name, body) in [("cone", cone), ("torus", torus)] {
            let rec = crate::solid::encode(&body).unwrap();
            let text = export_solids(&[(name, &rec)]).unwrap();
            let back = import(text.as_bytes()).unwrap();
            assert_eq!(back.len(), 1, "{name}");
            let (before, after) = (crate::solid::volume(&rec).unwrap(), crate::solid::volume(&back[0]).unwrap());
            eprintln!("{name}: volume {before} -> {after}");
            assert!((before - after).abs() <= 1e-6 * before, "{name}: {before} vs {after}");
        }
    }

    /// A box with a filleted edge (the kernel labels the round a cone of half angle 0) round trips
    /// with its volume.
    #[test]
    fn a_filleted_box_round_trips_with_its_volume() {
        let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 8.0]).unwrap();
        let rec = crate::solid::fillet(&b, &[0], 2.0).unwrap();
        let text = export_solids(&[("plate", &rec)]).unwrap();
        let back = import(text.as_bytes()).unwrap();
        let (before, after) = (crate::solid::volume(&rec).unwrap(), crate::solid::volume(&back[0]).unwrap());
        assert!((before - after).abs() <= 1e-6 * before, "{before} vs {after}");
    }

    #[test]
    fn a_cone_of_half_angle_0_is_a_cylinder_and_one_past_its_apex_starts_there() {
        let frame = crate::record::brep::Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] };
        let flat = Surface::Cone { frame: frame.clone(), radius_at_origin: 2.0, half_angle: 0.0 };
        assert!(matches!(step_label(&flat), Some(Surface::Cylinder { radius, .. }) if radius == 2.0));
        let past = Surface::Cone { frame, radius_at_origin: -1.0, half_angle: 0.25 };
        let Some(Surface::Cone { frame, radius_at_origin, .. }) = step_label(&past) else { panic!("moved") };
        assert_eq!(radius_at_origin, 0.0);
        assert!((frame.origin[2] - 1.0 / 0.25f64.tan()).abs() < 1e-12);
        let fine = Surface::Cone { frame: frame.clone(), radius_at_origin: 1.0, half_angle: 0.25 };
        assert!(step_label(&fine).is_none());
    }

    fn plate_text() -> String {
        let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 8.0]).unwrap();
        let rec = crate::solid::fillet(&b, &[0], 2.0).unwrap();
        export_solids(&[("plate", &rec)]).unwrap()
    }

    /// Every body a hostile file gives passes the record's checks.
    fn sound(r: &Result<Vec<Brep>>) {
        if let Ok(bodies) = r {
            for b in bodies {
                assert!(b.validate_built().is_ok(), "an invalid record was returned");
            }
        }
    }

    /// A file whose topology does not hold together (an edge's vertex that does not exist, a loop
    /// that names another face's edge) is refused or read as a valid record, never returned broken.
    #[test]
    fn a_file_with_broken_topology_is_refused() {
        let text = plate_text();
        let curve = text.lines().find(|l| l.contains("=EDGE_CURVE(")).unwrap();
        let v = curve.split(',').nth(1).unwrap();
        let dangling = text.replacen(curve, &curve.replacen(v, "#99999999", 1), 1);
        let r = import(dangling.as_bytes());
        assert!(r.is_err(), "a dangling vertex refuses the only body");
        let edges: Vec<&str> = text.lines().filter(|l| l.contains("=ORIENTED_EDGE(")).collect();
        let edge_of = |l: &str| l.rsplit(',').nth(1).unwrap().to_string();
        let (a, b) = (edge_of(edges[0]), edge_of(edges[edges.len() - 1]));
        assert_ne!(a, b);
        let crossed = text.replacen(edges[0], &edges[0].replacen(&a, &b, 1), 1);
        let r = import(crossed.as_bytes());
        sound(&r);
        let report = import_report(crossed.as_bytes());
        if let Ok(i) = &report {
            eprintln!("crossed loop: {} read, refused {:?}", i.bodies.len(), i.refused);
        }
    }

    /// Bytes of a valid file flipped and cut at random: every call returns, and what reads is valid.
    #[test]
    fn hostile_step_bytes_never_panic() {
        let base = plate_text().into_bytes();
        let mut state = 0x9e37_79b9_7f4a_7c15_u64;
        let mut next = move || {
            state ^= state << 13;
            state ^= state >> 7;
            state ^= state << 17;
            state
        };
        for round in 0..200 {
            let mut bytes = base.clone();
            for _ in 0..1 + next() % 8 {
                let i = (next() % bytes.len() as u64) as usize;
                bytes[i] = next() as u8;
            }
            if round % 3 == 0 {
                bytes.truncate((next() % (bytes.len() as u64 + 1)) as usize);
            }
            sound(&import(&bytes));
        }
    }

    #[test]
    fn nothing_to_export_and_garbage_are_errors() {
        assert!(export("x", &[]).is_err());
        assert!(import(b"").is_err());
        assert!(import(b"ISO-10303-21;\nHEADER;\nENDSEC;\nDATA;\n#1=FOO(;\nENDSEC;\nEND-ISO-10303-21;\n").is_err());
        assert!(import(&[0xff, 0xfe, 0x00, 0x80]).is_err());
    }

    #[test]
    fn a_file_over_the_size_limit_is_refused_before_parsing() {
        let big = vec![b' '; MAX_STEP_BYTES + 1];
        assert!(matches!(import(&big), Err(KernelError::Argument(_))));
    }
}
