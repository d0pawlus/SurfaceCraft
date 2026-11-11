//! STEP through solidcore: import of any STEP file into records (`step_brep::import`, used for
//! round-trip checks), and export of solid bodies (`step-brep::brep_to_model` +
//! `step-export`, design doc §10 milestone M5).
use crate::record::brep::Brep;
use crate::{KernelError, Result, op_err};
use solidcore::kernel::{Kernel, KernelOptions};

/// Largest STEP text accepted (bytes).
pub const MAX_STEP_BYTES: usize = 256 * 1024 * 1024;

fn check_size(text: &[u8]) -> Result<()> {
    if text.len() > MAX_STEP_BYTES {
        return Err(KernelError::Argument(format!("STEP file over {MAX_STEP_BYTES} bytes")));
    }
    Ok(())
}

fn to_records(breps: Vec<solidcore::topology::Brep>) -> Result<Vec<Brep>> {
    let mut out = Vec::with_capacity(breps.len());
    let mut k = Kernel::new(&KernelOptions::default());
    for mut b in breps {
        let (rec, _) =
            solidcore_convert::brep::to_record(&mut k, &mut b).map_err(|e| op_err("to_record", format!("{e:?}")))?;
        out.push(rec);
    }
    Ok(out)
}

/// Every body of a STEP file, as records.
pub fn import(text: &[u8]) -> Result<Vec<Brep>> {
    check_size(text)?;
    let breps =
        step_brep::import(text, &Default::default()).map_err(|e| op_err("step_brep::import", format!("{e:?}")))?;
    to_records(breps)
}

/// What the importer made of a file's PCURVEs when asked to build the faces' UV trims from them
/// (the importer takes a face's pcurves only when every edge use of it checks out against its 3-D
/// curve, and never on a B-spline face).
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct PcurveReport {
    /// Faces offered the file's pcurves.
    pub faces: usize,
    /// Edge uses that took their file pcurve.
    pub uses: usize,
    /// Of those, uses on a curved (not plane, not B-spline) face.
    pub curved_uses: usize,
    /// Seam edge uses, left to the importer's own projection by design.
    pub seam_uses: usize,
    /// Non-seam uses the importer projected instead of taking the file's pcurve.
    pub uses_projected: usize,
    /// Faces left to the importer's projection altogether.
    pub fallback_faces: usize,
}

/// [`import`] with the file's pcurves offered to the importer, and what became of them.
pub fn import_with_pcurves(text: &[u8]) -> Result<(Vec<Brep>, PcurveReport)> {
    check_size(text)?;
    let options =
        step_brep::BuildOptions { file_pcurves: true, mode: step_brep::BuildMode::StitchOnly, ..Default::default() };
    let report = step_brep::import_with_report_options(text, &Default::default(), options)
        .map_err(|e| op_err("step_brep::import_with_report_options", format!("{e:?}")))?;
    let s = &report.pcurves;
    let summary = PcurveReport {
        faces: s.faces,
        uses: s.uses,
        curved_uses: s.curved_uses,
        seam_uses: s.seam_uses,
        uses_projected: s.uses_projected,
        fallback_faces: s.fallbacks.values().sum(),
    };
    Ok((to_records(report.bodies)?, summary))
}

/// Writes solid bodies as one STEP file (AP214 with names).
pub fn export_solids(bodies: &[(&str, &Brep)]) -> Result<String> {
    if bodies.is_empty() {
        return Err(KernelError::Argument("nothing to export".into()));
    }
    let mut k = Kernel::new(&KernelOptions::default());
    let mut models = Vec::with_capacity(bodies.len());
    for (_, rec) in bodies {
        let kb = solidcore_convert::brep::to_kernel_body(&mut k, rec, true)
            .map_err(|e| op_err("to_kernel_body", format!("{e:?}")))?;
        let m = step_brep::brep_to_model::brep_to_model(&kb).map_err(|e| op_err("brep_to_model", format!("{e:?}")))?;
        models.push(m);
    }
    let mut it = models.into_iter();
    let mut m = it.next().ok_or_else(|| KernelError::Argument("nothing to export".into()))?;
    for more in it {
        m.bodies.extend(more.bodies);
    }
    step_export::write(&m).map_err(|e| op_err("step_export::write", e.message))
}
