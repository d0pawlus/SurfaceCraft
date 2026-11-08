//! STEP through solidcore: import of any STEP file into records (`step_brep::import`, used for
//! round-trip checks), and export of solid bodies (`step-brep::brep_to_model` +
//! `step-export`, design doc §10 milestone M5).
use crate::record::brep::Brep;
use crate::{KernelError, Result, op_err};
use solidcore::kernel::{Kernel, KernelOptions};

/// Largest STEP text accepted (bytes).
pub const MAX_STEP_BYTES: usize = 256 * 1024 * 1024;

/// Every body of a STEP file, as records.
pub fn import(text: &[u8]) -> Result<Vec<Brep>> {
    if text.len() > MAX_STEP_BYTES {
        return Err(KernelError::Argument(format!("STEP file over {MAX_STEP_BYTES} bytes")));
    }
    let breps =
        step_brep::import(text, &Default::default()).map_err(|e| op_err("step_brep::import", format!("{e:?}")))?;
    let mut out = Vec::with_capacity(breps.len());
    let mut k = Kernel::new(&KernelOptions::default());
    for mut b in breps {
        let (rec, _) =
            solidcore_convert::brep::to_record(&mut k, &mut b).map_err(|e| op_err("to_record", format!("{e:?}")))?;
        out.push(rec);
    }
    Ok(out)
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
