//! STEP export of sheet (open shell) and solid bodies, design doc §10 milestones M3 and M5: both
//! written by solidcore's STEP writer through the kernel adapter (`surfacecraft_kernel::step`), one
//! AP242 file per call. Sheets and solids go to separate files.

#[cfg(test)]
mod network_fixture;
#[cfg(test)]
mod solid_tests;
#[cfg(test)]
mod tests;

use crate::IoError;
use surfacecraft_kernel::record::brep::Brep;

fn bad(m: impl Into<String>) -> IoError {
    IoError::Export(m.into())
}

/// Is the record a sheet (no bounded material region)?
#[must_use]
pub fn is_sheet(b: &Brep) -> bool {
    b.regions.iter().skip(1).all(|reg| reg.void || reg.shells.is_empty()) && !b.faces.is_empty()
}

/// The bodies named `<name> 1`, `<name> 2`, … written as one file named `name`.
fn write(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if bodies.is_empty() {
        return Err(bad("nothing to export"));
    }
    let names: Vec<String> = (1..=bodies.len()).map(|i| format!("{name} {i}")).collect();
    let named: Vec<(&str, &Brep)> = names.iter().map(String::as_str).zip(bodies.iter().copied()).collect();
    surfacecraft_kernel::step::export(name, &named).map_err(|e| bad(e.to_string()))
}

/// Writes sheet bodies as one file: each body's open shells as a SHELL_BASED_SURFACE_MODEL of
/// OPEN_SHELLs, millimetres. A record the file cannot say faithfully (a number that is not
/// finite, a curve or surface STEP has no exact form for) is an error, never a file.
pub fn write_sheets(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if let Some(bi) = bodies.iter().position(|b| !is_sheet(b)) {
        return Err(bad(format!("body {bi} is a solid; sheets only here (solids go to a file of their own)")));
    }
    write(name, bodies)
}

/// Writes solid bodies as one file: each material region as a MANIFOLD_SOLID_BREP (BREP_WITH_VOIDS
/// with voids) whose vertices, edge curves and control points are written once and shared by the
/// faces that use them.
pub fn write_solids(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if let Some(bi) = bodies.iter().position(|b| is_sheet(b)) {
        return Err(bad(format!("body {bi} is a sheet; solids only here (sheets go to a file of their own)")));
    }
    write(name, bodies)
}
