//! Separate sheets sewn into one body (P9-O8): every sheet decoded as its own kernel body, then
//! gathered and sewn along the edges that meet within a tolerance (`solidcore_ops::stitch::sew`),
//! and the faces oriented: outward where they close into a solid.
use crate::record::brep::Brep;
use crate::{KernelError, Result, op_err};

/// The largest sewing tolerance (mm): the kernel takes no edge or vertex tolerance larger.
pub const MAX_TOLERANCE_MM: f64 = 1.0;

/// Most faces sewn in one call.
pub const MAX_FACES: usize = 200_000;

/// The sewn body.
#[derive(Clone, Debug)]
pub struct Sewn {
    /// The body: one record of every sheet's faces.
    pub brep: Brep,
    /// The faces close into a solid (no edge used by one face only, and a manifold solid).
    pub closed: bool,
    /// Edges still used by one face only.
    pub open_edges: usize,
}

/// `sheets` gathered into one body and sewn where their edges meet within `tolerance` mm (finite,
/// positive, at most [`MAX_TOLERANCE_MM`]).
///
/// # Errors
/// No sheet, a tolerance out of range, more than [`MAX_FACES`] faces, a sheet the kernel does not
/// read, or the kernel's refusal to sew.
pub fn sew(sheets: &[&Brep], tolerance: f64) -> Result<Sewn> {
    if sheets.is_empty() {
        return Err(KernelError::Argument("no sheets to sew".into()));
    }
    if !(tolerance.is_finite() && tolerance > 0.0 && tolerance <= MAX_TOLERANCE_MM) {
        return Err(KernelError::Argument(format!(
            "the sewing tolerance must be above 0 and at most {MAX_TOLERANCE_MM} mm, got {tolerance}"
        )));
    }
    let faces: usize = sheets.iter().map(|b| b.faces.len()).sum();
    if faces > MAX_FACES {
        return Err(KernelError::Argument(format!("{faces} faces to sew; at most {MAX_FACES} in one call")));
    }
    let mut bodies = Vec::with_capacity(sheets.len());
    for (i, b) in sheets.iter().enumerate() {
        let body = crate::solid::decode(b).map_err(|e| KernelError::Argument(format!("sheet {i}: {e}")))?;
        bodies.push(body);
    }
    let refs: Vec<&solidcore_ops::Body> = bodies.iter().collect();
    let sewn = solidcore_ops::stitch::sew(&refs, tolerance).map_err(|e| op_err("sew", e))?;
    Ok(Sewn { brep: crate::solid::encode(&sewn.body)?, closed: sewn.closed, open_edges: sewn.open_edges.len() })
}

#[cfg(test)]
mod tests;
