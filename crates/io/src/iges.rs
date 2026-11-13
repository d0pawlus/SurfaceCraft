// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/kernel/src/iges_out.rs (IGES 5.3 B-rep export: manifold solids 186 and shells 514 of
// faces 510, loops 508, edge lists 504 and vertex lists 502; B-spline geometry 126/128, lines 110,
// arcs 100 and conics 104 under 124 frames; name properties 406; the fixed-column file layout).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): works directly on solidcore-record without
// solidcraft-step's geometry layer; every face is written on the surface its (u, v) are on (its
// NURBS form when it has one, planes as exact bilinear 128 patches), so the record's loops keep
// their orientation; vertex loops; the outer loop found from the pcurves; no colours; a fixed
// time stamp (deterministic output, no clock on wasm); refusals as `IoError::Export`.
// Technical debt (design doc §10): replace with an IGES writer in solidcore if one is moved there.

//! IGES 5.3 export of record bodies, millimetres (design doc §10). Each bounded material region
//! becomes a manifold solid B-rep object (186) with its voids; every other face shell becomes an
//! open shell (514 form 2). Faces carry their exact geometry as B-spline surfaces (128) and
//! their edges as lines (110), arcs (100), conics (104) or B-spline curves (126). IGES loops run
//! counter-clockwise about their surface's normal, which is the record's convention for the
//! surface a face is parameterised on.

use std::collections::HashMap;
use std::fmt::Write as _;

use crate::IoError;
use surfacecraft_kernel::record::brep::{Brep, Curve, Curve2, Edge, Face, Loop, OrientedEdge, Shell, Surface};

type V3 = [f64; 3];

/// Largest directory or parameter sequence number the fixed columns hold.
const MAX_SEQUENCE: usize = 9_999_999;
mod curves;
mod format;
mod layout;
mod shells;
mod surfaces;
#[cfg(test)]
mod tests;
mod writer;

use format::*;
use layout::*;
use shells::*;
use writer::*;

/// IGES 5.3 text (millimetres) for record bodies: bounded material regions as manifold solid
/// B-rep objects (186), other face shells as open shells (514 form 2), each named `name`.
pub fn write_iges(name: &str, bodies: &[&Brep]) -> Result<String, IoError> {
    if bodies.is_empty() {
        return Err(bad("nothing to export"));
    }
    let mut w = W { ents: Vec::new(), max_coord: 1.0 };
    for (bi, b) in bodies.iter().enumerate() {
        b.validate().map_err(|e| bad(format!("body {bi}: {e}")))?;
        let items = layout_of(bi, b)?;
        // A closed shell is form 1, an open one form 2.
        for (outer, voids) in &items.solids {
            let params = shell(&mut w, b, *outer)?;
            let o = w.add(514, 1, params);
            let mut p = vec![o.to_string(), "1".into(), voids.len().to_string()];
            for &v in voids {
                let params = shell(&mut w, b, v)?;
                p.extend([w.add(514, 1, params).to_string(), "1".into()]);
            }
            top_level(&mut w, 186, 0, p, name);
        }
        for &si in &items.sheets {
            let params = shell(&mut w, b, si)?;
            top_level(&mut w, 514, 2, params, name);
        }
    }
    if w.ents.is_empty() {
        return Err(bad("nothing to export: no faces"));
    }
    layout(&w, name)
}
