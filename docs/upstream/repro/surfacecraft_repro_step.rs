// OBSOLETE since P9-O8 (solidcore pin c02c5c51c): `step-brep`, `step-import` and `step-export` are not in
// solidcore's workspace any more; STEP goes through `solidcore-step`. Kept for the record of what was
// reproduced on 4f2da85; the narrowing cone it shows round trips on c02c5c51c (draft 01).
//! Kernel-only reproduction for solidcore's STEP writer and reader.
//! Copy to `crates/solidcore-step/tests/surfacecraft_repro_step.rs` of a solidcore checkout and run:
//!   cargo test -p solidcore-step --test surfacecraft_repro_step
//!
//! Result on solidcore `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e` (2026-11-15, dev profile,
//! Windows x86_64): cone_narrowing_along_z_survives_step_round_trip passes (it failed on
//! `4f2da85` through the removed `step-brep` reader).
//!
//! The STEP problem of the kernel reproductions found on solidcore `4f2da85` while building a
//! scan-to-CAD application on it, ported to solidcore's own public API: a narrowing cone written
//! to STEP and read back. The other problems are in
//! `crates/solidcore-ops/tests/surfacecraft_repro.rs`; this one needs the STEP writer and reader,
//! which `solidcore-ops` cannot reach.
//!
//! The test asserts the EXPECTED, correct behaviour and is not ignored: when it fails, it names
//! how many bodies came back and what the reader refused. The "Actual" lines of the doc comment
//! are what `4f2da85` did.
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::indexing_slicing)] // test helpers

use solidcore_ops as ops;
use solidcore_step::p21;
use solidcore_step::topology::read_solids;
use solidcore_step::write::product::{bodies, ExportBody, Header};

/// Problem a: a cone that narrows along +z (radius 5 at the base, 1 at the top, height 10)
/// is exported to STEP and read back as ONE body.
///
/// Input: `ops::make_cone([0;3], 5, 1, 10)` as a record, `step::export_solids`, `step::import`.
/// Expected: one body back (the same solid as before).
/// Actual: the STEP import splits the solid into several bodies, one per face (the negative
/// half-angle conical face does not survive `step_brep::import`).
#[test]
fn cone_narrowing_along_z_survives_step_round_trip() {
    let cone = ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).unwrap_or_else(|e| panic!("make_cone failed: {e} ({e:?})"));
    let (rec, _) = cone.to_record().unwrap_or_else(|e| panic!("to_record failed: {e} ({e:?})"));
    let export = vec![ExportBody { name: "cone".to_string(), record: &rec, color: None, faces: Vec::new() }];
    let text = bodies(export, &Header::default()).unwrap_or_else(|e| panic!("STEP export failed: {e} ({e:?})"));
    let exchange = p21::parse(&text).unwrap_or_else(|e| panic!("the written STEP file does not parse: {e} ({e:?})"));
    let read = read_solids(&exchange);
    let faces: Vec<usize> = read.solids.iter().map(|s| s.brep.faces.len()).collect();
    let sheets: Vec<bool> = read.solids.iter().map(|s| s.sheet).collect();
    assert_eq!(
        read.solids.len(),
        1,
        "a single solid must come back as one body, got {} bodies (faces per body {faces:?}, sheet flags {sheets:?}, \
         items the reader refused {:?}, notes {:?})",
        read.solids.len(),
        read.failures,
        read.solids.iter().map(|s| s.notes.clone()).collect::<Vec<_>>()
    );
}
