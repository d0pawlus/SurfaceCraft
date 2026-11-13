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
///
/// A sheet this application wrote ([`FILE_MARK`] in the header) of [`OWN_SHEET_FACES`] faces or
/// more is built from the topology the file states (every edge and vertex shared as the writer
/// shared them) instead of being stitched by geometry, which takes time quadratic in the faces
/// (80 s at 4 000 patches, measured). The topology build is kept only when it passes
/// [`topology_is_sound`]; otherwise the sheet is stitched, as every other file is.
///
/// The mark is a byte search in the first [`HEADER_BYTES`] bytes, not a parse, and it is not
/// authentication: any file can carry it. It only chooses which of two builds is tried first; the
/// parser's limits are the default ones for every file (a file of the writer's average 69 bytes
/// an entity has under 4 million entities at the 256 MiB size limit, so no raised limit is
/// needed; measured: the whole import takes about 1.3 KB an entity).
pub fn import(text: &[u8]) -> Result<Vec<Brep>> {
    check_size(text)?;
    if is_own_file(text) { import_own(text) } else { to_records(stitched(text)?) }
}

fn stitched(text: &[u8]) -> Result<Vec<solidcore::topology::Brep>> {
    step_brep::import(text, &Default::default()).map_err(|e| op_err("step_brep::import", format!("{e:?}")))
}

/// [`import`] with every body stitched by geometry, as the importer does by default: the reference
/// of the faster build of this application's own sheets.
pub fn import_stitched(text: &[u8]) -> Result<Vec<Brep>> {
    check_size(text)?;
    to_records(stitched(text)?)
}

/// What the file header of this application's writer says in its originating-system fields.
const FILE_MARK: &[u8] = b"'SurfaceCraft','SurfaceCraft'";

/// Sheets with at least this many faces are built from the file's topology.
const OWN_SHEET_FACES: usize = 300;

/// Bytes of the file searched for [`FILE_MARK`]: the header comes first.
const HEADER_BYTES: usize = 2048;

fn is_own_file(text: &[u8]) -> bool {
    let head = text.get(..HEADER_BYTES.min(text.len())).unwrap_or_default();
    head.windows(FILE_MARK.len()).any(|w| w == FILE_MARK)
}

fn import_own(text: &[u8]) -> Result<Vec<Brep>> {
    let model =
        step_import::import(text, &Default::default()).map_err(|e| op_err("step_import::import", e.to_string()))?;
    model.bodies.iter().map(build_own).collect()
}

fn build_own(body: &step_import::model::Body) -> Result<Brep> {
    let sheet = body.kind == step_import::model::BodyKind::ShellModel;
    if sheet && body.faces.len() >= OWN_SHEET_FACES {
        let topology = step_brep::build_from_topology(body, step_brep::BuildOptions::default());
        if let Ok(built) = topology
            && built.face_failures.is_empty()
            && step_brep::validate_for_display(&built.brep).is_ok()
            && let Ok(mut records) = to_records(vec![built.brep])
            && let Some(record) = records.pop()
            && topology_is_sound(&record, body.faces.len())
        {
            return Ok(record);
        }
    }
    // the stitched build alone: the topology build was tried above (or is not wanted)
    let options = step_brep::BuildOptions { mode: step_brep::BuildMode::StitchOnly, ..Default::default() };
    let built =
        step_brep::build_with_options(body, options).map_err(|e| op_err("step_brep::build", format!("{e:?}")))?;
    to_records(vec![built])?.pop().ok_or_else(|| op_err("step_brep::build", "no body"))
}

/// Is the topology build a sheet the stitched build would have agreed to? Every face of the file
/// is there, and every edge is used once or twice, by two faces that run along it in opposite
/// directions (face sense and loop direction together).
fn topology_is_sound(r: &Brep, faces: usize) -> bool {
    use crate::record::brep::{Loop, Shell};
    use std::collections::HashMap;
    if r.faces.len() != faces {
        return false;
    }
    // per edge: the directions of its uses (true = along the edge)
    let mut uses: HashMap<u32, Vec<bool>> = HashMap::new();
    for shell in &r.shells {
        let Shell::Faces { faces: oriented, .. } = shell else { return false };
        for of in oriented {
            let Some(face) = r.faces.get(of.face as usize) else { return false };
            for lp in &face.loops {
                let Loop::Edges { edges } = lp else { return false };
                for oe in edges {
                    uses.entry(oe.edge).or_default().push(oe.forward == of.forward);
                }
            }
        }
    }
    r.edges.len() == uses.len()
        && uses.values().all(|d| match d.as_slice() {
            [_] => true,
            [a, b] => a != b,
            _ => false,
        })
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::record::brep::{Loop, Shell};

    fn boxed() -> (Brep, usize) {
        let b = crate::solid::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap();
        let n = b.faces.len();
        (b, n)
    }

    #[test]
    fn a_consistently_oriented_closed_body_is_sound() {
        let (b, n) = boxed();
        assert!(topology_is_sound(&b, n));
    }

    #[test]
    fn a_missing_face_is_not_sound() {
        let (b, n) = boxed();
        assert!(!topology_is_sound(&b, n + 1));
        assert!(!topology_is_sound(&b, n - 1));
    }

    #[test]
    fn a_face_of_reversed_sense_is_not_sound() {
        let (mut b, n) = boxed();
        if let Some(Shell::Faces { faces, .. }) = b.shells.first_mut() {
            faces[0].forward = !faces[0].forward;
        }
        assert!(!topology_is_sound(&b, n));
    }

    #[test]
    fn an_edge_used_three_times_or_not_at_all_is_not_sound() {
        let (b, n) = boxed();
        // one loop edge redirected to the edge of another face: that edge now has three uses
        let mut three = b.clone();
        let target = match three.faces[1].loops.first() {
            Some(Loop::Edges { edges }) => edges[0].edge,
            _ => panic!("edge loop"),
        };
        if let Some(Loop::Edges { edges }) = three.faces[0].loops.first_mut() {
            edges[0].edge = target;
        }
        assert!(!topology_is_sound(&three, n));
        // the edge the redirected loop edge no longer names has one use less: still sound by
        // use count alone, but the edge list no longer matches the uses
        let mut dropped = b.clone();
        dropped.edges.push(dropped.edges[0].clone());
        assert!(!topology_is_sound(&dropped, n));
    }

    #[test]
    fn a_vertex_loop_is_not_a_sheet_the_topology_build_may_keep() {
        let (mut b, n) = boxed();
        b.faces[0].loops = vec![Loop::Vertex { vertex: 0 }];
        assert!(!topology_is_sound(&b, n));
    }
}
