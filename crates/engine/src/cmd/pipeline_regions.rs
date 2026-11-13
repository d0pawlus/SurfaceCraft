//! The region selection of `scan.to_solid` (method `primitives`): which regions a primitive is fitted
//! to, which are left out and why.
use super::{LeftOut, MAX_INPUTS, Modelled, message_of, triangle_count};
use crate::Session;
use crate::cmd::ObjRef;
use crate::cmd::primitive::{FitIn, KindChoice, fit_cmd};
use surfacecraft_doc::ObjectId;
use surfacecraft_fit::PrimitiveKind;

/// A region as the selection sees it.
pub(super) struct Candidate {
    pub(super) id: ObjectId,
    pub(super) triangles: usize,
    pub(super) kind: Option<PrimitiveKind>,
    pub(super) rms: Option<f64>,
}

/// The regions of the scan, largest first, and the triangles in none of them.
pub(super) fn candidates(s: &Session, scan: ObjectId) -> (Vec<Candidate>, usize) {
    let mut list: Vec<Candidate> = s
        .doc
        .regions_of(scan)
        .map(|r| Candidate { id: r.id, triangles: r.triangles.len(), kind: r.kind, rms: r.rms })
        .collect();
    list.sort_by(|a, b| b.triangles.cmp(&a.triangles).then(a.id.cmp(&b.id)));
    let assigned: usize = list.iter().map(|c| c.triangles).sum();
    (list, triangle_count(s, scan).saturating_sub(assigned))
}

/// Why a region is not modelled; `None` when it is.
pub(super) fn refusal(c: &Candidate, min_triangles: usize, rms_limit: f64, taken: usize) -> Option<String> {
    if c.triangles < min_triangles {
        return Some(format!("fewer than {min_triangles} triangles"));
    }
    if c.kind.is_none() {
        return Some("no primitive kind fits it".to_string());
    }
    match c.rms {
        Some(rms) if rms > rms_limit => {
            Some(format!("RMS {rms:.3} mm is above {rms_limit:.3} mm: not a primitive at this noise"))
        }
        _ if taken >= MAX_INPUTS => Some(format!("over the limit of {MAX_INPUTS} inputs of one solid")),
        _ => None,
    }
}

pub(super) fn left_out(c: &Candidate, reason: String) -> LeftOut {
    LeftOut { region: c.id, triangles: c.triangles, kind: c.kind, rms: c.rms, reason }
}

/// Share of the scan's triangles in the regions a primitive would be fitted to.
pub(super) fn primitive_share(list: &[Candidate], min_triangles: usize, rms_limit: f64, total: usize) -> f64 {
    if total == 0 {
        return 0.0;
    }
    let fit: usize =
        list.iter().filter(|c| refusal(c, min_triangles, rms_limit, 0).is_none()).map(|c| c.triangles).sum();
    fit as f64 / total as f64
}

/// Fits a primitive to every region that qualifies. Returns the modelled regions and the others.
pub(super) fn fit_regions(
    s: &mut Session,
    list: &[Candidate],
    min_triangles: usize,
    rms_limit: f64,
) -> (Vec<Modelled>, Vec<LeftOut>) {
    let mut modelled: Vec<Modelled> = Vec::new();
    let mut out: Vec<LeftOut> = Vec::new();
    for c in list {
        if let Some(reason) = refusal(c, min_triangles, rms_limit, modelled.len()) {
            out.push(left_out(c, reason));
            continue;
        }
        let params =
            FitIn { region: Some(ObjRef::Id(c.id.0)), mesh: None, triangles: None, kind: KindChoice::Auto, name: None };
        match fit_cmd(s, params) {
            Ok(f) => modelled.push(Modelled {
                region: c.id,
                primitive: f.id,
                kind: f.primitive.kind(),
                triangles: c.triangles,
                rms: f.rms,
                role: String::new(),
                faces: Vec::new(),
                skipped: None,
            }),
            Err(e) => out.push(left_out(c, format!("no primitive: {}", message_of(&e)))),
        }
    }
    (modelled, out)
}
