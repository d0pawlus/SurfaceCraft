//! `scan.to_solid` with the size check (P9-U1): the run of the pipeline, and a size that looks like a
//! unit mistake among its issues, once. The finding is listed as an issue, and the scan report's copy
//! of it is dropped; a pass becomes a review.
use crate::Session;
use crate::cmd::CmdResult;
use crate::cmd::mesh::size_check;
use crate::cmd::pipeline::{Issue, ToSolidIn, ToSolidReport, Verdict, to_solid};
use surfacecraft_doc::ObjectKind;

/// The `scan.to_solid` command: the pipeline, with the size check added.
pub(crate) fn to_solid_checked(s: &mut Session, i: ToSolidIn) -> CmdResult<ToSolidReport> {
    let scan = i.scan.resolve(s, ObjectKind::Mesh)?;
    let size = size_check::warning_of(&s.doc, scan);
    let mut report = to_solid(s, i)?;
    let out = &mut report.result;
    if let Some(w) = size {
        out.scan_report.findings.retain(|f| f.code != w.code);
        out.issues.push(Issue { code: w.code.clone(), message: format!("{}. {}", w.message, w.suggestion) });
        if matches!(out.verdict, Verdict::Pass) {
            out.verdict = Verdict::Review;
        }
    }
    Ok(report)
}
