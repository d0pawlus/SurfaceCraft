//! SurfaceCraft kernel adapter (layer L1): the one place that talks to solidcore
//! (`solidcore-ops`, `solidcore-record`, `solidcore-tess`, `step-*`), design doc §5/§9.
//!
//! Bodies cross this boundary as the neutral record ([`record::Brep`]): it is plain data
//! (`Clone`, `Send`, serde), so the document can snapshot it for undo and save it in a
//! project, and every writer (STEP, IGES) reads the same form. Each operation decodes the
//! record into a kernel body, runs the `ops` call and encodes the result again.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

mod blends;
pub mod feature;
pub mod hybrid;
pub mod local;
pub mod naming;
pub mod nurbs;
pub mod patch_fit;
pub mod pcurve;
pub mod refit;
pub mod solid;
pub mod step;
pub mod tess;
pub mod vpatch;

pub use solid::{BodyInfo, Counts};
/// The neutral B-rep record (solidcore-record), re-exported so higher layers never name solidcore.
pub use solidcore_record as record;

/// What went wrong in the kernel.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum KernelError {
    /// An argument refused before the kernel ran (not finite, out of range…).
    #[error("invalid argument: {0}")]
    Argument(String),
    /// The kernel operation failed.
    #[error("kernel operation `{op}` failed: {message}")]
    Op {
        /// Operation name.
        op: &'static str,
        /// The kernel's message.
        message: String,
    },
}

/// Result alias.
pub type Result<T> = std::result::Result<T, KernelError>;

pub(crate) fn op_err(op: &'static str, e: impl std::fmt::Display) -> KernelError {
    KernelError::Op { op, message: e.to_string() }
}

pub(crate) fn finite_all(vals: &[f64]) -> bool {
    vals.iter().all(|v| v.is_finite())
}

/// Our row-major column-vector matrix (`p' = M·p`, translation in the last column) as the
/// kernel's row-vector form (`p' = p·A + t`, translation in the last row): the transpose.
pub(crate) fn to_kernel_matrix(m: &[[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let mut t = [[0.0; 4]; 4];
    for (i, row) in m.iter().enumerate() {
        for (j, v) in row.iter().enumerate() {
            if let Some(slot) = t.get_mut(j).and_then(|r| r.get_mut(i)) {
                *slot = *v;
            }
        }
    }
    t
}
