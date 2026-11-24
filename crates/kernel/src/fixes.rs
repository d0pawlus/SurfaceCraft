//! solidcore's opt-in upstream fixes (`solidcore::kernel::UpstreamFixes`): off unless the
//! application names them. Which ones the application runs with is an owner decision (pin
//! migration of P9-O8); until it is made they stay off, and this switch exists to measure them.
use crate::{Result, op_err};
use solidcore::kernel::UpstreamFixes;

/// The environment variable that turns every upstream fix on for the process when it is `1`
/// (a measurement switch; nothing reads it by default but [`apply_from_env`]).
pub const ENV: &str = "SURFACECRAFT_UPSTREAM_FIXES";

/// The kernel runs with every upstream fix solidcore offers (`on`), or with none, from the next
/// call on, for the whole process.
///
/// # Errors
/// The kernel's refusal (a call made from inside a kernel call).
pub fn set_all(on: bool) -> Result<()> {
    let names: &[&str] = if on { UpstreamFixes::NAMES } else { &[] };
    solidcore_ops::session::set_fixes(names.iter().copied()).map_err(|e| op_err("set_fixes", e))
}

/// [`set_all`]`(true)` when [`ENV`] is `1`; nothing otherwise. Returns whether the fixes are on.
///
/// # Errors
/// As [`set_all`].
pub fn apply_from_env() -> Result<bool> {
    let on = std::env::var(ENV).is_ok_and(|v| v == "1");
    if on {
        set_all(true)?;
    }
    Ok(on)
}

/// The names of the fixes the kernel runs with now.
#[must_use]
pub fn active() -> Vec<&'static str> {
    solidcore_ops::session::fixes()
}
