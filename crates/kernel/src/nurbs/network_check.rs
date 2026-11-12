//! Limits and the record check of a Bézier patch network (P9-O3).
//!
//! `Brep::validate` counts every knot, control point and pcurve point against solidcore's
//! `MAX_ITEMS` (4 000 000), a bound meant for records that come from outside. A bicubic patch of a
//! closed network costs about 100 items, so a network of roughly 40 000 patches is refused with
//! `item budget` although the record is our own. The network is built here, in memory, from
//! slices the caller already holds, so it is checked with `Brep::validate_built` (every check but
//! the budget) and bounded by [`MAX_NETWORK_PATCHES`] instead.
use crate::op_err;
use crate::record::brep::Brep;
use crate::solid::HEAL_FACE_LIMIT;
use crate::{KernelError, Result};

/// The most patches one network may have: memory and run time stay bounded for hostile callers.
/// Well above the 60 000 the engine allows `surface.wrap`.
pub const MAX_NETWORK_PATCHES: usize = 250_000;

/// Refuses a network with no patch or more than [`MAX_NETWORK_PATCHES`].
pub(crate) fn check_patch_count(count: usize) -> Result<()> {
    if count == 0 || count > MAX_NETWORK_PATCHES {
        return Err(KernelError::Argument(format!("a network has 1 to {MAX_NETWORK_PATCHES} patches, got {count}")));
    }
    Ok(())
}

/// Every structural check of the record without the input item budget.
pub(crate) fn validate_network_record(brep: &Brep) -> Result<()> {
    brep.validate_built().map_err(|e| op_err("record validate", e))
}

/// Is the kernel's healer run on a network of `patches` faces? It is not above
/// [`HEAL_FACE_LIMIT`]: it grows faster than linearly (see `docs/upstream/`).
pub(crate) fn heal_allowed(patches: usize) -> bool {
    patches <= HEAL_FACE_LIMIT
}
