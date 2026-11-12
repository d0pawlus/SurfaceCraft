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
/// Twice the 60 000 the engine allows `surface.wrap`. The kernel's conversion of a record is
/// linear up to about 60 000 faces and then grows faster (`fill_shells` checks `shell.faces`
/// for every face it inserts; draft 09): measured on a cube sphere, `ci` profile, shared
/// machine, the build of the network takes 3.4 s at 60 000 patches, 10.8 s at 121 000 and 37 s at
/// 240 000, and the volume of the body (a decode and a measurement) 9 s, 25 s and 111 s. This
/// limit keeps the build near 11 s and one decode near 25 s.
pub const MAX_NETWORK_PATCHES: usize = 120_000;

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_size_guard_accepts_the_limit_and_refuses_one_more() {
        assert!(check_patch_count(1).is_ok());
        assert!(check_patch_count(MAX_NETWORK_PATCHES).is_ok());
        let over = check_patch_count(MAX_NETWORK_PATCHES + 1).map_err(|e| e.to_string());
        assert!(over.is_err_and(|m| m.contains("patches, got")));
        assert!(check_patch_count(0).is_err());
    }
}
