//! What the merge reads of a group of patches, worked out once per body and passed along: its boundary,
//! the points along each boundary edge and the side the patches face.
use super::boundary::{Boundary, boundary, edge_points};
use super::patch_outward;
use super::place::Gauge;
use crate::hybrid::{V, add, norm};
use crate::record::brep::Brep;
use crate::{KernelError, Result, finite_all};

/// A group of patches as the merge sees it.
pub(super) struct Group {
    pub boundary: Boundary,
    /// The points along each boundary edge (its end vertices included), in the order of the boundary's loops.
    pub edges: Vec<Vec<V>>,
    /// The sum of the outward directions of the patches.
    pub outward: V,
}

impl Group {
    /// All the boundary points.
    pub(super) fn points(&self) -> Vec<V> {
        self.edges.iter().flatten().copied().collect()
    }

    /// The largest distance of the boundary points from the primitive (mm).
    ///
    /// # Errors
    /// A point the primitive has no closest point to.
    pub(super) fn gap(&self, gauge: &Gauge) -> Result<f64> {
        let mut gap = 0.0_f64;
        for p in self.edges.iter().flatten() {
            let d = gauge.distance(*p).ok_or_else(|| KernelError::Argument("the shape cannot be merged".into()))?;
            gap = gap.max(d);
        }
        Ok(gap)
    }
}

/// The boundary, edge points and outward direction of the patches `faces` of `body`.
///
/// # Errors
/// A group that has no simple boundary, an edge that cannot be evaluated, and patches that do not face one way.
pub(super) fn group_of(body: &Brep, faces: &[usize]) -> Result<Group> {
    let boundary = boundary(body, faces)?;
    let mut outward = [0.0; 3];
    for &f in faces {
        outward = add(outward, patch_outward(body, f)?);
    }
    if !finite_all(&outward) || norm(outward) < 1e-9 {
        return Err(KernelError::Argument("the patches of the group do not face one way".into()));
    }
    let mut edges = Vec::new();
    for le in boundary.loops.iter().flatten() {
        edges.push(edge_points(body, le.edge)?);
    }
    Ok(Group { boundary, edges, outward })
}
