//! Walls: how thin the scan is, and the voxel and cube that keep them. A wall must be at least
//! `2k + 2` voxels thick for the closing and opening (which remove everything up to `2k` voxels wide)
//! to keep it; a thinner one is resolved with a finer voxel or refused, never merged.
use super::{MAX_K, RemeshError};
use crate::{Bvh, Mesh, par};

/// Samples of the surface probed for its thickness.
const PROBE_SAMPLES: usize = 2000;
/// A wall must be this many voxels thick beyond the closing: `2k + 2`.
const EXTRA_VOXELS: f64 = 2.0;
/// A wall is resolved with this many voxels more than the minimum (the rest of the margin).
const MARGIN_VOXELS: f64 = 0.5;
/// Share of the probes allowed to find a thinner wall (those are detail, not the wall).
const PERCENTILE: usize = 2;
/// The voxel is never made finer than the default divided by this.
const MAX_REFINE: f64 = 8.0;
/// A share of the surface this large that lies within the removal width of another surface is a thin
/// wall everywhere (a plate), not a few thin features.
const THIN_SURFACE_SHARE: f64 = 0.2;
/// A share this large (below [`THIN_SURFACE_SHARE`]) is reported: thin features that will be removed.
const THIN_REPORT_SHARE: f64 = 0.02;
/// A wall that is thin everywhere is cut by a removal no wider than this share of its thickness.
const THIN_REMOVAL_SHARE: f64 = 0.45;

/// What the probes found.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Walls {
    /// The low percentile of the thicknesses above the removal width.
    pub wall: Option<f64>,
    /// The median of the thicknesses up to the removal width.
    pub thin: Option<f64>,
    /// Share of the probes that found a surface within the removal width.
    pub thin_share: f64,
}

/// Distances from area-weighted sample points, along the inward normal, to the surface on the other
/// side. Those up to `ignore` (what the closing removes: handles, fins, or a plate thinner than that)
/// are the `thin` ones.
pub(super) fn probe(mesh: &Mesh, bvh: &Bvh, h: f64, ignore: f64) -> Walls {
    let offset = 0.5 * h;
    let samples = mesh.sample_points(PROBE_SAMPLES, &mut crate::Rng::new(7));
    let mut hits: Vec<f64> = par::map_slice(&samples, |(p, n)| {
        let dir = -crate::normalize(*n)?;
        bvh.ray_cast(&(p + dir * offset), &dir).map(|(_, t)| t + offset)
    })
    .into_iter()
    .flatten()
    .collect();
    hits.sort_by(f64::total_cmp);
    let split = hits.partition_point(|t| *t <= ignore);
    let (thin, thick) = hits.split_at(split);
    Walls {
        wall: thick.get(thick.len() * PERCENTILE / 100).copied(),
        thin: thin.get(thin.len() / 2).copied(),
        thin_share: thin.len() as f64 / hits.len().max(1) as f64,
    }
}

/// The voxel, cube half-size and notes for the walls found.
pub(super) struct Resolution {
    pub h: f64,
    pub k: u32,
    pub refined: bool,
    pub warnings: Vec<String>,
}

/// Voxel and cube for walls `walls`. The width of what is removed stays fixed in mm (a finer voxel
/// gets a larger cube) unless the part is a thin wall everywhere, where it shrinks with the wall.
pub(super) fn resolution(
    voxel: Option<f64>,
    min_feature: Option<f64>,
    tolerance: f64,
    walls: &Walls,
) -> Result<Resolution, RemeshError> {
    let h0 = voxel.unwrap_or(1.5 * tolerance);
    let k0 = match min_feature {
        None => 1,
        Some(w) => ((w / (2.0 * h0)).round() as u32).min(MAX_K),
    };
    let need = |k: u32, h: f64| (2.0 * f64::from(k) + EXTRA_VOXELS) * h;
    let thin_surface = walls.thin_share > THIN_SURFACE_SHARE;
    let t = [walls.wall, thin_surface.then_some(walls.thin).flatten()].into_iter().flatten().reduce(f64::min);
    let mut warnings = Vec::new();
    if !thin_surface && walls.thin_share > THIN_REPORT_SHARE {
        let thin = walls.thin.unwrap_or(0.0);
        warnings.push(format!(
            "{:.0} % of the surface lies within {:.2} mm of another surface (about {thin:.2} mm apart); closing and opening remove those features",
            100.0 * walls.thin_share,
            2.0 * f64::from(k0) * h0 + 0.5 * h0
        ));
    }
    let Some(t) = t.filter(|t| *t < need(k0, h0)) else {
        return Ok(Resolution { h: h0, k: k0, refined: false, warnings });
    };
    let suggested = |k: u32| t / (2.0 * f64::from(k) + EXTRA_VOXELS + MARGIN_VOXELS);
    let thin = |k: u32, h: f64| RemeshError::ThinWall {
        thickness: t,
        required: need(k, h),
        voxel: h,
        suggested: suggested(k),
    };
    if voxel.is_some() || (thin_surface && min_feature.is_some_and(|w| w >= t)) {
        return Err(thin(k0, h0));
    }
    let removed = 2.0 * f64::from(k0) * h0;
    let removed = if thin_surface { removed.min(THIN_REMOVAL_SHARE * t) } else { removed };
    let (mut h, mut k) = (h0, k0);
    for _ in 0..8 {
        k = ((removed / (2.0 * h)).ceil() as u32).clamp(1, MAX_K);
        h = h.min(suggested(k)).max(h0 / MAX_REFINE);
    }
    if t < need(k, h) {
        return Err(thin(k, h));
    }
    Ok(Resolution { h, k, refined: true, warnings })
}
