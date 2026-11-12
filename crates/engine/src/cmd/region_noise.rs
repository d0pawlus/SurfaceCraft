//! The scanner noise `region.auto_segment` and `region.resegment` hand to the segmentation: given
//! by the caller, measured on the scan (never derived from the fits of the regions), or a
//! documented fallback.
use super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_deviation::noise_estimate;
use surfacecraft_fit::segment::{SegmentOptions, fallback_noise};
use surfacecraft_geom::{Aabb, Mesh, Point3};

/// Where the noise used by the segmentation came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum NoiseSource {
    /// Measured on the geometry that is segmented (`deviation.noise`): the whole mesh for
    /// `region.auto_segment`, the selected triangles for `region.resegment`.
    Estimated,
    /// Measured on the whole mesh because the selection is too small or coarse to measure, and
    /// the noise is the same all over the mesh (the blocks of the mesh agree).
    EstimatedMesh,
    /// Passed in `options.noise`.
    Given,
    /// No reliable, uniform estimate (a very coarse mesh, or noise that differs by more than a
    /// factor of two between parts of the mesh): a quarter of the tolerance, with which only
    /// small regions are consolidated into freeform areas.
    Fallback,
}

/// The noise used and where it came from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct NoiseUsed {
    pub sigma: f64,
    pub source: NoiseSource,
}

/// The mesh is cut into this many blocks along each axis of its box to see whether the noise is the
/// same all over it.
const BLOCKS: usize = 3;
/// A block with fewer triangles is not measured.
const MIN_BLOCK_TRIANGLES: usize = 300;
/// The noise is not uniform when the largest block estimate exceeds the smallest by more than this
/// factor (the estimator reads 0.0307 to 0.0333 for a true 0.03 on every shape and density, so
/// a factor of two is far outside its own spread).
const HETEROGENEOUS_RATIO: f64 = 2.0;

/// The estimated noise (sigma, mm) of `mesh`, when it can be trusted: enough smooth samples.
fn reliable_noise(mesh: &Mesh) -> Option<f64> {
    let e = noise_estimate(mesh);
    (e.smooth && e.sigma.is_finite() && e.sigma > 0.0).then_some(e.sigma)
}

/// Estimates of the blocks of `mesh` (those with enough triangles and a reliable estimate).
fn block_noise(mesh: &Mesh) -> Vec<f64> {
    let b = Aabb::from_points(&mesh.positions);
    let cell = |c: &Point3| {
        let k = |v: f64, lo: f64, hi: f64| (((v - lo) / (hi - lo).max(1e-12) * BLOCKS as f64) as usize).min(BLOCKS - 1);
        (k(c.x, b.min.x, b.max.x) * BLOCKS + k(c.y, b.min.y, b.max.y)) * BLOCKS + k(c.z, b.min.z, b.max.z)
    };
    let mut cells: Vec<Vec<u32>> = vec![Vec::new(); BLOCKS * BLOCKS * BLOCKS];
    for t in 0..mesh.triangle_count() {
        let Some(c) = mesh.face_centroid(t) else { continue };
        if let Some(v) = cells.get_mut(cell(&c)) {
            v.push(t as u32);
        }
    }
    cells
        .iter()
        .filter(|tris| tris.len() >= MIN_BLOCK_TRIANGLES)
        .filter_map(|tris| reliable_noise(&mesh.submesh(tris)))
        .collect()
}

/// Is the noise of `mesh` the same all over it (as far as it can be told)?
fn is_uniform(mesh: &Mesh) -> bool {
    let blocks = block_noise(mesh);
    let lo = blocks.iter().copied().fold(f64::INFINITY, f64::min);
    let hi = blocks.iter().copied().fold(0.0, f64::max);
    blocks.len() < 2 || hi <= HETEROGENEOUS_RATIO * lo
}

/// Options for segmenting with the noise filled in, and the noise used (`None` for the report when
/// the freeform consolidation, the only user of the noise, is off). The noise is given by the
/// caller; else measured on `target`, the geometry that is segmented (the whole mesh, or the
/// selected triangles as a mesh of their own), when that is reliable and uniform; else, for a
/// selection too small to measure, on the `whole` mesh when that is uniform; else the fallback.
pub(super) fn with_noise(
    whole: &Mesh,
    target: &Mesh,
    options: &SegmentOptions,
) -> CmdResult<(SegmentOptions, Option<NoiseUsed>)> {
    if let Some(n) = options.noise
        && !(n.is_finite() && n > 0.0)
    {
        return Err(CmdError::Params("noise must be a positive number of millimetres".into()));
    }
    if !options.freeform {
        return Ok((options.clone(), None));
    }
    if let Some(sigma) = options.noise {
        return Ok((options.clone(), Some(NoiseUsed { sigma, source: NoiseSource::Given })));
    }
    let measured = match reliable_noise(target) {
        Some(sigma) => is_uniform(target).then_some((sigma, NoiseSource::Estimated)),
        None => reliable_noise(whole).filter(|_| is_uniform(whole)).map(|sigma| (sigma, NoiseSource::EstimatedMesh)),
    };
    Ok(match measured {
        Some((sigma, source)) => {
            (SegmentOptions { noise: Some(sigma), ..options.clone() }, Some(NoiseUsed { sigma, source }))
        }
        None => {
            let sigma = fallback_noise(options.tolerance);
            (options.clone(), Some(NoiseUsed { sigma, source: NoiseSource::Fallback }))
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, Rng, Vec3, synth};

    /// A flat plate with noise `sigma` (150 cells a side) at x offset `x0`.
    fn plate(x0: f64, sigma: f64, seed: u64) -> Mesh {
        let m = synth::plane(Point3::new(x0, 0.0, 0.0), Vec3::x(), Vec3::y(), 100.0, 100.0, 150, 150);
        let mut m = m;
        synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
        m
    }

    /// Eight vertices: too few to measure.
    fn tiny() -> Mesh {
        synth::cuboid(Point3::origin(), Vec3::new(1.0, 1.0, 1.0), 1)
    }

    fn used(whole: &Mesh, target: &Mesh) -> NoiseUsed {
        let (_, noise) = with_noise(whole, target, &SegmentOptions::default()).expect("valid options");
        noise.expect("the consolidation is on")
    }

    /// A selection too small to measure takes the whole mesh's estimate when the mesh is uniform.
    #[test]
    fn a_selection_too_small_to_measure_takes_a_uniform_mesh_estimate() {
        let whole = plate(0.0, 0.03, 1);
        let n = used(&whole, &tiny());
        assert_eq!(n.source, NoiseSource::EstimatedMesh);
        assert!((0.02..0.045).contains(&n.sigma), "{}", n.sigma);
    }

    /// ... and the fallback when the mesh's parts have very different noise: the two cannot be
    /// shown to agree.
    #[test]
    fn a_selection_too_small_to_measure_in_a_mixed_mesh_uses_the_fallback() {
        let mut whole = plate(0.0, 0.03, 1);
        whole.append(&plate(200.0, 0.005, 2));
        let n = used(&whole, &tiny());
        assert_eq!((n.source, n.sigma), (NoiseSource::Fallback, fallback_noise(0.2)));
    }

    /// A selection that can be measured is measured, whatever the rest of the mesh is like.
    #[test]
    fn a_measurable_selection_is_measured_in_a_mixed_mesh() {
        let selection = plate(0.0, 0.03, 1);
        let mut whole = selection.clone();
        whole.append(&plate(200.0, 0.005, 2));
        let n = used(&whole, &selection);
        assert_eq!(n.source, NoiseSource::Estimated);
        assert!((0.02..0.045).contains(&n.sigma), "{}", n.sigma);
    }
}
