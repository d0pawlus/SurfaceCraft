//! Noise of a scan: the median-based estimate of its standard deviation from the mesh.

use super::*;

/// Noise estimate of a scan (σ, mm); see [`noise_estimate`].
#[must_use]
pub fn estimate_noise(mesh: &Mesh) -> f64 {
    noise_estimate(mesh).sigma
}

/// A noise estimate, and whether it rests on smooth samples only.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NoiseEstimate {
    /// Estimated σ (mm); zero when there is nothing to measure.
    pub sigma: f64,
    /// False when too few smooth samples remained, so the estimate includes samples near sharp
    /// edges and is inflated (a coarse mesh of a box, for example).
    pub smooth: bool,
}

/// Noise estimate of a scan (σ, mm): median distance of vertices to a local quadric through
/// their neighbours, scaled to a standard deviation (design doc §10: warn when a tolerance is
/// below the scanner noise). Samples whose neighbourhood spans a sharp edge (neighbour normals
/// more than 30° off the local plane) are left out when enough smooth samples remain, so edges
/// and corners do not inflate the estimate.
#[must_use]
pub fn noise_estimate(mesh: &Mesh) -> NoiseEstimate {
    let n = mesh.vertex_count();
    if n < 16 {
        return NoiseEstimate { sigma: 0.0, smooth: false };
    }
    let tree = KdTree::build(&mesh.positions);
    let stride = n.div_ceil(4000).max(1);
    let normals = mesh.vertex_normals();
    let cos_sharp = 30f64.to_radians().cos();
    let mut d: Vec<f64> = Vec::new();
    let mut d_smooth: Vec<f64> = Vec::new();
    for p in mesh.positions.iter().step_by(stride) {
        let near = tree.knn(p, 17);
        // neighbours without the point itself (leave-one-out), local quadric height field
        let nb: Vec<Point3> =
            near.iter().filter(|(_, d2)| *d2 > 0.0).filter_map(|(i, _)| mesh.positions.get(*i).copied()).collect();
        let Some(f) = surfacecraft_fit::plane::fit_plane(&nb, None) else { continue };
        let z = f.normal;
        let sharp =
            near.iter().filter_map(|(i, _)| normals.get(*i)).any(|nn| nn.norm() > 0.5 && nn.dot(&z).abs() < cos_sharp);
        let x = surfacecraft_geom::any_perpendicular(&z);
        let y = z.cross(&x);
        let o = f.point.coords;
        let mut ata = nalgebra::Matrix6::<f64>::zeros();
        let mut atb = nalgebra::Vector6::<f64>::zeros();
        for q in &nb {
            let w = q.coords - o;
            let (u, v, h) = (w.dot(&x), w.dot(&y), w.dot(&z));
            let row = nalgebra::Vector6::new(u * u, u * v, v * v, u, v, 1.0);
            ata += row * row.transpose();
            atb += row * h;
        }
        let Some(c) = ata.lu().solve(&atb) else { continue };
        let w = p.coords - o;
        let (u, v, h) = (w.dot(&x), w.dot(&y), w.dot(&z));
        let pred = c[0] * u * u + c[1] * u * v + c[2] * v * v + c[3] * u + c[4] * v + c[5];
        d.push((h - pred).abs());
        if !sharp {
            d_smooth.push((h - pred).abs());
        }
    }
    let smooth = d_smooth.len() >= 50 && d_smooth.len() * 5 >= d.len();
    if smooth {
        d = d_smooth;
    }
    if d.is_empty() {
        return NoiseEstimate { sigma: 0.0, smooth };
    }
    d.sort_by(f64::total_cmp);
    // median |x| of a normal variable is 0.6745 σ; the prediction's own error adds ≈ 10 %
    let sigma = d.get(d.len() / 2).copied().unwrap_or(0.0) / 0.6745 / 1.1;
    NoiseEstimate { sigma, smooth }
}
