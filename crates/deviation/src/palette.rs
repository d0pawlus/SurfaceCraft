//! Colour map of the deviation: the palette constants, the linear interpolation and the colour per value.

use super::*;

/// Within ±tolerance (green).
pub const IN_TOLERANCE: [f32; 4] = [0.2, 0.8, 0.3, 1.0];
/// Just above +tolerance: the start of the above-ramp (yellow-orange).
pub const ABOVE_NEAR: [f32; 4] = [1.0, 0.85, 0.1, 1.0];
/// At +3·tolerance and beyond: the end of the above-ramp (red).
pub const ABOVE_FAR: [f32; 4] = [1.0, 0.0, 0.1, 1.0];
/// Just below −tolerance: the start of the below-ramp (cyan).
pub const BELOW_NEAR: [f32; 4] = [0.1, 0.75, 1.0, 1.0];
/// At −3·tolerance and beyond: the end of the below-ramp (blue).
pub const BELOW_FAR: [f32; 4] = [0.1, 0.0, 1.0, 1.0];
/// Points that were not measured (NaN, farther than the reference's reach).
pub const UNMEASURED: [f32; 4] = [0.45, 0.45, 0.45, 1.0];

/// Linear interpolation of two RGBA colours (`k` in 0..=1).
fn lerp(a: [f32; 4], b: [f32; 4], k: f32) -> [f32; 4] {
    [a[0] + (b[0] - a[0]) * k, a[1] + (b[1] - a[1]) * k, a[2] + (b[2] - a[2]) * k, 1.0]
}

/// Colour per value: green within ±tol, through yellow/orange to red above, through cyan to
/// blue below; grey for NaN (not measured).
#[must_use]
pub fn colorize(values: &[f64], tolerance: f64) -> Vec<[f32; 4]> {
    let tol = if tolerance.is_finite() && tolerance > 0.0 { tolerance } else { 0.1 };
    values
        .iter()
        .map(|&d| {
            if !d.is_finite() {
                return palette::UNMEASURED;
            }
            let t = (d / tol) as f32;
            if t.abs() <= 1.0 {
                palette::IN_TOLERANCE
            } else if t > 0.0 {
                let k = ((t - 1.0) / 2.0).clamp(0.0, 1.0);
                lerp(palette::ABOVE_NEAR, palette::ABOVE_FAR, k)
            } else {
                let k = ((-t - 1.0) / 2.0).clamp(0.0, 1.0);
                lerp(palette::BELOW_NEAR, palette::BELOW_FAR, k)
            }
        })
        .collect()
}
