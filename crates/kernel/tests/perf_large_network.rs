//! P9 performance: the stages of a large patch network (build, tessellation, volume and body info) at 1 000 ... 60 000 patches, with the fitted growth exponent (log-log slope).
//! Ignored by default (minutes): run with `--profile ci` or `--release`,
//! `cargo test --profile ci -p surfacecraft-kernel --test perf_large_network -- --ignored --nocapture`.
//! `SURFACECRAFT_PERF_PATCHES=1000,10000` picks the sizes (STEP is measured in the engine's
//! `p9_perf`).
mod common;

use common::{cube_sphere, n_for};
use std::time::Instant;
use surfacecraft_kernel::nurbs::sheet_from_bezier_network_as_is;
use surfacecraft_kernel::{solid, tess};

const RADIUS: f64 = 50.0;
/// The chord of `body.compare` at its default tolerance (0.1 mm × 0.05).
const CHORD: f64 = 0.005;
const DEFAULT_SIZES: [usize; 4] = [1_000, 4_000, 16_000, 60_000];

fn sizes() -> Vec<usize> {
    std::env::var("SURFACECRAFT_PERF_PATCHES")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| DEFAULT_SIZES.to_vec())
}

/// Least-squares slope of ln(time) against ln(size).
fn exponent(points: &[(f64, f64)]) -> f64 {
    let pts: Vec<(f64, f64)> = points.iter().filter(|p| p.1 > 0.0).map(|&(n, t)| (n.ln(), t.ln())).collect();
    let k = pts.len() as f64;
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (sxx, sxy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0 * p.0, a.1 + p.0 * p.1));
    (k * sxy - sx * sy) / (k * sxx - sx * sx)
}

fn timed<T>(f: impl FnOnce() -> T) -> (T, f64) {
    let t = Instant::now();
    let r = f();
    (r, t.elapsed().as_secs_f64())
}

#[test]
#[ignore = "minutes of CPU; run explicitly with --profile ci or --release"]
fn perf_large_network_stages() {
    let mut rows: Vec<[f64; 5]> = Vec::new();
    // the first volume of a process builds the kernel's tables: not a cost of any size
    solid::volume(&solid::make_box([0.0; 3], [1.0; 3]).unwrap()).unwrap();
    for count in sizes() {
        let net = cube_sphere(n_for(count), RADIUS);
        let n = net.patches.len();
        let (sheet, build) = timed(|| sheet_from_bezier_network_as_is(&net.patches, &net.corners).unwrap());
        let (topo, tessellate) = timed(|| tess::tessellate_topo(&sheet, Some(CHORD)).unwrap());
        let (volume, t_volume) = timed(|| solid::volume(&sheet).unwrap());
        let open = sheet_from_bezier_network_as_is(&net.patches[1..], &net.corners[1..]).unwrap();
        let (info, t_info) = timed(|| solid::info(&open));
        eprintln!(
            "PERF patches {n}: build {build:.2} s, tessellate {tessellate:.2} s ({} tris), volume {t_volume:.2} s \
             ({volume:.3}), info of the open sheet {t_info:.2} s ({:?})",
            topo.mesh.triangle_count(),
            info.volume
        );
        rows.push([n as f64, build, tessellate, t_volume, t_info]);
    }
    for (k, name) in ["build", "tessellate", "volume", "info of an open sheet"].iter().enumerate() {
        let pts: Vec<(f64, f64)> = rows.iter().map(|r| (r[0], r[k + 1])).collect();
        eprintln!("PERF exponent {name}: {:.2}", exponent(&pts));
    }
}
