//! P8 (H2) measurements: STL import with `weld`, hole filling and decimation to a tolerance on
//! generated meshes. Ignored by default; run in release, one stage per process so the peak memory
//! of a stage can be read from the operating system (`/usr/bin/time -l` on macOS, `-v` on Linux):
//!
//! `cargo test --release -p surfacecraft-io --test h2_perf perf_weld -- --ignored --nocapture`
//!
//! `SURFACECRAFT_H2_MILLIONS` sets the size in millions of triangles (default 1);
//! `RAYON_NUM_THREADS` the thread count.
use std::time::Instant;
use surfacecraft_geom::reduce::{ToleranceOptions, reduce_to_tolerance};
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, edit, synth};
use surfacecraft_io::{Limits, MeshFormat, read_mesh, write_mesh};

fn millions() -> f64 {
    std::env::var("SURFACECRAFT_H2_MILLIONS").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0)
}

/// A bumpy, slightly noisy sphere of about `millions` × 10⁶ triangles.
fn organic(millions: f64) -> Mesh {
    let nv = ((millions * 1e6 / 4.0).sqrt()).round().max(8.0) as usize;
    let mut m = synth::sphere(Point3::origin(), 20.0, 2 * nv, nv);
    for p in &mut m.positions {
        let (th, ph) = (p.y.atan2(p.x), (p.z / 20.0).clamp(-1.0, 1.0).acos());
        let bump = 1.0 + 0.05 * (4.0 * th).sin() * (3.0 * ph).cos() + 0.02 * (9.0 * ph + th).sin();
        *p = Point3::from(p.coords * bump);
    }
    synth::add_noise(&mut m, 0.004, &mut Rng::new(7));
    m
}

fn mem_hint() {
    eprintln!("(peak memory: run this test under `/usr/bin/time -l`, line `maximum resident set size`)");
}

#[test]
#[ignore = "measurement; run in release"]
fn perf_stl_parse_and_weld() {
    let m = organic(millions());
    let bytes = write_mesh(&m, MeshFormat::Stl);
    let tris = m.triangle_count();
    drop(m);
    let t = Instant::now();
    let back = read_mesh(&bytes, MeshFormat::Stl, &Limits::default()).unwrap();
    eprintln!(
        "import (parse + weld) of {tris} triangles: {:.2} s, {} vertices",
        t.elapsed().as_secs_f64(),
        back.vertex_count()
    );
    mem_hint();
}

#[test]
#[ignore = "measurement; run in release"]
fn perf_weld() {
    let m = organic(millions());
    let mut soup = Mesh { positions: Vec::new(), triangles: Vec::new() };
    for t in &m.triangles {
        let i = soup.positions.len() as u32;
        soup.positions.extend(t.iter().map(|&v| m.positions[v as usize]));
        soup.triangles.push([i, i + 1, i + 2]);
    }
    drop(m);
    let n = soup.positions.len();
    let t = Instant::now();
    // SURFACECRAFT_H2_SKIP_WELD=1 builds the soup only: the difference of the peak memory of the two
    // runs is what the weld itself needs
    let removed = if std::env::var("SURFACECRAFT_H2_SKIP_WELD").is_ok() { 0 } else { soup.weld(0.0) };
    eprintln!("weld of {n} vertices: {:.2} s, removed {removed}", t.elapsed().as_secs_f64());
    mem_hint();
}

/// Removes the triangles around `count` seeds: holes of a few dozen edges.
fn punch(m: &Mesh, count: usize, radius: f64) -> Mesh {
    let mut rng = Rng::new(11);
    let seeds: Vec<Point3> = (0..count).map(|_| m.positions[rng.index(m.positions.len())]).collect();
    let keep: Vec<[u32; 3]> = m
        .triangles
        .iter()
        .copied()
        .filter(|t| {
            let c = (m.positions[t[0] as usize].coords
                + m.positions[t[1] as usize].coords
                + m.positions[t[2] as usize].coords)
                / 3.0;
            seeds.iter().all(|s| (c - s.coords).norm() > radius)
        })
        .collect();
    let mut out = Mesh { positions: m.positions.clone(), triangles: keep };
    edit::compact(&mut out);
    out
}

#[test]
#[ignore = "measurement; run in release"]
fn perf_fill_holes_many() {
    let m = organic(millions());
    let mut holed = punch(&m, 300, 0.6);
    drop(m);
    let loops = edit::boundary_loops(&holed).len();
    let t = Instant::now();
    let rep = edit::fill_holes(&mut holed, 4000, true);
    eprintln!("fill {loops} holes (smooth): {:.2} s, {rep:?}", t.elapsed().as_secs_f64());
}

/// A strip (0.1 mm grid, 5000 × 200 cells) with one hole `columns` wide and 120 rows high.
fn strip_with_hole(columns: usize) -> Mesh {
    let strip = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 500.0, 20.0, 5000, 200);
    let (x0, x1) = (100.0, 100.0 + columns as f64 * 0.1);
    let keep: Vec<[u32; 3]> = strip
        .triangles
        .iter()
        .copied()
        .filter(|t| {
            let c = (strip.positions[t[0] as usize].coords
                + strip.positions[t[1] as usize].coords
                + strip.positions[t[2] as usize].coords)
                / 3.0;
            !(c.x > x0 && c.x < x1 && c.y > 4.0 && c.y < 16.0)
        })
        .collect();
    Mesh { positions: strip.positions, triangles: keep }
}

/// `SURFACECRAFT_H2_COLUMNS` (comma separated) sets the widths of the hole in columns of 0.1 mm;
/// the loop has about 2 × (columns + 120) edges. The rim of the strip (10 400 edges) is not filled.
#[test]
#[ignore = "measurement; run in release"]
fn perf_fill_hole_big() {
    let widths: Vec<usize> = std::env::var("SURFACECRAFT_H2_COLUMNS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![2500]);
    let smooth_too = std::env::var("SURFACECRAFT_H2_SMOOTH").is_ok();
    for columns in widths {
        let holed = strip_with_hole(columns);
        for smooth in [false, true].into_iter().filter(|s| !s || smooth_too) {
            let mut m = holed.clone();
            let longest = edit::boundary_loops(&m).iter().map(Vec::len).filter(|&n| n < 10_400).max().unwrap_or(0);
            let t = Instant::now();
            let max = std::env::var("SURFACECRAFT_H2_MAX_EDGES").ok().and_then(|v| v.parse().ok()).unwrap_or(9000);
            let rep = edit::fill_holes(&mut m, max, smooth);
            eprintln!("hole of {longest} edges, smooth {smooth}: {:.2} s, {rep:?}", t.elapsed().as_secs_f64());
        }
    }
}

#[test]
#[ignore = "measurement; run in release"]
fn perf_decimate_tolerance() {
    let mut m = organic(millions());
    let before = m.triangle_count();
    let t = Instant::now();
    let rep = reduce_to_tolerance(&mut m, &edit::DecimateOptions::default(), &ToleranceOptions::new(0.05)).unwrap();
    eprintln!(
        "decimate {before} triangles to tolerance 0.05: {:.2} s, {} left, {rep:?}",
        t.elapsed().as_secs_f64(),
        m.triangle_count()
    );
}
