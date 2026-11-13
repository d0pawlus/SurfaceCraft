//! Tests of the chain fit: entities recovered from synthetic sections and hostile inputs.

use super::primitive::*;
use super::types::*;
use super::*;

use crate::profiles;
use surfacecraft_geom::Rng;

enum Seg {
    Line([f64; 2], [f64; 2]),
    /// Centre, radius, start and end angle (degrees).
    Arc([f64; 2], f64, f64, f64),
}

/// Samples a path of segments about `step` mm apart.
fn sample(segs: &[Seg], step: f64) -> Vec<P2> {
    let mut out: Vec<P2> = Vec::new();
    for s in segs {
        match s {
            Seg::Line(a, b) => {
                let n = (dist(*a, *b) / step).ceil().max(1.0) as usize;
                for k in 0..n {
                    let f = k as f64 / n as f64;
                    out.push([a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]);
                }
            }
            Seg::Arc(c, r, a0, a1) => {
                let n = ((a1 - a0).abs().to_radians() * r / step).ceil().max(1.0) as usize;
                for k in 0..n {
                    let a = (a0 + (a1 - a0) * k as f64 / n as f64).to_radians();
                    out.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
                }
            }
        }
    }
    out
}

/// Rotates by `deg` about the origin, moves by `t` and adds Gaussian noise.
fn scan(pts: &[P2], deg: f64, t: P2, sigma: f64, seed: u64) -> Vec<P2> {
    let mut rng = Rng::new(seed);
    let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
    pts.iter()
        .map(|p| [c * p[0] - s * p[1] + t[0] + sigma * rng.gauss(), s * p[0] + c * p[1] + t[1] + sigma * rng.gauss()])
        .collect()
}

fn rounded_rect(w: f64, h: f64, r: f64) -> Vec<Seg> {
    vec![
        Seg::Line([r, 0.0], [w - r, 0.0]),
        Seg::Arc([w - r, r], r, -90.0, 0.0),
        Seg::Line([w, r], [w, h - r]),
        Seg::Arc([w - r, h - r], r, 0.0, 90.0),
        Seg::Line([w - r, h], [r, h]),
        Seg::Arc([r, h - r], r, 90.0, 180.0),
        Seg::Line([0.0, h - r], [0.0, r]),
        Seg::Arc([r, r], r, 180.0, 270.0),
    ]
}

fn radii(s: &Sketch) -> Vec<f64> {
    s.entities
        .iter()
        .filter_map(|e| match e {
            Entity::Arc { center, a, .. } => Some(dist(s.points[*center], s.points[*a])),
            Entity::Circle { radius, .. } => Some(*radius),
            Entity::Line { .. } => None,
        })
        .collect()
}

fn count(r: &ChainReport, k: RelationKind) -> usize {
    r.relations_kept.iter().filter(|x| x.kind == k).count()
}

fn fit(pts: Vec<P2>, closed: bool) -> ChainsFit {
    fit_chains(&SketchPlane::default(), &[Chain2 { points: pts, closed }], &ChainOptions::default())
}

#[test]
fn rounded_rectangle_noisy_and_rotated() {
    let pts = scan(&sample(&rounded_rect(40.0, 20.0, 5.0), 0.25), 0.7, [12.3, -4.5], 0.02, 11);
    let f = fit(pts, true);
    let r = &f.chains[0];
    assert_eq!((r.lines, r.arcs, r.circles), (4, 4, 0), "{r:?}");
    assert!((f.frame_rotation_degrees - 0.7).abs() < 0.1, "{}", f.frame_rotation_degrees);
    assert_eq!(count(r, RelationKind::Horizontal), 2, "{:?}", r.relations_kept);
    assert_eq!(count(r, RelationKind::Vertical), 2);
    assert_eq!(count(r, RelationKind::Tangent), 8, "{:#?}", r);
    assert_eq!(count(r, RelationKind::EqualRadius), 3, "{:?}", r.relations_kept);
    assert_ne!(r.status, Some(SolveStatus::OverDefined));
    assert_ne!(f.report.status, SolveStatus::OverDefined);
    for rad in radii(&f.sketch) {
        assert!((rad - 5.0).abs() < 0.05, "{rad}");
    }
    let rr = radii(&f.sketch);
    assert!(rr.iter().all(|x| (x - rr[0]).abs() < 1e-6), "{rr:?}");
    assert!(r.after.rms <= 0.05 && r.after.max <= 0.15, "{:?}", r.after);
    assert_eq!(profiles(&f.sketch, 1e-6).len(), 1);
}

#[test]
fn slot_d_shape_and_open_chain() {
    // slot: two lines and two half circles
    let slot = [
        Seg::Line([-15.0, -5.0], [15.0, -5.0]),
        Seg::Arc([15.0, 0.0], 5.0, -90.0, 90.0),
        Seg::Line([15.0, 5.0], [-15.0, 5.0]),
        Seg::Arc([-15.0, 0.0], 5.0, 90.0, 270.0),
    ];
    let f = fit(scan(&sample(&slot, 0.25), 0.7, [3.0, 2.0], 0.02, 5), true);
    let r = &f.chains[0];
    assert_eq!((r.lines, r.arcs), (2, 2), "{r:?}");
    assert_eq!(count(r, RelationKind::Tangent), 4, "{:#?}", r);
    assert_ne!(r.status, Some(SolveStatus::OverDefined));
    for rad in radii(&f.sketch) {
        assert!((rad - 5.0).abs() < 0.05, "{rad}");
    }
    // D: an arc of 253.7 degrees and its chord
    let a0 = 90.0 + 53.13;
    let d = [
        Seg::Arc([0.0, 0.0], 10.0, a0, a0 + 253.74),
        Seg::Line(
            [10.0 * (a0 + 253.74).to_radians().cos(), 10.0 * (a0 + 253.74).to_radians().sin()],
            [10.0 * a0.to_radians().cos(), 10.0 * a0.to_radians().sin()],
        ),
    ];
    let f = fit(scan(&sample(&d, 0.25), 0.7, [0.0, 0.0], 0.02, 6), true);
    let r = &f.chains[0];
    assert_eq!((r.lines, r.arcs), (1, 1), "{r:?}");
    assert_eq!(count(r, RelationKind::Vertical) + count(r, RelationKind::Horizontal), 1);
    assert!((radii(&f.sketch)[0] - 10.0).abs() < 0.05);
    assert_ne!(r.status, Some(SolveStatus::OverDefined));
    // an open chain: line, quarter arc, line
    let l = [
        Seg::Line([0.0, 0.0], [20.0, 0.0]),
        Seg::Arc([20.0, 5.0], 5.0, -90.0, 0.0),
        Seg::Line([25.0, 5.0], [25.0, 25.0]),
    ];
    let f = fit(scan(&sample(&l, 0.25), 0.7, [0.0, 0.0], 0.02, 7), false);
    let r = &f.chains[0];
    assert!(!r.closed);
    assert_eq!((r.lines, r.arcs), (2, 1), "{r:?}");
    assert_eq!(count(r, RelationKind::Tangent), 2);
    assert!(profiles(&f.sketch, 1e-6).is_empty(), "an open chain stays open");
    assert_eq!(f.sketch.points.len(), 4 + 1, "two lines + arc share their junctions, one centre");
}

/// The four gate shapes over many noise seeds, rotations and offsets: structure, radii, relations.
#[test]
fn shapes_are_recovered_for_every_seed() {
    let slot = [
        Seg::Line([-15.0, -5.0], [15.0, -5.0]),
        Seg::Arc([15.0, 0.0], 5.0, -90.0, 90.0),
        Seg::Line([15.0, 5.0], [-15.0, 5.0]),
        Seg::Arc([-15.0, 0.0], 5.0, 90.0, 270.0),
    ];
    let rect = rounded_rect(40.0, 20.0, 5.0);
    let (mut worst_dr, mut worst_after) = (0.0f64, 0.0f64);
    for seed in 0..40u64 {
        for (deg, t) in [(0.7, [12.3, -4.5]), (-0.9, [0.0, 0.0]), (90.6, [-30.0, 8.0]), (0.0, [1.0, 1.0])] {
            let f = fit(scan(&sample(&rect, 0.25), deg, t, 0.02, seed), true);
            let r = &f.chains[0];
            assert_eq!((r.lines, r.arcs), (4, 4), "seed {seed} {deg}: {r:#?}");
            assert_eq!(count(r, RelationKind::Horizontal) + count(r, RelationKind::Vertical), 4, "seed {seed} {deg}");
            assert_eq!(count(r, RelationKind::Tangent), 8, "seed {seed} {deg}: {r:#?}");
            assert_eq!(count(r, RelationKind::EqualRadius), 3, "seed {seed} {deg}");
            assert_ne!(r.status, Some(SolveStatus::OverDefined));
            for rad in radii(&f.sketch) {
                worst_dr = worst_dr.max((rad - 5.0).abs());
            }
            worst_after = worst_after.max(r.after.max);
            let f = fit(scan(&sample(&slot, 0.25), deg, t, 0.02, seed), true);
            let r = &f.chains[0];
            assert_eq!((r.lines, r.arcs), (2, 2), "seed {seed} {deg}: {r:#?}");
            assert_eq!(count(r, RelationKind::Tangent), 4, "seed {seed} {deg}: {r:#?}");
            assert_ne!(r.status, Some(SolveStatus::OverDefined));
            for rad in radii(&f.sketch) {
                worst_dr = worst_dr.max((rad - 5.0).abs());
            }
        }
    }
    eprintln!("worst radius error {worst_dr:.4} mm, worst max deviation after {worst_after:.4} mm");
    assert!(worst_dr < 0.05, "{worst_dr}");
}

#[test]
fn long_wavy_chain_is_fitted_but_not_related() {
    let pts: Vec<P2> = (0..3000)
        .map(|k| {
            let x = f64::from(k) * 0.02;
            [x, 5.0 * x.sin()]
        })
        .collect();
    let t = std::time::Instant::now();
    let f = fit(pts, false);
    let r = &f.chains[0];
    assert!(r.entities > MAX_RELATION_ENTITIES, "{}", r.entities);
    assert!(r.relations_kept.is_empty() && r.note.is_some());
    assert!(r.before.max <= 3.0 * 0.05 + 1e-9, "{:?}", r.before);
    assert!(t.elapsed().as_secs() < 20, "{:?}", t.elapsed());
}

#[test]
fn two_chains_share_one_sketch_with_shifted_indices() {
    let a = scan(&sample(&rounded_rect(40.0, 20.0, 5.0), 0.25), 0.0, [0.0, 0.0], 0.01, 1);
    let hole: Vec<P2> = (0..120)
        .map(|k| {
            let t = f64::from(k) / 120.0 * TAU;
            [20.0 + 4.0 * t.cos(), 10.0 + 4.0 * t.sin()]
        })
        .collect();
    let f = fit_chains(
        &SketchPlane::default(),
        &[Chain2 { points: a, closed: true }, Chain2 { points: hole, closed: true }],
        &ChainOptions::default(),
    );
    assert_eq!(f.chains.len(), 2);
    assert_eq!((f.chains[0].entities, f.chains[1].entities), (8, 1));
    assert_eq!(f.chains[1].circles, 1);
    assert_eq!(f.chains[1].first_entity, 8);
    assert!(f.sketch.validate().is_ok());
    assert_eq!(profiles(&f.sketch, 1e-6).len(), 2);
    assert_ne!(f.report.status, SolveStatus::OverDefined);
    // the sketch as a whole solves without conflicts
    let mut sk = f.sketch.clone();
    let rep = solve(&mut sk).unwrap();
    assert_ne!(rep.status, SolveStatus::OverDefined, "{rep:?}");
}

#[test]
fn random_chains_finish_quickly() {
    let mut rng = Rng::new(3);
    let noise: Vec<P2> = (0..20_000).map(|_| [rng.range(-50.0, 50.0), rng.range(-50.0, 50.0)]).collect();
    let walk: Vec<P2> = noise
        .iter()
        .scan([0.0, 0.0], |p, d| {
            *p = [p[0] + d[0] * 0.01, p[1] + d[1] * 0.01];
            Some(*p)
        })
        .collect();
    let t = std::time::Instant::now();
    for (pts, closed) in [(noise, true), (walk.clone(), false), (walk, true)] {
        let f = fit(pts, closed);
        assert_eq!(f.chains.len(), 1);
        assert!(f.chains[0].samples <= MAX_SAMPLES);
    }
    assert!(t.elapsed().as_secs() < 60, "{:?}", t.elapsed());
    eprintln!("random chains: {:?}", t.elapsed());
}

#[test]
fn hostile_chains_do_not_panic() {
    let plane = SketchPlane::default();
    let o = ChainOptions::default();
    let weird = vec![
        Chain2 { points: vec![], closed: true },
        Chain2 { points: vec![[0.0, 0.0]], closed: false },
        Chain2 { points: vec![[0.0, 0.0]; 50], closed: true },
        Chain2 { points: vec![[f64::NAN, 0.0], [1.0, 1.0], [2.0, 0.0]], closed: false },
        Chain2 { points: vec![[0.0, 0.0], [1e300, 1e300], [-1e300, 5.0], [3.0, 3.0], [4.0, 1e-300]], closed: true },
        Chain2 { points: (0..300).map(|k| [f64::from(k % 7), f64::from(k % 5)]).collect(), closed: true },
        Chain2 { points: (0..100).map(|k| [f64::from(k), 0.0]).collect(), closed: true },
    ];
    let f = fit_chains(&plane, &weird, &o);
    assert_eq!(f.chains.len(), weird.len());
    for t in [f64::NAN, -1.0, 0.0, f64::INFINITY, 1e300] {
        let o = ChainOptions { tolerance: t, angle_tolerance_degrees: t, align_frame: true };
        let _ = fit_chains(&plane, &weird, &o);
    }
}
