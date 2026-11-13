//! Decimation: quadric error metric edge collapse (Garland & Heckbert 1997) with boundary and
//! feature-edge constraint planes, link-condition and normal-flip checks, optionally guarded by
//! a measured deviation from the original surface. The collapse loop is in [`crate::collapse`];
//! with a guard a large mesh is first reduced in independent parts (see [`crate::partition`]).
use crate::collapse::{Collapser, Quadric, plane_quadric, vertex_origins};
use crate::edit::compact;
use crate::{Bvh, Mesh, Point3, Vec3, tri};
use std::collections::HashMap;

/// Decimation settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecimateOptions {
    /// Stop at this many triangles.
    pub target_triangles: usize,
    /// Never move the surface more than this (mm, approximate quadric distance); `None` = no limit.
    pub max_error: Option<f64>,
    /// Edges with a dihedral angle above this are kept sharp (degrees).
    pub feature_angle_deg: f64,
    /// Keep boundary edges in place.
    pub preserve_boundary: bool,
}

impl Default for DecimateOptions {
    fn default() -> Self {
        Self { target_triangles: 0, max_error: None, feature_angle_deg: 40.0, preserve_boundary: true }
    }
}

/// Quadric edge-collapse decimation; returns the number of triangles removed.
pub fn decimate(mesh: &mut Mesh, opts: &DecimateOptions) -> usize {
    decimate_guarded(mesh, opts, None).removed
}

/// What a decimation did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct DecimateReport {
    /// Triangles removed.
    pub removed: usize,
    /// Collapses refused because they would have turned a triangle against the surface it came
    /// from (attempts; an edge tried again counts again). A reduction that stops above its target
    /// because of them is what keeps thin walls and sharp ridges from being folded over.
    pub turns_refused: usize,
}

/// [`decimate`] with a report of what stood in the way of the target.
pub fn decimate_with_report(mesh: &mut Mesh, opts: &DecimateOptions) -> DecimateReport {
    decimate_guarded(mesh, opts, None)
}

/// Measured guard of a collapse: the reduction of [`crate::reduce`] checks every collapse against
/// the original surface instead of trusting the quadric alone (which overestimates the error of noisy
/// data and ignores the sagitta inside a triangle).
pub(crate) struct Guard<'a> {
    /// The original surface.
    pub bvh: &'a Bvh,
    /// Per-vertex error budget (mm), indexed by the vertices of the mesh before the call; a collapse
    /// is allowed only when the measured deviation is within the smaller budget of its two vertices,
    /// and the merged vertex keeps that smaller budget. Its content is undefined after the call.
    pub budget: &'a mut Vec<f64>,
}

/// With a guard, meshes with at least this many triangles are reduced in parts first.
const PARTS_FROM_TRIANGLES: usize = 100_000;

/// [`decimate`] with an optional measured [`Guard`]; without one it is [`decimate`].
pub(crate) fn decimate_guarded(
    mesh: &mut Mesh,
    opts: &DecimateOptions,
    mut guard: Option<Guard<'_>>,
) -> DecimateReport {
    if guard.as_ref().is_some_and(|g| g.budget.len() != mesh.positions.len()) {
        return DecimateReport::default();
    }
    if mesh.validate().is_err() || mesh.triangles.len() <= opts.target_triangles.max(4) {
        return DecimateReport::default();
    }
    let before = mesh.triangles.len();
    let mut tris = std::mem::take(&mut mesh.triangles);
    let mut pos = std::mem::take(&mut mesh.positions);
    let (mut q, mut locked) = setup(&tris, &pos, opts);
    let mut origin = vertex_origins(&tris, &pos);
    let mut refused = 0;
    let max_cost = opts.max_error.filter(|e| e.is_finite() && *e > 0.0).map(|e| e * e);
    if let Some(g) = guard.as_mut()
        && tris.len() >= PARTS_FROM_TRIANGLES
    {
        let whole = crate::partition::Whole { tris, pos, q, origin, locked, refused: 0 };
        let done = crate::partition::reduce_in_parts(whole, g, opts, max_cost);
        (tris, pos, q, origin, locked, refused) = (done.tris, done.pos, done.q, done.origin, done.locked, done.refused);
    }
    let mut c = Collapser::new(tris, pos, q, origin, locked, guard, true);
    c.run(opts.target_triangles, max_cost);
    refused += c.turns_refused();
    let (tris, pos, ..) = c.finish();
    mesh.positions = pos;
    mesh.triangles = tris;
    compact(mesh);
    DecimateReport { removed: before - mesh.triangles.len(), turns_refused: refused }
}

/// The vertex quadrics (face planes, plus constraint planes along boundary and feature edges) and
/// the vertices that must not move (on an edge of three or more faces).
fn setup(tris: &[[u32; 3]], pos: &[Point3], opts: &DecimateOptions) -> (Vec<Quadric>, Vec<bool>) {
    let nv = pos.len();
    let mut q = vec![Quadric::zeros(); nv];
    let mut fnormal: Vec<Vec3> = Vec::with_capacity(tris.len());
    for t in tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let n = crate::normalize(tri::cross(&a, &b, &c)).unwrap_or_else(Vec3::zeros);
        fnormal.push(n);
        let k = plane_quadric(&n, &a, 1.0);
        for &v in t {
            q[v as usize] += k;
        }
    }
    let cos_feat = opts.feature_angle_deg.clamp(1.0, 179.0).to_radians().cos();
    let mut edge_faces: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(tris.len() * 3 / 2);
    for (i, t) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edge_faces.entry((a.min(b), a.max(b))).or_default().push(i as u32);
        }
    }
    let mut locked = vec![false; nv];
    let mut edge_list: Vec<(&(u32, u32), &Vec<u32>)> = edge_faces.iter().collect();
    edge_list.sort_unstable_by_key(|(k, _)| **k);
    for (&(a, b), fs) in edge_list {
        let (pa, pb) = (pos[a as usize], pos[b as usize]);
        let e = pb - pa;
        let len2 = e.norm_squared();
        let boundary = fs.len() == 1;
        let feature = fs.len() == 2 && fnormal[fs[0] as usize].dot(&fnormal[fs[1] as usize]) < cos_feat;
        if fs.len() > 2 {
            locked[a as usize] = true;
            locked[b as usize] = true;
            continue;
        }
        if (boundary && opts.preserve_boundary) || feature {
            for &f in fs {
                if let Some(side) = crate::normalize(e.cross(&fnormal[f as usize])) {
                    let k = plane_quadric(&side, &pa, 1000.0 * len2.max(1e-12));
                    q[a as usize] += k;
                    q[b as usize] += k;
                }
            }
        }
    }
    (q, locked)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, synth};

    const SEEDS: u64 = 20;
    /// A guarded reduction may stop above its target, but not by more than this factor.
    const MAX_OVERSHOOT: usize = 2;

    /// What a collapse loop left of a mesh.
    struct Run {
        /// Triangles left.
        reached: usize,
        /// Of those, how many face away from the original orientation of their corners.
        turned: usize,
        /// Collapses refused for turning a triangle against the surface.
        refused: usize,
    }

    /// Collapses `m` with the loop down to `target` triangles (and within `max_error`, if given).
    fn collapse(m: &Mesh, target: usize, max_error: Option<f64>, guard: bool) -> Run {
        let opts = DecimateOptions { target_triangles: target, feature_angle_deg: 181.0, ..DecimateOptions::default() };
        let (q, locked) = setup(&m.triangles, &m.positions, &opts);
        let origin = vertex_origins(&m.triangles, &m.positions);
        let mut c = Collapser::new(m.triangles.clone(), m.positions.clone(), q, origin, locked, None, true);
        c.orient_guard = guard;
        c.run(target, max_error.map(|e| e * e));
        let refused = c.turns_refused();
        let (tris, pos, _, origin) = c.finish();
        let turned = tris
            .iter()
            .filter(|t| {
                let n = tri::cross(&pos[t[0] as usize], &pos[t[1] as usize], &pos[t[2] as usize]);
                let r: Vec3 = t.iter().map(|&v| origin[v as usize]).sum();
                n.dot(&r) <= 0.0
            })
            .count();
        Run { reached: tris.len(), turned, refused }
    }

    /// A noisy plate, much thinner than it is wide: two faces 1 mm apart.
    fn plate(seed: u64, noise: f64) -> Mesh {
        let mut m = synth::rounded_cuboid(Point3::origin(), Vec3::new(40.0, 40.0, 1.0), 0.4, 30);
        synth::add_noise(&mut m, noise, &mut Rng::new(seed));
        m
    }

    #[test]
    fn a_reduction_never_returns_a_triangle_turned_against_the_surface() {
        for target in [4, 12, 60] {
            let turned: usize = (1..=SEEDS).map(|seed| collapse(&plate(seed, 0.05), target, None, true).turned).sum();
            assert_eq!(turned, 0, "{SEEDS} seeds reduced to {target} triangles");
        }
    }

    /// The guard must not be a way of refusing everything: the plate still gets (nearly) down to its
    /// target. Measured: the target exactly at 12 and 60 triangles for all 20 seeds, and at most 12
    /// triangles when asked for 4.
    #[test]
    fn the_guard_still_lets_the_reduction_reach_its_target() {
        for target in [12, 60] {
            for seed in 1..=SEEDS {
                let run = collapse(&plate(seed, 0.05), target, None, true);
                assert!(
                    run.reached <= MAX_OVERSHOOT * target,
                    "seed {seed}: {} triangles for a target of {target}",
                    run.reached
                );
            }
        }
    }

    /// The test above is only worth something if the guard does refuse collapses, and the plain
    /// reductions do turn triangles without it.
    #[test]
    fn without_the_guard_the_same_reductions_do_turn_triangles_and_with_it_collapses_are_refused() {
        let (mut turned, mut refused_with, mut refused_without) = (0, 0, 0);
        for target in [4, 12, 60] {
            for seed in 1..=SEEDS {
                let m = plate(seed, 0.05);
                let off = collapse(&m, target, None, false);
                turned += off.turned;
                refused_without += off.refused;
                refused_with += collapse(&m, target, None, true).refused;
            }
        }
        assert!(turned > 0, "the plates never needed the guard");
        assert!(refused_with > 0, "the guard never refused a collapse");
        assert_eq!(refused_without, 0);
    }

    /// The price of the guard on thin and sharp shapes: triangles left for the same error budget,
    /// guarded against unguarded. Measured over the seeds and printed; asserted within a factor.
    fn cost_of_the_guard(shape: impl Fn(u64) -> Mesh, max_error: f64) -> f64 {
        let (mut with, mut without) = (0, 0);
        for seed in 1..=SEEDS {
            let m = shape(seed);
            with += collapse(&m, 0, Some(max_error), true).reached;
            without += collapse(&m, 0, Some(max_error), false).reached;
        }
        let factor = with as f64 / without as f64;
        eprintln!("guarded {with} against unguarded {without} triangles over {SEEDS} seeds: factor {factor:.2}");
        factor
    }

    #[test]
    fn on_a_thin_closed_plate_the_guard_costs_less_than_a_factor_of_two_in_triangles() {
        let factor = cost_of_the_guard(|seed| plate(seed, 0.03), 0.3);
        assert!(factor <= MAX_OVERSHOOT as f64, "factor {factor}");
    }

    #[test]
    fn on_a_sharp_ridge_the_guard_costs_less_than_a_factor_of_two_in_triangles() {
        let ridge = |seed: u64| {
            let mut m = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 20);
            synth::add_noise(&mut m, 0.03, &mut Rng::new(seed));
            m
        };
        let factor = cost_of_the_guard(ridge, 0.3);
        assert!(factor <= MAX_OVERSHOOT as f64, "factor {factor}");
    }
}
