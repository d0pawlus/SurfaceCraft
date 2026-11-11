//! One region: a seed patch, its fit, and the growth from it (the unit of work of segmentation).
//!
//! A job reads the owner array and nothing else that changes while regions are being found, so
//! several jobs can run at once on the same snapshot of it (see `seeds`).
use super::fits::{fit_kind_tris, fit_tris};
use crate::primitive::{Primitive, PrimitiveFit};
use std::collections::{HashSet, VecDeque};
use surfacecraft_geom::{Mesh, Vec3};

/// Owner of a triangle that no region has claimed.
pub(super) const UNOWNED: u32 = u32::MAX;
/// Fewest triangles of a seed patch that is fitted.
const MIN_PATCH: usize = 8;
/// A patch stops growing at this many seed sizes of triangles, whatever its area.
const PATCH_SIZE_LIMIT: usize = 8;
/// Regions up to this many seed sizes of triangles may still change kind when they are refitted.
const SMALL_REGION: usize = 20;
/// Smoothness of a patch: the least dot product of the normals of neighbouring triangles in it.
const PATCH_MIN_DOT: f64 = 0.94;

/// Does triangle `t` belong to the region of `prim`: corners within `tol`, normal within `cos`.
pub(super) fn accepts(mesh: &Mesh, fnorm: &[Option<Vec3>], prim: &Primitive, t: usize, tol: f64, cos: f64) -> bool {
    let Some(corners) = mesh.triangle(t) else { return false };
    if corners.iter().any(|c| prim.distance(c).abs() > tol) {
        return false;
    }
    let (Some(n), Some(c)) = (fnorm.get(t).copied().flatten(), mesh.face_centroid(t)) else { return false };
    prim.normal(&c).dot(&n).abs() >= cos
}

/// What a job reads: the mesh, its normals and neighbours, and the growth settings.
pub(super) struct Ctx<'a> {
    pub(super) mesh: &'a Mesh,
    pub(super) normals: &'a [Vec3],
    pub(super) nb: &'a [[u32; 3]],
    pub(super) fnorm: &'a [Option<Vec3>],
    pub(super) areas: &'a [f64],
    pub(super) seed_tris: usize,
    pub(super) seed_area: f64,
    pub(super) tol: f64,
    pub(super) cos: f64,
}

/// What came of one seed.
pub(super) enum Outcome {
    /// The patch was too small to fit.
    Tiny,
    /// The seed fit failed or was not within the tolerance.
    Rejected,
    /// The region that grew (unsorted) with its fit; whether it is big enough is the caller's call.
    Grown { region: Vec<u32>, fit: PrimitiveFit },
    /// The region outgrew the size a speculative job may reach (the result is not usable).
    TooBig,
}

/// A seed's patch and what grew from it.
pub(super) struct Job {
    pub(super) patch: Vec<u32>,
    pub(super) outcome: Outcome,
}

/// A set of triangles as bits (one allocation of `n / 8` bytes, zeroed by the allocator).
struct Bits(Vec<u64>);

impl Bits {
    fn new(n: usize) -> Self {
        Self(vec![0; n.div_ceil(64)])
    }

    fn contains(&self, i: u32) -> bool {
        self.0.get(i as usize / 64).is_some_and(|w| w >> (i % 64) & 1 == 1)
    }

    fn insert(&mut self, i: u32) {
        if let Some(w) = self.0.get_mut(i as usize / 64) {
            *w |= 1 << (i % 64);
        }
    }
}

impl Ctx<'_> {
    /// Seed patch: the smooth unowned neighbourhood of `seed`, breadth first, up to the seed
    /// area. Depends only on which triangles are owned.
    pub(super) fn patch(&self, seed: u32, owner: &[u32]) -> Vec<u32> {
        let mut patch = vec![seed];
        let mut seen: HashSet<u32> = HashSet::from([seed]);
        let mut area = self.areas.get(seed as usize).copied().unwrap_or(0.0);
        let mut q = VecDeque::from([seed]);
        while let Some(t) = q.pop_front() {
            let full = patch.len() >= self.seed_tris && area >= self.seed_area;
            if full || patch.len() >= PATCH_SIZE_LIMIT * self.seed_tris {
                break;
            }
            let Some(n0) = self.fnorm.get(t as usize).copied().flatten() else { continue };
            for &o in self.nb.get(t as usize).into_iter().flatten() {
                if o == UNOWNED || owner.get(o as usize) != Some(&UNOWNED) || seen.contains(&o) {
                    continue;
                }
                if self.fnorm.get(o as usize).copied().flatten().is_some_and(|m| m.dot(&n0) > PATCH_MIN_DOT) {
                    seen.insert(o);
                    patch.push(o);
                    area += self.areas.get(o as usize).copied().unwrap_or(0.0);
                    q.push_back(o);
                }
            }
        }
        patch
    }

    /// Patch, seed fit, growth and the refits of the growth for `seed` against `owner`. With
    /// `cap`, growth gives up at that many triangles ([`Outcome::TooBig`]).
    pub(super) fn run(&self, seed: u32, owner: &[u32], cap: Option<usize>) -> Job {
        let patch = self.patch(seed, owner);
        if patch.len() < MIN_PATCH {
            return Job { patch, outcome: Outcome::Tiny };
        }
        crate::stats::seed_fit();
        let sw = crate::stats::Stopwatch::start();
        let seed_fit = fit_tris(self.mesh, self.normals, &patch);
        sw.stop(crate::stats::Phase::Seed);
        let outcome = match seed_fit {
            Some(fit) if fit.rms <= self.tol => self.grow(&patch, fit, owner, cap),
            _ => Outcome::Rejected,
        };
        Job { patch, outcome }
    }

    /// Breadth-first growth across neighbours that the primitive accepts, refitting each time the
    /// region doubles.
    fn grow(&self, patch: &[u32], mut fit: PrimitiveFit, owner: &[u32], cap: Option<usize>) -> Outcome {
        let sw = crate::stats::Stopwatch::start();
        let mut seen = Bits::new(self.nb.len());
        let mut region: Vec<u32> = Vec::new();
        let mut q: VecDeque<u32> = VecDeque::new();
        for &t in patch {
            if accepts(self.mesh, self.fnorm, &fit.primitive, t as usize, self.tol, self.cos) {
                seen.insert(t);
                region.push(t);
                q.push_back(t);
            }
        }
        let mut next_refit = region.len().max(MIN_PATCH) * 2;
        while let Some(t) = q.pop_front() {
            for &o in self.nb.get(t as usize).into_iter().flatten() {
                if o == UNOWNED || seen.contains(o) || owner.get(o as usize) != Some(&UNOWNED) {
                    continue;
                }
                if accepts(self.mesh, self.fnorm, &fit.primitive, o as usize, self.tol, self.cos) {
                    seen.insert(o);
                    region.push(o);
                    q.push_back(o);
                }
            }
            if cap.is_some_and(|c| region.len() > c) {
                return Outcome::TooBig;
            }
            if region.len() >= next_refit {
                next_refit *= 2;
                self.refit(&region, &mut fit);
            }
        }
        sw.stop(crate::stats::Phase::Grow);
        Outcome::Grown { region, fit }
    }

    /// Refit while growing: small regions may still change kind (a narrow chamfer band starts out
    /// looking like a cylinder); large ones refit only their kind (the automatic choice runs once
    /// more at the end), because the general kinds' iterative fits dominate the time otherwise.
    fn refit(&self, region: &[u32], fit: &mut PrimitiveFit) {
        crate::stats::grow_refit();
        let sw = crate::stats::Stopwatch::start();
        let refit = if region.len() <= SMALL_REGION * self.seed_tris {
            fit_tris(self.mesh, self.normals, region)
        } else {
            fit_kind_tris(self.mesh, self.normals, region, fit.primitive.kind())
        };
        sw.stop(crate::stats::Phase::Refit);
        if let Some(f) = refit
            && f.rms <= self.tol
        {
            *fit = f;
        }
    }
}
