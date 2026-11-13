//! Finding regions: seeds are taken in the order of flatness, each grows a region on the
//! triangles no earlier region owns.
//!
//! The result is that of the plain sequential loop (one seed after the other), on any number of
//! threads. The jobs of the next seeds run in parallel on a snapshot of the owner array and are
//! *speculative*: a speculative job is used only if running it now would give the same result. A
//! job reads the owner array in two places (the seed patch and the growth across neighbours) and
//! only to skip owned triangles; owned triangles only ever get more. So a job is the same on a
//! later state when its patch comes out equal and none of the triangles it grew over has been
//! claimed since (a neighbour that was claimed meanwhile and that the job did not accept is
//! skipped either way).
use super::Segment;
use super::grow::{Ctx, Job, Outcome, UNOWNED};
use std::collections::{HashMap, VecDeque};

/// Most regions kept (the rest of the mesh stays unassigned).
const MAX_REGIONS: usize = 10_000;
/// How far past the current seed look-ahead looks for seeds that are still free.
const LOOK_AHEAD_SCAN: usize = 4096;
/// A speculative job gives up growing at this many seed sizes of triangles: larger regions are
/// rare, and a wasted speculation must stay cheap.
const SPECULATION_REGION: usize = 20;
/// Speculative jobs per thread at the start and at most.
const START_BATCH_PER_THREAD: usize = 2;
const MAX_BATCH_PER_THREAD: usize = 4;
/// Look-ahead batch limits (jobs).
const BATCH_LIMITS: (usize, usize) = (2, 128);
/// The batch size is adapted after this many speculative jobs.
const ADAPT_AFTER: usize = 16;
/// Cached speculative jobs kept at most per unit of batch size.
const CACHE_PER_BATCH: usize = 8;

/// A fragment next to another fragment is evidence of an organic area: the seeds around it are
/// skipped up to this many seed patches of area (they would only make more fragments).
const HALO_PATCHES: f64 = 8.0;
/// Smoothness of the halo: the least dot product of the normals of neighbouring triangles it
/// crosses (it stops at the same creases as a seed patch).
const HALO_MIN_DOT: f64 = 0.94;

/// What marks organic territory to the seed loop (see `freeform`).
#[derive(Clone, Copy, Debug)]
pub(super) struct FreeformHints {
    /// Regions with fewer triangles than this are fragments.
    pub weak_tris: usize,
}

/// The state of the seed loop.
struct Seeder<'a, 'b> {
    ctx: &'a Ctx<'b>,
    order: &'a [(u64, u32)],
    min_triangles: usize,
    /// Fragments (rejected seed patches and weak regions) are noted, and their surroundings
    /// skipped, when set.
    hints: Option<FreeformHints>,
    /// Triangles of fragments found so far.
    fragment: Vec<bool>,
    owner: Vec<u32>,
    tried: Vec<bool>,
    out: Vec<Segment>,
    /// Speculative jobs by seed, computed on an earlier (smaller) owner state.
    spec: HashMap<u32, Job>,
    batch: usize,
    max_batch: usize,
    spec_made: usize,
    spec_used: usize,
    /// Totals of the whole run (the two above restart when the batch size is adapted).
    totals: super::SegmentStats,
}

/// Regions of the mesh, grown from the seeds of `order` (`(flatness key, triangle)`).
pub(super) fn grow_regions(
    ctx: &Ctx<'_>,
    order: &[(u64, u32)],
    min_triangles: usize,
    hints: Option<FreeformHints>,
    cancel: &surfacecraft_geom::Cancel,
) -> (Vec<Segment>, super::SegmentStats) {
    let threads = surfacecraft_geom::par::threads();
    let (lo, hi) = BATCH_LIMITS;
    let mut s = Seeder {
        ctx,
        order,
        min_triangles,
        hints,
        fragment: if hints.is_some() { vec![false; ctx.nb.len()] } else { Vec::new() },
        owner: vec![UNOWNED; ctx.nb.len()],
        tried: vec![false; ctx.nb.len()],
        out: Vec::new(),
        spec: HashMap::new(),
        batch: if threads > 1 { (START_BATCH_PER_THREAD * threads).clamp(lo, hi) } else { 1 },
        max_batch: if threads > 1 { (MAX_BATCH_PER_THREAD * threads).clamp(lo, hi) } else { 1 },
        spec_made: 0,
        spec_used: 0,
        totals: super::SegmentStats::default(),
    };
    for (idx, &(_, seed)) in order.iter().enumerate() {
        // a cancelled run stops here; its caller drops what was found so far
        if s.out.len() >= MAX_REGIONS || cancel.is_cancelled() {
            break;
        }
        s.step(idx, seed);
    }
    (s.out, s.totals)
}

impl Seeder<'_, '_> {
    fn is_free(&self, t: u32) -> bool {
        self.owner.get(t as usize) == Some(&UNOWNED) && self.tried.get(t as usize) == Some(&false)
    }

    /// The sequential algorithm's step for the seed at `order[idx]`.
    fn step(&mut self, idx: usize, seed: u32) {
        if !self.is_free(seed) {
            return;
        }
        let patch = self.ctx.patch(seed, &self.owner);
        for &t in &patch {
            if let Some(f) = self.tried.get_mut(t as usize) {
                *f = true;
            }
        }
        let job = match self.spec.remove(&seed) {
            Some(job) if self.still_valid(&job, &patch) => {
                self.spec_used += 1;
                self.totals.speculative_used += 1;
                crate::stats::speculation(0, 1);
                job
            }
            _ => self.run_batch(idx, seed),
        };
        self.commit(job);
    }

    /// Would `job` (computed on an earlier state) come out the same now, given the current
    /// `patch` of its seed?
    fn still_valid(&self, job: &Job, patch: &[u32]) -> bool {
        if job.patch != patch {
            return false;
        }
        match &job.outcome {
            Outcome::Grown { region, .. } => region.iter().all(|&t| self.owner.get(t as usize) == Some(&UNOWNED)),
            Outcome::TooBig => false,
            Outcome::Tiny | Outcome::Rejected => true,
        }
    }

    /// Claims the region of `job` if it is big enough.
    fn commit(&mut self, job: Job) {
        let weak = self.hints.map_or(0, |h| h.weak_tris);
        let Outcome::Grown { mut region, fit } = job.outcome else {
            if matches!(job.outcome, Outcome::Rejected) {
                self.note_fragment(&job.patch);
            }
            return;
        };
        if region.len() < self.min_triangles {
            self.note_fragment(&job.patch);
            return;
        }
        if region.len() < weak {
            self.note_fragment(&region);
        }
        let id = self.out.len() as u32;
        for &t in &region {
            if let Some(o) = self.owner.get_mut(t as usize) {
                *o = id;
            }
        }
        region.sort_unstable();
        self.out.push(Segment { triangles: region, primitive: fit.primitive, rms: fit.rms, freeform: false });
    }

    /// Notes a fragment (`tris`): one that touches an earlier fragment sits in organic territory,
    /// and the free triangles around it are skipped as seeds.
    fn note_fragment(&mut self, tris: &[u32]) {
        if self.hints.is_none() {
            return;
        }
        let touches = tris.iter().any(|&t| {
            let nb = self.ctx.nb.get(t as usize).into_iter().flatten();
            nb.into_iter().any(|&o| o != UNOWNED && self.fragment.get(o as usize) == Some(&true))
        });
        for &t in tris {
            if let Some(f) = self.fragment.get_mut(t as usize) {
                *f = true;
            }
        }
        if touches {
            self.skip_around(tris);
        }
    }

    /// Marks the free, smooth triangles around `tris`, breadth first up to the halo area, as tried.
    fn skip_around(&mut self, tris: &[u32]) {
        let limit = HALO_PATCHES * self.ctx.seed_area;
        let mut area = 0.0;
        let mut queue: VecDeque<u32> = tris.iter().copied().collect();
        while let Some(t) = queue.pop_front() {
            if area >= limit {
                break;
            }
            let Some(n0) = self.ctx.fnorm.get(t as usize).copied().flatten() else { continue };
            for &o in self.ctx.nb.get(t as usize).into_iter().flatten() {
                let free = o != UNOWNED && self.is_free(o);
                let smooth =
                    self.ctx.fnorm.get(o as usize).copied().flatten().is_some_and(|m| m.dot(&n0) > HALO_MIN_DOT);
                if free && smooth {
                    if let Some(f) = self.tried.get_mut(o as usize) {
                        *f = true;
                    }
                    area += self.ctx.areas.get(o as usize).copied().unwrap_or(0.0);
                    queue.push_back(o);
                }
            }
        }
    }

    /// Runs the job of `seed` now, together with speculative jobs for the next free seeds (the
    /// seed's own job is the only one that may grow without a limit); returns the seed's job.
    fn run_batch(&mut self, idx: usize, seed: u32) -> Job {
        self.adapt_batch();
        let mut seeds = vec![seed];
        if self.batch > 1 {
            let next = self.order.iter().skip(idx + 1).take(LOOK_AHEAD_SCAN).map(|&(_, o)| o);
            let free = next.filter(|&o| self.is_free(o) && !self.spec.contains_key(&o));
            seeds.extend(free.take(self.batch - 1));
            if self.spec.len() > CACHE_PER_BATCH * self.batch {
                self.spec.clear();
            }
        }
        let cap = Some(SPECULATION_REGION * self.ctx.seed_tris);
        let jobs = surfacecraft_geom::par::map_slice(&seeds, |&sd| {
            self.ctx.run(sd, &self.owner, if sd == seed { None } else { cap })
        });
        self.spec_made += seeds.len() - 1;
        self.totals.speculative_jobs += seeds.len() - 1;
        crate::stats::speculation(seeds.len() - 1, 0);
        let mut own = None;
        for (sd, job) in seeds.into_iter().zip(jobs) {
            if sd == seed {
                own = Some(job);
            } else {
                self.spec.insert(sd, job);
            }
        }
        own.unwrap_or_else(|| self.ctx.run(seed, &self.owner, None))
    }

    /// Adapts the look-ahead to how much of the speculative work was needed.
    fn adapt_batch(&mut self) {
        if self.batch > 1 && self.spec_made >= ADAPT_AFTER {
            let grow = self.spec_used * 2 >= self.spec_made;
            self.batch = if grow { (self.batch * 2).min(self.max_batch) } else { (self.batch / 2).max(BATCH_LIMITS.0) };
            self.spec_made = 0;
            self.spec_used = 0;
        }
    }
}
