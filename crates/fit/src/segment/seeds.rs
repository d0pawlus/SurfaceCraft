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
use std::collections::HashMap;

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

/// The state of the seed loop.
struct Seeder<'a, 'b> {
    ctx: &'a Ctx<'b>,
    order: &'a [(u64, u32)],
    min_triangles: usize,
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
    cancel: &surfacecraft_geom::Cancel,
) -> (Vec<Segment>, super::SegmentStats) {
    let threads = surfacecraft_geom::par::threads();
    let (lo, hi) = BATCH_LIMITS;
    let mut s = Seeder {
        ctx,
        order,
        min_triangles,
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
        let Outcome::Grown { mut region, fit } = job.outcome else { return };
        if region.len() < self.min_triangles {
            return;
        }
        let id = self.out.len() as u32;
        for &t in &region {
            if let Some(o) = self.owner.get_mut(t as usize) {
                *o = id;
            }
        }
        region.sort_unstable();
        self.out.push(Segment { triangles: region, primitive: fit.primitive, rms: fit.rms });
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
