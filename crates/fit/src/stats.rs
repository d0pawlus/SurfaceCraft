//! Process-wide work counters of the fitting code (P2 measurements): how many fits, LM runs and
//! iterations a segmentation took. Relaxed atomics, so counting costs nothing worth measuring;
//! results never depend on them.
use std::sync::atomic::{AtomicU64, Ordering};

static LM_RUNS: AtomicU64 = AtomicU64::new(0);
static LM_ITERATIONS: AtomicU64 = AtomicU64::new(0);
static SEED_FITS: AtomicU64 = AtomicU64::new(0);
static GROW_REFITS: AtomicU64 = AtomicU64::new(0);
static FINAL_FITS: AtomicU64 = AtomicU64::new(0);
static SPEC_JOBS: AtomicU64 = AtomicU64::new(0);
static SPEC_USED: AtomicU64 = AtomicU64::new(0);

/// Segmentation phases timed by [`Stopwatch`].
#[derive(Clone, Copy, Debug)]
pub enum Phase {
    /// Neighbours, smoothed normals, areas, seed order.
    Prep,
    /// Finding regions, wall time of the whole seed loop (jobs run in parallel inside it).
    Regions,
    /// Seed patches and their fits (summed over the jobs, speculative ones included).
    Seed,
    /// Region growth including its refits (summed over jobs).
    Grow,
    /// Refits while growing (summed over jobs; part of `Grow`).
    Refit,
    /// Final fits (wall time of the parallel pass).
    Final,
    /// Merging adjacent regions.
    Merge,
    /// Moving boundary triangles between regions.
    Relax,
    /// Merging fragments into freeform regions.
    Freeform,
}

const PHASES: [(&str, Phase); 9] = [
    ("prep", Phase::Prep),
    ("regions", Phase::Regions),
    ("seed", Phase::Seed),
    ("grow", Phase::Grow),
    ("refit", Phase::Refit),
    ("final", Phase::Final),
    ("merge", Phase::Merge),
    ("relax", Phase::Relax),
    ("freeform", Phase::Freeform),
];

static PHASE_NS: [AtomicU64; 9] = [
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
    AtomicU64::new(0),
];

/// Measures one stretch of a phase (no-op on wasm, which has no clock).
pub(crate) struct Stopwatch(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Stopwatch {
    pub(crate) fn start() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self(std::time::Instant::now())
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self()
        }
    }

    pub(crate) fn stop(self, phase: Phase) {
        #[cfg(not(target_arch = "wasm32"))]
        if let Some(c) = PHASE_NS.get(phase as usize) {
            c.fetch_add(self.0.elapsed().as_nanos() as u64, Ordering::Relaxed);
        }
        #[cfg(target_arch = "wasm32")]
        let _ = phase;
    }
}

/// Seconds spent per segmentation phase since the last [`reset`].
#[must_use]
pub fn phase_seconds() -> Vec<(&'static str, f64)> {
    PHASES
        .iter()
        .map(|(n, p)| (*n, PHASE_NS.get(*p as usize).map_or(0.0, |c| c.load(Ordering::Relaxed) as f64 * 1e-9)))
        .collect()
}

/// A copy of the counters.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Levenberg–Marquardt minimisations started.
    pub lm_runs: u64,
    /// LM iterations over all runs.
    pub lm_iterations: u64,
    /// Fits of seed patches in segmentation.
    pub seed_fits: u64,
    /// Refits while a region grows.
    pub grow_refits: u64,
    /// Final fits of finished regions.
    pub final_fits: u64,
    /// Speculative region jobs run ahead of their turn (their fits are counted above too).
    pub speculative_jobs: u64,
    /// Speculative jobs whose result was used.
    pub speculative_used: u64,
}

pub(crate) fn lm_run(iterations: usize) {
    LM_RUNS.fetch_add(1, Ordering::Relaxed);
    LM_ITERATIONS.fetch_add(iterations as u64, Ordering::Relaxed);
}

pub(crate) fn seed_fit() {
    SEED_FITS.fetch_add(1, Ordering::Relaxed);
}

pub(crate) fn grow_refit() {
    GROW_REFITS.fetch_add(1, Ordering::Relaxed);
}

pub(crate) fn speculation(jobs: usize, used: usize) {
    SPEC_JOBS.fetch_add(jobs as u64, Ordering::Relaxed);
    SPEC_USED.fetch_add(used as u64, Ordering::Relaxed);
}

pub(crate) fn final_fit() {
    FINAL_FITS.fetch_add(1, Ordering::Relaxed);
}

/// The current values.
#[must_use]
pub fn snapshot() -> Counters {
    Counters {
        lm_runs: LM_RUNS.load(Ordering::Relaxed),
        lm_iterations: LM_ITERATIONS.load(Ordering::Relaxed),
        seed_fits: SEED_FITS.load(Ordering::Relaxed),
        grow_refits: GROW_REFITS.load(Ordering::Relaxed),
        final_fits: FINAL_FITS.load(Ordering::Relaxed),
        speculative_jobs: SPEC_JOBS.load(Ordering::Relaxed),
        speculative_used: SPEC_USED.load(Ordering::Relaxed),
    }
}

/// Sets every counter to zero.
pub fn reset() {
    for c in [&LM_RUNS, &LM_ITERATIONS, &SEED_FITS, &GROW_REFITS, &FINAL_FITS, &SPEC_JOBS, &SPEC_USED] {
        c.store(0, Ordering::Relaxed);
    }
    for c in &PHASE_NS {
        c.store(0, Ordering::Relaxed);
    }
}
