//! The rounds of the driver (design doc §4.3): state the conditions, solve, measure, and grow the
//! net where the misses are until every target holds, the worst miss stops improving, the cost
//! bound of one solve is reached, or [`MAX_ROUNDS`] are done.
use super::measure::{self, Goal, Measure};
use super::rows::{self, Inputs, Layout, SideSamples, Zone};
use super::start::{Interior, Loop};
use super::{
    Continuity, G0_TARGET, G2_TARGET_REL, MAX_ROUNDS, MIN_IMPROVEMENT, Request, Round, Summary, grow, grow_other,
    prefer,
};
use crate::nurbs::SurfaceProbe;
use crate::record::brep::Surface;
use crate::{KernelError, Result, poll, vpatch};
use surfacecraft_geom::Cancel;

#[cfg(test)]
use super::RoundTrace;

/// What the rounds are run against.
pub(super) struct Context<'a> {
    pub req: &'a Request,
    pub lp: &'a Loop,
    pub zones: &'a [Zone],
    /// The fixed parameter domain.
    pub domain: [[f64; 2]; 2],
    pub start_net: [usize; 2],
    pub smoothing: f64,
    pub cancel: &'a Cancel,
}

/// A growth the cost bound of one solve stopped.
#[derive(Clone, Copy, Debug)]
pub(super) struct CostStop {
    /// The net that was wanted next.
    pub net: [usize; 2],
    /// Estimated seconds of its solve.
    pub seconds: f64,
}

/// What the rounds produced.
pub(super) struct Outcome {
    pub best: Round,
    pub rounds: u32,
    /// Sides whose gap the growth no longer lowers.
    pub frozen: Vec<bool>,
    pub stop: Option<CostStop>,
}

/// What carries over from one round to the next.
struct State {
    net: [usize; 2],
    best: Option<Round>,
    cache: Option<(Layout, Vec<SideSamples>)>,
    prev: Option<Surface>,
    prev_worst: f64,
    /// Sides whose gap the growth no longer lowers.
    frozen: Vec<bool>,
    /// Every side's gap in the round before.
    prev_gaps: Vec<f64>,
    /// The net before the last growth.
    from: [usize; 2],
    /// Whether the last step grew the net.
    grew: bool,
    /// Whether the last round already was a retry.
    retried: bool,
    /// G2 rows are linearised about the previous round's surface: the curvature error of the last
    /// round that had them.
    prev_curv: Option<f64>,
    rounds: u32,
    stop: Option<CostStop>,
}

enum Flow {
    Next,
    Stop,
}

fn arg(msg: impl Into<String>) -> KernelError {
    KernelError::Argument(msg.into())
}

impl Context<'_> {
    /// Runs the rounds; `interior` takes the closest points of every round.
    pub fn run(&self, interior: &mut Interior, #[cfg(test)] trace: &mut Vec<RoundTrace>) -> Result<Outcome> {
        let mut st = State {
            net: self.start_net,
            best: None,
            cache: None,
            prev: None,
            prev_worst: f64::INFINITY,
            frozen: vec![false; self.lp.sides.len()],
            prev_gaps: Vec::new(),
            from: self.start_net,
            grew: false,
            retried: false,
            prev_curv: None,
            rounds: 0,
            stop: None,
        };
        for round in 1..=MAX_ROUNDS {
            poll(self.cancel)?;
            let flow = self.round(
                &mut st,
                interior,
                round,
                #[cfg(test)]
                trace,
            )?;
            if matches!(flow, Flow::Stop) {
                break;
            }
        }
        let best = st.best.ok_or_else(|| arg("the patch could not be fitted"))?;
        Ok(Outcome { best, rounds: st.rounds, frozen: st.frozen, stop: st.stop })
    }

    /// The cost bound: the round's solve is refused when it is estimated above
    /// [`vpatch::MAX_SOLVE_SECONDS`]. A growth that hits it ends the rounds (`Some`); the first
    /// round has nothing to fall back on and is an error.
    fn cost_stop(&self, problem: &vpatch::Problem, st: &State) -> Result<Option<CostStop>> {
        let seconds = problem.estimated_seconds();
        if seconds <= vpatch::MAX_SOLVE_SECONDS {
            return Ok(None);
        }
        if st.best.is_none() {
            return Err(arg(format!(
                "the control net {} × {} with {} hard rows is estimated at {seconds:.1} s per solve (limit {:.1} s): \
                 give a smaller `control` or fewer hard points, curves and G1/G2 sides",
                st.net[0],
                st.net[1],
                problem.hard_scalar_rows(),
                vpatch::MAX_SOLVE_SECONDS
            )));
        }
        Ok(Some(CostStop { net: st.net, seconds }))
    }

    /// A side whose gap a growth step along it lowered by less than [`MIN_IMPROVEMENT`] is as close
    /// as it gets (a step that grew only the other direction says nothing about the side).
    fn freeze(&self, st: &mut State, m: &Measure, main: &[usize]) {
        for (k, (s, before)) in m.sides.iter().zip(&st.prev_gaps).enumerate() {
            let along = st.grew && main.get(k).is_some_and(|&d| st.net.get(d) > st.from.get(d));
            if along
                && !s.soft
                && s.gap_max > G0_TARGET
                && before - s.gap_max < MIN_IMPROVEMENT * before
                && let Some(f) = st.frozen.get_mut(k)
            {
                *f = true;
            }
        }
        st.prev_gaps = m.sides.iter().map(|s| s.gap_max).collect();
    }

    /// The net of the next round: `None` ends the rounds.
    fn next_net(&self, st: &mut State, improved: bool, miss: [f64; 2], refine: bool) -> Option<[usize; 2]> {
        let max = self.req.max_control;
        // a round that did not help gets one retry in the direction(s) not grown last time
        let wanted = if improved {
            st.retried = false;
            Some(grow(st.net, miss, max))
        } else if st.retried {
            None
        } else {
            st.retried = true;
            Some(grow_other(st.net, st.from, max))
        };
        match wanted {
            Some(n) if n != st.net => Some(n),
            _ if refine => Some(st.net),
            _ => None,
        }
    }

    /// The conditions of the round at the state's net (the samples of the sides are cached per net).
    fn problem(&self, st: &mut State, interior: &Interior, layout: &Layout) -> Result<vpatch::Problem> {
        if st.cache.as_ref().is_none_or(|(l, _)| l.net != st.net) {
            st.cache = Some((*layout, rows::sample_sides(self.lp, layout, self.cancel)?));
        }
        let Some((_, samples)) = st.cache.as_ref() else { return Err(arg("the samples of the sides are missing")) };
        rows::build_problem(&Inputs {
            lp: self.lp,
            layout,
            sides: samples,
            interior,
            zones: self.zones,
            prev: st.prev.as_ref(),
            smoothing: self.smoothing,
        })
    }

    /// Solves the problem and measures the surface; the seconds of the solve go to the test trace.
    fn solve_and_measure(
        &self,
        st: &State,
        interior: &mut Interior,
        layout: &Layout,
        problem: &vpatch::Problem,
    ) -> Result<(vpatch::Solution, Measure, f64)> {
        let Some((_, samples)) = st.cache.as_ref() else { return Err(arg("the samples of the sides are missing")) };
        // a wall clock panics on wasm: only the tests time the solve
        #[cfg(test)]
        let started = std::time::Instant::now();
        let solution = vpatch::solve_cancellable(problem, self.cancel)?;
        #[cfg(test)]
        let solve_secs = started.elapsed().as_secs_f64();
        #[cfg(not(test))]
        let solve_secs = 0.0;
        let probe = SurfaceProbe::new(&solution.surface)?;
        let goal = Goal { tolerance: self.req.tolerance, frozen: &st.frozen, cancel: self.cancel };
        let m = measure::measure(&solution.surface, &probe, self.lp, layout, samples, interior, &goal)?;
        Ok((solution, m, solve_secs))
    }

    fn round(
        &self,
        st: &mut State,
        interior: &mut Interior,
        round: u32,
        #[cfg(test)] trace: &mut Vec<RoundTrace>,
    ) -> Result<Flow> {
        let had_rows = st.prev.is_some();
        let layout = Layout::new(self.domain, st.net);
        let problem = self.problem(st, interior, &layout)?;
        if let Some(stop) = self.cost_stop(&problem, st)? {
            st.stop = Some(stop);
            return Ok(Flow::Stop);
        }
        let (solution, m, _solve_secs) = self.solve_and_measure(st, interior, &layout, &problem)?;
        st.rounds = round;
        #[cfg(test)]
        trace.push(RoundTrace {
            net: st.net,
            rms: m.rms,
            max: m.max,
            gap_max: m.sides.iter().filter(|s| !s.soft).map(|s| s.gap_max).fold(0.0, f64::max),
            solve_secs: _solve_secs,
            hard_rows: problem.hard_scalar_rows(),
        });
        Ok(self.keep_and_grow(st, round, had_rows, solution, m))
    }

    /// Keeps the round when it is the best so far, and decides whether and how the net grows.
    fn keep_and_grow(
        &self,
        st: &mut State,
        round: u32,
        had_rows: bool,
        solution: vpatch::Solution,
        m: Measure,
    ) -> Flow {
        self.freeze(st, &m, &measure::main_directions(self.lp));
        let data_ok = m.points == 0 || m.max <= self.req.tolerance;
        let done = m.sides_ok(&st.frozen) && data_ok;
        let worst = m.worst(&st.frozen);
        let miss = m.miss;
        let improved = round == 1 || st.prev_worst - worst >= MIN_IMPROVEMENT * st.prev_worst;
        let summary = Summary { data_ok, data_max: m.max, side_miss: m.side_miss() };
        // G2 on a net that cannot grow: the first round has no curvature rows, so another round at the
        // same net states them; then rounds repeat while the curvature error still falls by 5 %
        let has_g2 = self.lp.sides.iter().any(|s| s.continuity == Continuity::G2);
        let curv = m.curvature_miss();
        let refine = has_g2
            && !done
            && (!had_rows || (curv > G2_TARGET_REL && st.prev_curv.is_none_or(|p| p - curv >= MIN_IMPROVEMENT * p)));
        st.prev_curv = had_rows.then_some(curv);
        if st.best.as_ref().is_none_or(|b| prefer(&summary, &b.summary)) {
            st.best = Some(Round {
                surface: solution.surface.clone(),
                measure: m,
                net: st.net,
                dropped: solution.dropped,
                hard_residual: solution.hard_residual,
                summary,
            });
        }
        if done || round == MAX_ROUNDS {
            return Flow::Stop;
        }
        let Some(next) = self.next_net(st, improved, miss, refine) else { return Flow::Stop };
        st.prev_worst = st.prev_worst.min(worst);
        st.prev = Some(solution.surface);
        st.grew = next != st.net;
        if st.grew {
            st.from = st.net;
        }
        st.net = next;
        Flow::Next
    }
}
