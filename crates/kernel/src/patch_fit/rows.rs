//! The conditions of one round: dense samples of the sides, the collocation points where rows are
//! hard, the soft rows between them and the [`Problem`] handed to the solver (design doc §4.3).
use super::Continuity;
use super::geom::{V2, V3, add, dot, scale, thin_indices, unit};
use super::reference::RefSample;
use super::start::{Interior, Loop};
use crate::record::brep::Surface;
use crate::vpatch::{DataPoint, Problem, Row, derivatives};
use super::start::named;
use crate::{KernelError, Result, poll};
use surfacecraft_geom::Cancel;

/// Weight of the soft rows that stand in for dense side conditions, against a data weight of 1.
pub(super) const W_SOFT: f64 = 1e4;
/// Weight of a row of a soft side (traced on a scan) in units of the mean data weight.
const SOFT_SIDE_WEIGHT: f64 = 1.0;
/// Fewer data points than this count as no data.
pub(super) const MIN_DATA: usize = 3;
/// Samples per knot interval of a side.
const PER_INTERVAL: usize = 20;
/// Dense samples over all sides (the measurement and the soft rows scale with them).
const TOTAL_DENSE: usize = 30_000;
/// Soft rows handed to the solver (the solver takes 20 000 rows in all).
const MAX_SOFT_ROWS: usize = 6_000;
const MAX_ROWS: usize = 20_000;
/// Samples between two looks at the cancellation flag.
const POLL_EVERY: usize = 64;
/// Points of the outline polygon given to the solver.
const MAX_OUTLINE: usize = 3_000;
/// Rows whose relaxation factor is below this are dropped.
const MIN_FACTOR: f64 = 1e-6;

/// The control net, the fixed parameter domain and the knot cell size of one round.
#[derive(Clone, Copy, Debug)]
pub(super) struct Layout {
    pub net: [usize; 2],
    pub domain: [[f64; 2]; 2],
    pub cell: V2,
}

impl Layout {
    pub fn new(domain: [[f64; 2]; 2], net: [usize; 2]) -> Self {
        let cells = |n: usize| n.saturating_sub(3).max(1) as f64;
        Self {
            net,
            domain,
            cell: [(domain[0][1] - domain[0][0]) / cells(net[0]), (domain[1][1] - domain[1][0]) / cells(net[1])],
        }
    }

    pub fn inside(&self, uv: V2) -> bool {
        (0..2).all(|d| uv[d] >= self.domain[d][0] && uv[d] <= self.domain[d][1])
    }
}

/// A dense sample of a side.
pub(super) struct Sample {
    pub xyz: V3,
    pub uv: V2,
    /// Arc length from the start of the side.
    pub s: f64,
    /// Parameter direction across the side, into the patch (unit).
    pub dir: V2,
    /// The reference face there (G1/G2 sides).
    pub refs: Option<RefSample>,
}

/// The dense samples of one side (first and last included) and the indices of its collocation points.
pub(super) struct SideSamples {
    pub samples: Vec<Sample>,
    pub colloc: Vec<usize>,
}

/// Where a corner conflict relaxes the G1/G2 rows of a side: `length` from its start or its end.
#[derive(Clone, Copy, Debug)]
pub(super) struct Zone {
    pub side: usize,
    pub at_end: bool,
    pub length: f64,
}

fn relax_factor(zones: &[Zone], side: usize, s: f64, len: f64) -> f64 {
    let mut f = 1.0_f64;
    for z in zones.iter().filter(|z| z.side == side && z.length > 0.0) {
        let d = if z.at_end { len - s } else { s };
        f = f.min((d / z.length).clamp(0.0, 1.0));
    }
    f
}

/// Indices of `uv` at about one per knot cell of path length, ends included. The path is measured
/// in cells (a step across a long cell counts less than one across a short cell): the bicubic
/// has about one degree of freedom per cell along a side, and more hard rows than that over-determine
/// the side curve, so that the pseudo-inverse of the hard rows amplifies every inconsistency
/// between the samples (their noise, the sag of a polyline) into the control points.
pub(super) fn collocation(uv: &[V2], cell: V2) -> Vec<usize> {
    let n = uv.len();
    if n == 0 {
        return Vec::new();
    }
    let mut cum = Vec::with_capacity(n);
    let mut s = 0.0;
    cum.push(0.0);
    for w in uv.windows(2) {
        s += ((w[1][0] - w[0][0]) / cell[0]).hypot((w[1][1] - w[0][1]) / cell[1]);
        cum.push(s);
    }
    let m = if cell[0] > 0.0 && cell[1] > 0.0 && s.is_finite() { s.ceil().max(1.0) as usize } else { 1 };
    let m = m.min(n.saturating_sub(1)).max(1);
    let mut out = Vec::with_capacity(m + 3);
    for j in 0..=m {
        let target = s * j as f64 / m as f64;
        let k = cum.partition_point(|c| *c < target).min(n - 1);
        let at = |i: usize| cum.get(i).copied().unwrap_or(0.0);
        let lower = k.checked_sub(1).filter(|&l| target - at(l) < at(k) - target);
        out.push(lower.unwrap_or(k));
    }
    out.push(0);
    out.push(n - 1);
    out.sort_unstable();
    out.dedup();
    out
}

fn dense_count(lp: &Loop, layout: &Layout) -> usize {
    let mut cells = 1usize;
    for side in &lp.sides {
        let path: f64 = side
            .uv
            .windows(2)
            .map(|w| (w[1][0] - w[0][0]).abs() / layout.cell[0] + (w[1][1] - w[0][1]).abs() / layout.cell[1])
            .sum();
        cells = cells.max(path.ceil().clamp(1.0, 1e4) as usize);
    }
    let n = PER_INTERVAL * cells + 1;
    n.clamp(41, 801).min(TOTAL_DENSE / lp.sides.len().max(1)).max(8)
}

/// Dense samples of every side for the current net, the same count on every side.
pub(super) fn sample_sides(lp: &Loop, layout: &Layout, cancel: &Cancel) -> Result<Vec<SideSamples>> {
    let nd = dense_count(lp, layout);
    let mut out = Vec::with_capacity(lp.sides.len());
    for side in &lp.sides {
        let len = side.length();
        let mut samples = Vec::with_capacity(nd);
        for j in 0..nd {
            if j % POLL_EVERY == 0 {
                poll(cancel)?;
            }
            let s = if j + 1 == nd { len } else { len * j as f64 / (nd - 1) as f64 };
            let (xyz, uv) = side.at(s);
            let dir = lp.inward(side.tangent_uv(s)?);
            let refs = match (&side.probe, side.continuity) {
                (Some(p), Continuity::G1 | Continuity::G2) => Some(p.locate(xyz).map_err(|e| named(side.label, e))?),
                _ => None,
            };
            samples.push(Sample { xyz, uv, s, dir, refs });
        }
        let uvs: Vec<V2> = samples.iter().map(|x| x.uv).collect();
        out.push(SideSamples { colloc: collocation(&uvs, layout.cell), samples });
    }
    Ok(out)
}

/// What one round needs to state its problem.
pub(super) struct Inputs<'a> {
    pub lp: &'a Loop,
    pub layout: &'a Layout,
    pub sides: &'a [SideSamples],
    pub interior: &'a Interior,
    pub zones: &'a [Zone],
    /// The previous round's surface (curvature rows of G2 sides are linearised about it).
    pub prev: Option<&'a Surface>,
    pub smoothing: f64,
}

/// The hard and the soft rows of a round.
#[derive(Default)]
struct Rows {
    hard: Vec<Row>,
    soft: Vec<(Row, f64)>,
}

impl Rows {
    /// States one row: of a hard side, hard at a collocation point and heavily soft elsewhere; of a
    /// soft side (`soft_side` is its weight), soft with that weight. `factor` relaxes it near a conflict.
    fn put(&mut self, row: Row, collocation: bool, factor: f64, soft_side: Option<f64>) {
        if let Some(w) = soft_side {
            if factor > MIN_FACTOR {
                self.soft.push((row, w * factor));
            }
        } else if collocation && factor >= 1.0 {
            self.hard.push(row);
        } else if factor > MIN_FACTOR {
            self.soft.push((row, W_SOFT * factor));
        }
    }
}

/// `S_dd · n = κ |S_d|²` with `κ` of the reference face along the previous round's `S_d`.
fn curvature_row(prev: &Surface, smp: &Sample, r: &RefSample) -> Result<Option<Row>> {
    let d = derivatives(prev, smp.uv)?;
    let sd = add(scale(d[1], smp.dir[0]), scale(d[2], smp.dir[1]));
    let Some(w) = unit(sd) else { return Ok(None) };
    let Some(kappa) = r.curvature(w) else { return Ok(None) };
    let value = kappa * dot(sd, sd);
    if !value.is_finite() {
        return Ok(None);
    }
    Ok(Some(Row::Curvature { uv: smp.uv, dir: smp.dir, normal: r.normal, value }))
}

fn rows_per_sample(c: Continuity) -> usize {
    match c {
        Continuity::G0 => 1,
        Continuity::G1 => 2,
        Continuity::G2 => 3,
    }
}

/// Keeps at most `budget` of the rows, evenly spread (first and last kept).
fn thin_rows(rows: &mut Vec<(Row, f64)>, budget: usize) {
    if rows.len() <= budget {
        return;
    }
    if budget < 2 {
        rows.truncate(budget);
        return;
    }
    let kept: Vec<(Row, f64)> =
        thin_indices(rows.len(), budget).into_iter().filter_map(|i| rows.get(i).cloned()).collect();
    *rows = kept;
}

/// The data that count: inside the outline and the parameter domain; fewer than 3 are none.
fn counted_data(inp: &Inputs) -> Vec<DataPoint> {
    let mut data = Vec::with_capacity(inp.interior.data.len());
    for p in &inp.interior.data {
        if !p.out && inp.layout.inside(p.uv) {
            data.push(DataPoint { uv: p.uv, at: p.xyz, weight: p.w });
        }
    }
    if data.len() < MIN_DATA {
        data.clear();
    }
    data
}

/// Every how many candidate soft rows one is taken, so that at most [`MAX_SOFT_ROWS`] come out.
fn soft_stride(inp: &Inputs, dense_soft: bool) -> usize {
    let mut candidates: usize = inp
        .sides
        .iter()
        .zip(&inp.lp.sides)
        .filter(|(_, sd)| !sd.soft || dense_soft)
        .map(|(ss, sd)| ss.samples.len().saturating_sub(ss.colloc.len()) * rows_per_sample(sd.continuity))
        .sum();
    candidates += inp.interior.curves.iter().map(Vec::len).sum::<usize>();
    candidates.div_ceil(MAX_SOFT_ROWS).max(1)
}

/// Where the samples of a side are collocation points.
fn collocation_flags(n: usize, colloc: &[usize]) -> Vec<bool> {
    let mut is_col = vec![false; n];
    for &c in colloc {
        if let Some(x) = is_col.get_mut(c) {
            *x = true;
        }
    }
    is_col
}

/// How the rows of the sides are weighted.
struct SideRows {
    /// Weight of a row of a soft side (the mean data weight).
    soft_w: f64,
    /// Soft sides put rows at every thinned sample (there is no data to follow).
    dense_soft: bool,
    stride: usize,
}

/// The rows of the sides: position, tangent and curvature rows at the samples taken.
fn side_rows(inp: &Inputs, w: &SideRows, rows: &mut Rows) -> Result<()> {
    let sides_n = inp.lp.sides.len();
    for (j, (ss, sd)) in inp.sides.iter().zip(&inp.lp.sides).enumerate() {
        let len = sd.length();
        let n = ss.samples.len();
        let prev_soft = inp.lp.sides.get((j + sides_n - 1) % sides_n.max(1)).is_some_and(|p| p.soft);
        let side_w = sd.soft.then_some(w.soft_w);
        let is_col = collocation_flags(n, &ss.colloc);
        for (i, smp) in ss.samples.iter().enumerate() {
            let col = is_col.get(i).copied().unwrap_or(false);
            let take = if sd.soft && !w.dense_soft { col } else { col || i.is_multiple_of(w.stride) };
            if !take {
                continue;
            }
            // the corner is stated once, by the side that starts there: hard when either side is hard
            if i + 1 < n {
                let weight = if i == 0 && !prev_soft { None } else { side_w };
                rows.put(Row::Position { uv: smp.uv, at: smp.xyz }, col, 1.0, weight);
            }
            let Some(r) = &smp.refs else { continue };
            let f = relax_factor(inp.zones, j, smp.s, len);
            rows.put(Row::Tangent { uv: smp.uv, dir: smp.dir, normal: r.normal }, col, f, side_w);
            if sd.continuity == Continuity::G2
                && let Some(prev) = inp.prev
                && let Some(row) = curvature_row(prev, smp, r)?
            {
                rows.put(row, col, f, side_w);
            }
        }
    }
    Ok(())
}

/// The rows of the interior: hard points, and the curves at their collocation points.
fn interior_rows(inp: &Inputs, stride: usize, rows: &mut Rows) {
    for p in &inp.interior.hard {
        rows.hard.push(Row::Position { uv: p.uv, at: p.xyz });
    }
    for c in &inp.interior.curves {
        let uvs: Vec<V2> = c.iter().map(|p| p.uv).collect();
        let is_col = collocation_flags(c.len(), &collocation(&uvs, inp.layout.cell));
        for (i, p) in c.iter().enumerate() {
            let col = is_col.get(i).copied().unwrap_or(false);
            if col || i.is_multiple_of(stride) {
                rows.put(Row::Position { uv: p.uv, at: p.xyz }, col, 1.0, None);
            }
        }
    }
}

/// The outline polygon given to the solver: the side samples' parameters, thinned.
fn outline_polygon(sides: &[SideSamples]) -> Vec<V2> {
    let mut outline: Vec<V2> = Vec::new();
    for ss in sides {
        let n = ss.samples.len();
        outline.extend(ss.samples.iter().take(n.saturating_sub(1)).map(|s| s.uv));
    }
    thin_indices(outline.len(), MAX_OUTLINE).into_iter().filter_map(|i| outline.get(i).copied()).collect()
}

/// States the round's problem: hard rows at the collocation points, soft rows at the other dense
/// samples (thinned to a budget), data, outline. A soft side puts only soft rows, at its collocation
/// points (at every thinned sample when there is no data to follow), weighted like a mean data point;
/// its corners stay hard where a hard side meets it.
pub(super) fn build_problem(inp: &Inputs) -> Result<Problem> {
    let data = counted_data(inp);
    let mean_w = if data.is_empty() { 1.0 } else { data.iter().map(|d| d.weight).sum::<f64>() / data.len() as f64 };
    let dense_soft = data.is_empty();
    let stride = soft_stride(inp, dense_soft);
    let mut rows = Rows::default();
    side_rows(inp, &SideRows { soft_w: SOFT_SIDE_WEIGHT * mean_w, dense_soft, stride }, &mut rows)?;
    interior_rows(inp, stride, &mut rows);
    let Rows { hard, mut soft } = rows;

    let soft_points = inp.interior.soft.iter().filter(|p| inp.layout.inside(p.uv)).count();
    if hard.len() + soft_points > MAX_ROWS {
        return Err(KernelError::Argument(format!(
            "the conditions need {} hard rows and {soft_points} soft points; at most {MAX_ROWS} rows",
            hard.len()
        )));
    }
    // over the budget, the soft rows are thinned evenly (not cut off at the last sides)
    thin_rows(&mut soft, MAX_ROWS - hard.len() - soft_points);
    // the soft points: soft position rows, weighted against a mean data point (1 without data); they
    // are not data, so they neither count toward the minimum number of data points nor are thinned
    for p in inp.interior.soft.iter().filter(|p| inp.layout.inside(p.uv)) {
        soft.push((Row::Position { uv: p.uv, at: p.xyz }, p.w * mean_w));
    }
    Ok(Problem {
        domain: inp.layout.domain,
        control: inp.layout.net,
        hard,
        soft,
        data,
        outline: outline_polygon(inp.sides),
        smoothing: inp.smoothing,
    })
}
