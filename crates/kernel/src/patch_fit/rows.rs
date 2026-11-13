//! The conditions of one round: dense samples of the sides, the collocation points where rows are
//! hard, the soft rows between them and the [`Problem`] handed to the solver (design doc §4.3).
use super::Continuity;
use super::geom::{V2, V3, add, dist2, dot, scale, thin_indices, unit};
use super::reference::RefSample;
use super::start::{Interior, Loop};
use crate::record::brep::Surface;
use crate::vpatch::{DataPoint, Problem, Row, derivatives};
use crate::{KernelError, Result};

/// Weight of the soft rows that stand in for dense side conditions, against a data weight of 1.
pub(super) const W_SOFT: f64 = 1e4;
/// Samples per knot interval of a side.
const PER_INTERVAL: usize = 20;
/// Dense samples over all sides (the measurement and the soft rows scale with them).
const TOTAL_DENSE: usize = 30_000;
/// Soft rows handed to the solver (the solver takes 20 000 rows in all).
const MAX_SOFT_ROWS: usize = 6_000;
const MAX_ROWS: usize = 20_000;
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

/// Indices of `uv` at about one per knot cell of path length, ends included.
pub(super) fn collocation(uv: &[V2], cell: V2) -> Vec<usize> {
    let n = uv.len();
    if n == 0 {
        return Vec::new();
    }
    let mut cum = Vec::with_capacity(n);
    let mut s = 0.0;
    cum.push(0.0);
    for w in uv.windows(2) {
        s += dist2(w[0], w[1]);
        cum.push(s);
    }
    let step = cell[0].min(cell[1]);
    let m = if step > 0.0 && s.is_finite() { (s / step).ceil().max(1.0) as usize } else { 1 };
    let m = m.min(n.saturating_sub(1)).max(1);
    let mut out = Vec::with_capacity(m + 3);
    for j in 0..=m {
        let target = s * j as f64 / m as f64;
        let k = cum.partition_point(|c| *c < target).min(n - 1);
        let lower = k.checked_sub(1).filter(|&l| target - cum[l] < cum[k] - target);
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
pub(super) fn sample_sides(lp: &Loop, layout: &Layout) -> Result<Vec<SideSamples>> {
    let nd = dense_count(lp, layout);
    let mut out = Vec::with_capacity(lp.sides.len());
    for side in &lp.sides {
        let len = side.length();
        let mut samples = Vec::with_capacity(nd);
        for j in 0..nd {
            let s = if j + 1 == nd { len } else { len * j as f64 / (nd - 1) as f64 };
            let (xyz, uv) = side.at(s);
            let dir = lp.inward(side.tangent_uv(s)?);
            let refs = match (&side.probe, side.continuity) {
                (Some(p), Continuity::G1 | Continuity::G2) => Some(p.locate(xyz)?),
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

/// The problem and how many data points fell outside the parameter domain.
pub(super) struct Built {
    pub problem: Problem,
    pub ignored: usize,
}

fn put(hard: &mut Vec<Row>, soft: &mut Vec<(Row, f64)>, row: Row, collocation: bool, factor: f64) {
    if collocation && factor >= 1.0 {
        hard.push(row);
    } else if factor > MIN_FACTOR {
        soft.push((row, W_SOFT * factor));
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

/// States the round's problem: hard rows at the collocation points, soft rows at the other dense
/// samples (thinned to a budget), data, outline.
pub(super) fn build_problem(inp: &Inputs) -> Result<Built> {
    let mut hard: Vec<Row> = Vec::new();
    let mut soft: Vec<(Row, f64)> = Vec::new();
    let mut candidates: usize = inp
        .sides
        .iter()
        .zip(&inp.lp.sides)
        .map(|(ss, sd)| ss.samples.len().saturating_sub(ss.colloc.len()) * rows_per_sample(sd.continuity))
        .sum();
    candidates += inp.interior.curves.iter().map(Vec::len).sum::<usize>();
    let stride = candidates.div_ceil(MAX_SOFT_ROWS).max(1);

    for (j, (ss, sd)) in inp.sides.iter().zip(&inp.lp.sides).enumerate() {
        let len = sd.length();
        let n = ss.samples.len();
        let mut is_col = vec![false; n];
        for &c in &ss.colloc {
            if let Some(x) = is_col.get_mut(c) {
                *x = true;
            }
        }
        for (i, smp) in ss.samples.iter().enumerate() {
            let col = is_col.get(i).copied().unwrap_or(false);
            if !(col || i.is_multiple_of(stride)) {
                continue;
            }
            // the corner is stated once, by the side that starts there
            if i + 1 < n {
                put(&mut hard, &mut soft, Row::Position { uv: smp.uv, at: smp.xyz }, col, 1.0);
            }
            let Some(r) = &smp.refs else { continue };
            let f = relax_factor(inp.zones, j, smp.s, len);
            put(&mut hard, &mut soft, Row::Tangent { uv: smp.uv, dir: smp.dir, normal: r.normal }, col, f);
            if sd.continuity == Continuity::G2
                && let Some(prev) = inp.prev
                && let Some(row) = curvature_row(prev, smp, r)?
            {
                put(&mut hard, &mut soft, row, col, f);
            }
        }
    }

    for p in &inp.interior.hard {
        hard.push(Row::Position { uv: p.uv, at: p.xyz });
    }
    for c in &inp.interior.curves {
        let uvs: Vec<V2> = c.iter().map(|p| p.uv).collect();
        let mut is_col = vec![false; c.len()];
        for k in collocation(&uvs, inp.layout.cell) {
            if let Some(x) = is_col.get_mut(k) {
                *x = true;
            }
        }
        for (i, p) in c.iter().enumerate() {
            let col = is_col.get(i).copied().unwrap_or(false);
            if col || i.is_multiple_of(stride) {
                put(&mut hard, &mut soft, Row::Position { uv: p.uv, at: p.xyz }, col, 1.0);
            }
        }
    }
    if hard.len() > MAX_ROWS {
        return Err(KernelError::Argument(format!("the conditions need {} hard rows; at most {MAX_ROWS}", hard.len())));
    }
    soft.truncate(MAX_ROWS - hard.len());

    let mut data = Vec::with_capacity(inp.interior.data.len());
    let mut ignored = 0usize;
    for p in &inp.interior.data {
        if inp.layout.inside(p.uv) {
            data.push(DataPoint { uv: p.uv, at: p.xyz, weight: p.w });
        } else {
            ignored += 1;
        }
    }

    let mut outline: Vec<V2> = Vec::new();
    for ss in inp.sides {
        let n = ss.samples.len();
        outline.extend(ss.samples.iter().take(n.saturating_sub(1)).map(|s| s.uv));
    }
    let outline: Vec<V2> =
        thin_indices(outline.len(), MAX_OUTLINE).into_iter().filter_map(|i| outline.get(i).copied()).collect();

    let problem = Problem {
        domain: inp.layout.domain,
        control: inp.layout.net,
        hard,
        soft,
        data,
        outline,
        smoothing: inp.smoothing,
    };
    Ok(Built { problem, ignored })
}
