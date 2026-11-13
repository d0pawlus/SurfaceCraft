//! Segmenting one chain into pieces: splitting, merging, absorbing slivers and refining the boundaries.

use super::primitive::*;
use super::types::*;
use super::*;

// ---------------------------------------------------------------------------------------------
// segmenting one chain

/// The samples of a chain; a closed chain is cyclic and has no repeated end sample.
pub(super) struct Samples {
    pub(super) q: Vec<P2>,
    pub(super) closed: bool,
}

impl Samples {
    pub(super) fn at(&self, i: usize) -> P2 {
        let m = self.q.len();
        if m == 0 {
            return [0.0, 0.0];
        }
        let k = if self.closed { i % m } else { i.min(m - 1) };
        self.q.get(k).copied().unwrap_or([0.0, 0.0])
    }

    /// Samples `s ..= s + len`.
    pub(super) fn pts(&self, s: usize, len: usize) -> Vec<P2> {
        (s..=s + len).map(|i| self.at(i)).collect()
    }

    /// The same, thinned to about `FIT_SAMPLES` (both ends kept) for quick decisions.
    fn pts_fit(&self, s: usize, len: usize) -> Vec<P2> {
        if len < FIT_SAMPLES {
            return self.pts(s, len);
        }
        let stride = len.div_ceil(FIT_SAMPLES - 1);
        let mut v: Vec<P2> = (0..len).step_by(stride).map(|k| self.at(s + k)).collect();
        v.push(self.at(s + len));
        v
    }

    pub(super) fn length(&self, s: usize, len: usize) -> f64 {
        let v = self.pts(s, len);
        v.windows(2).map(|w| dist(w[0], w[1])).sum()
    }

    fn norm_start(&self, s: usize) -> usize {
        if self.closed && !self.q.is_empty() { s % self.q.len() } else { s }
    }
}

/// A run of samples `s ..= s + len` described by one primitive.
#[derive(Clone, Copy, Debug)]
pub(super) struct Piece {
    pub(super) s: usize,
    pub(super) len: usize,
    pub(super) fit: Fit,
    pub(super) arc: bool,
}

/// Offset (1 ..= len − 1) of the sample farthest from the chord of `s ..= s + len`.
fn far_from_chord(sm: &Samples, s: usize, len: usize) -> usize {
    let (a, b) = (sm.at(s), sm.at(s + len));
    let d = unit(sub(b, a));
    let far = (1..len)
        .map(|k| {
            let p = sm.at(s + k);
            (k, d.map_or_else(|| dist(p, a), |d| cross(sub(p, a), d).abs()))
        })
        .max_by(|x, y| x.1.total_cmp(&y.1))
        .map_or(len / 2, |(k, _)| k);
    far.clamp(1, len.saturating_sub(1).max(1))
}

/// Splits the chain until every piece is a line or an arc within the tolerance.
fn split(sm: &Samples, ctx: &Ctx) -> Vec<Piece> {
    let m = sm.q.len();
    let mut stack: Vec<(usize, usize)> = Vec::new();
    if sm.closed {
        let far = (1..m)
            .max_by(|&a, &b| dist(sm.at(a), sm.at(0)).total_cmp(&dist(sm.at(b), sm.at(0))))
            .unwrap_or(m / 2)
            .clamp(1, m.saturating_sub(1).max(1));
        stack.push((far, m - far));
        stack.push((0, far));
    } else {
        stack.push((0, m.saturating_sub(1)));
    }
    let mut out: Vec<Piece> = Vec::new();
    while let Some((s, len)) = stack.pop() {
        let pts = sm.pts_fit(s, len);
        if let Some((fit, arc)) = best(&pts, ctx) {
            out.push(Piece { s, len, fit, arc });
            continue;
        }
        if len <= 1 || out.len() + stack.len() >= MAX_PIECES {
            out.push(Piece { s, len, fit: force_line(&pts), arc: false });
            continue;
        }
        let k = far_from_chord(sm, s, len);
        stack.push((sm.norm_start(s + k), len - k));
        stack.push((s, k));
    }
    out
}

/// Merges neighbouring pieces while the union of the best pair is still one primitive.
pub(super) fn merge(sm: &Samples, ctx: &Ctx, mut pieces: Vec<Piece>) -> Vec<Piece> {
    let m = sm.q.len();
    let mut cache: HashMap<(usize, usize), Option<(Fit, bool)>> = HashMap::new();
    loop {
        let n = pieces.len();
        if n < 2 {
            break;
        }
        let pairs = match (sm.closed, n) {
            (true, 2) => 1,
            (true, n) => n,
            (false, n) => n - 1,
        };
        let mut pick: Option<(usize, usize, usize, Fit, bool)> = None;
        for i in 0..pairs {
            let j = (i + 1) % n;
            let (Some(a), Some(b)) = (pieces.get(i), pieces.get(j)) else { continue };
            let len = a.len + b.len;
            if sm.closed && len > m {
                continue;
            }
            let key = (a.s, len);
            let res = *cache.entry(key).or_insert_with(|| best(&sm.pts_fit(a.s, len), ctx));
            if let Some((fit, arc)) = res
                && pick.is_none_or(|p| fit.rms < p.3.rms)
            {
                pick = Some((i, j, len, fit, arc));
            }
        }
        let Some((i, j, len, fit, arc)) = pick else { break };
        if let Some(p) = pieces.get_mut(i) {
            *p = Piece { s: p.s, len, fit, arc };
        }
        pieces.remove(j);
    }
    pieces
}

/// Absorbs pieces shorter than four tolerances into the neighbour that fits the union better.
fn absorb_tiny(sm: &Samples, ctx: &Ctx, pieces: &mut Vec<Piece>) {
    let min_len = 4.0 * ctx.tol;
    loop {
        let n = pieces.len();
        if n < 2 {
            return;
        }
        let Some((i, l)) =
            pieces.iter().enumerate().map(|(i, p)| (i, sm.length(p.s, p.len))).min_by(|a, b| a.1.total_cmp(&b.1))
        else {
            return;
        };
        if l >= min_len {
            return;
        }
        let Some(tiny) = pieces.get(i).copied() else { return };
        let mut options: Vec<(usize, Piece, f64)> = Vec::new();
        let prev = (sm.closed || i > 0).then(|| (i + n - 1) % n);
        let next = (sm.closed || i + 1 < n).then(|| (i + 1) % n);
        for (nb, before) in [(prev, true), (next, false)] {
            let Some(nb) = nb else { continue };
            let Some(o) = pieces.get(nb).copied() else { continue };
            let (s, len) = if before { (o.s, o.len + tiny.len) } else { (tiny.s, tiny.len + o.len) };
            if sm.closed && len > sm.q.len() {
                continue;
            }
            if let Some(fit) = refit_kind(&sm.pts(s, len), o.arc) {
                let cost = fit.rms * fit.rms * (len + 1) as f64;
                options.push((nb, Piece { s: sm.norm_start(s), len, fit, arc: o.arc }, cost));
            }
        }
        let Some((nb, merged, _)) = options.into_iter().min_by(|a, b| a.2.total_cmp(&b.2)) else { return };
        if let Some(slot) = pieces.get_mut(nb) {
            *slot = merged;
        }
        pieces.remove(i);
    }
}

/// Slides every boundary between two pieces to where the two refitted primitives have the least
/// squared error.
fn refine(sm: &Samples, pieces: &mut [Piece]) {
    const W: usize = 8;
    let m = sm.q.len();
    for _ in 0..3 {
        let n = pieces.len();
        if n < 2 {
            return;
        }
        let pairs = if sm.closed { n } else { n - 1 };
        let mut changed = false;
        for i in 0..pairs {
            let j = (i + 1) % n;
            let (Some(a), Some(b)) = (pieces.get(i).copied(), pieces.get(j).copied()) else { continue };
            let total = a.len + b.len;
            if total < 4 {
                continue;
            }
            let lo = 2.max(a.len.saturating_sub(W));
            let hi = (total - 2).min(a.len + W);
            let cost = |la: usize| -> f64 {
                match (refit_kind(&sm.pts(a.s, la), a.arc), refit_kind(&sm.pts(a.s + la, total - la), b.arc)) {
                    (Some(fa), Some(fb)) => {
                        fa.rms * fa.rms * (la + 1) as f64 + fb.rms * fb.rms * (total - la + 1) as f64
                    }
                    _ => f64::INFINITY,
                }
            };
            let costs: Vec<(usize, f64)> = (lo..=hi).map(|la| (la, cost(la))).collect();
            let Some((la, c)) = costs.into_iter().min_by(|x, y| x.1.total_cmp(&y.1)) else { continue };
            if la != a.len && c.is_finite() {
                if let Some(p) = pieces.get_mut(i) {
                    p.len = la;
                }
                if let Some(p) = pieces.get_mut(j) {
                    p.s = if sm.closed { (a.s + la) % m } else { a.s + la };
                    p.len = total - la;
                }
                changed = true;
            }
        }
        if !changed {
            return;
        }
    }
}

/// Refits every piece on all its samples; a piece whose arc cannot be fitted becomes a line.
pub(super) fn refit_pieces(sm: &Samples, pieces: &mut [Piece]) {
    for p in pieces.iter_mut() {
        let pts = sm.pts(p.s, p.len);
        if let Some(f) = refit_kind(&pts, p.arc) {
            p.fit = f;
        } else {
            p.arc = false;
            p.fit = force_line(&pts);
        }
    }
}

/// Removes repeated samples; `None` when the chain cannot be fitted.
fn clean(chain: &Chain2) -> Result<Samples, String> {
    if chain.points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("the chain has non-finite coordinates".into());
    }
    let mut q: Vec<P2> = Vec::with_capacity(chain.points.len());
    for p in &chain.points {
        if q.last().is_none_or(|l| dist(*l, *p) > 1e-9) {
            q.push(*p);
        }
    }
    let mut closed = chain.closed;
    if closed
        && q.len() > 1
        && let (Some(a), Some(b)) = (q.first().copied(), q.last().copied())
        && dist(a, b) <= 1e-9
    {
        q.pop();
    }
    if closed && q.len() < 4 {
        closed = false;
    }
    if q.len() > MAX_SAMPLES {
        let stride = q.len().div_ceil(MAX_SAMPLES);
        let last = q.last().copied();
        let mut t: Vec<P2> = q.iter().step_by(stride).copied().collect();
        if !closed
            && let Some(l) = last
            && t.last() != Some(&l)
        {
            t.push(l);
        }
        q = t;
    }
    if q.len() < 2 {
        return Err("the chain has fewer than two distinct samples".into());
    }
    Ok(Samples { q, closed })
}

fn extent(q: &[P2]) -> f64 {
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in q {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    dist(lo, hi)
}

/// Splits, merges and refines one chain into pieces.
pub(super) fn segment(chain: &Chain2, tol: f64) -> Result<(Samples, Vec<Piece>), String> {
    let sm = clean(chain)?;
    let ctx = Ctx { tol, rmax: 20.0 * extent(&sm.q).max(1.0) };
    if sm.length(0, sm.q.len() - 1) < 4.0 * tol {
        return Err("the chain is shorter than four tolerances".into());
    }
    let mut pieces = split(&sm, &ctx);
    for _ in 0..4 {
        let before = pieces.len();
        pieces = merge(&sm, &ctx, pieces);
        absorb_tiny(&sm, &ctx, &mut pieces);
        refine(&sm, &mut pieces);
        refit_pieces(&sm, &mut pieces);
        // boundaries moved: an arc that turned out flat is a line, and neighbours may now merge
        let mut flat = false;
        for p in pieces.iter_mut().filter(|p| p.arc && p.fit.sagitta <= FLAT * tol) {
            flat = true;
            p.arc = false;
        }
        if flat {
            refit_pieces(&sm, &mut pieces);
        }
        if !flat && pieces.len() == before {
            break;
        }
    }
    // a closed chain left as one line is a circle
    if sm.closed
        && pieces.len() == 1
        && let Some(p) = pieces.first_mut()
        && !p.arc
    {
        let all = sm.pts(0, sm.q.len());
        let f = fit_arc_raw(&all).ok_or_else(|| "the closed chain could not be fitted".to_string())?;
        *p = Piece { s: 0, len: sm.q.len(), fit: f, arc: true };
    }
    Ok((sm, pieces))
}
