//! Plane geometry and the fit of one piece: lines, circles and arcs through section samples.

use super::*;

// ---------------------------------------------------------------------------------------------
// plane geometry

pub(super) fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

pub(super) fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

pub(super) fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub(super) fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

pub(super) fn unit(v: P2) -> Option<P2> {
    let n = v[0].hypot(v[1]);
    (n > 1e-12 && n.is_finite()).then(|| [v[0] / n, v[1] / n])
}

/// A fitted curve: a line (point and unit direction) or a circle.
#[derive(Clone, Copy, Debug)]
pub(super) enum Prim {
    Line { o: P2, d: P2 },
    Circle { c: P2, r: f64 },
}

impl Prim {
    pub(super) fn dist(&self, q: P2) -> f64 {
        match *self {
            Prim::Line { o, d } => cross(sub(q, o), d).abs(),
            Prim::Circle { c, r } => (dist(q, c) - r).abs(),
        }
    }

    /// Signed distance and its gradient at `q`.
    pub(super) fn signed(&self, q: P2) -> (f64, P2) {
        match *self {
            Prim::Line { o, d } => (cross(sub(q, o), d), [d[1], -d[0]]),
            Prim::Circle { c, r } => {
                let v = sub(q, c);
                let n = v[0].hypot(v[1]).max(1e-12);
                (n - r, [v[0] / n, v[1] / n])
            }
        }
    }

    /// The point of the curve nearest to `q`.
    pub(super) fn project(&self, q: P2) -> P2 {
        match *self {
            Prim::Line { o, d } => {
                let t = dot(sub(q, o), d);
                [o[0] + d[0] * t, o[1] + d[1] * t]
            }
            Prim::Circle { c, r } => match unit(sub(q, c)) {
                Some(u) => [c[0] + u[0] * r, c[1] + u[1] * r],
                None => [c[0] + r, c[1]],
            },
        }
    }
}

/// The point nearest to both curves, found from `p0` (their intersection, or the point of tangency
/// when they touch). Falls back to the midpoint of the projections when it runs away.
pub(super) fn junction(a: &Prim, b: &Prim, p0: P2, limit: f64) -> P2 {
    let mut p = p0;
    for _ in 0..40 {
        let ((ra, ga), (rb, gb)) = (a.signed(p), b.signed(p));
        let lam = 1e-6;
        let m00 = ga[0] * ga[0] + gb[0] * gb[0] + lam;
        let m01 = ga[0] * ga[1] + gb[0] * gb[1];
        let m11 = ga[1] * ga[1] + gb[1] * gb[1] + lam;
        let (b0, b1) = (-(ga[0] * ra + gb[0] * rb), -(ga[1] * ra + gb[1] * rb));
        let det = m00 * m11 - m01 * m01;
        if det.abs() < 1e-18 {
            break;
        }
        let (dx, dy) = ((b0 * m11 - b1 * m01) / det, (m00 * b1 - m01 * b0) / det);
        p = [p[0] + dx, p[1] + dy];
        if dx.hypot(dy) < 1e-12 {
            break;
        }
    }
    if p.iter().all(|v| v.is_finite()) && dist(p, p0) <= limit {
        p
    } else {
        let (pa, pb) = (a.project(p0), b.project(p0));
        [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0]
    }
}

/// Where two curves that touch within three tolerances touch (midway between their nearest
/// points); `None` when they do not touch or cross at a corner. The point of tangency of a line and
/// an arc is not fixed by the samples along the line, so it is taken from the curves alone.
pub(super) fn tangent_point(a: &Prim, b: &Prim, tol: f64) -> Option<P2> {
    let touch = 3.0 * tol;
    let mid = |p: P2, q: P2| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0];
    match (a, b) {
        (line @ Prim::Line { .. }, Prim::Circle { c, r }) | (Prim::Circle { c, r }, line @ Prim::Line { .. }) => {
            let foot = line.project(*c);
            if (dist(foot, *c) - r).abs() > touch {
                return None;
            }
            let u = unit(sub(foot, *c))?;
            Some(mid(foot, [c[0] + u[0] * r, c[1] + u[1] * r]))
        }
        (Prim::Circle { c: c1, r: r1 }, Prim::Circle { c: c2, r: r2 }) => {
            let u = unit(sub(*c2, *c1))?;
            let d = dist(*c1, *c2);
            let (ext, int) = ((d - (r1 + r2)).abs(), (d - (r1 - r2).abs()).abs());
            if ext.min(int) > touch {
                return None;
            }
            let at = |c: &P2, r: f64, s: f64| [c[0] + s * u[0] * r, c[1] + s * u[1] * r];
            let (p1, p2) = if ext <= int {
                (at(c1, *r1, 1.0), at(c2, *r2, -1.0))
            } else if r1 >= r2 {
                (at(c1, *r1, 1.0), at(c2, *r2, 1.0))
            } else {
                (at(c1, *r1, -1.0), at(c2, *r2, -1.0))
            };
            Some(mid(p1, p2))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------------------------
// fitting one piece

/// A fitted piece: its curve and how well it matches the samples.
#[derive(Clone, Copy, Debug)]
pub(super) struct Fit {
    pub(super) prim: Prim,
    pub(super) rms: f64,
    pub(super) max: f64,
    /// Distance of the arc from its chord (0 for lines).
    pub(super) sagitta: f64,
    /// Signed angle swept by the samples around the centre (0 for lines).
    pub(super) sweep: f64,
}

fn stats(prim: &Prim, pts: &[P2]) -> (f64, f64) {
    let (mut sum, mut max) = (0.0, 0.0f64);
    for p in pts {
        let d = prim.dist(*p);
        sum += d * d;
        max = max.max(d);
    }
    ((sum / pts.len().max(1) as f64).sqrt(), max)
}

/// Total least-squares line.
fn fit_line(pts: &[P2]) -> Option<Fit> {
    let n = pts.len();
    if n < 2 {
        return None;
    }
    let nf = n as f64;
    let (mx, my) = (pts.iter().map(|p| p[0]).sum::<f64>() / nf, pts.iter().map(|p| p[1]).sum::<f64>() / nf);
    let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
    for p in pts {
        let (u, v) = (p[0] - mx, p[1] - my);
        sxx += u * u;
        syy += v * v;
        sxy += u * v;
    }
    if !(sxx + syy > 1e-24) {
        return None;
    }
    let th = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let prim = Prim::Line { o: [mx, my], d: [th.cos(), th.sin()] };
    let (rms, max) = stats(&prim, pts);
    (rms.is_finite() && max.is_finite()).then_some(Fit { prim, rms, max, sagitta: 0.0, sweep: 0.0 })
}

/// Least-squares circle: algebraic start, then Gauss–Newton on the distances.
fn fit_circle_raw(pts: &[P2]) -> Option<(P2, f64)> {
    let n = pts.len();
    if n < 3 {
        return None;
    }
    let nf = n as f64;
    let (mx, my) = (pts.iter().map(|p| p[0]).sum::<f64>() / nf, pts.iter().map(|p| p[1]).sum::<f64>() / nf);
    let (mut suu, mut svv, mut suv, mut sz, mut szu, mut szv) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in pts {
        let (u, v) = (p[0] - mx, p[1] - my);
        let z = u * u + v * v;
        suu += u * u;
        svv += v * v;
        suv += u * v;
        sz += z;
        szu += z * u;
        szv += z * v;
    }
    let det = suu * svv - suv * suv;
    if !(det.abs() > 1e-14 * (suu + svv).powi(2) && det.is_finite()) {
        return None;
    }
    let a = 0.5 * (szu * svv - szv * suv) / det;
    let b = 0.5 * (szv * suu - szu * suv) / det;
    let r2 = a * a + b * b + sz / nf;
    if !(r2 > 0.0 && r2.is_finite()) {
        return None;
    }
    let (mut c, mut r) = ([mx + a, my + b], r2.sqrt());
    let cost = |c: P2, r: f64| pts.iter().map(|p| (dist(*p, c) - r).powi(2)).sum::<f64>();
    let mut best = cost(c, r);
    for _ in 0..15 {
        let mut jtj = nalgebra::Matrix3::<f64>::zeros();
        let mut jtr = nalgebra::Vector3::<f64>::zeros();
        for p in pts {
            let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
            let rho = dx.hypot(dy).max(1e-12);
            let row = nalgebra::Vector3::new(-dx / rho, -dy / rho, -1.0);
            jtj += row * row.transpose();
            jtr += row * (rho - r);
        }
        let lam = 1e-9 * (jtj.trace() + 1.0);
        for k in 0..3 {
            jtj[(k, k)] += lam;
        }
        let Some(step) = jtj.lu().solve(&(-jtr)) else { break };
        let (cn, rn) = ([c[0] + step[0], c[1] + step[1]], r + step[2]);
        let cn_cost = cost(cn, rn);
        if !(rn > 0.0 && cn_cost.is_finite()) || cn_cost >= best {
            break;
        }
        let small = step.norm() < 1e-12 * r.max(1.0);
        (c, r, best) = (cn, rn, cn_cost);
        if small {
            break;
        }
    }
    (c.iter().all(|v| v.is_finite()) && r.is_finite() && r > 0.0).then_some((c, r))
}

/// Signed angle swept by consecutive samples around `c`, and the sum of the absolute steps.
fn sweep(pts: &[P2], c: P2) -> (f64, f64) {
    let (mut net, mut abs) = (0.0, 0.0);
    let mut prev: Option<f64> = None;
    for p in pts {
        let a = (p[1] - c[1]).atan2(p[0] - c[0]);
        if let Some(q) = prev {
            let mut d = a - q;
            while d > PI {
                d -= TAU;
            }
            while d <= -PI {
                d += TAU;
            }
            net += d;
            abs += d.abs();
        }
        prev = Some(a);
    }
    (net, abs)
}

/// A circle through the samples with its sweep, without judging whether it is an arc.
pub(super) fn fit_arc_raw(pts: &[P2]) -> Option<Fit> {
    let (c, r) = fit_circle_raw(pts)?;
    let prim = Prim::Circle { c, r };
    let (rms, max) = stats(&prim, pts);
    let (net, _) = sweep(pts, c);
    let sagitta = r * (1.0 - (net.abs().min(PI) / 2.0).cos());
    (rms.is_finite() && max.is_finite()).then_some(Fit { prim, rms, max, sagitta, sweep: net })
}

pub(super) struct Ctx {
    pub(super) tol: f64,
    /// Largest radius of an arc.
    pub(super) rmax: f64,
}

fn accepts(f: &Fit, tol: f64) -> bool {
    f.rms <= RMS_FACTOR * tol && f.max <= MAX_FACTOR * tol
}

/// Is the sample run an arc: at least 5 samples, one direction around the centre, at most a full turn.
fn fit_arc(pts: &[P2], ctx: &Ctx) -> Option<Fit> {
    if pts.len() < 5 {
        return None;
    }
    let f = fit_arc_raw(pts)?;
    let Prim::Circle { c, r } = f.prim else { return None };
    let (net, abs) = sweep(pts, c);
    (r <= ctx.rmax && abs > 1e-9 && net.abs() >= 0.8 * abs && net.abs() <= TAU * 1.02).then_some(f)
}

/// The primitive the samples are, if any: a line, or an arc that is clearly curved.
pub(super) fn best(pts: &[P2], ctx: &Ctx) -> Option<(Fit, bool)> {
    let tol = ctx.tol;
    let line = fit_line(pts).filter(|f| accepts(f, tol));
    if let Some(l) = line
        && (pts.len() < 5 || l.max <= 0.6 * tol)
    {
        return Some((l, false));
    }
    let arc = fit_arc(pts, ctx).filter(|f| accepts(f, tol) && f.sagitta > FLAT * tol);
    match (line, arc) {
        (Some(l), Some(a)) => Some(if l.rms > 1.5 * a.rms { (a, true) } else { (l, false) }),
        (Some(l), None) => Some((l, false)),
        (None, Some(a)) => Some((a, true)),
        (None, None) => None,
    }
}

pub(super) fn force_line(pts: &[P2]) -> Fit {
    fit_line(pts).unwrap_or_else(|| {
        let o = pts.first().copied().unwrap_or([0.0, 0.0]);
        Fit { prim: Prim::Line { o, d: [1.0, 0.0] }, rms: 0.0, max: 0.0, sagitta: 0.0, sweep: 0.0 }
    })
}

pub(super) fn refit_kind(pts: &[P2], arc: bool) -> Option<Fit> {
    if arc { fit_arc_raw(pts) } else { fit_line(pts) }
}
