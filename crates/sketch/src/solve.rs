//! The constraint solver and the degree-of-freedom analysis.
use crate::constraint::{Layout, entity_residuals};
use crate::{Entity, Sketch, SketchError};
use nalgebra::{DMatrix, DVector};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

/// How well constrained a sketch is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum SolveStatus {
    /// Solved, no freedom left.
    FullyDefined,
    /// Solved, some freedom left (`dof` > 0).
    UnderDefined,
    /// Some constraints are redundant or conflict ("sketch is overdefined").
    OverDefined,
    /// The constraints could not be satisfied (no redundancy found).
    Failed,
}

/// What a solve did.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SolveReport {
    /// Status.
    pub status: SolveStatus,
    /// Remaining degrees of freedom.
    pub dof: usize,
    /// Largest |residual| at the end (mm or unitless for angles).
    pub residual: f64,
    /// Iterations.
    pub iterations: usize,
    /// Constraints that add nothing (indices).
    pub redundant: Vec<usize>,
    /// Redundant constraints that are not satisfied (they contradict the others).
    pub conflicting: Vec<usize>,
    /// Human-readable summary.
    pub message: String,
}

const TOL: f64 = 1e-10;

/// Largest residual at which a solve still counts as satisfying its constraints (the bound of
/// [`SolveStatus::Failed`]).
pub const SATISFIED_TOL: f64 = 1e-6;

impl SolveReport {
    /// Are all constraints met, redundant ones included (residual within [`SATISFIED_TOL`])?
    #[must_use]
    pub fn is_satisfied(&self) -> bool {
        self.residual.is_finite() && self.residual <= SATISFIED_TOL
    }
}

fn unknowns(s: &Sketch, l: &Layout) -> Vec<f64> {
    let mut x: Vec<f64> = s.points.iter().flat_map(|p| [p[0], p[1]]).collect();
    x.resize(l.n, 0.0);
    for (e, v) in s.entities.iter().zip(&l.radius_var) {
        if let (Entity::Circle { radius, .. }, Some(k)) = (e, v)
            && let Some(slot) = x.get_mut(*k)
        {
            *slot = *radius;
        }
    }
    x
}

fn write_back(s: &mut Sketch, l: &Layout, x: &[f64]) {
    for (i, p) in s.points.iter_mut().enumerate() {
        if let (Some(a), Some(b)) = (x.get(2 * i), x.get(2 * i + 1)) {
            *p = [*a, *b];
        }
    }
    for (e, v) in s.entities.iter_mut().zip(&l.radius_var) {
        if let (Entity::Circle { radius, .. }, Some(k)) = (e, v)
            && let Some(r) = x.get(*k)
        {
            *radius = r.abs();
        }
    }
}

/// All residuals and, per constraint, its row range (entity rows first).
fn residuals(s: &Sketch, l: &Layout, x: &[f64]) -> (Vec<f64>, Vec<(usize, usize)>) {
    let mut out = Vec::new();
    entity_residuals(s, x, &mut out);
    let mut ranges = Vec::with_capacity(s.constraints.len());
    for c in &s.constraints {
        let start = out.len();
        c.residuals(s, l, x, &mut out);
        ranges.push((start, out.len()));
    }
    (out, ranges)
}

fn jacobian(s: &Sketch, l: &Layout, x: &[f64], m: usize) -> DMatrix<f64> {
    let n = x.len();
    let mut j = DMatrix::zeros(m, n);
    let mut xp = x.to_vec();
    for k in 0..n {
        let h = 1e-7 * (1.0 + x.get(k).map_or(0.0, |v| v.abs()));
        let orig = xp.get(k).copied().unwrap_or(0.0);
        if let Some(v) = xp.get_mut(k) {
            *v = orig + h;
        }
        let (fp, _) = residuals(s, l, &xp);
        if let Some(v) = xp.get_mut(k) {
            *v = orig - h;
        }
        let (fm, _) = residuals(s, l, &xp);
        if let Some(v) = xp.get_mut(k) {
            *v = orig;
        }
        for (r, (a, b)) in fp.iter().zip(&fm).enumerate().take(m) {
            j[(r, k)] = (a - b) / (2.0 * h);
        }
    }
    j
}

/// Largest absolute value; infinite when any value is not finite (`f64::max` would drop a NaN).
fn max_abs(v: &[f64]) -> f64 {
    if v.iter().any(|x| !x.is_finite()) {
        return f64::INFINITY;
    }
    v.iter().fold(0.0f64, |m, x| m.max(x.abs()))
}

fn overflow() -> SketchError {
    SketchError::Value("a constraint overflows: the sketch coordinates are too large to solve".into())
}

/// Solves the sketch in place and reports its state.
pub fn solve(s: &mut Sketch) -> Result<SolveReport, SketchError> {
    s.validate()?;
    let l = Layout::of(s);
    let mut x = unknowns(s, &l);
    let (mut f, _) = residuals(s, &l, &x);
    if !f.iter().all(|v| v.is_finite()) {
        return Err(overflow());
    }
    let m = f.len();
    let mut cost: f64 = f.iter().map(|v| v * v).sum();
    let mut lambda = 1e-6;
    let mut it = 0;
    while it < 200 && max_abs(&f) > TOL && m > 0 {
        it += 1;
        let j = jacobian(s, &l, &x, m);
        let jt = j.transpose();
        let a = &jt * &j;
        let g = &jt * DVector::from_vec(f.clone());
        let mut improved = false;
        for _ in 0..20 {
            let mut d = a.clone();
            for k in 0..x.len() {
                d[(k, k)] += lambda * (1.0 + a[(k, k)]);
            }
            let Some(step) = d.lu().solve(&(-&g)) else {
                lambda *= 10.0;
                continue;
            };
            let xn: Vec<f64> = x.iter().zip(step.iter()).map(|(a, b)| a + b).collect();
            let (fnew, _) = residuals(s, &l, &xn);
            let c: f64 = fnew.iter().map(|v| v * v).sum();
            if c.is_finite() && c < cost {
                x = xn;
                f = fnew;
                cost = c;
                lambda = (lambda * 0.1).max(1e-15);
                improved = true;
                break;
            }
            lambda *= 10.0;
        }
        if !improved {
            break;
        }
    }
    let residual = max_abs(&f);
    if !residual.is_finite() || !x.iter().all(|v| v.is_finite()) {
        return Err(overflow());
    }
    write_back(s, &l, &x);
    // rank analysis: rows in order, a constraint is redundant when its rows add no rank
    let (f2, ranges) = residuals(s, &l, &x);
    let j = jacobian(s, &l, &x, f2.len());
    let scale = j.iter().fold(0.0f64, |m, v| m.max(v.abs())).max(1.0);
    let mut basis: Vec<DVector<f64>> = Vec::new();
    let mut add_row = |row: DVector<f64>| -> bool {
        let mut r = row;
        for b in &basis {
            let c = r.dot(b);
            r -= b * c;
        }
        let n = r.norm();
        if n > 1e-7 * scale {
            basis.push(r / n);
            true
        } else {
            false
        }
    };
    let entity_rows = ranges.first().map_or(f2.len(), |r| r.0);
    for r in 0..entity_rows {
        add_row(j.row(r).transpose());
    }
    let mut redundant = Vec::new();
    let mut conflicting = Vec::new();
    for (ci, (a, b)) in ranges.iter().enumerate() {
        let mut all = true;
        for r in *a..*b {
            if !add_row(j.row(r).transpose()) {
                all = false;
            }
        }
        if !all {
            redundant.push(ci);
            if f2.get(*a..*b).is_some_and(|v| max_abs(v) > 1e-6) {
                conflicting.push(ci);
            }
        }
    }
    let rank = basis.len();
    let dof = x.len().saturating_sub(rank);
    let (status, message) = if !conflicting.is_empty() {
        (
            SolveStatus::OverDefined,
            format!("sketch is overdefined: constraints {conflicting:?} conflict with the others"),
        )
    } else if !redundant.is_empty() {
        (SolveStatus::OverDefined, format!("sketch is overdefined: constraints {redundant:?} are redundant"))
    } else if residual > SATISFIED_TOL {
        (SolveStatus::Failed, format!("the constraints cannot be satisfied (residual {residual:.3e})"))
    } else if dof == 0 {
        (SolveStatus::FullyDefined, "fully defined".into())
    } else {
        (SolveStatus::UnderDefined, format!("{dof} degrees of freedom left"))
    };
    Ok(SolveReport { status, dof, residual, iterations: it, redundant, conflicting, message })
}

/// Moves the unknowns (points and radii) of a solved sketch to minimise the sum of squares of
/// `data` (residuals that depend on the sketch, such as the distances of scan samples from their
/// entities) while every constraint stays satisfied: equality-constrained Gauss–Newton, each
/// step a KKT solve, each trial projected back onto the constraints. Returns the final sum of
/// squares; the sketch is left as it was when no step reduces it.
pub fn polish(s: &mut Sketch, data: &dyn Fn(&Sketch) -> Vec<f64>) -> Result<f64, SketchError> {
    s.validate()?;
    let l = Layout::of(s);
    let mut x = unknowns(s, &l);
    let n = x.len();
    let mut scratch = s.clone();
    let mut eval = |x: &[f64]| {
        write_back(&mut scratch, &l, x);
        data(&scratch)
    };
    let sumsq = |v: &[f64]| v.iter().map(|a| a * a).sum::<f64>();
    let mut r = eval(&x);
    let mut cost = sumsq(&r);
    if !cost.is_finite() || r.is_empty() || n == 0 {
        return Ok(cost);
    }
    for _ in 0..40 {
        let m = r.len();
        let mut jd = DMatrix::<f64>::zeros(m, n);
        let mut xp = x.clone();
        for k in 0..n {
            let h = 1e-6 * (1.0 + x.get(k).map_or(0.0, |v| v.abs()));
            let orig = xp.get(k).copied().unwrap_or(0.0);
            if let Some(v) = xp.get_mut(k) {
                *v = orig + h;
            }
            let rp = eval(&xp);
            if let Some(v) = xp.get_mut(k) {
                *v = orig - h;
            }
            let rm = eval(&xp);
            if let Some(v) = xp.get_mut(k) {
                *v = orig;
            }
            for (i, (a, b)) in rp.iter().zip(&rm).enumerate().take(m) {
                jd[(i, k)] = (a - b) / (2.0 * h);
            }
        }
        let (f, _) = residuals(s, &l, &x);
        let mc = f.len();
        let jc = jacobian(s, &l, &x, mc);
        let jt = jd.transpose();
        let mut a = &jt * &jd;
        let mu = 1e-9 * (1.0 + a.trace() / n as f64);
        for k in 0..n {
            a[(k, k)] += mu;
        }
        let g = &jt * DVector::from_column_slice(&r);
        let mut kkt = DMatrix::<f64>::zeros(n + mc, n + mc);
        kkt.view_mut((0, 0), (n, n)).copy_from(&a);
        kkt.view_mut((0, n), (n, mc)).copy_from(&jc.transpose());
        kkt.view_mut((n, 0), (mc, n)).copy_from(&jc);
        for i in 0..mc {
            kkt[(n + i, n + i)] = -1e-12;
        }
        let mut rhs = DVector::<f64>::zeros(n + mc);
        rhs.rows_mut(0, n).copy_from(&(-g));
        rhs.rows_mut(n, mc).copy_from(&(-DVector::from_vec(f)));
        let Some(sol) = kkt.lu().solve(&rhs) else { break };
        let mut t = 1.0;
        let mut accepted = false;
        for _ in 0..10 {
            let mut xn: Vec<f64> = x.iter().enumerate().map(|(k, v)| v + t * sol[k]).collect();
            restore(s, &l, &mut xn);
            let rn = eval(&xn);
            let cn = sumsq(&rn);
            if cn.is_finite() && cn < cost {
                let gain = cost - cn;
                (x, r, cost) = (xn, rn, cn);
                accepted = true;
                if gain < 1e-10 * cost.max(1e-30) {
                    accepted = false;
                }
                break;
            }
            t *= 0.5;
        }
        if !accepted {
            break;
        }
    }
    write_back(s, &l, &x);
    Ok(cost)
}

/// Newton steps of least norm that bring `x` back onto the constraints.
fn restore(s: &Sketch, l: &Layout, x: &mut [f64]) {
    for _ in 0..8 {
        let (f, _) = residuals(s, l, x);
        if max_abs(&f) < 1e-11 || f.is_empty() {
            break;
        }
        let jc = jacobian(s, l, x, f.len());
        let mut jjt = &jc * jc.transpose();
        for k in 0..f.len() {
            jjt[(k, k)] += 1e-12;
        }
        let Some(y) = jjt.lu().solve(&DVector::from_vec(f)) else { break };
        let dx = -(jc.transpose() * y);
        for (xi, d) in x.iter_mut().zip(dx.iter()) {
            *xi += d;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Constraint as C;

    /// A rectangle 40 × 20 at the origin, as a designer would constrain it.
    fn rectangle() -> Sketch {
        let mut s = Sketch::default();
        let p: Vec<usize> =
            [[0.1, -0.2], [39.0, 0.3], [41.0, 19.5], [-0.4, 21.0]].iter().map(|q| s.add_point(*q)).collect();
        for k in 0..4 {
            s.entities.push(Entity::Line { a: p[k], b: p[(k + 1) % 4] });
        }
        s.constraints = vec![
            C::Fix { point: p[0], at: [0.0, 0.0] },
            C::Horizontal { line: 0 },
            C::Vertical { line: 1 },
            C::Horizontal { line: 2 },
            C::Vertical { line: 3 },
            C::Length { line: 0, value: 40.0 },
            C::Length { line: 1, value: 20.0 },
        ];
        s
    }

    /// Gate M5: the solver satisfies a well-posed sketch and reports it fully defined.
    #[test]
    fn gate_m5_rectangle_fully_defined() {
        let mut s = rectangle();
        let r = solve(&mut s).unwrap();
        assert_eq!(r.status, SolveStatus::FullyDefined, "{r:?}");
        assert_eq!(r.dof, 0);
        assert!((s.points[2][0] - 40.0).abs() < 1e-9 && (s.points[2][1] - 20.0).abs() < 1e-9, "{:?}", s.points);
    }

    /// Gate M5: over-definition is detected — a redundant constraint, and a conflicting one.
    #[test]
    fn gate_m5_overdefinition_detected() {
        let mut s = rectangle();
        // the opposite side's length is implied: redundant but consistent
        s.constraints.push(C::Length { line: 2, value: 40.0 });
        let r = solve(&mut s).unwrap();
        assert_eq!(r.status, SolveStatus::OverDefined, "{r:?}");
        assert_eq!(r.redundant, vec![7]);
        assert!(r.conflicting.is_empty());
        // a contradicting length: conflicting
        let mut s = rectangle();
        s.constraints.push(C::Length { line: 2, value: 30.0 });
        let r = solve(&mut s).unwrap();
        assert_eq!(r.status, SolveStatus::OverDefined, "{r:?}");
        assert!(!r.conflicting.is_empty(), "{r:?}");
        assert!(r.message.contains("overdefined"));
    }

    #[test]
    fn under_defined_counts_dof_and_circles_arcs_work() {
        let mut s = rectangle();
        s.constraints.pop();
        let r = solve(&mut s).unwrap();
        assert_eq!((r.status, r.dof), (SolveStatus::UnderDefined, 1));
        // a circle tangent to a fixed horizontal line, centre fixed: radius follows
        let mut s = Sketch::default();
        let a = s.add_point([0.0, 0.0]);
        let b = s.add_point([10.0, 0.0]);
        s.entities.push(Entity::Line { a, b });
        let c = s.add_point([5.0, 3.0]);
        s.entities.push(Entity::Circle { center: c, radius: 1.0 });
        s.constraints = vec![
            C::Fix { point: a, at: [0.0, 0.0] },
            C::Fix { point: b, at: [10.0, 0.0] },
            C::Fix { point: c, at: [5.0, 4.0] },
            C::Tangent { line: 0, entity: 1 },
        ];
        let r = solve(&mut s).unwrap();
        assert_eq!(r.status, SolveStatus::FullyDefined, "{r:?}");
        let Entity::Circle { radius, .. } = s.entities[1] else { panic!() };
        assert!((radius - 4.0).abs() < 1e-9);
        // an arc keeps its ends on one circle and takes a radius
        let mut s = Sketch::default();
        let c = s.add_point([0.0, 0.0]);
        let a = s.add_point([5.0, 0.0]);
        let b = s.add_point([0.0, 4.0]);
        s.entities.push(Entity::Arc { center: c, a, b });
        s.constraints = vec![C::Fix { point: c, at: [0.0, 0.0] }, C::Radius { entity: 0, value: 6.0 }];
        let r = solve(&mut s).unwrap();
        assert!(r.residual < 1e-9, "{r:?}");
        let d = |i: usize| s.points[i][0].hypot(s.points[i][1]);
        assert!((d(a) - 6.0).abs() < 1e-9 && (d(b) - 6.0).abs() < 1e-9);
    }

    #[test]
    fn bad_sketches_are_errors() {
        let mut s = rectangle();
        s.constraints.push(C::Horizontal { line: 99 });
        assert!(solve(&mut s).is_err());
        let mut s = rectangle();
        s.points[0][0] = f64::NAN;
        assert!(solve(&mut s).is_err());
    }
}
