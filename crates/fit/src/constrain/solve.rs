//! Imposing the relations: the smallest parameter change that satisfies them, solved per group of coupled primitives.

use super::geometry::*;
use super::rows::*;
use super::types::*;
use super::*;

// ------------------------------------------------------------------------------------------
// imposing relations

/// Makes the directions of every symmetric pair mirror images of each other (their mean): a
/// no-op when they already are, as for axis-aligned designs.
fn symmetrise_directions(prims: &mut [Primitive], mirrors: &[Mirror], rels: &[Rel]) {
    for r in rels.iter().filter(|r| r.kind == RelKind::Symmetric) {
        let Some(m) = r.mirror.and_then(|k| mirrors.get(k)) else { continue };
        let (Some(da), Some(db)) = (prims.get(r.a).and_then(direction), prims.get(r.b).and_then(direction)) else {
            continue;
        };
        let signed = matches!(prims.get(r.a), Some(Primitive::Cone { .. }));
        let mut mb = m.dir(&db);
        if !signed && da.dot(&mb) < 0.0 {
            mb = -mb;
        }
        let Some(mean) = normalize(da + mb) else { continue };
        let mut new_b = m.dir(&mean);
        if !signed && db.dot(&new_b) < 0.0 {
            new_b = -new_b;
        }
        if (mean - da).norm() < 1e-14 && (new_b - db).norm() < 1e-14 {
            continue;
        }
        if let Some(p) = prims.get_mut(r.a) {
            set_direction(p, &mean);
        }
        if let Some(p) = prims.get_mut(r.b) {
            set_direction(p, &new_b);
        }
    }
}

fn find_root(parent: &[usize], i: usize) -> usize {
    let mut r = i;
    while parent.get(r).copied().unwrap_or(r) != r {
        r = parent.get(r).copied().unwrap_or(r);
    }
    r
}

fn join(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find_root(parent, a), find_root(parent, b));
    if ra != rb
        && let Some(p) = parent.get_mut(ra.max(rb))
    {
        *p = ra.min(rb);
    }
}

fn sparse_dot(a: &[(usize, f64)], b: &[(usize, f64)]) -> f64 {
    let (mut i, mut j, mut sum) = (0usize, 0usize, 0.0);
    while let (Some(x), Some(y)) = (a.get(i), b.get(j)) {
        match x.0.cmp(&y.0) {
            Ordering::Less => i += 1,
            Ordering::Greater => j += 1,
            Ordering::Equal => {
                sum += x.1 * y.1;
                i += 1;
                j += 1;
            }
        }
    }
    sum
}

/// LU factorisation with partial pivoting of a dense `n × n` matrix (row-major).
struct Lu {
    a: Vec<f64>,
    piv: Vec<usize>,
    n: usize,
}

impl Lu {
    pub(super) fn new(mut a: Vec<f64>, n: usize) -> Option<Lu> {
        if a.len() != n * n {
            return None;
        }
        let mut piv: Vec<usize> = (0..n).collect();
        for k in 0..n {
            let mut p = k;
            let mut best = a.get(k * n + k)?.abs();
            for i in (k + 1)..n {
                let v = a.get(i * n + k)?.abs();
                if v > best {
                    best = v;
                    p = i;
                }
            }
            if !(best > 1e-300) {
                return None;
            }
            if p != k {
                for j in 0..n {
                    a.swap(k * n + j, p * n + j);
                }
                piv.swap(k, p);
            }
            let akk = *a.get(k * n + k)?;
            for i in (k + 1)..n {
                let f = *a.get(i * n + k)? / akk;
                *a.get_mut(i * n + k)? = f;
                if f != 0.0 {
                    for j in (k + 1)..n {
                        let akj = *a.get(k * n + j)?;
                        *a.get_mut(i * n + j)? -= f * akj;
                    }
                }
            }
        }
        Some(Lu { a, piv, n })
    }

    fn at(&self, i: usize, j: usize) -> f64 {
        self.a.get(i * self.n + j).copied().unwrap_or(0.0)
    }

    fn solve(&self, b: &[f64]) -> Vec<f64> {
        let n = self.n;
        let mut x: Vec<f64> = self.piv.iter().map(|&p| b.get(p).copied().unwrap_or(0.0)).collect();
        for i in 0..n {
            let mut sum = x.get(i).copied().unwrap_or(0.0);
            for j in 0..i {
                sum -= self.at(i, j) * x.get(j).copied().unwrap_or(0.0);
            }
            if let Some(v) = x.get_mut(i) {
                *v = sum;
            }
        }
        for i in (0..n).rev() {
            let mut sum = x.get(i).copied().unwrap_or(0.0);
            for j in (i + 1)..n {
                sum -= self.at(i, j) * x.get(j).copied().unwrap_or(0.0);
            }
            if let Some(v) = x.get_mut(i) {
                *v = sum / self.at(i, i);
            }
        }
        x
    }
}

/// The smallest change `x` of the variables with `A x = rhs` for every row, and the residual of
/// each row. Rows are grouped by the primitives they couple; each group is solved on its own.
/// Errors carry a message and the row to blame.
fn solve_rows(rows: &[Row], nvars: usize) -> Result<(Vec<f64>, Vec<f64>), (String, usize)> {
    let mut parent: Vec<usize> = (0..nvars / 5 + 1).collect();
    for row in rows {
        let mut prims = row.coef.iter().map(|(v, _)| *v / 5);
        if let Some(first) = prims.next() {
            for other in prims {
                join(&mut parent, first, other);
            }
        }
    }
    let mut groups: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (ri, row) in rows.iter().enumerate() {
        let root = row.coef.first().map_or(0, |(v, _)| find_root(&parent, *v / 5));
        groups.entry(root).or_default().push(ri);
    }
    let mut x = vec![0.0; nvars];
    let mut resid = vec![0.0; rows.len()];
    for idx in groups.into_values() {
        let first = idx.first().copied().unwrap_or(0);
        let rs: Vec<&Row> = idx.iter().filter_map(|&i| rows.get(i)).collect();
        let k = rs.len();
        if k > MAX_COMPONENT_ROWS {
            return Err(("too many relations are coupled to solve them together".into(), first));
        }
        let mut g = vec![0.0; k * k];
        for a in 0..k {
            for b in a..k {
                let (Some(ra), Some(rb)) = (rs.get(a), rs.get(b)) else { continue };
                let d = sparse_dot(&ra.coef, &rb.coef);
                if let Some(v) = g.get_mut(a * k + b) {
                    *v = d;
                }
                if let Some(v) = g.get_mut(b * k + a) {
                    *v = d;
                }
            }
        }
        let mean_diag = (0..k).map(|a| g.get(a * k + a).copied().unwrap_or(0.0)).sum::<f64>() / (k.max(1) as f64);
        let lambda = 1e-10 * mean_diag.max(1e-12);
        for a in 0..k {
            if let Some(v) = g.get_mut(a * k + a) {
                *v += lambda;
            }
        }
        let Some(lu) = Lu::new(g, k) else {
            return Err(("the relations cannot be solved together".into(), first));
        };
        let mut r: Vec<f64> = rs.iter().map(|row| row.rhs).collect();
        for _ in 0..4 {
            let y = lu.solve(&r);
            for (row, yi) in rs.iter().zip(&y) {
                for (v, c) in &row.coef {
                    if let Some(xv) = x.get_mut(*v) {
                        *xv += c * yi;
                    }
                }
            }
            let mut worst = 0.0_f64;
            for (slot, row) in r.iter_mut().zip(&rs) {
                let ax: f64 = row.coef.iter().map(|(v, c)| c * x.get(*v).copied().unwrap_or(0.0)).sum();
                *slot = row.rhs - ax;
                worst = worst.max(slot.abs());
            }
            if worst < 1e-13 {
                break;
            }
        }
        for (e, &ri) in r.iter().zip(&idx) {
            if let Some(out) = resid.get_mut(ri) {
                *out = e.abs();
            }
        }
    }
    if x.iter().any(|v| !v.is_finite()) {
        return Err(("the relations gave a non-finite solution".into(), 0));
    }
    Ok((x, resid))
}

/// Imposes the relations on the primitives by the smallest change of their positions and radii.
/// Directions are only touched to make symmetric pairs mirror images. The primitives are left
/// unchanged when this fails (conflicting relations, an invalid result).
///
/// # Errors
/// A [`Failure`] naming the relation that conflicts most when the relations cannot all hold.
pub fn apply(prims: &mut [Primitive], mirrors: &[Mirror], rels: &[Rel]) -> Result<(), Failure> {
    let mut work: Vec<Primitive> = prims.to_vec();
    symmetrise_directions(&mut work, mirrors, rels);
    let mut rows: Vec<Row> = Vec::new();
    let mut owner: Vec<usize> = Vec::new();
    for (ri, r) in rels.iter().enumerate() {
        if let Some((rs, _, _)) = rows_of(&work, mirrors, r) {
            for row in rs {
                rows.push(merge_row(&row));
                owner.push(ri);
            }
        }
    }
    if rows.is_empty() {
        for (dst, src) in prims.iter_mut().zip(work) {
            *dst = src;
        }
        return Ok(());
    }
    if rows.len() > MAX_ROWS {
        return Err(Failure { message: "too many relations to solve together".into(), worst: None });
    }
    let (x, resid) = solve_rows(&rows, work.len() * 5)
        .map_err(|(message, row)| Failure { message, worst: owner.get(row).copied() })?;
    let (mut worst_row, mut worst) = (0usize, 0.0_f64);
    for (ri, e) in resid.iter().enumerate() {
        if *e > worst {
            worst = *e;
            worst_row = ri;
        }
    }
    if !(worst <= CONSISTENT) {
        return Err(Failure {
            message: format!("the relations conflict with each other (residual {worst:.2e} mm)"),
            worst: owner.get(worst_row).copied(),
        });
    }
    for (i, p) in work.iter_mut().enumerate() {
        let g = |k: usize| x.get(var(i, k)).copied().unwrap_or(0.0);
        move_anchor(p, &Vec3::new(g(0), g(1), g(2)));
        shift_radii(p, [g(3), g(4)]);
        if !p.is_valid() {
            return Err(Failure { message: "the snapped surface would be degenerate".into(), worst: None });
        }
    }
    for (dst, src) in prims.iter_mut().zip(work) {
        *dst = src;
    }
    Ok(())
}
