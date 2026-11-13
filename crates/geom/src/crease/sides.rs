//! The sides of the surface around a hole: the triangles around each vertex, the fitted sides and the wedge they form.

use super::*;

/// The triangles around each vertex of a mesh (as it is when this is built).
pub(crate) struct Around {
    pub(super) start: Vec<u32>,
    pub(super) tris: Vec<u32>,
}

impl Around {
    /// Indexes `mesh`; triangles with a vertex out of range are left out.
    pub(crate) fn new(mesh: &Mesh) -> Self {
        let n = mesh.positions.len();
        let valid = |t: &[u32; 3]| t.iter().all(|&v| (v as usize) < n);
        let mut start = vec![0u32; n + 1];
        for t in mesh.triangles.iter().filter(|t| valid(t)) {
            for &v in t {
                if let Some(c) = start.get_mut(v as usize + 1) {
                    *c += 1;
                }
            }
        }
        for i in 1..start.len() {
            start[i] += start[i - 1];
        }
        let mut next = start.clone();
        let mut tris = vec![0u32; start.last().copied().unwrap_or(0) as usize];
        for (i, t) in mesh.triangles.iter().enumerate().filter(|(_, t)| valid(t)) {
            for &v in t {
                if let Some(slot) = next.get_mut(v as usize) {
                    if let Some(place) = tris.get_mut(*slot as usize) {
                        *place = i as u32;
                    }
                    *slot += 1;
                }
            }
        }
        Self { start, tris }
    }

    pub(super) fn of(&self, v: u32) -> &[u32] {
        let (Some(&a), Some(&b)) = (self.start.get(v as usize), self.start.get(v as usize + 1)) else { return &[] };
        self.tris.get(a as usize..b as usize).unwrap_or(&[])
    }
}

/// One smooth side of the surface around a hole, as a height field over its mean plane.
pub(super) struct Side {
    pub(super) origin: Point3,
    pub(super) n: Vec3,
    pub(super) u: Vec3,
    pub(super) v: Vec3,
    pub(super) coef: [f64; TERMS],
}

fn basis(x: f64, y: f64) -> [f64; TERMS] {
    [1.0, x, y, x * x, x * y, y * y]
}

/// Least squares of z over the first `terms` basis functions; the coefficients and the RMS left.
fn fit_height(local: &[(f64, f64, f64)], terms: usize) -> Option<([f64; TERMS], f64)> {
    let terms = terms.min(TERMS);
    let mut a = SMatrix::<f64, TERMS, TERMS>::zeros();
    let mut b = SVector::<f64, TERMS>::zeros();
    for &(x, y, z) in local {
        let f = basis(x, y);
        for i in 0..terms {
            for j in 0..terms {
                a[(i, j)] += f[i] * f[j];
            }
            b[i] += f[i] * z;
        }
    }
    for i in terms..TERMS {
        a[(i, i)] = 1.0;
    }
    let c = a.lu().solve(&b)?;
    let coef: [f64; TERMS] = std::array::from_fn(|i| c[i]);
    if coef.iter().any(|x| !x.is_finite()) {
        return None;
    }
    let height = |x: f64, y: f64| basis(x, y).iter().zip(&coef).map(|(f, c)| f * c).sum::<f64>();
    let sq: f64 = local.iter().map(|&(x, y, z)| (z - height(x, y)).powi(2)).sum();
    Some((coef, (sq / local.len().max(1) as f64).sqrt()))
}

/// Do `count` points bend away from their plane: does the quadric leave less than
/// [`QUADRIC_GAIN`] of the plane's RMS, per point left free?
pub(super) fn bends(count: usize, plane_rms: f64, quadric_rms: f64) -> bool {
    let free = |terms: usize| count.saturating_sub(terms).max(1) as f64;
    quadric_rms / free(TERMS).sqrt() < QUADRIC_GAIN * plane_rms / free(PLANE_TERMS).sqrt()
}

impl Side {
    /// The side through `points` with mean normal `n`, and the RMS it leaves: a plane, or a
    /// quadric when the points are enough and bend away from the plane.
    pub(super) fn fit(points: &[Point3], n: Vec3) -> Option<(Side, f64)> {
        if points.len() < PLANE_TERMS {
            return None;
        }
        let origin = Point3::from(points.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / points.len() as f64);
        let u = crate::any_perpendicular(&n);
        let v = n.cross(&u);
        let local: Vec<(f64, f64, f64)> = points
            .iter()
            .map(|p| {
                let d = p - origin;
                (d.dot(&u), d.dot(&v), d.dot(&n))
            })
            .collect();
        let (plane, plane_rms) = fit_height(&local, PLANE_TERMS)?;
        let (coef, rms) = match fit_height(&local, TERMS) {
            Some((quadric, rms)) if points.len() >= MIN_QUADRIC_POINTS && bends(points.len(), plane_rms, rms) => {
                (quadric, rms)
            }
            _ => (plane, plane_rms),
        };
        Some((Side { origin, n, u, v, coef }, rms))
    }

    /// Height of `p` over the side, along the side's normal.
    pub(super) fn height(&self, p: &Point3) -> f64 {
        let d = p - self.origin;
        let f = basis(d.dot(&self.u), d.dot(&self.v));
        d.dot(&self.n) - f.iter().zip(&self.coef).map(|(f, c)| f * c).sum::<f64>()
    }
}

/// The sides of the surface around a hole and whether they make a convex or a concave wedge.
pub(super) struct Wedge {
    pub(super) sides: Vec<Side>,
    pub(super) convex: bool,
}

impl Wedge {
    pub(super) fn sign(&self) -> f64 {
        if self.convex { 1.0 } else { -1.0 }
    }

    /// The side `p` is highest over (lowest under, in a concave wedge) and that signed height.
    pub(super) fn top(&self, p: &Point3) -> Option<(usize, f64)> {
        self.sides
            .iter()
            .map(|s| self.sign() * s.height(p))
            .enumerate()
            .fold(None, |b: Option<(usize, f64)>, c| if b.is_none_or(|b| c.1 > b.1) { Some(c) } else { b })
    }

    /// Distance of `p` from the surface of the wedge.
    pub(super) fn off(&self, p: &Point3) -> f64 {
        self.top(p).map_or(f64::INFINITY, |(_, h)| h.abs())
    }

    /// `p` moved onto the surface of the wedge, and that point moved on to the edge or corner
    /// when further sides are within `snap` of it (`None` when none is).
    pub(super) fn onto(&self, snap: f64, on_side: f64, p: Point3) -> (Point3, Option<Point3>) {
        let mut q = p;
        for _ in 0..PROJECTIONS {
            let Some((k, h)) = self.top(&q) else { return (p, None) };
            if h.abs() < on_side {
                break;
            }
            if let Some(s) = self.sides.get(k) {
                q -= s.n * (self.sign() * h);
            }
        }
        let close: Vec<&Side> = self.sides.iter().filter(|s| self.sign() * s.height(&q) > -snap).collect();
        if close.len() < 2 {
            return (q, None);
        }
        (q, Some(onto_all(&close, q)))
    }
}

/// `p` moved onto every one of `sides` at once, by projecting onto them in turn.
pub(super) fn onto_all(sides: &[&Side], p: Point3) -> Point3 {
    let mut q = p;
    for _ in 0..PROJECTIONS {
        for s in sides {
            q -= s.n * s.height(&q);
        }
    }
    q
}
