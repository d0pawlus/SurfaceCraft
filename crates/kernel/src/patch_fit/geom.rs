//! Small vector helpers of the patch driver: plain arrays, no kernel calls. Planar helpers
//! (signed area, a self-intersection sweep) serve the loop checks of [`super::start`].

pub(super) type V3 = [f64; 3];
pub(super) type V2 = [f64; 2];

/// Largest coordinate magnitude accepted from callers (squares of it stay finite).
pub(super) const COORD_LIMIT: f64 = 1e12;

pub(super) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

pub(super) fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

pub(super) fn scale(a: V3, k: f64) -> V3 {
    [a[0] * k, a[1] * k, a[2] * k]
}

pub(super) fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub(super) fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

pub(super) fn norm(a: V3) -> f64 {
    dot(a, a).sqrt()
}

pub(super) fn dist(a: V3, b: V3) -> f64 {
    norm(sub(a, b))
}

/// `a / |a|`, `None` for a vector that is zero or not finite.
pub(super) fn unit(a: V3) -> Option<V3> {
    let n = norm(a);
    if n.is_finite() && n > 1e-300 { Some(scale(a, 1.0 / n)) } else { None }
}

pub(super) fn finite3(a: &V3) -> bool {
    a.iter().all(|v| v.is_finite() && v.abs() <= COORD_LIMIT)
}

pub(super) fn finite2(a: &V2) -> bool {
    a.iter().all(|v| v.is_finite() && v.abs() <= COORD_LIMIT)
}

pub(super) fn dist2(a: V2, b: V2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// `len` indices thinned to at most `cap` (≥ 2), evenly, first and last kept.
pub(super) fn thin_indices(len: usize, cap: usize) -> Vec<usize> {
    if len <= cap || cap < 2 {
        return (0..len).collect();
    }
    (0..cap).map(|k| k * (len - 1) / (cap - 1)).collect()
}

/// Twice the signed area of a closed polygon (positive: counter-clockwise).
pub(super) fn area2(p: &[V2]) -> f64 {
    let n = p.len();
    (0..n)
        .map(|i| {
            let (a, b) = (p[i], p[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum()
}

/// Is `p` inside the closed polygon (even-odd rule)? A point on the boundary counts either way.
pub(super) fn inside_polygon(poly: &[V2], p: V2) -> bool {
    let Some(&last) = poly.last() else { return false };
    let mut inside = false;
    let mut prev = last;
    for &cur in poly {
        if (prev[1] > p[1]) != (cur[1] > p[1]) {
            let x = prev[0] + (p[1] - prev[1]) * (cur[0] - prev[0]) / (cur[1] - prev[1]);
            if p[0] < x {
                inside = !inside;
            }
        }
        prev = cur;
    }
    inside
}

fn cross2(o: V2, a: V2, b: V2) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

fn point_segment_distance(p: V2, a: V2, b: V2) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
}

/// Distance between the closed segments `ab` and `cd` (zero when they cross or touch).
fn segment_gap(a: V2, b: V2, c: V2, d: V2) -> f64 {
    let (d1, d2, d3, d4) = (cross2(a, b, c), cross2(a, b, d), cross2(c, d, a), cross2(c, d, b));
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)) {
        return 0.0;
    }
    [
        point_segment_distance(a, c, d),
        point_segment_distance(b, c, d),
        point_segment_distance(c, a, b),
        point_segment_distance(d, a, b),
    ]
    .into_iter()
    .fold(f64::INFINITY, f64::min)
}

/// Two consecutive segments `s→p` and `s→q` fold back over each other (a spike).
fn folds_back(s: V2, p: V2, q: V2, eps: f64) -> bool {
    let (u, v) = ([p[0] - s[0], p[1] - s[1]], [q[0] - s[0], q[1] - s[1]]);
    let dot = u[0] * v[0] + u[1] * v[1];
    dot > 0.0 && (point_segment_distance(q, s, p) <= eps || point_segment_distance(p, s, q) <= eps)
}

/// Does the closed polygon cross, touch or fold over itself? Segments are swept in order of their
/// smallest `x`, so honest outlines cost little. Needs at least 3 corners, no repeated neighbours.
pub(super) fn polygon_self_intersects(poly: &[V2], eps: f64) -> bool {
    let n = poly.len();
    if n < 3 {
        return true;
    }
    let seg = |i: usize| (poly[i % n], poly[(i + 1) % n]);
    let lo = |i: usize| {
        let (a, b) = seg(i);
        a[0].min(b[0])
    };
    let hi = |i: usize| {
        let (a, b) = seg(i);
        a[0].max(b[0])
    };
    let mut order: Vec<usize> = (0..n).collect();
    order.sort_by(|&x, &y| lo(x).total_cmp(&lo(y)));
    for (k, &i) in order.iter().enumerate() {
        let (a, b) = seg(i);
        for &j in order.iter().skip(k + 1) {
            if lo(j) > hi(i) + eps {
                break;
            }
            let (c, d) = seg(j);
            if (i + 1) % n == j {
                if folds_back(b, a, d, eps) {
                    return true;
                }
            } else if (j + 1) % n == i {
                if folds_back(a, b, c, eps) {
                    return true;
                }
            } else if segment_gap(a, b, c, d) <= eps {
                return true;
            }
        }
    }
    false
}
