//! Vectors of the hybrid model: the plain `[f64; 3]` helpers, frames and the corners of a box.

pub(crate) type V = [f64; 3];

pub(crate) fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(crate) fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
pub(crate) fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
pub(crate) fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(crate) fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
pub(crate) fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}
pub(crate) fn unit(a: V) -> Option<V> {
    let n = norm(a);
    (n.is_finite() && n > 1e-12).then(|| scale(a, 1.0 / n))
}
pub(crate) fn perpendicular(a: V) -> V {
    let t = if a[0].abs() < 0.6 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    let p = sub(t, scale(a, dot(t, a)));
    unit(p).unwrap_or([0.0, 0.0, 1.0])
}

/// Column-vector rigid matrix taking local (x, y, z) to the frame at `origin` with z along `axis`.
pub(crate) fn frame_matrix(origin: V, axis: V) -> [[f64; 4]; 4] {
    let x = perpendicular(axis);
    let y = cross(axis, x);
    [
        [x[0], y[0], axis[0], origin[0]],
        [x[1], y[1], axis[1], origin[1]],
        [x[2], y[2], axis[2], origin[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

pub(crate) fn corners(lo: V, hi: V) -> Vec<V> {
    (0..8)
        .map(|k| {
            [
                if k & 1 == 0 { lo[0] } else { hi[0] },
                if k & 2 == 0 { lo[1] } else { hi[1] },
                if k & 4 == 0 { lo[2] } else { hi[2] },
            ]
        })
        .collect()
}
