//! Closed profiles of a sketch as 3-D segments, the input of extrude and revolve.
use crate::{Entity, Sketch};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::f64::consts::TAU;
use surfacecraft_geom::Vec3;

/// A profile segment in model space.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Segment3 {
    /// Straight segment.
    Line {
        /// Start.
        start: [f64; 3],
        /// End.
        end: [f64; 3],
    },
    /// Arc counter-clockwise about `axis` from angle `start` to `end`, angle 0 along `reference`.
    Arc {
        /// Centre.
        center: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Unit direction of angle 0.
        reference: [f64; 3],
        /// Radius.
        radius: f64,
        /// Start angle (radians).
        start: f64,
        /// End angle (radians), start < end ≤ start + 2π.
        end: f64,
    },
}

/// Closed loops of the sketch, each a chain of segments in order. Points closer than `tol`
/// count as joined. Circles are loops of their own; open chains are ignored.
#[must_use]
pub fn profiles(s: &Sketch, tol: f64) -> Vec<Vec<Segment3>> {
    chains(s, tol).into_iter().filter_map(|(closed, c)| closed.then_some(c)).collect()
}

/// Open chains of lines and arcs of the sketch (paths for sweeps and patterns), each from one
/// free end to the other, longest first.
#[must_use]
pub fn open_paths(s: &Sketch, tol: f64) -> Vec<Vec<Segment3>> {
    let mut v: Vec<Vec<Segment3>> =
        chains(s, tol).into_iter().filter_map(|(closed, c)| (!closed).then_some(c)).collect();
    v.sort_by(|a, b| path_length(b).total_cmp(&path_length(a)));
    v
}

impl Segment3 {
    /// Length of the segment.
    #[must_use]
    pub fn length(&self) -> f64 {
        match self {
            Segment3::Line { start, end } => {
                ((end[0] - start[0]).powi(2) + (end[1] - start[1]).powi(2) + (end[2] - start[2]).powi(2)).sqrt()
            }
            Segment3::Arc { radius, start, end, .. } => radius.abs() * (end - start).abs(),
        }
    }

    /// Point and unit tangent at arc length `s` from the segment start (clamped to the segment).
    #[must_use]
    pub fn at(&self, s: f64) -> ([f64; 3], [f64; 3]) {
        let len = self.length();
        let t = if len > 0.0 { (s / len).clamp(0.0, 1.0) } else { 0.0 };
        match self {
            Segment3::Line { start, end } => {
                let d = Vec3::from(*end) - Vec3::from(*start);
                let u = surfacecraft_geom::normalize(d).unwrap_or_else(Vec3::x);
                ((Vec3::from(*start) + d * t).into(), u.into())
            }
            Segment3::Arc { center, axis, reference, radius, start, end } => {
                let (c, a, r) = (Vec3::from(*center), Vec3::from(*axis), Vec3::from(*reference));
                let b = a.cross(&r);
                let ang = start + (end - start) * t;
                let p = c + (r * ang.cos() + b * ang.sin()) * *radius;
                let tan = b * ang.cos() - r * ang.sin();
                (p.into(), surfacecraft_geom::normalize(tan).unwrap_or_else(Vec3::x).into())
            }
        }
    }
}

/// Total length of a chain.
#[must_use]
pub fn path_length(path: &[Segment3]) -> f64 {
    path.iter().map(Segment3::length).sum()
}

/// `count` points (and unit tangents) spaced evenly by arc length along `path`, the first at its
/// start and the last at its end (`count` ≥ 2), or just the start for `count` = 1. `None` for an
/// empty or zero-length path or `count` = 0 or over 100 000.
#[must_use]
pub fn path_samples(path: &[Segment3], count: usize) -> Option<Vec<([f64; 3], [f64; 3])>> {
    let total = path_length(path);
    if count == 0 || count > 100_000 || path.is_empty() || !(total.is_finite() && total > 0.0) {
        return None;
    }
    let mut out = Vec::with_capacity(count);
    for k in 0..count {
        let target = if count == 1 { 0.0 } else { total * k as f64 / (count - 1) as f64 };
        let mut acc = 0.0;
        let mut found = None;
        for (i, seg) in path.iter().enumerate() {
            let l = seg.length();
            if target <= acc + l + 1e-12 || i + 1 == path.len() {
                found = Some(seg.at(target - acc));
                break;
            }
            acc += l;
        }
        out.push(found?);
    }
    Some(out)
}

/// Every chain of the sketch with whether it closes (see [`profiles`] and [`open_paths`]).
fn chains(s: &Sketch, tol: f64) -> Vec<(bool, Vec<Segment3>)> {
    let pl = &s.plane;
    let n = pl.normal();
    let pt = |i: usize| s.points.get(i).copied().unwrap_or([0.0, 0.0]);
    let same = |a: [f64; 2], b: [f64; 2]| (a[0] - b[0]).hypot(a[1] - b[1]) <= tol;
    let mut out = Vec::new();
    // circles
    for e in &s.entities {
        if let Entity::Circle { center, radius } = e {
            let c = pl.to_3d(pt(*center));
            out.push((
                true,
                vec![Segment3::Arc {
                    center: c.into(),
                    axis: n.into(),
                    reference: pl.x,
                    radius: *radius,
                    start: 0.0,
                    end: TAU,
                }],
            ));
        }
    }
    // chains of lines and arcs
    let curves: Vec<usize> =
        (0..s.entities.len()).filter(|&k| !matches!(s.entities.get(k), Some(Entity::Circle { .. }) | None)).collect();
    let mut used = vec![false; s.entities.len()];
    for &first in &curves {
        if used.get(first).copied().unwrap_or(true) {
            continue;
        }
        let Some((a0, b0)) = s.endpoints(first) else { continue };
        let start = pt(a0);
        let mut chain: Vec<(usize, bool)> = vec![(first, true)];
        if let Some(u) = used.get_mut(first) {
            *u = true;
        }
        let mut cur = pt(b0);
        let mut closed = same(cur, start);
        while !closed {
            let next = curves.iter().copied().find_map(|k| {
                if used.get(k).copied().unwrap_or(true) {
                    return None;
                }
                let (a, b) = s.endpoints(k)?;
                if same(pt(a), cur) {
                    Some((k, true, pt(b)))
                } else if same(pt(b), cur) {
                    Some((k, false, pt(a)))
                } else {
                    None
                }
            });
            let Some((k, fwd, end)) = next else { break };
            if let Some(u) = used.get_mut(k) {
                *u = true;
            }
            chain.push((k, fwd));
            cur = end;
            closed = same(cur, start);
        }
        if !closed {
            // an open chain also grows backwards from where it started
            let mut head = start;
            loop {
                let prev = curves.iter().copied().find_map(|k| {
                    if used.get(k).copied().unwrap_or(true) {
                        return None;
                    }
                    let (a, b) = s.endpoints(k)?;
                    if same(pt(b), head) {
                        Some((k, true, pt(a)))
                    } else if same(pt(a), head) {
                        Some((k, false, pt(b)))
                    } else {
                        None
                    }
                });
                let Some((k, fwd, far)) = prev else { break };
                if let Some(u) = used.get_mut(k) {
                    *u = true;
                }
                chain.insert(0, (k, fwd));
                head = far;
            }
        }
        let mut segs = Vec::with_capacity(chain.len());
        for (k, fwd) in chain {
            match s.entities.get(k) {
                Some(Entity::Line { a, b }) => {
                    let (p, q) = if fwd { (pt(*a), pt(*b)) } else { (pt(*b), pt(*a)) };
                    segs.push(Segment3::Line { start: pl.to_3d(p).into(), end: pl.to_3d(q).into() });
                }
                Some(Entity::Arc { center, a, b }) => {
                    let c = pt(*center);
                    let (pa, pb) = (pt(*a), pt(*b));
                    let ang = |q: [f64; 2]| (q[1] - c[1]).atan2(q[0] - c[0]);
                    let r = (pa[0] - c[0]).hypot(pa[1] - c[1]);
                    // CCW sweep from a to b in the sketch
                    let sweep = (ang(pb) - ang(pa)).rem_euclid(TAU);
                    let sweep = if sweep < 1e-12 { TAU } else { sweep };
                    // traversed from a (forward) about +n, or from b about −n
                    let (from, axis) = if fwd { (pa, n) } else { (pb, -n) };
                    let refd = pl.to_3d(from) - pl.to_3d(c);
                    let reference = surfacecraft_geom::normalize(refd).unwrap_or_else(|| Vec3::from(pl.x));
                    segs.push(Segment3::Arc {
                        center: pl.to_3d(c).into(),
                        axis: axis.into(),
                        reference: reference.into(),
                        radius: r,
                        start: 0.0,
                        end: sweep,
                    });
                }
                _ => {}
            }
        }
        out.push((closed, segs));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rectangle_and_slot_profiles() {
        let mut s = Sketch::default();
        let p: Vec<usize> = [[0.0, 0.0], [4.0, 0.0], [4.0, 2.0], [0.0, 2.0]].iter().map(|q| s.add_point(*q)).collect();
        // edges listed out of order and one reversed
        s.entities.push(Entity::Line { a: p[0], b: p[1] });
        s.entities.push(Entity::Line { a: p[3], b: p[2] });
        s.entities.push(Entity::Line { a: p[1], b: p[2] });
        s.entities.push(Entity::Line { a: p[3], b: p[0] });
        let c = s.add_point([10.0, 0.0]);
        s.entities.push(Entity::Circle { center: c, radius: 1.0 });
        let pr = profiles(&s, 1e-9);
        assert_eq!(pr.len(), 2);
        assert_eq!(pr[1].len(), 4);
        // the chain is continuous
        let ends: Vec<([f64; 3], [f64; 3])> = pr[1]
            .iter()
            .map(|g| if let Segment3::Line { start, end } = g { (*start, *end) } else { ([0.0; 3], [0.0; 3]) })
            .collect();
        for w in ends.windows(2) {
            assert_eq!(w[0].1, w[1].0);
        }
    }

    #[test]
    fn open_path_is_found_whole_and_sampled_by_arc_length() {
        let mut s = Sketch::default();
        // the first entity runs (10,10)→(10,0), the second is stored (0,0)→(10,0): the chain follows the first
        let p: Vec<usize> = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]].iter().map(|q| s.add_point(*q)).collect();
        s.entities.push(Entity::Line { a: p[2], b: p[1] });
        s.entities.push(Entity::Line { a: p[0], b: p[1] });
        assert!(profiles(&s, 1e-9).is_empty());
        let paths = open_paths(&s, 1e-9);
        assert_eq!(paths.len(), 1);
        assert_eq!(paths[0].len(), 2);
        assert!((path_length(&paths[0]) - 20.0).abs() < 1e-12);
        let pts = path_samples(&paths[0], 5).unwrap();
        // 5 points over 20 mm, from the start of the first entity: 0, 5, 10 (the corner), 15, 20
        let expect = [[10.0, 10.0], [10.0, 5.0], [10.0, 0.0], [5.0, 0.0], [0.0, 0.0]];
        for (g, e) in pts.iter().zip(expect) {
            assert!((g.0[0] - e[0]).abs() < 1e-9 && (g.0[1] - e[1]).abs() < 1e-9, "{pts:?}");
        }
        assert!(path_samples(&[], 3).is_none() && path_samples(&paths[0], 0).is_none());
    }
}
