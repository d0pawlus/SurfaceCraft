//! Plane sections of a mesh: the polylines where the plane cuts the triangles.
use crate::{Mesh, Point3, Vec3};
use std::collections::HashMap;

/// One section polyline.
#[derive(Clone, Debug, PartialEq)]
pub struct Polyline {
    /// Points in order.
    pub points: Vec<Point3>,
    /// Does it close on itself?
    pub closed: bool,
}

/// The section of `mesh` by the plane through `origin` with normal `normal`, chained into
/// polylines (each crossing point is identified by the mesh edge it lies on).
#[must_use]
pub fn section(mesh: &Mesh, origin: &Point3, normal: &Vec3) -> Vec<Polyline> {
    let Some(n) = crate::normalize(*normal) else { return Vec::new() };
    let dist: Vec<f64> = mesh.positions.iter().map(|p| (p - origin).dot(&n)).collect();
    // nudge exact zeros so every crossing lies strictly inside an edge
    let eps = 1e-12 * (1.0 + mesh.bbox().diagonal());
    let d = |i: u32| {
        let v = dist.get(i as usize).copied().unwrap_or(0.0);
        if v.abs() < eps { eps } else { v }
    };
    let key = |a: u32, b: u32| if a < b { (a, b) } else { (b, a) };
    let point = |a: u32, b: u32| -> Option<Point3> {
        let (pa, pb) = (mesh.positions.get(a as usize)?, mesh.positions.get(b as usize)?);
        let (da, db) = (d(a), d(b));
        let t = da / (da - db);
        Some(pa + (pb - pa) * t)
    };
    // segments between edge keys
    let mut adj: HashMap<(u32, u32), Vec<(u32, u32)>> = HashMap::new();
    for t in &mesh.triangles {
        let mut cut = Vec::with_capacity(2);
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if (d(a) > 0.0) != (d(b) > 0.0) {
                cut.push(key(a, b));
            }
        }
        if let [e0, e1] = cut.as_slice() {
            adj.entry(*e0).or_default().push(*e1);
            adj.entry(*e1).or_default().push(*e0);
        }
    }
    let mut seen: HashMap<(u32, u32), bool> = HashMap::new();
    let mut out = Vec::new();
    // start open chains at their ends first, then the closed loops
    let mut starts: Vec<(u32, u32)> = adj.iter().filter(|(_, v)| v.len() == 1).map(|(k, _)| *k).collect();
    let mut rest: Vec<(u32, u32)> = adj.keys().copied().collect();
    starts.sort_unstable();
    rest.sort_unstable();
    starts.extend(rest);
    for s in starts {
        if seen.contains_key(&s) {
            continue;
        }
        let mut chain = vec![s];
        seen.insert(s, true);
        let mut cur = s;
        loop {
            let next = adj.get(&cur).and_then(|v| v.iter().copied().find(|k| !seen.contains_key(k)));
            match next {
                Some(k) => {
                    seen.insert(k, true);
                    chain.push(k);
                    cur = k;
                }
                None => break,
            }
        }
        let closed = chain.len() > 2 && adj.get(&cur).is_some_and(|v| v.contains(&s));
        let points: Vec<Point3> = chain.iter().filter_map(|&(a, b)| point(a, b)).collect();
        if points.len() >= 2 {
            out.push(Polyline { points, closed });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;

    #[test]
    fn cylinder_section_is_a_circle() {
        let m = synth::cylinder(Point3::origin(), Vec3::z(), 5.0, 10.0, 64, 4);
        let s = section(&m, &Point3::new(0.0, 0.0, 3.3), &Vec3::z());
        assert_eq!(s.len(), 1);
        assert!(s[0].closed);
        assert!(s[0].points.iter().all(|p| (p.x.hypot(p.y) - 5.0).abs() < 0.02 && (p.z - 3.3).abs() < 1e-9));
        let b = synth::cuboid(Point3::origin(), Vec3::new(10.0, 6.0, 4.0), 4);
        let s = section(&b, &Point3::new(0.0, 0.0, 1.7), &Vec3::z());
        assert_eq!(s.len(), 1);
        assert!(s[0].closed);
        assert!(section(&b, &Point3::new(0.0, 0.0, 99.0), &Vec3::z()).is_empty());
    }
}
