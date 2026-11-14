//! A crease or a corner next to a group: a smooth patch would round it off. Two signs, both read from the
//! triangles outside what is taken out (the plan's own triangles are bent by the fold):
//! - a line of sharp edges that touch each other (a chain of at least [`CHAIN_EDGES`]);
//! - sharp edges that start at the plan and leave it on opposite sides (a crease that runs through the plan).
//!
//! Isolated sharp pairs are noise (a scan has a few between small triangles by chance) and are no sign.
use super::detect::{Incidence, MAX_ROW};
use crate::Mesh;
use std::collections::{BTreeSet, HashMap};

/// Neighbouring triangles outside the plan that meet at more than this many degrees are a sharp pair.
const SHARP_DEGREES: f64 = 30.0;
/// A chain of at least this many sharp edges that touch each other is a crease or corner: noise gives isolated
/// sharp pairs, a crease a line.
const CHAIN_EDGES: usize = 4;
/// Sharp edges that start at the plan leave it on opposite sides when their directions from its centre
/// are more than this far apart (cosine below it).
const OPPOSITE_COS: f64 = -0.5;

fn find(parent: &mut HashMap<u32, u32>, mut x: u32) -> u32 {
    while let Some(&p) = parent.get(&x) {
        if p == x {
            break;
        }
        let g = parent.get(&p).copied().unwrap_or(p);
        parent.insert(x, g);
        x = g;
    }
    x
}

/// Is there a crease or a corner in the two rings of triangles around `plan` (a group and its margin)? The
/// triangles of the plan are not looked at (they are the ones bent by the fold); a crease that runs through
/// the plan shows in the rings on both sides of it.
pub(super) fn near_a_feature(mesh: &Mesh, inc: &Incidence, neighbours: &[[u32; 3]], plan: &[u32]) -> bool {
    let inside: BTreeSet<u32> = plan.iter().copied().collect();
    let around = |from: &BTreeSet<u32>| -> BTreeSet<u32> {
        from.iter()
            .filter_map(|&t| mesh.triangles.get(t as usize))
            .flatten()
            .flat_map(|&v| inc.row(v).iter().copied().take(MAX_ROW))
            .filter(|t| !inside.contains(t))
            .collect()
    };
    let first = around(&inside);
    let both: BTreeSet<u32> = first.iter().copied().chain(around(&first)).collect();
    let cos = SHARP_DEGREES.to_radians().cos();
    let mut parent: HashMap<u32, u32> = HashMap::new();
    let mut edges: Vec<(u32, u32)> = Vec::new();
    for &t in &both {
        let (Some(n), Some(corners)) = (mesh.face_normal(t as usize), mesh.triangles.get(t as usize)) else { continue };
        for (k, &o) in neighbours.get(t as usize).into_iter().flatten().enumerate() {
            let sharp = o > t && both.contains(&o) && mesh.face_normal(o as usize).is_some_and(|m| n.dot(&m) < cos);
            if let (true, Some(&a), Some(&b)) = (sharp, corners.get(k), corners.get((k + 1) % 3)) {
                edges.push((a, b));
                parent.entry(a).or_insert(a);
                parent.entry(b).or_insert(b);
            }
        }
    }
    for &(a, b) in &edges {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        if ra != rb {
            parent.insert(rb, ra);
        }
    }
    let mut count: HashMap<u32, usize> = HashMap::new();
    for &(a, _) in &edges {
        *count.entry(find(&mut parent, a)).or_default() += 1;
    }
    count.values().any(|&n| n >= CHAIN_EDGES) || leaves_both_sides(mesh, plan, &edges)
}

/// Do sharp edges that start at a vertex of the plan lie on opposite sides of it?
fn leaves_both_sides(mesh: &Mesh, plan: &[u32], edges: &[(u32, u32)]) -> bool {
    let vertices: BTreeSet<u32> =
        plan.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flatten().copied().collect();
    let points = |v: &BTreeSet<u32>| -> Vec<crate::Point3> {
        v.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect()
    };
    let corners = points(&vertices);
    if corners.is_empty() {
        return false;
    }
    let centre = corners.iter().map(|p| p.coords).sum::<crate::Vec3>() / corners.len() as f64;
    let from_plan: Vec<crate::Vec3> = edges
        .iter()
        .filter(|(a, b)| vertices.contains(a) || vertices.contains(b))
        .filter_map(|&(a, b)| {
            Some((mesh.positions.get(a as usize)?.coords + mesh.positions.get(b as usize)?.coords) * 0.5 - centre)
        })
        .filter_map(crate::normalize)
        .collect();
    from_plan.iter().enumerate().any(|(i, u)| from_plan.iter().skip(i + 1).any(|w| u.dot(w) < OPPOSITE_COS))
}
