//! Splitting a region into charts that are height fields (P8-H4). A region that wraps around
//! (a pipe wall, the side of a bottle) is not a height field over any plane, so one fitted patch
//! cannot follow it. The triangles are grown into charts: a chart starts at the lowest unassigned
//! triangle and takes neighbours across edges while their normals stay within a cone of the
//! seed's normal; charts too small to fit are merged into the neighbour they share most edges
//! with. Charts do not overlap, and neighbouring charts share the vertices along their common
//! boundary, so the surfaces fitted to them meet within the fit residual (the gap is measured by
//! the caller). This is a split, not a G1 network: tangent continuity across charts is not
//! enforced.
use std::collections::{HashMap, VecDeque};
use surfacecraft_geom::{Mesh, Vec3};

/// Chart growth limits.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SplitParams {
    /// Half-angle (degrees) of the cone around the seed normal that a chart's normals stay in.
    pub max_angle_deg: f64,
    /// Charts with fewer triangles are merged into a neighbour.
    pub min_triangles: usize,
    /// Most charts returned (the smallest are merged into neighbours down to this count).
    pub max_pieces: usize,
}

/// The charts of a region.
#[derive(Clone, Debug, PartialEq)]
pub struct Split {
    /// Triangle ids of each chart, ascending; charts in order of their lowest triangle.
    pub charts: Vec<Vec<u32>>,
    /// Triangles left out: degenerate ones and parts too small to stand alone.
    pub dropped: usize,
}

const OUTSIDE: u32 = u32::MAX;
const UNASSIGNED: u32 = u32::MAX - 1;
/// Times the cone of a chart is re-centred on the mean normal of what it grew.
const AXIS_REFINEMENTS: usize = 3;

/// Grows `tris` of `mesh` into charts (see the module docs).
#[must_use]
pub fn split_charts(mesh: &Mesh, tris: &[u32], p: &SplitParams) -> Split {
    let n = mesh.triangle_count();
    let mut ids: Vec<u32> = tris.iter().copied().filter(|&t| (t as usize) < n).collect();
    ids.sort_unstable();
    ids.dedup();
    let normals: HashMap<u32, Vec3> = ids.iter().filter_map(|&t| Some((t, mesh.face_normal(t as usize)?))).collect();
    let nb = mesh.triangle_neighbours();
    let mut chart_of = vec![OUTSIDE; n];
    for t in normals.keys() {
        if let Some(c) = chart_of.get_mut(*t as usize) {
            *c = UNASSIGNED;
        }
    }
    let mut dropped = ids.len() - normals.len();
    let cos = p.max_angle_deg.to_radians().cos();
    let mut count = 0u32;
    for &seed in &ids {
        let Some(&first_axis) = normals.get(&seed) else { continue };
        if chart_of.get(seed as usize) != Some(&UNASSIGNED) {
            continue;
        }
        // the seed may sit at the edge of its chart: re-centre the cone on the chart's mean normal
        let mut axis = first_axis;
        for round in 0..=AXIS_REFINEMENTS {
            let members = grow(&nb, &normals, &mut chart_of, seed, axis, cos, count);
            if round == AXIS_REFINEMENTS {
                break;
            }
            let mean: Vec3 = members.iter().filter_map(|t| normals.get(t)).sum();
            for t in &members {
                if let Some(c) = chart_of.get_mut(*t as usize) {
                    *c = UNASSIGNED;
                }
            }
            axis = if mean.norm() > 1e-9 { mean.normalize() } else { first_axis };
        }
        count += 1;
    }
    merge_small(&nb, &ids, &mut chart_of, count as usize, p, &mut dropped);
    let mut charts: Vec<Vec<u32>> = Vec::new();
    let mut slot: HashMap<u32, usize> = HashMap::new();
    for &t in &ids {
        let c = chart_of.get(t as usize).copied().unwrap_or(OUTSIDE);
        if c >= UNASSIGNED {
            continue;
        }
        let k = *slot.entry(c).or_insert_with(|| {
            charts.push(Vec::new());
            charts.len() - 1
        });
        if let Some(list) = charts.get_mut(k) {
            list.push(t);
        }
    }
    Split { charts, dropped }
}

/// Assigns chart `id` to `seed` and to every triangle reachable from it across edges whose normal
/// stays within the cone (`cos` of its half-angle) around `axis`; returns the chart's triangles.
fn grow(
    nb: &[[u32; 3]],
    normals: &HashMap<u32, Vec3>,
    chart_of: &mut [u32],
    seed: u32,
    axis: Vec3,
    cos: f64,
    id: u32,
) -> Vec<u32> {
    let mut queue = VecDeque::from([seed]);
    let mut members = vec![seed];
    if let Some(c) = chart_of.get_mut(seed as usize) {
        *c = id;
    }
    while let Some(t) = queue.pop_front() {
        for &o in nb.get(t as usize).into_iter().flatten() {
            let open = chart_of.get(o as usize) == Some(&UNASSIGNED);
            if open && normals.get(&o).is_some_and(|m| m.dot(&axis) >= cos) {
                if let Some(c) = chart_of.get_mut(o as usize) {
                    *c = id;
                }
                members.push(o);
                queue.push_back(o);
            }
        }
    }
    members
}

/// Edges shared between charts and the triangle count of each chart.
fn chart_graph(nb: &[[u32; 3]], ids: &[u32], chart_of: &[u32], count: usize) -> (Vec<usize>, Vec<HashMap<usize, usize>>) {
    let mut size = vec![0usize; count];
    let mut adj: Vec<HashMap<usize, usize>> = vec![HashMap::new(); count];
    for &t in ids {
        let c = chart_of.get(t as usize).copied().unwrap_or(OUTSIDE);
        if c >= UNASSIGNED {
            continue;
        }
        if let Some(s) = size.get_mut(c as usize) {
            *s += 1;
        }
        for &o in nb.get(t as usize).into_iter().flatten() {
            let oc = chart_of.get(o as usize).copied().unwrap_or(OUTSIDE);
            if oc < UNASSIGNED && oc != c {
                if let Some(m) = adj.get_mut(c as usize) {
                    *m.entry(oc as usize).or_default() += 1;
                }
            }
        }
    }
    (size, adj)
}

/// Merges charts below `min_triangles` (and the smallest ones above `max_pieces`) into the
/// neighbouring chart they share most edges with; a chart without neighbours that is still too
/// small is dropped. Works on the graph of charts, so its cost does not grow with the mesh.
fn merge_small(nb: &[[u32; 3]], ids: &[u32], chart_of: &mut [u32], count: usize, p: &SplitParams, dropped: &mut usize) {
    let (mut size, mut adj) = chart_graph(nb, ids, chart_of, count);
    let mut parent: Vec<u32> = (0..count as u32).collect();
    let mut alive = vec![true; count];
    let mut live = count;
    let cap = p.max_pieces.max(1);
    loop {
        let pick = (0..count)
            .filter(|&c| alive[c] && (size[c] < p.min_triangles || (live > cap && !adj[c].is_empty())))
            .min_by_key(|&c| (size[c], c));
        let Some(small) = pick else { break };
        alive[small] = false;
        live -= 1;
        let neighbours = std::mem::take(&mut adj[small]);
        let Some((&to, _)) = neighbours.iter().max_by_key(|&(&o, &e)| (e, std::cmp::Reverse(o))) else {
            parent[small] = OUTSIDE;
            *dropped += size[small];
            continue;
        };
        parent[small] = to as u32;
        size[to] += size[small];
        for (&o, &e) in &neighbours {
            adj[o].remove(&small);
            if o != to {
                *adj[o].entry(to).or_default() += e;
                *adj[to].entry(o).or_default() += e;
            }
        }
    }
    let root = |mut c: u32| {
        while let Some(&up) = parent.get(c as usize) {
            if up == c || up == OUTSIDE {
                return up;
            }
            c = up;
        }
        OUTSIDE
    };
    for &t in ids {
        if let Some(c) = chart_of.get_mut(t as usize).filter(|c| **c < UNASSIGNED) {
            *c = root(*c);
        }
    }
}

/// Vertices shared by pairs of charts: `(chart a, chart b, vertex ids)` for `a < b`, only pairs
/// that share at least one vertex.
#[must_use]
pub fn shared_vertices(mesh: &Mesh, charts: &[Vec<u32>]) -> Vec<(usize, usize, Vec<u32>)> {
    let mut owners: HashMap<u32, Vec<usize>> = HashMap::new();
    for (k, chart) in charts.iter().enumerate() {
        let mut vs: Vec<u32> =
            chart.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flat_map(|t| t.iter().copied()).collect();
        vs.sort_unstable();
        vs.dedup();
        for v in vs {
            owners.entry(v).or_default().push(k);
        }
    }
    let mut pairs: HashMap<(usize, usize), Vec<u32>> = HashMap::new();
    for (v, list) in &owners {
        for (i, &a) in list.iter().enumerate() {
            for &b in list.iter().skip(i + 1) {
                pairs.entry((a, b)).or_default().push(*v);
            }
        }
    }
    let mut out: Vec<(usize, usize, Vec<u32>)> = pairs
        .into_iter()
        .map(|((a, b), mut v)| {
            v.sort_unstable();
            (a, b, v)
        })
        .collect();
    out.sort_by_key(|(a, b, _)| (*a, *b));
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resample::height_grid;
    use surfacecraft_geom::Point3;

    /// An open half-pipe of radius 15 mm wrapping `sweep` radians, 40 mm tall.
    fn pipe(sweep: f64, nu: usize, nv: usize) -> Mesh {
        let mut positions = Vec::new();
        for j in 0..=nv {
            for i in 0..=nu {
                let th = sweep * i as f64 / nu as f64;
                positions.push(Point3::new(15.0 * th.cos(), 15.0 * th.sin(), 40.0 * j as f64 / nv as f64));
            }
        }
        let w = (nu + 1) as u32;
        let mut triangles = Vec::new();
        for j in 0..nv as u32 {
            for i in 0..nu as u32 {
                let a = j * w + i;
                triangles.push([a, a + 1, a + w + 1]);
                triangles.push([a, a + w + 1, a + w]);
            }
        }
        Mesh { positions, triangles }
    }

    fn params(angle: f64) -> SplitParams {
        SplitParams { max_angle_deg: angle, min_triangles: 40, max_pieces: 16 }
    }

    fn is_height_field(mesh: &Mesh, tris: &[u32]) -> bool {
        let normals = mesh.vertex_normals();
        let (pts, nrm) = crate::segment::region_points(mesh, &normals, tris);
        height_grid(&pts, &nrm, 20, 20).is_some()
    }

    #[test]
    fn wrapped_pipe_splits_into_height_fields() {
        let mesh = pipe(240f64.to_radians(), 120, 40);
        let all: Vec<u32> = (0..mesh.triangle_count() as u32).collect();
        assert!(!is_height_field(&mesh, &all), "the 240 degree pipe is not a height field");
        let s = split_charts(&mesh, &all, &params(45.0));
        assert!(s.charts.len() >= 3 && s.charts.len() <= 5, "{} charts", s.charts.len());
        assert_eq!(s.dropped, 0);
        // the charts partition the region
        let total: usize = s.charts.iter().map(Vec::len).sum();
        assert_eq!(total, all.len());
        for c in &s.charts {
            assert!(is_height_field(&mesh, c), "a chart of {} triangles folds", c.len());
        }
        // neighbouring charts share vertices; the pipe's charts form a chain
        let shared = shared_vertices(&mesh, &s.charts);
        assert!(shared.len() >= s.charts.len() - 1, "{} pairs", shared.len());
        assert!(shared.iter().all(|(_, _, v)| v.len() >= 40));
    }

    #[test]
    fn a_height_field_stays_one_chart() {
        let mesh = pipe(60f64.to_radians(), 40, 20);
        let all: Vec<u32> = (0..mesh.triangle_count() as u32).collect();
        let s = split_charts(&mesh, &all, &params(45.0));
        assert_eq!(s.charts.len(), 1);
    }

    #[test]
    fn slivers_merge_and_limits_hold() {
        // 250 degrees leave a sliver after three 45 degree charts; it must merge, not stand alone
        let mesh = pipe(250f64.to_radians(), 120, 40);
        let all: Vec<u32> = (0..mesh.triangle_count() as u32).collect();
        let s = split_charts(&mesh, &all, &SplitParams { max_angle_deg: 45.0, min_triangles: 400, max_pieces: 16 });
        assert!(s.charts.iter().all(|c| c.len() >= 400), "{:?}", s.charts.iter().map(Vec::len).collect::<Vec<_>>());
        // the piece cap merges the smallest charts
        let two = split_charts(&mesh, &all, &SplitParams { max_angle_deg: 45.0, min_triangles: 1, max_pieces: 2 });
        assert_eq!(two.charts.len(), 2);
        assert_eq!(two.charts.iter().map(Vec::len).sum::<usize>(), all.len());
        // hostile input: out-of-range ids, nothing, degenerate cone
        assert!(split_charts(&mesh, &[u32::MAX], &params(45.0)).charts.is_empty());
        assert!(split_charts(&mesh, &[], &params(45.0)).charts.is_empty());
        let degenerate = split_charts(&mesh, &all, &params(0.0));
        assert_eq!(degenerate.charts.iter().map(Vec::len).sum::<usize>() + degenerate.dropped, all.len());
    }
}
