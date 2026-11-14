//! The loop with its parameters: dedupe and close the sides, snap to the references, smooth,
//! map to the parameter plane and check the outline.
use super::super::curve::smooth_sides;
use super::super::geom::{V2, V3, area2, cross, dist, dist2, dot, polygon_self_intersects, scale, thin_indices};
use super::super::reference::RefProbe;
use super::super::{Continuity, Request};
use super::{
    Affine, COS_MAX_TILT, Loop, MAX_AFFINE_POINTS, MAX_CHECK_POINTS, MAX_POLYGON, NOT_A_HEIGHT_FIELD, Poly,
    SNAP_ACCEPT_REL, SNAP_LIMIT, SideData, arg, bbox2, dedupe, fit_affine, fit_plane, named, snap_to_references,
};
use crate::{Result, poll};
use surfacecraft_geom::Cancel;

/// The size of the loop: its largest extent along an axis and the diagonal of its box.
struct Extent {
    span: f64,
    diag: f64,
}

fn extent(req: &Request) -> Extent {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in req.sides.iter().flat_map(|s| s.points.iter()) {
        for c in 0..3 {
            lo[c] = lo[c].min(p[c]);
            hi[c] = hi[c].max(p[c]);
        }
    }
    Extent { span: (0..3).map(|c| hi[c] - lo[c]).fold(0.0_f64, f64::max), diag: dist(lo, hi) }
}

/// The caller's number of every side.
fn labels(req: &Request) -> Vec<usize> {
    (0..req.sides.len()).map(|i| req.side_ids.get(i).copied().unwrap_or(i)).collect()
}

type SidePoints = (Vec<Vec<V3>>, Vec<Vec<V2>>);

/// The sides without repeated points (and their parameters).
fn dedupe_sides(req: &Request, labels: &[usize], tol: f64) -> Result<SidePoints> {
    let mut pts: Vec<Vec<V3>> = Vec::with_capacity(req.sides.len());
    let mut uvs: Vec<Vec<V2>> = Vec::with_capacity(req.sides.len());
    for (i, s) in req.sides.iter().enumerate() {
        let (p, q) = dedupe(&s.points, s.uv.as_ref(), tol);
        if p.len() < 2 {
            return Err(arg(format!("side {} has no length", labels.get(i).copied().unwrap_or(i))));
        }
        pts.push(p);
        uvs.push(q);
    }
    Ok((pts, uvs))
}

/// Closes the loop: the next side starts where this one ends (within `close_tol`).
fn close_loop(pts: &mut [Vec<V3>], uvs: &mut [Vec<V2>], flat: bool, close_tol: f64, labels: &[usize]) -> Result<()> {
    let n = pts.len();
    let end_of = |k: usize| {
        let end = pts.get(k).and_then(|p| p.last()).copied().unwrap_or([0.0; 3]);
        (end, uvs.get(k).and_then(|q| q.last()).copied().unwrap_or([0.0; 2]))
    };
    let ends: Vec<(V3, V2)> = (0..n).map(end_of).collect();
    let uv_close = ends.iter().map(|(_, q)| q[0].abs().max(q[1].abs())).fold(1.0_f64, f64::max) * 1e-7;
    let label = |k: usize| labels.get(k).copied().unwrap_or(k);
    for (k, &(end, end_uv)) in ends.iter().enumerate() {
        let nxt = (k + 1) % n;
        let first = pts.get(nxt).and_then(|p| p.first()).copied().ok_or_else(|| arg("a side has no points"))?;
        let d = dist(end, first);
        if d > close_tol {
            return Err(arg(format!(
                "the sides do not close: side {} ends {d:.3e} mm from the start of side {}",
                label(k),
                label(nxt)
            )));
        }
        let first_uv = uvs.get(nxt).and_then(|q| q.first()).copied();
        if flat && first_uv.is_none_or(|f| dist2(end_uv, f) > uv_close) {
            return Err(arg(format!(
                "the parameters of side {} and side {} do not meet at their corner",
                label(k),
                label(nxt)
            )));
        }
        if let Some(f) = pts.get_mut(nxt).and_then(|p| p.first_mut()) {
            *f = end;
        }
        if flat && let Some(f) = uvs.get_mut(nxt).and_then(|q| q.first_mut()) {
            *f = end_uv;
        }
    }
    Ok(())
}

/// The reference probes of the G1/G2 sides.
fn reference_probes(req: &Request, labels: &[usize]) -> Result<Vec<Option<RefProbe>>> {
    req.sides
        .iter()
        .enumerate()
        .map(|(i, s)| {
            if s.continuity == Continuity::G0 {
                return Ok(None);
            }
            let label = labels.get(i).copied().unwrap_or(i);
            s.reference.as_ref().map(RefProbe::new).transpose().map_err(|e| named(label, e))
        })
        .collect()
}

/// The affine map of the loop: fitted to the given parameters (a flattening), else the best-fit
/// plane, oriented like the first reference face's normal; a G1/G2 side whose reference leans more
/// than 75° off that plane makes the outline no height field.
fn loop_affine(
    pts: &[Vec<V3>],
    uvs: &[Vec<V2>],
    flat: bool,
    probes: &[Option<RefProbe>],
    labels: &[usize],
) -> Result<Affine> {
    if flat {
        let all_pts: Vec<V3> = pts.iter().flatten().copied().collect();
        let all_uv: Vec<V2> = uvs.iter().flatten().copied().collect();
        let keep = thin_indices(all_pts.len(), MAX_AFFINE_POINTS);
        let p: Vec<V3> = keep.iter().filter_map(|&i| all_pts.get(i).copied()).collect();
        let q: Vec<V2> = keep.iter().filter_map(|&i| all_uv.get(i).copied()).collect();
        return fit_affine(&p, &q);
    }
    let label = |k: usize| labels.get(k).copied().unwrap_or(k);
    let mut plane = fit_plane(pts)?;
    let first =
        probes.iter().zip(pts.iter()).enumerate().find_map(|(k, (pr, p))| pr.as_ref().zip(p.first()).map(|x| (k, x)));
    let flip = match first {
        Some((k, (pr, p))) => dot(plane.n, pr.locate(*p).map_err(|e| named(label(k), e))?.normal) < 0.0,
        None => {
            let k = (0..3).fold(0, |b, c| if plane.n[c].abs() > plane.n[b].abs() { c } else { b });
            plane.n[k] < 0.0
        }
    };
    if flip {
        plane.n = scale(plane.n, -1.0);
        plane.e2 = cross(plane.n, plane.e1);
    }
    for (k, (pr, p)) in probes.iter().zip(pts.iter()).enumerate() {
        let Some(pr) = pr else { continue };
        for i in thin_indices(p.len(), 5) {
            let Some(q) = p.get(i) else { continue };
            if dot(pr.locate(*q).map_err(|e| named(label(k), e))?.normal, plane.n).abs() < COS_MAX_TILT {
                return Err(arg(NOT_A_HEIGHT_FIELD));
            }
        }
    }
    Ok(Affine { a: [plane.e1, plane.e2], b: [-dot(plane.e1, plane.c), -dot(plane.e2, plane.c)] })
}

/// A side's parameters without the last point (which is the next side's first).
fn body(u: &[V2]) -> &[V2] {
    u.get(..u.len().saturating_sub(1)).unwrap_or_default()
}

/// What the outline in the parameter plane gives the later stages.
struct Outline {
    ccw: bool,
    bbox: [[f64; 2]; 2],
    polygon: Vec<V2>,
}

/// Checks the outline in the parameter plane (not degenerate, not crossing itself) and thins it.
fn check_outline(side_uv: &[Vec<V2>], flat: bool) -> Result<Outline> {
    let n = side_uv.len();
    let outline: Vec<V2> = side_uv.iter().flat_map(|u| body(u).iter().copied()).collect();
    let bbox = bbox2(side_uv.iter().flatten().copied());
    let ext = [bbox[0][1] - bbox[0][0], bbox[1][1] - bbox[1][0]];
    if !(ext[0].is_finite() && ext[1].is_finite() && ext[0].min(ext[1]) > 1e-9 * ext[0].max(ext[1])) {
        return Err(arg("the outline is degenerate in the parameter plane"));
    }
    let thinned = |cap: usize| -> Vec<V2> {
        side_uv
            .iter()
            .flat_map(|u| {
                let b = body(u);
                thin_indices(b.len(), cap).into_iter().filter_map(move |i| b.get(i).copied())
            })
            .collect()
    };
    // crossing check on a thinned copy
    if polygon_self_intersects(&thinned((MAX_CHECK_POINTS / n).max(8)), 1e-9 * ext[0].max(ext[1])) {
        return Err(arg(if flat { "the flattening of the outline crosses itself" } else { NOT_A_HEIGHT_FIELD }));
    }
    Ok(Outline { ccw: area2(&outline) > 0.0, bbox, polygon: thinned((MAX_POLYGON / n).max(8)) })
}

/// The loop with parameters, checked: closed, a height field (or a flattening that does not cross
/// itself), its references usable. Hard sides of three or more points are densified along a smooth
/// curve through their points; the warnings say which could not be.
pub(super) fn build_loop(req: &Request, cancel: &Cancel) -> Result<(Loop, Vec<String>)> {
    let flat = req.sides.iter().all(|s| s.uv.is_some());
    let ex = extent(req);
    let labels = labels(req);
    let (mut pts, mut uvs) = dedupe_sides(req, &labels, 1e-9 * ex.span.max(1.0))?;
    close_loop(&mut pts, &mut uvs, flat, 1e-7 * ex.span.max(1.0), &labels)?;
    if pts.iter().map(|p| p.len().saturating_sub(1)).sum::<usize>() < 3 {
        return Err(arg("the outline needs at least 3 distinct points"));
    }
    let probes = reference_probes(req, &labels)?;
    let accept = (SNAP_ACCEPT_REL * ex.span).max(10.0 * SNAP_LIMIT);
    let snaps = snap_to_references(&mut pts, &probes, &labels, accept, cancel)?;
    poll(cancel)?;
    // the points are samples of a smooth curve: the dense samples between them follow it, not the chords
    let soft: Vec<bool> = req.sides.iter().map(|s| s.soft).collect();
    let notes = smooth_sides(&mut pts, &mut uvs, flat, &soft, ex.diag);
    poll(cancel)?;
    let affine = loop_affine(&pts, &uvs, flat, &probes, &labels)?;
    let side_uv: Vec<Vec<V2>> =
        if flat { uvs } else { pts.iter().map(|p| p.iter().map(|q| affine.uv(*q)).collect()).collect() };
    let Outline { ccw, bbox, polygon } = check_outline(&side_uv, flat)?;
    let sides: Vec<SideData> = req
        .sides
        .iter()
        .zip(pts)
        .zip(side_uv)
        .zip(probes)
        .zip(snaps)
        .zip(labels)
        .map(|(((((s, p), q), probe), snap), label)| SideData {
            label,
            poly: Poly::new(p),
            uv: q,
            continuity: s.continuity,
            probe,
            soft: s.soft,
            snap,
        })
        .collect();
    Ok((Loop { sides, affine, ccw, bbox, diag: ex.diag, polygon }, notes))
}
