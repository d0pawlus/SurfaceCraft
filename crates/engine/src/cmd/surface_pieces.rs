//! The pieces of a `surface.fit` (design doc §6.3, M3; P8-H4): one chart of the region that is a
//! height field over its best-fit plane is resampled onto a grid (mesh side, `fit::resample`)
//! and fitted by the kernel (`kernel::nurbs`). A region that is not one height field is grown
//! into charts (`fit::charts`) when `split` is on, and a piece that misses the tolerance on every
//! net is offered finer charts. `max_pieces` is a hard bound: every call is given the number of
//! pieces it may add and hands its charts shares that sum to at most that.
use super::surface_fit::GapOut;
use super::{CmdError, CmdResult};
use surfacecraft_fit::charts::{SplitParams, shared_vertices, split_charts};
use surfacecraft_fit::resample::{HeightGrid, height_grid};
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};
use surfacecraft_kernel::nurbs::{SurfaceProbe, fit_grid};
use surfacecraft_kernel::record::brep::Surface;

/// Points measured per fit (evenly strided) when choosing the control net.
const PROBE_POINTS: usize = 1500;
/// Control nets tried, smallest first, when `control` is not given.
const NET_LADDER: [usize; 7] = [6, 8, 11, 15, 20, 27, 36];
/// When no net meets the tolerance, nets whose subset maximum is within this factor of the best
/// subset maximum are measured on every point and the best of them is kept.
const NEAR_BEST: f64 = 1.25;
/// Fewest points a piece needs.
pub(super) const MIN_POINTS: usize = 16;
/// Narrowest cone (degrees) of normals a chart may be grown with.
pub(super) const MIN_PIECE_ANGLE: f64 = 10.0;
/// Charts with fewer triangles are merged into a neighbour (when the merged cone allows) ...
const MIN_CHART_TRIANGLES: usize = 32;
/// ... and so are charts with a smaller share of the region being split.
const MIN_CHART_SHARE: f64 = 0.02;
/// A chart that is still not a height field is split again with a cone this many times smaller,
/// at most [`MAX_RESPLITS`] levels deep.
const RESPLIT_FACTOR: f64 = 0.5;
const MAX_RESPLITS: u32 = 3;
/// A piece that misses the tolerance is replaced by finer charts only when there are at most this many.
const MAX_REFINE_CHARTS: usize = 4;
/// Shared vertices measured per pair of pieces when reporting the gap between their sheets.
const GAP_SAMPLES: usize = 400;

/// Validated settings of one fit.
#[derive(Clone, Copy, Debug)]
pub(super) struct Settings {
    pub degree: usize,
    pub tol: f64,
    pub control: Option<[usize; 2]>,
    pub outline_tol: f64,
    pub split: bool,
    pub angle: f64,
    pub max_pieces: usize,
}

/// A fitted piece: its spline and statistics, not yet cut to its outline.
pub(super) struct Piece {
    pub srf: Surface,
    pub grid: HeightGrid,
    pub triangles: Vec<u32>,
    pub control: [usize; 2],
    pub rms: f64,
    pub max: f64,
    pub measured: usize,
    pub warning: Option<String>,
}

/// What fitting one chart gave.
enum Fitted {
    Piece(Box<Piece>),
    NotHeightField,
    TooFew,
}

/// Pieces fitted so far and what was left out.
#[derive(Default)]
pub(super) struct Outcome {
    pub pieces: Vec<Piece>,
    pub skipped: Vec<String>,
}

/// What every fit of one command shares.
pub(super) struct Ctx<'a> {
    pub mesh: &'a Mesh,
    pub normals: &'a [Vec3],
    pub st: &'a Settings,
}

fn measure(probe: &SurfaceProbe, pts: &[Point3]) -> CmdResult<(f64, f64, usize)> {
    let mut sq = 0.0;
    let mut max: f64 = 0.0;
    let mut n = 0;
    for p in pts {
        let (d, _) = probe.closest([p.x, p.y, p.z])?;
        sq += d * d;
        max = max.max(d);
        n += 1;
    }
    Ok(((sq / n.max(1) as f64).sqrt(), max, n))
}

/// Statistics of one candidate net.
struct Net {
    srf: Surface,
    control: [usize; 2],
    /// `(rms, max, count)` over the points measured.
    stats: (f64, f64, usize),
    /// Were `stats` taken over every point (not only the probe subset)?
    full: bool,
    /// Maximum over the probe subset (a lower bound of the maximum over all points).
    probe_max: f64,
}

/// The smallest net whose maximum deviation, over every point, meets `tol`; when none does, the
/// net with the smallest maximum over every point (among those the subset puts near the best). A
/// net is first judged on an evenly strided subset; only one that meets `tol` there is measured
/// on all points, and if it fails there the search goes on to the next net.
fn select_net(grid: &HeightGrid, pts: &[Point3], st: &Settings) -> CmdResult<Net> {
    let stride = pts.len().div_ceil(PROBE_POINTS).max(1);
    let probe_pts: Vec<Point3> = pts.iter().step_by(stride).copied().collect();
    let rows = grid.grid.len();
    let candidates: Vec<[usize; 2]> = match st.control {
        Some(c) => vec![c],
        None => NET_LADDER.iter().map(|&c| [c, c]).filter(|c| c[0] > st.degree && c[0] <= rows).collect(),
    };
    let mut tried: Vec<Net> = Vec::new();
    for c in candidates {
        let srf = fit_grid(&grid.grid, c[0], c[1], st.degree)?;
        let probe = SurfaceProbe::new(&srf)?;
        let sub = measure(&probe, &probe_pts)?;
        let (stats, full) = if sub.1 <= st.tol { (measure(&probe, pts)?, true) } else { (sub, false) };
        let done = full && stats.1 <= st.tol;
        tried.push(Net { srf, control: c, stats, full, probe_max: sub.1 });
        if done {
            break;
        }
    }
    if tried.last().is_some_and(|n| n.full && n.stats.1 <= st.tol) {
        return tried.pop().ok_or_else(|| CmdError::Failed("no surface could be fitted".into()));
    }
    let floor = tried.iter().map(|n| n.probe_max).fold(f64::INFINITY, f64::min);
    let mut best: Option<Net> = None;
    for mut net in tried.into_iter().filter(|n| n.full || n.probe_max <= NEAR_BEST * floor) {
        if !net.full {
            net.stats = measure(&SurfaceProbe::new(&net.srf)?, pts)?;
            net.full = true;
        }
        if best.as_ref().is_none_or(|b| net.stats.1 < b.stats.1) {
            best = Some(net);
        }
    }
    best.ok_or_else(|| CmdError::Failed("no surface could be fitted".into()))
}

/// Fits one chart. `Ok(NotHeightField)` / `Ok(TooFew)` are not errors: the caller may split.
fn fit_piece(cx: &Ctx, tris: &[u32]) -> CmdResult<Fitted> {
    let st = cx.st;
    let (pts, nrm) = region_points(cx.mesh, cx.normals, tris);
    if pts.len() < MIN_POINTS {
        return Ok(Fitted::TooFew);
    }
    let max_ctrl = st.control.map_or(40, |c| c[0].max(c[1]));
    let g = (max_ctrl * 2).clamp(30, 120);
    let Some(grid) = height_grid(&pts, &nrm, g, g) else { return Ok(Fitted::NotHeightField) };
    let net = select_net(&grid, &pts, st)?;
    let (rms, max, measured) = net.stats;
    let miss = (max > st.tol).then(|| {
        format!("max deviation {max:.4} mm exceeds the {} mm target; split the region or raise the control net", st.tol)
    });
    let fold = (grid.fold > st.tol)
        .then(|| format!("the region is far from a height field (fold {:.3} mm): check the patch", grid.fold));
    let warning = [miss, fold].into_iter().flatten().reduce(|a, b| format!("{a}; {b}"));
    Ok(Fitted::Piece(Box::new(Piece {
        srf: net.srf,
        grid,
        triangles: tris.to_vec(),
        control: net.control,
        rms,
        max,
        measured,
        warning,
    })))
}

/// Fewest triangles a chart of a region of `n` triangles may have.
fn min_chart(n: usize) -> usize {
    MIN_CHART_TRIANGLES.max((n as f64 * MIN_CHART_SHARE) as usize)
}

/// Fits each chart in turn, giving it what is left of `allow` pieces less one for every chart
/// still to come, so the total never exceeds `allow`.
fn fit_charts(cx: &Ctx, charts: &[Vec<u32>], angle: f64, depth: u32, allow: usize, out: &mut Outcome) -> CmdResult<()> {
    let base = out.pieces.len();
    for (k, chart) in charts.iter().enumerate() {
        let used = out.pieces.len() - base;
        let after = charts.len() - k - 1;
        let share = allow.saturating_sub(used + after);
        if share == 0 {
            out.skipped.push(format!(
                "a part of {} triangles was left out: the limit of {} pieces",
                chart.len(),
                cx.st.max_pieces
            ));
            continue;
        }
        fit_chart(cx, chart, angle, depth, share, out)?;
    }
    Ok(())
}

/// Fits `tris` as at most `allow` pieces; a chart that is not a height field is split (when
/// `split` is on) by a cone `angle` wide and each part fitted in turn.
pub(super) fn fit_chart(
    cx: &Ctx,
    tris: &[u32],
    angle: f64,
    depth: u32,
    allow: usize,
    out: &mut Outcome,
) -> CmdResult<()> {
    let st = cx.st;
    match fit_piece(cx, tris)? {
        Fitted::Piece(p) => match refine_by_split(cx, tris, angle, depth, allow, &p)? {
            Some(finer) => {
                out.pieces.extend(finer.pieces);
                out.skipped.extend(finer.skipped);
            }
            None => out.pieces.push(*p),
        },
        Fitted::TooFew => {
            out.skipped.push(format!("a part of {} triangles has too few points for a surface", tris.len()))
        }
        Fitted::NotHeightField => {
            if !st.split || depth > MAX_RESPLITS || allow < 2 {
                out.skipped.push(format!("a part of {} triangles is not a height field", tris.len()));
                return Ok(());
            }
            let params = SplitParams { max_angle_deg: angle, min_triangles: min_chart(tris.len()), max_pieces: allow };
            let split = split_charts(cx.mesh, tris, &params);
            if split.dropped > 0 {
                out.skipped.push(format!(
                    "{} triangles are too small a part to fit, or beyond the limit of pieces",
                    split.dropped
                ));
            }
            if split.charts.len() <= 1 && split.charts.first().is_some_and(|c| c.len() == tris.len()) {
                out.skipped.push(format!(
                    "a part of {} triangles is not a height field and cannot be split further",
                    tris.len()
                ));
                return Ok(());
            }
            let next = (angle * RESPLIT_FACTOR).max(MIN_PIECE_ANGLE);
            fit_charts(cx, &split.charts, next, depth + 1, allow, out)?;
        }
    }
    Ok(())
}

/// A piece that misses the tolerance on every net, in a split fit: tries its chart as finer
/// charts (a cone [`RESPLIT_FACTOR`] times narrower), within `allow` pieces. The finer pieces
/// replace it only when all of them fit and the worst of them deviates less than `piece` did.
fn refine_by_split(
    cx: &Ctx,
    tris: &[u32],
    angle: f64,
    depth: u32,
    allow: usize,
    piece: &Piece,
) -> CmdResult<Option<Outcome>> {
    let st = cx.st;
    let next = angle * RESPLIT_FACTOR;
    if !st.split
        || st.control.is_some()
        || piece.max <= st.tol
        || depth >= MAX_RESPLITS
        || allow < 2
        || next < MIN_PIECE_ANGLE
    {
        return Ok(None);
    }
    let cap = MAX_REFINE_CHARTS.min(allow);
    let params = SplitParams { max_angle_deg: next, min_triangles: min_chart(tris.len()), max_pieces: cap };
    let split = split_charts(cx.mesh, tris, &params);
    if split.charts.len() < 2 || split.charts.len() > cap {
        return Ok(None);
    }
    let mut finer = Outcome::default();
    fit_charts(cx, &split.charts, next, depth + 1, allow, &mut finer)?;
    let worst = finer.pieces.iter().map(|p| p.max).fold(0.0, f64::max);
    let better = split.dropped == 0 && finer.skipped.is_empty() && !finer.pieces.is_empty() && worst < piece.max;
    Ok(better.then_some(finer))
}

/// Largest gap between the sheets of neighbouring pieces, measured at the mesh vertices the two
/// pieces share: the distance between the closest points of the two sheets to that vertex. With
/// `delivered` (the tessellations of the delivered, trimmed sheets) the closest points are on
/// those faces, otherwise on the fitted splines.
pub(super) fn piece_gaps(mesh: &Mesh, pieces: &[Piece], delivered: Option<&[Mesh]>) -> CmdResult<Vec<GapOut>> {
    let lists: Vec<Vec<u32>> = pieces.iter().map(|p| p.triangles.clone()).collect();
    let probes: Vec<SurfaceProbe> = pieces.iter().map(|p| SurfaceProbe::new(&p.srf)).collect::<Result<_, _>>()?;
    let bvhs: Option<Vec<Bvh>> = delivered.map(|d| d.iter().map(Bvh::build).collect());
    let closest = |k: usize, p: &Point3| -> CmdResult<Option<Point3>> {
        match bvhs.as_ref().and_then(|b| b.get(k)) {
            Some(bvh) => Ok(bvh.closest(p, f64::INFINITY).map(|h| h.point)),
            None => Ok(probes
                .get(k)
                .map(|pr| pr.closest([p.x, p.y, p.z]))
                .transpose()?
                .map(|(_, q)| Point3::new(q[0], q[1], q[2]))),
        }
    };
    let mut gaps = Vec::new();
    for (a, b, verts) in shared_vertices(mesh, &lists) {
        let stride = verts.len().div_ceil(GAP_SAMPLES).max(1);
        let mut max_gap: f64 = 0.0;
        for &v in verts.iter().step_by(stride) {
            let Some(p) = mesh.positions.get(v as usize) else { continue };
            if let (Some(qa), Some(qb)) = (closest(a, p)?, closest(b, p)?) {
                max_gap = max_gap.max((qa - qb).norm());
            }
        }
        gaps.push(GapOut { a, b, shared_vertices: verts.len(), max_gap });
    }
    Ok(gaps)
}
