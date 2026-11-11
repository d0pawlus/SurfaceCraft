//! `surface.fit`: NURBS sheets fitted to a region (design doc §6.3, milestone M3), trimmed to the
//! region's outline and split into height-field pieces when the region wraps (P8-H4).
//!
//! A piece is a region (or a chart of one) that is a height field over its best-fit plane: the
//! points are resampled onto a grid (mesh side, `fit::resample`), the kernel fits the B-spline
//! (`kernel::nurbs`), and, when asked, the sheet is trimmed to the boundary loops of the region
//! (`fit::outline` + `kernel::nurbs::trimmed_sheet`).
use super::{CmdError, CmdResult, ObjRef};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, SurfaceObject};
use surfacecraft_fit::charts::{SplitParams, shared_vertices, split_charts};
use surfacecraft_fit::outline::region_outline;
use surfacecraft_fit::resample::{HeightGrid, height_grid};
use surfacecraft_fit::segment::region_points;
use surfacecraft_geom::{Mesh, Point3, Vec3};
use surfacecraft_kernel::nurbs::{SurfaceProbe, fit_grid, sheet_from_spline, trimmed_sheet};
use surfacecraft_kernel::record::brep::{Brep, Surface};

/// Points measured per fit (evenly strided) when choosing the control net.
const PROBE_POINTS: usize = 1500;
/// Control nets tried, smallest first, when `control` is not given.
const NET_LADDER: [usize; 7] = [6, 8, 11, 15, 20, 27, 36];
/// Fewest points a piece needs.
const MIN_POINTS: usize = 16;
/// Defaults and limits of the new parameters.
const DEFAULT_OUTLINE_TOLERANCE: f64 = 0.1;
const OUTLINE_TOLERANCE_RANGE: (f64, f64) = (0.001, 10.0);
const DEFAULT_PIECE_ANGLE: f64 = 45.0;
const PIECE_ANGLE_RANGE: (f64, f64) = (10.0, 80.0);
const DEFAULT_MAX_PIECES: usize = 16;
const MAX_PIECES_LIMIT: usize = 64;
/// Charts with fewer triangles are merged into a neighbour.
const MIN_CHART_TRIANGLES: usize = 32;
/// ... and fewer than this share of the chart being split.
const MIN_CHART_SHARE: f64 = 0.02;
/// A chart that is still not a height field is split again with a cone this many times smaller,
/// at most this many times.
const RESPLIT_FACTOR: f64 = 0.5;
const MAX_RESPLITS: u32 = 3;
/// A piece that misses the tolerance is replaced by finer charts only when there are at most this many.
const MAX_REFINE_CHARTS: usize = 4;
/// Shared vertices measured per pair of pieces when reporting the gap between their sheets.
const GAP_SAMPLES: usize = 400;
const SPLIT_NOTE: &str = "the region was split into height-field pieces: each is its own sheet, the pieces meet \
                          within the gap reported (shared boundary vertices), tangent continuity across them is \
                          not enforced (a split, not a G1 network)";

/// Fit parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FitIn {
    /// Region to fit.
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh` (all when omitted with `mesh`).
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Control net `[u, v]`; omitted → the smallest net meeting `tolerance`.
    #[serde(default)]
    pub control: Option<[usize; 2]>,
    /// Degree (default 3).
    #[serde(default)]
    pub degree: Option<usize>,
    /// Target maximum deviation (mm, default 0.2 — design doc §11.1).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Trim each sheet to the outline of its region (outer loop and holes, smoothed and simplified
    /// to `outline_tolerance`) instead of leaving the rectangle of the parameter domain. Omitted →
    /// trimmed only when `split` produced several pieces.
    #[serde(default)]
    pub trim: Option<bool>,
    /// Tolerance (mm, 0.001 to 10, default 0.1) of the outline simplification: every dropped
    /// boundary vertex stays within it of the smoothed outline. Holes smaller than five
    /// tolerances across are ignored.
    #[serde(default)]
    pub outline_tolerance: Option<f64>,
    /// Split a region that is not a height field (it wraps around) into height-field pieces and
    /// fit each one (default false: such a region is refused). A split, not a G1 network.
    #[serde(default)]
    pub split: Option<bool>,
    /// Half-angle (degrees, 10 to 80, default 45) of the cone of normals one piece may span.
    #[serde(default)]
    pub max_piece_angle: Option<f64>,
    /// Most pieces (1 to 64, default 16).
    #[serde(default)]
    pub max_pieces: Option<usize>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// What the outline of a trimmed piece looked like.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct OutlineOut {
    /// Corners of the outer loop after simplification.
    pub corners: usize,
    /// Holes in the face.
    pub holes: usize,
    /// Holes ignored as smaller than five outline tolerances across.
    pub holes_ignored: usize,
    /// Loops of disconnected parts of the region that were not trimmed to.
    pub islands_ignored: usize,
    /// Area inside the outline (outer loop minus holes) measured in the fit plane, mm².
    pub area: f64,
    /// Outline simplification tolerance used, mm.
    pub tolerance: f64,
}

/// One sheet of a split fit.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PieceOut {
    /// Surface object of the piece.
    pub id: ObjectId,
    /// Triangles of the piece.
    pub triangles: usize,
    /// Control net used.
    pub control: [usize; 2],
    /// RMS deviation of the piece's points from its sheet (mm).
    pub rms: f64,
    /// Largest deviation of the piece's points from its sheet (mm).
    pub max: f64,
    /// Points measured.
    pub measured: usize,
    /// Within the target tolerance?
    pub within_tolerance: bool,
    /// Trimmed to the piece's outline?
    pub trimmed: bool,
    /// Outline facts when trimmed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline: Option<OutlineOut>,
    /// Advice for this piece.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
}

/// The gap between the sheets of two neighbouring pieces.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct GapOut {
    /// Index in `pieces` of the first piece.
    pub a: usize,
    /// Index in `pieces` of the second piece.
    pub b: usize,
    /// Mesh vertices the two pieces share.
    pub shared_vertices: usize,
    /// Largest distance between the two sheets' closest points to a shared vertex (mm).
    pub max_gap: f64,
}

/// Fit result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FitOut {
    /// New surface object (the first piece when the region was split).
    pub id: ObjectId,
    /// Control net used (of the first piece when split).
    pub control: [usize; 2],
    /// Degree.
    pub degree: usize,
    /// RMS deviation of the region points from the surface (mm); over all pieces when split.
    pub rms: f64,
    /// Largest deviation (mm); the worst piece when split.
    pub max: f64,
    /// Points measured.
    pub measured: usize,
    /// Within the target tolerance (every piece when split)?
    pub within_tolerance: bool,
    /// Advice when the target was not met or the region folds.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
    /// Is the (first) sheet trimmed to the region outline?
    pub trimmed: bool,
    /// Outline facts of the first sheet when trimmed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline: Option<OutlineOut>,
    /// The sheets of a split fit, in order (empty when the region is one piece).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pieces: Vec<PieceOut>,
    /// Gaps between neighbouring pieces.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub gaps: Vec<GapOut>,
    /// Largest gap over all neighbouring pieces (mm).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gap_max: Option<f64>,
    /// Parts of the region left out: why.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub skipped: Vec<String>,
    /// What a split means for the result.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// Validated settings of one fit.
#[derive(Clone, Copy, Debug)]
struct Settings {
    degree: usize,
    tol: f64,
    control: Option<[usize; 2]>,
    outline_tol: f64,
    split: bool,
    angle: f64,
    max_pieces: usize,
}

/// A fitted piece: its spline and statistics, not yet cut to its outline.
struct Piece {
    srf: Surface,
    grid: HeightGrid,
    triangles: Vec<u32>,
    control: [usize; 2],
    rms: f64,
    max: f64,
    measured: usize,
    warning: Option<String>,
}

/// A piece's sheet, trimmed to its outline or not.
struct Sheet {
    brep: Brep,
    trimmed: bool,
    outline: Option<OutlineOut>,
    warning: Option<String>,
}

/// What fitting one chart gave.
enum Fitted {
    Piece(Box<Piece>),
    NotHeightField,
    TooFew,
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
}

/// The smallest net whose maximum deviation, over every point, meets `tol` (else the best
/// tried). A net is first judged on an evenly strided subset; only one that meets `tol` there is
/// measured on all points, and if it fails there the search goes on to the next net.
fn select_net(grid: &HeightGrid, pts: &[Point3], st: &Settings) -> CmdResult<Net> {
    let stride = pts.len().div_ceil(PROBE_POINTS).max(1);
    let probe_pts: Vec<Point3> = pts.iter().step_by(stride).copied().collect();
    let rows = grid.grid.len();
    let candidates: Vec<[usize; 2]> = match st.control {
        Some(c) => vec![c],
        None => NET_LADDER.iter().map(|&c| [c, c]).filter(|c| c[0] > st.degree && c[0] <= rows).collect(),
    };
    let mut best: Option<(Net, f64)> = None;
    for c in candidates {
        let srf = fit_grid(&grid.grid, c[0], c[1], st.degree)?;
        let probe = SurfaceProbe::new(&srf)?;
        let sub = measure(&probe, &probe_pts)?;
        let (stats, full) = if sub.1 <= st.tol { (measure(&probe, pts)?, true) } else { (sub, false) };
        let done = full && stats.1 <= st.tol;
        if best.as_ref().is_none_or(|b| sub.1 < b.1) {
            best = Some((Net { srf, control: c, stats, full }, sub.1));
        }
        if done {
            break;
        }
    }
    let (mut net, _) = best.ok_or_else(|| CmdError::Failed("no surface could be fitted".into()))?;
    if !net.full {
        net.stats = measure(&SurfaceProbe::new(&net.srf)?, pts)?;
    }
    Ok(net)
}

/// The sheet trimmed to the outline of `tris`, with the outline's facts; the reason as `Err`
/// when it cannot be trimmed.
fn trim_sheet(mesh: &Mesh, tris: &[u32], grid: &HeightGrid, srf: &Surface, tol: f64) -> Result<(Brep, OutlineOut), String> {
    let outline = region_outline(mesh, tris, grid, tol).ok_or("the region has no boundary loop to trim to")?;
    let probe = SurfaceProbe::new(srf).map_err(|e| e.to_string())?;
    let to_uv = |loop_pts: &[Point3]| -> Result<Vec<[f64; 2]>, String> {
        loop_pts
            .iter()
            .map(|p| probe.closest_uv([p.x, p.y, p.z]).map(|(_, _, uv)| uv).map_err(|e| e.to_string()))
            .collect()
    };
    let outer = to_uv(&outline.outer)?;
    let holes = outline.holes.iter().map(|h| to_uv(h)).collect::<Result<Vec<_>, _>>()?;
    let sheet = trimmed_sheet(srf, &outer, &holes).map_err(|e| e.to_string())?;
    let out = OutlineOut {
        corners: outline.outer.len(),
        holes: outline.holes.len(),
        holes_ignored: outline.holes_ignored,
        islands_ignored: outline.islands_ignored,
        area: outline.area,
        tolerance: tol,
    };
    Ok((sheet, out))
}

/// Fits one chart. `Ok(NotHeightField)` / `Ok(TooFew)` are not errors: the caller may split.
fn fit_piece(mesh: &Mesh, normals: &[Vec3], tris: &[u32], st: &Settings) -> CmdResult<Fitted> {
    let (pts, nrm) = region_points(mesh, normals, tris);
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

/// The sheet of a piece: trimmed to the region outline when `trim`, else the whole rectangle of
/// the spline's parameter domain. A piece that cannot be trimmed stays untrimmed, with a warning.
fn make_sheet(mesh: &Mesh, p: &Piece, trim: bool, st: &Settings) -> CmdResult<Sheet> {
    if trim {
        match trim_sheet(mesh, &p.triangles, &p.grid, &p.srf, st.outline_tol) {
            Ok((brep, outline)) => return Ok(Sheet { brep, trimmed: true, outline: Some(outline), warning: None }),
            Err(why) => {
                let warning = format!("the sheet could not be trimmed to the region outline ({why}); it is untrimmed");
                return Ok(Sheet { brep: sheet_from_spline(&p.srf)?, trimmed: false, outline: None, warning: Some(warning) });
            }
        }
    }
    Ok(Sheet { brep: sheet_from_spline(&p.srf)?, trimmed: false, outline: None, warning: None })
}

/// Pieces fitted so far and what was left out.
#[derive(Default)]
struct Outcome {
    pieces: Vec<Piece>,
    skipped: Vec<String>,
}

/// Fewest triangles a chart of a region of `n` triangles may have.
fn min_chart(n: usize) -> usize {
    MIN_CHART_TRIANGLES.max((n as f64 * MIN_CHART_SHARE) as usize)
}

/// Fits `tris`; a chart that is not a height field is split (when `st.split`) by a cone `angle`
/// wide and each part fitted in turn, at most [`MAX_RESPLITS`] levels deep.
fn fit_chart(mesh: &Mesh, normals: &[Vec3], tris: &[u32], st: &Settings, angle: f64, depth: u32, out: &mut Outcome) -> CmdResult<()> {
    match fit_piece(mesh, normals, tris, st)? {
        Fitted::Piece(p) => match refine_by_split(mesh, normals, tris, st, angle, depth, &p, out)? {
            Some(finer) => {
                out.pieces.extend(finer.pieces);
                out.skipped.extend(finer.skipped);
            }
            None => out.pieces.push(*p),
        },
        Fitted::TooFew => out.skipped.push(format!("a part of {} triangles has too few points for a surface", tris.len())),
        Fitted::NotHeightField => {
            let room = out.pieces.len() + out.skipped.len() < st.max_pieces;
            if !st.split || depth > MAX_RESPLITS || !room {
                out.skipped.push(format!("a part of {} triangles is not a height field", tris.len()));
                return Ok(());
            }
            let params = SplitParams { max_angle_deg: angle, min_triangles: min_chart(tris.len()), max_pieces: st.max_pieces };
            let split = split_charts(mesh, tris, &params);
            if split.dropped > 0 {
                out.skipped.push(format!("{} triangles are too small a part to fit", split.dropped));
            }
            if split.charts.len() <= 1 && split.charts.first().is_some_and(|c| c.len() == tris.len()) {
                out.skipped.push(format!("a part of {} triangles is not a height field and cannot be split further", tris.len()));
                return Ok(());
            }
            let next = (angle * RESPLIT_FACTOR).max(PIECE_ANGLE_RANGE.0);
            for chart in &split.charts {
                fit_chart(mesh, normals, chart, st, next, depth + 1, out)?;
            }
        }
    }
    Ok(())
}

/// A piece that misses the tolerance on every net, in a split fit: tries its chart as finer
/// charts (a cone [`RESPLIT_FACTOR`] times narrower). The finer pieces replace it only when all
/// of them fit and the worst of them deviates less than `piece` did.
#[allow(clippy::too_many_arguments)]
fn refine_by_split(
    mesh: &Mesh,
    normals: &[Vec3],
    tris: &[u32],
    st: &Settings,
    angle: f64,
    depth: u32,
    piece: &Piece,
    out: &Outcome,
) -> CmdResult<Option<Outcome>> {
    let next = angle * RESPLIT_FACTOR;
    let room = out.pieces.len() + out.skipped.len() + 1 < st.max_pieces;
    if !st.split || st.control.is_some() || piece.max <= st.tol || depth >= MAX_RESPLITS || !room || next < PIECE_ANGLE_RANGE.0 {
        return Ok(None);
    }
    let params = SplitParams { max_angle_deg: next, min_triangles: min_chart(tris.len()), max_pieces: st.max_pieces };
    let split = split_charts(mesh, tris, &params);
    if split.charts.len() < 2 || split.charts.len() > MAX_REFINE_CHARTS {
        return Ok(None);
    }
    let mut finer = Outcome::default();
    for chart in &split.charts {
        fit_chart(mesh, normals, chart, st, next, depth + 1, &mut finer)?;
    }
    let worst = finer.pieces.iter().map(|p| p.max).fold(0.0, f64::max);
    let better = split.dropped == 0 && finer.skipped.is_empty() && !finer.pieces.is_empty() && worst < piece.max;
    Ok(better.then_some(finer))
}

/// Largest gap between the sheets of neighbouring pieces, measured at the mesh vertices the two
/// pieces share: the distance between the closest points of the two sheets to that vertex.
fn piece_gaps(mesh: &Mesh, pieces: &[Piece]) -> CmdResult<Vec<GapOut>> {
    let lists: Vec<Vec<u32>> = pieces.iter().map(|p| p.triangles.clone()).collect();
    let probes: Vec<SurfaceProbe> = pieces.iter().map(|p| SurfaceProbe::new(&p.srf)).collect::<Result<_, _>>()?;
    let mut gaps = Vec::new();
    for (a, b, verts) in shared_vertices(mesh, &lists) {
        let (Some(pa), Some(pb)) = (probes.get(a), probes.get(b)) else { continue };
        let stride = verts.len().div_ceil(GAP_SAMPLES).max(1);
        let mut max_gap: f64 = 0.0;
        for &v in verts.iter().step_by(stride) {
            let Some(p) = mesh.positions.get(v as usize) else { continue };
            let (_, qa) = pa.closest([p.x, p.y, p.z])?;
            let (_, qb) = pb.closest([p.x, p.y, p.z])?;
            max_gap = max_gap.max(((qa[0] - qb[0]).powi(2) + (qa[1] - qb[1]).powi(2) + (qa[2] - qb[2]).powi(2)).sqrt());
        }
        gaps.push(GapOut { a, b, shared_vertices: verts.len(), max_gap });
    }
    Ok(gaps)
}

/// The mesh, its triangles and the region they came from.
struct Input {
    mesh: Mesh,
    tris: Vec<u32>,
    region: Option<ObjectId>,
}

fn resolve_input(s: &Session, i: &FitIn) -> CmdResult<Input> {
    let (mesh_id, tris, region) = match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let reg = s.doc.region(rid).ok_or_else(|| CmdError::Params(format!("no region {rid}")))?;
            (reg.mesh, reg.triangles.to_vec(), Some(rid))
        }
        (None, Some(m)) => {
            let mid = m.resolve(s, ObjectKind::Mesh)?;
            let all = s.doc.mesh(mid).map_or(0, |m| m.mesh.triangle_count()) as u32;
            (mid, i.triangles.clone().unwrap_or_else(|| (0..all).collect()), None)
        }
        _ => return Err(CmdError::Params("give `region`, or `mesh` (and optionally `triangles`)".into())),
    };
    let mesh = s.doc.mesh(mesh_id).map(|m| (*m.mesh).clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh_id}")))?;
    if let Some(t) = tris.iter().find(|&&t| t as usize >= mesh.triangle_count()) {
        return Err(CmdError::Params(format!("triangle {t} out of range")));
    }
    Ok(Input { mesh, tris, region })
}

fn in_range(v: f64, (lo, hi): (f64, f64)) -> bool {
    v.is_finite() && v >= lo && v <= hi
}

fn settings(i: &FitIn) -> CmdResult<Settings> {
    let degree = i.degree.unwrap_or(3);
    if !(1..=5).contains(&degree) {
        return Err(CmdError::Params("degree must be 1..=5".into()));
    }
    let tol = i.tolerance.unwrap_or(0.2);
    if !(tol.is_finite() && tol > 0.0) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    let outline_tol = i.outline_tolerance.unwrap_or(DEFAULT_OUTLINE_TOLERANCE);
    if !in_range(outline_tol, OUTLINE_TOLERANCE_RANGE) {
        return Err(CmdError::Params("outline_tolerance must be between 0.001 and 10 mm".into()));
    }
    let angle = i.max_piece_angle.unwrap_or(DEFAULT_PIECE_ANGLE);
    if !in_range(angle, PIECE_ANGLE_RANGE) {
        return Err(CmdError::Params("max_piece_angle must be between 10 and 80 degrees".into()));
    }
    let max_pieces = i.max_pieces.unwrap_or(DEFAULT_MAX_PIECES);
    if !(1..=MAX_PIECES_LIMIT).contains(&max_pieces) {
        return Err(CmdError::Params(format!("max_pieces must be 1..={MAX_PIECES_LIMIT}")));
    }
    let split = i.split.unwrap_or(false);
    Ok(Settings { degree, tol, control: i.control, outline_tol, split, angle, max_pieces })
}

fn refusal(skipped: &[String], split: bool) -> CmdError {
    if split {
        return CmdError::Failed(format!("no piece of the region could be fitted: {}", skipped.join("; ")));
    }
    CmdError::Failed(
        "the region is not a single patch over its plane (it folds or is degenerate); split it, or set `split` to \
         fit it as several height-field pieces"
            .into(),
    )
}

fn add_piece(s: &mut Session, p: &Piece, sheet: &Sheet, name: String, region: Option<ObjectId>, degree: usize) -> ObjectId {
    let display = surfacecraft_kernel::tess::tessellate(&sheet.brep, None).unwrap_or_default();
    s.doc.add_surface(SurfaceObject {
        id: ObjectId(0),
        name,
        sheet: Arc::new(sheet.brep.clone()),
        region,
        control: p.control,
        degree,
        rms: p.rms,
        max: p.max,
        display: Arc::new(display),
        parent: None,
        visible: true,
    })
}

pub(super) fn fit_cmd(s: &mut Session, i: FitIn) -> CmdResult<FitOut> {
    let st = settings(&i)?;
    let input = resolve_input(s, &i)?;
    let normals = input.mesh.vertex_normals();
    if region_points(&input.mesh, &normals, &input.tris).0.len() < MIN_POINTS {
        return Err(CmdError::Params(format!("too few points for a surface (need {MIN_POINTS})")));
    }
    let mut out = Outcome::default();
    fit_chart(&input.mesh, &normals, &input.tris, &st, st.angle, 0, &mut out)?;
    if out.pieces.is_empty() {
        return Err(refusal(&out.skipped, st.split));
    }
    let many = out.pieces.len() > 1;
    // the pieces of a split tile the region only when each is cut to its outline
    let trim = i.trim.unwrap_or(many);
    let gaps = if many { piece_gaps(&input.mesh, &out.pieces)? } else { Vec::new() };
    let base = i.name.clone().unwrap_or_else(|| "surface".into());
    let mut piece_out = Vec::with_capacity(out.pieces.len());
    for (k, p) in out.pieces.iter().enumerate() {
        let sheet = make_sheet(&input.mesh, p, trim, &st)?;
        let name = if many { format!("{base} {}", k + 1) } else { base.clone() };
        let id = add_piece(s, p, &sheet, name, input.region, st.degree);
        let warning = [p.warning.clone(), sheet.warning.clone()].into_iter().flatten().reduce(|a, b| format!("{a}; {b}"));
        piece_out.push(PieceOut {
            id,
            triangles: p.triangles.len(),
            control: p.control,
            rms: p.rms,
            max: p.max,
            measured: p.measured,
            within_tolerance: p.max <= st.tol,
            trimmed: sheet.trimmed,
            outline: sheet.outline,
            warning,
        });
    }
    Ok(summarise(&st, out.skipped, piece_out, gaps))
}

/// The result of the whole fit from its pieces.
fn summarise(st: &Settings, skipped: Vec<String>, pieces: Vec<PieceOut>, gaps: Vec<GapOut>) -> FitOut {
    let total: usize = pieces.iter().map(|p| p.measured).sum();
    let sq: f64 = pieces.iter().map(|p| p.rms * p.rms * p.measured as f64).sum();
    let max = pieces.iter().map(|p| p.max).fold(0.0, f64::max);
    let many = pieces.len() > 1;
    let first = pieces.first().cloned();
    let (id, control, trimmed, outline) = first
        .as_ref()
        .map_or((ObjectId(0), [0, 0], false, None), |f| (f.id, f.control, f.trimmed, f.outline.clone()));
    let mut warning: Vec<String> = pieces.iter().filter_map(|p| p.warning.clone()).collect();
    warning.dedup();
    if !skipped.is_empty() {
        warning.push("part of the region was left out (see `skipped`)".into());
    }
    let gap_max = gaps.iter().map(|g| g.max_gap).reduce(f64::max);
    FitOut {
        id,
        control,
        degree: st.degree,
        rms: (sq / total.max(1) as f64).sqrt(),
        max,
        measured: total,
        within_tolerance: max <= st.tol && skipped.is_empty(),
        warning: (!warning.is_empty()).then(|| warning.join("; ")),
        trimmed,
        outline,
        pieces: if many { pieces } else { Vec::new() },
        gaps,
        gap_max,
        skipped,
        note: many.then(|| SPLIT_NOTE.to_string()),
    }
}
