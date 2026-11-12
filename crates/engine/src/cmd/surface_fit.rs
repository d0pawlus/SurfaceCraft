//! `surface.fit`: NURBS sheets fitted to a region (design doc §6.3, milestone M3), trimmed to the
//! region's outline and split into height-field pieces when the region wraps (P8-H4).
//!
//! The pieces are fitted in `surface_pieces`; here the sheets are made (the whole rectangle of the
//! spline, or trimmed to the outline of the region with `fit::outline` and
//! `kernel::nurbs::trimmed_sheet`), the geometry delivered is measured (`surface_check`) and the
//! result reports three separate numbers: the surface deviation (region points to the spline),
//! the outline deviation (region boundary vertices to the trim edges) and the delivered
//! deviation (region points to the tessellated sheets). `within_tolerance` needs all that apply.
use super::surface_check::{delivered_deviation, outline_deviation};
use super::surface_mask::{BoxIn, ExclusionOut, Mask, SphereIn};
use super::surface_pieces::{Ctx, MIN_POINTS, Outcome, Piece, Settings, fit_chart, piece_gaps};
use super::surface_report::{
    CoverageOut, CoverageShares, PieceCoverage, SampleCounts, piece_coverage, sample_counts, summarise_coverage,
};
use super::{CmdError, CmdResult, ObjRef};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind, SurfaceObject};
use surfacecraft_fit::outline::region_outline;
use surfacecraft_geom::{Mesh, Point3};
use surfacecraft_kernel::nurbs::{MAX_OUTLINE_CORNERS, SurfaceProbe, sheet_from_spline, trimmed_sheet};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::tess::tessellate_all;

/// Defaults and limits of the new parameters.
const DEFAULT_OUTLINE_TOLERANCE: f64 = 0.1;
const OUTLINE_TOLERANCE_RANGE: (f64, f64) = (0.001, 10.0);
const DEFAULT_PIECE_ANGLE: f64 = 45.0;
const PIECE_ANGLE_RANGE: (f64, f64) = (10.0, 80.0);
const DEFAULT_MAX_PIECES: usize = 16;
const MAX_PIECES_LIMIT: usize = 64;
/// Chord (mm) of the tessellation the delivered deviation is measured on, as a share of the
/// tolerance, within these limits.
const DELIVERED_CHORD_SHARE: f64 = 0.025;
const DELIVERED_CHORD_RANGE: (f64, f64) = (0.002, 0.01);
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
    /// Trim each sheet to the outline of its region (outer loop and holes, simplified to
    /// `outline_tolerance`) instead of leaving the rectangle of the parameter domain. Omitted →
    /// trimmed only when `split` produced several pieces.
    #[serde(default)]
    pub trim: Option<bool>,
    /// Tolerance (mm, 0.001 to 10, default 0.1) of the trim outline: no boundary vertex of the
    /// region is farther than this from the trim edge. Holes smaller than five tolerances across
    /// are ignored.
    #[serde(default)]
    #[schemars(range(min = 0.001, max = 10.0))]
    pub outline_tolerance: Option<f64>,
    /// Split a region that is not a height field (it wraps around) into height-field pieces and
    /// fit each one (default false: such a region is refused). A split, not a G1 network.
    #[serde(default)]
    pub split: Option<bool>,
    /// Half-angle (degrees, 10 to 80, default 45) of the cone of normals one piece may span.
    #[serde(default)]
    #[schemars(range(min = 10.0, max = 80.0))]
    pub max_piece_angle: Option<f64>,
    /// Most pieces (1 to 64, default 16).
    #[serde(default)]
    #[schemars(range(min = 1, max = 64))]
    pub max_pieces: Option<usize>,
    /// Triangles (mesh indices) to leave out of the fit and its deviation numbers; the region and mesh stay unchanged, indices outside the region are ignored and counted.
    #[serde(default)]
    pub exclude_triangles: Option<Vec<u32>>,
    /// Spheres to leave out: a triangle whose centroid is inside one is excluded like one in `exclude_triangles`.
    #[serde(default)]
    pub exclude_spheres: Option<Vec<SphereIn>>,
    /// Boxes to leave out: a triangle whose centroid is inside one is excluded like one in `exclude_triangles`.
    #[serde(default)]
    pub exclude_boxes: Option<Vec<BoxIn>>,
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
    /// Holes ignored as smaller than five outline tolerances across (noise).
    pub holes_ignored: usize,
    /// Boundary loops of disconnected parts of the region that were NOT trimmed to: the face
    /// covers only the largest part (`within_tolerance` is false when this is not zero).
    pub islands_ignored: usize,
    /// Area inside the outline (outer loop minus holes) measured in the fit plane, mm².
    pub area: f64,
    /// Outline simplification tolerance asked for, mm.
    pub tolerance: f64,
    /// Outline deviation: the largest distance (mm) from the closest point on the surface of a
    /// boundary vertex of the region to the trim edges of the delivered face (in the surface).
    pub deviation: f64,
}

/// One sheet of a fit.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PieceOut {
    /// Surface object of the piece.
    pub id: ObjectId,
    /// Triangles of the piece.
    pub triangles: usize,
    /// Control net used.
    pub control: [usize; 2],
    /// RMS deviation of the piece's points from its spline (mm).
    pub rms: f64,
    /// Largest deviation of the piece's points from its spline (mm); same as `surface_deviation`.
    pub max: f64,
    /// Surface deviation: largest distance of the piece's points from its spline (mm).
    pub surface_deviation: f64,
    /// Outline deviation (mm), when trimmed: boundary vertices of the region to the trim edges.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_deviation: Option<f64>,
    /// Delivered deviation (mm), when trimmed: the piece's points to the tessellated delivered
    /// sheets (all pieces together), chord 0.002 to 0.01 mm.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivered_deviation: Option<f64>,
    /// Points measured.
    pub measured: usize,
    /// Shares of the sheet's parameter domain with data under them, bridged holes and extrapolation.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub coverage: Option<CoverageShares>,
    /// Surface deviation within the tolerance, and, when trimming was asked for, the sheet is
    /// trimmed, its outline deviation within `outline_tolerance`, its delivered deviation within
    /// the tolerance and no part of the region left out.
    pub within_tolerance: bool,
    /// Trimmed to the piece's outline?
    pub trimmed: bool,
    /// Why the sheet is untrimmed although trimming was asked for.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trim_error: Option<String>,
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
    /// Largest distance between the two sheets' closest points to a shared vertex (mm): on the
    /// delivered (trimmed) faces when trimmed, else on the splines.
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
    /// Largest deviation of the region points from the splines (mm); the worst piece when split.
    pub max: f64,
    /// Surface deviation: same as `max`.
    pub surface_deviation: f64,
    /// Outline deviation: the worst piece's (mm), when any sheet is trimmed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_deviation: Option<f64>,
    /// Delivered deviation: the worst piece's (mm), when any sheet is trimmed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub delivered_deviation: Option<f64>,
    /// Points measured (distinct points over all pieces); same as `samples.measured`.
    pub measured: usize,
    /// What became of the points of the region: eligible, measured, excluded, failed.
    #[serde(default)]
    pub samples: SampleCounts,
    /// The exclusion masks applied and what they took out; absent when none was given. Excluded
    /// points are in none of the deviation numbers.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub exclusion: Option<ExclusionOut>,
    /// Which parts of the patch's parameter domain have scan data under them, which bridge a hole
    /// of the data and which extrapolate.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub coverage: Option<CoverageOut>,
    /// True only when every piece is within tolerance (see `PieceOut::within_tolerance`) and no
    /// part of the region was left out.
    pub within_tolerance: bool,
    /// Advice when the target was not met or the region folds.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub warning: Option<String>,
    /// Is the (first) sheet trimmed to the region outline?
    pub trimmed: bool,
    /// Why a sheet is untrimmed although trimming was asked for (the first such piece).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub trim_error: Option<String>,
    /// Outline facts of the first sheet when trimmed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline: Option<OutlineOut>,
    /// Boundary loops of disconnected parts left out of the trimmed faces, over all pieces.
    pub islands_ignored: usize,
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

/// A piece's sheet and what is known of it.
struct Sheet {
    brep: Brep,
    /// Tessellation at the delivered chord, kept when the sheet was checked by meshing it.
    tess: Option<Mesh>,
    trimmed: bool,
    outline: Option<OutlineOut>,
    trim_error: Option<String>,
}

/// The sheet trimmed to the outline of `tris`; the reason as `Err` when it cannot be trimmed.
fn trim_sheet(mesh: &Mesh, tris: &[u32], p: &Piece, tol: f64) -> Result<(Brep, OutlineOut), String> {
    let outline = region_outline(mesh, tris, &p.grid, tol, MAX_OUTLINE_CORNERS)
        .ok_or("the region has no boundary loop to trim to")?;
    let probe = SurfaceProbe::new(&p.srf).map_err(|e| e.to_string())?;
    let to_uv = |loop_pts: &[Point3]| -> Result<Vec<[f64; 2]>, String> {
        loop_pts
            .iter()
            .map(|q| probe.closest_uv([q.x, q.y, q.z]).map(|(_, _, uv)| uv).map_err(|e| e.to_string()))
            .collect()
    };
    let outer = to_uv(&outline.outer)?;
    let holes = outline.holes.iter().map(|h| to_uv(h)).collect::<Result<Vec<_>, _>>()?;
    let sheet = trimmed_sheet(&p.srf, &outer, &holes).map_err(|e| e.to_string())?;
    // in-surface: the boundary vertices lie on the scan, the trim edges on the spline; measure
    // from the vertices' closest points on the spline, so the surface deviation is not counted twice
    let on_surface = outline
        .boundary
        .iter()
        .map(|q| probe.closest([q.x, q.y, q.z]).map(|(_, c)| Point3::new(c[0], c[1], c[2])).map_err(|e| e.to_string()))
        .collect::<Result<Vec<_>, _>>()?;
    let deviation = outline_deviation(&on_surface, &sheet).ok_or("the trimmed sheet has no edges")?;
    let out = OutlineOut {
        corners: outline.outer.len(),
        holes: outline.holes.len(),
        holes_ignored: outline.holes_ignored,
        islands_ignored: outline.islands_ignored,
        area: outline.area,
        tolerance: tol,
        deviation,
    };
    Ok((sheet, out))
}

/// The sheet of a piece: trimmed to the region outline when `trim`, else the whole rectangle of
/// the spline's parameter domain. A piece that cannot be trimmed stays untrimmed, and says why.
fn make_sheet(mesh: &Mesh, p: &Piece, trim: bool, st: &Settings) -> CmdResult<Sheet> {
    if !trim {
        return Ok(Sheet {
            brep: sheet_from_spline(&p.srf)?,
            tess: None,
            trimmed: false,
            outline: None,
            trim_error: None,
        });
    }
    let tried = trim_sheet(mesh, &p.triangles, p, st.outline_tol).and_then(|(brep, outline)| {
        // the delivered face must mesh: a face the kernel's mesher refuses is no sheet to give
        let tess = tessellate_all(&brep, Some(delivered_chord(st.tol))).map_err(|e| e.to_string())?;
        Ok((brep, tess, outline))
    });
    match tried {
        Ok((brep, tess, outline)) => {
            Ok(Sheet { brep, tess: Some(tess), trimmed: true, outline: Some(outline), trim_error: None })
        }
        Err(why) => Ok(Sheet {
            brep: sheet_from_spline(&p.srf)?,
            tess: None,
            trimmed: false,
            outline: None,
            trim_error: Some(why),
        }),
    }
}

/// Chord (mm) of the tessellation the delivered deviation is measured on.
fn delivered_chord(tol: f64) -> f64 {
    (tol * DELIVERED_CHORD_SHARE).clamp(DELIVERED_CHORD_RANGE.0, DELIVERED_CHORD_RANGE.1)
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
    let mesh =
        s.doc.mesh(mesh_id).map(|m| (*m.mesh).clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh_id}")))?;
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

/// The tessellations of the sheets and the deviation of the points of each piece from all of
/// them together; `None` when no sheet is trimmed (the sheets are then the splines themselves).
fn delivered(
    cx: &Ctx,
    pieces: &[Piece],
    sheets: &[Sheet],
    tol: f64,
) -> CmdResult<(Vec<Option<f64>>, Option<Vec<Mesh>>)> {
    if !sheets.iter().any(|s| s.trimmed) {
        return Ok((vec![None; pieces.len()], None));
    }
    let chord = delivered_chord(tol);
    let tess = sheets
        .iter()
        .map(|s| match &s.tess {
            Some(t) => Ok(t.clone()),
            None => Ok(tessellate_all(&s.brep, Some(chord))?),
        })
        .collect::<CmdResult<Vec<Mesh>>>()?;
    let mut union = Mesh::default();
    for t in &tess {
        union.append(t);
    }
    let dev = pieces
        .iter()
        .map(|p| Some(delivered_deviation(&cx.mask.points(cx.mesh, cx.normals, &p.triangles).0, &union)))
        .collect();
    Ok((dev, Some(tess)))
}

fn add_piece(
    s: &mut Session,
    p: &Piece,
    sheet: &Sheet,
    name: String,
    region: Option<ObjectId>,
    degree: usize,
) -> CmdResult<ObjectId> {
    let display = tessellate_all(&sheet.brep, None)?;
    Ok(s.doc.add_surface(SurfaceObject {
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
    }))
}

/// One piece's result: what is known, and whether it is within tolerance.
fn piece_out(
    id: ObjectId,
    p: &Piece,
    sheet: Sheet,
    delivered: Option<f64>,
    cov: Option<&PieceCoverage>,
    (trim, st): (bool, &Settings),
) -> PieceOut {
    let outline_dev = sheet.outline.as_ref().map(|o| o.deviation);
    let islands = sheet.outline.as_ref().map_or(0, |o| o.islands_ignored);
    let trim_ok = !trim
        || (sheet.trimmed
            && islands == 0
            && outline_dev.is_some_and(|d| d <= st.outline_tol)
            && delivered.is_some_and(|d| d <= st.tol));
    let notes = [
        p.warning.clone(),
        (p.failed > 0)
            .then(|| format!("{} point(s) could not be projected onto the spline and are not measured", p.failed)),
        sheet
            .trim_error
            .as_ref()
            .map(|why| format!("the sheet could not be trimmed to the region outline ({why}); it is untrimmed")),
        (islands > 0)
            .then(|| format!("{islands} disconnected part(s) of the region are not covered by the trimmed face")),
        outline_dev.filter(|d| *d > st.outline_tol).map(|d| {
            format!("the trim edges are {d:.3} mm from the region boundary (outline tolerance {})", st.outline_tol)
        }),
        delivered
            .filter(|d| *d > st.tol)
            .map(|d| format!("the delivered sheets deviate {d:.4} mm from the region points (target {})", st.tol)),
    ];
    PieceOut {
        id,
        triangles: p.triangles.len(),
        control: p.control,
        rms: p.rms,
        max: p.max,
        surface_deviation: p.max,
        outline_deviation: outline_dev,
        delivered_deviation: delivered,
        measured: p.measured,
        coverage: cov.map(|c| CoverageShares::from(&c.coverage)),
        within_tolerance: p.max <= st.tol && trim_ok,
        trimmed: sheet.trimmed,
        trim_error: sheet.trim_error,
        outline: sheet.outline,
        warning: notes.into_iter().flatten().reduce(|a, b| format!("{a}; {b}")),
    }
}

pub(super) fn fit_cmd(s: &mut Session, i: FitIn) -> CmdResult<FitOut> {
    let st = settings(&i)?;
    let input = resolve_input(s, &i)?;
    let normals = input.mesh.vertex_normals();
    let (mask, exclusion) = Mask::build(
        &input.mesh,
        &input.tris,
        i.exclude_triangles.as_deref(),
        i.exclude_spheres.as_deref(),
        i.exclude_boxes.as_deref(),
    )?;
    let cx = Ctx { mesh: &input.mesh, normals: &normals, st: &st, mask: &mask };
    if mask.points(&input.mesh, &normals, &input.tris).0.len() < MIN_POINTS {
        return Err(CmdError::Params(format!(
            "too few points for a surface (need {MIN_POINTS}), after the exclusions"
        )));
    }
    let mut out = Outcome::default();
    fit_chart(&cx, &input.tris, st.angle, 0, st.max_pieces, &mut out)?;
    if out.pieces.is_empty() {
        return Err(refusal(&out.skipped, st.split));
    }
    let many = out.pieces.len() > 1;
    // the pieces of a split tile the region only when each is cut to its outline
    let trim = i.trim.unwrap_or(many);
    let sheets = out.pieces.iter().map(|p| make_sheet(&input.mesh, p, trim, &st)).collect::<CmdResult<Vec<_>>>()?;
    let (dev, tess) = delivered(&cx, &out.pieces, &sheets, st.tol)?;
    let gaps = if many { piece_gaps(&input.mesh, &out.pieces, tess.as_deref())? } else { Vec::new() };
    let covers: Vec<Option<PieceCoverage>> = out.pieces.iter().map(|p| piece_coverage(&input.mesh, &mask, p)).collect();
    let base = i.name.clone().unwrap_or_else(|| "surface".into());
    let samples = sample_counts(&input.mesh, &mask, &input.tris, &out.pieces);
    let mut results = Vec::with_capacity(out.pieces.len());
    for (k, (((p, sheet), d), cov)) in out.pieces.iter().zip(sheets).zip(dev).zip(&covers).enumerate() {
        let name = if many { format!("{base} {}", k + 1) } else { base.clone() };
        let id = add_piece(s, p, &sheet, name, input.region, st.degree)?;
        results.push(piece_out(id, p, sheet, d, cov.as_ref(), (trim, &st)));
    }
    let report = Report { samples, exclusion, coverage: summarise_coverage(&covers) };
    Ok(summarise(&st, out.skipped, results, gaps, report))
}

/// What the fit says about its own measurement.
struct Report {
    samples: SampleCounts,
    exclusion: Option<ExclusionOut>,
    coverage: Option<CoverageOut>,
}

/// The result of the whole fit from its pieces.
fn summarise(st: &Settings, skipped: Vec<String>, pieces: Vec<PieceOut>, gaps: Vec<GapOut>, report: Report) -> FitOut {
    let total: usize = pieces.iter().map(|p| p.measured).sum();
    let sq: f64 = pieces.iter().map(|p| p.rms * p.rms * p.measured as f64).sum();
    let max = pieces.iter().map(|p| p.max).fold(0.0, f64::max);
    let worst = |f: fn(&PieceOut) -> Option<f64>| pieces.iter().filter_map(f).reduce(f64::max);
    let many = pieces.len() > 1;
    let first = pieces.first().cloned();
    let (id, control, trimmed, outline) =
        first.as_ref().map_or((ObjectId(0), [0, 0], false, None), |f| (f.id, f.control, f.trimmed, f.outline.clone()));
    let mut warning: Vec<String> = pieces.iter().filter_map(|p| p.warning.clone()).collect();
    warning.dedup();
    if !skipped.is_empty() {
        warning.push("part of the region was left out (see `skipped`)".into());
    }
    let mask_note = report.exclusion.as_ref().map(|e| {
        format!(
            "{} point(s) of {} were excluded by mask and are in none of the deviation numbers",
            e.points, report.samples.eligible
        )
    });
    let gap_max = gaps.iter().map(|g| g.max_gap).reduce(f64::max);
    FitOut {
        id,
        control,
        degree: st.degree,
        rms: (sq / total.max(1) as f64).sqrt(),
        max,
        surface_deviation: max,
        outline_deviation: worst(|p| p.outline_deviation),
        delivered_deviation: worst(|p| p.delivered_deviation),
        measured: report.samples.measured,
        samples: report.samples,
        exclusion: report.exclusion,
        coverage: report.coverage,
        within_tolerance: pieces.iter().all(|p| p.within_tolerance) && skipped.is_empty(),
        warning: (!warning.is_empty()).then(|| warning.join("; ")),
        trimmed,
        trim_error: pieces.iter().find_map(|p| p.trim_error.clone()),
        outline,
        islands_ignored: pieces.iter().filter_map(|p| p.outline.as_ref()).map(|o| o.islands_ignored).sum(),
        pieces: if many { pieces } else { Vec::new() },
        gaps,
        gap_max,
        skipped,
        note: [many.then(|| SPLIT_NOTE.to_string()), mask_note]
            .into_iter()
            .flatten()
            .reduce(|a, b| format!("{a}; {b}")),
    }
}
