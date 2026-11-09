//! `section.stack` and `export.section_sheet`: plane sections of a mesh taken as a stack of
//! parallel planes (per level: chains, perimeter, area, centroid, size and fitted circles; across
//! the stack: the draft per side and the taper of the outline radii), and the chains written as
//! 2-D drawings in plane coordinates (DXF or SVG).
use super::{CmdError, CmdResult, CommandSpec, ObjRef, write_file};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::ObjectKind;
use surfacecraft_geom::section::{self, PlaneFrame, StackReport};
use surfacecraft_geom::{Mesh, Point3, Vec3};
use surfacecraft_io::{DrawLayer, DrawPath};

/// Most planes one command cuts.
const MAX_LEVELS: usize = 500;
/// Default tolerance (mm) of the drawing simplification.
const DEFAULT_TOLERANCE: f64 = 0.001;

/// Parameters of `section.stack`: `count` planes at offsets `start + k · step` (mm) along
/// `normal` from `origin`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StackIn {
    /// Mesh (id or name).
    pub mesh: ObjRef,
    /// A point of the reference plane, mm (default [0, 0, 0]).
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// Normal of the planes (default [0, 0, 1]).
    #[serde(default)]
    pub normal: Option<[f64; 3]>,
    /// Offset of the first plane along the normal, mm (default 0).
    #[serde(default)]
    pub start: Option<f64>,
    /// Distance between consecutive planes, mm (default 0; must be non-zero when count > 1).
    #[serde(default)]
    pub step: Option<f64>,
    /// Number of planes, 1 to 500 (default 1).
    #[serde(default)]
    pub count: Option<usize>,
}

/// Plane axes in model coordinates: `u` and `v` lie in the plane, `normal` is the stack direction.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AxesOut {
    /// First in-plane axis: the side `+u` of the trends.
    pub u: [f64; 3],
    /// Second in-plane axis (`u × normal`).
    pub v: [f64; 3],
    /// Unit normal.
    pub normal: [f64; 3],
}

/// A fitted circle of a near-circular closed chain.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct CircleOut {
    /// Centre in plane coordinates `[u, v]`, mm.
    pub center: [f64; 2],
    /// Radius, mm.
    pub radius: f64,
    /// Largest distance of a chain point from the circle, mm.
    pub max_dev: f64,
}

/// One plane of the stack.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct LevelOut {
    /// Position in the stack (0 is the first plane).
    pub index: usize,
    /// Distance of the plane from the origin along the normal, mm.
    pub offset: f64,
    /// Closed chains.
    pub closed: usize,
    /// Open chains.
    pub open: usize,
    /// Length of every chain (closing edges included), mm.
    pub perimeter: f64,
    /// Area enclosed by the closed chains, holes subtracted, mm².
    pub area: f64,
    /// Centroid of that area (or of the open chains), model coordinates, mm.
    pub centroid: Option<[f64; 3]>,
    /// Extent of every chain along `u` and `v`, mm.
    pub size: [f64; 2],
    /// Fitted circles of the near-circular closed chains, largest first.
    pub circles: Vec<CircleOut>,
}

/// The draft of one side of the outline.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SideOut {
    /// `+u`, `-u`, `+v` or `-v`.
    pub side: String,
    /// Draft in degrees from the least-squares slope of the side against height: positive when the
    /// side moves inward as the stack rises.
    pub draft_deg: f64,
    /// Root-mean-square residual of that fit, mm (small means a straight draft).
    pub rms: f64,
}

/// The taper of the outline radii over the stack.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RadiusOut {
    /// Taper in degrees: positive when the radius shrinks as the stack rises.
    pub taper_deg: f64,
    /// Slope of radius against offset (mm per mm).
    pub slope: f64,
    /// Radius extrapolated to the origin plane, mm.
    pub radius_at_zero: f64,
    /// Root-mean-square residual of the linear fit, mm.
    pub rms: f64,
    /// Levels whose outline is near-circular.
    pub levels: usize,
}

/// The trends over the stack.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TrendOut {
    /// `prismatic` (constant chains, area and zero draft), `tapered` (linear draft that is not
    /// constant), `varying`, or `insufficient` (fewer than two levels with an outline).
    pub profile: String,
    /// True when the profile is constant along the stack.
    pub prismatic: bool,
    /// Draft per side (empty when insufficient).
    pub sides: Vec<SideOut>,
    /// Taper of the outline radii, when at least two outlines are near-circular.
    pub radius: Option<RadiusOut>,
}

/// Result of `section.stack`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct StackOut {
    /// The plane axes the levels are measured in.
    pub axes: AxesOut,
    /// One entry per plane, in order.
    pub levels: Vec<LevelOut>,
    /// The trends over the stack.
    pub trend: TrendOut,
}

/// Output format of `export.section_sheet`.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema, PartialEq, Eq)]
#[serde(rename_all = "lowercase")]
pub enum SheetFormat {
    /// ASCII DXF: LINE and LWPOLYLINE entities, one layer per level, millimetres.
    #[default]
    Dxf,
    /// SVG: one path per chain, viewBox in millimetres.
    Svg,
}

/// Parameters of `export.section_sheet`: the stack of `section.stack`, drawn.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SheetIn {
    /// Mesh (id or name).
    pub mesh: ObjRef,
    /// A point of the reference plane, mm (default [0, 0, 0]).
    #[serde(default)]
    pub origin: Option<[f64; 3]>,
    /// Normal of the planes (default [0, 0, 1]).
    #[serde(default)]
    pub normal: Option<[f64; 3]>,
    /// Offset of the first plane along the normal, mm (default 0).
    #[serde(default)]
    pub start: Option<f64>,
    /// Distance between consecutive planes, mm (default 0; must be non-zero when count > 1).
    #[serde(default)]
    pub step: Option<f64>,
    /// Number of planes, 1 to 500: 1 gives one section (default 1).
    #[serde(default)]
    pub count: Option<usize>,
    /// `dxf` (default) or `svg`.
    #[serde(default)]
    pub format: SheetFormat,
    /// Collinear points within this distance (mm) of a straight run are dropped (default 0.001).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Output path; omitted → the drawing comes back as `text`.
    #[serde(default)]
    pub path: Option<String>,
}

/// Result of `export.section_sheet`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SheetOut {
    /// The format written.
    pub format: SheetFormat,
    /// Layers written (one per plane).
    pub levels: usize,
    /// Paths written (one per chain).
    pub chains: usize,
    /// Size of the drawing in bytes.
    pub bytes: usize,
    /// Path written.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// The drawing as text when no path was given.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub text: Option<String>,
}

/// The mesh an object reference names, shared rather than copied.
fn mesh_of(s: &Session, r: &ObjRef) -> CmdResult<Arc<Mesh>> {
    let id = r.resolve(s, ObjectKind::Mesh)?;
    s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))
}

/// The planes of a stack, checked.
struct Planes {
    frame: PlaneFrame,
    start: f64,
    step: f64,
    count: usize,
}

fn planes(
    origin: Option<[f64; 3]>,
    normal: Option<[f64; 3]>,
    start: Option<f64>,
    step: Option<f64>,
    count: Option<usize>,
) -> CmdResult<Planes> {
    let frame =
        PlaneFrame::new(Point3::from(origin.unwrap_or([0.0; 3])), Vec3::from(normal.unwrap_or([0.0, 0.0, 1.0])))
            .ok_or_else(|| CmdError::Params("the planes need a finite origin and a non-zero, finite normal".into()))?;
    let (start, step, count) = (start.unwrap_or(0.0), step.unwrap_or(0.0), count.unwrap_or(1));
    if !(start.is_finite() && step.is_finite()) {
        return Err(CmdError::Params("start and step must be finite (mm)".into()));
    }
    if count == 0 || count > MAX_LEVELS {
        return Err(CmdError::Params(format!("count must be between 1 and {MAX_LEVELS}")));
    }
    if count > 1 && step == 0.0 {
        return Err(CmdError::Params("a stack of more than one plane needs a non-zero step".into()));
    }
    Ok(Planes { frame, start, step, count })
}

fn xyz(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn point_xyz(p: Point3) -> [f64; 3] {
    [p.x, p.y, p.z]
}

fn stack_out(r: &StackReport) -> StackOut {
    let f = &r.frame;
    let levels = r
        .levels
        .iter()
        .enumerate()
        .map(|(index, l)| LevelOut {
            index,
            offset: l.offset,
            closed: l.closed,
            open: l.open,
            perimeter: l.perimeter,
            area: l.area,
            centroid: l.centroid.map(point_xyz),
            size: l.size,
            circles: l
                .circles
                .iter()
                .map(|c| CircleOut { center: c.center, radius: c.radius, max_dev: c.max_dev })
                .collect(),
        })
        .collect();
    let t = &r.trend;
    StackOut {
        axes: AxesOut { u: xyz(f.u), v: xyz(f.v), normal: xyz(f.normal) },
        levels,
        trend: TrendOut {
            profile: t.profile.to_string(),
            prismatic: t.profile == "prismatic",
            sides: t
                .sides
                .iter()
                .map(|s| SideOut { side: s.side.to_string(), draft_deg: s.draft_deg, rms: s.rms })
                .collect(),
            radius: t.radius.as_ref().map(|rt| RadiusOut {
                taper_deg: rt.taper_deg,
                slope: rt.fit.slope,
                radius_at_zero: rt.fit.intercept,
                rms: rt.fit.rms,
                levels: rt.levels,
            }),
        },
    }
}

fn stack_cmd(s: &mut Session, i: StackIn) -> CmdResult<StackOut> {
    let mesh = mesh_of(s, &i.mesh)?;
    let p = planes(i.origin, i.normal, i.start, i.step, i.count)?;
    let report = section::stack(&mesh, &p.frame, p.start, p.step, p.count);
    Ok(stack_out(&report))
}

fn sheet_cmd(s: &mut Session, i: SheetIn) -> CmdResult<SheetOut> {
    let mesh = mesh_of(s, &i.mesh)?;
    let p = planes(i.origin, i.normal, i.start, i.step, i.count)?;
    let tol = i.tolerance.unwrap_or(DEFAULT_TOLERANCE);
    if !(tol.is_finite() && tol >= 0.0) {
        return Err(CmdError::Params("tolerance must be zero or positive (mm)".into()));
    }
    let mut layers = Vec::with_capacity(p.count);
    let mut chains = 0usize;
    for k in 0..p.count {
        let offset = p.start + p.step * k as f64;
        let polys = section::section(&mesh, &p.frame.at(offset), &p.frame.normal);
        let paths: Vec<DrawPath> = polys
            .iter()
            .map(|pl| {
                let pts: Vec<[f64; 2]> = pl.points.iter().map(|q| p.frame.to_2d(q)).collect();
                DrawPath { points: section::simplify(&pts, pl.closed, tol), closed: pl.closed }
            })
            .filter(|d| d.points.len() >= 2)
            .collect();
        chains += paths.len();
        layers.push(DrawLayer { name: format!("LEVEL_{k:03}_{offset:.3}"), paths });
    }
    let text = match i.format {
        SheetFormat::Dxf => surfacecraft_io::dxf::write_dxf(&layers)?,
        SheetFormat::Svg => surfacecraft_io::svg::write_svg(&layers)?,
    };
    let bytes = text.len();
    match i.path {
        Some(path) => {
            write_file(s, &path, text.as_bytes())?;
            Ok(SheetOut { format: i.format, levels: p.count, chains, bytes, path: Some(path), text: None })
        }
        None => Ok(SheetOut { format: i.format, levels: p.count, chains, bytes, path: None, text: Some(text) }),
    }
}

/// Sections and drawings of meshes.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "export.section_sheet",
        "Export section sheet…",
        "File",
        false,
        "Writes the chains of `mesh` cut by `count` parallel planes (origin, normal, start, step; count 1 is one section) as a 2-D drawing in plane coordinates, mm: `format` dxf (LINE/LWPOLYLINE, one layer per level) or svg (one path per chain), to `path` or as `text`.",
        SheetIn,
        SheetOut,
        sheet_cmd
    ),
    command!(
        "section.stack",
        "Section stack…",
        "Mesh",
        false,
        "Cuts `mesh` with `count` parallel planes (origin, normal, start, step). Per level: closed and open chains, perimeter, enclosed area, centroid, size in the plane and fitted circles of near-circular chains. Over the stack: draft angle per side by least squares, taper of the fitted radii, and whether the profile is constant (prismatic).",
        StackIn,
        StackOut,
        stack_cmd
    ),
];
