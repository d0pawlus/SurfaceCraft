//! `sketch.*`: 2-D sketches with constraints, and sketches from mesh sections.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::{Point3, Vec3, section::section};
use surfacecraft_sketch::{
    Constraint, Entity, Sketch, SketchPlane, SolveReport,
    chains::{Chain2, ChainOptions, ChainReport, fit_chains},
    fit2d::fit_polyline,
    solve,
};

/// A sketch as reported.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SketchOut {
    /// Sketch id.
    pub id: ObjectId,
    /// Points.
    pub points: usize,
    /// Entities.
    pub entities: usize,
    /// Constraints.
    pub constraints: usize,
    /// Closed profiles found.
    pub profiles: usize,
    /// Solve result.
    pub report: Option<SolveReport>,
    /// Features that failed when the history was replayed.
    pub failed_features: usize,
}

fn out(s: &Session, id: ObjectId, failed: usize) -> CmdResult<SketchOut> {
    let o = s.doc.sketch(id).ok_or_else(|| CmdError::Params(format!("no sketch {id}")))?;
    Ok(SketchOut {
        id,
        points: o.sketch.points.len(),
        entities: o.sketch.entities.len(),
        constraints: o.sketch.constraints.len(),
        profiles: surfacecraft_sketch::profiles(&o.sketch, 1e-6).len(),
        report: o.report.clone(),
        failed_features: failed,
    })
}

/// Solves a sketch, stores the report and replays the history that depends on it.
pub(crate) fn resolve(s: &mut Session, id: ObjectId) -> CmdResult<SketchOut> {
    let o = s.doc.sketch_mut(id).ok_or_else(|| CmdError::Params(format!("no sketch {id}")))?;
    let mut sk = o.sketch.clone();
    let rep = solve(&mut sk).map_err(|e| CmdError::Params(e.to_string()))?;
    o.sketch = sk;
    o.report = Some(rep);
    let failed = crate::replay::replay(s);
    out(s, id, failed)
}

/// A plane given by origin and normal (x chosen), or origin and both axes.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PlaneIn {
    /// Origin.
    pub origin: [f64; 3],
    /// Normal (when no axes).
    #[serde(default)]
    pub normal: Option<[f64; 3]>,
    /// X axis (with `y`).
    #[serde(default)]
    pub x: Option<[f64; 3]>,
    /// Y axis (with `x`).
    #[serde(default)]
    pub y: Option<[f64; 3]>,
}

fn plane(p: &PlaneIn) -> CmdResult<SketchPlane> {
    let pl = match (p.normal, p.x, p.y) {
        (Some(n), None, None) => SketchPlane::from_normal(p.origin, n),
        (None, Some(x), Some(y)) => {
            let (x, y) = (Vec3::from(x), Vec3::from(y));
            let xn = surfacecraft_geom::normalize(x);
            let yn = xn.and_then(|xn| surfacecraft_geom::normalize(y - xn * y.dot(&xn)));
            xn.zip(yn).map(|(a, b)| SketchPlane { origin: p.origin, x: a.into(), y: b.into() })
        }
        _ => None,
    };
    pl.filter(SketchPlane::is_valid)
        .ok_or_else(|| CmdError::Params("plane needs `normal`, or `x` and `y`, finite and non-degenerate".into()))
}

/// Create parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CreateIn {
    /// Sketch plane (default XY).
    #[serde(default)]
    pub plane: Option<PlaneIn>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn create(s: &mut Session, i: CreateIn) -> CmdResult<SketchOut> {
    let pl = match &i.plane {
        Some(p) => plane(p)?,
        None => SketchPlane::default(),
    };
    let id = s.doc.add_sketch(i.name.as_deref().unwrap_or("sketch"), Sketch { plane: pl, ..Sketch::default() });
    out(s, id, 0)
}

/// An entity to add, in sketch coordinates.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case", deny_unknown_fields)]
pub enum NewEntity {
    /// Line between two points.
    Line {
        /// Start.
        from: [f64; 2],
        /// End.
        to: [f64; 2],
    },
    /// Arc counter-clockwise from `start` to `end` around `center`.
    Arc {
        /// Centre.
        center: [f64; 2],
        /// Start point.
        start: [f64; 2],
        /// End point.
        end: [f64; 2],
    },
    /// Circle.
    Circle {
        /// Centre.
        center: [f64; 2],
        /// Radius.
        radius: f64,
    },
    /// Closed polygon through the points.
    Polygon {
        /// Corners.
        points: Vec<[f64; 2]>,
    },
}

/// Add parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AddIn {
    /// Sketch id or name.
    pub sketch: ObjRef,
    /// Entities; end points equal to existing points are shared.
    pub entities: Vec<NewEntity>,
}

fn point_at(sk: &mut Sketch, p: [f64; 2]) -> usize {
    if let Some(i) = sk.points.iter().position(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 1e-9) {
        return i;
    }
    sk.add_point(p)
}

fn add(s: &mut Session, i: AddIn) -> CmdResult<SketchOut> {
    let id = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let o = s.doc.sketch_mut(id).ok_or_else(|| CmdError::Params(format!("no sketch {id}")))?;
    if o.sketch.points.len() + i.entities.len() * 4 > surfacecraft_sketch::MAX_ITEMS {
        return Err(CmdError::Params("the sketch would be too large".into()));
    }
    for e in &i.entities {
        let pts: Vec<[f64; 2]> = match e {
            NewEntity::Line { from, to } => vec![*from, *to],
            NewEntity::Arc { center, start, end } => vec![*center, *start, *end],
            NewEntity::Circle { center, .. } => vec![*center],
            NewEntity::Polygon { points } => points.clone(),
        };
        if pts.iter().flatten().any(|v| !v.is_finite()) {
            return Err(CmdError::Params("coordinates must be finite".into()));
        }
        match e {
            NewEntity::Line { from, to } => {
                let (a, b) = (point_at(&mut o.sketch, *from), point_at(&mut o.sketch, *to));
                if a == b {
                    return Err(CmdError::Params("a line needs two different points".into()));
                }
                o.sketch.entities.push(Entity::Line { a, b });
            }
            NewEntity::Arc { center, start, end } => {
                let (c, a, b) =
                    (point_at(&mut o.sketch, *center), point_at(&mut o.sketch, *start), point_at(&mut o.sketch, *end));
                o.sketch.entities.push(Entity::Arc { center: c, a, b });
            }
            NewEntity::Circle { center, radius } => {
                if !(radius.is_finite() && *radius > 0.0) {
                    return Err(CmdError::Params("radius must be positive".into()));
                }
                let c = point_at(&mut o.sketch, *center);
                o.sketch.entities.push(Entity::Circle { center: c, radius: *radius });
            }
            NewEntity::Polygon { points } => {
                if points.len() < 3 {
                    return Err(CmdError::Params("a polygon needs 3 points".into()));
                }
                let ids: Vec<usize> = points.iter().map(|p| point_at(&mut o.sketch, *p)).collect();
                for k in 0..ids.len() {
                    let (a, b) = (ids[k], ids[(k + 1) % ids.len()]);
                    if a != b {
                        o.sketch.entities.push(Entity::Line { a, b });
                    }
                }
            }
        }
    }
    resolve(s, id)
}

/// Constrain parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ConstrainIn {
    /// Sketch id or name.
    pub sketch: ObjRef,
    /// Constraints to add (indices refer to the sketch's points and entities).
    pub constraints: Vec<Constraint>,
}

fn constrain(s: &mut Session, i: ConstrainIn) -> CmdResult<SketchOut> {
    let id = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let o = s.doc.sketch_mut(id).ok_or_else(|| CmdError::Params(format!("no sketch {id}")))?;
    o.sketch.constraints.extend(i.constraints);
    // validate before touching the history (the engine rolls back on error)
    o.sketch.validate().map_err(|e| CmdError::Params(e.to_string()))?;
    resolve(s, id)
}

/// Edit-constraint parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct EditIn {
    /// Sketch id or name.
    pub sketch: ObjRef,
    /// Constraint index.
    pub index: usize,
    /// New constraint (omitted → the constraint is removed).
    #[serde(default)]
    pub constraint: Option<Constraint>,
}

fn edit(s: &mut Session, i: EditIn) -> CmdResult<SketchOut> {
    let id = i.sketch.resolve(s, ObjectKind::Sketch)?;
    let o = s.doc.sketch_mut(id).ok_or_else(|| CmdError::Params(format!("no sketch {id}")))?;
    if i.index >= o.sketch.constraints.len() {
        return Err(CmdError::Params(format!("constraint {} does not exist", i.index)));
    }
    match i.constraint {
        Some(c) => {
            if let Some(slot) = o.sketch.constraints.get_mut(i.index) {
                *slot = c;
            }
        }
        None => {
            o.sketch.constraints.remove(i.index);
        }
    }
    o.sketch.validate().map_err(|e| CmdError::Params(e.to_string()))?;
    resolve(s, id)
}

/// One sketch.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SketchIn {
    /// Sketch id or name.
    pub sketch: ObjRef,
}

fn solve_cmd(s: &mut Session, i: SketchIn) -> CmdResult<SketchOut> {
    let id = i.sketch.resolve(s, ObjectKind::Sketch)?;
    resolve(s, id)
}

/// Section parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SectionIn {
    /// Mesh.
    pub mesh: ObjRef,
    /// Section plane origin.
    pub origin: [f64; 3],
    /// Section plane normal.
    pub normal: [f64; 3],
    /// Fit tolerance of lines/arcs (mm, default 0.05).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

fn from_section(s: &mut Session, i: SectionIn) -> CmdResult<SketchOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let pl = SketchPlane::from_normal(i.origin, i.normal)
        .filter(SketchPlane::is_valid)
        .ok_or_else(|| CmdError::Params("section plane must be finite with a non-zero normal".into()))?;
    let tol = i.tolerance.unwrap_or(0.05);
    if !(tol.is_finite() && tol > 0.0) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    let polys = section(&m, &Point3::from(i.origin), &Vec3::from(i.normal));
    // the longest closed section (or longest open one)
    let len = |p: &surfacecraft_geom::section::Polyline| p.points.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>();
    let best = polys
        .iter()
        .max_by(|a, b| (a.closed, len(a)).partial_cmp(&(b.closed, len(b))).unwrap_or(std::cmp::Ordering::Equal))
        .ok_or_else(|| CmdError::Failed("the plane does not cut the mesh".into()))?;
    let pts2: Vec<[f64; 2]> = best.points.iter().map(|p| pl.to_2d(p)).collect();
    let sk = fit_polyline(pl, &pts2, best.closed, tol);
    if sk.entities.is_empty() {
        return Err(CmdError::Failed("no lines or arcs could be fitted to the section".into()));
    }
    let id = s.doc.add_sketch(i.name.as_deref().unwrap_or("section"), sk);
    resolve(s, id)
}

/// Chain-fit parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FitChainsIn {
    /// Mesh.
    pub mesh: ObjRef,
    /// Section plane origin.
    pub origin: [f64; 3],
    /// Section plane normal.
    pub normal: [f64; 3],
    /// Fit tolerance (mm): RMS bound of a line or arc and of the chain after its relations (the
    /// maximum may be three times as large). Default `max(0.05, 2 × noise_sigma)`.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Noise of the scan (mm, default 0), from which the default tolerance follows.
    #[serde(default)]
    pub noise_sigma: Option<f64>,
    /// Angle within which lines count as horizontal, vertical, parallel or perpendicular, and
    /// three times within which a junction counts as tangent (degrees, default 1).
    #[serde(default)]
    pub angle_tolerance_degrees: Option<f64>,
    /// Turn the sketch frame to the dominant line direction when the lines lie within 5 degrees of
    /// the axes (default true), so a part scanned a little off the plane axes still gets
    /// horizontal and vertical lines.
    #[serde(default)]
    pub align_frame: Option<bool>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// Most section chains fitted in one call (the longest, closed ones first).
const MAX_CHAINS: usize = 64;

/// Chain-fit result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FitChainsOut {
    /// The sketch.
    pub sketch: SketchOut,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// Angle tolerance used (degrees).
    pub angle_tolerance_degrees: f64,
    /// How far the sketch frame was turned to the part's axes (degrees).
    pub frame_rotation_degrees: f64,
    /// Section chains beyond the 64 fitted.
    pub chains_not_fitted: usize,
    /// One report per fitted chain (closed first, longest first): entities, relations kept and
    /// refused with reasons, deviation before and after the relations, solve status.
    pub chains: Vec<ChainReport>,
}

fn fit_chains_cmd(s: &mut Session, i: FitChainsIn) -> CmdResult<FitChainsOut> {
    let mesh = i.mesh.resolve(s, ObjectKind::Mesh)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let pl = SketchPlane::from_normal(i.origin, i.normal)
        .filter(SketchPlane::is_valid)
        .ok_or_else(|| CmdError::Params("section plane must be finite with a non-zero normal".into()))?;
    let sigma = i.noise_sigma.unwrap_or(0.0);
    if !(sigma.is_finite() && (0.0..=1e6).contains(&sigma)) {
        return Err(CmdError::Params("noise_sigma must be finite and not negative".into()));
    }
    let tol = i.tolerance.unwrap_or_else(|| ChainOptions::tolerance_for_noise(sigma));
    if !(tol.is_finite() && tol > 0.0 && tol <= 1e6) {
        return Err(CmdError::Params("tolerance must be positive".into()));
    }
    let angle = i.angle_tolerance_degrees.unwrap_or(1.0);
    if !(angle.is_finite() && angle > 0.0 && angle <= 20.0) {
        return Err(CmdError::Params("angle_tolerance_degrees must be in (0, 20]".into()));
    }
    let polys = section(&m, &Point3::from(i.origin), &Vec3::from(i.normal));
    if polys.is_empty() {
        return Err(CmdError::Failed("the plane does not cut the mesh".into()));
    }
    let len = |p: &surfacecraft_geom::section::Polyline| p.points.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>();
    let mut order: Vec<(bool, f64, usize)> = polys.iter().enumerate().map(|(k, p)| (p.closed, len(p), k)).collect();
    order.sort_by(|a, b| b.0.cmp(&a.0).then(b.1.total_cmp(&a.1)));
    let chains_not_fitted = order.len().saturating_sub(MAX_CHAINS);
    let chains: Vec<Chain2> = order
        .iter()
        .take(MAX_CHAINS)
        .filter_map(|(closed, _, k)| {
            polys.get(*k).map(|p| Chain2 { points: p.points.iter().map(|q| pl.to_2d(q)).collect(), closed: *closed })
        })
        .collect();
    let opts =
        ChainOptions { tolerance: tol, angle_tolerance_degrees: angle, align_frame: i.align_frame.unwrap_or(true) };
    let fit = fit_chains(&pl, &chains, &opts);
    if fit.sketch.entities.is_empty() {
        let why = fit.chains.iter().find_map(|c| c.note.clone()).unwrap_or_default();
        return Err(CmdError::Failed(format!("no lines or arcs could be fitted to the section: {why}")));
    }
    let frame = fit.frame_rotation_degrees;
    let id = s.doc.add_sketch(i.name.as_deref().unwrap_or("section"), fit.sketch);
    if let Some(o) = s.doc.sketch_mut(id) {
        o.report = Some(fit.report);
    }
    let failed = crate::replay::replay(s);
    Ok(FitChainsOut {
        sketch: out(s, id, failed)?,
        tolerance: tol,
        angle_tolerance_degrees: angle,
        frame_rotation_degrees: frame,
        chains_not_fitted,
        chains: fit.chains,
    })
}

/// `sketch.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "sketch.create",
        "New sketch",
        "Sketch",
        true,
        "Empty sketch on `plane` (default XY).",
        CreateIn,
        SketchOut,
        create
    ),
    command!(
        "sketch.add",
        "Add sketch entities",
        "Sketch",
        true,
        "Adds lines/arcs/circles/polygons (sketch coordinates) to `sketch`, solves, replays dependent features.",
        AddIn,
        SketchOut,
        add
    ),
    command!(
        "sketch.constrain",
        "Constrain",
        "Sketch",
        true,
        "Adds constraints to `sketch` and solves it (reports degrees of freedom and over-definition).",
        ConstrainIn,
        SketchOut,
        constrain
    ),
    command!(
        "sketch.edit_constraint",
        "Edit constraint",
        "Sketch",
        true,
        "Replaces (or removes) constraint `index` of `sketch`, solves and replays the history.",
        EditIn,
        SketchOut,
        edit
    ),
    command!(
        "sketch.solve",
        "Solve sketch",
        "Sketch",
        true,
        "Solves `sketch` and replays dependent features.",
        SketchIn,
        SketchOut,
        solve_cmd
    ),
    command!(
        "sketch.fit_chains",
        "Fit section chains",
        "Sketch",
        true,
        "Cuts `mesh` with a plane and fits every section chain into connected lines and arcs (split and merge within `tolerance`, closed chains stay closed), then adds the relations that hold within the tolerance (horizontal, vertical, tangent, concentric, collinear, equal radius, parallel, perpendicular) without over-defining the sketch; reports per chain the relations kept and refused and the deviation before and after.",
        FitChainsIn,
        FitChainsOut,
        fit_chains_cmd
    ),
    command!(
        "sketch.from_section",
        "Sketch from section",
        "Sketch",
        true,
        "Cuts `mesh` with a plane and fits lines and arcs (within `tolerance`) into a constrained sketch.",
        SectionIn,
        SketchOut,
        from_section
    ),
];
