//! Local cutting tools of the hybrid model (plan P3b, Z2): the solids a blind hole, a pocket or a
//! through slot take out of the stock.
//!
//! The global cuts of [`crate::hybrid::solid_from_cuts`] (a plane as a half space, a full
//! revolution as an intersection) assume every recognised surface lies on the outer envelope of
//! the part. A pocket floor, a hole bottom or a slot wall does not: it is a *local* surface that
//! bounds a tool, and the tool is subtracted from the stock. The tools are built here with the
//! kernel's own `revolve` and `extrude`; the planner (the engine) decides which regions form one.
use crate::feature::ProfileCurve;
use crate::hybrid::{V, add, corners, cross, dot, norm, perpendicular, scale, sub, unit};
use crate::record::brep::{Brep, Surface};
use crate::solid::{decode, encode};
use crate::{KernelError, Result, finite_all, op_err, solid};
use solidcore_ops as ops;

/// The bottom of a blind hole.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum HoleBottom {
    /// A flat bottom perpendicular to the axis, at the tool's `base`.
    Flat,
    /// A conical (drill point) bottom: the cone's apex is the tool's `base`, `half_angle` in
    /// (0, π/2) (59° for a 118° drill).
    Cone {
        /// Half angle, radians.
        half_angle: f64,
    },
}

/// A solid to subtract from the stock.
#[derive(Clone, Debug, PartialEq)]
pub enum LocalTool {
    /// A blind hole: a cylinder of `radius` about the axis through `base` along unit `axis`
    /// (pointing out of the material, towards the opening), from the bottom to past the stock.
    BlindHole {
        /// The flat bottom's centre, or the cone's apex.
        base: [f64; 3],
        /// Unit axis towards the opening.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
        /// The bottom.
        bottom: HoleBottom,
    },
    /// A prismatic pocket or slot: a closed profile in a plane perpendicular to `direction`,
    /// extruded along it. A pocket starts at the profile's plane (its floor) and runs out of the
    /// stock; a *through* tool runs across the whole stock.
    Prism {
        /// The closed loop (consecutive curves joined end to start).
        profile: Vec<ProfileCurve>,
        /// Unit direction out of the material (the pocket's opening).
        direction: [f64; 3],
        /// Cut through the whole stock instead of starting at the profile's plane.
        through: bool,
        /// Radius of the fillet between the floor and the walls, if any.
        floor_fillet: Option<f64>,
        /// Closed loops in the profile's plane (same winding rules as `profile`) that stay as
        /// material inside the tool: the islands of a pocket, or the ribs and posts of a face that
        /// is cut down everywhere else. Empty for a through tool.
        islands: Vec<Vec<ProfileCurve>>,
    },
}

/// Largest coordinate or radius of a tool's profile (mm): anything beyond is not a part.
const MAX_PROFILE_EXTENT: f64 = 1e7;

/// A curve with finite coordinates in range and, for an arc, a positive radius.
fn curve_is_sound(c: &ProfileCurve) -> bool {
    let sound = |p: &[f64; 3]| p.iter().all(|v| v.is_finite() && v.abs() <= MAX_PROFILE_EXTENT);
    match c {
        ProfileCurve::Line { start, end } => sound(start) && sound(end),
        ProfileCurve::Arc { center, axis, reference, radius, start, end } => {
            sound(center)
                && finite_all(axis)
                && finite_all(reference)
                && radius.is_finite()
                && *radius > 0.0
                && *radius <= MAX_PROFILE_EXTENT
                && start.is_finite()
                && end.is_finite()
        }
    }
}

fn check_tool(t: &LocalTool) -> Result<()> {
    let ok = match t {
        LocalTool::BlindHole { base, axis, radius, bottom } => {
            finite_all(base)
                && finite_all(axis)
                && unit(*axis).is_some()
                && radius.is_finite()
                && *radius > 0.0
                && match bottom {
                    HoleBottom::Flat => true,
                    HoleBottom::Cone { half_angle } => {
                        half_angle.is_finite() && *half_angle > 1e-3 && *half_angle < std::f64::consts::FRAC_PI_2 - 1e-3
                    }
                }
        }
        LocalTool::Prism { profile, direction, floor_fillet, through, islands } => {
            !profile.is_empty()
                && profile.iter().all(curve_is_sound)
                && islands.iter().all(|l| l.iter().all(curve_is_sound))
                && finite_all(direction)
                && unit(*direction).is_some()
                && floor_fillet.is_none_or(|r| r.is_finite() && r > 0.0)
                && (!*through || islands.is_empty())
                && islands.iter().all(|l| !l.is_empty())
        }
    };
    if ok { Ok(()) } else { Err(KernelError::Argument("local tool parameters must be finite and in range".into())) }
}

fn profile_points(c: &ProfileCurve) -> Vec<V> {
    match c {
        ProfileCurve::Line { start, end } => vec![*start, *end],
        ProfileCurve::Arc { center, axis, reference, radius, start, end } => {
            let a = unit(*axis).unwrap_or([0.0, 0.0, 1.0]);
            let r = unit(*reference).unwrap_or_else(|| perpendicular(a));
            let t = cross(a, r);
            [*start, 0.5 * (start + end), *end]
                .iter()
                .map(|&th| add(*center, add(scale(r, radius * th.cos()), scale(t, radius * th.sin()))))
                .collect()
        }
    }
}

fn shift_curve(c: &ProfileCurve, by: V) -> ProfileCurve {
    match c {
        ProfileCurve::Line { start, end } => ProfileCurve::Line { start: add(*start, by), end: add(*end, by) },
        ProfileCurve::Arc { center, axis, reference, radius, start, end } => ProfileCurve::Arc {
            center: add(*center, by),
            axis: *axis,
            reference: *reference,
            radius: *radius,
            start: *start,
            end: *end,
        },
    }
}

fn curves(l: &[ProfileCurve]) -> Vec<ops::sweep::Curve> {
    l.iter()
        .map(|c| match c {
            ProfileCurve::Line { start, end } => ops::sweep::Curve::Line { start: *start, end: *end },
            ProfileCurve::Arc { center, axis, reference, radius, start, end } => ops::sweep::Curve::Arc {
                center: *center,
                axis: *axis,
                reference: *reference,
                radius: *radius,
                start: *start,
                end: *end,
            },
        })
        .collect()
}

/// The tool as a solid, sized to run out of the stock `lo`..`hi` on the open side.
fn tool_solid(t: &LocalTool, lo: V, hi: V) -> Result<ops::Body> {
    let cs = corners(lo, hi);
    let margin = 0.1 * norm(sub(hi, lo)) + 1.0;
    match t {
        LocalTool::BlindHole { base, axis, radius, bottom } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            let e = perpendicular(a);
            let top = cs.iter().map(|c| dot(sub(*c, *base), a)).fold(f64::NEG_INFINITY, f64::max) + margin;
            let at = |rho: f64, s: f64| add(*base, add(scale(e, rho), scale(a, s)));
            let line = |p: V, q: V| ops::sweep::Curve::Line { start: p, end: q };
            let mut prof = Vec::new();
            match bottom {
                HoleBottom::Flat => {
                    prof.push(line(at(0.0, 0.0), at(*radius, 0.0)));
                    prof.push(line(at(*radius, 0.0), at(*radius, top)));
                }
                HoleBottom::Cone { half_angle } => {
                    let rim = radius / half_angle.tan();
                    if top <= rim {
                        return Err(KernelError::Argument("the hole's cone reaches out of the stock".into()));
                    }
                    prof.push(line(at(0.0, 0.0), at(*radius, rim)));
                    prof.push(line(at(*radius, rim), at(*radius, top)));
                }
            }
            prof.push(line(at(*radius, top), at(0.0, top)));
            prof.push(line(at(0.0, top), at(0.0, 0.0)));
            ops::sweep::revolve(&prof, *base, a, std::f64::consts::TAU, true).map_err(|e| op_err("sweep::revolve", e))
        }
        LocalTool::Prism { profile, direction, through, islands, .. } => {
            let d = unit(*direction).ok_or_else(|| KernelError::Argument("direction is zero".into()))?;
            let anchor = profile.iter().flat_map(profile_points).next().unwrap_or([0.0; 3]);
            let off = |c: &V| dot(sub(*c, anchor), d);
            let top = cs.iter().map(off).fold(f64::NEG_INFINITY, f64::max) + margin;
            if *through {
                let bottom = cs.iter().map(off).fold(f64::INFINITY, f64::min) - margin;
                let moved: Vec<ProfileCurve> = profile.iter().map(|c| shift_curve(c, scale(d, bottom))).collect();
                return ops::sweep::extrude(&curves(&moved), d, top - bottom, 0.0, Default::default(), true)
                    .map_err(|e| op_err("sweep::extrude", e));
            }
            if top <= 0.0 {
                return Err(KernelError::Argument("the pocket's floor lies above the stock".into()));
            }
            let outer = ops::sweep::extrude(&curves(profile), d, top, 0.0, Default::default(), true)
                .map_err(|e| op_err("sweep::extrude", e))?;
            islands.iter().try_fold(outer, |tool, island| subtract_island(tool, island, d, top))
        }
    }
}

/// How far an island's prism starts below the floor and ends above the tool's top, so that its
/// faces are never coplanar with the tool's bottom or top.
const ISLAND_OVERSHOOT: f64 = 1.0;

/// The pocket tool with one island left standing: the island's prism is taken out of it.
fn subtract_island(tool: ops::Body, island: &[ProfileCurve], d: V, top: f64) -> Result<ops::Body> {
    let moved: Vec<ProfileCurve> = island.iter().map(|c| shift_curve(c, scale(d, -ISLAND_OVERSHOOT))).collect();
    let prism = ops::sweep::extrude(&curves(&moved), d, top + 2.0 * ISLAND_OVERSHOOT, 0.0, Default::default(), true)
        .map_err(|e| op_err("sweep::extrude (island)", e))?;
    ops::boolean::boolean(tool, prism, ops::BooleanOp::Subtract)
        .map_err(|e| op_err("boolean (island)", e))?
        .ok_or_else(|| KernelError::Op { op: "boolean (island)", message: "the island removed the whole tool".into() })
}

/// Edges (record indices) of the faces of `rec` that lie in the plane through `point` with
/// normal along `dir` and whose vertices all lie within `bounds` (min, max, padded).
fn floor_edges(rec: &Brep, point: V, dir: V, lo: V, hi: V) -> Vec<usize> {
    let inside = |p: V| (0..3).all(|i| p[i] >= lo[i] && p[i] <= hi[i]);
    let mut out: Vec<usize> = Vec::new();
    for f in &rec.faces {
        let Surface::Plane { origin, u_axis, v_axis } = &f.surface else { continue };
        let Some(n) = unit(cross(*u_axis, *v_axis)) else { continue };
        if dot(n, dir).abs() < 0.9999 || dot(sub(*origin, point), n).abs() > 1e-4 {
            continue;
        }
        let mut edges = Vec::new();
        for l in &f.loops {
            if let crate::record::brep::Loop::Edges { edges: oes } = l {
                edges.extend(oes.iter().map(|oe| oe.edge as usize));
            }
        }
        let all_in = edges.iter().all(|&e| {
            rec.edges.get(e).is_some_and(|ed| {
                ed.vertices.iter().all(|&v| rec.vertices.get(v as usize).is_some_and(|v| inside(v.point)))
            })
        });
        if all_in {
            out.extend(edges);
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// The outcome of applying one tool.
pub(crate) enum Applied {
    /// Subtracted (the text says how).
    Done(&'static str),
    /// Subtracted, but part of it could not be honoured (the reason).
    Partly(String),
}

/// Subtracts `tool` from `body` (the stock `lo`..`hi` bounds the tool's reach). On failure the
/// body is left as it was.
pub(crate) fn subtract_tool(body: ops::Body, tool: &LocalTool, lo: V, hi: V) -> (ops::Body, Result<Applied>) {
    let attempt = |body: &ops::Body| -> Result<ops::Body> {
        check_tool(tool)?;
        let t = tool_solid(tool, lo, hi)?;
        let copy = body.copy().map_err(|e| op_err("copy", e))?;
        ops::boolean::boolean(copy, t, ops::BooleanOp::Subtract)
            .map_err(|e| op_err("boolean", format!("{e} (subtract {tool:?})")))?
            .ok_or_else(|| KernelError::Op { op: "boolean", message: "the tool removed everything".into() })
    };
    let cut = match attempt(&body) {
        Ok(b) => b,
        Err(e) => return (body, Err(e)),
    };
    // the floor fillet is one more kernel call on the pocket's floor edges
    if let LocalTool::Prism { profile, direction, through: false, floor_fillet: Some(r), .. } = tool {
        let filleted = (|| -> Result<ops::Body> {
            let d = unit(*direction).ok_or_else(|| KernelError::Argument("direction is zero".into()))?;
            let pts: Vec<V> = profile.iter().flat_map(profile_points).collect();
            let anchor = *pts.first().ok_or_else(|| KernelError::Argument("empty profile".into()))?;
            let pad = 1e-3;
            let mut blo = [f64::INFINITY; 3];
            let mut bhi = [f64::NEG_INFINITY; 3];
            for p in &pts {
                for i in 0..3 {
                    blo[i] = blo[i].min(p[i]);
                    bhi[i] = bhi[i].max(p[i]);
                }
            }
            // arcs bulge past their sampled points by less than the radius; the profile's own
            // extent bounds the floor face to within the pocket
            let grow = norm(sub(bhi, blo)) * 0.05 + pad;
            let rec = encode(&cut)?;
            let edges = floor_edges(&rec, anchor, d, sub(blo, [grow; 3]), add(bhi, [grow; 3]));
            if edges.is_empty() {
                return Err(KernelError::Op { op: "floor fillet", message: "the pocket's floor was not found".into() });
            }
            decode(&solid::fillet(&rec, &edges, *r)?)
        })();
        return match filleted {
            Ok(b) => (b, Ok(Applied::Done("subtract+fillet"))),
            Err(e) => (cut, Ok(Applied::Partly(format!("the floor fillet r = {r:.2} failed: {e}")))),
        };
    }
    let how = match tool {
        LocalTool::BlindHole { .. } => "subtract (blind hole)",
        LocalTool::Prism { through: true, .. } => "subtract (through slot)",
        LocalTool::Prism { .. } => "subtract (pocket)",
    };
    (cut, Ok(Applied::Done(how)))
}

/// A solid to unite with the stock (plan P3b, Z3). A chamfer or round on a boss's top edge is a
/// blend of the finished boss, not part of this tool.
#[derive(Clone, Debug, PartialEq)]
pub enum LocalAdd {
    /// A cylindrical boss standing on a plane: the cylinder of `radius` about the axis through
    /// `base` (a point of the base plane) along unit `axis` (out of the material), `height` tall,
    /// sunk `embed` below the base plane so that it fuses with the body.
    Boss {
        /// A point of the axis in the base plane.
        base: [f64; 3],
        /// Unit axis, pointing away from the body.
        axis: [f64; 3],
        /// Radius.
        radius: f64,
        /// Height from the base plane to the top.
        height: f64,
        /// How far the tool is sunk into the body.
        embed: f64,
    },
    /// A dome: the part of the ball of `radius` about `center` that lies on the side of the plane
    /// (through `plane_point`, unit `normal` pointing out of the body) the normal points to.
    Dome {
        /// Centre of the ball.
        center: [f64; 3],
        /// Radius of the ball.
        radius: f64,
        /// A point of the base plane.
        plane_point: [f64; 3],
        /// Unit normal of the base plane, out of the body.
        normal: [f64; 3],
    },
}

fn check_add(t: &LocalAdd) -> Result<()> {
    let ok = match t {
        LocalAdd::Boss { base, axis, radius, height, embed } => {
            finite_all(base)
                && finite_all(axis)
                && unit(*axis).is_some()
                && radius.is_finite()
                && *radius > 0.0
                && height.is_finite()
                && *height > 0.0
                && embed.is_finite()
                && *embed >= 0.0
        }
        LocalAdd::Dome { center, radius, plane_point, normal } => {
            finite_all(center)
                && finite_all(plane_point)
                && finite_all(normal)
                && unit(*normal).is_some()
                && radius.is_finite()
                && *radius > 0.0
        }
    };
    if ok { Ok(()) } else { Err(KernelError::Argument("local add parameters must be finite and in range".into())) }
}

/// The solid of an add, to be united with the body.
fn add_solid(t: &LocalAdd) -> Result<ops::Body> {
    match t {
        LocalAdd::Boss { base, axis, radius, height, embed } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            let b = ops::make_cylinder([0.0, 0.0, -embed], *radius, height + embed)
                .map_err(|e| op_err("make_cylinder", e))?;
            b.transform(&crate::to_kernel_matrix(&crate::hybrid::frame_matrix(*base, a)))
                .map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        LocalAdd::Dome { center, radius, plane_point, normal } => {
            let n = unit(*normal).ok_or_else(|| KernelError::Argument("normal is zero".into()))?;
            let ball = ops::make_sphere(*center, *radius).map_err(|e| op_err("make_sphere", e))?;
            // the ball cut at the base plane (sunk a little: the part below the plane is inside the
            // body), keeping the side the normal points to
            let sink = 0.5_f64.min(0.25 * radius);
            let p = sub(*plane_point, scale(n, sink));
            crate::hybrid::keep_front(&ball, p, n, 4.0 * radius + 1.0)
        }
    }
}

/// Unites `add` with `body`. On failure the body is left as it was.
pub(crate) fn unite_add(body: ops::Body, add: &LocalAdd) -> (ops::Body, Result<Applied>) {
    let attempt = |body: &ops::Body| -> Result<ops::Body> {
        check_add(add)?;
        let t = add_solid(add)?;
        let copy = body.copy().map_err(|e| op_err("copy", e))?;
        ops::boolean::boolean(copy, t, ops::BooleanOp::Union)
            .map_err(|e| op_err("boolean", format!("{e} (unite {add:?})")))?
            .ok_or_else(|| KernelError::Op { op: "boolean", message: "the union is empty".into() })
    };
    match attempt(&body) {
        Ok(b) => {
            let how = match add {
                LocalAdd::Boss { .. } => "unite (boss)",
                LocalAdd::Dome { .. } => "unite (dome)",
            };
            (b, Ok(Applied::Done(how)))
        }
        Err(e) => (body, Err(e)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::volume;

    fn stock() -> ops::Body {
        ops::make_box([0.0; 3], [60.0, 50.0, 20.0]).unwrap()
    }

    fn rounded_rect(x0: f64, y0: f64, x1: f64, y1: f64, r: f64, z: f64) -> Vec<ProfileCurve> {
        use std::f64::consts::{FRAC_PI_2, PI};
        let up = [0.0, 0.0, 1.0];
        let rx = [1.0, 0.0, 0.0];
        let arc = |cx: f64, cy: f64, a0: f64| ProfileCurve::Arc {
            center: [cx, cy, z],
            axis: up,
            reference: rx,
            radius: r,
            start: a0,
            end: a0 + FRAC_PI_2,
        };
        let line = |a: [f64; 2], b: [f64; 2]| ProfileCurve::Line { start: [a[0], a[1], z], end: [b[0], b[1], z] };
        vec![
            line([x0 + r, y0], [x1 - r, y0]),
            arc(x1 - r, y0 + r, -FRAC_PI_2),
            line([x1, y0 + r], [x1, y1 - r]),
            arc(x1 - r, y1 - r, 0.0),
            line([x1 - r, y1], [x0 + r, y1]),
            arc(x0 + r, y1 - r, FRAC_PI_2),
            line([x0, y1 - r], [x0, y0 + r]),
            arc(x0 + r, y0 + r, PI),
        ]
    }

    #[test]
    fn pocket_blind_holes_and_through_slot_volumes() {
        let (lo, hi) = ([0.0; 3], [60.0, 50.0, 20.0]);
        let pi = std::f64::consts::PI;
        // pocket 30 × 20 with r = 4 corners, floor at z = 12
        let pocket = LocalTool::Prism {
            profile: rounded_rect(8.0, 8.0, 38.0, 28.0, 4.0, 12.0),
            direction: [0.0, 0.0, 1.0],
            through: false,
            floor_fillet: None,
            islands: Vec::new(),
        };
        let (b, r) = subtract_tool(stock(), &pocket, lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done(_)));
        let area = 30.0 * 20.0 - (4.0 - pi) * 16.0;
        let v = volume(&encode(&b).unwrap()).unwrap();
        assert!((v - (60.0 * 50.0 * 20.0 - area * 8.0)).abs() < 1e-3, "pocket volume {v}");
        // flat and 118° blind holes
        let hole = |bottom, base: [f64; 3]| LocalTool::BlindHole { base, axis: [0.0, 0.0, 1.0], radius: 4.0, bottom };
        let (b2, r) =
            subtract_tool(b, &hole(HoleBottom::Cone { half_angle: 59f64.to_radians() }, [48.0, 38.0, 5.6]), lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done(_)));
        let rim = 4.0 / 59f64.to_radians().tan();
        let cyl = pi * 16.0 * (20.0 - 5.6 - rim);
        let cone = pi * 16.0 * rim / 3.0;
        let v2 = volume(&encode(&b2).unwrap()).unwrap();
        assert!((v2 - (v - cyl - cone)).abs() < 1e-3, "hole volume {v2}");
        let (b3, r) = subtract_tool(b2, &hole(HoleBottom::Flat, [10.0, 40.0, 10.0]), lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done(_)));
        let v3 = volume(&encode(&b3).unwrap()).unwrap();
        assert!((v3 - (v2 - pi * 16.0 * 10.0)).abs() < 1e-3, "flat hole volume {v3}");
        // through window with rounded corners, profile anywhere along the direction
        let window = LocalTool::Prism {
            profile: rounded_rect(20.0, 34.0, 34.0, 46.0, 3.0, 7.0),
            direction: [0.0, 0.0, 1.0],
            through: true,
            floor_fillet: None,
            islands: Vec::new(),
        };
        let (b4, r) = subtract_tool(b3, &window, lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done(_)));
        let wa = 14.0 * 12.0 - (4.0 - pi) * 9.0;
        let v4 = volume(&encode(&b4).unwrap()).unwrap();
        assert!((v4 - (v3 - 20.0 * wa)).abs() < 1e-3, "window volume {v4}");
        // bad tools are refused and leave the body alone
        let bad = LocalTool::BlindHole { base: [0.0; 3], axis: [0.0; 3], radius: 1.0, bottom: HoleBottom::Flat };
        let (b5, r) = subtract_tool(b4, &bad, lo, hi);
        assert!(r.is_err());
        assert!((volume(&encode(&b5).unwrap()).unwrap() - v4).abs() < 1e-9);
    }

    #[test]
    fn open_slot_with_a_far_closing_line_has_a_volume() {
        let (lo, hi) = ([0.0, 0.0, 0.0], [50.0, 40.0, 10.0]);
        let z = 5.0;
        let far = 150.0;
        let slot = LocalTool::Prism {
            profile: vec![
                ProfileCurve::Line { start: [far, 34.0, z], end: [40.0, 34.0, z] },
                ProfileCurve::Arc {
                    center: [40.0, 30.0, z],
                    axis: [0.0, 0.0, 1.0],
                    reference: [1.0, 0.0, 0.0],
                    radius: 4.0,
                    start: std::f64::consts::FRAC_PI_2,
                    end: 3.0 * std::f64::consts::FRAC_PI_2,
                },
                ProfileCurve::Line { start: [40.0, 26.0, z], end: [far, 26.0, z] },
                ProfileCurve::Line { start: [far, 26.0, z], end: [far, 34.0, z] },
            ],
            direction: [0.0, 0.0, 1.0],
            through: true,
            floor_fillet: None,
            islands: Vec::new(),
        };
        let plate = ops::make_box(lo, [50.0, 40.0, 10.0]).unwrap();
        let (b, r) = subtract_tool(plate, &slot, lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done(_)));
        let rec = encode(&b).unwrap();
        let v = volume(&rec).unwrap();
        let area = 10.0 * 8.0 + std::f64::consts::PI * 16.0 / 2.0;
        assert!((v - (20000.0 - area * 10.0)).abs() < 1e-3, "volume {v}");
    }

    #[test]
    fn tool_faces_map_back_to_their_shapes() {
        let (lo, hi) = ([0.0; 3], [60.0, 50.0, 20.0]);
        let pocket = LocalTool::Prism {
            profile: rounded_rect(8.0, 8.0, 38.0, 28.0, 4.0, 12.0),
            direction: [0.0, 0.0, 1.0],
            through: false,
            floor_fillet: None,
            islands: Vec::new(),
        };
        let (b, _) = subtract_tool(stock(), &pocket, lo, hi);
        let hole = LocalTool::BlindHole {
            base: [48.0, 38.0, 5.6],
            axis: [0.0, 0.0, 1.0],
            radius: 4.0,
            bottom: HoleBottom::Cone { half_angle: 59f64.to_radians() },
        };
        let (b, _) = subtract_tool(b, &hole, lo, hi);
        let rec = encode(&b).unwrap();
        // the sweep of an arc and the revolve come back as rational splines; they are found by
        // their corners
        use crate::hybrid::{Shape, faces_on_shape};
        let corner = Shape::Cylinder { point: [12.0, 12.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 4.0 };
        assert_eq!(faces_on_shape(&rec, &corner, 0.05).len(), 1);
        let wall = Shape::Cylinder { point: [48.0, 38.0, 0.0], axis: [0.0, 0.0, 1.0], radius: 4.0 };
        assert_eq!(faces_on_shape(&rec, &wall, 0.05).len(), 1);
        let apex = [48.0, 38.0, 5.6];
        let point = Shape::Cone { apex, axis: [0.0, 0.0, 1.0], half_angle: 59f64.to_radians() };
        assert_eq!(faces_on_shape(&rec, &point, 0.05).len(), 1);
        let floor = Shape::Plane { point: [0.0, 0.0, 12.0], normal: [0.0, 0.0, 1.0] };
        assert_eq!(faces_on_shape(&rec, &floor, 0.05).len(), 1);
    }

    #[test]
    fn floor_fillet_adds_material_in_the_corner() {
        let (lo, hi) = ([0.0; 3], [60.0, 50.0, 20.0]);
        let mk = |ff| LocalTool::Prism {
            profile: rounded_rect(8.0, 8.0, 38.0, 28.0, 4.0, 12.0),
            direction: [0.0, 0.0, 1.0],
            through: false,
            floor_fillet: ff,
            islands: Vec::new(),
        };
        let (plain, _) = subtract_tool(stock(), &mk(None), lo, hi);
        let (rounded, r) = subtract_tool(stock(), &mk(Some(1.0)), lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done("subtract+fillet")));
        let (vp, vr) = (volume(&encode(&plain).unwrap()).unwrap(), volume(&encode(&rounded).unwrap()).unwrap());
        // the fillet ring: perimeter × (1 − π/4) r²
        let pi = std::f64::consts::PI;
        let perimeter = 2.0 * (22.0 + 12.0) + 2.0 * pi * 4.0;
        let ring = perimeter * (1.0 - pi / 4.0);
        assert!(((vr - vp) - ring).abs() < 0.02 * ring, "fillet ring {} vs {ring}", vr - vp);
        assert_eq!(encode(&rounded).unwrap().faces.len(), encode(&plain).unwrap().faces.len() + 8);
    }

    const UP: [f64; 3] = [0.0, 0.0, 1.0];
    const RX: [f64; 3] = [1.0, 0.0, 0.0];

    fn line(a: [f64; 2], b: [f64; 2], z: f64) -> ProfileCurve {
        ProfileCurve::Line { start: [a[0], a[1], z], end: [b[0], b[1], z] }
    }

    /// An arc counter-clockwise about +z (`sweep` > 0) or clockwise (`sweep` < 0, about the
    /// reversed axis, as the planner writes it), from `from` (radians about the centre).
    fn arc(c: [f64; 2], r: f64, from: f64, sweep: f64, z: f64) -> ProfileCurve {
        let center = [c[0], c[1], z];
        if sweep >= 0.0 {
            ProfileCurve::Arc { center, axis: UP, reference: RX, radius: r, start: from, end: from + sweep }
        } else {
            ProfileCurve::Arc {
                center,
                axis: [0.0, 0.0, -1.0],
                reference: RX,
                radius: r,
                start: -from,
                end: -from - sweep,
            }
        }
    }

    fn pocket_prism(profile: Vec<ProfileCurve>, islands: Vec<Vec<ProfileCurve>>) -> LocalTool {
        LocalTool::Prism { profile, direction: UP, through: false, floor_fillet: None, islands }
    }

    fn removed(tool: &LocalTool) -> f64 {
        let (lo, hi) = ([0.0; 3], [60.0, 50.0, 20.0]);
        let (b, r) = subtract_tool(stock(), tool, lo, hi);
        assert!(matches!(r.unwrap(), Applied::Done(_)));
        60.0 * 50.0 * 20.0 - volume(&encode(&b).unwrap()).unwrap()
    }

    #[test]
    fn a_profile_with_a_non_finite_number_or_a_bad_radius_is_refused() {
        let z = 12.0;
        let (lo, hi) = ([0.0; 3], [60.0, 50.0, 20.0]);
        let mut bad_line = rect(8.0, 8.0, 38.0, 28.0, z);
        bad_line[0] = ProfileCurve::Line { start: [f64::NAN, 8.0, z], end: [38.0, 8.0, z] };
        let huge = vec![arc([25.0, 18.0], 1e300, 0.0, PI_F, z), arc([25.0, 18.0], 1e300, PI_F, PI_F, z)];
        let zero = vec![arc([25.0, 18.0], 0.0, 0.0, PI_F, z), arc([25.0, 18.0], 0.0, PI_F, PI_F, z)];
        let negative = vec![arc([25.0, 18.0], -4.0, 0.0, PI_F, z), arc([25.0, 18.0], -4.0, PI_F, PI_F, z)];
        let nan_arc = vec![arc([25.0, f64::NAN], 4.0, 0.0, PI_F, z)];
        let bad_island = pocket_prism(rect(8.0, 8.0, 38.0, 28.0, z), vec![nan_arc.clone()]);
        for tool in [
            pocket_prism(bad_line, vec![]),
            pocket_prism(huge, vec![]),
            pocket_prism(zero, vec![]),
            pocket_prism(negative, vec![]),
            bad_island,
        ] {
            let (b, r) = subtract_tool(stock(), &tool, lo, hi);
            assert!(r.is_err(), "{tool:?}");
            assert!((volume(&encode(&b).unwrap()).unwrap() - 60000.0).abs() < 1e-9, "the body is left alone");
        }
    }

    #[test]
    fn l_shaped_pocket_with_a_clockwise_reflex_round() {
        use std::f64::consts::{FRAC_PI_2, PI};
        let z = 12.0;
        let profile = vec![
            line([8.0, 8.0], [38.0, 8.0], z),
            line([38.0, 8.0], [38.0, 20.0], z),
            line([38.0, 20.0], [22.0, 20.0], z),
            arc([22.0, 22.0], 2.0, -FRAC_PI_2, -FRAC_PI_2, z),
            line([20.0, 22.0], [20.0, 34.0], z),
            line([20.0, 34.0], [8.0, 34.0], z),
            line([8.0, 34.0], [8.0, 8.0], z),
        ];
        let area = 30.0 * 12.0 + 12.0 * 14.0 + (4.0 - PI);
        let v = removed(&pocket_prism(profile, vec![]));
        assert!((v - area * 8.0).abs() < 1e-3, "removed {v} vs {}", area * 8.0);
    }

    #[test]
    fn two_arcs_next_to_each_other_make_a_lens_and_a_circle() {
        let z = 12.0;
        // lens: the part of two discs of radius 10 (centres 12 apart) that both cover
        let a = (8.0_f64).atan2(6.0);
        let lens =
            vec![arc([-6.0 + 30.0, 25.0], 10.0, -a, 2.0 * a, z), arc([6.0 + 30.0, 25.0], 10.0, PI_F - a, 2.0 * a, z)];
        let area = 2.0 * (100.0 * 0.6_f64.acos() - 6.0 * 8.0);
        let v = removed(&pocket_prism(lens, vec![]));
        assert!((v - area * 8.0).abs() < 1e-3, "lens {v} vs {}", area * 8.0);
        // a circle drawn as two half arcs
        let circle = vec![arc([30.0, 25.0], 8.0, -PI_F / 2.0, PI_F, z), arc([30.0, 25.0], 8.0, PI_F / 2.0, PI_F, z)];
        let v = removed(&pocket_prism(circle, vec![]));
        assert!((v - PI_F * 64.0 * 8.0).abs() < 1e-3, "circle {v}");
    }

    const PI_F: f64 = std::f64::consts::PI;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64, z: f64) -> Vec<ProfileCurve> {
        vec![
            line([x0, y0], [x1, y0], z),
            line([x1, y0], [x1, y1], z),
            line([x1, y1], [x0, y1], z),
            line([x0, y1], [x0, y0], z),
        ]
    }

    #[test]
    fn pocket_with_an_island_keeps_the_island() {
        let z = 12.0;
        let island = rect(16.0, 14.0, 24.0, 20.0, z);
        let v = removed(&pocket_prism(rect(8.0, 8.0, 38.0, 28.0, z), vec![island]));
        assert!((v - (30.0 * 20.0 - 48.0) * 8.0).abs() < 1e-3, "pocket with island removed {v}");
    }

    /// A post is drawn as two half arcs (the planner never writes one arc of 2 pi).
    #[test]
    fn a_post_drawn_as_two_half_arcs_stays_standing() {
        let z = 12.0;
        let post = vec![arc([25.0, 18.0], 4.0, 0.0, PI_F, z), arc([25.0, 18.0], 4.0, PI_F, PI_F, z)];
        let v = removed(&pocket_prism(rect(8.0, 8.0, 38.0, 28.0, z), vec![post]));
        assert!((v - (600.0 - PI_F * 16.0) * 8.0).abs() < 1e-3, "removed {v}");
    }

    #[test]
    fn a_complement_tool_leaves_a_rib_standing_on_a_cut_down_face() {
        let z = 12.0;
        let rib = rect(10.0, 20.0, 30.0, 26.0, z);
        let v = removed(&pocket_prism(rect(-100.0, -100.0, 200.0, 200.0, z), vec![rib]));
        assert!((v - (60.0 * 50.0 - 120.0) * 8.0).abs() < 1e-3, "complement removed {v}");
    }
}
