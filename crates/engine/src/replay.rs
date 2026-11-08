//! Replay of the feature history: every feature in order, reading the current state of its
//! input bodies and writing its output body. A failing feature records its error and leaves its
//! body as it was; later features still run.
use crate::Session;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{BoolOp, FeatureOp, Mode, ObjectId};
use surfacecraft_geom::{Transform, Vec3};
use surfacecraft_kernel::feature::{BoolKind, ProfileCurve, boolean, extrude, pattern, revolve, shell};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::solid::{chamfer, fillet};
use surfacecraft_sketch::{Segment3, profiles};

fn curves(seg: &[Segment3]) -> Vec<ProfileCurve> {
    seg.iter()
        .map(|g| match g {
            Segment3::Line { start, end } => ProfileCurve::Line { start: *start, end: *end },
            Segment3::Arc { center, axis, reference, radius, start, end } => ProfileCurve::Arc {
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

fn combine(base: Option<&Brep>, made: Brep, mode: Mode) -> Result<Brep, String> {
    match (mode, base) {
        (Mode::New, _) => Ok(made),
        (Mode::Add, Some(b)) => boolean(b, &made, BoolKind::Union).map_err(|e| e.to_string()),
        (Mode::Cut, Some(b)) => boolean(b, &made, BoolKind::Subtract).map_err(|e| e.to_string()),
        (_, None) => Err("add/cut needs an existing target body".into()),
    }
}

/// Signed area of a profile loop projected on the sketch plane (for outer/hole ordering).
fn loop_area(s: &surfacecraft_sketch::Sketch, seg: &[Segment3]) -> f64 {
    let pts: Vec<[f64; 2]> = seg
        .iter()
        .flat_map(|g| match g {
            Segment3::Line { start, .. } => vec![s.plane.to_2d(&surfacecraft_geom::Point3::from(*start))],
            Segment3::Arc { center, axis, reference, radius, start, end } => {
                let (c, a, r) = (Vec3::from(*center), Vec3::from(*axis), Vec3::from(*reference));
                let b = a.cross(&r);
                (0..8)
                    .map(|k| {
                        let t = start + (end - start) * f64::from(k) / 8.0;
                        let p = c + (r * t.cos() + b * t.sin()) * *radius;
                        s.plane.to_2d(&surfacecraft_geom::Point3::from(p))
                    })
                    .collect()
            }
        })
        .collect();
    let n = pts.len();
    (0..n)
        .map(|i| {
            let (p, q) = (pts[i], pts[(i + 1) % n]);
            p[0] * q[1] - q[0] * p[1]
        })
        .sum::<f64>()
        * 0.5
}

fn run(s: &Session, op: &FeatureOp, bodies: &HashMap<ObjectId, Brep>, own: Option<&Brep>) -> Result<Brep, String> {
    let body =
        |id: &ObjectId| bodies.get(id).ok_or_else(|| format!("body {id} does not exist at this point of the history"));
    match op {
        FeatureOp::Base { brep } => Ok((**brep).clone()),
        FeatureOp::Extrude { sketch, distance, draft_deg, mode, target } => {
            let sk = s.doc.sketch(*sketch).ok_or_else(|| format!("sketch {sketch} is gone"))?;
            let mut loops = profiles(&sk.sketch, 1e-6);
            if loops.is_empty() {
                return Err("the sketch has no closed profile".into());
            }
            // outer boundary (largest) first, the rest are holes
            loops.sort_by(|a, b| loop_area(&sk.sketch, b).abs().total_cmp(&loop_area(&sk.sketch, a).abs()));
            let ls: Vec<Vec<ProfileCurve>> = loops.iter().map(|l| curves(l)).collect();
            let n = sk.sketch.plane.normal();
            let made = extrude(&ls, n.into(), *distance, draft_deg.to_radians()).map_err(|e| e.to_string())?;
            let base = target.as_ref().map(body).transpose()?.or(own);
            combine(base, made, *mode)
        }
        FeatureOp::Revolve { sketch, axis_origin, axis_direction, angle_deg, mode, target } => {
            let sk = s.doc.sketch(*sketch).ok_or_else(|| format!("sketch {sketch} is gone"))?;
            let loops = profiles(&sk.sketch, 1e-6);
            let first = loops.first().ok_or("the sketch has no closed profile")?;
            let made = revolve(&curves(first), *axis_origin, *axis_direction, angle_deg.to_radians())
                .map_err(|e| e.to_string())?;
            let base = target.as_ref().map(body).transpose()?.or(own);
            combine(base, made, *mode)
        }
        FeatureOp::Fillet { target, edges, radius } => fillet(body(target)?, edges, *radius).map_err(|e| e.to_string()),
        FeatureOp::Chamfer { target, edges, distance } => {
            chamfer(body(target)?, edges, *distance).map_err(|e| e.to_string())
        }
        FeatureOp::Shell { target, faces, thickness } => {
            shell(body(target)?, faces, *thickness).map_err(|e| e.to_string())
        }
        FeatureOp::Pattern { target, count, translate, rotate_axis, rotate_origin, rotate_deg } => {
            let mut step = Transform::translation(Vec3::from(*translate));
            if let Some(ax) = rotate_axis {
                let o = Vec3::from(*rotate_origin);
                let rot = Transform::translation(o)
                    .then_after(&Transform::rotation_axis_angle(&Vec3::from(*ax), rotate_deg.to_radians()))
                    .then_after(&Transform::translation(-o));
                step = step.then_after(&rot);
            }
            pattern(body(target)?, *count, &step.to_rows()).map_err(|e| e.to_string())
        }
        FeatureOp::Boolean { target, tool, op } => {
            let k = match op {
                BoolOp::Union => BoolKind::Union,
                BoolOp::Subtract => BoolKind::Subtract,
                BoolOp::Intersect => BoolKind::Intersect,
            };
            boolean(body(target)?, body(tool)?, k).map_err(|e| e.to_string())
        }
    }
}

/// Replays the whole history; returns the number of features that failed.
pub fn replay(s: &mut Session) -> usize {
    let feature_bodies: Vec<ObjectId> = s.doc.features.iter().map(|f| f.body).collect();
    // bodies outside the history keep their geometry; feature bodies start empty
    let mut bodies: HashMap<ObjectId, Brep> =
        s.doc.bodies.iter().filter(|b| !feature_bodies.contains(&b.id)).map(|b| (b.id, (*b.brep).clone())).collect();
    let mut consumed: Vec<ObjectId> = Vec::new();
    let mut failed = 0;
    let ops: Vec<(usize, FeatureOp, ObjectId, bool)> =
        s.doc.features.iter().enumerate().map(|(i, f)| (i, f.op.clone(), f.body, f.suppressed)).collect();
    for (i, op, out, suppressed) in ops {
        let result = if suppressed { Ok(None) } else { run(s, &op, &bodies, bodies.get(&out)).map(Some) };
        let err = match result {
            Ok(Some(b)) => {
                bodies.insert(out, b);
                if let FeatureOp::Boolean { tool, .. } = &op {
                    consumed.push(*tool);
                }
                None
            }
            Ok(None) => None,
            Err(e) => {
                failed += 1;
                Some(e)
            }
        };
        if let Some(f) = s.doc.features.get_mut(i) {
            f.error = err;
        }
    }
    // write the results back into body objects
    for id in feature_bodies {
        match bodies.get(&id) {
            Some(b) => {
                if let Some(obj) = s.doc.body_mut(id) {
                    if *obj.brep != *b {
                        obj.brep = Arc::new(b.clone());
                    }
                    obj.visible = !consumed.contains(&id);
                } else {
                    let name = format!("body {}", id.0);
                    s.doc.bodies.push(surfacecraft_doc::BodyObject {
                        id,
                        name,
                        brep: Arc::new(b.clone()),
                        visible: true,
                    });
                }
            }
            None => {
                s.doc.bodies.retain(|o| o.id != id);
            }
        }
    }
    failed
}
