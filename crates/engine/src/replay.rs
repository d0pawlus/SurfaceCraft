//! Replay of the feature history: every feature in order, reading the current state of its
//! input bodies and writing its output body. A failing feature records its error and leaves its
//! body as it was; later features still run.
use crate::Session;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::Document;
use surfacecraft_doc::{BoolOp, FeatureOp, Mode, ObjectId, Side};
use surfacecraft_geom::{Transform, Vec3};
use surfacecraft_kernel::feature::{
    BoolKind, ProfileCurve, boolean, draft, extrude, loft, mirror, pattern, pattern_placed, revolve, shell, split,
    sweep,
};
use surfacecraft_kernel::naming::{EdgeSig, FaceSig, capture_edges, capture_faces, resolve_edges, resolve_faces};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::solid::{chamfer, fillet};
use surfacecraft_sketch::{Segment3, open_paths, path_samples, profiles};

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

/// The outer (largest) closed profile of a sketch as profile curves.
fn outer_profile(sk: &surfacecraft_sketch::Sketch) -> Result<Vec<ProfileCurve>, String> {
    let mut loops = profiles(sk, 1e-6);
    loops.sort_by(|a, b| loop_area(sk, b).abs().total_cmp(&loop_area(sk, a).abs()));
    loops.first().map(|l| curves(l)).ok_or_else(|| "the sketch has no closed profile".to_string())
}

/// The longest open chain of a sketch.
fn path_of(sk: &surfacecraft_sketch::Sketch) -> Result<Vec<Segment3>, String> {
    open_paths(sk, 1e-6)
        .into_iter()
        .next()
        .ok_or_else(|| "the path sketch has no open chain of lines and arcs".to_string())
}

/// Persistent naming: binds the edge or face references of a feature to the body it reads.
/// Without signatures they are captured from the stored indices (first run); with signatures
/// the indices are re-resolved geometrically against the current body, so an earlier edit that
/// moves the topology cannot move the reference. Returns whether the operation changed.
fn bind_refs(op: &mut FeatureOp, bodies: &HashMap<ObjectId, Brep>) -> Result<bool, String> {
    match op {
        FeatureOp::Fillet { target, edges, edge_refs, .. } | FeatureOp::Chamfer { target, edges, edge_refs, .. } => {
            bind_edges(bodies.get(target), edges, edge_refs)
        }
        FeatureOp::Shell { target, faces, face_refs, .. } | FeatureOp::Draft { target, faces, face_refs, .. } => {
            bind_faces(bodies.get(target), faces, face_refs)
        }
        _ => Ok(false),
    }
}

fn bind_edges(body: Option<&Brep>, edges: &mut Vec<usize>, refs: &mut Vec<EdgeSig>) -> Result<bool, String> {
    let Some(b) = body else { return Ok(false) };
    if refs.is_empty() {
        *refs = capture_edges(b, edges).map_err(|e| e.to_string())?;
        return Ok(!refs.is_empty());
    }
    let resolved = resolve_edges(b, refs).map_err(|e| e.to_string())?;
    let changed = resolved != *edges;
    *edges = resolved;
    Ok(changed)
}

fn bind_faces(body: Option<&Brep>, faces: &mut Vec<usize>, refs: &mut Vec<FaceSig>) -> Result<bool, String> {
    let Some(b) = body else { return Ok(false) };
    if refs.is_empty() {
        *refs = capture_faces(b, faces).map_err(|e| e.to_string())?;
        return Ok(!refs.is_empty());
    }
    let resolved = resolve_faces(b, refs).map_err(|e| e.to_string())?;
    let changed = resolved != *faces;
    *faces = resolved;
    Ok(changed)
}

/// Absolute placements of a pattern along a path: the start, then evenly spaced points; with
/// `follow` each copy also turns with the path tangent.
fn path_placements(chain: &[Segment3], count: usize, follow: bool) -> Result<Vec<[[f64; 4]; 4]>, String> {
    let pts = path_samples(chain, count).ok_or("the path is empty or has no length, or the count is zero")?;
    let (p0, t0) = pts.first().map(|(p, t)| (Vec3::from(*p), Vec3::from(*t))).ok_or("no path points")?;
    let mut out = Vec::with_capacity(pts.len());
    for (p, t) in &pts {
        let (p, t) = (Vec3::from(*p), Vec3::from(*t));
        let m = if follow {
            let axis = t0.cross(&t);
            let (sin, cos) = (axis.norm(), t0.dot(&t));
            let rot = if sin > 1e-9 {
                Transform::rotation_axis_angle(&(axis / sin), sin.atan2(cos))
            } else if cos < 0.0 {
                Transform::rotation_axis_angle(&surfacecraft_geom::any_perpendicular(&t0), std::f64::consts::PI)
            } else {
                Transform::translation(Vec3::zeros())
            };
            Transform::translation(p).then_after(&rot).then_after(&Transform::translation(-p0))
        } else {
            Transform::translation(p - p0)
        };
        out.push(m.to_rows());
    }
    Ok(out)
}

fn run(doc: &Document, op: &FeatureOp, bodies: &HashMap<ObjectId, Brep>, own: Option<&Brep>) -> Result<Brep, String> {
    let body =
        |id: &ObjectId| bodies.get(id).ok_or_else(|| format!("body {id} does not exist at this point of the history"));
    match op {
        FeatureOp::Base { brep } => Ok((**brep).clone()),
        FeatureOp::Extrude { sketch, distance, draft_deg, mode, target } => {
            let sk = doc.sketch(*sketch).ok_or_else(|| format!("sketch {sketch} is gone"))?;
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
            let sk = doc.sketch(*sketch).ok_or_else(|| format!("sketch {sketch} is gone"))?;
            let loops = profiles(&sk.sketch, 1e-6);
            let first = loops.first().ok_or("the sketch has no closed profile")?;
            let made = revolve(&curves(first), *axis_origin, *axis_direction, angle_deg.to_radians())
                .map_err(|e| e.to_string())?;
            let base = target.as_ref().map(body).transpose()?.or(own);
            combine(base, made, *mode)
        }
        FeatureOp::Fillet { target, edges, radius, .. } => {
            fillet(body(target)?, edges, *radius).map_err(|e| e.to_string())
        }
        FeatureOp::Chamfer { target, edges, distance, .. } => {
            chamfer(body(target)?, edges, *distance).map_err(|e| e.to_string())
        }
        FeatureOp::Shell { target, faces, thickness, .. } => {
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
        FeatureOp::Loft { sketches, degree, mode, target } => {
            let mut secs = Vec::with_capacity(sketches.len());
            for id in sketches {
                let sk = doc.sketch(*id).ok_or_else(|| format!("sketch {id} is gone"))?;
                secs.push(outer_profile(&sk.sketch)?);
            }
            let deg = degree.unwrap_or_else(|| u32::try_from(secs.len().saturating_sub(1)).unwrap_or(3).clamp(1, 3));
            let made = loft(&secs, deg, true).map_err(|e| e.to_string())?;
            let base = target.as_ref().map(body).transpose()?.or(own);
            combine(base, made, *mode)
        }
        FeatureOp::Sweep { profile, path, follow, mode, target } => {
            let psk = doc.sketch(*profile).ok_or_else(|| format!("sketch {profile} is gone"))?;
            let ksk = doc.sketch(*path).ok_or_else(|| format!("sketch {path} is gone"))?;
            let made = sweep(&outer_profile(&psk.sketch)?, &curves(&path_of(&ksk.sketch)?), *follow)
                .map_err(|e| e.to_string())?;
            let base = target.as_ref().map(body).transpose()?.or(own);
            combine(base, made, *mode)
        }
        FeatureOp::Draft { target, faces, angle_deg, neutral_origin, neutral_normal, pull, .. } => {
            let pull = pull.unwrap_or(*neutral_normal);
            draft(body(target)?, faces, *neutral_origin, *neutral_normal, pull, angle_deg.to_radians())
                .map_err(|e| e.to_string())
        }
        FeatureOp::Mirror { target, plane_origin, plane_normal, merge } => {
            mirror(body(target)?, *plane_origin, *plane_normal, *merge).map_err(|e| e.to_string())
        }
        FeatureOp::Split { target, plane_origin, plane_normal, keep } => {
            split(body(target)?, *plane_origin, *plane_normal, *keep == Side::Front).map_err(|e| e.to_string())
        }
        FeatureOp::PathPattern { target, path, count, follow } => {
            let ksk = doc.sketch(*path).ok_or_else(|| format!("sketch {path} is gone"))?;
            let placements = path_placements(&path_of(&ksk.sketch)?, *count, *follow)?;
            pattern_placed(body(target)?, &placements).map_err(|e| e.to_string())
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
    replay_doc(&mut s.doc)
}

/// [`replay`] on a document alone (a scratch copy, as a refit evaluates trial parameters).
pub fn replay_doc(doc: &mut Document) -> usize {
    let feature_bodies: Vec<ObjectId> = doc.features.iter().map(|f| f.body).collect();
    // bodies outside the history keep their geometry; feature bodies start empty
    let mut bodies: HashMap<ObjectId, Brep> =
        doc.bodies.iter().filter(|b| !feature_bodies.contains(&b.id)).map(|b| (b.id, (*b.brep).clone())).collect();
    let mut consumed: Vec<ObjectId> = Vec::new();
    let mut failed = 0;
    let ops: Vec<(usize, FeatureOp, ObjectId, bool)> =
        doc.features.iter().enumerate().map(|(i, f)| (i, f.op.clone(), f.body, f.suppressed)).collect();
    for (i, mut op, out, suppressed) in ops {
        let mut rebound = false;
        let result = if suppressed {
            Ok(None)
        } else {
            bind_refs(&mut op, &bodies).and_then(|changed| {
                rebound = changed;
                run(doc, &op, &bodies, bodies.get(&out)).map(Some)
            })
        };
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
        if let Some(f) = doc.features.get_mut(i) {
            f.error = err;
            // keep the resolved indices and any newly captured signatures
            if rebound {
                f.op = op;
            }
        }
    }
    write_back(doc, &bodies, &consumed, &feature_bodies);
    drop_orphan_dormant(doc);
    failed
}

/// Writes the results into the body objects. A body whose feature produced no geometry leaves the
/// model but is kept in `dormant_bodies` with its id, name and visibility; when geometry returns
/// it comes back as the same object.
fn write_back(
    doc: &mut Document,
    bodies: &HashMap<ObjectId, Brep>,
    consumed: &[ObjectId],
    feature_bodies: &[ObjectId],
) {
    for id in feature_bodies {
        let Some(brep) = bodies.get(id) else {
            if let Some(pos) = doc.bodies.iter().position(|o| o.id == *id) {
                let obj = doc.bodies.remove(pos);
                doc.dormant_bodies.retain(|o| o.id != *id);
                doc.dormant_bodies.push(obj);
            }
            continue;
        };
        if doc.body(*id).is_none() {
            let revived = doc.dormant_bodies.iter().position(|o| o.id == *id).map(|pos| doc.dormant_bodies.remove(pos));
            let obj = revived.unwrap_or_else(|| surfacecraft_doc::BodyObject {
                id: *id,
                name: format!("body {}", id.0),
                brep: Arc::new(brep.clone()),
                visible: true,
                consumed: false,
            });
            doc.bodies.push(obj);
        }
        if let Some(obj) = doc.body_mut(*id) {
            if *obj.brep != *brep {
                obj.brep = Arc::new(brep.clone());
            }
            obj.consumed = consumed.contains(id);
        }
    }
}

/// Dormant bodies whose feature is gone (deleted) do not linger.
fn drop_orphan_dormant(doc: &mut Document) {
    let has_feature = |id: ObjectId| doc.features.iter().chain(doc.rolled_back.iter()).any(|f| f.body == id);
    let keep: Vec<bool> = doc.dormant_bodies.iter().map(|b| has_feature(b.id)).collect();
    let mut keep = keep.into_iter();
    doc.dormant_bodies.retain(|_| keep.next().unwrap_or(false));
}
