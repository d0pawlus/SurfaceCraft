//! The pose `align.auto` proposed, as the open tool shows it before it is applied.
//!
//! The proposal is the last entry of the command log, read while the auto-align tool is open
//! with the settings the proposal was made for, and only when it was made since the tool was
//! opened. Any other command, an edited form or a closed tool therefore drops it. The viewport
//! draws the scan moved by the proposed matrix (the way a gizmo drag is drawn); the document
//! changes only when the tool runs the command again with `apply: true`.
use crate::app::SurfaceCraftApp;
use serde_json::Value;
use std::sync::Arc;
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::{Aabb, Point3, Transform};

/// The command whose proposal the tool panel previews.
pub const COMMAND: &str = "align.auto";
/// The parameter that makes the command move the scan instead of proposing.
const APPLY: &str = "apply";

/// A pose proposed for the settings in the open form.
#[derive(Clone, Debug, PartialEq)]
pub struct Proposal {
    /// The meshes the motion would move: the scan and the meshes named in `also`.
    pub objects: Vec<ObjectId>,
    /// Row-major world matrix of the motion.
    pub matrix: [[f64; 4]; 4],
    /// Do enough faces agree with the proposed frame? False: a guess to check by eye.
    pub confident: bool,
}

/// `params` without the `apply` switch: what a proposal is made for.
fn settings(params: &Value) -> Value {
    match params {
        Value::Object(map) => {
            Value::Object(map.iter().filter(|(k, _)| *k != APPLY).map(|(k, v)| (k.clone(), v.clone())).collect())
        }
        other => other.clone(),
    }
}

/// `params` with the `apply` switch set to `apply`. Anything but an object is returned as it is
/// (the command refuses it with its own message).
#[must_use]
pub fn with_apply(params: &Value, apply: bool) -> Value {
    match params {
        Value::Object(map) => {
            let rest = map.iter().filter(|(k, _)| *k != APPLY).map(|(k, v)| (k.clone(), v.clone()));
            Value::Object(rest.chain([(APPLY.to_string(), Value::Bool(apply))]).collect())
        }
        other => other.clone(),
    }
}

/// Do the form's parameters ask to move the scan at once?
#[must_use]
pub fn asks_to_apply(params: &Value) -> bool {
    params.get(APPLY).and_then(Value::as_bool).unwrap_or(false)
}

/// The object a command parameter names, by id or by name (as the engine reads a reference).
fn named(app: &SurfaceCraftApp, reference: &Value) -> Option<ObjectId> {
    match reference {
        Value::Number(n) => n.as_u64().map(ObjectId),
        Value::String(name) => app.session.doc.find_by_name(name).map(|(id, _)| id),
        _ => None,
    }
}

/// The proposal to show: the last command was an `align.auto` that only proposed, the tool is
/// still open, and its form still holds the settings the proposal was made for.
#[must_use]
pub fn current(app: &SurfaceCraftApp) -> Option<Proposal> {
    if app.palette_command != Some(COMMAND) {
        return None;
    }
    let last = app.session.log.last().filter(|e| e.command == COMMAND && e.ok)?;
    if app.tool_opened_on.as_ref().is_some_and(|before| Arc::ptr_eq(before, last)) {
        // proposed before the tool was opened this time (and cancelled then): not on screen
        return None;
    }
    if last.outcome.get("applied").and_then(Value::as_bool) != Some(false) {
        return None;
    }
    let form: Value = serde_json::from_str(&app.palette_params).ok()?;
    if settings(&form) != settings(&last.params) {
        return None;
    }
    let matrix = serde_json::from_value(last.outcome.get("matrix")?.clone()).ok()?;
    let scan = last.outcome.get("id").and_then(Value::as_u64)?;
    let also = last.params.get("also").and_then(Value::as_array).into_iter().flatten();
    let objects = std::iter::once(ObjectId(scan)).chain(also.filter_map(|r| named(app, r))).collect();
    let confident = last.outcome.get("confident").and_then(Value::as_bool).unwrap_or(false);
    Some(Proposal { objects, matrix, confident })
}

/// The box around `bbox` moved by `motion` (its eight corners, moved).
fn moved_box(bbox: &Aabb, motion: &Transform) -> Aabb {
    let (lo, hi) = (bbox.min, bbox.max);
    let corners: Vec<Point3> = (0..8u8)
        .map(|i| {
            let pick = |bit: u8, a: f64, b: f64| if i & bit == 0 { a } else { b };
            motion.apply(&Point3::new(pick(1, lo.x, hi.x), pick(2, lo.y, hi.y), pick(4, lo.z, hi.z)))
        })
        .collect();
    Aabb::from_points(&corners)
}

/// Where the proposal on screen puts its meshes: the box to frame so the proposed pose is in
/// view (a fresh scan is often far from the origin the proposal moves it to). `None` without a
/// proposal.
#[must_use]
pub fn framed_box(app: &SurfaceCraftApp) -> Option<Aabb> {
    let proposal = current(app)?;
    let motion = Transform::from_rows(&proposal.matrix)?;
    let corners: Vec<Point3> = app
        .session
        .doc
        .meshes
        .iter()
        .filter(|m| proposal.objects.contains(&m.id) && !m.mesh.bbox().is_empty())
        .flat_map(|m| {
            let moved = moved_box(&m.mesh.bbox(), &motion);
            [moved.min, moved.max]
        })
        .collect();
    (!corners.is_empty()).then(|| Aabb::from_points(&corners))
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    #[test]
    fn with_apply_replaces_the_switch_and_keeps_the_rest() {
        let params = json!({"object": 3, "mode": "all_faces", "apply": false});

        let applied = with_apply(&params, true);

        assert_eq!(applied, json!({"object": 3, "mode": "all_faces", "apply": true}));
        assert_eq!(params["apply"], false, "the input is not changed");
    }

    #[test]
    fn with_apply_leaves_what_is_not_an_object_alone() {
        assert_eq!(with_apply(&json!([1, 2]), true), json!([1, 2]));
    }

    #[test]
    fn settings_ignore_only_the_apply_switch() {
        let proposed = json!({"object": 3, "apply": false});

        assert_eq!(settings(&proposed), settings(&json!({"object": 3, "apply": true})));
        assert_eq!(settings(&proposed), settings(&json!({"object": 3})));
        assert_ne!(settings(&proposed), settings(&json!({"object": 3, "mode": "all_faces"})));
    }

    #[test]
    fn asks_to_apply_reads_only_a_true_switch() {
        assert!(asks_to_apply(&json!({"apply": true})));
        assert!(!asks_to_apply(&json!({"apply": false})));
        assert!(!asks_to_apply(&json!({"apply": "yes"})));
        assert!(!asks_to_apply(&json!({})));
    }
}
