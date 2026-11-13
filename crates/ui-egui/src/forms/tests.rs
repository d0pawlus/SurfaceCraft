//! Tests of the form: schema classification, defaults, validation and the widgets' values.
use super::names::accepted_kinds;
use super::rows::{described_default, start_value};
use super::schema::{classify, default_value, variants};
use super::validate::validate;
use super::widgets::option_hint;
use crate::i18n::{self, Lang};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectKind;

use super::*;
use surfacecraft_engine::cmd;

fn schema_of(id: &str) -> Value {
    (cmd::find(id).map(|c| c.input)).map_or(Value::Null, |f| f())
}

#[test]
fn a_number_or_expression_union_is_edited_as_a_number() {
    let number = json!({"type": "number", "x-expr": true});
    let expr = json!({"type": "object", "properties": {"expr": {"type": "string"}}, "required": ["expr"]});
    let root = json!({});
    let union = json!({"anyOf": [number, expr]});
    assert_eq!(classify(&root, &union).0, Kind::Num);
    let int = json!({"anyOf": [{"type": "integer", "x-expr": true}, expr]});
    assert_eq!(classify(&root, &int).0, Kind::Int);
    let optional = json!({"anyOf": [union, {"type": "null"}]});
    let (kind, nullable, _) = classify(&root, &optional);
    assert!(kind == Kind::Num && nullable);
    // a union of other things is still not a number
    let other = json!({"anyOf": [{"type": "number"}, {"type": "object"}]});
    assert_eq!(classify(&root, &other).0, Kind::Any);
}

#[test]
fn classifies_the_schemas_of_every_command_without_panicking() {
    for c in cmd::registry() {
        let s = (c.input)();
        let (k, _, inner) = classify(&s, &s);
        assert!(matches!(k, Kind::Object | Kind::Any), "{} root is {k:?}", c.id);
        let v = default_value(&s, inner, None);
        // required parameters get a value of the right type
        let issues = validate(&s, &v, &[]);
        let bad: Vec<_> = issues.iter().filter(|i| !i.message.contains('#') && i.path.is_empty()).collect();
        assert!(bad.is_empty(), "{}: {bad:?}", c.id);
    }
}

#[test]
fn extrude_schema_has_an_object_reference_a_number_and_an_enum() {
    let s = schema_of("feature.extrude");
    let props = s["properties"].as_object().unwrap();
    assert_eq!(classify(&s, &props["sketch"]).0, Kind::ObjRef);
    assert_eq!(classify(&s, &props["distance"]).0, Kind::Num);
    assert!(matches!(classify(&s, &props["mode"]).0, Kind::Enum(v) if v.len() == 3));
    let (k, nullable, _) = classify(&s, &props["target"]);
    assert_eq!((k, nullable), (Kind::ObjRef, true));
}

#[test]
fn an_object_parameter_takes_only_the_kinds_its_name_or_description_names() {
    use ObjectKind::{Body, Mesh, Primitive, Region, Surface};
    assert_eq!(accepted_kinds("mesh", None), vec![Mesh]);
    assert_eq!(accepted_kinds("regions", Some("Regions of the mesh.")), vec![Region]);
    assert_eq!(accepted_kinds("object", Some("Mesh id or name.")), vec![Mesh]);
    assert_eq!(
        accepted_kinds("against", Some("Reference: a primitive, a mesh or a body.")),
        vec![Mesh, Primitive, Body]
    );
    assert_eq!(accepted_kinds("object", Some("Surface (or sheet body) to thicken.")), vec![Surface, Body]);
    assert!(accepted_kinds("object", Some("Object id or name.")).is_empty(), "any object");
    // a mesh tool: a region is refused before the command runs, the mesh is fine
    let objects =
        [ObjInfo { id: 1, kind: Mesh, name: "scan".into() }, ObjInfo { id: 2, kind: Region, name: "top".into() }];
    let s = schema_of("mesh.decimate");
    assert!(validate(&s, &json!({"object": 2}), &objects).iter().any(|i| i.path == "object"));
    assert!(validate(&s, &json!({"object": 1}), &objects).iter().all(|i| i.path != "object"));
    // the field starts with the selection only when it fits, else with the first object that does
    assert_eq!(preferred_object(&objects, Some(2), &[Mesh]), Some(1));
    assert_eq!(preferred_object(&objects, Some(2), &[]), Some(2));
    assert_eq!(preferred_object(&objects, Some(2), &[Body]), None);
}

#[test]
fn a_sketch_entity_and_a_constraint_are_alternatives_with_fields() {
    let s = schema_of("sketch.add");
    let item = &s["properties"]["entities"]["items"];
    let (kind, _, node) = classify(&s, item);
    assert_eq!(kind, Kind::Any);
    let vars = variants(&s, node).expect("entities are alternatives");
    assert_eq!(vars.iter().map(|v| v.tag).collect::<Vec<_>>(), ["line", "arc", "circle", "polygon"]);
    // a new entity is a whole line, ready to edit as fields
    let line = default_value(&s, item, None);
    assert_eq!(line, json!({"line": {"from": [0.0, 0.0], "to": [0.0, 0.0]}}));
    assert!(vars[0].holds(&line) && !vars[2].holds(&line));
    let circle = default_value(&s, vars[2].schema, None);
    assert_eq!(circle, json!({"circle": {"center": [0.0, 0.0], "radius": 0.0}}));
    // constraints carry their kind in a `type` field next to the others
    let s = schema_of("sketch.constrain");
    let item = &s["properties"]["constraints"]["items"];
    let vars = variants(&s, classify(&s, item).2).expect("constraints are alternatives");
    assert_eq!(vars.len(), 15);
    assert!(vars.iter().all(|v| v.tag_field == Some("type")));
    let fix = default_value(&s, item, None);
    assert_eq!(fix["type"], "fix");
    assert!(fix.get("point").is_some() && fix.get("at").is_some(), "{fix}");
    // every alternative has a name in both languages
    for v in &vars {
        for lang in Lang::ALL {
            assert!(i18n::catalog(lang).contains_key(&format!("variant.{}", v.tag)), "{lang:?} variant.{}", v.tag);
        }
    }
    // the choices of an enum say what they mean
    let s = schema_of("feature.boolean");
    let (kind, _, node) = classify(&s, &s["properties"]["op"]);
    assert!(matches!(kind, Kind::Enum(_)), "{kind:?}");
    assert_eq!(option_hint(&s, node, &json!("union")).as_deref(), Some("Union."));
    // a 3-2-1 datum is not such a list: it keeps its own editor
    let s = schema_of("align.datum");
    assert!(variants(&s, classify(&s, &s["properties"]["primary"]).2).is_none());
}

#[test]
fn a_ticked_optional_parameter_starts_with_the_default_its_description_states() {
    assert_eq!(
        described_default("Fraction of triangles to keep, (0, 1] (default 0.5 when `target` is not given)."),
        Some(json!(0.5))
    );
    assert_eq!(described_default("Edges sharper than this stay in place (degrees, default 40)."), Some(json!(40.0)));
    assert_eq!(described_default("Keep the open boundary in place (default true)."), Some(json!(true)));
    assert_eq!(described_default("Default 3."), Some(json!(3.0)));
    assert_eq!(described_default("Mesh they belong to (default: the object being aligned)."), None);
    assert_eq!(described_default("Triangles to keep."), None);
    // ticking the field on must not change what the tool does: it shows the default
    let s = schema_of("mesh.decimate");
    let props = s["properties"].as_object().unwrap();
    let tokens = Tokens::resolve(crate::theme::Preset::Dark);
    let (issues, request) = (HashMap::new(), RefCell::new(None));
    let ctx = FormCtx {
        command: "mesh.decimate",
        objects: &[],
        selected: None,
        pick_objects: &[],
        pick_points: &[],
        pick_triangles: &[],
        picking: None,
        pick_request: &request,
        tokens: &tokens,
        issues: &issues,
    };
    let start = |name: &str| start_value(&s, &props[name], name, &classify(&s, &props[name]).0, &ctx);
    assert_eq!(start("ratio"), json!(0.5));
    assert_eq!(start("feature_angle_deg"), json!(40.0));
    assert_eq!(start("preserve_boundary"), json!(true));
}

#[test]
fn validation_reports_missing_and_out_of_range_values() {
    let s = schema_of("mesh.synth");
    let issues = validate(&s, &json!({"shape": "sphere", "resolution": -3}), &[]);
    assert!(issues.iter().any(|i| i.path == "resolution"), "{issues:?}");
    let issues = validate(&s, &json!({}), &[]);
    assert!(issues.iter().any(|i| i.path == "shape"), "{issues:?}");
}
