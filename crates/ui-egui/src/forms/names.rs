//! Names, units and hints of parameters and objects: what a field is called, which unit it shows,
//! which kinds of object it takes and what its tooltip says.
use super::schema::*;
use super::*;
use crate::i18n::{self, find, t};
use serde_json::Value;
use surfacecraft_doc::ObjectKind;

// ---------------------------------------------------------------------------------------------
// Widgets
// ---------------------------------------------------------------------------------------------

/// The unit suffix a numeric parameter gets from its name.
#[must_use]
pub fn unit_of(name: &str) -> &'static str {
    if name.ends_with("_deg") || name == "angle" {
        " °"
    } else if [
        "radius",
        "distance",
        "size",
        "thickness",
        "tolerance",
        "length",
        "width",
        "height",
        "offset",
        "diameter",
    ]
    .iter()
    .any(|k| name.contains(k))
    {
        " mm"
    } else {
        ""
    }
}

/// The kinds of object a reference parameter takes: the one its name says (`mesh`, `region`,
/// `target`), else the ones its description names ("Scan mesh.", "Reference: a primitive, a mesh
/// or a body."). Empty when neither says: any object.
#[must_use]
pub fn accepted_kinds(name: &str, description: Option<&str>) -> Vec<ObjectKind> {
    const TABLE: [(&str, &str, ObjectKind); 8] = [
        ("sketch", "sketches", ObjectKind::Sketch),
        ("mesh", "meshes", ObjectKind::Mesh),
        ("region", "regions", ObjectKind::Region),
        ("primitive", "primitives", ObjectKind::Primitive),
        ("surface", "surfaces", ObjectKind::Surface),
        ("quad", "quads", ObjectKind::Quad),
        ("feature", "features", ObjectKind::Feature),
        ("body", "bodies", ObjectKind::Body),
    ];
    let n = name.to_ascii_lowercase();
    if n == "target" {
        return vec![ObjectKind::Body];
    }
    if let Some((_, _, kind)) = TABLE.iter().find(|(one, _, _)| n.starts_with(one)) {
        return vec![*kind];
    }
    let text = description.unwrap_or_default().to_ascii_lowercase();
    let words: Vec<&str> = text.split(|c: char| !c.is_ascii_alphabetic()).filter(|w| !w.is_empty()).collect();
    TABLE.iter().filter(|(one, many, _)| words.contains(one) || words.contains(many)).map(|(_, _, k)| *k).collect()
}

/// The object a reference parameter starts with: the selected one when it is of a kind the
/// parameter takes, else the first object that is.
#[must_use]
pub fn preferred_object(objects: &[ObjInfo], selected: Option<u64>, kinds: &[ObjectKind]) -> Option<u64> {
    let fits = |o: &&ObjInfo| kinds.is_empty() || kinds.contains(&o.kind);
    let selected = selected.and_then(|id| objects.iter().find(|o| o.id == id)).filter(fits);
    selected.or_else(|| objects.iter().find(fits)).map(|o| o.id)
}

/// The kinds of `kinds` as words ("mesh, body"), for a message.
pub(super) fn kind_names(kinds: &[ObjectKind]) -> String {
    kinds.iter().map(|k| t(kind_name_key(*k))).collect::<Vec<_>>().join(", ")
}

/// The last name of a value path (`a.b[0]` is `b`).
pub(super) fn leaf_name(path: &str) -> &str {
    let last = path.rsplit('.').next().unwrap_or(path);
    last.split('[').next().unwrap_or(last)
}

pub(super) fn kind_name_key(kind: ObjectKind) -> &'static str {
    match kind {
        ObjectKind::Mesh => "kind.mesh",
        ObjectKind::Body => "kind.body",
        ObjectKind::Region => "kind.region",
        ObjectKind::Primitive => "kind.primitive",
        ObjectKind::Surface => "kind.surface",
        ObjectKind::Quad => "kind.quad",
        ObjectKind::Sketch => "kind.sketch",
        ObjectKind::Feature => "kind.feature",
    }
}

/// The label of parameter `name`: the catalogue's (`param.<command>.<name>`, then
/// `param.<name>`), else the schema's name in plain words.
pub(super) fn label_of(ctx: &FormCtx<'_>, name: &str) -> String {
    let specific = format!("param.{}.{name}", ctx.command);
    let generic = format!("param.{name}");
    find(&specific).or_else(|| find(&generic)).map_or_else(|| humanize(name), str::to_string)
}

/// A schema name in plain words: `max_distance` is "Max distance", `draft_deg` is "Draft" (the
/// field shows the unit).
#[must_use]
pub fn humanize(name: &str) -> String {
    let base = name.strip_suffix("_deg").or_else(|| name.strip_suffix("_mm")).unwrap_or(name);
    let words = base.replace('_', " ");
    let mut chars = words.chars();
    match chars.next() {
        Some(first) => first.to_uppercase().collect::<String>() + chars.as_str(),
        None => name.to_string(),
    }
}

pub(super) fn description_of(root: &Value, schema: &Value) -> Option<String> {
    schema
        .get("description")
        .or_else(|| resolve(root, schema).get("description"))
        .and_then(Value::as_str)
        .map(str::to_string)
}

/// What the tooltip of a parameter says: its description in the interface language.
pub(super) fn hover_of(root: &Value, schema: &Value) -> Option<String> {
    description_of(root, schema).map(|d| i18n::hint(&d))
}
