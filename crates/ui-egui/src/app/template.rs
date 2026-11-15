//! The parameters a tool's form starts with: the required fields of a command's schema, filled
//! from what is selected and picked in the viewport.
use serde_json::{Value, json};
use surfacecraft_doc::{ObjectId, ObjectKind};

/// What the UI knows when it prefills parameters.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PickContext {
    /// Selected object.
    pub selected: Option<ObjectId>,
    /// Picked points.
    pub points: Vec<[f64; 3]>,
    /// Picked triangles (original ids of the scan).
    pub triangles: Vec<u32>,
    /// Every object of the document (id, kind), so object references can be prefilled with a
    /// real object instead of a made-up id.
    pub objects: Vec<(ObjectId, ObjectKind)>,
    /// Picked B-rep faces (indices in the body record).
    pub faces: Vec<usize>,
    /// Picked B-rep edges (indices in the body record).
    pub edges: Vec<usize>,
}

impl PickContext {
    /// The object a reference parameter called `name` should point at: the selection when it is
    /// of a kind the parameter takes (see [`crate::forms::accepted_kinds`]), else the first
    /// object that is.
    #[must_use]
    pub fn reference_for(&self, name: &str, description: Option<&str>) -> Option<ObjectId> {
        let kinds = crate::forms::accepted_kinds(name, description);
        let fits = |k: ObjectKind| kinds.is_empty() || kinds.contains(&k);
        if let Some(sel) = self.selected
            && self.objects.iter().any(|(id, k)| *id == sel && fits(*k))
        {
            return Some(sel);
        }
        self.objects.iter().find(|(_, k)| fits(*k)).map(|(id, _)| *id)
    }
}

/// Whether a property schema is an object reference (`ObjRef`: an id or a name).
fn is_obj_ref(p: &Value) -> bool {
    p.get("$ref").and_then(Value::as_str).is_some_and(|r| r.ends_with("/ObjRef"))
}

/// Example parameters from an input schema: required properties with neutral values; `object`
/// gets the selected object, `from`/`points` the picked points, `triangles` the picked triangles.
#[must_use]
pub fn template(schema: &Value, ctx: &PickContext) -> Value {
    let mut out = serde_json::Map::new();
    let props = schema.get("properties").and_then(Value::as_object);
    let required: Vec<&str> = schema
        .get("required")
        .and_then(Value::as_array)
        .map(|a| a.iter().filter_map(Value::as_str).collect())
        .unwrap_or_default();
    if let Some(props) = props {
        for name in required {
            let p = props.get(name).cloned().unwrap_or(Value::Null);
            let described = p.get("description").and_then(Value::as_str);
            let v = match name {
                _ if is_obj_ref(&p) || name == "object" || name == "mesh" => {
                    ctx.reference_for(name, described).map_or(json!(""), |id| json!(id.0))
                }
                _ if p.get("items").is_some_and(is_obj_ref) => {
                    let n = p.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
                    let id = ctx.reference_for(name, described).map_or(json!(""), |id| json!(id.0));
                    Value::Array(vec![id; n])
                }
                "from" | "points" if !ctx.points.is_empty() => json!(ctx.points),
                "triangles" if !ctx.triangles.is_empty() => json!(ctx.triangles),
                "faces" if !ctx.faces.is_empty() => json!(ctx.faces),
                "edges" if !ctx.edges.is_empty() => json!(ctx.edges),
                _ => match crate::fieldpick::PickMode::of(&p).and_then(crate::fieldpick::PickMode::default_value) {
                    Some(datum) => datum,
                    None => example(&p, schema),
                },
            };
            out.insert(name.to_string(), v);
        }
    }
    Value::Object(out)
}

fn example(p: &Value, root: &Value) -> Value {
    if let Some(r) = p.get("$ref").and_then(Value::as_str) {
        let name = r.rsplit('/').next().unwrap_or("");
        if let Some(def) = root.get("$defs").and_then(|d| d.get(name)) {
            if let Some(e) = def.get("enum").and_then(Value::as_array).and_then(|a| a.first()) {
                return e.clone();
            }
            if let Some(one) = def.get("oneOf").and_then(Value::as_array).and_then(|a| a.first()) {
                if let Some(c) = one.get("const") {
                    return c.clone();
                }
                if let Some(e) = one.get("enum").and_then(Value::as_array).and_then(|a| a.first()) {
                    return e.clone();
                }
            }
            return example(def, root);
        }
    }
    // a number that may also be an expression object: the number
    if let Some(first) = p.get("anyOf").and_then(|a| a.get(0)).filter(|f| f.get("x-expr").is_some()) {
        return example(first, root);
    }
    match p.get("type").and_then(Value::as_str) {
        Some("string") => json!(""),
        Some("number") | Some("integer") => json!(0),
        Some("boolean") => json!(false),
        Some("array") => {
            let n = p.get("minItems").and_then(Value::as_u64).unwrap_or(0);
            let item = p.get("items").map_or(json!(0), |i| example(i, root));
            Value::Array((0..n).map(|_| item.clone()).collect())
        }
        Some("object") => template(p, &PickContext::default()),
        _ => json!(null),
    }
}
