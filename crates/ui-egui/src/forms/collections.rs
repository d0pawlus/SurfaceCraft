//! Lists and object pickers: the items of a list parameter, the object reference picker and the
//! free-form JSON text editor.
use super::names::*;
use super::rows::*;
use super::schema::*;
use super::widgets::*;
use super::*;
use crate::fieldpick::{PickKind, PickMode};
use crate::i18n::t;
use serde_json::{Value, json};
use surfacecraft_doc::ObjectKind;

#[allow(clippy::too_many_arguments)]
pub(super) fn list_widget(
    ui: &mut egui::Ui,
    root: &Value,
    schema: &Value,
    value: &mut Value,
    name: &str,
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<egui::Id>,
) -> bool {
    let mut changed = false;
    let item_schema = schema.get("items").cloned().unwrap_or(Value::Bool(true));
    let (item_kind, _, _) = classify(root, &item_schema);
    let max = schema.get("maxItems").and_then(Value::as_u64).unwrap_or(u64::MAX) as usize;
    let min = schema.get("minItems").and_then(Value::as_u64).unwrap_or(0) as usize;
    let item_kinds = accepted_kinds(name, schema.get("description").and_then(Value::as_str));
    let Some(items) = value.as_array_mut() else { return false };
    let items_len_over_min = items.len() > min;
    let mut remove = None;
    for (i, v) in items.iter_mut().enumerate() {
        // top-aligned: an item of several rows keeps its number and Remove on its first row
        ui.horizontal_top(|ui| {
            let ipath = format!("{path}[{i}]");
            ui.label(format!("{}", i + 1));
            if item_kind == Kind::ObjRef {
                changed |= object_picker(ui, v, &item_kinds, &ipath, ctx, label);
            } else {
                changed |= value_widget(ui, root, &item_schema, v, name, &ipath, ctx, label);
            }
            if items_len_over_min && ui.small_button(t("form.remove")).clicked() {
                remove = Some(i);
            }
        });
        error_line(ui, ctx, &format!("{path}[{i}]"));
    }
    if let Some(i) = remove {
        items.remove(i);
        changed = true;
    }
    ui.horizontal(|ui| {
        if items.len() < max && ui.small_button(t("form.add")).clicked() {
            items.push(start_value(root, &item_schema, name, &item_kind, ctx));
            changed = true;
        }
        let is_point = is_tuple(&item_schema, root) && item_schema.get("minItems").and_then(Value::as_u64) == Some(3);
        // what the viewport fills is said by the schema's marker, not by the parameter's name
        let mode = PickMode::of(schema).map(PickMode::kind);
        if mode == Some(PickKind::Points) && is_point {
            pick_button(ui, ctx, name, path, PickKind::Points);
        }
        if mode == Some(PickKind::Edges) && item_kind == Kind::Int {
            pick_button(ui, ctx, name, path, PickKind::Edges);
        }
        if is_point && !ctx.pick_points.is_empty() && ui.small_button(t("form.use_picked_points")).clicked() {
            *items = ctx.pick_points.iter().map(|p| json!(p)).collect();
            changed = true;
        }
        if item_kind == Kind::Int
            && name.contains("triangle")
            && !ctx.pick_triangles.is_empty()
            && ui.small_button(t("form.use_picked_triangles")).clicked()
        {
            *items = ctx.pick_triangles.iter().map(|x| json!(x)).collect();
            changed = true;
        }
        if item_kind == Kind::ObjRef
            && !ctx.pick_objects.is_empty()
            && ui.small_button(t("form.use_picked_objects")).clicked()
        {
            let mut ids: Vec<u64> = Vec::new();
            for id in ctx.pick_objects {
                if !ids.contains(id) {
                    ids.push(*id);
                }
            }
            *items = ids.into_iter().map(|id| json!(id)).collect();
            changed = true;
        }
    });
    pick_hint_line(ui, ctx, name);
    changed
}

pub(super) fn object_picker(
    ui: &mut egui::Ui,
    value: &mut Value,
    kinds: &[ObjectKind],
    path: &str,
    ctx: &FormCtx<'_>,
    label: Option<egui::Id>,
) -> bool {
    let mut changed = false;
    let fits = |id: u64| ctx.objects.iter().any(|o| o.id == id && (kinds.is_empty() || kinds.contains(&o.kind)));
    let current_id = value.as_u64();
    let describe = |o: &ObjInfo| format!("{} {} (#{})", t(kind_name_key(o.kind)), o.name, o.id);
    let shown = match value {
        Value::Number(n) => {
            n.as_u64().and_then(|id| ctx.objects.iter().find(|o| o.id == id)).map_or_else(|| format!("#{n}"), describe)
        }
        Value::String(s) => s.clone(),
        _ => t("form.none").to_string(),
    };
    // only objects of a kind the parameter takes (and the current one, so a wrong value shows)
    let candidates: Vec<&ObjInfo> = ctx.objects.iter().filter(|o| fits(o.id) || Some(o.id) == current_id).collect();
    let mut chosen: Option<u64> = None;
    let resp = egui::ComboBox::from_id_salt(path).selected_text(shown).width(180.0).show_ui(ui, |ui| {
        if candidates.is_empty() {
            ui.weak(t("form.no_objects"));
        }
        for o in &candidates {
            if ui.selectable_label(current_id == Some(o.id), describe(o)).clicked() {
                chosen = Some(o.id);
            }
        }
    });
    if let Some(l) = label {
        resp.response.labelled_by(l);
    }
    // the shortcuts are offered only when they would change the field to an object that fits
    let offer = |id: u64| current_id != Some(id) && fits(id);
    if let Some(sel) = ctx.selected.filter(|id| offer(*id))
        && ui.small_button(t("form.use_selection")).on_hover_text(t("form.use_selection_hint")).clicked()
    {
        chosen = Some(sel);
    }
    if let Some(p) = ctx.pick_objects.last().copied().filter(|id| offer(*id) && ctx.selected != Some(*id))
        && ui.small_button(t("form.use_pick")).on_hover_text(t("form.use_pick_hint")).clicked()
    {
        chosen = Some(p);
    }
    if let Some(id) = chosen
        && current_id != Some(id)
    {
        *value = json!(id);
        changed = true;
    }
    changed
}

/// Free-form JSON of one value (parameters whose schema has no structure).
pub(super) fn json_text(ui: &mut egui::Ui, value: &mut Value, path: &str, label: Option<egui::Id>) -> bool {
    let id = ui.scope_id().with(("form-json", path));
    let mut text: String = ui.data_mut(|d| d.get_temp(id)).unwrap_or_else(|| value.to_string());
    let mut changed = false;
    let r = ui.add(egui::TextEdit::singleline(&mut text).code_editor().desired_width(160.0));
    if let Some(l) = label {
        r.clone().labelled_by(l);
    }
    if r.changed() {
        if let Ok(v) = serde_json::from_str::<Value>(&text) {
            *value = v;
            changed = true;
        }
        ui.data_mut(|d| d.insert_temp(id, text.clone()));
    } else if !r.has_focus() {
        ui.data_mut(|d| d.remove::<String>(id));
    }
    if serde_json::from_str::<Value>(&text).is_err() {
        ui.label(t("form.err.json"));
    }
    changed
}
