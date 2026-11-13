//! The unit of a scan import in the interface (P9-U1). A scan file has no unit, so an import never
//! takes a unit silently: imports started without the form (menu, welcome card, drag-and-drop, the
//! browser's inbox) are always millimetres. The import form starts with the unit chosen last in this
//! session (never kept across a restart), and says so. What an import or a conversion did is said in
//! words in the interface language.
use crate::i18n::{t, tf};
use serde_json::{Value, json};
use surfacecraft_doc::LengthUnit;

/// The command whose unit the interface remembers.
pub const IMPORT: &str = "mesh.import";
/// The parameter that carries the unit.
const UNITS: &str = "units";

/// `params` of `command` with the unit of this session put in, so that the form starts with it.
#[must_use]
pub fn prefilled(command: &str, params: Value, remembered: LengthUnit) -> Value {
    match params {
        Value::Object(mut map) if command == IMPORT => {
            map.insert(UNITS.to_string(), json!(remembered));
            Value::Object(map)
        }
        other => other,
    }
}

/// Parameters of an import started without the form: always millimetres, never the unit of an
/// earlier choice.
#[must_use]
pub fn file_import_params(params: Value) -> Value {
    match params {
        Value::Object(mut map) => {
            map.insert(UNITS.to_string(), json!(LengthUnit::Mm));
            Value::Object(map)
        }
        other => other,
    }
}

/// The unit a parameter set of `command` chose, when it names one that is known.
#[must_use]
pub fn chosen(command: &str, params: &Value) -> Option<LengthUnit> {
    if command != IMPORT {
        return None;
    }
    params.get(UNITS).and_then(|v| serde_json::from_value(v.clone()).ok())
}

/// The unit's name in the interface language (its enum label).
fn unit_name(unit: LengthUnit) -> String {
    t(&format!("enum.{}", unit.symbol())).to_string()
}

/// The status line of a form whose unit is not millimetres: the unit is shown before the import.
#[must_use]
pub fn form_note(remembered: LengthUnit) -> Option<String> {
    (!remembered.is_mm()).then(|| {
        let factor = remembered.mm_per_unit().to_string();
        tf("units.form_default", &[("unit", &unit_name(remembered)), ("factor", &factor)])
    })
}

/// What an import or a conversion did with the unit, in words; `None` when it did nothing.
#[must_use]
pub fn result_note(command: &str, result: &Value) -> Option<String> {
    let unit: LengthUnit =
        serde_json::from_value(result.get("source_units").or_else(|| result.get("from"))?.clone()).ok()?;
    let factor = result.get("scale_factor").or_else(|| result.get("factor"))?.as_f64()?;
    let factor_text = factor.to_string();
    match command {
        "mesh.import" if !unit.is_mm() || factor != 1.0 => {
            Some(tf("units.imported", &[("unit", &unit_name(unit)), ("factor", &factor_text)]))
        }
        "mesh.set_units" if factor == 1.0 => Some(tf("units.acknowledged", &[("unit", &unit_name(unit))])),
        "mesh.set_units" => Some(tf("units.converted", &[("unit", &unit_name(unit)), ("factor", &factor_text)])),
        _ => None,
    }
}

/// The warning of a result (`scale_warning`) in the interface language: the size, and what to do.
/// `None` when the result carries none.
#[must_use]
pub fn warning_text(result: &Value) -> Option<String> {
    let warning = result.get("scale_warning")?;
    let key = match warning.get("problem")?.as_str()? {
        "metres_or_inches" => "units.warning.metres",
        "not_millimetres" => "units.warning.tiny",
        "micrometres" => "units.warning.micro",
        _ => return None,
    };
    let extent = format!("{:.4}", warning.get("extent")?.as_f64()?);
    Some(tf(key, &[("extent", extent.as_str())]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_form_starts_with_the_session_unit_for_an_import_only() {
        let params = json!({"path": "a.stl", "units": null});
        assert_eq!(prefilled(IMPORT, params.clone(), LengthUnit::In)["units"], "in");
        assert_eq!(prefilled("mesh.info", params.clone(), LengthUnit::In), params);
    }

    #[test]
    fn a_menu_or_inbox_import_is_always_millimetres() {
        assert_eq!(file_import_params(json!({"path": "a.stl"}))["units"], "mm");
        assert_eq!(file_import_params(json!({"data": "x", "units": "m"}))["units"], "mm");
    }

    #[test]
    fn the_chosen_unit_is_read_back_when_it_is_known() {
        assert_eq!(chosen(IMPORT, &json!({"units": "um"})), Some(LengthUnit::Um));
        assert_eq!(chosen(IMPORT, &json!({"units": "yd"})), None);
        assert_eq!(chosen(IMPORT, &json!({})), None);
    }

    #[test]
    fn a_warning_is_said_in_words_with_its_size() {
        let result = json!({"scale_warning": {"problem": "metres_or_inches", "extent": 0.045}});
        let text = warning_text(&result).unwrap();
        assert!(text.contains("0.0450"), "{text}");
        assert_eq!(warning_text(&json!({"id": 1})), None);
    }

    #[test]
    fn only_a_unit_that_changed_something_is_announced() {
        let metres = json!({"source_units": "m", "scale_factor": 1.0});
        assert!(result_note("mesh.import", &metres).is_some());
        let plain = json!({"source_units": "mm", "scale_factor": 1.0});
        assert_eq!(result_note("mesh.import", &plain), None);
    }
}
