//! The unit of a scan import in the interface (P9-U1). A scan file has no unit, so the import
//! form starts with the unit chosen last time, and a size that looks like a unit mistake is said
//! in words in the interface language.
use crate::i18n::tf;
use serde_json::{Value, json};
use surfacecraft_doc::LengthUnit;

/// The command whose unit the interface remembers.
pub const IMPORT: &str = "mesh.import";
/// The parameter that carries the unit.
const UNITS: &str = "units";

/// `params` of `command` with the remembered unit put in, so that the form starts with it.
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

/// The unit a parameter set of `command` chose, when it names one that is known.
#[must_use]
pub fn chosen(command: &str, params: &Value) -> Option<LengthUnit> {
    if command != IMPORT {
        return None;
    }
    params.get(UNITS).and_then(|v| serde_json::from_value(v.clone()).ok())
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
    fn the_form_starts_with_the_remembered_unit_for_an_import_only() {
        let params = json!({"path": "a.stl", "units": null});
        let import = prefilled(IMPORT, params.clone(), LengthUnit::In);
        assert_eq!(import["units"], "in");
        assert_eq!(prefilled("mesh.info", params.clone(), LengthUnit::In), params);
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
}
