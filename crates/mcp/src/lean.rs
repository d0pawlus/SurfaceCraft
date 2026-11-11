//! Lean tool results: what an agent reads costs tokens, so numbers carry the digits that mean
//! something and the text is compact JSON. Also the hints added to an error, so that the next
//! call can be right without a round trip to `describe_command`.
use serde_json::{Map, Number, Value, json};
use surfacecraft_engine::cmd::find;

/// Decimals kept in a float of magnitude 1e-3 or more (millimetres: a nanometre).
const DECIMALS: i32 = 6;
/// Significant digits kept in a smaller float.
const SMALL_DIGITS: usize = 6;
/// Below this magnitude a float keeps significant digits instead of decimals.
const SMALL: f64 = 1e-3;
/// Most names offered in `did_you_mean`.
const MAX_SUGGESTIONS: usize = 3;
/// Characters of a wanted name that are compared.
const MAX_NAME_CHARS: usize = 64;
/// Largest edit distance of a suggestion.
const MAX_DISTANCE: usize = 3;

/// `x` with the digits that carry meaning: 6 decimals, or 6 significant digits below 1e-3.
fn round_float(x: f64) -> f64 {
    if !x.is_finite() || x == 0.0 {
        return x;
    }
    if x.abs() >= SMALL {
        let scale = 10f64.powi(DECIMALS);
        return (x * scale).round() / scale;
    }
    format!("{x:.*e}", SMALL_DIGITS - 1).parse().unwrap_or(x)
}

/// A copy of `v` with every float rounded (see [`round_float`]). Integers, strings and
/// structure are untouched.
#[must_use]
pub fn round(v: &Value) -> Value {
    match v {
        Value::Number(n) if n.is_f64() => {
            n.as_f64().map(round_float).and_then(Number::from_f64).map_or_else(|| v.clone(), Value::Number)
        }
        Value::Array(list) => Value::Array(list.iter().map(round).collect()),
        Value::Object(map) => Value::Object(map.iter().map(|(k, x)| (k.clone(), round(x))).collect::<Map<_, _>>()),
        other => other.clone(),
    }
}

/// A tool result carrying `v`: compact JSON text and the same value as structured content.
#[must_use]
pub fn text_result(v: &Value, is_error: bool) -> Value {
    let lean = round(v);
    json!({"content": [{"type": "text", "text": lean.to_string()}], "structuredContent": lean, "isError": is_error})
}

/// A tool result reporting `message` as an error.
#[must_use]
pub fn tool_error(message: &str) -> Value {
    text_result(&json!({"error": message}), true)
}

/// A tool result reporting `message` as an error, with the names that were probably meant.
#[must_use]
pub fn tool_error_with(message: &str, wanted: &str, known: &[&str]) -> Value {
    text_result(&json!({"error": message, "did_you_mean": closest(wanted, known)}), true)
}

/// Edit distance between two short names.
fn distance(a: &str, b: &str) -> usize {
    let (a, b): (Vec<char>, Vec<char>) = (a.chars().collect(), b.chars().collect());
    let mut row: Vec<usize> = (0..=b.len()).collect();
    for (i, ca) in a.iter().enumerate() {
        let mut diagonal = row.first().copied().unwrap_or(0);
        if let Some(first) = row.first_mut() {
            *first = i + 1;
        }
        for (j, cb) in b.iter().enumerate() {
            let left = row.get(j).copied().unwrap_or(0);
            let up = row.get(j + 1).copied().unwrap_or(0);
            let cost = diagonal + usize::from(ca != cb);
            diagonal = up;
            if let Some(cell) = row.get_mut(j + 1) {
                *cell = cost.min(left + 1).min(up + 1);
            }
        }
    }
    row.last().copied().unwrap_or(0)
}

/// The names of `known` closest to `wanted`, nearest first: those within a few edits, and those
/// that contain it or are contained in it; at most three.
#[must_use]
pub fn closest<'a>(wanted: &str, known: &[&'a str]) -> Vec<&'a str> {
    // a hostile name can be megabytes long: only its start is compared
    let wanted: String = wanted.chars().take(MAX_NAME_CHARS).collect::<String>().to_lowercase();
    if wanted.is_empty() {
        return Vec::new();
    }
    let length = wanted.chars().count();
    let mut scored: Vec<(usize, &'a str)> = known
        .iter()
        .filter_map(|k| {
            let related = k.contains(wanted.as_str()) || wanted.contains(k);
            let far = length.abs_diff(k.chars().count()) > MAX_DISTANCE;
            if far && !related {
                return None;
            }
            let d = distance(&wanted, k);
            (related || d <= MAX_DISTANCE).then_some((d, *k))
        })
        .collect();
    scored.sort();
    scored.into_iter().map(|(_, k)| k).take(MAX_SUGGESTIONS).collect()
}

/// The error `e` of command `id` with what the caller needs to get the next call right: the
/// input schema after a parameter error, the nearest ids after an unknown command.
#[must_use]
pub fn with_hints(id: &str, e: &Value, command_ids: &[&str]) -> Value {
    let mut out = e.clone();
    let Some(obj) = out.as_object_mut() else { return out };
    match e.get("kind").and_then(Value::as_str) {
        Some("invalid_params") => {
            if let Some(spec) = find(id) {
                obj.insert("command".into(), json!(id));
                obj.insert("input_schema".into(), (spec.input)());
            }
        }
        Some("unknown_command") => {
            obj.insert("did_you_mean".into(), json!(closest(id, command_ids)));
        }
        _ => {}
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rounding_keeps_integers_strings_and_small_values() {
        let v = json!({
            "id": 12,
            "name": "0.123456789012",
            "radius": 5.001_357_425_932_506,
            "tiny": 1.234_567_891_234e-9,
            "list": [30.000_414_260_955_07, -0.000_128_505_876_741_576_13, 2.0],
            "big": 11_752.505_940_268_244,
        });
        let r = round(&v);
        assert_eq!(r["id"], 12);
        assert_eq!(r["name"], "0.123456789012");
        assert_eq!(r["radius"], 5.001_357);
        assert_eq!(r["tiny"], 1.234_57e-9);
        assert_eq!(r["list"], json!([30.000_414, -0.000_128_506, 2.0]));
        assert_eq!(r["big"], 11_752.505_94);
    }

    #[test]
    fn closest_prefers_related_names_and_caps_the_list() {
        let known = ["mesh.import", "mesh.import_stream", "mesh.info", "export.step", "import.step"];
        assert_eq!(closest("mesh.imprt", &known).first(), Some(&"mesh.import"));
        assert!(closest("import", &known).len() <= 3);
        assert!(closest("zzzzzzzzzz", &known).is_empty());
        assert!(closest("", &known).is_empty());
        let huge = "x".repeat(2_000_000);
        assert!(closest(&huge, &known).is_empty(), "a huge name is cut, not compared in full");
    }

    #[test]
    fn a_parameter_error_gets_the_schema_and_an_unknown_command_gets_suggestions() {
        let ids = ["scan.qa", "scan.to_solid"];
        let e = with_hints("scan.qa", &json!({"kind": "invalid_params", "message": "missing field"}), &ids);
        assert_eq!(e["command"], "scan.qa");
        assert!(e["input_schema"]["properties"]["object"].is_object(), "{e}");
        let e = with_hints("scan.to_solids", &json!({"kind": "unknown_command", "message": "x"}), &ids);
        assert_eq!(e["did_you_mean"][0], "scan.to_solid");
        let e = with_hints("scan.qa", &json!({"kind": "failed", "message": "x"}), &ids);
        assert!(e.get("input_schema").is_none());
    }
}
