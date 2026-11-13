//! Structural shortening of the compact listing: prose of the input schemas of the commands
//! listed in full is reduced, so that command authors never trim their docs to fit a budget.
//! Names, types, enum values, defaults and `required` lists are never touched; the full text
//! stays available from `describe_command` and in every parameter error.
use serde_json::{Map, Value};

/// Hard cap (characters, "…" included) of one shortened description.
pub(crate) const MAX_DESCRIPTION_CHARS: usize = 120;
/// Hard cap (characters) of one command summary in an area tool's verb list.
pub(crate) const MAX_SUMMARY_CHARS: usize = 80;
/// Abbreviations whose full stop does not end a sentence.
const ABBREVIATIONS: &[&str] = &["e.g", "i.e", "vs", "cf", "etc", "approx"];
const ELLIPSIS: char = '…';

/// Whether the full stop at byte `at` of `text` ends a sentence: it is followed by white space or
/// the end, the text after it does not go on in lower case or with a digit ("1. cut 2. fill",
/// "no. 5"), and it does not close an abbreviation.
fn ends_sentence(text: &str, at: usize) -> bool {
    let after = text.get(at + 1..).unwrap_or_default();
    if after.chars().next().is_some_and(|c| !c.is_whitespace()) {
        return false;
    }
    if after.trim_start().chars().next().is_some_and(|c| c.is_lowercase() || c.is_ascii_digit()) {
        return false;
    }
    let before = text.get(..at).unwrap_or_default();
    let word = before.rsplit(char::is_whitespace).next().unwrap_or_default();
    let word = word.trim_start_matches(|c: char| !c.is_alphanumeric()).to_lowercase();
    !ABBREVIATIONS.contains(&word.as_str())
}

/// The first sentence of `text`: up to the first ". " (or the end), never inside backticks and
/// never after an abbreviation such as "e.g.".
pub(crate) fn first_sentence(text: &str) -> &str {
    let text = text.trim();
    let mut in_code = false;
    for (i, c) in text.char_indices() {
        match c {
            '`' => in_code = !in_code,
            '.' if !in_code && ends_sentence(text, i) => return text.get(..=i).unwrap_or(text),
            _ => {}
        }
    }
    text
}

/// Where to cut `text` so that at most `budget` characters remain: at a word boundary when there
/// is one, and never inside a backtick span.
fn cut_point(text: &str, budget: usize) -> usize {
    let hard = text.char_indices().nth(budget).map_or(text.len(), |(i, _)| i);
    let head = text.get(..hard).unwrap_or_default();
    let by_word = head.rfind(char::is_whitespace).filter(|&i| i > 0).unwrap_or(hard);
    let open_code = text.get(..by_word).unwrap_or_default().matches('`').count() % 2 == 1;
    let outside_code = if open_code { text.get(..by_word).and_then(|h| h.rfind('`')).unwrap_or(0) } else { by_word };
    if outside_code > 0 { outside_code } else { by_word }
}

/// `text` as its first sentence of at most `cap` characters; "…" marks a cut inside a sentence.
pub(crate) fn shorten_description(text: &str, cap: usize) -> String {
    let sentence = first_sentence(text);
    if sentence.chars().count() <= cap {
        return sentence.to_string();
    }
    let cut = cut_point(sentence, cap.saturating_sub(1));
    let head =
        sentence.get(..cut).unwrap_or_default().trim_end_matches(|c: char| c.is_whitespace() || ",;:".contains(c));
    format!("{}{ELLIPSIS}", balanced(head))
}

/// `head` without its last backtick when it opens a code span the cut left unclosed (a span longer
/// than the cap that starts the sentence), so the output never carries a dangling backtick.
fn balanced(head: &str) -> String {
    if head.matches('`').count().is_multiple_of(2) {
        return head.to_string();
    }
    let Some(at) = head.rfind('`') else { return head.to_string() };
    let (before, after) = head.split_at(at);
    format!("{before}{}", after.get(1..).unwrap_or_default())
}

/// Keywords whose value is one schema.
const SCHEMA_KEYS: &[&str] = &["items", "additionalProperties", "not", "if", "then", "else"];
/// Keywords whose value is a list of schemas.
const SCHEMA_LISTS: &[&str] = &["anyOf", "oneOf", "allOf", "prefixItems"];
/// Keywords whose value maps names to schemas.
const SCHEMA_MAPS: &[&str] = &["properties", "patternProperties", "$defs", "definitions"];

fn walk_all<'a>(items: impl Iterator<Item = &'a Value>, depth: usize) -> (Vec<Value>, usize) {
    items.map(|v| walk(v, depth)).fold((Vec::new(), 0), |(mut list, n), (v, c)| {
        list.push(v);
        (list, n + c)
    })
}

/// `schema` with its descriptions below the top level shortened, and the number of cuts.
fn walk(schema: &Value, depth: usize) -> (Value, usize) {
    let Value::Object(map) = schema else { return (schema.clone(), 0) };
    let mut out = Map::new();
    let mut cuts = 0;
    for (key, value) in map {
        let key = key.as_str();
        let (new, n) = match value {
            Value::String(text) if key == "description" && depth > 0 => {
                let short = shorten_description(text, MAX_DESCRIPTION_CHARS);
                let cut = short != *text;
                (Value::String(short), usize::from(cut))
            }
            Value::Object(inner) if SCHEMA_MAPS.contains(&key) => {
                let (list, n) = walk_all(inner.values(), depth + 1);
                (Value::Object(inner.keys().cloned().zip(list).collect()), n)
            }
            Value::Object(_) if SCHEMA_KEYS.contains(&key) => walk(value, depth + 1),
            Value::Array(inner) if SCHEMA_LISTS.contains(&key) => {
                let (list, n) = walk_all(inner.iter(), depth + 1);
                (Value::Array(list), n)
            }
            other => (other.clone(), 0),
        };
        out.insert(key.to_string(), new);
        cuts += n;
    }
    (Value::Object(out), cuts)
}

/// A copy of `schema` with every property description shortened, and how many were cut.
pub(crate) fn shorten_schema(schema: &Value) -> (Value, usize) {
    walk(schema, 0)
}
