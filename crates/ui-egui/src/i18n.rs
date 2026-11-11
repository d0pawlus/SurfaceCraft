//! Interface translations: every UI string is a key in `assets/locales/<lang>.json`.
//!
//! Lookup never blocks and never fails: a key missing in the chosen language falls back to
//! English, and a key missing there too shows the key itself, so a gap in a catalogue is visible
//! but never an error. The catalogues are embedded; a catalogue that does not parse is empty.
use serde::{Deserialize, Serialize};
use std::cell::Cell;
use std::collections::HashMap;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, Ordering};

/// An interface language.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum Lang {
    /// English (the fallback of every other language).
    #[default]
    En,
    /// Polish.
    Pl,
}

impl Lang {
    /// All languages, in menu order.
    pub const ALL: [Lang; 2] = [Lang::En, Lang::Pl];

    /// The catalogue key of this language's own name (`lang.en`, `lang.pl`).
    #[must_use]
    pub fn name_key(self) -> &'static str {
        match self {
            Lang::En => "lang.en",
            Lang::Pl => "lang.pl",
        }
    }

    /// The language of the desktop session (`LC_ALL`, `LC_MESSAGES`, `LANG`; English on the web
    /// and when none is set).
    #[must_use]
    pub fn from_system() -> Lang {
        #[cfg(not(target_arch = "wasm32"))]
        {
            for var in ["LC_ALL", "LC_MESSAGES", "LANG"] {
                if let Ok(v) = std::env::var(var)
                    && !v.is_empty()
                {
                    return Lang::from_locale(&v);
                }
            }
        }
        Lang::En
    }

    /// The language of a locale tag such as `pl_PL.UTF-8` or `en-US` (English when unknown).
    #[must_use]
    pub fn from_locale(tag: &str) -> Lang {
        if tag.trim().to_ascii_lowercase().starts_with("pl") { Lang::Pl } else { Lang::En }
    }
}

type Catalog = HashMap<String, String>;

const EN_JSON: &str = include_str!("../assets/locales/en.json");
const PL_JSON: &str = include_str!("../assets/locales/pl.json");

/// Polish texts of the engine's parameter descriptions, keyed by the English description.
const HINTS_PL_JSON: &str = include_str!("../assets/locales/hints.pl.json");
/// Polish templates of the engine's messages, keyed by the English template (`{}` placeholders).
const ERRORS_PL_JSON: &str = include_str!("../assets/locales/errors.pl.json");

fn parse(json: &str) -> Catalog {
    serde_json::from_str(json).unwrap_or_default()
}

/// The Polish texts of the engine's parameter descriptions, by English description.
#[must_use]
pub fn hints_pl() -> &'static HashMap<String, String> {
    static HINTS: OnceLock<Catalog> = OnceLock::new();
    HINTS.get_or_init(|| parse(HINTS_PL_JSON))
}

/// A message template of the engine with its Polish text. `parts` are the English literals
/// between the placeholders (one more than there are placeholders).
struct MessageTemplate {
    parts: Vec<String>,
    text: String,
    /// Letters in the literals: the more, the more specific the template.
    weight: usize,
}

/// A template needs this many letters outside its placeholders to be matched at all, so that
/// `{}: {}` or `{} {}` never swallow an unrelated message.
const MIN_TEMPLATE_LETTERS: usize = 3;
/// How deep a value that is itself a message ("invalid parameters: no mesh 5") is translated.
const MAX_MESSAGE_DEPTH: u8 = 3;

fn compile_templates(map: &Catalog) -> Vec<MessageTemplate> {
    let mut out: Vec<MessageTemplate> = map
        .iter()
        .filter_map(|(english, text)| {
            let parts: Vec<String> = english.split("{}").map(str::to_string).collect();
            let weight = parts.iter().map(|p| p.chars().filter(|c| c.is_alphabetic()).count()).sum();
            let same_placeholders = text.matches("{}").count() + 1 == parts.len();
            (weight >= MIN_TEMPLATE_LETTERS && same_placeholders).then(|| MessageTemplate {
                parts,
                text: text.clone(),
                weight,
            })
        })
        .collect();
    // the most specific template first; the text breaks ties so the order never depends on the map
    out.sort_by(|a, b| b.weight.cmp(&a.weight).then_with(|| a.parts.cmp(&b.parts)));
    out
}

fn message_templates_pl() -> &'static Vec<MessageTemplate> {
    static TEMPLATES: OnceLock<Vec<MessageTemplate>> = OnceLock::new();
    TEMPLATES.get_or_init(|| compile_templates(&parse(ERRORS_PL_JSON)))
}

/// The values `message` has where the template has placeholders; `None` when the message is not
/// of this template.
fn captures<'a>(parts: &[String], message: &'a str) -> Option<Vec<&'a str>> {
    let (first, rest) = parts.split_first()?;
    let mut tail = message.strip_prefix(first.as_str())?;
    let Some((last, middle)) = rest.split_last() else {
        return tail.is_empty().then(Vec::new);
    };
    let mut values = Vec::with_capacity(rest.len());
    for part in middle {
        // two placeholders with nothing between them cannot be told apart
        let (value, after) = if part.is_empty() { None } else { tail.split_once(part.as_str()) }?;
        values.push(value);
        tail = after;
    }
    values.push(tail.strip_suffix(last.as_str())?);
    values.iter().all(|v| !v.is_empty()).then_some(values)
}

fn translate_message(templates: &[MessageTemplate], message: &str, depth: u8) -> Option<String> {
    let (template, values) = templates.iter().find_map(|t| Some((t, captures(&t.parts, message)?)))?;
    let mut out = String::new();
    for (i, piece) in template.text.split("{}").enumerate() {
        out.push_str(piece);
        if let Some(value) = values.get(i) {
            let inner = (depth < MAX_MESSAGE_DEPTH).then(|| translate_message(templates, value, depth + 1)).flatten();
            out.push_str(inner.as_deref().unwrap_or(value));
        }
    }
    Some(out)
}

/// A message of the engine (an error, a note) in the interface language. The engine writes its
/// messages in English; in Polish the message is matched against the known templates and the
/// values put into the Polish one. A message no template knows is shown as the engine wrote it.
#[must_use]
pub fn engine_message(message: &str) -> String {
    match lang() {
        Lang::Pl => translate_message(message_templates_pl(), message, 0).unwrap_or_else(|| message.to_string()),
        Lang::En => message.to_string(),
    }
}

/// The Polish templates of the engine's messages, by English template (for the tests).
#[must_use]
pub fn engine_messages_pl() -> HashMap<String, String> {
    parse(ERRORS_PL_JSON)
}

/// A parameter description of the engine in the interface language. The engine writes them in
/// English with parameter names in backticks; the English text is shown without the backticks.
#[must_use]
pub fn hint(description: &str) -> String {
    match lang() {
        Lang::Pl => hints_pl().get(description).map_or_else(|| description.replace('`', ""), String::clone),
        Lang::En => description.replace('`', ""),
    }
}

/// Whether the keyboard has a Command key where others have Control (macOS, also in a browser).
static MAC_KEYS: AtomicBool = AtomicBool::new(cfg!(target_os = "macos"));

/// Tells the catalogues which keyboard the user has: texts that name the command key
/// (`{cmd}+K`) say "Cmd" on a Mac and "Ctrl" elsewhere. The desktop build knows from its target;
/// the web build is told by the shell, which asks the browser.
pub fn set_mac_keys(mac: bool) {
    MAC_KEYS.store(mac, Ordering::Relaxed);
}

/// A catalogue with the command key of the keyboard written into its texts.
fn parse_catalog(json: &str, mac: bool) -> Catalog {
    let key = if mac { "Cmd" } else { "Ctrl" };
    parse(json).into_iter().map(|(k, text)| (k, text.replace("{cmd}", key))).collect()
}

/// The embedded catalogue of a language.
#[must_use]
pub fn catalog(lang: Lang) -> &'static HashMap<String, String> {
    static EN: OnceLock<Catalog> = OnceLock::new();
    static PL: OnceLock<Catalog> = OnceLock::new();
    static EN_MAC: OnceLock<Catalog> = OnceLock::new();
    static PL_MAC: OnceLock<Catalog> = OnceLock::new();
    match (lang, MAC_KEYS.load(Ordering::Relaxed)) {
        (Lang::En, false) => EN.get_or_init(|| parse_catalog(EN_JSON, false)),
        (Lang::Pl, false) => PL.get_or_init(|| parse_catalog(PL_JSON, false)),
        (Lang::En, true) => EN_MAC.get_or_init(|| parse_catalog(EN_JSON, true)),
        (Lang::Pl, true) => PL_MAC.get_or_init(|| parse_catalog(PL_JSON, true)),
    }
}

thread_local! {
    static CURRENT: Cell<Lang> = const { Cell::new(Lang::En) };
}

/// Sets the language `t` uses on this (the UI) thread.
pub fn set_lang(lang: Lang) {
    CURRENT.with(|c| c.set(lang));
}

/// The language `t` uses on this thread.
#[must_use]
pub fn lang() -> Lang {
    CURRENT.with(Cell::get)
}

/// The text of `key` in `lang`, with the English fallback and the key as the last resort.
#[must_use]
pub fn lookup(lang: Lang, key: &str) -> &str {
    catalog(lang).get(key).or_else(|| catalog(Lang::En).get(key)).map_or(key, String::as_str)
}

/// The text of `key` in the current language.
#[must_use]
pub fn t(key: &str) -> &str {
    lookup(lang(), key)
}

/// The text of `key` in the current language or English, `None` when neither has it.
#[must_use]
pub fn find(key: &str) -> Option<&str> {
    catalog(lang()).get(key).or_else(|| catalog(Lang::En).get(key)).map(String::as_str)
}

/// The text of `key` with each `{name}` replaced by the matching argument.
#[must_use]
pub fn tf(key: &str, args: &[(&str, &str)]) -> String {
    let mut s = t(key).to_string();
    for (name, value) in args {
        s = s.replace(&format!("{{{name}}}"), value);
    }
    s
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    fn templates(pairs: &[(&str, &str)]) -> Vec<MessageTemplate> {
        compile_templates(&pairs.iter().map(|(k, v)| ((*k).to_string(), (*v).to_string())).collect())
    }

    #[test]
    fn an_engine_message_is_matched_to_its_template_and_keeps_its_values() {
        let t = templates(&[
            ("no mesh {}", "nie ma skanu {}"),
            ("invalid parameters: {}", "nieprawidlowe parametry: {}"),
            ("{} is a {}, not a {}", "{} to {}, a nie {}"),
            ("tolerance must be positive", "tolerancja musi byc dodatnia"),
            ("{}: {}", "{} - {}"),
        ]);
        assert_eq!(translate_message(&t, "no mesh 5", 0).as_deref(), Some("nie ma skanu 5"));
        assert_eq!(
            translate_message(&t, "tolerance must be positive", 0).as_deref(),
            Some("tolerancja musi byc dodatnia")
        );
        assert_eq!(translate_message(&t, "#3 is a Body, not a Mesh", 0).as_deref(), Some("#3 to Body, a nie Mesh"));
        // a message inside a message is translated too
        assert_eq!(
            translate_message(&t, "invalid parameters: no mesh scan", 0).as_deref(),
            Some("nieprawidlowe parametry: nie ma skanu scan")
        );
        // nothing matches: the caller shows the engine's own words; a template that is all
        // placeholders is never used
        assert_eq!(translate_message(&t, "the solver did not converge", 0), None);
        assert_eq!(translate_message(&t, "Fill holes: done", 0), None);
        assert_eq!(translate_message(&t, "no mesh ", 0), None, "an empty value is no match");
    }

    #[test]
    fn the_polish_templates_keep_the_placeholders_of_the_english_ones() {
        let all = engine_messages_pl();
        assert!(all.len() > 200, "the message catalogue is embedded: {}", all.len());
        for (english, polish) in &all {
            assert_eq!(english.matches("{}").count(), polish.matches("{}").count(), "{english} / {polish}");
        }
        set_lang(Lang::Pl);
        let shown = engine_message("no mesh 5");
        set_lang(Lang::En);
        assert!(shown.contains('5') && !shown.contains("no mesh"), "{shown}");
        assert_eq!(engine_message("no mesh 5"), "no mesh 5");
    }

    #[test]
    fn the_command_key_is_named_after_the_keyboard() {
        for lang in Lang::ALL {
            for (mac, key) in [(false, "Ctrl+K"), (true, "Cmd+K")] {
                let hint = parse_catalog(if lang == Lang::En { EN_JSON } else { PL_JSON }, mac);
                let text = hint.get("ribbon.search_hint").expect("search hint");
                assert!(text.contains(key), "{lang:?} mac={mac}: {text}");
                assert!(hint.values().all(|t| !t.contains("{cmd}")), "{lang:?}: a text still has the placeholder");
            }
            // no text names the Control key outright
            let raw = parse(if lang == Lang::En { EN_JSON } else { PL_JSON });
            assert!(raw.values().all(|t| !t.contains("Ctrl")), "{lang:?}: write {{cmd}} instead of Ctrl");
        }
    }

    #[test]
    fn a_hint_is_shown_in_the_interface_language_without_backticks() {
        assert_eq!(hint("Moves `object` to the origin."), "Moves object to the origin.");
        assert!(hints_pl().len() > 200, "the hint catalogue is embedded: {}", hints_pl().len());
    }
}
