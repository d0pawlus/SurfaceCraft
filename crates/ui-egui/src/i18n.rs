//! Interface translations: every UI string is a key in `assets/locales/<lang>.json`.
//!
//! Lookup never blocks and never fails: a key missing in the chosen language falls back to
//! English, and a key missing there too shows the key itself, so a gap in a catalogue is visible
//! but never an error. The catalogues are embedded; a catalogue that does not parse is empty.
use serde::{Deserialize, Serialize};
use std::cell::Cell;
use std::collections::HashMap;
use std::sync::OnceLock;

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

    /// The language of a locale tag such as `pl_PL.UTF-8` or `en-US` (English when unknown).
    #[must_use]
    pub fn from_locale(tag: &str) -> Lang {
        if tag.trim().to_ascii_lowercase().starts_with("pl") { Lang::Pl } else { Lang::En }
    }
}

type Catalog = HashMap<String, String>;

const EN_JSON: &str = include_str!("../assets/locales/en.json");
const PL_JSON: &str = include_str!("../assets/locales/pl.json");

fn parse(json: &str) -> Catalog {
    serde_json::from_str(json).unwrap_or_default()
}

/// The embedded catalogue of a language.
#[must_use]
pub fn catalog(lang: Lang) -> &'static HashMap<String, String> {
    static EN: OnceLock<Catalog> = OnceLock::new();
    static PL: OnceLock<Catalog> = OnceLock::new();
    match lang {
        Lang::En => EN.get_or_init(|| parse(EN_JSON)),
        Lang::Pl => PL.get_or_init(|| parse(PL_JSON)),
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

/// The text of `key` with each `{name}` replaced by the matching argument.
#[must_use]
pub fn tf(key: &str, args: &[(&str, &str)]) -> String {
    let mut s = t(key).to_string();
    for (name, value) in args {
        s = s.replace(&format!("{{{name}}}"), value);
    }
    s
}
