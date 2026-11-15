//! Every locale file names each key once. serde_json keeps the last of two equal keys without a
//! word, so a duplicate silently replaces the text before it.
use serde::Deserialize;
use serde::de::{self, IgnoredAny, MapAccess, Visitor};
use std::collections::BTreeMap;

/// The catalogue files of `assets/locales`, with their raw text.
const LOCALES: [(&str, &str); 4] = [
    ("en.json", include_str!("../assets/locales/en.json")),
    ("pl.json", include_str!("../assets/locales/pl.json")),
    ("hints.pl.json", include_str!("../assets/locales/hints.pl.json")),
    ("errors.pl.json", include_str!("../assets/locales/errors.pl.json")),
];

/// The top-level keys of a JSON object, in the order they appear in the text (repeats included).
struct KeysInOrder(Vec<String>);

impl<'de> Deserialize<'de> for KeysInOrder {
    fn deserialize<D: serde::Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
        struct Keys;
        impl<'de> Visitor<'de> for Keys {
            type Value = KeysInOrder;
            fn expecting(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
                f.write_str("a JSON object")
            }
            fn visit_map<A: MapAccess<'de>>(self, mut map: A) -> Result<KeysInOrder, A::Error> {
                let mut keys = Vec::new();
                while let Some(key) = map.next_key::<String>()? {
                    map.next_value::<IgnoredAny>()?;
                    keys.push(key);
                }
                Ok(KeysInOrder(keys))
            }
        }
        deserializer.deserialize_map(Keys).map_err(de::Error::custom)
    }
}

/// The keys that occur more than once in `json`, with how often each occurs.
fn repeated_keys(json: &str) -> BTreeMap<String, usize> {
    let keys: KeysInOrder = serde_json::from_str(json).expect("a JSON object");
    let mut count: BTreeMap<String, usize> = BTreeMap::new();
    for key in keys.0 {
        *count.entry(key).or_insert(0) += 1;
    }
    count.retain(|_, n| *n > 1);
    count
}

#[test]
fn no_locale_file_names_a_key_twice() {
    let mut report = Vec::new();
    for (file, text) in LOCALES {
        for (key, n) in repeated_keys(text) {
            report.push(format!("{file}: {key} x{n}"));
        }
    }
    assert!(report.is_empty(), "keys named more than once (the last one silently wins):\n{}", report.join("\n"));
}

/// The first key that does not come strictly after the one before it (byte order, which is the
/// order of `str::cmp` and of `BTreeMap`), as `(previous, key)`. A repeated key is out of order too.
fn first_out_of_order(keys: &[String]) -> Option<(&str, &str)> {
    keys.windows(2).find(|w| w[0] >= w[1]).map(|w| (w[0].as_str(), w[1].as_str()))
}

/// `errors.pl.json` stays sorted by key, so that a new template lands in one place and a merge of two
/// branches that both add templates conflicts only where they really touch. The lookup does not read
/// the order (`i18n::compile_templates` sorts the templates by specificity itself), so this is a
/// matter of the file, not of the translation chosen.
#[test]
fn errors_pl_json_is_sorted_by_key() {
    let (_, text) = LOCALES.iter().find(|(file, _)| *file == "errors.pl.json").expect("the file is listed");
    let keys: KeysInOrder = serde_json::from_str(text).expect("a JSON object");
    assert!(keys.0.len() > 600, "the catalogue was read: {} keys", keys.0.len());
    let broken = first_out_of_order(&keys.0);
    assert!(
        broken.is_none(),
        "keys must be in byte order, without repeats; {:?} is followed by {:?}",
        broken.map(|b| b.0),
        broken.map(|b| b.1)
    );
}

#[test]
fn the_order_check_finds_a_key_out_of_place_and_a_repeat() {
    let keys = |list: &[&str]| list.iter().map(|k| (*k).to_string()).collect::<Vec<_>>();
    assert_eq!(first_out_of_order(&keys(&["a", "b", "c"])), None);
    assert_eq!(first_out_of_order(&keys(&["a", "c", "b"])), Some(("c", "b")));
    assert_eq!(first_out_of_order(&keys(&["a", "b", "b"])), Some(("b", "b")));
    assert_eq!(first_out_of_order(&keys(&[])), None);
}
