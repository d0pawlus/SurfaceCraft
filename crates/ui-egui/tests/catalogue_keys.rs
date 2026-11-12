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
