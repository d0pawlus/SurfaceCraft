//! Formatting of the IGES text: numbers, names, vector helpers and the wrapping of fixed-column fields.

use super::*;

pub(super) fn bad(m: impl Into<String>) -> IoError {
    IoError::Export(format!("IGES: {}", m.into()))
}

pub(super) fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
pub(super) fn dot(a: V3, b: V3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
pub(super) fn cross(a: V3, b: V3) -> V3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// A real in IGES free format (always with a decimal point; `{:?}` round-trips exactly).
pub(super) fn real(x: f64) -> String {
    let x = if x.is_finite() { x } else { 0.0 };
    let s = format!("{x:?}").replace('e', "E");
    if s.contains('.') {
        s
    } else if let Some((m, e)) = s.split_once('E') {
        format!("{m}.E{e}")
    } else {
        format!("{s}.")
    }
}

/// A real from the input: refused unless finite.
pub(super) fn num(x: f64) -> Result<String, IoError> {
    if x.is_finite() { Ok(real(x)) } else { Err(bad("non-finite number")) }
}

/// A Hollerith string (non-printable or non-ASCII characters replaced by `_`).
pub(super) fn holl(s: &str) -> String {
    let clean: String = s.chars().map(|c| if c.is_ascii() && !c.is_ascii_control() { c } else { '_' }).collect();
    format!("{}H{clean}", clean.len())
}

/// Fields joined by `delim`, ended by `end`, broken into lines of at most `width` columns
/// between fields (a longer field is split, as strings may be).
pub(super) fn wrap(fields: &[&str], delim: char, end: char, width: usize) -> Vec<String> {
    let mut lines = Vec::new();
    let mut cur = String::new();
    for (i, f) in fields.iter().enumerate() {
        let sep = if i + 1 == fields.len() { end } else { delim };
        let item = format!("{f}{sep}");
        if cur.len() + item.len() > width && !cur.is_empty() {
            lines.push(std::mem::take(&mut cur));
        }
        if item.len() > width {
            let chars: Vec<char> = item.chars().collect();
            for c in chars.chunks(width) {
                let piece: String = c.iter().collect();
                if piece.len() == width {
                    lines.push(piece);
                } else {
                    cur = piece;
                }
            }
            continue;
        }
        cur.push_str(&item);
    }
    if !cur.is_empty() {
        lines.push(cur);
    }
    lines
}
