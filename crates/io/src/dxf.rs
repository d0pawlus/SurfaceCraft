//! ASCII DXF writer: LINE and LWPOLYLINE entities, one layer per drawing layer, millimetres.
//! The header declares AutoCAD 2000 (AC1015) and `$INSUNITS` 4 (millimetres). Written by hand,
//! no dependency.
use crate::{DrawLayer, DrawPath, IoError};

/// Most points one drawing may hold (a guard on the writers' output size).
pub(crate) const MAX_POINTS: usize = 5_000_000;

/// Writes `layers` as an ASCII DXF document: group code and value on separate CRLF lines.
pub fn write_dxf(layers: &[DrawLayer]) -> Result<String, IoError> {
    let points: usize = layers.iter().flat_map(|l| l.paths.iter()).map(|p| p.points.len()).sum();
    if points > MAX_POINTS {
        return Err(IoError::Limit(format!("the drawing has {points} points; the limit is {MAX_POINTS}")));
    }
    let names: Vec<String> = layers.iter().map(|l| layer_name(&l.name)).collect();
    let mut out = String::new();
    pair(&mut out, 0, "SECTION");
    pair(&mut out, 2, "HEADER");
    pair(&mut out, 9, "$ACADVER");
    pair(&mut out, 1, "AC1015");
    pair(&mut out, 9, "$INSUNITS");
    pair(&mut out, 70, "4");
    pair(&mut out, 0, "ENDSEC");
    pair(&mut out, 0, "SECTION");
    pair(&mut out, 2, "TABLES");
    pair(&mut out, 0, "TABLE");
    pair(&mut out, 2, "LAYER");
    pair(&mut out, 70, &names.len().to_string());
    for name in &names {
        pair(&mut out, 0, "LAYER");
        pair(&mut out, 2, name);
        pair(&mut out, 70, "0");
        pair(&mut out, 62, "7");
        pair(&mut out, 6, "CONTINUOUS");
    }
    pair(&mut out, 0, "ENDTAB");
    pair(&mut out, 0, "ENDSEC");
    pair(&mut out, 0, "SECTION");
    pair(&mut out, 2, "ENTITIES");
    for (layer, name) in layers.iter().zip(&names) {
        for path in &layer.paths {
            entity(&mut out, name, path);
        }
    }
    pair(&mut out, 0, "ENDSEC");
    pair(&mut out, 0, "EOF");
    Ok(out)
}

/// One group code line and its value line.
fn pair(out: &mut String, code: u16, value: &str) {
    out.push_str(&code.to_string());
    out.push_str("\r\n");
    out.push_str(value);
    out.push_str("\r\n");
}

/// A two-point open path is a LINE; any other path with two or more points is an LWPOLYLINE
/// (its group 70 flag bit 1 closes it). Shorter paths are skipped.
fn entity(out: &mut String, layer: &str, path: &DrawPath) {
    match (path.points.as_slice(), path.closed) {
        ([a, b], false) => {
            pair(out, 0, "LINE");
            pair(out, 8, layer);
            pair(out, 10, &num(a[0]));
            pair(out, 20, &num(a[1]));
            pair(out, 30, "0");
            pair(out, 11, &num(b[0]));
            pair(out, 21, &num(b[1]));
            pair(out, 31, "0");
        }
        (pts, closed) if pts.len() >= 2 => {
            pair(out, 0, "LWPOLYLINE");
            pair(out, 8, layer);
            pair(out, 90, &pts.len().to_string());
            pair(out, 70, if closed { "1" } else { "0" });
            for p in pts {
                pair(out, 10, &num(p[0]));
                pair(out, 20, &num(p[1]));
            }
        }
        _ => {}
    }
}

/// A coordinate in millimetres, six decimals; a non-finite value is written as zero.
fn num(v: f64) -> String {
    if v.is_finite() { format!("{v:.6}") } else { "0.000000".to_string() }
}

/// A DXF layer name: letters, digits and `_ - + .` (anything else becomes `_`), at most 255
/// characters; `LAYER` when nothing is left.
fn layer_name(name: &str) -> String {
    let clean: String = name
        .chars()
        .take(255)
        .map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '+' | '.') { c } else { '_' })
        .collect();
    if clean.is_empty() { "LAYER".to_string() } else { clean }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn closed_square_is_one_lwpolyline_on_its_layer() {
        let square = DrawPath { points: vec![[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, 10.0]], closed: true };
        let layer = DrawLayer { name: "LEVEL_000_0.000".into(), paths: vec![square] };
        let text = write_dxf(&[layer]).unwrap();
        assert!(text.contains("AC1015"));
        assert!(text.contains("LWPOLYLINE"));
        assert!(text.contains("LEVEL_000_0.000"));
        assert!(text.ends_with("0\r\nEOF\r\n"));
    }

    #[test]
    fn layer_names_keep_only_safe_characters() {
        assert_eq!(layer_name("a/b:c"), "a_b_c");
        assert_eq!(layer_name(""), "LAYER");
    }
}
