//! SVG writer: one `<path>` per chain, one group per layer. The width, height and viewBox are in
//! millimetres (one user unit is one millimetre), and y is flipped so that the second plane axis
//! points up on the page. Written by hand, no dependency.
use crate::dxf::MAX_POINTS;
use crate::{DrawLayer, DrawPath, IoError};

/// Writes `layers` as an SVG document.
pub fn write_svg(layers: &[DrawLayer]) -> Result<String, IoError> {
    let points: usize = layers.iter().flat_map(|l| l.paths.iter()).map(|p| p.points.len()).sum();
    if points > MAX_POINTS {
        return Err(IoError::Limit(format!("the drawing has {points} points; the limit is {MAX_POINTS}")));
    }
    // the page bounds, in page coordinates (x, -y)
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in layers.iter().flat_map(|l| l.paths.iter()).flat_map(|p| p.points.iter()) {
        if p[0].is_finite() && p[1].is_finite() {
            lo = [lo[0].min(p[0]), lo[1].min(-p[1])];
            hi = [hi[0].max(p[0]), hi[1].max(-p[1])];
        }
    }
    let (x0, y0, w, h, stroke) = if lo[0] <= hi[0] && lo[1] <= hi[1] {
        let span = (hi[0] - lo[0]).max(hi[1] - lo[1]);
        let m = 1.0 + 0.02 * span;
        (lo[0] - m, lo[1] - m, hi[0] - lo[0] + 2.0 * m, hi[1] - lo[1] + 2.0 * m, (0.001 * span).max(0.1))
    } else {
        (0.0, 0.0, 10.0, 10.0, 0.1)
    };
    let mut out = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}mm\" height=\"{}mm\" viewBox=\"{} {} {} {}\">\n",
        num(w),
        num(h),
        num(x0),
        num(y0),
        num(w),
        num(h)
    );
    for layer in layers {
        out.push_str(&format!(
            "<g id=\"{}\" fill=\"none\" stroke=\"#000000\" stroke-width=\"{}\">\n",
            layer_id(&layer.name),
            num(stroke)
        ));
        for path in &layer.paths {
            if path.points.len() >= 2 {
                out.push_str(&format!("<path d=\"{}\"/>\n", path_data(path)));
            }
        }
        out.push_str("</g>\n");
    }
    out.push_str("</svg>\n");
    Ok(out)
}

/// The path data: `M` to the first point, `L` to the others, `Z` when closed.
fn path_data(path: &DrawPath) -> String {
    let mut d = String::new();
    for (i, p) in path.points.iter().enumerate() {
        d.push_str(if i == 0 { "M " } else { " L " });
        d.push_str(&num(p[0]));
        d.push(' ');
        // the page's y axis points down
        d.push_str(&num(0.0 - p[1]));
    }
    if path.closed {
        d.push_str(" Z");
    }
    d
}

/// A coordinate with four decimals; a non-finite value is written as zero.
fn num(v: f64) -> String {
    if v.is_finite() { format!("{v:.4}") } else { "0.0000".to_string() }
}

/// An XML id for a layer: `layer_` followed by the name's letters, digits, `_`, `-` and `.`.
fn layer_id(name: &str) -> String {
    let clean: String =
        name.chars().map(|c| if c.is_ascii_alphanumeric() || matches!(c, '_' | '-' | '.') { c } else { '_' }).collect();
    format!("layer_{clean}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_closed_path_per_chain() {
        let path = DrawPath { points: vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]], closed: true };
        let layer = DrawLayer { name: "LEVEL_000_0.000".into(), paths: vec![path] };
        let text = write_svg(&[layer]).unwrap();
        assert_eq!(text.matches("<path").count(), 1);
        assert!(text.contains(" Z\"/>"));
        assert!(text.contains("viewBox="));
    }
}
