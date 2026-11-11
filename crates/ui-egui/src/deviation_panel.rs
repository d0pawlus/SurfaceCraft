//! The Deviation panel: the legend of the current deviation map, its statistics, histogram and
//! tolerance bands, the hotspots (see [`hotspots`]), the readings pinned in the viewport (see
//! [`readout`]), the tolerance control that reruns `deviation.compute`, and the maps with their
//! commands.
//!
//! The current map is the latest successful `deviation.compute` of a scan that still has its
//! colour map (the engine keeps the colours; the statistics and the report are the command's own
//! output in the session log). Every colour is a theme token or the output of
//! `surfacecraft_deviation::colorize`, the function that paints the viewport, so the legend cannot
//! disagree with the map.
pub mod hotspots;
pub mod readout;

use crate::app::SurfaceCraftApp;
use crate::i18n::{t, tf};
use crate::theme::{Token, Tokens, to_color32};
use egui::{Color32, Sense, Stroke, Vec2};
use serde_json::{Value, json};
use surfacecraft_deviation::{Band, DeviationStats, Distribution, Histogram, colorize, palette};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::EngineError;
use surfacecraft_engine::cmd::deviation::ComputeOut;

/// Every catalogue key the panel uses (checked against both languages by the tests).
pub const KEYS: &[&str] = &[
    "deviation.legend",
    "deviation.below",
    "deviation.in_tolerance",
    "deviation.above",
    "deviation.far",
    "deviation.unmeasured",
    "deviation.range_below",
    "deviation.range_in",
    "deviation.range_above",
    "deviation.range_far",
    "deviation.map_of",
    "deviation.against",
    "deviation.no_stats",
    "deviation.stats",
    "deviation.count",
    "deviation.min",
    "deviation.max",
    "deviation.mean",
    "deviation.rms",
    "deviation.max_abs",
    "deviation.within",
    "deviation.histogram",
    "deviation.bin",
    "deviation.bands",
    "deviation.band",
    "deviation.tolerance_label",
    "deviation.recompute",
    "deviation.none",
    "deviation.clear",
    "deviation.compute",
    "deviation.noise",
    "deviation.hotspots",
    "deviation.find_hotspots",
    "deviation.hotspots_none",
    "deviation.hotspots_shown",
    "deviation.hotspot_row",
    "deviation.excess",
    "deviation.missing",
    "deviation.focus_hint",
    "deviation.readout",
    "deviation.readout_none",
    "deviation.pinned",
    "deviation.pin_hint",
    "deviation.unpin",
    "deviation.triangle",
];

/// Narrowest panel (points) that fits two statistics, each a label and a number, in one row.
const TWO_STATS_MIN_WIDTH: f32 = 460.0;
/// Colour-map range in tolerances: the gradient and the histogram's shading run from −3·tol to +3·tol.
const FAR_TOLERANCES: f64 = 3.0;
/// Slices of the legend's gradient bar; each takes the colour of its centre.
const GRADIENT_SLICES: usize = 60;
const GRADIENT_HEIGHT: f32 = 14.0;
const MIN_GRADIENT_WIDTH: f32 = 60.0;
/// The histogram's height (px), with the room of its axis labels below the bars.
const HISTOGRAM_HEIGHT: f32 = 64.0;
const AXIS_ROOM: f32 = 14.0;
const AXIS_FONT_SIZE: f32 = 11.0;
const MIN_HISTOGRAM_WIDTH: f32 = 120.0;
/// Opacity of the tolerance band shaded behind the histogram bars.
const BAND_SHADE: f32 = 0.18;
const BAR_RADIUS: f32 = 2.0;
const SWATCH_RADIUS: f32 = 3.0;
/// Width of the marker lines drawn at ±tolerance.
const TICK_WIDTH: f32 = 1.5;

/// The current map: the scan, the command that made it and its statistics.
#[derive(Clone, Debug, PartialEq)]
pub struct Current {
    /// The scan the map is on.
    pub object: ObjectId,
    /// Its name.
    pub name: String,
    /// Parameters of the `deviation.compute` that made the map (reference, region, tolerance).
    pub params: Value,
    /// Statistics of the map.
    pub stats: DeviationStats,
    /// Histogram and tolerance bands of the signed deviation, from the same command.
    pub report: Distribution,
    /// The engine's warning (tolerance below the estimated scan noise), if any.
    pub warning: Option<String>,
}

/// The map shown: the selected scan's if it has one, else the most recent mapped scan's.
#[must_use]
pub fn current(app: &SurfaceCraftApp) -> Option<Current> {
    let selected = app.selected_object.filter(|id| app.session.live_map(*id).is_some());
    selected.and_then(|id| latest(app, Some(id))).or_else(|| latest(app, None))
}

/// The latest computation of a mapped scan (of `only`, when given) that is still on the scan.
fn latest(app: &SurfaceCraftApp, only: Option<ObjectId>) -> Option<Current> {
    app.session.log.iter().rev().filter(|e| e.ok && e.command == "deviation.compute").find_map(|e| {
        // a map computed without `show` left the colours of the earlier computation on the scan
        if e.params.get("show").and_then(Value::as_bool) == Some(false) {
            return None;
        }
        let out: ComputeOut = serde_json::from_value(e.outcome.clone()).ok()?;
        if app.session.live_map(out.id).is_none() || only.is_some_and(|s| s != out.id) {
            return None;
        }
        let name = app.session.doc.summaries().into_iter().find(|s| s.id == out.id).map(|s| s.name).unwrap_or_default();
        Some(Current {
            object: out.id,
            name,
            params: e.params.clone(),
            stats: out.stats,
            report: out.report,
            warning: out.warning,
        })
    })
}

/// Reruns `deviation.compute` of the current map with another tolerance and the same reference.
/// `None` when there is no current map.
pub fn recompute(app: &mut SurfaceCraftApp, tolerance: f64) -> Option<Result<Value, EngineError>> {
    let params = recompute_params(app, tolerance)?;
    Some(app.run_command("deviation.compute", params))
}

/// The parameters of the current map's `deviation.compute` with another tolerance.
fn recompute_params(app: &SurfaceCraftApp, tolerance: f64) -> Option<Value> {
    let mut params = current(app)?.params;
    if let Some(m) = params.as_object_mut() {
        m.insert("tolerance".into(), json!(tolerance));
    }
    Some(params)
}

/// One row of the legend: the bands it names, the theme tokens that paint them and the range.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegendRow {
    /// Catalogue key of the row's name.
    pub key: &'static str,
    /// Catalogue key of the range text (`{tol}`, `{far}` = 3·tol), `None` for no range.
    pub range: Option<&'static str>,
    /// The swatches of the row, each a token that equals `colorize` at its band.
    pub chips: &'static [Token],
}

/// The legend rows, from the negative side to the positive, unmeasured last.
pub const LEGEND: [LegendRow; 5] = [
    LegendRow { key: "deviation.below", range: Some("deviation.range_below"), chips: &[Token::DeviationBelowNear] },
    LegendRow {
        key: "deviation.in_tolerance",
        range: Some("deviation.range_in"),
        chips: &[Token::DeviationInTolerance],
    },
    LegendRow { key: "deviation.above", range: Some("deviation.range_above"), chips: &[Token::DeviationAboveNear] },
    LegendRow {
        key: "deviation.far",
        range: Some("deviation.range_far"),
        chips: &[Token::DeviationBelowFar, Token::DeviationAboveFar],
    },
    LegendRow { key: "deviation.unmeasured", range: None, chips: &[Token::DeviationUnmeasured] },
];

/// The colour `colorize` gives `value` at `tolerance`, as the byte colour of the theme.
#[must_use]
pub fn colour_at(value: f64, tolerance: f64) -> Color32 {
    to_color32(colorize(&[value], tolerance).first().copied().unwrap_or(palette::UNMEASURED))
}

/// The gradient bar of the legend: `(value, colour)` slices over −3·tol … +3·tol, the colours of
/// `colorize` at the slice centres.
#[must_use]
pub fn gradient(tolerance: f64) -> Vec<(f64, Color32)> {
    (0..GRADIENT_SLICES)
        .map(|i| {
            let v = -FAR_TOLERANCES * tolerance
                + (i as f64 + 0.5) / GRADIENT_SLICES as f64 * 2.0 * FAR_TOLERANCES * tolerance;
            (v, colour_at(v, tolerance))
        })
        .collect()
}

/// One histogram bin: its range (mm), its count and the colour of its centre.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bin {
    /// Lower edge (mm).
    pub lo: f64,
    /// Upper edge (mm).
    pub hi: f64,
    /// Points in the bin.
    pub count: usize,
    /// Colour of the bin (`colorize` at its centre).
    pub colour: Color32,
}

/// The bins of a report's histogram: equal bins over the report's range, each coloured by
/// `colorize` at its centre for tolerance `tol`.
#[must_use]
pub fn bins(hist: &Histogram, tol: f64) -> Vec<Bin> {
    let width = (hist.hi - hist.lo) / hist.counts.len().max(1) as f64;
    hist.counts
        .iter()
        .enumerate()
        .map(|(i, &count)| {
            let lo = hist.lo + i as f64 * width;
            let hi = lo + width;
            Bin { lo, hi, count, colour: colour_at((lo + hi) / 2.0, tol) }
        })
        .collect()
}

/// Text of a number with a fixed number of decimals.
pub(crate) fn num(v: f64, decimals: usize) -> String {
    format!("{v:.decimals$}")
}

/// Draws the Deviation panel of `app`.
pub(crate) fn show(app: &mut SurfaceCraftApp, ui: &mut egui::Ui) {
    let tokens = app.tokens.clone();
    match current(app) {
        Some(cur) => map_section(app, ui, &tokens, &cur),
        None if app.session.maps.is_empty() => {
            ui.weak(t("deviation.none"));
        }
        None => {
            ui.weak(t("deviation.no_stats"));
        }
    }
    ui.separator();
    maps_list(app, ui);
    ui.separator();
    if ui.button(t("deviation.compute")).clicked() {
        app.choose_command("deviation.compute");
    }
    if ui.button(t("deviation.noise")).clicked() {
        app.choose_command("deviation.noise");
    }
}

/// The mapped scans, each with its Clear button.
fn maps_list(app: &mut SurfaceCraftApp, ui: &mut egui::Ui) {
    let mut mapped: Vec<ObjectId> =
        app.session.maps.keys().copied().filter(|id| app.session.live_map(*id).is_some()).collect();
    mapped.sort();
    let mut clear = None;
    for id in mapped {
        ui.horizontal(|ui| {
            ui.label(format!("{id}"));
            if ui.small_button(t("deviation.clear")).clicked() {
                clear = Some(id);
            }
        });
    }
    if let Some(id) = clear {
        let _ = app.run_command("deviation.clear", json!({"object": id.0}));
    }
}

/// The current map, in the order the tab's height shows it: the statistics and the histogram
/// first, then the bands, the tolerance control and the legend, then the hotspots and the pinned
/// readings.
fn map_section(app: &mut SurfaceCraftApp, ui: &mut egui::Ui, tokens: &Tokens, cur: &Current) {
    let tol = cur.stats.tolerance;
    let icon = app.settings.theme.density.metrics().icon;
    header(ui, cur);
    statistics(ui, cur);
    ui.strong(t("deviation.histogram"));
    histogram(ui, tokens, &cur.report.histogram, tol);
    bands(ui, &cur.report.bands);
    tolerance_control(app, ui, tokens, cur);
    legend(ui, tokens, tol, icon);
    ui.separator();
    hotspots::section(app, ui, cur);
    ui.separator();
    readout::pins_section(app, ui);
}

/// The map's name and its reference.
fn header(ui: &mut egui::Ui, cur: &Current) {
    let reference = match cur.params.get("against") {
        Some(Value::String(s)) => s.clone(),
        Some(other) => other.to_string(),
        None => String::new(),
    };
    ui.horizontal_wrapped(|ui| {
        ui.strong(tf("deviation.map_of", &[("name", &cur.name)]));
        ui.weak(tf("deviation.against", &[("ref", &reference)]));
    });
}

/// The statistics of the map, two label-and-value pairs per row.
fn statistics(ui: &mut egui::Ui, cur: &Current) {
    let s = &cur.stats;
    let tol = s.tolerance;
    ui.strong(t("deviation.stats"));
    let rows = [
        (t("deviation.count").to_string(), s.count.to_string()),
        (t("deviation.min").to_string(), num(s.min, 3)),
        (t("deviation.max").to_string(), num(s.max, 3)),
        (t("deviation.mean").to_string(), num(s.mean, 3)),
        (t("deviation.rms").to_string(), num(s.rms, 3)),
        (t("deviation.max_abs").to_string(), num(s.max_abs, 3)),
        (tf("deviation.within", &[("tol", &num(tol, 3))]), format!("{} %", num(s.within * 100.0, 1))),
    ];
    let per_row = stats_per_row(ui.available_width());
    egui::Grid::new(("deviation-stats", cur.object.0)).num_columns(2 * per_row).spacing([10.0, 2.0]).show(ui, |ui| {
        for row in rows.chunks(per_row) {
            for (label, value) in row {
                ui.label(label.as_str());
                ui.monospace(value.as_str());
            }
            ui.end_row();
        }
    });
}

/// Statistics shown side by side in one row: two where the panel is wide enough for two labels
/// with their numbers, otherwise one, so that no number is cut off at the panel's edge.
fn stats_per_row(width: f32) -> usize {
    if width >= TWO_STATS_MIN_WIDTH { 2 } else { 1 }
}

/// The share of the measured points within each tolerance band, as the report gives them.
fn bands(ui: &mut egui::Ui, bands: &[Band]) {
    ui.strong(t("deviation.bands"));
    for band in bands {
        let share = num(band.share * 100.0, 1);
        ui.label(tf("deviation.band", &[("tol", &num(band.tolerance, 3)), ("share", &share)]));
    }
}

/// The tolerance of the map, editable, and the Recompute button that reruns `deviation.compute`
/// with it. The edited value is kept per map and tolerance, so it survives from frame to frame.
fn tolerance_control(app: &mut SurfaceCraftApp, ui: &mut egui::Ui, tokens: &Tokens, cur: &Current) {
    let tol = cur.stats.tolerance;
    let id = ui.scope_id().with(("deviation.tolerance", cur.object.0, tol.to_bits()));
    let mut edit = ui.data_mut(|d| *d.get_temp_mut_or(id, tol));
    let mut recompute_now = false;
    ui.horizontal(|ui| {
        let l = ui.label(t("deviation.tolerance_label"));
        ui.add(egui::DragValue::new(&mut edit).range(0.001..=10.0).speed(0.001).max_decimals(3)).labelled_by(l.id);
        recompute_now = ui.button(t("deviation.recompute")).clicked();
    });
    ui.data_mut(|d| d.insert_temp(id, edit));
    if let Some(w) = &cur.warning {
        ui.colored_label(tokens.color(Token::StatusWarning), w);
    }
    if recompute_now && let Some(params) = recompute_params(app, edit) {
        app.launch("deviation.compute", params, crate::jobs::After::Nothing);
    }
}

/// The legend: the gradient bar with the tolerance ticks, then one row per band.
fn legend(ui: &mut egui::Ui, tokens: &Tokens, tol: f64, icon: f32) {
    ui.strong(t("deviation.legend"));
    gradient_bar(ui, tokens, tol);
    for row in &LEGEND {
        ui.horizontal(|ui| {
            for &token in row.chips {
                let (rect, _) = ui.allocate_exact_size(Vec2::splat(icon), Sense::hover());
                ui.painter().rect_filled(rect, SWATCH_RADIUS, tokens.color(token));
            }
            ui.label(t(row.key));
            if let Some(range) = row.range {
                ui.weak(range_text(range, tol));
            }
        });
    }
}

/// `"< −0.100 mm"`-style text of a legend range.
fn range_text(key: &str, tol: f64) -> String {
    tf(key, &[("tol", &num(tol, 3)), ("far", &num(FAR_TOLERANCES * tol, 3))])
}

/// The gradient bar of the legend, with the ±tolerance band marked by two ticks.
fn gradient_bar(ui: &mut egui::Ui, tokens: &Tokens, tol: f64) {
    let width = ui.available_width().max(MIN_GRADIENT_WIDTH);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, GRADIENT_HEIGHT), Sense::hover());
    let painter = ui.painter_at(rect);
    let colours = gradient(tol);
    let slice = rect.width() / colours.len().max(1) as f32;
    for (i, (_, colour)) in colours.into_iter().enumerate() {
        let x = rect.left() + slice * i as f32;
        let part = egui::Rect::from_min_max(egui::pos2(x, rect.top()), egui::pos2(x + slice + 0.5, rect.bottom()));
        painter.rect_filled(part, 0.0, colour);
    }
    let x_at = |v: f64| rect.left() + ((v / (2.0 * FAR_TOLERANCES * tol)) + 0.5) as f32 * rect.width();
    for v in [-tol, tol] {
        painter.vline(x_at(v), rect.y_range(), Stroke::new(TICK_WIDTH, tokens.color(Token::TextPrimary)));
    }
}

/// The histogram of the report: one bar per bin over the report's range, with the tolerance band
/// shaded behind the bars. A bar's hover text gives its range and its count.
fn histogram(ui: &mut egui::Ui, tokens: &Tokens, hist: &Histogram, tol: f64) {
    let bins = bins(hist, tol);
    let max = bins.iter().map(|b| b.count).max().unwrap_or(0).max(1) as f32;
    let width = ui.available_width().max(MIN_HISTOGRAM_WIDTH);
    let (rect, _) = ui.allocate_exact_size(Vec2::new(width, HISTOGRAM_HEIGHT), Sense::hover());
    let painter = ui.painter_at(rect);
    let base = rect.bottom() - AXIS_ROOM;
    let span = (hist.hi - hist.lo).max(f64::MIN_POSITIVE);
    let x_at = |v: f64| rect.left() + ((v - hist.lo) / span) as f32 * rect.width();
    let band = egui::Rect::from_min_max(egui::pos2(x_at(-tol), rect.top()), egui::pos2(x_at(tol), base));
    let shade = tokens.color(Token::DeviationInTolerance).gamma_multiply(BAND_SHADE);
    painter.rect_filled(band.intersect(rect), 0.0, shade);
    let slot = rect.width() / bins.len().max(1) as f32;
    for (i, bin) in bins.iter().enumerate() {
        let h = bin.count as f32 / max * (base - rect.top());
        let left = rect.left() + slot * i as f32;
        let bar = egui::Rect::from_min_max(egui::pos2(left + 1.0, base - h), egui::pos2(left + slot - 1.0, base));
        painter.rect_filled(bar, BAR_RADIUS, bin.colour);
        let text =
            tf("deviation.bin", &[("lo", &num(bin.lo, 3)), ("hi", &num(bin.hi, 3)), ("n", &bin.count.to_string())]);
        ui.interact(bar, ui.scope_id().with(("deviation-bin", i)), Sense::hover()).on_hover_text(text);
    }
    let font = egui::FontId::proportional(AXIS_FONT_SIZE);
    let muted = tokens.color(Token::TextMuted);
    // the ticks stay on the plot even if the range were narrower than the tolerance
    let tick = |v: f64| x_at(v).clamp(rect.left(), rect.right());
    painter.text(egui::pos2(tick(-tol), rect.bottom()), egui::Align2::CENTER_BOTTOM, num(-tol, 3), font.clone(), muted);
    painter.text(egui::pos2(tick(tol), rect.bottom()), egui::Align2::CENTER_BOTTOM, num(tol, 3), font, muted);
}

#[cfg(test)]
mod tests {
    use super::stats_per_row;

    /// The right dock is about 310 points wide by default: two statistics in a row there cut the
    /// second number off at the panel's edge.
    #[test]
    fn a_narrow_panel_shows_one_statistic_per_row() {
        assert_eq!(stats_per_row(310.0), 1);
        assert_eq!(stats_per_row(459.9), 1);
        assert_eq!(stats_per_row(460.0), 2);
        assert_eq!(stats_per_row(900.0), 2);
    }
}
