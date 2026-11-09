//! The Deviation panel after `mesh.synth` (noise) and `deviation.compute`: it shows the statistics
//! and a histogram whose bins add up to the measured count, its legend is the colour map of the
//! viewport (`colorize`), and its tolerance control reruns the command.
//!
//! `SURFACECRAFT_SHOTS=<dir> cargo test -p surfacecraft-ui-egui --test deviation_panel` also writes
//! `p6-g4-deviation.png` (needs a GPU; the owner's local review picture).
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_deviation::{colorize, palette};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::deviation_panel::{self, LEGEND};
use surfacecraft_ui_egui::i18n::{self, Lang};
use surfacecraft_ui_egui::theme::{Preset, Token, Tokens, to_color32};

/// A noisy plane (σ = 0.04 mm) measured against the clean plane, computed with tolerance 0.1 mm.
fn scanned() -> SurfaceCraftApp {
    let mut app = SurfaceCraftApp::headless();
    let scan =
        json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 40, "noise": 0.04, "seed": 7, "name": "scan"});
    app.run_command("mesh.synth", scan).expect("the noisy scan");
    app.run_command("mesh.synth", json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 40, "name": "ref"}))
        .expect("the reference");
    app.run_command("deviation.compute", json!({"object": "scan", "against": "ref", "tolerance": 0.1}))
        .expect("the deviation map");
    app
}

fn harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1000.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
}

#[test]
fn the_panel_shows_the_statistics_and_the_histogram_adds_up_to_the_count() {
    let app = scanned();
    let cur = deviation_panel::current(&app).expect("the scan has a map");
    assert_eq!(cur.name, "scan");
    assert!(cur.stats.count > 1000, "points measured: {}", cur.stats.count);
    // the bins of the histogram hold every measured point
    let bins = deviation_panel::bins(&cur.stats);
    assert_eq!(bins.len(), 10);
    assert_eq!(bins.iter().map(|b| b.count).sum::<usize>(), cur.stats.count);
    // statistics are ordered and the noise sits inside the tolerance
    assert!(cur.stats.min <= cur.stats.mean && cur.stats.mean <= cur.stats.max);
    assert!(cur.stats.rms > 0.0 && cur.stats.rms < 0.1, "rms {}", cur.stats.rms);
    assert!(cur.stats.within > 0.9, "within {}", cur.stats.within);
    // the panel draws them: the tab, the legend, the statistics with the measured count
    let mut h = harness(app);
    h.run();
    h.get_by_label("Deviation").click();
    h.run();
    h.run();
    let count_label = i18n::lookup(Lang::En, "deviation.count");
    assert!(h.query_by_label(count_label).is_some(), "the statistics table shows {count_label}");
    assert!(h.query_by_label(&cur.stats.count.to_string()).is_some(), "the measured count is on screen");
    assert!(h.query_by_label(i18n::lookup(Lang::En, "deviation.legend")).is_some());
    assert!(h.query_by_label("Map of scan").is_some(), "the panel names the map");
}

#[test]
fn the_legend_colours_are_the_colours_of_colorize() {
    let tol = 0.1;
    // the tokens of every preset equal the viewport's colours at the bands' samples
    let samples: [(Token, f64); 4] = [
        (Token::DeviationInTolerance, 0.0),
        (Token::DeviationBelowFar, -3.0 * tol),
        (Token::DeviationAboveFar, 3.0 * tol),
        (Token::DeviationBelowNear, -1.001 * tol),
    ];
    for preset in Preset::ALL {
        let tokens = Tokens::resolve(preset);
        for (token, value) in samples {
            let viewport = to_color32(colorize(&[value], tol)[0]);
            assert_eq!(tokens.color(token), viewport, "{token:?} at {value} in {preset:?}");
        }
        let above_near = to_color32(colorize(&[1.001 * tol], tol)[0]);
        assert_eq!(tokens.color(Token::DeviationAboveNear), above_near, "above near in {preset:?}");
        let unmeasured = to_color32(colorize(&[f64::NAN], tol)[0]);
        assert_eq!(tokens.color(Token::DeviationUnmeasured), unmeasured, "unmeasured in {preset:?}");
    }
    // the legend rows name every band, and each swatch is one of those tokens
    let mut chips: Vec<Token> = LEGEND.iter().flat_map(|r| r.chips.iter().copied()).collect();
    chips.sort_by_key(|t| t.name());
    chips.dedup();
    assert_eq!(chips.len(), 6, "six swatches: {chips:?}");
    // the gradient bar is colorize at its slice centres, from −3·tol to +3·tol
    let gradient = deviation_panel::gradient(tol);
    assert_eq!(gradient.len(), 60);
    for (value, colour) in &gradient {
        assert_eq!(*colour, to_color32(colorize(&[*value], tol)[0]), "gradient at {value}");
    }
    assert!((gradient[0].0 + 2.95 * tol).abs() < 1e-12, "the first slice is centred at −2.95·tol");
    // the palette is one definition: the in-tolerance colour is the palette's
    assert_eq!(colorize(&[0.0], tol)[0], palette::IN_TOLERANCE);
}

#[test]
fn the_tolerance_control_reruns_the_command_with_the_same_reference() {
    let mut app = scanned();
    let before = deviation_panel::current(&app).expect("a map").stats;
    let result = deviation_panel::recompute(&mut app, 0.02).expect("there is a map to recompute");
    result.expect("deviation.compute accepts the tolerance");
    let after = deviation_panel::current(&app).expect("the new map");
    assert_eq!(after.stats.tolerance, 0.02);
    assert_eq!(after.stats.count, before.count, "the same points are measured");
    assert!(after.stats.within < before.within, "a tighter band holds fewer points");
    assert_eq!(after.stats.histogram.iter().sum::<usize>(), after.stats.count);
    assert_eq!(after.params.get("against").and_then(|v| v.as_str()), Some("ref"));
    // without a map there is nothing to recompute
    let mut empty = SurfaceCraftApp::headless();
    assert!(deviation_panel::recompute(&mut empty, 0.1).is_none());
}

#[test]
fn the_panel_catalogue_keys_exist_in_both_languages() {
    for key in deviation_panel::KEYS {
        for lang in Lang::ALL {
            assert!(i18n::catalog(lang).contains_key(*key), "{key} in {lang:?}");
        }
    }
    for row in &LEGEND {
        assert!(deviation_panel::KEYS.contains(&row.key), "{}", row.key);
        if let Some(range) = row.range {
            assert!(deviation_panel::KEYS.contains(&range), "{range}");
        }
    }
}

#[test]
fn write_the_deviation_panel_screenshot() {
    let Some(dir) = std::env::var_os("SURFACECRAFT_SHOTS") else { return };
    let dir = std::path::PathBuf::from(dir);
    std::fs::create_dir_all(&dir).expect("screenshot directory");
    let mut h = Harness::builder()
        .with_size((1500.0, 900.0))
        .with_pixels_per_point(1.0)
        .wgpu()
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), scanned());
    h.run();
    h.get_by_label("Deviation").click();
    h.run();
    h.run();
    let image = match h.render() {
        Ok(i) => i,
        Err(e) => panic!("render: {e}"),
    };
    let (w, hgt) = (image.width(), image.height());
    let png = surfacecraft_ui_egui::shot::encode_png(&image, w, hgt).expect("png");
    std::fs::write(dir.join("p6-g4-deviation.png"), png).expect("write the screenshot");
}
