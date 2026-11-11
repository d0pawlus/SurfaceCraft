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
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::Point3;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::deviation_panel::readout::{self, MAX_PINS, Reading, Spot, with_pin, without_pin};
use surfacecraft_ui_egui::deviation_panel::{self, LEGEND, hotspots};
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

/// A window tall enough for the whole Deviation tab, so that its lower buttons are on screen.
fn tall_harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1600.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
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
    let bins = deviation_panel::bins(&cur.report.histogram, cur.stats.tolerance);
    assert_eq!(bins.len(), 20, "the report's default bins");
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

/// A flat 40 mm scan 0.05 mm above a flat reference, mapped with tolerance 0.02 mm: every point is
/// off the tolerance, so the hotspots are one group. The plane's `center` is its corner, so it is
/// set at (−20, −20) to be centred on the origin, where the default camera looks.
fn offset_map() -> SurfaceCraftApp {
    let mut app = SurfaceCraftApp::headless();
    let plane = |z: f64, name: &str| json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 40, "center": [-20.0, -20.0, z], "name": name});
    app.run_command("mesh.synth", plane(0.05, "scan")).expect("the offset scan");
    app.run_command("mesh.synth", plane(0.0, "ref")).expect("the reference");
    app.run_command("deviation.compute", json!({"object": "scan", "against": "ref", "tolerance": 0.02}))
        .expect("the deviation map");
    app
}

fn scan_id(app: &SurfaceCraftApp) -> ObjectId {
    app.session.doc.summaries().into_iter().find(|s| s.name == "scan").expect("the scan").id
}

/// A view of 800 × 600 pixels: its centre ray meets the scan at the origin.
const VIEW: (f64, f64) = (800.0, 600.0);

fn spot(object: u64, triangle: u32) -> Spot {
    Spot { object: ObjectId(object), triangle, point: [0.0; 3] }
}

#[test]
fn the_histogram_bins_and_bands_are_the_command_output() {
    let mut app = scanned();
    let out = app
        .run_command("deviation.compute", json!({"object": "scan", "against": "ref", "tolerance": 0.1}))
        .expect("the map");
    let cur = deviation_panel::current(&app).expect("a map");
    let counts: Vec<u64> =
        out["report"]["histogram"]["counts"].as_array().expect("the bins").iter().filter_map(|c| c.as_u64()).collect();
    let shown: Vec<u64> =
        deviation_panel::bins(&cur.report.histogram, cur.stats.tolerance).iter().map(|b| b.count as u64).collect();
    assert_eq!(shown, counts, "the bins shown are the command's");
    let shares: Vec<f64> =
        out["report"]["bands"].as_array().expect("the bands").iter().filter_map(|b| b["share"].as_f64()).collect();
    let shown_shares: Vec<f64> = cur.report.bands.iter().map(|b| b.share).collect();
    assert_eq!(shown_shares, shares, "the bands shown are the command's");
    assert!(shares.windows(2).all(|w| w[0] <= w[1]), "the bands are cumulative: {shares:?}");
    let at_tolerance = cur.report.bands.iter().find(|b| (b.tolerance - cur.stats.tolerance).abs() < 1e-12);
    let at_tolerance = at_tolerance.expect("a band at the tolerance");
    assert!((at_tolerance.share - cur.stats.within).abs() < 1e-9, "the band at the tolerance is the within share");
}

#[test]
fn a_reading_is_pinned_once_and_the_oldest_goes_past_the_limit() {
    let pinned = with_pin(&[], spot(1, 3));
    assert_eq!(pinned, vec![spot(1, 3)]);
    assert_eq!(with_pin(&pinned, spot(1, 3)), pinned, "a triangle is pinned once");
    let many = (0..MAX_PINS + 2).fold(Vec::new(), |pins, t| with_pin(&pins, spot(1, t as u32)));
    assert_eq!(many.len(), MAX_PINS);
    assert_eq!(many.first().map(|s| s.triangle), Some(2), "the two oldest are gone");
    assert_eq!(without_pin(&pinned, 0), Vec::<Spot>::new(), "a removed pin is gone");
    assert_eq!(without_pin(&pinned, 4), pinned, "an index past the end removes nothing");
}

#[test]
fn the_reading_at_the_centre_of_the_view_is_the_engines_reading() {
    let mut app = offset_map();
    let scan = scan_id(&app);
    let (vw, vh) = VIEW;
    let at_centre = readout::spot_at(&mut app, scan, vw / 2.0, vh / 2.0, vw, vh).expect("the scan is at the centre");
    assert_eq!(at_centre.object, scan);
    let Reading::Measured(value) = readout::read(&mut app, at_centre) else { panic!("the centre is measured") };
    // 0.05 mm off the reference; the sign follows the mesh normals, so only the size is known
    assert!((value.abs() - 0.05).abs() < 1e-3, "{value}");
    let logged = app
        .run_command("deviation.at", json!({"object": scan.0, "triangle": at_centre.triangle}))
        .expect("the command reads the same triangle");
    let by_command = logged["deviation"].as_f64().expect("a deviation");
    assert!((by_command - value).abs() < 1e-9, "the readout is the command's: {by_command} and {value}");
}

#[test]
fn a_click_on_the_scan_pins_its_reading_until_it_is_removed() {
    let mut app = offset_map();
    let scan = scan_id(&app);
    let (vw, vh) = VIEW;
    let at_centre = readout::spot_at(&mut app, scan, vw / 2.0, vh / 2.0, vw, vh).expect("the scan at the centre");
    readout::pin_click(&mut app, vw / 2.0, vh / 2.0, vw, vh);
    readout::pin_click(&mut app, vw / 2.0, vh / 2.0, vw, vh);
    assert_eq!(app.deviation.pins, vec![at_centre], "the same triangle is pinned once");
    let mut ui = tall_harness(app);
    ui.run();
    ui.get_by_label("Deviation").click();
    ui.run();
    ui.run();
    assert!(ui.query_by_label(i18n::lookup(Lang::En, "deviation.pinned")).is_some(), "the pinned section");
    assert!(ui.query_by_label(i18n::lookup(Lang::En, "deviation.unpin")).is_some(), "the pin's Remove button");
    // the Remove button sits below the fold of the 336 px tab, so the removal is driven on the state
    let remaining = readout::without_pin(&ui.state().deviation.pins, 0);
    ui.state_mut().deviation.pins = remaining;
    ui.run();
    assert!(ui.state().deviation.pins.is_empty(), "a removed pin is gone");
}

#[test]
fn a_hotspot_centres_the_view_on_its_centroid() {
    let mut app = offset_map();
    let cur = deviation_panel::current(&app).expect("a map");
    app.run_command("deviation.hotspots", hotspots::params(&cur)).expect("the hotspots");
    let found = hotspots::latest(&app, &cur).expect("the run is listed for this map");
    let first = found.hotspots.first().expect("the offset scan is a hotspot");
    app.camera.target = Point3::origin();
    hotspots::focus(&mut app, first);
    assert_eq!(app.camera.target, hotspots::centroid(first));
    assert!((app.camera.target.z - 0.05).abs() < 1e-6, "the scan is 0.05 mm up: {:?}", app.camera.target);
}

/// The Deviation tab shows the bands and the hotspot search. The search button is below the fold
/// of the 336 px tab, so its run and the focus it gives are driven on the state here (the same
/// calls the button and a group's row make) and the button itself is listed for a local check.
#[test]
fn the_tab_shows_the_bands_and_the_hotspot_search_and_a_group_focuses_the_view() {
    let mut ui = tall_harness(offset_map());
    ui.run();
    ui.get_by_label("Deviation").click();
    ui.run();
    assert!(ui.query_by_label(i18n::lookup(Lang::En, "deviation.bands")).is_some(), "the bands");
    assert!(ui.query_by_label(i18n::lookup(Lang::En, "deviation.find_hotspots")).is_some(), "the search button");
    let cur = deviation_panel::current(ui.state()).expect("a map");
    ui.state_mut().run_command("deviation.hotspots", hotspots::params(&cur)).expect("the search");
    let found = hotspots::latest(ui.state(), &cur).expect("the panel lists the run");
    let first = found.hotspots.first().expect("one hotspot").clone();
    ui.state_mut().camera.target = Point3::origin();
    hotspots::focus(ui.state_mut(), &first);
    ui.run();
    assert_eq!(ui.state().camera.target, hotspots::centroid(&first));
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
