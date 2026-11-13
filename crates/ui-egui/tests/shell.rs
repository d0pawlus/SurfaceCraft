//! Smoke tests of the Craft-family shell: the dock layout shows the viewport and the panels, a
//! theme preset resolves different tokens, and the English and Polish catalogues are complete.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::dock::{self, Panel};
use surfacecraft_ui_egui::guide::Step;
use surfacecraft_ui_egui::i18n::{self, Lang};
use surfacecraft_ui_egui::settings::UiSettings;
use surfacecraft_ui_egui::shell::AppAction;
use surfacecraft_ui_egui::theme::{Density, Preset, Theme, Token, Tokens};
use surfacecraft_ui_egui::workflow::Stage;

fn harness_with(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1000.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
}

fn harness() -> Harness<'static, SurfaceCraftApp> {
    harness_with(SurfaceCraftApp::headless())
}

#[test]
fn dock_layout_shows_the_viewport_and_every_panel() {
    let mut h = harness();
    h.run();
    let app = h.state();
    // every panel but the command log, which is for scripting and shown on request
    for panel in Panel::ALL.into_iter().filter(|p| *p != Panel::Log) {
        assert!(app.workspace.layout.contains(&panel), "{panel:?} is in the default layout");
    }
    assert!(!app.workspace.layout.contains(&Panel::Log), "the command log starts hidden");
    // the viewport is the biggest thing on screen and sits between the docks: the right dock is
    // one column, so the viewport keeps most of the window's width
    let vp = app.viewport_rect.expect("the viewport was drawn");
    assert!(vp.width() > 800.0 && vp.height() > 400.0, "viewport {vp:?}");
    assert!(h.query_by_label(i18n::lookup(Lang::En, "panel.regions")).is_some());
    assert!(h.query_by_label(i18n::lookup(Lang::En, "panel.deviation")).is_some(), "the deviation tab");
    // the panels' tabs are on screen under their translated titles
    for key in ["panel.objects", "panel.history", "panel.properties", "panel.regions", "panel.deviation"] {
        let title = i18n::lookup(Lang::En, key);
        assert!(h.query_by_label(title).is_some(), "tab {title}");
    }
    // View > Command log shows the log beside the deviation map, and hides it again
    h.state_mut().perform(AppAction::ToggleLog);
    h.run();
    assert!(h.state().workspace.layout.contains(&Panel::Log));
    assert!(h.query_by_label(i18n::lookup(Lang::En, "panel.log")).is_some(), "the log's tab");
    h.state_mut().perform(AppAction::ToggleLog);
    h.run();
    assert!(!h.state().workspace.layout.contains(&Panel::Log));
}

#[test]
fn the_ribbon_has_a_tab_per_stage_and_shows_the_stage_tools() {
    let mut h = harness();
    h.run();
    for stage in Stage::ALL {
        let title = i18n::lookup(Lang::En, stage.title_key());
        assert!(h.query_by_label(title).is_some(), "stage tab {title}");
    }
    // the first stage is open: its tools are buttons, another stage's are not on screen
    assert_eq!(h.state().stage, Stage::Scan);
    assert!(h.query_by_label("Fill holes").is_some());
    assert!(h.query_by_label("3-2-1 align").is_none());
    h.get_by_label("Align").click();
    h.run();
    assert_eq!(h.state().stage, Stage::Align);
    assert!(h.query_by_label("Fill holes").is_none());
    // a tool button opens the tool, it runs nothing by itself
    let log = h.state().session.log.len();
    h.get_by_label("3-2-1 align").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("align.datum"));
    assert_eq!(h.state().session.log.len(), log);
    // choosing a command from anywhere brings the ribbon to its stage
    h.state_mut().choose_command("feature.extrude");
    h.run();
    assert_eq!(h.state().stage, Stage::Solid);
    // a tool chosen while the tool panel is closed opens the panel again
    dock::set_visible(&mut h.state_mut().workspace, Panel::Properties, false).unwrap();
    assert!(!h.state().workspace.layout.contains(&Panel::Properties));
    h.state_mut().choose_command("feature.extrude");
    h.run();
    assert!(h.state().workspace.layout.contains(&Panel::Properties));
    // Cancel closes the tool and the guide is back
    h.get_by_label("Cancel").click();
    h.run();
    assert_eq!(h.state().palette_command, None);
    assert!(h.query_by_label(i18n::lookup(Lang::En, "guide.title")).is_some());
}

#[test]
fn enter_confirms_the_open_tool_and_escape_closes_it() {
    let mut h = harness();
    h.state_mut().choose_command("body.box");
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    h.run();
    h.key_press(egui::Key::Enter);
    h.run();
    assert_eq!(h.state().session.doc.bodies.len(), 1, "Enter is OK: {}", h.state().status);
    assert_eq!(h.state().palette_command, None);
    // Esc closes without running
    h.state_mut().choose_command("body.box");
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    h.run();
    h.key_press(egui::Key::Escape);
    h.run();
    assert_eq!(h.state().palette_command, None, "Esc is Cancel");
    assert_eq!(h.state().session.doc.bodies.len(), 1);
    // while a field has the keyboard, Enter is the field's: the search does not run the tool
    h.state_mut().choose_command("body.box");
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    h.run();
    h.get_by_label("Search tools").focus();
    h.run();
    h.key_press(egui::Key::Enter);
    h.run();
    assert_eq!(h.state().session.doc.bodies.len(), 1, "a focused field keeps Enter");
    assert_eq!(h.state().palette_command, Some("body.box"));
}

#[test]
fn an_empty_document_welcomes_and_the_guide_leads_through_the_stages() {
    let mut h = harness();
    h.run();
    h.run();
    // the welcome card offers the three ways to start; the guide's first step is the import
    let import = i18n::lookup(Lang::En, "welcome.import");
    assert!(h.query_by_label(import).is_some(), "the welcome card");
    assert_eq!(h.state().progress().next(), Some(Step::Import));
    // the sample part is a command like any other: it fills the document and the card goes
    h.get_all_by_label(i18n::lookup(Lang::En, "action.sample_scan")).next().expect("sample button").click();
    h.run();
    h.run();
    assert_eq!(h.state().session.doc.meshes.len(), 1, "{}", h.state().status);
    assert!(h.query_by_label(import).is_none(), "the card is only for an empty document");
    assert_eq!(h.state().progress().next(), Some(Step::Clean));
    // the guide's button opens the stage of the step to do now
    h.state_mut().run_command("align.center", json!({"object": "sample"})).unwrap();
    h.run();
    assert_eq!(h.state().progress().next(), Some(Step::Segment));
    h.get_by_label("Open Segment").click();
    h.run();
    assert_eq!(h.state().stage, Stage::Segment);
    assert!(h.query_by_label("Auto segment").is_some());
}

#[test]
fn the_navigation_buttons_stay_put_when_the_active_tool_has_settings() {
    use surfacecraft_ui_egui::viewtools::Tool;
    let fit = i18n::lookup(Lang::En, "view.fit");
    let mut h = harness();
    h.run();
    h.run();
    let before = h.get_by_label(fit).rect();
    // the lasso has a setting ("Through"): it sits above the bar and must not push the buttons,
    // or a click aimed at the next tool lands on another one
    h.state_mut().tools.tool = Tool::Lasso;
    h.run();
    h.run();
    assert!(h.query_by_label(i18n::lookup(Lang::En, "tool.through")).is_some(), "the lasso's setting");
    assert_eq!(h.get_by_label(fit).rect(), before, "the bar moved with the tool");
    h.state_mut().tools.tool = Tool::Brush;
    h.run();
    h.run();
    assert_eq!(h.get_by_label(fit).rect(), before, "the bar moved with the tool");
}

#[test]
fn a_narrow_viewport_keeps_the_tools_on_one_row_and_the_views_on_the_next() {
    let mut h = Harness::builder()
        .with_size((980.0, 760.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), SurfaceCraftApp::headless());
    h.run();
    h.run();
    let top =
        |h: &Harness<'static, SurfaceCraftApp>, key: &str| h.get_by_label(i18n::lookup(Lang::En, key)).rect().top();
    let tools = top(&h, "tool.navigate");
    // every pointer tool is on the first row
    assert_eq!(top(&h, "tool.rotate"), tools, "the pointer tools share a row");
    // the section plane and the two views are together under them, never one button alone
    let views = top(&h, "view.fit");
    assert!(views > tools, "the views are on the second row");
    assert_eq!(top(&h, "section.title"), views);
    assert_eq!(top(&h, "view.iso"), views);
}

#[test]
fn the_viewport_is_protected_and_the_layout_survives_a_round_trip() {
    let mut ws = dock::default_workspace(240.0);
    assert!(ws.validate().is_ok());
    // a file without the viewport falls back to the default layout
    let mut broken = dock::default_workspace(240.0);
    broken.layout.floating.clear();
    broken.layout.root = None;
    dock::reconcile(&mut broken, &ws);
    assert!(broken.layout.contains(&Panel::Viewport));
    // the viewport cannot be hidden, a side panel can and comes back
    dock::set_visible(&mut ws, Panel::Viewport, false).unwrap();
    assert!(ws.layout.contains(&Panel::Viewport));
    dock::set_visible(&mut ws, Panel::History, false).unwrap();
    assert!(!ws.layout.contains(&Panel::History));
    dock::set_visible(&mut ws, Panel::History, true).unwrap();
    assert!(ws.layout.contains(&Panel::History));
    // settings keep the layout; garbage gives the defaults
    let settings = UiSettings {
        theme: Theme { preset: Preset::Light, density: Density::Compact },
        lang: Lang::Pl,
        workspace: Some(ws.clone()),
        layout_version: surfacecraft_ui_egui::settings::LAYOUT_VERSION,
        ..UiSettings::default()
    };
    let back = UiSettings::from_json(&settings.to_json());
    assert_eq!(back, settings);
    assert_eq!(UiSettings::from_json("{ not json"), UiSettings::default());
    // a layout saved under an older arrangement is dropped, the theme and language are kept
    let mut older = settings.clone();
    older.layout_version = 1;
    let migrated = UiSettings::from_json(&serde_json::to_string(&older).unwrap());
    assert_eq!(migrated.workspace, None);
    assert_eq!(migrated.lang, Lang::Pl);
    // and an app started from them uses them
    let app = SurfaceCraftApp::with_settings(back);
    assert_eq!(app.workspace.layout, ws.layout);
    assert_eq!(app.settings.theme.preset, Preset::Light);
}

#[test]
fn a_theme_preset_resolves_different_tokens_and_the_menu_switches_it() {
    let dark = Tokens::resolve(Preset::Dark);
    let light = Tokens::resolve(Preset::Light);
    let high = Tokens::resolve(Preset::HighContrast);
    for token in [Token::SurfaceBase, Token::TextPrimary, Token::AccentPrimary, Token::ViewportBackground] {
        assert_ne!(dark.color(token), light.color(token), "{token:?} differs between Dark and Light");
        assert_ne!(dark.color(token), high.color(token), "{token:?} differs between Dark and High contrast");
    }
    // the default is dark
    assert_eq!(Theme::default().preset, Preset::Dark);
    // text is readable on its surface in every preset (contrast ratio of at least 4.5)
    for preset in Preset::ALL {
        let t = Tokens::resolve(preset);
        assert!(contrast(t.color(Token::TextPrimary), t.color(Token::SurfaceBase)) >= 4.5, "{preset:?}");
    }
    // clicking View > Theme > Light changes the settings, the tokens and egui's visuals
    let mut h = harness();
    h.run();
    let before = h.ctx.global_style().visuals.panel_fill;
    h.get_by_label("View").click();
    h.run();
    h.get_by_label("Theme").click();
    h.run();
    h.get_by_label("Light").click();
    h.run();
    h.run();
    assert_eq!(h.state().settings.theme.preset, Preset::Light);
    assert_eq!(h.state().tokens.preset(), Preset::Light);
    assert_ne!(h.ctx.global_style().visuals.panel_fill, before);
    assert_eq!(h.ctx.global_style().visuals.panel_fill, light.color(Token::SurfaceBase));
    // the layout is the same in every preset: density changes spacing, not the arrangement
    assert_ne!(Density::Compact.metrics(), Density::Comfortable.metrics());
}

fn luminance(c: egui::Color32) -> f32 {
    let lin = |v: u8| {
        let v = f32::from(v) / 255.0;
        if v <= 0.03928 { v / 12.92 } else { ((v + 0.055) / 1.055).powf(2.4) }
    };
    0.2126 * lin(c.r()) + 0.7152 * lin(c.g()) + 0.0722 * lin(c.b())
}

fn contrast(a: egui::Color32, b: egui::Color32) -> f32 {
    let (la, lb) = (luminance(a), luminance(b));
    (la.max(lb) + 0.05) / (la.min(lb) + 0.05)
}

#[test]
fn the_english_and_polish_catalogues_are_complete() {
    let en = i18n::catalog(Lang::En);
    let pl = i18n::catalog(Lang::Pl);
    assert!(!en.is_empty() && !pl.is_empty(), "catalogues parse");
    let missing_pl: Vec<_> = en.keys().filter(|k| !pl.contains_key(*k)).collect();
    let missing_en: Vec<_> = pl.keys().filter(|k| !en.contains_key(*k)).collect();
    assert!(missing_pl.is_empty(), "keys without a Polish text: {missing_pl:?}");
    assert!(missing_en.is_empty(), "keys without an English text: {missing_en:?}");
    for (key, text) in en.iter().chain(pl.iter()) {
        assert!(!text.trim().is_empty(), "{key} is empty");
    }
    // the same placeholders in both languages
    fn placeholders(s: &str) -> Vec<&str> {
        let mut v: Vec<&str> = s.split('{').skip(1).filter_map(|r| r.split('}').next()).collect();
        v.sort_unstable();
        v
    }
    for (key, text) in en {
        assert_eq!(placeholders(text), placeholders(&pl[key]), "placeholders of {key}");
    }
    // every key an enum hands out exists, in both languages
    let mut keys: Vec<&str> = Vec::new();
    keys.extend(Panel::ALL.iter().map(|p| p.title_key()));
    keys.extend(Preset::ALL.iter().map(|p| p.name_key()));
    keys.extend(Density::ALL.iter().map(|d| d.name_key()));
    keys.extend(Lang::ALL.iter().map(|l| l.name_key()));
    keys.extend(AppAction::ALL.iter().flat_map(|a| [a.label_key(), a.short_key(), a.hint_key()]));
    keys.extend(Stage::ALL.iter().flat_map(|s| [s.title_key(), s.hint_key()]));
    keys.extend(Step::ALL.iter().flat_map(|s| [s.title_key(), s.hint_key()]));
    keys.extend(surfacecraft_ui_egui::viewtools::Tool::ALL.iter().flat_map(|t| [t.key(), t.hint_key()]));
    for key in keys {
        assert!(en.contains_key(key) && pl.contains_key(key), "catalogue key {key}");
    }
    // a missing key never fails: it falls back to English, then to the key itself
    assert_eq!(i18n::lookup(Lang::Pl, "no.such.key"), "no.such.key");
    assert_eq!(Lang::from_locale("pl_PL.UTF-8"), Lang::Pl);
    assert_eq!(Lang::from_locale("de_DE"), Lang::En);
}

/// A text with a character the bundled fonts cannot draw would panic the frame that shows it:
/// every catalogue text is laid out once here.
#[test]
fn every_catalogue_text_can_be_drawn() {
    for lang in Lang::ALL {
        let mut texts: Vec<String> = i18n::catalog(lang).values().cloned().collect();
        if lang == Lang::Pl {
            // the Polish texts of the engine's descriptions and messages are drawn like any other
            texts.extend(i18n::hints_pl().values().cloned());
            texts.extend(i18n::engine_messages_pl().into_values());
        }
        let mut h = Harness::builder().build_ui(move |ui| {
            for text in &texts {
                ui.label(text);
            }
        });
        h.run();
    }
}

/// Every `description` text anywhere in `schema`.
fn descriptions(schema: &serde_json::Value, out: &mut Vec<String>) {
    match schema {
        serde_json::Value::Object(map) => {
            for (key, value) in map {
                match value.as_str() {
                    Some(text) if key == "description" => out.push(text.to_string()),
                    _ => descriptions(value, out),
                }
            }
        }
        serde_json::Value::Array(items) => items.iter().for_each(|v| descriptions(v, out)),
        _ => {}
    }
}

#[test]
fn every_parameter_description_has_a_polish_text() {
    let mut all = Vec::new();
    for c in surfacecraft_engine::registry() {
        descriptions(&(c.input)(), &mut all);
    }
    all.sort();
    all.dedup();
    let missing: Vec<&String> = all.iter().filter(|d| !i18n::hints_pl().contains_key(*d)).collect();
    assert!(missing.is_empty(), "{} descriptions without a Polish text: {missing:#?}", missing.len());
}

#[test]
fn switching_the_language_translates_the_menus() {
    let mut h = harness();
    h.run();
    assert!(h.query_by_label("File").is_some());
    h.state_mut().settings.lang = Lang::Pl;
    h.run();
    h.run();
    assert!(h.query_by_label("Plik").is_some(), "menu in Polish");
    assert!(h.query_by_label("File").is_none());
    h.state_mut().settings.lang = Lang::En;
    h.run();
    h.run();
}

#[test]
fn panels_show_the_state_of_the_document() {
    let mut h = harness();
    h.state_mut().run_command("mesh.synth", json!({"shape": "sphere", "size": [10.0], "resolution": 12})).unwrap();
    h.run();
    // the object tree groups by kind, the history lists the command
    h.get_by_label_contains("Meshes (1)");
    assert_eq!(h.state().session.history.undo_labels().len(), 1);
    // the toolbar's undo is a command: after clicking it the tree is empty again
    h.get_by_label("Undo").click();
    h.run();
    assert!(h.state().session.doc.summaries().is_empty());
}

#[test]
fn the_dock_never_reports_an_error_in_a_small_or_unsized_window() {
    // The web canvas can be 800x600 or not sized yet on the first frame: the shell must not put a
    // "docking geometry" message into the status bar for that.
    for (w, h) in [(800.0_f32, 600.0_f32), (640.0, 480.0), (400.0, 300.0), (0.0, 0.0), (1.0, 1.0)] {
        let app = SurfaceCraftApp::headless();
        let ready = app.status.clone();
        let mut harness =
            Harness::builder().with_size((w, h)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
        harness.run();
        harness.run();
        assert_eq!(harness.state().status, ready, "status after the first frames at {w}x{h}");
    }
}
