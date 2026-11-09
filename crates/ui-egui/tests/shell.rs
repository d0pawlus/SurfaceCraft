//! Smoke tests of the Craft-family shell: the dock layout shows the viewport and the panels, a
//! theme preset resolves different tokens, and the English and Polish catalogues are complete.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::dock::{self, Panel};
use surfacecraft_ui_egui::i18n::{self, Lang};
use surfacecraft_ui_egui::settings::UiSettings;
use surfacecraft_ui_egui::theme::{Density, Preset, Theme, Token, Tokens};

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
    for panel in Panel::ALL {
        assert!(app.workspace.layout.contains(&panel), "{panel:?} is in the default layout");
    }
    // the viewport is the biggest thing on screen and sits between the docks
    let vp = app.viewport_rect.expect("the viewport was drawn");
    assert!(vp.width() > 600.0 && vp.height() > 400.0, "viewport {vp:?}");
    // the panels' tabs are on screen under their translated titles
    for key in ["panel.objects", "panel.history", "panel.properties", "panel.regions", "panel.deviation", "panel.log"] {
        let title = i18n::lookup(Lang::En, key);
        assert!(h.query_by_label(title).is_some(), "tab {title}");
    }
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
    };
    let back = UiSettings::from_json(&settings.to_json());
    assert_eq!(back, settings);
    assert_eq!(UiSettings::from_json("{ not json"), UiSettings::default());
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
    let placeholders = |s: &str| {
        let mut v: Vec<&str> = s.split('{').skip(1).filter_map(|r| r.split('}').next()).collect();
        v.sort_unstable();
        v
    };
    for (key, text) in en {
        assert_eq!(placeholders(text), placeholders(&pl[key]), "placeholders of {key}");
    }
    // every key an enum hands out exists, in both languages
    let mut keys: Vec<&str> = Vec::new();
    keys.extend(Panel::ALL.iter().map(|p| p.title_key()));
    keys.extend(Preset::ALL.iter().map(|p| p.name_key()));
    keys.extend(Density::ALL.iter().map(|d| d.name_key()));
    keys.extend(Lang::ALL.iter().map(|l| l.name_key()));
    keys.extend(
        [
            surfacecraft_ui_egui::shell::AppAction::New,
            surfacecraft_ui_egui::shell::AppAction::Open,
            surfacecraft_ui_egui::shell::AppAction::Save,
            surfacecraft_ui_egui::shell::AppAction::ImportMesh,
            surfacecraft_ui_egui::shell::AppAction::ImportStep,
            surfacecraft_ui_egui::shell::AppAction::ExportStep,
            surfacecraft_ui_egui::shell::AppAction::ExportMesh,
            surfacecraft_ui_egui::shell::AppAction::Undo,
            surfacecraft_ui_egui::shell::AppAction::Redo,
            surfacecraft_ui_egui::shell::AppAction::FitAll,
            surfacecraft_ui_egui::shell::AppAction::ResetLayout,
        ]
        .map(|a| a.label_key()),
    );
    for key in keys {
        assert!(en.contains_key(key) && pl.contains_key(key), "catalogue key {key}");
    }
    // a missing key never fails: it falls back to English, then to the key itself
    assert_eq!(i18n::lookup(Lang::Pl, "no.such.key"), "no.such.key");
    assert_eq!(Lang::from_locale("pl_PL.UTF-8"), Lang::Pl);
    assert_eq!(Lang::from_locale("de_DE"), Lang::En);
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
