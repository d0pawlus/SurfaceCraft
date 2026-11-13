//! P9-U1: the unit of a scan import in the interface. The form shows the choice with its labels in
//! both languages, starts with the unit chosen last time, and a size that looks like a unit mistake
//! is said in words in the status line.
use serde_json::{Value, json};
use surfacecraft_doc::{LengthUnit, base64};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::forms::{self, Kind};
use surfacecraft_ui_egui::i18n::{self, Lang};
use surfacecraft_ui_egui::settings::UiSettings;

/// A triangle 40 mm × 30 mm (as numbers: 0.04 × 0.03), as an OBJ file encoded for the form (`data`).
fn triangle_data() -> String {
    base64::encode(b"v 0 0 0\nv 0.04 0 0\nv 0 0.03 0\nf 1 2 3\n")
}

fn params(app: &SurfaceCraftApp) -> Value {
    serde_json::from_str(&app.palette_params).unwrap()
}

#[test]
fn the_units_choice_has_its_labels_in_both_languages() {
    let codes = ["mm", "cm", "m", "in", "ft", "um"];
    let english: Vec<String> = codes.iter().map(|u| i18n::lookup(Lang::En, &format!("enum.{u}")).to_string()).collect();
    let polish: Vec<String> = codes.iter().map(|u| i18n::lookup(Lang::Pl, &format!("enum.{u}")).to_string()).collect();
    assert_eq!(english, ["Millimetres", "Centimetres", "Metres", "Inches", "Feet", "Micrometres"]);
    assert_eq!(polish, ["Milimetry", "Centymetry", "Metry", "Cale", "Stopy", "Mikrometry"]);
}

#[test]
fn the_import_form_shows_the_units_as_a_choice_of_six() {
    let schema = (surfacecraft_engine::cmd::find("mesh.import").unwrap().input)();
    let (kind, _, _) = forms::classify(&schema, &schema["properties"]["units"]);
    assert!(matches!(kind, Kind::Enum(ref options) if options.len() == 6), "{kind:?}");
}

#[test]
fn the_import_form_starts_with_the_unit_chosen_last_time() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("mesh.import");
    assert_eq!(params(&app)["units"], "mm");
    app.palette_params = json!({"data": triangle_data(), "format": "obj", "name": "a", "units": "in"}).to_string();
    app.run_palette();
    assert_eq!(app.settings.import_units, LengthUnit::In);
    app.choose_command("mesh.import");
    assert_eq!(params(&app)["units"], "in");
}

#[test]
fn a_metre_file_imported_without_units_says_so_in_the_status_line() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("mesh.import");
    app.palette_params = json!({"data": triangle_data(), "format": "obj", "name": "metres"}).to_string();
    app.run_palette();
    assert!(app.status.starts_with("The largest side is 0.0400 mm"), "{}", app.status);
    i18n::set_lang(Lang::Pl);
    let polish = surfacecraft_ui_egui::import_units::warning_text(&json!({
        "scale_warning": {"problem": "metres_or_inches", "extent": 0.04}
    }));
    i18n::set_lang(Lang::En);
    assert!(polish.is_some_and(|t| t.starts_with("Najdłuższy wymiar to 0.0400 mm")));
}

#[test]
fn a_form_import_in_metres_is_announced_in_the_status_line() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("mesh.import");
    app.palette_params = json!({"data": triangle_data(), "format": "obj", "name": "m", "units": "m"}).to_string();
    app.run_palette();
    assert!(app.status.contains("Metres") && app.status.contains("1000"), "{}", app.status);
}

#[test]
fn the_unit_is_not_kept_across_a_restart() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("mesh.import");
    app.palette_params = json!({"data": triangle_data(), "format": "obj", "name": "m", "units": "m"}).to_string();
    app.run_palette();
    let saved = UiSettings::from_json(&app.settings.to_json());
    assert_eq!(saved.import_units, LengthUnit::Mm, "a non-mm unit is never persisted");
}

#[test]
fn a_fresh_app_starts_with_millimetres_even_when_saved_settings_say_metres() {
    let saved = UiSettings::from_json(r#"{"import_units":"m"}"#);
    let app = SurfaceCraftApp::with_settings(saved);
    assert_eq!(app.settings.import_units, LengthUnit::Mm);
}

#[test]
fn the_form_preselects_the_session_unit_and_says_so() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("mesh.import");
    app.palette_params = json!({"data": triangle_data(), "format": "obj", "name": "m", "units": "cm"}).to_string();
    app.run_palette();
    app.status.clear();
    app.choose_command("mesh.import");
    assert_eq!(serde_json::from_str::<Value>(&app.palette_params).unwrap()["units"], "cm");
    assert!(app.status.contains("Centimetres"), "the preselected unit is shown: {}", app.status);
}

#[test]
fn menu_and_inbox_imports_always_use_millimetres_unless_the_user_picks() {
    let params = surfacecraft_ui_egui::import_units::file_import_params(json!({"path": "scan.stl"}));
    assert_eq!(params["units"], "mm");
}
