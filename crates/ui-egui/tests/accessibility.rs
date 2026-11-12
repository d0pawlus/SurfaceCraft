//! Accessibility of the window as a screen reader sees it (the AccessKit tree): every button has
//! a name in both languages, an open form names its fields and offers OK and Cancel, the 3D view
//! has a role and a label, and Tab walks an open form in a sensible order.
use egui::Role;
use egui_kittest::Harness;
use egui_kittest::kittest::NodeT;
use serde_json::json;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::command_panel::FOOTER_HEIGHT_GUESS;
use surfacecraft_ui_egui::i18n::{self, Lang};
use surfacecraft_ui_egui::workflow::{self, Entry, Stage};

/// Roles of the controls a user types into or ticks.
const FIELD_ROLES: [Role; 4] = [Role::TextInput, Role::SpinButton, Role::CheckBox, Role::ComboBox];

/// One node of the AccessKit tree: its role, its name and whether it has keyboard focus.
#[derive(Debug, Clone)]
struct Named {
    role: Role,
    name: String,
    /// Named by another node (`labelled_by`), which a screen reader reads as the name.
    labelled: bool,
    focused: bool,
}

fn harness(lang: Lang) -> Harness<'static, SurfaceCraftApp> {
    let mut app = SurfaceCraftApp::headless();
    app.settings.lang = lang;
    harness_with(app)
}

fn harness_with(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run();
    h
}

/// Every node of the tree, in reading order.
fn tree(h: &Harness<'static, SurfaceCraftApp>) -> Vec<Named> {
    h.root()
        .children_recursive()
        .map(|node| {
            let access = node.accesskit_node();
            Named {
                role: access.role(),
                name: access.label().unwrap_or_default().to_owned(),
                labelled: access.labelled_by().next().is_some(),
                focused: node.is_focused(),
            }
        })
        .collect()
}

/// Roles of the controls a user operates. Each one needs a name, or another node that labels it.
const CONTROL_ROLES: [Role; 13] = [
    Role::Button,
    Role::CheckBox,
    Role::ComboBox,
    Role::TextInput,
    Role::MultilineTextInput,
    Role::SpinButton,
    Role::Slider,
    Role::Tab,
    Role::MenuItem,
    Role::RadioButton,
    Role::Link,
    Role::DisclosureTriangle,
    Role::ListBoxOption,
];

/// The controls of the tree that have no accessible name, listed by role and position.
fn unnamed_controls(h: &Harness<'static, SurfaceCraftApp>, context: &str) -> Vec<String> {
    tree(h)
        .iter()
        .enumerate()
        .filter(|(_, n)| CONTROL_ROLES.contains(&n.role) && n.name.trim().is_empty() && !n.labelled)
        .map(|(at, n)| format!("{context}: {:?} number {at} of the tree has no name", n.role))
        .collect()
}

/// Every control of the ribbon, the navigation bar, the sketch tool bar, the object list and an
/// open form has a name in the language.
fn assert_every_control_is_named(lang: Lang) {
    let mut app = SurfaceCraftApp::headless();
    app.settings.lang = lang;
    app.run_command("mesh.synth", json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 4, "name": "part"}))
        .expect("an object");
    let mut h = harness_with(app);
    let mut unnamed = Vec::new();
    for stage in Stage::ALL {
        h.state_mut().stage = stage;
        h.run();
        unnamed.extend(unnamed_controls(&h, &format!("{lang:?} ribbon {stage:?}")));
    }
    // the sketch tool bar only exists while a sketch is edited
    h.state_mut().new_sketch_on([0.0, 0.0, 1.0]);
    h.run();
    h.run();
    unnamed.extend(unnamed_controls(&h, &format!("{lang:?} sketch tool bar")));
    // an open form: its fields, its in-form buttons, OK and Cancel
    h.state_mut().choose_command("align.datum");
    h.run();
    h.run();
    unnamed.extend(unnamed_controls(&h, &format!("{lang:?} open form")));
    // a form with number fields (spin buttons) for a matrix
    h.state_mut().choose_command("mesh.transform");
    h.run();
    h.run();
    unnamed.extend(unnamed_controls(&h, &format!("{lang:?} transform form")));
    // the check must see the controls, or an empty tree would pass it
    let buttons = tree(&h).iter().filter(|n| n.role == Role::Button).count();
    assert!(buttons >= 20, "the tree has the ribbon and bar buttons, found {buttons}");
    assert!(unnamed.is_empty(), "controls without an accessible name:\n{}", unnamed.join("\n"));
}

#[test]
fn every_control_is_named_in_english() {
    assert_every_control_is_named(Lang::En);
}

#[test]
fn every_control_is_named_in_polish() {
    assert_every_control_is_named(Lang::Pl);
}

#[test]
fn an_open_form_names_each_field_and_offers_ok_and_cancel() {
    let mut h = harness(Lang::En);
    h.state_mut().choose_command("sketch.create");
    h.run();
    h.run();
    let nodes = tree(&h);
    let fields: Vec<&Named> = nodes.iter().filter(|n| FIELD_ROLES.contains(&n.role)).collect();
    assert!(!fields.is_empty(), "the form has fields");
    let unnamed: Vec<&Named> = fields.iter().copied().filter(|n| n.name.trim().is_empty()).collect();
    assert!(unnamed.is_empty(), "fields without a label: {unnamed:?}");
    let has_button = |name: &str| nodes.iter().any(|n| n.role == Role::Button && n.name == name);
    assert!(has_button(i18n::lookup(Lang::En, "form.ok")), "OK is a named button");
    assert!(has_button(i18n::lookup(Lang::En, "form.cancel")), "Cancel is a named button");
}

#[test]
fn the_3d_view_has_a_role_and_a_translated_label() {
    for lang in [Lang::En, Lang::Pl] {
        let key = "viewport.name";
        assert!(i18n::catalog(lang).contains_key(key), "{lang:?} catalogue has {key}");
        let mut h = harness(lang);
        h.run();
        let label = i18n::lookup(lang, key);
        let view = tree(&h).into_iter().find(|n| n.name == label);
        let view = view.unwrap_or_else(|| panic!("{lang:?}: no node labelled {label:?}"));
        assert_eq!(view.role, Role::Canvas, "{lang:?}: the 3D view is a canvas");
    }
}

/// Tools whose forms are checked for Tab order: a short form, a long one and one with optional
/// fields.
const FORM_TOOLS: [&str; 3] = ["sketch.create", "align.datum", "mesh.synth"];

/// The name of the first form field of the open tool: the first field that is not the ribbon's
/// search field.
fn first_form_field<'a>(h: &'a Harness<'static, SurfaceCraftApp>, lang: Lang) -> egui_kittest::Node<'a> {
    let search = i18n::lookup(lang, "ribbon.search");
    h.root()
        .children_recursive()
        .find(|n| {
            let access = n.accesskit_node();
            FIELD_ROLES.contains(&access.role()) && access.label().is_some_and(|label| label != search)
        })
        .expect("a field in the form")
}

/// Focuses the first field of the open form and presses Tab until the walk has passed the footer.
/// Returns the nodes Tab visited, as "Role:name".
fn tab_through_form(h: &mut Harness<'static, SurfaceCraftApp>, lang: Lang) -> Vec<String> {
    first_form_field(h, lang).focus();
    h.run();
    let mut visited: Vec<String> = Vec::new();
    for _ in 0..80 {
        h.key_press(egui::Key::Tab);
        h.run();
        if let Some(focused) = tree(h).into_iter().find(|n| n.focused) {
            visited.push(format!("{:?}:{}", focused.role, focused.name));
        }
    }
    visited
}

/// The Tab order the UI uses in a tool: the form's fields (the in-form buttons such as "Pick in
/// viewport" may come among them), then the preview switch, then OK, then Cancel. OK is drawn to
/// the right of Cancel, but Tab reaches it first.
fn assert_form_then_footer(visited: &[String], lang: Lang, context: &str) {
    let ok = format!("Button:{}", i18n::lookup(lang, "form.ok"));
    let cancel = format!("Button:{}", i18n::lookup(lang, "form.cancel"));
    let ok_at =
        visited.iter().position(|v| *v == ok).unwrap_or_else(|| panic!("{context}: Tab reaches OK: {visited:?}"));
    assert!(ok_at > 0, "{context}: a field comes before OK: {visited:?}");
    let before_ok = &visited[ok_at - 1];
    assert!(
        FIELD_ROLES.iter().any(|role| before_ok.starts_with(&format!("{role:?}:"))),
        "{context}: the entry before OK is a form field or the preview switch, not {before_ok:?}: {visited:?}"
    );
    assert_eq!(visited.get(ok_at + 1), Some(&cancel), "{context}: Cancel follows OK: {visited:?}");
}

#[test]
fn tab_visits_the_fields_then_ok_and_cancel_in_every_checked_form_and_language() {
    for lang in [Lang::En, Lang::Pl] {
        for tool in FORM_TOOLS {
            let mut app = SurfaceCraftApp::headless();
            app.settings.lang = lang;
            app.choose_command(tool);
            let mut h = harness_with(app);
            h.run();
            h.run();
            let visited = tab_through_form(&mut h, lang);
            assert_form_then_footer(&visited, lang, &format!("{lang:?} {tool}"));
        }
    }
}

#[test]
fn the_tab_order_is_unchanged_with_a_pinned_failure() {
    for lang in [Lang::En, Lang::Pl] {
        let mut app = failed_transform();
        app.settings.lang = lang;
        let mut h = harness_with(app);
        h.run();
        h.run();
        let visited = tab_through_form(&mut h, lang);
        assert_form_then_footer(&visited, lang, &format!("{lang:?} with a failure pinned"));
    }
}

/// The report of `mesh.synth` (a noisy plane) against the clean plane, as in the Deviation tests.
fn measured_app() -> SurfaceCraftApp {
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

#[test]
fn the_deviation_histogram_and_colour_scale_are_labelled_images() {
    for lang in [Lang::En, Lang::Pl] {
        let mut app = measured_app();
        app.settings.lang = lang;
        let mut h = Harness::builder()
            .with_size((1600.0, 1600.0))
            .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
        h.run();
        h.run();
        for key in ["deviation.histogram.name", "deviation.scale"] {
            let label = i18n::lookup(lang, key);
            assert!(i18n::catalog(lang).contains_key(key), "{lang:?} catalogue has {key}");
            let chart = tree(&h).into_iter().find(|n| n.name == label);
            let chart = chart.unwrap_or_else(|| panic!("{lang:?}: no node labelled {label:?}"));
            assert_eq!(chart.role, Role::Image, "{lang:?}: {label} is an image");
        }
    }
}

#[test]
fn tab_reaches_every_ribbon_tab_and_the_first_tools() {
    let mut h = harness(Lang::En);
    let mut visited: Vec<String> = Vec::new();
    for _ in 0..200 {
        h.key_press(egui::Key::Tab);
        h.run();
        if let Some(focused) = tree(&h).into_iter().find(|n| n.focused) {
            visited.push(focused.name);
        }
    }
    for stage in Stage::ALL {
        let title = i18n::lookup(Lang::En, stage.title_key());
        assert!(visited.iter().any(|v| v == title), "Tab reaches the {title} tab: {visited:?}");
    }
    // the tools of the open stage (Scan at start) are on the way too
    let scan_tools: Vec<String> = Stage::Scan
        .entries()
        .iter()
        .filter_map(|entry| match entry {
            Entry::Cmd(id) => surfacecraft_engine::cmd::find(id).map(workflow::label_of),
            _ => None,
        })
        .collect();
    assert!(!scan_tools.is_empty(), "the Scan stage has tools");
    for tool in &scan_tools {
        assert!(visited.contains(tool), "Tab reaches the tool {tool}: {visited:?}");
    }
}

#[test]
fn tab_reaches_the_object_list_rows() {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 4, "name": "part"}))
        .expect("an object");
    let mut h = harness_with(app);
    let mut visited: Vec<String> = Vec::new();
    for _ in 0..300 {
        h.key_press(egui::Key::Tab);
        h.run();
        if let Some(focused) = tree(&h).into_iter().find(|n| n.focused) {
            visited.push(format!("{:?}:{}", focused.role, focused.name));
        }
    }
    assert!(visited.iter().any(|v| v.contains("part")), "Tab reaches the object row 'part': {visited:?}");
}

/// A failed `mesh.transform` (scale -1 is refused by the engine): the tool stays open with the
/// failure shown above its buttons, as the `p6-form-error` screenshot does.
fn failed_transform() -> SurfaceCraftApp {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "bracket", "name": "scan"})).expect("the scan");
    app.choose_command("mesh.transform");
    let matrix = [[1.0, 0.0, 0.0, 5.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
    app.palette_params = json!({"object": "scan", "matrix": matrix, "scale": -1.0}).to_string();
    let _ = app.run_palette();
    app
}

/// The rectangle of the button named `name` in the tree.
fn button_rect(h: &Harness<'static, SurfaceCraftApp>, name: &str) -> egui::Rect {
    h.root()
        .children_recursive()
        .find(|n| {
            let access = n.accesskit_node();
            access.role() == Role::Button && access.label().is_some_and(|label| label == name)
        })
        .unwrap_or_else(|| panic!("a button named {name:?}"))
        .rect()
}

/// The footer (OK, Cancel) must lie inside the Properties panel and below the form, and the footer
/// must be no taller than a normal footer: the rows of messages, buttons and the JSON/Form switch.
fn assert_footer_fits(h: &Harness<'static, SurfaceCraftApp>, context: &str) {
    let panel = h.state().tool_panel_rect.expect("the Properties panel was drawn");
    let footer = h.state().tool_footer_rect.expect("the tool footer was drawn");
    for name in [i18n::lookup(Lang::En, "form.ok"), i18n::lookup(Lang::En, "form.cancel")] {
        let button = button_rect(h, name);
        assert!(
            button.bottom() <= panel.bottom() + 0.5,
            "{context}: {name} ends at {} below the panel bottom {}",
            button.bottom(),
            panel.bottom()
        );
        assert!(button.top() >= footer.top() - 0.5, "{context}: {name} is in the footer");
    }
    assert!(footer.bottom() <= panel.bottom() + 0.5, "{context}: the footer ends inside the panel");
}

/// The Properties panel's first frame: nothing is carried over from an earlier frame.
fn first_frame(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.step();
    h
}

#[test]
fn the_tool_footer_fits_the_panel_on_the_first_frame_of_a_long_form() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("align.datum");
    let mut h = first_frame(app);
    assert_footer_fits(&h, "a long form, first frame");
    h.run();
    assert_footer_fits(&h, "a long form, settled");
}

#[test]
fn the_tool_footer_fits_the_panel_on_the_first_frame_after_a_multi_line_failure() {
    let mut h = first_frame(failed_transform());
    assert_footer_fits(&h, "after a failed run, first frame");
    h.run();
    assert_footer_fits(&h, "after a failed run, settled");
}

#[test]
fn the_tool_footer_fits_the_panel_on_the_first_frame_after_switching_from_a_failed_tool() {
    let mut h = first_frame(failed_transform());
    h.run();
    h.state_mut().choose_command("sketch.create");
    h.step();
    assert_footer_fits(&h, "after switching from a failed tool, first frame");
    h.run();
    assert_footer_fits(&h, "after switching from a failed tool, settled");
}

#[test]
fn the_tool_footer_fits_the_panel_in_the_frame_that_shows_a_click_failure() {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "bracket", "name": "scan"})).expect("the scan");
    app.choose_command("mesh.transform");
    let matrix = [[1.0, 0.0, 0.0, 5.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
    app.palette_params = json!({"object": "scan", "matrix": matrix, "scale": -1.0}).to_string();
    let mut h = harness_with(app);
    h.run();
    // OK runs the failing command; the frame after the click shows its failure
    let ok = i18n::lookup(Lang::En, "form.ok");
    h.root()
        .children_recursive()
        .find(|n| n.accesskit_node().role() == Role::Button && n.accesskit_node().label().is_some_and(|l| l == ok))
        .expect("the OK button")
        .click();
    h.step();
    h.step();
    assert_footer_fits(&h, "in the frame after a click failure");
    h.run();
    assert_footer_fits(&h, "after a click failure, settled");
}

#[test]
fn the_footer_height_guess_covers_a_normal_footer() {
    let mut app = SurfaceCraftApp::headless();
    app.choose_command("align.datum");
    let h = first_frame(app);
    let footer = h.state().tool_footer_rect.expect("the tool footer was drawn");
    assert!(
        footer.height() <= FOOTER_HEIGHT_GUESS,
        "a normal footer is {} px, the guess is {FOOTER_HEIGHT_GUESS} px",
        footer.height()
    );
}

#[test]
fn object_row_buttons_name_their_object_in_both_languages() {
    for lang in [Lang::En, Lang::Pl] {
        let mut app = SurfaceCraftApp::headless();
        app.settings.lang = lang;
        app.run_command("mesh.synth", json!({"shape": "plane", "size": [40.0, 40.0], "resolution": 4, "name": "part"}))
            .expect("an object");
        let mut h = harness_with(app);
        h.run();
        let names: Vec<String> = tree(&h).into_iter().map(|n| n.name).collect();
        let show = i18n::lookup(lang, "objects.show_hide_named").replace("{name}", "part");
        let delete = i18n::lookup(lang, "objects.delete_named").replace("{name}", "part");
        assert!(names.contains(&show), "{lang:?}: a show/hide button named {show:?}");
        assert!(names.contains(&delete), "{lang:?}: a delete button named {delete:?}");
    }
}
