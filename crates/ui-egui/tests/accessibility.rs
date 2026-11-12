//! Accessibility of the window as a screen reader sees it (the AccessKit tree): every button has
//! a name in both languages, an open form names its fields and offers OK and Cancel, the 3D view
//! has a role and a label, and Tab walks an open form in a sensible order.
use egui::Role;
use egui_kittest::Harness;
use egui_kittest::kittest::NodeT;
use serde_json::json;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::i18n::{self, Lang};
use surfacecraft_ui_egui::workflow::{self, Entry, Stage};

/// Roles of the controls a user types into or ticks.
const FIELD_ROLES: [Role; 4] = [Role::TextInput, Role::SpinButton, Role::CheckBox, Role::ComboBox];

/// One node of the AccessKit tree: its role, its name and whether it has keyboard focus.
#[derive(Debug, Clone)]
struct Named {
    role: Role,
    name: String,
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
                focused: node.is_focused(),
            }
        })
        .collect()
}

/// The buttons of the tree with no accessible name, listed with the context they were found in.
fn unnamed_buttons(h: &Harness<'static, SurfaceCraftApp>, context: &str) -> Vec<String> {
    tree(h)
        .iter()
        .enumerate()
        .filter(|(_, n)| n.role == Role::Button && n.name.trim().is_empty())
        .map(|(at, _)| format!("{context}: button number {at} of the tree has no name"))
        .collect()
}

fn assert_every_button_is_named(lang: Lang) {
    let mut h = harness(lang);
    let mut unnamed = Vec::new();
    for stage in Stage::ALL {
        h.state_mut().stage = stage;
        h.run();
        unnamed.extend(unnamed_buttons(&h, &format!("{lang:?} ribbon {stage:?}")));
    }
    // the sketch tool bar only exists while a sketch is edited
    h.state_mut().new_sketch_on([0.0, 0.0, 1.0]);
    h.run();
    h.run();
    unnamed.extend(unnamed_buttons(&h, &format!("{lang:?} sketch tool bar")));
    // the check must see the buttons, or an empty tree would pass it
    let buttons = tree(&h).iter().filter(|n| n.role == Role::Button).count();
    assert!(buttons >= 20, "the tree has the ribbon and bar buttons, found {buttons}");
    assert!(unnamed.is_empty(), "buttons without an accessible name:\n{}", unnamed.join("\n"));
}

#[test]
fn every_button_of_every_ribbon_tab_and_the_sketch_bar_is_named_in_english() {
    assert_every_button_is_named(Lang::En);
}

#[test]
fn every_button_of_every_ribbon_tab_and_the_sketch_bar_is_named_in_polish() {
    assert_every_button_is_named(Lang::Pl);
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

#[test]
fn tab_moves_from_the_form_fields_to_cancel_and_ok() {
    let mut h = harness(Lang::En);
    h.state_mut().choose_command("sketch.create");
    h.run();
    h.run();
    // the first text field of the form, not the ribbon's search field
    let search = i18n::lookup(Lang::En, "ribbon.search");
    let first_field = h
        .root()
        .children_recursive()
        .find(|n| {
            let access = n.accesskit_node();
            FIELD_ROLES.contains(&access.role()) && access.label().is_some_and(|label| label != search)
        })
        .unwrap_or_else(|| panic!("a field in the form: {:?}", tree(&h)));
    first_field.focus();
    h.run();
    let mut visited: Vec<String> = Vec::new();
    for _ in 0..80 {
        h.key_press(egui::Key::Tab);
        h.run();
        if let Some(focused) = tree(&h).into_iter().find(|n| n.focused) {
            visited.push(format!("{:?}:{}", focused.role, focused.name));
        }
    }
    let cancel = format!("Button:{}", i18n::lookup(Lang::En, "form.cancel"));
    let ok = format!("Button:{}", i18n::lookup(Lang::En, "form.ok"));
    let is_field = |entry: &str| FIELD_ROLES.iter().any(|role| entry.starts_with(&format!("{role:?}:")));
    let first_button = visited
        .iter()
        .position(|v| v.starts_with("Button:"))
        .unwrap_or_else(|| panic!("Tab reaches a button: {visited:?}"));
    assert!(
        visited[..first_button].iter().all(|v| is_field(v)),
        "only form fields come before the first button: {visited:?}"
    );
    assert!(
        visited[first_button] == ok || visited[first_button] == cancel,
        "the first button after the fields is OK or Cancel: {visited:?}"
    );
    assert!(visited.contains(&ok) && visited.contains(&cancel), "Tab reaches OK and Cancel: {visited:?}");
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
        for key in ["deviation.histogram", "deviation.scale"] {
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
    assert!(
        visited.iter().any(|v| v == "Button:Show / hide: part"),
        "the show/hide button names its object: {visited:?}"
    );
    assert!(visited.iter().any(|v| v == "Button:Delete: part"), "the delete button names its object: {visited:?}");
}
