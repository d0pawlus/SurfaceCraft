//! The scan-to-CAD workflow as the ribbon shows it: the stages in the order the work goes, the
//! tools each stage promotes and where every other command of the engine is found.
//!
//! The ribbon is data: a stage lists its promoted entries, and every command of the registry that
//! belongs to the stage and is not promoted lands in the stage's "More" list, so a command added
//! to the engine is reachable from the UI without touching this file.
use crate::i18n;
use crate::shell::AppAction;
use surfacecraft_engine::cmd::CommandSpec;

/// A stage of the work, a tab of the ribbon.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
pub enum Stage {
    /// Bring a scan in and clean it.
    #[default]
    Scan,
    /// Put the scan on the world axes.
    Align,
    /// Split the scan into regions.
    Segment,
    /// Fit reference shapes and cut sections.
    Shapes,
    /// Draw and constrain sketches.
    Sketch,
    /// Build the solid from features.
    Solid,
    /// Fit freeform surfaces.
    Surface,
    /// Compare the model with the scan.
    Inspect,
    /// Write the result out.
    Export,
}

/// One thing a stage shows in the ribbon.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Entry {
    /// An engine command, opened in the tool panel.
    Cmd(&'static str),
    /// A shell action (a file dialog, not a parameter form).
    Act(AppAction),
    /// A new sketch on one of the world planes.
    NewSketch,
    /// Edit the selected sketch.
    EditSketch,
    /// A gap between two groups of tools.
    Gap,
}

use Entry::{Act, Cmd, Gap};

const SCAN: &[Entry] = &[
    Act(AppAction::ImportMesh),
    Gap,
    Cmd("mesh.clean"),
    Cmd("mesh.remove_outliers"),
    Cmd("mesh.remove_small"),
    Cmd("mesh.fill_holes"),
    Gap,
    Cmd("mesh.smooth"),
    Cmd("mesh.decimate"),
];
const ALIGN: &[Entry] =
    &[Cmd("align.principal"), Cmd("align.datum"), Cmd("align.n_point"), Gap, Cmd("align.icp"), Cmd("align.center")];
const SEGMENT: &[Entry] = &[
    Cmd("region.auto_segment"),
    Cmd("region.create"),
    Gap,
    Cmd("region.grow"),
    Cmd("region.enlarge"),
    Cmd("region.shrink"),
    Gap,
    Cmd("region.merge"),
    Cmd("region.split"),
];
const SHAPES: &[Entry] = &[
    Cmd("primitive.fit"),
    Cmd("primitive.symmetry"),
    Gap,
    Cmd("primitive.auto_constrain"),
    Cmd("primitive.trim_to_region"),
    Gap,
    Cmd("section.stack"),
];
const SKETCH: &[Entry] =
    &[Entry::NewSketch, Entry::EditSketch, Gap, Cmd("sketch.from_section"), Cmd("sketch.fit_chains")];
const SOLID: &[Entry] = &[
    Cmd("feature.extrude"),
    Cmd("feature.revolve"),
    Cmd("feature.sweep"),
    Cmd("feature.loft"),
    Gap,
    Cmd("feature.fillet"),
    Cmd("feature.chamfer"),
    Cmd("feature.shell"),
    Cmd("feature.draft"),
    Gap,
    Cmd("feature.boolean"),
    Cmd("feature.pattern"),
    Cmd("feature.mirror"),
    Gap,
    Cmd("body.from_regions"),
];
const SURFACE: &[Entry] = &[
    Cmd("surface.fit"),
    Cmd("surface.auto"),
    Cmd("surface.from_curves"),
    Gap,
    Cmd("quad.create"),
    Cmd("quad.retopo"),
    Gap,
    Cmd("surface.thicken"),
    Cmd("surface.trim_to_region"),
];
const INSPECT: &[Entry] = &[
    Cmd("deviation.compute"),
    Cmd("deviation.hotspots"),
    Cmd("deviation.stats"),
    Gap,
    Cmd("deviation.noise"),
    Cmd("deviation.to_primitive"),
    Cmd("deviation.clear"),
];
const EXPORT: &[Entry] =
    &[Act(AppAction::ExportStep), Cmd("export.iges"), Act(AppAction::ExportMesh), Gap, Cmd("export.section_sheet")];

/// Commands a file dialog of the shell already covers: the ribbon shows the dialog, and the form
/// with a typed path stays reachable through the search and the Tools menu.
const DIALOG_TWINS: &[&str] = &["mesh.import", "mesh.export", "export.step", "import.step"];

impl Stage {
    /// Every stage, in the order of the work.
    pub const ALL: [Stage; 9] = [
        Stage::Scan,
        Stage::Align,
        Stage::Segment,
        Stage::Shapes,
        Stage::Sketch,
        Stage::Solid,
        Stage::Surface,
        Stage::Inspect,
        Stage::Export,
    ];

    /// The stable name of the stage (catalogue keys, settings).
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Stage::Scan => "scan",
            Stage::Align => "align",
            Stage::Segment => "segment",
            Stage::Shapes => "shapes",
            Stage::Sketch => "sketch",
            Stage::Solid => "solid",
            Stage::Surface => "surface",
            Stage::Inspect => "inspect",
            Stage::Export => "export",
        }
    }

    /// The catalogue key of the tab's title.
    #[must_use]
    pub fn title_key(self) -> &'static str {
        match self {
            Stage::Scan => "stage.scan",
            Stage::Align => "stage.align",
            Stage::Segment => "stage.segment",
            Stage::Shapes => "stage.shapes",
            Stage::Sketch => "stage.sketch",
            Stage::Solid => "stage.solid",
            Stage::Surface => "stage.surface",
            Stage::Inspect => "stage.inspect",
            Stage::Export => "stage.export",
        }
    }

    /// The catalogue key of the sentence that says what the stage is for.
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            Stage::Scan => "stage.scan.hint",
            Stage::Align => "stage.align.hint",
            Stage::Segment => "stage.segment.hint",
            Stage::Shapes => "stage.shapes.hint",
            Stage::Sketch => "stage.sketch.hint",
            Stage::Solid => "stage.solid.hint",
            Stage::Surface => "stage.surface.hint",
            Stage::Inspect => "stage.inspect.hint",
            Stage::Export => "stage.export.hint",
        }
    }

    /// The icon of the stage (also the icon of its tools that have none of their own).
    #[must_use]
    pub fn icon(self) -> &'static str {
        match self {
            Stage::Scan => "stage-scan",
            Stage::Align => "stage-align",
            Stage::Segment => "stage-regions",
            Stage::Shapes => "stage-shapes",
            Stage::Sketch => "stage-sketch",
            Stage::Solid => "stage-solid",
            Stage::Surface => "stage-surface",
            Stage::Inspect => "stage-inspect",
            Stage::Export => "export",
        }
    }

    /// The entries the ribbon promotes for this stage, in order.
    #[must_use]
    pub fn entries(self) -> &'static [Entry] {
        match self {
            Stage::Scan => SCAN,
            Stage::Align => ALIGN,
            Stage::Segment => SEGMENT,
            Stage::Shapes => SHAPES,
            Stage::Sketch => SKETCH,
            Stage::Solid => SOLID,
            Stage::Surface => SURFACE,
            Stage::Inspect => INSPECT,
            Stage::Export => EXPORT,
        }
    }

    /// The stage a command belongs to (`None`: not a modelling tool, e.g. `app.*` or `doc.*`).
    #[must_use]
    pub fn of(id: &str) -> Option<Stage> {
        if matches!(id, "mesh.export") {
            return Some(Stage::Export);
        }
        if matches!(id, "import.step") {
            return Some(Stage::Scan);
        }
        let area = id.split('.').next().unwrap_or("");
        match area {
            "mesh" => Some(Stage::Scan),
            "align" => Some(Stage::Align),
            "region" => Some(Stage::Segment),
            "primitive" | "section" => Some(Stage::Shapes),
            "sketch" => Some(Stage::Sketch),
            "feature" | "body" => Some(Stage::Solid),
            "surface" | "quad" => Some(Stage::Surface),
            "deviation" => Some(Stage::Inspect),
            "export" => Some(Stage::Export),
            _ => None,
        }
    }

    /// Whether the ribbon promotes command `id` in this stage.
    #[must_use]
    pub fn promotes(self, id: &str) -> bool {
        self.entries().iter().any(|e| matches!(e, Entry::Cmd(c) if *c == id))
    }

    /// The commands of this stage the ribbon does not promote (its "More" list), in registry order.
    #[must_use]
    pub fn more(self, commands: &'static [CommandSpec]) -> Vec<&'static CommandSpec> {
        commands
            .iter()
            .filter(|c| Stage::of(c.id) == Some(self) && !self.promotes(c.id) && !DIALOG_TWINS.contains(&c.id))
            .collect()
    }

    /// Every command of this stage (the Tools menu), in registry order.
    #[must_use]
    pub fn all(self, commands: &'static [CommandSpec]) -> Vec<&'static CommandSpec> {
        commands.iter().filter(|c| Stage::of(c.id) == Some(self)).collect()
    }
}

/// The icon of command `id`: its own, or its stage's.
#[must_use]
pub fn icon_of(id: &str) -> &'static str {
    match id {
        "mesh.import" | "import.step" => "import",
        "mesh.clean" => "clean",
        "mesh.remove_outliers" => "spikes",
        "mesh.remove_small" => "small-pieces",
        "mesh.fill_holes" => "fill-holes",
        "mesh.smooth" => "smooth",
        "mesh.decimate" => "decimate",
        "mesh.synth" => "sample",
        "mesh.transform" => "tool-move",
        "mesh.info" | "primitive.info" | "surface.info" => "info",
        "align.principal" => "align-axes",
        "align.datum" => "align-321",
        "align.n_point" => "align-points",
        "align.icp" => "align-fine",
        "align.center" => "align-center",
        "region.auto_segment" | "region.resegment" => "auto-segment",
        "region.create" | "region.insert" => "region-new",
        "region.grow" => "region-grow",
        "region.enlarge" => "region-enlarge",
        "region.shrink" => "region-shrink",
        "region.merge" => "region-merge",
        "region.split" | "region.separate" => "region-split",
        "primitive.fit" => "fit-shape",
        "primitive.symmetry" => "symmetry",
        "primitive.auto_constrain" | "sketch.constrain" | "sketch.edit_constraint" => "constrain",
        "primitive.trim_to_region" | "surface.trim_to_region" | "feature.split" => "trim",
        "section.stack" => "sections",
        "sketch.create" => "sketch-new",
        "sketch.from_section" => "sketch-section",
        "sketch.fit_chains" => "sketch-chains",
        "feature.extrude" => "extrude",
        "feature.revolve" | "feature.suggest_axis" => "revolve",
        "feature.sweep" => "sweep",
        "feature.loft" => "loft",
        "feature.fillet" | "body.fillet" | "feature.suggest_fillet" => "fillet",
        "feature.chamfer" | "body.chamfer" => "chamfer",
        "feature.shell" => "shell",
        "feature.draft" | "feature.suggest_draft" => "draft",
        "feature.boolean" => "boolean",
        "feature.pattern" | "feature.path_pattern" => "pattern",
        "feature.mirror" => "mirror",
        "feature.tree" | "feature.replay" => "history",
        "body.from_regions" | "body.plan_regions" => "solid-from-regions",
        "surface.fit" => "surface-fit",
        "surface.auto" => "surface-auto",
        "surface.from_curves" => "surface-curves",
        "surface.thicken" => "thicken",
        "quad.create" | "quad.split" | "quad.merge" | "quad.crease" | "quad.snap" => "quad",
        "quad.retopo" => "retopo",
        "deviation.compute" => "deviation",
        "deviation.hotspots" => "hotspots",
        "deviation.stats" => "stats",
        "deviation.noise" => "noise",
        "deviation.clear" => "clear",
        "export.step" => "export-step",
        "export.iges" => "export-iges",
        "mesh.export" => "export-mesh",
        "export.section_sheet" => "export-sheet",
        other => Stage::of(other).map_or("play", Stage::icon),
    }
}

/// The icon of a shell action in the ribbon.
#[must_use]
pub fn action_icon(action: AppAction) -> &'static str {
    match action {
        AppAction::ExportStep => "export-step",
        AppAction::ExportMesh => "export-mesh",
        other => other.icon().unwrap_or("play"),
    }
}

/// A command label without the ellipsis the engine's menu labels end with.
fn without_ellipsis(label: &str) -> &str {
    label.trim_end_matches('…').trim_end_matches("...").trim_end()
}

/// The name of a command as the UI shows it: the catalogue's (`cmd.<id>`) or the engine's label.
#[must_use]
pub fn label_of(spec: &CommandSpec) -> String {
    i18n::find(&format!("cmd.{}", spec.id)).map_or_else(|| without_ellipsis(spec.label).to_string(), str::to_string)
}

/// What a command does in one plain sentence: the catalogue's (`cmd.<id>.hint`) or the engine's
/// summary.
#[must_use]
pub fn hint_of(spec: &CommandSpec) -> String {
    i18n::find(&format!("cmd.{}.hint", spec.id)).map_or_else(|| spec.summary.to_string(), str::to_string)
}

/// The commands a search for `query` finds: every modelling command whose name, id or summary
/// holds each word of the query, names that start with the query first.
#[must_use]
pub fn search(commands: &'static [CommandSpec], query: &str) -> Vec<&'static CommandSpec> {
    let words: Vec<String> = query.split_whitespace().map(str::to_lowercase).collect();
    if words.is_empty() {
        return Vec::new();
    }
    let first = words.first().cloned().unwrap_or_default();
    let mut hits: Vec<(bool, &'static CommandSpec)> = commands
        .iter()
        .filter(|c| Stage::of(c.id).is_some())
        .filter_map(|c| {
            let label = label_of(c).to_lowercase();
            let hay = format!("{label} {} {} {}", c.id, c.label.to_lowercase(), c.summary.to_lowercase());
            words.iter().all(|w| hay.contains(w.as_str())).then(|| (!label.starts_with(&first), c))
        })
        .collect();
    hits.sort_by_key(|(late, _)| *late);
    hits.into_iter().map(|(_, c)| c).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_engine::cmd;

    fn commands() -> &'static [CommandSpec] {
        surfacecraft_engine::Session::new().commands()
    }

    #[test]
    fn every_promoted_command_exists_and_belongs_to_its_stage() {
        for stage in Stage::ALL {
            for entry in stage.entries() {
                if let Entry::Cmd(id) = entry {
                    assert!(cmd::find(id).is_some(), "{id} is a command of the engine");
                    assert_eq!(Stage::of(id), Some(stage), "{id} is promoted in its own stage");
                }
            }
        }
    }

    #[test]
    fn every_modelling_command_is_reachable_from_exactly_one_stage() {
        for c in commands() {
            let Some(stage) = Stage::of(c.id) else { continue };
            let promoted = stage.promotes(c.id);
            let listed = stage.more(commands()).iter().any(|m| m.id == c.id);
            let twin = DIALOG_TWINS.contains(&c.id);
            assert!(promoted || listed || twin, "{} is in the ribbon of {stage:?}", c.id);
            assert!(!(promoted && listed), "{} is shown once", c.id);
            assert!(stage.all(commands()).iter().any(|m| m.id == c.id), "{} is in the Tools menu", c.id);
        }
    }

    #[test]
    fn only_service_commands_have_no_stage() {
        let without: Vec<&str> = commands().iter().filter(|c| Stage::of(c.id).is_none()).map(|c| c.id).collect();
        for id in &without {
            let area = id.split('.').next().unwrap_or("");
            assert!(matches!(area, "app" | "doc" | "edit" | "guide"), "{id} needs a stage");
        }
    }

    #[test]
    fn dialog_twins_are_covered_by_a_shell_action() {
        let acts: Vec<AppAction> = Stage::ALL
            .iter()
            .flat_map(|s| s.entries())
            .filter_map(|e| if let Entry::Act(a) = e { Some(*a) } else { None })
            .collect();
        assert!(acts.contains(&AppAction::ImportMesh));
        assert!(acts.contains(&AppAction::ExportStep));
        assert!(acts.contains(&AppAction::ExportMesh));
        for id in DIALOG_TWINS {
            assert!(cmd::find(id).is_some(), "{id} is a command of the engine");
        }
    }

    #[test]
    fn labels_drop_the_menu_ellipsis_and_fall_back_to_the_engine() {
        assert_eq!(without_ellipsis("Export IGES…"), "Export IGES");
        assert_eq!(without_ellipsis("Reconstruct from points..."), "Reconstruct from points");
        let extrude = cmd::find("feature.extrude").expect("extrude");
        assert!(!label_of(extrude).is_empty());
        assert!(!hint_of(extrude).is_empty());
    }

    #[test]
    fn every_tool_has_a_name_and_a_plain_hint_in_both_languages() {
        for lang in i18n::Lang::ALL {
            let cat = i18n::catalog(lang);
            for c in commands().iter().filter(|c| Stage::of(c.id).is_some()) {
                assert!(cat.contains_key(&format!("cmd.{}", c.id)), "{lang:?}: name of {}", c.id);
                assert!(cat.contains_key(&format!("cmd.{}.hint", c.id)), "{lang:?}: hint of {}", c.id);
            }
        }
    }

    #[test]
    fn search_matches_names_ids_and_every_word() {
        let ids = |q: &str| search(commands(), q).iter().map(|c| c.id).collect::<Vec<_>>();
        assert!(ids("extrude").contains(&"feature.extrude"));
        assert!(ids("mesh.synth").contains(&"mesh.synth"));
        assert!(ids("fill holes").contains(&"mesh.fill_holes"));
        assert!(ids("").is_empty());
        assert!(ids("no such tool anywhere").is_empty());
        // service commands are not tools
        assert!(!ids("app").iter().any(|id| id.starts_with("app.")));
        // a name that starts with the query comes before one that only holds it
        let fillet = ids("fillet");
        assert_eq!(Stage::of(fillet[0]), Some(Stage::Solid));
    }

    #[test]
    fn every_stage_has_tools_a_name_and_an_icon() {
        for stage in Stage::ALL {
            assert!(stage.entries().iter().any(|e| *e != Entry::Gap), "{stage:?} has tools");
            assert!(crate::icons::NAMES.contains(&stage.icon()), "{stage:?} icon {}", stage.icon());
            for lang in i18n::Lang::ALL {
                let cat = i18n::catalog(lang);
                assert!(cat.contains_key(stage.title_key()), "{lang:?} {}", stage.title_key());
                assert!(cat.contains_key(stage.hint_key()), "{lang:?} {}", stage.hint_key());
            }
        }
        for c in commands() {
            assert!(crate::icons::NAMES.contains(&icon_of(c.id)), "icon of {}: {}", c.id, icon_of(c.id));
        }
    }
}
