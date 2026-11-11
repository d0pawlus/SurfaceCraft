//! The "what next" guide: the steps from a scan to an exported model, each marked done from what
//! the document holds and what has been run, so somebody who has never reverse-engineered a part
//! always sees the next thing to do.
use crate::workflow::Stage;
use surfacecraft_doc::ObjectKind;

/// A step of the guide.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Bring a scan in.
    Import,
    /// Remove spikes, loose pieces and holes.
    Clean,
    /// Put the scan on the world axes.
    Align,
    /// Split the scan into regions.
    Segment,
    /// Build surfaces or a solid over the scan.
    Model,
    /// Compare the model with the scan.
    Check,
    /// Write the model out.
    Export,
}

impl Step {
    /// Every step, in order.
    pub const ALL: [Step; 7] =
        [Step::Import, Step::Clean, Step::Align, Step::Segment, Step::Model, Step::Check, Step::Export];

    /// The stable name of the step (catalogue keys).
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Step::Import => "import",
            Step::Clean => "clean",
            Step::Align => "align",
            Step::Segment => "segment",
            Step::Model => "model",
            Step::Check => "check",
            Step::Export => "export",
        }
    }

    /// The catalogue key of the step's title.
    #[must_use]
    pub fn title_key(self) -> &'static str {
        match self {
            Step::Import => "guide.import",
            Step::Clean => "guide.clean",
            Step::Align => "guide.align",
            Step::Segment => "guide.segment",
            Step::Model => "guide.model",
            Step::Check => "guide.check",
            Step::Export => "guide.export",
        }
    }

    /// The catalogue key of the sentence that explains the step.
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            Step::Import => "guide.import.hint",
            Step::Clean => "guide.clean.hint",
            Step::Align => "guide.align.hint",
            Step::Segment => "guide.segment.hint",
            Step::Model => "guide.model.hint",
            Step::Check => "guide.check.hint",
            Step::Export => "guide.export.hint",
        }
    }

    /// The ribbon stage where the step is done.
    #[must_use]
    pub fn stage(self) -> Stage {
        match self {
            Step::Import | Step::Clean => Stage::Scan,
            Step::Align => Stage::Align,
            Step::Segment => Stage::Segment,
            Step::Model => Stage::Solid,
            Step::Check => Stage::Inspect,
            Step::Export => Stage::Export,
        }
    }

    /// Whether the step may be skipped without blocking the ones after it.
    #[must_use]
    pub fn optional(self) -> bool {
        matches!(self, Step::Clean | Step::Check)
    }
}

/// What the guide reads from the session.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Progress {
    /// The kinds of the objects in the document.
    pub kinds: Vec<ObjectKind>,
    /// The ids of the commands that ran successfully, oldest first.
    pub ran: Vec<String>,
}

impl Progress {
    fn has(&self, kind: ObjectKind) -> bool {
        self.kinds.contains(&kind)
    }

    fn ran_any(&self, test: impl Fn(&str) -> bool) -> bool {
        self.ran.iter().any(|id| test(id))
    }

    /// Whether `step` is done.
    #[must_use]
    pub fn done(&self, step: Step) -> bool {
        match step {
            Step::Import => self.has(ObjectKind::Mesh),
            Step::Clean => self.ran_any(|id| {
                matches!(
                    id,
                    "mesh.clean"
                        | "mesh.remove_outliers"
                        | "mesh.remove_small"
                        | "mesh.fill_holes"
                        | "mesh.smooth"
                        | "mesh.decimate"
                )
            }),
            Step::Align => self.ran_any(|id| id.starts_with("align.")),
            Step::Segment => self.has(ObjectKind::Region),
            Step::Model => self.has(ObjectKind::Body) || self.has(ObjectKind::Surface),
            Step::Check => self.ran_any(|id| id == "deviation.compute"),
            Step::Export => self.ran_any(|id| id.starts_with("export.") || id == "mesh.export"),
        }
    }

    /// The step to do now: the first required step that is not done, or, when only optional ones
    /// are left before the last done step, the first step after everything that is done. `None`
    /// once the model has been exported.
    #[must_use]
    pub fn next(&self) -> Option<Step> {
        let last_done = Step::ALL.iter().rposition(|s| self.done(*s));
        Step::ALL
            .iter()
            .enumerate()
            .find(|(i, s)| !self.done(**s) && (!s.optional() || last_done.is_none_or(|l| *i > l)))
            .map(|(_, s)| *s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn progress(kinds: &[ObjectKind], ran: &[&str]) -> Progress {
        Progress { kinds: kinds.to_vec(), ran: ran.iter().map(|s| (*s).to_string()).collect() }
    }

    #[test]
    fn an_empty_document_starts_with_the_import() {
        let p = Progress::default();
        assert_eq!(p.next(), Some(Step::Import));
        assert!(Step::ALL.iter().all(|s| !p.done(*s)));
    }

    #[test]
    fn a_fresh_scan_suggests_cleaning_then_aligning() {
        let p = progress(&[ObjectKind::Mesh], &["mesh.import"]);
        assert!(p.done(Step::Import));
        assert_eq!(p.next(), Some(Step::Clean));
        let cleaned = progress(&[ObjectKind::Mesh], &["mesh.import", "mesh.fill_holes"]);
        assert_eq!(cleaned.next(), Some(Step::Align));
    }

    #[test]
    fn a_skipped_optional_step_is_not_suggested_again() {
        // aligned without cleaning: the guide moves on instead of going back to the clean-up
        let p = progress(&[ObjectKind::Mesh], &["mesh.import", "align.principal"]);
        assert!(!p.done(Step::Clean));
        assert_eq!(p.next(), Some(Step::Segment));
    }

    #[test]
    fn the_guide_ends_after_the_export() {
        let kinds = [ObjectKind::Mesh, ObjectKind::Region, ObjectKind::Body];
        let modelled = progress(&kinds, &["mesh.clean", "align.icp", "region.auto_segment"]);
        assert_eq!(modelled.next(), Some(Step::Check));
        let checked = progress(&kinds, &["mesh.clean", "align.icp", "deviation.compute"]);
        assert_eq!(checked.next(), Some(Step::Export));
        let exported = progress(&kinds, &["mesh.clean", "align.icp", "deviation.compute", "export.step"]);
        assert_eq!(exported.next(), None);
    }

    #[test]
    fn a_surface_counts_as_a_model_and_every_step_has_a_stage() {
        assert!(progress(&[ObjectKind::Surface], &[]).done(Step::Model));
        assert!(!progress(&[ObjectKind::Sketch], &[]).done(Step::Model));
        for step in Step::ALL {
            assert!(Stage::ALL.contains(&step.stage()));
            for lang in crate::i18n::Lang::ALL {
                let cat = crate::i18n::catalog(lang);
                assert!(cat.contains_key(step.title_key()), "{lang:?} {}", step.title_key());
                assert!(cat.contains_key(step.hint_key()), "{lang:?} {}", step.hint_key());
            }
        }
    }
}
