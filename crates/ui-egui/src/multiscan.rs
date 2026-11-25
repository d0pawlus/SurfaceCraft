//! The "Many scans" wizard: pick the scans of one part, align the pairs coarsely, align them all
//! globally, merge them into one mesh, with the proposal or the result of each step on screen
//! before the next one starts.
//!
//! This module is the state machine and nothing else. It never touches the session: an action
//! gives back [`Effects`] (the command to run, undo steps to take, whether to close), and the
//! outcome of a command is fed back through [`Wizard::on_result`], so the unit tests drive a whole
//! run with canned outcomes and no window. The panel that draws it and the glue that runs the
//! commands (in the background, as a script would call them) are in [`crate::multiscan_panel`];
//! the wizard adds no geometry of its own.
//!
//! Undo: the wizard records the undo steps its own commands made by comparing the history's
//! labels before and after each of them (an applied alignment and a merge make one step, a
//! proposal or a refusal none). Going back a step, and cancelling, undo exactly those steps, and
//! only while the history still ends with them: if something else was done meanwhile, the wizard
//! says so and leaves the document alone.
pub mod report;
#[cfg(test)]
mod tests;

use report::{GlobalReport, Matrix, MergeReport, PairProposal};
use serde_json::{Value, json};
use std::collections::BTreeSet;
use surfacecraft_doc::ObjectId;

/// The coarse alignment of two scans.
pub const PAIR_COMMAND: &str = "align.pair_coarse";
/// The alignment of all scans together.
pub const GLOBAL_COMMAND: &str = "align.global";
/// The merge into one mesh.
pub const MERGE_COMMAND: &str = "mesh.merge";

/// A step of the wizard.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Step {
    /// Choose the scans.
    Pick,
    /// Align the pairs coarsely, accepting or rejecting each pose.
    Pairs,
    /// Align all scans together and read the loop error.
    Global,
    /// Merge the scans into one mesh.
    Merge,
}

impl Step {
    /// Every step, in order.
    pub const ALL: [Step; 4] = [Step::Pick, Step::Pairs, Step::Global, Step::Merge];

    /// The catalogue key of the step's title.
    #[must_use]
    pub fn title_key(self) -> &'static str {
        match self {
            Step::Pick => "wizard.step.pick",
            Step::Pairs => "wizard.step.pairs",
            Step::Global => "wizard.step.global",
            Step::Merge => "wizard.step.merge",
        }
    }

    /// The catalogue key of the sentence that says what the step does.
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            Step::Pick => "wizard.step.pick.hint",
            Step::Pairs => "wizard.step.pairs.hint",
            Step::Global => "wizard.step.global.hint",
            Step::Merge => "wizard.step.merge.hint",
        }
    }
}

/// Where the global alignment starts from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StartChoice {
    /// The scans are roughly in place (the coarse step put them there): every pair starts from
    /// the current poses.
    Current,
    /// The full search for every pair, with no assumption about where the scans lie.
    Search,
}

impl StartChoice {
    fn parameter(self) -> &'static str {
        match self {
            StartChoice::Current => "current",
            StartChoice::Search => "search",
        }
    }
}

/// What a command run is for: carried with the run and handed back with its outcome.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Purpose {
    /// Propose the pose of pair `n`.
    ProposePair(usize),
    /// Apply the proposed pose of pair `n`.
    AcceptPair(usize),
    /// Propose the global poses.
    ProposeGlobal,
    /// Apply the global poses.
    ApplyGlobal,
    /// Merge the scans.
    Merge,
}

/// A command to run, as a script or MCP client would send it.
#[derive(Clone, Debug, PartialEq)]
pub struct Request {
    /// The command id.
    pub command: &'static str,
    /// Its parameters.
    pub params: Value,
    /// What the run is for.
    pub purpose: Purpose,
}

/// What the wizard asks the app to do after an action or an outcome.
#[derive(Debug, Default, PartialEq)]
pub struct Effects {
    /// A command to run; its outcome comes back through [`Wizard::on_result`].
    pub run: Option<Request>,
    /// Undo steps to take first.
    pub undo: usize,
    /// Close the wizard.
    pub close: bool,
    /// The object to select when it closes (the merged mesh).
    pub select: Option<u64>,
}

/// Why the last action did not do what was asked, for the panel to say.
#[derive(Clone, Debug, PartialEq)]
pub enum Problem {
    /// A command failed (or was cancelled): the engine's message.
    Engine(String),
    /// The history no longer ends with what the wizard did: nothing was undone.
    Stale,
    /// The tolerance typed is not a positive number.
    BadTolerance,
    /// The command ran but moved nothing.
    NothingMoved,
    /// The command answered with something the wizard cannot read.
    Unreadable,
}

/// A pair of scans of the coarse step: the second is brought onto the first.
#[derive(Clone, Debug, PartialEq)]
pub struct Pair {
    /// The scan that moves.
    pub object: ObjectId,
    /// The scan it is brought onto (the one before it in the list).
    pub target: ObjectId,
    /// Where the pair stands.
    pub state: PairState,
}

/// Where a pair stands.
#[derive(Clone, Debug, PartialEq)]
pub enum PairState {
    /// Not proposed yet.
    Pending,
    /// A pose is proposed and shown; the user has not decided.
    Proposed(PairProposal),
    /// The pose was applied.
    Accepted(PairProposal),
    /// The user left the scan where it is.
    Rejected,
    /// The proposal failed (the engine's message).
    Failed(String),
}

impl PairState {
    fn resolved(&self) -> bool {
        matches!(self, PairState::Accepted(_) | PairState::Rejected)
    }
}

/// Where the global alignment stands.
#[derive(Clone, Debug, PartialEq)]
pub enum GlobalState {
    /// Not proposed (yet, or it failed).
    Idle,
    /// Proposed and shown.
    Proposed(GlobalReport),
    /// Applied.
    Applied(GlobalReport),
}

/// Where the merge stands.
#[derive(Clone, Debug, PartialEq)]
pub enum MergeState {
    /// Not run.
    Idle,
    /// Refused: nothing was made.
    Refused(MergeReport),
    /// Done: the new mesh is in the document.
    Done(MergeReport),
}

/// The merge's settings.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MergeOptions {
    /// The tolerance in mm as typed (empty: the command's default).
    pub tolerance: String,
    /// Merge anyway when the scans look misaligned (`alignment: "warn"`).
    pub warn: bool,
    /// The name of the new mesh (empty: the command's default).
    pub name: String,
}

/// Something the user did in the wizard.
#[derive(Clone, Debug, PartialEq)]
pub enum Action {
    /// Add the mesh to the scans, or take it off.
    ToggleScan(ObjectId),
    /// On to the next step.
    Next,
    /// Back to the step before: what this one did is undone.
    Back,
    /// Close the wizard: everything it did is undone.
    Cancel,
    /// Close the wizard without undoing (when the history was changed meanwhile).
    Leave,
    /// Apply the proposed pose of the current pair.
    AcceptPair,
    /// Leave the current pair's scan where it is.
    RejectPair,
    /// Propose the current pair again.
    RetryPair,
    /// Show another pose of the current pair (0 is the best).
    ChoosePose(usize),
    /// Change where the global alignment starts from.
    SetStart(StartChoice),
    /// Let a weakly held scan move (or not).
    ToggleWeak(usize),
    /// Apply the global poses.
    ApplyGlobal,
    /// Merge anyway when the scans look misaligned.
    SetWarn(bool),
    /// The tolerance field.
    SetTolerance(String),
    /// The name field.
    SetName(String),
    /// Run the merge.
    Merge,
    /// Undo the merge (or drop its refusal) to change its settings.
    RetryMerge,
    /// Close the wizard, keeping the merged mesh.
    Finish,
}

/// The wizard.
#[derive(Clone, Debug)]
pub struct Wizard {
    step: Step,
    scans: Vec<ObjectId>,
    pairs: Vec<Pair>,
    start: StartChoice,
    weak: BTreeSet<usize>,
    global: GlobalState,
    merge: MergeState,
    options: MergeOptions,
    problem: Option<Problem>,
    running: Option<Purpose>,
    /// The step each undo step the wizard made belongs to, oldest first.
    ledger: Vec<Step>,
    /// The undo labels after the last of them (what the history must still end with).
    labels: Vec<String>,
}

impl Wizard {
    /// A wizard at its first step, with `preselected` scans chosen; `history` is the labels of
    /// the undo steps the document has now.
    #[must_use]
    pub fn new(history: &[String], preselected: Vec<ObjectId>) -> Self {
        Wizard {
            step: Step::Pick,
            scans: preselected,
            pairs: Vec::new(),
            start: StartChoice::Current,
            weak: BTreeSet::new(),
            global: GlobalState::Idle,
            merge: MergeState::Idle,
            options: MergeOptions::default(),
            problem: None,
            running: None,
            ledger: Vec::new(),
            labels: history.to_vec(),
        }
    }

    /// The step shown.
    #[must_use]
    pub fn step(&self) -> Step {
        self.step
    }

    /// The scans chosen, in the order the pairs follow.
    #[must_use]
    pub fn scans(&self) -> &[ObjectId] {
        &self.scans
    }

    /// The pairs of the coarse step.
    #[must_use]
    pub fn pairs(&self) -> &[Pair] {
        &self.pairs
    }

    /// The pair waiting for a decision: the first not accepted or rejected.
    #[must_use]
    pub fn current_pair(&self) -> Option<(usize, &Pair)> {
        self.pairs.iter().enumerate().find(|(_, p)| !p.state.resolved())
    }

    /// Where the global alignment starts from.
    #[must_use]
    pub fn start(&self) -> StartChoice {
        self.start
    }

    /// The weakly held scans (positions) the user lets move.
    #[must_use]
    pub fn accepted_weak(&self) -> &BTreeSet<usize> {
        &self.weak
    }

    /// Where the global alignment stands.
    #[must_use]
    pub fn global(&self) -> &GlobalState {
        &self.global
    }

    /// Where the merge stands.
    #[must_use]
    pub fn merge(&self) -> &MergeState {
        &self.merge
    }

    /// The merge's settings.
    #[must_use]
    pub fn options(&self) -> &MergeOptions {
        &self.options
    }

    /// Why the last action did nothing, if it did not.
    #[must_use]
    pub fn problem(&self) -> Option<&Problem> {
        self.problem.as_ref()
    }

    /// The command running now, if one is: actions wait for it.
    #[must_use]
    pub fn running(&self) -> Option<Purpose> {
        self.running
    }

    /// Records that something outside a command failed (an undo), for the panel to say.
    pub fn fail(&mut self, message: String) {
        self.problem = Some(Problem::Engine(message));
    }

    /// Undo steps the wizard has made in the document so far.
    #[must_use]
    pub fn undo_steps(&self) -> usize {
        self.ledger.len()
    }

    /// Do the scans chosen still exist? (`exists` asks the document.)
    #[must_use]
    pub fn refers_only_to(&self, exists: impl Fn(ObjectId) -> bool) -> bool {
        self.scans.iter().all(|id| exists(*id))
    }

    /// Is every pair accepted or rejected?
    #[must_use]
    pub fn pairs_resolved(&self) -> bool {
        !self.pairs.is_empty() && self.pairs.iter().all(|p| p.state.resolved())
    }

    /// May the wizard go on to the next step now?
    #[must_use]
    pub fn can_next(&self) -> bool {
        self.running.is_none()
            && match self.step {
                Step::Pick => self.scans.len() >= 2,
                Step::Pairs => self.pairs_resolved(),
                Step::Global => true,
                Step::Merge => false,
            }
    }

    /// The scans moved by the proposal on screen, with their motions: the pose of the pair
    /// waiting for a decision, or the poses of the scans the global alignment would move.
    #[must_use]
    pub fn preview(&self) -> Vec<(ObjectId, Matrix)> {
        if self.running.is_some() {
            return Vec::new();
        }
        match (self.step, &self.global) {
            (Step::Pairs, _) => match self.current_pair() {
                Some((_, Pair { object, state: PairState::Proposed(p), .. })) => {
                    p.chosen_pose().map(|pose| (*object, pose.matrix)).into_iter().collect()
                }
                _ => Vec::new(),
            },
            (Step::Global, GlobalState::Proposed(r)) => r
                .scans
                .iter()
                .filter(|s| s.placed && !s.fixed && (s.held_by_loop || self.weak.contains(&s.index)))
                .filter_map(|s| self.scans.get(s.index).map(|id| (*id, s.matrix)))
                .collect(),
            _ => Vec::new(),
        }
    }

    /// Does something the user did: the effects to carry out. Ignored while a command runs.
    pub fn act(&mut self, action: Action, history: &[String]) -> Effects {
        if self.running.is_some() {
            return Effects::default();
        }
        self.problem = None;
        match action {
            Action::ToggleScan(id) => self.toggle_scan(id),
            Action::Next => self.next(),
            Action::Back => self.back(history),
            Action::Cancel => self.cancel(history),
            Action::Leave => Effects { close: true, ..Effects::default() },
            Action::AcceptPair => self.accept_pair(),
            Action::RejectPair => self.reject_pair(),
            Action::RetryPair => self.propose_next_pair(),
            Action::ChoosePose(k) => self.choose_pose(k),
            Action::SetStart(c) => self.set_start(c),
            Action::ToggleWeak(i) => self.toggle_weak(i),
            Action::ApplyGlobal => self.apply_global(),
            Action::SetWarn(w) => self.edit_options(|o| o.warn = w),
            Action::SetTolerance(t) => self.edit_options(|o| o.tolerance = t),
            Action::SetName(n) => self.edit_options(|o| o.name = n),
            Action::Merge => self.run_merge(),
            Action::RetryMerge => self.retry_merge(history),
            Action::Finish => self.finish(),
        }
    }

    /// Takes the outcome of the command that ran for `purpose`; `history` is the labels of the
    /// undo steps the document has now.
    pub fn on_result(&mut self, purpose: Purpose, result: Result<Value, String>, history: &[String]) -> Effects {
        if self.running != Some(purpose) {
            return Effects::default();
        }
        self.running = None;
        match (purpose, result) {
            (Purpose::ProposePair(i), result) => self.pair_proposed(i, result),
            (Purpose::AcceptPair(i), Ok(v)) => return self.pair_accepted(i, &v, history),
            (Purpose::ProposeGlobal, Ok(v)) => self.global_proposed(&v),
            (Purpose::ApplyGlobal, Ok(v)) => self.global_applied(&v, history),
            (Purpose::Merge, Ok(v)) => self.merged(&v, history),
            (_, Err(message)) => self.problem = Some(Problem::Engine(message)),
        }
        Effects::default()
    }

    fn run(&mut self, command: &'static str, params: Value, purpose: Purpose) -> Effects {
        self.running = Some(purpose);
        Effects { run: Some(Request { command, params, purpose }), ..Effects::default() }
    }

    fn ids(&self) -> Vec<u64> {
        self.scans.iter().map(|id| id.0).collect()
    }

    fn toggle_scan(&mut self, id: ObjectId) -> Effects {
        if self.step != Step::Pick {
            return Effects::default();
        }
        match self.scans.iter().position(|s| *s == id) {
            Some(at) => {
                self.scans.remove(at);
            }
            None => self.scans.push(id),
        }
        Effects::default()
    }

    fn next(&mut self) -> Effects {
        if !self.can_next() {
            return Effects::default();
        }
        match self.step {
            Step::Pick => {
                self.pairs = self
                    .scans
                    .windows(2)
                    .filter_map(|w| match w {
                        [target, object] => Some(Pair { object: *object, target: *target, state: PairState::Pending }),
                        _ => None,
                    })
                    .collect();
                self.step = Step::Pairs;
                self.propose_pair(0)
            }
            Step::Pairs => {
                // after accepted poses the scans are roughly in place; after a rejected one the
                // pairs have to be searched for again
                let all_accepted = self.pairs.iter().all(|p| matches!(p.state, PairState::Accepted(_)));
                self.start = if all_accepted { StartChoice::Current } else { StartChoice::Search };
                self.step = Step::Global;
                self.global = GlobalState::Idle;
                self.weak.clear();
                self.propose_global()
            }
            Step::Global => {
                self.step = Step::Merge;
                self.merge = MergeState::Idle;
                Effects::default()
            }
            Step::Merge => Effects::default(),
        }
    }

    /// Takes back the undo steps of `step`: how many, or `None` (with the problem set) when the
    /// history no longer ends with the wizard's own steps.
    fn unwind(&mut self, step: Step, history: &[String]) -> Option<usize> {
        let n = self.ledger.iter().filter(|s| **s == step).count();
        if n > 0 && history != self.labels.as_slice() {
            self.problem = Some(Problem::Stale);
            return None;
        }
        self.ledger.retain(|s| *s != step);
        self.labels.truncate(self.labels.len().saturating_sub(n));
        Some(n)
    }

    fn back(&mut self, history: &[String]) -> Effects {
        let Some(undo) = self.unwind(self.step, history) else { return Effects::default() };
        match self.step {
            Step::Pick => return Effects::default(),
            Step::Pairs => {
                self.pairs.clear();
                self.step = Step::Pick;
            }
            Step::Global => {
                self.global = GlobalState::Idle;
                self.weak.clear();
                self.step = Step::Pairs;
            }
            Step::Merge => {
                self.merge = MergeState::Idle;
                self.step = Step::Global;
            }
        }
        Effects { undo, ..Effects::default() }
    }

    fn cancel(&mut self, history: &[String]) -> Effects {
        let n = self.ledger.len();
        if n > 0 && history != self.labels.as_slice() {
            self.problem = Some(Problem::Stale);
            return Effects::default();
        }
        self.ledger.clear();
        Effects { undo: n, close: true, ..Effects::default() }
    }

    /// Notes the undo step a command made (or none): the history now is `history`.
    fn record(&mut self, step: Step, history: &[String]) {
        if history == self.labels.as_slice() {
            return;
        }
        let one_more = history.len() == self.labels.len() + 1 && history.starts_with(&self.labels);
        if one_more {
            self.ledger.push(step);
        } else {
            // something else changed the history: what the wizard did can no longer be undone
            self.ledger.clear();
            self.problem = Some(Problem::Stale);
        }
        self.labels = history.to_vec();
    }

    // ------------------------------------------------------------------ the pairs

    fn propose_pair(&mut self, i: usize) -> Effects {
        let Some(pair) = self.pairs.get(i) else { return Effects::default() };
        let params = json!({"object": pair.object.0, "target": pair.target.0, "apply": false});
        self.run(PAIR_COMMAND, params, Purpose::ProposePair(i))
    }

    fn pair_proposed(&mut self, i: usize, result: Result<Value, String>) {
        let state = match result {
            Ok(v) => report::pair_proposal(&v).map_or_else(|| PairState::Failed(String::new()), PairState::Proposed),
            Err(message) => PairState::Failed(message),
        };
        if let Some(pair) = self.pairs.get_mut(i) {
            pair.state = state;
        }
    }

    fn accept_pair(&mut self) -> Effects {
        let Some((i, pair)) = self.current_pair() else { return Effects::default() };
        let PairState::Proposed(p) = &pair.state else { return Effects::default() };
        if !p.acceptable() {
            return Effects::default();
        }
        let params = json!({"object": pair.object.0, "target": pair.target.0, "apply": true, "choose": p.chosen});
        self.run(PAIR_COMMAND, params, Purpose::AcceptPair(i))
    }

    fn pair_accepted(&mut self, i: usize, outcome: &Value, history: &[String]) -> Effects {
        if !report::applied(outcome) {
            self.problem = Some(Problem::NothingMoved);
            return Effects::default();
        }
        self.record(Step::Pairs, history);
        if let Some(pair) = self.pairs.get_mut(i)
            && let PairState::Proposed(p) = &pair.state
        {
            pair.state = PairState::Accepted(p.clone());
        }
        self.propose_next_pair()
    }

    fn propose_next_pair(&mut self) -> Effects {
        match self.current_pair() {
            Some((i, _)) => self.propose_pair(i),
            None => Effects::default(),
        }
    }

    fn reject_pair(&mut self) -> Effects {
        let Some((i, _)) = self.current_pair() else { return Effects::default() };
        if let Some(pair) = self.pairs.get_mut(i) {
            pair.state = PairState::Rejected;
        }
        self.propose_next_pair()
    }

    fn choose_pose(&mut self, k: usize) -> Effects {
        let Some((i, _)) = self.current_pair() else { return Effects::default() };
        if let Some(Pair { state: PairState::Proposed(p), .. }) = self.pairs.get_mut(i)
            && k < p.poses.len()
        {
            p.chosen = k;
        }
        Effects::default()
    }

    // ----------------------------------------------------------------- the global

    fn global_params(&self, apply: bool) -> Value {
        let mut params = json!({
            "objects": self.ids(),
            "fixed": self.scans.first().map(|id| id.0),
            "start": self.start.parameter(),
            "apply": apply,
        });
        if apply && let Some(map) = params.as_object_mut() {
            let weak: Vec<u64> = self.weak.iter().filter_map(|i| self.scans.get(*i)).map(|id| id.0).collect();
            map.insert("accept_weak".into(), json!(weak));
        }
        params
    }

    fn propose_global(&mut self) -> Effects {
        let params = self.global_params(false);
        self.run(GLOBAL_COMMAND, params, Purpose::ProposeGlobal)
    }

    fn global_proposed(&mut self, outcome: &Value) {
        match report::global_report(outcome) {
            Some(r) => {
                self.weak.retain(|i| r.scans.iter().any(|s| s.index == *i && s.weakly_held()));
                self.global = GlobalState::Proposed(r);
            }
            None => {
                self.global = GlobalState::Idle;
                self.problem = Some(Problem::Unreadable);
            }
        }
    }

    fn set_start(&mut self, choice: StartChoice) -> Effects {
        if self.step != Step::Global || matches!(self.global, GlobalState::Applied(_)) || choice == self.start {
            return Effects::default();
        }
        self.start = choice;
        self.weak.clear();
        self.propose_global()
    }

    fn toggle_weak(&mut self, i: usize) -> Effects {
        let weakly_held = match &self.global {
            GlobalState::Proposed(r) => r.scans.iter().any(|s| s.index == i && s.weakly_held()),
            _ => false,
        };
        if weakly_held && !self.weak.remove(&i) {
            self.weak.insert(i);
        }
        Effects::default()
    }

    fn apply_global(&mut self) -> Effects {
        match &self.global {
            GlobalState::Proposed(r) if !r.failed() => {
                let params = self.global_params(true);
                self.run(GLOBAL_COMMAND, params, Purpose::ApplyGlobal)
            }
            _ => Effects::default(),
        }
    }

    fn global_applied(&mut self, outcome: &Value, history: &[String]) {
        let Some(r) = report::global_report(outcome) else {
            self.problem = Some(Problem::Unreadable);
            return;
        };
        if report::applied(outcome) {
            self.record(Step::Global, history);
            self.global = GlobalState::Applied(r);
        } else {
            self.problem = Some(Problem::NothingMoved);
            self.global = GlobalState::Proposed(r);
        }
    }

    // ------------------------------------------------------------------ the merge

    fn edit_options(&mut self, edit: impl FnOnce(&mut MergeOptions)) -> Effects {
        if self.step == Step::Merge && !matches!(self.merge, MergeState::Done(_)) {
            edit(&mut self.options);
        }
        Effects::default()
    }

    fn run_merge(&mut self) -> Effects {
        if self.step != Step::Merge || matches!(self.merge, MergeState::Done(_)) {
            return Effects::default();
        }
        let mut params = json!({
            "objects": self.ids(),
            "alignment": if self.options.warn { "warn" } else { "refuse" },
        });
        let tolerance = self.options.tolerance.trim();
        let name = self.options.name.trim();
        if let Some(map) = params.as_object_mut() {
            if !tolerance.is_empty() {
                match tolerance.parse::<f64>() {
                    Ok(t) if t.is_finite() && t > 0.0 => {
                        map.insert("tolerance".into(), json!(t));
                    }
                    _ => {
                        self.problem = Some(Problem::BadTolerance);
                        return Effects::default();
                    }
                }
            }
            if !name.is_empty() {
                map.insert("name".into(), json!(name));
            }
        }
        self.run(MERGE_COMMAND, params, Purpose::Merge)
    }

    fn merged(&mut self, outcome: &Value, history: &[String]) {
        match report::merge_report(outcome) {
            Some(r) if !r.refused && r.id.is_some() => {
                self.record(Step::Merge, history);
                self.merge = MergeState::Done(r);
            }
            Some(r) => self.merge = MergeState::Refused(r),
            None => self.problem = Some(Problem::Unreadable),
        }
    }

    fn retry_merge(&mut self, history: &[String]) -> Effects {
        if self.step != Step::Merge {
            return Effects::default();
        }
        let Some(undo) = self.unwind(Step::Merge, history) else { return Effects::default() };
        self.merge = MergeState::Idle;
        Effects { undo, ..Effects::default() }
    }

    fn finish(&mut self) -> Effects {
        match &self.merge {
            MergeState::Done(r) if self.step == Step::Merge => {
                Effects { close: true, select: r.id, ..Effects::default() }
            }
            _ => Effects::default(),
        }
    }
}
