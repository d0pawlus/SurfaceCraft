//! The dock layout of the shell: panel identities, the default arrangement ("docks beside a
//! canvas": the protected viewport in the middle, panel docks on both sides, the log below) and
//! the rules `craft_ui::docking` enforces around them. Panel contents are drawn by the app
//! ([`crate::app`]); this module owns only identity, arrangement and persistence.
use crate::app::SurfaceCraftApp;
use craft_ui::docking::{
    Action, Column, DockContent, DockHeights, Group, Layout, Node, PanelLimits, Permissions, Side,
    Workspace as CraftWorkspace,
};
use craft_ui::layout::{SplitAxis, SplitSize};
use serde::{Deserialize, Serialize};

/// A panel of the shell.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "camelCase")]
pub enum Panel {
    /// The 3D viewport (protected: always docked in the middle).
    Viewport,
    /// The object tree.
    Objects,
    /// The operation history.
    History,
    /// Command parameters and the assistant.
    Properties,
    /// Regions of the selected scan.
    Regions,
    /// Deviation maps and their legend.
    Deviation,
    /// The command log.
    Log,
}

impl Panel {
    /// Every panel, in menu order.
    pub const ALL: [Panel; 7] = [
        Panel::Viewport,
        Panel::Objects,
        Panel::History,
        Panel::Properties,
        Panel::Regions,
        Panel::Deviation,
        Panel::Log,
    ];

    /// The catalogue key of the panel's title.
    #[must_use]
    pub fn title_key(self) -> &'static str {
        match self {
            Panel::Viewport => "panel.viewport",
            Panel::Objects => "panel.objects",
            Panel::History => "panel.history",
            Panel::Properties => "panel.properties",
            Panel::Regions => "panel.regions",
            Panel::Deviation => "panel.deviation",
            Panel::Log => "panel.log",
        }
    }

    /// The icon of the panel in the catalogue (`None`: no icon).
    #[must_use]
    pub fn icon(self) -> &'static str {
        match self {
            Panel::Viewport => "viewport",
            Panel::Objects => "objects",
            Panel::History => "history",
            Panel::Properties => "properties",
            Panel::Regions => "regions",
            Panel::Deviation => "deviation",
            Panel::Log => "history",
        }
    }

    /// The panels the user may hide and show again.
    #[must_use]
    pub fn closable(self) -> bool {
        self != Panel::Viewport
    }
}

/// The saved arrangement: craft-ui's workspace over SurfaceCraft's panels.
pub type Workspace = CraftWorkspace<Panel>;

/// The viewport never moves, floats, closes or takes tabs; panels dock beside it.
#[must_use]
pub fn permissions(panel: &Panel) -> Permissions {
    if *panel == Panel::Viewport {
        Permissions { accepts_splits: true, ..Permissions::PROTECTED }
    } else {
        Permissions::default()
    }
}

fn tabs(panels: &[Panel]) -> Node<Panel> {
    Node::Tabs { panels: panels.to_vec(), active: 0 }
}

fn split(axis: SplitAxis, size: SplitSize, first: Node<Panel>, second: Node<Panel>) -> Node<Panel> {
    Node::Split { axis, size, first: Box::new(first), second: Box::new(second) }
}

fn dock(side: Side, width: f32, groups: &[&[Panel]]) -> Node<Panel> {
    let groups = groups.iter().map(|panels| Group::new(panels.to_vec(), 0)).collect();
    Node::Dock { side, columns: vec![Column::new(groups, width)] }
}

/// The default arrangement for a dock column width (in points): objects over history on the left,
/// the viewport over the log in the middle, properties over regions and deviation on the right.
#[must_use]
pub fn default_workspace(side_width: f32) -> Workspace {
    let centre =
        split(SplitAxis::Vertical, SplitSize::FixedSecond(150.0), tabs(&[Panel::Viewport]), tabs(&[Panel::Log]));
    let right = dock(Side::End, side_width + 40.0, &[&[Panel::Properties], &[Panel::Regions, Panel::Deviation]]);
    let left = dock(Side::Start, side_width, &[&[Panel::Objects], &[Panel::History]]);
    Workspace::new(Layout {
        root: Some(split(
            SplitAxis::Horizontal,
            SplitSize::FixedFirst(side_width),
            left,
            split(SplitAxis::Horizontal, SplitSize::FixedSecond(side_width + 40.0), centre, right),
        )),
        floating: Vec::new(),
    })
}

/// Fits a loaded workspace to the panels registered now and to the viewport rules: an invalid
/// file gives the default arrangement; the viewport is always shown.
pub fn reconcile(workspace: &mut Workspace, default: &Workspace) {
    workspace.reconcile(|p| Panel::ALL.contains(p), permissions, default);
    if !workspace.layout.contains(&Panel::Viewport)
        || workspace.layout.floating.iter().any(|w| w.contains(&Panel::Viewport))
    {
        *workspace = default.clone();
    }
}

/// Shows or hides a panel (the View menu); the viewport cannot be hidden.
pub fn set_visible(workspace: &mut Workspace, panel: Panel, visible: bool) -> Result<(), String> {
    if !visible && !panel.closable() {
        return Ok(());
    }
    workspace.set_visible(panel, visible, None, permissions).map_err(|e| e.to_string())
}

/// Applies the actions the dock area emitted (a drag, a menu item of a tab) to the workspace.
pub fn apply_actions(workspace: &mut Workspace, actions: Vec<Action<Panel>>) -> Result<(), String> {
    let mut next = workspace.clone();
    next.apply_all(actions, permissions).map_err(|e| e.to_string())?;
    reconcile(&mut next, &default_workspace(240.0));
    *workspace = next;
    Ok(())
}

/// The panel contents, titles, size limits and icons for the dock area.
pub struct Panels<'a> {
    /// The application whose state the panels show.
    pub app: &'a mut SurfaceCraftApp,
}

impl DockContent<Panel> for Panels<'_> {
    fn body(&mut self, ui: &mut egui::Ui, panel: &Panel) {
        if *panel == Panel::Viewport {
            self.app.viewport_panel(ui);
            return;
        }
        // panel content sits inside the density's padding
        let pad = self.app.settings.theme.density.metrics().panel_padding;
        egui::Frame::NONE.inner_margin(egui::Margin::same(pad.round() as i8)).show(ui, |ui| match panel {
            Panel::Objects => scroll(ui, "dock-objects", |ui| self.app.objects_panel(ui)),
            Panel::History => scroll(ui, "dock-history", |ui| self.app.history_panel(ui)),
            Panel::Properties => scroll(ui, "dock-properties", |ui| self.app.palette_panel(ui)),
            Panel::Regions => scroll(ui, "dock-regions", |ui| self.app.regions_panel(ui)),
            Panel::Deviation => scroll(ui, "dock-deviation", |ui| self.app.deviation_panel(ui)),
            Panel::Log => self.app.log_panel(ui),
            Panel::Viewport => {}
        });
    }

    fn logical_label(&self, panel: &Panel) -> String {
        crate::i18n::t(panel.title_key()).to_string()
    }

    fn permissions(&self, panel: &Panel) -> Permissions {
        permissions(panel)
    }

    fn limits(&self, panel: &Panel) -> PanelLimits {
        let (min, max_width) = match panel {
            Panel::Viewport => (egui::vec2(320.0, 200.0), 1_000_000.0),
            Panel::Log => (egui::vec2(240.0, 60.0), 1_000_000.0),
            _ => (egui::vec2(200.0, 80.0), 720.0),
        };
        PanelLimits { min, max: egui::vec2(max_width, 1_000_000.0) }
    }

    fn dock_heights(&self, panel: &Panel) -> Option<DockHeights> {
        // The object tree keeps more room than the history under it; the properties (command
        // list, parameters, assistant) take the right dock's height, regions and deviation the rest.
        match panel {
            Panel::Objects => Some(DockHeights::new(360.0)),
            Panel::Properties => Some(DockHeights::new(640.0).compact(360.0).fill(460.0)),
            Panel::Regions | Panel::Deviation => Some(DockHeights::new(150.0).compact(110.0)),
            _ => None,
        }
    }

    fn paint_icon(&mut self, painter: &egui::Painter, rect: egui::Rect, panel: &Panel, color: egui::Color32) -> bool {
        crate::icons::paint_in(painter, rect, panel.icon(), color);
        true
    }
}

fn scroll(ui: &mut egui::Ui, id: &str, add: impl FnOnce(&mut egui::Ui)) {
    egui::ScrollArea::vertical().id_salt(id).auto_shrink([false, false]).show(ui, add);
}
