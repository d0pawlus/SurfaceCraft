//! UI settings saved between runs: theme preset, density, language and the dock layout.
use crate::dock::Workspace;
use crate::i18n::Lang;
use crate::theme::Theme;
use serde::{Deserialize, Serialize};

/// The key under which the settings are stored (eframe storage on the desktop).
pub const STORAGE_KEY: &str = "surfacecraft.ui.v1";

/// Version of the dock arrangement. A layout saved under another version is dropped on load, so
/// an update shows its new default arrangement instead of the old one.
pub const LAYOUT_VERSION: u32 = 3;

/// Everything the shell remembers.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct UiSettings {
    /// Theme preset and density.
    pub theme: Theme,
    /// Interface language.
    pub lang: Lang,
    /// The dock layout (`None`: the default layout).
    pub workspace: Option<Workspace>,
    /// The [`LAYOUT_VERSION`] the layout was saved under.
    pub layout_version: u32,
    /// The unit the last scan import chose (the import form starts with it).
    pub import_units: surfacecraft_doc::LengthUnit,
}

impl UiSettings {
    /// The settings as JSON, the layout saved under the current [`LAYOUT_VERSION`].
    #[must_use]
    pub fn to_json(&self) -> String {
        let saved = Self { layout_version: LAYOUT_VERSION, ..self.clone() };
        serde_json::to_string(&saved).unwrap_or_default()
    }

    /// Settings from JSON; anything unreadable gives the defaults (a damaged file never blocks
    /// the start). A layout saved under another arrangement version is dropped.
    #[must_use]
    pub fn from_json(json: &str) -> Self {
        let mut settings: Self = serde_json::from_str(json).unwrap_or_default();
        if settings.layout_version != LAYOUT_VERSION {
            settings.workspace = None;
        }
        settings
    }
}
