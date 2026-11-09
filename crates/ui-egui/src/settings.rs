//! UI settings saved between runs: theme preset, density, language and the dock layout.
use crate::dock::Workspace;
use crate::i18n::Lang;
use crate::theme::Theme;
use serde::{Deserialize, Serialize};

/// The key under which the settings are stored (eframe storage on the desktop).
pub const STORAGE_KEY: &str = "surfacecraft.ui.v1";

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
}

impl UiSettings {
    /// The settings as JSON.
    #[must_use]
    pub fn to_json(&self) -> String {
        serde_json::to_string(self).unwrap_or_default()
    }

    /// Settings from JSON; anything unreadable gives the defaults (a damaged file never blocks
    /// the start).
    #[must_use]
    pub fn from_json(json: &str) -> Self {
        serde_json::from_str(json).unwrap_or_default()
    }
}
