//! Semantic theme tokens: the only place of the UI that knows colours and pixel sizes.
//!
//! The rest of the UI asks for a meaning (`Token::SurfaceRaised`, `Token::TextMuted`,
//! `Token::AccentPrimary`, `Token::DeviationAboveFar`), never for a colour or a size; a preset
//! resolves every token to a colour and a density resolves the spacing, so one layout serves all
//! presets (Craft framework principle "apps ask for meaning"; `cargo xtask ui-containers` rejects
//! raw colours elsewhere). Presets are data: Dark (the default of a creative app), Light and High
//! contrast. egui's widget states are painted from the tokens through
//! `craft_ui::theme::WidgetPalette`, the operation the Craft apps share.
use craft_ui::theme::WidgetPalette;
use egui::{Color32, CornerRadius, Stroke, Vec2};
use serde::{Deserialize, Serialize};

/// A theme preset.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Preset {
    /// Dark, the default of a creative application.
    #[default]
    Dark,
    /// Light.
    Light,
    /// High contrast (black, white and saturated accents).
    HighContrast,
}

impl Preset {
    /// All presets, in menu order.
    pub const ALL: [Preset; 3] = [Preset::Dark, Preset::Light, Preset::HighContrast];

    /// The catalogue key of the preset's name.
    #[must_use]
    pub fn name_key(self) -> &'static str {
        match self {
            Preset::Dark => "theme.dark",
            Preset::Light => "theme.light",
            Preset::HighContrast => "theme.high_contrast",
        }
    }

    /// Whether egui should start from its dark visuals.
    #[must_use]
    pub fn is_dark(self) -> bool {
        self != Preset::Light
    }
}

/// How much room controls take.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Density {
    /// Tight spacing for expert use.
    Compact,
    /// Roomier spacing (the default).
    #[default]
    Comfortable,
}

impl Density {
    /// Both densities, in menu order.
    pub const ALL: [Density; 2] = [Density::Compact, Density::Comfortable];

    /// The catalogue key of the density's name.
    #[must_use]
    pub fn name_key(self) -> &'static str {
        match self {
            Density::Compact => "density.compact",
            Density::Comfortable => "density.comfortable",
        }
    }

    /// The spacing of this density.
    #[must_use]
    pub fn metrics(self) -> Metrics {
        match self {
            Density::Compact => Metrics {
                item_spacing: Vec2::new(6.0, 3.0),
                button_padding: Vec2::new(6.0, 2.0),
                interact_height: 20.0,
                icon_button: 24.0,
                icon: 16.0,
                tab_height: 24.0,
                side_dock_width: 240.0,
                panel_padding: 4.0,
            },
            Density::Comfortable => Metrics {
                item_spacing: Vec2::new(8.0, 5.0),
                button_padding: Vec2::new(8.0, 4.0),
                interact_height: 26.0,
                icon_button: 30.0,
                icon: 18.0,
                tab_height: 28.0,
                side_dock_width: 280.0,
                panel_padding: 8.0,
            },
        }
    }
}

/// Spacing and sizes resolved from a [`Density`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Metrics {
    /// Gap between neighbouring controls.
    pub item_spacing: Vec2,
    /// Padding inside a button.
    pub button_padding: Vec2,
    /// Minimum height of an interactive control.
    pub interact_height: f32,
    /// Side of a toolbar icon button.
    pub icon_button: f32,
    /// Side of an icon inside a button.
    pub icon: f32,
    /// Height of a dock tab.
    pub tab_height: f32,
    /// Space between a panel's edge and its content.
    pub panel_padding: f32,
    /// Default width of a side dock column.
    pub side_dock_width: f32,
}

/// The theme choice that is saved in the settings.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(default)]
pub struct Theme {
    /// Colour preset.
    pub preset: Preset,
    /// Spacing.
    pub density: Density,
}

macro_rules! tokens {
    ($($variant:ident => $name:literal,)*) => {
        /// A semantic colour: what a colour means, not what it looks like.
        #[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
        pub enum Token {
            $(
                #[doc = $name]
                $variant,
            )*
        }

        impl Token {
            /// Every token, in declaration order.
            pub const ALL: &'static [Token] = &[$(Token::$variant,)*];

            /// The dotted name (`surface.raised`), stable and used in documents and tests.
            #[must_use]
            pub fn name(self) -> &'static str {
                match self {
                    $(Token::$variant => $name,)*
                }
            }
        }
    };
}

tokens! {
    SurfaceBase => "surface.base",
    SurfaceRaised => "surface.raised",
    SurfaceSunken => "surface.sunken",
    SurfaceHover => "surface.hover",
    SurfaceActive => "surface.active",
    TextPrimary => "text.primary",
    TextMuted => "text.muted",
    TextOnAccent => "text.on_accent",
    AccentPrimary => "accent.primary",
    AccentHover => "accent.hover",
    BorderSubtle => "border.subtle",
    BorderStrong => "border.strong",
    FocusRing => "focus.ring",
    StatusOk => "status.ok",
    StatusWarning => "status.warning",
    StatusError => "status.error",
    ViewportBackground => "viewport.background",
    ObjectMesh => "object.mesh",
    ObjectBody => "object.body",
    ObjectPrimitive => "object.primitive",
    ObjectSurface => "object.surface",
    ObjectQuad => "object.quad",
    ObjectSketch => "object.sketch",
    Highlight => "object.highlight",
    AxisX => "axis.x",
    AxisY => "axis.y",
    AxisZ => "axis.z",
    DeviationBelowNear => "deviation.below_near",
    DeviationBelowFar => "deviation.below_far",
    DeviationInTolerance => "deviation.in_tolerance",
    DeviationAboveNear => "deviation.above_near",
    DeviationAboveFar => "deviation.above_far",
    DeviationUnmeasured => "deviation.unmeasured",
}

/// One linear channel in 0..1 as a byte (the rounding every deviation colour goes through).
fn channel(v: f32) -> u8 {
    (v.clamp(0.0, 1.0) * 255.0).round() as u8
}

/// A linear RGBA colour of the deviation map as the byte colour the theme resolves to.
#[must_use]
pub fn to_color32(c: [f32; 4]) -> Color32 {
    Color32::from_rgba_unmultiplied(channel(c[0]), channel(c[1]), channel(c[2]), channel(c[3]))
}

/// The deviation colours, shared with `surfacecraft_deviation::colorize` (the viewport).
fn deviation_bytes(c: [f32; 4]) -> [u8; 4] {
    [channel(c[0]), channel(c[1]), channel(c[2]), channel(c[3])]
}

/// `[r, g, b, a]` of a token in one preset (straight alpha).
#[allow(clippy::too_many_lines)]
fn raw(preset: Preset, token: Token) -> [u8; 4] {
    use Token::*;
    // object and deviation colours are shared by the presets except for contrast
    let shared = |t: Token| -> [u8; 4] {
        match t {
            ObjectMesh => [158, 168, 184, 255],
            ObjectBody => [89, 140, 217, 255],
            ObjectPrimitive => [242, 191, 51, 140],
            ObjectSurface => [140, 217, 242, 204],
            ObjectQuad => [230, 102, 230, 89],
            ObjectSketch => [255, 255, 77, 255],
            Highlight => [255, 140, 26, 255],
            AxisX => [230, 51, 51, 255],
            AxisY => [51, 204, 51, 255],
            AxisZ => [64, 102, 242, 255],
            DeviationBelowNear => deviation_bytes(surfacecraft_deviation::palette::BELOW_NEAR),
            DeviationBelowFar => deviation_bytes(surfacecraft_deviation::palette::BELOW_FAR),
            DeviationInTolerance => deviation_bytes(surfacecraft_deviation::palette::IN_TOLERANCE),
            DeviationAboveNear => deviation_bytes(surfacecraft_deviation::palette::ABOVE_NEAR),
            DeviationAboveFar => deviation_bytes(surfacecraft_deviation::palette::ABOVE_FAR),
            DeviationUnmeasured => deviation_bytes(surfacecraft_deviation::palette::UNMEASURED),
            _ => [0, 0, 0, 255],
        }
    };
    match preset {
        Preset::Dark => match token {
            SurfaceBase => [30, 33, 38, 255],
            SurfaceRaised => [40, 44, 51, 255],
            SurfaceSunken => [22, 24, 28, 255],
            SurfaceHover => [54, 59, 68, 255],
            SurfaceActive => [66, 73, 85, 255],
            TextPrimary => [226, 230, 236, 255],
            TextMuted => [145, 153, 165, 255],
            TextOnAccent => [255, 255, 255, 255],
            AccentPrimary => [64, 132, 232, 255],
            AccentHover => [96, 154, 240, 255],
            BorderSubtle => [58, 63, 72, 255],
            BorderStrong => [92, 100, 113, 255],
            FocusRing => [120, 170, 250, 255],
            StatusOk => [88, 190, 128, 255],
            StatusWarning => [232, 176, 64, 255],
            StatusError => [238, 94, 94, 255],
            ViewportBackground => [20, 23, 28, 255],
            other => shared(other),
        },
        Preset::Light => match token {
            SurfaceBase => [243, 245, 248, 255],
            SurfaceRaised => [255, 255, 255, 255],
            SurfaceSunken => [230, 233, 238, 255],
            SurfaceHover => [226, 232, 241, 255],
            SurfaceActive => [208, 218, 233, 255],
            TextPrimary => [28, 33, 41, 255],
            TextMuted => [92, 101, 114, 255],
            TextOnAccent => [255, 255, 255, 255],
            AccentPrimary => [30, 100, 206, 255],
            AccentHover => [22, 80, 170, 255],
            BorderSubtle => [212, 217, 226, 255],
            BorderStrong => [150, 159, 173, 255],
            FocusRing => [30, 100, 206, 255],
            StatusOk => [26, 128, 70, 255],
            StatusWarning => [166, 112, 8, 255],
            StatusError => [194, 40, 40, 255],
            ViewportBackground => [217, 222, 230, 255],
            ObjectMesh => [120, 130, 148, 255],
            other => shared(other),
        },
        Preset::HighContrast => match token {
            SurfaceBase => [0, 0, 0, 255],
            SurfaceRaised => [10, 10, 10, 255],
            SurfaceSunken => [0, 0, 0, 255],
            SurfaceHover => [38, 38, 38, 255],
            SurfaceActive => [64, 64, 64, 255],
            TextPrimary => [255, 255, 255, 255],
            TextMuted => [214, 214, 214, 255],
            TextOnAccent => [0, 0, 0, 255],
            AccentPrimary => [255, 221, 0, 255],
            AccentHover => [255, 240, 120, 255],
            BorderSubtle => [160, 160, 160, 255],
            BorderStrong => [255, 255, 255, 255],
            FocusRing => [0, 220, 255, 255],
            StatusOk => [60, 255, 120, 255],
            StatusWarning => [255, 200, 0, 255],
            StatusError => [255, 90, 90, 255],
            ViewportBackground => [0, 0, 0, 255],
            ObjectMesh => [220, 220, 220, 255],
            ObjectBody => [80, 160, 255, 255],
            other => shared(other),
        },
    }
}

/// Every token resolved for one preset.
#[derive(Clone, Debug, PartialEq)]
pub struct Tokens {
    preset: Preset,
    colors: Vec<Color32>,
}

impl Default for Tokens {
    fn default() -> Self {
        Self::resolve(Preset::default())
    }
}

impl Tokens {
    /// Resolves all tokens for `preset`.
    #[must_use]
    pub fn resolve(preset: Preset) -> Self {
        let colors = Token::ALL
            .iter()
            .map(|&t| {
                let [r, g, b, a] = raw(preset, t);
                Color32::from_rgba_unmultiplied(r, g, b, a)
            })
            .collect();
        Self { preset, colors }
    }

    /// The preset these tokens resolve.
    #[must_use]
    pub fn preset(&self) -> Preset {
        self.preset
    }

    /// The colour a token means in this preset.
    #[must_use]
    pub fn color(&self, token: Token) -> Color32 {
        Token::ALL
            .iter()
            .position(|&t| t == token)
            .and_then(|i| self.colors.get(i))
            .copied()
            .unwrap_or(Color32::TRANSPARENT)
    }

    /// The token as straight-alpha RGBA in 0..1, the form the viewport's renderer takes.
    #[must_use]
    pub fn rgba(&self, token: Token) -> [f32; 4] {
        let [r, g, b, a] = self.color(token).to_srgba_unmultiplied();
        [f32::from(r) / 255.0, f32::from(g) / 255.0, f32::from(b) / 255.0, f32::from(a) / 255.0]
    }

    /// Start and end colour of each of the three axis lines of the viewport (x, y, z).
    #[must_use]
    pub fn axis_vertex_colors(&self) -> [[f32; 4]; 6] {
        let (x, y, z) = (self.rgba(Token::AxisX), self.rgba(Token::AxisY), self.rgba(Token::AxisZ));
        [x, x, y, y, z, z]
    }

    /// The viewport clear colour.
    #[must_use]
    pub fn viewport_background(&self) -> [f32; 3] {
        let [r, g, b, _] = self.rgba(Token::ViewportBackground);
        [r, g, b]
    }

    /// Viewport colour of an object kind.
    #[must_use]
    pub fn object(&self, kind: surfacecraft_doc::ObjectKind) -> [f32; 4] {
        use surfacecraft_doc::ObjectKind as K;
        let token = match kind {
            K::Body => Token::ObjectBody,
            K::Primitive => Token::ObjectPrimitive,
            K::Surface => Token::ObjectSurface,
            K::Quad => Token::ObjectQuad,
            K::Sketch => Token::ObjectSketch,
            K::Mesh | K::Region | K::Feature => Token::ObjectMesh,
        };
        self.rgba(token)
    }
}

/// Installs the theme into an egui context: visuals from the tokens, widget states through
/// `WidgetPalette`, spacing from the density.
pub fn apply(ctx: &egui::Context, theme: &Theme) {
    let t = Tokens::resolve(theme.preset);
    let m = theme.density.metrics();
    let c = |token| t.color(token);
    let egui_theme = if theme.preset.is_dark() { egui::Theme::Dark } else { egui::Theme::Light };
    let mut v = if theme.preset.is_dark() { egui::Visuals::dark() } else { egui::Visuals::light() };
    WidgetPalette {
        noninteractive: c(Token::SurfaceBase),
        inactive: c(Token::SurfaceRaised),
        hovered: c(Token::SurfaceHover),
        active: c(Token::SurfaceActive),
        open: c(Token::SurfaceActive),
    }
    .apply_to(&mut v.widgets, CornerRadius::same(4), Stroke::new(1.0, c(Token::TextPrimary)));
    v.widgets.noninteractive.bg_stroke = Stroke::new(1.0, c(Token::BorderSubtle));
    v.widgets.noninteractive.fg_stroke = Stroke::new(1.0, c(Token::TextMuted));
    v.widgets.inactive.bg_stroke = Stroke::new(1.0, c(Token::BorderSubtle));
    v.widgets.hovered.bg_stroke = Stroke::new(1.0, c(Token::BorderStrong));
    v.widgets.active.bg_stroke = Stroke::new(1.0, c(Token::AccentPrimary));
    v.widgets.open.bg_stroke = Stroke::new(1.0, c(Token::BorderStrong));
    v.selection.bg_fill = c(Token::AccentPrimary).gamma_multiply(0.55);
    v.selection.stroke = Stroke::new(1.0, c(Token::FocusRing));
    v.hyperlink_color = c(Token::AccentPrimary);
    v.panel_fill = c(Token::SurfaceBase);
    v.window_fill = c(Token::SurfaceRaised);
    v.window_stroke = Stroke::new(1.0, c(Token::BorderSubtle));
    v.extreme_bg_color = c(Token::SurfaceSunken);
    v.faint_bg_color = c(Token::SurfaceRaised);
    v.warn_fg_color = c(Token::StatusWarning);
    v.error_fg_color = c(Token::StatusError);
    ctx.set_theme(egui_theme);
    ctx.set_visuals_of(egui_theme, v);
    ctx.global_style_mut(|s| {
        s.spacing.item_spacing = m.item_spacing;
        s.spacing.button_padding = m.button_padding;
        s.spacing.interact_size.y = m.interact_height;
    });
}

/// The icon-button style of the toolbars, from tokens.
#[must_use]
pub fn icon_button_style(t: &Tokens) -> craft_ui::buttons::IconButtonStyle {
    use craft_ui::buttons::{ButtonVisuals, IconButtonStyle};
    let visuals = |fill: Token, stroke: Option<Token>, icon: Token| ButtonVisuals {
        fill: if fill == Token::SurfaceBase { Color32::TRANSPARENT } else { t.color(fill) },
        stroke: stroke.map_or(Stroke::NONE, |s| Stroke::new(1.0, t.color(s))),
        icon: t.color(icon),
    };
    IconButtonStyle {
        normal: visuals(Token::SurfaceBase, None, Token::TextPrimary),
        hovered: visuals(Token::SurfaceHover, Some(Token::BorderSubtle), Token::TextPrimary),
        pressed: Some(visuals(Token::SurfaceActive, Some(Token::BorderStrong), Token::TextPrimary)),
        selected: visuals(Token::SurfaceActive, Some(Token::AccentPrimary), Token::AccentHover),
        corner_radius: CornerRadius::same(4),
        focus_stroke: Stroke::new(1.5, t.color(Token::FocusRing)),
    }
}

/// The style of the dock area (tabs, borders, drop previews) from tokens and density.
#[must_use]
pub fn dock_style(ui: &egui::Ui, t: &Tokens, m: &Metrics) -> craft_ui::docking::DockStyle {
    let mut s = craft_ui::docking::DockStyle::from_ui(ui);
    s.tab_height = m.tab_height;
    s.min_pane = 96.0;
    s.background = t.color(Token::SurfaceBase);
    s.tab_background = t.color(Token::SurfaceSunken);
    s.active_background = t.color(Token::SurfaceRaised);
    s.floating_background = t.color(Token::SurfaceRaised);
    s.floating_stroke = Stroke::new(1.0, t.color(Token::BorderStrong));
    s.text = t.color(Token::TextPrimary);
    s.inactive_text = t.color(Token::TextMuted);
    s.border = Stroke::new(1.0, t.color(Token::BorderSubtle));
    s.accent = t.color(Token::AccentPrimary);
    // creative-app arrangement: docks on the window's edges, the viewport takes no tabs
    s.new_dock_zones = true;
    s.side_edge_zones = false;
    s.title_bar_docks = false;
    s.localized(|key| crate::i18n::find(key).map(str::to_owned))
}
