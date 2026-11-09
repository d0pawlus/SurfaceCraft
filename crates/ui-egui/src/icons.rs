//! The SurfaceCraft icon catalogue: a small own set of monochrome SVG icons (see ATTRIBUTION.md),
//! prepared once through `craft_ui::icons::SvgIconSet` and tinted at draw time from theme tokens.
use craft_ui::icons::{SvgIconSet, SvgStyle};
use std::sync::OnceLock;

macro_rules! icons {
    ($($name:literal),* $(,)?) => {
        /// Names of all icons in the catalogue.
        pub const NAMES: &[&str] = &[$($name),*];
        const FILES: &[(&str, &[u8])] =
            &[$(($name, include_bytes!(concat!("../assets/icons/", $name, ".svg")) as &[u8])),*];
    };
}

icons!(
    "new",
    "open",
    "save",
    "import",
    "export",
    "undo",
    "redo",
    "fit",
    "objects",
    "history",
    "properties",
    "regions",
    "deviation",
    "eye",
    "eye-off",
    "trash",
    "play",
    "theme",
    "viewport",
);

/// The prepared catalogue (`currentColor` becomes white so a draw-time tint recolours it).
#[must_use]
pub fn set() -> &'static SvgIconSet {
    static SET: OnceLock<SvgIconSet> = OnceLock::new();
    SET.get_or_init(|| {
        SvgIconSet::new(
            FILES,
            SvgStyle { current_color: "white", stroke_width: None },
            "bytes://surfacecraft/icons/",
            "new",
        )
    })
}

/// Makes the SVG image loader available to the context (idempotent).
pub fn install(ctx: &egui::Context) {
    egui_extras::install_image_loaders(ctx);
}

/// Paints icon `name` into `rect` tinted with `tint`.
pub fn paint(ui: &egui::Ui, rect: egui::Rect, name: &str, tint: egui::Color32) {
    egui::Image::new(set().source(name)).tint(tint).paint_at(ui, rect);
}

/// The panel/dock icon for the dock chrome: paints `name` into `rect` of `painter`.
pub fn paint_in(painter: &egui::Painter, rect: egui::Rect, name: &str, tint: egui::Color32) {
    // The dock hands us a painter, not a Ui: the icon is drawn as its image's texture once the
    // loader has it ready (the next frame at the latest).
    let ctx = painter.ctx();
    if let Ok(egui::load::TexturePoll::Ready { texture }) =
        egui::Image::new(set().source(name)).load_for_size(ctx, rect.size())
    {
        let uv = egui::Rect::from_min_max(egui::pos2(0.0, 0.0), egui::pos2(1.0, 1.0));
        painter.image(texture.id, rect, uv, tint);
    }
}
