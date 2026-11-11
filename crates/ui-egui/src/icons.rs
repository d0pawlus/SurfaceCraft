//! The SurfaceCraft icon catalogue: an own set of monochrome SVG line icons (see ATTRIBUTION.md),
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
    "align-321",
    "align-axes",
    "align-center",
    "align-fine",
    "align-points",
    "auto-segment",
    "boolean",
    "chamfer",
    "check",
    "chevron-down",
    "clean",
    "clear",
    "close",
    "constrain",
    "decimate",
    "draft",
    "export-iges",
    "export-mesh",
    "export-sheet",
    "export-step",
    "extrude",
    "fill-holes",
    "fillet",
    "fit-shape",
    "guide",
    "help",
    "hotspots",
    "info",
    "loft",
    "log",
    "mirror",
    "more",
    "noise",
    "pattern",
    "quad",
    "region-enlarge",
    "region-grow",
    "region-merge",
    "region-new",
    "region-shrink",
    "region-split",
    "retopo",
    "revolve",
    "sample",
    "search",
    "section-plane",
    "sections",
    "shell",
    "sketch-chains",
    "sketch-edit",
    "sketch-new",
    "sketch-section",
    "small-pieces",
    "smooth",
    "solid-from-regions",
    "spikes",
    "stage-align",
    "stage-inspect",
    "stage-regions",
    "stage-scan",
    "stage-shapes",
    "stage-sketch",
    "stage-solid",
    "stage-surface",
    "stats",
    "surface-auto",
    "surface-curves",
    "surface-fit",
    "sweep",
    "symmetry",
    "thicken",
    "tool-brush",
    "tool-edges",
    "tool-faces",
    "tool-lasso",
    "tool-move",
    "tool-navigate",
    "tool-rotate",
    "tool-smart",
    "trim",
    "view-home",
    "sk-line",
    "sk-polyline",
    "sk-arc3",
    "sk-arc-centre",
    "sk-circle",
    "sk-rect",
    "sk-horizontal",
    "sk-vertical",
    "sk-parallel",
    "sk-perpendicular",
    "sk-tangent",
    "sk-coincident",
    "sk-equal",
    "sk-fix",
    "sk-dimension",
    "sk-angle",
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
