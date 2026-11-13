//! The sketch tools: the drawing tools, the constraint tools, and what each one looks for.
use super::*;

/// A sketch tool.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Tool {
    /// No drawing: pick and double-click dimensions.
    Select,
    /// A line from two clicks.
    Line,
    /// A chain of lines (Enter, right click or a click on the last point ends it; a click on
    /// the first point closes it).
    Polyline,
    /// An arc from start, end and a point on it.
    Arc3,
    /// An arc from centre, start and end direction.
    ArcCentre,
    /// A circle from centre and a point on it.
    Circle,
    /// A rectangle from two corners.
    Rect,
    /// Horizontal line.
    Horizontal,
    /// Vertical line.
    Vertical,
    /// Two parallel lines.
    Parallel,
    /// Two perpendicular lines.
    Perpendicular,
    /// A line tangent to an arc or circle.
    Tangent,
    /// Two coincident points.
    Coincident,
    /// Two lines of equal length.
    Equal,
    /// A point fixed where it is.
    Fix,
    /// A length, radius or distance, typed after the click.
    Dimension,
    /// The angle between two lines, typed after the clicks.
    Angle,
}

impl Tool {
    /// The drawing tools.
    pub const DRAW: [Tool; 6] = [Tool::Line, Tool::Polyline, Tool::Arc3, Tool::ArcCentre, Tool::Circle, Tool::Rect];
    /// The constraint tools.
    pub const CONSTRAIN: [Tool; 10] = [
        Tool::Horizontal,
        Tool::Vertical,
        Tool::Parallel,
        Tool::Perpendicular,
        Tool::Tangent,
        Tool::Coincident,
        Tool::Equal,
        Tool::Fix,
        Tool::Dimension,
        Tool::Angle,
    ];

    /// The catalogue key of the tool's name.
    #[must_use]
    pub fn label_key(self) -> &'static str {
        match self {
            Tool::Select => "sketch.tool.select",
            Tool::Line => "sketch.tool.line",
            Tool::Polyline => "sketch.tool.polyline",
            Tool::Arc3 => "sketch.tool.arc3",
            Tool::ArcCentre => "sketch.tool.arc_centre",
            Tool::Circle => "sketch.tool.circle",
            Tool::Rect => "sketch.tool.rect",
            Tool::Horizontal => "sketch.tool.horizontal",
            Tool::Vertical => "sketch.tool.vertical",
            Tool::Parallel => "sketch.tool.parallel",
            Tool::Perpendicular => "sketch.tool.perpendicular",
            Tool::Tangent => "sketch.tool.tangent",
            Tool::Coincident => "sketch.tool.coincident",
            Tool::Equal => "sketch.tool.equal",
            Tool::Fix => "sketch.tool.fix",
            Tool::Dimension => "sketch.tool.dimension",
            Tool::Angle => "sketch.tool.angle",
        }
    }

    /// The icon of the tool in the sketch toolbar.
    #[must_use]
    pub fn icon(self) -> &'static str {
        match self {
            Tool::Select => "tool-navigate",
            Tool::Line => "sk-line",
            Tool::Polyline => "sk-polyline",
            Tool::Arc3 => "sk-arc3",
            Tool::ArcCentre => "sk-arc-centre",
            Tool::Circle => "sk-circle",
            Tool::Rect => "sk-rect",
            Tool::Horizontal => "sk-horizontal",
            Tool::Vertical => "sk-vertical",
            Tool::Parallel => "sk-parallel",
            Tool::Perpendicular => "sk-perpendicular",
            Tool::Tangent => "sk-tangent",
            Tool::Coincident => "sk-coincident",
            Tool::Equal => "sk-equal",
            Tool::Fix => "sk-fix",
            Tool::Dimension => "sk-dimension",
            Tool::Angle => "sk-angle",
        }
    }

    /// The catalogue key of the hint shown while the tool is active.
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            Tool::Select => "sketch.hint.select",
            Tool::Line => "sketch.hint.line",
            Tool::Polyline => "sketch.hint.polyline",
            Tool::Arc3 => "sketch.hint.arc3",
            Tool::ArcCentre => "sketch.hint.arc_centre",
            Tool::Circle => "sketch.hint.circle",
            Tool::Rect => "sketch.hint.rect",
            Tool::Horizontal | Tool::Vertical | Tool::Fix => "sketch.hint.one",
            Tool::Parallel | Tool::Perpendicular | Tool::Equal => "sketch.hint.two_lines",
            Tool::Tangent => "sketch.hint.tangent",
            Tool::Coincident => "sketch.hint.two_points",
            Tool::Dimension => "sketch.hint.dimension",
            Tool::Angle => "sketch.hint.angle",
        }
    }

    /// Does the tool draw geometry?
    #[must_use]
    pub fn draws(self) -> bool {
        Tool::DRAW.contains(&self)
    }

    /// What the first click of a constraint tool looks for.
    #[must_use]
    pub fn want(self) -> Want {
        match self {
            Tool::Horizontal | Tool::Vertical | Tool::Parallel | Tool::Perpendicular | Tool::Equal | Tool::Angle => {
                Want::Line
            }
            Tool::Tangent => Want::LineOrCircular,
            Tool::Coincident | Tool::Fix => Want::Point,
            _ => Want::PointOrEntity,
        }
    }
}
