# Sketches and features

Model a part from sketches. A closed profile is extruded, revolved, lofted or swept, and the
rest of the history (fillets, booleans, patterns) is kept as features. Features can be edited,
suppressed and replayed.

## Sketch

- `sketch.create` with `plane` and `name`. The plane is `{"origin": [x, y, z], "normal": [x, y, z]}`,
  or `origin` with `x` and `y` axes. Without `plane` the sketch is on XY.
- `sketch.add` with `sketch` and `entities`. Each entity is one of `line` (`from`, `to`), `arc`
  (`center`, `start`, `end`, counter-clockwise), `circle` (`center`, `radius`) or `polygon`
  (`points`). Shared end points are joined.
- `sketch.constrain` with `sketch` and `constraints` (their shape is in `describe_command`).
  It solves and reports the degrees of freedom and any over-definition.
- `sketch.edit_constraint` replaces (or removes, when `constraint` is omitted) constraint
  `index`. `sketch.solve` solves again and replays the features that depend on the sketch.
- `sketch.from_section` cuts a `mesh` with a plane (`origin`, `normal`, `tolerance`) and fits
  lines and arcs into a constrained sketch. Use it to redesign a scanned section.

## Features

- `feature.extrude` with `sketch`, `distance` (negative for the other side), `draft_deg`,
  `mode` (`new`, `add` or `cut`) and `target` for add and cut.
- `feature.revolve` with `sketch`, `axis_origin`, `axis_direction`, `angle_deg` (default 360),
  `mode` and `target`.
- `feature.loft` skins a solid through sketch profiles. `feature.sweep` sweeps a profile along
  an open path sketch.
- `feature.fillet` and `feature.chamfer` with `body`, `edges` (indices) and `size`.
- `feature.shell` with `body`, `faces` (the faces removed) and `thickness`.
- `feature.pattern` with `body` and `count`; `translate` for a linear pattern, or `rotate_axis`,
  `rotate_origin` and `rotate_deg` for a circular one. `feature.path_pattern` repeats a body
  along a path sketch.
- `feature.boolean` with `body`, `tool` and `op` (`union`, `subtract` or `intersect`).
- `feature.mirror`, `feature.split` (keeps the `front` or `back` side) and `feature.draft`
  complete the set.

## Edit and replay

- `feature.tree` lists the history with dependencies and errors. Start there.
- `feature.edit` with `feature` and `set`, for example `{"distance": 12}`, changes parameters
  and replays the history.
- `feature.suppress` with `feature` and `suppressed` turns a feature off, or on again.
- `feature.replay` rebuilds every body made by features.

## Redesign helpers

- `feature.suggest_axis` gives the axis of a turned scan region, for `feature.revolve`.
- `feature.suggest_draft` gives the draft angle of a scanned face, for `feature.draft`.
- `feature.suggest_fillet` gives the radius of a rounded scan edge, for `feature.fillet`.

## Check

- `feature.tree` shows the error of each failing feature. A feature that fails is not applied
  and the engine rolls the history back, so the document stays as it was.
