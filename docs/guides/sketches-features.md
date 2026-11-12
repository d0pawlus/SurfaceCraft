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
- `sketch.move_point` with `sketch`, `point` (index) and `to` (sketch coordinates) moves one point
  and solves: the constraints move the rest of the sketch with it, and a point they fix stays
  where they put it. It is what a mouse drag issues. In the editor, press on a point in the
  select tool and drag it; Esc during the drag cancels it; a release where nothing moves issues
  nothing; one drag is one undo step; the point under the press is the one dragged, and an undo
  or any other change during the drag cancels it. A move that would leave a dimension unsatisfied
  (a dimensioned line collapsed to a point) gives the nearest valid sketch, or an error when none
  exists; the sketch is never committed with an unsatisfied constraint. Alt+drag still orbits the
  camera.
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
- `feature.rollback` with `to` moves the roll-back marker. `to` is `"end"`, `{"index": n}` (the
  first rolled-back feature is at position `n`) or `{"feature": ref}`. The features from the
  marker on are parked: not replayed, so the body is as it was at that step. A feature added now
  goes in front of the parked ones, and `"end"` brings every parked feature back. `feature.tree`
  reports `marker` (the number of active features) and `rolled_back` on each node. In the history
  panel, click the marker to restore everything, drag it in front of a feature, or use "Roll back
  here" on a row. Parked features cannot be edited or suppressed until the marker moves past them.
  Parked features and the bodies only they write keep their names reserved and come back with
  them. When a parked feature no longer replays (a chamfer added meanwhile took its edge), `"end"`
  still restores it, marks it failed and lists it in `failures` of the result with the reason.
- `feature.replay` rebuilds every body made by features.

## Redesign helpers

- `feature.suggest_axis` gives the axis of a turned scan region, for `feature.revolve`.
- `feature.suggest_draft` gives the draft angle of a scanned face, for `feature.draft`.
- `feature.suggest_fillet` gives the radius of a rounded scan edge, for `feature.fillet`.

## Check

- `feature.tree` shows the error of each failing feature. A feature that fails is not applied
  and the engine rolls the history back, so the document stays as it was.
