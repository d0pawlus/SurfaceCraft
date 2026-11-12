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

## Parameters and expressions

A document holds a table of named parameters. A dimension of a sketch and any numeric field of a
feature can be given as an expression over them, and changing a parameter recomputes everything
that uses it in one undo step.

- `param.set` with `name`, `expression` and an optional `comment` creates or changes a parameter.
  A name is a letter or underscore, then letters, digits and underscores (case-sensitive; `pi`
  and the function names are reserved). A parameter may use other parameters; a cycle is refused
  and named (`a -> b -> a`), and so is an expression that does not evaluate. The result lists the
  values that changed and `failures`: the sketches that cannot be solved and the features that
  fail with the new values. The change itself still applies, and one undo restores everything.
  A change that would stop another parameter from evaluating (`h = sqrt(w - 10)` with `w = 5`) is
  refused whole.
- `param.list` shows each parameter with its expression, value and users; `param.eval` calculates
  an expression without storing it; `param.rename` rewrites every use of a name; `param.delete`
  is refused, with the list of users, while anything uses the parameter. Features rolled back by
  `feature.rollback` count as users, and their expressions are re-evaluated when they come back.
- The language: numbers (`40`, `1.5e-3`), `+ - * / ^`, parentheses, unary minus, parameter names,
  `pi`, and the functions `sin cos tan asin acos atan atan2` (angles in degrees, results of the
  inverse ones too), `sqrt abs min max floor ceil round`. `^` is right-associative and binds
  tighter than a minus sign in front of it: `2^3^2` is 512 and `-2^2` is -4, while `2^-1` is 0.5.
  The language has no units: lengths are millimetres, angles degrees. A division by zero, a
  square root of a negative number or a result that is not finite is an error, and so is an
  expression longer than 1024 characters or nested more than 64 levels.
- Where an expression goes: write `{"expr": "w / 2"}` in place of the number. In
  `sketch.constrain` and `sketch.edit_constraint` that is the `value` of a length, distance or
  radius and the `degrees` of an angle; in feature commands it is any numeric field
  (`distance`, `draft_deg`, `angle_deg`, `size`, `thickness`, `count`, `degree`, and each
  component of a vector such as `translate` or `plane_normal`), also inside `feature.edit`'s
  `set`. A count must come out whole (to within 1e-9 of an integer) and at most 100000. A plain
  number given later for the same field replaces the expression. `feature.tree` shows the
  `expressions` next to the values.

One example, a block whose depth and fillet follow parameters (edge 4 stands for a vertical edge
of the block: read the real index from the body):

```json
{"commands": [
  {"command": "param.set", "params": {"name": "w", "expression": "40"}},
  {"command": "param.set", "params": {"name": "h", "expression": "w / 2"}},
  {"command": "param.set", "params": {"name": "d", "expression": "10"}},
  {"command": "param.set", "params": {"name": "r", "expression": "d / 5"}},
  {"command": "sketch.create", "params": {"name": "base"}},
  {"command": "sketch.add", "params": {"sketch": "base", "entities": [
    {"polygon": {"points": [[0, 0], [40, 0], [40, 20], [0, 20]]}}]}},
  {"command": "sketch.constrain", "params": {"sketch": "base", "constraints": [
    {"type": "fix", "point": 0, "at": [0, 0]},
    {"type": "horizontal", "line": 0}, {"type": "vertical", "line": 1},
    {"type": "horizontal", "line": 2}, {"type": "vertical", "line": 3},
    {"type": "length", "line": 0, "value": {"expr": "w"}},
    {"type": "length", "line": 1, "value": {"expr": "h"}}]}},
  {"command": "feature.extrude", "params": {"sketch": "base", "distance": {"expr": "d"}, "name": "blk"}},
  {"command": "feature.fillet", "params": {"body": "blk", "edges": [4], "size": {"expr": "r"}}},
  {"command": "param.set", "params": {"name": "w", "expression": "60"}}
]}
```

The last step makes the block 60 by 30, still 10 deep with a 2 mm fillet; `param.set` with
`d` = 20 would make it 20 deep with a 4 mm fillet, and `edit.undo` takes either back.

## Redesign helpers

- `feature.suggest_axis` gives the axis of a turned scan region, for `feature.revolve`.
- `feature.suggest_draft` gives the draft angle of a scanned face, for `feature.draft`.
- `feature.suggest_fillet` gives the radius of a rounded scan edge, for `feature.fillet`.

## Check

- `feature.tree` shows the error of each failing feature. A feature that fails is not applied
  and the engine rolls the history back, so the document stays as it was.
