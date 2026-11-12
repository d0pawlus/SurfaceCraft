# Export

Write the model out. Each export takes a `path` (needs file access), or leaves `path` out and
returns the file as base64 `data`. The second works over MCP with files off.

## Choose the format

- `export.step` writes STEP AP214: surfaces (open shells) or solid bodies. `export.iges` writes
  IGES 5.3 for surfaces and bodies. Both take `objects` (ids or names), `path` or none, and
  `name`, the product name written into the file.
- A file holds sheets or solids, not both: export them in separate files. The result's `content`
  says which kind was written.
- A solid made of thousands of spline patches (the result of `surface.wrap`) is written compactly:
  vertices, edges and control points once, about 2 KB per bicubic patch. Solids with faces the
  compact writer does not support (for example intersection-curve edges) go through the solidcore
  exporter, which needs about 17 to 24 KB per patch.
- Scan meshes: `mesh.export` with `object`, `path` or none, and `format` (`stl`, `obj` or `ply`).
- Bringing data in: `import.step` reads the bodies of a STEP file, `mesh.import` reads meshes.

## Steps

1. Run `doc.list` (MCP: `inspect_design`) to get the ids of the objects to write.
2. Call `export.step` with `objects` and no `path`. Read `data` (base64) and `bytes`.
3. Decode `data` on the client, and check that `bodies` equals the number of objects you sent.

## Check

- `bodies` should equal the number of objects exported.
- To confirm a file reads back, import it with `import.step` into an empty document and compare
  the body count.
