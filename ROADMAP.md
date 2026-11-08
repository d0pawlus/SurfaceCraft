# Roadmap

Milestones from the design doc (§8); current state in [docs/STATUS.md](docs/STATUS.md).

- **M0 — skeleton:** workspace and family conventions, layers, mesh import (STL/OBJ/PLY), wgpu
  viewport, command registry, CLI, solidcore pin and kernel smoke test.
- **M1 — alignment:** 3-2-1 datum, N-point (Kabsch), ICP point-to-plane, world axes.
- **M2 — primitives:** plane/cylinder/cone/sphere/torus fits, automatic type, symmetry plane,
  deviation, persistent `Region` (auto-segment, merge, split).
- **M3 — fit surface:** one NURBS patch per region, deviation map, STEP export of surfaces.
- **M4 — freeform:** quad retopology, NURBS patches, snap/crease/merge, thicken to solid.
- **M5 — parametric:** 2-D sketch with constraint solver, sections, extrude/revolve,
  fillet/chamfer/shell, pattern/trim/cut/boolean, history DAG; STEP solids and IGES.
- **M6 — web + MCP + AI assistant.**
