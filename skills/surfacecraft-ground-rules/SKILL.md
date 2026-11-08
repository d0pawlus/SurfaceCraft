---
name: surfacecraft-ground-rules
description: Licences, assets, clean-room and test data rules for SurfaceCraft.
---

# Ground rules

- **Clean-room.** QuickSurface and Geomagic Design X were studied only through their documented
  behaviour (`docs/superpowers/specs/*functional-analysis.md`). Never add their code, icons, texts,
  layouts, or data. Their sample scans (e.g. `X:\QUICKSURFACE2026`) are used only locally:
  set `SURFACECRAFT_LOCAL_CORPUS` and run the ignored tests (`cargo test -- --ignored local_corpus`).
  Nothing derived from them (numbers, screenshots, exported files) is committed.
- **Licences.** Dependencies must be on the allowlist in `xtask/src/licences.rs` (MIT, Apache-2.0,
  BSD, ISC, Zlib, Unicode, CC0, BSL-1.0 …). GPL/LGPL/AGPL is refused. `cargo xtask licences --check`
  also regenerates `THIRD-PARTY-LICENSES.txt` (never edit it by hand).
- **Assets.** Every non-code file has a row in `ATTRIBUTION.md` (`cargo xtask assets`).
- **Family code** (solidcore, SolidCraft, CADCraft; MIT OR Apache-2.0): first use solidcore, then
  propose it upstream, and only then adapt code with attribution: keep the copyright header, mark
  the file as modified, add a row to ATTRIBUTION.md "Adapted code" and to NOTICE, and list it as debt
  in `docs/STATUS.md`. Never depend on heavy application crates (e.g. `solidcraft-kernel`) by git.
- **Test corpus.** In-repo data is synthetic (`surfacecraft_geom::synth`) or openly licensed
  (e.g. `tangency-labs/models`, CC0), each file with its licence in ATTRIBUTION.md.
