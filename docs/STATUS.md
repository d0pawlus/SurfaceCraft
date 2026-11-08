# SurfaceCraft — stan projektu

> Aktualizowany po każdym milestonie. Wystarcza do wznowienia pracy z samego repo.
> Źródła prawdy: [design doc](superpowers/specs/2026-11-08-surfacecraft-scan-to-cad-rust-design.md),
> [spike ekosystemu](superpowers/specs/2026-11-08-surfacecraft-ecosystem-spike.md). Plany: `docs/superpowers/plans/`.

## Milestones

| M | Zakres (§8) | Stan | Plan |
|---|---|---|---|
| M0 | szkielet, konwencje, import, viewport, rejestr komend, CLI, smoke kernela | **zamknięty** | [plan](superpowers/plans/2026-11-08-m0-skeleton.md) |
| M1 | wyrównanie: 3-2-1, N-point, ICP, world-axis | do zrobienia | — |
| M2 | prymitywy, auto-typ, symetria, deviation, Region | do zrobienia | — |
| M3 | Fit Surface (NURBS), mapa odchyłek, STEP powierzchni | do zrobienia | — |
| M4 | Freeform/AutoSurface, thicken | do zrobienia | — |
| M5 | szkic + solver, cechy, DAG, STEP brył, IGES | do zrobienia | — |
| M6 | web (wasm), MCP, asystent AI | do zrobienia | — |

## Jak uruchomić

```bash
cargo xtask ci
```

```bash
cargo run -p surfacecraft-cli -- run examples/scripts/m0-smoke.json
```

```bash
cargo run -p surfacecraft
```

## Pomiary

| Co | Wartość | Test |
|---|---|---|
| Smoke kernela: box 20×10×5 → fillet r=2 krawędzi 0 (rekord: 5 mm) | 7/15/10, V = 995,7080 (dokładnie 1000 − (1−π/4)·4·5) | `kernel::solid::tests::smoke_box_fillet_record` |
| Fuzz importu (3000 rund × 4 formaty, losowe bajty + uszkodzenia plików) | 0 panik | `io::mesh::tests::hostile_bytes_never_panic` |
| Wrogie parametry (każda komenda × 20 wartości) | 0 panik | `engine/tests/commands.rs::hostile_params_never_panic` |
| Render offscreen (GPU lokalnie; CI bez GPU — test się pomija) | piksel środka = siatka, róg = tło | `ui-egui::viewport::tests::offscreen_render_shows_the_mesh` |

## Decyzje buildera (z uzasadnieniem)

Szczegóły w planach milestone'ów.
- solidcore z git, pin `b9c02da` (commit sprawdzony w spike, natywnie i wasm).
- Ciała B-rep w dokumencie jako rekord `Brep` (undo, zapis, writery czytają jedną formę).
- Format projektu: JSON + siatki base64 (wersjonowany).
- Viewport: offscreen + blit (niezależność od bufora głębi okna, zrzuty dla MCP/AI).
- `craft-libs`: nieużywane (brak potrzeby, dodatkowe zależności git).

## Dług techniczny

- Adaptacje kodu rodziny: brak (pierwsza planowana w M3: zapis `SHELL_BASED_SURFACE_MODEL` z `solidcraft-step`).
- `surfacecraft-web` to tylko runner eframe (bez wczytywania plików w przeglądarce) — pełny web w M6.
- Brak osobnego repo `surfacecraft-corpus`: korpus w CI jest w całości syntetyczny (`geom::synth`).

## Luki / ryzyka

- CI nie ma GPU: test renderu offscreen pomija się na runnerach (działa lokalnie).
- Zależność git-only solidcore — pin trzeba okresowo podbijać i powtarzać smoke test.

## Następny krok

M1 — wyrównanie (plan `docs/superpowers/plans/…-m1-alignment.md`): Kabsch/Umeyama, ICP point-to-plane na
KD-tree, 3-2-1 datum z płaszczyzn/osi, world-axis (PCA); komendy `align.*`; bramki: odzyskanie znanej
transformacji ≤ 1e-6 mm (bez szumu) i zbieżność ICP Δ < 1e-4 mm/iter.
