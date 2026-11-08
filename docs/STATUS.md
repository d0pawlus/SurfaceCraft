# SurfaceCraft — stan projektu

> Aktualizowany po każdym milestonie. Wystarcza do wznowienia pracy z samego repo.
> Źródła prawdy: [design doc](superpowers/specs/2026-11-08-surfacecraft-scan-to-cad-rust-design.md),
> [spike ekosystemu](superpowers/specs/2026-11-08-surfacecraft-ecosystem-spike.md). Plany: `docs/superpowers/plans/`.

## Milestones

| M | Zakres (§8) | Stan | Plan |
|---|---|---|---|
| M0 | szkielet, konwencje, import, viewport, rejestr komend, CLI, smoke kernela | **zamknięty** | [plan](superpowers/plans/2026-11-08-m0-skeleton.md) |
| M1 | wyrównanie: 3-2-1, N-point, ICP, world-axis | **zamknięty** | [plan](superpowers/plans/2026-11-08-m1-alignment.md) |
| M2 | prymitywy, auto-typ, symetria, deviation, Region | **zamknięty** | [plan](superpowers/plans/2026-11-08-m2-primitives.md) |
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
| M1: ICP bez szumu (bracket, ruch 4° + 2,7 mm) | błąd 2,1e-12 mm (próg 1e-6), 4 iteracje, ostatnie Δ 2,6e-5 mm (próg 1e-4) | `align::icp::tests::gate_m1_icp_recovers_known_motion_noise_free` |
| M1: N-point i 3-2-1 bez szumu | ≤ 1e-9 mm | `gate_m1_n_point_command_recovers_known_motion`, `gate_m1_datum_321_recovers_known_motion` |
| M1: ICP z szumem σ = 0,02 mm | RMS 0,015 mm, 9 iteracji, kąt odzyskany 3,996° z 4° | `examples/scripts/m1-align.json`, `noisy_scan_converges_close_to_truth` |
| M2: fit przy σ = 0,05 mm (plane/sphere/cylinder/cone/torus) | RMS 0,0505 / 0,0495 / 0,0504 / 0,0504 / 0,0496 mm (próg 0,1) | `fit::primitive::tests::gate_m2_noisy_fits_within_rms` |
| M2: auto-typ, 5 rodzajów przy σ = 0,05 mm | 5/5 poprawnie; stożek 0,05° → walec | `fit::auto::tests::gate_m2_auto_type_is_correct` |
| M2: auto-segmentacja bracketu (σ = 0,05) | 6 płaszczyzn + walec + sfera, RMS regionów 0,038–0,052 mm, ≥ 90 % trójkątów przypisanych | `gate_m2_auto_segment_bracket`, `gate_m2_segment_and_fit_through_commands` |
| M2: walec bossa r = 6 mm | błąd promienia < 0,02 mm | `engine/tests/primitives.rs` |
| M2: estymacja szumu | płaszczyzna σ=0,05 → 0,03–0,08; bracket σ=0,05 → 0,078 (krawędzie i krzywizna zawyżają) | `deviation::tests` |

## Decyzje buildera (z uzasadnieniem)

Szczegóły w planach milestone'ów.
- solidcore z git, pin `b9c02da` (commit sprawdzony w spike, natywnie i wasm).
- Ciała B-rep w dokumencie jako rekord `Brep` (undo, zapis, writery czytają jedną formę).
- Format projektu: JSON + siatki base64 (wersjonowany).
- Viewport: offscreen + blit (niezależność od bufora głębi okna, zrzuty dla MCP/AI).
- `craft-libs`: nieużywane (brak potrzeby, dodatkowe zależności git).
- Solver nieliniowy (§10): własny Levenberg–Marquardt w `fit::lm` (≤ 7 parametrów; bez zależności).
- Auto-typ: najprostszy rodzaj z RMS ≤ 1,2 × najlepszego (+ odrzucanie wyników zdegenerowanych).
- Segmentacja: deterministyczny fit-and-grow na spójności meshu (wariant Efficient RANSAC).

## Dług techniczny

- Adaptacje kodu rodziny: brak (pierwsza planowana w M3: zapis `SHELL_BASED_SURFACE_MODEL` z `solidcraft-step`).
- `surfacecraft-web` to tylko runner eframe (bez wczytywania plików w przeglądarce) — pełny web w M6.
- Brak osobnego repo `surfacecraft-corpus`: korpus w CI jest w całości syntetyczny (`geom::synth`).
- Gizmo manipulatora w viewporcie (§6.1) — w M1 ruch ręczny tylko przez `mesh.transform`.
- Regiony: brak resegment/insert/separate/enlarge/shrink (§8 dopuszcza później); zaznaczanie w GUI tylko przez wskazanie + `region.grow` (bez pędzla/lasso).
- Estymator szumu zawyża przy ostrych krawędziach i dużej krzywiźnie względem gęstości siatki.

## Luki / ryzyka

- CI nie ma GPU: test renderu offscreen pomija się na runnerach (działa lokalnie).
- Zależność git-only solidcore — pin trzeba okresowo podbijać i powtarzać smoke test.

## Następny krok

M3 — Fit Surface (plan `docs/superpowers/plans/…-m3-fit-surface.md`): płat NURBS na region (parametryzacja regionu,
resampling do siatki → `srf_fit::lst_sq_approx` w kernelu, lub LSQ rozproszony na allowliście `app-geometry`),
mapa odchyłek przez `surface_closest_point`, arkusz jako rekord `Brep` z jedną ścianą `Spline`, eksport STEP
powierzchni przez adaptację zapisu `SHELL_BASED_SURFACE_MODEL` z `solidcraft-step` (z atrybucją, jako dług);
bramka: max odchyłka ≤ 0,2 mm i round-trip STEP.
