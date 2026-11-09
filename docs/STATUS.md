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
| M3 | Fit Surface (NURBS), mapa odchyłek, STEP powierzchni | **zamknięty** | [plan](superpowers/plans/2026-11-08-m3-fit-surface.md) |
| M4 | Freeform/AutoSurface, thicken | **zamknięty** | [plan](superpowers/plans/2026-11-08-m4-freeform.md) |
| M5 | szkic + solver, cechy, DAG, STEP brył, IGES | **zamknięty** | [plan](superpowers/plans/2026-11-08-m5-parametric.md) |
| M6 | web (wasm), MCP, asystent AI | **zamknięty** | [plan](superpowers/plans/2026-11-08-m6-web-mcp-ai.md) |

Etapy po MVP ([plan production-readiness](superpowers/plans/2026-11-09-production-readiness.md)):

| P | Zakres | Stan | Plan |
|---|---|---|---|
| P1 | realne skany: pomiar, edycja siatki, odporność segmentacji i szumu | **zamknięty** | [plan](superpowers/plans/2026-11-09-p1-real-scans.md) |
| P2 | wydajność i skala do 10 mln trójkątów: równoległość, LOD wyświetlania, kolory bez re-uploadu, pomiary | **w toku** (kryterium 3: import, segmentacja, ICP, fps spełnione; patrz tabela) | [plan](superpowers/plans/2026-11-09-p2-performance.md) |

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
| M3: Fit Surface, fala 60×40, A = 3 mm | bez szumu: siatka 6×6, RMS 0,020, max 0,067 mm; σ = 0,02: RMS 0,029, max 0,108 mm (próg 0,2) | `gate_m3_fit_surface_max_deviation` |
| M3: STEP arkusza, round-trip przez importer solidcore | 1 ściana, odległość powierzchni 1,7e-11 mm; odchyłka skanu zgodna < 0,02 mm | `gate_m3_sheet_step_roundtrip`, `gate_m3_surface_step_roundtrip_through_commands` |
| M4: AutoSurface, fala, 12 quadów | bez szumu max 0,044 mm, σ = 0,02 max 0,089 mm (próg 0,2); 12 łat, wspólne krawędzie | `gate_m4_autosurface_deviation_and_continuity` |
| M4: ciągłość łat | skok normalnej ≤ 8e-5 rad (różnice skończone; C² z konstrukcji) | jw. |
| M4: thicken 2 mm → bryła | objętość w zakresie pole × 2 mm (+< 20 %), eksport STEP + import | `split_merge_crease_and_thicken` |
| M5: solver — prostokąt 40×20 | fully defined, DOF 0, wymiary z błędem < 1e-9; więz redundantny i sprzeczny wykryte (status over_defined) | `gate_m5_rectangle_fully_defined`, `gate_m5_overdefinition_detected`, `gate_m5_sketch_solver_through_commands` |
| M5: historia — extrude, cut, fillet, edycja wymiaru 40→50, edycja głębokości, undo/redo, suppress | objętości zgodne z wartościami dokładnymi < 1e-2 mm³ | `gate_m5_features_dag_replay_and_undo` |
| M5: STEP bryły round-trip | V 7982,831853 → 7982,831853, 7 → 7 ścian | `gate_m5_step_roundtrip_and_iges` |
| M5: IGES | 186/514, sekcje S/G/D/P/T spójne, 128 odczytane bit w bit | `io::iges` testy |
| M5: podpowiedź promienia z meshu (fillet r = 3) | błąd < 0,05 mm | `section_sketch_pattern_shell_boolean_and_suggestion` |
| M6: MCP | `tools/list` = 104+ narzędzi: każda komenda rejestru + 8 ogólnych; przepływ synth → segment → STEP przez stdio | `gate_m6_mcp_stdio_exposes_and_runs_every_command`, `bridge_forwards_to_a_running_app` |
| M6: web | bundle 59 MB (release z line-tables), uruchomiony w przeglądarce: mesh.synth z palety → siatka w viewporcie WebGPU/WebGL; potok bez plików | `gate_m6_web_subset_runs_without_files`, `cargo xtask web` |
| M6: asystent AI | tryb align: 2 komendy dostawcy wykonane, wynik 60×30×10 wzdłuż osi, undo cofa | `gate_m6_assistant_align_mode`, `assist_runs_a_command_provider` |
| P1: lokalny korpus QS (10 plików, 99 k – 651 k trójkątów; dane poza repo) | 10/10 przechodzi import → clean (+dziury) → szum → segmentacja → prymitywy → freeform → ICP; 10 największych regionów każdego pliku w progu max(0,1 mm, 2σ): 100/100; ICP kopii przesuniętej o 3° + 1,8 mm: błąd ≤ 1,4e-11 mm; σ estymowane 0,0006–0,015 mm; przypisane 95–99 % trójkątów; segmentacja 7,8–41 s (651 k trójkątów → P2) | `local_corpus_full_pipeline` (`--ignored`, `SURFACECRAFT_LOCAL_CORPUS`) |
| P1: freeform na realnych danych | największy nieprzypisany kawałek ≥ 500 trójkątów: 1 plik jako płat max 0,167 mm (≤ 0,2); w 2 plikach kawałek zawinięty/złożony (odmowa — potrzebne P3/P4); w 7 resztki < 500 trójkątów | jw. |
| P1: korpus CAD „skanowany” w CI (rampa gęstości ×2, σ = 0,03, dziury, kolce, śmieci) | blok z zaokrągleniami r = 5: 6 płaszczyzn + 4 walce, promień ± 0,1 (ćwierć łuku); wałek z fazą 3 mm: walec r = 12 ± 0,03, stożek 45° ± 0,5°; po clean: 0 dziur, 1 komponent, każdy wierzchołek < 6σ od CAD; RMS ≤ 0,1 | `p1_scanned_filleted_block`, `p1_scanned_chamfered_cylinder` |
| P1: edycja siatki | decymacja QEM: pudełko zostaje pudełkiem (wierzchołki na ścianach < 1e-6), sfera 10 % trójkątów < 0,3 mm; Taubin: RMS szumu ×< 0,5 bez skurczu (< 0,05 mm); dziura gładka (cienka płyta) vs płaska na sferze: 1,5 vs 4,0 mm; deterministyczne | `geom::edit::tests`, `p1_mesh_edit_commands` |
| P1: estymator szumu | bracket σ = 0,05 → 0,052 (było 0,078); skanowany box σ = 0,03 → 0,034 | `noise_estimate_is_robust_to_edges_and_density` |
| P2: skala (maszyna referencyjna: AMD Ryzen 5 5600, 6 rdzeni / 12 wątków, 32 GB RAM, RTX 5070, Windows 11; pomiar na maszynie współdzielonej z innymi buildami, więc czasy są raczej górne) | 1 M (0,93 M trójkątów): import 0,12 s, clean 0,21 s, segmentacja 4,6 s (28 regionów), ICP 0,28 s, odchyłka 0,27 s. 5 M (4,77 M): import 0,62 s, clean 1,0 s, segmentacja 23,6 s (901 regionów), ICP 1,8 s (100 it.), odchyłka 1,6 s. 10 M (10,12 M): import 1,5 s (cel < 15), clean 2,3 s, segmentacja 54,8 s (cel < 60; 2060 regionów, 17,3 k dopasowań ziaren, 28,6 mln iteracji LM), ICP 3,1 s (cel < 10), odchyłka 4,0 s | `perf_scan_scale` (`--ignored`, `SURFACECRAFT_PERF_SIZES=1,5,10`) |
| P2: viewport | skan 10 M trójkątów przez LOD (1,6 M narysowanych, przygotowanie 2,5 s jednorazowo): 508 klatek/s w renderze offscreen przy krążącej kamerze (cel ≥ 30), geometria wysłana raz; zmiana kolorów = jeden `write_buffer` | `fps_with_a_large_scan` (`--ignored`, pomija się bez GPU), `colour_change_does_not_reupload_geometry` |
| P2: korpusy lokalne (dane poza repo) | QS2026 (10 plików): segmentacja 5,8–25,7 s (przed P2: 7,8–41 s), 99/100 największych regionów w progu (C10: 9/10, nie zbadane czy to nie stan z main po P1/relax); Creality: Mesh.ply 484 k trójkątów 21 s, Miata34 2,5 M 137 s (było 330–360 s; 9958 regionów = limit 10 000, kształt organiczny: „10 największych w progu" 10/10, przypisane 69 %), slupek_NB 3 M 145 s (9848 regionów, 71 %); Optimization.ply (chmura bez ścianek) pominięty | `local_corpus_full_pipeline` |

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
- Fit do punktów rozproszonych (§10): resampling regionu do siatki (MLS nad płaszczyzną PCA) → `lst_sq_approx` w kernelu; `app-geometry` pusta.
- Arkusz z płata: rekord `Brep` z jedną ścianą `Spline` składany w `kernel::nurbs` (sprawdzany przez `validate` i `Body::from_record`).
- Retopologia (§10): strukturalny układ quad na mapie wysokości (bez portu Instant Meshes i bez procesu zewnętrznego); AutoSurface = jeden B-spline na blok rozłożony na łaty Béziera (G1 z konstrukcji).
- Solver więzów: własny (LM + analiza rzędu jakobianu), bez adaptacji `solidcraft-sketch`.
- MCP: własny JSON-RPC (bez `rmcp`), narzędzie na komendę; bridged przez kanał sterowania 127.0.0.1 + token.
- Asystent AI: pętla narzędzi niezależna od dostawcy; dostawca = program z `SURFACECRAFT_AI_COMMAND` (wybór usługi LLM czeka na użytkownika).
- Historia: lista cech z replay (DAG z zależności przez ostatniego pisarza ciała); undo przez migawki dokumentu.

## Dług techniczny

- **Adaptacja (dług):** `crates/io/src/step/{p21,mod}.rs` z `solidcraft-step` (zapis otwartych powłok STEP) — do wymiany na `step-export` solidcore, gdy zapisze `ShellModel` (PR upstream opcjonalny, tylko za zgodą użytkownika).
- STEP arkuszy bez PCURVE (krawędzie tylko krzywą 3D); importer solidcore odtwarza je bez strat w testach.
- **Adaptacja (dług):** `crates/io/src/iges.rs` z `iges_out` SolidCrafta — do wymiany, jeśli writer IGES trafi do solidcore (propozycja upstream wymaga zgody użytkownika: działanie poza repo).
- Szkice w viewporcie przeliczane co klatkę (małe; do cache, gdy urosną).
- Web: bundle nieoptymalizowany rozmiarowo (59 MB; brak `wasm-opt`), limit 5 mln trójkątów; brak MCP/kanału sterowania i asystenta bez dostawcy po stronie przeglądarki.
- Brak osobnego repo `surfacecraft-corpus`: `tangency-labs/models` nie ma jeszcze modeli (2026-11-08), więc korpus
  poziomu 1 w CI to części CAD z kernela „skanowane” w teście (`synth::scanify`); gdy pojawią się modele CC0, dołożyć je.
- Gizmo manipulatora w viewporcie (§6.1) — w M1 ruch ręczny tylko przez `mesh.transform`.
- Regiony: brak resegment/insert/separate/enlarge/shrink (§8 dopuszcza później); zaznaczanie w GUI tylko przez wskazanie + `region.grow` (bez pędzla/lasso).
- Segmentacja skanów organicznych (Miata34, slupek: ~10 000 regionów, limit) trwa 140 s na 2,5–3 mln trójkątów; wzrost regionów i refity są sekwencyjne (równolegle tylko ziarna, rodzaje prymitywów, RMS, odchyłki). 10 M syntetycznych: 55 s przy celu 60 s — mały zapas.
- Import STL 10 M: `weld` sortuje 30 mln kluczy (~1 GB chwilowo); parsowanie binarnego STL jest jednowątkowe. Out-of-core (design doc §9) nadal bez implementacji; `PointSource`/`TriangleSource` to tylko szew.
- Pomiary P2 robione na maszynie współdzielonej z innymi buildami (OOM rustc przy równoległych kompilacjach solidcore); fps tylko offscreen, bez egui.
- Fragmentacja pierścieni (płaszczyzny z otworem): `merge_adjacent` dochodzi teraz do punktu stałego (≤ 64 przebiegów, odrzucone pary nie są fitowane ponownie); płaski pierścień syntetyczny i tak wychodził w jednym kawałku, więc przyczyny z walca z fazą i otworem (P3) nie odtworzono — do dalszego zbadania (ziarna na pierścieniu?).
- Edycja siatki: decymacja QEM i wypełnianie dziur bez równoległości; dziury > 4000 krawędzi pomijane.
- Fit Surface: jeden płat na region wysokościowy nad płaszczyzną (region zawinięty → odmowa, trzeba podzielić); brzeg płata = prostokąt w parametrach (bez przycinania do konturu regionu).
- Retopologia niestrukturalna (pole kierunków, regiony zawinięte/zamknięte) — brak; AutoSurface wymaga regionu-mapy wysokości.
- solidcore: `thicken` odmawia na wielościennym arkuszu (offset *partial*) — `surface.thicken` używa jednościennego arkusza-rodzica; bloki z przełamaniami pogrubia się osobno. Kandydat do zgłoszenia upstream (za zgodą).
- Indeksy krawędzi/ścian w cechach (fillet, shell) są pozycyjne: po zmianie topologii wcześniejszej cechy mogą wskazać inną krawędź (brak trwałego nazewnictwa topologii).
- Auto-constrain relacji między prymitywami (§6.2) i część operacji (trim/split, sweep, loft) nie mają komend cech.
- **Asystent AI bez skonfigurowanego dostawcy LLM:** działa z dowolnym programem-dostawcą, ale żadna usługa nie jest wybrana ani podpięta (decyzja użytkownika: dostawca i klucz API).
- Kanał sterowania zapisuje token w pliku tymczasowym (dostęp lokalnego użytkownika); jedna instancja aplikacji naraz.

## Luki / ryzyka

- CI nie ma GPU: test renderu offscreen pomija się na runnerach (działa lokalnie).
- Zależność git-only solidcore — pin trzeba okresowo podbijać i powtarzać smoke test.

## Następny krok

Wszystkie milestones M0–M6 są zamknięte. Do decyzji użytkownika:
1. Dostawca LLM i klucz API dla asystenta (dziś: dowolny program przez `SURFACECRAFT_AI_COMMAND`).
2. Upstream (za zgodą użytkownika, zgodnie z zasadami solidcore: najpierw issue z reprodukcją): zgłoszone
   tangency-labs/solidcore#3228 („sheet from surface”, `face_on` 1001), #3229 (thicken arkusza wielościennego → `NotSolid`),
   #3230 (propozycja writera IGES nad rekordem). Zapis STEP otwartych powłok jest już przenoszony upstream (#2579) —
   bez dublowania. Reprodukcja: gałąź `repro/surfacecraft-sheets` w forku `d0pawlus/solidcore`. Po decyzji maintainerów:
   PR-y lub wymiana adaptacji. Wpis w `craft-launcher` — nadal czeka na zgodę.
Plan dojścia do pełnoprawnego programu: [`docs/superpowers/plans/2026-11-09-production-readiness.md`](superpowers/plans/2026-11-09-production-readiness.md)
P1 zamknięty; następny: P2 (wydajność do 10 mln trójkątów) i równolegle P5 (stabilna parametryka).
Kandydaci post-MVP z design doc: retopologia niestrukturalna (Instant Meshes), druga strategia freeform (sieć łat
z krzywych cech), CAD Correct/Refit, trwałe nazewnictwo topologii w cechach, out-of-core/LOD dla 100 mln+ trójkątów.
