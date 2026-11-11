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
| P5 | stabilna parametryka (trwałe nazewnictwo topologii) i komplet cech (loft, sweep, draft, mirror, split, wzór po ścieżce), auto-więzy, podpowiedzi osi/kąta | **zamknięty** (luki w długu technicznym) | [plan](superpowers/plans/2026-11-09-p5-parametric.md) |
| P3 | model hybrydowy: prymitywy + płaty → jedna zamknięta bryła, STEP round-trip | **zamknięty** | [plan](superpowers/plans/2026-11-09-p3-hybrid.md) |
| P4 | freeform organiczny: retopologia niestrukturalna (pola kierunków), łaty G1, sieć łat z krzywych cech | **zamknięty** (PR) | [plan](superpowers/plans/2026-11-09-p4-organic.md) |
| P6 | ergonomia GUI zgodna z rodziną ArtCraft | **w toku: G1–G5 gotowe** (G1 powłoka: `craft-ui` docking, tokeny motywu, ikony SVG, i18n en/pl; G2 formularze komend, podgląd na żywo, edytor historii; G3 narzędzia viewportu: zaznaczanie, wybór ścian/krawędzi B-rep, gizmo, kostka widoku, przekrój; G4 panel odchyłek; G5 edytor szkicu; pomiary i zrzuty w tabeli Pomiary, luki w długu technicznym); G6: kryterium 6 sprawdzone ręcznie przez właściciela 2026-11-09 (import, wyrównanie do osi świata, ICP, automatyczne kolorowanie regionów, szkic z wymiarem dodanym ręcznie, wyciągnięcie z wpisaną wartością, fillet, eksport STEP — działają; luki: wskazywanie punktów 3-2-1 i krawędzi filletu myszą → naprawione w #42, do ponownego sprawdzenia); automatyczny test klikami odwołany na życzenie właściciela | [plan](superpowers/plans/2026-11-09-p6-gui.md) |
| P7 | dystrybucja: `cargo xtask dist` (binarki + `SHA256SUMS.txt`), metadane produktu, `wasm-opt`, [docs/distribution.md](distribution.md) | wydania: `.github/workflows/release.yml` (tag `v*` → paczki Windows zip, Linux tar.gz, macOS dmg/.app.zip + wspólny `SHA256SUMS.txt`), pierwsze wydanie `v0.1.0`; nightly codziennie o 4:00 czasu warszawskiego (pre-release `nightly`), oficjalne wydania tylko ręcznie przez właściciela; decyzje właściciela 2026-11-09: wydania **niepodpisane** (brak certyfikatów), **bez** wpisu w `craft-launcher` | [plan](superpowers/plans/2026-11-09-production-readiness.md) |
| P8 | backlog ScanRuler, CAD Correct/Refit, 100 mln+ trójkątów, dług, upstream (25 pakietów, 3 fale) | **w toku: fala 1 scalona** — A1 (odchyłki v2), A2 (#51, lokalne ICP na regionie), B1 (#55, auto-więzy: styczność, symetria, równe odległości), C1 (łańcuchy przekroju → linie i łuki), C2 (#52, stos przekrojów, DXF/SVG), D1 (decymacja z mierzoną tolerancją), E1 (MCP: przewodniki, lista kompaktowa, batch, kolejka), F1 (CAD Correct/Refit), H5 (#53, operacje na regionach); U1 (#54, pin solidcore `4f2da85`); poza planem S1 (#57, stabilizacja bramki P3: `mesh.clean`). Fale 2 i 3 nierozpoczęte | [plan](superpowers/plans/2026-11-09-p8-backlog-refit-scale-debt.md) |

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
| P5: stabilne odwołania (kryterium 5): wyciągnięcie + kieszeń przelotowa/ślepa + 2 fillety + shell, 20 losowych edycji (ziarno stałe) | fillety zawsze na krawędziach pionowych w tych samych narożnikach (pozycja znormalizowana), shell otwiera górną ścianę; objętość po filletach zgodna z postacią zamkniętą (błąd względny 5e-10); numeracja rekordu zmieniła się pod filletem 2 razy | `gate_p5_stable_references_over_20_random_edits` |
| P5: objętości nowych cech | loft kwadrat 20 → 10, h = 10: 2333,333333 (dokł.); sweep kwadratu 2×2 po łuku R = 10, 90°: 62,831853 (Pappus); draft 5° bloku 20×20×10: 3660,251034 (dokł.); mirror 2000; split 1500 / 500 / połowa sześcianu 500; wzór po prostej 4 × π·16 | `crates/engine/tests/p5_features.rs` |
| P5: auto-więzy na zaszumionym wsporniku (σ = 0,05) + 2 walce | 12 równoległości, 24 prostopadłości, 1 współosiowość, 3 równe promienie; po przyciągnięciu błąd kątowy < 1e-6° (przed: 0,084°), długościowy < 1e-9 mm | `auto_constrain_snaps_noisy_primitives` |
| P5: podpowiedzi | oś walca R = 8: oś ±z, środek ±0,001, R = 8,0001; draft ściany po 5°: 5,000000° | `suggest_axis_from_a_scan`, `draft_block_volume_and_suggestion` |
| P1: chmury punktów (PLY/OBJ bez ścian, XYZ) → siatka: normalne PCA + orientacja MST, ball pivoting 3 promienie (2,5·, 5·, 10· rozstaw), dziury ≤ 24 krawędzi | sfera/pudełko σ = 0,02–0,05: 0 pętli brzegowych (σ = 0,05: ≤ 4), wierzchołki ≤ 3σ od powierzchni; walec otwarty: 2 obrzeża; wspornik z 33 k punktów (σ = 0,03): 6 płaszczyzn + walec + sfera, RMS ≤ 0,1, r = 6 ± 0,05; lokalny skan 1,6 mln punktów (poza repo): import + rekonstrukcja 14 s (release, normalne wielowątkowo), 3,2 mln trójkątów, 12 pętli brzegowych, 0,7 % punktów pominiętych; cały potok `local_corpus` przechodzi (10/10 prymitywów, ICP 1e-13 mm; segmentacja 257 s → P2) | `p1_closed_shapes_from_noisy_points_are_watertight`, `p1_bracket_from_points_segments_and_fits`, `p1_open_cylinder_keeps_its_rims`, `geom::recon::tests`, `hostile_bytes_never_panic` |
| P1: estymator szumu | bracket σ = 0,05 → 0,052 (było 0,078); skanowany box σ = 0,03 → 0,034 | `noise_estimate_is_robust_to_edges_and_density` |
| P2: skala (maszyna referencyjna: AMD Ryzen 5 5600, 6 rdzeni / 12 wątków, 32 GB RAM, RTX 5070, Windows 11; pomiar na maszynie współdzielonej z innymi buildami, więc czasy są raczej górne) | 1 M (0,93 M trójkątów): import 0,12 s, clean 0,21 s, segmentacja 4,6 s (28 regionów), ICP 0,28 s, odchyłka 0,27 s. 5 M (4,77 M): import 0,62 s, clean 1,0 s, segmentacja 23,6 s (901 regionów), ICP 1,8 s (100 it.), odchyłka 1,6 s. 10 M (10,12 M): import 1,5 s (cel < 15), clean 2,3 s, segmentacja 54,8 s (cel < 60; 2060 regionów, 17,3 k dopasowań ziaren, 28,6 mln iteracji LM), ICP 3,1 s (cel < 10), odchyłka 4,0 s | `perf_scan_scale` (`--ignored`, `SURFACECRAFT_PERF_SIZES=1,5,10`) |
| P2: viewport | skan 10 M trójkątów przez LOD (1,6 M narysowanych, przygotowanie 2,5 s jednorazowo): 508 klatek/s w renderze offscreen przy krążącej kamerze (cel ≥ 30), geometria wysłana raz; zmiana kolorów = jeden `write_buffer` | `fps_with_a_large_scan` (`--ignored`, pomija się bez GPU), `colour_change_does_not_reupload_geometry` |
| P2: korpusy lokalne (dane poza repo) | QS2026 (10 plików): segmentacja 5,8–25,7 s (przed P2: 7,8–41 s), 99/100 największych regionów w progu (C10: 9/10, nie zbadane czy to nie stan z main po P1/relax); Creality: Mesh.ply 484 k trójkątów 21 s, Miata34 2,5 M 137 s (było 330–360 s; 9958 regionów = limit 10 000, kształt organiczny: „10 największych w progu" 10/10, przypisane 69 %), slupek_NB 3 M 145 s (9848 regionów, 71 %); Optimization.ply (chmura bez ścianek) pominięty | `local_corpus_full_pipeline` |

| P3: część A — blok 40×30 z filletami r = 5 i falistym wierzchem (skan σ = 0,03; 5 płaszczyzn + 4 walce + płat) | 1 zamknięta bryła, 10 ścian jak CAD; V 12771,75 vs CAD 12787,05 (0,12 %; CAD = kwadratura z 6e-6); max odchyłka skanu od bryły 0,177 mm (rms 0,031; 0 wierzchołków > 0,2); STEP round-trip V 1,3e-9, 10 → 10 ścian. Od #57 (P8-S1) bramka biegnie na 4 ziarnach szumu (21, 8, 11, 12): max 0,169 / 0,130 / 0,180 / 0,126 mm (macOS; skan różni się liczbowo między platformami, więc każda para ziarno–platforma to osobne losowanie), płat wierzchu dopasowany z tolerancją 0,15 | `gate_p3_block_with_freeform_top` |
| P3: część B — wałek r = 12 z fazą 3 mm i otworem przelotowym r = 4 (2 płaszczyzny, walec, stożek, walec wklęsły) | 1 bryła, 5 ścian; V 11751,67 vs CAD 11752,69 (0,009 %); max odchyłka 0,128 mm (rms 0,030); STEP round-trip V 1,7e-7, 5 → 5 ścian | `gate_p3_chamfered_cylinder_with_hole` |
| P3: `replace_face` (płaski wierzch bloku → płat), `*.trim_to_region` (arkusz ściany bryły na płacie/prymitywie) | blok 6 ścian, V 13020 (40·30·mean h); każde wejście `from_regions` ma dokładnie 1 ścianę | `gate_p3_block_with_freeform_top` |
| P3b Z1+Z2: blok 60×50×20 z kieszenią 30×20 (naroża r = 4, fillet dna r = 1) i otworem Ø8 z dnem stożkowym 118° (skan σ = 0,03; 11 płaszczyzn, 4 walce naroży, 4 walce fillet dna, ściana otworu, stożek) | 1 zamknięta bryła, 25 ścian jak CAD; V 54709,85 vs CAD 54686,11 (0,043 %); max odchyłka skanu od bryły 0,160 mm (rms 0,031; 0 wierzchołków > 0,2); STEP round-trip V 2,8e-11, 25 → 25 ścian. Bez prymitywów fillet dna: 17 ścian jak CAD bez fillet, V 0,044 % od CAD bez fillet, max odchyłka 0,33 mm w pasie fillet (0,16 poza nim) | `gate_p3b_block_with_pocket_and_blind_hole` |
| P3b Z2: płyta 50×40×10 z oknem przelotowym (r = 3) i rowkiem otwartym do krawędzi (półokrągły koniec r = 4) | 18 ścian jak CAD; V 17359,33 vs CAD 17345,93 (0,077 %); max odchyłka 0,138 mm; STEP round-trip V 4e-16 | `p3b_plate_with_window_and_open_slot` |

| P3b/Z4: `replace_face` w górę i w dół (płat falisty przecina płaską ścianę, wzniesienie 2,5 mm) | 1 zamknięta bryła, 6 ścian; V 13018,6646 vs kwadratura 13018,6605 (4e-6 %); STEP round-trip V < 1e-6; kolizja odrzucona z komunikatem | `gate_p3b_replace_face_up_and_down` |
| P3b/Z3: płyta 60×40×8 z zaokrągleniami r = 5 (krawędzie pionowe + górne, narożniki = łaty sferyczne) i czopem Ø16 do z = 20 z wklęsłym fillet r = 2 u podstawy (torus) i fazą 1,5 mm na czole (stożek); skan σ = 0,03, regiony przez selekcję geometryczną (jak w P3 część B) | 1 zamknięta bryła, 22 ściany jak CAD, 0 pominięć; V 20385,64 vs CAD 20444,29 (0,29 %); max odchyłka skanu od bryły 0,152 mm; STEP round-trip V < 1e-6, 22 → 22 ściany | `gate_p3b_plate_with_boss` |

| P4: zamknięty kształt organiczny (kula r = 20 z gładkimi wybrzuszeniami ±4 mm, skan σ = 0,025, `scanify`, `mesh.clean`) | `quad.retopo` (pole orientacji + pozycji): 74 ściany z pola (52 quady, 15 % wierzchołków nieregularnych, χ = 2) → 276 łat bikubicznych; max odchyłka skanu od łat 0,122 mm (niezależnie względem teselacji kernela 0,124; próg 0,2), łaty od dokładnego kształtu 0,041 mm; skok normalnej przez krawędzie: max 3,1°, średnio 0,30°; sieć zamknięta → bryła 276 ścian, V 33808,31 vs dokładna 33812,44 (0,012 %), każda krawędź w 2 ścianach | `gate_p4_closed_organic_blob` |
| P4: otwarta powierzchnia zawinięta o 240° (nie mapa wysokości; `surface.fit` i `quad.create` odmawiają), skan σ = 0,025 | układ z pola: 80 ścian (78 quadów, 1 % nieregularnych) → 318 łat; max odchyłka 0,102 mm (99. percentyl niezależnie 0,077); skok normalnej max 0,29°, średnio 0,047°; 2 % punktów poza brzegiem układu (nie liczone) | `gate_p4_open_wrapped_patch` |
| P4: obudowa z ostrymi krawędziami (60×40×24, ściany wybrzuszone do 2,5 mm), skan σ = 0,025, `surface.from_curves` | 6 łat stopnia 5 na 12 wspólnych krzywych cech, bryła zamknięta; max odchyłka 0,103 mm (rms 0,026); V 67316,73 vs dokładna 67326,83 (0,015 %) | `gate_p4_feature_curve_housing` |
| P4: lokalny organiczny skan auta (2,5 mln trójkątów; 4 regiony — okolice skrajnych wierzchołków, promień 2,5 % i 4 % przekątnej, 12–38 tys. trójkątów; dane poza repo, tylko wartości zbiorcze) | układ z pola 434–482 quady, 3–10 % nieregularnych, χ = 1 (dysk), 0 odwróconych, retopo 0,6–1,8 s; łaty: mediana odległości wierzchołków skanu od łat 0,008–0,03 mm; max (bez punktów poza brzegiem układu, 3–7 %) 0,125 / 0,375 / 0,245 / 8,1 mm — 1 z 4 w progu 0,2; skok normalnej średnio 0,04–0,5°, w 2 regionach pojedyncze krawędzie z odwróconą normalną (179°) przy brzegu regionu | `local_organic_region_retopo` (`--ignored`, `SURFACECRAFT_LOCAL_CORPUS`) |
| P6: integracja G2–G5 + poprawki (#29 #31 #32 #33 #34 #35 #36; Windows, RTX 5070, profil `ci`) | `cargo fmt --check` OK; `cargo xtask quick --tests` (ui-egui, engine, kernel, geom, fit, surfacecraft: statyczne kontrole, ui-containers, clippy `-D warnings`, testy) OK; `cargo xtask wasm` OK; testy GPU (`SURFACECRAFT_GPU_TESTS=1`, `--lib`) 30/30, w tym 3 `viewport`; `fps_with_a_large_scan`, 10 M trójkątów: 3397 klatek/s offscreen, 1,63 s przygotowania LOD, 1 upload; `cargo xtask web` OK (bundle 65 MB bez `wasm-opt`), serwer lokalny: index, `.wasm`, `.js` — HTTP 200. Zrzuty: `docs/screenshots/p6-g1-*.png` (powłoka), `p6-deviation.png`, `p6-sketch.png`, `p6-form-extrude.png`, `p6-gizmo-viewcube.png`, `p6-lasso.png`, `p6-section.png`. Dock: Deviation we własnej kolumnie prawego docka (statystyki i histogram bez przewijania w oknie 1500×900) | `write_shell_screenshots` (`SURFACECRAFT_SHOTS=<katalog>`), `docs/integration.md` |
| P6: integracja #39 #42 (UX powłoki + wskazywanie w widoku; Windows, RTX 5070, profil `ci`, `main` bb7ec75) | `cargo xtask quick --tests surfacecraft-ui-egui` OK (fmt, assets, licences, layers, ui-containers, app-geometry, clippy `-D warnings`; 91 testów, w tym 7 z `field_pick`, 7 z `shell`, 16 `sketch_editor`); testy GPU (`SURFACECRAFT_GPU_TESTS=1 --lib`) 32/32; zrzuty `write_shell_screenshots` obejrzane: Deviation jest zakładką prawego docka, pasek szkicu u góry, pasek narzędzi widoku bez nakładania na kostkę widoku (także w oknie 980×760), lista komend zwija się i rozwija (`p6-form-extrude.png` zwinięta); reset układu: `layout_version` 1 → układ domyślny, motyw i język zachowane (test `the_viewport_is_protected_and_the_layout_survives_a_round_trip`); komunikat o geometrii docka: odtworzony przy oknie 0×0 (okna 800×600, 640×480, 400×300 bez błędu), naprawiony w `dock_area`; `RUSTC_WRAPPER= cargo xtask web` OK (bundle 65 MB bez `wasm-opt`, 8 min 20 s, bez wdrożenia). Nie sprawdzone: wskazywanie myszą w prawdziwym oknie i w przeglądarce (brak człowieka przy stanowisku) | `write_shell_screenshots`, `field_pick`, `shell`, `docs/integration.md` |
| P4: kernel — sieć łat Béziera (`nurbs::sheet_from_bezier_network`) | sześcian z 6 łat bikubicznych: 12 wspólnych krawędzi, powłoka zamknięta = bryła, V = 1000 (1e-6); łaty o niezgodnym brzegu odrzucone | `kernel::nurbs::network_tests` |
| P8-S1 (#57): `mesh.clean` na skanie z ostrymi krawędziami | wypełnianie dziur zachowuje krawędzie i narożniki (`geom::crease`): 200 dziur na krawędziach zaszumionej siatki → 195 wyostrzonych, 0 wierzchołków łaty > 0,12 mm od CAD, 0 łat gorszych od gładkiej (łata, która się plącze, wraca do gładkiej); dziura na narożniku sześcianu: najgorszy wierzchołek łaty 0,878 mm → < 1e-6 mm; sąsiadujące kolce nie maskują się nawzajem (`hidden_spike`); bramka P3 część A lokalnie na ziarnach 1–30: 28/30 (0,111–0,180 mm; przed zmianą 3 z 13 sprawdzonych padały: 0,217 / 0,273 mm / przeciek `region.grow` przez sfazowaną krawędź); część B 0,129 mm. Na pinie `4f2da85` (#54, macOS): część A 0,121 / 0,136 / 0,144 / 0,125 mm, część B 0,130 mm; `p1_scanned_filleted_block` 0,144 mm i `p1_scanned_chamfered_cylinder` 0,145 mm (próg 6σ = 0,18), wałek z fazą w progu na ziarnach 1–40. Dwie poprawki z #54: trójkąt łaty z samych wierzchołków brzegu, odwrócony już po wygładzeniu, nie blokuje wyostrzenia (było 0,199 mm); strona krawędzi jest powierzchnią 2. stopnia, gdy ta zostawia < 0,8 reszty płaszczyzny na stopień swobody i ma ≥ 20 punktów (stożek brany za płaszczyznę dawał krawędź 0,2 mm w głąb bryły) | `geom::crease::tests`, `neighbouring_spikes_both_go`, `grow_smooth_does_not_leak_over_a_sharp_edge_of_a_noisy_scan`, `gate_p3_block_with_freeform_top` |

## Decyzje buildera (z uzasadnieniem)

Szczegóły w planach milestone'ów.
- solidcore z git, pin `4f2da85` (P8-U1, #54; wcześniej `b9c02da`). Nowy mesher: ściany płaskie dostają minimalną liczbę trójkątów, `chord` to wartość bezwzględna w mm — testy, które brały teselację za „skan”, dogęszczają ją (`synth::refine`).
- Ciała B-rep w dokumencie jako rekord `Brep` (undo, zapis, writery czytają jedną formę).
- Format projektu: JSON + siatki base64 (wersjonowany).
- Viewport: offscreen + blit (niezależność od bufora głębi okna, zrzuty dla MCP/AI).
- `craft-libs`: nieużywane (brak potrzeby, dodatkowe zależności git).
- Solver nieliniowy (§10): własny Levenberg–Marquardt w `fit::lm` (≤ 7 parametrów; bez zależności).
- Auto-typ: najprostszy rodzaj z RMS ≤ 1,2 × najlepszego (+ odrzucanie wyników zdegenerowanych).
- Segmentacja: deterministyczny fit-and-grow na spójności meshu (wariant Efficient RANSAC).
- Fit do punktów rozproszonych (§10): resampling regionu do siatki (MLS nad płaszczyzną PCA) → `lst_sq_approx` w kernelu; `app-geometry` pusta.
- Arkusz z płata: rekord `Brep` z jedną ścianą `Spline` składany w `kernel::nurbs` (sprawdzany przez `validate` i `Body::from_record`).
- Retopologia (§10): M4 — strukturalny układ quad na mapie wysokości; P4 — natywny, napisany z publikacji układ z pól Instant Meshes (4-RoSy + 2-PoSy) dla regionów zawiniętych/zamkniętych (bez procesu zewnętrznego i bez kodu referencyjnego); AutoSurface strukturalny = jeden B-spline na blok rozłożony na łaty Béziera (G1 z konstrukcji), niestrukturalny = łata bikubiczna na quad.
- Solver więzów: własny (LM + analiza rzędu jakobianu), bez adaptacji `solidcraft-sketch`.
- MCP: własny JSON-RPC (bez `rmcp`), narzędzie na komendę; bridged przez kanał sterowania 127.0.0.1 + token.
- Asystent AI: pętla narzędzi niezależna od dostawcy; dostawca = program z `SURFACECRAFT_AI_COMMAND` (wybór usługi LLM czeka na użytkownika).
- Historia: lista cech z replay (DAG z zależności przez ostatniego pisarza ciała); undo przez migawki dokumentu.

## Dług techniczny

- **Adaptacja (dług):** `crates/io/src/step/{p21,mod}.rs` z `solidcraft-step` (zapis otwartych powłok STEP) — do wymiany na `step-export` solidcore, gdy zapisze `ShellModel` (PR upstream opcjonalny, tylko za zgodą użytkownika).
- STEP arkuszy bez PCURVE (krawędzie tylko krzywą 3D); importer solidcore odtwarza je bez strat w testach.
- **Adaptacja (dług):** `crates/io/src/iges.rs` z `iges_out` SolidCrafta — do wymiany, jeśli writer IGES trafi do solidcore (propozycja upstream wymaga zgody użytkownika: działanie poza repo).
- Szkice w viewporcie przeliczane co klatkę (małe; do cache, gdy urosną).
- Web: bundle 45 MB po `wasm-opt -Oz` (binaryen przypięty w `deploy-web.yml`, #40; wersja z apt psuła tablicę externref i strona zostawała na ekranie ładowania), limit 5 mln trójkątów; komunikat „Panel layout change not applied: docking geometry is not finite and bounded” przy pierwszym starcie w przeglądarce miał jedną przyczynę: pierwsza klatka przy płótnie jeszcze bez rozmiaru (0×0); `dock_area` nie rysuje teraz docków, dopóki rozmiar nie jest skończony i dodatni (test `the_dock_never_reports_an_error_in_a_small_or_unsized_window`); brak MCP/kanału sterowania i asystenta bez dostawcy po stronie przeglądarki.
- P7: wydania niepodpisane (SmartScreen/Gatekeeper ostrzegają; podpis wymaga certyfikatów właściciela), brak MSI/AppImage i macOS Intel; launcher nie zainstalowałby tych paczek (Windows: brak VERSIONINFO w binarkach; macOS: wymaga Developer ID) — bez znaczenia, dopóki nie ma wpisu w `craft-launcher` ([docs/distribution.md](distribution.md)). Web: pliki czytane jednym buforem (`rfd` `read`); streaming pominięty — wymaga zmian w `io` i pętli wejścia UI.
- Brak osobnego repo `surfacecraft-corpus`: `tangency-labs/models` nie ma jeszcze modeli (2026-11-08), więc korpus
  poziomu 1 w CI to części CAD z kernela „skanowane” w teście (`synth::scanify`); gdy pojawią się modele CC0, dołożyć je.
- Gizmo manipulatora w viewporcie (§6.1) — w M1 ruch ręczny tylko przez `mesh.transform`.
- Regiony: resegment/insert/separate/enlarge/shrink są (P8-H5, #53); po enlarge/insert rodzaj i RMS regionu nie są przeliczane (zostają sprzed zmiany). `region.resegment` odmawia i zostawia regiony, gdy segmentacja nic nie znajdzie.
- Segmentacja skanów organicznych (Miata34, slupek: ~10 000 regionów, limit) trwa 140 s na 2,5–3 mln trójkątów; wzrost regionów i refity są sekwencyjne (równolegle tylko ziarna, rodzaje prymitywów, RMS, odchyłki). 10 M syntetycznych: 55 s przy celu 60 s — mały zapas.
- Import STL 10 M: `weld` sortuje 30 mln kluczy (~1 GB chwilowo); parsowanie binarnego STL jest jednowątkowe. Out-of-core (design doc §9) nadal bez implementacji; `PointSource`/`TriangleSource` to tylko szew.
- Pomiary P2 robione na maszynie współdzielonej z innymi buildami (OOM rustc przy równoległych kompilacjach solidcore); fps tylko offscreen, bez egui.
- Fragmentacja pierścieni: `merge_adjacent` dochodzi teraz do punktu stałego (≤ 64 przebiegów, odrzucone pary nie są fitowane ponownie); w teście P3 (walec z fazą i otworem) obie płaszczyzny pierścieniowe wychodzą z `auto_segment` w całości, więc ręczny wybór trójkątów z testu usunięto. Skan punktowy wspornika daje dodatkowy okruch walca (27 trójkątów) — test odsiewa regiony ≤ 100 trójkątów.
- Edycja siatki: decymacja QEM i wypełnianie dziur bez równoległości; dziury > 4000 krawędzi pomijane.
- Fit Surface: jeden płat na region wysokościowy nad płaszczyzną (region zawinięty → odmowa, trzeba podzielić); brzeg płata = prostokąt w parametrach (bez przycinania do konturu regionu).
- P4 (retopologia): hierarchia z decymacji QEM zamiast grafu Instant Meshes, jeden wątek, ekstrakcja przez sąsiedztwo komórek kraty (bez śledzenia krawędzi); przy osobliwościach 20–30 % ścian to trójkąty/pięciokąty → podział 4→1 całego układu (4× łat); gdy pole nie daje czystej siatki lub zostają odwrócone quady — układ zapasowy (decymacja → 3 quady na trójkąt), raportowany. Decymacja QEM jest wolna na idealnie płaskich siatkach (kilkanaście sekund przy 14 tys. trójkątów). Brak wyrównania do ostrych krawędzi w polu (to robi `surface.from_curves`).
- P4 (G1): łaty mają wspólne krzywe brzegowe i wspólne płaszczyzny styczne w wierzchołkach; wzdłuż krawędzi G1 jest przybliżone (skok mierzony, max 3° na blobie). Dokładne G1 przy dowolnej walencji = konstrukcja „vertex enclosure” (constrained fit krawędzi) — kandydat na prymityw kernela/upstream solidcore (za zgodą użytkownika). Do tego czasu `crates/fit/src/{patch,features}.rs` liczą ewaluację i dopasowanie łat Béziera po stronie aplikacji (wpisy w `xtask/app-geometry.txt` z markerem `APP-GEOMETRY:`; B-rep powstaje w kernelu).
- P4 (realne skany): na regionach rzeczywistego skanu wnętrze układu dopasowuje się do szumu skanera (mediana 0,01–0,03 mm), ale brzeg układu nie nadąża za postrzępionym brzegiem regionu (3–7 % punktów poza układem, pojedyncze łaty brzegowe z odwróconą normalną i odchyłką do 8 mm w jednym z 4 regionów); do poprawy: układ dopasowany do konturu regionu (wygładzony brzeg, wymuszone rogi) i kontrola odwróconych łat po podziale.
- P4 (`surface.from_curves`): obsługuje regiony z dokładnie 4 rogami (inne pomijane z powodem); rogi = wierzchołki styku ≥ 3 regionów; brak ciągłości stycznych na ostrych krawędziach z założenia.
- P4: `thicken` otwartej sieci wielołatowej nadal zależy od solidcore (offset *partial*); sieć zamknięta daje bryłę wprost (`surface.auto`/`surface.from_curves` tworzą `body`).
- solidcore: `thicken` odmawia na wielościennym arkuszu (offset *partial*) — `surface.thicken` używa jednościennego arkusza-rodzica; bloki z przełamaniami pogrubia się osobno. Kandydat do zgłoszenia upstream (za zgodą).
- Nazewnictwo topologii (P5) to sygnatury geometryczne z rekordu (`kernel::naming`), nie `provenance` solidcore (atrybuty nie przeżywają round-tripu rekordu). Ograniczenia: dwie krawędzie o tej samej pozycji, kierunku i sąsiadach są nieodróżnialne (wybierana pierwsza); odwołanie przechodzi, gdy najlepszy wynik ≤ 1,5, więc radykalna zmiana geometrii (np. krawędź zniknęła) kończy cechę błędem.
- solidcore: unia ≥ 3 obróconych kopii (dotyczy też starego `feature.pattern` z obrotem) oraz zdekodowanie zapisanego rekordu po `shell` filletowanej bryły z kieszenią (1 z 20 kroków bramki P5) kończy się `from_record: SM_ERR_INVALID_INPUT` — bryła jest zapisana i eksportowalna wynikiem operacji, ale kernel nie odczyta jej z rekordu (np. objętość = brak). Do zgłoszenia upstream (za zgodą).
- Cechy P5: `sweep` wymaga profilu położonego na początku ścieżki i ścieżki z linii/łuków (ścieżka ze splajnu — brak); `loft` bierze zewnętrzny profil każdego szkicu (bez krzywych prowadzących i dopasowania punktów startowych); `split` zostawia jedną stronę (drugą uzyskuje się na kopii); wzór po ścieżce dotyczy łańcucha linii i łuków (krzywa dowolna — brak); `draft` odmawia ścian, których kernel nie umie pochylić (nie-płaszczyzna/nie-stożek).
- Auto-więzy (§6.2): relacje równoległość, prostopadłość, współosiowość, równe promienie (płaszczyzna, walec, stożek, torus, kula) oraz od P8-B1 (#55) styczność, symetria i równe odległości.
- **Asystent AI bez skonfigurowanego dostawcy LLM:** działa z dowolnym programem-dostawcą, ale żadna usługa nie jest wybrana ani podpięta (decyzja użytkownika: dostawca i klucz API).
- P3 (`kernel::hybrid`): obsługiwane cięcia to płaszczyzna, wypukły walec styczny do dwóch płaszczyzn (fillet), walec/stożek/sfera/torus
  obejmujące oś w całości, płat swobodny; `replace_face` (P3b/Z4) dodaje i odcina materiał (ściana wyciągnięta o wzniesienie płata, suma, cięcie;
  odmawia, gdy wyciągnięcie koliduje z bryłą; obrys tylko z odcinków i łuków, ściana bez otworów).
- P3b (Z1+Z2, `engine::cmd::plan`, `kernel::local`): planer rozróżnia cięcia globalne (przed płaszczyzną < 0,5 % skanu) i lokalne;
  lokalne tworzą narzędzia odejmowane — otwór nieprzelotowy (walec + dno płaskie lub stożek wiertła, obrót profilu), kieszeń
  (podłoga + ścianki/naroża → profil, wyciągnięcie, opcjonalny fillet dna r z walców stycznych do dna), wybranie przelotowe i rowek
  otwarty (profil zamykany poza bryłą). `body.plan_regions` zwraca rolę i powód każdego regionu, narzędzia i graf styków
  (wypukły/wklęsły/styczny) — ta sama logika co `body.from_regions`. Ograniczenia: profil z kątem wklęsłym (reflex), dwa łuki obok
  siebie i kieszeń z wyspą są raportowane jako pominięte; ściany kieszeni, otworów i slotów z wyciągnięcia/obrotu wracają z kernela
  jako rational spline (rozpoznawane po wierzchołkach, nie jako walec/stożek analityczny); kierunek narzędzia jest przyciągany do osi
  modelu (≤ 0,5°) — narzędzie o kierunku dokładnie równym normalnej dopasowanej płaszczyzny bryły wywala boolean w solidcore
  (`from_record` odrzuca wynik, status SM_ERR_INVALID_INPUT) — do reprodukcji w Z5. Kolejne regiony płaszczyzny podzielonej przez
  segmentację są scalane po równości płaszczyzny (≤ 0,25 mm, normalne ≤ 3,6°) w planerze.
- P3b Z3 (`kernel::blends`, `kernel::local::LocalAdd`): częściowe kształty to zaokrąglenia, fazy i bryły lokalne. Krawędź zaokrąglana jest
  znajdowana w topologii bryły (dwie ściany płaskie równoległe do osi walca, wypukła lub wklęsła; krawędź kołowa między płaszczyzną i
  współosiowym walcem dla torusa i stożka), zaokrąglenia jednego promienia idą **jednym wywołaniem** `blend` (kernel robi łatę narożną =
  `BallPatch`, sfera dostaje status „corner patch”), faza stożka przez `chamfer_sides` (odległości z kąta połówkowego). Czop (wypukły pełny
  walec z płaszczyzną-czołem i płaszczyzną-podstawą) = `LocalAdd::Boss` (walec zagłębiony 0,5 mm, suma), jego płaszczyzna podstawy zostaje
  globalna (skan przed nią wyjaśnia czop); kopułka (sfera niestyczna do niczego, stojąca na płaszczyźnie globalnej) = `LocalAdd::Dome`
  (kula ∩ półprzestrzeń, suma). Kolejność: płaszczyzny → zaokrąglenia płaszczyzna–płaszczyzna → obroty globalne → odjęcia lokalne → dodania
  lokalne → pozostałe zaokrąglenia/fazy (od największego promienia) → płaty. Globalne płaszczyzny ≤ `square_planes_deg` (domyślnie 0,3°, 0 wyłącza) od osi modelu są prostowane, o ile przesunięcie nad regionem ≤ max(2σ, 0,05 mm) (raport `squared_planes` w `body.plan_regions`)
  (inaczej okrąg przecięcia z czopem jest elipsą i kernel nie widzi krawędzi kołowej). Pominięte (raportowane z powodem): stopnie i żebra
  (wypukły płat nad częścią ściany bez walca-czopa), wklęsła częściowa sfera (dołek), częściowy stożek/torus bez płaszczyzny i walca współosiowego.
- **Blokery solidcore (b9c02da), do zgłoszenia upstream za zgodą:** (1) bryła ze ścianą stożkową zapisaną jako rekord `Cone`
  z ujemnym półkątem (zwężający się wzdłuż z; np. `make_cone([0;3], 5, 1, 10)`) po `step_export` + `step_brep::import` rozpada się na
  osobne „bryły” po jednej ścianie (dodatni półkąt i stożek ostry 5→0 działają) — obejście: narzędzie stożkowe budowane
  rozszerzające się wzdłuż z (z dyskiem 0,01 mm przy wierzchołku); (2) fillet zwraca walec jako `Cone` o półkącie 0 (rozpoznawane
  w `faces_on_shape`); (3) przecięcie dwóch prawie współosiowych powierzchni obrotowych (osie różne o szum fitu) kończy się
  status 1001 — obejście aplikacji: `snap_axes` (auto-więz współosiowości, ≤ 3° / 0,5 mm).
  Reprodukcje (1–3): `crates/kernel/tests/upstream_repro.rs` (`#[ignore]`); szkice zgłoszeń: `docs/upstream/solidcore-p3.md`.
- `region.auto_segment` rozbija płaskie pierścienie (wałek z otworem: dwie płaszczyzny → ~85 kawałków po 30–280 trójkątów) —
  w teście P3 płaszczyzny wybrane geometrycznie (`region.create`); do poprawy w segmentacji (P1/P4).
- P6/G1: `craft-ui` przypięty do `acf6f79` (ostatni rev bez forka egui). Nowsze rewizje (`fonts`, `localized`, IME) wymagają `[patch.crates-io]` na `storytold/egui` i `storytold/winit` (tak robi CADCraft) — zmiana stosu UI całej aplikacji, osobna decyzja; do tego czasu czcionki to domyślne `egui` (`default_fonts`), bez `craft_ui::fonts`. `craft-dialogs` (rev `1a11a48`) nie zależy od `craft-ui`. `rfd` zostaje tylko w buildzie web (wybór plików w przeglądarce; desktop używa `craft-dialogs`). Brak własnego paska tytułu (`craft-window` tylko decyduje o ramce; przy `Frame::App` zostaje ramka systemu). Nazwy grup menu komend i etykiety komend pochodzą z silnika (po angielsku; grupy mają klucze `group.*`, etykiety komend jeszcze nie). Panel Właściwości nadal z edytorem JSON (G2).
- P6/G2–G5 (zintegrowane w `integration/2026-11-09`): brak przeciągania punktów w szkicu myszą (punkty przesuwa się wymiarami / `sketch.edit_constraint`); przekrój płaszczyzną osiową bez „czapki” (widać tylko kontur przekroju, nie wypełnienie); pędzel i lasso sprawdzają wszystkie trójkąty (brute force; wolne od ~1 mln trójkątów); podgląd na żywo (`Session::preview`) wyłączony dla ciężkich poleceń wg statycznej listy (bez wykrywania po czasie); brak znacznika cofania (roll-back) w historii; nazwy parametrów formularzy po angielsku (z JSON Schema), tłumaczone są tylko etykiety UI; przyciski paska narzędzi i szkicu są tekstowe (brak ikon SVG); pływający pasek narzędzi widoku i panel szkicu zasłaniają lewy górny róg viewportu (przy wąskim viewporcie także kostkę widoku); panel Właściwości: lista komend zajmuje górę, formularz wymaga przewijania. Tryb szkicu ma pierwszeństwo przed narzędziami viewportu (Esc anuluje najbardziej wewnętrzny gest, Alt+przeciąganie obraca kamerę wszędzie); w trybie szkicu nie ma kostki widoku ani paska narzędzi.
- P6/wskazywanie pól myszą (`fieldpick.rs`): przycisk „Wskaż w widoku” przy polach `primary`/`secondary`/`tertiary` (3-2-1: 3 punkty → płaszczyzna, 2 → oś, 1 → punkt, kolejno), przy listach punktów (`from`, `to`, `points`) i przy `edges` (fillet, chamfer); kliknięcie w widoku przełącza krawędź (ponowne kliknięcie usuwa), Esc lub prawy przycisk kończy. Ograniczenia: pola rozpoznawane po nazwie (nie ma jeszcze znacznika w schemacie komendy), wskazywanie ścian (`faces`) i trójkątów dalej przez narzędzia viewportu.
- P8-S1 (#57), nie zmierzone: `local_corpus_full_pipeline` i `perf_scan_scale` po zmianie `mesh.clean` (prawdziwe skany, 10 mln trójkątów — tylko maszyna referencyjna); czas na dziurę po dodaniu indeksu sąsiedztwa (przegląd pierwszej wersji zmierzył 3× wolniej); reguła `hidden_spike` na siatkach z cienkimi trójkątami. `crates/geom/src/crease.rs` ma ponad 1000 linii z testami (limit 800 — do podziału).
- Bramka P3 część A: ziarna 13 i 29 padają z przyczyn poza `mesh.clean` — 13: segmentacja typuje zgrubną ścianę +x jako sferę; 29: asercja po dopasowaniu płata wierzchu (przyczyna niezbadana); `p1_scanned_chamfered_cylinder` na ziarnach 27, 28, 31 pada na rodzajach regionów z segmentacji (test używa ziarna 5). Siatka z kernela różni się między macOS a Linux/Windows (inna liczba trójkątów), więc wynik lokalny na macOS nie przesądza o CI. `surface.fit` przy tolerancji 0,15 potrafi zwrócić max 0,19 bez dogęszczenia siatki.
- Uwagi z przeglądów P8 fali 1, nie naprawione: `align.icp` na regionie liczy RMS dodatkowym przebiegiem (#51); DXF (AC1015) nie sprawdzony w zewnętrznym programie, limit punktów arkusza sprawdzany po zbudowaniu warstw (#52).
- Kanał sterowania zapisuje token w pliku tymczasowym (dostęp lokalnego użytkownika); jedna instancja aplikacji naraz.

## Luki / ryzyka

- CI nie ma GPU: test renderu offscreen pomija się na runnerach (działa lokalnie).
- Zależność git-only solidcore — pin trzeba okresowo podbijać i powtarzać smoke test.

## Backlog z analizy ScanRuler (2026-11-09)

Źródło: [`2026-11-09-scanruler-functional-analysis.md`](superpowers/specs/2026-11-09-scanruler-functional-analysis.md) §6.3.
ScanRuler jest na AGPL-3.0 — każdy punkt to wyłącznie reimplementacja clean-room na podstawie opisu zachowania, bez kodu.

1. **Odchyłki:** mapa ze znakiem zgodna z orientacją powierzchni, zakres kolorów z percentyli, histogram i pasma,
   regiony hotspotów / błędu objętości, lokalne dokładne dopasowanie zaznaczonego obszaru, wyrównanie z par punktów
   jako fallback, odchyłka względem pojedynczego dopasowanego prymitywu.
2. **Bryła z powierzchni + Auto Constrain:** zamykanie zbioru dopasowanych powierzchni w bryłę oraz przyciąganie
   prymitywów do relacji (równoległe / prostopadłe / współosiowe) — pokrywa się z długiem §6.2 powyżej.
3. **Szkice z przekrojów:** dopasowanie całych łańcuchów przekroju jako połączonych linii i łuków z wywnioskowanymi
   relacjami, stosy przekrojów raportujące pochylenie (draft/taper), eksport arkusza przekrojów do DXF/SVG.
4. **Siatka:** decymacja z mierzoną i weryfikowaną tolerancją (oba kierunki) oraz edycja nieniszcząca na kopii
   z mapą odległości od oryginału.
5. **Ergonomia agenta i trwałe referencje:** przewodniki serwowane przez aplikację, kompaktowa/filtrowana lista
   narzędzi (dziś 104+), kolejka zadań, komenda kontroli skanu, raport ciało-vs-skan per ściana, wybór ścian/krawędzi
   po odcisku geometrycznym zamiast indeksu pozycyjnego (dług powyżej).

Do decyzji użytkownika: czy SurfaceCraft dostaje warstwę inspekcji (wymiary, GD&T, grubość ścianek, pomiar 2D).

## Następny krok

Wszystkie milestones M0–M6 są zamknięte. Do decyzji użytkownika:
1. Dostawca LLM i klucz API dla asystenta (dziś: dowolny program przez `SURFACECRAFT_AI_COMMAND`).
2. Upstream (za zgodą użytkownika, zgodnie z zasadami solidcore: najpierw issue z reprodukcją): zgłoszone
   tangency-labs/solidcore#3228 („sheet from surface”, `face_on` 1001), #3229 (thicken arkusza wielościennego → `NotSolid`),
   #3230 (propozycja writera IGES nad rekordem). Zapis STEP otwartych powłok jest już przenoszony upstream (#2579) —
   bez dublowania. Reprodukcja: gałąź `repro/surfacecraft-sheets` w forku `d0pawlus/solidcore`. Po decyzji maintainerów:
   PR-y lub wymiana adaptacji. Wpis w `craft-launcher` — właściciel zdecydował: nie teraz.
Plan dojścia do pełnoprawnego programu: [`docs/superpowers/plans/2026-11-09-production-readiness.md`](superpowers/plans/2026-11-09-production-readiness.md)
P1–P7 wykonane (P7 w zakresie zgód właściciela: wydania niepodpisane, bez launchera). Pozostaje: ponowne ręczne
sprawdzenie wskazywania 3-2-1 i krawędzi filletu (#42) oraz zbiorczy przebieg sprawdzeń lokalnych z #39 i #42
(`docs/integration.md`).
Kandydaci post-MVP z design doc: retopologia niestrukturalna (Instant Meshes), druga strategia freeform (sieć łat
z krzywych cech), CAD Correct/Refit, trwałe nazewnictwo topologii w cechach, out-of-core/LOD dla 100 mln+ trójkątów.
Etap P8 (backlog ScanRuler, CAD Correct/Refit, 100 mln+ trójkątów, dług, zgłoszenia upstream): [plan](superpowers/plans/2026-11-09-p8-backlog-refit-scale-debt.md) — w toku.
Stan na 2026-11-11: fala 1 scalona w całości (U1 = #54). Kolejność: (1) przebieg integracyjny na maszynie
referencyjnej po #57 i #54 (`local_corpus_full_pipeline`, `perf_scan_scale`, `docs/integration.md`); (2) fala 2: A3, B2,
D2, E2, G1, H1, H3, H4, H6 oraz podział `crease.rs`; (3) fala 3 (H8 odblokowane przez U1).
