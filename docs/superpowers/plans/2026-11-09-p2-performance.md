# P2 — Wydajność i skala do 10 mln trójkątów (plan szczegółowy)

> Etap P2 planu [production-readiness](2026-11-09-production-readiness.md). Gałęzie `p2-perf` (część 1)
> i `p2-perf-2` (część 2).

## Stan wyjściowy
Skan 0,93 mln trójkątów: import 0,2 s, clean 0,4 s, segmentacja 7 s po części 1 (przed nią: 330–360 s dla
2,5–3 mln trójkątów z korpusu Creality, ~10 000 regionów), ICP 0,5 s, odchyłka 0,5 s. Wyświetlanie: cały
skan trafiał do GPU jednym buforem z kolorami splecionymi z geometrią; zmiana nakładki odchyłek nie
zmieniała klucza bufora (kolory mogły w ogóle nie dotrzeć do GPU).

## Zadania
1. Pomiar `crates/engine/tests/perf.rs` (1/5/10 mln, `--ignored`, release): czasy etapów oraz liczniki
   pracy (`surfacecraft_fit::stats`: dopasowania ziaren, refity przy wzroście, dopasowania końcowe, uruchomienia
   i iteracje LM).
2. Równoległość tam, gdzie pomiar ją uzasadnił, zawsze deterministyczna i zgodna z wasm (`geom::par`: rayon
   poza wasm, pętle na wasm): `Mesh::weld` oparty na sortowaniu zamiast HashMap, usuwanie odstających
   (`edit::remove_outliers`).
3. LOD wyświetlania (`engine::display`): siatki powyżej `LOD_BUDGET` (2 mln trójkątów) rysowane przez
   `geom::lod` (skupianie wierzchołków), z pamięcią podręczną po wskaźniku siatki i budżecie. Kolory
   (nakładki odchyłek, regiony) mapowane `Lod::map_colors`, zaznaczenia `Lod::map_triangles`. Wskazywanie
   w UI używa pełnej siatki (`DisplayItem::pick_mesh`, BVH po `pick_key`), więc zwraca oryginalne numery
   trójkątów.
4. Viewport: strumień kolorów oddzielony od geometrii (dwa bufory wierzchołków), zmiana kolorów to jedno
   `queue.write_buffer`; klucz GPU = klucz geometrii, tożsamość kolorów (`colors_key`) liczona w
   `TessCache`, więc zmieniona nakładka zawsze dociera do GPU. Zaznaczenie rysowane osobnym buforem
   indeksów i potokiem z jednym kolorem na wywołanie (bez duplikowania wierzchołków).
5. Pomiar klatek: `viewport::bench_offscreen` i `crates/ui-egui/tests/perf_fps.rs` (`--ignored`, pomija się
   bez GPU): kamera krąży przez 60 klatek, każda czekana do końca na GPU.
6. Dostęp do siatki pod przyszłe out-of-core (§9): `geom::TriangleSource` obok `PointSource`, użyty w
   LOD (`source_bbox`, `source_area`); bez implementacji out-of-core.
7. Korpusy lokalne (`SURFACECRAFT_LOCAL_CORPUS`, dane poza repo): ponowny przebieg QS2026 i CrealityScan.

## Bramka (kryterium 3)
Import 10 mln trójkątów < 15 s, auto-segmentacja < 60 s, ICP < 10 s, viewport ≥ 30 fps. Wyniki z liczbami
i maszyną referencyjną w `docs/STATUS.md`. Progów §11.1 nie luzujemy.
