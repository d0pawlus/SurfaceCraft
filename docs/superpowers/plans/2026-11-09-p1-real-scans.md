# P1 — Realne skany: pomiar i naprawa (plan szczegółowy)

> Etap P1 planu [production-readiness](2026-11-09-production-readiness.md). Gałąź `p1-real-scans`.

## Stan wyjściowy (pomiar przed zmianami)
Pełny potok na lokalnym korpusie QS (10 plików, 99 k – 651 k trójkątów): import, szum, auto-segmentacja,
prymitywy, ICP — 10/10 bez błędu. Problemy: brak komend czyszczenia siatki; segmentacja alokowała tablicę
`nt` flag na każde ziarno (651 k trójkątów → 46 s); 300–600 regionów (nadsegmentacja); estymator szumu
zawyżał na krawędziach (bracket σ = 0,05 → 0,078); dopasowanie freeform testowane na regionach
niebędących mapą wysokości.

## Zadania
1. Test `#[ignore]` `local_corpus_full_pipeline` (`crates/engine/tests/local_corpus.rs`): import → clean
   (z wypełnianiem dziur) → szum → segmentacja → prymitywy (10 największych, próg max(0,1 mm, 2σ)) →
   freeform na największym nieprzypisanym kawałku → ICP kopii przesuniętej o znany ruch. Pliki raportowane
   numerem (C01…), do STATUS trafiają tylko zbiorcze liczby.
2. Edycja siatki w `geom::edit` + komendy `mesh.clean`, `mesh.remove_outliers`, `mesh.remove_small`,
   `mesh.fill_holes` (płaskie / gładkie: zagęszczenie + cienka płyta CG), `mesh.decimate` (QEM z płaszczyznami
   wiązań dla brzegu i ostrych krawędzi), `mesh.smooth` (Taubin). Edycja zmieniająca trójkąty usuwa regiony
   siatki (`Document::replace_mesh_topology`). Wrogie parametry: `hostile_params_never_panic` obejmuje je
   automatycznie (rejestr).
3. Korpus poziomu 1 w CI: `tangency-labs/models` nie zawiera jeszcze modeli (sprawdzone 2026-11-08), więc
   zamiast osobnego repo części CAD powstają w teście z kernela, są tesselowane i „skanowane”
   (`synth::scanify`: zagęszczenie z rampą gęstości ×2, σ = 0,03 mm, dziury, kolce, śmieci). Testy:
   `crates/engine/tests/scan_corpus.rs` (blok z zaokrągleniami r = 5, wałek z fazą 3 mm).
4. Segmentacja: ziarna o stałym polu (mediana pola × `seed_size`), dopasowania na ≤ 4000 punktach (RMS
   końcowy po wszystkich), wspólne znaczniki z numerem generacji, scalanie sąsiednich regionów tego samego
   rodzaju, gdy suma mieści się w tolerancji.
5. Estymator szumu: próbki z ostrym sąsiedztwem (normalne sąsiadów > 30° od lokalnej płaszczyzny) pomijane,
   gdy zostaje dość gładkich.

## Bramka
Kryterium 1 (10/10 plików korpusu lokalnego przechodzi cały potok) i 2 (prymitywy w progu; freeform raportowany
z rozróżnieniem regionów zawiniętych) + testy CI na korpusie generowanym (`p1_*`). Wynik w `docs/STATUS.md`.
