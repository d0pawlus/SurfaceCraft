# P9 — uzupełnienie braków względem Geomagic Design X, QuickSurface i ScanRuler (plan)

> Zlecenie właściciela z 2026-11-12. Źródło braków: [porównanie](../../comparison.md) (stan `main` 979ec54, po #75).
> Zasada clean-room bez zmian: funkcje z opisu zachowania i literatury, bez kodu, tekstów, ikon i układów
> Geomagic, QuickSurface i ScanRuler (AGPL), bez uruchamiania narzędzi ScanRuler.
> Plan nie powtarza fali 3 z [P8](2026-11-09-p8-backlog-refit-scale-debt.md) (G2, H2, H7–H11): te pakiety są tu
> wpisane jako zależności pod swoimi identyfikatorami.

## Kolejność etapów

| Etap | Cel | Bramka zamknięcia | Szacunek (praca agentów + przegląd) |
|---|---|---|---|
| **O** | Części organiczne do bryły na prawdziwych skanach | 13 z 13 skanów lokalnego korpusu: ≥ 99 % punktów w 0,2 mm, bryła zamknięta tam, gdzie skan jest zamknięty, 0 błędów jądra; skok normalnej poza krawędziami ostrymi średnio < 1° | 4–6 dni |
| **V** | Dowód na prawdziwych danych i w GUI | przebieg integracyjny na maszynie referencyjnej zapisany w STATUS; ręczne przejście skan → STEP w oknie i w przeglądarce bez błędu blokującego | 1 dzień + czas właściciela |
| **S** | Skala 100 mln+ trójkątów | 100 mln syntetycznych: import, widok ≥ 30 klatek/s, odchyłka i ICP bez wczytania całości do pamięci (szczyt < 8 GB) | 3–4 dni |
| **M** | Wiele skanów i auto-wyrównanie | 4 częściowe skany części syntetycznej (σ = 0,03, pokrycie 30 %) → jedna siatka, błąd pozy < 0,02 mm, 0 dziur na szwach | 3–4 dni |
| **C** | Kompletność CAD | każda nowa cecha: objętość zgodna z postacią zamkniętą, przeżywa 20 losowych edycji (jak bramka P5) | 5–7 dni |
| **R** | Gotowość produktu | podpisane paczki instalują się bez ostrzeżeń; czytnik ekranu odczytuje wstążkę i formularz; asystent wykonuje tryb align z prawdziwym dostawcą | 2–3 dni + zasoby właściciela |
| **I** | Warstwa inspekcji (wymaga decyzji D1) | kostka wzorcowa ze znanymi odchyłkami: wymiary ± 1 µm, płaskość i walcowość strefą minimalną zgodne z wartością analityczną, raport PASS/FAIL | 5–7 dni |
| **X** | Przekazanie drzewa cech do zewnętrznego CAD (wymaga decyzji D2) | model z bramki P5 odtworzony z wygenerowanego skryptu w docelowym CAD, objętość w 1e-6 | 3–4 dni |

Etapy O i V idą pierwsze i po kolei. S, M i C są od siebie niezależne i mogą biec równolegle (limit 3 kompilujących
agentów, [integration.md](../../integration.md)). R, I, X po decyzjach właściciela.

## Decyzje właściciela (blokują wskazane pakiety)

| Id | Decyzja | Blokuje | Domyślnie, gdy brak decyzji |
|---|---|---|---|
| D1 | Czy SurfaceCraft dostaje warstwę inspekcji (wymiary, GD&T, grubość ścianek, pomiar 2D)? Design doc §2 dziś ją wyklucza | etap I | etap I nie startuje |
| D2 | Czy robimy eksport drzewa cech jako skrypt dla zewnętrznego CAD? Design doc §3: push-do-CAD jest nie-celem | etap X | etap X nie startuje |
| D3 | Certyfikaty do podpisu (Windows, Apple Developer ID) | R1 | wydania zostają niepodpisane |
| D4 | Dostawca LLM i klucz dla asystenta | R2 | asystent zostaje z programem-dostawcą |
| D5 | Zgoda na zgłoszenia upstream do `tangency-labs/solidcraft` (healer ponadliniowy, `item budget`) | O3 (część upstream) | zostaje obejście po stronie aplikacji |
| D6 | Prawdziwe skany części pryzmatycznych do lokalnego korpusu oraz czy właściciel przepuści te same skany przez Geomagic i QuickSurface (tylko liczby zbiorcze trafiają do repo) | V2, V3 | próg wyboru metody zostaje sprawdzony tylko syntetycznie |

Zostają nie-celami (bez pakietu): sterowanie skanerem, formaty zamknięte (Parasolid, JT, natywne pliki CAD).

## Zasady wykonania

Jak w P8: jeden pakiet = jeden subagent we własnym worktree, gałąź `p9-<id>` z `origin/main`, jeden PR; agent tylko
`cargo xtask quick <pakiety>`; STATUS aktualizuje orkiestrator; każda funkcja to komenda `area.verb` ze schematem i
testem bramkowym na danych syntetycznych ze znaną prawdą; progi §11.1 bez zmian; bez zależności GPL/LGPL/AGPL.
Katalog kompilacji klonowany ze wzorca, najwyżej 3 kompilujących agentów naraz.

## Pakiety

### Etap O — części organiczne (cel produktu)

| Id | Zakres | Główne pliki | Model | Zależy od |
|---|---|---|---|---|
| O1 | `surface.wrap`: łaty adaptacyjne (duże na obszarach gładkich, małe przy detalu) zamiast siatki równomiernej, żeby 3 skany przekraczające 12 000 łat zmieściły się w tolerancji | `fit/wrap`, `engine/cmd/surface.rs` | Sonnet | — |
| O2 | Ciągłość styczna między łatami (dziś G0, skok 11–31°): wykrycie krawędzi ostrych (zostają G0), wspólne płaszczyzny styczne na pozostałych; raport skoku w wyniku komendy | `fit/{patch,wrap}.rs`, `kernel/nurbs.rs` | Sonnet | O1 |
| O3 | Błąd jądra `item budget` na skanie ok. 1 mln trójkątów: reprodukcja w czystym solidcore, obejście (budowa powłoki partiami); szkic zgłoszenia healera i limitu do `docs/upstream` | `crates/kernel`, `docs/upstream` | Sonnet | — |
| O4 | Czas: redukcja do tolerancji i owinięcie równolegle; cel ≤ 60 s na 1 mln trójkątów (dziś 160–370 s) | `geom/edit.rs`, `fit/wrap` | Sonnet | P8-H2 |
| O5 | STEP bryły wielołatowej: wspólne wierzchołki i krawędzie zapisywane raz; cel ≤ 6 KB na łatę (dziś ok. 24 KB) | `crates/io/step` | Sonnet | — |
| O6 | Prawdziwy model hybrydowy: `scan.to_solid` z metodą `hybrid` — prymitywy tam, gdzie pasują, łaty na reszcie, jedna zamknięta bryła | `engine/cmd/{pipeline,plan}`, `kernel/hybrid.rs` | Sonnet | O1, O2 |
| O7 | Segmentacja skanu organicznego bez dojścia do limitu 10 000 regionów: scalanie regionów swobodnych w obszary krzywiznowe | `fit/segment.rs` | Sonnet | — |
| — | P8-H9 (układ retopologii dopasowany do konturu regionu, odwrócone łaty, `surface.from_curves` bez 4 rogów), P8-H11 (maski i pokrycie w `surface.fit`) | | | |

### Etap V — weryfikacja

| Id | Zakres | Kto |
|---|---|---|
| V1 | Przebieg integracyjny na maszynie referencyjnej: `local_corpus_full_pipeline`, `local_corpus_scan_to_solid` (`method: auto`, dotąd niezmierzony), `perf_scan_scale`, `perf_segment_organic`, `perf_store_scale`, GPU/GUI wg list „Do sprawdzenia lokalnie” | orkiestrator na stacji referencyjnej |
| V2 | Ręczne przejście prawdziwego skanu przez GUI (okno natywne, Chrome/Windows, Firefox, Safari): import, czyszczenie, 3-2-1 wskazywaniem, podział, bryła, odchyłki, STEP; lista błędów → pakiety naprawcze w tym samym etapie | właściciel + orkiestrator |
| V3 | Tabela porównawcza na tych samych skanach: SurfaceCraft kontra wyniki Geomagic i QuickSurface (czas pracy, % punktów w tolerancji, liczba ścian) — po D6 | właściciel (programy licencjonowane) |

### Etap S — skala

| Id | Zakres | Model |
|---|---|---|
| P8-G2 | Magazyn out-of-core jako obiekt dokumentu i źródło widoku, odchyłek, ICP, szumu | Sonnet |
| P8-H2 | Równoległość QEM, dziur, parsowania STL; `weld` bez sortowania 30 mln kluczy | Sonnet |
| S1 | Strumieniowanie ASCII PLY, OBJ, XYZ; szwy kawałków na zgrubnych poziomach LOD; odchyłki trzymane poza sesją (dziś na wierzchołek w pamięci) | Sonnet |
| S2 | Segmentacja i `surface.wrap` na magazynie (kawałkami, z łączeniem na szwach) | Sonnet |

### Etap M — wiele skanów i auto-wyrównanie

| Id | Zakres | Główne pliki | Model |
|---|---|---|---|
| M1 | `align.auto`: poza z samego skanu (głosowanie orientacji ścian, największa płaska ściana na spód, długi bok wzdłuż X), wynik jako propozycja z podglądem; `align.symmetry` (poza na płaszczyźnie symetrii); `mesh.scale_from_length` | `crates/align`, `engine/cmd/align.rs` | Sonnet |
| M2 | `align.pair_coarse`: wstępne wyrównanie dwóch skanów bez punktów (24 permutacje osi głównych + deskryptory lokalne, ocena odporna), potem ICP | `crates/align` | Sonnet |
| M3 | `align.global`: wiele skanów naraz, relaksacja grafu póz po ICP parami, raport błędu pętli | `crates/align` | Sonnet |
| M4 | `mesh.merge`: scalenie wyrównanych skanów w jedną siatkę (usunięcie nakładek, rekonstrukcja szwów istniejącym ball pivotingiem) | `geom/recon.rs`, `engine/cmd/mesh.rs` | Sonnet |
| M5 | Kreator „Wiele skanów” w GUI: kolejne kroki M2 → M3 → M4 z podglądem | `ui-egui` | Haiku |

### Etap C — kompletność CAD

| Id | Zakres | Główne pliki | Model |
|---|---|---|---|
| C1 | Parametry nazwane i wyrażenia w wymiarach szkicu i polach cech; przeliczenie przy replay; wykrywanie cykli | `crates/doc`, `crates/sketch`, `engine/cmd/feature.rs` | Sonnet |
| C2 | `feature.hole` (prosty, pogłębiany, stożkowy; z regionu walca skanu), `feature.thread` (ISO metryczny; podziałka i średnica podpowiadane ze skanu), krzywa helisy i sweep po niej | `kernel/feature.rs`, `engine/cmd/feature.rs`, `crates/fit` | Sonnet |
| C3 | `feature.pipe`, `feature.emboss`, `body.move_face`, `body.delete_face` | `kernel`, `engine/cmd` | Sonnet |
| C4 | Podpowiedzi ze skanu przy tworzeniu cech: ścieżka sweepu, oś obrotu, płaszczyzna lustra, kierunek i liczba wzoru, grubość, oś rowka | `crates/fit`, `engine/cmd/feature.rs` | Sonnet |
| C5 | Naprawa siatki świadoma kształtu: `mesh.rewrap` (dziura na walcu domykana walcem), `mesh.defeature` | `geom/edit.rs`, `crates/fit` | Sonnet |
| C6 | Narzędzia siatki: cięcie płaszczyzną, lustro, offset, boolean z bryłą, owinięcie w jedną powłokę, przycięcie bryły do skanu; import 3MF; eksport STL/PLY/3MF w pozie | `geom`, `crates/io`, `engine/cmd/mesh.rs` | Haiku |
| C7 | Historia: znacznik cofania (roll-back), odtwarzanie z podpisami; zapamiętywanie ostatniego trybu wyniku (nowa bryła, suma, cięcie) na komendę | `crates/doc`, `ui-egui/history_panel.rs` | Haiku |
| C8 | Szkic nawinięty i rozwinięty (`sketch.roll`, `sketch.unroll`) dla części obrotowych i blachy | `crates/sketch`, `kernel` | Sonnet |
| — | P8-H7 (przeciąganie punktów szkicu), P8-H8 (sweep po splajnie, loft z krzywymi prowadzącymi, split z obiema stronami) | | |

### Etap R — gotowość produktu

| Id | Zakres | Model |
|---|---|---|
| R1 | Podpisane wydania (po D3), MSI, AppImage, macOS Intel, VERSIONINFO w binarkach Windows | Haiku |
| R2 | Asystent z prawdziwym dostawcą (po D4): program-dostawca w repo, zgoda użytkownika na wysyłkę danych, tryby align i workflow sprawdzone na skanie | Sonnet |
| R3 | Dostępność: drzewo AccessKit w oknie natywnym | Haiku |
| R4 | Kody błędów w `EngineError` zamiast dopasowywania tekstu; test kompletności tłumaczeń | Haiku |
| R5 | Jednostki mm/cal w wyświetlaniu, punkty odzyskiwania sesji, schematy myszy innych CAD-ów, SpaceMouse | Haiku |
| R6 | Nagrywanie makr: dziennik komend zapisywany jako skrypt `app.batch`, uruchamianie wsadowe na katalogu skanów z CLI | Haiku |
| R7 | Przegląd opcjonalnych liczb bez wartości domyślnej; formularze dla `feature.edit` i kroków `app.batch` | Haiku |
| — | P8-H10 (wiele instancji, token poza plikiem tymczasowym, mniejszy bundle web) | |

### Etap I — inspekcja (po D1)

| Id | Zakres | Model |
|---|---|---|
| I1 | Elementy konstrukcyjne (punkt, linia, płaszczyzna, okrąg z dopasowań i przecięć) przeliczane po refitcie | Sonnet |
| I2 | Wymiary między elementami (odległości, kąty, średnice, Ø min/max) z nominałem i tolerancją | Sonnet |
| I3 | GD&T strefą minimalną: płaskość, walcowość, kulistość, okrągłość, równoległość, prostopadłość, kątowość, współosiowość | Sonnet |
| I4 | Raport pomiarowy PASS/FAIL (tekst, CSV), eksport elementów bazowych do STEP | Haiku |
| I5 | Mapa grubości ścianek (promień wzdłuż normalnej, kula wpisana) | Sonnet |
| I6 | Pomiar 2D z obrazu płaskiego skanera (krawędzie podpikselowe, kalibracja skali) — najniższy priorytet | Sonnet |
| I7 | Zakładka „Pomiar” w GUI | Haiku |

### Etap X — przekazanie cech (po D2)

| Id | Zakres | Model |
|---|---|---|
| X1 | `export.feature_script`: drzewo cech jako skrypt dla jednego otwartego CAD (FreeCAD), szkice z więzami i wymiarami | Sonnet |
| X2 | Drugi cel wybrany przez właściciela (makro SolidWorks lub skrypt Fusion) | Sonnet |

## Fale

1. **Fala 1 (od razu):** O1, O3, O5, O7 + P8-H2, P8-H9.
2. **Fala 2:** O2, O4, O6, P8-H11 → V1, V2 (bramka etapu O na korpusie).
3. **Fala 3:** P8-G2, M1, M2, C1, C2, P8-H7, P8-H8.
4. **Fala 4:** S1, S2, M3, M4, C3, C4, C5.
5. **Fala 5:** M5, C6, C7, C8, R3–R7, P8-H10; R1, R2 po decyzjach.
6. **Fala 6 (po D1, D2):** I1–I7, X1–X2.

Po każdej fali: przebieg integracyjny, STATUS, aktualizacja [porównania](../../comparison.md).

## Ryzyka

- O2 (G1 przy dowolnej walencji) to znany trudny problem; jeśli konstrukcja po stronie aplikacji nie zejdzie
  poniżej 1°, pakiet kończy się raportem i propozycją prymitywu jądra (upstream, D5).
- O3 i O4 zależą od zachowania solidcore; bez zgody D5 zostają obejścia.
- Szacunki zakładają tempo fal 1 i 2 z P8 (18 pakietów w 2 dni) i wolną maszynę do kompilacji; ręczne
  sprawdzenia właściciela nie są w nich liczone.
