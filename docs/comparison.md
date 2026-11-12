# SurfaceCraft a Geomagic Design X, QuickSurface i ScanRuler — porównanie

> Stan na 2026-11-12, `main` 979ec54 (M0–M6 zamknięte, P8 fale 1 i 2 scalone, #75 scalony).
> Plan uzupełnienia braków: [P9](superpowers/plans/2026-11-12-p9-competitive-gaps.md).
> Źródła: [STATUS.md](STATUS.md) oraz analizy funkcjonalne w `docs/superpowers/specs/`
> ([Geomagic Design X 2026.1](superpowers/specs/2026-11-08-geomagic-designx-functional-analysis.md),
> [QuickSurface 2026](superpowers/specs/2026-11-08-quicksurface-functional-analysis.md),
> [ScanRuler 0.10.1](superpowers/specs/2026-11-09-scanruler-functional-analysis.md)).
> Kolumny konkurentów opisują **udokumentowane** funkcje, nie wyniki naszych pomiarów tych programów:
> żadnego z nich nie uruchamialiśmy na tym samym skanie co SurfaceCraft. ScanRuler rozwija się kilka wydań
> tygodniowo, więc jego kolumna starzeje się najszybciej.

Legenda: ✅ jest · ◐ częściowo lub tylko na danych syntetycznych · ✗ brak · — świadomie poza zakresem.

## Wniosek w pięciu zdaniach

1. **Szerokość funkcji:** SurfaceCraft ma odpowiednik prawie każdego kroku potoku QuickSurface (wyrównanie,
   prymitywy, freeform, szkic i cechy, odchyłki, STEP/IGES) i najważniejsze dodatki z Geomagic (regiony jako
   encja, podpowiedzi parametrów, CAD Correct, sieć łat z krzywych cech).
2. **Dojrzałość:** dokładność jest udowodniona głównie na skanach syntetycznych ze znaną prawdą. Na prawdziwych
   skanach działa import, czyszczenie, segmentacja, prymitywy i ICP; **części organiczne — cel produktu —
   przechodzą do modelu w progu 0,2 mm w 9 z 13 skanów** (`surface.wrap`), ale łaty stykają się bez ciągłości
   stycznej, 3 skany przekraczają limit łat, a 1 kończy się błędem jądra.
3. **Przewaga:** licencja MIT/Apache bez OCCT i Parasolid, jeden rdzeń komend dla desktopu, przeglądarki, CLI
   i MCP, bramki dokładności i odporności na wrogie dane w CI. Tego nie ma żaden z trzech.
4. **Największe braki względem Geomagic/QuickSurface:** jakość automatycznego powierzchniowania na realnych
   skanach, praca na wielu skanach naraz, przekazanie drzewa cech do SolidWorks/Inventor, lata szlifu GUI.
5. **Największe braki względem ScanRuler:** warstwa inspekcji (wymiary, GD&T, grubość ścianek, pomiar 2D),
   auto-wyrównanie, parametry i wyrażenia, gwinty.

## Produkt i platforma

| | SurfaceCraft | Geomagic Design X | QuickSurface | ScanRuler |
|---|---|---|---|---|
| Licencja | MIT OR Apache-2.0 | komercyjna (Hexagon) | komercyjna (KVS) | AGPL-3.0 + płatny wyjątek |
| Kernel B-rep | solidcore (Rust) | Parasolid | OpenCASCADE + Parasolid | OCCT (wasm) |
| Systemy | Windows, Linux, macOS, przeglądarka | Windows | Windows | tylko przeglądarka |
| CLI / skrypty | ✅ CLI, skrypty JSON, `app.batch` | ✅ makra, Batch Process Designer | ✗ (nie stwierdzono) | ◐ silnik headless dla części komend |
| MCP dla agentów | ✅ każda komenda, pełny silnik bez okna | ✗ | ✗ | ✅ przez otwartą kartę przeglądarki |
| Asystent AI w aplikacji | ◐ pętla jest, dostawca LLM niepodpięty | ✗ | ✅ Gemini (align, workflow) | ✗ (tylko agent zewnętrzny) |
| Dystrybucja | v0.1.0, nightly, **niepodpisane** | instalator, licencja CLM/HASP | instalator, updater | strona + Docker |
| Skala skanu | 10 mln trójkątów w pamięci (segmentacja 55 s); import strumieniowy jest, ale magazyn nie zasila jeszcze widoku ani odchyłek | „bez limitu” (out-of-core, LOD) | out-of-core w AutoSurface | ok. 1–2 mln w przykładach |

## Potok scan-to-CAD

| Funkcja | SurfaceCraft | Geomagic Design X | QuickSurface | ScanRuler |
|---|---|---|---|---|
| Import siatek i chmur | ✅ STL, PLY, OBJ, XYZ; chmura → siatka (ball pivoting) | ✅ + LAS, formaty skanerów | ✅ | ✅ STL, PLY, OBJ, 3MF |
| Import STEP | ✅ | ✅ + JT, NX, SolidWorks… | ✅ | ✅ |
| Sterowanie skanerem | — | ✅ | ✗ | ✗ |
| Wiele skanów: wstępne i globalne wyrównanie, scalanie | ✗ | ✅ Mesh Buildup Wizard | ✅ Global Fine Align | ✗ |
| Czyszczenie siatki | ✅ clean, dziury z zachowaniem krawędzi, wygładzanie, decymacja z mierzoną tolerancją, kopia robocza | ✅ + Rewrap, Defeature | ✅ | ✅ + wrap, boolean, shell, trim-to-scan |
| Wyrównanie 3-2-1, N punktów, ICP, osie świata | ✅ | ✅ | ✅ | ✅ |
| Lokalne ICP na regionie | ✅ | ✅ | ◐ | ✅ |
| Auto-wyrównanie z samego skanu | ◐ `align.principal` (osie główne) | ✅ Quick Fit | ◐ closest axis | ✅ z podglądem |
| Regiony jako trwała encja + auto-segmentacja | ✅ (resegment, merge, split, insert, separate, enlarge, shrink) | ✅ | ◐ ekstrakcja z zaznaczenia | ◐ tylko regiony płaskie |
| Prymitywy: płaszczyzna, walec, stożek, sfera, torus, symetria | ✅ z auto-typem po residuum | ✅ | ✅ | ✅ typ wybiera użytkownik |
| Auto-więzy prymitywów | ✅ równoległość, prostopadłość, współosiowość, równe promienie, styczność, symetria, równe odległości | ✅ | ✅ | ✅ |
| Fit Surface (jeden płat NURBS) | ✅ z przycinaniem i podziałem (krawędź przycięcia ząbkowana) | ✅ | ✅ | ✅ |
| AutoSurface (retopologia quad → łaty) | ◐ syntetyczne w progu; realne skany: brzeg regionu nie nadąża, 1 z 4 w 0,2 mm | ✅ | ✅ Quad-Remesher / Instant Meshes | ✗ nieplanowane |
| Sieć łat z krzywych cech | ◐ tylko regiony z 4 rogami | ✅ Class-A, edycja sieci | ◐ | ◐ rysowana ręcznie |
| Owinięcie całego skanu siatką łat | ◐ `surface.wrap`: 9 z 13 prawdziwych skanów ≥ 99 % punktów w 0,2 mm (6 brył, 3 arkusze); limit 12 000 łat | ✅ | ✅ | ✗ |
| Ciągłość łat | ◐ `surface.auto` G1 przybliżone (skok do 3°); `surface.wrap` tylko G0 (skok 11–31°) | ✅ G1/G2 | ✅ G1 | ✅ styczność |
| Pogrubienie arkusza w bryłę | ◐ arkusz jednościenny lub sieć zamknięta | ✅ | ✅ | ◐ |
| Bryła hybrydowa z regionów (prymitywy + płaty) | ◐ `body.from_regions`: kieszenie, otwory, fillety, czopy na częściach syntetycznych | ✅ ręcznie, cechami | ✅ ręcznie, cechami | ✅ auto-trim zbioru powierzchni |
| Skan → bryła jednym poleceniem | ◐ `scan.to_solid` z werdyktem pass/review/fail: część syntetyczna `pass`; ścieżka prymitywów na 13 skanach organicznych 0 zaliczeń; pełny przebieg `method: auto` na korpusie niezmierzony | ✗ | ✗ | ◐ agent, 69–96 % w tolerancji |
| CAD Correct (istniejąca bryła → nowy skan) | ✅ `body.refit_faces`, `body.refit_features` | ✅ | ✗ | ◐ snap faces |

## Modelowanie parametryczne

| Funkcja | SurfaceCraft | Geomagic Design X | QuickSurface | ScanRuler |
|---|---|---|---|---|
| Szkic 2D z solverem więzów | ✅ (bez przeciągania punktów myszą) | ✅ | ✅ | ✅ |
| Szkic z przekroju: łańcuchy → linie i łuki | ✅ `sketch.fit_chains`, stos przekrojów | ✅ | ✅ | ✅ |
| Extrude, revolve, cut, boolean, fillet, chamfer, shell | ✅ | ✅ | ✅ | ✅ |
| Loft, sweep, draft, mirror, split, wzory | ✅ (sweep i wzór tylko po liniach i łukach; loft bez krzywych prowadzących) | ✅ | ◐ | ✅ |
| Helix, gwint, otwór jako cecha, pipe, emboss | ✗ | ✅ | ◐ helix | ◐ otwór, gwint |
| Parametry nazwane i wyrażenia | ✗ | ✅ | ✗ (nie stwierdzono) | ✅ |
| Historia: edycja, suppress, replay, undo | ✅ (bez znacznika roll-back) | ✅ | ✅ | ✅ + odtwarzanie |
| Trwałe odwołania do ścian i krawędzi | ✅ sygnatury geometryczne, bramka 20 losowych edycji | ✅ | ✅ | ✅ |
| Podpowiedzi parametrów ze skanu | ◐ promień filletu, oś, kąt pochylenia | ✅ Redesign Assistant (kilkanaście rodzajów) | ◐ | ◐ |
| Przekazanie drzewa cech do SolidWorks / Inventor / NX | — | ✅ LiveTransfer | ✅ | ✗ |
| Eksport | STEP (bryły, arkusze z PCURVE), IGES, DXF/SVG przekrojów | STEP, IGES, Parasolid, JT… | STEP, IGES | STEP, STL, 3MF, DXF/SVG |

## Kontrola dokładności i inspekcja

| Funkcja | SurfaceCraft | Geomagic Design X | QuickSurface | ScanRuler |
|---|---|---|---|---|
| Mapa odchyłek skan ↔ model ze znakiem | ✅ | ✅ Accuracy Analyzer | ✅ | ✅ |
| Histogram, pasma, odczyt w punkcie, hotspoty | ✅ | ✅ | ◐ | ✅ |
| Odchyłka względem pojedynczego prymitywu | ✅ | ✅ | ✅ | ✅ |
| Raport bryła ↔ skan per ściana, kontrola skanu | ✅ `body.compare`, `scan.qa` | ◐ | ✗ | ✅ |
| Wymiary z tolerancją, GD&T, raport PASS/FAIL | ✗ (decyzja właściciela otwarta) | — (to Geomagic Control X) | ✗ | ✅ |
| Grubość ścianek, pomiar 2D z płaskiego skanera | ✗ | ◐ | ✗ | ✅ |

## Gdzie SurfaceCraft jest przed konkurentami

- **Otwartość i licencja:** jedyny z czterech, który można wbudować w zamknięty produkt bez opłat i bez
  LGPL/AGPL; brak zależności od Parasolid i OCCT.
- **Jeden rdzeń, cztery wejścia:** te same ponad 120 komend z JSON Schema w oknie, w przeglądarce, w CLI i przez MCP;
  pełny silnik działa bez okna (ScanRuler potrzebuje karty przeglądarki, Geomagic i QuickSurface nie mają MCP).
- **Udowodniona poprawność:** bramki dokładności na częściach ze znaną prawdą (objętość bryły 0,004–0,29 % od
  CAD, odchyłka skanu ≤ 0,2 mm), fuzz importerów i wrogie parametry każdej komendy bez panik.
- **Automatyczna bryła z regionów:** planer sam rozpoznaje kieszenie, otwory, fillety, fazy i czopy; w Geomagic
  i QuickSurface ten krok robi człowiek cechami.
- **Linux, macOS i przeglądarka** — Geomagic i QuickSurface są tylko na Windows.

## Gdzie SurfaceCraft jest za konkurentami

1. **Części organiczne na prawdziwych skanach** (cel produktu). `surface.wrap` mieści 9 z 13 skanów w 0,2 mm,
   ale: łaty stykają się G0, 3 skany wymagają ponad 12 000 łat, 1 skan ok. 1 mln trójkątów kończy się błędem
   jądra, owinięcie trwa 160–370 s przy milionie trójkątów, STEP bryły ma ok. 24 KB na łatę. Nie ma jeszcze
   jednej bryły łączącej prymitywy z łatami na prawdziwym skanie. Geomagic i QuickSurface robią to rutynowo.
2. **Weryfikacja na realnych danych i przez człowieka.** Żaden pakiet fali 2 nie był sprawdzony na prawdziwych
   skanach ani ręcznie w GUI; pomiarów na maszynie referencyjnej po fali 2 brak.
3. **Wiele skanów:** brak globalnego wyrównania i scalania.
4. **Skala powyżej 10 mln trójkątów:** magazyn out-of-core istnieje, ale nie jest obiektem dokumentu.
5. **Kompletność CAD:** brak parametrów i wyrażeń, gwintów, helisy, otworu jako cechy, sweepu po splajnie,
   znacznika roll-back, przeciągania punktów szkicu.
6. **Inspekcja:** brak wymiarów, GD&T, grubości ścianek, pomiaru 2D (ScanRuler ma komplet).
7. **Gotowość produktu:** wydania niepodpisane, asystent AI bez dostawcy, interfejs niewidoczny dla czytnika
   ekranu, część komunikatów jądra tylko po angielsku.

## Co z tego wynika dla kolejności prac

Pełny plan z pakietami, bramkami i decyzjami właściciela: [P9](superpowers/plans/2026-11-12-p9-competitive-gaps.md).
W skrócie: części organiczne (13 z 13 skanów w 0,2 mm, ciągłość styczna) → weryfikacja na maszynie referencyjnej
i w GUI → skala, wiele skanów i kompletność CAD równolegle → gotowość produktu → inspekcja i przekazanie cech
po decyzjach właściciela.
