# Od MVP do pełnoprawnego programu reverse engineeringu: plan

> Stan wyjściowy: M0–M6 zamknięte (docs/STATUS.md). Wszystkie bramki zmierzone na danych **syntetycznych**.
> Cel tego planu: program, którym inżynier odtwarza realną część ze skanu (mechaniczną i organiczną) do
> edytowalnego modelu CAD z kontrolą odchyłki — bez obejść przez JSON i bez zewnętrznego CAD-a.
> Zasady bez zmian: design doc (§5 podział obliczeń, §9 clean-room i kod rodziny, §11.1 progi, §12 decyzje),
> każdy etap = krótki plan szczegółowy, gałąź, testy, `cargo xtask ci`, PR, aktualizacja STATUS.md.

## Kryteria „pełnoprawności” (mierzalne)

1. **Realne skany:** na lokalnym korpusie QS (`SURFACECRAFT_LOCAL_CORPUS`, poza repo) dla każdego pliku: import,
   segmentacja, wyrównanie i rekonstrukcja kończą się bez błędu; zapisane w STATUS (bez danych i zrzutów z korpusu).
2. **Dokładność na realnych danych:** prymitywy RMS ≤ 0,1 mm tam, gdzie szum skanu (estymowany) na to pozwala;
   powierzchnie max ≤ 0,2 mm na regionach bez defektów; raport, gdy próg < szum (§10).
3. **Wydajność (MVP §12: do ~10 mln trójkątów):** import 10 mln trójkątów < 15 s, auto-segmentacja < 60 s,
   ICP < 10 s, viewport ≥ 30 fps — pomiar licznikami i czasem na maszynie referencyjnej, zapisany w STATUS.
4. **Model hybrydowy:** prymitywy i płaty swobodne łączą się z bryłą parametryczną w jedną zamkniętą bryłę
   eksportowaną do STEP (round-trip) — na co najmniej dwóch częściach referencyjnych.
5. **Stabilna parametryka:** edycja wcześniejszej cechy nie przestawia fillet/shell na inne krawędzie
   (test regresji na 20 losowych edycjach).
6. **Ergonomia:** pełny przepływ (import → wyrównanie → prymitywy → szkic → cechy → eksport) wykonalny wyłącznie
   myszą, bez edycji JSON; test GUI odtwarza go klikami.

## Etapy

Kolejność według ryzyka: najpierw prawda o realnych danych, potem to, czego bez niej nie da się dobrze zaprojektować.

### P1 — Realne skany: pomiar i naprawa (najpierw)
- Testy `#[ignore]` czytające `SURFACECRAFT_LOCAL_CORPUS`: każdy plik korpusu QS przez import → noise estimate →
  auto-segment → wyrównanie → fit; zapis metryk do STATUS (liczby, nie dane).
- Korpus w repo poziomu 1: wybrane pliki CC0 z `tangency-labs/models` w osobnym repo `surfacecraft-corpus`
  (§11.1), każdy z wierszem w ATTRIBUTION; testy CI na nich (siatki z prawdziwych modeli CAD + szum).
- Mesh-edit minimalny dla realnych danych: usuwanie outlierów i małych komponentów, wypełnianie dziur (płaskie +
  gładkie), decymacja z zachowaniem krawędzi, wygładzanie — jako komendy `mesh.*` z testami wrogiego wejścia.
- Odporność segmentacji na ostre krawędzie, nierównomierną gęstość i dziury; estymator szumu odporny na krzywiznę.
- Bramka: kryteria 1 i 2 na korpusie lokalnym + CI na korpusie CC0.

### P2 — Wydajność i skala do 10 mln trójkątów
- Pomiary: liczniki (zapytania BVH, iteracje LM, alokacje) i czasy na skanach 1/5/10 mln trójkątów.
- Równoległość (rayon, poza wasm): budowa BVH/KD, ICP, odchyłka, segmentacja; próbkowanie w fitach.
- Viewport: LOD siatki (decymacja do wyświetlania), aktualizacja kolorów bez re-uploadu geometrii, cache szkiców.
- Abstrakcja dostępu do meshu pod przyszłe out-of-core (§9), bez implementacji out-of-core.
- Bramka: kryterium 3.

### P3 — Model hybrydowy
- Prymityw → ściana B-rep: konwersja dopasowanego prymitywu do analitycznej powierzchni rekordu, przycinanie do
  konturu regionu (krzywe przecięć sąsiednich prymitywów).
- Płat swobodny przycinany do konturu regionu (dziś prostokąt w parametrach); zszywanie płatów z bryłą
  (`ops::stitch`, `split_body_by_sheet`), zamiana ściany bryły na płat (replace face).
- „Wypełnij bryłę” z zestawu ścian (prymitywy + płaty) → zamknięta powłoka → bryła.
- Bramka: kryterium 4 na dwóch częściach referencyjnych (syntetycznej i z korpusu CC0).

### P4 — Freeform dla kształtów organicznych
- Retopologia niestrukturalna (decyzja §10: port natywny Instant Meshes, MIT, z atrybucją) dla regionów zawiniętych
  i zamkniętych; łaty z quadów z ciągłością G1 (constrained fit krawędzi — kandydat na prymityw kernela upstream).
- Druga strategia freeform (§6.3): sieć łat z krzywych cech dla obudów z wyraźnymi krawędziami.
- Bramka: max odchyłka ≤ 0,2 mm na zamkniętym kształcie organicznym (syntetyczny + korpus), szczelna bryła po thicken
  lub zamknięciu.

### P5 — Stabilna parametryka i komplet cech
- Trwałe nazewnictwo topologii: cechy odwołują się do krawędzi/ścian przez historię powstania (provenance z
  `solidcore-ops`), nie przez indeks; naprawa odwołań po replay.
- Brakujące cechy: loft, sweep, trim/split, draft, mirror, pattern po krzywej; auto-więzy między prymitywami (§6.2).
- Redesign assistant (§6.4) w kreatorach: promień, oś, kąt pochylenia z meshu przy tworzeniu cechy.
- Bramka: kryterium 5 + testy objętości każdej nowej cechy.

### P6 — Ergonomia GUI
- Zaznaczanie: pędzel, lasso, „smart select” po krzywiźnie, zaznaczanie krawędzi i ścian bryły w viewporcie.
- Kreatory komend (formularze generowane ze schematów JSON zamiast edytora JSON), podgląd na żywo przed zatwierdzeniem.
- Gizmo manipulatora (§6.1), widoki standardowe, przekroje w viewporcie, mapa odchyłek z legendą i histogramem.
- Drzewo historii z edycją, wyłączaniem i błędami; edytor szkicu z rysowaniem i więzami myszą.
- Bramka: kryterium 6 (test GUI całego przepływu klikami) + przegląd UX z użytkownikiem.

### P7 — Dystrybucja (za zgodą użytkownika przy krokach poza repo)
- Instalatory Windows/macOS/Linux, bundle id, `SHA256SUMS.txt`, podpis macOS (kontrakt `craft-launcher`, §9).
- Web: optymalizacja rozmiaru (`wasm-opt`), ładowanie dużych plików strumieniowo.
- Wpis w `storytold/craft-launcher` i wydania — dopiero po zgodzie użytkownika.

## Zależności i równoległość
- P1 przed wszystkim (wyniki zmieniają priorytety P2–P4).
- P2 i P5 mogą iść równolegle po P1; P3 przed P4 (wspólne przycinanie i zszywanie); P6 równolegle z P3–P5,
  bo komendy już istnieją; P7 na końcu.
- Zgłoszenia upstream (tangency-labs/solidcore #3228–#3230) śledzone w każdym etapie; po ich rozwiązaniu adaptacje
  z długu wymieniamy na kod solidcore.

## Ryzyka
- Dojrzałość solidcore (offset, trim, przekroje *partial*) może blokować P3–P5 — obejścia tylko za zgodą (zasady).
- Wydajność wasm i pamięć przeglądarki dla dużych skanów — web pozostaje celem warunkowym (§11).
- Retopologia organiczna to praca badawcza; ryzyko harmonogramu największe w P4.
