# P5 — Stabilna parametryka i komplet cech (plan szczegółowy)

> Etap P5 planu [production-readiness](2026-11-09-production-readiness.md). Gałąź `p5-parametric`.

## Stan wyjściowy
Cechy `fillet`, `chamfer`, `shell` trzymają pozycyjne indeksy krawędzi i ścian rekordu `Brep`. Gdy wcześniejsza
cecha zmienia topologię (otwór przelotowy staje się ślepy, dochodzi zaokrąglenie wyżej w historii), ten sam
indeks wskazuje inną krawędź i fillet „przeskakuje”. Brak też komend dla loft, sweep, draft, mirror, split i
wzoru po krzywej; brak auto-więzów między prymitywami (§6.2) i podpowiedzi osi/kąta w kreatorach (§6.4).

## Decyzje
1. **Nazewnictwo = sygnatura geometryczna, nie `provenance`.** `solidcore-ops::provenance` żyje na atrybutach
   kernela, które nie przetrwają naszego round-tripu rekordu (każda operacja dekoduje rekord i koduje wynik).
   Sygnatura liczy się z samego rekordu (`kernel::naming`):
   - krawędź: środek i końce, styczna, rodzaj krzywej, rodzaj i normalne dwóch przyległych ścian;
   - ściana: rodzaj powierzchni, normalna (płaszczyzna) lub oś (walec, stożek, torus), środek ciężkości
     (na osi dla powierzchni obrotowych).
   Pozycje są normalizowane do bryły otaczającej ciała, więc skalowanie części nie zmienia sygnatury.
2. **Rozwiązywanie przy replay** (`engine::replay::bind_refs`): bez sygnatur (pierwsze uruchomienie lub edycja
   `edges`/`faces` komendą `feature.edit`) są zdejmowane z zapisanych indeksów; z sygnaturami indeksy są
   wyznaczane na nowo przez najlepsze dopasowanie (ważony koszt, próg `MAX_COST` = 1,5) i zapisywane w cesze.
   Brak dopasowania poniżej progu = cecha kończy się błędem (`feature.error`), nie paniką. Stare dokumenty
   ładują się bez zmian (`serde(default)`; sygnatury powstają przy pierwszym replay).
3. **Nowe cechy** jako warianty `FeatureOp` + komendy `feature.*` + opakowania w `kernel::feature`:
   `loft` (`ops::sweep::loft`), `sweep` (`sweep_placed`, profil na początku ścieżki), `draft`
   (`ops::offset::draft`, ściany z sygnaturami), `mirror` (transformacja odbijająca + suma), `split`
   (`split_body` płaszczyzną, zostaje jedna strona), `path_pattern` (kopie co równą długość łuku wzdłuż
   otwartego łańcucha szkicu, opcjonalnie obracane ze styczną; `sketch::open_paths`/`path_samples`).
4. **Auto-więzy** `primitive.auto_constrain`: relacje równoległe/prostopadłe (normalne i osie), współosiowe,
   równe promienie w tolerancjach kątowej (3°) i długości (0,2 mm); przyciąganie: klasy równoległości dostają
   średni kierunek, klasy prostopadłe są ortogonalizowane, grupy współosiowe dzielą jedną prostą, równe
   promienie — średnią. Plasterek wyświetlania jest rzutowany na nową powierzchnię.
5. **Redesign assistant**: `feature.suggest_axis` (oś obrotu z najlepszego walca/stożka/torusa/sfery dopasowanej
   do regionu skanu) i `feature.suggest_draft` (kąt pochylenia ściany skanu względem kierunku wyciągania),
   obok istniejącego `feature.suggest_fillet`.

## Testy (bramka)
- `gate_p5_stable_references_over_20_random_edits` (`crates/engine/tests/p5_parametric.rs`): wyciągnięcie +
  kieszeń (przelotowa ↔ ślepa) + dwa fillety + shell; 20 edycji (ziarno stałe) wymiarów szkicu i parametrów cech;
  po każdej: fillety na krawędziach w tych samych narożnikach (pozycja znormalizowana, kierunek pionowy), shell
  otwiera górną ścianę, objętości zgodne z postacią zamkniętą; test wymaga, by numeracja rekordu zmieniła się
  co najmniej raz (inaczej nie ćwiczy nazewnictwa).
- Objętości nowych cech (`p5_features.rs`): loft = ścięty ostrosłup, sweep po łuku = Pappus, draft = ścięty
  ostrosłup z tg 5°, mirror 2×, split 1500/500/połowa sześcianu, wzór po ścieżce = n × objętość.
- Auto-więzy na zaszumionym wsporniku (σ = 0,05 mm) + dwa dodatkowe walce; podpowiedzi osi i kąta.

## Luki
Zapisane w `docs/STATUS.md` (dług techniczny).
