# M2 — prymitywy: plan

> Źródło: design doc §6.2, §7, §8 (M2), §10, §11.1. Gałąź `m2-primitives`.

## Zakres
1. `fit::primitive` — płaszczyzna (PCA), sfera (Kåsa + LM), walec (oś z normalnych/PCA, okrąg Kåsy, LM 5 parametrów),
   stożek (oś z normalnych centrowanych, promień liniowy w wysokości, LM 6 parametrów), torus (skan promienia rurki:
   środki rurek `p − s·n` leżą na płaskim okręgu, potem LM 7 parametrów). Odległość ortogonalna, normalne, rzut,
   parametryzacja naturalna i łata do wyświetlania. Wynik = dokładne powierzchnie analityczne rekordu solidcore.
2. `fit::lm` — własny Levenberg–Marquardt (decyzja §10: zamiast `levenberg-marquardt`/`argmin` — ≤ 7 parametrów,
   gęste równania normalne, jedna strona kodu zamiast zależności).
3. `fit::auto` — auto-typ: wszystkie rodzaje, najprostszy z RMS ≤ 1,2 × najlepszego; zdegenerowane wyniki odrzucone
   z komunikatem („more suitable as a cylinder/plane”).
4. `fit::segment` — auto-segmentacja fit-and-grow (deterministyczny wariant Efficient RANSAC na spójności meshu),
   `grow_smooth` (zaznaczanie gładkiego obszaru).
5. `align::symmetry` — płaszczyzna symetrii: odbicie względem płaszczyzn głównych, ICP odbicia na oryginał,
   płaszczyzna z punktów środkowych `(p + Mp)/2`.
6. `deviation` — odchyłka ze znakiem względem prymitywu/meshu/bryły, statystyki (max, RMS, % w tolerancji, histogram),
   mapa kolorów, estymacja szumu skanu (kwadryka lokalna leave-one-out; §10: ostrzeżenie, gdy tolerancja < szum).
7. `doc` — `Region` (trwały, z typem i RMS) i `Primitive` jako obiekty pierwszej klasy; generyczne operacje na
   wszystkich rodzajach obiektów; usunięcie meshu usuwa jego regiony.
8. Komendy: `region.create/grow/auto_segment/merge/split/list`, `primitive.fit/info/symmetry`,
   `deviation.compute/clear/noise`. GUI: regiony kolorowane wg typu, łaty prymitywów, mapa odchyłek; wskazany trójkąt
   wypełnia `triangles` (np. `region.grow`).

## Bramki (testy `gate_m2_*`)
- RMS ≤ 0,1 mm na danych syntetycznych z szumem σ = 0,05 mm dla wszystkich 5 rodzajów (`fit::primitive`).
- Poprawny auto-typ dla 5 rodzajów (`fit::auto`), stożek prawie walcowy → walec.
- Region: auto-segmentacja bracketu → 6 płaszczyzn + walec + sfera, każdy region RMS ≤ 0,1 mm (`fit::segment`,
  `engine/tests/primitives.rs`); merge/split.

## Poza M2 (zgodnie z §8)
- Pozostałe operacje na regionach (resegment/insert/separate/enlarge/shrink).
- Auto-constrain relacji między prymitywami — w M5 (solver więzów).
