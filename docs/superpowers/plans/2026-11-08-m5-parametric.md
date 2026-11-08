# M5 — Parametryka: plan

> Źródło: design doc §6.4, §8 (M5), §10 (STEP brył, IGES). Gałąź `m5-parametric`.

## Zakres
1. `sketch` — szkic 2D na płaszczyźnie: punkty, linie, łuki, okręgi; więzy: fix, coincident, horizontal, vertical,
   distance, length, parallel, perpendicular, angle, equal, radius, point-on-line/circle, tangent, midpoint.
   Solver: tłumiony Gauss–Newton (LM) z numerycznym jakobianem; po rozwiązaniu analiza rzędu jakobianu (ortogonalizacja
   wierszy w kolejności więzów): DOF = n − rząd, więzy bez wkładu w rząd = redundantne, redundantne z niezerowym
   residuum = sprzeczne → **„sketch is overdefined”**. Profile zamknięte → krzywe 3D.
2. Przekroje: `geom::section` (płaszczyzna ∩ mesh, łańcuchy po krawędziach) + `sketch::fit2d` (segmentacja po
   krzywiźnie na proste i łuki, podział do tolerancji, auto-więzy H/V) → `sketch.from_section`.
3. `kernel::feature` — extrude (z pochyleniem, otwory odejmowane), revolve, boolean, shell, pattern
   (kopie + transformacja + `union_all`); macierze tłumaczone na konwencję `ops` (wektor-wiersz).
4. Historia: `doc::FeatureObject` (operacja z parametrami, ciało wynikowe, wyłączenie, błąd), `SketchObject`;
   `engine::replay` przelicza cechy po kolei — edycja wymiaru szkicu lub parametru cechy odtwarza wszystko dalej;
   undo/redo przez migawki dokumentu; ciała spoza historii wchodzą jako cecha `base`. `feature.tree` pokazuje DAG
   (zależności przez ostatniego pisarza ciała i szkic).
5. Komendy `sketch.*`, `feature.extrude/revolve/fillet/chamfer/shell/pattern/boolean/edit/suppress/tree/replay`,
   `feature.suggest_fillet` (podpowiedź promienia z meshu — wzorzec Redesign Assistant, §6.4), `body.to_mesh`.
6. STEP brył: `step-export` solidcore (round-trip przez `step_brep::import`). IGES 5.3: **adaptacja** `iges_out`
   SolidCrafta (§10 fallback; propozycja przeniesienia do solidcore wymagałaby działania poza repo — czeka na decyzję
   użytkownika) — encje 186/514, 510/508/504/502, 128/126/110/100/104, nazwy 406.

## Bramki (testy `gate_m5_*`)
- Solver: prostokąt w pełni zdefiniowany (DOF 0, wymiary dokładne); wykrycie więzu redundantnego i sprzecznego.
- Cechy i DAG: extrude → cut otworu → fillet; edycja wymiaru szkicu i parametru cechy przelicza historię (objętości
  dokładne), undo/redo, wyłączenie cechy.
- STEP brył round-trip (objętość i liczba ścian identyczne) + IGES (struktura, round-trip geometrii 128 w teście io).
