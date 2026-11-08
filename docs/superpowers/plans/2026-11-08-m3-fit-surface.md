# M3 — Fit Surface: plan

> Źródło: design doc §6.3, §8 (M3), §10 (STEP powierzchni: adaptacja z `solidcraft-step`), §11.1. Gałąź `m3-fit-surface`.

## Zakres
1. `kernel::nurbs` — fit siatki regularnej przez `srf_fit::lst_sq_approx` (solidcore), projekcja punktu
   `surface_closest_point`, ewaluacja; **arkusz jako rekord `Brep`** z jedną ścianą `Spline` i czterema krawędziami
   izoparametrycznymi (dokładne: brzegi zaciśniętego B-splajnu to skrajne rzędy siatki kontrolnej), pcurves liniowe,
   pętla CCW w (u, v); rekord przechodzi `validate()` i `Body::from_record` (spike §1.6: `face_on` zawodzi, `ops` nie ma
   „sheet from surface”).
2. `fit::resample` — luka „fit do punktów rozproszonych” (§6.3/§10) zamknięta **resamplingiem** po stronie meshu:
   parametryzacja regionu nad płaszczyzną PCA, wysokości węzłów siatki z ważonej kwadryki MLS (k = 24), wykrywanie
   regionu zawiniętego. NURBS buduje kernel, więc `app-geometry` pozostaje pusta (decyzja z uzasadnieniem: żadna
   geometria NURBS nie powstaje po stronie aplikacji).
3. `surface.fit` — siatka kontrolna rośnie (6, 8, 11, 15, 20, 27, 36) aż max odchyłka punktów regionu (mierzona przez
   kernel) ≤ tolerancja (domyślnie 0,2 mm, §11.1); `surface.info`; `deviation.compute` przeciw powierzchni (kernel).
4. STEP powierzchni: `io::step` — **adaptacja** zapisu otwartych powłok z `solidcraft-step` (atrybucja w nagłówkach,
   NOTICE, ATTRIBUTION.md „Adapted code”, dług w STATUS): `SHELL_BASED_SURFACE_MODEL`/`OPEN_SHELL`/`ADVANCED_FACE`,
   B-spline (też wymierne) i płaszczyzny, krawędzie line/B-spline, AP214, mm. Bryły (M5) przez `step-export` solidcore
   (`kernel::step::export_solids`). Import do round-tripu: `step_brep::import` solidcore (`kernel::step::import`).
5. Komendy `export.step`, `import.step`; GUI: menu Surface (paleta), File → Export selected as STEP / Import STEP,
   powierzchnie w viewporcie.

## Bramki (testy `gate_m3_*`)
- Max odchyłka powierzchni ≤ 0,2 mm: fala 60 × 40 mm, amplituda 3 mm, bez szumu i z σ = 0,02 mm.
- STEP powierzchni z round-tripem: zapis → import solidcore → ta sama powierzchnia (< 1e-6 mm) i ta sama odchyłka skanu.

## Uwaga o szumie
Max odchyłka punktów zawiera szum skanu; przy σ = 0,05 mm maksimum z tysięcy punktów zbliża się do 4σ = 0,2 mm, więc
bramkę mierzymy bez szumu i przy σ = 0,02 mm (raportujemy oba wyniki), progu nie zmieniając.
