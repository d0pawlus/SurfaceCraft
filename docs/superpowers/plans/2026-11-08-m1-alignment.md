# M1 — wyrównanie: plan

> Źródło: design doc §6.1, §8 (M1), §11.1 (progi). Gałąź `m1-alignment`.

## Zakres
1. `align` (L1, obliczenia na meshu wg §5):
   - `kabsch` — Kabsch/Umeyama bez skali (SVD, korekta odbicia, wagi, wykrywanie punktów współliniowych);
   - `icp` — ICP point-to-plane na BVH powierzchni celu (najbliższy punkt na trójkącie, nie wierzchołek),
     przycinanie par (3 × mediana albo `max_distance`), równania normalne 6 × 6 z tłumieniem Levenberga tylko
     dla kierunków ślizgu (np. płaszczyzna), dokładny obrót osiowo-kątowy, kryterium stopu = maksymalne
     przesunięcie próbki między iteracjami < `tolerance` (domyślnie 1e-4 mm, §11.1);
   - `datum` — 3-2-1: primary → +Z, secondary → +X (Gram-Schmidt), początek z trzech niezależnych więzów
     położenia w kolejności datumów (płaszczyzna 1, oś 2, punkt 3);
   - `principal` — osie główne powierzchni (dokładny moment drugiego rzędu trójkątów), znak z trzeciego momentu.
2. `fit::plane` — płaszczyzna PCA (potrzebna do datumów z trójkątów; reszta fitów w M2).
3. Komendy `align.n_point`, `align.icp`, `align.datum` (datumy jawne albo `plane_from_triangles`),
   `align.principal`, `align.center` (offset dużych współrzędnych); wszystkie z `apply` (tylko policz) i `also`
   (te same ruchy dla innych skanów).
4. GUI: wskazywanie na modelu (ray cast BVH; klik = zastąp, Shift+klik = dodaj), podświetlenie trójkątów,
   panel „Picks”; paleta wypełnia `from`/`points`/`triangles` wskazaniami.

## Decyzje
- ICP do powierzchni (BVH), nie do wierzchołków: na danych bez szumu poprawna poza ma zerowe residuum, więc
  Gauss-Newton zbiega kwadratowo i bramka 1e-6 mm jest osiągalna bez specjalnych trybów.
- Tryb „interaktywny manipulator” z §6.1 pokrywa w M1 `mesh.transform` (z palety i skryptów); gizmo w viewporcie
  odłożone (UX, nie algorytm).

## Bramki (testy `gate_m1_*`)
- Odzyskanie znanej transformacji bez szumu ≤ 1e-6 mm: ICP (`align::icp`, `engine/tests/align.rs`), N-point
  (`kabsch`, komenda), 3-2-1 (`datum`).
- Zbieżność ICP: `converged` przy Δ < 1e-4 mm/iter (raportowane `last_delta`).
