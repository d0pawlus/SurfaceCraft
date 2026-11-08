# SurfaceCraft — spike weryfikacyjny ekosystemu Rust

> Data: 2026-11-08
> Dotyczy: [`2026-11-08-surfacecraft-scan-to-cad-rust-design.md`](./2026-11-08-surfacecraft-scan-to-cad-rust-design.md) (§5, §6.4, §9, §10, §11)
> Cel: rozstrzygnąć harmonogram M2–M5 (kernel B-rep, dostawca STEP/IGES) i zebrać konwencje rodziny dla szkieletu M0.
> Stan źródeł: `tangency-labs/solidcore` @ `b9c02da` (2026-11-08), `tangency-labs/solidcraft` @ `fcd3186` (2026-11-08),
> `storytold/cadcraft` @ `e56cc4b` (2026-11-08); klony tylko-do-odczytu w `C:\spike\` (poza repo).
> Clean-room: z repozytoriów rodziny czytane były wyłącznie manifesty, dokumentacja i sygnatury API; nic nie
> zostało skopiowane do SurfaceCraft. Instalacji QuickSurface/Geomagic nie dotykano.

---

## 0. TL;DR

1. **solidcore ma już całe API potrzebne w §6.4**, ale nie w tym miejscu, które zakłada design doc. Extrude
   (także z pochyleniem), revolve, loft, sweep, split/trim, booleany, fillet/chamfer, shell/offset/thicken/draft
   i stitch są w crate **`solidcore-ops`**. Ten crate jest **tylko w git** (`publish = false`). Na crates.io jest
   wyłącznie sam kernel `solidcore` **1.0.1**, wydany 2026-10-30. Numer wersji odpowiada wersji upstream
   usd-brep, a README nadal mówi o „0.0.x / experimental”. Brakuje dedykowanego **pattern** (robi się go przez
   copy+transform+`union_all`). Brakuje też **dopasowania NURBS do rozproszonych punktów**: jest tylko LSQ na
   regularnej siatce.
2. **STEP nie jest już osobnym światem.** solidcore ma crate'y `step-import`/`step-brep`/`step-export` (git-only),
   które zapisują **bryły** (`MANIFOLD_SOLID_BREP`, analityka + B-spline, AP203/AP214/AP242). Nie zapisują jednak
   **arkuszy** (`SHELL_BASED_SURFACE_MODEL`), a tego potrzebuje M3. SolidCraft ma własny writer (`solidcraft-step`,
   MIT OR Apache-2.0), który zapisuje i bryły, i otwarte powłoki. Ma też **eksport IGES 5.3**
   (`solidcraft-kernel/iges_out.rs`, encje 186/514). Rodzina zapowiada scalenie STEP w jedną implementację w solidcore.
3. **`truck` przestał być opcją rodziny.** SolidCraft przeszedł z truck na solidcore w X 2026: truck zniknął z
   `Cargo.lock`, `truck-stepio` 0.3.0 pochodzi z 2024-09. Przekładanie topologii solidcore na truck tylko po to,
   żeby zapisać STEP, nie ma sensu.
4. **Launcher istnieje.** `storytold/craft-launcher` ma katalog z kategorią *community app* (SolidCraft dodano
   2026-10-28). Wpis wymaga decyzji maintainera. Założenie z §9/§11 („brak publicznego launchera”) jest nieaktualne.
5. **Probe uruchomiony natywnie i w wasm, z identycznymi wynikami** (§1.6). Box+fillet (1 i 12 krawędzi), chamfer,
   shell, extrude (także z pochyleniem), revolve, loft, boolean, „pattern”, fit NURBS + projekcja punktu oraz eksport
   STEP bryły z re-importem działają. Objętości zgadzają się z wartościami dokładnymi do 1e-4. Nie działają dwie
   rzeczy: arkusz z wolnego płata przez kernelowe `face_on` (błąd upstream 1001) i **zapis arkusza do STEP**
   (`step-export` odrzuca `ShellModel`). Arkusz NURBS przez `ops` (otwarty loft) działa.

**Wpływ na harmonogram:** M2–M5 nie wymagają zmiany kolejności. Zmienia się skład zależności: kernel z git
(pin `rev`, jak SolidCraft), STEP z rodziny, a nie z truck. Pojawiają się też dwie nowe decyzje: dopasowanie
punktów rozproszonych w M3 i miejsce obliczeń geometrycznych względem reguły „apps compute no geometry” (§1.5).

---

## 1. solidcore

### 1.1 Wersja i dystrybucja

| Fakt | Wartość | Dowód |
|---|---|---|
| Wersja na crates.io | `solidcore` **1.0.1** (2026-10-30); wcześniej 0.0.1 (2026-10-13) | [crates.io/crates/solidcore](https://crates.io/crates/solidcore) (API `versions`) |
| Co jest publikowane | tylko `solidcore` (kernel). README: „Only `solidcore` is published” | `README.md` → „Releasing”; `crates/*/Cargo.toml` → `publish = false` |
| Crate'y tylko w git | `solidcore-ops`, `solidcore-record`, `solidcore-convert`, `solidcore-tess`, `step-import`, `step-brep`, `step-export`, `ssi`, `solidcore-ssi`, `nurbs-grid-mesh` | [Cargo.toml workspace](https://github.com/tangency-labs/solidcore/blob/main/Cargo.toml); crates.io: `solidcore-ops`/`solidcore-record`/`step-export` → „does not exist” |
| Jak konsumuje to SolidCraft | git + pin `rev = "a445bdaa…"` (commit z 2026-11-07, dzień stary) dla `solidcore` i `solidcore-record`; `deny.toml` → `allow-git = ["https://github.com/tangency-labs/solidcore"]` | [solidcraft/Cargo.toml](https://github.com/tangency-labs/solidcraft/blob/main/Cargo.toml), `deny.toml` |
| Toolchain | `rust-toolchain.toml` → 1.98.0 + target `wasm32-unknown-unknown`; MSRV workspace 1.85 | `rust-toolchain.toml` |
| Licencja | Apache-2.0 (zgodna z MIT OR Apache-2.0 SurfaceCraft) | `LICENSE`, `NOTICE`, `ATTRIBUTION.md` |
| Rozbieżność | README: „API is experimental (0.0.x)… stays at 0.0.x until usable end to end”, a opublikowano 1.0.1. **Numer wersji nie oznacza stabilnego API.** | [README](https://github.com/tangency-labs/solidcore#status) |

**Wniosek:** SurfaceCraft musi zależeć od solidcore **z git, z pinem `rev`**, tak jak SolidCraft. Wersja z crates.io
nie zawiera warstwy `ops`/`record`/`tess`/`step`, a według [docs/layers.md](https://github.com/tangency-labs/solidcore/blob/main/docs/layers.md)
§3 aplikacje mają wołać właśnie `ops`/`record`/`tess`, nigdy `solidcore` wprost.

### 1.2 Status per-unit

Źródło: [PROGRESS.md](https://tangency-labs.github.io/solidcore/PROGRESS.md) i
[strona postępu](https://tangency-labs.github.io/solidcore/), regenerowane przy każdym merge (stan 2026-11-08).

- **281/282 unitów przeniesionych** (254 green, 27 partial, 1 blocked), 99,3% linii upstream usd-brep v1.0.1.
  Solids: **9/9 prymitywów przechodzi**. Strona zastrzega, że „green/partial” to *historyczny* status planu portu,
  a nie werdykt CI. Golden dowodzi niezmienności wyniku, nie jego poprawności.
- **Partial w obszarach ważnych dla SurfaceCraft:** `boolean.api_brep`, `offset.offset_executive_p1/p2`,
  `offset.offset_geometry_creation` (shell/offset), `sweep.api_brep`, `primitive.api_trim_ops` (trim),
  `topology.api_projection`, `topology.brep_planar_section` (przekroje dla M5!), `blend.fillet_corner_p2/p3`,
  `blend.fillet_geom_p4`, `blend.fillet_executive_p3`, `surface.torus`, `surface.plane_p1`, `mass.integrator`.
- **Blocked:** `blend.feature_executive` (3005 linii).
- Ostatnia migawka z odroczonymi przypadkami ([plan/STATUS.md](https://github.com/tangency-labs/solidcore/blob/main/plan/STATUS.md),
  CI z 2026-10-23): 554 873 przypadki, 0 fail, **4186 deferred** (trafiają w niepodpięte stuby między unitami).
  Najwięcej blokują `topology.face_p2/p4`, `topology.edge` i `intersect.curve_class_p4`. Od 2026-10-27 kernel
  porównuje się z własnymi goldenami, a nie z C++.
- CI na `main` (`b9c02da`) zielone ([Actions](https://github.com/tangency-labs/solidcore/actions)).

### 1.3 Pokrycie operacji z §6.4

Publiczne API aplikacji to `solidcore-ops` ([src/lib.rs](https://github.com/tangency-labs/solidcore/blob/main/crates/solidcore-ops/src/lib.rs)).

| Operacja (§6.4) | Jest? | Gdzie / sygnatura |
|---|---|---|
| Prymitywy | ✅ | `make_box/cylinder/sphere/cone/torus`; rekord ma **analityczne** `Plane/Cylinder/Cone/Sphere/Torus` (wynik fitu z M2 można zapisać dokładnie) |
| **Extrude** | ✅ | `sweep::extrude(profile, direction, distance, draft, DraftCorner, cap)`, także z pochyleniem (draft) i trzema typami narożników |
| **Revolve** | ✅ | `sweep::revolve(profile, origin, axis, angle, cap)` (`SmApiCreateRotationalSweep`) |
| **Loft** | ✅ | `sweep::loft(sections, degree 1..=3, cap)` (`SmApiCreateSkinPrimitive`), `loft_to_point` |
| Sweep | ✅ | `sweep_placed`, `sweep_stations`, `screw_sweep` (gwinty/helisa) |
| **Trim / cut** | ✅ (częściowo) | `split::split_body`, `split_body_by_sheet`, `split_faces`; `untrim::untrim_face`; `extend::extend_sheet`. Unit `primitive.api_trim_ops` jest partial |
| **Pattern** | ❌ (brak dedykowanego) | robi się go przez `Body::copy` + `Body::transform(4×4)` + `boolean::union_all`. To recepta po stronie aplikacji (w SolidCraft też) |
| Booleany | ✅ | `boolean::boolean(a,b,BooleanOp)`, `union_all`, `boolean2d`, `imprint` |
| Fillet / chamfer | ✅ | `blend(body, Blend::{Round,Curvature,Chord,Chamfer}, edges, sizes, setbacks, Corners)`, `fillet_variable`, `chamfer_sides`, setbacki, raport `TooLarge` z maksymalnym rozmiarem |
| Shell / offset | ✅ | `offset::{shell, offset_faces, offset_sheet, thicken, draft}`. **`thicken` pokrywa „thicken→bryła” z M4** |
| Stitch / heal | ✅ | `stitch::{stitch, merge_faces}`; kernel `heal` |
| Prism/przekroje | ✅ | `prism::{extrude_planar_regions, prism_section, planar_regions_boolean}`, `sweep::face_section`, `project_curves` |
| Pomiary | ✅ | `measure::*` (mass properties, area, bbox, point class, distances, gap samples), `Body::volume` |
| Serializacja | ✅ | `Body::to_record`/`from_record` (serde, `solidcore-record`), `.smb` (`file`) |

### 1.4 NURBS: jakie API

| Potrzeba | Jest? | API |
|---|---|---|
| Płat z punktów kontrolnych | ✅ | kernel: `nurbs::srf_basic::Surface::new(net: Vec<Vec<CPoint>>, p, q, u, v)`; rekord: `Surface::Spline {u_knots, v_knots, points}` / `RationalSpline {homogeneous}` |
| Fit do **siatki regularnej** | ✅ | `nurbs::srf_fit::{lst_sq_approx(pts, n, m, p, q, ParamMethod), interp_params, interp_given_knots, param_values}` (port `N_FitSrfLstSqApprox`) |
| Fit do **chmury rozproszonej** (Fit Surface z M3) | ❌ | brak. Trzeba resamplować region do siatki (parametryzacja) albo dopisać LSQ z rozproszonymi parametrami (`faer`) |
| Ewaluacja | ✅ | `Surface::eval(u, v, Side, Side)`, `srf_adv::eval_pt_derivs_unbounded` (pochodne), `surface::evaluate_1st/2nd_derivatives`, normalne |
| **Projekcja punktu na powierzchnię** | ✅ | `query::api_queries::surface_closest_point(k, &Surface, Point3) -> {closest{point,distance}, uvs}` (`SmApiSurfaceClosestPoint`); dla B-rep: `face_closest_point`, `brep_closest_point(_all)`, `brep_distance`. **To gotowy rdzeń mapy odchyłek dla płatów** |
| Arkusz (sheet body) z płata | ✅ (kernel) | `topology::brep::sheet::face_on(k, brep, surface, uv_box)`. W `ops` nie ma takiej funkcji; aplikacja buduje rekord `Brep` z jedną ścianą `Spline` i woła `Body::from_record` |
| Krzywe | ✅ | `nurbs::crv_fit`, `primitive::api_curves::{create_curve_interp_points, create_curve_approx_points}`, `query::curve_closest_point` |
| Import IGES NURBS | ✅ (tylko geometria) | `nurbs::iges::{iges_126_curve, iges_128_surface}`: budowa krzywej i płata z encji 126/128. **To nie jest reader ani writer plików** |
| Tessellacja do wyświetlania | ✅ | `solidcore-tess` („the one mesher”, siatka w (u,v), LOD), `nurbs-grid-mesh` |

Uwaga do wygody API: warstwa NURBS to wierny port C (indeksy `n,m,p,q`, `Side::Left/Right`, `CPoint`).
Konstrukcja `surface::Surface` z NURBS wymaga przejścia
`Surface::new(SurfaceKind::BSpline(BSplineSurface::from_nurb(nurb, false)))`. Trzeba się liczyć z cienkim adapterem w `kernel`.

### 1.5 Reguła warstw solidcore a architektura SurfaceCraft

[docs/layers.md](https://github.com/tangency-labs/solidcore/blob/main/docs/layers.md) ustala, że kernel „computes
everything” (także *fit*: interpolację, aproksymację, constrained fits), a **aplikacje nie liczą geometrii**.
Egzekwuje to shrink-only allowlist `cargo xtask app-geometry`, a w SolidCraft `mesh-decisions`. `ops` ma być
„the modelling API **both apps** call”, a §6 tego dokumentu planuje w `ops` „surface-modelling workflows (skin, align,
continuity-check, CV-curve editing, combs)”. Rodzina buduje więc warstwę powierzchniową, z której SurfaceCraft mógłby korzystać.

Konflikt z §5 design doc: warstwy `fit`/`align`/`deviation` SurfaceCraft liczą geometrię (LSQ, RANSAC, ICP).
Proponowana linia podziału:
- **obliczenia na meshu** (KD-tree, ICP, RANSAC, segmentacja, odchyłka mesh↔mesh) zostają w SurfaceCraft,
  bo to nie jest geometria B-rep;
- **tworzenie i ewaluacja geometrii B-rep/NURBS** (fit płata do punktów, projekcja, konstrukcja ścian) idzie przez
  solidcore. Czego brakuje (fit rozproszony, constrained fit krawędzi pod G1 w M4), to *kandydaci na prymityw kernela*,
  zgłaszani upstream, a do czasu ich wejścia trzymani w `fit` z jawną allowlistą.

Uwaga: SolidCraft **nie używa jeszcze `solidcore-ops`**. W jego `Cargo.lock` są tylko `solidcore` i `solidcore-record`,
a własny wrapper to `solidcraft-smlib`. `ops` jest młodszą, mniej sprawdzoną w produkcji warstwą niż sam kernel.

### 1.6 Build i przykład: wynik

| Krok | Wynik |
|---|---|
| Klon `tangency-labs/solidcore` (`--depth 1`, `b9c02da`) do `C:\spike\solidcore` | ✅ |
| Probe `C:\spike\probe` (path-deps na `solidcore`, `solidcore-ops`, `solidcore-convert`, `solidcore-record`, `step-brep`, `step-export`, `step-import`) | ✅ ≈230 linii; dwie poprawki nazw API przy pierwszej kompilacji (`BooleanOp::Subtract`, `solidcore_convert::brep::to_record`) |
| `cargo build --release` + uruchomienie natywnie (x86_64-pc-windows-gnu, cargo 1.99.1) | ✅ cały probe < 0,6 s |
| `cargo check --lib --target wasm32-unknown-unknown` (cały stos probe, także `ops` i `step-*`) | ✅ bez błędów |
| Build `wasm32-wasip1` z `-zstack-size=8388608` (39,8 MB .wasm, release) i **uruchomienie pod Node 26 WASI** | ✅ ~1 s; **wyniki identyczne z natywnymi** co do cyfry (determinizm); plik STEP różni się o 8 bajtów, najpewniej nagłówkiem (znacznik czasu) |

Wyniki (natywnie = wasm):

| Test | Wynik | Wartość dokładna / ocena |
|---|---|---|
| box 20×10×5 | 6 ścian / 12 krawędzi / 8 wierzchołków, V = 1000 | ✅ |
| fillet `Round r=2`, krawędź 0 | 7/15/10, V = 991,4159 | ✅ = 1000 − (4−π)·10 (krawędź 0 ma 10 mm; etykieta „exact” w probe zakładała 20 mm i jest błędna) |
| fillet `r=1`, wszystkie 12 krawędzi (narożniki kuliste) | 26/48/24, V = 971,2950 | ✅ = 432 + 444·1 + π·116/4 + 4π/3 = 971,2950 (wzór Steinera) |
| chamfer `d=1`, 2 krawędzie stykające się w narożniku | 8/17/11, V = 985,3333 | ✅ = 1000 − 0,5·30 + 1/3 |
| shell `t=1`, otwarta ściana 0 | 11/24/16, V = 694 | ✅ = 22·11·7 − 1000: **grubość dodatnia idzie na zewnątrz**, ściana 0 jest prostopadła do Y |
| extrude 10×4 o 3 | V = 120 | ✅ |
| extrude z pochyleniem 0,05 rad | V = 113,7849 | ✅ (< 120, zwęża się do wewnątrz) |
| revolve pierścienia r 2…4, h 3 | V = 113,0973 | ✅ = 36π |
| loft 10×10 → 6×6, h 8, st. 1 | V = 522,6667 | ✅ = h/3·(100+36+60); etykieta „533,33” w probe była błędna |
| box 20×20×5 − walec r3 | V = 1858,6283 | ✅ = 2000 − 45π |
| „pattern”: 4 walce, `union_all` | V = 201,0619 | ✅ = 64π |
| `lst_sq_approx` siatki 20×20 netem 8×8 st. 3 + `eval` | ok | ✅ |
| odchyłka siatki od płata przez `surface_closest_point` (49 próbek) | max **0,00098 mm** | ✅ mieści się w progu 0,2 mm z §11.1 z dużym zapasem |
| projekcja punktu spoza płata | dist 2,1443 | ✅ wiarygodne |
| arkusz z wolnego płata: kernel `sheet::face_on` | ❌ `Upstream { operation: "topology::brep::p8", code: 1001 }` | ścieżka niezalecana dla aplikacji (omija `ops`); nie diagnozowano dalej |
| arkusz NURBS przez `ops::sweep::loft(…, cap=false)` między dwiema krzywymi `Curve::Nurbs` | ✅ 1/4/4, rekord: 1 ściana `Spline`, 1 powłoka | ✅ ścieżka `ops` działa |
| arkusz → `brep_to_model` → `step_export::write` | ❌ `body #1 is ShellModel; step-export writes MANIFOLD_SOLID_BREP and FACETED_BREP` | **potwierdza lukę M3 z §2.1** |
| box z filletem → rekord → kernel → `brep_to_model` → `write` (AP203) | ✅ 26,8 kB, 1× `MANIFOLD_SOLID_BREP`, 1× `CYLINDRICAL_SURFACE` | ✅ fillet zapisany jako walec analityczny, nie B-spline |
| re-import tego STEP (`step_brep::import`) | ✅ 1 bryła | ✅ round-trip |

Wnioski dla M3: aplikacja nie może budować arkusza z „gołego” płata przez `ops`, bo `ops` nie ma takiej funkcji.
Musi zbudować rekord `Brep` z jedną ścianą `Spline` i 4 krawędziami (`Body::from_record`) albo dostać nową operację
`ops` („sheet from surface”), którą warto zgłosić upstream razem z PR `SHELL_BASED_SURFACE_MODEL`. Ścieżki przez rekord
nie testowano: ręczne składanie rekordu (ściany, pętle, krawędzie, pcurves) to zadanie dla M3.

Dowody uzupełniające (z repozytoriów):
- **wasm:** CI solidcore uruchamia `cargo check -p solidcore --target wasm32-unknown-unknown`
  ([ci.yml](https://github.com/tangency-labs/solidcore/blob/main/.github/workflows/ci.yml) l. 230–231; także
  `release.yml`). Sprawdzany jest **tylko crate `solidcore`**: `solidcore-ops`, `-record`, `-tess` i `step-*` nie są
  objęte kontrolą wasm w repo solidcore (probe sprawdził je sam, patrz tabela wyżej). SolidCraft sprawdza wasm dla wszystkich crate'ów warstw + `solidcraft-web`
  (`cargo xtask wasm`), więc `solidcore`+`solidcore-record` budują się na wasm w realnej aplikacji (publikowana jako
  `tangency-labs/solidcraft-app`). README wymaga `-zstack-size=8388608` (8 MiB) u konsumenta wasm.
- **Operacje:** `solidcore-ops` ma rig przypadków (`cargo test -p solidcore-ops --test kernel_cases`) w CI.
  SolidCraft 0.40.0-rc.3 produkcyjnie robi extrude/revolve/booleany/fillety na kernelu solidcore.

---

## 2. STEP / IGES

### 2.1 Co jest w rodzinie

| Ścieżka | Licencja / dystrybucja | Zapis brył | Zapis powierzchni (arkuszy) | Schematy | IGES |
|---|---|---|---|---|---|
| solidcore `step-brep::brep_to_model` → `step-export::write*` | Apache-2.0, **git-only** | ✅ `MANIFOLD_SOLID_BREP`, `FACETED_BREP`; płaszczyzna/walec/stożek/kula/torus/revolution/extrusion/**B-spline**; pcurves, `SEAM_CURVE` | ❌ `brep_to_model` oznacza arkusz jako `BodyKind::ShellModel`, ale `write()` go odrzuca („step-export writes MANIFOLD_SOLID_BREP and FACETED_BREP”) | AP203 (domyślnie), AP214, AP242 (`write_with_metadata`) | ❌ |
| solidcore `step-import`/`step-brep::import` | jw. | reader AP203/AP214 (clean-room) → `Brep`, z heal | — | — | — |
| SolidCraft `solidcraft-step` (`write::topology`) + `solidcraft-kernel::step_out` | MIT OR Apache-2.0, git-only | ✅ zamknięte powłoki → bryły | ✅ **otwarte powłoki → `SHELL_BASED_SURFACE_MODEL`/`OPEN_SHELL`** | AP242 + struktura produktu, złożenia, kolory | — |
| SolidCraft `solidcraft-kernel::iges_out` | MIT OR Apache-2.0, git-only | ✅ IGES 5.3 encja **186** (manifold solid) | ✅ encja **514 form 2** (open shell) | — | ✅ eksport (analityka, revolution, rational B-spline, nazwy 406, kolory 314); import `iges_in` |
| `truck-stepio` 0.3.0 | Apache-2.0, crates.io, **2024-09-20** | tylko kształty truck | tylko truck | AP203/214 | ❌ |

Źródła: [step-export/src/lib.rs](https://github.com/tangency-labs/solidcore/blob/main/crates/step-export/src/lib.rs)
(doc-komentarz modułu, l. 1265–1290), [step-brep/src/brep_to_model.rs](https://github.com/tangency-labs/solidcore/blob/main/crates/step-brep/src/brep_to_model.rs)
(l. 470–520), [solidcraft crates/step](https://github.com/tangency-labs/solidcraft/tree/main/crates/step),
[crates/kernel/src/iges_out.rs](https://github.com/tangency-labs/solidcraft/blob/main/crates/kernel/src/iges_out.rs),
[crates/kernel/src/step_out.rs](https://github.com/tangency-labs/solidcraft/blob/main/crates/kernel/src/step_out.rs),
[crates.io/crates/truck-stepio](https://crates.io/crates/truck-stepio).

**Kierunek rodziny:** [docs/layers.md](https://github.com/tangency-labs/solidcore/blob/main/docs/layers.md) §2:
„There is one STEP implementation… SolidCraft's `crates/step` and solidcore's `step-*` crates merge into it”.
§6 punkt 5 to „The STEP merge across record and ops”. Docelowo STEP ma być w `solidcore-record` + `solidcore-ops`.

**Czy topologię solidcore da się przełożyć na writer?** Tak, i to bez tłumaczenia. Oba writery czytają
**ten sam neutralny rekord** (`solidcore_record::brep::Brep`, schema 4), który daje `ops::Body::to_record()`. Ścieżka
solidcore'owa idzie z kernelowego `Brep` przez `brep_to_model`. Rekord ma też warianty opisane jako „truck's `Plane`”,
„truck's `RevolutedCurve`” (ślad migracji), ale truck nie jest już zależnością.

### 2.2 Rekomendacja dostawcy

- **M3 (STEP powierzchni):** zapis przez rekord (`Body::to_record`). Preferowany wariant: **mały PR upstream do
  `solidcore/step-export`** dodający `SHELL_BASED_SURFACE_MODEL`/`OPEN_SHELL` dla `BodyKind::ShellModel`. Model,
  `brep_to_model` i geometria B-spline już to obsługują, brakuje tylko gałęzi w writerze. To jest zgodne z „one STEP
  implementation” w solidcore. **Fallback:** `solidcraft-step` z git (pin rev), który już pisze otwarte powłoki.
  Warunek: musi być skompilowany z **tym samym rev `solidcore-record`** (pin SurfaceCraft = pin SolidCraft albo
  `[patch]`). Truck odpada.
- **M5 (STEP brył):** `step-brep::brep_to_model` + `step-export::write_with_metadata` (AP214/AP242) z solidcore;
  round-trip testowany przez `step_brep::import`. Po scaleniu STEP w `ops` przejść na API `ops`.
- **M5 (IGES):** w solidcore nie ma writera. Opcje, od preferowanej:
  (a) **zgłosić/zrobić przeniesienie `iges_out` SolidCrafta do solidcore** (zgodne z „one implementation per job”);
  (b) zależność git od `solidcraft-kernel` **odrzucić** (ciężki crate, ciągnie cały kernel SolidCrafta i własny pin solidcore);
  (c) własny writer IGES 5.3 nad `solidcore-record` (~500–800 linii, ze specyfikacji publicznej; SolidCraft ma 532 linie),
  z testem „write → `iges_in`/inny reader”. Adaptacja kodu SolidCrafta (MIT OR Apache-2.0) jest licencyjnie dozwolona
  z wpisem w `NOTICE`/`ATTRIBUTION.md`, bo tak robi sam SolidCraft z kodem CADCraft/GridCraft. Wymaga jednak świadomej
  decyzji, bo zasada z briefu mówi „konwencja, nie kod”.

---

## 3. Konwencje rodziny Crafting Apps → szkielet M0

Lista do przeniesienia **jako konwencja** (bez kodu, bez brandingu ArtCraft). Źródła:
[solidcraft](https://github.com/tangency-labs/solidcraft) (`Cargo.toml`, `AGENTS.md`, `xtask/src/{main,layers}.rs`,
`docs/mcp.md`, `crates/engine/src/cmd/mod.rs`), [cadcraft](https://github.com/storytold/cadcraft) (`AGENTS.md`, `Cargo.toml`),
[craft-contrib](https://github.com/storytold/craft-contrib) (`standards/house-rules-core.md`, `house-rules-rust.md`).

### 3.1 Workspace i nazwy
1. `[workspace] resolver = "3"`, `members = ["crates/*", "apps/*", "xtask"]`. `[workspace.package]`: `edition = "2024"`,
   `rust-version = "1.95"`, `license = "MIT OR Apache-2.0"`, `authors = ["SurfaceCraft contributors"]`.
2. Katalogi bez prefiksu, pakiety z prefiksem: `crates/geom` → `surfacecraft-geom` itd.; wszystkie wewnętrzne crate'y
   w `[workspace.dependencies]` jako `path`.
3. Aplikacje: `apps/surfacecraft` (desktop), `apps/surfacecraft-cli` (`run`, `mcp`, eksport, render, `licences`),
   `apps/surfacecraft-web` (eframe web runner, WebGPU z fallbackiem WebGL2).
4. Warstwy jak w solidcraft/cadcraft, zapisane w tabeli w `xtask/src/layers.rs` (np. L0 `geom` → L1 `kernel`/`fit`/
   `align`/`sketch`/`render` → L2 `doc` → L3 `io` → L4 `engine` → L5 `ui-egui`/`gpu`/`mcp` → apps), z jawną listą
   `INTRA_LAYER_ORDER` i listą `UI_CRATES` (`egui`, `eframe`, `egui-wgpu`, `wgpu`, `winit`, `egui_kittest`, `rfd`)
   dozwolonych dopiero od L5. Tylko `crates/kernel` zależy od solidcore (u SolidCrafta: „only the kernel uses it”).
5. Profile: `[profile.dev] opt-level = 1`, `[profile.dev.package."*"] opt-level = 2`, osobny `[profile.ci]`,
   `[profile.release] lto = "thin"`, `debug = "line-tables-only"`.

### 3.2 Bezpieczeństwo i lint
6. `[workspace.lints.rust] unsafe_code = "forbid"`; każdy crate `[lints] workspace = true`; każdy korzeń crate'a
   produkcyjnego `#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo,
   clippy::unreachable)]` (house rule Rust #1).
7. Wejście (pliki mesh, JSON komend, parametry MCP) jest „hostile”: `get()` zamiast indeksowania, limity alokacji
   i rekurencji, test `hostile_params_never_panic`. `Session::execute` pod `catch_unwind` (nie działa na wasm).
8. `deny.toml`: `unknown-git = "deny"`, `allow-git = ["https://github.com/tangency-labs/solidcore"]`; `clippy.toml`, `rustfmt.toml`.

### 3.3 `cargo xtask`
9. `cargo xtask ci` = kolejno: `fmt --check` → `assets` (każdy plik nie-kodowy ma wiersz w `ATTRIBUTION.md`) →
   `licences --check` (allowlista SPDX, zakaz GPL/LGPL/AGPL, generuje `THIRD-PARTY-LICENSES.txt`) → `layers` →
   `clippy --workspace --all-targets --profile ci -- -D warnings` → **wasm check** (`cargo check --target
   wasm32-unknown-unknown --profile ci -p <każdy crate warstwy> -p surfacecraft-web`) → testy (nextest) + doc-testy.
10. Osobne podkomendy: `wasm`, `layers`, `assets`, `licences`, `lists` (posortowane listy), `i18n --check`, później
    `app-geometry` (shrink-only allowlista obliczeń geometrii w aplikacji; por. §1.5). „Run `cargo xtask ci` before every commit”.

### 3.4 Pliki repo
11. `LICENSE-APACHE`, `LICENSE-MIT`, `NOTICE` („SurfaceCraft / Copyright (c) 2026 SurfaceCraft contributors / Licensed
    under MIT or Apache-2.0…” + sekcja „Third-party material”: solidcore Apache-2.0, ewentualne adaptacje z rodziny),
    `ATTRIBUTION.md` (tabela `File | Author | Source | Licence` + polityka: zero zasobów QS/GDX), `THIRD-PARTY-LICENSES.txt`
    (generowany, nie edytować ręcznie), `SECURITY.md`, `DISCLAIMER.md`, `ROADMAP.md`, `CONTRIBUTING.md`.
12. `AGENTS.md` jako **router**: najpierw „Hard rules (what enforces each)”, czyli każda reguła z nazwą sprawdzającego ją
    polecenia xtask, potem tabela „Working on X → read `skills/<app>-<area>/SKILL.md`”. `CLAUDE.md` zawiera tylko `@AGENTS.md`.
    Katalog `skills/` z `SKILL.md` na obszar (contributing, ground-rules, crash-safety, commands, ui, file-formats, tests,
    agent-tools…); serwer MCP serwuje je agentom (`skill_reference`).
13. Dla repo pod `storytold/`: blok house rules (core + rust) kopiowany **dosłownie** między markerami
    `<!-- craft-house-rules:start … sha256=… -->` z `storytold/craft-contrib/standards`, z allowlistą licencji
    (MIT, ISC, BSD-2/3, Apache-2.0, OFL-1.1, CC0-1.0, CC-BY-4.0, CC-BY-SA-4.0, …). Dla repo poza `storytold/`
    (jak `tangency-labs/solidcraft`) wystarczy własny AGENTS w tym samym duchu. **Do decyzji: pod jaką organizacją żyje SurfaceCraft.**
14. Korpusy testowe w osobnym repo (`<app>-corpus`), czcionki z `storytold/craft-fonts`, nigdy duże binaria w repo.
    Gotowe źródło CC0: [tangency-labs/models](https://github.com/tangency-labs/models) (STL/PLY/OBJ/STEP/IGES/3MF), który
    wprost wyklucza ABC/Thingi10K/GrabCAD. Pasuje do poziomu 1 korpusu z §11.1.

### 3.5 Rejestr komend, CLI, MCP
15. `CommandSpec` (solidcraft `crates/engine/src/cmd/mod.rs`): `id` z kropką (`"area.verb"`, np. `solid.extrude`,
    `sketch.line`), `label`, `tab`/`panel` (toolbar), `icon` (rysowana w kodzie), `shortcut`, `params` (jednolinijkowa
    dokumentacja), `enabled: fn(&Session)->Result<(),String>`, `run: fn(&mut Session, &serde_json::Value) -> Result<Value>`,
    `undoable`, `live` (klatka gestu), `reads_model`, `planned(issue)`, `contract` (schemat wejścia `schemars`, JSON Schema
    2020-12) i `output` (schemat wyniku). Konstrukcja `const fn` w stylu builder.
16. Skrypt JSON: `{"commands": [{"command": "<id>", "params": {…}}, …]}`. Te same komendy obsługują toolbar, paletę,
    skrypty, CLI, control channel i MCP. **Wywołania programowe nigdy nie otwierają dialogów.**
17. MCP: **własna implementacja** JSON-RPC 2.0 po stdio (linia = wiadomość), protokoły 2025-06-18/2025-03-26/2024-11-05;
    crate `mcp` zależy tylko od `engine` + `serde_json` (bez `rmcp`). Narzędzia: `list_commands` (filtrowanie, paginacja,
    `compact`), `describe_command`, `execute`, `batch` (rollback, `expect` z objętością/bbox/liczbą brył), `set_parameter`,
    `measure`, `screenshot`, `inspect_design`, `undo`/`redo`, `export`/`save`/`open`. Zasoby `<app>://design`, `<app>://commands`.
18. Tryb „bridged”: aplikacja desktopowa słucha na `127.0.0.1` (SolidCraft domyślnie `27878`, CADCraft `--control PORT`),
    JSON lines (`{"id":1,"method":"engine.execute","params":{…}}`, `ui.screenshot`), token sesji (`getrandom`).
    `surfacecraft-cli mcp` szuka działającej aplikacji, a bez niej startuje headless.

### 3.6 Render i UI
19. **egui 0.37 / eframe 0.37 / egui-wgpu 0.37 / wgpu 30 (+ naga 30)**, `rfd 0.17`, `web-time 1`, `pollster 0.4`.
    W rodzinie **nie ma `three-d` ani `rend3`**: SolidCraft rysuje viewport własnymi pipeline'ami `wgpu` w crate `gpu`
    (callback egui-wgpu), a tessellacja dla wyświetlania należy do kernela (`solidcore-tess`). SolidCraft
    vendoruje załatane `egui-wgpu` i `winit` (`[patch.crates-io]`); tego nie przenosić bez potrzeby.
20. Jeden zewnętrzny kontener UI, jedna ścieżka kamery, kolory z tokenów motywu (`cargo xtask ui-containers`);
    teksty UI w katalogach i18n (`assets/locales/*.json`; wspólne TSV w `storytold/craft-i18n`).
21. Opcjonalnie wspólne biblioteki `storytold/craft-libs` (`craft-ui`, `craft-window`, `craft-update`; Apache-2.0,
    pin git rev) używane przez CADCraft.

### 3.7 Listing w rodzinie
22. `storytold/craft-launcher` ([AGENTS.md](https://github.com/storytold/craft-launcher/blob/main/AGENTS.md),
    `crates/core/src/catalog.rs`) ma `AppKind::Community`: własny `bundle` (np. `io.tangencylabs.solidcraft`), własne
    repo z wydaniami i `SHA256SUMS.txt`, a na macOS podpis Developer ID. **Dodanie aplikacji to decyzja maintainera**
    („Agents never land or approve a third-party app”). Do M0 wystarczy przygotować wydania zgodne z tym kontraktem
    (nazwy assetów, sumy SHA256) i własne bundle id.

---

## 4. Ryzyka z §11 → status

| Ryzyko (§11) | Status | Uzasadnienie |
|---|---|---|
| Dojrzałość `solidcore` (ryzyko #1), w tym „niepotwierdzone extrude/revolve/loft/trim/pattern” | **częściowo obalone / nadal otwarte** | API istnieje dla extrude/revolve/loft/sweep/split/booleanów/filletów/shell/thicken (§1.3); pattern jest recepturą aplikacji. Otwarte: 27 partial + 1 blocked unit (m.in. offset, trim, planar section), API niestabilne mimo numeru 1.0.1, `ops` nieużywane jeszcze przez SolidCraft. Probe (§1.6) potwierdził poprawne objętości dla 11 operacji natywnie i w wasm; jest to jednak tylko przypadek „happy path” na prostych bryłach, nie realne modele |
| STEP/IGES to osobny workstream (solidcore bez translatorów) | **częściowo obalone** | STEP brył: jest w solidcore (`step-export`, git-only). STEP arkuszy: brak w solidcore, jest w `solidcraft-step`. IGES: tylko w SolidCraft. Workstream maleje do „integracja + mały PR + IGES” |
| wasm całego stosu (`truck` + `wgpu`) | **częściowo obalone** | Truck wypada z gry. `solidcore` sprawdzany na wasm w CI; pełny stos egui 0.37/wgpu 30 + solidcore działa w `solidcraft-web`. Probe z `ops`+`step-*` kompiluje się na `wasm32-unknown-unknown` i daje na `wasm32-wasip1` wyniki identyczne z natywnymi. Otwarte: brak kontroli wasm tych crate'ów w CI solidcore, rozmiar binarki (~40 MB nieoptymalizowanego .wasm), limity pamięci przeglądarki dla dużych skanów |
| Brak publicznego API launchera ArtCraft | **obalone** | `storytold/craft-launcher` (publiczny, Apache-2.0) z kategorią community app; SolidCraft wpisany 2026-10-28. Wpis wymaga zgody maintainera, API pluginu nadal brak |
| Solver więzów + pełne drzewo cech (M5) | **nadal otwarte** | poza zakresem spike'u; `solidcraft-sketch` (MIT OR Apache-2.0) to publiczna referencja konwencji |
| AutoSurface kosztowny obliczeniowo (M4) | **nadal otwarte** | poza zakresem spike'u |
| *(nowe)* Zależność git-only + pin `rev` + przenoszenie jobów między crate'ami solidcore (scalanie STEP, `app-geometry`) | **nowe ryzyko** | wymaga regularnego podbijania pinu (SolidCraft robi to co ~dzień) i testów kontraktowych na granicy `kernel` |
| *(nowe)* Reguła „apps compute no geometry” vs warstwy `fit`/`align`/`deviation` | **nowe ryzyko/decyzja** | §1.5 |
| *(nowe)* Brak fitu NURBS do punktów rozproszonych | **nowa luka M3** | resampling do siatki albo własny LSQ (`faer`), a docelowo prymityw kernela upstream |

---

## 5. Proponowane zmiany w design doc (do decyzji, nie wprowadzone)

1. **§1, §9, §11:** zastąpić „0.0.x, experimental” stanem rzeczywistym: crates.io `solidcore` 1.0.1 (numer = upstream
   usd-brep), README nadal deklaruje eksperymentalne API; **zależność z git z pinem `rev`**, bo `ops`/`record`/`tess`/`step-*` nie są publikowane.
2. **§5 `kernel`:** adapter na **`solidcore-ops` + `solidcore-record` + `solidcore-tess`** (nie bezpośrednio na `solidcore`),
   zgodnie z [docs/layers.md](https://github.com/tangency-labs/solidcore/blob/main/docs/layers.md) §3. Render B-rep przez `solidcore-tess`.
3. **§5:** dopisać linię podziału obliczeń (mesh-side w SurfaceCraft, B-rep/NURBS w solidcore) i allowlistę
   `app-geometry` w xtask (§1.5).
4. **§5 render / §10:** usunąć `three-d`/`rend3`; wybrać **surowe `wgpu` 30 przez egui-wgpu 0.37** (jak SolidCraft). Decyzja z §10 zamknięta.
5. **§6.3/§6.4/§10 „NURBS: truck vs solidcore”:** zamknąć na **solidcore** (płat z punktów kontrolnych, `lst_sq_approx`,
   ewaluacja, `surface_closest_point`). Usunąć `truck` z zależności i ryzyk. Dopisać lukę: fit do punktów rozproszonych.
6. **§6.4:** extrude/revolve/loft/trim potwierdzone w `ops`; pattern jako recepta copy+transform+`union_all` w `engine`.
   Dopisać `offset::thicken` jako dostawcę „thicken→bryła” w M4.
7. **§10 „Dostawca STEP/IGES”:** M3 to PR `SHELL_BASED_SURFACE_MODEL` do `solidcore/step-export` (luka potwierdzona
   przebiegiem probe) plus arkusz z płata przez rekord albo nowa operacja `ops` „sheet from surface” (fallback: `solidcraft-step`
   z tym samym pinem `solidcore-record`); M5 STEP to `step-brep`+`step-export` (AP214/AP242); M5 IGES to przeniesienie
   `iges_out` do solidcore albo własny writer IGES 5.3 nad rekordem.
8. **§9/§11 launcher:** zastąpić „brak publicznego launchera” informacją o `storytold/craft-launcher` (community app,
   decyzja maintainera, kontrakt wydań: SHA256SUMS, bundle id, podpis macOS).
9. **§9 konwencje:** dopisać listę z §3 (edition 2024, MSRV 1.95, `cargo xtask ci` z wasm-check i licences, AGENTS-router +
   `skills/`, `CommandSpec` ze schematami, MCP bez `rmcp`) oraz decyzję o organizacji (storytold → house rules z craft-contrib).
10. **§11.1 korpus:** dodać [tangency-labs/models](https://github.com/tangency-labs/models) (CC0) jako źródło korpusu poziomu 1.
11. **§11:** dopisać trzy nowe ryzyka z tabeli w §4.
12. **§8 M0:** dodać do M0 „pin solidcore rev + `deny.toml` allow-git + smoke test granicy `kernel` (box→fillet→to_record)”.
    Bez tego M2 nie ma czego weryfikować.

---

## 6. Odtworzenie probe

Probe nie trafia do repo SurfaceCraft. Źródło: `C:\spike\probe\src\lib.rs` (path-deps na `../solidcore` @ `b9c02da`),
kopia w folderze notatek sesji. Uruchomienie:

```bash
cd /c/spike/probe && cargo run --release --bin probe
```

```bash
cd /c/spike/probe && cargo check --lib --release --target wasm32-unknown-unknown
```

```bash
cd /c/spike/probe && RUSTFLAGS="-C link-arg=-zstack-size=8388608" cargo build --release --target wasm32-wasip1 --bin probe
```

Plik `.wasm` uruchamiany pod `node:wasi` (preview1), skryptem z ~8 linii. Następne kroki poza spike'iem: diagnoza błędu
1001 w `face_on` i budowa arkusza z płata przez rekord (M3).
