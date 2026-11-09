# P8 — backlog ScanRuler, CAD Correct/Refit, 100 mln+ trójkątów, dług, upstream (plan)

> Zlecenie właściciela z 2026-11-09. Źródła: [analiza ScanRuler](../specs/2026-11-09-scanruler-functional-analysis.md) §6.3
> (wyłącznie clean-room: z opisu zachowania i literatury, bez kodu AGPL i bez narzędzi ScanRuler),
> [design doc](../specs/2026-11-08-surfacecraft-scan-to-cad-rust-design.md) §4 (CAD Correct) i §9 (skala),
> dług z [STATUS](../../STATUS.md).

## Zasady wykonania
- Jeden pakiet = jeden subagent we własnym worktree, gałąź `p8-<id>` z `origin/main`, jeden PR.
- Modele: Haiku 5.5 (effort xhigh) do pakietów wykonawczych, Sonnet 5.5 (effort high) do numeryki, kernela/B-rep
  i zmian przekrojowych. Orkiestrator: plan, przegląd, scalanie, STATUS.
- Agent tylko sprawdza typy (`cargo xtask quick <pakiety>`); pełne `cargo xtask ci` robi GitHub Actions; rzeczy
  możliwe tylko na tej stacji idą do „Do sprawdzenia lokalnie” w PR i do przebiegu integracyjnego
  ([integration.md](../../integration.md)).
- Agent nie edytuje `docs/STATUS.md` (pomiary i dług podaje w opisie PR; STATUS aktualizuje orkiestrator).
- Każda nowa funkcja to komenda `area.verb` ze schematem i testem bramkowym na danych syntetycznych ze znaną prawdą.
- Progi §11.1 bez zmian. Brak nowych zależności GPL/LGPL/AGPL.

## Pakiety

| Id | Zakres | Główne pliki | Model | Fala |
|---|---|---|---|---|
| U1 | solidcore: podbicie pinu do HEAD, ponowienie reprodukcji, nowe reprodukcje (unia ≥ 3 kopii obróconych, `from_record` po shell, narzędzie równoległe do normalnej), szkice zgłoszeń; wymiana adaptacji STEP/IGES, jeśli upstream już to ma | `Cargo.toml`, `crates/kernel`, `docs/upstream` | Sonnet | 1 |
| A1 | Odchyłki v2 (rdzeń): mapa ze znakiem wg orientacji, zakres z percentyli, histogram, pasma, hotspoty, błąd objętości, odchyłka względem jednego prymitywu | `crates/deviation`, `engine/cmd/deviation.rs` | Haiku | 1 |
| A2 | Lokalne dokładne dopasowanie (ICP na zaznaczonym regionie) | `crates/align`, `engine/cmd/align.rs` | Haiku | 1 |
| A3 | Panel odchyłek: histogram, pasma, odczyt pod kursorem i przypięty, lista hotspotów | `ui-egui/deviation_panel.rs` | Haiku | 2 (po A1) |
| B1 | Auto-więzy: styczność, symetria, równe odległości; raport σ przed/po | `engine/cmd/primitive.rs`, `crates/fit` | Sonnet | 1 |
| B2 | Planer brył: profil z kątem wklęsłym, dwa łuki obok siebie, kieszeń z wyspą, stopnie i żebra | `engine/cmd/plan.rs`, `kernel/local.rs` | Sonnet | 2 |
| C1 | Łańcuchy przekroju → linie i łuki z wywnioskowanymi relacjami i przyciąganiem do osi | `crates/sketch/fit2d.rs`, `engine/cmd/sketch.rs` | Sonnet | 1 |
| C2 | Stos przekrojów (pochylenie, zbieżność, trend promienia) i eksport arkusza DXF/SVG | `geom/section.rs`, `crates/io` (nowe), `engine/cmd/section.rs` (nowy) | Haiku | 1 |
| D1 | Decymacja z mierzoną tolerancją w obu kierunkach i lokalnym dogęszczaniem | `geom/edit.rs`, `engine/cmd/mesh.rs` | Sonnet | 1 |
| D2 | Edycja nieniszcząca: kopia robocza, mapa odległości od oryginału, promote/discard | `crates/doc`, `engine/cmd/mesh.rs` | Haiku | 2 (po D1) |
| E1 | MCP: przewodniki (`guide.read`), kompaktowa i filtrowana lista narzędzi, batch, kolejka zadań | `crates/mcp`, `engine/cmd/app.rs`, `docs/guides` | Haiku | 1 |
| E2 | `scan.qa` (kontrola skanu) i `body.compare` (raport ciało-vs-skan per ściana) | `engine/cmd/qa.rs` (nowy) | Haiku | 2 |
| F1 | CAD Correct/Refit: dopasowanie istniejącej bryły do nowego skanu (parametry cech; ściany analityczne z wagami sztywności) | `engine/cmd/refit.rs` (nowy), `crates/fit`, `crates/kernel` | Sonnet | 1 |
| G1 | Out-of-core: import strumieniowy do magazynu kawałków na dysku, piramida LOD | `crates/geom`, `crates/io/mesh.rs` | Sonnet | 2 (po D1) |
| G2 | Out-of-core w silniku: wyświetlanie z LOD, odchyłki/ICP/szum przez `TriangleSource`/`PointSource` | `crates/engine`, `crates/deviation`, `crates/align` | Sonnet | 3 (po G1) |
| H1 | STEP arkuszy z PCURVE | `crates/io/step` | Sonnet | 2 |
| H2 | Równoległość: QEM, wypełnianie dziur (> 4000 krawędzi), parsowanie STL, `weld` bez sortowania 30 mln kluczy | `geom/edit.rs`, `io/mesh.rs` | Sonnet | 3 (po D1, G1) |
| H3 | Segmentacja skanów organicznych: równoległy wzrost regionów i refity | `fit/segment.rs` | Sonnet | 2 |
| H4 | Fit Surface: przycinanie do konturu regionu, automatyczny podział regionu zawiniętego | `crates/fit`, `kernel/nurbs.rs`, `engine/cmd/surface.rs` | Sonnet | 2 |
| H5 | Regiony: resegment, insert, separate, enlarge, shrink | `engine/cmd/region.rs` | Haiku | 1 |
| H6 | GUI I: pędzel/lasso na BVH, „czapka” przekroju, znacznik pól wskazywanych w schemacie komendy | `ui-egui/{select,sectionview,fieldpick}.rs` | Haiku | 2 |
| H7 | GUI II: przeciąganie punktów szkicu, cache szkiców, ikony SVG paska, znacznik cofania w historii, tłumaczenie etykiet komend | `ui-egui/{sketch_editor,history_panel,i18n,icons}` | Haiku | 3 (po A3, H6) |
| H8 | Cechy: sweep po splajnie, loft z krzywymi prowadzącymi, split z obiema stronami | `kernel/feature.rs`, `engine/cmd/feature.rs` | Sonnet | 3 (po U1) |
| H9 | Retopologia: układ dopasowany do konturu regionu, kontrola odwróconych łat, `surface.from_curves` dla regionów bez 4 rogów | `fit/{retopo,features}.rs` | Sonnet | 3 |
| H10 | Kanał sterowania: wiele instancji, token poza plikiem tymczasowym; web: mniejszy bundle | `ui-egui/control.rs`, `crates/mcp`, profile | Haiku | 3 |

Poza zakresem (decyzje lub zasoby właściciela): podpisywanie wydań, dostawca LLM, fork egui w `craft-ui`, repo korpusu,
GPU w CI, warstwa inspekcji (wymiary, GD&T).

## Upstream
Zgłoszenia idą do `tangency-labs/solidcraft` (repo `tangency-labs/solidcore` to lustro tylko do odczytu), jako praca
wyłącznie nad kernelem: reprodukcja w czystym solidcore, sprawdzona na bieżącym stanie kernela, bez kodu SurfaceCraft.
Już zgłoszone: #13617 (writer IGES), #13618 (thicken arkusza wielościennego), #13619 (arkusz z powierzchni B-spline).
Zgłasza orkiestrator po weryfikacji przez U1 i sprawdzeniu duplikatów.

## Zamknięcie
Przebieg integracyjny na jednym buildzie (korpusy lokalne, wydajność, 100 mln syntetycznych trójkątów, GPU/GUI),
wyniki do STATUS, aktualizacja długu.
