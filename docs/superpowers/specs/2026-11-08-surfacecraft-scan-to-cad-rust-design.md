# SurfaceCraft — Scan-to-CAD w Rust — dokument projektowy

> Status: **ZATWIERDZONY** (2026-11-08) — następny krok: plan implementacji
> Data: 2026-11-08
> Autor: Dominik Pawluś (z Claude)
> Nazwa: **SurfaceCraft** *(zatwierdzona przez użytkownika)*
> Rewizja 2026-11-08 (po przeglądzie): ujednolicona kolejność eksportu (STEP w M3, IGES w M5 — §8/§10/§11),
> `Region` przypisany do M2 (§6.2/§8), korpus testowy rozdzielony na repo/lokalny zgodnie z clean-room (§9/§11.1),
> nota o progach względem szumu skanu (§10, progi zatwierdzone bez zmian).
> Rewizja 2026-11-08 (po spike'u ekosystemu — [raport](./2026-11-08-surfacecraft-ecosystem-spike.md)): solidcore z git
> (pin `rev`) przez `solidcore-ops`/`-record`/`-tess`; STEP z solidcore zamiast `truck` (truck usunięty); render = surowe
> `wgpu` przez egui-wgpu; podział obliczeń mesh↔B-rep; konwencje rodziny do M0; zasada „upstream, potem adaptacja”
> dla kodu rodziny; repo na koncie autora; nowe ryzyka (§1, §5, §6.3, §6.4, §8, §9, §10, §11, §12).

---

## 1. Intencja i „dlaczego"

Samodzielna, otwartoźródłowa aplikacja desktopowa (Rust) do **inżynierii wstecznej skanów 3D** —
zamiany surowej siatki ze skanera (STL/OBJ/PLY) w czystą geometrię CAD. SurfaceCraft ma być
pełnoprawną aplikacją z rodziny **„Crafting Apps"** (ekosystem Tangency Labs / ArtCraft:
`solidcraft`, `cadcraft`, `fieldcraft`, `pcbcraft`), zajmując niszę, której w tej rodzinie nie ma:
**scan → CAD**. SolidCraft *buduje* bryły parametrycznie; SurfaceCraft *odtwarza* je z pomiarów.

Dlaczego teraz: istniejący komercyjny lider tej niszy (QuickSurface, KVS Ltd) jest zamknięty,
oparty na płatnym kernelu Parasolid i MFC, i częściowo deleguje operacje parametryczne do
SolidWorks/Inventor. Pojawienie się czysto-rustowego, otwartego kernela B-rep (`solidcore`,
port NVIDIA SMLib, Apache-2.0, buduje się na `wasm32`) zaczyna umożliwiać zbudowanie w pełni
samodzielnego, otwartego odpowiednika bez komercyjnych zależności. **Zastrzeżenie:** na crates.io jest
`solidcore` 1.0.1, ale numer odpowiada wersji upstream usd-brep, a README nadal deklaruje API eksperymentalne;
warstwa operacji (`solidcore-ops`), rekord, tessellacja i STEP są **tylko w git**. Dojrzałość kernela pozostaje
głównym ryzykiem projektu (patrz §11), a nie faktem dokonanym.

> Materiał źródłowy analizy funkcjonalnej QuickSurface:
> [`2026-11-08-quicksurface-functional-analysis.md`](./2026-11-08-quicksurface-functional-analysis.md).
> Drugi, niezależny materiał porównawczy (Geomagic Design X, Hexagon/Rapidform rodowód):
> [`2026-11-08-geomagic-designx-functional-analysis.md`](./2026-11-08-geomagic-designx-functional-analysis.md) —
> potwierdza większość wzorców z QS niezależnym przykładem i dorzuca kilka nowych (patrz §6.2, §6.3, §6.4, §7).
> Repozytoria rodziny (publiczne): `github.com/tangency-labs/solidcore`, `.../solidcraft`,
> `github.com/storytold/cadcraft`, `.../artcraft`.

Sukces = użytkownik wczytuje „brudny" skan i wychodzi z niego z: (a) wyrównanym meshem w układzie
współrzędnych, (b) wyekstrahowanymi prymitywami, (c) dopasowanymi powierzchniami swobodnymi oraz
(d) **edytowalnym, parametrycznym drzewem cech**, które można wyeksportować do STEP/IGES — wszystko
w jednej aplikacji, bez zewnętrznego CAD-a.

## 2. Zakres: cztery funkcje rdzeniowe (decyzja użytkownika)

Aplikacja ma mieć **wszystkie cztery** (potwierdzone przez użytkownika):

1. **Szybkie wyrównanie siatki** do układu współrzędnych.
2. **Ekstrakcja prymitywów** (płaszczyzna, walec, stożek, kula, torus, płaszczyzna symetrii).
3. **Dopasowanie powierzchni swobodnych** (NURBS / freeform na bazie siatki quadów).
4. **Pełne odtworzenie drzewa cech parametrycznych** (szkic → cechy → historia).

Podstawą merytoryczną jest analiza funkcjonalna QuickSurface, zapisana w repo jako
[`2026-11-08-quicksurface-functional-analysis.md`](./2026-11-08-quicksurface-functional-analysis.md)
(katalog 2890 komend, baza wiedzy wideo, instrukcja obsługi). Z niej pochodzą obserwacje
oznaczone niżej jako „wzorzec z QS". Druga analiza, [`2026-11-08-geomagic-designx-functional-analysis.md`
](./2026-11-08-geomagic-designx-functional-analysis.md) (katalog 457 komend z oficjalnego User Guide,
mapa modułów), dorzuca obserwacje oznaczone jako „wzorzec z GDX" — niezależny, konkurencyjny produkt
potwierdzający te same wzorce i wskazujący kilka dodatkowych wartych rozważenia. **Nie kopiujemy kodu
QuickSurface ani Geomagic Design X** — korzystamy wyłącznie z analizy *zachowania/funkcji* (udokumentowanych
komend, nie zdekompilowanego kodu), zgodnie z zasadami clean-room ekosystemu (patrz §9).

## 3. Cele / nie-cele / anty-cele

**Cele**
- Cztery funkcje z §2 działające na realnych skanach, z mierzalną dokładnością (mapa odchyłek).
- Zgodność z konwencjami rodziny Crafting Apps: czysty Rust, UI w egui, rdzeń sterowany komendami,
  serwer MCP, CLI, build web (wasm), licencja MIT OR Apache-2.0.
- Jeden rdzeń → cztery wejścia: **desktop, web, CLI, MCP** (jak solidcraft/cadcraft).
- Eksport do **STEP i IGES** (oraz STL/OBJ dla meshu) bez zewnętrznego CAD-a.
- Stały feedback dokładności (deviation) przy każdej operacji.

**Nie-cele (świadomie poza tą wersją)**
- Własny kernel B-rep — używamy `solidcore`, nie piszemy swojego.
- Przetwarzanie chmur punktów bez triangulacji (zakładamy mesh na wejściu; triangulacja to osobny temat).
- Edycja/naprawa meshu na poziomie QS (fill holes, remove outliers, decimation) — minimalny zestaw
  w MVP, pełny pakiet później.
- Integracja „push do SolidWorks/Inventor" (QS to ma; my jesteśmy samodzielni — eksport plikowy wystarcza).
- Współbieżna edycja wielu użytkowników, chmura, konta.
- Bezpośrednie sterowanie sprzętem skanera/sondą (GDX ma to jako LiveCapture/Scanner Direct Control —
  dziesiątki sterowników producentów; osobny, duży workstream integracyjny, poza zakresem).
- Komercyjny translator multi-CAD klasy Spatial 3D InterOp/Exchange (GDX go używa obok Parasolid) —
  STEP/IGES plikowy wystarcza, nie gonimy pokrycia formatów na poziomie Parasolid/Spatial.

*(Oba powyższe potwierdzone jako świadomie poza zakresem po analizie GDX — patrz
[analiza GDX](./2026-11-08-geomagic-designx-functional-analysis.md) §8.)*

**Anty-cele (uznamy projekt za nieudany, jeśli)**
- Wciągnięcie komercyjnej/zamkniętej zależności (Parasolid, OCCT jako główny kernel łamiący „pure Rust",
  cokolwiek niekompatybilnego z MIT/Apache).
- Skopiowanie kodu lub zasobów (ikony, teksty, układy) QuickSurface albo brandingu ArtCraft do forka.
- Monolit, w którym logika geometryczna jest wpleciona w UI (łamie warstwowanie rodziny).
- „Czarna skrzynka" bez informacji o odchyłce — użytkownik musi widzieć, jak wierna jest rekonstrukcja.

## 4. Użytkownik i główny przepływ

Użytkownik: inżynier/projektant robiący reverse engineering części (mechanicznych i organicznych)
ze skanu 3D. Główny przepływ (wzorzec z QS, dwuetapowy):

1. **Import meshu** → ewentualne szybkie oczyszczenie.
2. **Ekstrakcja prymitywów** z zaznaczonych regionów.
3. **Wyrównanie** do układu współrzędnych (primary/secondary/tertiary z prymitywów, albo N-point/ICP).
4. **Rekonstrukcja**: dla regionów analitycznych — prymitywy/przekroje → szkic → cechy parametryczne;
   dla regionów organicznych — Fit Surface / freeform.
5. **Kontrola odchyłki** (mapa kolorów, max/RMS) i **eksport** STEP/IGES.

Model **hybrydowy** (analityczne + freeform w jednym modelu) jest celem, nie opcją — to kluczowa
przewaga QS warta powtórzenia.

## 5. Architektura (decyzja Claude, wzorowana na cadcraft/solidcraft)

Warstwy, gdzie „nic poniżej `ui` nie wie o egui" (wymienialny front):

```
geom      — typy geometryczne f64: punkty, wektory, mesh, KD-tree, przekroje, dopasowania LSQ
kernel    — adapter na solidcore-ops + solidcore-record + solidcore-tess (B-rep): prymitywy, extrude/revolve/
            loft/boolean/fillet/shell/thicken, NURBS, tessellacja do wyświetlania; JEDYNY crate zależny od solidcore
fit       — algorytmy dopasowania: prymitywy (LSQ/RANSAC), NURBS, retopo/quad, symmetry
align     — wyrównanie: 3-2-1 datum, Kabsch/Umeyama (N-point), ICP point-to-plane
sketch    — geometria 2D + solver więzów (Newton/LSQ), fit łuków/linii do przekrojów
doc       — model dokumentu: DAG historii cech, parametry, undo/redo, (de)serializacja projektu
engine    — sesje, rejestr komend, snapy, wykonanie komendy (źródło prawdy dla wszystkich wejść)
io        — STEP/IGES (przez rekord solidcore + writery rodziny — patrz §10), STL/OBJ/PLY (mesh), format projektu
deviation — analiza odchyłki mesh↔powierzchnia (KD-tree), dane do mapy kolorów
ui-egui   — desktop + web (wasm) front
cli       — `surfacecraft-cli` (run skryptów JSON, batch, eksport)
mcp       — serwer MCP (`surfacecraft-cli mcp`), sterowanie agentami tymi samymi komendami
ai        — (post-MVP) asystent LLM: tryby „align" i „workflow", wejście = zrzut viewportu
```

**Zasada command-driven (wzorzec z rodziny):** każda akcja UI = komenda wywoływalna także jako JSON
i przez MCP. To automatycznie daje skryptowalność, testowalność i integrację z agentami.

**Podział obliczeń geometrycznych (reguła rodziny „apps compute no geometry”, solidcore `docs/layers.md`):**
- **obliczenia na meshu** (KD-tree, ICP, RANSAC, segmentacja regionów, odchyłka mesh↔mesh) zostają w SurfaceCraft
  (`geom`/`fit`/`align`/`deviation`) — to nie jest geometria B-rep;
- **tworzenie i ewaluacja geometrii B-rep/NURBS** (płat z punktów, projekcja punktu na powierzchnię, konstrukcja
  ścian, operacje bryłowe) idzie przez solidcore (`kernel`). Brakujące prymitywy (np. fit NURBS do punktów
  rozproszonych, constrained fit krawędzi pod G1) zgłaszamy upstream, a do czasu ich wejścia trzymamy w `fit`
  na jawnej allowliście (`cargo xtask app-geometry`, shrink-only).

**Render 3D:** surowe **`wgpu` 30 przez `egui-wgpu` 0.37** (własne pipeline'y viewportu, jak SolidCraft); tessellacja
B-rep do wyświetlania z `solidcore-tess`. Rodzina nie używa `three-d`/`rend3`.

**Matematyka:** `nalgebra` (SVD, macierze, kwaterniony) + `faer` (szybkie LSQ/układy liniowe).
Bez OpenBLAS/Fortran (w przeciwieństwie do QS).

## 6. Mapowanie funkcji na algorytmy i komponenty

Skrót z analiz QS i GDX (pełny materiał: [analiza QS](./2026-11-08-quicksurface-functional-analysis.md),
[analiza GDX](./2026-11-08-geomagic-designx-functional-analysis.md)). „obserwacja" = ze stringów/dokumentacji;
„algorytm" = standard/wniosek.

### 6.1 Wyrównanie
- Tryby: 3-2-1 datum (primary/secondary/tertiary), N-point, interaktywny manipulator, fine-align (ICP),
  align-to-axis, offset chmury przy dużych współrzędnych. *(obserwacja: `Align by Coordinate System`,
  `Align by N points`, `Fine align … minimising the overlapping distance`)*
- Algorytm: Gram-Schmidt dla datum; **Kabsch/Umeyama (SVD)** dla N-point; **ICP point-to-plane** + KD-tree.
- Rust: `align` + `nalgebra` + `kiddo`/`kd-tree`. **Najłatwiejsza — beachhead MVP.**

### 6.2 Ekstrakcja prymitywów
- Typy: plane/cylinder/cone/sphere/torus + symmetry plane; fit „through selected triangles";
  auto-constrain (relacje między prymitywami); auto-detekcja typu po residuum. *(obserwacja:
  `Fit cone through selected triangles`, `Failed to fit cone. It is more suitable to be a cylinder`)*
- Algorytm: PCA (płaszczyzna), nieliniowy LSQ/Levenberg-Marquardt (sfera/walec/stożek/torus);
  wybór typu = min RMS; tryb automatyczny = **Efficient RANSAC (Schnabel 2007)**; symmetry = dopasowanie
  płaszczyzny minimalizującej odległość mesh↔odbicie.
- **Region jako encja pierwszej klasy** *(wzorzec z GDX: `Auto Segment`/`Resegment`/`Merge`/`Split`/
  `Insert`/`Separate` na trwałym typie `Region`, nie tylko doraźne zaznaczenie trójkątów)* — zamiast
  fitować prymityw tylko do chwilowego zaznaczenia, `doc` powinien trzymać `Region` jako trwały,
  edytowalny obiekt (z przypisanym typem geometrii po klasyfikacji), wielokrotnego użytku do pomiaru,
  selekcji i późniejszego fitu. Podnosi UX bez zmiany algorytmu dopasowania.
- Rust: `fit` + `levenberg-marquardt`/`argmin`; więzy przez `sketch`/solver. Reprezentacja w `solidcore`.
  `doc` zyskuje typ `Region` (patrz wyżej) obok prymitywów — **w zakresie M2** (auto-segment + merge/split;
  reszta operacji na regionach może dojść później, patrz §8).

### 6.3 Powierzchnie swobodne
- Dwa poziomy: `Fit Surface` (jeden płat NURBS na region, deviation na żywo) oraz `AutoSurface/Freeform`
  oparte na **quadach** (`Target Quads Count`, split 4×, merge, snap-to-mesh, crease, wrap). *(obserwacja:
  obecność InstantMeshes/xremeshlib w QS)*
- Algorytm: parametryzacja + LSQ na punktach kontrolnych B-spline/NURBS; retopo = pole kierunków+pozycji
  (Instant Meshes) → łaty Béziera/NURBS zszyte z ciągłością G1.
- Rust: NURBS = **`solidcore`** (płat z punktów kontrolnych, `srf_fit::lst_sq_approx` na siatce regularnej,
  ewaluacja z pochodnymi, `surface_closest_point` jako rdzeń mapy odchyłek, arkusz z płata). **Luka:** brak fitu
  NURBS do punktów **rozproszonych** — w M3 resampling regionu do siatki (parametryzacja) albo własny LSQ (`faer`)
  w `fit` na allowliście (§5), docelowo prymityw kernela upstream. Retopo = port/adaptacja **Instant Meshes (MIT)**
  — *decyzja do buildera: port natywny vs wywołanie jako proces na starcie* (patrz §10).
- **Druga, uzupełniająca strategia** *(wzorzec z GDX: `Trace Feature Line` → `Extract Contour Curves` →
  `Construct Patch Network` → `Relax Contour`/`Relax Patches` → `Repair Patches` → zszycie NURBS)* —
  sieć łat wyznaczona przez krzywe cech (Class-A style) zamiast retopologii quadowej po gęstym meshu.
  Lepsza dla części z wyraźnymi krawędziami charakterystycznymi (obudowy, karoserie); nie zastępuje
  retopo-quad, tylko dokłada drugi tryb pracy na tym samym silniku NURBS-stitching. Nie-MVP, do
  rozważenia po M4 (patrz też §10).
- **Najcięższa część algorytmicznie.**

### 6.4 Drzewo cech parametrycznych
- History-based: `History View`, `Dual features tree`, solver więzów szkicu (`sketch is overdefined`).
  Potok: Mesh Section/Stacked Sections → Fit Primitives in 2D → constraints+dimensions →
  Extrude/Revolve → Fillet/Chamfer/Shell/Pattern/Trim/Cut/Boolean/Move Face/Helix; Roll/Unroll.
- Algorytm: solver więzów 2D (Newton na równaniach więzów); cechy = węzły DAG historii; edycja = replay DAG.
- **Podpowiadanie parametrów cechy** *(wzorzec z GDX: Redesign Assistant™ — przy tworzeniu fillet/draft/
  sweep/revolve/pattern silnik od razu ekstrahuje z mesha w okolicy promień, oś, kąt pochylenia, odległość
  offsetu itp., zamiast czekać na ręczny input)* — ten sam LSQ-fit co ekstrakcja prymitywów (§6.2), tylko
  wywołany lokalnie w momencie tworzenia cechy. Tanie algorytmicznie, duży zysk UX; wpleść w kreatory cech
  w M5, nie jako osobny moduł.
- Rust: kernel = **`solidcore-ops`** — extrude (z pochyleniem), revolve, loft, sweep, split/trim, booleany,
  fillet/chamfer, shell/offset/draft, stitch, przekroje i pomiary są w API (spike §1.3). Trim i przekrój płaski
  (`brep_planar_section`) mają w porcie status *partial* — do testu przed M5. **Pattern** nie ma dedykowanej
  operacji: recepta copy + transform + `union_all` w `engine`. Solver więzów = własny (`faer`) albo adaptacja
  `solidcraft-sketch` wg zasady z §9; DAG+replay = `doc`/`engine`. STEP/IGES — patrz §10.
- **Największy pojedynczy kawałek pracy** (solver więzów + DAG + komplet cech).

## 7. Wzorce z QuickSurface i Geomagic Design X warte powtórzenia (jako wzór, nie kod)

Potwierdzone niezależnie przez oba produkty (QS + GDX — patrz obie analizy funkcjonalne):
- Potok dwuetapowy „ekstrahuj prymitywy → wyrównaj".
- **Deviation jako stały feedback** przy każdej operacji (mapa kolorów + max/RMS; w GDX: Accuracy Analyzer™).
- **Hybrid CAD** (analityczne + freeform w jednym modelu; w GDX: „True Hybrid Modeler").
- Auto-detekcja typu prymitywu po residuum (tani, efektowny UX).
- Wieloetapowy ICP dla scalania wielu skanów (w GDX opakowany jako kreator: Mesh Buildup Wizard™ —
  Data Preparation → Data Editing → Pre-Align → Best-Fit Align → Merge; wzorzec UX wart powtórzenia
  dla przypadku wielu-skanów, bez nowego algorytmu).

Dodatkowe, tylko z GDX (patrz [analiza GDX](./2026-11-08-geomagic-designx-functional-analysis.md) §1–§4):
- **Region jako trwała encja pierwszej klasy** (§6.2 wyżej).
- **Podpowiadanie parametrów cechy przy tworzeniu** w stylu Redesign Assistant™ (§6.4 wyżej).
- **CAD Correct / Refit** — dopasowanie (deformacja) *istniejącej* bryły CAD do nowego skanu, per-face
  wagi sztywności (loosely/tightly/free faces), global/local control vertices. Realny use-case inżynierii
  zmian („mam już model, część się zmieniła, popraw CAD do stanu as-built"). Nie wymaga nowego kernela —
  tylko ważony fit powierzchni B-rep na istniejącej bryle przez `solidcore`. **Kandydat post-MVP**
  (po M5), nie w zakresie czterech funkcji rdzeniowych z §2.
- Druga strategia freeform — sieć łat z krzywych cech (§6.3 wyżej).

Asystent AI (tryby „align"/„workflow", wejście = obraz viewportu) — post-MVP, przez `ai` + MCP.

## 8. Kolejność budowy (milestones)

Rosnąco wg trudności; każdy kamień daje działającą, testowalną całość:

- **M0 — szkielet:** workspace i konwencje rodziny (§9), warstwy z kontrolą `cargo xtask layers`, import mesh
  (STL/OBJ/PLY), viewport wgpu, rejestr komend, CLI; **pin `rev` solidcore + `deny.toml` (allow-git) + smoke test
  granicy `kernel`** (box → fillet → `to_record`), żeby M2+ miało na czym się opierać.
- **M1 — Wyrównanie (beachhead):** 3-2-1, N-point, ICP, world-axis. Pierwsza użyteczna wersja.
- **M2 — Prymitywy:** fit plane/cylinder/cone/sphere/torus + auto-typ + symmetry; deviation;
  **`Region` jako trwała encja w `doc`** (auto-segment + merge/split; pozostałe operacje — resegment/insert/
  separate/enlarge/shrink — mogą dojść później, patrz §6.2).
- **M3 — Fit Surface:** pojedynczy płat NURBS na region + mapa odchyłek; **eksport STEP** (powierzchnie/shell —
  pierwszy writer, patrz §10).
- **M4 — Freeform/AutoSurface:** retopo quad + łaty NURBS + snap/crease/merge; thicken→bryła (`offset::thicken`).
- **M5 — Parametryka:** szkic 2D + solver więzów, przekroje, extrude/revolve, fillet/chamfer/shell,
  pattern/trim/cut/boolean, DAG historii; eksport STEP pełnych brył parametrycznych + **eksport IGES**.
- **M6 — Web (wasm) + MCP** równolegle utrzymywane; **AI assistant** jako ostatnie.

## 9. Ograniczenia

- **Pure Rust / wasm-compatible** — kernel `solidcore` (Apache-2.0, czysty Rust bez toolchainu C++), zero
  Parasolid/OCCT jako głównego kernela, zero zależności Fortran/BLAS natywnych (preferuj `faer`).
  **Dystrybucja:** zależność **z git z pinem `rev`** (`solidcore`, `solidcore-ops`, `solidcore-record`,
  `solidcore-tess`, `step-*` — poza `solidcore` nic nie jest na crates.io), `deny.toml` z `unknown-git = "deny"`
  i `allow-git = ["https://github.com/tangency-labs/solidcore"]`. wasm: CI solidcore sprawdza tylko crate
  `solidcore` — resztę sprawdza nasz `cargo xtask wasm`; konsument wasm potrzebuje `-zstack-size=8388608`.
- **Licencja MIT OR Apache-2.0**; pliki LICENSE-APACHE, LICENSE-MIT, NOTICE, ATTRIBUTION.md, generowany
  THIRD-PARTY-LICENSES.txt; `cargo xtask ci`.
- **Clean-room:** żadnego kodu/zasobów QuickSurface, Geomagic Design X ani brandingu ArtCraft w forku; tylko
  analiza funkcji. Dotyczy także danych testowych: przykładowe skany z instalacji QS/GDX są tylko lokalne (§11.1).
- **Kod rodziny Crafting Apps (decyzja użytkownika 2026-11-08): „najpierw upstream, potem adaptacja”.** Clean-room
  dotyczy tylko kodu zamkniętego; otwarty kod rodziny (MIT OR Apache-2.0, np. SolidCraft) wolno wykorzystać, w tej
  kolejności: (1) użyć tego, co jest w solidcore; (2) brakującą rzecz zaproponować/dopisać **upstream** do solidcore
  (zgodnie z regułą rodziny „one implementation per job”); (3) dopiero gdy upstream się nie da albo blokuje
  milestone — **adaptacja** kodu z repo rodziny do SurfaceCraft z atrybucją: zachowane nagłówki copyright, wpis
  w `NOTICE` i `ATTRIBUTION.md`, oznaczenie zmodyfikowanych plików (Apache-2.0 §4), bez nazw/ikon/brandingu
  źródła. Adaptacje oznaczamy jako dług do wymiany na wersję z solidcore, gdy ta powstanie. **Nie** zależymy od
  ciężkich crate'ów aplikacji (np. `solidcraft-kernel`) przez git.
- **Konwencje rodziny (twarde wymaganie, §12; szczegóły: [raport spike'u](./2026-11-08-surfacecraft-ecosystem-spike.md) §3):**
  workspace `resolver = "3"`, `crates/*` + `apps/*` + `xtask`, edition 2024, MSRV 1.95, pakiety `surfacecraft-<crate>`;
  aplikacje `apps/surfacecraft`, `apps/surfacecraft-cli`, `apps/surfacecraft-web`; tabela warstw w `xtask` (crate'y UI
  dopiero od warstwy UI); `unsafe_code = "forbid"` + deny `unwrap/expect/panic/todo/unimplemented/unreachable`
  w kodzie produkcyjnym; wejście traktowane jako wrogie (bez panik); `cargo xtask ci` = fmt → assets → licences →
  layers → clippy `-D warnings` → wasm check → testy; `AGENTS.md` jako router do `skills/<obszar>/SKILL.md`,
  `CLAUDE.md` = `@AGENTS.md`; `CommandSpec` z id `area.verb` i schematami wejścia/wyjścia (JSON Schema), skrypty
  `{"commands": [...]}`, wywołania programowe bez dialogów; MCP jako własny JSON-RPC 2.0 po stdio (bez `rmcp`),
  tryb „bridged” do działającej aplikacji przez `127.0.0.1` z tokenem sesji.
- **Repo i organizacja (decyzja użytkownika 2026-11-08):** SurfaceCraft żyje na koncie autora
  (`github.com/d0pawlus/surfacecraft`, na razie prywatne). Blok house rules z `storytold/craft-contrib` nie jest
  obowiązkowy — własny `AGENTS.md` w duchu SolidCrafta; blok można dodać, jeśli projekt trafi do organizacji rodziny.
- **Dołączenie do rodziny = zgodność konwencji + wpis w `storytold/craft-launcher`** jako *community app*
  (decyzja maintainera; kontrakt wydań: własne bundle id, `SHA256SUMS.txt`, podpis Developer ID na macOS).
  API pluginu nie ma. Wydania przygotowujemy zgodnie z tym kontraktem.
- Wejście: mesh (nie surowa chmura). Jednostki: mm (z obsługą skali przy imporcie, jak QS).
- **Skala i rozszerzalność (decyzja użytkownika):** MVP celuje w mesh **do ~10 mln trójkątów, in-memory,
  desktop**. Architektura musi jednak zostawiać ścieżkę rozwoju do **100 mln+** bez przepisywania rdzenia:
  warstwa `geom`/`deviation` powinna abstrahować dostęp do meshu (indeks przestrzenny, iteracja) tak, by
  dało się później podmienić na **out-of-core + LOD** (wzorzec z GDX). Nie implementujemy tego w MVP, ale
  żadna decyzja z M0–M5 nie może tego zablokować (np. brak założenia „cały mesh zawsze w RAM" w API warstw).

## 10. Co zostaje do decyzji buildera

- **Dostawca STEP/IGES (M3/M5) — rozstrzygnięty po spike'u, wg zasady z §9:** wszystkie writery czytają ten sam
  neutralny rekord (`Body::to_record` → `solidcore_record::brep::Brep`), więc nie ma tłumaczenia topologii.
  - **M3 — STEP powierzchni:** PR upstream do `solidcore/step-export` dodający `SHELL_BASED_SURFACE_MODEL`/`OPEN_SHELL`
    dla `BodyKind::ShellModel` (model i `brep_to_model` już to obsługują; luka potwierdzona przebiegiem probe). Fallback:
    adaptacja zapisu otwartych powłok z `solidcraft-step`. Osobno: `ops` nie ma operacji „arkusz z płata” (kernelowe
    `face_on` zwraca błąd 1001), więc w M3 arkusz budujemy jako rekord `Brep` z jedną ścianą `Spline`
    (`Body::from_record`) i równolegle zgłaszamy upstream operację `ops` „sheet from surface”.
  - **M5 — STEP brył:** `step-brep::brep_to_model` + `step-export::write_with_metadata` (AP214/AP242), round-trip przez
    `step_brep::import`; po scaleniu STEP w `solidcore-ops` przejście na API `ops`.
  - **M5 — IGES:** propozycja przeniesienia `iges_out` SolidCrafta do solidcore; fallback: adaptacja `iges_out`
    (IGES 5.3, encje 186/514) z atrybucją.
  - `truck`/`truck-stepio` odpada (rodzina z niego zrezygnowała; truck-stepio nieaktualizowany od 2024-09).
  Kolejność bez zmian: **STEP w M3**, **IGES w M5**.
- **Retopologia (M4):** port Instant Meshes do Rust vs wywołanie binarki jako proces na start
  (rekomendacja: najpierw proces/MVP, potem natywny port).
- ~~NURBS: `truck` vs `solidcore`~~ — **zamknięte: solidcore** (§6.3). Otwarte zostaje tylko: jak zamknąć lukę fitu do
  punktów rozproszonych w M3 (resampling vs własny LSQ vs PR upstream).
- Biblioteka solvera nieliniowego: `levenberg-marquardt` vs `argmin`.
- ~~Warstwa renderu~~ — **zamknięte: surowe `wgpu` przez `egui-wgpu`** (§5).
- Opcjonalne wspólne biblioteki `storytold/craft-libs` (`craft-ui`, `craft-window`, `craft-update`; Apache-2.0) —
  użyć czy nie, do decyzji przy M0.
- Format projektu (serializacja DAG): `serde` + JSON/ron/binarny — dobór po M0.
- Dokładny zestaw komend mesh-edit w MVP (minimalny vs rozszerzony).
- **Progi względem szumu skanu:** zatwierdzone progi z §11.1 są bezwzględne (mm), a szum typowych skanerów
  bywa tego samego rzędu (0,02–0,1 mm). Do rozważenia przy M2: estymacja szumu meshu (wzorzec z GDX:
  `Mesh Roughness → Estimate`) i ostrzeżenie, gdy próg jest poniżej szumu. Zatwierdzone progi zostają domyślne;
  to tylko dodatkowa informacja dla użytkownika.
- **Strategia freeform (post-M4):** czy dokładać drugą ścieżkę „sieć łat z krzywych cech" (wzorzec GDX,
  patrz §6.3/§7) obok retopo-quad, i w jakiej kolejności — do decyzji po ocenie wyników M4 na realnych skanach.

## 11. Ryzyka i założenia

- **Dojrzałość `solidcore` — ryzyko #1 projektu (częściowo obalone po spike'u).** API dla extrude/revolve/loft/
  sweep/split/booleanów/filletów/shell/thicken istnieje w `solidcore-ops`; pattern to recepta aplikacji. Nadal otwarte:
  27 unitów *partial* + 1 *blocked* w porcie (m.in. offset, trim, przekrój płaski, `boolean.api_brep`), API niestabilne
  mimo numeru 1.0.1, a `solidcore-ops` nie jest jeszcze używane produkcyjnie nawet przez SolidCraft. Probe ze spike'u
  (raport §1.6) dał poprawne objętości dla 11 operacji (fillet, chamfer, shell, extrude z pochyleniem, revolve, loft,
  boolean, pattern) natywnie i w wasm, ale to „happy path” na prostych bryłach, nie realne modele. **Działanie:**
  smoke test granicy `kernel` w M0; przed M2 i przed M5 testy realnych operacji na naszych przypadkach; luki —
  upstream albo własne operacje na prymitywach kernela (§9). *(ryzyko)*
- **STEP/IGES (częściowo obalone)** — STEP brył jest w solidcore, STEP arkuszy i IGES są w SolidCrafcie; workstream
  maleje do „integracja + mały PR upstream + IGES” (§10). *(ryzyko)*
- **wasm całego stosu (częściowo obalone)** — truck wypada; `solidcore` jest sprawdzany na wasm w CI, a stos egui 0.37/
  wgpu 30 + solidcore działa w `solidcraft-web`. Probe z `ops` + `step-*` kompiluje się na wasm i daje wyniki
  identyczne z natywnymi. Otwarte: brak kontroli wasm tych crate'ów w CI solidcore (sprawdza nasz `xtask wasm`),
  rozmiar binarki (~40 MB nieoptymalizowanego .wasm), limity pamięci przeglądarki dla dużych skanów — web zostaje celem warunkowym,
  desktop-first. *(ryzyko/decyzja Claude)*
- ~~Brak publicznego API launchera~~ — **obalone:** `storytold/craft-launcher` przyjmuje *community apps* (§9).
- **Zależność git-only z pinem `rev`** *(nowe)* — solidcore przenosi zadania między crate'ami (scalanie STEP w `ops`,
  `app-geometry`); wymaga regularnego podbijania pinu i testów kontraktowych na granicy `kernel`. *(ryzyko)*
- **Reguła „apps compute no geometry”** *(nowe)* — nasze `fit`/`align`/`deviation` liczą geometrię meshu; podział
  z §5 i allowlista `app-geometry` muszą być akceptowalne dla rodziny, jeśli SurfaceCraft ma trafić do launchera. *(ryzyko/decyzja)*
- **Brak fitu NURBS do punktów rozproszonych** *(nowe, M3)* — resampling do siatki albo własny LSQ, docelowo
  prymityw kernela upstream (§6.3). *(luka)*
- **Solver więzów + pełne drzewo cech (M5)** to research-grade i największe ryzyko harmonogramu;
  nawet QS robi to pół-automatycznie/interaktywnie — zakładamy workflow z udziałem użytkownika, nie 100% auto.
- **AutoSurface** (M4) jest kosztowny obliczeniowo i pamięciowo (QS używa dysku out-of-core). *(ryzyko wydajności)*

### 11.1 Korpus testowy i tolerancje (decyzja Claude; cele do potwierdzenia — §12)
- **Korpus testowy — dwa poziomy (zgodnie z clean-room z §9):**
  1. **Korpus w repo (CI, od M1):** wyłącznie dane na licencji zgodnej z MIT/Apache —
     **skany syntetyczne** (próbkowanie znanych prymitywów/brył CAD + szum gaussowski + dziury/outliery; znana
     prawda bazowa = dokładny test progów z niżej), własne skany autora oraz publiczne zbiory na wolnej licencji
     (licencję każdego pliku zapisać w `ATTRIBUTION.md`). Gotowe źródło CC0 z rodziny:
     [`tangency-labs/models`](https://github.com/tangency-labs/models) (STL/PLY/OBJ/STEP/IGES/3MF). Zgodnie z konwencją
     rodziny duże pliki trzymamy w osobnym repo `surfacecraft-corpus`, nie w repo kodu.
  2. **Korpus lokalny (tylko maszyna dewelopera, NIGDY w repo):** skany z instalacji QS (`X:\QUICKSURFACE2026`):
     `machine_part.stl` (~32 MB), `steel.stl` (~32 MB), `basic_part.stl`, `qsm.stl`/`lego.stl`/`tool.stl`/
     `impeller.stl`/`bearinghousing.stl`/`aerosol_knob.stl`/`yholder.stl` (rząd setek tys.–kilku mln trójkątów)
     oraz `basic_part_aligned.stp` jako referencja wyniku. To zasoby KVS Ltd — używamy ich do lokalnych testów
     realizmu (ścieżka przez zmienną środowiskową, testy `#[ignore]`/feature-gated), nie commitujemy ich ani
     pochodnych (wyników, zrzutów) do repo.
- **Tolerancje domyślne (konfigurowalne):** dopasowanie prymitywu — RMS odchyłki ≤ 0,1 mm (ostrzeżenie > 0,05);
  Fit Surface/freeform — max odchyłki ≤ 0,2 mm; ICP — zbieżność przy Δ < 1e-4 mm/iterację. QS minimalny tolerance
  to 0,001 mm — przyjmujemy to jako dolną granicę ustawienia. Każdy kamień milowy ma test „pass/fail" o te progi.

## 12. Ustalenia (potwierdzone przez użytkownika 2026-11-08)

- **Nazwa:** **SurfaceCraft** (zatwierdzona).
- **Skala:** MVP do **~10 mln trójkątów** (in-memory, desktop); architektura **rozszerzalna do 100 mln+**
  w przyszłości (out-of-core/LOD) — patrz §9. Nie implementujemy out-of-core w MVP, ale go nie blokujemy.
- **Dokładność:** domyślne progi z §11.1 — **zatwierdzone**.
- **AI assistant:** później (**M6**) — zatwierdzone.
- **Platformy:** desktop (Win/macOS/Linux) od M0; web+MCP później (warunkowo) — zatwierdzone.
- **Zgodność z rodziną Tangency/ArtCraft:** **twarde wymaganie** (determinuje licencję MIT/Apache,
  warstwowanie, command-driven + MCP) — zatwierdzone.
- **Kod rodziny:** „najpierw upstream, potem adaptacja z atrybucją” (§9) — zatwierdzone 2026-11-08.
- **Repo:** konto autora, `d0pawlus/surfacecraft`, prywatne; bez obowiązkowych house rules storytold (§9) —
  zatwierdzone 2026-11-08.

---

*Dokument **zatwierdzony**. Następny krok: plan implementacji (superpowers:writing-plans), dopiero potem kod.
Do tego czasu nic nie jest budowane.*
