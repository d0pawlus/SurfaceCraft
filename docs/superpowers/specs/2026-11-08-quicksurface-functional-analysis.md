# QuickSurface 2026 — analiza funkcjonalna (materiał referencyjny dla SurfaceCraft)

> Źródło: analiza RE instalacji `X:\QUICKSURFACE2026` (KVS Ltd).
> Charakter: analiza **zachowania/funkcji** z zasobów shipowanych (katalog komend, baza wiedzy,
> instrukcja). **Nie zawiera kodu QuickSurface** — binarka jest spakowana (Themida) i nieczytelna
> statycznie. Materiał do użycia clean-room: wzorzec funkcji, nie implementacja.
> Oznaczenia: *obserwacja* = ze stringów/dokumentacji QS; *algorytm* = wniosek/standard branżowy.

## Stos QuickSurface (kontekst)
- `QuickSurface.exe` (13.7 MB, x64, MFC + GDI+, spakowany Themida), podpisany: **KVS Ltd**, Sofia/Blagoevgrad, BG (Sectigo).
- Kernele geometrii: **OpenCASCADE** (~35 `TK*.dll`) + **Parasolid** (`pskernel.dll`, 73 MB).
- Retopologia: **Quad-Remesher/Exoside** (`xremesh*`), **Instant Meshes** (`InstantMeshes.exe`, `nanogui.dll`), `flatten.exe`.
- Algebra: OpenBLAS/LAPACK/libgfortran + `ChSolver.dll` (solver — prawdopodobnie więzy/dopasowanie).
- Sieć: WININET/WINHTTP/WS2_32/urlmon/IPHLPAPI (licencja+update), `QSUpdater.exe`.
- Asystent AI: pliki promptów + baza wiedzy dla **Google Gemini** (tryby „align" i „workflow", wejście = obraz viewportu).
- Katalog UI: `language.txt` — **2890 pozycji** (424 menu, 674 dialog, 282 string-table, 1510 w kodzie).

## Potok reverse engineering (dwuetapowy, obserwacja)
Import mesh → (opcjonalne oczyszczenie) → **Extract Primitives** → **Align** (prymitywy jako primary/secondary/tertiary)
→ rekonstrukcja (analityczna: przekroje→szkic→cechy; organiczna: Fit Surface/freeform) → **Deviation** → eksport STEP/IGES.
Model **hybrydowy** (analityczne + freeform razem) to kluczowa cecha.

---

## 1. Wyrównanie do układu współrzędnych
*Obserwacja (komendy):* `Align by Coordinate System` (primary/secondary/tertiary → 3-2-1 datum), `Align by N points`,
`Align Interactive`, `Fine align` („minimising the overlapping distance" = ICP), `Best Fit to CAD`,
`Align to closest axis`, `Align to world axis`, offset chmury przy dużych współrzędnych, symmetry plane jako referencja,
`Global Fine Align` (wiele skanów).
*Algorytm:* Gram-Schmidt dla datum; **Kabsch/Umeyama (SVD)** dla N-point; **ICP point-to-plane** + KD-tree.

## 2. Ekstrakcja prymitywów
*Obserwacja:* typy **plane/cylinder/cone/sphere/torus** + **symmetry plane**; skróty P/C/O/E; „fit through selected triangles";
`Auto Constrain` (relacje równoległość/prostopadłość/współosiowość); auto-detekcja typu po residuum
(„Failed to fit cone. It is more suitable to be a cylinder"); „Finding the symmetry plane…"; „Extract visible cylinders centre line".
*Algorytm:* PCA (płaszczyzna); nieliniowy LSQ/Levenberg-Marquardt (sfera/walec/stożek/torus); wybór typu = min RMS;
tryb automatyczny = **Efficient RANSAC (Schnabel 2007)**; symmetry = dopasowanie płaszczyzny min. odległość mesh↔odbicie.

## 3. Powierzchnie swobodne (freeform / NURBS)
*Obserwacja:* `Fit Surface` („Approximates selected triangles and generates a single surface with best fitting" + deviation na żywo);
`Freeform/AutoSurface` oparte na **quadach** (`Target Quads Count`, „Actual number of quads", split 4×, `Merge Quad Surfaces`,
`Snap to Mesh`, `Wrap Surface`, `Crease Edges`, „From quad OBJ"); `Automatic Surfacing` (out-of-core, „not suitable for autosurfacing");
eksport **IGES**; `Thicken/Offset Freeform`, „Creating Solid Shell of Free Form Surfaces".
*Algorytm:* parametryzacja + LSQ na punktach kontrolnych B-spline/NURBS; retopo = pole kierunków+pozycji (Instant Meshes) →
łaty Béziera/NURBS zszyte z ciągłością G1. Deviation = odległość punkt↔powierzchnia (KD-tree).

## 4. Drzewo cech parametrycznych
*Obserwacja:* `Parametric History Based Modelling`, `History View`, `Dual features tree`, solver więzów szkicu
(„sketch is overdefined", „This dimension overdefines the sketch"). Potok: `Mesh Section`/`Stacked Sections` →
`Fit Primitives in 2D Sketch` (linie/łuki/okręgi do punktów przekroju) → constraints+dimensions → `Extrude`/`Revolve`
(oś z construction line) → `Fillet`/`Chamfer`/`Shell`/`Pattern (Linear/Circular)`/`Trim`/`Cut`/`Boolean`/`Move Face`/`Helix`;
`Roll/Unroll Sketch` (2D↔3D). **Uwaga:** QS część operacji parametrycznych **deleguje do SolidWorks/Inventor**
(„transfer to SOLIDWORKS", „export to Autodesk Inventor", „Dual features tree") — SurfaceCraft musi mieć to w 100% natywnie.
*Algorytm:* solver więzów 2D (Newton na równaniach więzów); przekroje = cięcie meshu płaszczyzną → polilinie →
fit łuków/linii (RANSAC 2D + LSQ); cechy = węzły DAG historii; edycja = replay DAG na kernelu B-rep.

---

## Wzorce warte powtórzenia (wzór, nie kod)
- Potok dwuetapowy „ekstrahuj prymitywy → wyrównaj".
- **Deviation jako stały feedback** (mapa kolorów + max/RMS) przy każdej operacji — buduje zaufanie.
- **Hybrid CAD** (analityczne + freeform w jednym modelu).
- Auto-detekcja typu prymitywu po residuum.
- Asystent AI (tryby „align"/„workflow", wejście = zrzut viewportu).

## Materiał źródłowy (instalacja QS)
Wszystkie pliki poniżej leżą bezpośrednio w katalogu instalacji `X:\QUICKSURFACE2026` — analizę można powtórzyć
bez dodatkowych kroków. To zasoby KVS Ltd: czytamy je lokalnie, nie kopiujemy do repo (clean-room — patrz
design doc §9 i §11.1).
- `language.txt` — pełny katalog 2890 komend (menu/dialog/string-table/kod).
- `knowledgebase_general.md`, `knowledgebase_align.md` — biblioteka wideo pogrupowana wg funkcji (Basics, Aligning,
  Mesh Editing, 2D Sketch, CAD Features, Surfacing, Free Form, Workshops, Academy).
- `QSHelp.pdf` — instrukcja obsługi (spis treści = mapa funkcji).
- Pliki promptów Gemini: `prompt_align_gemini.md`, `prompt_wf_gemini.md`, `system-instructions-align-gemini.md`,
  `system-instructions-wf-gemini.md`.
- Przykładowe skany (`*.stl`) i `basic_part_aligned.stp` — lokalny korpus testowy (design doc §11.1).
