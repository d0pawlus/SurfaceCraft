# Geomagic Design X 2026.1 — analiza funkcjonalna (materiał referencyjny #2 dla SurfaceCraft)

> Źródło: analiza RE instalacji `X:\Geomagic\Geomagic Design X 2026.1` (Hexagon AB / Geomagic, dawniej Rapidform).
> Charakter: analiza **zachowania/funkcji** z zasobów shipowanych — mapa modułów (nazwy plików .exe/.dll),
> pełny katalog komend z oficjalnego User Guide (`GeomagicDesignX_UserGuide.chm`, zdekompilowany narzędziem
> systemowym `hh.exe -decompile`, 485 tematów HTML, w tym pełne drzewo TOC), `GeomagicDesignX_ReleaseNotes.pdf`
> (wersja 2026.1.0) oraz próbki projektów w `Sample/`. **Nie zawiera kodu Geomagic Design X** — nie dekompilowano
> binarek; materiał to wyłącznie udokumentowane zachowanie/komendy, do użycia clean-room: wzorzec funkcji, nie
> implementacja. Uzupełnia [`2026-11-08-quicksurface-functional-analysis.md`](./2026-11-08-quicksurface-functional-analysis.md)
> (ten sam dzień, ten sam cel: materiał wejściowy do `2026-11-08-surfacecraft-scan-to-cad-rust-design.md`).
> Oznaczenia: *obserwacja* = z nazw komend/dokumentacji GDX; *algorytm* = wniosek/standard branżowy.

## Stos Geomagic Design X (kontekst)

- `RapidformMain.exe` (7,6 MB, x64, MFC + BCGSoft BCGControlBar skin engine) uruchamiany przez stub
  `GeomagicDesignX.exe`. Rodowód po **Rapidform XOR** potwierdzony nazwami: `rf*.dll` (setki modułów), format
  projektu **`.xrl`**/`.xpc` (`rfXRL(Base).dll`, `rfXODocument.dll`, `rfXOFileBase.dll`).
- Kernel geometrii: **Parasolid** (`pskernel.dll`) + katalog `LegacyParasolid` (kompatybilność wsteczna) +
  **Spatial 3D InterOp/Exchange** (`SPAX*Parasolid*.dll`, `SPAIop*.dll` — komercyjny pakiet tłumaczy wielu
  formatów CAD: JT, SolidEdge, SolidWorks, UG/NX, dane PMI/produkcyjne). To ten sam kernel co w QuickSurface —
  potwierdza, że Parasolid jest de facto standardem w tej niszy (i że `solidcore`, czysto-rustowy, jest
  rzeczywistą alternatywą wartą obstawienia).
- Algebra/accel: Intel **MKL** (`mkl_*.dll`) + `libiomp5md.dll` (OpenMP), CUDA runtime (`cudart64_80.dll` — GPU
  dla wybranych operacji, prawdopodobnie auto-segmentacja/remesh dużych siatek).
- Integracje CAD natywne (**LiveTransfer**): dedykowane serwery per-wersja dla **SolidWorks, Inventor, NX/UG,
  Creo/Pro-E, AutoCAD** (`rf{Creo,Inventor,NX,SW,UG,ACAD}*IFServer.dll`, dziesiątki wersji) — przesyłają cechy
  parametryczne (nie tylko geometrię) bezpośrednio do drzewa historii docelowego CAD-a.
- Integracje skanerów (**Scanner Direct Control / LiveCapture**): sterowniki/SDK per-producent — Creaform, FARO
  (w tym Cobalt Array Imager), Hexagon Absolute/Romer, Konica Minolta, Shining3D, Z Corporation ZScanner,
  ScanTech (DefinSight/TViewer), AICON, Kreon, Leica (`LgsxSdk.dll` — format `.lgsx`), NDI, Perceptron, Zeiss,
  RPS Metrology, PMT, plus obrotowy stolik (`InusRotaryTableControl64.dll`) i kamery FLIR/PointGrey
  (`FlyCapture2_v140.dll`, zapewne fotogrametria).
- Formaty danych skanu: `laslib64.dll` (**LAS/LIDAR**), `LgsxSdk.dll` (Leica reality-capture `.lgsx`), LeadTools
  (`Lf*/Lt*.dll`) i FreeImage dla tekstur/obrazów.
- Licencjonowanie: **CimLM / Hexagon CLM 12.0** (`CimLicManager.exe`, `CimLmServ.dll`) + klucz sprzętowy HASP
  (`hasp_windows_x64_*.dll`, Thales Sentinel) — osobna warstwa, nieistotna funkcjonalnie dla SurfaceCraft.
- Skrypty/automatyzacja: `rfAddinScript.dll` (Addin) + `DecodeScript.py` w `Bin` — makro/skrypt record&replay;
  osobny silnik **Batch Process Designer** i **Scan Process Designer** (wizualny edytor procesów wsadowych).
- Dokumentacja: `GeomagicDesignX_UserGuide.chm` (83 MB, 485 tematów) i `_Tutorial.chm` — pełny katalog komend
  z opisami "Benefits / How to Use / Options in Detail", analogiczny do `language.txt`+knowledgebase z analizy QS.

## Potok reverse engineering (obserwacja z dokumentacji)

Import mesh/chmura (`Mesh Buildup Wizard`, wieloetapowe wyrównanie+scalanie wielu skanów) → **Auto Segment**
(automatyczna klasyfikacja regionów mesha na typy geometryczne: płaszczyzna/walec/stożek/kula/torus) →
**Align** (Align Wizard / Quick Fit / Best Fit / Datum Match / 3-2-1) → rekonstrukcja: dla regionów
analitycznych **Redesign Assistant** ekstrahuje parametry cech (promień, oś, kąt pochylenia, grubość…) →
szkic/cechy parametryczne (`Extrude/Revolve/Sweep/Loft/Pipe` + `Fillet/Chamfer/Draft/Pattern/Shell/Boolean`);
dla regionów organicznych **Fit Surface Patches / Auto Surface / Boundary Fit** (NURBS) → **Accuracy Analyzer**
(odchyłka na żywo, cały czas) → `LiveTransfer` do SolidWorks/Inventor/NX/Creo/AutoCAD **lub** eksport
STEP/IGES/Parasolid/JT itd. Model **hybrydowy** (solid + surface + mesh naraz) to centralna teza produktu —
dokładnie ta sama co w QuickSurface, tu nazwana wprost "True Hybrid Modeler".

---

## 1. Segmentacja regionów (NOWOŚĆ względem analizy QS)

*Obserwacja:* `Tools > Region Tools > Auto Segment` / `Resegment` / `Merge` / `Split` / `Insert` / `Separate` /
`Enlarge` / `Shrink`. **Region** to pełnoprawny typ encji (jak Mesh, Point Cloud, Sketch), z własną zakładką
REGION w Ribbon. Auto Segment ma własny panel "Geometry Classification Toolbar": `Tolerance`, `Outlier`,
`Sensitivity` (gęstość podziału), `Mesh Roughness` (+przycisk `Estimate` — automatyczna heurystyka szumu),
`CAD Tessellated Mesh` (tryb bez-szumowy vs tryb skanu), `Preserve Existing Region`, `Merge Same Primitive
Shapes`. Po klasyfikacji region "wie", jakim typem prymitywu jest (hover pokazuje typ geometrii).
*Algorytm:* segmentacja = region growing po krzywiźnie/normalnych + klasyfikacja per-region (RANSAC/LSQ fit +
wybór typu po residuum — ten sam mechanizm co "auto-detekcja typu" w QS, ale tu **poprzedza** ręczną ekstrakcję
jako osobny, jawny krok na całym meshu, a nie tylko na zaznaczeniu).
*Różnica vs QS:* QuickSurface ekstrahuje prymitywy z ręcznie/łatwo zaznaczonych trójkątów (`fit through selected
triangles`) — segmentacja całego mesha na regiony jest tu domyślnym trybem automatycznym, ale nie pierwszoklasową
encją trwałą w drzewie. Geomagic robi odwrotnie: **Region jest encją pierwszej klasy**, trwałą, edytowalną
(merge/split/insert/separate) i wielokrotnego użytku (do pomiaru, selekcji, późniejszego fitu).

## 2. Asystent ekstrakcji parametrów cech — Redesign Assistant™

*Obserwacja (z materiału marketingowego GDX, potwierdzone nazwami w UI):* automatyczna ekstrakcja z mesha:
promień i środek zaokrąglenia, płaszczyzna i profil szkicu (auto-sketch), krzywa ścieżki wyciągnięcia (sweep),
oś wyciągnięcia, płaszczyzna lustra, oś obrotu (revolve), oś rury (pipe), **kąt pochylenia (draft angle)**,
krzywa przekroju 3D do loftu, krzywa cechy, odległość offsetu/pogrubienia, oś walca/stożka, oś rowka (slot), oś
i kierunek wzoru (pattern), krzywa sylwetki, linia podziału (parting line), linia karbu (bead line). Działa
"intelligent real-time 2D/3D geometry recognition and snapping" — podpowiada wartości podczas tworzenia cechy,
zamiast wymagać ręcznego pomiaru.
*Algorytm:* dla każdego typu cechy — dopasowanie LSQ odpowiedniego prymitywu/przekroju do punktów mesha w
sąsiedztwie operacji, wyciągnięcie parametru (promień/kąt/odległość/oś) z dopasowania zamiast z inputu
użytkownika. To rozszerzenie tego samego RANSAC/LSQ-fit z sekcji 2 analizy QS, zastosowane *w momencie tworzenia
cechy parametrycznej*, nie tylko przy ekstrakcji prymitywu.

## 3. Mesh Buildup Wizard™ — wieloetapowy potok wielu skanów

*Obserwacja:* kreator 5-etapowy: `Data Preparation` (analiza stanu danych) → `Data Editing` (usuwanie szumu/
nadmiarowych punktów) → `Data Pre-Aligning` (szybkie wstępne wyrównanie wielu plików) → `Best-Fit Aligning`
(wyrównanie po kształcie geometrycznym) → `Data Merging` (scalanie + optymalizacja mesha). Dostępny też jako
samodzielne `Tools > Scan Tools > Mesh Buildup Wizard`.
*Algorytm:* ICP wieloetapowy (coarse→fine, tak jak `Global Fine Align` w QS) + scalanie/decymacja siatek po
rejestracji. Potwierdza wzorzec z analizy QS (ICP + KD-tree), dodaje tylko framing "wizard" (kolejne etapy w
jednym UI) — nie nowy algorytm, ale dobry wzorzec UX dla wielu-skanów w SurfaceCraft.

## 4. Refit / Rewrap / Defeature — operacje "mesh ↔ geometria"

*Obserwacja:*
- `Tools > Mesh Tools > Refit` — 3 tryby: **To Analytic Shape** (dopasowanie regionu do
  płaszczyzny/walca/stożka/kuli/torusa/boxa, z opcją `Fix Radius`, `Axis Constraint`), **To Mesh/Point Cloud**
  (odtworzenie zniekształconego mesha z referencji), **To Surface** (dopasowanie mesha do istniejącej
  powierzchni).
- `Insert > CAD Correct > Refit` (Pro) — **deformuje istniejącą bryłę CAD** tak, by pasowała do nowego skanu:
  per-face wagi sztywności (`Loosely/Tightly/Free Faces`, suwak), `Global`/`Local` control vertices, `Compensation
  Factor`, `Smoothness`. Typowy use-case: aktualizacja starego modelu CAD do stanu "as-built" po zmianach w
  narzędziu/formie.
- `Rewrap` (Pro) — wypełnia braki w meshu **w oparciu o rozpoznane kształty geometryczne** (lepsze niż zwykłe
  fill-holes, bo wie, że dziura jest na walcu i domyka ją jako walec).
- `Defeature` — usuwa zaznaczony kształt (np. napis, znacznik, uchwyt mocujący) i domyka otoczenie `Flat`/
  `Curvature`-fill.
*Algorytm:* nieliniowy LSQ fit z wagami per-wierzchołek/per-face (Refit do kształtu/CAD), interpolacja
  harmoniczna/curvature-flow (Rewrap, Defeature fill), identyczny mechanizm dopasowania co ekstrakcja prymitywów,
  tylko kierunek odwrotny (kształt→mesh zamiast mesh→kształt).
*Warte uwagi:* "CAD Correct" (dopasowanie **istniejącej** bryły CAD do nowego skanu) to realny, częsty use-case
w RE ("mam już model, part się zmienił, popraw CAD") — QuickSurface go nie eksponuje w analizie #1.

## 5. Powierzchnie swobodne — alternatywna ścieżka: sieć łat z krzywych cech

*Obserwacja:* obok `Fit Surface Patches` / `Auto Surface` / `Boundary Fit` (odpowiedniki `Fit Surface`/
`AutoSurface` z QS), Geomagic ma drugi, oparty na **krzywych cech** potok w `3D Sketch`: `Trace Feature Line` →
`Extract Contour Curves` → `Construct Patch Network` → `Shuffle Patch Groups` / `Relax Contour` / `Relax
Patches` → `Repair Patches` / `Remove Patches` → zszycie w NURBS. To klasyczne **Class-A surfacing**: siatka
łat czworokątnych wyznaczona przez sieć krzywych charakterystycznych (feature lines), a nie przez retopologię
quadową meshowo-gęstą (jak Instant Meshes w QS).
*Algorytm:* ekstrakcja krzywych grzbietowych/cech (curvature ridges lub ręczne `Pencil`/`Section`/`Intersection`
na mesh) → topologia sieci łat (ręczna/pół-auto) → relaksacja siatki łat (sprężynowy/Laplacian solver) →
powierzchnia B-spline per łata, zszyta z ciągłością G1/G2.
*Warte uwagi:* to **druga, uzupełniająca strategia** do tej opisanej w `2026-11-08-surfacecraft-scan-to-cad-rust-design.md` §6.3 (retopo-quad).
Dla części o wyraźnych krawędziach charakterystycznych (konsumenckie obudowy, karoserie) sieć łat z krzywych
cech daje bardziej "inżynierski"/edytowalny wynik niż czysty retopo quadowy — warto rozważyć jako alternatywną
ścieżkę M4+ (nie zamiennik, tylko drugi tryb pracy), zwłaszcza że `solidcore`/`truck` i tak muszą umieć
NURBS-patch-stitching dla obu podejść.

## 6. Drzewo cech parametrycznych — potwierdzenie i rozszerzenie

*Obserwacja:* pełny zestaw cech solid+surface (Extrude/Revolve/Sweep/Loft/Pipe, Solid Offset, Thicken, Shell,
Cut, Boolean; Mesh Fit, Fit Surface Patches, Auto Surface, Boundary Fit, Extend/Trim/Untrim/Solidify/Sew/Reverse
Normal), kreatory (`Modeling Wizard`: Solid/Surface Primitive, Extrusion/Revolution/Sweep/Loft/Pipe Wizard),
cechy modelujące (Fillet, Chamfer, Draft, Emboss, Linear/Circular/Curve Pattern, Mirror, Helix/Spiral Curve,
Move/Delete/Replace/Split Face, Transform/Duplicate/Delete Body), historia (`Roll Back/Forward/To End`,
`Suppress/Unsuppress`, `Regenerate All`, `Edit Feature`), **dual mesh/solid sketch** (`MESH SKETCH` vs `SKETCH`,
`3D MESH SKETCH` vs `3D SKETCH`, `Roll/Unroll Sketch` do konwersji 2D↔3D na zakrzywionej powierzchni). Release
notes 2026.1 pokazują aktywny rozwój UX historii: *"Remember Previously Used Operator Type"* dla
Extrude/Revolve/Loft/Sweep/Primitive (boolean join/cut/intersect zapamiętany między użyciami).
*Algorytm:* bez zmian względem analizy QS — DAG historii, replay na kernelu B-rep, solver więzów szkicu 2D.
Potwierdza, że ten zakres (SurfaceCraft §6.4 / M5) jest właściwie zakrojony; jedyny nowy drobiazg: domyślne
zapamiętywanie ostatnio użytego Result Operator per-komenda — tani, warty dodania do UX (nie do algorytmu).

## 7. Obsługa dużych chmur punktów (out-of-core) — potwierdzenie architektoniczne

*Obserwacja:* "Unlimited Point Cloud Capacity" — cache na dysku gdy chmura przekracza RAM, adaptacyjna gęstość
wyświetlania wg LOD/viewpointu, 3 tryby cieniowania do analizy (`Depth`, `X-Ray`, `Height`), tryb
`Walk-through` (kamera wewnątrz modelu — dla dużych obiektów: budynki, hale). Format `.xpc` zoptymalizowany pod
masywne chmury. Selekcja: `Extend To Similar` / `Extend To Sudden Change` (rozrost zaznaczenia od punktu
zalążkowego po podobieństwie/krawędzi).
*Różnica vs QS:* QuickSurface (z analizy #1) też wspomina AutoSurface "out-of-core", ale GDX dokumentuje to
znacznie szerzej jako cechę całej aplikacji (nie tylko jednej komendy). Było to wkładem do pytania o skalę
skanów, **rozstrzygniętego** w `2026-11-08-surfacecraft-scan-to-cad-rust-design.md` §9/§12 (MVP: do ~10 mln
trójkątów in-memory; architektura rozszerzalna do 100 mln+ przez out-of-core/LOD). Potwierdza, że
**in-core-only** jest uzasadnionym uproszczeniem na MVP, bo docelowe skany nie sięgają metrologii wielkoskalowej
(budynki/hale); dla pojedynczych części (typowy RE mechaniczny) in-memory wystarcza.

## 8. LiveCapture, Scanner Direct Control, LiveTransfer, Batch/Scan Process Designer — świadomie poza zakresem

*Obserwacja:* bezpośrednie sterowanie skanerem/sondą (LiveCapture: LiveAlign/LiveModel/LiveGeometry/Probe
Location, dziesiątki producentów CMM/skanerów), natywny eksport cech parametrycznych do 5 CAD-ów (LiveTransfer),
wizualny edytor procesów wsadowych (Batch Process Designer, Scan Process Designer) i makro-skrypt
(`rfAddinScript.dll`+`DecodeScript.py`). Każdy z tych obszarów to osobny, duży workstream integracyjny
(sterowniki sprzętowe, API partnerskie CAD-ów).
*Rekomendacja:* **potwierdzić** istniejące nie-cele SurfaceCraft (`2026-11-08-surfacecraft-scan-to-cad-rust-design.md` §3) — brak sterowania
sprzętem, brak push-do-CAD, plikowy STEP/IGES wystarcza. Jedyny element wart odnotowania do roadmapy: prosty
**macro recorder / batch runner** nad istniejącą architekturą command-driven (§5 "Zasada command-driven") —
tani UX dodatek po M5/M6, bo infrastruktura (komendy jako JSON) już i tak jest w planie.

---

## Wzorce warte powtórzenia w SurfaceCraft (konkretne rekomendacje)

1. **Region jako encja pierwszej klasy** (§1 wyżej) — rozszerzyć `fit`/`doc` o trwały typ `Region` z
   `auto_segment`/`resegment`/`merge`/`split`/`insert`/`separate`, nie tylko doraźne zaznaczenie trójkątów do
   fitu. Podnosi jakość UX ekstrakcji prymitywów (§6.2 w design doc) i daje wielokrotnego użytku selekcję do
   pomiaru/raportu odchyłki.
2. **Redesign-Assistant-style podpowiadanie parametrów** (§2 wyżej) — gdy użytkownik tworzy cechę (fillet,
   draft, sweep, revolve…), silnik `fit` powinien od razu zaproponować promień/kąt/oś/odległość wyliczone z
   mesha w okolicy, zamiast czekać na ręczny input. Tani koncepcyjnie (ten sam LSQ co ekstrakcja prymitywów),
   duży efekt UX — warto wpleść w M5 jako integralną część kreatorów cech, nie osobny moduł.
3. **CAD Correct / Refit do istniejącej bryły** (§4 wyżej) — wart rozważenia jako funkcja post-MVP: "popraw
   istniejący model CAD (STEP) do zgodności z nowym skanem" — osobny, wysoko wartościowy use-case
   (inżynieria zmian/as-built), niewymagający nowego kernela (tylko ważony fit powierzchni B-rep + redeploy
   przez `solidcore`).
4. **Druga strategia freeform — sieć łat z krzywych cech** (§5 wyżej) — odnotować w §6.3/§10 design doc jako
   alternatywną (nie zastępczą) ścieżkę do retopo-quad; przydatna dla części z wyraźnymi krawędziami
   charakterystycznymi.
5. **Potwierdzenia bez zmian:** Hybrid modeling, deviation-jako-feedback (Accuracy Analyzer), auto-detekcja
   typu prymitywu po residuum, wieloetapowy ICP dla wielu skanów (Mesh Buildup Wizard) — wszystko już w design
   doc, Geomagic tylko potwierdza słuszność wyboru niezależnym, konkurencyjnym przykładem.
6. **Potwierdzenia nie-celów:** sterowanie sprzętem skanera, push-do-CAD (LiveTransfer), komercyjny
   multi-CAD translator (Spatial/Parasolid-grade) — zostają poza zakresem SurfaceCraft; plikowy STEP/IGES
   pozostaje właściwą decyzją.
7. **Drobne, tanie dodatki do rozważenia później:** zapamiętywanie ostatnio użytego Result Operator per-komenda
   (§6), `Roll/Unroll` dla części nawiniętych/obrotowych (blacha, rury) — niszowe, nie-MVP.

## Materiał źródłowy (instalacja GDX)

Rozpakowane dane z sesji analizy leżały w tymczasowym scratchpadzie i **nie są zachowane** — poniżej, jak je
odtworzyć z instalacji `X:\Geomagic\Geomagic Design X 2026.1`. Wyników rozpakowania nie commitujemy do repo
(materiał Hexagon; clean-room — patrz design doc §9).

- `Help\GeomagicDesignX_UserGuide.chm` — dekompilacja narzędziem systemowym Windows do dowolnego katalogu
  roboczego poza repo: `hh.exe -decompile <katalog_docelowy> "<instalacja>\Help\GeomagicDesignX_UserGuide.chm"`
  → 485 tematów HTML + pełne drzewo TOC (`.hhc`). Obok jest `Help\GeomagicDesignX_Tutorial.chm`.
- `Documentation\GeomagicDesignX_ReleaseNotes.pdf` (wersja 2026.1.0, marzec 2026) — nowe funkcje i poprawki tej wersji.
- Mapa modułów `Bin/*.dll`, `Exchange/code/bin/*.dll` — nazwy plików jako wskazówka architektury (bez
  dekompilacji kodu).
- `Sample/Basic/*.xrl`, `*.stl` — przykładowe projekty (np. `Bottle_with_Regions.xrl`, `LiveTransfer.xrl`,
  `CastingPart.stl`, `SheetMetal.stl`) potwierdzające realne scenariusze użycia (segmentacja regionów,
  live-transfer, odlewy, blacha).
