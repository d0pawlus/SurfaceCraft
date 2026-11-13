# Płat z warunkami (`surface.patch`): jeden przycięty płat B-spline na obrys n-boczny — projekt

- Data: 2026-11-13 (zastępuje wersję z 2026-11-12, która proponowała n łat zamiast jednego płata).
- Status: **do akceptacji przez właściciela**.
- Dotyczy: nowej komendy `surface.patch` i jej użycia przez `surface.from_curves`.

> **Aktualizacja 2026-11-13 (po przeniesieniu na aktualny `main`, 6ae45ea).** Dokument powstał
> na checkoucie o 335 commitów starszym niż `main`. Na `main` jest już część rzeczy, które niżej
> są opisane jako „do zrobienia”; obowiązują te poprawki:
>
> - `kernel::nurbs::trimmed_sheet(surface, outer_uv, holes)` **istnieje** (P8-H4): pętle jako
>   wielokąty w (u, v), krawędzie jako łamane na płacie zagęszczane do strzałki 0,004 mm. Krok 1
>   z §9 odpada; `surface.patch` używa tej funkcji. `SurfaceProbe::closest_uv` też istnieje.
> - `surface.from_curves` **obsługuje już** regiony o dowolnej liczbie rogów (P8-H9,
>   `fit/src/features_ngon.rs`): gdy którykolwiek region nie jest czworoboczny, wszystkie są
>   dzielone w środku na łaty czworoboczne. Automat z §6 nie zastępuje tej ścieżki po cichu:
>   dochodzi opcja `ngon: "split"` (domyślnie, zachowanie dzisiejsze) albo `"patch"` (jeden
>   przycięty płat na region nieczworoboczny, bez globalnego podziału). Zmiana wartości
>   domyślnej na `"patch"` dopiero po pomiarze łączenia [C]. Dane regionu (`RegionRec`: rogi,
>   boki, punkty, trójkąty) są już zbierane, więc „praca w `features.rs`” z §6 się zmniejsza.
> - Ciągłość styczna w sieci łat (`surface.wrap`, `continuity: g1`, P9-O2, `fit/src/patch_g1.rs`)
>   jest już na `main` w wersji najmniejszych kwadratów; załącznik A jest tylko notatką z prób.
> - Spłaszczanie i obrys regionu: `fit::outline::region_outline` istnieje; `fit/src/param.rs`
>   nadal jest do zrobienia.
> - Plan wdrożenia: `docs/superpowers/plans/2026-11-13-surface-patch.md`.

Oznaczenia: **[W]** decyzja właściciela, **[C]** decyzja Claude (do sprawdzenia przy czytaniu),
**[P]** wynik próby (kod prób usunięty, liczby w §3).

## 1. Po co

SurfaceCraft ma dostać bezpośredni odpowiednik narzędzia klasy „constrained surface”: dla
zamkniętego obrysu o dowolnej liczbie boków powstaje **jeden** płat B-spline, przycięty do
obrysu, który dokładnie (w tolerancji) spełnia warunki na bokach — G0, G1 albo G2 względem
sąsiednich ścian — i przechodzi przez zadane punkty i krzywe wewnętrzne albo dopasowuje się do
punktów skanu [W]. Narzędzie działa i ręcznie (komenda z CLI, MCP i palety), i w automacie [W].

Dziś `surface.from_curves` robi łatę tylko z regionu o czterech narożnikach; pozostałe regiony
trafiają do `skipped`, więc sieć ma dziury. Ręcznego wypełniania obrysu nie ma w ogóle.

## 2. Cele, nie-cele, anty-cele

**Cele**

- Jeden płat na obrys o 1 lub więcej bokach (zamknięta pojedyncza krzywa też jest obrysem).
- Ciągłość na każdym boku osobno: G0, G1 lub G2 względem wskazanej ściany sąsiedniej.
- Warunki wewnętrzne: punkty (twarde albo z wagą), krzywe (G0), chmura punktów skanu (dopasowanie).
- Wywołanie ręczne i z automatu od pierwszej wersji [W]; automat = `surface.from_curves` dla
  regionów innych niż czworoboczne.
- Wynik zawsze zmierzony: osiągnięta szczelina, kąt i skok krzywizny na każdym boku, odchyłka
  od danych, lista warunków pominiętych lub sprzecznych.

**Nie-cele (ta wersja)**

- G1 w sieci łat `surface.auto` (dług P4). To inny problem; materiał z prób jest w załączniku A
  i czeka na osobny projekt [C].
- Dedykowane narzędzie interaktywne w GUI (gizmo, podgląd na żywo). Komenda jest dostępna z
  palety jak każda inna; wskazywanie krawędzi i ścian idzie przez istniejące zaznaczanie [C].
- Siły i ciśnienie jako dodatkowe obciążenie płata [C — narzędzia tej klasy to mają; do
  dodania później bez zmiany architektury, bo to tylko prawa strona układu].
- Wewnętrzne pętle przycięcia (dziury w płacie), płaty zamknięte/okresowe, wagi wymierne.
- Obrys, którego nie da się jednoznacznie rozłożyć na płaszczyźnie (patrz §4.2), bez danych
  siatki: komenda odmawia z przyczyną.

**Anty-cele**

- Poluzowanie progów z §11.1 design doc albo pogorszenie `gate_p3_block_with_freeform_top`.
- Ciche „best effort”: warunek niespełniony, sprzeczny lub pominięty musi być w wyniku z liczbą.
- Kod, nazwy, stałe lub struktura z jakiegokolwiek zamkniętego produktu. Podstawą są wyłącznie
  publikacje z §10 i publiczne API solidcore. Lokalne materiały analityczne właściciela nie są
  źródłem implementacji i nie trafiają do repo. Nazwa komendy i teksty nie używają cudzych
  znaków towarowych.

## 3. Co ustaliły próby [P]

Pin solidcore `4f2da85`.

1. **Przycięta ściana na płacie B-spline działa.** Rekord z jedną ścianą `Surface::Spline`,
   pętlą n krawędzi, pcurve'ami w (u, v) i krzywymi 3D dopasowanymi do obrazu pcurve'ów jest
   przyjmowany przez `Body::from_record`, tesselowany z polem zgodnym do 0,03 % (obrysy 3-, 5-,
   7-boczne i niewypukły 10-boczny) i przechodzi STEP tam i z powrotem po pogrubieniu.
2. **Pogrubienie wymaga krawędzi na płacie do ok. 1e-5 mm.** Gdy krzywa 3D krawędzi odbiega od
   płata o 2e-4 mm lub więcej, `thicken` zawodzi („not a closed solid”), **niezależnie od
   deklarowanej tolerancji krawędzi** (1e-4 i 1e-2 dały to samo). Przy odchyleniu ≤ 5e-5 mm
   działa. Zmierzone tylko dla pogrubienia jednej ściany; czy zamknięta powłoka, objętość, STEP
   i operacje boolowskie mają to samo wymaganie, **nie jest sprawdzone** i rozstrzyga o §6.
3. **Wypełnienie n-boczne z kernela** (`fill_n_sided_hole`: n łat przez podział w środku)
   mieści się w 0,2 mm tylko dla regionów łagodnych lub małych. Nie jest używane w tym
   projekcie, bo właściciel chce jednego płata; zostaje jako punkt odniesienia do porównań.

## 4. Rozwiązanie

### 4.1 Komenda `surface.patch`

Wejście (wszystko jako JSON Schema; programowe wywołanie nie otwiera okien):

- `sides`: lista boków obrysu. Bok to jedno z:
  - krawędź bryły lub arkusza: `{ body, edge }`,
  - łamana w przestrzeni: `{ points: [[x, y, z], …] }` (co najmniej 2 punkty). Dwa punkty to
    odcinek; więcej punktów jest najpierw wygładzane do krzywej B-spline w `side_tolerance`
    (domyślnie 0,001 mm; dla zaszumionych punktów trzeba ją podnieść), a G0 jest liczone
    względem tej krzywej; błąd wygładzenia jest w raporcie [C],
  - bok regionu siatki: używany wewnętrznie przez automat (krzywa cechy).
  Każdy bok ma `continuity`: `g0` (domyślnie), `g1`, `g2`, oraz dla `g1`/`g2` ścianę
  odniesienia `{ body, face }`. Dla krawędzi bryły ściana odniesienia jest wyznaczana sama,
  gdy krawędź ma dokładnie jedną ścianę po stronie przeciwnej do płata; inaczej trzeba ją podać.
- Boki nie muszą być podane w kolejności; komenda składa z nich jedną zamkniętą pętlę. Przerwa
  między końcami większa niż `gap_tolerance` (domyślnie 0,01 mm) to błąd z położeniem przerwy.
  Mniejsze przerwy są zamykane przed liczeniem: końce dostają wspólny narożnik, a bok z łamanej
  jest do niego dociągany (krawędzie brył się nie zmieniają, narożnik leży na nich), więc każdy
  narożnik ma jeden cel G0 [C].
- `points`: punkty wewnętrzne, każdy z `weight` albo `hard: true`.
- `curves`: łamane wewnętrzne, przez które płat ma przejść (G0, twarde).
- `region` albo `mesh` + `triangles`: punkty skanu do dopasowania (miękkie).
- `tolerance` (odchyłka od danych, domyślnie 0,2 mm), `smoothing` (względna sztywność,
  domyślnie dobrana automatycznie), `control` (rozmiar sieci, domyślnie automatyczny),
  `max_control` (domyślnie 16, najwyżej 24 na kierunek [C]), `conflicts`: `relax` (domyślnie)
  albo `fail` (§4.5), `name`.

Wyjście: obiekt powierzchni — arkusz z jedną przyciętą ścianą, `parent: None` (pole `parent`
oznacza „ta sama geometria” i `surface.thicken` bierze je w pierwszej kolejności, więc płat
nieprzycięty nie może tam trafić) — oraz raport z §7. Komenda **zwraca arkusz i nie wszywa go
w bryłę**, także gdy boki są krawędziami bryły; wszycie to osobny, późniejszy krok [C].

Nie ma ciągłości między bokami podanej osobno: narożniki wynikają z boków.

### 4.2 Przebieg

1. **Obrys.** Złożenie pętli, kontrola zamknięcia, orientacja (przeciwnie do wskazówek zegara
   względem normalnej: z normalnych ścian odniesienia lub regionu; bez nich z płaszczyzny
   najlepszego dopasowania), próbkowanie boków.
2. **Parametry startowe (u, v).**
   - Jest siatka regionu: spłaszczenie trójkątów regionu (mapa harmoniczna z brzegiem na
     wielokącie wypukłym, potem kilka kroków ARAP) — `crates/fit/src/param.rs`, czyste
     obliczenia na siatce.
   - Nie ma siatki: rzut obrysu i warunków na płaszczyznę najlepszego dopasowania. Jeśli rzut
     obrysu sam się przecina albo normalne ścian odniesienia odchylają się od normalnej
     płaszczyzny o więcej niż 75° [C], komenda odmawia („obrys nie jest polem wysokości; podaj
     region siatki”). Rozszerzenie na później: odwzorowanie obrysu na wielokąt wypukły według
     długości łuku, które zniosłoby tę odmowę dla wejść bez danych wewnętrznych.
3. **Dziedzina.** Prostokąt w (u, v) obejmujący obrys z marginesem jednej komórki sieci
   kontrolnej; płat jest potem przycinany. Margines usuwa efekt brzegu przy dopasowaniu.
4. **Rozwiązanie z warunkami** (§4.3), potem **korekta parametrów punktów danych**: każdy punkt
   wewnętrzny jest rzutowany na aktualny płat (`surface_closest_point` solidcore daje (u, v)).
   Parametry próbek brzegu nie są rzutowane (rzut próbki z twardym G0 zwraca ją samą); obraz
   obrysu w (u, v) pochodzi ze spłaszczenia i zostaje ustalony na cały przebieg.
5. **Zagęszczanie.** Jeśli po rundzie któraś miara przekracza cel (szczelina G0, kąt G1,
   odchyłka od danych), sieć rośnie tam, gdzie reszta jest największa (wstawienie węzłów przez
   solidcore), do `max_control`. Pętla 4–5 kończy się, gdy cele są spełnione, gdy runda nie
   poprawia najgorszej miary o więcej niż 5 %, albo po 6 rundach [C].
6. **Przycięcie.** Obraz obrysu w (u, v) (rzuty próbek boków) → pcurve'y; krzywe 3D krawędzi
   = krzywe na płacie dopasowane do ≤ 1e-6 mm (wymóg z §3 pkt 2); budowa ściany w kernelu.

### 4.3 Układ

Płat tensorowy B-spline stopnia 3 × 3, niewymierny, węzły zaciśnięte. Niewiadome: wszystkie
punkty kontrolne.

Minimalizowane:

- energia zginania `∫∫ ‖S_uu‖² + 2‖S_uv‖² + ‖S_vv‖²` (kwadratura Gaussa na przedziałach węzłów)
  liczona tylko po komórkach, które przecinają obrys lub leżą w jego wnętrzu, plus jedna
  warstwa marginesu — to ona wypełnia obszary bez danych. Komórki poza obrysem dostają tylko
  słabą regularyzację; inaczej energia sprzęgałaby ramiona obrysu niewypukłego w poprzek wcięcia;
- suma ważonych kwadratów odległości punktów danych od płata w ich (u, v).

`smoothing` ustala stosunek obu członów; wartość automatyczna jest skalowana do śladów macierzy,
żeby nie zależała od jednostek i gęstości danych.

Warunki, wszystkie liniowe względem punktów kontrolnych:

| Warunek | Wiersze | Uwagi |
|---|---|---|
| G0 na boku | `S(u_k, v_k) = C(t_k)` | |
| G1 na boku | `S_d(u_k, v_k) · n_k = 0` | `d` = kierunek w (u, v) prostopadły do obrazu boku, `n_k` = normalna ściany odniesienia. Tylko wiersz poprzeczny: wiersz wzdłuż boku wynika z G0 i byłby prawie zależny |
| G2 na boku | `S_dd(u_k, v_k) · n_k = κ_k · ‖S_d‖²` | `κ_k` = krzywizna normalna ściany odniesienia w kierunku `S_d`; prawa strona z poprzedniej rundy (linearyzacja); wymaga G1 na tym boku |
| Punkt twardy | `S(u, v) = Q` | |
| Krzywa wewnętrzna | jak G0 | |

**Twarde w punktach kolokacji, mierzone gęsto.** Wiersze boków są wymuszane dokładnie w
punktach kolokacji, których liczba jest dopasowana do swobody płata przy brzegu (nie więcej
niż jeden zestaw wierszy na przedział węzłów przecinany przez bok, zawsze w narożnikach) —
gęstsze twarde próbkowanie przewymiarowałoby układ i dawało fałszywe sprzeczności. Między
punktami kolokacji płat nie leży dokładnie na boku: **osiągnięta szczelina, kąt i błąd
krzywizny są mierzone na gęstej siatce próbek** (≥ 20 na przedział) i to te liczby trafiają do
raportu i sterują zagęszczaniem. Dodatkowe próbki między punktami kolokacji wchodzą do członu
miękkiego z dużą wagą.

**Czego układ sam nie wykryje — kontrole geometryczne.**

- Wiersze G1 mają zerową prawą stronę, więc nigdy nie są sprzeczne algebraicznie: dwie różne
  normalne w narożniku wymuszają po cichu zdegenerowany płat. Dlatego zgodność narożników jest
  sprawdzana **przed** rozwiązaniem: kąt między płaszczyznami stycznymi ścian odniesienia
  dwóch boków G1/G2 w narożniku oraz to, czy styczna boku G0 leży w płaszczyźnie sąsiedniego
  boku G1. Próg zgodności 0,5° [C].
- Wiersz G1 nie zależy od znaku normalnej, więc płat zawinięty z powrotem na ścianę
  odniesienia (ostry „dziób”) spełnia go tak samo jak gładkie przejście. Po rozwiązaniu jest
  sprawdzane, czy `S_d` wychodzi od ściany odniesienia na zewnątrz jej obrysu; kąty w raporcie
  są ze znakiem. Zawinięcie to błąd boku, nie sukces.

Rozwiązanie: rozkład macierzy warunków z wyborem elementu głównego (QR z pivotem lub SVD) →
rząd, baza jądra, rozwiązanie szczególne; minimalizacja w jądrze (Cholesky macierzy
zredukowanej). Macierze gęste, `nalgebra` (jest w workspace, działa w wasm). Bez boków G1/G2
współrzędne się rozprzęgają (jedna macierz n·m × n·m, trzy prawe strony). Z G1/G2 układ jest
wspólny: przy 24 × 24 to 1728 niewiadomych skalarnych, macierze po ok. 24 MB i rząd 2e9
operacji na rundę. Limit 24 wynika z tego budżetu (wasm, kilka rund) [C]; większe sieci
wymagałyby solvera pasmowego lub rzadkiego i są poza tą wersją.

Dane zdegenerowane: bez żadnych danych wewnętrznych zadanie nadal ma jedno rozwiązanie
(energia + warunki brzegowe); mała regularyzacja na przekątnej usuwa jądro energii, gdy
warunków jest za mało.

### 4.4 Sąsiednie ściany jako odniesienie

`kernel` dostaje zapytanie: dla ściany bryły i punktu przy jej krawędzi — normalna i krzywizna
normalna w zadanym kierunku stycznym (przez zapytania solidcore; dla ścian analitycznych
i B-spline tak samo). Normalna jest brana po tej stronie, która daje gładkie przejście, a nie
ostrą krawędź: znak ustala orientacja pętli.

W narożniku obrysu, gdzie spotykają się dwa boki G1 o różnych ścianach odniesienia, płat może
być styczny do obu tylko wtedy, gdy ich płaszczyzny styczne się tam zgadzają. To częsty
przypadek, nie wyjątek: przy wypełnianiu otworu w bryle ściany odniesienia zwykle spotykają
się pod kątem.

### 4.5 Warunki zależne i niezgodne narożniki

- Wiersze **zależne** (powtórzone informacje) są pomijane; raport podaje ich liczbę.
- **Niezgodny narożnik** (kontrola z §4.3). Przy `conflicts: relax` (domyślnie [C]) warunki
  G1/G2 obu boków są w otoczeniu narożnika zdejmowane z twardych i przechodzą do członu
  miękkiego z wagą malejącą do zera w narożniku; G0 zostaje twarde. Wynik wraca, a raport
  podaje dla każdego takiego narożnika kąt niezgodności, długość strefy złagodzenia i osiągnięty
  kąt na bokach. Przy `conflicts: fail` komenda kończy się błędem z tą samą listą, bez wyniku.
- Nigdy po cichu: każdy warunek niespełniony w celu jest w raporcie, a `within_targets` jest
  wtedy `false`.

## 5. Gdzie to mieszka

| Co | Gdzie | Uwagi |
|---|---|---|
| Przycięta ściana z płata i pętli w (u, v) | `crates/kernel/src/nurbs.rs`: `trimmed_sheet` | konstrukcja z próby §3 pkt 1 |
| Normalna i krzywizna ściany odniesienia | `crates/kernel/src/nurbs.rs` lub `hybrid.rs` | zapytania solidcore |
| Solver (energia, warunki, rozwiązanie, zagęszczanie) | `crates/kernel/src/vpatch.rs` *(nowy)* | baza i pochodne, wstawianie węzłów, rzut punktu przez solidcore; po stronie aplikacji montaż i rozwiązanie układu |
| Spłaszczenie regionu siatki | `crates/fit/src/param.rs` *(nowy)* | obliczenia na siatce, bez wyjątku |
| Komenda | `crates/engine/src/cmd/surface_patch.rs` *(nowy)* | `surface.patch` |
| Automat | `crates/fit/src/features.rs`, `crates/engine/src/cmd/curves.rs` | §6 |

Solver w `kernel`, nie w `fit` [C]: potrzebuje funkcji bazowych, wstawiania węzłów i rzutu
punktu, a tylko `kernel` może używać solidcore; dzięki temu po stronie aplikacji zostaje
najmniej geometrii. `kernel` dostaje zależność `nalgebra` (w workspace i na liście licencji;
`kernel` i `fit` leżą w tej samej warstwie — `cargo xtask layers` do potwierdzenia przez
wykonawcę).

**Wyjątek app-geometry [W, 2026-11-12].** `crates/kernel/src/vpatch.rs` dostaje nagłówek
`APP-GEOMETRY:` („stands in for a solidcore constrained variational surface fit: n-sided
boundary, G0/G1/G2 sides, interior points and curves”), wpis w `xtask/app-geometry.txt` z datą
zgody i dług w `docs/STATUS.md`. Lista rośnie o ten jeden plik. `trimmed_sheet` składa rekord
tak, jak dziś `sheet_from_spline`, więc nie jest nowym rodzajem wyjątku.

## 6. Automat i łączenie z sąsiadami

`surface.from_curves`: region o liczbie narożników innej niż 4 (także 1–3 i 5+) i jednym
obrysie idzie do tego samego solvera: boki = krzywe cech (G0 — krawędzie ostre zostają ostre),
dane = wierzchołki regionu, parametry startowe = spłaszczenie trójkątów regionu. Regiony
czworoboczne zostają na dzisiejszej ścieżce Béziera [C]. Regiony z dziurami zostają w `skipped`.

Praca w `features.rs`, której dziś nie ma:

- regiony inne niż czworoboczne są dziś odrzucane **przed** dopasowaniem krzywych boków, więc
  bok między dwoma takimi regionami nie ma krzywej. Krzywe boków muszą powstawać dla
  wszystkich regionów z jednym obrysem i być współdzielone jak dziś;
- narożniki są środkami skupisk wierzchołków, nie wierzchołkami siatki, więc brzeg
  spłaszczonej siatki nie pokrywa się z krzywymi boków. Obraz boku w (u, v) bierze się z
  wierzchołków brzegu regionu między narożnikami, a próbki krzywej boku są do nich przypisywane
  proporcjonalnie do długości łuku.

**Łączenie z sąsiadami — główne ryzyko projektu.** Dla krzywego boku płat bikubiczny o sieci
do 24 punktów nie zejdzie ze szczeliną do krzywej cechy poniżej rzędu 1e-4 mm (błąd
interpolacji splajnem sześciennym rośnie z czwartą potęgą długości przedziału węzłów), a
`thicken` potrzebuje 1e-5 mm (§3 pkt 2). Trzeba więc **założyć**, że wspólna krawędź będzie
leżała na płacie n-bocznym i na sąsiedzie tylko w tolerancji rzędu 1e-4–1e-3 mm.

- Krawędź przyciętego płata jest jego własną krzywą (na płacie do 1e-6 mm).
- Do zrobienia w kernelu: złożenie jednej powłoki z sieci Béziera i arkuszy przyciętych
  (dziś `nurbs::stitch` przyjmuje jeden rekord i nic tych arkuszy nie łączy), potem szycie z
  tolerancją równą największej osiągniętej szczelinie.
- **Pomiar przed wpięciem (§9 krok 4, wcześnie):** na bloku z jednym regionem n-bocznym
  sprawdzić, przy jakiej szczelinie zszyta powłoka jest przyjmowana jako zamknięta bryła, daje
  objętość, przechodzi STEP i operację boolowską. Od wyniku zależy, czy `from_curves` z
  regionami n-bocznymi zgłasza bryłę.
- Jeśli zamknięcie nie jest wiarygodne przy osiągalnej szczelinie: wynik to arkusz z
  `closed: false`, osiągniętą szczeliną i ostrzeżeniem; bryła nie jest zgłaszana, a decyzja
  wraca do właściciela. Przygotowane możliwości: zgłoszenie do solidcore (tolerancja krawędzi
  nie jest honorowana w `thicken`) oraz odwrócenie kolejności — najpierw płat n-boczny, potem
  jego własne krzywe brzegowe jako boki sąsiednich łat czworobocznych (nie pomaga, gdy
  sąsiadują dwa regiony n-boczne).

Arkusze i odchyłka od skanu nie zależą od tego ryzyka; zależy od niego tylko zamknięta bryła.

## 7. Raport

`net`, `degree`, `rounds`, odchyłka od danych (`rms`, `p99`, `max`, liczba punktów),
`within_tolerance` (dane) i `within_targets` (wszystkie cele boków); dla każdego boku: żądana ciągłość, `gap_max_mm`, `angle_max_deg` (G1/G2),
`curvature_error_max` (G2, względny), liczba próbek; `dropped` (zależne), `conflicts`
(sprzeczne, z położeniem i resztą), `warnings` (m.in. „obszar bez danych wypełniony energią”,
„sieć osiągnęła max_control”, „za mało danych na rozmiar sieci”).

Wejście jest wrogie: `hostile_params_never_panic`, brak `unwrap/expect/panic/unsafe`, limity
(boki ≤ 64, punkty łamanej ≤ 10 000, punkty danych przycinane deterministycznie do 50 000 [C]),
brak `HashMap` w ścieżce liczbowej.

## 8. Bramki i testy

Istniejące bez zmian: `gate_p3_block_with_freeform_top`, `organic.rs`, `freeform.rs`, progi §11.1.

Cele liczbowe nowej funkcji, mierzone gęsto między punktami kolokacji [C, do potwierdzenia]:
G0 ≤ 1e-3 mm (boki proste i łagodne dochodzą znacznie niżej; liczba osiągnięta jest w
raporcie); G1 ≤ 0,05° poza strefami złagodzenia przy niezgodnych narożnikach; G2: względny błąd
krzywizny normalnej ≤ 5 %; dopasowanie ≤ `tolerance` (0,2 mm).

| Test | Kryterium |
|---|---|
| `trimmed_sheet_n_sided` (kernel) | obrysy 3-, 5-, 7-boczny i niewypukły: `from_record` przyjmuje, pole z tesselacji w 0,1 %, pogrubienie i STEP zachowują liczbę ścian |
| `vpatch_recovers_known_surface` | próbki ze znanego płata bez szumu, obrys prostokątny G0 → odchyłka ≤ 1e-6 względem rozmiaru |
| `vpatch_boundary_only` | sam obrys 5-boczny na płaszczyźnie → płat płaski do 1e-9; na czaszy kuli → gładki, bez fal (stosunek p99/mediana krzywizny ≤ 5) |
| `vpatch_g1_to_cylinder`, `vpatch_g2_to_cylinder` | otwór 5-boczny w walcu (wszystkie boki mają tę samą ścianę odniesienia, narożniki zgodne) wypełniony z G1/G2: kąt ≤ 0,05°, błąd krzywizny ≤ 5 % |
| `vpatch_incompatible_corner` | dwa boki G1 o płaszczyznach różnych o 30° w narożniku: `relax` daje wynik, strefę i kąty w raporcie, `within_targets = false`; `fail` daje błąd z położeniem |
| `vpatch_rejects_fold` | bok G1 z płatem zawiniętym na ścianę odniesienia jest zgłaszany jako błąd boku |
| `vpatch_noise_stays_smooth` | σ = 0,03, 1600 punktów → rms w ±20 % σ |
| `vpatch_nonconvex_outline` | obrys w kształcie U z danymi tylko w ramionach: ramiona nie wpływają na siebie (odchyłka jak dla każdego osobno ±10 %) |
| `param_flattening_is_bijective` (fit) | regiony 3-, 5-boczny i wcięty: brak odwróconych trójkątów |
| `surface_patch_manual` (engine) | arkusz falisty z otworem 5-bocznym: `surface.patch` po krawędziach otworu z G1 do arkusza → arkusz, raport w celach |
| `surface_patch_box_hole` (engine) | blok z usuniętą ścianą, boki G1 do ścian bocznych (narożniki niezgodne o 90°, poza regułą 75° dla rzutu): komenda odmawia z czytelną przyczyną albo, z G0, daje płaską ścianę do 1e-9 |
| `from_curves_fills_n_sided` (engine) | bryła z regionem 3- i 5-bocznym, σ = 0,03: `skipped` puste, max ≤ 0,2 mm |
| `from_curves_closes_with_n_sided` | jw.: sieć zamknięta i bryła — albo uczciwe `closed: false` z osiągniętą szczeliną (§6) |
| wrogie parametry | `surface.patch` i nowe ścieżki `from_curves` |

Lokalnie (poza CI): korpus lokalny P1 — liczba regionów pominiętych przed/po i czas.

## 9. Kolejność

1. Kernel: `trimmed_sheet` + test.
2. Solver: energia + G0 + punkty na dziedzinie prostokątnej (testy odtworzenia i szumu).
3. Obrys, rzut na płaszczyznę, przycięcie → `surface.patch` z G0 (ręcznie, bez siatki).
4. Pomiar łączenia z §6 (arkusz przycięty + sąsiednie łaty, powłoka, bryła, STEP, boolean) —
   wcześnie, bo rozstrzyga, co automat może obiecać.
5. G1, potem G2: zapytania o ścianę odniesienia, kontrole narożników i zawinięcia.
6. `features.rs` (krzywe boków dla wszystkich regionów) + `param.rs` (spłaszczenie regionu) →
   automat w `from_curves`; bryła tylko, jeśli krok 4 na to pozwolił.
7. STATUS.md, przewodniki (`docs/guides/fit-surfaces.md`, `build-solid.md`), lista app-geometry.

Kroki 1–3 dają działające narzędzie ręczne z G0. Krok 5 zależy od 2–3, krok 6 od 2–4.

**Upstream [W].** tangency-labs/solidcraft#759 („Patch on edges: G1/G2, N-sided loops,
minimum-energy fit, guide points”) opisuje dokładnie tę funkcję po stronie kernela, a #11672
prosi o nią wprost; nowe zgłoszenie byłoby duplikatem. Po kroku 4 powstaje w `docs/upstream/`
szkic komentarza do #759 (wyniki, wymaganie 1e-5 mm z §3 pkt 2 jako osobna obserwacja z
reprodukcją na samym API solidcore); wysyłka po akceptacji treści przez właściciela.

## 10. Literatura (podstawa implementacji)

- Piegl, Tiller, *The NURBS Book*, 2. wyd., 1997 — baza i pochodne, wstawianie węzłów, rzut
  punktu, dopasowanie z warunkami.
- Welch, Witkin, *Variational surface modeling*, SIGGRAPH 1992 — energia i warunki liniowe.
- Celniker, Gossard, *Deformable curve and surface finite elements for free-form shape design*,
  SIGGRAPH 1991 — energia zginania płata z warunkami.
- Floater, *Parametrization and smooth approximation of surface triangulations*, CAGD 1997;
  Liu i in., *A local/global approach to mesh parameterization*, SGP 2008 — spłaszczanie.
- Golub, Van Loan, *Matrix Computations* — metoda jądra, QR z wyborem elementu głównego.

## 11. Co zostaje wykonawcy

Sygnatury i podział na funkcje, dobór automatycznego `smoothing` i startowego rozmiaru sieci
(kryterium: testy szumu i odtworzenia), gęstość próbek warunków, strategia zagęszczania,
kształt zapytania o ścianę odniesienia, szczegóły składania pętli z boków.

## Załącznik A — materiał do osobnego projektu: G1 w sieci łat `surface.auto`

Próba z 2026-11-12: łaty Coonsa z solidcore na wspólnych polach pochodnych zmniejszają skok
normalnych 2–3× (kula max 1,30° → 0,58°, p95 0,49° → 0,15°; blob p95 0,31–0,54° → 0,12–0,17°),
ale kernel zgłasza niezgodność skrętu w narożnikach prawie każdej łaty. Wniosek z recenzji:
warunki w narożniku są sprzeczne przy każdej walencji, jeśli pola normalnych krawędzi są
dobierane niezależnie; potrzebny jest wspólny symetryczny operator kształtu w każdym
wierzchołku układu (z dopasowania kwadryki do skanu), pola normalnych krawędzi z pochodnymi
końcowymi zgodnymi z tym operatorem, i łaty stopnia 5 dopasowywane z warunkiem `D · n = 0`
wymuszanym w 2p + 3 punktach na przedział. Dług P4 w STATUS.md pozostaje otwarty.

## Wynik wdrożenia (2026-11-13)

Wdrożono w gałęzi `surface-patch`: siedem commitów na `main` 6ae45ea. `18efcbd` projekt i plan;
`5073cdb` solver `kernel::vpatch` i spłaszczanie `fit::param`; `aec35cb` sterownik
`kernel::patch_fit`; `656e9c4` i `1939c00` poprawki sterownika i pomiar łączenia; `ede16a4`
komenda `surface.patch`; `866968e` opcja `ngon` w `surface.from_curves`. Treść powyżej nie została
zmieniona. Poniżej wyniki i odstępstwa od projektu.

### Zmierzone (dane syntetyczne)

- Pięciobocny otwór w walcu R = 20 mm, same boki (tylko obrys): G1 → siatka 6×6, szczelina do
  0,0006 mm, kąt do 0,047°, 91 ms. G2 → siatka 9×9, szczelina do 0,0004 mm, kąt do 0,016°, błąd
  krzywizny normalnej do 2,0 %, 187 ms. G0 → szczelina do 0,001 mm.
- Falista powierzchnia z = 3·sin(x/12)·cos(y/9), region 40 × 24 mm, 2150 punktów, szum σ = 0,03 mm,
  cztery twarde boki zadane łamanymi po 401, 41 lub 21 punktów: w każdym przypadku siatka 13×13,
  jedna runda, rms 0,030 mm, max 0,100 mm, szczelina boków 0,00018 mm.
- Płaszczyzna z szumem σ = 0,03 mm, pięciokąt R = 20 mm, boki z tym samym szumem: twarde boki
  rms 0,110 mm, max 0,60 mm, `within_targets` = false (zgłoszone). Miękkie boki: rms w granicy
  szumu.
- `surface.from_curves` na skanie (σ = 0,03 mm) sześcianu z jednym ściętym narożnikiem (3 regiony
  czworoboczne, 3 pięciobocne, 1 trójbocny). `ngon: "split"`: 30 płatów Béziera, rms 0,031 mm,
  max 0,167 mm, sieć zamknięta. `ngon: "patch"`: 3 płaty Béziera i 4 przycięte płaty, max każdego
  ≤ 0,2 mm, bez bryły.
- Łączenie (blok 40 × 30 × 20 mm; czapa z `fit_patch`: płaska, wypukła o 2 mm, na zakrzywionych
  krawędziach górnych). `nurbs::stitch` nie zszywa osobnych arkuszy przy żadnej tolerancji
  z zakresu 1e-6 … 1e-2 mm, także dla płaskiej czapy o szczelinie 1e-14 mm. Rekord niezszytych
  arkuszy jest odrzucany przy dekodowaniu (`from_record`). Droga hybrydowa (`hybrid::cut_by_spline`
  we wszystkich trzech wariantach, `hybrid::replace_face` dla płaskiej i wypukłej czapy) daje
  zamkniętą bryłę o 6 ścianach; STEP zachowuje ściany i objętość; odcięcie pudełkiem
  10 × 10 × 30 mm się udaje. Błąd objętości: płaska dokładna, zakrzywione krawędzie 5e-5 względnie,
  wypukła 5e-4 względnie. Max odchyłka danych 0,026 mm.
- Testy: biblioteka kernela 93 OK; engine `surface_patch` 7, `curves_ngon_patch` 5, bramka
  `curves_ngon` 1, `commands` 8, `organic` 4; ui-egui `shell` 14; `cargo xtask quick` dla kernel,
  fit, engine i ui-egui OK; `cargo xtask wasm` OK.
- Lokalna uwaga środowiskowa: `cargo xtask wasm` uruchomiono bez sccache. Na tym stanowisku
  sccache kończy się na web-sys błędem 206 Windows (za długa linia poleceń). To problem środowiska,
  nie kodu.

### Odstępstwa od projektu

1. **Boki z skanu są miękkie.** Nowa flaga boku `soft` (`surface.patch`); automat `surface.from_curves`
   używa miękkich boków. Twardy G0 na zaszumionym boku albo na boku z zamazaną ostrą krawędzią dawał
   dopasowanie 2 do 4 razy gorsze niż szum. Projekt zakładał twardy G0 wszędzie. Szczelina boku
   miękkiego jest raportowana, nie jest celem.
2. **Brak `side_tolerance` i wygładzania łamanych przez tolerancję.** Parametr `side_tolerance`
   i wygładzanie B-splajnem w tolerancji zostały usunięte. Bok z co najmniej trzema punktami jest
   zagęszczany wzdłuż gładkiej interpolacji sześciennej przez te punkty (solidcore `crv_fit::p2::interp`);
   zwrot powyżej 45° w punkcie dzieli bok. Zaszumione łamane obsługują
   miękkie boki.
3. **`ngon: "patch"` nie tworzy bryły.** Przycięte płaty wracają jako osobne obiekty powierzchni
   (patrz pomiar łączenia). Regiony z jednym lub dwoma rogami nadal są pomijane.
4. **`surface.patch` zwraca arkusz.** Komenda nie wszywa arkusza w bryłę. Bryłę z takim płatem daje
   istniejąca droga hybrydowa: płat jako cięcie albo jako zastąpiona ściana. Obie funkcje są
   kernelowe, bez polecenia.
5. **Nie zrobiono:** siły i ciśnienia, wewnętrzne pętle przycięcia, dedykowane narzędzie w GUI
   albo pozycja na wstążce (polecenie jest w palecie, CLI i MCP). Domyślna wartość `ngon` pozostaje
   `split`.

### Otwarte

- Zszywanie osobnych arkuszy: zgłoszenie upstream w `docs/upstream/11-decode-unsewn-sheets-refused.md`
  (szkic, niewysłany, wymaga reprodukcji kernel-only).
- Komentarz do tangency-labs/solidcraft#759 w `docs/upstream/10-comment-solidcraft-759-constrained-patch.md`
  (szkic, niewysłany).
