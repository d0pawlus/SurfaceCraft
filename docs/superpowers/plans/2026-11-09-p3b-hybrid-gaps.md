# P3b — Model hybrydowy: luki po P3 (plan szczegółowy)

> Kontynuacja [P3](2026-11-09-p3-hybrid.md) (PR #13). Gałąź bazowa: `p3b-hybrid-gaps` od `origin/p3-hybrid`.
> Zasady bez zmian: B-rep wyłącznie przez solidcore (`crates/kernel`), progi §11.1 nietykalne, lokalnie
> `cargo xtask quick <pakiety>`, pełne CI na PR.

## Diagnoza

`kernel::hybrid::solid_from_cuts` buduje bryłę z kolejnych cięć bloku wyjściowego:
1. każda płaszczyzna = **globalna półprzestrzeń** (`split_body`, zostaje strona materiału),
2. wypukły częściowy walec styczny do dwóch płaszczyzn = fillet ich krawędzi,
3. pełne walce/stożki/sfery/torusy = przecięcie (wypukłe) lub różnica (wklęsłe) z bryłą tego kształtu,
4. płaty = cięcie splajnem.

Krok 1 zakłada, że każda płaszczyzna leży na „otoczce” części. Ścianka lub dno kieszeni, dno otworu
nieprzelotowego czy ścianka rowka to płaszczyzny **lokalne** — jako półprzestrzeń ścięłyby całą bryłę.
Dlatego luki z P3 mają wspólny rdzeń: brakuje rozróżnienia cięć globalnych i lokalnych oraz budowy
lokalnych brył-narzędzi (odjąć / dodać) z grup sąsiednich regionów.

## Zadania

### Z1. Klasyfikacja cięć: globalne vs lokalne (rdzeń, przed Z2–Z4)
- Dla każdej płaszczyzny: test „przed płaszczyzną nie ma skanu” na punktach całej siatki (tolerancja
  ~3σ + 0,1 mm, odporność na outliery: odsetek, nie pojedynczy punkt). Brak skanu przed → globalna
  (półprzestrzeń jak dziś); inaczej → lokalna.
- To samo dla walców/stożków/sfer/torusów: czy region obejmuje pełny obrót i czy kształt jest „otoczką”.
- Graf sąsiedztwa regionów (wspólne krawędzie siatki, istnieje w `merge_adjacent`/`relax_boundaries`)
  z rodzajem styku: wypukły / wklęsły (znak kąta dwuściennego między normalnymi regionów na granicy).
- Komenda diagnostyczna (np. `body.plan_regions`): dla każdego regionu rola (globalna, lokalna-odjąć,
  lokalna-dodać, blend) i powód — ta sama logika, której używa `body.from_regions`.

### Z2. Kieszenie i otwory nieprzelotowe (lokalne narzędzia odejmowane)
- **Otwór nieprzelotowy:** wklęsły pełny walec + dno (płaszczyzna prostopadła do osi lub stożek
  wiertła). Narzędzie: walec ograniczony osiowo — od dna (minimum rzutu punktów regionu na oś albo
  wierzchołek stożka) do poza bryłę po stronie wlotu; z dnem stożkowym = walec ∪ stożek (`make_cone`
  z dodatnim półkątem — patrz Z5a). Odejmowany po krokach globalnych.
- **Kieszeń:** płaszczyzna dna (lokalna) + ścianki (lokalne płaszczyzny i walce naroży, styk wklęsły
  z dnem). Profil = przekrój ścianek płaszczyzną dna, uporządkowany wg grafu sąsiedztwa: odcinki
  (płaszczyzny) i łuki (walce naroży) → `ProfileCurve` → `kernel::feature::extrude` od dna do poza górną
  powierzchnię bryły → odjęcie. Fillet dna kieszeni (wklęsły walec styczny do dna i ścianki) = fillet
  krawędzi po odjęciu (Z3).
- **Rowek / wybranie przelotowe** (ścianki bez dna): ten sam profil, wyciągnięcie przez całą bryłę.
- Przypadki, których nie da się złożyć (profil nie zamyka się), raportowane jako pominięte z powodem.

### Z3. Częściowe kształty jako zaokrąglenia, fazy i lokalne bryły
- **Wklęsły częściowy walec styczny do dwóch płaszczyzn** = fillet krawędzi wklęsłej (naroże kieszeni,
  wewnętrzne zaokrąglenie) — ta sama logika `edge_between` co dla wypukłego, środek po stronie bez materiału.
- **Częściowy stożek przy krawędzi kołowej** = faza tej krawędzi (`solid::chamfer`); przy krawędzi
  prostej — faza o kącie z półkąta.
- **Częściowy torus:** styczny do płaszczyzny i walca o wspólnej osi = fillet krawędzi kołowej
  (wklęsły u podstawy czopa, wypukły na krawędzi tarczy), promień = mały promień torusa.
- **Częściowa sfera:** narożnik trzech zaokrągleń → zaokrąglać krawędzie zbiegające się w wierzchołku
  **jednym wywołaniem** (`ops::blend` z wieloma krawędziami), żeby kernel zrobił łatę narożną; sferę
  porównać z nią (promień). Kopułka (sfera nie styczna do niczego) = lokalna bryła: (kula ∩ półprzestrzeń
  nad płaszczyzną podstawy) ∪ bryła.
- Kolejność: globalne → lokalne odjęcia (Z2) → lokalne dodania → zaokrąglenia/fazy (od największego
  promienia) → płaty.

### Z4. `body.replace_face` dodające materiał
- Dziś: cięcie splajnem — płat może tylko zabrać materiał.
- Nowe: wyciągnięcie płaskiej ściany na zewnątrz o wysokość ponad maksimum płata (`ops::extrude`
  ściany / pętli), suma z bryłą, potem cięcie płatem jak dziś. Płat może leżeć wyżej i niżej niż ściana
  w różnych miejscach. Ograniczenie: wyciągnięcie nie może kolidować z inną geometrią (sprawdzenie
  i komunikat).
- To samo narzędzie jako „lokalne dodanie” w Z3 (wypukły płat nad częścią ściany).

### Z5. Problemy kernela solidcore (pin `b9c02da`)
a) stożek z ujemnym półkątem po `step_export` + `step_brep::import` rozpada się na jednościenne bryły;
b) `ops::blend` zwraca walec zaokrąglenia jako `Cone` o półkącie 0;
c) przecięcie prawie współosiowych powierzchni obrotowych (osie różne o szum dopasowania) → status 1001.
- Teraz: minimalne reprodukcje jako testy `#[ignore]` w `crates/kernel/tests/upstream_repro.rs` (każdy
  opisany, z oczekiwanym i obecnym wynikiem) + szkic treści zgłoszeń w `docs/upstream/solidcore-p3.md`
  zgodnie z zasadami solidcore (najpierw issue z reprodukcją).
- **Zgłoszenie upstream (tangency-labs/solidcore) wymaga zgody właściciela** — działanie poza repo.
- Po naprawie upstream: podbicie pinu, usunięcie obejść (`snap_axes` zostaje jako auto-więz §6.2,
  ale nie jako warunek konieczny), testy reprodukcji bez `#[ignore]`.

## Bramka (testy `gate_p3b_*`, skan σ = 0,03 przez `synth::scanify`, części CAD z kernela)
1. **Blok z kieszenią i otworem nieprzelotowym:** kieszeń prostokątna z narożami r = 4 i filletem dna
   r = 1, otwór Ø8 głębokości 12 z dnem stożkowym 118°. Jedna zamknięta bryła, liczba ścian jak CAD,
   ΔV ≤ 0,5 %, max odchyłka skanu od bryły ≤ 0,2 mm, STEP round-trip.
2. **Płyta z czopem:** czop walcowy z wklęsłym zaokrągleniem u podstawy (torus), zaokrąglone krawędzie
   płyty spotykające się w narożniku (sfera), faza kołowa na czole czopa (stożek częściowy). Te same kryteria.
3. **`replace_face` w górę i w dół:** płat przecinający płaską ścianę (część wyżej, część niżej).
   Objętość zgodna z kwadraturą < 0,1 %.
4. Każde wejście `from_regions` ma rolę i status (zastosowane / pominięte z powodem) — 0 pominięć
   w częściach 1–2.

## Podział i równoległość (subagenci)
| Zadanie | Model | Zależy od | Pliki |
|---|---|---|---|
| Z1 + Z2 (rdzeń, kieszenie, otwory) | Sonnet | — | `kernel/src/hybrid.rs`, `engine/src/cmd/hybrid.rs`, `engine/tests/hybrid.rs` |
| Z3 (zaokrąglenia, fazy, lokalne bryły) | Sonnet | Z1 (klasyfikacja) | jw. — **po** Z1+Z2, ta sama gałąź |
| Z4 (`replace_face` z dodawaniem) | Sonnet | — | wydzielone funkcje w `kernel/src/hybrid.rs` (bez zmian w `solid_from_cuts`) |
| Z5 (reprodukcje + szkic zgłoszeń) | Haiku | — | `kernel/tests/upstream_repro.rs`, `docs/upstream/` |

Równolegle od razu: Z1+Z2, Z4, Z5 (rozłączne pliki/funkcje). Z3 po scaleniu Z1+Z2. Review (Opus) każdego
PR przed merge'em; scalanie do `main` tylko za zgodą właściciela.
