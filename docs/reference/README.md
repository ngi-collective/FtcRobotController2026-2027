# BIOBUZZ (2026-2027) Reference Material

Authoritative FIRST Tech Challenge season references, downloaded **2026-09-13**.

This repo's vendored FTC SDK is **v11.2 (DECODE, 2025-2026)**, so its
`AprilTagGameDatabase` contains DECODE tags, **not** BIOBUZZ tags. Everything
BIOBUZZ-specific must come from the files below.

## Files

| File | Bytes | Source | What it is for |
| --- | --- | --- | --- |
| `biobuzz-competition-manual.pdf` | 6,948,629 | <https://ftc-resources.firstinspires.org/ftc/game/manual> | Official BIOBUZZ Competition Manual V1, 173 pages. Game rules, ARENA/FIELD specs, AprilTags (§9.9), SCORING ELEMENTS (§9.8). |
| `biobuzz-competition-manual.html` | 1,373,698 | <https://ftc-resources.firstinspires.org/ftc/archive/2027/game/cm-html> | Same manual, V1, as HTML. **Use this for text search** — see the PDF caveat below. |
| `biobuzz-field-cad.step` | 35,206,208 | <https://ftc-resources.firstinspires.org/ftc/field/field-cad-step> | Full FIELD assembly, STEP AP242 (`am-5850 BIOBUZZ 8-10-26`). The only source of numeric AprilTag / FIELD-element coordinates. **Not committed** — see below. |

SHA-256:

```
2ee0ea8327da47deb871e33307898c13ee5310d5af260007dc11bb4471181f5a  biobuzz-competition-manual.pdf
a2a849925a04b41a63296d6a277fa25cad95a32486e3ee4a246d15a3aac0d082  biobuzz-competition-manual.html
05b35961c7df847741031f00fda73ddd068537f809e92a11b1cd59a94bcc8331  biobuzz-field-cad.step
```

## The STEP file is deliberately not committed

34 MB that would sit in every clone forever, never be diffed, and duplicate data we
have already extracted into version-controlled scenario config. It is gitignored
(`docs/reference/*.step`). Fetch it when you need to re-derive field geometry:

```bash
curl -L -o docs/reference/biobuzz-field-cad.step \
  https://ftc-resources.firstinspires.org/ftc/field/field-cad-step
```

Verify against the SHA-256 above. FIRST calls the CAD the *official* representation of
the FIELD, carrying a general tolerance of ±1 in (2.5 cm), so treat anything measured
from it as accurate to that and no better.

## Source URLs

- Competition Manual (PDF, redirects straight to the file): <https://ftc-resources.firstinspires.org/ftc/game/manual>
- Game and Season Materials index: <https://ftc-resources.firstinspires.org/ftc/archive/2027/game>
- Playing Field Resources index: <https://ftc-resources.firstinspires.org/ftc/field>
- Field CAD (Onshape, public, read-only): <https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73>
- Field CAD (STEP export, no auth needed): <https://ftc-resources.firstinspires.org/ftc/field/field-cad-step>
- FLOWER scoring volume (Onshape element): <https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/8f89b946e295be9c543faa08>

Not captured: the Onshape *native* document (browser UI and the
`/api/*/blobelements`, `/api/*/assemblies`, `/api/*/partstudios` endpoints all
require authentication — HTTP 401). Only `/api/v6/documents/...` metadata and
`/api/v6/assemblies/.../bom` answer anonymously. The STEP export above supersedes
this: it is the same V1 geometry, so no CAD information was lost.

## Caveats

- **There is no "Part 1 / Part 2" this season.** BIOBUZZ ships one combined
  *Competition Manual*. Prior-season Part 1/Part 2 naming does not apply.
- **The PDF text layer silently drops digits.** In §9.9 the AprilTag ID list
  extracts as `AprilTag ID's 0, , , ...` instead of `30, 31, 32, 33`; the §9.11
  audio-cue table loses numbers the same way. Figure callouts are vector art with
  no text layer at all. Grep `biobuzz-competition-manual.html` (correct digits) and
  read figures as rendered images, never the PDF text layer.
- Manual figure dimensions are nominal with a **+/- 1 in. (2.5 cm)** tolerance
  (§9.1, p.63). The 3D CAD model is the official FIELD representation.

## Key extracted facts (with citations)

### AprilTags (manual §9.9, pp.74-77)

- 16 tags, IDs **30-45**, family **36h11**, **3.25 in. (8.25 cm)** square (p.74, p.76).
- Grouped into four **AprilTag Clusters** of 4 tags on one sticker, applied to the
  bottom face of each CELL, facing **downward** toward the TILES, bottom edge
  toward FIELD center (p.75).
- ID assignment (p.76):
  - `30, 31, 32, 33` — red CELL, side opposite the audience ("Red Scoring")
  - `34, 35, 36, 37` — red CELL, audience side ("Red Audience")
  - `38, 39, 40, 41` — blue CELL, audience side ("Blue Audience")
  - `42, 43, 44, 45` — blue CELL, side opposite the audience ("Blue Scoring")
- Cluster geometry (Figure 9-15, p.75): tag centerlines at **+/-2.75 in.** and
  **+/-6.5 in.** from the cluster centerline (one row of four, 3.75 in. and 5.5 in.
  center-to-center pitches); two 0.5 in. dia Reference Holes at **+/-7.0 in.**,
  2.75 in. below the tag centerline; reference-hole centerline is 9.938 in.
  (25.25 cm) from the front of the CELL.
- **The manual gives no numeric AprilTag FIELD coordinates.** It only says the
  Reference Holes "can be used to measure the location of the AprilTag Cluster
  relative to the rest of the FIELD" (p.74). Coordinates must come from the CAD.

### AprilTag cluster poses derived from `biobuzz-field-cad.step`

CAD frame: origin at FIELD center, **+Y up**, red alliance at **-X**, audience at
**+Z**. Sticker parts are `am-5888-{red1,red2,blue1,blue2}: ... Goal April Tag`,
each a 17.0 x 5.0 x 0.01 in. plate. Cluster-center coordinates, inches:

| Cluster | IDs | x | y (height) | z | Plate normal |
| --- | --- | --- | --- | --- | --- |
| Red Audience | 34-37 | -12.742 | 49.666 | +12.887 | (0, -0.866, +0.5) |
| Red Scoring | 30-33 | -12.742 | 35.647 | -11.394 | (0, -0.866, +0.5) |
| Blue Audience | 38-41 | +12.758 | 35.647 | +11.394 | (0, -0.866, -0.5) |
| Blue Scoring | 42-45 | +12.758 | 49.666 | -12.887 | (0, -0.866, -0.5) |

All four planes are tilted **30 deg from horizontal**, facing downward. Each HIVE is
bi-stable, so these are one of **two** states per HIVE; the CAD happens to capture
the red and blue HIVEs in opposite states, giving both poses. Cross-checks: x =
+/-12.75 matches the 25.5 in. HIVE center-to-center (Figure 9-10, p.71); each
cluster center is 14.10 in. from the pivot axis at y = 43.95 in., z = 0, matching
the manual's 43.95 in. pivot height (p.69).

### FIELD geometry (manual §9.2 p.64, §9.6-9.7 pp.69-72; CAD-measured values marked)

- FIELD: ~144 x 144 in. (365.75 x 365.75 cm) inside the perimeter wall, 36 interlocking
  foam TILES nominally 24 x 24 x 0.59 in. (p.64). CAD-measured: TILE pitch 23.502 in.,
  perimeter inner face at +/-71.08 in., wall top 11.31 in. above the TILE surface.
- TILE coordinates (§9.4 p.67) are a setup aid only: tiles `A1`-`F6` (letters left to
  right, numbers 1-6 from the audience side), seams `V`-`Z` / `1`-`5`. Not a metric frame.
- HIVE Structure at FIELD center: frame 49.46 in. wide x 38.95 in. deep, pivot axis
  43.95 in. above TILES (p.69); HIVE center-to-center 25.5 in.; bottom of HIVE 25.5 in.,
  bottom of HIVE opening 53.5 in., top of HIVE opening 65.6 in. above TILES (Figure 9-10,
  p.71); CELLS 18.8 in. apart, opening 20 x 14 x 12 in. deep (p.70).
- 4 FLOWERS on the perimeter wall, top opening ~4 in. dia at ~21.5 in. above the TILES,
  1.25 in. backstop, 3.55 x 3.57 in. retrieval opening at the bottom (p.72). Three rings:
  the upper and middle joined by four HIPS pipes, the middle and lower by square extrusion
  on the perimeter-wall side only, which is what leaves the retrieval opening open.
  CAD-measured centers, in 90 deg rotational symmetry about FIELD center (inches):
  `(+23.39, z=+68.04)`, `(x=+68.04, -23.39)`, `(x=-68.04, +23.39)`, `(-23.39, z=-68.04)`.
- Alliance-coloured features a vision pipeline could key on: red/blue CELLS (coloured
  rails, translucent panels), red/blue GARDEN and LOADING ZONE gaffer tape, BIOBUZZ logo
  panels on the frame, yellow POLLEN and red/blue NECTAR.

### HIVE and FLOWER parts derived from `biobuzz-field-cad.step`

Same frame as the AprilTag table above: CAD origin at FIELD center, **+Y up**, red at
**-X**, audience at **+Z**. Heights are `y`; everything in inches.

| Part | Number | Size | Placement |
| --- | --- | --- | --- |
| `am-5876: A-Frame Leg` | 4 | 1 x 1 x 45.75 | foot `(+/-23.81, y=0.53, +/-18.56)` to apex `(+/-12.74, y=41.09, +/-0.51)` |
| `am-5875: A-Frame Top Bar` | 1 | 1 x 1 x 24 | `(0, y=41.45, 0)`, along x |
| `am-5877: ACM Panel` | 2 | 30.13 x 11.12 x 0.118 | `(0, y=39.18, +/-1.97)`, leaning 24 deg with the frame |
| `am-5878: Sheet Metal Foot Bar` | 2 | 2.0 (x) x 38.95 (z) x 2.149 (y) | `(+/-23.75, y=1.07, 0)`, on the TILES |
| `am-5863: Axle Holder` | 2 | 2.82 x 2.71 x 14.91 | `(+/-12.06, y=42.96, +/-0.69)` |
| `am-5873: Goal Pivot Bracket` | 2 per HIVE | 13.78 x 3.59 x 0.07 | `(-12.20 and -13.28, y=43.15, -0.46)` on the red HIVE |
| CELL interior | 4 | 20 x 14 x 12 deep | closed end 9.42 from the pivot, offset 5.53 along the CELL's own up |
| `am-5857: Flower Layer C` (upper ring) | 4 | 5.86 x 4.80 oval, 1.19 thick | `y=20.81` |
| `am-5858: Flower Layer B` (middle ring) | 4 | 5.88 x 5.07 oval, 1.40 thick | `y=4.59` |
| `am-5859: Flower Layer X` (lower ring) | 4 | 5.88 x 4.92 oval, 0.90 thick | `y=0.25` |
| `am-5862: Flower HIPS Pipe` | 4 per FLOWER | 0.53 dia x 17.0 | `y=4.25` to `21.25`, at +/-48 and +/-137 deg to the facing, 2.2-2.7 out |
| `am-5892: Flower Peanut Support` | 2 per FLOWER | 1 x 1 x 3.75 | `y=0.35` to `4.10`, at +/-145 deg to the facing, 2.21 out |
| `am-5884: Flower Backstop` | 4 | 3.86 x 3.75 x 0.42 | horizontal, `y=22.45`, 0.76 toward the wall |

All four CELLs are identical to 0.01 in. in their own pivot-relative frame, and a CELL is
30 deg off level in either tip state, so the two stable positions are **60 deg** apart.
Rotating one measured AprilTag plate by that lands on the other measured plate to 0.02 in.
Four of the manual's published figures are reproduced rather than set: frame 49.5 x 38.96
(vs 49.46 x 38.95), CELLs 18.84 apart (vs 18.8), HIVE opening 53.39-65.51 (vs 53.5-65.6),
FLOWER retrieval opening 3.49 tall (vs 3.55).

Per-part colours are in the STEP file as `COLOUR_RGB` on each part's `STYLED_ITEM`, and
they are meaningful: TILES `(128,128,128)`, POLLEN `(255,239,63)`, red/blue cable ties
`(221,82,40)` / `(22,81,176)`. HIVE skins, A-frame and ACM panels are `(230,230,230)`;
FLOWER upper ring `(255,186,82)`, other rings `(48,48,48)`, HIPS pipe `(95,167,61)`,
backstop `(100,28,101)`. The Goal Ribs and NECTAR carry a styled item with no `COLOUR_RGB`,
so their red and blue are not recoverable this way.

### SCORING ELEMENTS (manual §9.8 p.74, §10.3.1 pp.83-84)

- **POLLEN**: ~2.8 in. (7.1 cm) yellow polyethylene balls (`am-5851_yellow`), 40 per MATCH.
- **NECTAR**: ~3.6 in. (9.1 cm) red (`am-5852_red`) / blue (`am-5852_blue`) balls, 8 each.
- Not perfectly spherical; sizes vary.
- Staging: 4 POLLEN in each of 4 FLOWERS, 4 in each GARDEN, 4 pre-loaded per ROBOT;
  3 NECTAR in each upward-facing CELL, 5 in each ALLIANCE AREA.

### Scoring (manual §10.5 pp.86-91, Table 10-2)

| Achievement | AUTO | TELEOP |
| --- | --- | --- |
| LEAVE - no longer contacting the perimeter wall | 3 | - |
| PARK - at least partially in the LOADING ZONE | 5 | 5 |
| HIVE TIP | 20 | 20 |
| POLLEN and/or NECTAR remaining in a CELL, each | - | 2 |
| FLOWER Bottom NECTAR Bonus | - | 5 |
| POLLEN and/or NECTAR in an owned FLOWER, each | - | 2 |
| POLLEN and/or NECTAR in a GARDEN, each | - | 1 |

Ranking points: SWARM (LEAVE + PARK at or above a threshold), POLLINATOR 1 and POLLINATOR 2
(number of TIPS at or above a threshold), 1 each; WIN 3.

The clauses that decide whether something scores, which are easier to get wrong than the numbers:

- **CELL** (§10.5.1): "At the end of the MATCH, any POLLEN and/or NECTAR left in an upward-facing
  CELL will earn points for that ALLIANCE." Points follow the CELL, so an opponent's NECTAR in
  your CELL scores for you; a downward-facing CELL scores nothing. No "partially within" clause,
  unlike the two below.
- **HIVE TIP** (§10.5.1): the HIVE moves between its two stable states *and* the damper that was
  clear of the frame begins to contact it. LAUNCHING into the upward-facing CELL is the only
  allowed way to cause one (G417).
- **FLOWER** (§10.5.2): an element scores when "at least partially within the FLOWER scoring
  volume: between the top ring and the middle ring". The ALLIANCE with the **top-most** NECTAR of
  its colour owns the FLOWER and scores everything in it, whoever put it there; the **bottom-most**
  earns the separate bonus. Scoring cannot begin until one minute remains (G410), and only by
  placing elements in the top.
- **GARDEN** (§10.5.3): at least partially in the zone; scores for the ALLIANCE whose colour the
  GARDEN is, whoever placed it, and either ALLIANCE may remove elements from either GARDEN.
- A MATCH is a 30 s AUTO, an 8 s transition, and a 2 min TELEOP (§10.1).

### What the CELL's geometry forces on a launcher (derived, not quoted)

Worked out from the numbers above rather than stated in the manual, and load-bearing enough to
write down once:

- A raised CELL's **mouth centre** is 1.51 m above the tiles and 0.30 m from the FIELD's centre
  line on the side it faces, being the CELL interior's centre walked half its 12 in. depth along
  its own outward normal. That normal sits **30° above horizontal**, so the opening is a nearly
  vertical rectangle leaning back.
- A legal ROBOT therefore cannot stand more than about **1.4 m** from it along that axis: the
  perimeter is 1.79 m out and half a chassis plus the mouth of a launcher eats the rest. The
  straight line from a ball on the tiles to the CELL's mouth is already close to **60°**, so a
  flatter launcher cannot reach the opening at all, at any speed.
- A ball must also arrive with its velocity pointing *into* the opening, which means shallower
  than 60° above horizontal, and slowly enough not to bounce back out: the basket's back panel
  returns about a third of what hits it, straight back at the mouth it came in through.
- Between them those leave a **steep, slow lob**: from a 0.86 m stand-off a 75° launch needs
  5.6 m/s, and the band that stays in the basket is about ±5% of that. A 4 in. compliant wheel on
  a bare goBILDA 5203 (6000 rpm) running at half transfer gives 16 m/s flat out, so this is a
  third of a stick and the interesting part of the range is nowhere near the top of it.
- The speed needed is **not monotonic in range**. At a fixed angle it bottoms out at
  `2 rise / tan p` &mdash; 0.79 m for a raised CELL at 75&deg; &mdash; and climbs on both sides, so
  a closer shot is harder than a mid-range one. Inside `rise / tan p` (0.39 m) there is no shot at
  any speed: the ball cannot come down steeply enough to be inside the opening at that height. A
  driver parked under the HIVE has to reverse, not spin faster.
- The **cluster origin is the aim point**, within 1.42 in. FIRST's member offsets
  (`+7.1874 in` rise, `-5.622 in` depth) walked back from the CAD's tag plate land 1.40 in below
  the centre of the CELL opening and 0.21 in outside its plane &mdash; identically for all four
  CELLs, in both tip states. So an OpMode can shoot at what it detected, with no offset table.
  Three sources agree and none of them mentions the others; see
  `BioBuzzFieldTest.everyClusterOriginSitsAtItsCellsOpening`.

### What it takes to tip a HIVE (derived, not quoted)

The manual says a HIVE "will hold its position until enough POLLEN or NECTAR are LAUNCHED into the
upwards-facing CELL" (&sect;9.6) and never says how many. Two things it does say bracket the
answer, and both are needed:

- A MATCH stages **three NECTAR in each upward-facing CELL** (&sect;10.3.1), and the field then
  stands there. A ball settles about 0.24 m horizontally from the pivot axis, so three NECTAR
  weigh **0.34 N&middot;m** on it and a HIVE that tipped under its own match setup would be
  unusable.
- A **TIP is worth 20** against 2 per element left in a CELL, so ten elements is the break-even. A
  HIVE needing more than ten POLLEN would make tipping strictly worse than filling, and the
  manual's own overview has ROBOTS tipping inside a 30-second AUTO.

So the hold is between 0.34 and about 1.1 N&middot;m. The simulator uses 0.6, which the manual
cannot confirm; measure it on a real field by putting POLLEN into a raised CELL one at a time and
counting, or by pushing the lip of a CELL with a fish scale at a known radius.

Two consequences worth knowing before designing a shooter:

- **The threshold is on arriving, not on weight.** Three NECTAR placed in a CELL stay; the same
  three dropped from 10 cm above its mouth tip it. Once a HIVE leaves its stop the hold falls away
  towards the midpoint while the balls roll outwards as the CELL flattens, so anything that starts
  it moving takes it the whole way. This is why the manual can say LAUNCHING is the only legal way
  to cause a TIP.
- **A tip is a motion, not an instant**: 0.4 to 1.1 s depending on how hard the HIVE was hit,
  which is the window the manual's warning about "LAUNCHING at the downward-facing CELL while a
  HIVE is tipping" lives in.
