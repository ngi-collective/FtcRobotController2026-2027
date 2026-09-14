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
  1.25 in. backstop, 3.55 x 3.57 in. retrieval opening at the bottom (p.72).
  CAD-measured centers, in 90 deg rotational symmetry about FIELD center (inches):
  `(+23.39, z=+68.04)`, `(x=+68.04, -23.39)`, `(x=-68.04, +23.39)`, `(-23.39, z=-68.04)`;
  top ring at y = 21.25.
- Alliance-coloured features a vision pipeline could key on: red/blue CELLS (coloured
  rails, translucent panels), red/blue GARDEN and LOADING ZONE gaffer tape, BIOBUZZ logo
  panels on the frame, yellow POLLEN and red/blue NECTAR.

### SCORING ELEMENTS (manual §9.8 p.74, §10.3.1 pp.83-84)

- **POLLEN**: ~2.8 in. (7.1 cm) yellow polyethylene balls (`am-5851_yellow`), 40 per MATCH.
- **NECTAR**: ~3.6 in. (9.1 cm) red (`am-5852_red`) / blue (`am-5852_blue`) balls, 8 each.
- Not perfectly spherical; sizes vary.
- Staging: 4 POLLEN in each of 4 FLOWERS, 4 in each GARDEN, 4 pre-loaded per ROBOT;
  3 NECTAR in each upward-facing CELL, 5 in each ALLIANCE AREA.
