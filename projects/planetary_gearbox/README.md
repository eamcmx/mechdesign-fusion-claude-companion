# projects/planetary_gearbox

Volume 1, Chapter 15 (Internal Gears and Planetary Gearboxes).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `PlanetaryPrompt/`: Chapter 15 of Mechanical Design in Autodesk Fusion with Claude: the planetary gearbox, sun, planets, ring, carrier and a sun bearing, As-built Joints and Motion Links, ready to drive.

## checks/ (1): run the script as a user does and measure what Fusion built

- `planetary_check`

## controls/ (2): one prompt line broken on purpose: the check must then fail

- `planetary_moved_after`: breaks "Create each component WHERE IT BELONGS and never move it afterwards"
- `planetary_ring_phase`: PlanetaryPrompt against the acceptance criteria of its prompt (Chapter 15), measured on what Fusion built:

## captures/ (2): the pictures in the book

`planetary`, `planetary_open`

## results/ (3 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
