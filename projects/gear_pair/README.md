# projects/gear_pair

Volume 1, Chapter 12 (Spur Gears and the Gear Pair).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `GearPairOnPlate/`: Chapter 12 of Mechanical Design in Autodesk Fusion with Claude: a parametric spur gear pair on a base plate, two shafts, As-built Joints and a Motion Link, ready to drive.

## checks/ (1): run the script as a user does and measure what Fusion built

- `gearpair_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `gearpair_half_tooth`: GearPairOnPlate against the acceptance criteria of its prompt (Chapter 12), measured on what Fusion built:

## captures/ (2): the pictures in the book

`gear_pair`, `gear_pair_no_phase`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
