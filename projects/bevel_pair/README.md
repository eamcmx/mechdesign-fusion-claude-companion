# projects/bevel_pair

Volume 1, Chapter 17 (Bevel Gears).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BevelPair/`: Chapter 17: a pair of straight bevel gears on shafts at right angles, Tredgold teeth lofted heel to toe.

## checks/ (1): run the script as a user does and measure what Fusion built

- `bevel_pair_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `bevel_pair_unphased`: breaks "The pinion's tooth 0 points at the gear (world +X); the gear is turned half a tooth so that a gap meets it"

## captures/ (1): the pictures in the book

`bevel_pair`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
