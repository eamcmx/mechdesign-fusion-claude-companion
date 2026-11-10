# projects/fit_pair

Volume 1, Chapter 7 (Fits and Tolerances).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `FitPair/`: Chapter 7: a pin in a hole, each made at a chosen limit of its ISO 286 tolerance, so that a clearance, transition or interference fit can be measured.

## checks/ (1): run the script as a user does and measure what Fusion built

- `fit_pair_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `fit_pair_nominal`: breaks "The model is made at the chosen limits, not at the nominal size"

## captures/ (1): the pictures in the book

`fit_pair`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
