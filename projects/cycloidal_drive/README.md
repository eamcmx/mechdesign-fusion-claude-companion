# projects/cycloidal_drive

Volume 1, Chapter 19 (Cycloidal and Harmonic Drives).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `CycloidalDrive/`: Chapter 19: a single-disc cycloidal drive, the disc outline generated from the ring pins seen from the turning disc.

## checks/ (1): run the script as a user does and measure what Fusion built

- `cycloidal_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `cycloidal_disc_not_turning`: breaks "Motion Links: Input to Disc, N_d x 360 deg of the input for (N_d + 1) x 360 deg of the disc, reversed"

## captures/ (2): the pictures in the book

`cycloidal`, `cycloidal_open`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
