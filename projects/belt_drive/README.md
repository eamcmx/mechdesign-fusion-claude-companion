# projects/belt_drive

Volume 1, Chapter 20 (Belts and Chains).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BeltDrive/`: Chapter 20: a GT2 timing-belt drive, the centre distance solved for a belt of whole teeth.

## checks/ (1): run the script as a user does and measure what Fusion built

- `belt_drive_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `belt_reversed`: breaks "A Motion Link ... NOT reversed (a belt keeps the direction)"

## captures/ (1): the pictures in the book

`belt_drive`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
