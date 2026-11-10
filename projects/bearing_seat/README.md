# projects/bearing_seat

Volume 1, Chapter 8 (Rolling Bearings).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BearingSeat/`: Chapter 8: a deep groove ball bearing (6002 or 61902) on a shaft and in a housing; the balls orbit at the cage's speed; the rating life is reported.

## checks/ (1): run the script as a user does and measure what Fusion built

- `bearing_seat_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `bearing_seat_cage_speed`: breaks "The balls orbit at the cage's speed, not the shaft's"

## captures/ (1): the pictures in the book

`bearing_seat`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
