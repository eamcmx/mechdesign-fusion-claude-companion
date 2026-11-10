# projects/spring_plunger

Volume 1, Chapter 10 (Springs, Pins and Retaining Rings).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `SpringPlunger/`: Chapter 10: a sleeve on a helical compression spring, held by a DIN 471 retaining ring on a rod pinned into a base; the spring's rate, forces, stresses and safety factors.

## checks/ (1): run the script as a user does and measure what Fusion built

- `spring_plunger_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `spring_plunger_no_groove`: breaks "The groove is cut in the rod: diameter d2, width m, its upper flank level with the ring's top face"

## captures/ (1): the pictures in the book

`spring_plunger`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
