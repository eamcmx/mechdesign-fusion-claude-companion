# projects/four_bar

Volume 1, Chapter 22 (Linkages).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `FourBar/`: Chapter 22: a four-bar crank-rocker linkage, one closed loop of four revolute joints.

## checks/ (1): run the script as a user does and measure what Fusion built

- `four_bar_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `four_bar_crossed`: breaks "the rocker's end C where the circle of radius b round B meets the circle of radius c round D, on the upper branch (the open linkage)"

## captures/ (1): the pictures in the book

`four_bar`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
