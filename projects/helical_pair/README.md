# projects/helical_pair

Volume 1, Chapter 14 (Helical and Herringbone Gears).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `HelicalPair/`: Chapter 14: a pair of helical or herringbone gears on a base plate, each tooth swept along the axis with a twist.

## checks/ (1): run the script as a user does and measure what Fusion built

- `helical_pair_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `helical_same_hand`: breaks "Gear 1 right-handed, gear 2 left-handed: two external helical gears mesh only with opposite hands"

## captures/ (2): the pictures in the book

`helical_pair`, `herringbone_pair`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
