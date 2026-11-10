# projects/geneva

Volume 1, Chapter 25 (Geneva and Ratchet).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `Geneva/`: Chapter 25: an external Geneva drive, a driver with a pin and a locking disc indexing a slotted wheel.

## checks/ (1): run the script as a user does and measure what Fusion built

- `geneva_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `geneva_centre_distance`: breaks "centre distance C = a / sin(pi / n), so that the pin enters and leaves each slot along it"

## captures/ (1): the pictures in the book

`geneva`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
