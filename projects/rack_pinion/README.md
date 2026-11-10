# projects/rack_pinion

Volume 1, Chapter 16 (Rack and Pinion).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `RackPinion/`: Chapter 16: a spur pinion on a fixed shaft driving a straight rack, linked by a revolute-to-slider Motion Link.

## checks/ (1): run the script as a user does and measure what Fusion built

- `rack_pinion_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `rack_tooth_under_tooth`: breaks "A gap of the rack is centred under the pinion's tooth 0 at x = 0"

## captures/ (1): the pictures in the book

`rack_pinion`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
