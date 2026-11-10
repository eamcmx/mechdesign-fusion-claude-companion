# projects/bolted_flange

Volume 1, Chapter 9 (Bolted Joints).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BoltedFlange/`: Chapter 9: two flange plates clamped by hexagon bolts and nuts; preload, tightening torque and the joint's safety factors under an external load.

## checks/ (1): run the script as a user does and measure what Fusion built

- `bolted_flange_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `bolted_flange_off_holes`: breaks "Bolt i on the bolt circle at angle 360 (i - 1) / N degrees"

## captures/ (1): the pictures in the book

`bolted_flange`

## references/ (2 sources)

The catalogues, data sheets, standards and books the project was sized from, listed in `references/README.md` with their links; the copies stay with their publishers.

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
