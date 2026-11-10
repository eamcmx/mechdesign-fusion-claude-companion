# projects/lead_screw_axis

Volume 1, Chapter 21 (Screws and Linear Guides).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `LeadScrewAxis/`: Chapter 21: a linear axis, a trapezoidal lead screw and a carriage with its nut on a guide rod; torque, efficiency and self-locking.

## checks/ (1): run the script as a user does and measure what Fusion built

- `lead_screw_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `lead_screw_wrong_way`: breaks "the carriage going towards -x when the screw turns positively about +x (a right-hand thread)"

## captures/ (1): the pictures in the book

`lead_screw`

## references/ (1 sources)

The catalogues, data sheets, standards and books the project was sized from, listed in `references/README.md` with their links; the copies stay with their publishers.

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
