# projects/sealed_cover

Volume 1, Chapter 11 (Couplings and Seals).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `SealedCover/`: Chapter 11: a pressure housing closed by a bolted cover and sealed by an O-ring in a face groove; squeeze, gland fill and the bolts' margin, the squeeze measured by Fusion's interference check.

## checks/ (1): run the script as a user does and measure what Fusion built

- `sealed_cover_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `sealed_cover_floating`: breaks "O-ring section centre at radius R_c = d1 / 2 + W / 2, height -L + W / 2 (resting on the groove bottom)"

## captures/ (2): the pictures in the book

`sealed_cover`, `sealed_cover_open`

## references/ (1 sources)

The catalogues, data sheets, standards and books the project was sized from, listed in `references/README.md` with their links; the copies stay with their publishers.

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
