# projects/first_assembly

Volume 1, Chapter 2 (Fusion as a Programmable CAD System).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `FirstAssembly/`: Chapter 2: a block with a shaft that turns in a hole through it. Two components, the shaft created where it belongs, an As-built revolute joint.

## checks/ (1): run the script as a user does and measure what Fusion built

- `first_assembly_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `first_assembly_moved_after`: breaks "Create the Shaft component WHERE IT BELONGS ... Never move it afterwards"

## captures/ (2): the pictures in the book

`first_assembly`, `first_assembly_moved`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
