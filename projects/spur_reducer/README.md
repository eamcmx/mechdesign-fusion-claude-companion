# projects/spur_reducer: The spur reducer

Volume 2, Chapter 27 (From a Machine to Its Subsystems): a single-stage spur reducer designed subsystem by subsystem.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `SpurReducer/`: Chapter 27: a single-stage spur reducer designed by subsystems. Sizing along the load path, then a housing, two shaft assemblies and all their parts, the shafts turning on their bearings.

## lib/

- `reducer_design.py`: the sizing of Chapter 27's single-stage spur reducer, along its load path, with the data of Budynas and Nisbett (2015), Shigley's Mechanical Engineering Design, 10th edition.

## checks/ (1): run the script as a user does and measure what Fusion built

- `spur_reducer_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `spur_reducer_short_spacer`: breaks "a spacer from the gear's hub to the inner ring of the output's second bearing"

## captures/ (4): the pictures in the book

`spur_reducer`, `spur_reducer_exploded`, `spur_reducer_open`, `spur_reducer_shafts`

## references/ (1 sources)

The catalogues, data sheets, standards and books the project was sized from, listed in `references/README.md` with their links; the copies stay with their publishers.

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
