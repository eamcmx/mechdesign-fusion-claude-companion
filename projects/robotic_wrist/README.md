# projects/robotic_wrist: The robotic wrist

Volume 2, Chapters 39 and 40: the bevel differential with its drives, and the wrist integrated. Volume 3, Chapter 52: the wrist in detail.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `WristDetail/`: Chapter 52: builds Chapter 40's wrist in detail, its motors, gear units and slip ring drawn to their makers' figures, its bearings' abutments inside SKF's bands, the stop disc's key the length that is made, and every part's material and fit as Chapter 51 decided them.
- `WristDifferential/`: Chapter 39: the pitch-roll wrist. A miter bevel differential on two trunnions, two side gears with pulleys cut on their hubs, an output gear on the roll shaft, two belt-driven servo motors with gearheads; pitch and roll as joints, the gears and motors on motion links.
- `WristIntegration/`: Chapter 40: the pitch-roll wrist made ready for its arm. Chapter 39's wrist with the tool flange to ISO 9409-1-31.5-4-M5, a bored roll shaft and a slip ring for the tool's wires, and pitch stops with buffer pads on the left trunnion.

## lib/

- `wrist_design.py`: the pitch-roll wrist of Volume 2, Chapters 39 and 40.
- `wrist_detail.py`: Chapter 52 of Volume 3, the robotic wrist in detail.

## checks/ (3): run the script as a user does and measure what Fusion built

- `wrist_check`
- `wrist_detail_check`
- `wrist_integration_check`

## controls/ (3): one prompt line broken on purpose: the check must then fail

- `wrist_detail_spacers`: breaks "the roll shaft's collar and spacers S4 and S5 18 mm across"
- `wrist_gear_phase`: breaks "a gap of the output gear faces each side gear's tooth"
- `wrist_stop_lug`: breaks "the stop disc's lug on +z at rest, the block on -z"

## captures/ (8): the pictures in the book

`wrist`, `wrist_detail`, `wrist_detail_exploded`, `wrist_exploded`, `wrist_integrated`, `wrist_integration_exploded`, `wrist_open`, `wrist_stop`

## references/ (8 sources)

The catalogues, data sheets, standards and books the project was sized from, listed in `references/README.md` with their links; the copies stay with their publishers.

## results/ (6 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
