# projects/bevel_differential_wrist

Volume 1, Chapter 26 (The Bevel Differential).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BevelWristPrompt/`: Chapter 26 of Mechanical Design in Autodesk Fusion with Claude: the bevel differential of a robot wrist (frame, yoke, output, idler and two side gears), As-built Joints and Motion Links, ready to pitch and roll.

## checks/ (1): run the script as a user does and measure what Fusion built

- `bevelwrist_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `bevelwrist_phase`: BevelWristPrompt against the acceptance criteria of its prompt (Chapter 26), measured on what Fusion built:

## captures/ (2): the pictures in the book

`bevel_wrist`, `bevel_wrist_open`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
