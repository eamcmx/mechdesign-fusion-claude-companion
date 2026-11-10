# projects/compound_train

Volume 1, Chapter 13 (Gear Trains).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `CompoundTrain/`: Chapter 13: a two-stage reverted compound spur gear train, input and output on one spindle, a cluster gear on a countershaft.

## checks/ (1): run the script as a user does and measure what Fusion built

- `compound_train_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `compound_train_stage2_phase`: breaks "Each driven gear is turned so that a tooth of its driver sits in one of its gaps on the line of centres"

## captures/ (1): the pictures in the book

`compound_train`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
