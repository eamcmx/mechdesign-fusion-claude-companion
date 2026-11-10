# projects/cam_follower

Volume 1, Chapter 24 (Cams and Followers).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `CamFollower/`: Chapter 24: a disc cam generated from a cycloidal motion law, and a translating roller follower.

## checks/ (1): run the script as a user does and measure what Fusion built

- `cam_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `cam_harmonic_rise`: breaks "cycloidal rise s = h (u - sin(2 pi u) / (2 pi))"

## captures/ (1): the pictures in the book

`cam_follower`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
