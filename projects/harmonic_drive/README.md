# projects/harmonic_drive

Volume 1, Chapter 19 (Cycloidal and Harmonic Drives).

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `HarmonicDrive/`: Chapter 19: a cup-type harmonic drive, the flexspline's teeth carried on its bent pitch line and turned by rebuilding it at each wave generator angle.

## checks/ (1): run the script as a user does and measure what Fusion built

- `harmonic_check`

## controls/ (1): one prompt line broken on purpose: the check must then fail

- `harmonic_flexspline_not_turned`: breaks "the flexspline turned as a whole by rho = -phi (z_c - z_f) / z_f"

## captures/ (1): the pictures in the book

`harmonic`

## results/ (2 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
