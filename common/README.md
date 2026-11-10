# common: Common

What every project uses: `companion_paths.py` (where everything is kept), the parts library `lib/mdfc_parts.py`, and `BallScrewAxis`, which the flap actuator and the parallel-jaw gripper build their screws with.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BallScrewAxis/`: Chapter 21: a linear axis on a ball screw, the screw's and the nut's groove one helical tube cut by its section swept with a twist, three circuits of balls on its centre line.

## lib/

- `mdfc_parts.py`: the parts library of Volume 2 of "Mechanical Design in Autodesk Fusion with Claude": the machine elements of Volume 1 as functions that build a part inside a given component, so that a subsystem script can put real parts together.
