# projects/rear_axle: The rear axle

Volume 2, Chapters 28 to 31: the final drive and differential, the half shafts, wheel bearings and housing, and the whole axle. Volume 3, Chapter 54: the axle in detail.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `AxleDetail/`: Chapter 54: builds Chapter 31's rear axle in detail, its tapered bearings' abutments inside SKF's bands, the pinion's spacer from cone to cone, the cups' bores closed, the half shafts' splines to ISO 14, the adjusters threaded, and every part's material and fit as Chapter 51 decided them.
- `AxleHousing/`: Chapter 30: the rear axle's banjo housing and its two semi-floating wheel ends, sized for the hardest turn, the pothole and the truck's life, built as three subassemblies of real parts.
- `RearAxle/`: Chapter 31: the rear axle put together. The carrier of Chapter 29 bolted into the housing of Chapter 30, the half shafts splined into the side gears; the axle turns straight and in a turn.
- `RearAxleCarrier/`: Chapter 29: the rear axle's carrier. A spiral bevel final drive 12:47 on tapered roller bearings and an open differential of four spiders, sized by the AGMA rating and built as three subassemblies of real parts.

## lib/

- `axle_design.py`: the rear axle of Chapters 28 to 31 (Volume 2): a live beam axle for a light pickup truck.
- `axle_detail.py`: Chapter 54: the rear axle in detail.
- `spiral_bevel.py`: the teeth of a spiral bevel pair at concept level (Volume 2, Chapter 29).

## checks/ (6): run the script as a user does and measure what Fusion built

- `axle_detail_check`
- `axle_housing_check`
- `axle_housing_try`
- `rear_axle_carrier_check`
- `rear_axle_carrier_try`
- `rear_axle_check`

## controls/ (6): one prompt line broken on purpose: the check must then fail

- `axle_detail_spacer`: breaks "the spacer from cone A's front face to cone B's, 67.8 long"
- `carrier_same_hand`: breaks "the crown wheel's teeth of the opposite hand to the pinion's"
- `housing_round_flange`: breaks "the carrier's flange cut flat 166 mm below the axle, with the carrier's plate"
- `housing_shoulder_inboard`: breaks "the bearing's seat, a shoulder of d_S against which the bearing is pressed, a flange"
- `rear_axle_necks_in_drum`: breaks "the drum's cavity clear of the necks' walls"
- `rear_axle_short_splines`: breaks "each half shaft's splines from 35 mm, deep in its side gear"

## captures/ (12): the pictures in the book

`axle_detail`, `axle_detail_exploded`, `axle_housing`, `axle_housing_ends`, `axle_housing_exploded`, `rear_axle`, `rear_axle_carrier`, `rear_axle_carrier_exploded`, `rear_axle_carrier_gears`, `rear_axle_carrier_open`, `rear_axle_exploded`, `rear_axle_inside`

## results/ (12 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
