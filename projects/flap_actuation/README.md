# projects/flap_actuation: The flap actuation system

Volume 2, Chapters 32 to 34: the ball-screw actuator and the flap system it drives. Volume 3, Chapter 53: the actuator in detail.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `FlapActuator/`: Chapter 33: the flap actuator. A ball screw 32 x 10 turned through a spur gear, an idler and a pinion by a bought motor unit; thrust bearings, nut, rod, rod end, aluminium housing on two trunnions; checked over the whole stroke.
- `FlapDetail/`: Chapter 53: builds Chapter 34's flap actuation system in detail, the motor unit drawn to maxon's figures, the pinion keyed on the gearhead's shaft with its end screw, the pins' bearing seats split out as the zones that are hardened, and every part's material and fit as Chapter 51 decided them.
- `FlapSystem/`: Chapter 34: the flap actuation system put together. The actuator of Chapter 33 on the spar fitting, its rod end in the horn on the flap, the flap on two hinge brackets; posed from 0 to 40 deg.

## lib/

- `flap_design.py`: the trailing-edge flap's actuation system of Volume 2, Chapters 32 to 34.
- `flap_detail.py`: Chapter 53: the flap actuator in detail.

## checks/ (3): run the script as a user does and measure what Fusion built

- `flap_actuator_check`
- `flap_detail_check`
- `flap_system_check`

## controls/ (3): one prompt line broken on purpose: the check must then fail

- `flap_actuator_short_slot`: breaks "the guide key on the rod runs in a slot along the housing's bore, to the gland"
- `flap_detail_key_hole`: CONTROL for the prompt's hard constraint that maxon's key passes the housing's back wall:
- `flap_system_no_spacers`: breaks "a spacer each side of the rod end, so that the horn's cheeks stand clear of its shank"

## captures/ (9): the pictures in the book

`flap_actuator`, `flap_actuator_exploded`, `flap_actuator_open`, `flap_detail_exploded`, `flap_detail_hinge`, `flap_system`, `flap_system_down`, `flap_system_exploded`, `flap_system_station`

## results/ (6 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
