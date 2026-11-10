# projects/scara_robot: The SCARA robot

Volume 2, Chapters 35 to 38: the J1 and J2 modules, the tool joints J3 and J4, and the robot put together.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `ScaraJointModules/`: Chapter 36: the SCARA's J1 and J2 modules. A servo motor with its brake on a module plate driving a Harmonic Drive CSG-2UH housed unit, ratio 50: J1 a CSG-40, J2 a CSG-20.
- `ScaraRobot/`: Chapter 38: the SCARA put together. The base column with the J1 module, the first arm with the J2 module, the second arm with the ball screw spline, J1 to J4 as joints.
- `ScaraToolJoints/`: Chapter 37: the SCARA's J3 and J4. A HIWIN SP20 FB ball screw spline at the second arm's end, its two nuts driven by Gates GT3 belts from two servo motors with brakes.

## lib/

- `scara_design.py`: the SCARA robot of Volume 2, Chapters 35 to 38.

## checks/ (3): run the script as a user does and measure what Fusion built

- `scara_modules_check`
- `scara_robot_check`
- `scara_tool_check`

## controls/ (3): one prompt line broken on purpose: the check must then fail

- `scara_module_long_shaft`: breaks "the motor's shaft ends inside the wave generator's bore"
- `scara_robot_closed_arm`: breaks "a window through the first arm at J2 for the J2 motor"
- `scara_tool_short_shaft`: breaks "the shaft long enough to stay in the screw nut over the whole stroke"

## captures/ (10): the pictures in the book

`scara_modules`, `scara_modules_exploded`, `scara_modules_open`, `scara_robot`, `scara_robot_cycle`, `scara_robot_exploded`, `scara_robot_folded`, `scara_tool`, `scara_tool_exploded`, `scara_tool_open`

## references/ (7 sources)

The catalogues, data sheets, standards and books the project was sized from, listed in `references/README.md` with their links; the copies stay with their publishers.

## results/ (8 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
