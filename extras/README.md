# extras: Extras

Scripts kept for reference that no chapter of the three volumes prints, with their checks, controls and pictures. `python tools/run_check.py --all` leaves them out.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `CarDifferential/`: Chapter 27: the rear axle of a car. A final-drive pinion and ring gear turn an open bevel differential whose spiders drive the two side gears and half shafts.
- `JointDrive/`: Chapter 34: the drive of a SCARA's first joint. The load's inertia measured on the model, a move, and the motor's torque and speed choose the harmonic gearhead's ratio.
- `LandingGear/`: Chapter 30: the retracting main landing gear of a light aircraft. A leg on a trunnion and a two-link drag brace locked over centre; driving the brace through dead centre unlocks and retracts the leg.
- `Palletizer/`: Chapter 36: a four-axis palletizing robot. A crank and three parallelograms keep the load level and the forearm's motor on the turntable; the angles for every box of a pallet.
- `ScaraArm/`: Chapter 33: a SCARA robot. Two arms turn in the horizontal plane, a quill slides and turns the tool; forward and inverse kinematics, workspace and joint limits.
- `SingleCylinderEngine/`: Chapter 28: a four-stroke side-valve single-cylinder engine. Slider-crank, timing gears 20:40 and two cam-driven valves, built at top dead centre of compression.
- `SphericalWrist/`: Chapter 35: a roll-pitch-roll robot wrist. Three axes meeting at one point, the inverse for any orientation, and the singularity where the two rolls line up.
- `StarGearbox/`: Chapter 32: the star gearbox of a geared turbofan. A planetary set with the carrier held: the sun drives five stars on fixed pins and the ring drives the fan the other way.
- `SteeringRack/`: Chapter 29: rack-and-pinion steering. Rack, tie rods and knuckles laid out to follow Ackermann's condition within 1 deg up to 30 deg of inner steer, with a transmission angle of at least 35 deg.

## checks/ (10): run the script as a user does and measure what Fusion built

- `ball_screw_check`
- `car_differential_check`
- `engine_check`
- `joint_drive_check`
- `landing_gear_check`
- `palletizer_check`
- `scara_check`
- `spherical_wrist_check`
- `star_gearbox_check`
- `steering_check`

## controls/ (10): one prompt line broken on purpose: the check must then fail

- `ball_screw_wrong_way`: breaks "One Motion Link: 360 deg of the screw for one lead of the carriage, in the direction that keeps the nut's groove on the screw's"
- `car_differential_right_unphased`: breaks "the right side gear is turned to do the same with spider 0"
- `engine_cam_same_way`: breaks "the camshaft at (60, 0) turning the other way at half the speed"
- `joint_drive_same_way`: breaks "a Motion Link from the motor shaft to J1 at n : 1, reversed: with the circular spline held, the flexspline turns against the wave generator"
- `landing_gear_under_centre`: breaks "the knee angle is 180 deg - over-centre when the leg is down ... down (knee on the locked side, the right of A to B)"
- `palletizer_rod1_low`: breaks "rod 1 from a pivot on the turntable 180 mm behind the shoulder, at its height"
- `scara_ik_sign`: breaks "q1 = atan2(y, x) - atan2(L2 sin q2, L1 + L2 cos q2)"
- `spherical_wrist_branch`: breaks "a = atan2(s r10, -s r20)"
- `star_ring_phase`: breaks "the ring's tooth spaces at k 2 pi / Zr + phi_r with phi_r = 0 for Zp odd, pi / Zr for Zp even"
- `steering_pinion_reversed`: breaks "a turn about +x moves the rack towards +y, which steers the car to the right"

## captures/ (21): the pictures in the book

`ball_screw`, `car_differential`, `car_differential_open`, `engine`, `engine_open`, `joint_drive`, `joint_drive_open`, `landing_gear`, `landing_gear_half`, `landing_gear_up`, `palletizer`, `palletizer_side`, `scara`, `scara_elbow2`, `spherical_wrist`, `spherical_wrist_rest`, `star_gearbox`, `star_gearbox_noring`, `star_gearbox_open`, `steering`, `steering_lock`

## results/ (18 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
