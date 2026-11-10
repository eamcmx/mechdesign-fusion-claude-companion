# projects/palletizer: The palletizer

Volume 2, Chapters 41 to 48: the drives, the arm and the whole robot, the tool changer, the parallel-jaw, vacuum and bag grippers and the compliance unit. Volume 3, Chapter 55: the palletizer in detail; Chapter 62: the bag gripper's stop bracket.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BagGripper/`: Chapter 47: the palletizer's bag gripper: adapter, beam and end plates, two jaws of tines on 6206 bearings turned by DSBC-80 cylinders with clamping units, a press plate, the bag and the pick-up station's rollers.
- `ClampGripper/`: Chapter 45: the palletizer's parallel-jaw carton gripper: an adapter to ISO 9409-1-160-6-M10, an aluminium beam on HGR25 rails, two jaws moved together by a right- and left-hand ballscrew, its bearings, the motor with gearhead and brake.
- `ComplianceUnit/`: Chapter 48: the palletizer's compliance unit: a vertical float between the tool changer and the vacuum or bag gripper, on two HGR25 rails and four HGH25CA blocks, held down by four springs on hanger bolts, with a touch-down sensor.
- `PalletizerDetail/`: Chapter 55: builds one of the palletizer's assemblies in detail (the robot, the bag gripper, the compliance unit or the vacuum gripper) with its own script: the rods' eyes and pins, the jaws' locating and floating ends and the plugs' welds, the O-ring's gland and the gallery plugs, the plate's through bolts; then every part's material and fit as Chapter 51 decided them, and this chapter's classes by hand.
- `PalletizerDrives/`: Chapter 42: the palletizer's drives. The base and an SKF RKS.23 0841 slewing ring, the turret turned by an RV-500N through a disc coupling, J2's RV-500N with the gas balancer in the lower arm's fork, the crank with its counterweight turned by J3's RV-500N.
- `PalletizerRobot/`: Chapter 43: the whole palletizer. Chapter 42's drives (J1 on an RV-700N), the lower arm to the elbow, the upper arm with its yoke, front fork and rear lever, the parallel rod, the levelling rods and the triangle, the wrist with J4's RV-80N; every pin on its SKF bearing.
- `StopBracket/`: Chapter 62: the bag gripper's opening stop brackets as Part 10 leaves them, the arm turned 45 degrees about its own axis so the shock absorber's push lies square to a face, its root rounded; the faces, fillets, screw-head clearance, volume and the range sweep measured.
- `ToolChanger/`: Chapter 44: the palletizer's tool flange to ISO 9409-1-160-6-M10 and its tool changer: the master with a piston whose cam drives six segments into the tool plate's ring; the tool plate with the same ISO pattern for the gripper.
- `VacuumGripper/`: Chapter 46: the palletizer's vacuum gripper: adapter, beams and plate, six zones of six SPB1 80 cups on spring compensators, manifolds and SCPSi 2-14 ejectors.

## lib/

- `bag_gripper.py`: Chapter 47 of Volume 2, the palletizer's gripper for bags, and the services every gripper takes across the tool changer and the robot's joints.
- `bag_gripper_weighed.json`: data read by the libraries (masses as Fusion weighed them, written by a check)
- `clamp_gripper.py`: Chapter 45 of Volume 2, the palletizer's parallel-jaw gripper for cartons.
- `clamp_gripper_decels.json`: data read by the libraries
- `clamp_gripper_weighed.json`: data read by the libraries (masses as Fusion weighed them, written by a check)
- `compliance_unit.py`: Chapter 48 of Volume 2, the compliance unit: a vertical float between the tool plate and a gripper, so that the gripper gives upward when its load touches the stack and a sensor stops the robot before the arm feels the stack.
- `compliance_unit_weighed.json`: data read by the libraries (masses as Fusion weighed them, written by a check)
- `palletizer_design.py`: the four-axis palletizer of Volume 2, Chapters 41 to 43.
- `palletizer_detail.py`: Chapter 55: the palletizer in detail.
- `palletizer_drives.py`: Chapter 42 of Volume 2, the palletizer's drives.
- `palletizer_links.py`: Chapter 43 of Volume 2, the palletizer's links, the integration and the parts list.
- `palletizer_weighed.json`: data read by the libraries (masses as Fusion weighed them, written by a check)
- `tool_changer.py`: Chapter 44 of Volume 2, the palletizer's tool flange and its tool changer.
- `tool_changer_weighed.json`: data read by the libraries (masses as Fusion weighed them, written by a check)
- `vacuum_gripper.py`: Chapter 46 of Volume 2, the palletizer's vacuum gripper for light cartons and slip sheets.
- `vacuum_gripper_weighed.json`: data read by the libraries (masses as Fusion weighed them, written by a check)

## checks/ (9): run the script as a user does and measure what Fusion built

- `bag_gripper_check`
- `clamp_gripper_check`
- `compliance_unit_check`
- `palletizer_detail_check`
- `palletizer_drives_check`
- `palletizer_robot_check`
- `stop_bracket_check`
- `tool_changer_check`
- `vacuum_gripper_check`

## controls/ (9): one prompt line broken on purpose: the check must then fail

- `bag_gripper_tines`: breaks "each tine's foot under the bag to its tip at y = +-40"
- `clamp_gripper_bearing_seat`: breaks "The fixed bearing BK25 at x = -880 and the supported BF25 at +880, outside the jaws' carriages at their widest"
- `compliance_unit_carriage`: breaks "the carriage: a plate 176 x 200 x 32, 2 mm clear of the rails' running faces"
- `palletizer_detail_groove`: breaks "the O-ring's groove 24.30 across and 3.40 wide, to ERIKS's dynamic table"
- `palletizer_drives_crank_plane`: breaks "The crank and its block outboard of the J3 cheek and of the base's flange: the crank's plane at y = 640 mm"
- `palletizer_robot_rod_depth`: breaks "The parallel rod a flat bar 40 mm deep in the arm's plane: at full reach it passes 190 mm from J3's axis, beside the reducer's case of 162.5 mm"
- `stop_bracket_diagonal`: breaks "the arm turned 45 deg about its own axis, so that the shock absorber's push lies square to a face of the arm"
- `tool_changer_travel`: breaks "Free, each segment drawn in by 3 mm, inside the tool plate's bore of 65.25 mm"
- `vacuum_gripper_manifold`: breaks "each zone's manifold 150 x 60 between the zone's two rows of cups"

## captures/ (25): the pictures in the book

`bag_gripper`, `bag_gripper_exploded`, `bag_gripper_open`, `clamp_gripper`, `clamp_gripper_below`, `clamp_gripper_exploded`, `compliance_unit`, `compliance_unit_exploded`, `compliance_unit_raised`, `palletizer_detail_exploded`, `palletizer_detail_eye`, `palletizer_drives`, `palletizer_drives_exploded`, `palletizer_drives_q3`, `palletizer_robot`, `palletizer_robot_exploded`, `palletizer_robot_reach`, `stop_bracket_turned`, `stop_bracket_turned_top`, `tool_changer`, `tool_changer_cut`, `tool_changer_exploded`, `vacuum_gripper`, `vacuum_gripper_exploded`, `vacuum_gripper_top`

## results/ (20 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
