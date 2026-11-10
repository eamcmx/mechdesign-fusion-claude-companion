# design_office: The design office

Volume 3, Chapters 49 to 66: the work done on every machine. Part numbers, standard parts, materials and fits, interference over the range, mass properties, static and modal studies, fatigue, stack-ups, drawings, bills of materials, assembly and making.

## Scripts

Each folder holds the script, its manifest and `PROMPT.txt`, the prompt it was written from.

- `BillOfMaterials/`: Chapter 64: builds each of the nine assemblies and makes its bill of materials from Fusion's census: part numbers, descriptions as ordered, quantities, materials or makers, masses (the makers' for bought parts where a saved page gives them); writes the numbers into the components.
- `DesignForAssembly/`: Chapter 65: the bag gripper through Boothroyd and Dewhurst's test for unnecessary parts: every piece answered and coloured in the model, the theoretical minimum and the design efficiency; then the three fixings the test found missing (tines welded to their tubes, end blocks one with the end plates, the brackets screwed), checked for interference.
- `FatigueSpots/`: Chapter 60: builds the bag gripper with its stops and measures in the model the geometry the fatigue calculations take: the stop bolts' grip, the brackets' roots, the long stub's keyseat and key, the short stub's circlip groove against its bearing.
- `Making/`: Chapter 66: a made part programmed in the Manufacture workspace: the opening stop bracket milled in three setups, its R2.5 root cut by a cutter with that corner radius, and the axle's pinion spacer turned long for grinding; toolpaths, machining times and Fanuc NC programs.
- `MassProperties/`: Chapter 57: builds one of Part 9's detailed assemblies and weighs it: every group's mass, centre and moments of inertia about its centre, and for the robot its eleven moving bodies in the form Chapter 43's cycle reads.
- `MaterialsFits/`: Chapter 51: builds one of the nine assemblies, gives every made part its material from Fusion's library and every fitting face its tolerance class and roughness, found on the model and decided by rule.
- `PartDrawings/`: Chapter 63: an ISO drawing for one part of each machine, built through RangeSweep.py and saved to a cloud folder, with the datums, geometric tolerances, dimensions and requirement table Part 10 and Chapter 51 decided, exported as PDF.
- `PartNumbers/`: Chapter 49: builds one of Volume 2's assemblies with its own script and writes every part's part number and description into its component: made parts by machine, subassembly and item, bought parts by serial with their designation.
- `RangeSweep/`: Chapter 56: builds one of Part 9's detailed assemblies, the bag gripper with or without its stops, and sweeps it over its whole range of motion, reporting every interference, the contacts made in motion and the small gaps left.
- `StackGaps/`: Chapter 61: measures the small gaps Chapter 56's sweep found, each at its closest approach and in the states either side, with its direction, for the tolerance stack-ups.
- `StandardParts/`: Chapter 50: builds the bag gripper with its own script, numbers it as Chapter 49 does, describes every bought part by its full designation and details the driven stub shafts with their standard parts.
- `StudyAssemblies/`: Chapter 62: the real assemblies the Simulation studies run on, copied out of the detailed machines with their neighbours: the opening stop (arm turned or on its diagonal) on its end plate, the spar fitting on the spar web, the flap's hinge bracket on its rib.
- `StudyParts/`: Chapters 58, 59 and 62: the parts the Simulation studies run on, each in its own design with its material: the calibration cantilever, the long stub's keyseat, a tine, the opening stop bracket sharp and with its 2.5 mm root, and a spar-fitting lug.

## lib/

- `bom_detail.py`: Chapter 64 of Volume 3, the bills of materials.
- `detail_inventory.py`: Chapter 49 of Volume 3, from concept to detailed design.
- `dfma.py`: Chapter 65: design for manufacture and assembly.
- `drawings_detail.py`: Chapter 63: the drawings.
- `fatigue_detail.py`: Chapter 60: fatigue of the notches the detail added at the bag gripper's ends, where the jaws swing twice and strike their stops twice in every cycle, 600 cycles an hour over the brief's 20 000 hours: the stop brackets' M8 bolts, the brackets' roots, the long stub shaft's keyseat under the cylinder's torque, and the short stub's circlip groove.
- `fea_studies.py`: Chapters 58, 59 and 62: the hand calculations the Simulation studies are set against, part by part, in the units of the book (mm, N, MPa; Hz).
- `fusion_joints.py`: the joints of an assembly found on the model in Fusion (Chapter 51), for the census (checks/joint_census.py) and for MaterialsFits.py.
- `lugs_brackets.py`: Chapter 62: the opening stop bracket and the flap's spar-fitting lug, the hand calculation against Fusion's finite elements, and the change the bracket takes from it.
- `making.py`: Chapter 66: making the parts.
- `mass_detail.py`: Chapter 57: the detailed machines weighed against the masses their design calculations used.
- `materials_fits.py`: Chapter 51 of Volume 3, materials, fits and surface finish.
- `named_materials.json`: data read by the libraries
- `range_sweep.py`: Chapter 56: interference and motion over the whole range.
- `stackups.py`: Chapter 61: tolerance stack-ups.
- `standard_parts.py`: Chapter 50 of Volume 3, standard parts.

## checks/ (15): run the script as a user does and measure what Fusion built

- `bill_of_materials_check`
- `design_for_assembly_check`
- `fatigue_spots_check`
- `joint_census`
- `making_check`
- `mass_properties_check`
- `materials_fits_check`
- `materials_masses`
- `part_drawings_check`
- `part_numbers_check`
- `range_sweep_check`
- `stack_gaps_check`
- `standard_parts_check`
- `study_assemblies_check`
- `study_parts_check`

## controls/ (15): one prompt line broken on purpose: the check must then fail

- `bill_of_materials_bodies`: breaks "count a bought part once per set: a bearing's rings and balls, a tapered bearing's cone and cup, a reducer's case and output are one piece"
- `design_for_assembly_material`: breaks "answer each question for a future redesign, not for the parts as they are (Dewhurst & Meeker, 2014, p. 5)"
- `fatigue_spots_farface`: breaks "the bolts' grip: along the bolt's axis, from the head's underside ... to the nut's face nearer the head"
- `making_sharp_tine`: breaks "the tine's inside corner rounded to lib/making.py's radius"
- `making_square_cutter`: breaks "the arm's root cut by a cutter whose corner radius is the drawing's 2.5 mm"
- `mass_properties_axis`: breaks "the moments of inertia about the world's axes moved to the centre by the parallel-axis rule"
- `materials_fits_aluminium`: CONTROL for the rule in lib/materials_fits.py's decide() that an aluminium housing takes J7 where SKF's table gives H7:
- `part_drawings_no_root`: breaks "carry into the model what Part 10 decided for its shape before drawing it: the opening bracket's arm root filleted 2.5 mm"
- `part_numbers_names`: breaks "kind_of(): the name on the parts list that a component's name stands for"
- `range_sweep_springs`: breaks "leave out the parts that are not the machine: ... the compliance unit's springs, which are drawn rigid at their free length"
- `stack_gaps_pin`: breaks "leaving out the names a gap excludes (the horn's pin from the horn)"
- `standard_parts_shoulder`: breaks "the shoulder from bearing_6206(): the first whole millimetre over SKF's da min"
- `study_assemblies_home_pose`: breaks "arms: the robot at the cycle's high point"
- `study_assemblies_unturned`: breaks "the opening bracket with Chapter 62's change: its arm turned"
- `study_parts_sharp_keyseat`: breaks "its floor's four edges rounded to r/d = 0.02, 0.6 mm"

## captures/ (27): the pictures in the book

`bill_of_materials_exploded`, `bill_of_materials_robot`, `design_for_assembly_colours`, `design_for_assembly_exploded`, `fatigue_spots_end`, `fatigue_spots_exploded`, `lug_fitting_exploded`, `making_tine_nest`, `making_weldment_exploded`, `mass_properties_centres`, `mass_properties_robot_exploded`, `materials_fits_exploded`, `materials_fits_wrist`, `part_drawings_exploded`, `part_numbers_bag`, `part_numbers_exploded`, `part_numbers_robot`, `range_sweep_flap`, `range_sweep_stops`, `range_sweep_stops_closed`, `range_sweep_stops_exploded`, `stack_gaps_axle_exploded`, `stack_gaps_changer_cut`, `standard_parts_end`, `standard_parts_exploded`, `stub_torque_exploded`, `tine_on_jaw_exploded`

## results/ (134 files)

The last run of every check and control above, each headed by the Fusion version and the date, and the files the scripts write.
