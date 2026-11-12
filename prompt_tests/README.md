# Prompt tests: each script written from its prompt alone

Each folder here holds a Fusion script written by a fresh Claude model from the book's printed prompt, as a reader
would get it, and the result of the project's own check run on that script in Autodesk Fusion. The model saw the
prompt, the files the prompt names, the parts library `common/lib/mdfc_parts.py` and the check's interface (the
names it calls, not its numbers). Seven of the scripts here (RangeSweep, MassProperties, StudyAssemblies,
PartDrawings, Making, RearAxleCarrier, PalletizerDrives) were written with `FUSION_API_TRAPS.md` as well, as a
reader's model will be. It did not see the companion's own script and did not run
Fusion. The rules it followed are in `BRIEF.md`.

    python prompt_tests/run_prompt_test.py KeyedShaft      -> prompt_tests/KeyedShaft/check_result.txt

`run_prompt_test.py` runs the project's check unchanged but for the path of the script it loads.

Since this run every prompt has gained a last FUSION CONVENTIONS line that names its check and the names that check
looks for (`CHECK_INTERFACE.md`, `tools/check_names.py`), and the design points the models raised have been settled in
the companion's own scripts and their prompts, each script's check run again on 12 November: the pinion seal
60 x 95 x 10 and the differential's module limit (RearAxleCarrier), the ring nut M60 x 1.5 and the gearhead ratio
passed through (FlapActuator), the pinion's end washer (FlapDetail), the shaft ends inside their washers' faces
(WristDifferential, WristIntegration), the manifold 200 x 100 x 18 (VacuumGripper), the Geneva wheel's relief and
its 1 deg steps through the index, the cycloidal disc's 72 points a lobe and its chord test, the lead screw's layout
for a large screw, the bearing seat's shoulders, the counterweight's standoffs and block, and the closing stop
bracket's lip. The table below is the run of 11 and 12 November, before those changes.

## Results (Autodesk Fusion 2706.1.17, 11 and 12 November 2026)

58 of the 60 prompts gave a script that passes its check: the part or assembly built, measured and moved as the
prompt's acceptance criteria say.

| Prompt | Project | Check | Lines passed |
|---|---|---|---|
| AxleDetail | `projects/rear_axle` | ALL PASS | 16 |
| AxleHousing | `projects/rear_axle` | ALL PASS | 16 |
| BagGripper | `projects/palletizer` | ALL PASS | 16 |
| BallScrewAxis | `common` | ALL PASS | 13 |
| BearingSeat | `projects/bearing_seat` | ALL PASS | 16 |
| BeltDrive | `projects/belt_drive` | ALL PASS | 15 |
| BevelPair | `projects/bevel_pair` | ALL PASS | 13 |
| BevelWristPrompt | `projects/bevel_differential_wrist` | ALL PASS | 17 |
| BillOfMaterials | `design_office` | ALL PASS | 29 |
| BoltedFlange | `projects/bolted_flange` | ALL PASS | 12 |
| CamFollower | `projects/cam_follower` | ALL PASS | 12 |
| ClampGripper | `projects/palletizer` | ALL PASS | 13 |
| ComplianceUnit | `projects/palletizer` | ALL PASS | 16 |
| CompoundTrain | `projects/compound_train` | ALL PASS | 24 |
| CycloidalDrive | `projects/cycloidal_drive` | ALL PASS | 15 |
| DesignForAssembly | `design_office` | ALL PASS | 16 |
| FatigueSpots | `design_office` | ALL PASS | 9 |
| FirstAssembly | `projects/first_assembly` | ALL PASS | 15 |
| FitPair | `projects/fit_pair` | ALL PASS | 14 |
| FlapActuator | `projects/flap_actuation` | ALL PASS | 14 |
| FlapDetail | `projects/flap_actuation` | ALL PASS | 19 |
| FlapSystem | `projects/flap_actuation` | ALL PASS | 11 |
| FourBar | `projects/four_bar` | ALL PASS | 14 |
| GearPairOnPlate | `projects/gear_pair` | ALL PASS | 23 |
| Geneva | `projects/geneva` | ALL PASS | 12 |
| HarmonicDrive | `projects/harmonic_drive` | ALL PASS | 15 |
| HelicalPair | `projects/helical_pair` | ALL PASS | 25 |
| KeyedShaft | `projects/keyed_shaft` | ALL PASS | 18 |
| KneeToggle | `projects/knee_toggle` | ALL PASS | 14 |
| LeadScrewAxis | `projects/lead_screw_axis` | ALL PASS | 14 |
| Making | `design_office` | NO VERDICT | 2 ok, 0 FAIL |
| MassProperties | `design_office` | ALL PASS | 15 |
| MaterialsFits | `design_office` | ALL PASS | 14 |
| PalletizerDetail | `projects/palletizer` | ALL PASS | 27 |
| PalletizerDrives | `projects/palletizer` | ALL PASS | 14 |
| PalletizerRobot | `projects/palletizer` | ALL PASS | 9 |
| PartDrawings | `design_office` | 4 FAILED | 30 ok, 4 FAIL |
| PartNumbers | `design_office` | ALL PASS | 13 |
| PlanetaryPrompt | `projects/planetary_gearbox` | ALL PASS | 23 |
| RackPinion | `projects/rack_pinion` | ALL PASS | 15 |
| RangeSweep | `design_office` | ALL PASS | 15 |
| RearAxleCarrier | `projects/rear_axle` | ALL PASS | 22 |
| RearAxle | `projects/rear_axle` | ALL PASS | 12 |
| ScaraJointModules | `projects/scara_robot` | ALL PASS | 15 |
| ScaraRobot | `projects/scara_robot` | ALL PASS | 16 |
| ScaraToolJoints | `projects/scara_robot` | ALL PASS | 16 |
| SealedCover | `projects/sealed_cover` | ALL PASS | 15 |
| SpringPlunger | `projects/spring_plunger` | ALL PASS | 18 |
| SpurReducer | `projects/spur_reducer` | ALL PASS | 29 |
| StackGaps | `design_office` | ALL PASS | 10 |
| StandardParts | `design_office` | ALL PASS | 17 |
| StopBracket | `projects/palletizer` | ALL PASS | 15 |
| StudyAssemblies | `design_office` | ALL PASS | 31 |
| StudyParts | `design_office` | ALL PASS | 24 |
| ToolChanger | `projects/palletizer` | ALL PASS | 13 |
| VacuumGripper | `projects/palletizer` | ALL PASS | 13 |
| WormWheel | `projects/worm_wheel` | ALL PASS | 14 |
| WristDetail | `projects/robotic_wrist` | ALL PASS | 16 |
| WristDifferential | `projects/robotic_wrist` | ALL PASS | 21 |
| WristIntegration | `projects/robotic_wrist` | ALL PASS | 23 |

The two that do not pass:

- **PartDrawings**: five of its six drawings are made with their tables and PDFs. The stop bracket's automatic drawing
  fails inside Fusion's drawing engine ("Exception in acadm.crx ARX Command", then "Error writing/closing file")
  before any view is drawn, and the spar fitting's two hole dimensions are not placed.
- **Making**: the script takes the bracket out of its machine and programs its three setups, then Fusion refuses its
  machining-time call ("Machining time could not be calculated") and closes.
