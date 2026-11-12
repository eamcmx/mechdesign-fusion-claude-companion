# Mechanical Design in Autodesk Fusion with Claude: companion repository

The scripts, checks and results of the book *Mechanical Design in Autodesk Fusion with Claude* by Emmanuel A.
Merchán-Cruz. Every script in the book was written from the five-part prompt printed with it, run in Autodesk
Fusion, and checked against numbers stated before the run.

## How it is kept

The repository is kept the way a design office keeps its jobs: one folder for each exercise or machine, and every
file of that job inside it. Each folder has a `README.md` listing what it holds.

| Folder | What it holds |
|---|---|
| `projects/<project>/` | One exercise of Volume 1 or one machine of Volume 2 |
| `projects/<project>/<Name>/` | A Fusion script (`<Name>.py`, `<Name>.manifest`) and the prompt it was written from (`PROMPT.txt`) |
| `projects/<project>/lib/` | The machine's design calculations, and the masses Fusion weighed that they read back |
| `projects/<project>/checks/` | The checks: each runs a script the way a user does (Run, the dialog, OK) and measures what Fusion built |
| `projects/<project>/controls/` | One line of a prompt broken on purpose; the matching check must then fail |
| `projects/<project>/captures/` | The scripts that build the views printed in the book |
| `projects/<project>/results/` | The last result of every check and control, with the Fusion version and the date of the run |
| `projects/<project>/references/` | The catalogues, data sheets, standards and books the project was sized from, with their links (the copies stay with their publishers) |
| `design_office/` | The work of Volume 3 on every machine, kept the same way: part numbers, standard parts, materials and fits, the range sweep, masses, studies, fatigue, stack-ups, drawings, bills of materials, assembly, making |
| `common/` | `companion_paths.py`, which knows where everything is kept; the parts library `lib/mdfc_parts.py`; `BallScrewAxis` |
| `extras/` | Scripts kept for reference that no chapter prints, with their checks; left out of `--all` |
| `tools/` | `run_check.py` (runs a check and keeps its result), `fusion_mcp.py` (talks to Fusion's MCP server), `prompt_lint.py` (the five parts of a prompt), `check_names.py` (the names a prompt's check looks for), `capture.py` (the book's figures) |
| `FUSION_API_TRAPS.md` | What Fusion's API does that its documentation does not say: the traps met writing these scripts, each with what to do. Give it to the model with your prompts |
| `CHECK_INTERFACE.md` | What a script must offer for the companion's check to run it: `build()`, `VERSION`, refusals, the dialog, the names. Each prompt's last convention line names its check; give the model that check too, for its names only |

A script that uses another project's library or script finds it through `common/companion_paths.py`: it walks up
from its own folder to the one that holds `common/`, imports `companion_paths`, and every `lib/` folder is then on
the path (`CP.script('BagGripper')` gives a script's file). No script holds a path into another project's folder.

## The projects

| Folder | Book | Scripts |
|---|---|---|
| `projects/first_assembly` | Vol. 1, Ch. 2 Fusion as a programmable CAD system | `FirstAssembly` |
| `projects/keyed_shaft` | Vol. 1, Ch. 6 Shafts, keys and hubs | `KeyedShaft` |
| `projects/fit_pair` | Vol. 1, Ch. 7 Fits and tolerances | `FitPair` |
| `projects/bearing_seat` | Vol. 1, Ch. 8 Rolling bearings | `BearingSeat` |
| `projects/bolted_flange` | Vol. 1, Ch. 9 Bolted joints | `BoltedFlange` |
| `projects/spring_plunger` | Vol. 1, Ch. 10 Springs, pins and retaining rings | `SpringPlunger` |
| `projects/sealed_cover` | Vol. 1, Ch. 11 Couplings and seals | `SealedCover` |
| `projects/gear_pair` | Vol. 1, Ch. 12 Spur gears and the gear pair | `GearPairOnPlate` |
| `projects/compound_train` | Vol. 1, Ch. 13 Gear trains | `CompoundTrain` |
| `projects/helical_pair` | Vol. 1, Ch. 14 Helical and herringbone gears | `HelicalPair` |
| `projects/planetary_gearbox` | Vol. 1, Ch. 15 Internal gears and planetary gearboxes | `PlanetaryPrompt` |
| `projects/rack_pinion` | Vol. 1, Ch. 16 Rack and pinion | `RackPinion` |
| `projects/bevel_pair` | Vol. 1, Ch. 17 Bevel gears | `BevelPair` |
| `projects/worm_wheel` | Vol. 1, Ch. 18 Worm and wheel | `WormWheel` |
| `projects/cycloidal_drive` | Vol. 1, Ch. 19 Cycloidal and harmonic drives | `CycloidalDrive` |
| `projects/harmonic_drive` | Vol. 1, Ch. 19 Cycloidal and harmonic drives | `HarmonicDrive` |
| `projects/belt_drive` | Vol. 1, Ch. 20 Belts and chains | `BeltDrive` |
| `projects/lead_screw_axis` | Vol. 1, Ch. 21 Screws and linear guides | `LeadScrewAxis` |
| `projects/four_bar` | Vol. 1, Ch. 22 Linkages | `FourBar` |
| `projects/knee_toggle` | Vol. 1, Ch. 23 Levers, toggles and the adaptive finger | `KneeToggle` |
| `projects/cam_follower` | Vol. 1, Ch. 24 Cams and followers | `CamFollower` |
| `projects/geneva` | Vol. 1, Ch. 25 Geneva and ratchet | `Geneva` |
| `projects/bevel_differential_wrist` | Vol. 1, Ch. 26 The bevel differential | `BevelWristPrompt` |
| `projects/spur_reducer` | Vol. 2, Ch. 27 | `SpurReducer` |
| `projects/rear_axle` | Vol. 2, Ch. 28 to 31; Vol. 3, Ch. 54 | `RearAxleCarrier`, `AxleHousing`, `RearAxle`, `AxleDetail` |
| `projects/flap_actuation` | Vol. 2, Ch. 32 to 34; Vol. 3, Ch. 53 | `FlapActuator`, `FlapSystem`, `FlapDetail` |
| `projects/scara_robot` | Vol. 2, Ch. 35 to 38 | `ScaraJointModules`, `ScaraToolJoints`, `ScaraRobot` |
| `projects/robotic_wrist` | Vol. 2, Ch. 39 and 40; Vol. 3, Ch. 52 | `WristDifferential`, `WristIntegration`, `WristDetail` |
| `projects/palletizer` | Vol. 2, Ch. 41 to 48; Vol. 3, Ch. 55 and 62 | `PalletizerDrives`, `PalletizerRobot`, `ToolChanger`, `ClampGripper`, `VacuumGripper`, `BagGripper`, `ComplianceUnit`, `PalletizerDetail`, `StopBracket` |
| `design_office` | Vol. 3, Ch. 49 to 66 | `PartNumbers`, `StandardParts`, `MaterialsFits`, `RangeSweep`, `MassProperties`, `StudyParts`, `StudyAssemblies`, `FatigueSpots`, `StackGaps`, `PartDrawings`, `BillOfMaterials`, `DesignForAssembly`, `Making` |

The checks and controls of each project are listed in its `README.md`; Appendix B of Volume 1 lists those of
Volume 1 with what each control breaks.

## Running a script in Fusion

1. In Fusion: **Utilities > Add-Ins > Scripts and Add-Ins** (Shift+S).
2. Click the green **+** next to *My Scripts* and pick the script's folder, for example
   `projects/gear_pair/GearPairOnPlate`.
3. Select the script and click **Run**. Each script opens a new design, so nothing you have open is touched.

A script that uses the libraries must stay inside its project folder in the repository, where it finds `common/`.

## Running the checks

The checks talk to Fusion through its MCP server (Chapter 3 of the book):

1. In Fusion, open **Preferences > General > API**, tick *Fusion MCP Server (runs locally on this device)*, Apply, OK.
2. `python tools/fusion_mcp.py tools` finds the server's port (27182 by default, or the next ports up) and lists
   its tools. If it finds none, set `FUSION_MCP_URL` to the address Fusion prints in its Text Commands palette
   (**File > View > Show Text Commands**), for example `http://127.0.0.1:27183/mcp`.

```
python tools/run_check.py gearpair_check                  # -> projects/gear_pair/results/gearpair_check.txt
python tools/run_check.py controls/planetary_ring_phase   # a control: it must fail
python tools/run_check.py --all                           # every check of every project but extras/
python tools/run_check.py --cleanup                       # close the designs the checks opened
python tools/prompt_lint.py --all                         # the five parts of every prompt
```

A check is found by its name in whichever project keeps it, and its result goes to that project's `results/`. A
check prints `ALL PASS` or the lines that failed. Every design a check opens is marked and closed again, unsaved, by
`--cleanup`; nothing else in Fusion is touched. The server runs any Python it is given, with your rights: read a
script before you run it, and switch the server off when you are not using it.

## Versions

Results in the book were measured in Autodesk Fusion 2706.1.17 in November 2026. When Fusion changes, the checks are
run again and any result that no longer holds is corrected here first.

## Trademarks

Autodesk, Autodesk Fusion and Fusion are registered trademarks or trademarks of Autodesk, Inc., and/or its
subsidiaries and/or affiliates in the USA and/or other countries. Claude and Anthropic are trademarks of Anthropic,
PBC. This repository and the book are independent and not sponsored, endorsed or approved by Autodesk or Anthropic.
