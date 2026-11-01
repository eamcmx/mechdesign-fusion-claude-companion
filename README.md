# Mechanical Design in Autodesk Fusion with Claude: companion repository

The scripts, checks and results of the book *Mechanical Design in Autodesk Fusion with Claude* by Emmanuel A.
Merchán-Cruz. Every script in the book was written from the five-part prompt printed with it, run in Autodesk
Fusion, and checked against numbers stated before the run.

## What is here

| Folder | What it holds |
|---|---|
| `scripts/<Name>/` | A Fusion script (`<Name>.py`, `<Name>.manifest`) and the prompt it was written from (`PROMPT.txt`) |
| `checks/` | One check per script: it runs the script the way a user does (Run, the dialog, OK) and measures what Fusion built |
| `controls/` | One line of a prompt broken on purpose; the matching check must now fail |
| `results/` | The last result of every check and control, with the Fusion version and the date of the run |
| `tools/` | `run_check.py` (runs a check and keeps its result), `fusion_mcp.py` (talks to Fusion's MCP server), `prompt_lint.py` (the five parts of a prompt), `capture.py` (the book's figures) |

## Scripts by chapter (Volume 1)

| Chapter | Script | Check | Controls |
|---|---|---|---|
| 2 Fusion as a programmable CAD system | `FirstAssembly` | `first_assembly_check` | `first_assembly_moved_after` |
| 6 Shafts, keys and hubs | `KeyedShaft` | `keyed_shaft_check` | `keyed_shaft_key_on_top` |
| 7 Fits and tolerances | `FitPair` | `fit_pair_check` | `fit_pair_nominal` |
| 8 Rolling bearings | `BearingSeat` | `bearing_seat_check` | `bearing_seat_cage_speed` |
| 9 Bolted joints | `BoltedFlange` | `bolted_flange_check` | `bolted_flange_off_holes` |
| 10 Springs, pins and retaining rings | `SpringPlunger` | `spring_plunger_check` | `spring_plunger_no_groove` |
| 11 Couplings and seals | `SealedCover` | `sealed_cover_check` | `sealed_cover_floating` |
| 12 Spur gears and the gear pair | `GearPairOnPlate` | `gearpair_check` | `gearpair_half_tooth` |
| 13 Gear trains | `CompoundTrain` | `compound_train_check` | `compound_train_stage2_phase` |
| 14 Helical and herringbone gears | `HelicalPair` | `helical_pair_check` | `helical_same_hand` |
| 15 Internal gears and planetary gearboxes | `PlanetaryPrompt` | `planetary_check` | `planetary_ring_phase`, `planetary_moved_after` |
| 16 Rack and pinion | `RackPinion` | `rack_pinion_check` | `rack_tooth_under_tooth` |
| 17 Bevel gears | `BevelPair` | `bevel_pair_check` | `bevel_pair_unphased` |
| 18 Worm and wheel | `WormWheel` | `worm_wheel_check` | `worm_wheel_left_hand` |
| 19 Cycloidal and harmonic drives | `CycloidalDrive` | `cycloidal_check` | `cycloidal_disc_not_turning` |
| 20 Belts and chains | `BeltDrive` | `belt_drive_check` | `belt_reversed` |
| 21 Screws and linear guides | `LeadScrewAxis` | `lead_screw_check` | `lead_screw_wrong_way` |
| 22 Linkages | `FourBar` | `four_bar_check` | `four_bar_crossed` |
| 23 Levers, toggles and the adaptive finger | `KneeToggle` | `knee_toggle_check` | `knee_unequal_links` |
| 24 Cams and followers | `CamFollower` | `cam_check` | `cam_harmonic_rise` |
| 25 Geneva and ratchet | `Geneva` | `geneva_check` | `geneva_centre_distance` |
| 26 The bevel differential | `BevelWristPrompt` | `bevelwrist_check` | `bevelwrist_phase` |

## Running a script in Fusion

1. In Fusion: **Utilities > Add-Ins > Scripts and Add-Ins** (Shift+S).
2. Click the green **+** next to *My Scripts* and pick the script's folder in `scripts/`.
3. Select the script and click **Run**. Each script opens a new design, so nothing you have open is touched.

## Running the checks

The checks talk to Fusion through its MCP server (Chapter 3 of the book):

1. In Fusion, open **Preferences > General > API**, tick *Fusion MCP Server (runs locally on this device)*, Apply, OK.
2. `python tools/fusion_mcp.py tools` finds the server's port (27182 by default, or the next ports up) and lists
   its tools. If it finds none, set `FUSION_MCP_URL` to the address Fusion prints in its Text Commands palette
   (**File > View > Show Text Commands**), for example `http://127.0.0.1:27183/mcp`.

```
python tools/run_check.py gearpair_check
python tools/run_check.py controls/planetary_ring_phase
python tools/run_check.py --all
python tools/run_check.py --cleanup
```

A check prints `ALL PASS` or the lines that failed. Every design a check opens is marked and closed again, unsaved,
by `--cleanup`; nothing else in Fusion is touched. The server runs any Python it is given, with your rights: read a
script before you run it, and switch the server off when you are not using it.

## Versions

Results in the book were measured in Autodesk Fusion 2706.1.17 on 1 November 2026. When Fusion changes, the checks
are run again and any result that no longer holds is corrected here first.

## Trademarks

Autodesk, Autodesk Fusion and Fusion are registered trademarks or trademarks of Autodesk, Inc., and/or its
subsidiaries and/or affiliates in the USA and/or other countries. Claude and Anthropic are trademarks of Anthropic,
PBC. This repository and the book are independent and not sponsored, endorsed or approved by Autodesk or Anthropic.
