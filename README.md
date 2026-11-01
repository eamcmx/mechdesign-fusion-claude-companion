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
| `tools/` | `run_check.py` (runs a check and keeps its result), `fusion_mcp.py` (talks to Fusion's local endpoint) |

## Scripts by chapter (Volume 1)

| Chapter | Script | Check | Controls |
|---|---|---|---|
| 11 Spur gears and the gear pair | `GearPairOnPlate` | `gearpair_check` | `gearpair_half_tooth` |
| 14 Internal gears and planetary gearboxes | `PlanetaryPrompt` | `planetary_check` | `planetary_ring_phase`, `planetary_moved_after` |
| 25 The bevel differential | `BevelWristPrompt` | `bevelwrist_check` | `bevelwrist_phase` |

## Running a script in Fusion

1. In Fusion: **Utilities > Add-Ins > Scripts and Add-Ins** (Shift+S).
2. Click the green **+** next to *My Scripts* and pick the script's folder in `scripts/`.
3. Select the script and click **Run**. Each script opens a new design, so nothing you have open is touched.

## Running the checks

Fusion must be running with its local endpoint on: Fusion 2705 or later prints `MCP - http://127.0.0.1:27183/mcp` in
its Text Commands palette.

```
python tools/run_check.py gearpair_check
python tools/run_check.py controls/planetary_ring_phase
python tools/run_check.py --all
python tools/run_check.py --cleanup
```

A check prints `ALL PASS` or the lines that failed. Every design a check opens is marked and closed again, unsaved,
by `--cleanup`; nothing else in Fusion is touched.

## Versions

Results in the book were measured in Autodesk Fusion 2706.1.17 on 1 November 2026. When Fusion changes, the checks
are run again and any result that no longer holds is corrected here first.

## Trademarks

Autodesk, Autodesk Fusion and Fusion are registered trademarks or trademarks of Autodesk, Inc., and/or its
subsidiaries and/or affiliates in the USA and/or other countries. Claude and Anthropic are trademarks of Anthropic,
PBC. This repository and the book are independent and not sponsored, endorsed or approved by Autodesk or Anthropic.
