#!/bin/sh
# The Fusion steps waiting for Fusion's MCP server (3 November 2026), in the order they are needed. Run from the
# companion folder once Fusion answers (python tools/fusion_mcp.py tools prints its tools). Each step closes the
# designs it opened. Fusion's runner has failed after about 15 heavy runs in one session (RecursionError in
# __getattr__): if a step prints that, restart Fusion and go on from that step.
set -e
cd "$(dirname "$0")/.."
B=../book

# Chapter 47: the exploded view, then its balloons
python tools/capture.py captures/bag_gripper_exploded.py $B/vol2/figures/ch47_exploded current
python tools/balloons.py $B/vol2/figures/ch47_exploded_current.png --width 115
python tools/run_check.py --cleanup

# Chapter 48: the check (writes lib/compliance_unit_weighed.json), the control, the pictures
python tools/run_check.py compliance_unit_check
python tools/run_check.py --cleanup
python tools/run_check.py controls/compliance_unit_carriage
python tools/run_check.py --cleanup
python tools/capture.py captures/compliance_unit.py $B/vol2/figures/ch48_unit current
python tools/capture.py captures/compliance_unit_raised.py $B/vol2/figures/ch48_raised current
python tools/capture.py captures/compliance_unit_exploded.py $B/vol2/figures/ch48_exploded current
python tools/balloons.py $B/vol2/figures/ch48_exploded_current.png --width 115
python tools/run_check.py --cleanup

# Chapter 49: the check, the control, the two pictures
python tools/run_check.py part_numbers_check
python tools/run_check.py --cleanup
python tools/run_check.py controls/part_numbers_names
python tools/run_check.py --cleanup
python tools/capture.py captures/part_numbers_bag.py $B/vol3/figures/ch49_bag_numbered current
python tools/capture.py captures/part_numbers_robot.py $B/vol3/figures/ch49_robot_numbered current
python tools/run_check.py --cleanup
