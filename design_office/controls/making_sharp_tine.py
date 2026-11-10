# CONTROL for the prompt's hard constraint "the tine's inside corner rounded to lib/making.py's radius": the script
# leaves the corner as the gripper's model has it, sharp. The whole check runs on it. Expected: the tine's three lines
# fail (its volume against the rounded hand figure, its corner, and the nest, whose share of the plate is the sharp
# outline's 10.100 % against the rounded tine's 10.117 %), and they alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/design_office/checks/making_check.py'
EDITS = [("    if tn['r'] > 0 and len(corner) == 1:\n", "    if False:\n")]


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'making_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the tine's corner left sharp; the tine's volume, corner and nest lines above MUST fail, and they alone")
