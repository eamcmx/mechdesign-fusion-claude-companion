# CONTROL for the prompt's hard constraint "the tine's inside corner rounded to lib/making.py's radius": the script
# leaves the corner as the gripper's model has it, sharp. The whole check runs on it. Expected: the tine's two lines
# fail (its volume against the rounded hand figure, and its corner), and they alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/making_check.py'
EDITS = [("    if tn['r'] > 0 and len(corner) == 1:\n", "    if False:\n")]


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'making_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the tine's corner left sharp; the tine's volume and corner lines above MUST fail, and they alone")
