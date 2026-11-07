# CONTROL for the prompt's hard constraint "the arm turned 45 deg about its own axis, so that the shock absorber's push
# lies square to a face of the arm": the script leaves the arm as Chapter 56 made it, its sides along the end plate's
# edges and the push along its diagonal, and still rounds the root. The whole check runs on it, its results kept apart
# in results/stop_bracket_control.json. Expected: on both brackets the sides line (45 deg off), the volume line and
# the heads' line (the fillet's toe round the arm's edge runs under the M8 heads), and the sweep's interference line
# fail, and they alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/stop_bracket_check.py'
EDITS = [("TURNED = True ", "TURNED = False")]


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    src = src.replace("'stop_bracket.json'", "'stop_bracket_control.json'")
    env = {'__name__': 'stop_bracket_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the arm left on its diagonal; the sides, volume, heads and sweep lines above MUST fail, and they alone")
