# CONTROL for the prompt's hard constraint "the bolts' grip: along the bolt's axis, from the head's underside ... to
# the nut's face nearer the head": the grip taken to the nut's far face instead. The whole check runs on it, its
# results kept apart in results/fatigue_spots_control.json. Expected: the bolts' line fails, the grip a nut's
# thickness long, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/fatigue_spots_check.py'
OLD = "    near = min(planes, key=lambda s: abs(s - under))\n"
NEW = "    near = max(planes, key=lambda s: abs(s - under))\n"
OLD_OUT = "os.path.join(COMP, 'results', 'fatigue_spots.json')"
NEW_OUT = "os.path.join(COMP, 'results', 'fatigue_spots_control.json')"


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r), (%r, %r)])\n" % (OLD, NEW, OLD_OUT, NEW_OUT))
    env = {'__name__': 'fatigue_spots_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the grip taken to the nut\'s far face; the bolts\' line above MUST fail, and it alone')
