# CONTROL for the prompt's hard constraint that maxon's key passes the housing's back wall: the hole for the
# gearhead's shaft left at Chapter 33's 14 mm, so the key's corner, 7.76 mm from the axis, stands in the wall. The whole
# check runs on it. Expected: the line on the inside of the actuator fails on the key and the housing, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/flap_detail_check.py'
OLD = "HOLE_R = 8.5\n"
NEW = "HOLE_R = 7.0\n"


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    env = {'__name__': 'flap_detail_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the hole for the shaft left at 14 mm; the line on the actuator\'s inside above MUST fail, and it alone')
