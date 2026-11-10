# CONTROL for the prompt's hard constraint "the spacer from cone A's front face to cone B's, 67.8 long": the tube that
# draws the spacer passed through unchanged, so it stays Chapter 29's 63.3, placed from the bearings' total widths. The
# whole check runs on it. Expected: the spacer's line fails, the spacer 2.25 mm short of each cone, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/projects/rear_axle/checks/axle_detail_check.py'
OLD = "    if on_x and near(r_out, 28.0) and near(p0[0], L['spacer_v2'][0], 1e-3):"
NEW = "    if False:"


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    env = {'__name__': 'axle_detail_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the spacer left at 63.3 mm; the spacer\'s line above MUST fail, and it alone')
