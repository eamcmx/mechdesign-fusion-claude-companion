# CONTROL for the prompt's hard constraint "the O-ring's groove 24.30 across and 3.40 wide, to ERIKS's dynamic
# table": the cut that draws the groove passed through unchanged, so it stays Chapter 48's 25 by 3.5, as deep as the
# O-ring is thick. The whole check runs on it. Expected: the groove's line fails, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/projects/palletizer/checks/palletizer_detail_check.py'
OLD = "            return plain_zcyl(zc - G['width'] / 2.0, zc + G['width'] / 2.0, G['groove_d'] / 2.0, x, y)"
NEW = "            return plain_zcyl(z0, z1, r, x, y)"


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    env = {'__name__': 'palletizer_detail_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the O-ring\'s groove left 25 x 3.5; the groove\'s line above MUST fail, and it alone')
