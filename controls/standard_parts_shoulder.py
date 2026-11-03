# CONTROL for the prompt's DERIVED line "the shoulder from bearing_6206(): the first whole millimetre over SKF's da min":
# the shaft drawn with Table 11-2's 35 mm instead, past the dialog's refusal. The whole check runs on it. Expected:
# 35 lies under SKF's 35.6, so the shoulder's line fails, and every other line passes, the stack still closed.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/standard_parts_check.py'
OLD = "    d_sh = shoulder\n"
NEW = "    d_sh = 35.0\n"


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    env = {'__name__': 'standard_parts_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the shoulder 35 mm; the shoulder\'s line above MUST fail, and it alone')
