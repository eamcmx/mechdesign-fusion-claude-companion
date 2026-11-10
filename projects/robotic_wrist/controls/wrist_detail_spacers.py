# CONTROL for the prompt's hard constraint "the roll shaft's collar and spacers S4 and S5 18 mm across": the ring that
# draws S4 and S5 passed through unchanged, so the spacers stay Chapter 39's 19 mm. The whole check runs on it.
# Expected: S4 and S5 measure 19, over the 6002's da max of 18.5, so the abutments' line fails, and every other line
# passes, the spacers still clear of everything they do not touch.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/projects/robotic_wrist/checks/wrist_detail_check.py'
OLD = "    if (ri, ro) == (7.5, 9.5):"
NEW = "    if False:"


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    env = {'__name__': 'wrist_detail_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: spacers S4 and S5 left at 19 mm; the abutments\' line above MUST fail, and it alone')
