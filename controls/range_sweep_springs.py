# CONTROL for the prompt's hard constraint "leave out the parts that are not the machine: ... the compliance unit's
# springs, which are drawn rigid at their free length": the springs swept with the rest. The whole check runs on it.
# Expected: the unit's line fails, the springs meeting what they sit on as the carriage rises, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/range_sweep_check.py'
OLD = "EXCLUDE = dict(unit=('Spring',), clamp=('Carton',),"
NEW = "EXCLUDE = dict(unit=('No such part',), clamp=('Carton',),"


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r)])\n" % (OLD, NEW))
    src = src.replace("OUT = '%s/results/range_sweep.json'" % comp, "OUT = '%s/results/range_sweep_control.json'" % comp)
    env = {'__name__': 'range_sweep_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the compliance unit\'s springs swept with the rest; the unit\'s line above MUST fail, and it alone')
