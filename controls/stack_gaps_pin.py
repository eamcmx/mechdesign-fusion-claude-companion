# CONTROL for the prompt's hard constraint "leaving out the names a gap excludes (the horn's pin from the horn)": every
# part whose name starts with the second name measured, the horn's pin with the horn. The whole check runs on it, its
# results kept apart in results/stack_gaps_control.json. Expected: the flap's line fails, the rod end measured to the
# pin in its own bore, and it alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/stack_gaps_check.py'
OLD = "o.component.name.startswith(b) and not o.component.name.startswith(tuple(b_not))"
NEW = "o.component.name.startswith(b)"
OLD_OUT = "os.path.join(COMP, 'results', 'stack_gaps.json')"
NEW_OUT = "os.path.join(COMP, 'results', 'stack_gaps_control.json')"


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("    ns = fc.load(PATH)\n", "    ns = fc.load(PATH, [(%r, %r), (%r, %r)])\n" % (OLD, NEW, OLD_OUT, NEW_OUT))
    env = {'__name__': 'stack_gaps_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print('CONTROL: the horn\'s pin measured with the horn; the flap\'s line above MUST fail, and it alone')
