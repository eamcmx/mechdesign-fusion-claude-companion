# CONTROL for the prompt's hard constraint "carry into the model what Part 10 decided for its shape before drawing it:
# the opening bracket's arm root filleted 2.5 mm": the script draws the bracket as the machine's model leaves it,
# without the fillet. The whole check runs on it, into a folder of its own, its results kept apart in
# results/part_drawings_control.json and results/drawings_control/. Expected: the bracket's root line fails, and it
# alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/part_drawings_check.py'
EDITS = [("        done[ch['kind']] = CHANGES[ch['kind']](root, body, got['m'], ch)\n", "        pass\n"),
         ("os.path.join(COMP, 'results', 'part_drawings.json')", "os.path.join(COMP, 'results', 'part_drawings_control.json')"),
         ("os.path.join(COMP, 'results', 'drawings')", "os.path.join(COMP, 'results', 'drawings_control')")]


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    src = src.replace("'Chapter 63 check %s'", "'Chapter 63 control %s'")
    src = src.replace("'results', 'drawings', p['pdf']", "'results', 'drawings_control', p['pdf']")
    env = {'__name__': 'part_drawings_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the bracket drawn without its root fillet; the bracket's root line above MUST fail, and it alone")
