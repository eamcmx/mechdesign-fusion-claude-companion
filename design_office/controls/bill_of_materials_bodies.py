# CONTROL for the prompt's hard constraint "count a bought part once per set: a bearing's rings and balls, a tapered
# bearing's cone and cup, a reducer's case and output are one piece": the script counts every body of a bought part as
# a piece. The whole check runs on it, its results kept apart in results/bom_control.json and results/bom_control/.
# Expected: the axle's bearings and the robot's reducers fail, and they alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/design_office/checks/bill_of_materials_check.py'
EDITS = [("    b = BD.finish(BD.bom(key, rows))\n",
          "    BD._sets = lambda names: sum(len(v) for v in names.values())\n    b = BD.finish(BD.bom(key, rows))\n"),
         ("os.path.join(PROJECT, 'results', 'bom.json')", "os.path.join(PROJECT, 'results', 'bom_control.json')"),
         ("os.makedirs(os.path.join(PROJECT, 'results', 'bom'), exist_ok=True)", "os.makedirs(os.path.join(PROJECT, 'results', 'bom_control'), exist_ok=True)"),
         ("path = os.path.join(PROJECT, 'results', 'bom', '%s.csv' % key)", "path = os.path.join(PROJECT, 'results', 'bom_control', '%s.csv' % key)")]


def run(_context):
    comp = '__COMPANION__'                                    # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    src = src.replace("'results', 'bom', m['csv']", "'results', 'bom_control', m['csv']")
    env = {'__name__': 'bill_of_materials_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: every body of a bought part counted as a piece; the two set lines above MUST fail, and they alone")
