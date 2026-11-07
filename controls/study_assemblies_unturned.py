# CONTROL for the prompt's hard constraint "the opening bracket with Chapter 62's change: its arm turned": the script
# builds the stop study on the bracket as Chapter 56 left it, the arm on its diagonal, with the 2.5 mm root. The whole
# check runs on it. Expected: the stop's bracket volume line fails (99 541.2 against the turned arm's 99 399.2), and it
# alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/study_assemblies_check.py'
EDITS = [("picks=picks, turned=(which == 'stop'))", "picks=picks, turned=False)")]


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'study_assemblies_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the stop study on the unturned arm; the stop's bracket line above MUST fail, and it alone")
