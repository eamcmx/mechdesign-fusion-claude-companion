# CONTROL for the prompt's hard constraint "arms: the robot at the cycle's high point": the script copies the arms as
# the robot is built, every joint at zero, and never turns them to the high point. The whole check runs on it.
# Expected: the arms' picks line fails (the tool flange's outer face is not where the high point puts it) and the tool
# flange's line fails (its box away from the high point's flange), and they alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/study_assemblies_check.py'
EDITS = [("return dict(machine='robot', pose=(hi['q2'], hi['q3']),", "return dict(machine='robot', pose=None,")]


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'study_assemblies_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the arms at the robot's built pose; the arms' picks and tool flange lines above MUST fail, and they alone")
