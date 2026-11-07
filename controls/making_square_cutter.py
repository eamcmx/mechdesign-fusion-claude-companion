# CONTROL for the prompt's hard constraint "the arm's root cut by a cutter whose corner radius is the drawing's
# 2.5 mm": the script takes the library's 10 mm bull nose as it comes, its corner radius 1 mm, so the root it cuts is
# R1, not the drawing's R2.5. The whole check runs on it. Expected: the root line fails (CR=1. in the program), and it
# alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/checks/making_check.py'
EDITS = [("    setp(bull.parameters, 'tool_cornerRadius', '2.5 mm', notes)\n", "    pass\n")]


def run(_context):
    comp = CHECK.rsplit('/checks/', 1)[0]                     # the companion's folder, as run_check filled it in
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'making_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the root cut with the library's 1 mm corner; the root line above MUST fail, and it alone")
