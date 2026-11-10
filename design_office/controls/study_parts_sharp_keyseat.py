# CONTROL for the prompt's hard constraint "its floor's four edges rounded to r/d = 0.02, 0.6 mm": the script cuts the
# keyseat and leaves its floor's corners sharp, the cut Table 7-1's factors are not for. The whole check runs on it.
# Expected: the keyseat's faces fail (8 for 12), and they alone: the four fillets hold 4.5 mm3 of 105 319, inside the
# volume's 0.01 % tolerance, so the faces are the line that sees them.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/design_office/checks/study_parts_check.py'
EDITS = [("    if which == 'keyseat':                    # the keyseat's bottom corners, r/d = 0.02\n"
          "        floor = _face_at(body, (75.0, 0.0, 11.0), (0, 0, 1))\n"
          "        _fillet(body, list(floor.edges), 0.6)\n",
          "    if False:                                 # the keyseat's corners left sharp\n"
          "        pass\n")]


def run(_context):
    comp = '__COMPANION__'
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'study_parts_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the keyseat's floor left sharp; its faces above MUST fail, and they alone")
