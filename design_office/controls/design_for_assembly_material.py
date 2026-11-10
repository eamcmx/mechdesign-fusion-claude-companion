# CONTROL for the prompt's hard constraint "answer each question for a future redesign, not for the parts as they are
# (Dewhurst & Meeker, 2014, p. 5)": the script answers "must be of a different material" for every steel part that
# stands on aluminium as it is drawn (the two cylinder brackets, the four stop brackets, the dowel pin), the mistake
# the paper describes. The whole check runs on it.
# Expected: the theoretical minimum, the questions that keep the pieces and the design efficiency fail, and they alone.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

CHECK = '__COMPANION__/design_office/checks/design_for_assembly_check.py'
EDITS = [("    tagged = tag(des, occ, lines0)\n",
          "    STEEL_ON_AL = ('P13-003', 'P13-017', 'P13-018', 'S-0052')\n"
          "    DF.BAG = [(l[0], l[1], l[2], l[3], l[1], 'material') + tuple(l[6:]) if l[0] in STEEL_ON_AL else l\n"
          "              for l in DF.BAG]\n"
          "    tagged = tag(des, occ, lines0)\n"),
         ("os.path.join(PROJECT, 'results', 'dfa.json')", "os.path.join(PROJECT, 'results', 'dfa_control.json')")]


def run(_context):
    comp = '__COMPANION__'
    src = open(CHECK, encoding='utf-8').read().replace('__' + 'COMPANION' + '__', comp)
    src = src.replace("'results', 'dfa.json'", "'results', 'dfa_control.json'")      # before the edits go in
    src = src.replace("EDITS = ()", "EDITS = %r" % (EDITS,))
    env = {'__name__': 'design_for_assembly_check_control'}
    exec(compile(src, CHECK, 'exec'), env)
    env['run'](_context)
    print("CONTROL: the steel parts on aluminium answered 'different material' as they are; the minimum, its questions "
          "and the efficiency above MUST fail, and they alone")
