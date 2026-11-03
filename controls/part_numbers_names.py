# CONTROL for the prompt's DERIVED line "kind_of(): the name on the parts list that a component's name stands for":
# the components matched to the parts list by their names as they are. Expected: a parts list names a kind once
# ("Tine"), Fusion names each component ("Tine 1" to "Tine 16"), so every part that comes more than once is left
# unnumbered and the check's first line fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import detail_inventory as D
importlib.reload(D)

PATH = '__COMPANION__/scripts/PartNumbers/PartNumbers.py'
OLD = "        kind = D.kind_of(comp.name, nmap)\n"
NEW = "        kind = comp.name if comp.name in nmap else None\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    print(ns['build']('bag').splitlines()[0])
    fc.mark()
    left = [n for n in ns['_last']['left'] if not n.startswith('Bag') and not n.startswith('Roller')]
    fc.ok(not left, 'every part numbered: %d left unnumbered, e.g. %s' % (len(left), sorted(left)[:4]))
    print('CONTROL: components matched by their own names; the check above MUST fail')
