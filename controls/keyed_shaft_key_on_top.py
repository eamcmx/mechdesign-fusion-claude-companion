# CONTROL for the prompt line "The key sits with t1 of its height in the shaft's slot and the rest in the hub's slot":
# the key is drawn standing on the shaft's surface instead of in its slot. Expected: it no longer straddles the shaft's
# surface, and it reaches h - t2 = 6 - 2.8 = 3.2 mm past the hub's slot, an overlap of b * 3.2 * L_key = 6 * 3.2 * 18
# = 345.6 mm^3 with the hub.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/KeyedShaft/KeyedShaft.py'
OLD = "    rect(sk, -b / 2, R - t1, b / 2, R - t1 + h)\n"
NEW = "    rect(sk, -b / 2, R, b / 2, R + h)\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    print('  interference: %s' % c)
    fc.ok(not c, 'no interference')
    print('CONTROL: the key stands on the shaft instead of in its slot; the check above MUST fail')
