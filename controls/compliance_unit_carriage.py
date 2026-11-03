# CONTROL for the prompt line "the carriage: a plate 176 x 200 x 32, 2 mm clear of the rails' running faces": the
# carriage made as wide as the cheeks allow, 220 across. Expected: at rest its top only meets the rails' lower ends,
# so it passes; raised the whole stroke it runs 28 mm up into both rails, and the raised check fails.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/scripts/ComplianceUnit/ComplianceUnit.py'
OLD = "L = cu.LAY\n"
NEW = "L = dict(cu.LAY, carriage_xy=(110.0, 100.0))\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    fc.ok(not c, 'at rest: no interference %s' % (c or ''))
    G = ns['_last']['groups']
    ns['set_state'](G, ns['L']['stroke'])
    springs = [o for o in leaves if o.component.name.startswith('Spring ') and not o.component.name.startswith('Spring seat')]
    c = fc.interference([o for o in leaves if o not in springs])
    vol = round(sum(v for a, b, v in c), 1)
    fc.ok(not c, 'raised the whole stroke: no interference but the springs (%d clashes, %.1f mm3) %s'
          % (len(c), vol, sorted(set((a, b) for a, b, v in c))[:3]))
    print('CONTROL: the carriage 220 across; the raised check above MUST fail')
