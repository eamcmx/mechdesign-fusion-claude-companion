# CONTROL for the prompt line "a gap of the output gear faces each side gear's tooth": the output gear is put on
# its shaft turned so that a tooth, not a gap, faces each side gear, as a gear drawn at its own zero would be. The
# links still hold every angle to its equation, so only the interference check can see it: it must find the
# output gear inside both side gears.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)

PATH = '__COMPANION__/projects/robotic_wrist/WristDifferential/WristDifferential.py'


def run(_context):
    ns = fc.load(PATH, edits=[("psi = (90.0 - 180.0 / g['z']) if phase is None else phase",
                               "psi = 90.0 if phase is None else phase")])
    print(ns['VERSION'])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    gears = [o for o in root.allOccurrences if 'bevel gear' in o.component.name]
    ns['pose'](0.0, 0.0)
    h = fc.interference(gears)
    fc.ok(not h, 'no interference among the three bevel gears at rest %s' % (h or ''))
    print('CONTROL: the output gear turned half a tooth; the interference check MUST fail')
    fc.verdict()
