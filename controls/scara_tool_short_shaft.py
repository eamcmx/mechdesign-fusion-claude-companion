# CONTROL for the prompt line "the shaft long enough to stay in the screw nut over the whole stroke": the shaft is
# cut 50 mm shorter at its top, as a shaft ordered for the nuts' span and the stroke alone would be. Nothing
# touches anything else, so the interference checks pass; at the full stroke the shaft's top is inside the screw
# nut's rotor and the engagement check must fail.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/ScaraToolJoints/ScaraToolJoints.py'


def top_z(occ):
    return max(f.boundingBox.maxPoint.z * 10 for body in occ.bRepBodies for f in body.faces
               if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType)


def run(_context):
    ns = fc.load(PATH)
    ns['sd'].LAY37['above'] = 5.0 - 50.0
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    by = {x.component.name: x for x in leaves}
    ns['pose'](200.0, 0.0)
    c = fc.interference(leaves)
    fc.ok(not c, 'no interference at the full stroke %s' % (c or ''))
    shaft_top = top_z(by['Ball screw spline shaft SP20'])
    rotor_top = top_z(by['Screw nut SP20 rotor'])
    fc.ok(shaft_top >= rotor_top + 5.0 - 0.01, 'at the full stroke the shaft\'s top %.2f mm, the screw nut\'s rotor\'s top %.2f mm' % (shaft_top, rotor_top))
    ns['sd'].LAY37['above'] = 5.0
    print('CONTROL: the shaft 50 mm short; the engagement check MUST fail')
