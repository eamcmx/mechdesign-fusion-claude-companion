# Pictures for Chapter 49: the palletizer's robot at the standard cycle's low point, numbered by PartNumbers.py and
# coloured by what its numbers say, made parts blue, bought parts yellow; from Chapter 43's camera, taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import palletizer_design as pd
import adsk.core, adsk.fusion

lo = pd.waypoints()['lo']
POSE = (0.0, lo['q2'], lo['q3'])
EYE = (1.0, -1.35, 0.55)
TARGET = (60.0, 0.0, 140.0)          # cm
MADE, BOUGHT = 'Paint - Enamel Glossy (Blue)', 'Paint - Enamel Glossy (Yellow)'


def run(_context):
    import mdfc_parts as mp
    pn = fc.load('__COMPANION__/scripts/PartNumbers/PartNumbers.py')
    robot = fc.load('__COMPANION__/scripts/PalletizerRobot/PalletizerRobot.py')
    print(robot['build'](*POSE).splitlines()[0])
    fc.mark()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    root = des.rootComponent
    done, part_of, left = pn['number'](root, 'robot')
    print('  numbered %d components; with a bought part %d; left %s' % (len(done), len(part_of), sorted(set(left))))
    for o in root.occurrences:
        o.appearance = None
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count:
            mp.colour(des, o, BOUGHT if c.partNumber.startswith('S-') or c.description.startswith('with the ') else MADE)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 900.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
