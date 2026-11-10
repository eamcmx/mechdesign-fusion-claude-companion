# Pictures for Chapter 49: the bag gripper numbered by PartNumbers.py and coloured by what its numbers say, made parts
# blue, bought parts yellow, the bag and the station's rollers hidden; from Chapter 47's camera, taken as 'current'.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

EYE = (0.85, -1.0, 0.45)
TARGET = (0.0, 0.0, -28.0)          # cm
MADE, BOUGHT = 'Paint - Enamel Glossy (Blue)', 'Paint - Enamel Glossy (Yellow)'


def colour_by_number(des, root):
    import mdfc_parts as mp
    for o in root.occurrences:                                   # the subassemblies' own colours off
        o.appearance = None
    made = bought = hidden = 0
    for o in root.allOccurrences:
        c = o.component
        if c.bRepBodies.count == 0:
            continue
        if o.fullPathName.startswith('Bag') or o.fullPathName.startswith('Station'):
            o.isLightBulbOn = False
            hidden += 1
        elif c.partNumber.startswith('S-') or c.description.startswith('with the '):
            mp.colour(des, o, BOUGHT)
            bought += 1
        else:
            mp.colour(des, o, MADE)
            made += 1
    print('  coloured: %d made, %d bought, %d hidden' % (made, bought, hidden))


def run(_context):
    sys.path.insert(0, '__COMPANION__/common')
    import companion_paths                   # every lib/ folder on sys.path
    ns = fc.load('__COMPANION__/design_office/PartNumbers/PartNumbers.py')
    print(ns['build']('bag').splitlines()[0])
    fc.mark()
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    colour_by_number(des, des.rootComponent)
    vp = adsk.core.Application.get().activeViewport
    cam = vp.camera
    n = math.sqrt(sum(c * c for c in EYE))
    cam.target = adsk.core.Point3D.create(*TARGET)
    cam.eye = adsk.core.Point3D.create(*[TARGET[i] + 400.0 * EYE[i] / n for i in range(3)])
    cam.upVector = adsk.core.Vector3D.create(0, 0, 1)
    cam.isFitView = True
    vp.camera = cam
    vp.refresh()
