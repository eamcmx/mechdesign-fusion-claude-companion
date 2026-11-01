# Pictures for Chapter 30: LandingGear with its defaults, the upper brace driven to FRACTION of its travel from down
# to up (0 = down and locked). The capture tool's argument picks the view; the fraction is set below by sed copies.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

FRACTION = 1.0


def run(_context):
    ns = fc.load('__COMPANION__/scripts/LandingGear/LandingGear.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    g = ns['gear']()
    up = fc.occ('Upper brace')
    RU = fc.rotation(up)
    ju = fc.joint('Upper brace turns')
    ju.jointMotion.rotationValue = 0.01
    adsk.doEvents()
    s = 1 if -fc.turn_deg(fc.rotation(up), RU, (0, 1, 0)) > 0 else -1
    total = (g['th_up'] - g['th_down']) * FRACTION
    n = int(abs(math.degrees(total)) // 3) + 1
    for k in range(1, n + 1):
        ju.jointMotion.rotationValue = total * k / n * s
        adsk.doEvents()
    adsk.core.Application.get().activeViewport.fit()
