# Pictures for Chapter 31: FlapActuator with its defaults, the screw turned to FRACTION of the travel to 40 deg.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

FRACTION = 1.0


def run(_context):
    ns = fc.load('__COMPANION__/scripts/FlapActuator/FlapActuator.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    g = ns['drive'](5.0, 16.0, 0.005, 100.0, -45.0, 60.0, 150.0, 400.0, 1500.0, 40.0)
    js = fc.joint('Screw turns')
    nut = fc.occ('Nut')
    x0 = fc.position_mm(nut)[0]
    js.jointMotion.rotationValue = math.radians(36.0)
    adsk.doEvents()
    s = 1 if fc.position_mm(nut)[0] > x0 else -1
    u_end = g['travel'] * FRACTION
    u = 0.0
    while u < u_end - 1e-9:
        u = min(u + 2.0, u_end)
        js.jointMotion.rotationValue = math.radians(u / 5.0 * 360.0) * s
        adsk.doEvents()
    adsk.core.Application.get().activeViewport.fit()
