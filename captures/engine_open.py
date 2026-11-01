# Pictures for Chapter 28: SingleCylinderEngine with its defaults, the block hidden and the crank turned 60 deg so
# that the rod leans and the inlet valve is lifted.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def run(_context):
    ns = fc.load('__COMPANION__/scripts/SingleCylinderEngine/SingleCylinderEngine.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    fc.occ('Block').isLightBulbOn = False
    e = ns['engine'](60.0, 54.0, 90.0, 10, 50, 50, 10)
    jc = fc.joint('Crank turns')
    crank = fc.occ('Crankshaft')
    R0 = fc.rotation(crank)
    jc.jointMotion.rotationValue = 0.1
    adsk.doEvents()
    sc = 1 if fc.turn_deg(fc.rotation(crank), R0, (0, 0, 1)) > 0 else -1
    for d in (30.0, 60.0, 90.0, 120.0, 150.0, 180.0, 210.0, 240.0, 270.0, 300.0, 330.0, 360.0, 390.0, 420.0):
        jc.jointMotion.rotationValue = math.radians(d) * sc          # to theta = 780 deg: inlet opening
        adsk.doEvents()
    jv = fc.joint('Inlet valve slides')
    jv.jointMotion.slideValue = 0.1
    adsk.doEvents()
    s = 1 if fc.position_mm(fc.occ('Inlet valve'))[1] > 0 else -1
    jv.jointMotion.slideValue = ns['valve_lift'](e, 6.0, 'in', 780.0) * 0.1 * s
    adsk.doEvents()
    adsk.core.Application.get().activeViewport.fit()
