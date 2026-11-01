# Pictures for Chapter 33: ScaraArm with its defaults, set by its inverse kinematics to reach TARGET with elbow
# ELBOW, standing on the workspace plate of checks/scara_check.py: the floor its tool covers.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

TARGET, ELBOW = (300.0, 200.0, 250.0, 0.0), 0


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ScaraArm/ScaraArm.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    s = ns['scara']()
    des = fc.design()
    try:
        des.rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    occ, area, r_x = fc.load('__COMPANION__/checks/scara_check.py')['workspace'](s)
    ns['colour'](des, occ, 'Paint - Enamel Glossy (Green)')
    print('workspace %.1f cm2, the arcs cross %.3f mm behind the axis' % (area / 100, r_x))
    q1, q2, d3, q4 = ns['ik'](s, *TARGET)[ELBOW]
    for n, v in (('J1', q1), ('J2', q2), ('J4', q4)):
        fc.joint(n).jointMotion.rotationValue = math.radians(v)
    fc.joint('J3').jointMotion.slideValue = -d3 / 10.0
    adsk.doEvents()
    print('elbow %d: q1 %.1f, q2 %.1f, d3 %.1f, q4 %.1f' % (ELBOW, q1, q2, d3, q4))
    adsk.core.Application.get().activeViewport.fit()
