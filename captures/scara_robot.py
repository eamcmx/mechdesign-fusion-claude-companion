# Pictures for Chapter 38: the robot at rest, its arms along +X. The robot is posed through its joints, and the joints' glyphs are hidden rather
# than the joints deleted (deleting a joint puts its parts back as built).
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (0.0, 0.0, 0.0, 0.0)
HIDE = ()


def run(_context):
    ns = fc.load('__COMPANION__/scripts/ScaraRobot/ScaraRobot.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    ns['pose'](*POSE)
    des = fc.design()
    for comp in des.allComponents:
        comp.isJointsFolderLightBulbOn = False
        try:
            comp.isOriginFolderLightBulbOn = False
        except Exception:
            pass
    for o in des.rootComponent.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
