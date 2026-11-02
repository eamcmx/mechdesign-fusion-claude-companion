# Pictures for Chapter 39: the pitch-roll wrist at pitch 25 deg and roll 30 deg (the links move the gears and the
# motors), the joints' glyphs hidden.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

POSE = (25.0, 30.0)
HIDE = ()


def run(_context):
    ns = fc.load('__COMPANION__/scripts/WristDifferential/WristDifferential.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    ns['pose'](*POSE)
    root = fc.design().rootComponent
    root.isJointsFolderLightBulbOn = False
    for o in root.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
