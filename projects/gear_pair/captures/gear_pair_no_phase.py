# Picture for Chapter 12's control: the driven gear not turned (no mesh phase): a tooth meets a tooth on the line of
# centres.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

EDIT = ('    return math.pi - math.pi / z2\n', '    return 0.0\n')


def run(_context):
    ns = fc.load('__COMPANION__/projects/gear_pair/GearPairOnPlate/GearPairOnPlate.py', [EDIT])
    print(ns['build']().splitlines()[0])
    fc.mark()
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
