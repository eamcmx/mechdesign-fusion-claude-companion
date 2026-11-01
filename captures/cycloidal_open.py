# Picture for Chapter 19: CycloidalDrive with the output flange hidden: the disc, the ring pins, the eccentric.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def run(_context):
    ns = fc.load('__COMPANION__/scripts/CycloidalDrive/CycloidalDrive.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for o in root.occurrences:
        if o.component.name == 'Output':
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
