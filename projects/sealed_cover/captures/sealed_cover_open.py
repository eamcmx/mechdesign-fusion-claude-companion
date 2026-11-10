# Picture for Chapter 11: SealedCover with the cover and the bolts hidden, the O-ring in its face groove.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def run(_context):
    ns = fc.load('__COMPANION__/projects/sealed_cover/SealedCover/SealedCover.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for o in root.occurrences:
        if o.component.name == 'Cover' or o.component.name.startswith('Bolt'):
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
