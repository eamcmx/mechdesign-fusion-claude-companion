# Picture for Chapter 7: FitPair, 20 H7/g6 at maximum material; the block see-through so the pin in its hole shows.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def colour(o, name):
    des = fc.design()
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
    ap = des.appearances.itemByName(name) or des.appearances.addByCopy(lib.appearances.itemByName(name), name)
    o.appearance = ap


def run(_context):
    ns = fc.load('__COMPANION__/scripts/FitPair/FitPair.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    colour(fc.occ('Block'), 'Paint - Enamel Glossy (Grey)')
    colour(fc.occ('Pin'), 'Paint - Enamel Glossy (Blue)')
    fc.occ('Block').component.bRepBodies.item(0).opacity = 0.35
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
