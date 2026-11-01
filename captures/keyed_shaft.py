# Picture for Chapter 5: KeyedShaft with its defaults; the hub see-through so the key in its slots can be seen.
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
    ns = fc.load('__COMPANION__/scripts/KeyedShaft/KeyedShaft.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    colour(fc.occ('Bearing block'), 'Paint - Enamel Glossy (Grey)')
    colour(fc.occ('Shaft'), 'Paint - Enamel Glossy (Blue)')
    colour(fc.occ('Key'), 'Paint - Enamel Glossy (Red)')
    colour(fc.occ('Hub'), 'Paint - Enamel Glossy (Green)')
    fc.occ('Hub').component.bRepBodies.item(0).opacity = 0.4
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
