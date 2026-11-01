# Picture for Chapter 8: BoltedFlange with its defaults (4 x M8 on a 60 mm bolt circle).
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
    ns = fc.load('__COMPANION__/scripts/BoltedFlange/BoltedFlange.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    for o in fc.design().rootComponent.occurrences:
        n = o.component.name
        colour(o, 'Paint - Enamel Glossy (%s)' % ('Grey' if n == 'Plate 1' else 'Blue' if n == 'Plate 2' else 'Black'))
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
