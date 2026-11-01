# Picture for Chapter 8: BearingSeat, 6002, with the housing and the outer ring see-through so the balls show.
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
    ns = fc.load('__COMPANION__/scripts/BearingSeat/BearingSeat.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    for n, c in (('Housing', 'Grey'), ('Outer ring', 'Yellow'), ('Inner ring', 'Yellow'), ('Balls', 'Red'), ('Shaft', 'Blue')):
        colour(fc.occ(n), 'Paint - Enamel Glossy (%s)' % c)
    fc.occ('Housing').component.bRepBodies.item(0).opacity = 0.25
    fc.occ('Outer ring').component.bRepBodies.item(0).opacity = 0.45
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
