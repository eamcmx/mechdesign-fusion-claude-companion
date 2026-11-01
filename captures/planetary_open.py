# Picture for Chapter 15: PlanetaryPrompt with its defaults, the carrier and the sun bearing hidden, the parts coloured
# for the picture only (the script leaves them grey): ring grey, sun blue, planets yellow.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def colour(design, occ, name):
    ap = design.appearances.itemByName(name)
    if not ap:
        lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
        ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
    occ.appearance = ap


def run(_context):
    ns = fc.load('__COMPANION__/scripts/PlanetaryPrompt/PlanetaryPrompt.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    des = fc.design()
    root = des.rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for o in root.occurrences:
        n = o.component.name
        if n in ('Carrier', 'Sun bearing'):
            o.isLightBulbOn = False
        elif n == 'Sun':
            colour(des, o, 'Paint - Enamel Glossy (Blue)')
        elif n.startswith('Planet'):
            colour(des, o, 'Paint - Enamel Glossy (Yellow)')
        elif n == 'Ring':
            colour(des, o, 'Paint - Enamel Glossy (Grey)')
    adsk.core.Application.get().activeViewport.fit()
