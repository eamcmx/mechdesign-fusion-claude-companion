# Picture for Chapter 26: BevelWristPrompt with its defaults, the frame and the yoke hidden: the four miter gears round
# one apex, coloured for the picture only (output blue, idler grey, side gears yellow).
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
    ns = fc.load('__COMPANION__/scripts/BevelWristPrompt/BevelWristPrompt.py')
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
        if n in ('Frame', 'Yoke'):
            o.isLightBulbOn = False
        elif n == 'Output gear':
            colour(des, o, 'Paint - Enamel Glossy (Blue)')
        elif n.startswith('Side'):
            colour(des, o, 'Paint - Enamel Glossy (Yellow)')
        else:
            colour(des, o, 'Paint - Enamel Glossy (Grey)')
    adsk.core.Application.get().activeViewport.fit()
