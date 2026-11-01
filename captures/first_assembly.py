# Picture for Chapter 2: FirstAssembly with its defaults, block grey and shaft blue, the shaft's joint turned 30 deg.
import math, sys, importlib
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
    ns = fc.load('__COMPANION__/scripts/FirstAssembly/FirstAssembly.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    colour(fc.occ('Block'), 'Paint - Enamel Glossy (Grey)')
    colour(fc.occ('Shaft'), 'Paint - Enamel Glossy (Blue)')
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = True
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
