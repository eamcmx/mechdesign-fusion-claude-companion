# Picture for Chapter 2's control: the shaft created at the origin and moved after creation ends up back at the origin,
# inside the solid block. The block is shown see-through so the buried shaft can be seen.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

OLD = "    shaft = place(root, 'Shaft', x, y, 0.0)\n"
NEW = ("    shaft = place(root, 'Shaft')\n"
       "    mv = adsk.core.Matrix3D.create()\n"
       "    mv.translation = V3(x * CM, y * CM, 0)\n"
       "    shaft.transform = mv\n")


def colour(o, name):
    des = fc.design()
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
    ap = des.appearances.itemByName(name) or des.appearances.addByCopy(lib.appearances.itemByName(name), name)
    o.appearance = ap


def run(_context):
    ns = fc.load('__COMPANION__/scripts/FirstAssembly/FirstAssembly.py', [(OLD, NEW)])
    ns['build']()
    fc.mark()
    colour(fc.occ('Block'), 'Paint - Enamel Glossy (Grey)')
    colour(fc.occ('Shaft'), 'Paint - Enamel Glossy (Red)')
    fc.occ('Block').component.bRepBodies.item(0).opacity = 0.35
    adsk.core.Application.get().activeViewport.fit()
