# Pictures for Chapter 34: FlapSystem with the flap at DELTA deg; HIDE lists the components hidden for the view
# (searched in every subassembly).
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

DELTA = 0.0
HIDE = ('Upper skin',)


def run(_context):
    ns = fc.load('__COMPANION__/scripts/FlapSystem/FlapSystem.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    ns['pose'](DELTA, ns['_last_joints'])
    adsk.doEvents()
    root = fc.design().rootComponent
    for comp in fc.design().allComponents:                     # the joints stay (deleting one puts its parts back as
        try:                                                    # built); only their glyphs are hidden
            comp.isJointsFolderLightBulbOn = False
        except Exception:
            pass
    for o in root.allOccurrences:
        if any(o.component.name.startswith(h) for h in HIDE):
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
