# Pictures for Chapter 15: PlanetaryPrompt with its defaults.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def run(_context):
    ns = fc.load('__COMPANION__/projects/planetary_gearbox/PlanetaryPrompt/PlanetaryPrompt.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
