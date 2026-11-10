# Pictures for Chapter 14: HelicalPair as a herringbone; the script colours the parts.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core


def run(_context):
    ns = fc.load('__COMPANION__/projects/helical_pair/HelicalPair/HelicalPair.py')
    print(ns['build'](herringbone=True).splitlines()[0])
    fc.mark()
    try:
        fc.design().rootComponent.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    adsk.core.Application.get().activeViewport.fit()
