# Pictures for Chapter 54: the rear axle's carrier in detail, assembled, built by AxleDetail.py inside Chapter 31's
# axle with the housing and the wheel ends hidden. Taken from the standard views capture.py names.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)


def run(_context):
    ns = fc.load('__COMPANION__/projects/rear_axle/AxleDetail/AxleDetail.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    for top in root.occurrences:
        if top.component.name == 'Housing' or top.component.name.startswith('Wheel end'):
            top.isLightBulbOn = False
    import adsk.core
    adsk.core.Application.get().activeViewport.fit()
