# Pictures for Chapter 34: JointDrive with the column and the outer arm hidden, to show the drive.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

HIDE = ('Base', 'Arm 2', 'Quill', 'Tool')


def run(_context):
    ns = fc.load('__COMPANION__/scripts/JointDrive/JointDrive.py')
    print(ns['build']().splitlines()[0])
    fc.mark()
    root = fc.design().rootComponent
    try:
        root.isJointsFolderLightBulbOn = False
    except Exception:
        pass
    for o in root.occurrences:
        if o.component.name in HIDE:
            o.isLightBulbOn = False
    adsk.core.Application.get().activeViewport.fit()
