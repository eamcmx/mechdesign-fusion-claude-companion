# CONTROL for the prompt line "The crank and its block outboard of the J3 cheek and of the base's flange: the crank's
# plane at y = 640 mm": the crank is left where the starting estimate put it, at y = 495 mm. Expected: the crank's web
# cuts into the J3 cheek and its bearing at rest, and with the crank turned down to q3 = 60 deg into the base's
# flange; the interference checks fail.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/PalletizerDrives/PalletizerDrives.py'
OLD = "Y_CRANK = 640.0                          # the crank's plane, outboard of the J3 cheek and the base's flange"
NEW = "Y_CRANK = 495.0                          # the crank's plane, outboard of the J3 cheek and the base's flange"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    j3, s3 = ns['_last']['joints'][3]
    j3.jointMotion.rotationValue = -s3 * math.radians(60.0)
    adsk.doEvents()
    c = fc.interference(leaves)
    fc.ok(not any('Base flange' in (a, b) for a, b, _ in c), 'J3 at 60 deg: the crank clear of the base\'s flange %s'
          % [x for x in c if 'Base flange' in x[:2]])
    print('CONTROL: the crank at y = 495 mm; the checks above MUST fail')
