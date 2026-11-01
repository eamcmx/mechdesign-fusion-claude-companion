# CONTROL for the prompt line "the carrier's flange cut flat 166 mm below the axle, with the carrier's plate": the
# flange is left round, 410 mm across, as it was first drawn. Everything fits and turns; the housing's lowest point
# is the flange's, 205 mm below the axle, and the truck keeps 160 mm of ground clearance instead of 195.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/AxleHousing/AxleHousing.py'
OLD = "    mp.union(body, flange)                                                           # the carrier's flange, flat below"
NEW = "    mp.union(body, mp.tube((NECK_X[1] - 14.0, 0, 0), (NECK_X[1], 0, 0), 205.0, NECK_R - WALL))"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    housing = [o for o in root.allOccurrences if o.component.name == 'Banjo housing'][0]
    low = -housing.bRepBodies.item(0).boundingBox.minPoint.z * 10
    print('  the housing reaches %.2f mm below the axle: %.2f mm above the road' % (low, 365 - low))
    fc.ok(365 - low >= 195.0, 'at least 195 mm of ground clearance')
    print('CONTROL: the carrier\'s flange is round; the clearance check MUST fail')
