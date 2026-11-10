# CONTROL for the prompt line "a turn about +x moves the rack towards +y, which steers the car to the right": the
# Motion Link is set the other way. Expected: nothing collides and every knuckle still sits where the loop puts it for
# the rack's position, but the rack goes the wrong way: turning the steering wheel to the right steers the car left.
# Only the check of the rack's travel against the pinion's turn can catch it.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/extras/SteeringRack/SteeringRack.py'
OLD = "li.isReversed = sp * sr < 0                                    # a turn about +x moves the rack towards +y"
NEW = "li.isReversed = sp * sr > 0"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    root = fc.design().rootComponent
    jp = fc.joint('Pinion turns')
    rack, pin, kr = fc.occ('Rack'), fc.occ('Pinion'), fc.occ('Knuckle R')
    y0, R0, RR = fc.position_mm(rack)[1], fc.rotation(pin), fc.rotation(kr)
    hits, moves = [], []
    for k in range(1, 4):
        jp.jointMotion.rotationValue = math.radians(36.0 * k)
        adsk.doEvents()
        turn = fc.turn_deg(fc.rotation(pin), R0, (1, 0, 0))
        moves.append((round(turn, 1), round(fc.position_mm(rack)[1] - y0, 3),
                      round(fc.turn_deg(fc.rotation(kr), RR, (0, 0, 1)), 3)))
        hits += fc.interference()
    print('  (pinion about +x, rack along y, right knuckle about z): %s; overlaps: %s' % (moves, hits))
    fc.ok(not hits, 'no interference')
    fc.ok(all((m[0] > 0) == (m[1] > 0) for m in moves), 'a turn about +x moves the rack towards +y (a right turn)')
    print('CONTROL: the pinion linked the other way; only the direction check can catch it')
