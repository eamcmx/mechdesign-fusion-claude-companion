# CONTROL for the prompt line "the knee angle is 180 deg - over-centre when the leg is down ... down (knee on the
# locked side, the right of A to B)": the knee is built on the other side, 3 deg short of straight instead of 3 deg
# past it. Expected: nothing collides and the gear still retracts, but the lock is gone: a forward push on the leg
# folds the knee the unlocking way, so a drag load on landing could retract the gear.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/LandingGear/LandingGear.py'
OLD = "    K0 = knee(A, a1, a2, B0, -1)"
NEW = "    K0 = knee(A, a1, a2, B0, 1)"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    leg, upper = fc.occ('Leg'), fc.occ('Upper brace')
    RL, RU = fc.rotation(leg), fc.rotation(upper)
    jl = fc.joint('Leg turns')
    jl.jointMotion.rotationValue = 0.01
    adsk.doEvents()
    s_leg = 1 if -fc.turn_deg(fc.rotation(leg), RL, (0, 1, 0)) > 0 else -1
    jl.jointMotion.rotationValue = math.radians(0.5) * s_leg
    adsk.doEvents()
    dth = -fc.turn_deg(fc.rotation(upper), RU, (0, 1, 0))
    print('  pushed 0.5 deg forward from down, the upper brace turned %.4f deg' % dth)
    fc.ok(dth < 0, 'the knee goes further the locked way when the leg is pushed forward')
    print('CONTROL: the knee short of centre; the lock check above MUST fail')
