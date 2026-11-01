# CONTROL for the prompt line "A right-hand screw: a turn about +x moves the nut towards -x": the Motion Link is set
# the other way, as for a left-hand screw. Expected: nothing collides and the flap still follows the nut exactly; but
# the motor, whose sense is fixed by the flap lever, now drives the nut the wrong way: asked for flap down, the flap
# would go up against its stop. Only the check of the screw's hand catches it.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/FlapActuator/FlapActuator.py'
OLD = "li.isReversed = s_s * s_n > 0                                  # right-hand: a turn about +x moves the nut to -x"
NEW = "li.isReversed = s_s * s_n < 0"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    js = fc.joint('Screw turns')
    nut, scr = fc.occ('Nut'), fc.occ('Screw')
    x0, RS = fc.position_mm(nut)[0], fc.rotation(scr)
    js.jointMotion.rotationValue = math.radians(36.0)
    adsk.doEvents()
    turned = fc.turn_deg(fc.rotation(scr), RS, (1, 0, 0))
    moved = fc.position_mm(nut)[0] - x0
    print('  the screw turned %.1f deg about +x; the nut moved %.3f mm along x' % (turned, moved))
    fc.ok((turned > 0) != (moved > 0), 'a right-hand screw: a turn about +x moves the nut towards -x')
    print('CONTROL: the screw linked as a left-hand one; the hand check above MUST fail')
