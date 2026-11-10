# CONTROL for the prompt line "The balls orbit at the cage's speed, not the shaft's": the Motion Link carries the balls
# round 1 : 1 with the shaft, as if they were fixed to the inner ring. Expected: with the shaft at 90 degrees the balls
# turn 90 degrees, not 0.391489 * 90 = 35.234 degrees.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/bearing_seat/BearingSeat/BearingSeat.py'
OLD = "    li.valueTwo = VS('%.6f deg' % (360.0 * g['k']))\n"
NEW = "    li.valueTwo = VS('360 deg')\n"


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    R0 = fc.rotation(fc.occ('Balls'))
    fc.joint('Shaft turns').jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    a = fc.turn_deg(fc.rotation(fc.occ('Balls')), R0, (0, 0, 1))
    fc.ok(abs(a - 35.234) < 0.001, 'balls %.3f deg (the cage speed gives 35.234)' % a)
    print('CONTROL: the balls carried round with the shaft; the check above MUST fail')
