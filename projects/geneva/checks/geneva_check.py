# Geneva (Chapter 25) against the acceptance criteria of its prompt, measured on what Fusion built: the driver stepped
# through one turn, 10 deg at a time in the dwell and 1 deg at a time while the pin is in a slot (the slots' mouth
# corners pass the locking disc's relief between the 10 deg steps); at each step the driver's real turn measured on its body, the wheel set by the
# Geneva law worked here (still during the dwell), then Interference and Fusion's minimum distance between them.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/geneva/Geneva/Geneva.py'
N, A = 4, 30.0
C = A / math.sin(math.pi / N)
ALPHA = math.pi / 2 - math.pi / N


def wheel_turn(phi):
    """The wheel's turn (rad) for a driver turn phi (rad) from the built position, the pin at theta = pi."""
    theta = math.pi + phi                                         # from the line of centres
    if theta < 2 * math.pi - ALPHA:
        return 0.0                                                # dwell before the index
    if theta > 2 * math.pi + ALPHA:
        return -2 * math.pi / N                                   # dwell after it
    t = theta - 2 * math.pi
    gamma = lambda x: math.atan2(A * math.sin(x), A * math.cos(x) - C)
    g0, g = gamma(-ALPHA), gamma(t)
    return ((g - g0 + math.pi) % (2 * math.pi)) - math.pi


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    g = ns['geneva'](N, A, 8.0)
    fc.ok(abs(g['C'] - C) < 1e-9 and abs(g['b'] - 30.0) < 1e-9 and abs(g['peak'] - A / (C - A)) < 1e-12,
          'C %.3f, b %.3f, indexing %.1f deg, dwell %.1f deg, peak speed ratio %.3f'
          % (g['C'], g['b'], math.degrees(g['index']), math.degrees(g['dwell']), g['peak']))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Driver', 'Ground', 'Wheel'], 'three components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 2 and root.motionLinks.count == 0,
          'two revolute As-built Joints, no Motion Link')
    jd, jw = fc.joint('Driver turns'), fc.joint('Wheel turns')
    Rd, Rw = fc.rotation(fc.occ('Driver')), fc.rotation(fc.occ('Wheel'))
    mm = adsk.core.Application.get().measureManager
    hits, gaps = [], []
    lo_i, hi_i = 180.0 - math.degrees(ALPHA), 180.0 + math.degrees(ALPHA)          # the index, in the driver's turn
    steps = sorted(set([10.0 * k for k in range(1, 37)] + [float(d) for d in range(int(lo_i) - 5, int(hi_i) + 6)]))
    for ang in steps:
        jd.jointMotion.rotationValue = math.radians(ang)
        adsk.doEvents()
        turned = math.radians(fc.turn_deg(fc.rotation(fc.occ('Driver')), Rd, (0, 0, 1)) % 360.0)
        if ang >= 360.0:
            turned = 2 * math.pi
        jw.jointMotion.rotationValue = wheel_turn(turned)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Driver'), fc.occ('Wheel')])
        if hit:
            hits.append((ang, hit))
        gaps.append((ang, mm.measureMinimumDistance(fc.occ('Driver').bRepBodies.item(0),
                                                       fc.occ('Wheel').bRepBodies.item(0)).value * 10))
    tw = fc.turn_deg(fc.rotation(fc.occ('Wheel')), Rw, (0, 0, 1))
    print('  clearance (mm) at every 30 deg: ' + ', '.join('%d: %.3f' % (a, d) for a, d in gaps if a % 30 == 0))
    fc.ok(not hits, 'no interference at the %d steps %s' % (len(steps), hits or ''))
    lo = min(d for _, d in gaps)
    fc.ok(lo >= 0.15, 'at least 0.15 mm between driver and wheel at every step (smallest %.3f mm)' % lo)
    fc.ok(abs(tw + 90) < 1e-3, 'after one driver turn the wheel has turned %.3f deg' % tw)
    for k in range(35, -1, -1):                                   # back in steps under 180 deg
        jd.jointMotion.rotationValue = math.radians(10.0 * k)
        adsk.doEvents()
    jw.jointMotion.rotationValue = 0
    fc.refused(ns, 'from 3 to 12', n=2)
    fc.refused(ns, 'too large', dp=12.0)
    fc.refused(ns, 'fall apart', n=3)
    fc.verdict()
