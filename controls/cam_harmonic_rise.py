# CONTROL for the prompt line "cycloidal rise s = h (u - sin(2 pi u) / (2 pi))": the cam is built for a harmonic rise
# h (1 - cos(pi u)) / 2 instead, while the follower is still set to the cycloidal lift. Expected: during the rise the
# roller cuts into the cam or leaves it; at the dwells and the return the gap is still 0.05 mm.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/CamFollower/CamFollower.py'
OLD = "            u = t / b1\n            return h * (u - math.sin(2 * math.pi * u) / (2 * math.pi))\n"
NEW = "            u = t / b1\n            return h * (1 - math.cos(math.pi * u)) / 2\n"


def cycloidal(t):
    if t < 120:
        u = t / 120.0
        return 10.0 * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    return None


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    jc, jf = fc.joint('Cam turns'), fc.joint('Follower slides')
    mm = adsk.core.Application.get().measureManager
    rows = []
    for k in range(0, 13):
        jc.jointMotion.rotationValue = math.radians(10.0 * k)
        adsk.doEvents()
        jf.jointMotion.slideValue = cycloidal(10.0 * k) * 0.1 if k < 12 else 1.0
        adsk.doEvents()
        hit = fc.interference([fc.occ('Cam'), fc.occ('Follower')])
        gap = mm.measureMinimumDistance(fc.occ('Cam').bRepBodies.item(0), fc.occ('Follower').bRepBodies.item(0)).value * 10
        rows.append((10 * k, round(hit[0][2], 3) if hit else 0.0, round(gap, 4)))
    print('  (cam deg, overlap mm^3, gap mm) through the rise: %s' % rows)
    fc.ok(all(r[1] == 0 and abs(r[2] - 0.05) < 0.002 for r in rows), 'no interference and a 0.05 mm gap through the rise')
    print('CONTROL: a cam for a harmonic rise run on a cycloidal one; the check MUST fail')
