# CONTROL for the prompt line "the camshaft at (60, 0) turning the other way at half the speed": the Motion Link is
# set the other way, so the camshaft turns WITH the crank, as it would through a chain or a belt. Expected: at rest
# nothing is wrong; as the crank turns, the valves (set by the lift law) and the lobes part company: the rollers
# either sink into their lobes or stand off them, and the measured gap is no longer 0.05 mm.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/SingleCylinderEngine/SingleCylinderEngine.py'
OLD = "li.isReversed = sc * sk_ > 0                                   # the camshaft turns the other way"
NEW = "li.isReversed = sc * sk_ < 0"
H, R, L = 6.0, 27.0, 90.0
EVENTS = {'in': (-10.0, 240.0), 'ex': (490.0, 240.0)}


def lift(which, theta):
    o, D = EVENTS[which]
    t = ((theta - o) / 2.0) % 360.0
    b = D / 4.0
    if t < b:
        u = t / b
        return H * (u - math.sin(2 * math.pi * u) / (2 * math.pi))
    if t < 2 * b:
        u = (t - b) / b
        return H * (1 - (u - math.sin(2 * math.pi * u) / (2 * math.pi)))
    return 0.0


def run(_context):
    ns = fc.load(PATH, [(OLD, NEW)])
    ns['build']()
    fc.mark()
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    jc = fc.joint('Crank turns')
    jv = {'in': fc.joint('Inlet valve slides'), 'ex': fc.joint('Exhaust valve slides')}
    occ_v = {'in': fc.occ('Inlet valve'), 'ex': fc.occ('Exhaust valve')}
    crank, cam = fc.occ('Crankshaft'), fc.occ('Camshaft')
    Rc0 = fc.rotation(crank)
    jc.jointMotion.rotationValue = 0.1
    adsk.doEvents()
    sc = 1 if fc.turn_deg(fc.rotation(crank), Rc0, (0, 0, 1)) > 0 else -1
    jc.jointMotion.rotationValue = 0.0
    sv = {}
    for w in ('in', 'ex'):
        jv[w].jointMotion.slideValue = 0.1
        adsk.doEvents()
        sv[w] = 1 if fc.position_mm(occ_v[w])[1] > 0 else -1
        jv[w].jointMotion.slideValue = 0.0
    mm = adsk.core.Application.get().measureManager
    gaps, hits = [], []
    for i in range(1, 25):
        d = 30.0 * i
        jc.jointMotion.rotationValue = math.radians(d) * sc
        for w in ('in', 'ex'):
            jv[w].jointMotion.slideValue = lift(w, 360.0 + d) * 0.1 * sv[w]
        adsk.doEvents()
        for w in ('in', 'ex'):
            gaps.append(mm.measureMinimumDistance(cam.bRepBodies.item(0), occ_v[w].bRepBodies.item(0)).value * 10)
        hit = fc.interference([cam, occ_v['in'], occ_v['ex']])
        if hit:
            hits.append((360 + d, hit))
    print('  roller to lobe from %.4f to %.4f mm; %d steps with overlaps, the first: %s'
          % (min(gaps), max(gaps), len(hits), hits[:2]))
    fc.ok(0.045 <= min(gaps) and max(gaps) <= 0.055, 'each roller 0.05 mm from its lobe at every step')
    fc.ok(not hits, 'nothing interferes at any of the 24 steps')
    print('CONTROL: the camshaft turning with the crank; the motion checks above MUST fail')
