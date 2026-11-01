# RackPinion (Chapter 15) against the acceptance criteria of its prompt, measured on what Fusion built: the numbers
# recomputed independently; the pinion driven and the rack's travel read from its joint AND its body; no tooth
# collision while the pinion turns through one tooth pitch.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/RackPinion/RackPinion.py'


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    m, z, al = 2.0, 20, math.radians(20)
    r = m * z / 2
    rb, ra = r * math.cos(al), r + m
    want = dict(r=r, p=math.pi * m, per_turn=math.pi * m * z, per_deg=r * math.pi / 180,
                eps=(math.sqrt(ra * ra - rb * rb) - r * math.sin(al) + m / math.sin(al)) / (math.pi * m * math.cos(al)),
                limit=(16 / 2 - 2) * math.pi * m)
    k = ns['rack_pinion'](m, z, 16, 20.0)
    bad = [x for x in want if abs(k[x] - want[x]) > 1e-9]
    fc.ok(not bad, 'r %.2f, p %.3f, travel per turn %.3f, contact ratio %.3f, limits %.3f %s'
          % (k['r'], k['p'], k['per_turn'], k['eps'], k['limit'], bad or ''))
    fc.ok((round(k['per_turn'], 3), round(k['eps'], 3), round(k['limit'], 3)) == (125.664, 1.769, 37.699),
          'the rounded values of the prompt')
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Base plate', 'Pinion', 'Rack', 'Shaft'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion', 'RigidJointMotion', 'SliderJointMotion'],
          'As-built Joints: rigid, revolute, slider %s' % kinds)
    fc.ok(root.motionLinks.count == 1, 'one Motion Link')
    jp, jr = fc.joint('Pinion turns'), fc.joint('Rack slides')
    lim = jr.jointMotion.slideLimits
    fc.ok(abs(lim.minimumValue * 10 + want['limit']) < 1e-6 and abs(lim.maximumValue * 10 - want['limit']) < 1e-6,
          'the rack limited to plus and minus %.3f mm' % (lim.maximumValue * 10))
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    x0 = fc.position_mm(fc.occ('Rack'))[0]
    jp.jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    v = jr.jointMotion.slideValue * 10
    x1 = fc.position_mm(fc.occ('Rack'))[0] - x0
    fc.ok(abs(v - 90 * want['per_deg']) < 1e-4 and abs(x1 - 90 * want['per_deg']) < 1e-4,
          'pinion driven 90 deg: the rack joint %.4f mm, its body moved %.4f mm (want +%.4f)' % (v, x1, 90 * want['per_deg']))
    worst = []
    for i in range(13):
        ang = i * 360.0 / z / 12
        jp.jointMotion.rotationValue = math.radians(ang)
        adsk.doEvents()
        hit = fc.interference([fc.occ('Pinion'), fc.occ('Rack')])
        if hit:
            worst.append((round(ang, 2), hit))
    fc.ok(not worst, 'no tooth collision while the pinion turns 0 to 18 deg in 13 positions %s' % (worst or ''))
    jp.jointMotion.rotationValue = 0
    adsk.doEvents()
    fc.refused(ns, 'undercut', z=12)
    fc.refused(ns, 'cannot travel', n_r=6)
    fc.verdict()
