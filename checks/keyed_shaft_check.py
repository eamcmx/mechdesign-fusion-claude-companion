# KeyedShaft (Chapter 6) against the acceptance criteria of its prompt, measured on what Fusion built.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/KeyedShaft/KeyedShaft.py'


def bbox_mm(o):
    b = o.component.bRepBodies.item(0).boundingBox
    m = o.transform2
    p0, p1 = b.minPoint.copy(), b.maxPoint.copy()
    p0.transformBy(m)
    p1.transformBy(m)
    return [v * 10 for v in (p0.x, p0.y, p0.z, p1.x, p1.y, p1.z)]


def sizes(D, want):
    """Key b x h, how deep it sits in the shaft (t1), the hub's outside diameter: measured on the bodies."""
    k, h = bbox_mm(fc.occ('Key')), bbox_mm(fc.occ('Hub'))
    b, hh, t1, od = k[3] - k[0], k[4] - k[1], D / 2 - k[1], h[3] - h[0]
    fc.ok(all(abs(a - w) < 1e-3 for a, w in zip((b, hh, t1, od), want)),
          'D = %g: key %.3f x %.3f, %.3f into the shaft, hub OD %.3f mm (want %g x %g, %g, %g)' % ((D, b, hh, t1, od) + tuple(want)))
    fc.ok(k[1] < D / 2 < k[4], 'the key straddles the shaft surface (%.2f < %.2f < %.2f)' % (k[1], D / 2, k[4]))


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['Bearing block', 'Hub', 'Key', 'Shaft'], 'four components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(fc.occ('Bearing block').isGrounded, 'the bearing block is grounded')
    kinds = sorted(type(j.jointMotion).__name__ for j in root.asBuiltJoints)
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion', 'RigidJointMotion', 'RigidJointMotion'],
          'As-built Joints: one revolute, two rigid (%s)' % kinds)
    sizes(20.0, (6, 6, 3.5, 36))
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    R0 = {n: fc.rotation(fc.occ(n)) for n in ('Shaft', 'Key', 'Hub', 'Bearing block')}
    fc.joint('Shaft turns').jointMotion.rotationValue = math.radians(90)
    adsk.doEvents()
    a = {n: fc.turn_deg(fc.rotation(fc.occ(n)), R0[n], (0, 0, 1)) for n in R0}
    fc.ok(all(abs(abs(a[n]) - 90) < 1e-6 for n in ('Shaft', 'Key', 'Hub')) and abs(a['Bearing block']) < 1e-9,
          'shaft joint at 90: shaft %.3f, key %.3f, hub %.3f, block %.3f deg about z' % (a['Shaft'], a['Key'], a['Hub'], a['Bearing block']))
    c = fc.interference()
    fc.ok(not c, 'no interference at 90 degrees %s' % (c or ''))
    fc.joint('Shaft turns').jointMotion.rotationValue = 0
    msg = ns['build'](D=30.0)
    fc.mark()
    print('  D = 30: ' + msg.splitlines()[2])
    sizes(30.0, (8, 7, 4.0, 46))
    msg = ns['build']()
    fc.mark()
    want = ('F = 2T/D = 3000 N', '= 27.78 MPa', '= 66.67 MPa', '= 19.10 MPa')
    fc.ok(all(w in msg for w in want), 'the message: F = 3000 N, tau 27.78, sigma 66.67, tau_shaft 19.10 MPa')
    fc.refused(ns, 'longer than the hub', L_key=30.0)
    fc.refused(ns, 'outside the DIN 6885 table', D=70.0)
    fc.refused(ns, 'crushing stress 222.2 MPa', T=100.0)
    fc.verdict()
