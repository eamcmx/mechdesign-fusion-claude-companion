# Palletizer (Chapter 36) against the acceptance criteria of its prompt, measured on what Fusion built: every box of
# the pallet reached with the joints set from the script's inverse, the tool where this check's own forward kinematics
# puts it for the angles Fusion holds, and the load level; the motors' gravity torques measured by virtual work (the
# bodies' potential energy as each motor turns 1 deg either way) against the sum of weights times levers.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/Palletizer/Palletizer.py'
A, L2, L3, E, H_TCP = (150.0, 700.0), 900.0, 1000.0, 250.0, 220.0
A1, OFF2 = (-30.0, 700.0), (0.0, 150.0)
G = 9.80665
MOVING = ['Upper arm', 'Forearm', 'Crank', 'Rod 3', 'Elbow plate', 'Rod 1', 'Rod 2', 'Bracket', 'Tool']


def wrist(t2, t3):
    B = (A[0] + L2 * math.cos(math.radians(t2)), A[1] + L2 * math.sin(math.radians(t2)))
    return B, (B[0] + L3 * math.cos(math.radians(t3)), B[1] + L3 * math.sin(math.radians(t3)))


def apply(T, p):
    """A point p (mm) moved by an occurrence's transform."""
    return tuple(sum(T.getCell(i, j) * p[j] for j in range(3)) + T.getCell(i, 3) * 10 for i in range(3))


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = [o.component.name for o in root.occurrences]
    fc.ok(names == ['Base', 'Turntable'] + MOVING, '11 components: %s' % names)
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 13, '13 revolute As-built Joints, three loops')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    occ = {n: fc.occ(n) for n in ['Turntable'] + MOVING}
    R0 = {n: fc.rotation(o) for n, o in occ.items()}
    js = {n: fc.joint(n) for n in ('J1 turntable', 'J2 upper arm', 'Elbow', 'J4 tool')}
    axes = {'J1 turntable': ((0, 0, 1), 'Turntable'), 'J2 upper arm': ((0, -1, 0), 'Upper arm'),
            'Elbow': ((0, -1, 0), 'Forearm'), 'J4 tool': ((0, 0, 1), 'Tool')}
    sign = {}
    for n, (ax, part) in axes.items():                 # each joint's sign, measured: + turns x towards z (or y)
        js[n].jointMotion.rotationValue = 0.1
        adsk.doEvents()
        t = fc.turn_deg(fc.rotation(occ[part]), R0[part], ax)
        if n == 'Elbow':                               # the elbow's turn is the forearm's relative to the upper arm
            t -= fc.turn_deg(fc.rotation(occ['Upper arm']), R0['Upper arm'], ax)
        sign[n] = 1 if t > 0 else -1
        js[n].jointMotion.rotationValue = 0.0
        adsk.doEvents()
    print('  signs: %s' % sign)

    def pose(q1, t2, t3):
        """The two drives close one loop, and Fusion shares a move of either between them. Setting the shoulder
        turns the loop as one body and keeps the elbow's angle, so: the elbow's angle first, then the shoulder."""
        js['J1 turntable'].jointMotion.rotationValue = math.radians(sign['J1 turntable'] * q1)
        js['Elbow'].jointMotion.rotationValue = math.radians(sign['Elbow'] * (t3 - t2 + 90.0))
        adsk.doEvents()
        js['J2 upper arm'].jointMotion.rotationValue = math.radians(sign['J2 upper arm'] * (t2 - 90.0))
        adsk.doEvents()
        h1 = fc.turn_deg(fc.rotation(occ['Turntable']), R0['Turntable'], (0, 0, 1))
        Rtt = fc.rotation(occ['Turntable'])
        rel = lambda n: [[sum(fc.rotation(occ[n])[i][k] * R0[n][j][k] for k in range(3)) for j in range(3)]
                         for i in range(3)]
        # the arm's angles in its own plane: each body's turn, with the turntable's taken out
        unturn = lambda Rn: [[sum(Rtt[k][i] * Rn[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
        I3 = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        h2 = 90.0 + fc.turn_deg(unturn(rel('Upper arm')), I3, (0, -1, 0))
        h3 = fc.turn_deg(unturn(rel('Forearm')), I3, (0, -1, 0))
        Rb = unturn(rel('Bracket'))
        tilt = math.degrees(math.atan2(math.hypot(Rb[0][2], Rb[1][2]), Rb[2][2]))
        W0 = wrist(90.0, 0.0)[1]
        P = apply(occ['Tool'].transform2, (W0[0], 0.0, W0[1] - H_TCP))
        return (h1, h2, h3), P, tilt

    rows, worst_fk, worst_t, worst_tilt, clashes = [], 0.0, 0.0, 0.0, []
    for k, (x, y, z) in enumerate(ns['pattern'](1200.0, 3)):
        q, why = ns['ik'](x, y, z)
        held, P, tilt = pose(*q)
        B, W = wrist(held[1], held[2])
        r = W[0]
        own = (r * math.cos(math.radians(held[0])), r * math.sin(math.radians(held[0])), W[1] - H_TCP)
        bound = (math.hypot(x, y) + L2 + L3) * math.radians(0.05)
        worst_fk = max(worst_fk, math.dist(P, own))
        worst_t = max(worst_t, math.dist(P, (x, y, z)) / bound)
        worst_tilt = max(worst_tilt, tilt)
        rows.append('  box %2d (%5.0f %5.0f %4.0f): asked %7.3f %7.3f %8.3f  held %6.1f %6.1f %6.1f  off %.3f mm'
                    % (k + 1, x, y, z, q[0], q[1], q[2], held[0], held[1], held[2], math.dist(P, (x, y, z))))
        hit = fc.interference()
        if hit:
            clashes.append((k + 1, hit))
    print('\n'.join(rows[:4] + ['  ...'] + rows[-4:]))
    fc.ok(worst_tilt < 1e-6, 'the load level at all 24 boxes (worst tilt %.1e deg)' % worst_tilt)
    fc.ok(worst_fk < 1e-6, 'the tool where forward kinematics puts it for the angles held (worst %.1e mm)' % worst_fk)
    fc.ok(worst_t <= 1.0, 'and within the rounding of each box\'s place point (worst %.0f %% of its bound)'
          % (100 * worst_t))
    fc.ok(not clashes, 'no interference at any box %s' % (clashes or ''))

    def energy():
        """The moving bodies' potential energy (J), from their masses and centres of mass."""
        return sum(occ[n].physicalProperties.mass * G * occ[n].physicalProperties.centerOfMass.z / 100.0 for n in MOVING)

    for t2, t3 in ((90.0, 0.0), (50.0, -30.0)):
        held, _, _ = pose(0.0, t2, t3)
        m = {n: occ[n].physicalProperties.mass for n in MOVING}
        cx = {n: occ[n].physicalProperties.centerOfMass.x * 10 for n in MOVING}     # mm
        B, W = wrist(t2, t3)
        C = (A[0] + E * math.cos(math.radians(t3 + 180)), A[1] + E * math.sin(math.radians(t3 + 180)))
        B2 = (B[0] + OFF2[0], B[1] + OFF2[1])
        lev2 = (m['Upper arm'] * (cx['Upper arm'] - A[0]) + m['Rod 3'] * (cx['Rod 3'] - C[0])
                + m['Rod 1'] * (cx['Rod 1'] - A1[0])
                + sum(m[n] for n in ('Forearm', 'Elbow plate', 'Rod 2', 'Bracket', 'Tool')) * (B[0] - A[0]))
        lev3 = (m['Crank'] * (cx['Crank'] - A[0]) + m['Rod 3'] * (C[0] - A[0]) + m['Forearm'] * (cx['Forearm'] - B[0])
                + m['Rod 2'] * (cx['Rod 2'] - B2[0]) + (m['Bracket'] + m['Tool']) * (W[0] - B[0]))
        T2h, T3h = G * lev2 / 1000.0, G * lev3 / 1000.0
        vw = []
        for which, base in (('J2', (t2, t3)), ('J3', (t2, t3))):
            U = []
            for d in (-1.0, 1.0):
                pose(0.0, base[0] + (d if which == 'J2' else 0.0), base[1] + (d if which == 'J3' else 0.0))
                U.append(energy())
            vw.append((U[1] - U[0]) / math.radians(2.0))
        pose(0.0, t2, t3)
        print('  upper arm %.0f, forearm %.0f deg: motor 2 %.3f N m by virtual work, %.3f N m by levers; motor 3 %.3f '
              'and %.3f N m' % (t2, t3, vw[0], T2h, vw[1], T3h))
        tol = lambda T: 1e-3 * max(abs(T), 10.0)            # 0.1 %, or 0.01 N m where the torque is nil
        fc.ok(abs(vw[0] - T2h) < tol(T2h) and abs(vw[1] - T3h) < tol(T3h),
              'the gravity torques by virtual work and by levers agree at %.0f, %.0f deg' % (t2, t3))
    print('  masses kg: %s' % ', '.join('%s %.2f' % (n, occ[n].physicalProperties.mass) for n in MOVING))
    pose(0.0, 90.0, 0.0)
    fc.refused(ns, 'from 1 to 5', layers=6)
    fc.refused(ns, 'the upper arm at 39.0 deg', xc=1300.0)
    fc.refused(ns, 'the forearm at -74.7 deg', xc=800.0)
    fc.verdict()
