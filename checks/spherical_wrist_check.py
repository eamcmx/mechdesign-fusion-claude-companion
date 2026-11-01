# SphericalWrist (Chapter 35) against the acceptance criteria of its prompt, measured on what Fusion built: targets
# given as yaw-pitch-roll (another convention than the wrist's), the joints set from the script's inverse for each
# solution, the flange's orientation measured against the target and against this check's own product of the angles
# Fusion holds; the singularity shown on the bodies; a path that passes 0.5 deg beside it; no interference at the
# pitch limits; four refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/SphericalWrist/SphericalWrist.py'
PMAX = 110.0
TARGETS = [(40, -30, 15), (-70, 20, 100), (10, 80, -45), (100, 0, 30)]      # yaw, pitch, roll (deg)


def R_axis(axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return {'x': [[1, 0, 0], [0, c, -s], [0, s, c]], 'y': [[c, 0, s], [0, 1, 0], [-s, 0, c]],
            'z': [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]


def mm(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def angle_between(A, B):
    """The angle (deg) of the rotation A B^T that takes B to A (atan2, exact for small angles too)."""
    D = [[sum(A[i][k] * B[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
    return math.degrees(math.atan2(math.sqrt(sum(x * x for x in w)) / 2, (D[0][0] + D[1][1] + D[2][2] - 1) / 2))


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = [o.component.name for o in root.occurrences]
    fc.ok(names == ['Forearm', 'Roll housing', 'Pitch housing', 'Flange'], '4 components: %s' % names)
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 3, 'three revolute As-built Joints')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))
    js = [j for j in root.asBuiltJoints]
    parts = [fc.occ('Roll housing'), fc.occ('Pitch housing'), fc.occ('Flange')]
    R0 = [fc.rotation(o) for o in parts]
    axes = [(1, 0, 0), (0, 1, 0), (1, 0, 0)]
    signs = []
    for j, o, r0, ax in zip(js, parts, R0, axes):                         # each joint's sign, measured here
        j.jointMotion.rotationValue = 0.1
        adsk.doEvents()
        signs.append(1 if fc.turn_deg(fc.rotation(o), r0, ax) > 0 else -1)
        j.jointMotion.rotationValue = 0.0
        adsk.doEvents()
    print('  joint signs measured: %s' % signs)

    def set_angles(q):
        for j, s, v in zip(js, signs, q):
            j.jointMotion.rotationValue = math.radians(s * v)
        adsk.doEvents()
        held = [s * math.degrees(j.jointMotion.rotationValue) for j, s in zip(js, signs)]
        Rm = [[sum(fc.rotation(parts[2])[i][k] * R0[2][j][k] for k in range(3)) for j in range(3)] for i in range(3)]
        return held, Rm

    worst_fk, worst_t, clashes, n = 0.0, 0.0, [], 0
    for yaw, pit, rol in TARGETS:
        T = mm(mm(R_axis('z', yaw), R_axis('y', pit)), R_axis('x', rol))
        sols = ns['ik'](T, PMAX)
        fc.ok(len(sols) == 2, 'yaw %g, pitch %g, roll %g: two solutions %s'
              % (yaw, pit, rol, ['(%.3f, %.3f, %.3f)' % q for q in sols]))
        for q in sols:
            held, Rm = set_angles(q)
            own = mm(mm(R_axis('x', held[0]), R_axis('y', held[1])), R_axis('x', held[2]))
            worst_fk = max(worst_fk, angle_between(Rm, own))
            worst_t = max(worst_t, angle_between(Rm, T))
            n += 1
            hit = fc.interference()
            if hit:
                clashes.append((q, hit))
    fc.ok(worst_fk < 1e-6, 'at all %d poses the flange is turned as the product of the angles held (worst %.1e deg)'
          % (n, worst_fk))
    fc.ok(worst_t <= 0.15 + 1e-9, 'and within the rounding of three angles of the target (worst %.4f deg of 0.15)'
          % worst_t)
    fc.ok(not clashes, 'no interference at any pose %s' % (clashes or ''))
    T = mm(R_axis('z', 120), R_axis('y', 10))
    fc.ok(ns['ik'](T, PMAX) == [], 'a tool %.1f deg from the forearm\'s axis is out of reach'
          % math.degrees(math.acos(T[0][0])))

    _, Ra = set_angles((30.0, 0.0, -30.0))
    _, Rb = set_angles((0.0, 0.0, 0.0))
    fc.ok(angle_between(Ra, Rb) < 1e-6, 'at J5 = 0, J4 = 30 with J6 = -30 turns the flange %.1e deg from J4 = J6 = 0: '
          'the singularity' % angle_between(Ra, Rb))
    one = ns['ik'](Rb, PMAX, 30.0)
    fc.ok(len(one) == 1 and abs(one[0][0] - 30) < 1e-9 and abs(one[0][2] + 30) < 1e-9,
          'there the inverse keeps J4 where it was and gives one solution %s' % one)

    prev, rows, worst_axis = (0.0, 0.0, 0.0), [], 0.0
    roll_h = parts[0]
    for k in range(41):
        v = -10.0 + 0.5 * k
        T = mm(R_axis('z', 0.5), R_axis('y', v))
        sols = ns['ik'](T, PMAX, prev[0])
        q = min(sols, key=lambda s: sum(abs((a - b + 180) % 360 - 180) for a, b in zip(s, prev)))
        held, Rm = set_angles(q)
        worst_axis = max(worst_axis, math.degrees(math.acos(max(-1.0, min(1.0, sum(Rm[i][0] * T[i][0]
                                                                                   for i in range(3)))))))
        rows.append((v, fc.turn_deg(fc.rotation(roll_h), R0[0], (1, 0, 0)), held[1]))
        prev = q
    steps = [abs((rows[i + 1][1] - rows[i][1] + 180) % 360 - 180) for i in range(40)]
    total = sum(steps)
    k = steps.index(max(steps))
    print('  path: the tool axis from 10 deg below to 10 deg above the forearm\'s axis, 0.5 deg beside it, in 0.5 deg '
          'steps')
    print('  J4 measured: %s' % ', '.join('%.1f' % r[1] for r in rows[16:25]))
    fc.ok(worst_axis <= 0.15 + 1e-9 and total > 170 and max(steps) > 40,
          'the tool axis followed within %.4f deg; J4 turned %.1f deg in all, %.1f deg in the one step from %g to %g deg'
          % (worst_axis, total, max(steps), rows[k][0], rows[k + 1][0]))
    set_angles((0.0, 0.0, 0.0))

    clashes = []
    for p in (PMAX, -PMAX):
        for r6 in (0.0, 90.0, 180.0, -90.0):
            set_angles((0.0, p, r6))
            hit = fc.interference()
            if hit:
                clashes.append((p, r6, hit))
    fc.ok(not clashes, 'no interference at the pitch limits with J6 at 0, 90, 180 and -90 deg %s' % (clashes or ''))
    set_angles((0.0, 0.0, 0.0))
    fc.refused(ns, 'beyond 110 deg', pmax=115.0)
    fc.refused(ns, 'below 90 deg', pmax=80.0)
    fc.refused(ns, 'at least 20 mm', tool=10.0)
    fc.verdict()
