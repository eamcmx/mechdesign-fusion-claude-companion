# JointDrive (Chapter 34) against the acceptance criteria of its prompt, measured on what Fusion built: the inertia
# of each moving part about J1 worked out by hand (plates with round ends, cylinders, the parallel-axis theorem) from
# the densities of the materials Fusion assigned, stretched and folded, against Fusion's mass properties; the move,
# the ratios and the torques recomputed independently; the arm turning -1/n of the motor shaft, measured on the bodies.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/extras/JointDrive/JointDrive.py'
L1, L2, W1, W2, T1, T2, Q2 = 250.0, 200.0, 80.0, 60.0, 40.0, 30.0, 150.0
Z_FLANGE, Z_TOP = 332.0, 642.0                           # the flange and the quill's top, the quill up (mm)


def stadium(L, w):
    """Area and polar moment about its centroid of a plate L between the centres of its round ends, w wide."""
    r, Ar = w / 2, L * w
    Ah, c = math.pi * r * r / 2, 4 * r / (3 * math.pi)
    J0 = Ar * (L * L + w * w) / 12 + Ar * (L / 2) ** 2 + Ah * r * r / 2 + Ah * (r * r / 2 - c * c) + Ah * (L + c) ** 2
    A = Ar + 2 * Ah
    return A, J0 - A * (L / 2) ** 2


def disc(r):
    A = math.pi * r * r
    return A, A * r * r / 2


def parts(rho_al, rho_st, q2):
    """kg m2 about J1 for the arm at J2 = q2 (deg): Arm 1, Arm 2, quill, tool. Densities in kg/mm3."""
    d2 = lambda a: L1 * L1 + a * a + 2 * L1 * a * math.cos(math.radians(q2))   # squared distance from J1
    A, Jc = stadium(L1, W1)
    I1 = rho_al * (T1 * (Jc + A * (L1 / 2) ** 2) + 10.0 * disc(40.0)[1])        # the arm and its hub
    A, Jc = stadium(L2, W2)
    Ah, Jh = disc(12.5)
    I2 = rho_al * T2 * (Jc + A * d2(L2 / 2) - Jh - Ah * d2(L2))
    A, Jc = disc(12.0)
    Iq = rho_st * (Z_TOP - Z_FLANGE - 0.001) * (Jc + A * d2(L2))
    Af, Jf = disc(25.0)
    Ap, Jp = disc(6.0)
    It = rho_st * (10.0 * (Jf + Af * d2(L2)) + 50.0 * (Jp + Ap * d2(L2)))
    return [v * 1e-6 for v in (I1, I2, Iq, It)]


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    th, t, dwell = math.pi / 2, 0.4, 0.2
    w = th / (2 * t / 3)                                  # the area under the speed: w (t/3 + t/3 + t/3 halves)
    alpha = w / (t / 3)                                   # w reached in a third of the move
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = [o.component.name for o in root.occurrences]
    fc.ok(names == ['Base', 'Motor', 'Gearhead', 'Motor shaft', 'Arm 1', 'Arm 2', 'Quill', 'Tool'],
          '8 components: %s' % names)
    mats = {n: fc.occ(n).component.bRepBodies.item(0).material.name for n in ('Arm 1', 'Arm 2', 'Quill', 'Tool')}
    fc.ok(mats == {'Arm 1': 'Aluminum', 'Arm 2': 'Aluminum', 'Quill': 'Steel', 'Tool': 'Steel'}, 'materials %s' % mats)
    rho = {n: fc.occ(n).component.bRepBodies.item(0).physicalProperties.density * 1e-3 for n in mats}   # kg/mm3
    print('  densities %.1f and %.1f kg/m3' % (rho['Arm 1'] * 1e9, rho['Quill'] * 1e9))
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and len(kinds) == 5 and root.motionLinks.count == 1,
          'five As-built Joints, one Motion Link: %s' % kinds)
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    occs = [fc.occ(n) for n in ('Arm 1', 'Arm 2', 'Quill', 'Tool')]
    j2 = fc.joint('J2')
    res = {}
    for q2 in (0.0, Q2):
        j2.jointMotion.rotationValue = math.radians(q2)
        adsk.doEvents()
        meas = [o.physicalProperties.getXYZMomentsOfInertia()[3] * 1e-4 for o in occs]
        hand = parts(rho['Arm 1'], rho['Quill'], q2)
        rows = ', '.join('%s %.6f (%.6f)' % (o.component.name, a, b) for o, a, b in zip(occs, meas, hand))
        fc.ok(all(abs(a - b) < 1e-6 * b for a, b in zip(meas, hand)), 'J2 at %.0f deg, kg m2 measured (by hand): %s'
              % (q2, rows))
        res[q2] = (sum(meas), sum(hand))
    j2.jointMotion.rotationValue = 0.0
    adsk.doEvents()

    J_L = res[0.0][1] + 2.0 * 0.45 ** 2
    eta, Jm = 0.8, 0.3e-4
    n_opt = math.sqrt(J_L / (eta * Jm))
    n_speed = 3000.0 / (w * 60 / (2 * math.pi))
    duty = math.sqrt((2 * t / 3) / (t + dwell))
    rows = [(n, (Jm * n + J_L / (eta * n)) * alpha) for n in (30, 50, 80, 100, 120, 160)]
    ok_rows = [(n, T) for n, T in rows if n <= n_speed and T <= 3.82 and T * duty <= 1.27]
    n, Tm = min(ok_rows, key=lambda r: r[1])
    d = ns['drive'](J_L)
    fc.ok(abs(d['w'] - w) < 1e-9 and abs(d['alpha'] - alpha) < 1e-9 and abs(d['n_opt'] - n_opt) < 1e-6
          and abs(d['n_speed'] - n_speed) < 1e-9 and d['n'] == n and abs(d['T_m'] - Tm) < 1e-9
          and abs(d['T_rms'] - Tm * duty) < 1e-9, 'the move and the drive recomputed independently')
    print('  arm %.5f stretched, %.5f folded; with the payload %.5f kg m2; %.4f rad/s, %.3f rad/s2, %.3f N m at the '
          'joint; best ratio %.1f, speed allows %.2f; chosen %d:1, %.3f N m peak, %.3f N m RMS'
          % (res[0.0][1], res[Q2][1], J_L, w, alpha, J_L * alpha, n_opt, n_speed, n, Tm, Tm * duty))

    js, arm = fc.joint('Motor'), fc.occ('Arm 1')
    R0 = fc.rotation(arm)
    turns = []
    for k in range(1, 5):
        js.jointMotion.rotationValue = math.radians(90.0 * k)
        adsk.doEvents()
        turns.append(fc.turn_deg(fc.rotation(arm), R0, (0, 0, 1)))
    fc.ok(all(abs(a + 90.0 * (k + 1) / n) < 1e-6 for k, a in enumerate(turns)),
          'the motor shaft turned 90 deg four times: the arm %s deg (want -%.1f each)'
          % (', '.join('%.3f' % a for a in turns), 90.0 / n))
    js.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.refused(ns, 'between 0 and 100', eta=120.0)
    fc.refused(ns, 'below every standard ratio', t=0.12)
    fc.refused(ns, 'keeps the motor within', m=20.0)
    fc.verdict()
