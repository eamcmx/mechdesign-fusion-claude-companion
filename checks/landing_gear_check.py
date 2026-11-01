# LandingGear (Chapter 30) against the acceptance criteria of its prompt, measured on what Fusion built: the leg's
# angle for each drive angle worked out here by intersecting the lower link's circle with the brace point's circle
# about the trunnion, followed from one step to the next; the lock tested by pushing the leg forward from down; no
# interference at any step.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/LandingGear/LandingGear.py'
L, RW, BL, A, A1, OVER = 900.0, 250.0, 400.0, (125.0, 75.0), 300.0, 3.0


def lower_link():
    B0 = (0.0, -BL)
    d0 = math.hypot(A[0] - B0[0], A[1] - B0[1])
    # law of cosines with the knee at 180 - OVER, solved for a2 by Newton
    a2 = d0 - A1
    for _ in range(60):
        f = A1 * A1 + a2 * a2 - 2 * A1 * a2 * math.cos(math.radians(180 - OVER)) - d0 * d0
        a2 -= f / (2 * a2 - 2 * A1 * math.cos(math.radians(180 - OVER)))
    return a2


A2 = lower_link()


def leg_for(theta, prev):
    """The brace point B for the upper link at theta (radians, in the xz plane from +x towards +z): on the circle of
    radius BL about the trunnion and at A2 from the knee, the intersection nearer the previous B."""
    K = (A[0] + A1 * math.cos(theta), A[1] + A1 * math.sin(theta))
    D = math.hypot(*K)
    m = (BL * BL - A2 * A2 + D * D) / (2 * D)
    h = math.sqrt(max(0.0, BL * BL - m * m))
    mx, mz = m * K[0] / D, m * K[1] / D
    c = [(mx - h * K[1] / D, mz + h * K[0] / D), (mx + h * K[1] / D, mz - h * K[0] / D)]
    return min(c, key=lambda q: math.hypot(q[0] - prev[0], q[1] - prev[1]))


def theta_locked(alpha):
    """The upper link's angle with the leg at alpha and the knee on the locked side (the right of A to B)."""
    B = (BL * math.sin(alpha), -BL * math.cos(alpha))
    dx, dz = B[0] - A[0], B[1] - A[1]
    D = math.hypot(dx, dz)
    m = (A1 * A1 - A2 * A2 + D * D) / (2 * D)
    h = math.sqrt(max(0.0, A1 * A1 - m * m))
    K = (A[0] + m * dx / D + h * dz / D, A[1] + m * dz / D - h * dx / D)
    return math.atan2(K[1] - A[1], K[0] - A[0])


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    g = ns['gear']()
    th_down = theta_locked(0.0)
    fc.ok(abs(g['a2'] - A2) < 1e-9 and abs(g['th_down'] - th_down) < 1e-9,
          'a2 and the drive angle down recomputed independently (%.3f mm, %.3f deg)' % (A2, math.degrees(th_down)))
    tq = max(abs(ns['drive_torque'](g, g['al_dc'] + (math.pi / 2 - g['al_dc']) * k / 400)) for k in range(1, 400))
    got = (round(g['a2'], 3), round(-math.degrees(g['al_dc']), 4), round(math.degrees(g['th_down']), 3),
           round(math.degrees(g['th_dc']), 3), round(math.degrees(g['th_up']), 3), round(g['phi_up'], 2),
           round(g['arm'], 3), round(g['F_brace']), round(tq, 2))
    fc.ok(got == (191.332, 0.0902, -105.912, -104.815, 22.815, 66.73, 101.797, 112970, 106.12),
          'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(len(names) == 4, '4 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion'] * 4 and root.motionLinks.count == 0,
          'four revolute As-built Joints, no Motion Link')
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    leg, upper = fc.occ('Leg'), fc.occ('Upper brace')
    RL, RU = fc.rotation(leg), fc.rotation(upper)
    # the lock: push the leg 0.5 deg forward from down; the knee must go further the locked way
    jl, ju = fc.joint('Leg turns'), fc.joint('Upper brace turns')
    jl.jointMotion.rotationValue = 0.01
    adsk.doEvents()
    s_leg = 1 if -fc.turn_deg(fc.rotation(leg), RL, (0, 1, 0)) > 0 else -1     # +value turns the leg forward?
    jl.jointMotion.rotationValue = math.radians(0.5) * s_leg
    adsk.doEvents()
    dth = -fc.turn_deg(fc.rotation(upper), RU, (0, 1, 0))                      # in the xz plane, +x towards +z
    want = math.degrees(theta_locked(math.radians(0.5)) - th_down)
    fc.ok(abs(dth - want) < 1e-3 and dth < 0,
          'pushed 0.5 deg forward from down, the upper brace turned %.4f deg the locked way (worked out %.4f)'
          % (dth, want))
    jl.jointMotion.rotationValue = 0.0
    adsk.doEvents()

    # retraction: the upper brace driven from down through dead centre to up
    ju.jointMotion.rotationValue = 0.01
    adsk.doEvents()
    s_up = 1 if -fc.turn_deg(fc.rotation(upper), RU, (0, 1, 0)) > 0 else -1
    ju.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    th_dc, th_up = g['th_dc'], g['th_up']
    targets = [th_dc] + [th_down + math.radians(3.0) * k for k in range(1, 100)
                         if th_down + math.radians(3.0) * k < th_up] + [th_up]
    targets = sorted(set(targets))
    prev = (0.0, -BL)
    worst, hits, aft, kept = 0.0, [], None, []
    for th in targets:
        ju.jointMotion.rotationValue = (th - th_down) * s_up
        adsk.doEvents()
        # Fusion keeps a revolute joint's angle to 0.1 deg: work from the angle it holds, measured on the body
        th_held = th_down + math.radians(-fc.turn_deg(fc.rotation(upper), RU, (0, 1, 0)))
        kept.append(abs(math.degrees(th_held - th)))
        B = leg_for(th_held, prev)
        prev = B
        want_al = math.degrees(math.atan2(B[0], -B[1]))
        got_al = -fc.turn_deg(fc.rotation(leg), RL, (0, 1, 0))
        worst = max(worst, abs(got_al - want_al))
        if abs(th - th_dc) < 1e-12:
            aft = got_al
        hit = fc.interference()
        if hit:
            hits.append((round(math.degrees(th), 1), hit))
    print('  the drive angle Fusion kept differed from the one asked by up to %.4f deg (it keeps tenths)' % max(kept))
    fc.ok(worst < 1e-3, 'the leg at the angle worked out here for the drive angle held, at all %d steps (worst %.1e deg)'
          % (len(targets), worst))
    fc.ok(aft is not None and abs(aft + 0.0902) < 1e-4,
          'at dead centre the leg stood %.4f deg aft of down: the travel to unlock' % (-aft if aft is not None else 0))
    fc.ok(abs(got_al - 90.0) < 0.05, 'up: the leg turned %.3f deg forward (90 at the drive angle asked, %.3f deg)'
          % (got_al, math.degrees(th_up)))
    fc.ok(not hits, 'nothing interferes at any step %s' % (hits or ''))
    for th in reversed(targets[:-1]):
        ju.jointMotion.rotationValue = (th - th_down) * s_up
        adsk.doEvents()
    ju.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.refused(ns, 'keep it from 1 to 10 deg', over=12.0)
    fc.refused(ns, 'does not fold steadily', Az=-100.0)
    fc.refused(ns, 'folded to 16.98 deg', a1=385.0)
    fc.verdict()
