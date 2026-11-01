# ScaraArm (Chapter 33) against the acceptance criteria of its prompt, measured on what Fusion built: the numbers
# recomputed independently (the floor area by counting a grid of points); the joints set from the script's inverse
# kinematics for six targets, both elbows; the angles Fusion holds (it keeps tenths of a degree) read back from the
# bodies; the tool where this check's own forward kinematics puts it for those angles, and within their rounding of
# the target; a value beyond a joint's limit ignored; unreachable points found unreachable; four refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/ScaraArm/ScaraArm.py'
L1, L2, H, STROKE, Q1, Q2 = 250.0, 200.0, 400.0, 150.0, 132.0, 150.0
Z_FLANGE = H + 1 + 40 + 1 - 110                          # the flange 110 mm below the second arm, the quill up
TARGETS = [(300, 200, 300, 0), (-150, 350, 250, 90), (420, -100, 200, -45), (130, 0, 332, 0),
           (0, -400, 182, 30), (-300, -300, 260, 180)]


def reachable(x, y):
    r2 = x * x + y * y
    c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1:
        return False
    for sg in (1, -1):
        q2 = sg * math.acos(c2)
        q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        q1 = (q1 + math.pi) % (2 * math.pi) - math.pi
        if abs(math.degrees(q1)) <= Q1 and abs(math.degrees(q2)) <= Q2:
            return True
    return False


def flange(q1, q12, d3):
    """Where the flange must be for the angles of the two arms (deg, world) and the quill d3 down."""
    a, b = math.radians(q1), math.radians(q12)
    return (L1 * math.cos(a) + L2 * math.cos(b), L1 * math.sin(a) + L2 * math.sin(b), Z_FLANGE - d3)


def workspace(s):
    """A 1 mm plate on the floor, the shape of the floor the tool covers: the reach between the J1 limits, the arcs
    the tool traces with J1 at its limits up to where they cross behind the robot, less the inner radius. Returns the
    occurrence, the area Fusion measures (mm^2) and how far behind the axis the arcs cross (mm)."""
    P3, c = adsk.core.Point3D.create, 0.1
    L1, L2, h = s['L1'], s['L2'], math.radians(s['q1max'])
    cb = math.cos(math.pi - h)                    # behind the robot the arcs at +-beta just close the circle
    r_x = L1 * cb + math.sqrt((L1 * cb) ** 2 - L1 * L1 + L2 * L2)
    q2_x = math.acos((r_x * r_x - L1 * L1 - L2 * L2) / (2 * L1 * L2))
    root = fc.design().rootComponent
    occ = root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = 'Workspace'
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    arcs = sk.sketchCurves.sketchArcs
    arcs.addByCenterStartSweep(P3(0, 0, 0), P3(s['reach'] * math.cos(h) * c, -s['reach'] * math.sin(h) * c, 0), 2 * h)
    for sg in (1, -1):
        arcs.addByCenterStartSweep(P3(L1 * math.cos(h) * c, sg * L1 * math.sin(h) * c, 0),
                                   P3(s['reach'] * math.cos(h) * c, sg * s['reach'] * math.sin(h) * c, 0), sg * q2_x)
    sk.sketchCurves.sketchCircles.addByCenterRadius(P3(0, 0, 0), s['inner'] * c)
    prof = [p for p in sk.profiles if p.profileLoops.count == 2][0]
    area = prof.areaProperties().area * 100.0
    ext = occ.component.features.extrudeFeatures
    ext.addSimple(prof, adsk.core.ValueInput.createByReal(0.1), adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    sk.isVisible = False
    return occ, area, r_x


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    s = ns['scara']()
    inner = math.sqrt(L1 ** 2 + L2 ** 2 - 2 * L1 * L2 * math.cos(math.radians(180 - Q2)))   # law of cosines
    h = 2.0
    grid = sum(1 for i in range(-226, 227) for j in range(-226, 227) if reachable(i * h, j * h)) * h * h
    want = dict(reach=L1 + L2, inner=inner, z_flange=Z_FLANGE, res=(L1 + L2) * math.pi / 1800,
                T_payload=2.0 * 0.45 ** 2 * 10.0, c_column=inner - 60 - 25, c_arm=L2 / 2 - 40 - 12,
                c_base=Z_FLANGE - 60 - STROKE - 40)
    bad = [k for k in want if abs(s[k] - want[k]) > 1e-6]
    fc.ok(not bad, 'all values recomputed independently %s' % (bad or ''))
    fc.ok(abs(s['area'] - grid) < 1e-3 * grid, 'floor area %.0f cm2, a %.0f mm grid counts %.0f cm2'
          % (s['area'] / 100, h, grid / 100))
    got = (round(s['reach'], 1), round(s['inner'], 3), round(s['area'] / 100), round(s['res'], 3),
           round(s['T_payload'], 3), round(s['c_column'], 3), round(s['c_arm'], 3), round(s['c_base'], 3))
    fc.ok(got == (450.0, 126.085, 5426, 0.785, 4.05, 41.085, 48.0, 82.0), 'the rounded values of the prompt %s' % (got,))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = [o.component.name for o in root.occurrences]
    fc.ok(names == ['Base', 'Arm 1', 'Arm 2', 'Quill', 'Tool'], '5 components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(root.joints.count == 0 and kinds == ['RevoluteJointMotion', 'RevoluteJointMotion', 'SliderJointMotion',
                                               'RevoluteJointMotion'], 'four As-built Joints: %s' % kinds)
    j1, j2, j3, j4 = [fc.joint(n) for n in ('J1', 'J2', 'J3', 'J4')]
    lims = []
    for j, k in ((j1, 'r'), (j2, 'r'), (j3, 's'), (j4, 'r')):
        lim = j.jointMotion.rotationLimits if k == 'r' else j.jointMotion.slideLimits
        f = math.degrees if k == 'r' else (lambda v: v * 10)
        lims.append((round(f(lim.minimumValue), 6), round(f(lim.maximumValue), 6)))
    fc.ok(lims == [(-Q1, Q1), (-Q2, Q2), (-STROKE, 0.0), (-180.0, 180.0)], 'limits %s' % lims)
    c = fc.interference()
    fc.ok(not c, 'no interference at rest %s' % (c or ''))

    a1, a2, tool = fc.occ('Arm 1'), fc.occ('Arm 2'), fc.occ('Tool')
    R1, R2, RT = fc.rotation(a1), fc.rotation(a2), fc.rotation(tool)
    rows, worst_fk, worst_t, worst_bound, clashes = [], 0.0, 0.0, 0.0, []
    for x, y, z, phi in TARGETS:
        sols = ns['ik'](s, x, y, z, phi)
        fc.ok(len(sols) >= 1, 'target (%g, %g, %g) at %g deg: %d elbow(s) inside the limits' % (x, y, z, phi, len(sols)))
        for q1, q2, d3, q4 in sols:
            j1.jointMotion.rotationValue = math.radians(q1)
            j2.jointMotion.rotationValue = math.radians(q2)
            j3.jointMotion.slideValue = -d3 / 10.0
            j4.jointMotion.rotationValue = math.radians(q4)
            adsk.doEvents()
            h1 = fc.turn_deg(fc.rotation(a1), R1, (0, 0, 1))                 # the angles held, read off the bodies
            h12 = fc.turn_deg(fc.rotation(a2), R2, (0, 0, 1))
            ht = fc.turn_deg(fc.rotation(tool), RT, (0, 0, 1))
            p = fc.position_mm(tool)
            e = flange(h1, h12, -j3.jointMotion.slideValue * 10)
            worst_fk = max(worst_fk, math.dist(p, e))
            dt = math.dist(p, (x, y, z))
            r = math.hypot(x, y)
            bound = (r + L2) * math.radians(0.05)                             # half a tenth at J1 and at J2
            worst_t, worst_bound = max(worst_t, dt), max(worst_bound, dt / bound)
            dphi = abs((ht - phi + 180) % 360 - 180)
            rows.append('  (%7.1f %7.1f %5.1f %6.1f)  asked %8.3f %8.3f  held %7.1f %7.1f  off %.3f mm (bound %.3f), '
                        '%.3f deg' % (x, y, z, phi, q1, q2, h1, h12 - h1, dt, bound, dphi))
            fc.ok(dphi <= 0.15 + 1e-9, 'the tool turned to %.3f deg for %g' % (ht, phi))
            hit = fc.interference()
            if hit:
                clashes.append(((x, y), hit))
    print('\n'.join(rows))
    fc.ok(worst_fk < 1e-6, 'at every pose the tool is where forward kinematics puts it for the angles held (worst %.1e '
          'mm)' % worst_fk)
    fc.ok(worst_bound <= 1.0, 'and within the rounding of the target: worst %.3f mm, %.0f %% of its bound'
          % (worst_t, 100 * worst_bound))
    fc.ok(not clashes, 'no interference at any pose %s' % (clashes or ''))

    j2.jointMotion.rotationValue = math.radians(30.0)
    adsk.doEvents()
    j2.jointMotion.rotationValue = math.radians(Q2 + 10)
    adsk.doEvents()
    held = math.degrees(j2.jointMotion.rotationValue)
    fc.ok(abs(held - 30.0) < 1e-9, 'J2 asked %.0f deg, beyond its limit: it ignored it and kept %.1f deg' % (Q2 + 10, held))
    for j in (j1, j2, j4):
        j.jointMotion.rotationValue = 0.0
    j3.jointMotion.slideValue = 0.0
    adsk.doEvents()
    out = [(460, 0, 300, 0), (100, 0, 300, 0), (-440, 0, 300, 0), (300, 200, Z_FLANGE + 1, 0)]
    fc.ok(all(not ns['ik'](s, *t) for t in out), 'unreachable: beyond the reach, inside the inner radius, behind the '
          'robot, above the quill top')
    plate, a_f, r_x = workspace(s)
    fc.ok(abs(a_f - s['area']) < 1e-4 * s['area'], 'the outline drawn in Fusion encloses %.1f cm2 (the integral %.1f cm2);'
          ' the arcs cross %.3f mm behind the axis' % (a_f / 100, s['area'] / 100, r_x))
    c = fc.interference()
    fc.ok(not c, 'the workspace plate on the floor touches nothing %s' % (c or ''))
    fc.refused(ns, 'within 170 deg', q1max=175)
    fc.refused(ns, 'from the column', q2max=165)
    fc.refused(ns, 'from the first arm', L1=400, L2=200, q2max=170)
    fc.refused(ns, 'above the base', stroke=250)
    fc.verdict()
