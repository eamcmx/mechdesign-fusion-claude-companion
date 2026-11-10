# =============================================================================
# CarDifferential.py  -  Chapter 27 of "Mechanical Design in Autodesk Fusion with Claude": the rear axle of a car. A
# final-drive pinion turns a ring gear bolted to the differential case; the case carries N spiders on pins, and the
# spiders drive the two side gears and their half shafts. Written from PROMPT.txt beside it; checked by
# checks/car_differential_check.py.
#
#   INPUTS        the dialog: m_f, z1, z2, b_f, m_d, z_s, z_p, N, b_d, alpha
#   DERIVED       axle(): both bevel pairs, the torques; layout(): the case round the gears; phases(): which way each
#                 spider and the right side gear must be turned
#   HARD RULES    build() refuses undercut pinions, faces over R / 3, and spiders that cannot mesh with both side
#                 gears (2 z_s / N not whole); the phases; the case at z1 / z2 of the pinion; side gears opposite
#   CONVENTIONS   components created where they belong, local z along each axis; bevel gears as in Chapter 17;
#                 the case, housings and half shafts from TemporaryBRepManager; As-built Joints with the direction
#                 tried and its sign measured; Motion Links at the ratios of the teeth
#   ACCEPTANCE    the message; the case and half shafts at 105.517 deg per pinion turn; the side gears opposite in
#                 the case; no collision; three refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'CarDifferential 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
BACKLASH = 0.2                             # mm on the virtual pitch circle
D_PINION, D_PIN, D_AXLE = 24.0, 20.0, 30.0
ENGINE_T, FIRST_GEAR = 200.0, 3.5          # N m and the gearbox ratio of the torque example


def pair(m, z1, z2, alpha_deg, b):
    """A bevel pair at right angles: cone angles, cone distance, virtual teeth (z1 is the smaller gear)."""
    al = math.radians(alpha_deg)
    d1 = math.atan2(z1, z2)
    d2 = math.pi / 2 - d1
    R = (m * z1 / 2.0) / math.sin(d1)
    return dict(d1=d1, d2=d2, R=R, k=(R - b) / R, zv1=z1 / math.cos(d1), zv2=z2 / math.cos(d2),
                zv_min=2 / math.sin(al) ** 2)


def geometry(m, z, delta, R, alpha_deg, b, d):
    """One gear, in its own frame (apex at the origin, axis along z), as in Chapter 17. d is what goes through it."""
    sd, cd = math.sin(delta), math.cos(delta)
    g = dict(m=m, z=int(z), alpha=math.radians(alpha_deg), b=b, d=d, delta=delta, r=m * z / 2.0, R=R, k=(R - b) / R)
    g['rv'] = R * math.tan(delta)
    g['rbv'], g['rav'], g['rfv'] = g['rv'] * math.cos(g['alpha']), g['rv'] + m, g['rv'] - 1.25 * m
    g['half_v'] = (math.pi * m / 2.0 - BACKLASH / 2.0) / (2.0 * g['rv'])
    g['heel_root'] = (R * sd - 1.25 * m * cd, R * cd + 1.25 * m * sd)
    g['toe_root'] = (g['k'] * g['heel_root'][0], g['k'] * g['heel_root'][1])
    g['z_back'] = g['heel_root'][1] + 2.0
    g['r_back'] = (R - g['z_back'] * cd) / sd
    g['hub_r'] = d / 2.0 + 6.2
    g['h_end'] = g['z_back'] + 8.0
    g['bore'] = d + 0.4
    return g


def axle(m_f, z1, z2, b_f, m_d, z_s, z_p, N, b_d, alpha):
    """Everything the axle needs, computed from the inputs."""
    f = pair(m_f, z1, z2, alpha, b_f)
    d = pair(m_d, z_p, z_s, alpha, b_d)                             # the spider is the differential's pinion
    T_p = ENGINE_T * FIRST_GEAR
    T_c = T_p * z2 / z1
    return dict(ratio=z2 / z1, d1=f['d1'], d2=f['d2'], R_f=f['R'], zv1=f['zv1'], dp=d['d1'], ds=d['d2'], R_d=d['R'],
                zvp=d['zv1'], zv_min=f['zv_min'], T_p=T_p, T_c=T_c, T_half=T_c / 2, per_spider=2.0 * z_s / N)


def layout(sp, sd):
    """The case round the gears (mm), from the spiders' and side gears' sizes."""
    case_in = sp['h_end'] + 3.0
    wall_in = sd['z_back'] + 1.5
    return dict(case_in=case_in, case_out=case_in + 8.0, wall_in=wall_in, wall_out=wall_in + 8.0,
                hole=sd['hub_r'] + 0.8)


def phases(z_s, z_p, N):
    """The angle each gear is turned by in its own frame (from its local x, about its local z).
    Left side gear: local x = +Z, local z = +Y, phase 0. Spider i: local x = +Y (towards the left side gear), local z
    along psi_i. Right side gear: local x = +Z, local z = -Y. A gear shows a tooth at angle a when (a - phase) is a
    whole number of tooth pitches, and a gap when it is a whole number and a half."""
    def shows(a, phase, z):
        f = ((a - phase) * z / (2 * math.pi)) % 1.0
        if f < 1e-9 or f > 1 - 1e-9:
            return 'tooth'
        if abs(f - 0.5) < 1e-9:
            return 'gap'
        return None
    psi = [2 * math.pi * i / N for i in range(N)]
    sp = []
    for p in psi:
        left = shows(p, 0.0, z_s)                                    # the left gear at the spider's angle
        if left is None:
            raise ValueError('spider at %.1f deg meets the left side gear between a tooth and a gap' % math.degrees(p))
        sp.append(math.pi / z_p if left == 'tooth' else 0.0)          # gap against a tooth, tooth against a gap
    towards_right = [shows(math.pi, ph, z_p) for ph in sp]           # what each spider shows the right side gear
    right = 0.0 if towards_right[0] == 'gap' else math.pi / z_s
    for p, face in zip(psi, towards_right):                          # in the right gear's frame the spider is at -psi
        if shows(-p, right, z_s) == face or shows(-p, right, z_s) is None:
            raise ValueError('the right side gear cannot meet every spider tooth to gap')
    return psi, sp, right


# ---------- Fusion building blocks (mm at the interface) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cone(p0, p1, r0, r1):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r0 * CM, P3(*[v * CM for v in p1]), r1 * CM)


def boolean(a, kind, *bs):
    for b in bs:
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.UnionBooleanType, *bs)


def cut(a, *bs):
    return boolean(a, adsk.fusion.BooleanTypes.DifferenceBooleanType, *bs)


def add_body(comp, tb, name):
    """A temporary body into a component, through a base feature (as a parametric design needs)."""
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def place(root, name, x_axis=(1, 0, 0), z_axis=(0, 0, 1)):
    """A component created where it belongs: origin at the apex, local z along z_axis, local x along x_axis."""
    zx, zy, zz = z_axis
    xx, xy, xz = x_axis
    y_axis = (zy * xz - zz * xy, zz * xx - zx * xz, zx * xy - zy * xx)          # y = z x x
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(P3(0, 0, 0), V3(*x_axis), V3(*y_axis), V3(*z_axis))
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    return occ


def polyline(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(P3(pts[0][0], pts[0][1], 0), P3(pts[1][0], pts[1][1], 0))
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, P3(q[0], q[1], 0)).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def inv(a):
    return math.tan(a) - a


def tooth_uv(g, n_flank=8, n_tip=3, extra=1.2):
    """The virtual spur tooth at the heel, (u, v): u across the tooth, v outward from the virtual pitch circle."""
    rv, rb, ra, rf, a = g['rv'], g['rbv'], g['rav'], g['rfv'], g['alpha']
    phi = lambda rho: g['half_v'] + inv(a) - inv(math.acos(min(1.0, rb / rho)))
    lo = max(rf, rb)
    right = []
    for i in range(n_flank + 1):
        rho = lo + (ra - lo) * i / n_flank
        p = phi(rho)
        right.append((rho * math.sin(p), rho * math.cos(p) - rv))
    pt = phi(ra)
    tip = [(ra * math.sin(pt - 2 * pt * i / (n_tip + 1)), ra * math.cos(pt - 2 * pt * i / (n_tip + 1)) - rv)
           for i in range(1, n_tip + 1)]
    left = [(-u, v) for (u, v) in reversed(right)]
    p0, rb_ = phi(lo), rf - extra
    bottom = [(-rb_ * math.sin(p0), rb_ * math.cos(p0) - rv), (rb_ * math.sin(p0), rb_ * math.cos(p0) - rv)]
    return right + tip + left + bottom


def bevel_gear(occ, g, phase):
    """Blank (root cone, back cone, back face, hub, bore), then one tooth lofted heel to toe, joined, patterned z
    times. Everything in the gear's own frame: apex at the origin, axis along local z."""
    comp = occ.component
    hr, tr = g['heel_root'], g['toe_root']
    blank = cone((0, 0, tr[1]), (0, 0, hr[1]), tr[0], hr[0])
    union(blank, cone((0, 0, hr[1] - 0.001), (0, 0, g['z_back']), hr[0], g['r_back']))
    union(blank, cone((0, 0, g['z_back'] - 0.5), (0, 0, g['h_end']), g['hub_r'], g['hub_r']))
    cut(blank, cone((0, 0, tr[1] - 1), (0, 0, g['h_end'] + 1), g['bore'] / 2, g['bore'] / 2))
    body = add_body(comp, blank, comp.name)

    d, R = g['delta'], g['R']
    sd, cd = math.sin(d), math.cos(d)
    heel = [(R * sd + v * cd, u, R * cd - v * sd) for (u, v) in tooth_uv(g)]
    c, s = math.cos(phase), math.sin(phase)
    heel = [(x * c - y * s, x * s + y * c, zz) for (x, y, zz) in heel]           # tooth 0 turned by `phase`
    helper = comp.sketches.add(comp.xYConstructionPlane)
    helper.isVisible = False
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.JoinFeatureOperation)
    for scale in (1.0, g['k']):                                                    # heel, then toe
        pts = [(scale * x, scale * y, scale * zz) for (x, y, zz) in heel]
        n = len(pts)
        sp = [helper.sketchPoints.add(P3(*[v * CM for v in pts[j]])) for j in (0, n // 3, (2 * n) // 3)]
        pin = comp.constructionPlanes.createInput()
        pin.setByThreePoints(sp[0], sp[1], sp[2])
        plane = comp.constructionPlanes.add(pin)
        plane.isLightBulbOn = False
        sk = comp.sketches.add(plane)
        back = sk.transform.copy()
        back.invert()
        sk.isComputeDeferred = True
        q = []
        for (x, y, zz) in pts:
            p = P3(x * CM, y * CM, zz * CM)
            p.transformBy(back)
            q.append((p.x, p.y))
        polyline(sk, q)
        sk.isComputeDeferred = False
        sk.isVisible = False
        li.loftSections.add(sk.profiles.item(0))
    li.isSolid = True
    li.participantBodies = [body]
    feats = adsk.core.ObjectCollection.create()
    feats.add(lofts.add(li))
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = VI(g['z'])
    pi.totalAngle = VS('360 deg')
    pats.add(pi)
    return body


def case_body(lay, psi):
    """Tube, two end walls with holes for the side gears' hubs, and one pin per spider: one body, in world mm."""
    L = lay
    body = cone((0, -L['wall_out'], 0), (0, L['wall_out'], 0), L['case_out'], L['case_out'])
    cut(body, cone((0, -L['wall_in'], 0), (0, L['wall_in'], 0), L['case_in'], L['case_in']))
    cut(body, cone((0, -L['wall_out'] - 1, 0), (0, L['wall_out'] + 1, 0), L['hole'], L['hole']))
    reach = L['case_in'] + 4.0
    for p in psi:
        union(body, cone((0, 0, 0), (reach * math.sin(p), 0, reach * math.cos(p)), D_PIN / 2, D_PIN / 2))
    return body


def origin_point(occ):
    sk = occ.component.sketches.add(occ.component.xYConstructionPlane)
    sk.isVisible = False
    return sk.sketchPoints.add(P3(0, 0, 0)).createForAssemblyContext(occ)


def rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute_about(root, moving, fixed, axis, name):
    """An As-built revolute joint that turns `moving` about the world `axis`: the X, Y and Z joint directions are
    tried in turn and the one that really turns the part about `axis` is kept. Returns (joint, sign)."""
    J = adsk.fusion.JointDirections
    for direction in (J.ZAxisJointDirection, J.XAxisJointDirection, J.YAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(origin_point(moving)))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.2
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
        w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])            # the turn, as a vector
        dot = sum(w[i] * axis[i] for i in range(3))
        if abs(dot) > 0.1:
            j.name = name
            return j, (1 if dot > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about %s' % (moving.name, axis))


def rigid(root, moving, fixed, name):
    inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(origin_point(moving)))
    inp.setAsRigidJointMotion()
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def link(root, j1, j2, n1, n2, reverse, name):
    """A Motion Link: n1 turns of joint 1 for n2 turns of joint 2 (whole numbers of turns, in degrees)."""
    g = math.gcd(n1, n2)
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * n1 // g))
    li.valueTwo = VS('%d deg' % (360 * n2 // g))
    li.isReversed = reverse
    root.motionLinks.add(li).name = name


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the axle ----------
def build(m_f=4.0, z1=17, z2=58, b_f=30.0, m_d=5.0, z_s=22, z_p=15, N=2, b_d=20.0, alpha=20.0):
    z1, z2, z_s, z_p, N = (int(round(v)) for v in (z1, z2, z_s, z_p, N))
    a = axle(m_f, z1, z2, b_f, m_d, z_s, z_p, N, b_d, alpha)
    for name, zv in (('pinion', a['zv1']), ('spider', a['zvp'])):
        if zv < a['zv_min']:
            raise ValueError('The %s has %.2f virtual teeth (z / cos(delta)): below %.1f it is undercut.'
                             % (name, zv, a['zv_min']))
    if b_f > a['R_f'] / 3:
        raise ValueError('Final-drive face width %.1f mm is more than R_f / 3 = %.2f mm.' % (b_f, a['R_f'] / 3))
    if b_d > a['R_d'] / 3:
        raise ValueError('Differential face width %.1f mm is more than R_d / 3 = %.2f mm.' % (b_d, a['R_d'] / 3))
    if not 2 <= N <= 4 or abs(a['per_spider'] - round(a['per_spider'])) > 1e-9:
        raise ValueError('%d spiders: N must be 2 to 4 and 2 z_s / N a whole number; here 2 z_s / N = %.2f.'
                         % (N, a['per_spider']))
    psi, sp_phase, right_phase = phases(z_s, z_p, N)

    gp = geometry(m_f, z1, a['d1'], a['R_f'], alpha, b_f, D_PINION)
    gsp = geometry(m_d, z_p, a['dp'], a['R_d'], alpha, b_d, D_PIN)
    gsd = geometry(m_d, z_s, a['ds'], a['R_d'], alpha, b_d, D_AXLE)
    lay = layout(gsp, gsd)
    gr = geometry(m_f, z2, a['d2'], a['R_f'], alpha, b_f, 2 * lay['case_out'])

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    housings = []
    for side, s in (('L', 1), ('R', -1)):                             # tubes round the half shafts, grounded
        h = place(root, 'Housing ' + side)
        t = cone((0, 60 * s, 0), (0, 300 * s, 0), 24.0, 24.0)
        cut(t, cone((0, 59 * s, 0), (0, 301 * s, 0), 16.0, 16.0))
        add_body(h.component, t, 'Housing ' + side)
        h.isGrounded = True
        housings.append(h)
    pin_frame = dict(x_axis=(0, -1, 0), z_axis=(1, 0, 0))            # the pinion: axis +X, tooth 0 towards the ring
    ps = place(root, 'Pinion shaft', **pin_frame)
    add_body(ps.component, cone((0, 0, gp['toe_root'][1] - 2.0), (0, 0, gp['h_end'] + 5.0), D_PINION / 2, D_PINION / 2),
             'Pinion shaft')
    ps.isGrounded = True
    pinion = place(root, 'Pinion', **pin_frame)
    bevel_gear(pinion, gp, 0.0)
    ring = place(root, 'Ring gear', x_axis=(1, 0, 0), z_axis=(0, -1, 0))
    bevel_gear(ring, gr, math.pi / z2)                              # half a tooth: a gap meets the pinion's tooth 0
    case = place(root, 'Case')
    add_body(case.component, case_body(lay, psi), 'Case')
    spiders = []
    for i, (p, ph) in enumerate(zip(psi, sp_phase)):
        s = place(root, 'Spider %d' % (i + 1), x_axis=(0, 1, 0), z_axis=(math.sin(p), 0, math.cos(p)))
        bevel_gear(s, gsp, ph)
        spiders.append(s)
    side_l = place(root, 'Side gear L', x_axis=(0, 0, 1), z_axis=(0, 1, 0))
    bevel_gear(side_l, gsd, 0.0)
    side_r = place(root, 'Side gear R', x_axis=(0, 0, 1), z_axis=(0, -1, 0))
    bevel_gear(side_r, gsd, right_phase)
    shafts = []
    for side, s in (('L', 1), ('R', -1)):
        h = place(root, 'Half shaft ' + side)
        add_body(h.component, cone((0, 25 * s, 0), (0, 300 * s, 0), D_AXLE / 2, D_AXLE / 2), 'Half shaft ' + side)
        shafts.append(h)

    jp, sg_p = revolute_about(root, pinion, ps, (1, 0, 0), 'Pinion turns')
    jc, sg_c = revolute_about(root, case, housings[0], (0, 1, 0), 'Case turns')
    rigid(root, ring, case, 'Ring bolted to case')
    jl, sg_l = revolute_about(root, side_l, case, (0, 1, 0), 'Side gear L turns in case')
    jr, sg_r = revolute_about(root, side_r, case, (0, 1, 0), 'Side gear R turns in case')
    js = []
    for i, (s, p) in enumerate(zip(spiders, psi)):
        js.append(revolute_about(root, s, case, (math.sin(p), 0, math.cos(p)), 'Spider %d turns on its pin' % (i + 1)))
    rigid(root, shafts[0], side_l, 'Half shaft L on side gear L')
    rigid(root, shafts[1], side_r, 'Half shaft R on side gear R')

    link(root, jp, jc, z2, z1, sg_p * sg_c < 0, 'Final drive %d:%d' % (z1, z2))            # case = +(z1/z2) pinion
    for i, (j, sg) in enumerate(js):
        link(root, jl, j, z_p, z_s, sg_l * sg > 0, 'Side L - spider %d' % (i + 1))         # spider = -(z_s/z_p) side
    link(root, jl, jr, 1, 1, sg_l * sg_r > 0, 'Side L - side R')                          # right = -left, in the case

    colour(design, pinion, 'Paint - Enamel Glossy (Blue)')
    colour(design, ring, 'Paint - Enamel Glossy (Yellow)')
    for s in spiders:
        colour(design, s, 'Paint - Enamel Glossy (Red)')
    for s in (side_l, side_r):
        colour(design, s, 'Paint - Enamel Glossy (Green)')
    app.activeViewport.fit()

    return ('Car rear axle built: final drive %d : %d (ratio %.4f), open differential with %d spiders of %d teeth and '
            'side gears of %d.\n\n'
            'Final drive: delta1 = %.3f deg, delta2 = %.3f deg, R_f = %.3f mm, z_v1 = %.2f\n'
            'Differential: delta_p = %.3f deg, delta_s = %.3f deg, R_d = %.3f mm, z_vp = %.2f; 2 z_s / N = %d\n'
            'With 200 N m in a first gear of 3.5: pinion %.1f N m, case %.2f N m, %.2f N m per half shaft.\n\n'
            'Drive Joints on "Pinion turns" turns the case %.3f deg per pinion turn; on "Side gear L turns in case" '
            'the two half shafts turn opposite ways.\n\n[%s]'
            % (z1, z2, a['ratio'], N, z_p, z_s, math.degrees(a['d1']), math.degrees(a['d2']), a['R_f'], a['zv1'],
               math.degrees(a['dp']), math.degrees(a['ds']), a['R_d'], a['zvp'], round(a['per_spider']), a['T_p'],
               a['T_c'], a['T_half'], 360.0 * z1 / z2, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(m_f=ins.itemById('m_f').value / CM, z1=ins.itemById('z1').value, z2=ins.itemById('z2').value,
                     b_f=ins.itemById('b_f').value / CM, m_d=ins.itemById('m_d').value / CM,
                     z_s=ins.itemById('z_s').value, z_p=ins.itemById('z_p').value, N=ins.itemById('N').value,
                     b_d=ins.itemById('b_d').value / CM, alpha=math.degrees(ins.itemById('alpha').value))
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m_f', 'Final-drive module at the heel', 'mm', VI(4.0 * CM))
        ins.addIntegerSpinnerCommandInput('z1', 'Pinion teeth', 6, 60, 1, 17)
        ins.addIntegerSpinnerCommandInput('z2', 'Ring gear teeth', 20, 120, 1, 58)
        ins.addValueInput('b_f', 'Final-drive face width', 'mm', VI(30.0 * CM))
        ins.addValueInput('m_d', 'Differential module at the heel', 'mm', VI(5.0 * CM))
        ins.addIntegerSpinnerCommandInput('z_s', 'Side gear teeth', 8, 60, 1, 22)
        ins.addIntegerSpinnerCommandInput('z_p', 'Spider teeth', 6, 60, 1, 15)
        ins.addIntegerSpinnerCommandInput('N', 'Spiders', 1, 6, 1, 2)
        ins.addValueInput('b_d', 'Differential face width', 'mm', VI(20.0 * CM))
        ins.addValueInput('alpha', 'Pressure angle', 'deg', VI(math.radians(20.0)))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'CarDifferentialCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Car differential', 'Final drive and open differential')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Car differential could not start:\n' + traceback.format_exc())
