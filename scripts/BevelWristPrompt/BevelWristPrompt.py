# =============================================================================
# BevelWristPrompt.py  -  the script that the bevel-wrist prompt of Topic 5 asks for (PROMPT.txt beside it),
# written from that prompt alone: every size, rule, joint and link below is a line of the prompt.
#
#   INPUTS        FIELDS (the dialog)
#   DERIVED       geometry(): delta, r, R, k, the virtual (Tredgold) tooth, heel and toe roots, back face, hub, S
#   HARD RULES    build() refuses an odd z and b > R/3; the four gears round one apex; the mesh phases
#   CONVENTIONS   components created where they belong, with the gear's axis as local z; blanks from
#                 TemporaryBRepManager in a base feature; teeth lofted heel to toe and patterned; As-built Joints with
#                 the joint direction tried and its sign measured; the meshes as 1:1 Motion Links from Roll
#   ACCEPTANCE    the message; pitch, roll and the combined pose; no interference at rest, in two poses and through
#                 one tooth pitch of roll; z = 30 and z = 41 refused
#
# Tested in Fusion: workshop/fusion_toolkit/_dev/cases/bevelwrist_prompt_check.py
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BevelWristPrompt 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
FLANGE_R, FLANGE_T = 18.0, 5.0             # mm, the output flange

# (id, label, default, unit)
FIELDS = [('m', 'Module at the heel', 2.0, 'mm'), ('z', 'Teeth on each gear (even)', 40, 'n'),
          ('alpha', 'Pressure angle', 20.0, 'deg'), ('b', 'Face width', 16.0, 'mm'),
          ('backlash', 'Backlash on the pitch circle', 0.2, 'mm'), ('d', 'Yoke pin diameter', 8.0, 'mm')]


# ---------- DERIVED ----------
def geometry(m, z, alpha_deg, b, backlash, d):
    delta = math.radians(45.0)
    sd, cd = math.sin(delta), math.cos(delta)
    r = m * z / 2.0
    R = r / sd
    g = dict(m=m, z=int(z), alpha=math.radians(alpha_deg), b=b, d=d, delta=delta, r=r, R=R, k=(R - b) / R)
    g['rv'] = R * math.tan(delta)                                   # virtual pitch radius, r / cos(delta)
    g['rbv'], g['rav'], g['rfv'] = g['rv'] * math.cos(g['alpha']), g['rv'] + m, g['rv'] - 1.25 * m
    g['half_v'] = (math.pi * m / 2.0 - backlash / 2.0) / (2.0 * g['rv'])
    g['heel_root'] = (R * sd - 1.25 * m * cd, R * cd + 1.25 * m * sd)
    g['toe_root'] = (g['k'] * g['heel_root'][0], g['k'] * g['heel_root'][1])
    g['z_back'] = g['heel_root'][1] + 2.0
    g['r_back'] = (R - g['z_back'] * cd) / sd                       # where the back cone meets the back face
    g['hub_r'] = d / 2.0 + 6.2
    g['h_end'] = g['z_back'] + 8.0
    g['bore'] = d + 0.4
    g['S'] = math.sqrt((g['h_end'] + FLANGE_T) ** 2 + FLANGE_R ** 2)
    return g


def inv(a):
    return math.tan(a) - a


def tooth_uv(g, n_flank=8, n_tip=3, extra=1.2):
    """The virtual spur tooth at the heel, (u, v): u across the tooth, v outward from the virtual pitch circle.
    Involute flanks from the base circle; below it (and down to just under the root) a radial line."""
    rv, rb, ra, rf, a = g['rv'], g['rbv'], g['rav'], g['rfv'], g['alpha']
    phi = lambda rho: g['half_v'] + inv(a) - inv(math.acos(min(1.0, rb / rho)))   # half-angle of the tooth at rho
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


# ---------- Fusion building blocks (mm at the interface) ----------
def tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def cone(p0, p1, r0, r1):
    return tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r0 * CM, P3(*[v * CM for v in p1]), r1 * CM)


def box(x0, x1, y0, y1, z0, z1):
    c = P3((x0 + x1) / 2 * CM, (y0 + y1) / 2 * CM, (z0 + z1) / 2 * CM)
    obb = adsk.core.OrientedBoundingBox3D.create(c, V3(1, 0, 0), V3(0, 1, 0), (x1 - x0) * CM, (y1 - y0) * CM, (z1 - z0) * CM)
    return tbm().createBox(obb)


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
    body = comp.bRepBodies.add(tb, bf)
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


def bevel_gear(occ, g, phase, flange):
    """Blank (root cone, back cone, back face, hub, bore), then one tooth lofted heel to toe, joined, patterned z
    times. Everything in the gear's own frame: apex at the origin, axis along local z."""
    comp = occ.component
    hr, tr = g['heel_root'], g['toe_root']
    blank = cone((0, 0, tr[1]), (0, 0, hr[1]), tr[0], hr[0])
    union(blank, cone((0, 0, hr[1] - 0.001), (0, 0, g['z_back']), hr[0], g['r_back']))
    union(blank, cone((0, 0, g['z_back'] - 0.5), (0, 0, g['h_end']), g['hub_r'], g['hub_r']))
    if flange:
        union(blank, cone((0, 0, g['h_end'] - 0.001), (0, 0, g['h_end'] + FLANGE_T), FLANGE_R, FLANGE_R))
    top = g['h_end'] + (FLANGE_T if flange else 0)
    cut(blank, cone((0, 0, tr[1] - 1), (0, 0, top + 1), g['bore'] / 2, g['bore'] / 2))
    body = add_body(comp, blank, occ.component.name)

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


def link(root, j_a, s_a, j_b, s_b, k, name):
    """1:1 Motion Link so that (world turn of b) = k x (world turn of a), k = +1 or -1."""
    li = root.motionLinks.createInput(j_a, j_b)
    li.valueOne = VS('360 deg')
    li.valueTwo = VS('360 deg')
    li.isReversed = (s_a * s_b * k) < 0
    ml = root.motionLinks.add(li)
    ml.name = name
    return ml


# ---------- the wrist ----------
def build(m=2.0, z=40, alpha=20.0, b=16.0, backlash=0.2, d=8.0):
    z = int(round(z))
    g = geometry(m, z, alpha, b, backlash, d)
    if z % 2:
        raise ValueError('z = %d is odd: each gear meets two others 180 degrees apart, so z must be even.' % z)
    if b > g['R'] / 3 + 1e-9:
        raise ValueError('Face width %.1f mm is more than R / 3 = %.2f / 3 = %.2f mm.' % (b, g['R'], g['R'] / 3))

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent
    h, S = g['h_end'], g['S']

    # frame: two cheeks on a base plate, bores on the pitch axis
    frame = place(root, 'Frame')
    plate_top = -(S + 15.0)
    yo = h + 12.0
    fb = box(-S, S, -yo, yo, plate_top - 6.0, plate_top)
    for sgn in (1, -1):
        y0, y1 = sorted((sgn * (h + 4.0), sgn * (h + 12.0)))
        union(fb, box(-25.0, 25.0, y0, y1, plate_top - 0.001, 25.0))
    cut(fb, cone((0, -yo - 1, 0), (0, yo + 1, 0), g['bore'] / 2, g['bore'] / 2))
    add_body(frame.component, fb, 'Frame')

    # yoke: the cross of the roll pin (Z) and the pitch pin (Y)
    yoke = place(root, 'Yoke')
    yb = cone((0, 0, -h), (0, 0, h + FLANGE_T), d / 2, d / 2)
    union(yb, cone((0, -(yo + 6.0), 0), (0, yo + 6.0, 0), d / 2, d / 2))
    add_body(yoke.component, yb, 'Yoke')

    # the four gears round one apex: sides point a tooth at +Z, output and idler a gap at +Y
    out = place(root, 'Output gear', (0, 1, 0), (0, 0, 1))
    bevel_gear(out, g, math.pi / z, True)
    idl = place(root, 'Idler gear', (0, 1, 0), (0, 0, -1))
    bevel_gear(idl, g, math.pi / z, False)
    sR = place(root, 'Side gear R', (0, 0, 1), (0, 1, 0))
    bevel_gear(sR, g, 0.0, False)
    sL = place(root, 'Side gear L', (0, 0, 1), (0, -1, 0))
    bevel_gear(sL, g, 0.0, False)

    # joints (directions tried, signs measured) and the meshes as 1:1 motion links from Roll
    frame.isGrounded = True
    Y, Z = (0, 1, 0), (0, 0, 1)
    j_p, s_p = revolute_about(root, yoke, frame, Y, 'Pitch')
    j_r, s_r = revolute_about(root, out, yoke, Z, 'Roll')
    j_i, s_i = revolute_about(root, idl, yoke, Z, 'Idler on yoke')
    j_R, s_R = revolute_about(root, sR, yoke, Y, 'Side gear R on yoke')
    j_L, s_L = revolute_about(root, sL, yoke, Y, 'Side gear L on yoke')
    link(root, j_r, s_r, j_L, s_L, +1, 'Mesh: roll -> side L')
    link(root, j_r, s_r, j_R, s_R, -1, 'Mesh: roll -> side R')
    link(root, j_r, s_r, j_i, s_i, -1, 'Mesh: roll -> idler')
    app.activeViewport.fit()

    return ('Bevel wrist built.\n\n'
            'Four miter gears: z = %d, module %.2f mm at the heel, cone distance R = %.2f mm, face %.1f mm\n'
            'Drive "Pitch" to pitch the wrist and "Roll" to roll it. With the side gears\' world angles about +Y:\n'
            '  pitch = (thL + thR) / 2,   roll = (thL - thR) / 2\n\n[%s]' % (z, m, g['R'], b, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            inputs = args.command.commandInputs
            p = {}
            for key, _, _, unit in FIELDS:
                v = inputs.itemById(key).value
                p[key] = v / CM if unit == 'mm' else (math.degrees(v) if unit == 'deg' else v)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        cmd.setDialogInitialSize(420, 280)
        inputs = cmd.commandInputs
        for key, label, dflt, unit in FIELDS:
            if unit == 'n':
                inputs.addIntegerSpinnerCommandInput(key, label, 6, 200, 1, int(dflt))
            elif unit == 'deg':
                inputs.addValueInput(key, label, 'deg', VS('%g deg' % dflt))
            else:
                inputs.addValueInput(key, label, 'mm', VI(dflt * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BevelWristPromptCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bevel wrist (from the prompt)',
                                                         'The bevel differential of a robot wrist, jointed and linked')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Bevel wrist could not start:\n' + traceback.format_exc())
