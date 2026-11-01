# =============================================================================
# BevelPair.py  -  Chapter 16 of "Mechanical Design in Autodesk Fusion with Claude": a pair of straight bevel gears on
# shafts at right angles, assembled and ready to turn with Drive Joints. The gears are built as the bevel wrist of
# Topic 5 builds its miter gears, for any pair of cone angles. Written from PROMPT.txt beside it; checked by
# checks/bevel_pair_check.py.
#
#   INPUTS        the dialog: m, z1, z2, alpha, b, d
#   DERIVED       pair(): the cone angles, R, k, the virtual teeth, the contact ratio; geometry(): one gear's sizes
#   HARD RULES    build() refuses b > R / 3 and a pinion whose virtual teeth undercut; one apex; the gear turned
#                 half a tooth; the gear turning the other way at z1 / z2
#   CONVENTIONS   components created where they belong, local z along the axis; blanks from TemporaryBRepManager in
#                 a base feature; teeth lofted heel to toe and patterned; As-built Joints with the direction tried
#                 and its sign measured; a Motion Link at the ratio of the teeth
#   ACCEPTANCE    the message; 90 deg of the pinion turns the gear -45 deg about +X; no collision; two refusals
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math
import traceback

import adsk.core
import adsk.fusion

VERSION = 'BevelPair 1.0 (2026-11-01)'
CM = 0.1                                   # Fusion works in centimetres: millimetres x 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
BACKLASH = 0.2                             # mm on the virtual pitch circle


def pair(m, z1, z2, alpha_deg, b):
    al = math.radians(alpha_deg)
    d1 = math.atan2(z1, z2)
    d2 = math.pi / 2 - d1
    R = (m * z1 / 2.0) / math.sin(d1)
    rv1, rv2 = R * math.tan(d1), R * math.tan(d2)
    eps = (math.sqrt((rv1 + m) ** 2 - (rv1 * math.cos(al)) ** 2) + math.sqrt((rv2 + m) ** 2 - (rv2 * math.cos(al)) ** 2)
           - (rv1 + rv2) * math.sin(al)) / (math.pi * m * math.cos(al))
    return dict(d1=d1, d2=d2, R=R, k=(R - b) / R, zv1=z1 / math.cos(d1), zv2=z2 / math.cos(d2), eps=eps,
                zv_min=2 / math.sin(al) ** 2)


def geometry(m, z, delta, R, alpha_deg, b, d):
    """One gear of the pair, in its own frame (apex at the origin, axis along z), as in the bevel wrist."""
    sd, cd = math.sin(delta), math.cos(delta)
    g = dict(m=m, z=int(z), alpha=math.radians(alpha_deg), b=b, d=d, delta=delta, r=m * z / 2.0, R=R, k=(R - b) / R)
    g['rv'] = R * math.tan(delta)                                   # virtual pitch radius, r / cos(delta)
    g['rbv'], g['rav'], g['rfv'] = g['rv'] * math.cos(g['alpha']), g['rv'] + m, g['rv'] - 1.25 * m
    g['half_v'] = (math.pi * m / 2.0 - BACKLASH / 2.0) / (2.0 * g['rv'])
    g['heel_root'] = (R * sd - 1.25 * m * cd, R * cd + 1.25 * m * sd)
    g['toe_root'] = (g['k'] * g['heel_root'][0], g['k'] * g['heel_root'][1])
    g['z_back'] = g['heel_root'][1] + 2.0
    g['r_back'] = (R - g['z_back'] * cd) / sd                       # where the back cone meets the back face
    g['hub_r'] = d / 2.0 + 6.2
    g['h_end'] = g['z_back'] + 8.0
    g['bore'] = d + 0.4
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


def bevel_gear(occ, g, phase):
    """Blank (root cone, back cone, back face, hub, bore), then one tooth lofted heel to toe, joined, patterned z
    times. Everything in the gear's own frame: apex at the origin, axis along local z."""
    comp = occ.component
    hr, tr = g['heel_root'], g['toe_root']
    blank = cone((0, 0, tr[1]), (0, 0, hr[1]), tr[0], hr[0])
    union(blank, cone((0, 0, hr[1] - 0.001), (0, 0, g['z_back']), hr[0], g['r_back']))
    union(blank, cone((0, 0, g['z_back'] - 0.5), (0, 0, g['h_end']), g['hub_r'], g['hub_r']))
    cut(blank, cone((0, 0, tr[1] - 1), (0, 0, g['h_end'] + 1), g['bore'] / 2, g['bore'] / 2))
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


def shaft(occ, g):
    """A shaft of diameter d along the part's own z, through the gear's bore, as one body."""
    body = cone((0, 0, g['toe_root'][1] - 2.0), (0, 0, g['h_end'] + 5.0), g['d'] / 2, g['d'] / 2)
    add_body(occ.component, body, occ.component.name)


def colour(design, occ, name):
    try:
        ap = design.appearances.itemByName(name)
        if not ap:
            lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Appearance Library')
            ap = design.appearances.addByCopy(lib.appearances.itemByName(name), name)
        occ.appearance = ap
    except Exception:
        pass


# ---------- the pair ----------
def build(m=2.0, z1=20, z2=40, alpha=20.0, b=12.0, d=8.0):
    z1, z2 = int(round(z1)), int(round(z2))
    p = pair(m, z1, z2, alpha, b)
    if b > p['R'] / 3:
        raise ValueError('Face width %.1f mm is more than R / 3 = %.2f mm.' % (b, p['R'] / 3))
    if p['zv1'] < p['zv_min']:
        raise ValueError('The pinion has z_v1 = z1 / cos(delta1) = %.2f virtual teeth: below %.1f it is undercut.'
                         % (p['zv1'], p['zv_min']))
    g1 = geometry(m, z1, p['d1'], p['R'], alpha, b, d)
    g2 = geometry(m, z2, p['d2'], p['R'], alpha, b, d)

    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design.rootComponent

    frame1 = dict(x_axis=(1, 0, 0), z_axis=(0, 0, 1))              # the pinion's axis +Z, tooth 0 towards +X
    frame2 = dict(x_axis=(0, 0, 1), z_axis=(1, 0, 0))              # the gear's axis +X, tooth 0 towards +Z
    s1 = place(root, 'Shaft 1', **frame1)
    shaft(s1, g1)
    s1.isGrounded = True
    s2 = place(root, 'Shaft 2', **frame2)
    shaft(s2, g2)
    s2.isGrounded = True
    pinion = place(root, 'Pinion', **frame1)
    bevel_gear(pinion, g1, 0.0)
    gear = place(root, 'Gear', **frame2)
    bevel_gear(gear, g2, math.pi / z2)                              # half a tooth: a gap meets the pinion's tooth 0

    j1, sg1 = revolute_about(root, pinion, s1, (0, 0, 1), 'Pinion turns')
    j2, sg2 = revolute_about(root, gear, s2, (1, 0, 0), 'Gear turns')
    g = math.gcd(z1, z2)
    li = root.motionLinks.createInput(j1, j2)
    li.valueOne = VS('%d deg' % (360 * z2 // g))                  # z2 / z1 turns of the pinion ...
    li.valueTwo = VS('%d deg' % (360 * z1 // g))                  # ... for one turn of the gear
    li.isReversed = sg1 * sg2 > 0                                  # world turns opposite: gear = -(z1/z2) pinion
    root.motionLinks.add(li).name = 'Mesh %d:%d' % (z1, z2)
    colour(design, pinion, 'Paint - Enamel Glossy (Blue)')
    colour(design, gear, 'Paint - Enamel Glossy (Yellow)')
    app.activeViewport.fit()

    return ('Bevel pair built: module %.2f at the heel, %d : %d teeth, shafts at 90 deg.\n\n'
            'Pitch cone angles delta1 = %.3f deg, delta2 = %.3f deg; cone distance R = %.3f mm\n'
            'Virtual (Tredgold) teeth z_v1 = %.2f, z_v2 = %.2f; contact ratio %.3f\n\n'
            'Drive Joints on "Pinion turns": 90 deg turns the gear %.1f deg about +X.\n\n[%s]'
            % (m, z1, z2, math.degrees(p['d1']), math.degrees(p['d2']), p['R'], p['zv1'], p['zv2'], p['eps'],
               -90.0 * z1 / z2, VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            p = dict(m=ins.itemById('m').value / CM, z1=ins.itemById('z1').value, z2=ins.itemById('z2').value,
                     alpha=math.degrees(ins.itemById('alpha').value), b=ins.itemById('b').value / CM,
                     d=ins.itemById('d').value / CM)
            ui.messageBox(build(**p), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m', 'Module at the heel', 'mm', VI(2.0 * CM))
        ins.addIntegerSpinnerCommandInput('z1', 'Pinion teeth', 6, 200, 1, 20)
        ins.addIntegerSpinnerCommandInput('z2', 'Gear teeth', 6, 200, 1, 40)
        ins.addValueInput('alpha', 'Pressure angle', 'deg', VI(math.radians(20.0)))
        ins.addValueInput('b', 'Face width', 'mm', VI(12.0 * CM))
        ins.addValueInput('d', 'Shaft diameter', 'mm', VI(8.0 * CM))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'BevelPairCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bevel pair', 'Two straight bevel gears at right angles')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('Bevel pair could not start:\n' + traceback.format_exc())
