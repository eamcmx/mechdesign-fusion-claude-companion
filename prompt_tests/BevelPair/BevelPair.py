# =============================================================================
# BevelPair.py  -  a pair of straight bevel gears on shafts at right angles, drawn by Tredgold's approximation,
# assembled with As-built Joints and a Motion Link so that Drive Joints turns them.
#
#   Inputs (a dialog): module m at the heel, teeth z1 and z2, pressure angle alpha, face width b, shaft diameter d.
#   Everything else is derived. Sizes here are in mm and degrees; the Fusion API gets cm and radians.
#
#   Frames: both pitch cones have their apex at the world origin. The pinion's axis is world +Z, the gear's world
#   +X. Every component has its origin at the apex and its own z along its axis, and its tooth 0 on its own +x;
#   the gear's own +x is turned half a tooth from world +Z, so that a gap of the gear meets tooth 0 of the pinion.
#
#   Written from the printed prompt alone (prompt test).
# =============================================================================
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion():
    """Walk up from this file to the first folder holding common/companion_paths.py, put common/ on sys.path and
    import it (every lib/ of the companion is then on sys.path). None when the script is run outside it."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            c = os.path.join(d, 'common')
            if c not in sys.path:
                sys.path.insert(0, c)
            import companion_paths
            return companion_paths
        up = os.path.dirname(d)
        if up == d:
            return None
        d = up


CP = _companion()

VERSION = 'BevelPair v1.0 (prompt test, 11 Nov 2026)'
CMD_ID = 'mdfc_BevelPair_prompt_test'
CM = 0.1                                   # mm to cm

DEFAULTS = dict(m=2.0, z1=20, z2=40, alpha=20.0, b=12.0, d=8.0)

BACKLASH = 0.2          # mm on the virtual pitch circle: each tooth pi m / 2 - BACKLASH / 2 thick
HUB_EXTRA = 6.2         # hub radius = d / 2 + HUB_EXTRA
HUB_LENGTH = 8.0
BORE_CLEARANCE = 0.4    # bore = d + BORE_CLEARANCE
BACK_FACE = 2.0         # the back face is this far (along the axis) behind the heel root
ROOT_DEPTH = 0.75       # the heel outline is closed this many modules below the root circle, inside the blank
LIP_TOE = 0.5           # the root cone of the blank runs this far (along the axis) in front of the toe...
LIP_HEEL = 0.5          # ...and its back cone sits this far behind the heel, so that no face of a tooth is
                        # tangent to a face of the blank (the heel plane of a tooth touches the back cone)
FLANK_SEGMENTS = 10     # straight segments on each involute flank
TIP_SEGMENTS = 4        # on the tip arc
SHAFT_BEFORE = 4.0      # a shaft starts this far in front of its gear's toe face...
SHAFT_AFTER = 10.0      # ...and ends this far beyond the end of the hub


# ---------------------------------------------------------------- design calculations (no Fusion needed)
def pair(m, z1, z2, alpha_deg, b):
    """The derived values of the pair: cone angles d1, d2 (rad), pitch radii, cone distance R, k = (R - b) / R,
    the virtual spur gears of Tredgold's approximation (radii rv, teeth zv) and their contact ratio eps."""
    al = math.radians(alpha_deg)
    d1 = math.atan(z1 / float(z2))
    d2 = math.pi / 2.0 - d1
    r1, r2 = m * z1 / 2.0, m * z2 / 2.0
    R = r1 / math.sin(d1)
    k = (R - b) / R
    rv1, rv2 = R * math.tan(d1), R * math.tan(d2)
    zv1, zv2 = z1 / math.cos(d1), z2 / math.cos(d2)

    def approach(rv):                      # from the pitch point to where the tip circle cuts the line of action
        return math.sqrt((rv + m) ** 2 - (rv * math.cos(al)) ** 2)

    eps = (approach(rv1) + approach(rv2) - (rv1 + rv2) * math.sin(al)) / (math.pi * m * math.cos(al))
    return dict(m=m, z1=z1, z2=z2, alpha=al, alpha_deg=alpha_deg, b=b, d1=d1, d2=d2, r1=r1, r2=r2, R=R, k=k,
                rv1=rv1, rv2=rv2, zv1=zv1, zv2=zv2, eps=eps, ratio=z2 / float(z1),
                zv_min=2.0 / math.sin(al) ** 2)


def _cone(p, which):
    return p['d1'] if which == 1 else p['d2']


def _inv(a):
    return math.tan(a) - a


def tooth_2d(p, which, n=FLANK_SEGMENTS):
    """Tooth 0 of the virtual spur gear of the pinion (which = 1) or the gear (2), in its own plane: x from the
    virtual centre along the tooth, y across it (mm). Involute flanks from the base circle (or the root circle,
    whichever is larger) to the tip, a radial line below that, closed ROOT_DEPTH modules below the root circle."""
    m, al = p['m'], p['alpha']
    rv = p['rv1'] if which == 1 else p['rv2']
    rb, ra, rf = rv * math.cos(al), rv + m, rv - 1.25 * m
    half_p = (math.pi * m / 2.0 - BACKLASH / 2.0) / (2.0 * rv)        # half the tooth's angle on the pitch circle

    def half(r):
        if r <= rb:
            return half_p + _inv(al)
        return half_p + _inv(al) - _inv(math.acos(rb / r))

    if half(ra) <= 0.0:
        raise ValueError('the teeth of the %s come to a point before the tip circle' % ('pinion', 'gear')[which - 1])
    r_lo = max(rb, rf)
    r_in = rf - ROOT_DEPTH * m
    t_lo = math.sqrt(max((r_lo / rb) ** 2 - 1.0, 0.0))
    t_hi = math.sqrt((ra / rb) ** 2 - 1.0)
    flank = [(r_in, half(r_lo))]                                       # (radius, half angle) from the bottom up
    for i in range(n + 1):
        t = t_lo + (t_hi - t_lo) * i / n
        r = rb * math.sqrt(1.0 + t * t)
        flank.append((r, half(r)))
    pts = [(r * math.cos(h), -r * math.sin(h)) for r, h in flank]     # one flank up
    h_a = half(ra)
    for j in range(1, TIP_SEGMENTS):                                   # across the tip
        a = -h_a + 2.0 * h_a * j / TIP_SEGMENTS
        pts.append((ra * math.cos(a), ra * math.sin(a)))
    pts += [(r * math.cos(h), r * math.sin(h)) for r, h in reversed(flank)]   # the other flank down
    return pts


def heel_outline(p, which):
    """Tooth 0 at the heel in the part's own frame (mm): drawn in the plane square to the pitch cone's line at the
    heel (it touches the back cone along the tooth's middle line), the virtual centre on the axis at R / cos(delta)
    and the tooth pointing along the back cone away from the axis, towards the part's own +x."""
    dl = _cone(p, which)
    c, s = math.cos(dl), math.sin(dl)
    bz = p['R'] / c
    return [(x * c, y, bz - x * s) for x, y in tooth_2d(p, which)]


def blank(p, which, d):
    """The blank of the pinion or the gear along its own z (mm): (axial position, radius) of its corners."""
    dl = _cone(p, which)
    c, s = math.cos(dl), math.sin(dl)
    m, R, k = p['m'], p['R'], p['k']
    bz = R / c                                         # the apex of the back cone, on the axis
    rf = R * math.tan(dl) - 1.25 * m                   # the virtual root radius
    r_hr, a_hr = rf * c, bz - rf * s                   # the heel root point
    g = r_hr / a_hr                                    # the root cone: radius = g * axial
    out = dict(a_hr=a_hr, r_hr=r_hr)
    out['a_toe'] = k * a_hr - LIP_TOE
    out['r_toe'] = g * out['a_toe']
    out['a_heel'] = (bz + LIP_HEEL) / (1.0 + g * s / c)          # the root cone meets the back cone
    out['r_heel'] = g * out['a_heel']
    out['a_back'] = a_hr + BACK_FACE
    out['r_back'] = (bz + LIP_HEEL - out['a_back']) * c / s      # on the back cone
    out['r_hub'] = d / 2.0 + HUB_EXTRA
    out['a_hub'] = out['a_back'] + HUB_LENGTH
    out['r_bore'] = (d + BORE_CLEARANCE) / 2.0
    out['a_shaft0'] = out['a_toe'] - SHAFT_BEFORE
    out['a_shaft1'] = out['a_hub'] + SHAFT_AFTER
    return out


def refuse(p, d):
    """Raise ValueError when the pair breaks a hard constraint of the prompt (or cannot be drawn)."""
    if p['m'] <= 0 or p['b'] <= 0 or d <= 0:
        raise ValueError('m, b and d must be positive')
    if p['z1'] < 3 or p['z2'] < 3:
        raise ValueError('z1 and z2 must be at least 3')
    if p['b'] > p['R'] / 3.0 + 1e-9:
        raise ValueError('face width b = %.2f mm is more than R / 3 = %.2f mm (R = %.3f mm): refused'
                         % (p['b'], p['R'] / 3.0, p['R']))
    if p['zv1'] < p['zv_min'] - 1e-9:
        raise ValueError('the pinion has z_v1 = z1 / cos(delta1) = %.2f virtual teeth, fewer than 2 / sin^2(alpha)'
                         ' = %.2f, so it would be undercut: refused' % (p['zv1'], p['zv_min']))
    for which, name in ((1, 'pinion'), (2, 'gear')):
        tooth_2d(p, which)
        bl = blank(p, which, d)
        if bl['r_back'] <= bl['r_hub'] or bl['r_toe'] <= bl['r_bore'] + 0.5:
            raise ValueError('the shaft (d = %.1f mm) is too large for the %s\'s blank' % (d, name))


# ---------------------------------------------------------------- Fusion helpers
def _P(q):
    return adsk.core.Point3D.create(q[0] * CM, q[1] * CM, q[2] * CM)


def _frame(z_axis, x_axis):
    """Matrix3D of a component at the apex, its own z along z_axis and its own x along x_axis (world)."""
    y = (z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1], z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
         z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0])
    mat = adsk.core.Matrix3D.create()
    mat.setWithCoordinateSystem(adsk.core.Point3D.create(0, 0, 0), adsk.core.Vector3D.create(*x_axis),
                                adsk.core.Vector3D.create(*y), adsk.core.Vector3D.create(*z_axis))
    return mat


def _new_occ(root, name, mat, grounded):
    occ = root.occurrences.addNewComponent(mat)
    occ.component.name = name
    occ.isGrounded = grounded
    return occ


def _base_body(comp, tb, name):
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tb, bf)
    bf.finishEdit()
    body = bf.bodies.item(0)
    body.name = name
    return body


def _tcone(a0, r0, a1, r1):
    """A temporary cylinder or cone on the part's own z axis, from a0 (radius r0) to a1 (radius r1), mm."""
    return adsk.fusion.TemporaryBRepManager.get().createCylinderOrCone(_P((0, 0, a0)), r0 * CM, _P((0, 0, a1)),
                                                                        r1 * CM)


def _blank_body(bl):
    tbm = adsk.fusion.TemporaryBRepManager.get()
    U, D = adsk.fusion.BooleanTypes.UnionBooleanType, adsk.fusion.BooleanTypes.DifferenceBooleanType
    body = _tcone(bl['a_toe'], bl['r_toe'], bl['a_heel'], bl['r_heel'])                 # root cone, toe to heel
    for tool, kind in ((_tcone(bl['a_heel'], bl['r_heel'], bl['a_back'], bl['r_back']), U),   # back cone
                       (_tcone(bl['a_back'], bl['r_hub'], bl['a_hub'], bl['r_hub']), U),     # back face and hub
                       (_tcone(bl['a_toe'] - 1.0, bl['r_bore'], bl['a_hub'] + 1.0, bl['r_bore']), D)):   # bore
        if not tbm.booleanOperation(body, tool, kind):
            raise RuntimeError('a boolean of the blank failed')
    return body


def _sketch_loop(sk, pts, what):
    sp = [sk.modelToSketchSpace(_P(q)) for q in pts]
    off = max(abs(q.z) for q in sp)
    if off > 1e-4:
        raise RuntimeError('the %s is %.4f mm off its plane' % (what, off / CM))
    for q in sp:
        q.z = 0.0
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(sp[0], sp[1])
    prev = first.endSketchPoint
    for q in sp[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    if sk.profiles.count != 1:
        raise RuntimeError('the %s gave %d profiles, not 1' % (what, sk.profiles.count))
    return sk.profiles.item(0)


def _teeth(comp, body, p, which):
    """Tooth 0 lofted from its heel outline to its toe outline (the heel scaled by k towards the apex), joined to
    the blank, then patterned round the part's own z axis."""
    dl = _cone(p, which)
    R, k = p['R'], p['k']
    z = p['z1'] if which == 1 else p['z2']
    bz = R / math.cos(dl)
    apex_b = (0.0, 0.0, bz)                                            # the back cone's apex
    pitch = (R * math.sin(dl), 0.0, R * math.cos(dl))                  # the pitch point at the heel
    traces = comp.sketches.add(comp.xZConstructionPlane)
    traces.name = 'Heel and toe lines'
    tl = traces.sketchCurves.sketchLines
    lines = [tl.addByTwoPoints(traces.modelToSketchSpace(_P([f * v for v in apex_b])),
                               traces.modelToSketchSpace(_P([f * v for v in pitch]))) for f in (1.0, k)]
    heel = heel_outline(p, which)
    profiles = []
    for line, pts, what in ((lines[0], heel, 'heel outline'),
                            (lines[1], [(k * x, k * y, k * w) for x, y, w in heel], 'toe outline')):
        ci = comp.constructionPlanes.createInput()
        ci.setByAngle(line, adsk.core.ValueInput.createByString('90 deg'), comp.xZConstructionPlane)
        plane = comp.constructionPlanes.add(ci)
        plane.name = what.replace('outline', 'plane').capitalize()
        sk = comp.sketches.add(plane)
        sk.name = what.capitalize()
        profiles.append(_sketch_loop(sk, pts, what))
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.JoinFeatureOperation)
    li.loftSections.add(profiles[0])
    li.loftSections.add(profiles[1])
    li.isSolid = True
    try:
        li.participantBodies = [body]
    except Exception:
        pass                                   # the component has one body, which a join takes anyway
    tooth = lofts.add(li)
    feats = adsk.core.ObjectCollection.create()
    feats.add(tooth)
    pats = comp.features.circularPatternFeatures
    pi = pats.createInput(feats, comp.zConstructionAxis)
    pi.quantity = adsk.core.ValueInput.createByReal(z)
    pi.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    pi.isSymmetric = False
    pattern = pats.add(pi)
    healthy = adsk.fusion.FeatureHealthStates.HealthyFeatureHealthState
    for f, what in ((tooth, 'tooth loft'), (pattern, 'tooth pattern')):
        if f.healthState != healthy:
            raise RuntimeError('the %s of the %s is not healthy: %s' % (what, comp.name, f.errorOrWarningMessage))
    if comp.bRepBodies.count != 1:
        raise RuntimeError('the %s has %d bodies, not 1' % (comp.name, comp.bRepBodies.count))
    comp.bRepBodies.item(0).name = comp.name


def _rot(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _turn_deg(R1, R0, axis):
    """The angle (deg) of the rotation R1 R0^T about a world axis, right-handed."""
    D = [[sum(R1[a][c] * R0[b][c] for c in range(3)) for b in range(3)] for a in range(3)]
    w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
    s = sum(w[i] * axis[i] for i in range(3)) / 2.0
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.degrees(math.atan2(s, c))


def _set(joint, deg):
    joint.jointMotion.rotationValue = math.radians(deg)
    adsk.doEvents()


def _revolute(root, mover, holder, axis, name, test=30.0):
    """An As-built revolute joint of `mover` on `holder` at the apex. Each joint direction is tried; the one kept
    really turns `mover` about the world `axis`. Returns the joint and the sign of the turn for a positive value."""
    R0 = _rot(mover)
    tried = []
    J = adsk.fusion.JointDirections
    for label, direction in (('Z', J.ZAxisJointDirection), ('X', J.XAxisJointDirection),
                             ('Y', J.YAxisJointDirection)):
        point = mover.component.originConstructionPoint.createForAssemblyContext(mover)
        ji = root.asBuiltJoints.createInput(mover, holder, adsk.fusion.JointGeometry.createByPoint(point))
        ji.setAsRevoluteJointMotion(direction)
        joint = root.asBuiltJoints.add(ji)
        _set(joint, test)
        turn = _turn_deg(_rot(mover), R0, axis)
        _set(joint, 0.0)
        tried.append('%s: %.3f deg' % (label, turn))
        if abs(abs(turn) - test) < 1e-3:
            joint.name = name
            return joint, (1 if turn > 0 else -1)
        joint.deleteMe()
    raise RuntimeError('no joint direction turns the %s about its axis (%s)' % (mover.component.name,
                                                                              ', '.join(tried)))


def _turns(root, jp, pinion, gear, deg):
    """Drive the pinion's joint to `deg`: the turns (deg) of the pinion about +Z and the gear about +X."""
    Rp, Rg = _rot(pinion), _rot(gear)
    _set(jp, deg)
    tp = _turn_deg(_rot(pinion), Rp, (0, 0, 1))
    tg = _turn_deg(_rot(gear), Rg, (1, 0, 0))
    _set(jp, 0.0)
    return tp, tg


def _motion_link(root, jp, jg, sp, sg, pinion, gear, z1, z2):
    """A Motion Link between the two joints at the ratio of the teeth (pinion z2 deg, gear z1 deg). The direction:
    the gear must turn about +X against the pinion's turn about +Z, so the link is reversed when the two joints
    measured the same sign. Checked by driving the pinion; turned round if the measurement says so."""
    first = sp * sg > 0
    seen = []
    for reverse in (first, not first):
        li = root.motionLinks.createInput(jp, jg)
        li.valueOne = adsk.core.ValueInput.createByString('%d deg' % z2)
        li.valueTwo = adsk.core.ValueInput.createByString('%d deg' % z1)
        li.isReversed = reverse
        link = root.motionLinks.add(li)
        tp, tg = _turns(root, jp, pinion, gear, 18.0)
        seen.append('reversed %s: pinion %.3f, gear %.3f' % (reverse, tp, tg))
        if abs(abs(tp) - 18.0) < 1e-3 and abs(tg + tp * z1 / float(z2)) < 1e-3:
            try:
                link.name = 'Pinion drives gear'
            except Exception:
                pass
            return link
        link.deleteMe()
    raise RuntimeError('the Motion Link does not turn the gear at -z1/z2 of the pinion (%s)' % '; '.join(seen))


# ---------------------------------------------------------------- the build
def build(m=2.0, z1=20, z2=40, alpha=20.0, b=12.0, d=8.0):
    """Check the inputs (ValueError when refused), then build the pair in a new design. Returns the message."""
    z1, z2 = int(round(z1)), int(round(z2))
    p = pair(float(m), z1, z2, float(alpha), float(b))
    refuse(p, float(d))
    d = float(d)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent

    psi = math.pi / z2                                        # half a tooth of the gear
    gear_x = (0.0, -math.sin(psi), math.cos(psi))             # world +Z turned half a tooth about +X
    shaft1 = _new_occ(root, 'Shaft 1', _frame((0, 0, 1), (1, 0, 0)), True)
    pinion = _new_occ(root, 'Pinion', _frame((0, 0, 1), (1, 0, 0)), False)
    shaft2 = _new_occ(root, 'Shaft 2', _frame((1, 0, 0), (0, 0, 1)), True)
    gear = _new_occ(root, 'Gear', _frame((1, 0, 0), gear_x), False)

    for which, occ, shaft in ((1, pinion, shaft1), (2, gear, shaft2)):
        bl = blank(p, which, d)
        body = _base_body(occ.component, _blank_body(bl), occ.component.name)
        _teeth(occ.component, body, p, which)
        _base_body(shaft.component, _tcone(bl['a_shaft0'], d / 2.0, bl['a_shaft1'], d / 2.0), shaft.component.name)

    jp, sp = _revolute(root, pinion, shaft1, (0, 0, 1), 'Pinion turns')
    jg, sg = _revolute(root, gear, shaft2, (1, 0, 0), 'Gear turns')
    _motion_link(root, jp, jg, sp, sg, pinion, gear, z1, z2)
    tp, tg = _turns(root, jp, pinion, gear, 90.0 * sp)       # +90 deg about +Z, measured on the bodies
    app.activeViewport.fit()

    return ('Straight bevel pair: z1 = %d, z2 = %d, m = %g mm, alpha = %g deg, b = %g mm, d = %g mm\n'
            'pitch cone angles: delta1 = %.3f deg, delta2 = %.3f deg\n'
            'cone distance R = %.3f mm (b / R = %.3f, k = %.4f)\n'
            'virtual teeth: z_v1 = %.2f, z_v2 = %.2f\n'
            'contact ratio of the virtual spur gears = %.3f, ratio z2 / z1 = %.3f\n'
            'pinion driven %.1f deg about +Z: gear %.3f deg about +X\n'
            '%s'
            % (z1, z2, p['m'], p['alpha_deg'], p['b'], d, math.degrees(p['d1']), math.degrees(p['d2']), p['R'],
               p['b'] / p['R'], p['k'], p['zv1'], p['zv2'], p['eps'], p['ratio'], tp, tg, VERSION))


# ---------------------------------------------------------------- the dialog
_handlers = []
_state = {'standalone': False}


def _values(ins):
    return dict(m=ins.itemById('m').value / CM, z1=ins.itemById('z1').value, z2=ins.itemById('z2').value,
                alpha=math.degrees(ins.itemById('alpha').value), b=ins.itemById('b').value / CM,
                d=ins.itemById('d').value / CM)


def _derived_text(v):
    try:
        p = pair(v['m'], int(v['z1']), int(v['z2']), v['alpha'], v['b'])
        refuse(p, v['d'])
        return ('delta1 = %.3f deg, delta2 = %.3f deg<br>R = %.3f mm, R / 3 = %.2f mm, k = %.4f<br>'
                'z_v1 = %.2f, z_v2 = %.2f<br>contact ratio = %.3f, ratio = %.3f'
                % (math.degrees(p['d1']), math.degrees(p['d2']), p['R'], p['R'] / 3.0, p['k'], p['zv1'], p['zv2'],
                   p['eps'], p['ratio']))
    except ValueError as e:
        return 'Refused: %s' % e
    except Exception:
        return 'Not a valid pair'


class _Changed(adsk.core.InputChangedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            ins = args.firingEvent.sender.commandInputs
            ins.itemById('derived').formattedText = _derived_text(_values(ins))
        except Exception:
            pass


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            msg = build(**_values(args.command.commandInputs))
            ui.messageBox(msg, 'Bevel pair')
        except ValueError as e:
            ui.messageBox('Refused: %s\n%s' % (e, VERSION), 'Bevel pair')
        except Exception:
            ui.messageBox('BevelPair failed:\n%s\n%s' % (traceback.format_exc(), VERSION), 'Bevel pair')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if _state['standalone']:
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        try:
            cmd = args.command
            ins = cmd.commandInputs
            VS = adsk.core.ValueInput.createByString
            ins.addValueInput('m', 'Module m at the heel', 'mm', VS('%g mm' % DEFAULTS['m']))
            ins.addIntegerSpinnerCommandInput('z1', 'Pinion teeth z1', 3, 400, 1, DEFAULTS['z1'])
            ins.addIntegerSpinnerCommandInput('z2', 'Gear teeth z2', 3, 400, 1, DEFAULTS['z2'])
            ins.addValueInput('alpha', 'Pressure angle alpha', 'deg', VS('%g deg' % DEFAULTS['alpha']))
            ins.addValueInput('b', 'Face width b', 'mm', VS('%g mm' % DEFAULTS['b']))
            ins.addValueInput('d', 'Shaft diameter d', 'mm', VS('%g mm' % DEFAULTS['d']))
            ins.addTextBoxCommandInput('derived', 'Derived', _derived_text(dict(DEFAULTS)), 5, True)
            for event, handler in ((cmd.execute, _Execute()), (cmd.destroy, _Destroy()),
                                   (cmd.inputChanged, _Changed())):
                event.add(handler)
                _handlers.append(handler)
        except Exception:
            ui = adsk.core.Application.get().userInterface
            ui.messageBox('BevelPair failed:\n%s\n%s' % (traceback.format_exc(), VERSION))


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        _state['standalone'] = context is not None
        old = ui.commandDefinitions.itemById(CMD_ID)
        if old:
            old.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Bevel pair', 'A pair of straight bevel gears')
        created = _Created()
        cd.commandCreated.add(created)
        _handlers.append(created)
        cd.execute()
        if _state['standalone']:
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('BevelPair failed:\n%s\n%s' % (traceback.format_exc(), VERSION))


def stop(context):
    try:
        cd = adsk.core.Application.get().userInterface.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
    except Exception:
        pass
