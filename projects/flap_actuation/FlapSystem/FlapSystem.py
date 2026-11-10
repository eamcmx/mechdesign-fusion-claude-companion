# =============================================================================
# FlapSystem.py  -  Chapter 34 of "Mechanical Design in Autodesk Fusion with Claude": the flap actuation system put
# together. The actuator of Chapter 33 (FlapActuator.py, loaded, not copied) hangs on its trunnions in a fitting on
# the rear spar; its rod end is pinned in the fork of the horn on the flap's middle; the flap turns on two hinge
# brackets bolted to ribs inside the wing and out through the cove. The wing's spar and skins are drawn as the brief's section, not designed. The sizing is
# lib/flap_design.system_design(); the parts common/lib/mdfc_parts.py. Written from PROMPT.txt beside it; checked by
# checks/flap_system_check.py.
#
#   INPUTS        the dialog: the screw's operating factor f_p (passed to the actuator)
#   DERIVED       flap_design.system_design(): the flap's forces, the hinges, the horn, the fitting; SYS positions;
#                 linkage(): the actuator's swing and length for each flap angle
#   HARD RULES    build() refuses what the actuator refuses, and the horn's root or cheeks, a pin, a bracket or its
#                 eye, or a lug of the fitting under 2 against yield at the ultimate load
#   CONVENTIONS   the hinge line on X (along the span, the flap's middle at x = 0), Y aft, Z up; three subassemblies
#                 (Wing, grounded; Actuator, with its own six; Flap); the flap turns about X, the actuator swings
#                 about its gimbal's axis; pose(delta) sets the flap, the swing and the rod from the linkage
#   ACCEPTANCE    the message; no interference at 0 to 40 deg; the rod end's centre on the horn's pin at each angle
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import flap_design as fd        # noqa: E402
import mdfc_parts as mp         # noqa: E402
importlib.reload(fd)
importlib.reload(mp)


def _load(name, script):
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


ACT = _load('mdfc_flapactuator', 'FlapActuator')

VERSION = 'FlapSystem 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
S = fd.SYS
A = fd.LINK['A']
SPAN = S['span'] / 2


def design(fp=3.0):
    o = ACT.design(fp)
    s = fd.system_design()
    return dict(act=o, sys=s)


def refuse(D):
    ACT.refuse(D['act'])
    s = D['sys']
    short = []
    for name, sig, allow in (('the horn\'s root', s['horn_sigma'], fd.SY_7075),
                             ('the horn\'s pin', s['horn_pin']['sigma'], fd.SY_4140),
                             ('the hinge pins', s['hinge_pin']['sigma'], fd.SY_4140),
                             ('the hinge brackets', s['bracket_sigma'], fd.SY_7075),
                             ('the brackets\' eyes', max(s['bracket_lug'].values()), fd.SY_7075),
                             ('the fitting\'s lugs', max(s['fit_lug'].values()), fd.SY_7075),
                             ('the horn\'s cheeks', max(s['horn_cheek'].values()), fd.SY_7075)):
        if allow / sig < 2.0:
            short.append('%s keep %.2f' % (name, allow / sig))
    if short:
        raise ValueError('The flap system falls short at the ultimate load: %s.' % '; '.join(short))


# ---------- geometry helpers (world mm: X along the span, Y aft, Z up; the hinge on X) ----------
def along_x(y, z, x0, x1, r, r_in=0.0):
    return mp.tube((x0, y, z), (x1, y, z), r, r_in) if r_in > 0 else mp.cyl((x0, y, z), (x1, y, z), r)


def _combine(occ, tool, op):
    """Join or cut a temporary body given in the occurrence's frame into its first body, through a base feature."""
    comp = occ.component
    body = comp.bRepBodies.item(0)
    bf = comp.features.baseFeatures.add()
    bf.startEdit()
    comp.bRepBodies.add(tool, bf)
    bf.finishEdit()
    tools = adsk.core.ObjectCollection.create()
    tools.add(bf.bodies.item(0))
    ci = comp.features.combineFeatures.createInput(body, tools)
    ci.operation = op
    comp.features.combineFeatures.add(ci)


def cut(occ, *tools):
    for t in tools:
        _combine(occ, t, adsk.fusion.FeatureOperations.CutFeatureOperation)


def join(occ, *tools):
    for t in tools:
        _combine(occ, t, adsk.fusion.FeatureOperations.JoinFeatureOperation)


def _extrude(occ, pts, x0, x1, op):
    """Extrude the polygon pts = [(y, z), ...] from x0 to x1 along the span in the occurrence's component (whose
    frame is the world's, as every part here sits at its parent's origin)."""
    comp = occ.component
    ci = comp.constructionPlanes.createInput()
    ci.setByOffset(comp.yZConstructionPlane, VI(x0 * CM))
    sk = comp.sketches.add(comp.constructionPlanes.add(ci))
    sk.isVisible = False
    sk.isComputeDeferred = True
    model = [sk.modelToSketchSpace(P3(x0 * CM, y * CM, z * CM)) for y, z in pts]
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(model[0], model[1])
    prev = first.endSketchPoint
    for q in model[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    ext = comp.features.extrudeFeatures
    inp = ext.createInput(sk.profiles.item(0), op)
    normal = sk.xDirection.crossProduct(sk.yDirection)
    inp.setDistanceExtent(False, VI((x1 - x0) * CM * (1 if normal.x > 0 else -1)))
    if op != adsk.fusion.FeatureOperations.NewBodyFeatureOperation:
        inp.participantBodies = [comp.bRepBodies.item(0)]
    return ext.add(inp)


def prism(parent, name, pts, x0, x1, material='Aluminum'):
    """A part whose section is the polygon pts = [(y, z), ...], from x0 to x1 along the span, in a component of its
    own at the parent's origin (so its frame is the parent's)."""
    o = mp.new_occ(parent.component, name)
    body = _extrude(o, pts, x0, x1, adsk.fusion.FeatureOperations.NewBodyFeatureOperation).bodies.item(0)
    body.name = name
    mp.set_material(body, material)
    return o


def add_prism(occ, pts, x0, x1):
    _extrude(occ, pts, x0, x1, adsk.fusion.FeatureOperations.JoinFeatureOperation)


def clip(poly, y0, y1, z0, z1):
    """The polygon cut to the box y0..y1, z0..z1 (Sutherland and Hodgman, one side of the box at a time)."""
    def side(pts, inside, cross):
        out = []
        for i, p in enumerate(pts):
            q = pts[i - 1]
            if inside(p):
                if not inside(q):
                    out.append(cross(q, p))
                out.append(p)
            elif inside(q):
                out.append(cross(q, p))
        return out
    def at_y(v):
        return lambda a, b: (v, a[1] + (b[1] - a[1]) * (v - a[0]) / (b[0] - a[0]))
    def at_z(v):
        return lambda a, b: (a[0] + (b[0] - a[0]) * (v - a[1]) / (b[1] - a[1]), v)
    pts = list(poly)
    pts = side(pts, lambda p: p[0] >= y0, at_y(y0))
    pts = side(pts, lambda p: p[0] <= y1, at_y(y1))
    pts = side(pts, lambda p: p[1] >= z0, at_z(z0))
    pts = side(pts, lambda p: p[1] <= z1, at_z(z1))
    return pts


def spherical_bearing(parent, name, x0, y, z, b_out, b_in, D, d, R=14.5):
    """A radial spherical plain bearing on an axis along X through (y, z): the outer ring b_out wide with its
    sphere, the inner ring b_in wide, both centred on x0."""
    o = mp.new_occ(parent.component, name)
    outer = along_x(y, z, x0 - b_out / 2, x0 + b_out / 2, D / 2)
    mp.cut(outer, mp.sphere((x0, y, z), R))
    mp.add_body(o, outer, 'outer ring', 'Steel')
    inner = mp.intersect(mp.sphere((x0, y, z), R - 0.01), mp.box(x0 - b_in / 2, x0 + b_in / 2, y - R, y + R, z - R, z + R))
    mp.cut(inner, along_x(y, z, x0 - b_in, x0 + b_in, d / 2))
    mp.add_body(o, inner, 'inner ring', 'Steel')
    return o


def pin(parent, name, y, z, x0, x1, head):
    """A pin along X from x0 to x1, with a head and a nut, head mm thick, 15 mm in radius. The shank is 19.98 mm, a
    clearance fit in the 20 mm bores of the cheeks and the bearings."""
    p = along_x(y, z, x0 - 0.01, x1, 9.99)
    mp.union(p, along_x(y, z, x0 - head, x0, 15.0))
    o = mp.new_occ(parent.component, name)
    mp.add_body(o, p, name, 'Steel')
    n = mp.new_occ(parent.component, name.replace('pin', 'pin nut'))
    mp.add_body(n, along_x(y, z, x1, x1 + head, 15.0, 10.0), 'nut', 'Steel')
    return o, n


# ---------- the wing: spar, skins, the spar's fitting and the hinge brackets ----------
def wing(root):
    asm = mp.new_occ(root, 'Wing')
    sp = S['spar']
    up, lo = fd.skin_up, fd.skin_low
    web = mp.box(-SPAN - 100, SPAN + 100, sp[0], sp[1], lo(sp[0]), up(sp[0]) + 2.0)
    for bx, bz in S['bolts']:
        mp.cut(web, mp.cyl((bx, sp[0] - 1, bz), (bx, sp[1] + 1, bz), 4.05))
    ACT.part(asm, 'Rear spar web', web, 'Aluminum')
    prism(asm, 'Upper skin', [(sp[1], up(sp[1])), (-60.0, 160.0), (10.0, 150.0), (10.0, 152.0), (-60.0, 162.0),
                              (sp[1], up(sp[1]) + 2.0)], -SPAN - 100, SPAN + 100)
    ls = prism(asm, 'Lower skin', [(sp[1], lo(sp[1])), (-90.0, -55.0), (-90.0, -53.0), (sp[1], lo(sp[1]) + 2.0)],
               -SPAN - 100, SPAN + 100)
    # the spar's fitting: a base plate on the web's aft face and two lugs round the trunnions' bearings
    b0, b1 = S['base']
    fit = mp.box(-S['base_x'], S['base_x'], b0, b1, S['base_z'][0], S['base_z'][1])
    for s in (1, -1):
        l0, l1 = sorted((s * S['lug'][0], s * S['lug'][1]))
        lugb = mp.box(l0, l1, b1 - 0.01, A[0], A[1] - S['lug_eye_r'], A[1] + S['lug_eye_r'])
        mp.union(lugb, mp.cyl((l0, A[0], A[1]), (l1, A[0], A[1]), S['lug_eye_r']))
        mp.cut(lugb, mp.cyl((l0 - 1, A[0], A[1]), (l1 + 1, A[0], A[1]), fd.GE20['D'] / 2))
        mp.union(fit, lugb)
    for bx, bz in S['bolts']:
        mp.cut(fit, mp.cyl((bx, b0 - 1, bz), (bx, b1 + 1, bz), 4.05))
    ACT.part(asm, 'Spar fitting', fit, 'Aluminum')
    for s, tag in ((1, 'R'), (-1, 'L')):
        spherical_bearing(asm, 'Bearing GE 20 ES fitting %s' % tag, s * 0.5 * (S['lug'][0] + S['lug'][1]), A[0], A[1],
                          fd.GE20['C_out'], fd.GE20['B'], fd.GE20['D'], fd.GE20['d'])
    for k, (bx, bz) in enumerate(S['bolts']):
        ACT.part(asm, 'Fitting bolt M8x35 %d' % (k + 1), mp.cap_screw(8.0, 35.0, (bx, b1, bz), (0, -1, 0)))
        ACT.part(asm, 'Fitting nut M8 %d' % (k + 1), mp.nut(8.0, (bx, sp[0], bz), (0, -1, 0)))
    # the hinge brackets: a plate bolted to the side of a trailing-edge rib inside the wing, standing out of the wing
    # through the cove, aft of the lower skin's edge, to an eye round the hinge's bearing
    for h, tag in zip(S['hinges'], ('L', 'R')):
        t = S['bracket_t'] / 2
        xa, xb = S['bracket_at']
        za, zb, ztop = lo(xa) + S['rib_gap'], lo(xb) + S['rib_gap'], S['bracket_top']
        xm, zm = 0.5 * (xa + xb), 0.5 * (za + ztop)
        dl = math.hypot(xm, zm)
        pv = (zm / dl, -xm / dl)                                   # square to the bracket's line, up and forwards
        if pv[1] < 0:
            pv = (-pv[0], -pv[1])
        er = S['eye_r']
        pl = [(xa, za), (xb, zb), (-er * pv[0], -er * pv[1]), (er * pv[0], er * pv[1]), (xa, ztop)]
        br = prism(asm, 'Hinge bracket %s' % tag, pl, h - t, h + t)
        join(br, along_x(0.0, 0.0, h - t, h + t, er))
        cut(br, along_x(0.0, 0.0, h - t - 1, h + t + 1, fd.GE20['D'] / 2))
        rib = [(S['rib'][0], lo(S['rib'][0]) + 2.0), (S['rib'][1], lo(S['rib'][1]) + 2.0), (S['rib'][1], up(S['rib'][1])),
               (S['rib'][0], up(S['rib'][0]))]
        rb = prism(asm, 'Hinge rib %s' % tag, rib, h + t, h + t + S['rib_t'])
        for by, bz in S['bracket_bolts']:
            cut(br, mp.cyl((h - t - 1, by, bz), (h + t + 1, by, bz), 4.05))
            cut(rb, mp.cyl((h + t - 1, by, bz), (h + t + S['rib_t'] + 1, by, bz), 4.05))
        spherical_bearing(asm, 'Bearing GE 20 ES hinge %s' % tag, h, 0.0, 0.0, fd.GE20['C_out'], fd.GE20['B'],
                          fd.GE20['D'], fd.GE20['d'])
        for k, (by, bz) in enumerate(S['bracket_bolts']):
            ACT.part(asm, 'Bracket bolt M8x25 %s%d' % (tag, k + 1), mp.cap_screw(8.0, 25.0, (h - t, by, bz), (1, 0, 0)))
            ACT.part(asm, 'Bracket nut M8 %s%d' % (tag, k + 1), mp.nut(8.0, (h + t + S['rib_t'], by, bz), (1, 0, 0)))
    return asm


# ---------- the flap, its horn and its hinge fittings ----------
def flap(root):
    asm = mp.new_occ(root, 'Flap')
    outline = [(x, z) for x, z in fd.flap_outline(0.0)]
    fp = prism(asm, 'Flap panel', outline, -SPAN, SPAN)
    hs = S['horn_slot']
    slot_h = mp.box(-hs, hs, -45.0, 45.0, -45.0, 45.0)
    cut(fp, slot_h)
    for h in S['hinges']:
        cut(fp, mp.box(h - S['slot'], h + S['slot'], -45.0, 45.0, -45.0, 45.0))
    # the horn: a forged fork, its root filling the flap's slot, two cheeks out to the eye round the pin
    tip = fd.linkage(0.0)['tip']
    L_ = math.hypot(*tip)
    dh = (-tip[0] / L_, -tip[1] / L_)                     # from the tip towards the hinge
    ph = (-dh[1], dh[0])
    q = (tip[0] + S['horn_neck'][0] * dh[0], tip[1] + S['horn_neck'][0] * dh[1])
    r0, rn, re = S['horn_root'] / 2, S['horn_neck'][1], S['horn_eye_r']
    cheek = [(r0 * ph[0], r0 * ph[1]), (q[0] + rn * ph[0], q[1] + rn * ph[1]), (tip[0] + re * ph[0], tip[1] + re * ph[1]),
             (tip[0] - re * ph[0], tip[1] - re * ph[1]), (q[0] - rn * ph[0], q[1] - rn * ph[1]), (-r0 * ph[0], -r0 * ph[1])]
    c0, c1 = S['horn_cheek']
    horn = prism(asm, 'Horn', clip(outline, -45.0, 45.0, -45.0, 45.0), -hs, hs)   # the root: what the slot took out
    add_prism(horn, cheek, c0, c1)
    add_prism(horn, cheek, -c1, -c0)
    join(horn, along_x(tip[0], tip[1], c0, c1, re), along_x(tip[0], tip[1], -c1, -c0, re))
    cut(horn, along_x(tip[0], tip[1], -c1 - 1, c1 + 1, 10.0))
    for s, tag in ((1, 'R'), (-1, 'L')):
        a, b = sorted((s * S['spacer'][0], s * S['spacer'][1]))
        ACT.part(asm, 'Spacer %s' % tag, along_x(tip[0], tip[1], a, b, 14.0, 10.0))
    pin(asm, 'Horn pin', tip[0], tip[1], -c1, c1, S['horn_head'][1] - S['horn_head'][0])
    # the hinge fittings: two cheeks at each hinge, round the bracket's bearing, and the hinge pin
    k0, k1 = S['cheek']
    re_h = S['hinge_eye_r']
    for h, tag in zip(S['hinges'], ('L', 'R')):
        for s, side in ((1, 'o'), (-1, 'i')):
            a, b = sorted((h + s * k0, h + s * k1))
            ch = prism(asm, 'Hinge fitting %s%s' % (tag, side), [(0.0, re_h), (S['hinge_arm'], re_h - 5.0),
                                                                 (S['hinge_arm'], -re_h + 5.0), (0.0, -re_h)], a, b)
            join(ch, along_x(0.0, 0.0, a, b, re_h))
            cut(ch, along_x(0.0, 0.0, a - 1, b + 1, 10.0))
        pin(asm, 'Hinge pin %s' % tag, 0.0, 0.0, h - k1, h + k1, S['pin_head'][1] - S['pin_head'][0])
    return asm, tip


# ---------- joints ----------
def revolute_x(root, moving, fixed, point, name):
    """An As-built revolute joint about the line through `point` parallel to X; the direction that turns the part
    about X is kept, and its sign returned."""
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.XAxisJointDirection, Jd.YAxisJointDirection, Jd.ZAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        loc = point
        if moving.assemblyContext is None:
            t = moving.transform2.translation
            loc = (point[0] - t.x * 10, point[1] - t.y * 10, point[2] - t.z * 10)
        pt = sk.sketchPoints.add(P3(*[v * CM for v in loc])).createForAssemblyContext(moving)
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = ACT.rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = ACT.rotation(moving)
        j.jointMotion.rotationValue = 0.0
        Dm = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        wv = (Dm[2][1] - Dm[1][2], Dm[0][2] - Dm[2][0], Dm[1][0] - Dm[0][1])
        if abs(wv[0]) > 0.02 and abs(wv[1]) < 0.02 and abs(wv[2]) < 0.02:
            j.name = name
            return j, (1 if wv[0] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about x' % moving.name)


def actuator(root, o):
    """The actuator of Chapter 33, built by its own script's functions inside one occurrence whose origin is the
    gimbal's axis, its joints and Motion Links remade between the proxies."""
    act = mp.new_occ(root, 'Actuator', (0.0, A[0], A[1]))
    hs, _ = ACT.housing(act.component)
    sc, _, th_G = ACT.screw(act.component)
    rd, _, c = ACT.rod(act.component)
    tr = ACT.train(act.component, th_G)
    for x in (sc, rd, tr['Idler'][0], tr['Pinion'][0], tr['Encoder pinion'][0]):
        x.isGroundToParent = False
    px = {x.component.name: x for x in act.childOccurrences}
    H = px['Housing']
    Lz = ACT.L
    js, ss = ACT.revolute_y(root, px['Screw'], H, (0, 0, 0), 'Screw turns')
    ji, si = ACT.revolute_y(root, px['Idler'], H, (0, 0, Lz['z_I']), 'Idler turns')
    jp, sp = ACT.revolute_y(root, px['Pinion'], H, (0, 0, Lz['z_P']), 'Pinion turns')
    je, se = ACT.revolute_y(root, px['Encoder pinion'], H, (ACT.SENSOR[0], 0, ACT.SENSOR[1]), 'Encoder pinion turns')
    jr, sr = ACT.slider_y(root, px['Rod'], H, 'Rod slides')
    zG, G = ACT.G['z_G'], ACT.G
    ACT.link(root, js, ss, 360.0, ji, si, 360.0 * zG / G['z_I'], False, 'Gear to idler 54:30')
    ACT.link(root, js, ss, 360.0, jp, sp, 360.0 * zG / G['z_P'], True, 'Gear to pinion 54:18')
    ACT.link(root, js, ss, 360.0, je, se, 360.0 * zG / G['z_S'], False, 'Gear to encoder pinion 54:26')
    li = root.motionLinks.createInput(js, jr)
    li.valueOne = VS('360 deg')
    li.valueTwo = VI(10.0 * CM)
    li.isReversed = (ss * sr) > 0
    root.motionLinks.add(li).name = 'Lead 10 mm'
    return act, jr, sr


def angles(delta):
    """The actuator's angle (deg, from +Y towards +Z about the gimbal) and its rod's travel for the flap at delta."""
    k = fd.linkage(delta)
    return math.degrees(math.atan2(k['u'][1], k['u'][0])), k['length'] - fd.linkage(0.0)['length']


def place_actuator(act, angle):
    """The actuator turned by `angle` degrees about the gimbal's axis (X through A), from +Y towards +Z. Set as the
    occurrence's transform, not through a joint: Fusion holds a joint's angle to 0.1 deg, and the actuator swings
    only 0.15 to 0.82 deg."""
    m = adsk.core.Matrix3D.create()
    m.translation = adsk.core.Vector3D.create(0.0, A[0] * CM, A[1] * CM)
    r = adsk.core.Matrix3D.create()
    r.setToRotation(math.radians(angle), adsk.core.Vector3D.create(1, 0, 0), P3(0.0, A[0] * CM, A[1] * CM))
    m.transformBy(r)
    act.transform2 = m


def pose(delta, joints):
    """The flap down by delta, the actuator swung and its rod out as the linkage says: the loop is closed by the
    numbers, and the check measures the gap between the rod end and the horn's pin."""
    jf, sf, act, jr, sr = joints
    sw, out = angles(delta)
    jf.jointMotion.rotationValue = -sf * math.radians(delta)          # trailing edge down: about -X
    place_actuator(act, sw)
    jr.jointMotion.slideValue = sr * out * CM


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        wds = o.component.name.split()
        if wds[-1].isdigit() or wds[-1] in ('L', 'R', 'Lo', 'Li', 'Ro', 'Ri', 'L1', 'L2', 'R1', 'R2'):
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


_last_joints = None                 # the last build's joints and their signs, for pose() from a check


def build(fp=3.0):
    D = design(fp)
    refuse(D)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    wg = wing(root)
    wg.isGrounded = True
    act, jr, sr = actuator(root, D['act'])
    fl, tip = flap(root)
    jf, sf = revolute_x(root, fl, wg, (0, 0, 0), 'Flap turns')
    joints = (jf, sf, act, jr, sr)
    global _last_joints
    _last_joints = joints
    pose(0.0, joints)
    for oc, col in ((wg, 'Paint - Enamel Glossy (Grey)'), (fl, 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(design_, oc, col)
    app.activeViewport.fit()
    s = D['sys']
    pl = parts_list(root)
    return ('Flap system built: the actuator on the spar\'s fitting, its rod end on the horn\'s pin, the flap on two '
            'hinges.\nAt 40 deg and the ultimate: hinges %.0f N each, horn %.1f MPa (factor %.2f), horn pin %.1f MPa, '
            'brackets %.1f MPa, fitting lugs %.1f MPa.\n\nParts: %d components; %s.\n\n[%s]'
            % (s['R_hinge'], s['horn_sigma'], s['horn_n'], s['horn_pin']['sigma'], s['bracket_sigma'],
               max(s['fit_lug'].values()), sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())),
               VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('fp', 'Screw operating factor f_p', 3.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            ui.messageBox(build(**{k: ins.itemById(k).value for k, _, _ in FIELDS}), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('Something went wrong:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        for key, label, dflt in FIELDS:
            ins.addValueInput(key, label, '', VI(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'FlapSystemCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap system', 'The flap actuation system of Chapter 34')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FlapSystem could not start:\n' + traceback.format_exc())
