# =============================================================================
# FlapSystem.py  -  Chapter 34 of "Mechanical Design in Autodesk Fusion with Claude": one flap's actuation system
# as one machine. The actuator of Chapter 33 (FlapActuator.py, loaded and called, not copied) hangs on its
# trunnions in a fitting bolted to the rear spar's aft face; its rod end is pinned in the fork of a horn on the
# flap's middle; the flap turns on two hinge brackets bolted to trailing-edge ribs, each carrying an SKF GE 20 ES on
# the hinge line. The wing's spar and skins are drawn from the brief's section. The sizing is
# lib/flap_design.system_design(); the linkage lib/flap_design.linkage(); the parts common/lib/mdfc_parts.py.
#
#   INPUTS        the dialog: the screw's operating factor f_p, passed to the actuator
#   DERIVED       flap_design.system_design(): the hinges' reaction, the horn, the pins, cheeks, eyes and lugs, the
#                 brackets, their bolts and the ribs, the fitting and its bolts; flap_design.linkage(): the
#                 actuator's angle about its gimbal and its rod's travel; flap_design.SYS: the positions
#   HARD RULES    build() refuses what the actuator refuses, and the horn's root or cheeks, a pin, a bracket or its
#                 eye, or a lug of the fitting under 2 against yield at the ultimate load
#   CONVENTIONS   the hinge line on X along the span, the flap's middle at x = 0, Y aft, Z up; the gimbal at
#                 (y, z) = (-420, 114); three subassemblies: Wing (grounded), Actuator (its own six), Flap; the
#                 flap on an As-built revolute joint about X; the actuator placed by its occurrence's transform
#   ACCEPTANCE    the message; the parts list; at 0 to 40 deg the rod end's centre on the horn's pin and no
#                 interference; both hinges' bearings on the hinge line; f_p 5.0 refused
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
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
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


FA = _load('mdfc_flapactuator', 'FlapActuator')        # the actuator of Chapter 33, loaded, not copied

VERSION = 'FlapSystem 1.0 (2026-11-12)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString
S = fd.SYS
L = fd.LAY
GE = fd.GE20
A = fd.LINK['A']                       # the gimbal (y, z), mm
N_MIN = 2.0                            # the least factor against yield at the ultimate load
DK = 29.0                              # the GE 20 ES's sphere diameter (SKF), mm
T_SKIN = 2.0                           # the skins' thickness, mm (assumed)
PIN_R = 9.98                           # the pins' radius: a clearance fit in their 20 mm bores
UP_END, LOW_END = -60.0, -90.0         # where the skins end at the cove: the ends of skin_up() and skin_low()
HORN_SLOT_END = S['horn_root']         # the horn's slot in the flap's nose runs aft to y = 70
HINGE_SLOT_END = S['hinge_arm']        # the hinges' slots run aft to y = 45, where the hinge fittings end
NUT_T = 6.0                            # the pins' nuts, as thick as the pins' heads

_last_joints = None


# ---------- the numbers ----------
def factors(s):
    """Each part's least factor against yield at the ultimate load: 7075-T6 for the horn, the brackets and the
    fitting (Shigley Table A-22), 4140 for the pins (Table A-21), shear yield 0.577 S_y."""
    sy_al, sy_st = fd.SY_7075, fd.SY_4140

    def pin(p):
        return min(sy_st / p['sigma'], 0.577 * sy_st / p['tau'])

    def lug(q):
        return sy_al / max(q.values())

    return [("the horn's root", sy_al / s['horn_sigma']),
            ("the horn's cheeks", lug(s['horn_cheek'])),
            ('the horn pin', pin(s['horn_pin'])),
            ('the hinge pins', pin(s['hinge_pin'])),
            ('the hinge brackets', s['bracket_n']),
            ("the brackets' eyes", lug(s['bracket_lug'])),
            ("the fitting's lugs", lug(s['fit_lug']))]


def design(fp=3.0):
    act = FA.design(fp)
    s = fd.system_design()
    k0, k1 = fd.linkage(0.0), fd.linkage(40.0)
    return dict(fp=fp, act=act, sys=s, n=factors(s), k0=k0, k1=k1)


def refuse(D):
    short = []
    try:
        FA.refuse(D['act'])
    except ValueError as e:
        short.append(str(e).rstrip('.'))
    low = ['%s: %.2f, %.2f short of %.0f' % (k, n, N_MIN - n, N_MIN) for k, n in D['n'] if n < N_MIN]
    if low:
        short.append('Against yield at the ultimate load, ' + '; '.join(low))
    if short:
        raise ValueError('The flap system is refused. %s.' % '. '.join(short))


# ---------- geometry helpers (world mm; the flap at 0 deg, the actuator along +Y from its gimbal) ----------
def xcyl(x0, x1, y, z, r):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def ycyl(x, z, y0, y1, r):
    return mp.cyl((x, y0, z), (x, y1, z), r)


def prism(pts, x0, x1):
    """A convex polygon in the (y, z) plane drawn from x0 to x1: a box trimmed by one half-space for each edge."""
    clean = []
    for p in pts:
        if not clean or math.hypot(p[0] - clean[-1][0], p[1] - clean[-1][1]) > 1e-6:
            clean.append((float(p[0]), float(p[1])))
    if math.hypot(clean[0][0] - clean[-1][0], clean[0][1] - clean[-1][1]) < 1e-6:
        clean.pop()
    if sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(clean, clean[1:] + clean[:1])) < 0:
        clean.reverse()
    n = len(clean)
    keep = []
    for i in range(n):
        a, b, c = clean[i - 1], clean[i], clean[(i + 1) % n]
        cr = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if cr < -1e-9:
            raise RuntimeError('prism(): the outline is not convex at (%.2f, %.2f)' % b)
        if cr > 1e-9:
            keep.append(b)
    ys, zs = [p[0] for p in keep], [p[1] for p in keep]
    body = mp.box(x0, x1, min(ys) - 1.0, max(ys) + 1.0, min(zs) - 1.0, max(zs) + 1.0)
    W, LG, xm, H = 2000.0, 6000.0, 0.5 * (x0 + x1), (x1 - x0) + 10.0
    for p, q in zip(keep, keep[1:] + keep[:1]):
        dy, dz = q[0] - p[0], q[1] - p[1]
        ln = math.hypot(dy, dz)
        e = (dy / ln, dz / ln)
        nv = (e[1], -e[0])                                       # outward, the outline running anticlockwise
        c = P3(xm * CM, (0.5 * (p[0] + q[0]) + nv[0] * W / 2) * CM, (0.5 * (p[1] + q[1]) + nv[1] * W / 2) * CM)
        obb = adsk.core.OrientedBoundingBox3D.create(c, V3(0, e[0], e[1]), V3(0, nv[0], nv[1]), LG * CM, W * CM, H * CM)
        mp.cut(body, mp.tbm().createBox(obb))
    return body


def ge20(parent, name, x, y, z):
    """An SKF GE 20 ES on an axis along x through (x, y, z): the outer ring C wide with its spherical bore, the inner
    ring B wide, its sphere 0.01 mm smaller, the 20 mm bore."""
    o = mp.new_occ(parent.component, name)
    rs = DK / 2.0
    outer = mp.cut(xcyl(x - GE['C_out'] / 2, x + GE['C_out'] / 2, y, z, GE['D'] / 2), mp.sphere((x, y, z), rs))
    inner = mp.intersect(mp.sphere((x, y, z), rs - 0.01),
                         mp.box(x - GE['B'] / 2, x + GE['B'] / 2, y - rs - 1, y + rs + 1, z - rs - 1, z + rs + 1))
    mp.cut(inner, xcyl(x - GE['B'] / 2 - 1, x + GE['B'] / 2 + 1, y, z, GE['d'] / 2))
    mp.add_body(o, outer, 'Outer ring', 'Steel')
    mp.add_body(o, inner, 'Inner ring', 'Steel')
    return o


def side(x):
    return 'R' if x > 0 else 'L'


# ---------- the wing: spar, skins, the spar's fitting, the hinge brackets and ribs ----------
def wing(occ):
    half = S['span'] / 2.0
    up, lo = fd.skin_up, fd.skin_low
    ys0, ys1 = S['spar']
    FA.part(occ, 'Upper skin', prism([(ys0, up(ys0)), (UP_END, up(UP_END)), (UP_END, up(UP_END) - T_SKIN),
                                      (ys0, up(ys0) - T_SKIN)], -half, half), 'Aluminum')
    FA.part(occ, 'Lower skin', prism([(ys0, lo(ys0)), (LOW_END, lo(LOW_END)), (LOW_END, lo(LOW_END) + T_SKIN),
                                      (ys0, lo(ys0) + T_SKIN)], -half, half), 'Aluminum')
    web = mp.box(-half, half, ys0, ys1, max(lo(ys0), lo(ys1)) + T_SKIN + 0.5, min(up(ys0), up(ys1)) - T_SKIN - 0.5)
    for x, z in S['bolts']:
        mp.cut(web, ycyl(x, z, ys0 - 1.0, ys1 + 1.0, 4.5))
    FA.part(occ, 'Rear spar web', web, 'Aluminum')

    # the spar's fitting: the base plate on the web's aft face, two lugs round the trunnions' bearings
    b0, b1 = S['base']
    ay, az = A
    er = S['lug_eye_r']
    fit = mp.box(-S['base_x'], S['base_x'], b0, b1, S['base_z'][0], S['base_z'][1])
    lugs = []
    for sg in (1, -1):
        xa, xb = sorted((sg * S['lug'][0], sg * S['lug'][1]))
        lugs.append((xa, xb))
        mp.union(fit, mp.box(xa, xb, b1, ay, az - er + 1.0, az + er - 1.0), xcyl(xa, xb, ay, az, er))
    for xa, xb in lugs:
        mp.cut(fit, xcyl(xa - 1.0, xb + 1.0, ay, az, GE['D'] / 2))
    for x, z in S['bolts']:
        mp.cut(fit, ycyl(x, z, b0 - 1.0, b1 + 1.0, 4.5))
    FA.part(occ, 'Spar fitting', fit, 'Aluminum')
    for xa, xb in lugs:
        xc = 0.5 * (xa + xb)
        ge20(occ, 'Bearing GE 20 ES fitting %s' % side(xc), xc, ay, az)
    for k, (x, z) in enumerate(S['bolts']):
        FA.part(occ, 'Fitting bolt M8x35 %d' % (k + 1), mp.cap_screw(8.0, 35.0, (x, b1, z), (0, -1, 0)))
        FA.part(occ, 'Fitting nut M8 %d' % (k + 1), mp.nut(8.0, (x, ys0, z), (0, -1, 0)))

    # the hinges: a bracket bolted to a trailing-edge rib, out through the cove to a GE 20 ES on the hinge line
    hb, rt, g = S['bracket_t'] / 2.0, S['rib_t'], S['rib_gap']
    ya, yb = S['bracket_at']
    z_top = S['bracket_top']
    z_bot = lo(yb) + T_SKIN + g                        # the bracket's foot clear of the lower skin by the rib's gap
    r0, r1 = S['rib']
    nb = 0
    for xh in S['hinges']:
        t = side(xh)
        sg = 1 if xh > 0 else -1                       # outboard
        rx0, rx1 = sorted((xh - sg * hb, xh - sg * (hb + rt)))     # the rib on the bracket's inboard face
        rib = prism([(r0, lo(r0) + T_SKIN + g), (r1, lo(r1) + T_SKIN + g), (r1, up(r1) - T_SKIN - g),
                     (r0, up(r0) - T_SKIN - g)], rx0, rx1)
        br = mp.box(xh - hb, xh + hb, ya, yb, z_bot, z_top)
        ea = S['eye_r'] - 2.0                          # the arm ends inside the eye
        mp.union(br, prism([(yb, z_bot), (0.0, -ea), (0.0, ea), (yb, z_top)], xh - hb, xh + hb),
                 xcyl(xh - hb, xh + hb, 0.0, 0.0, S['eye_r']))
        mp.cut(br, xcyl(xh - hb - 1.0, xh + hb + 1.0, 0.0, 0.0, GE['D'] / 2))
        for y, z in S['bracket_bolts']:
            mp.cut(br, xcyl(xh - hb - 1.0, xh + hb + 1.0, y, z, 4.5))
            mp.cut(rib, xcyl(rx0 - 1.0, rx1 + 1.0, y, z, 4.5))
        FA.part(occ, 'Hinge rib %s' % t, rib, 'Aluminum')
        FA.part(occ, 'Hinge bracket %s' % t, br, 'Aluminum')
        ge20(occ, 'Bearing GE 20 ES hinge %s' % t, xh, 0.0, 0.0)
        for y, z in S['bracket_bolts']:
            nb += 1
            ax = (-sg, 0, 0)
            FA.part(occ, 'Bracket bolt M8x25 %d' % nb, mp.cap_screw(8.0, 25.0, (xh + sg * hb, y, z), ax))
            FA.part(occ, 'Bracket nut M8 %d' % nb, mp.nut(8.0, (xh - sg * (hb + rt), y, z), ax))


# ---------- the flap: the panel, the horn on its middle, the hinge fittings and pins ----------
def horn_root_chord():
    """The horn's root: a chord SYS['horn_root'] long across the flap's nose, from just inside the nose at 150 deg
    to just under the upper surface (inside the outline, so that the cheeks cross its faces cleanly)."""
    p1 = (38.0 * math.cos(math.radians(150.0)), 38.0 * math.sin(math.radians(150.0)))
    zu = lambda y: 40.0 + 2.0 * y / 120.0 - 1.0                   # 1 mm under the upper surface, (0, 40) to (120, 42)
    a, b = 0.0, 120.0
    for _ in range(60):
        m = 0.5 * (a + b)
        if math.hypot(m - p1[0], zu(m) - p1[1]) < S['horn_root']:
            a = m
        else:
            b = m
    return p1, (a, zu(a))


def flap(occ):
    half = S['span'] / 2.0
    out = fd.flap_outline(0.0, 550.0)
    hs = S['horn_slot']
    panel = prism(out, -half, half)
    mp.cut(panel, mp.box(-hs, hs, -60.0, HORN_SLOT_END, -60.0, 60.0))
    for xh in S['hinges']:
        mp.cut(panel, mp.box(xh - S['slot'], xh + S['slot'], -60.0, HINGE_SLOT_END, -60.0, 60.0))
    FA.part(occ, 'Flap panel', panel, 'Aluminum')

    # the horn: a root filling the slot in the nose and two cheeks out to the pin's eye
    th = math.radians(fd.LINK['theta0'])
    ty, tz = fd.LINK['r'] * math.cos(th), fd.LINK['r'] * math.sin(th)
    nv = (math.sin(th), -math.cos(th))
    re = S['horn_eye_r']
    p1, p2 = horn_root_chord()
    horn = prism(out, -hs, hs)
    mp.cut(horn, mp.box(-hs - 1.0, hs + 1.0, HORN_SLOT_END, 700.0, -60.0, 60.0))
    ri = re - 1.5                                                 # the cheeks' sides end inside the eye
    arm = [p1, p2, (ty + ri * nv[0], tz + ri * nv[1]), (ty - ri * nv[0], tz - ri * nv[1])]
    for xa, xb in ((S['horn_cheek'][0], S['horn_cheek'][1]), (-S['horn_cheek'][1], -S['horn_cheek'][0])):
        mp.union(horn, prism(arm, xa, xb), xcyl(xa, xb, ty, tz, re))
    mp.cut(horn, xcyl(-hs - 1.0, hs + 1.0, ty, tz, 10.0))
    FA.part(occ, 'Horn', horn, 'Aluminum')
    h0, h1 = S['horn_head']
    pin = xcyl(-h1 - 0.8, h0, ty, tz, PIN_R)
    mp.union(pin, xcyl(h0, h1, ty, tz, 15.0))
    FA.part(occ, 'Horn pin', pin)
    FA.part(occ, 'Horn pin nut', mp.nut(20.0, (-h0, ty, tz), (-1, 0, 0), NUT_T))
    s0, s1 = S['spacer']
    for xa, xb in ((s0, s1), (-s1, -s0)):
        FA.part(occ, 'Spacer %s' % side(xa), mp.tube((xa, ty, tz), (xb, ty, tz), 14.0, 10.0))

    # the hinge fittings: two cheeks either side of each bracket's bearing, the pin through them
    c0, c1 = S['cheek']
    ph0, ph1 = S['pin_head']
    rh = S['hinge_eye_r']
    for xh in S['hinges']:
        t = side(xh)
        for k, (xa, xb) in enumerate(((xh - c1, xh - c0), (xh + c0, xh + c1))):
            f = mp.box(xa, xb, 0.0, HINGE_SLOT_END, -rh + 1.0, rh - 1.0)
            mp.union(f, xcyl(xa, xb, 0.0, 0.0, rh))
            mp.cut(f, xcyl(xa - 1.0, xb + 1.0, 0.0, 0.0, 10.0))
            FA.part(occ, 'Hinge fitting %s %d' % (t, k + 1), f, 'Aluminum')
        pin = xcyl(xh - ph1 - 0.8, xh + ph0, 0.0, 0.0, PIN_R)
        mp.union(pin, xcyl(xh + ph0, xh + ph1, 0.0, 0.0, 15.0))
        FA.part(occ, 'Hinge pin %s' % t, pin)
        FA.part(occ, 'Hinge pin nut %s' % t, mp.nut(20.0, (xh - ph0, 0.0, 0.0), (-1, 0, 0), NUT_T))
    return (ty, tz)


# ---------- joints ----------
def revolute_x(root, moving, fixed, name):
    """An As-built revolute joint about the hinge line (x through the origin); the direction that really turns the
    part about x is kept, and its sign returned."""
    Jd = adsk.fusion.JointDirections
    for direction in (Jd.XAxisJointDirection, Jd.YAxisJointDirection, Jd.ZAxisJointDirection):
        inp = root.asBuiltJoints.createInput(moving, fixed,
                                             adsk.fusion.JointGeometry.createByPoint(FA._point(moving, (0, 0, 0))))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = FA.rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = FA.rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][k] * r0[c][k] for k in range(3)) for c in range(3)] for a in range(3)]
        wv = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(wv[0]) > 0.02 and abs(wv[1]) < 0.02 and abs(wv[2]) < 0.02:
            j.name = name
            return j, (1 if wv[0] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about x' % moving.name)


def actuator_joints(root, act):
    """FlapActuator's joints and Motion Links, made in the root between the Actuator occurrence's children."""
    ch = {o.component.name: o for o in act.childOccurrences}
    hs = ch['Housing']
    js, ss = FA.revolute_y(root, ch['Screw'], hs, (0, 0, 0), 'Screw turns')
    ji, si = FA.revolute_y(root, ch['Idler'], hs, (0, 0, L['z_I']), 'Idler turns')
    jp, sp = FA.revolute_y(root, ch['Pinion'], hs, (0, 0, L['z_P']), 'Pinion turns')
    je, se = FA.revolute_y(root, ch['Encoder pinion'], hs, (FA.SENSOR[0], 0, FA.SENSOR[1]), 'Encoder pinion turns')
    jr, sr = FA.slider_y(root, ch['Rod'], hs, 'Rod slides')
    G = L['gear']
    FA.link(root, js, ss, 360.0, ji, si, 360.0 * G['z_G'] / G['z_I'], False, 'Gear to idler 54:30')
    FA.link(root, js, ss, 360.0, jp, sp, 360.0 * G['z_G'] / G['z_P'], True, 'Gear to pinion 54:18')
    FA.link(root, js, ss, 360.0, je, se, 360.0 * G['z_G'] / G['z_S'], False, 'Gear to encoder pinion 54:26')
    lead = float(fd.HIWIN[fd.NUT][1])
    li = root.motionLinks.createInput(js, jr)
    li.valueOne = VS('360 deg')                                     # one turn of the screw ...
    li.valueTwo = VI(lead * CM)                                     # ... moves the rod one lead
    li.isReversed = (ss * sr) > 0
    root.motionLinks.add(li).name = 'Lead %d mm' % int(lead)
    return jr, sr


def pose(delta, joints=None):
    """The flap down delta degrees on its joint, the rod's travel on its slider and the actuator turned about its
    gimbal by the exact angle of flap_design.linkage(): the loop closed by the numbers."""
    J = joints or _last_joints
    k = fd.linkage(delta)
    J['flap'].jointMotion.rotationValue = -J['sf'] * math.radians(delta)      # trailing edge down: -delta about +x
    J['rod'].jointMotion.slideValue = J['sr'] * (k['length'] - J['c']) * CM
    phi = math.atan2(k['u'][1], k['u'][0])                                   # the actuator's +y turned towards +z
    c, s = math.cos(phi), math.sin(phi)
    m = adsk.core.Matrix3D.create()
    m.setWithArray([1.0, 0.0, 0.0, 0.0,
                    0.0, c, -s, A[0] * CM,
                    0.0, s, c, A[1] * CM,
                    0.0, 0.0, 0.0, 1.0])
    J['act'].transform2 = m
    try:
        snaps = J['design'].snapshots
        if snaps.hasPendingSnapshot:
            snaps.add()
    except Exception:
        pass
    return dict(delta=delta, phi=math.degrees(phi), travel=k['length'] - J['c'], mu=k['mu'])


def parts_list(root):
    return FA.parts_list(root)


def _n(v):
    return '{:,.0f}'.format(v).replace(',', ' ')


def build(fp=3.0):
    global _last_joints
    D = design(fp)
    refuse(D)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    wg = mp.new_occ(root, 'Wing')
    act = mp.new_occ(root, 'Actuator')
    fl = mp.new_occ(root, 'Flap')
    wg.isGrounded = True
    wing(wg)

    # the actuator, built by FlapActuator's own functions into the Actuator component, at the identity
    ac = act.component
    hs, _ = FA.housing(ac)
    sc, _, th_G = FA.screw(ac)
    rd, _, c = FA.rod(ac)
    tr = FA.train(ac, th_G)
    for o in (sc, rd, tr['Idler'][0], tr['Pinion'][0], tr['Encoder pinion'][0]):
        o.isGroundToParent = False                    # free to turn and slide on their joints; the housing stays
    hs.isGroundToParent = True
    jr, sr = actuator_joints(root, act)
    t0 = D['k0']['length'] - c
    t1 = D['k1']['length'] - c
    lo, hi = sorted((sr * t0 * CM, sr * t1 * CM))
    lim = jr.jointMotion.slideLimits
    lim.isMinimumValueEnabled, lim.minimumValue = True, lo - 0.001
    lim.isMaximumValueEnabled, lim.maximumValue = True, hi + 0.001

    tip = flap(fl)
    jf, sf = revolute_x(root, fl, wg, 'Flap turns')
    rl = jf.jointMotion.rotationLimits
    a0, a1 = sorted((0.0, -sf * math.radians(40.0)))
    rl.isMinimumValueEnabled, rl.minimumValue = True, a0 - 1e-6
    rl.isMaximumValueEnabled, rl.maximumValue = True, a1 + 1e-6

    for oc, col in ((hs, 'Paint - Enamel Glossy (Grey)'), (sc, 'Steel - Satin'), (rd, 'Paint - Enamel Glossy (Blue)'),
                    (tr['Idler'][0], 'Paint - Enamel Glossy (Yellow)'), (tr['Pinion'][0], 'Paint - Enamel Glossy (Red)'),
                    (tr['Encoder pinion'][0], 'Paint - Enamel Glossy (Yellow)'), (fl, 'Paint - Enamel Glossy (Yellow)')):
        mp.colour(des, oc, col)
    _last_joints = dict(flap=jf, sf=sf, rod=jr, sr=sr, act=act, c=c, design=des, tip=tip)
    pose(0.0, _last_joints)
    app.activeViewport.fit()
    swing = [math.degrees(math.atan2(k['u'][1], k['u'][0])) for k in (fd.linkage(4.0 * i) for i in range(11))]

    s, a, n = D['sys'], D['act'], dict(D['n'])
    pl = parts_list(root)
    return ('Flap actuation system built: one flap, 0 to 40 deg, the actuator at f_p %.2f (its nut lasts %.2f times '
            'the %d strokes).\n'
            'At the ultimate load (40 deg): the actuator pushes %s N; the hinges %s N each.\n'
            "The horn: its root %.1f MPa (%.2f against yield), its cheeks %.1f MPa (%.2f). The horn pin %.1f MPa in "
            'bending, %.1f MPa in shear (%.2f).\n'
            'The hinges: the pins %.1f MPa (%.2f); the brackets %.1f MPa (%.2f), their eyes %.1f MPa (%.2f); the '
            'bracket bolts %.1f MPa in shear (%.2f), the ribs %.1f MPa in bearing (%.2f).\n'
            "The spar's fitting: the lugs %.1f MPa (%.2f); its bolts %s N each if the push reversed (proof %s N).\n"
            'The linkage: rod travel %.2f mm; the actuator %.2f to %.2f deg about its gimbal; transmission angle '
            '%.2f deg flap up, %.2f deg at 40.\n\n'
            'Parts: %d components in %d kinds; %s.\n\n[%s]'
            % (D['fp'], a['nut']['life'], fd.STROKES, _n(s['ult']['F']), _n(s['R_hinge']),
               s['horn_sigma'], n["the horn's root"], max(s['horn_cheek'].values()), n["the horn's cheeks"],
               s['horn_pin']['sigma'], s['horn_pin']['tau'], n['the horn pin'],
               s['hinge_pin']['sigma'], n['the hinge pins'], s['bracket_sigma'], s['bracket_n'],
               max(s['bracket_lug'].values()), n["the brackets' eyes"], s['bracket_bolt_tau'], s['bracket_bolt_n'],
               s['rib_bearing'], s['rib_bearing_n'],
               max(s['fit_lug'].values()), n["the fitting's lugs"], _n(s['fit_bolt_load']), _n(s['fit_bolt_proof']),
               D['k1']['length'] - D['k0']['length'], min(swing), max(swing),
               D['k0']['mu'], D['k1']['mu'],
               sum(pl.values()), len(pl), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


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
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Flap system', "One flap's actuation system, Chapter 34")
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('FlapSystem could not start:\n' + traceback.format_exc())
