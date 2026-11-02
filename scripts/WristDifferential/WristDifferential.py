# =============================================================================
# WristDifferential.py  -  Chapter 39 of "Mechanical Design in Autodesk Fusion with Claude": the pitch-roll wrist. A
# miter bevel differential: two side gears on the pitch axis, each with a 36-groove HTD 5M pulley cut on its hub and
# running on two 61902 bearings on a trunnion; an output gear on the roll shaft, which turns in the pitch carrier on
# two 6002 bearings; the carrier's trunnions turn in the frame's cheeks on two more 6002s. Each side gear is driven by
# a 350-5M belt from an 18-groove pulley on a servo motor's 15:1 planetary gearhead; the two motors lie side by side
# under the carrier. Every size comes from lib/wrist_design.py. Written from PROMPT.txt beside it; checked by
# checks/wrist_check.py.
#
#   INPUTS        the dialog: the bevel set's module, teeth and face width
#   DERIVED       wrist_design.layout(): every axial position from the gear set and the bearings; the belt's centre
#                 distance from its stock length; the duty and the stall ratings of the teeth
#   HARD RULES    build() refuses a set whose face breaks Table 13-3, whose toe leaves the carrier no room, whose
#                 teeth fail the duty or the drive's torque limit, or which is not the 40-tooth, 60 mm set the
#                 layout and the geometry factors were drawn for
#   CONVENTIONS   origin at the bevel gears' apex, y along the pitch axis, z along the roll axis at rest, x forward;
#                 L = +y (as in Chapter 31). Groups: Frame (grounded), Carrier (pitch), Roll output (on the carrier),
#                 Input L and Input R (on the carrier), Motor L and Motor R outputs, each on a body-less belt helper
#                 that turns with twice the pitch. Motion links: thL = phi - psi, thR = phi + psi on the carrier;
#                 each motor turns 2 (phi -+ psi)
#   ACCEPTANCE    the message; the links; no interference at rest and at eight poses; the masses measured
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import math
import os
import sys
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
LIB = os.path.join(os.path.dirname(os.path.dirname(HERE)), 'lib')
if LIB not in sys.path:
    sys.path.insert(0, LIB)
import wrist_design as wd       # noqa: E402
importlib.reload(wd)

VERSION = 'WristDifferential 1.0 (2026-11-02)'
MM = 0.1
BACKLASH = 0.15                 # mm at the heel, modelled (the real set is shimmed to less: Chapter 40)
_TBM = None


def tbm():
    global _TBM
    if _TBM is None:
        _TBM = adsk.fusion.TemporaryBRepManager.get()
    return _TBM


def P(x, y, z):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def V(x, y, z):
    n = math.sqrt(x * x + y * y + z * z)
    return adsk.core.Vector3D.create(x / n, y / n, z / n)


# ---------- the design and its rules ----------
def design(module=1.5, teeth=40, face=11.0):
    m, z, b = float(module), int(round(teeth)), float(face)
    o = dict(m=m, z=z, b=b)
    o['g'] = g = wd.bevel(m, z, b)
    o['space'] = wd.space()
    o['L'] = wd.layout(dict(m=m, z=z, b=b)) if z == 40 and abs(m * z - 60.0) < 1e-6 else None
    o['loads'] = ld = wd.loads()
    o['mot'] = mo = wd.motors()
    mv = wd.moves()
    o['duty'] = wd.rate(g, ld['T_in'], mv['wr'] * 60 / (2 * math.pi), wd.brief()['life_h'] * 3600 / wd.brief()['cycle'])
    o['stall'] = wd.rate(g, mo['stall_in'], 0, 1e3, KA=1.0, credit=False, dynamic=False)
    o['belt'] = wd.belt()
    o['shaft'] = wd.roll_shaft()
    return o


def refuse(o):
    g, sp = o['g'], o['space']
    short = []
    if o['b'] > g['b_max'] + 1e-9:
        short.append('a face of %.1f mm against Table 13-3\'s %.2f (0.3 R or 10 m)' % (o['b'], g['b_max']))
    if g['zff'] < sp['zff_min'] - 1e-9:
        short.append('a front face %.2f mm from the apex, where the carrier needs %.2f' % (g['zff'], sp['zff_min']))
    if o['b'] < g['R'] / 4 - 1e-9:
        short.append('a face of %.1f mm, under a quarter of the cone distance (%.2f)' % (o['b'], g['R'] / 4))
    if o['z'] != 40 or abs(o['m'] * o['z'] - 60.0) > 1e-6:
        short.append('%d teeth of module %g: the layout is drawn for the 60 mm set, and the geometry factors of '
                     'Figs. 15-6 and 15-7 were read for 40 teeth on 40' % (o['z'], o['m']))
    elif o['duty']['SF'] < 1.5 or o['duty']['SH'] < 1.2 or o['stall']['SF'] < 1.5:
        short.append('safety factors %.2f and %.2f on the duty and %.2f at the torque limit'
                     % (o['duty']['SF'], o['duty']['SH'], o['stall']['SF']))
    if short:
        raise ValueError('This bevel set falls short: %s.' % '; '.join(short))


# ---------- temporary bodies (mm, world or a part's own frame) ----------
UNION = adsk.fusion.BooleanTypes.UnionBooleanType
DIFF = adsk.fusion.BooleanTypes.DifferenceBooleanType
INTER = adsk.fusion.BooleanTypes.IntersectionBooleanType


def cyl(p0, p1, r0, r1=None):
    return tbm().createCylinderOrCone(P(*p0), r0 * MM, P(*p1), (r0 if r1 is None else r1) * MM)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return cyl((x, y, z0), (x, y, z1), r)


def zcone(z0, r0, z1, r1, x=0.0, y=0.0):
    return cyl((x, y, z0), (x, y, z1), r0, r1)


def ycyl(y0, y1, r, x=0.0, z=0.0):
    return cyl((x, y0, z), (x, y1, z), r)


def xcyl(x0, x1, r, y=0.0, z=0.0):
    return cyl((x0, y, z), (x1, y, z), r)


def box(x0, x1, y0, y1, z0, z1):
    obb = adsk.core.OrientedBoundingBox3D.create(P((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), V(1, 0, 0), V(0, 1, 0),
                                                 abs(x1 - x0) * MM, abs(y1 - y0) * MM, abs(z1 - z0) * MM)
    return tbm().createBox(obb)


def _bool(a, items, kind):
    for b in items:
        if b is None:
            continue
        if isinstance(b, (list, tuple)):
            _bool(a, b, kind)
            continue
        if not tbm().booleanOperation(a, b, kind):
            raise RuntimeError('a boolean of temporary bodies failed')
    return a


def union(a, *bs):
    return _bool(a, bs, UNION)


def cut(a, *bs):
    return _bool(a, bs, DIFF)


def inter(a, *bs):
    return _bool(a, bs, INTER)


def ring(z0, z1, ri, ro, x=0.0, y=0.0):
    return cut(zcyl(z0, z1, ro, x, y), zcyl(z0 - 1, z1 + 1, ri, x, y))


def rotz(body, deg):
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(deg), adsk.core.Vector3D.create(0, 0, 1), adsk.core.Point3D.create(0, 0, 0))
    tbm().transform(body, m)
    return body


def polar(r, deg):
    return r * math.cos(math.radians(deg)), r * math.sin(math.radians(deg))


def hex_z(x, y, z0, z1, s):
    h = None
    for k in range(3):
        a = math.radians(60 * k)
        obb = adsk.core.OrientedBoundingBox3D.create(P(x, y, (z0 + z1) / 2), V(-math.sin(a), math.cos(a), 0),
                                                     V(math.cos(a), math.sin(a), 0), 3 * s * MM, s * MM, abs(z1 - z0) * MM)
        b = tbm().createBox(obb)
        h = b if h is None else inter(h, b)
    return h


SHCS = {3: (5.5, 2.5), 4: (7.0, 3.0), 5: (8.5, 4.0), 6: (10.0, 5.0), 8: (13.0, 6.0)}   # ISO 4762: dk, s


def shcs(dn, L, low_head=False):
    """ISO 4762 (or DIN 7984): the head's underside at z = 0, the head up, the shank down to -L."""
    dk, s = SHCS[dn]
    k = float(dn)
    if low_head:
        dk, s, k = {6: (10.0, 4.0, 4.0), 5: (8.5, 3.0, 3.5), 4: (7.0, 2.5, 2.8)}[dn]
    head = union(zcyl(0, k - 0.3, dk / 2), zcone(k - 0.31, dk / 2, k, dk / 2 - 0.3))
    cut(head, hex_z(0, 0, k - 0.55 * dn, k + 1, s))
    union(head, zcyl(-L, 0.2, dn / 2))
    return head


def csk(dn, L):
    """ISO 10642: the head's top at z = 0, the shank down to -L (L includes the head)."""
    dk = 2.0 * dn
    hh = (dk - dn) / 2
    head = zcone(-hh, dn / 2, 0, dk / 2)
    union(head, zcyl(-L, -hh + 0.1, dn / 2))
    cut(head, hex_z(0, 0, -0.6 * dn, 1, 0.67 * dn))
    return head


def bearing(d, D, B):
    """A sealed deep-groove ball bearing as rings and seals, from z = 0 to B."""
    t = (D - d) / 2.0
    ro_in, ri_out = D / 2 - 0.3 * t, d / 2 + 0.3 * t
    outer = cut(zcyl(0, B, D / 2), zcyl(-1, B + 1, ro_in))
    inner = cut(zcyl(0, B, ri_out), zcyl(-1, B + 1, d / 2))
    s1 = cut(zcyl(0.35, 1.0, ro_in), zcyl(0, 1.2, ri_out))
    s2 = cut(zcyl(B - 1.0, B - 0.35, ro_in), zcyl(B - 1.2, B, ri_out))
    return [(outer, 'outer ring', 'bearing'), (inner, 'inner ring', 'bearing'), (s1, 'seal A', 'seal'), (s2, 'seal B', 'seal')]


# ---------- the bevel teeth (each gear in its own frame, its axis along local z) ----------
def tooth_geom(g):
    d = math.radians(45.0)
    a = math.radians(g['alpha'])
    rv = g['R'] * math.tan(d)
    return dict(delta=d, R=g['R'], k=g['k'], z=g['z'], alpha_r=a, rv=rv, rbv=rv * math.cos(a), rav=rv + g['ha'],
                rfv=rv - g['hf'], half_v=(math.pi * g['m'] / 2 - BACKLASH / 2) / (2 * rv))


def _inv(a):
    return math.tan(a) - a


def tooth_uv(t, n_flank=8, n_tip=3, extra=1.0):
    """The tooth's outline in the heel's tangent plane: Tredgold's virtual spur gear, u along the pitch circle, v out."""
    rv, rb, ra, rf, a = t['rv'], t['rbv'], t['rav'], t['rfv'], t['alpha_r']

    def phi(rho):
        return t['half_v'] + _inv(a) - _inv(math.acos(rb / rho))
    right = []
    for i in range(n_flank + 1):
        rho = rf + (ra - rf) * i / n_flank
        p = phi(rho)
        right.append((rho * math.sin(p), rho * math.cos(p) - rv))
    pt = phi(ra)
    tip = [(ra * math.sin(pt - 2 * pt * i / (n_tip + 1)), ra * math.cos(pt - 2 * pt * i / (n_tip + 1)) - rv)
           for i in range(1, n_tip + 1)]
    left = [(-u, v) for (u, v) in reversed(right)]
    pr = phi(rf)
    rb_ = rf - extra
    bottom = [(-rb_ * math.sin(pr), rb_ * math.cos(pr) - rv), (rb_ * math.sin(pr), rb_ * math.cos(pr) - rv)]
    return right + tip + left + bottom


def tooth_points3d(t):
    sd_, cd = math.sin(t['delta']), math.cos(t['delta'])
    R = t['R']
    return [(R * sd_ + v * cd, u, R * cd - v * sd_) for (u, v) in tooth_uv(t)]


def _plane_perp(comp, delta, dist):
    """The construction plane {p . (sin d, 0, cos d) = dist} in the component's frame (the sign checked)."""
    cps = comp.constructionPlanes
    nh = (math.sin(delta), 0.0, math.cos(delta))
    base = None
    for ang in (delta, -delta):
        inp = cps.createInput()
        inp.setByAngle(comp.yConstructionAxis, adsk.core.ValueInput.createByReal(ang), comp.xYConstructionPlane)
        cp = cps.add(inp)
        n = cp.geometry.normal
        if abs(abs(n.x * nh[0] + n.y * nh[1] + n.z * nh[2]) - 1) < 1e-6:
            base = cp
            break
        cp.deleteMe()
    if base is None:
        raise RuntimeError('the tooth\'s base plane was not found')
    base.isLightBulbOn = False
    for s in (1, -1):
        inp = cps.createInput()
        inp.setByOffset(base, adsk.core.ValueInput.createByReal(s * dist * MM))
        cp = cps.add(inp)
        o, n = cp.geometry.origin, cp.geometry.normal
        if abs((o.x * nh[0] + o.y * nh[1] + o.z * nh[2]) / MM - dist) < 1e-3:
            cp.isLightBulbOn = False
            return cp
        cp.deleteMe()
    raise RuntimeError('the tooth\'s offset plane was not found')


def _closed_poly(sk, pts):
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(pts[0], pts[1])
    prev = first.endSketchPoint
    for q in pts[2:]:
        prev = lines.addByTwoPoints(prev, q).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)


def _to_sk(sk, x, y, z):
    inv = sk.transform.copy()
    inv.invert()
    p = P(x, y, z)
    p.transformBy(inv)
    if abs(p.z) > 1e-5:
        raise RuntimeError('a point off its sketch plane by %.6f cm' % p.z)
    return adsk.core.Point3D.create(p.x, p.y, 0)


def add_bevel_teeth(comp, t, blank_body):
    """One straight tooth lofted from the heel to the toe (the toe's section the heel's scaled about the apex),
    patterned round the axis and joined to the blank."""
    pts = tooth_points3d(t)
    k = t['k']
    sk1 = comp.sketches.add(_plane_perp(comp, t['delta'], t['R']))
    sk1.isComputeDeferred = True
    _closed_poly(sk1, [_to_sk(sk1, *p) for p in pts])
    sk1.isComputeDeferred = False
    sk2 = comp.sketches.add(_plane_perp(comp, t['delta'], t['R'] * k))
    sk2.isComputeDeferred = True
    _closed_poly(sk2, [_to_sk(sk2, k * p[0], k * p[1], k * p[2]) for p in pts])
    sk2.isComputeDeferred = False
    if sk1.profiles.count != 1 or sk2.profiles.count != 1:
        raise RuntimeError('tooth profiles: %d / %d' % (sk1.profiles.count, sk2.profiles.count))
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    li.loftSections.add(sk1.profiles.item(0))
    li.loftSections.add(sk2.profiles.item(0))
    li.isSolid = True
    tooth = lofts.add(li).bodies.item(0)
    ents = adsk.core.ObjectCollection.create()
    ents.add(tooth)
    pats = comp.features.circularPatternFeatures
    pin = pats.createInput(ents, comp.zConstructionAxis)
    pin.quantity = adsk.core.ValueInput.createByReal(t['z'])
    pin.totalAngle = adsk.core.ValueInput.createByString('360 deg')
    pin.isSymmetric = False
    pats.add(pin)
    tools = adsk.core.ObjectCollection.create()
    for i in range(comp.bRepBodies.count):
        bb = comp.bRepBodies.item(i)
        if bb.name != blank_body.name:
            tools.add(bb)
    ci = comp.features.combineFeatures.createInput(blank_body, tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    ci.isKeepToolBodies = False
    comp.features.combineFeatures.add(ci)
    sk1.isVisible = False
    sk2.isVisible = False


def gear_blank(g, back):
    """The root cone from the toe to the heel, then the back cone from the heel's root to (r, z) = back."""
    hr, tr = g['heel_root'], g['toe_root']
    b = zcone(tr[1], tr[0], hr[1], hr[0])
    union(b, zcone(hr[1] - 0.001, hr[0], back[1], back[0]))
    return b


def htd_teeth(z0, z1, Z):
    """An HTD 5M pulley's toothed rim, from z0 to z1: the outside diameter, the grooves as round cuts (concept)."""
    pd = Z * 5.0 / math.pi
    ro = (pd - 1.143) / 2
    body = zcyl(z0, z1, ro)
    rg, depth = 1.55, 2.06
    for i in range(Z):
        x, y = polar(ro - depth + rg, 360.0 * i / Z)
        cut(body, zcyl(z0 - 1, z1 + 1, rg, x, y))
    return body, ro


# ---------- the parts (mm). Side-axis parts are drawn along local z = |y| ----------
def side_gear_blank(g, L):
    """The side gear and its pulley in one steel part: the bevel's blank, the back cone down to the pulley's root,
    the 36 grooves from L['pulley'], the bore for B1 at the front and B2 inside the pulley, a shoulder for each."""
    zff = g['zff']
    hr = g['heel_root']
    r_end = 26.0
    b = gear_blank(g, (r_end, hr[1] + (hr[0] - r_end)))
    teeth, ro = htd_teeth(L['pulley'][0], L['pulley'][1], wd.PULLEYS39[1])
    union(b, teeth)
    cut(b, zcyl(zff - 1, zff + 7.0, 14.0), zcyl(zff + 6.9, L['B2'][0] + 0.1, 12.5), zcyl(L['B2'][0], L['pulley'][1] + 1, 14.0))
    return b


def top_gear_blank(g, L, key_deg):
    """The output gear: the bevel's blank with a flat back at L['gear_back'], a hub to L['hub'][1], bore 15 H7 and a
    keyway for the 5 x 5 key (hub depth t2 = 2.3)."""
    zff = g['zff']
    hr = g['heel_root']
    zb = L['gear_back']
    rb = hr[0] - (zb - hr[1])
    b = gear_blank(g, (rb, zb))
    union(b, zcyl(zb - 0.5, L['hub'][1], 14.0))
    cut(b, zcyl(zff - 1, L['hub'][1] + 1, 7.5))
    cut(b, rotz(box(6.0, 9.8, -2.5, 2.5, zff - 1, L['hub'][1] + 1), key_deg))
    return b


def roll_shaft(L):
    s = union(zcyl(L['shaft_bot'], L['collar'][0] + 0.1, 7.5), zcyl(L['collar'][0], L['collar'][1], 10.0),
              zcyl(L['collar'][1] - 0.1, L['shaft_top'], 7.5))
    cut(s, ring(L['ring471'][0], L['ring471'][1], 7.15, 9.0))
    k0, k1 = L['key']
    cut(s, box(4.5, 10, -2.5, 2.5, k0 + 2.5, k1), xcyl(4.5, 10, 2.5, 0, k0 + 2.5))
    cut(s, zcyl(L['shaft_top'] - 13, L['shaft_top'] + 1, 3.0))          # M6 tapped (threads drawn at the nominal size)
    return s


def key_5x5(L):
    k0, k1 = L['key']
    return union(box(4.5, 9.5, -2.5, 2.5, k0 + 2.5, k1), xcyl(4.5, 9.5, 2.5, 0, k0 + 2.5))


def tool_flange(L):
    f0, f1 = L['flange']
    f = zcyl(f0, f1, 22.5)
    cut(f, zcyl(f0 - 1, L['shaft_top'], 7.5), zcyl(L['shaft_top'], f1 + 1, 11.0), box(6.0, 9.8, -2.5, 2.5, f0 - 1, L['shaft_top']))
    for a in (45, 135, 225, 315):
        x, y = polar(15.75, a)
        cut(f, zcyl(f1 - 8, f1 + 1, 2.5, x, y))          # 4 x M5 on 31.5, ISO 9409-1-31.5-4-M5
    x, y = polar(15.75, 0)
    cut(f, zcyl(f1 - 6, f1 + 1, 2.5, x, y))              # the 5 H7 pin's hole
    return f


def carrier_block(L, g):
    """7075-T6: the roll bearings' seats along z, the trunnions' spigot bores along y, rounded about y and chamfered
    at 45 deg under the output gear's toe."""
    h = L['y_cs']
    c = inter(box(-h, h, -h, h, L['z_cb'], L['z_ct']), ycyl(-h - 1, h + 1, 28.5),
              zcone(L['z_ct'] - 40.0, 21.0 + 40.0, L['z_ct'], 21.0))
    cut(c, zcyl(L['z_cb'] - 1, L['z_ct'] + 1, 11.5))
    cut(c, zcyl(L['B4'][0], L['z_ct'] + 1, 16.0), zcyl(L['groove'][0], L['groove'][1], 16.85))
    cut(c, zcyl(L['z_cb'] - 1, L['B5'][1] + 0.5, 16.0))
    for s in (-1, 1):
        cut(c, ycyl(s * L['tr_spigot'], s * (h + 1), 7.5))
        for ang in (45, 135, 225, 315):
            x, z = polar(10.0, ang)
            cut(c, ycyl(s * L['tr_spigot'], s * (h + 1), 1.5, x, z))      # M3 tapped
    return c


def trunnion(L):
    y0, y1 = L['y_fl']
    t = union(zcyl(L['tr_spigot'], y0 + 0.1, 7.5), zcyl(y0, y1, 15.0), zcyl(y1 - 0.1, L['tr_end'], 7.5))
    for ang in (45, 135, 225, 315):
        x, y = polar(10.0, ang)
        cut(t, zcyl(y0 - 1, y1 + 1, 1.7, x, y), zcone(y1 - 1.5, 1.7, y1 + 0.01, 3.1, x, y))
    cut(t, zcyl(L['tr_end'] - 10, L['tr_end'] + 1, 2.5))                  # M5 tapped
    return t


def cheek_cutters(L):
    """In the cheek's own frame: the plate in local xy (u = -x, v = z for the +y cheek), its thickness local z 0..12
    from the inner face."""
    T = L['cheek'][1] - L['cheek'][0]
    lip = L['lip'][1] - L['lip'][0]
    c = union(zcyl(-1, lip + 0.01, 13.0), zcyl(lip, T + 1, 16.0))
    for a in (45, 135, 225, 315):
        x, y = polar(18, a)
        union(c, zcyl(T - 8, T + 1, 2.0, x, y))                       # M4 tapped (the cap)
    tx, tz, tr = L['tie']
    for u in (-tx, tx):
        union(c, zcyl(-1, T + 1, 4.5, u, tz), zcyl(T - 8.5, T + 1, 7.0, u, tz))   # M8 counterbored (the tie bars)
    xm, zm = L['x_m'], L['z_m']
    union(c, box(xm - 22, xm + 22, zm - 22, zm + 22, -1, T + 1))      # the window for the other motor
    zb = L['base'][1]
    for u in (-50.0, -15.0, 55.0):
        union(c, ycyl(zb - 1, zb + 12, 2.5, u, T / 2))                 # M5 tapped (the base)
    for u in DOWELS:
        union(c, ycyl(zb - 1, zb + 9.5, 2.5, u, T / 2))                # dowel 5
    return c


DOWELS = (-33.0, 0.0)          # u in the cheek's frame: clear of the screws and below the window's side


def cheek_outline(L):
    zb = L['base'][1]
    return [('line', (24, 0), (24, -25)), ('line', (24, -25), (62, -50)), ('line', (62, -50), (62, zb)),
            ('line', (62, zb), (-62, zb)), ('line', (-62, zb), (-62, -50)), ('line', (-62, -50), (-24, -25)),
            ('line', (-24, -25), (-24, 0)), ('arc', (-24, 0), (0, 24), (24, 0))]


def bearing_cap(L, fixed=True):
    y3 = L['B3'][1]
    T = L['cheek'][1]
    c = union(ring(y3 + (0.0 if fixed else 0.5), T + 0.1, 13.0, 16.0), zcyl(T, T + 8, 22.0))
    cut(c, zcyl(y3 - 1, T + 6.5, 11.0))
    for a in (45, 135, 225, 315):
        x, y = polar(18, a)
        cut(c, zcyl(T - 1, T + 9, 2.25, x, y))
    return c


def base_plate(L):
    z0, z1 = L['base']
    b = box(-62, 62, -70, 70, z0, z1)
    yc = sum(L['cheek']) / 2
    for s in (1, -1):
        for u in (-50.0, -15.0, 55.0):
            x = -s * u
            cut(b, zcyl(z0 - 1, z1 + 1, 2.75, x, s * yc), zcyl(z0 - 1, z0 + 5.5, 5.0, x, s * yc))
        for u in DOWELS:
            cut(b, zcyl(z0 + 3.5, z1 + 1, 2.5, -s * u, s * yc))
        for x in (14.0, 40.0):
            cut(b, zcyl(z0 - 1, z1 + 1, 2.75, s * x, s * 38.0), zcyl(z0 - 1, z0 + 5.5, 5.0, s * x, s * 38.0))
    for x in (-45, 45):
        for y in (-10, 10):
            cut(b, zcyl(z0 - 1, z0 + 10, 3.0, x, y))               # M6 tapped: the arm's interface
    for x in (-55, 55):
        cut(b, zcyl(z0 - 1, z0 + 8, 3.0, x, 0))                    # 6 H7 locating
    cut(b, zcyl(z0 - 1, z1 + 1, 12.0))                             # the cables' way through
    return b


def tie_bar(L):
    tx, tz, tr = L['tie']
    y = L['cheek'][0]
    t = ycyl(-y, y, tr)
    cut(t, ycyl(-y - 1, -y + 18, 4.0), ycyl(y - 18, y + 1, 4.0))
    return t


def motor_bracket(L):
    """The right-hand bracket in world coordinates (the left one is the same part turned 180 deg about z): a plate
    in the gearhead's face plane, slotted 3 mm either way along z for the belt's tension, on a foot."""
    xm, zm = L['x_m'], L['z_m']
    y0, y1 = L['bracket']
    zb = L['base'][1]
    b = union(box(xm - 25, xm + 25, y0, y1, zb, zm + 25), box(xm - 25, xm + 25, y1 - 0.01, 44.0, zb, zb + 6))

    def slot(x, z, r, dz):
        return union(ycyl(y0 - 1, y1 + 1, r, x, z - dz), ycyl(y0 - 1, y1 + 1, r, x, z + dz), box(x - r, x + r, y0 - 1, y1 + 1, z - dz, z + dz))
    cut(b, slot(xm, zm, 12.0, 3.0))
    for dx in (-16, 16):
        for dz in (-16, 16):
            cut(b, slot(xm + dx, zm + dz, 2.2, 3.0))
    for x in (14.0, 40.0):
        cut(b, zcyl(zb - 1, zb + 7, 2.5, x, 38.0))
    return b


def gearmotor(L):
    """The right-hand motor in world coordinates: the gearhead's square flange and round body, the motor and its
    brake as 40 mm squares, a connector; the face at |y| = gh_face, the body inward (assumed outline)."""
    xm, zm = L['x_m'], L['z_m']
    F = wd.MOTOR39['flange'] / 2
    y = L['gh_face']
    fl = union(box(xm - F, xm + F, y - 5, y, zm - F, zm + F), ycyl(y - 0.1, y + 2.0, 10.0, xm, zm))
    cut(fl, ycyl(y - 6, y + 3, 4.2, xm, zm))                           # the output shaft's way out
    for dx in (-16, 16):
        for dz in (-16, 16):
            cut(fl, ycyl(y - 6, y + 1, 2.0, xm + dx, zm + dz))
    y_gh = y - 5 - wd.GEARHEAD['L']
    gh = ycyl(y_gh, y - 5, wd.GEARHEAD['D'] / 2, xm, zm)
    y_m = y_gh - wd.MOTOR39['L_m']
    mo = box(xm - F, xm + F, y_m, y_gh, zm - F, zm + F)
    y_b = y_m - wd.MOTOR39['L_br']
    br = box(xm - F, xm + F, y_b, y_m, zm - F, zm + F)
    cn = box(xm - 6, xm + 6, y_b + 6, y_b + 18, zm + F, zm + F + 6)
    return [(fl, 'gearhead flange', 'alu'), (gh, 'planetary gearhead 15:1', 'dark'), (mo, 'servo motor 50 W', 'stator'),
            (br, 'holding brake', 'dark'), (cn, 'connector', 'black')]


def motor_output(L):
    """The gearhead's output shaft (8 mm, a flat) and the 18-groove pulley on it, flanged both sides, with two M4
    set screws in its inner hub; in world coordinates for the right-hand motor."""
    xm, zm = L['x_m'], L['z_m']
    h0, h1 = L['m_hub']
    (fa0, fa1), (fb0, fb1) = L['m_flanges']
    teeth, ro = htd_teeth(fa1 - 0.01, fb0 + 0.01, wd.PULLEYS39[0])
    m = adsk.core.Matrix3D.create()
    m.setToRotation(-math.pi / 2, adsk.core.Vector3D.create(1, 0, 0), adsk.core.Point3D.create(0, 0, 0))   # local z -> world +y
    tbm().transform(teeth, m)
    mv = adsk.core.Matrix3D.create()
    mv.translation = adsk.core.Vector3D.create(xm * MM, 0, zm * MM)
    tbm().transform(teeth, mv)
    body = union(ycyl(h0, h1 + 0.01, 10.0, xm, zm), ycyl(fa0, fa1, ro + 2.5, xm, zm), teeth, ycyl(fb0, fb1, ro + 2.5, xm, zm))
    cut(body, ycyl(h0 - 1, fb1 + 1, 4.0, xm, zm))
    ys = (h0 + h1) / 2
    cut(body, cyl((xm, ys, zm + 3.0), (xm, ys, zm + 11.0), 2.0), cyl((xm + 3.0, ys, zm), (xm + 11.0, ys, zm), 2.0))
    s0, s1 = L['m_shaft']
    shaft = cut(ycyl(s0, s1, 4.0, xm, zm), box(xm - 5, xm + 5, s0 - 1, s1 + 1, zm + 3.5, zm + 5))
    g1 = cyl((xm, ys, zm + 3.5), (xm, ys, zm + 7.5), 2.0)
    g2 = cyl((xm + 4.0, ys, zm), (xm + 8.0, ys, zm), 2.0)
    return [(body, 'pulley 18 grooves', 'alu'), (shaft, 'gearhead output shaft 8', 'steel'),
            (g1, 'set screw M4 (on the flat)', 'screw'), (g2, 'set screw M4', 'screw')]


# ---------- looks and the builder ----------
LOOKS = {
    'steel': ('Steel', 'Steel - Satin'),
    'gear': ('Steel', 'Steel - Satin'),
    'frame': ('Aluminum 6061', 'Aluminum - Anodized Rough (Grey)'),
    'carrier': ('Aluminum 6061', 'Aluminum - Anodized Glossy (Blue)'),
    'alu': ('Aluminum 6061', 'Aluminum - Satin'),
    'bearing': ('Steel', 'Stainless Steel - Polished'),
    'seal': ('Rubber', 'Plastic - Matte (Red)'),
    'black': ('ABS Plastic', 'Plastic - Matte (Black)'),
    'stator': ('Steel', 'Plastic - Matte (Black)'),
    'dark': ('Aluminum 6061', 'Paint - Enamel Glossy (Dark Grey)'),
    'screw': ('Steel', 'Coating - Black Oxide'),
    'belt': ('Rubber', 'Rubber - Hard'),
    'stainless': ('Stainless Steel', 'Stainless Steel - Satin'),
}
_cache = {}


def _find(app, kind, name):
    key = (kind, name)
    if key in _cache:
        return _cache[key]
    found = None
    libs = app.materialLibraries
    order = ['Fusion Appearance Library', 'Fusion Material Library'] if kind == 'a' else ['Fusion Material Library']
    for ln in order:
        lib = libs.itemByName(ln)
        if lib is None:
            continue
        coll = lib.appearances if kind == 'a' else lib.materials
        found = coll.itemByName(name)
        if found:
            break
    if found is None and kind == 'm':
        lib = libs.itemByName('Fusion Material Library')
        for i in range(lib.materials.count):
            if name.split()[0].lower() in lib.materials.item(i).name.lower():
                found = lib.materials.item(i)
                break
    _cache[key] = found
    return found


def apply_look(app, body, look):
    mat, ap = LOOKS.get(look, LOOKS['steel'])
    m = _find(app, 'm', mat)
    if m:
        try:
            body.material = m
        except Exception:
            pass
    a = _find(app, 'a', ap)
    if a:
        try:
            body.appearance = a
        except Exception:
            pass


def frame(X, Y, Z, T=(0, 0, 0)):
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P(*T), V(*X), V(*Y), V(*Z))
    return m


def ident(T=(0, 0, 0)):
    return frame((1, 0, 0), (0, 1, 0), (0, 0, 1), T)


SIDE = {1: ((0, 0, 1), (1, 0, 0), (0, 1, 0)), -1: ((0, 0, 1), (-1, 0, 0), (0, -1, 0))}   # local z = +-y, local x = +z


def side(sign, cx=0.0, cy=0.0, cz=0.0):
    """A side-axis frame (local z along sign * y); (cx, cy, cz) a point in that frame."""
    X, Y, Z = SIDE[sign]
    T = tuple(cx * X[i] + cy * Y[i] + cz * Z[i] for i in range(3))
    return frame(X, Y, Z, T)


DOWN = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
ROT180Z = ((-1, 0, 0), (0, -1, 0), (0, 0, 1))


class Builder:
    def __init__(self, app, root):
        self.app = app
        self.root = root
        self.parts = {}

    def comp(self, parent, name, M=None):
        occ = parent.occurrences.addNewComponent(M if M is not None else ident())
        occ.component.name = name
        return occ

    def inst(self, parent, comp, M):
        return parent.occurrences.addExistingComponent(comp, M)

    def bodies(self, comp, items):
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        looks = {}
        for i, (tb, name, look) in enumerate(items):
            b = comp.bRepBodies.add(tb, bf)
            nm = name if name not in looks else '%s %d' % (name, i)
            b.name = nm
            looks[nm] = look
        bf.finishEdit()
        for i in range(bf.bodies.count):
            b = bf.bodies.item(i)
            apply_look(self.app, b, looks.get(b.name, 'steel'))
        return bf

    def part(self, parent, key, name, items, M=None):
        """A component of temporary bodies; later uses of the same key are further occurrences of it."""
        if key in self.parts:
            return self.inst(parent, self.parts[key], M if M is not None else ident())
        occ = self.comp(parent, name, M)
        self.bodies(occ.component, items)
        self.parts[key] = occ.component
        return occ

    def screw(self, parent, dn, L, M, kind='shcs'):
        key = '%s-M%d-%g' % (kind, dn, L)
        if kind == 'shcs':
            nm, tb = 'Cap screw ISO 4762 M%dx%g' % (dn, L), shcs(dn, L)
        elif kind == 'low':
            nm, tb = 'Low-head screw DIN 7984 M%dx%g' % (dn, L), shcs(dn, L, low_head=True)
        else:
            nm, tb = 'Countersunk screw ISO 10642 M%dx%g' % (dn, L), csk(dn, L)
        return self.part(parent, key, nm, [(tb, nm, 'screw')], M)


def ground_children(comp):
    """Every part of a bolted group is grounded to it, or only the first follows the group's joint."""
    occs = [comp.occurrences.item(i) for i in range(comp.occurrences.count)]
    for _ in range(3):
        for o in occs:
            if not o.isGroundToParent:
                o.isGroundToParent = True
        if all(o.isGroundToParent for o in occs):
            break
    left = [o.name for o in occs if not o.isGroundToParent]
    if left:
        raise RuntimeError('not grounded to its group in %s: %s' % (comp.name, left))


def gear_part(B, parent, key, name, blank, t, M):
    if key in B.parts:
        return B.inst(parent, B.parts[key], M)
    occ = B.comp(parent, name, M)
    c = occ.component
    bf = B.bodies(c, [(blank, 'gear', 'gear')])
    add_bevel_teeth(c, t, bf.bodies.item(0))
    B.parts[key] = c
    return occ


# ---------- the groups ----------
def make_input(B, parent, o):
    """The left input's group, built on +y at identity: the side gear with its pulley and its two 61902s. The
    other input is the same component turned 180 deg about z (its tooth still on top)."""
    L, g = o['L'], o['g']
    occ = B.comp(parent, 'Input drive')
    a = occ.component
    gear_part(B, a, 'side', 'Side bevel gear with 36-groove pulley', side_gear_blank(g, L), tooth_geom(g), side(1))
    for y0 in (L['B1'][0], L['B2'][0]):
        B.part(a, 'b61902', 'Bearing 61902-2RS1', bearing(15, 28, 7), side(1, 0, 0, y0))
    return occ


def make_carrier(B, parent, o):
    L, g = o['L'], o['g']
    occ = B.comp(parent, 'Carrier')
    c = occ.component
    B.part(c, 'carrier', 'Carrier block', [(carrier_block(L, g), 'carrier block', 'carrier')], ident())
    for sg in (1, -1):
        B.part(c, 'trunnion', 'Trunnion', [(trunnion(L), 'trunnion', 'steel')], side(sg))
        B.part(c, 's1', 'Spacer S1', [(ring(L['S1'][0], L['S1'][1], 7.5, 9.0), 'spacer', 'steel')], side(sg))
        B.part(c, 's2', 'Spacer S2', [(ring(L['S2'][0], L['S2'][1], 7.5, 9.0), 'spacer', 'steel')], side(sg))
        B.part(c, 's3', 'Spacer S3', [(ring(L['S3'][0], L['S3'][1], 7.5, 9.0), 'spacer', 'steel')], side(sg))
        B.part(c, 'washer5', 'End washer 18x5.5x2.5', [(ring(L['tr_end'], L['tr_end'] + 2.5, 2.75, 9.0), 'washer', 'steel')], side(sg))
        B.screw(c, 5, 12, side(sg, 0, 0, L['tr_end'] + 2.5))
        for ang in (45, 135, 225, 315):
            x, y = polar(10.0, ang)
            B.screw(c, 3, 8, side(sg, x, y, L['y_fl'][1]), kind='csk')
    B.part(c, 'b6002', 'Bearing 6002-2RSH', bearing(15, 32, 9), ident((0, 0, L['B4'][0])))
    B.inst(c, B.parts['b6002'], ident((0, 0, L['B5'][0])))
    gr = L['groove']
    B.part(c, 'ring472', 'Retaining ring DIN 472-32', [(ring(gr[0] + 0.05, gr[0] + 1.25, 13.8, 16.8), 'ring', 'screw')], ident())
    return occ


def make_roll(B, parent, o, phase=None):
    L, g = o['L'], o['g']
    occ = B.comp(parent, 'Roll output')
    c = occ.component
    psi = (90.0 - 180.0 / g['z']) if phase is None else phase    # a gap of the output gear faces each side gear's tooth
    B.part(c, 'rollshaft', 'Roll shaft', [(roll_shaft(L), 'roll shaft', 'steel')], ident())
    cp, sp = math.cos(math.radians(psi)), math.sin(math.radians(psi))
    gear_part(B, c, 'top', 'Output bevel gear', top_gear_blank(g, L, -psi), tooth_geom(g), frame((cp, sp, 0), (-sp, cp, 0), (0, 0, 1)))
    B.part(c, 'key', 'Key DIN 6885 A 5x5x18', [(key_5x5(L), 'key', 'steel')], ident())
    B.part(c, 's4', 'Spacer S4', [(ring(L['S4'][0], L['S4'][1], 7.5, 9.5), 'spacer', 'steel')], ident())
    B.part(c, 'flange', 'Tool flange ISO 9409-1-31.5-4-M5', [(tool_flange(L), 'tool flange', 'stainless')], ident())
    B.part(c, 'washer6', 'End washer 18x6.4x2', [(ring(L['shaft_top'], L['shaft_top'] + 2.0, 3.2, 9.0), 'washer', 'steel')], ident())
    B.screw(c, 6, 12, ident((0, 0, L['shaft_top'] + 2.0)), kind='low')
    r4 = L['ring471']
    B.part(c, 'ring471', 'Retaining ring DIN 471-15', [(ring(r4[0] + 0.05, r4[1] - 0.05, 7.15, 9.6), 'ring', 'screw')], ident())
    return occ, psi


def make_cheek(B, c, L):
    sk = c.sketches.add(c.xYConstructionPlane)
    sk.isComputeDeferred = True
    lines = sk.sketchCurves.sketchLines
    arcs = sk.sketchCurves.sketchArcs
    pts = {}

    def sp(xy):
        return pts.get(xy) or adsk.core.Point3D.create(xy[0] * MM, xy[1] * MM, 0)
    for seg in cheek_outline(L):
        if seg[0] == 'line':
            ln = lines.addByTwoPoints(sp(seg[1]), sp(seg[2]))
            pts[seg[1]], pts[seg[2]] = ln.startSketchPoint, ln.endSketchPoint
        else:
            a = arcs.addByThreePoints(sp(seg[1]), adsk.core.Point3D.create(seg[2][0] * MM, seg[2][1] * MM, 0), sp(seg[3]))
            pts[seg[1]], pts[seg[3]] = a.startSketchPoint, a.endSketchPoint
    sk.isComputeDeferred = False
    prof = max([sk.profiles.item(i) for i in range(sk.profiles.count)], key=lambda p: p.areaProperties().area)
    ext = c.features.extrudeFeatures
    ei = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal((L['cheek'][1] - L['cheek'][0]) * MM))
    body = ext.add(ei).bodies.item(0)
    body.name = 'cheek'
    sk.isVisible = False
    bf = B.bodies(c, [(cheek_cutters(L), 'cutters', 'frame')])
    tools = adsk.core.ObjectCollection.create()
    tools.add(bf.bodies.item(0))
    ci = c.features.combineFeatures.createInput(body, tools)
    ci.operation = adsk.fusion.FeatureOperations.CutFeatureOperation
    c.features.combineFeatures.add(ci)
    apply_look(B.app, c.bRepBodies.itemByName('cheek'), 'frame')


def belt_loops(L):
    D, d = wd.pd5(wd.PULLEYS39[1]), wd.pd5(wd.PULLEYS39[0])
    ro_l, ro_s = (D - 1.143) / 2, (d - 1.143) / 2
    return (0.0, 0.0), (L['x_m'], L['z_m']), (ro_l, ro_s), (ro_l + 1.74, ro_s + 1.74)


def belt_loop_points(c1, c2, r1, r2):
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    d = math.hypot(dx, dy)
    ux, uy = dx / d, dy / d
    wx, wy = -uy, ux
    s = (r1 - r2) / d
    c = math.sqrt(1 - s * s)
    out = {}
    for sign in (1, -1):
        nx, ny = ux * s + sign * wx * c, uy * s + sign * wy * c
        out[sign] = ((c1[0] + r1 * nx, c1[1] + r1 * ny), (c2[0] + r2 * nx, c2[1] + r2 * ny))
    return out, (c1[0] - ux * r1, c1[1] - uy * r1), (c2[0] + ux * r2, c2[1] + uy * r2)


def make_belt(B, c, L):
    """The belt as a band round the two pulleys' outside diameters, 1.74 mm thick, in the belt's own frame: local x
    = world x, local y = world z, extruded along local z = -y from the belt's outer edge."""
    sk = c.sketches.add(c.xYConstructionPlane)
    c1, c2, rin, rout = belt_loops(L)
    sk.isComputeDeferred = True
    Q = lambda xy: adsk.core.Point3D.create(xy[0] * MM, xy[1] * MM, 0)
    for r1, r2 in ((rin[0], rin[1]), (rout[0], rout[1])):
        tang, m1, m2 = belt_loop_points(c1, c2, r1, r2)
        lines = sk.sketchCurves.sketchLines
        arcs = sk.sketchCurves.sketchArcs
        spts = sk.sketchPoints
        p1, p2 = spts.add(Q(tang[1][0])), spts.add(Q(tang[1][1]))
        p3, p4 = spts.add(Q(tang[-1][1])), spts.add(Q(tang[-1][0]))
        lines.addByTwoPoints(p1, p2)
        arcs.addByThreePoints(p2, Q(m2), p3)            # arcs are kept counter-clockwise: never chain on their end
        lines.addByTwoPoints(p3, p4)
        arcs.addByThreePoints(p4, Q(m1), p1)
    sk.isComputeDeferred = False
    prof = None
    for i in range(sk.profiles.count):
        if sk.profiles.item(i).profileLoops.count == 2:
            prof = sk.profiles.item(i)
    if prof is None:
        raise RuntimeError('the belt\'s profile was not found (%d profiles)' % sk.profiles.count)
    ext = c.features.extrudeFeatures
    ei = ext.createInput(prof, adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal((L['belt'][1] - L['belt'][0]) * MM))
    body = ext.add(ei).bodies.item(0)
    body.name = 'belt'
    sk.isVisible = False
    apply_look(B.app, body, 'belt')


def make_frame(B, parent, o):
    L = o['L']
    occ = B.comp(parent, 'Frame')
    f = occ.component
    B.part(f, 'base', 'Base plate', [(base_plate(L), 'base plate', 'frame')], ident())
    y0 = L['cheek'][0]
    cheek_M = {1: frame((-1, 0, 0), (0, 0, 1), (0, 1, 0), (0, y0, 0)), -1: frame((1, 0, 0), (0, 0, 1), (0, -1, 0), (0, -y0, 0))}
    cocc = B.comp(f, 'Side cheek', cheek_M[1])
    make_cheek(B, cocc.component, L)
    B.parts['cheek'] = cocc.component
    B.inst(f, cocc.component, cheek_M[-1])
    T = L['cheek'][1]
    for sg in (1, -1):
        B.part(f, 'cap%d' % sg, 'Bearing cap %s' % ('fixed' if sg > 0 else 'floating'),
               [(bearing_cap(L, sg > 0), 'bearing cap', 'frame')], side(sg))
        B.inst(f, B.parts['b6002'], side(sg, 0, 0, L['B3'][0]))
        for ang in (45, 135, 225, 315):
            x, y = polar(18, ang)
            B.screw(f, 4, 12, side(sg, x, y, T + 8))
        X, Y, Z = SIDE[sg]
        tx, tz, tr = L['tie']
        for x in (-tx, tx):
            B.screw(f, 8, 20, side(sg, tz, x * sg, T - 8.5))
        yc = sum(L['cheek']) / 2
        for u in (-50.0, -15.0, 55.0):
            B.screw(f, 5, 16, frame(*DOWN, T=(-sg * u, sg * yc, L['base'][0] + 5.5)))
        for u in DOWELS:
            B.part(f, 'dowel5', 'Dowel ISO 8734 5x16', [(zcyl(0, 16, 2.5), 'dowel', 'steel')],
                   ident((-sg * u, sg * yc, L['base'][1] - 8.0)))
        for x in (14.0, 40.0):
            B.screw(f, 5, 12, frame(*DOWN, T=(sg * x, sg * 38.0, L['base'][0] + 5.5)))
    tx, tz, tr = L['tie']
    for x in (-tx, tx):
        B.part(f, 'tiebar', 'Tie bar', [(tie_bar(L), 'tie bar', 'frame')], ident((x, 0, tz)))
    B.part(f, 'bracket', 'Motor bracket', [(motor_bracket(L), 'motor bracket', 'frame')], ident())
    B.inst(f, B.parts['bracket'], frame(*ROT180Z))
    B.part(f, 'motor', 'Servo motor with brake and 15:1 gearhead', gearmotor(L), ident())
    B.inst(f, B.parts['motor'], frame(*ROT180Z))
    for sg in (1, -1):
        for dx in (-16, 16):
            for dz in (-16, 16):
                B.screw(f, 4, 10, side(sg, L['z_m'] + dz, L['x_m'] + dx, L['bracket'][1]))
    bocc = B.comp(f, 'Belt HTD 350-5M-15', frame((1, 0, 0), (0, 0, 1), (0, -1, 0), (0, L['belt'][1], 0)))
    make_belt(B, bocc.component, L)
    B.inst(f, bocc.component, frame((-1, 0, 0), (0, 0, 1), (0, 1, 0), (0, -L['belt'][1], 0)))
    return occ


# ---------- joints ----------
_jsp = {}


def joint_point(occ, xyz=(0.0, 0.0, 0.0)):
    comp = occ.component
    key = (comp.name, xyz)
    if key not in _jsp:
        sk = comp.sketches.add(comp.xYConstructionPlane)
        _jsp[key] = sk.sketchPoints.add(P(*xyz))
        sk.isVisible = False
    return _jsp[key].createForAssemblyContext(occ)


def revolute(root, moving, fixed, direction, name, xyz=(0.0, 0.0, 0.0)):
    geo = adsk.fusion.JointGeometry.createByPoint(joint_point(moving, xyz))
    inp = root.asBuiltJoints.createInput(moving, fixed, geo)
    inp.setAsRevoluteJointMotion(direction)
    j = root.asBuiltJoints.add(inp)
    j.name = name
    return j


def _rot(m):
    return [[m.getCell(i, j) for j in range(3)] for i in range(3)]


def joint_sign(j, axis):
    """+1 if a positive value of j turns its moving part right-handed about the world axis, -1 if left-handed."""
    mo = j.jointMotion
    occ = j.occurrenceOne
    mo.rotationValue = 0.0
    R0 = _rot(occ.transform2)
    mo.rotationValue = 0.2
    R1 = _rot(occ.transform2)
    mo.rotationValue = 0.0
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    w = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
    d = w[0] * axis[0] + w[1] * axis[1] + w[2] * axis[2]
    if abs(d) < 0.1:
        raise RuntimeError('joint %s does not turn about %s (w = %s)' % (j.name, axis, w))
    return 1 if d > 0 else -1


def link(root, ja, jb, k, sa, sb, name):
    """A Motion Link so that (b's angle) = k (a's angle), each about its world axis; sa, sb the joints' signs."""
    r = sb * k * sa
    inp = root.motionLinks.createInput(ja, jb)
    inp.valueOne = adsk.core.ValueInput.createByString('360 deg')
    inp.valueTwo = adsk.core.ValueInput.createByString('%g deg' % (360 * abs(r)))
    inp.isReversed = r < 0
    ml = root.motionLinks.add(inp)
    ml.name = name
    return ml


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        count[o.component.name] = count.get(o.component.name, 0) + 1
    return count


_last = {}


def pose(pitch, roll):
    """Pitch phi and roll psi in degrees: the carrier's and the output's joints are set; the links move the side
    gears and the motors (each motor 2 (phi -+ psi), its helper carrying the 2 phi share)."""
    J, S = _last['joints'], _last['signs']
    J['pitch'].jointMotion.rotationValue = S['pitch'] * math.radians(pitch)
    J['roll'].jointMotion.rotationValue = S['roll'] * math.radians(roll)
    adsk.doEvents()


def build(module=1.5, teeth=40, face=11.0, phase=None):
    o = design(module, teeth, face)
    refuse(o)
    app = adsk.core.Application.get()
    _jsp.clear()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    B = Builder(app, root)
    L = o['L']
    occ_c = make_carrier(B, root, o)
    occ_r, psi = make_roll(B, root, o, phase)
    occ_iL = make_input(B, root, o)                          # built on +y: the left input (x forward, z up)
    occ_iR = B.inst(root, occ_iL.component, frame(*ROT180Z))
    occ_f = make_frame(B, root, o)
    occ_mL = B.part(root, 'mout', 'Motor output', motor_output(L), ident())   # in the +y belt's plane
    occ_mR = B.inst(root, occ_mL.component, frame(*ROT180Z))
    occ_hL = B.comp(root, 'Belt helper', ident())
    occ_hR = B.inst(root, occ_hL.component, frame(*ROT180Z))
    for x in (occ_c, occ_r, occ_iL, occ_iR, occ_mL, occ_mR, occ_hL, occ_hR):
        x.isGroundToParent = False
    occ_f.isGrounded = True
    for sub in (occ_f, occ_c, occ_r, occ_iL):
        ground_children(sub.component)
    Jd = adsk.fusion.JointDirections
    J = {}
    J['pitch'] = revolute(root, occ_c, occ_f, Jd.YAxisJointDirection, 'Pitch (carrier)')
    J['roll'] = revolute(root, occ_r, occ_c, Jd.ZAxisJointDirection, 'Roll (output on the carrier)')
    J['inL'] = revolute(root, occ_iL, occ_c, Jd.YAxisJointDirection, 'Input L (on the carrier)')
    J['inR'] = revolute(root, occ_iR, occ_c, Jd.YAxisJointDirection, 'Input R (on the carrier)')
    xm, zm = L['x_m'], L['z_m']
    J['hL'] = revolute(root, occ_hL, occ_f, Jd.YAxisJointDirection, 'Belt helper L', (xm, 0.0, zm))
    J['hR'] = revolute(root, occ_hR, occ_f, Jd.YAxisJointDirection, 'Belt helper R', (xm, 0.0, zm))
    J['mL'] = revolute(root, occ_mL, occ_hL, Jd.YAxisJointDirection, 'Motor L', (xm, 0.0, zm))
    J['mR'] = revolute(root, occ_mR, occ_hR, Jd.YAxisJointDirection, 'Motor R', (xm, 0.0, zm))
    Y, Z = (0, 1, 0), (0, 0, 1)
    S = {k: joint_sign(j, Z if k == 'roll' else Y) for k, j in J.items()}
    # the mesh: on the carrier the left (+y) gear turns -psi, the right (-y) gear +psi (rolling at the pitch point)
    link(root, J['roll'], J['inL'], -1, S['roll'], S['inL'], 'Mesh: roll to gear L')
    link(root, J['roll'], J['inR'], +1, S['roll'], S['inR'], 'Mesh: roll to gear R')
    # the belts, 36/18: each motor turns twice its gear's absolute angle, 2 phi on its helper plus 2 (-+ psi) on it
    link(root, J['pitch'], J['hL'], +2, S['pitch'], S['hL'], 'Belt L: the pitch share')
    link(root, J['pitch'], J['hR'], +2, S['pitch'], S['hR'], 'Belt R: the pitch share')
    link(root, J['roll'], J['mL'], -2, S['roll'], S['mL'], 'Belt L: the roll share')
    link(root, J['roll'], J['mR'], +2, S['roll'], S['mR'], 'Belt R: the roll share')
    lim = J['pitch'].jointMotion.rotationLimits
    lim.isMinimumValueEnabled = True
    lim.minimumValue = math.radians(-wd.brief()['pitch_max'])
    lim.isMaximumValueEnabled = True
    lim.maximumValue = math.radians(wd.brief()['pitch_max'])
    _last.update(joints=J, signs=S, occ=dict(frame=occ_f, carrier=occ_c, roll=occ_r, inL=occ_iL, inR=occ_iR,
                                              mL=occ_mL, mR=occ_mR, hL=occ_hL, hR=occ_hR), psi=psi, o=o)
    for attr in ('isJointsFolderLightBulbOn', 'isOriginFolderLightBulbOn'):
        try:
            setattr(root, attr, False)
        except Exception:
            pass
    app.activeViewport.fit()
    pl = parts_list(root)
    g, du, st, mo, bt, sh = o['g'], o['duty'], o['stall'], o['mot'], o['belt'], o['shaft']
    return ('Pitch-roll wrist built.\nBevel set m %g, %d teeth, face %g mm: pitch diameter %.1f, front face %.2f mm from '
            'the apex (the carrier needs %.2f). Duty: S_F %.2f, S_H %.2f; at the drives\' limit: S_F %.2f.\nMotors: '
            '%.3f N m peak, %.3f RMS, %.0f rev/min; belts 350-5M at %.2f mm. The payload on the roll shaft: %.1f Hz.\n'
            '\nParts: %d components; %s.\n\n[%s]'
            % (o['m'], o['z'], o['b'], g['d'], g['zff'], o['space']['zff_min'], du['SF'], du['SH'], st['SF'],
               mo['peak'], mo['rms'], mo['nmax'], bt['C'], sh['f'], sum(pl.values()),
               ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('module', 'Bevel module (mm)', 1.5), ('teeth', 'Teeth on each bevel gear', 40.0), ('face', 'Face width (mm)', 11.0)]


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
            ins.addValueInput(key, label, '', adsk.core.ValueInput.createByReal(dflt))
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'WristDifferentialCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Pitch-roll wrist', 'The bevel differential wrist of Chapter 39')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('WristDifferential could not start:\n' + traceback.format_exc())
