# -*- coding: utf-8 -*-
# =============================================================================
# WristDifferential.py: the pitch-roll wrist of Volume 2, Chapter 39, written from its prompt alone.
#
# A miter bevel differential: two side gears on the pitch axis (y), each with a 36-groove pulley driven by a
# toothed belt from a servo motor with a 15:1 gearhead, and an output gear on the roll shaft (z) that turns the
# tool flange. The carrier pitches on two trunnions in the frame's cheeks. Every axial position comes from
# lib/wrist_design.py (bevel(), layout(), the motors and the belt); the script draws them, joins the groups,
# links the joints, poses the wrist and lists the parts.
#
# Origin at the gears' apex, y along the pitch axis (L is +y), z along the roll axis at rest, x forward.
# Sizes in millimetres; the Fusion API works in centimetres and radians.
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


def _common_dir():
    """The companion's common/ folder: the first folder above this script that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % os.path.abspath(__file__))
        d = up


_COMMON = _common_dir()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP           # noqa: E402,F401  (puts every lib/ folder on sys.path)
import wrist_design as wd              # noqa: E402
import mdfc_parts as mp                # noqa: E402
importlib.reload(wd)                   # Fusion keeps modules between runs: pick up edits to the libraries
importlib.reload(mp)

VERSION = 'WristDifferential 1.0 (prompt test)'
CMD_ID = 'mdfcWristDifferentialPromptTest'

CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal
VS = adsk.core.ValueInput.createByString

# ---------- the prompt's limits ----------
SF_DUTY, SH_DUTY, SF_LIMIT = 1.5, 1.2, 1.5

# ---------- sizes the prompt and the library leave open (assumed, each named where it is used) ----------
BACKLASH = wd.BACKLASH40['bevel_mm']   # circular backlash at the heel's pitch circle: the library's shimmed set
ROOT_DROP = 0.6                        # each tooth's section is closed this far below the root, inside the blank
CHAMFER_GAP = 0.5                      # the carrier's 45 deg chamfer this far inside the output gear's toe cone
SHAFT_D = 15.0                         # the roll shaft and the trunnions: the bearings' bore
COLLAR_D = 20.0                        # the roll shaft's collar (layout: '20 mm')
SPACER_D = 18.0                        # spacers S1 to S4: clear of the outer rings, on the inner rings only
FLANGE_R = 20.0                        # the trunnion's flange: inside the side gear's toe
MID_BORE_R = 12.0                      # the side gear's bore between its two 61902s
HUB_R = 12.0                           # the output gear's hub, 36 to 40
KEY = dict(b=5.0, h=5.0, t1=3.0, t2=2.3)   # DIN 6885 A for a 15 mm shaft
RING471 = dict(groove_d=14.3, s=1.0, od=18.0)
RING472 = dict(groove_d=33.7, s=1.2, id=29.0)
CSK_POS = ((13.0, 6.0), (-13.0, 6.0), (13.0, -6.0), (-13.0, -6.0))   # (x, z) of the M3 screws in each trunnion flange
PLD = 0.5715                           # HTD 5M pitch line differential (Gates): OD = PD - 2 PLD
BELT_BACK = 1.54                       # the belt's back above its land (3.6 mm belt, 2.06 mm teeth), drawn as a band
BELT_GAP = 0.05                        # the band's land clear of the pulleys' tips
GROOVE_R, GROOVE_IN = 1.6, 0.5         # each pulley groove drawn round, its centre this far inside the OD
R_BIG = wd.pd5(wd.PULLEYS39[1]) / 2.0 - PLD       # 36 grooves, tip radius
R_SMALL = wd.pd5(wd.PULLEYS39[0]) / 2.0 - PLD     # 18 grooves, tip radius
R_SMALL_FL = R_SMALL + 2.75            # the small pulley's flanges, both sides
SMALL_HUB_R = 9.0
W = 62.0                               # the cheeks' and the base plate's half width in x
BASE_Y = 66.0                          # the base plate's half length in y (2 mm past the cheeks' outer faces)
CHEEK_TOP = 34.0
LIP_R = 13.0                           # the cheek's lip: inside the 6002's outer ring, outside its inner ring
WIN_GAP = 2.0                          # the window in the far cheek, round the motor's 40 mm body
CAP = dict(r=25.0, t=4.0, boss_r=14.0, boss_t=4.0, spig_ri=12.5, pocket_r=11.0, bolt_r=21.0, float_short=0.5)
CAP_ANG = (45.0, 135.0, 225.0, 315.0)
TIE_CB = 5.5                           # the tie bar screws' counterbore depth in the cheek
BRACKET = dict(top=24.0, foot_y=42.0, foot_t=5.0, screw_y=36.0, screw_dx=13.0, hole_r=7.0)
GH_PCD_R = 16.0                        # the gearhead's four M4 on a 32 mm circle
M8_X = (-54.0, 40.0)                   # the cheek-to-base screws, x in the +y cheek (the -y cheek turned 180 deg)
DOWEL_X = (15.0, 55.0)
M8_CB = 8.5                            # their counterbore depth in the base plate's underside


# =============================================================================
# The design: the set, the refusals, the ratings, the layout (no Fusion needed)
# =============================================================================
def design(module=1.5, teeth=40, face=11.0):
    """The bevel set in Shigley's proportions, checked against the prompt's hard constraints, and the layout drawn
    from it. Raises ValueError, naming every reason, for a set the wrist cannot take."""
    m, z, b = float(module), int(round(teeth)), float(face)
    g = wd.bevel(m, z, b)
    sp = wd.space()
    why = []
    if b > g['b_max'] + 1e-9:
        why.append("the face %.2f mm is above Shigley's Table 13-3 limit, 0.3 R or 10 m, whichever is smaller "
                   "(%.2f mm)" % (b, g['b_max']))
    if b < g['R'] / 4.0 - 1e-9:
        why.append('the face %.2f mm is under R / 4 (%.2f mm)' % (b, g['R'] / 4.0))
    if g['zff'] < sp['zff_min'] - 1e-9:
        why.append("the front face is %.3f mm from the apex, nearer than the carrier needs (%.1f mm: the roll "
                   "bearing's radius %.1f, its seat's wall %.1f, the trunnion's flange %.1f and a shim space %.1f)"
                   % (g['zff'], sp['zff_min'], sp['D'] / 2.0, wd.WALL, wd.FLANGE_T, wd.SHIM))
    rated = (z == 40 and abs(g['d'] - 60.0) < 1e-6)
    if not rated:
        why.append('the layout is drawn, and the geometry factors I and J (Figs. 15-6 and 15-7) were read, for the '
                   '40-tooth, 60 mm set only: 40 teeth on 40')
    out = dict(g=g, space=sp, belt=wd.belt())
    if rated:
        br = wd.brief()
        o = wd.loads(br)
        mv = wd.moves(br)
        mo = wd.motors(br)
        cycles = br['life_h'] * 3600.0 / br['cycle']
        duty = wd.rate(g, o['T_in'], mv['wr'] * 60.0 / (2 * math.pi), cycles)
        lim = wd.rate(g, mo['stall_in'], 0.0, 1e3, KA=1.0, credit=False, dynamic=False)
        if duty['SF'] < SF_DUTY or duty['SH'] < SH_DUTY:
            why.append('the teeth fail the duty: S_F %.2f (at least %.1f), S_H %.2f (at least %.1f)'
                       % (duty['SF'], SF_DUTY, duty['SH'], SH_DUTY))
        if lim['SF'] < SF_LIMIT:
            why.append("the teeth fail at the drives' torque limit: S_F %.2f (at least %.1f)" % (lim['SF'], SF_LIMIT))
        out.update(brief=br, loads=o, moves=mv, motors=mo, duty=duty, limit=lim, cycles=cycles)
    if why:
        raise ValueError('Refused (module %g mm, %d teeth, face %g mm): %s.' % (m, z, b, '; '.join(why)))
    out['L'] = wd.layout(dict(m=m, z=z, b=b))
    return out


# =============================================================================
# Placements: (R, t) with R a 3 x 3 rotation (rows) and t a translation in mm
# =============================================================================
I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
IDENT = (I3, (0.0, 0.0, 0.0))
RZ180 = (((-1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0))   # the R side from the L side
RX90 = (((1.0, 0.0, 0.0), (0.0, 0.0, -1.0), (0.0, 1.0, 0.0)), (0.0, 0.0, 0.0))     # a part's y axis onto z


def tr(x, y, z):
    return (I3, (float(x), float(y), float(z)))


def rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return (((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), (0.0, 0.0, 0.0))


def compose(A, B):
    """A after B."""
    RA, tA = A
    RB, tB = B
    R = tuple(tuple(sum(RA[i][k] * RB[k][j] for k in range(3)) for j in range(3)) for i in range(3))
    t = tuple(sum(RA[i][k] * tB[k] for k in range(3)) + tA[i] for i in range(3))
    return (R, t)


def mirrored(places):
    return [compose(RZ180, p) for p in places]


def frame_at(p, axis):
    """A fastener's placement: its local -z along `axis` (into the part), its local origin at p."""
    n = math.sqrt(sum(a * a for a in axis))
    zw = [-a / n for a in axis]
    ref = (1.0, 0.0, 0.0) if abs(zw[0]) < 0.9 else (0.0, 1.0, 0.0)
    dp = sum(ref[i] * zw[i] for i in range(3))
    xw = [ref[i] - dp * zw[i] for i in range(3)]
    nx = math.sqrt(sum(a * a for a in xw))
    xw = [a / nx for a in xw]
    yw = [zw[1] * xw[2] - zw[2] * xw[1], zw[2] * xw[0] - zw[0] * xw[2], zw[0] * xw[1] - zw[1] * xw[0]]
    R = tuple((xw[i], yw[i], zw[i]) for i in range(3))
    return (R, tuple(float(c) for c in p))


def m3d(A):
    R, t = A
    m = adsk.core.Matrix3D.create()
    m.setWithCoordinateSystem(P3(t[0] * CM, t[1] * CM, t[2] * CM), V3(R[0][0], R[1][0], R[2][0]),
                              V3(R[0][1], R[1][1], R[2][1]), V3(R[0][2], R[1][2], R[2][2]))
    return m


# =============================================================================
# Temporary bodies, mm
# =============================================================================
def ycyl(y0, y1, r, x=0.0, z=0.0):
    return mp.cyl((x, y0, z), (x, y1, z), r)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def ytube(y0, y1, ro, ri, x=0.0, z=0.0):
    return mp.tube((x, y0, z), (x, y1, z), ro, ri)


def ztube(z0, z1, ro, ri, x=0.0, y=0.0):
    return mp.tube((x, y, z0), (x, y, z1), ro, ri)


def cone(p0, r0, p1, r1):
    return mp.tbm().createCylinderOrCone(P3(*[v * CM for v in p0]), r0 * CM, P3(*[v * CM for v in p1]), r1 * CM)


def grooves(body, n, r_tip, y0, y1, xc=0.0, zc=0.0):
    """n round grooves, parallel to y, cut into a pulley's tip cylinder."""
    rc = r_tip - GROOVE_IN
    for k in range(n):
        a = 2 * math.pi * k / n
        mp.cut(body, ycyl(y0 - 1.0, y1 + 1.0, GROOVE_R, xc + rc * math.cos(a), zc + rc * math.sin(a)))
    return body


def halfplane(n, k, y0, y1):
    """The slab {n.p <= k} of the xz plane (n a unit vector (nx, nz)), from y0 to y1, as a large box."""
    S = 1000.0
    c = P3(n[0] * (k - S / 2) * CM, (y0 + y1) / 2.0 * CM, n[1] * (k - S / 2) * CM)
    ob = adsk.core.OrientedBoundingBox3D.create(c, V3(n[0], 0.0, n[1]), V3(0.0, 1.0, 0.0), S * CM, (y1 - y0) * CM, S * CM)
    return mp.tbm().createBox(ob)


def hull2(c1, r1, c2, r2, y0, y1):
    """The convex hull of two circles in the xz plane (centres (x, z)), from y0 to y1: both circles and the
    quadrilateral between their outer tangents and the chords through the tangent points."""
    dx, dz = c2[0] - c1[0], c2[1] - c1[1]
    C = math.hypot(dx, dz)
    ux, uz = dx / C, dz / C
    s = (r1 - r2) / C
    q = math.sqrt(1.0 - s * s)
    body = mp.cyl((c1[0], y0, c1[1]), (c1[0], y1, c1[1]), r1)
    mp.union(body, mp.cyl((c2[0], y0, c2[1]), (c2[0], y1, c2[1]), r2))
    quad = None
    for sg in (1.0, -1.0):
        n = (s * ux - sg * q * uz, s * uz + sg * q * ux)          # outward normal of one tangent, n.u = s
        hp = halfplane(n, n[0] * c1[0] + n[1] * c1[1] + r1, y0, y1)
        quad = hp if quad is None else mp.intersect(quad, hp)
    mp.intersect(quad, halfplane((-ux, -uz), -(ux * c1[0] + uz * c1[1] + r1 * s), y0, y1))
    mp.intersect(quad, halfplane((ux, uz), ux * c2[0] + uz * c2[1] + r2 * s, y0, y1))
    mp.union(body, quad)
    return body


def key_slot(zr, x0, x1, kb):
    """A form-A slot along z on the +x side: kb wide, from x0 to x1, round ends at zr[0] and zr[1]."""
    r = kb / 2.0
    s = mp.box(x0, x1, -r, r, zr[0] + r, zr[1] - r)
    mp.union(s, mp.cyl((x0, 0.0, zr[0] + r), (x1, 0.0, zr[0] + r), r), mp.cyl((x0, 0.0, zr[1] - r), (x1, 0.0, zr[1] - r), r))
    return s


# =============================================================================
# The bevel teeth: the heel's section is the tooth of Tredgold's virtual spur gear, drawn in the plane square to
# the pitch element at R; the toe's section is the heel's scaled about the apex (by k), so the tooth runs to it
# =============================================================================
def _inv(a):
    return math.tan(a) - a


def _rot2(p, a):
    c, s = math.cos(a), math.sin(a)
    return (p[0] * c - p[1] * s, p[0] * s + p[1] * c)


def virtual_tooth(rv, m, alpha, ha, hf, backlash, n=10):
    """One tooth of the virtual spur gear (pitch radius rv), centred on +x: involute flanks from the root (or the
    base circle) to the tip, a tip arc, closed ROOT_DROP below the root circle."""
    rb = rv * math.cos(alpha)
    ra, rf = rv + ha, rv - hf
    r_lo = max(rb, rf)

    def t_at(rr):
        return 0.0 if rr <= rb else math.sqrt((rr / rb) ** 2 - 1.0)
    half = (math.pi * m / 2.0 - backlash / 2.0) / (2.0 * rv)
    rot = half + _inv(alpha)
    right, left = [], []
    for i in range(n + 1):
        t = t_at(r_lo) + (t_at(ra) - t_at(r_lo)) * i / n
        px, py = rb * (math.cos(t) + t * math.sin(t)), rb * (math.sin(t) - t * math.cos(t))
        right.append(_rot2((px, -py), rot))
        left.append(_rot2((px, py), -rot))
    a_l, a_r = math.atan2(left[0][1], left[0][0]), math.atan2(right[0][1], right[0][0])
    r_in = rf - ROOT_DROP
    pts = [(r_in * math.cos(a_r), r_in * math.sin(a_r)), (r_in * math.cos(a_l), r_in * math.sin(a_l))]
    pts.extend(left)
    a0, a1 = math.atan2(left[-1][1], left[-1][0]), math.atan2(right[-1][1], right[-1][0])
    pts.extend((ra * math.cos(a0 + (a1 - a0) * j / 5.0), ra * math.sin(a0 + (a1 - a0) * j / 5.0)) for j in range(1, 5))
    pts.extend(reversed(right))
    return pts


def heel_section(g, a, u, backlash):
    """The heel's section (mm, 3D) of the tooth centred on the direction u from the gear's axis a."""
    d = math.radians(g['delta'])
    t = (a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0])
    e = [math.cos(d) * a[i] + math.sin(d) * u[i] for i in range(3)]       # the pitch element
    w = [math.cos(d) * u[i] - math.sin(d) * a[i] for i in range(3)]       # along the back cone, outward
    rv = g['R'] * math.tan(d)
    C = [g['R'] * e[i] - rv * w[i] for i in range(3)]                     # the virtual gear's centre
    pts = virtual_tooth(rv, g['m'], math.radians(g['alpha']), g['ha'], g['hf'], backlash)
    return [tuple(C[i] + px * w[i] + py * t[i] for i in range(3)) for px, py in pts]


def _polygon(comp, plane, pts):
    sk = comp.sketches.add(plane)
    sk.isComputeDeferred = True
    q = []
    for p in pts:
        s = sk.modelToSketchSpace(P3(p[0] * CM, p[1] * CM, p[2] * CM))
        q.append(P3(s.x, s.y, 0.0))
    lines = sk.sketchCurves.sketchLines
    first = lines.addByTwoPoints(q[0], q[1])
    prev = first.endSketchPoint
    for pt in q[2:]:
        prev = lines.addByTwoPoints(prev, pt).endSketchPoint
    lines.addByTwoPoints(prev, first.startSketchPoint)
    sk.isComputeDeferred = False
    if sk.profiles.count < 1:
        raise RuntimeError('a tooth section did not close')
    return sk.profiles.item(0)


def _plane_square_to(comp, sketch_plane, e, dist):
    """A construction plane square to the direction e (through the apex) at dist from the apex: the middle of a
    line from the apex to 2 dist e."""
    sk = comp.sketches.add(sketch_plane)
    p1 = sk.modelToSketchSpace(P3(*[2.0 * dist * c * CM for c in e]))
    p0 = sk.modelToSketchSpace(P3(0.0, 0.0, 0.0))
    ln = sk.sketchCurves.sketchLines.addByTwoPoints(P3(p0.x, p0.y, 0.0), P3(p1.x, p1.y, 0.0))
    planes = comp.constructionPlanes
    ci = planes.createInput()
    try:
        ci.setByDistanceOnPath(ln, VI(0.5))
    except Exception:
        ci.setByDistanceOnPath(comp.features.createPath(ln), VI(0.5))
    return planes.add(ci)


def add_teeth(comp, g, axis_key):
    """Tooth 0 lofted from its heel section to its toe section and joined to the blank, then patterned z times
    about the gear's axis. axis 'y': a side gear, tooth 0 towards +z; axis 'z': the output gear, tooth 0 towards
    its own +x."""
    if axis_key == 'y':
        a, u = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
        splane, pax = comp.yZConstructionPlane, comp.yConstructionAxis
    else:
        a, u = (0.0, 0.0, 1.0), (1.0, 0.0, 0.0)
        splane, pax = comp.xZConstructionPlane, comp.zConstructionAxis
    d = math.radians(g['delta'])
    e = [math.cos(d) * a[i] + math.sin(d) * u[i] for i in range(3)]
    heel = heel_section(g, a, u, BACKLASH)
    toe = [tuple(g['k'] * c for c in p) for p in heel]
    p_heel = _polygon(comp, _plane_square_to(comp, splane, e, g['R']), heel)
    p_toe = _polygon(comp, _plane_square_to(comp, splane, e, g['k'] * g['R']), toe)
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.JoinFeatureOperation)
    li.loftSections.add(p_heel)
    li.loftSections.add(p_toe)
    li.isSolid = True
    try:
        li.participantBodies = [comp.bRepBodies.item(0)]
    except Exception:
        pass
    loft = lofts.add(li)
    feats = adsk.core.ObjectCollection.create()
    feats.add(loft)
    pats = comp.features.circularPatternFeatures
    pin = pats.createInput(feats, pax)
    pin.quantity = VI(g['z'])
    pin.totalAngle = VS('360 deg')
    pats.add(pin)


# =============================================================================
# The parts (temporary bodies, mm). Parts drawn on the L side are given in world coordinates at rest; the R side's
# are the same parts turned 180 deg about z. Fasteners, bearings and washers are given in their own frames.
# =============================================================================
def mk_side_gear(L, g):
    hr, hh = g['heel_root']
    tr_, th = g['toe_root']
    p0, p1 = L['pulley']
    D2 = wd.BEARINGS['61902-2RS1']['D'] / 2.0
    body = cone((0.0, th, 0.0), tr_, (0.0, hh, 0.0), hr)              # the root cone, the front face through the toe's root
    mp.union(body, ycyl(hh, p0, hr))                                 # behind the heel, to the pulley
    pul = grooves(ycyl(p0, p1, R_BIG), wd.PULLEYS39[1], R_BIG, p0, p1)
    mp.union(body, pul)
    mp.cut(body, ycyl(th - 1.0, L['B1'][1], D2), ycyl(L['B1'][1] - 0.5, L['B2'][0] + 0.5, MID_BORE_R),
           ycyl(L['B2'][0], p1 + 1.0, D2))
    return [(body, 'Gear and pulley')]


def mk_output_gear(L, g, phase):
    """In the gear's own frame, which is turned `phase` deg about z: the keyway, at the world's +x, turned back."""
    hr, hh = g['heel_root']
    tr_, th = g['toe_root']
    top = L['hub'][1]
    body = cone((0.0, 0.0, th), tr_, (0.0, 0.0, hh), hr)
    mp.union(body, zcyl(hh, L['gear_back'], hr))
    mp.union(body, zcyl(L['gear_back'], top, HUB_R))
    mp.cut(body, zcyl(th - 1.0, top + 1.0, SHAFT_D / 2.0))
    kw = mp.box(0.0, SHAFT_D / 2.0 + KEY['t2'], -KEY['b'] / 2.0, KEY['b'] / 2.0, th - 1.0, top + 1.0)
    mp.tbm().transform(kw, m3d(rz(-phase)))
    mp.cut(body, kw)
    return [(body, 'Output gear')]


def mk_roll_shaft(L):
    r = SHAFT_D / 2.0
    s = zcyl(L['shaft_bot'], L['collar'][0], r)
    mp.union(s, zcyl(L['collar'][0], L['collar'][1], COLLAR_D / 2.0))
    mp.union(s, zcyl(L['collar'][1], L['shaft_top'], r))
    mp.cut(s, ztube(L['ring471'][0], L['ring471'][1], r + 0.5, RING471['groove_d'] / 2.0))
    mp.cut(s, key_slot(L['key'], r - KEY['t1'], r + 1.0, KEY['b']))
    mp.cut(s, zcyl(L['shaft_top'] - 14.0, L['shaft_top'] + 1.0, 3.0))            # M6, drawn plain
    return [(s, 'Shaft')]


def mk_key(L):
    r = SHAFT_D / 2.0
    return [(key_slot(L['key'], r - KEY['t1'], r - KEY['t1'] + KEY['h'], KEY['b']), 'Key')]


def mk_tool_flange(L):
    f0, f1 = L['flange']
    top = L['shaft_top']
    f = zcyl(f0, f1, 22.5)                                                        # 45 mm, 11 thick (layout)
    mp.cut(f, zcyl(f0 - 1.0, top, SHAFT_D / 2.0))
    mp.cut(f, mp.box(0.0, SHAFT_D / 2.0 + KEY['t2'], -KEY['b'] / 2.0, KEY['b'] / 2.0, f0 - 1.0, top + 0.5))
    mp.cut(f, zcyl(top, f1 + 1.0, 10.0))                                          # the end washer's recess
    for k in range(4):                                                            # ISO 9409-1-31.5-4-M5
        a = math.radians(45.0 + 90.0 * k)
        mp.cut(f, zcyl(f1 - 8.0, f1 + 1.0, 2.5, 15.75 * math.cos(a), 15.75 * math.sin(a)))
    mp.cut(f, zcyl(f1 - 6.0, f1 + 1.0, 2.5, 15.75, 0.0))                          # the 5 mm pin hole on +x
    return [(f, 'Flange')]


def mk_carrier(L, g):
    H = L['y_cs']
    zt, zb = L['z_ct'], L['z_cb']
    blk = mp.box(-H, H, -H, H, zb, zt)
    c = g['toe_root'][0] + g['toe_root'][1] - CHAMFER_GAP        # the output gear's toe cone is r + z = const
    mp.intersect(blk, cone((0.0, 0.0, zb - 10.0), c - (zb - 10.0), (0.0, 0.0, c - 2.0), 2.0))
    mp.cut(blk, zcyl(zb - 1.0, L['B5'][1] + 1.0, 16.0),               # the floating 6002's seat, open below
           zcyl(L['B5'][1] + 1.0, L['B4'][0], 13.0),                  # between the bearings: the fixed one's shoulder
           zcyl(L['B4'][0], zt + 1.0, 16.0),                          # the fixed 6002's seat
           zcyl(L['groove'][0], L['groove'][1], RING472['groove_d'] / 2.0))
    for sy in (1.0, -1.0):
        mp.cut(blk, mp.cyl((0.0, sy * (L['tr_spigot'] - 1.0), 0.0), (0.0, sy * (H + 0.5), 0.0), SHAFT_D / 2.0))
        for x, z in CSK_POS:
            mp.cut(blk, mp.cyl((x, sy * (H - 6.0), z), (x, sy * (H + 0.5), z), 1.5))     # M3, drawn plain
    return [(blk, 'Block')]


def mk_trunnion(L):
    r = SHAFT_D / 2.0
    y0, y1 = L['y_fl']
    t = ycyl(L['tr_spigot'], y0, r)
    mp.union(t, ycyl(y0, y1, FLANGE_R))
    mp.union(t, ycyl(y1, L['tr_end'], r))
    mp.cut(t, ycyl(L['tr_end'] - 15.5, L['tr_end'] + 1.0, 2.5))                  # M5, drawn plain
    for x, z in CSK_POS:
        mp.cut(t, cone((x, y1 + 0.5, z), 3.7, (x, y1 - 1.5, z), 1.7))              # 90 deg countersink
        mp.cut(t, mp.cyl((x, y0 - 0.5, z), (x, y1, z), 1.7))
    return [(t, 'Trunnion')]


def mk_ytube(y0, y1, name):
    return [(ytube(y0, y1, SPACER_D / 2.0, SHAFT_D / 2.0), name)]


def mk_bearing(name):
    bg = wd.BEARINGS[name]
    inner, outer, balls, _g = mp.bearing_bodies(bg['d'], bg['D'], bg['B'], 0.0)
    return [(inner, 'Inner ring'), (outer, 'Outer ring'), (balls, 'Balls')]


def mk_cap_screw(d, length):
    return [(mp.cap_screw(d, length, (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)), 'Screw')]


def mk_csk_m3():
    d, length, dk = 3.0, 8.0, 6.0
    k = (dk - d) / 2.0
    s = cone((0.0, 0.0, 0.0), dk / 2.0, (0.0, 0.0, -k), d / 2.0)
    mp.union(s, zcyl(-length, -k + 0.001, d / 2.0))
    mp.cut(s, zcyl(-1.2, 0.5, 1.0))
    return [(s, 'Screw')]


def mk_low_head_m6():
    d, length, dk, k = 6.0, 12.0, 10.0, 4.0
    s = zcyl(0.0, k, dk / 2.0)
    mp.cut(s, zcyl(k - 2.5, k + 1.0, 2.3))
    mp.union(s, zcyl(-length, 0.001, d / 2.0))
    return [(s, 'Screw')]


def mk_washer(D, d, s):
    return [(ztube(0.0, s, D / 2.0, d / 2.0), 'Washer')]


def mk_dowel():
    return [(zcyl(-8.0, 8.0, 2.5), 'Dowel')]


def mk_base(L, m8, dowels, m5):
    zb0, zb1 = L['base']
    p = mp.box(-W, W, -BASE_Y, BASE_Y, zb0, zb1)
    for x, y in m8:
        mp.cut(p, zcyl(zb0 + M8_CB - 0.5, zb1 + 1.0, 4.5, x, y), zcyl(zb0 - 1.0, zb0 + M8_CB, 7.0, x, y))
    for x, y in dowels:
        mp.cut(p, zcyl(zb1 - 8.0, zb1 + 1.0, 2.5, x, y))
    for x, y in m5:
        mp.cut(p, zcyl(zb1 - 9.0, zb1 + 1.0, 2.5, x, y))                         # M5, drawn plain
    return [(p, 'Plate')]


def mk_cheek(L):
    """The +y cheek; its window takes the far (R) motor's body."""
    c0, c1 = L['cheek']
    zb1 = L['base'][1]
    ym = (c0 + c1) / 2.0
    ch = mp.box(-W, W, c0, c1, zb1, CHEEK_TOP)
    mp.cut(ch, ycyl(c0 - 1.0, L['lip'][1], LIP_R), ycyl(L['lip'][1], c1 + 1.0, wd.BEARINGS['6002-2RSH']['D'] / 2.0))
    hw = wd.MOTOR39['flange'] / 2.0 + WIN_GAP
    mp.cut(ch, mp.box(-L['x_m'] - hw, -L['x_m'] + hw, c0 - 1.0, c1 + 1.0, L['z_m'] - hw, L['z_m'] + hw))
    tx, tz, _tr = L['tie']
    for x in (tx, -tx):
        mp.cut(ch, ycyl(c0 - 1.0, c1 - TIE_CB, 2.75, x, tz), ycyl(c1 - TIE_CB, c1 + 1.0, 4.5, x, tz))
    for a in CAP_ANG:
        ar = math.radians(a)
        mp.cut(ch, ycyl(c1 - 7.0, c1 + 1.0, 2.0, CAP['bolt_r'] * math.cos(ar), CAP['bolt_r'] * math.sin(ar)))   # M4
    for x in M8_X:
        mp.cut(ch, zcyl(zb1 - 1.0, zb1 + 18.0, 4.0, x, ym))                       # M8, drawn plain
    for x in DOWEL_X:
        mp.cut(ch, zcyl(zb1 - 1.0, zb1 + 9.0, 2.5, x, ym))
    return [(ch, 'Cheek')]


def mk_tie_bar(L):
    tx, tz, r = L['tie']
    c0 = L['cheek'][0]
    t = ycyl(-c0, c0, r, tx, tz)
    mp.cut(t, ycyl(c0 - 11.0, c0 + 1.0, 2.5, tx, tz), ycyl(-c0 - 1.0, -c0 + 11.0, 2.5, tx, tz))   # M5, drawn plain
    return [(t, 'Bar')]


def mk_cap(L, spigot_from):
    """A bearing cap on the +y cheek: its spigot to the 6002's outer ring (fixed) or short of it (floating), a
    pocket for the trunnion's end washer and screw."""
    c1 = L['cheek'][1]
    D = wd.BEARINGS['6002-2RSH']['D'] / 2.0
    cap = ycyl(c1, c1 + CAP['t'], CAP['r'])
    mp.union(cap, ycyl(c1 + CAP['t'], c1 + CAP['t'] + CAP['boss_t'], CAP['boss_r']))
    mp.union(cap, ytube(spigot_from, c1, D, CAP['spig_ri']))
    mp.cut(cap, ycyl(L['tr_end'] - 0.1, c1 + CAP['t'] + CAP['boss_t'] - 1.0, CAP['pocket_r']))
    for a in CAP_ANG:
        ar = math.radians(a)
        mp.cut(cap, ycyl(c1 - 0.1, c1 + CAP['t'] + 0.1, 2.25, CAP['bolt_r'] * math.cos(ar), CAP['bolt_r'] * math.sin(ar)))
    return [(cap, 'Cap')]


def mk_bracket(L):
    xm, zm = L['x_m'], L['z_m']
    h = wd.MOTOR39['flange'] / 2.0
    b0, b1 = L['bracket']
    zb1 = L['base'][1]
    br = mp.box(xm - h, xm + h, b0, b1, zb1, zm + BRACKET['top'])
    mp.union(br, mp.box(xm - h, xm + h, b1, BRACKET['foot_y'], zb1, zb1 + BRACKET['foot_t']))
    mp.cut(br, ycyl(b0 - 1.0, b1 + 1.0, BRACKET['hole_r'], xm, zm))
    for a in CAP_ANG:
        ar = math.radians(a)
        mp.cut(br, ycyl(b0 - 1.0, b1 + 1.0, 2.25, xm + GH_PCD_R * math.cos(ar), zm + GH_PCD_R * math.sin(ar)))
    for dx in (-BRACKET['screw_dx'], 0.0, BRACKET['screw_dx']):
        mp.cut(br, zcyl(zb1 - 1.0, zb1 + BRACKET['foot_t'] + 1.0, 2.75, xm + dx, BRACKET['screw_y']))
    return [(br, 'Bracket')]


def mk_motor(L):
    """The servo with its brake and gearhead (outlines MOTOR39 and GEARHEAD, assumed): the gearhead round, its face
    at the bracket, then a 5 mm adapter, the motor and the brake, 40 mm square, running inward across the frame."""
    xm, zm = L['x_m'], L['z_m']
    gh = L['gh_face']
    h = wd.MOTOR39['flange'] / 2.0
    y_gh = gh - wd.GEARHEAD['L']
    y_ad = y_gh - 5.0
    y_mo = y_ad - wd.MOTOR39['L_m']
    m = ycyl(y_gh, gh, wd.GEARHEAD['D'] / 2.0, xm, zm)
    for ya, yb in ((y_ad, y_gh), (y_mo, y_ad), (L['m_end'], y_mo)):
        mp.union(m, mp.box(xm - h, xm + h, ya, yb, zm - h, zm + h))
    mp.cut(m, ycyl(L['m_shaft'][0] - 0.5, gh + 0.5, wd.GEARHEAD['shaft'] / 2.0 + 0.2, xm, zm))
    for a in CAP_ANG:
        ar = math.radians(a)
        mp.cut(m, ycyl(gh - 9.0, gh + 0.5, 2.0, xm + GH_PCD_R * math.cos(ar), zm + GH_PCD_R * math.sin(ar)))   # M4
    return [(m, 'Motor')]


def mk_motor_output(L):
    """The gearhead's output shaft with the small 18-groove pulley, flanged both sides."""
    xm, zm = L['x_m'], L['z_m']
    (f0, f1), (f2, f3) = L['m_flanges']
    s = ycyl(L['m_shaft'][0], L['m_hub'][0], wd.GEARHEAD['shaft'] / 2.0, xm, zm)
    mp.union(s, ycyl(L['m_hub'][0], L['m_hub'][1], SMALL_HUB_R, xm, zm))
    mp.union(s, ycyl(f0, f1, R_SMALL_FL, xm, zm))
    mp.union(s, grooves(ycyl(f1, f2, R_SMALL, xm, zm), wd.PULLEYS39[0], R_SMALL, f1, f2, xm, zm))
    mp.union(s, ycyl(f2, f3, R_SMALL_FL, xm, zm))
    return [(s, 'Shaft and pulley')]


def mk_belt(L):
    y0, y1 = L['belt']
    c2 = (L['x_m'], L['z_m'])
    outer = hull2((0.0, 0.0), R_BIG + BELT_GAP + BELT_BACK, c2, R_SMALL + BELT_GAP + BELT_BACK, y0, y1)
    inner = hull2((0.0, 0.0), R_BIG + BELT_GAP, c2, R_SMALL + BELT_GAP, y0 - 1.0, y1 + 1.0)
    mp.cut(outer, inner)
    return [(outer, 'Belt')]


# =============================================================================
# Components
# =============================================================================
MATERIALS = {
    'steel': ('Steel', 'Steel, Alloy', 'Steel Alloy'),
    'gear steel': ('Steel, Alloy', 'Steel Alloy', 'Steel'),
    '7075': ('Aluminum 7075', 'Aluminum 7075-T6', 'Aluminum 6061', 'Aluminum'),
    'aluminium': ('Aluminum 6061', 'Aluminum'),
    'rubber': ('Rubber', 'Rubber, Silicone', 'Rubber, Butyl'),
}
_mat_cache = {}


def material(key):
    if key in _mat_cache:
        return _mat_cache[key]
    found = None
    app = adsk.core.Application.get()
    for lname in ('Fusion Material Library', 'Fusion 360 Material Library'):
        try:
            lib = app.materialLibraries.itemByName(lname)
        except Exception:
            lib = None
        if not lib:
            continue
        for n in MATERIALS.get(key, ()):
            mt = lib.materials.itemByName(n)
            if mt:
                found = mt
                break
        if found:
            break
    _mat_cache[key] = found
    return found


class Builder:
    def __init__(self, des):
        self.des = des
        self.root = des.rootComponent
        self.kinds = {}

    def group(self, name, grounded=False):
        o = self.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        o.component.name = name
        if grounded:
            o.isGrounded = True
        return o

    def part(self, parent, name, places, make=None, mat=None):
        """Occurrences of the component `name` in `parent` (a component) at the placements given; the component
        and its bodies are made the first time it is asked for."""
        todo = list(places)
        occs = []
        if name not in self.kinds:
            o0 = parent.occurrences.addNewComponent(m3d(todo.pop(0)))
            comp = o0.component
            comp.name = name
            self.kinds[name] = comp
            self.add_bodies(comp, make(), mat)
            occs.append(o0)
        comp = self.kinds[name]
        for p in todo:
            occs.append(parent.occurrences.addExistingComponent(comp, m3d(p)))
        if parent != self.root:
            for o in occs:
                o.isGroundToParent = True       # else a part stays behind when its group turns on its joint
        return occs

    def add_bodies(self, comp, bodies, mat):
        bf = comp.features.baseFeatures.add()
        bf.startEdit()
        for tb, _n in bodies:
            comp.bRepBodies.add(tb, bf)
        bf.finishEdit()
        mt = material(mat) if mat else None
        for i in range(bf.bodies.count):
            body = bf.bodies.item(i)
            try:
                body.name = bodies[i][1]
            except Exception:
                pass
            if mt:
                try:
                    body.material = mt
                except Exception:
                    pass


def parts_list(root):
    """{component name: count} of every occurrence that holds bodies."""
    out = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            out[o.component.name] = out.get(o.component.name, 0) + 1
    return out


# =============================================================================
# Joints, motion links and poses
# =============================================================================
AXES = {'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0), 'z': (0.0, 0.0, 1.0)}
_last = {}
ORDER = ('hL', 'hR', 'inL', 'inR', 'mL', 'mR', 'carrier', 'roll')   # driven joints first, the two drivers last


def _rot(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _probe(j, occ, axis):
    """Turn the joint 0.3 rad and back: +1 or -1 when the occurrence turned about +axis or -axis, 0 when nothing
    was seen to turn, None when it turned about another axis."""
    R0 = _rot(occ)
    try:
        j.jointMotion.rotationValue = 0.3
        R1 = _rot(occ)
    finally:
        j.jointMotion.rotationValue = 0.0
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    w = ((D[2][1] - D[1][2]) / 2.0, (D[0][2] - D[2][0]) / 2.0, (D[1][0] - D[0][1]) / 2.0)
    n = math.sqrt(sum(c * c for c in w))
    if n < 0.05:
        return 0
    c = sum(w[i] * axis[i] for i in range(3)) / n
    return 1 if c > 0.98 else (-1 if c < -0.98 else None)


def _revolute(root, occ1, occ2, point, ax, name, notes):
    """An As-built revolute joint of occ1 on occ2 about the world axis `ax` through `point`, and the sign of its
    value against a right-handed turn about that axis."""
    dirs = {'x': adsk.fusion.JointDirections.XAxisJointDirection,
            'y': adsk.fusion.JointDirections.YAxisJointDirection,
            'z': adsk.fusion.JointDirections.ZAxisJointDirection}
    for k in [ax] + [c for c in 'xyz' if c != ax]:
        inp = root.asBuiltJoints.createInput(occ1, occ2, adsk.fusion.JointGeometry.createByPoint(point))
        inp.setAsRevoluteJointMotion(dirs[k])
        j = root.asBuiltJoints.add(inp)
        s = _probe(j, occ1, AXES[ax])
        if s in (1, -1):
            j.name = name
            return j, s
        if s == 0:
            j.name = name
            notes.append('joint %s: no turn seen when probed, its sign taken as +' % name)
            return j, 1
        j.deleteMe()
    raise RuntimeError('no revolute joint about %s could be made for %s' % (ax, name))


def _set_first(obj, names, value, notes):
    have = dir(obj)
    for n in names:
        if n in have:
            setattr(obj, n, value)
            return True
    notes.append('motion link input has none of %s; it has %s' % (names, [n for n in have if not n.startswith('_')]))
    return False


def _make_link(root, j1, j2, ratio, reverse, notes):
    links = root.motionLinks
    inp = links.createInput(j1, j2)
    _set_first(inp, ('valueOne', 'firstValue'), VS('%.6f deg' % (360.0 / abs(ratio))), notes)
    _set_first(inp, ('valueTwo', 'secondValue'), VS('360 deg'), notes)
    _set_first(inp, ('isReversed', 'isFlipped'), bool(reverse), notes)
    return links.add(inp)


def _set_value(j, v):
    """Set a joint's value; a joint driven by a motion link may refuse, and its driver sets it."""
    try:
        j.jointMotion.rotationValue = v
        return True
    except Exception:
        try:
            j.jointMotion.rotationValue = math.atan2(math.sin(v), math.cos(v))
            return True
        except Exception:
            return False


def _link(root, J, S, a, b, rho, notes):
    """A motion link of joint b on joint a: b's turn = rho times a's (world, right-handed), checked by driving a."""
    r_eff = rho * S[a] * S[b]
    q = abs(r_eff)
    # the reading of the link's two values and its reversal taken first, then the other readings
    variants = [(q, r_eff < 0), (q, r_eff > 0)]
    if abs(q - 1.0) > 1e-9:
        variants += [(1.0 / q, r_eff < 0), (1.0 / q, r_eff > 0)]
    got = None
    for ratio, rev in variants:
        try:
            lk = _make_link(root, J[a], J[b], ratio, rev, notes)
        except Exception as e:
            notes.append('motion link %s-%s not made: %s' % (a, b, e))
            return None
        try:
            J[a].jointMotion.rotationValue = 0.2
            got = J[b].jointMotion.rotationValue
        except Exception as e:
            notes.append('motion link %s-%s not checked (%s), kept as made' % (a, b, e))
            _set_value(J[a], 0.0)
            _set_value(J[b], 0.0)
            return lk
        _set_value(J[a], 0.0)
        _set_value(J[b], 0.0)
        if abs(got - 0.2 * r_eff) < 1e-4:
            return lk
        if abs(got) < 1e-9:
            notes.append('motion link %s-%s: not followed when driven through the API, kept as made' % (a, b))
            return lk
        lk.deleteMe()
    notes.append('motion link %s-%s dropped: it drove %.4f rad for %.4f' % (a, b, got, 0.2 * r_eff))
    return None


def pose(pitch, roll):
    """Pose the wrist at pitch phi and roll psi (deg): the carrier phi, the output psi on it, gear L -psi and gear
    R +psi on the carrier, each belt helper 2 phi, motor L -2 psi and motor R +2 psi on its helper."""
    st = _last
    phi, psi = math.radians(pitch), math.radians(roll)
    want = dict(carrier=phi, roll=psi, inL=-psi, inR=psi, hL=2 * phi, hR=2 * phi, mL=-2 * psi, mR=2 * psi)
    for k in ORDER:
        _set_value(st['joints'][k], st['signs'][k] * want[k])


# =============================================================================
# The build
# =============================================================================
def build(module=1.5, teeth=40, face=11.0):
    global _last
    d = design(module, teeth, face)            # refuses before anything is drawn
    L, g = d['L'], d['g']
    notes = []
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    B = Builder(des)
    phase = 180.0 / g['z']                     # the output gear turned half a pitch: a gap faces each side gear's tooth

    frame = B.group('Frame', grounded=True)
    carrier = B.group('Carrier')
    roll = B.group('Roll output')
    inL = B.group('Input L')
    inR = B.group('Input R')
    hL = B.group('Belt helper L')
    hR = B.group('Belt helper R')
    mL, mR = B.part(root, 'Motor output', [IDENT, RZ180], lambda: mk_motor_output(L), 'steel')
    fc_, cc, rc = frame.component, carrier.component, roll.component

    # --- the inputs: each side gear on two 61902s
    B.part(inL.component, 'Side bevel gear with 36-groove pulley', [IDENT], lambda: mk_side_gear(L, g), 'gear steel')
    b1 = [tr(0, L['B1'][0], 0), tr(0, L['B2'][0], 0)]
    B.part(inL.component, 'Bearing 61902-2RS1', b1, lambda: mk_bearing('61902-2RS1'), 'steel')
    B.part(inR.component, 'Side bevel gear with 36-groove pulley', [RZ180])
    B.part(inR.component, 'Bearing 61902-2RS1', mirrored(b1))

    # --- the roll output
    B.part(rc, 'Roll shaft', [IDENT], lambda: mk_roll_shaft(L), 'steel')
    B.part(rc, 'Output bevel gear', [rz(phase)], lambda: mk_output_gear(L, g, phase), 'gear steel')
    B.part(rc, 'Tool flange ISO 9409-1-31.5-4-M5', [IDENT], lambda: mk_tool_flange(L), '7075')
    B.part(rc, 'Key DIN 6885 A 5x5x18', [IDENT], lambda: mk_key(L), 'steel')
    B.part(rc, 'Spacer S4', [IDENT], lambda: [(ztube(L['S4'][0], L['S4'][1], SPACER_D / 2.0, SHAFT_D / 2.0), 'Spacer')], 'steel')
    B.part(rc, 'End washer 18x6.4x2', [tr(0, 0, L['shaft_top'])], lambda: mk_washer(18.0, 6.4, 2.0), 'steel')
    B.part(rc, 'Low-head screw DIN 7984 M6x12', [tr(0, 0, L['shaft_top'] + 2.0)], mk_low_head_m6, 'steel')
    r471 = L['ring471'][0]
    B.part(rc, 'Retaining ring DIN 471-15', [IDENT],
           lambda: [(ztube(r471, r471 + RING471['s'], RING471['od'] / 2.0, RING471['groove_d'] / 2.0), 'Ring')], 'steel')

    # --- the carrier: block, trunnions, the side gears' spacers, the roll bearings
    B.part(cc, 'Carrier block', [IDENT], lambda: mk_carrier(L, g), '7075')
    B.part(cc, 'Trunnion', [IDENT, RZ180], lambda: mk_trunnion(L), 'steel')
    for nm in ('S1', 'S2', 'S3'):
        B.part(cc, 'Spacer ' + nm, [IDENT, RZ180], lambda nm=nm: mk_ytube(L[nm][0], L[nm][1], 'Spacer'), 'steel')
    csk = [frame_at((x, L['y_fl'][1], z), (0, -1, 0)) for x, z in CSK_POS]
    B.part(cc, 'Countersunk screw ISO 10642 M3x8', csk + mirrored(csk), mk_csk_m3, 'steel')
    tw = frame_at((0, L['tr_end'], 0), (0, -1, 0))
    B.part(cc, 'End washer 18x5.5x2.5', [tw, compose(RZ180, tw)], lambda: mk_washer(18.0, 5.5, 2.5), 'steel')
    ts = frame_at((0, L['tr_end'] + 2.5, 0), (0, -1, 0))
    B.part(cc, 'Cap screw ISO 4762 M5x16', [ts, compose(RZ180, ts)], lambda: mk_cap_screw(5.0, 16.0), 'steel')
    B.part(cc, 'Bearing 6002-2RSH', [compose(tr(0, 0, L['B4'][0]), RX90), compose(tr(0, 0, L['B5'][0]), RX90)],
           lambda: mk_bearing('6002-2RSH'), 'steel')
    b4t = L['B4'][1]
    B.part(cc, 'Retaining ring DIN 472-32', [IDENT],
           lambda: [(ztube(b4t, b4t + RING472['s'], RING472['groove_d'] / 2.0, RING472['id'] / 2.0), 'Ring')], 'steel')

    # --- the frame
    ym = sum(L['cheek']) / 2.0
    m8 = [(x, ym) for x in M8_X]
    dw = [(x, ym) for x in DOWEL_X]
    m5 = [(L['x_m'] + dx, BRACKET['screw_y']) for dx in (-BRACKET['screw_dx'], 0.0, BRACKET['screw_dx'])]
    both = lambda pts: pts + [(-x, -y) for x, y in pts]
    B.part(fc_, 'Base plate', [IDENT], lambda: mk_base(L, both(m8), both(dw), both(m5)), 'aluminium')
    B.part(fc_, 'Side cheek', [IDENT, RZ180], lambda: mk_cheek(L), 'aluminium')
    B.part(fc_, 'Tie bar', [IDENT, RZ180], lambda: mk_tie_bar(L), 'steel')
    B.part(fc_, 'Bearing cap fixed', [IDENT], lambda: mk_cap(L, L['tr_end']), 'aluminium')
    B.part(fc_, 'Bearing cap floating', [RZ180], lambda: mk_cap(L, L['tr_end'] + CAP['float_short']), 'aluminium')
    b3 = tr(0, L['B3'][0], 0)
    B.part(fc_, 'Bearing 6002-2RSH', [b3, compose(RZ180, b3)])
    B.part(fc_, 'Motor bracket', [IDENT, RZ180], lambda: mk_bracket(L), 'aluminium')
    B.part(fc_, 'Servo motor with brake and 15:1 gearhead', [IDENT, RZ180], lambda: mk_motor(L), 'aluminium')
    B.part(fc_, 'Belt HTD %d-5M-%d' % (wd.belt()['stock'], int(wd.BELT39['width'])), [IDENT, RZ180],
           lambda: mk_belt(L), 'rubber')
    c1 = L['cheek'][1]
    caps = [frame_at((CAP['bolt_r'] * math.cos(math.radians(a)), c1 + CAP['t'], CAP['bolt_r'] * math.sin(math.radians(a))),
                     (0, -1, 0)) for a in CAP_ANG]
    B.part(fc_, 'Cap screw ISO 4762 M4x10', caps + mirrored(caps), lambda: mk_cap_screw(4.0, 10.0), 'steel')
    ghs = [frame_at((L['x_m'] + GH_PCD_R * math.cos(math.radians(a)), L['bracket'][1],
                     L['z_m'] + GH_PCD_R * math.sin(math.radians(a))), (0, -1, 0)) for a in CAP_ANG]
    B.part(fc_, 'Cap screw ISO 4762 M4x12', ghs + mirrored(ghs), lambda: mk_cap_screw(4.0, 12.0), 'steel')
    feet = [frame_at((x, y, L['base'][1] + BRACKET['foot_t']), (0, 0, -1)) for x, y in m5]
    B.part(fc_, 'Cap screw ISO 4762 M5x12', feet + mirrored(feet), lambda: mk_cap_screw(5.0, 12.0), 'steel')
    tx, tz, _r = L['tie']
    ties = [frame_at((tx, c1 - TIE_CB, tz), (0, -1, 0)), frame_at((tx, -(c1 - TIE_CB), tz), (0, 1, 0))]
    B.part(fc_, 'Cap screw ISO 4762 M5x16', ties + mirrored(ties))
    m8s = [frame_at((x, y, L['base'][0] + M8_CB), (0, 0, 1)) for x, y in m8]
    B.part(fc_, 'Cap screw ISO 4762 M8x20', m8s + mirrored(m8s), lambda: mk_cap_screw(8.0, 20.0), 'steel')
    dws = [tr(x, y, L['base'][1]) for x, y in dw]
    B.part(fc_, 'Dowel ISO 8734 5x16', dws + mirrored(dws), mk_dowel, 'steel')

    # --- the bevel teeth (features on the gears' components)
    for nm, ax in (('Side bevel gear with 36-groove pulley', 'y'), ('Output bevel gear', 'z')):
        try:
            add_teeth(B.kinds[nm], g, ax)
        except Exception as e:
            notes.append('teeth of the %s not drawn: %s' % (nm, e))

    # --- joints: As-built revolutes, each probed for the sign of its value
    sk = root.sketches.add(root.xZConstructionPlane)
    sk.name = 'Joint origins'

    def spt(x, y, z):
        s = sk.modelToSketchSpace(P3(x * CM, y * CM, z * CM))
        return sk.sketchPoints.add(P3(s.x, s.y, 0.0))
    O = spt(0.0, 0.0, 0.0)
    HLp = spt(L['x_m'], 0.0, L['z_m'])
    HRp = spt(-L['x_m'], 0.0, L['z_m'])
    J, S = {}, {}
    for key, o1, o2, pt, ax, nm in (('carrier', carrier, frame, O, 'y', 'Pitch'),
                                    ('roll', roll, carrier, O, 'z', 'Roll'),
                                    ('inL', inL, carrier, O, 'y', 'Gear L'),
                                    ('inR', inR, carrier, O, 'y', 'Gear R'),
                                    ('hL', hL, frame, HLp, 'y', 'Belt helper L'),
                                    ('hR', hR, frame, HRp, 'y', 'Belt helper R'),
                                    ('mL', mL, hL, HLp, 'y', 'Motor L'),
                                    ('mR', mR, hR, HRp, 'y', 'Motor R')):
        J[key], S[key] = _revolute(root, o1, o2, pt, ax, nm, notes)
    pmax = math.radians(d['brief']['pitch_max'])
    try:
        lim = J['carrier'].jointMotion.rotationLimits
        lim.isMinimumValueEnabled = True
        lim.minimumValue = -pmax
        lim.isMaximumValueEnabled = True
        lim.maximumValue = pmax
    except Exception as e:
        notes.append('pitch limits not set: %s' % e)

    # --- motion links, two joints each
    links = []
    for a, b, rho in (('roll', 'inL', -1.0), ('roll', 'inR', 1.0), ('carrier', 'hL', 2.0), ('carrier', 'hR', 2.0),
                      ('roll', 'mL', -2.0), ('roll', 'mR', 2.0)):
        links.append(_link(root, J, S, a, b, rho, notes))

    _last = dict(occ=dict(frame=frame, carrier=carrier, roll=roll, inL=inL, inR=inR, hL=hL, hR=hR, mL=mL, mR=mR),
                 joints=J, signs=S, links=links, design=d, notes=notes, root=root)
    pose(0.0, 0.0)
    return message(d, parts_list(root), notes)


def message(d, pl, notes):
    g, du, li, mo, o, bt = d['g'], d['duty'], d['limit'], d['motors'], d['loads'], d['belt']
    m39, gh = wd.MOTOR39, wd.GEARHEAD
    lines = [
        VERSION,
        'Bevel set: miter, module %.2f mm, %d teeth on %d, face %.1f mm; pitch diameter %.1f mm, cone distance '
        '%.3f mm, addendum %.2f mm, dedendum %.3f mm (Shigley, Table 13-3).' % (g['m'], g['z'], g['z'], g['b'], g['d'],
                                                                                 g['R'], g['ha'], g['hf']),
        "Front face: %.3f mm from the apex, through the toe's root; the carrier needs %.1f mm." % (g['zff'],
                                                                                              d['space']['zff_min']),
        "Safety factors on the duty (%.2f N m at a side gear, %.2g cycles): S_F %.2f, S_H %.2f; at the drives' torque "
        "limit (%.2f N m): S_F %.2f, S_H %.2f." % (o['T_in'], d['cycles'], du['SF'], du['SH'], mo['stall_in'], li['SF'],
                                                   li['SH']),
        'Motors: two servos with brakes (rated %.2f N m, peak %.2f N m, brake %.2f N m, assumed), %.0f:1 gearheads and '
        '%d/%d belts, %.0f:1 in all; each needs %.3f N m peak, %.3f N m rms, %.0f rev/min, %.3f N m to hold and a '
        '%.3f N m brake.' % (m39['rated'], m39['peak'], m39['brake'], gh['i'], wd.PULLEYS39[0], wd.PULLEYS39[1],
                             mo['i'], mo['peak'], mo['rms'], mo['nmax'], mo['hold'], mo['brake_need']),
        'Belts: HTD %d-5M-%d, centre distance %.3f mm, %.1f teeth in mesh on the small pulley.'
        % (bt['stock'], int(wd.BELT39['width']), bt['C'], bt['tim']),
        'Parts: %d components in %d kinds:' % (sum(pl.values()), len(pl)),
    ]
    lines += ['  %s: %d' % (k, pl[k]) for k in sorted(pl)]
    if notes:
        lines.append('Notes:')
        lines += ['  ' + n for n in notes]
    return '\n'.join(lines)


# =============================================================================
# The dialog
# =============================================================================
_handlers = []


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addFloatSpinnerCommandInput('module', 'Module (mm)', '', 0.5, 5.0, 0.25, 1.5)
        ins.addIntegerSpinnerCommandInput('teeth', 'Teeth', 10, 120, 1, 40)
        ins.addFloatSpinnerCommandInput('face', 'Face width (mm)', '', 1.0, 40.0, 0.5, 11.0)
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        h2 = _Destroy()
        cmd.destroy.add(h2)
        _handlers.append(h2)


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        ins = args.command.commandInputs
        try:
            msg = build(module=ins.itemById('module').value, teeth=ins.itemById('teeth').value,
                        face=ins.itemById('face').value)
            ui.messageBox(msg)
        except ValueError as e:
            ui.messageBox('%s\n%s' % (e, VERSION))
        except Exception:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if __name__ != 'mdfc_check':           # a check that loads this file keeps running after the dialog
            adsk.terminate()


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Pitch-roll wrist',
                                                       'Builds the pitch-roll wrist of Chapter 39')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if __name__ != 'mdfc_check':
            adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('%s failed:\n%s' % (VERSION, traceback.format_exc()))


def stop(context):
    _handlers.clear()
