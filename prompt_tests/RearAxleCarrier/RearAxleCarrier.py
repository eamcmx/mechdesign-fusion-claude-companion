# =============================================================================
# RearAxleCarrier.py  -  the carrier of a light truck's rear axle (Volume 2, Chapter 29), built as a subsystem of
# real parts: the spiral bevel pinion on two tapered roller bearings in the carrier's nose, the crown wheel bolted to
# the differential's case, the case on two tapered roller bearings in the saddles under bearing caps, and an open
# differential of spiders on a cross and two side gears, the parts turning as they do in the axle.
#
# World frame: the pair's apex at the origin, +X forward along the pinion, the axle along Y (+Y on the left), +Z up.
# The numbers come from projects/rear_axle/lib/axle_design.py; the teeth from spiral_bevel.py (member(), section(),
# to_world()); the parts from common/lib/mdfc_parts.py. All three are loaded, not copied. Sizes in mm; the API in cm.
# =============================================================================
import importlib
import math
import os
import re
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_root():
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return d
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % __file__)
        d = up


_COMMON = os.path.join(_companion_root(), 'common')
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402  (puts every lib/ folder on sys.path)
importlib.reload(CP)
import axle_design as ad  # noqa: E402
import spiral_bevel as sb  # noqa: E402
import mdfc_parts as P  # noqa: E402
importlib.reload(ad)
importlib.reload(sb)
importlib.reload(P)

VERSION = 'RearAxleCarrier prompt-test v1 (12 Nov 2026)'

CM = 0.1
P3 = adsk.core.Point3D.create
O = (0.0, 0.0, 0.0)
STEEL, IRON = 'Steel', 'Iron, Cast'

# ---------- the pair and the differential ----------
Z_P, Z_C = 12, 47                       # pinion and crown wheel
Z_S, Z_Q = 22, 15                       # side gears and spiders
BETA, ALPHA = 35.0, 20.0                # spiral angle and normal pressure angle, degrees
KA_PEAK = 1.0                           # the design torque is the peak itself
KA_CRUISE = ad.KA_TABLE[('light', 'light')]
CLEAR_MIN, WALL, GAP = 195.0, 8.0, 6.0  # ground clearance, the housing's wall and the gap to the teeth
F_DIFF = 24.0                           # the differential's face (axle_design.differential's own value)
N_SECTIONS = 7

# ---------- tapered roller bearings: SKF d, D, T, a from axle_design; the cone's B and the cup's C (ISO 355) ----
TRB_BC = {'32308': (33.0, 27.0), '32309': (36.0, 30.0), '32213': (31.0, 27.0)}
BORE_PLUS = 0.05                        # each cup's bore is 0.1 mm over the cup, 0.05 a side

# ---------- the pinion's shaft ----------
BACK_LEN, R_COLLAR, L_COLLAR = 2.5, 29.0, 3.0
SHIM_T, R_SHIM_IN, R_SHIM_OUT = 1.5, 22.6, 28.0
R_MID = 21.0
S_PINION = 111.0                        # between the pinion bearings' pressure points (chosen)
R_SPACER_IN, R_SPACER_OUT = 21.2, 24.0
SPL_P = (32.0, 38.0, 8, 6.0)            # 8 x 32 x 38, teeth 6 wide
L_SPL = 36.0
R_THREAD = 15.0                         # M30
FL_HUB, FL_DISC, R_FL_HUB, R_FL_DISC = 24.0, 14.0, 30.0, 65.0
WASHER_T, R_WASHER_IN, R_WASHER_OUT = 4.0, 15.5, 28.0
SEAL_AT, SEAL_W, R_SEAL_IN, R_SEAL_OUT = 8.0, 10.0, 30.0, 42.5
NOSE_FRONT = 18.0                       # the nose's front face beyond bearing B's back face

# ---------- the carrier ----------
X_PLATE, T_PLATE, R_PLATE, HOLE_PLATE = 170.0, 16.0, 200.0, 13.0
R_NOSE_A, R_NOSE_B, R_RIDGE, R_FRONT = 62.0, 56.0, 40.0, 41.0
Y_SHOULDER = 74.0
Y_LEG0, Y_LEG1, Z_LEG = 78.0, 125.0, 85.0
Y_CAPBOLT = (Y_LEG0 + Y_LEG1) / 2.0
CAP_DEPTH, Z_CAPBOLT, HOLE_CAP, TAP_DEPTH, L_CAPBOLT = 75.0, 68.0, 13.2, 16.0, 90.0
L_ADJ, R_ADJ_IN = 18.0, 45.0

# ---------- the case, the crown wheel and the differential ----------
R_CASE, R_CAVITY, R_BACK = 84.0, 76.0, 74.0
R_BOSS, R_FLANGE, T_FLANGE = 46.0, 106.0, 14.0
R_SPIGOT = 170.4 / 2.0                  # the crown wheel's bore
R_BOLT_C, N_BOLT_C, L_CBOLT = 186.0 / 2.0, 10, 35.0
R_CASE_BORE, R_HUB = 25.5, 25.0
SPL_S = (35.0, 40.0, 8, 7.0)            # the side gears' hubs, 8 x 35 x 40
R_ARM_END, R_SPIDER_PLAY, R_WINDOW = 82.0, 0.1, 24.0


class Refused(ValueError):
    """A design the prompt's hard constraints refuse."""


# =============================================================================
# The numbers (no Fusion)
# =============================================================================
def _pt(g, s, v):
    """(radius, axial) in the gear's own frame of the point v outward of the virtual pitch circle, section at s."""
    sd, cd = math.sin(g['delta']), math.cos(g['delta'])
    return s * (g['Ae'] * sd + v * cd), s * (g['Ae'] * cd - v * sd)


def _root(g):
    """The root cone at the toe and the heel, (axial, radius), and its half angle."""
    rt, zt = _pt(g, g['k'], -g['hf'])
    rh, zh = _pt(g, 1.0, -g['hf'])
    return (zt, rt), (zh, rh), math.atan2(rh, zh)


def _heel_reach(g):
    """The farthest the heel section reaches from the apex, and its largest angle from the gear's axis."""
    sd, cd = math.sin(g['delta']), math.cos(g['delta'])
    far, ang = 0.0, 0.0
    for (u, v) in sb.tooth_uv(g):
        x, y, z = g['Ae'] * sd + v * cd, u, g['Ae'] * cd - v * sd
        far = max(far, math.sqrt(x * x + y * y + z * z))
        ang = max(ang, math.atan2(math.hypot(x, y), z))
    return far, ang


def layout(gp, gc, gs, gq):
    """Where the parts sit (mm), from the gears' cones, SKF's T and a and the fixed sizes of the prompt."""
    L = {}
    # the pinion along +X: the root cone, the back of the head, the collar, the shim and bearing A
    L['p_toe'], L['p_heel'], _ = _root(gp)
    L['p_foot'] = _pt(gp, 1.0, -gp['hf'] - 1.0)[1]
    L['p_min'] = _pt(gp, gp['k'], gp['ha'])[1]          # the toe's tips, the pinion's nearest point to the apex
    L['x_hb'] = L['p_foot'] + BACK_LEN
    L['x_c'] = L['x_hb'] + L_COLLAR
    L['x_A'] = L['x_c'] + SHIM_T
    TA, aA = ad.SKF_TRB['32309'][2:4]
    TB, aB = ad.SKF_TRB['32308'][2:4]
    L['x_ppA'] = L['x_A'] + TA - aA                       # back to back: each pressure point T - a from the cone's back
    L['x_mean'] = gp['Rm'] * math.cos(gp['delta'])
    L['a'] = L['x_ppA'] - L['x_mean']
    L['s'] = S_PINION
    L['x_ppB'] = L['x_ppA'] + S_PINION
    L['x_B'] = L['x_ppB'] + TB - aB                       # bearing B's back face, towards the flange
    L['x_nf'] = L['x_B'] + NOSE_FRONT
    L['x_fl'] = L['x_B'] + FL_HUB + FL_DISC                 # the flange's front face
    L['x_end'] = L['x_fl'] + WASHER_T + 0.8 * 30.0 + 3.0
    # the crown wheel along -Y
    L['c_toe'], L['c_heel'], _ = _root(gc)
    foot = _pt(gc, 1.0, -gc['hf'] - 1.0)[1]
    L['z_back'] = math.ceil((foot + 1.0) * 2.0) / 2.0       # its back face, against the case's flange
    L['z_front'] = L['z_back'] - (L_CBOLT - T_FLANGE)       # its inner ring's front face, where the bolts end
    L['R_ring'] = L['p_min'] - 1.0                          # the inner ring stays 1 mm short of the pinion's toe
    L['da'] = 2.0 * _pt(gc, 1.0, gc['ha'])[0]
    # the case's bearings, face to face: cones' backs on the shoulders at -/+74, pressure points T - a further out
    TC, aC = ad.SKF_TRB['32213'][2:4]
    L['y_ppC'] = Y_SHOULDER + TC - aC
    L['L'] = 2.0 * L['y_ppC']
    L['c'] = L['y_ppC'] - gc['Rm'] * math.cos(gc['delta'])
    L['y_journal'] = Y_SHOULDER + TRB_BC['32213'][0] + 3.0
    # the differential's gears
    L['s_toe'], L['s_heel'], L['s_root'] = _root(gs)
    L['q_toe'], L['q_heel'], L['q_root'] = _root(gq)
    fs, _ = _heel_reach(gs)
    fq, L['q_spread'] = _heel_reach(gq)
    L['diff_reach'] = max(fs, fq)
    return L


def design(m=6.5, F=52.0, m_d=5.5, N=4):
    """Everything the carrier is decided by, for the final-drive module m at the heel, the face F, the differential's
    module m_d and N spiders. 'refusals' lists what the hard constraints refuse, each saying why."""
    if not (m > 0 and F > 0 and m_d > 0):
        raise Refused('the modules and the face must be positive')
    if int(N) != N or N < 1:
        raise Refused('the number of spiders must be a whole number, at least 1')
    N = int(N)
    b = ad.brief()
    d = ad.duty(b)
    u = ad.gear_duty(b)
    g = ad.bevel_geometry(Z_P, Z_C, m, F, BETA, ALPHA)
    D = dict(m=m, F=F, m_d=m_d, N=N, b=b, d=d, u=u, g=g)
    D['peak'] = ad.rate(g, u['T1_peak'], u['n1_peak'], u['n_peak'], KA_PEAK)
    D['cruise'] = ad.rate(g, u['T1_cruise'], u['n1_cruise'], u['n_cruise'], KA_CRUISE)
    D['clearance'] = b['r_stat'] - g['da2'] / 2.0 - WALL - GAP
    D['diff'] = ad.differential(d, z_s=Z_S, z_p=Z_Q, N=N, m=m_d, F=F_DIFF)
    D['gp'] = sb.member(Z_P, Z_C, m, F, BETA, ALPHA, pinion=True)
    D['gc'] = sb.member(Z_C, Z_P, m, F, BETA, ALPHA, pinion=False)
    D['gs'] = sb.member(Z_S, Z_Q, m_d, F_DIFF, 0.0, ALPHA, pinion=False)
    D['gq'] = sb.member(Z_Q, Z_S, m_d, F_DIFF, 0.0, ALPHA, pinion=True)
    ref = []
    if D['peak']['SF1'] < 1.0:
        ref.append("module %.2f: the pinion's bending factor of %.3f at the design torque is under 1"
                   % (m, D['peak']['SF1']))
    if D['clearance'] < CLEAR_MIN:
        ref.append('module %.2f: the crown wheel of %.1f mm leaves %.1f mm of ground clearance, under %.0f mm'
                   % (m, g['da2'], D['clearance'], CLEAR_MIN))
    if F > g['Ae'] / 3.0:
        ref.append('a face of %.1f mm is over a third of the cone distance (%.1f mm)' % (F, g['Ae'] / 3.0))
    q = 2.0 * Z_S / N
    if abs(q - round(q)) > 1e-9:
        ref.append('%d spiders: 2 z_s / N = %.2f is not a whole number, so the spiders cannot all mesh' % (N, q))
    if F < g['Ae']:
        L = layout(D['gp'], D['gc'], D['gs'], D['gq'])
        D['layout'] = L
        D['pb'] = ad.pinion_bearings(D['g'], u, a=L['a'], s=L['s'])
        D['cb'] = ad.case_bearings(D['g'], u, d, c=L['c'], L=L['L'])
        D['bolts'] = ad.crown_bolts(d, n=N_BOLT_C, r_b=R_BOLT_C)
        # what the fixed sizes of the case and the crown wheel still allow
        if R_BOLT_C + 6.0 > L['p_min'] - 0.5:
            ref.append("the M12 bolts on 186 mm would reach the pinion's toe (%.1f mm from the axle)" % L['p_min'])
        if L['diff_reach'] > R_BACK - 0.05:
            ref.append("the differential's teeth at module %.2f reach past the gears' spherical backs of radius %.0f mm"
                       % (m_d, R_BACK))
        if N > 1 and 2.0 * L['q_spread'] > 2.0 * math.pi / N - 0.02:
            ref.append('%d spiders do not fit round the side gears' % N)
    D['refusals'] = ref
    return D


# =============================================================================
# Frames and turns
# =============================================================================
FRAME_L = ((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), (0.0, 1.0, 0.0))     # side gear L: own z = +Y, own x = +Z
FRAME_R = ((0.0, 0.0, 1.0), (-1.0, 0.0, 0.0), (0.0, -1.0, 0.0))   # side gear R: own z = -Y, own x = +Z


def arm(N, k):
    """Spider k's arm (k from 0), outwards: in the XZ plane at k 360/N degrees from +Z towards +X."""
    p = 2.0 * math.pi * k / N
    return (math.sin(p), 0.0, math.cos(p)), p


def spider_frame(N, k):
    """Own z along the arm, own x towards side gear L (+Y): the pitch cones touch in the own xz plane."""
    a, p = arm(N, k)
    return ((0.0, 1.0, 0.0), (-math.cos(p), 0.0, math.sin(p)), a)


def spider_phase(N, k):
    """The spider's tooth 0 from its own x: a gap where side gear L has a tooth, rolled on by z_s/z_p of the arm's
    angle round L (tooth 0 of L on +Z; side gear R half a pitch round)."""
    return Z_S / float(Z_Q) * arm(N, k)[1] - math.pi / Z_Q


def _turn(Fa, Fb, psi):
    """The 3 x 3 rotation taking what was drawn in frame Fa to frame Fb turned psi about Fb's own z."""
    c, s = math.cos(psi), math.sin(psi)
    ex, ey, ez = Fb
    cols = ([c * ex[i] + s * ey[i] for i in range(3)], [-s * ex[i] + c * ey[i] for i in range(3)], ez)
    return [[sum(cols[k][i] * Fa[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _mat(R):
    mx = adsk.core.Matrix3D.create()
    for i in range(3):
        for j in range(3):
            mx.setCell(i, j, R[i][j])
    return mx


def _at(v, t):
    return (v[0] * t, v[1] * t, v[2] * t)


def _frustum(p0, r0, p1, r1):
    return P.tbm().createCylinderOrCone(P3(*_at(p0, CM)), r0 * CM, P3(*_at(p1, CM)), r1 * CM)


# =============================================================================
# Teeth: one tooth lofted through its transverse sections, then turned round the blank
# =============================================================================
def _loft_tooth(comp, sections):
    """A tooth lofted through closed sections (lists of world points, mm) in comp; returned as a temporary body. The
    loft is deleted afterwards so that the part keeps one body."""
    marks = comp.sketches.add(comp.xYConstructionPlane)
    marks.name = 'tooth section points'
    sketches, planes, profiles = [marks], [], []
    for pts in sections:
        i0, i1 = len(pts) - 2, len(pts) - 1                 # the two corners of the foot
        mid = [(pts[i0][c] + pts[i1][c]) / 2.0 for c in range(3)]
        i2 = max(range(len(pts)), key=lambda i: math.dist(pts[i], mid))
        sps = [marks.sketchPoints.add(marks.modelToSketchSpace(P3(*_at(pts[i], CM)))) for i in (i0, i1, i2)]
        cpi = comp.constructionPlanes.createInput()
        cpi.setByThreePoints(sps[0], sps[1], sps[2])
        pl = comp.constructionPlanes.add(cpi)
        planes.append(pl)
        sk = comp.sketches.add(pl)
        sketches.append(sk)
        q = [sk.modelToSketchSpace(P3(*_at(p, CM))) for p in pts]
        off = max(abs(a.z) for a in q)
        if off > 1e-4:
            raise RuntimeError('a tooth section is not flat (%.1e cm off its plane)' % off)
        q = [P3(a.x, a.y, 0.0) for a in q]
        lines = sk.sketchCurves.sketchLines
        first = lines.addByTwoPoints(q[0], q[1])
        prev = first.endSketchPoint
        for a in q[2:]:
            prev = lines.addByTwoPoints(prev, a).endSketchPoint
        lines.addByTwoPoints(prev, first.startSketchPoint)
        if sk.profiles.count < 1:
            raise RuntimeError('a tooth section gave no profile')
        profiles.append(sk.profiles.item(0))
    lofts = comp.features.loftFeatures
    li = lofts.createInput(adsk.fusion.FeatureOperations.NewBodyFeatureOperation)
    for pr in profiles:
        li.loftSections.add(pr)
    li.isSolid = True
    loft = lofts.add(li)
    tooth = P.tbm().copy(loft.bodies.item(0))
    loft.deleteMe()
    for sk in sketches:
        sk.isVisible = False
    for pl in planes:
        pl.isLightBulbOn = False
    adsk.doEvents()
    return tooth


def _add_teeth(blank, tooth, Fa, Fb, phase, z):
    """z copies of the tooth (drawn in Fa at phase 0) joined to the blank in frame Fb, the first at `phase`."""
    for j in range(z):
        t = P.tbm().copy(tooth)
        P.tbm().transform(t, _mat(_turn(Fa, Fb, phase + 2.0 * math.pi * j / z)))
        P.union(blank, t)
        if j % 6 == 5:
            adsk.doEvents()
    return blank


def _sections(g, frame, phase, n):
    return [[sb.to_world(frame, p) for p in sb.section(g, s, phase)] for s in sb.scales(g, n)]


# =============================================================================
# Parts
# =============================================================================
def _one_body(comp):
    """A body that came in as several lumps (a cone and its rollers) joined back into one."""
    if comp.bRepBodies.count <= 1:
        return
    tools = adsk.core.ObjectCollection.create()
    for i in range(1, comp.bRepBodies.count):
        tools.add(comp.bRepBodies.item(i))
    ci = comp.features.combineFeatures.createInput(comp.bRepBodies.item(0), tools)
    ci.operation = adsk.fusion.FeatureOperations.JoinFeatureOperation
    comp.features.combineFeatures.add(ci)


def _fill(occ, tb, material):
    P.add_body(occ, tb, occ.component.name, material)
    _one_body(occ.component)
    adsk.doEvents()
    return occ


def _part(parent, name, tb, material):
    return _fill(P.new_occ(parent, name), tb, material)


def _bearings(L):
    """The four tapered roller bearings, (cone with its rollers, cup), each from its cone's back face."""
    out = {}
    for key, code, p0, axis in (('A', '32309', (L['x_A'], 0.0, 0.0), (1.0, 0.0, 0.0)),
                                ('B', '32308', (L['x_B'], 0.0, 0.0), (-1.0, 0.0, 0.0)),
                                ('C1', '32213', (0.0, -Y_SHOULDER, 0.0), (0.0, -1.0, 0.0)),
                                ('C2', '32213', (0.0, Y_SHOULDER, 0.0), (0.0, 1.0, 0.0))):
        d, D, T = ad.SKF_TRB[code][:3]
        B, C = TRB_BC[code]
        cone, cup, _ = P.taper_bodies(d, D, T, B, C, p0, axis)
        out[key] = (code, cone, cup)
    return out


def _casting(L):
    """The carrier: the mounting plate, the nose with the cups' bores and the seal's, two legs with the saddles."""
    x_A, x_B = L['x_A'], L['x_B']
    DA, TA = ad.SKF_TRB['32309'][1:3]
    DB, TB = ad.SKF_TRB['32308'][1:3]
    CB = TRB_BC['32308'][1]
    DC = ad.SKF_TRB['32213'][1]
    body = P.cyl((X_PLATE, 0, 0), (X_PLATE + T_PLATE, 0, 0), R_PLATE)
    P.intersect(body, P.box(X_PLATE - 1, X_PLATE + T_PLATE + 1, -R_PLATE - 1, R_PLATE + 1, ad.JOINT_BOTTOM,
                            R_PLATE + 1))                                   # cut flat 166 mm below the axle
    for (y, z) in ad.joint_bolts():
        P.cut(body, P.cyl((X_PLATE - 1, y, z), (X_PLATE + T_PLATE + 1, y, z), HOLE_PLATE / 2.0))
    x_rear = min(x_A + 6.0, X_PLATE)
    x_step = x_B - 36.0
    P.union(body, P.cyl((x_rear, 0, 0), (x_step, 0, 0), R_NOSE_A), P.cyl((x_step, 0, 0), (L['x_nf'], 0, 0), R_NOSE_B))
    for sgn in (1, -1):
        y0, y1 = sorted((sgn * Y_LEG0, sgn * Y_LEG1))
        P.union(body, P.box(0.0, X_PLATE + 1.0, y0, y1, -Z_LEG, Z_LEG))
    # the saddles and the holes tapped for the caps' bolts
    for sgn in (1, -1):
        P.cut(body, P.cyl((0, sgn * (Y_LEG0 - 1), 0), (0, sgn * (Y_LEG1 + 1), 0), DC / 2.0 + BORE_PLUS))
        for z in (-Z_CAPBOLT, Z_CAPBOLT):
            P.cut(body, P.cyl((-1.0, sgn * Y_CAPBOLT, z), (TAP_DEPTH, sgn * Y_CAPBOLT, z), 6.0))
    # the nose: cup A's bore, the ridge both cups' backs sit against, cup B's bore, the seal's bore
    P.cut(body, P.cyl((x_rear - 1, 0, 0), (x_A + TA, 0, 0), DA / 2.0 + BORE_PLUS))
    P.cut(body, P.cyl((x_A + TA, 0, 0), (x_B - TB, 0, 0), R_RIDGE))
    P.cut(body, P.cyl((x_B - TB, 0, 0), (x_B - (TB - CB) + 1.0, 0, 0), DB / 2.0 + BORE_PLUS))
    P.cut(body, P.cyl((x_B - (TB - CB) + 1.0, 0, 0), (x_B + SEAL_AT, 0, 0), R_FRONT))
    P.cut(body, P.cyl((x_B + SEAL_AT, 0, 0), (L['x_nf'] + 1.0, 0, 0), R_SEAL_OUT + BORE_PLUS))
    return body


def _cap(sgn):
    DC = ad.SKF_TRB['32213'][1]
    y0, y1 = sorted((sgn * Y_LEG0, sgn * Y_LEG1))
    c = P.box(-CAP_DEPTH, 0.0, y0, y1, -Z_LEG, Z_LEG)
    P.cut(c, P.cyl((0, sgn * (Y_LEG0 - 1), 0), (0, sgn * (Y_LEG1 + 1), 0), DC / 2.0 + BORE_PLUS))
    for z in (-Z_CAPBOLT, Z_CAPBOLT):
        P.cut(c, P.cyl((-CAP_DEPTH - 1, sgn * Y_CAPBOLT, z), (1.0, sgn * Y_CAPBOLT, z), HOLE_CAP / 2.0))
    return c


def _carrier(comp, D, brg):
    L = D['layout']
    _part(comp, 'Carrier casting', _casting(L), IRON)
    for side, sgn in (('L', 1), ('R', -1)):
        _part(comp, 'Bearing cap ' + side, _cap(sgn), IRON)
    k = 0
    for sgn in (1, -1):
        for z in (-Z_CAPBOLT, Z_CAPBOLT):
            k += 1
            _part(comp, 'Cap bolt M12x90 %d' % k,
                  P.cap_screw(12.0, L_CAPBOLT, (-CAP_DEPTH, sgn * Y_CAPBOLT, z), (1.0, 0.0, 0.0)), STEEL)
    TC, DC = ad.SKF_TRB['32213'][2], ad.SKF_TRB['32213'][1]
    for side, sgn in (('L', 1), ('R', -1)):
        y0 = sgn * (Y_SHOULDER + TC)                      # behind the cup's back face
        _part(comp, 'Adjuster ' + side, P.tube((0, y0, 0), (0, y0 + sgn * L_ADJ, 0), DC / 2.0, R_ADJ_IN), STEEL)
    for key in ('A', 'B', 'C1', 'C2'):
        code, cone, cup = brg[key]
        _part(comp, 'Bearing %s %s cup' % (key, code), cup, STEEL)
    x_B = L['x_B']
    _part(comp, 'Pinion seal 60x85x10',
          P.tube((x_B + SEAL_AT, 0, 0), (x_B + SEAL_AT + SEAL_W, 0, 0), R_SEAL_OUT, R_SEAL_IN), STEEL)


def _pinion_blank(L):
    (xt, rt), (xh, rh) = L['p_toe'], L['p_heel']
    x_A, x_B = L['x_A'], L['x_B']
    BA, BB = TRB_BC['32309'][0], TRB_BC['32308'][0]
    s = _frustum((xt, 0, 0), rt, (xh, 0, 0), rh)
    P.union(s, P.cyl((xh, 0, 0), (L['x_hb'], 0, 0), rh))                         # the back of the head
    P.union(s, P.cyl((L['x_hb'], 0, 0), (L['x_c'], 0, 0), R_COLLAR))             # the collar
    P.union(s, P.cyl((L['x_c'], 0, 0), (x_A + BA, 0, 0), ad.SKF_TRB['32309'][0] / 2.0))   # bearing A's seat
    P.union(s, P.cyl((x_A + BA, 0, 0), (x_B - BB - 3.0, 0, 0), R_MID))
    P.union(s, P.cyl((x_B - BB - 3.0, 0, 0), (x_B, 0, 0), ad.SKF_TRB['32308'][0] / 2.0))  # bearing B's seat
    P.union(s, P.spline_shaft((x_B, 0, 0), (x_B + L_SPL, 0, 0), *SPL_P))
    P.union(s, P.cyl((x_B + L_SPL, 0, 0), (L['x_end'], 0, 0), R_THREAD))            # M30
    return s


def _pinion(comp, D, brg):
    L = D['layout']
    gp = D['gp']
    x_A, x_B = L['x_A'], L['x_B']
    BA, BB = TRB_BC['32309'][0], TRB_BC['32308'][0]
    occ = P.new_occ(comp, 'Pinion shaft z12')
    tooth = _loft_tooth(occ.component, _sections(gp, sb.PINION_FRAME, 0.0, N_SECTIONS))
    shaft = _add_teeth(_pinion_blank(L), tooth, sb.PINION_FRAME, sb.PINION_FRAME, 0.0, Z_P)
    _fill(occ, shaft, STEEL)
    _part(comp, 'Pinion depth shim', P.tube((L['x_c'], 0, 0), (x_A, 0, 0), R_SHIM_OUT, R_SHIM_IN), STEEL)
    _part(comp, 'Bearing A 32309 cone', brg['A'][1], STEEL)
    _part(comp, 'Spacer', P.tube((x_A + BA, 0, 0), (x_B - BB, 0, 0), R_SPACER_OUT, R_SPACER_IN), STEEL)
    _part(comp, 'Bearing B 32308 cone', brg['B'][1], STEEL)
    fl = P.cyl((x_B, 0, 0), (x_B + FL_HUB, 0, 0), R_FL_HUB)
    P.union(fl, P.cyl((x_B + FL_HUB, 0, 0), (L['x_fl'], 0, 0), R_FL_DISC))
    P.cut(fl, P.spline_hole((x_B - 1.0, 0, 0), (L['x_fl'] + 1.0, 0, 0), *SPL_P))
    for k in range(4):
        a = math.pi / 4.0 + k * math.pi / 2.0
        p = (52.0 * math.cos(a), 52.0 * math.sin(a))
        P.cut(fl, P.cyl((x_B + FL_HUB - 1.0, p[0], p[1]), (L['x_fl'] + 1.0, p[0], p[1]), 5.5))
    _part(comp, 'Companion flange', fl, STEEL)
    _part(comp, 'Washer', P.tube((L['x_fl'], 0, 0), (L['x_fl'] + WASHER_T, 0, 0), R_WASHER_OUT, R_WASHER_IN), STEEL)
    _part(comp, 'Pinion nut M30', P.nut(30.0, (L['x_fl'] + WASHER_T, 0, 0), (1.0, 0.0, 0.0)), STEEL)


def _case(L, N, r_pin):
    """One casting: a spherical cavity, two windows, the flange for ten M12 bolts on 186 mm, the spigot, the
    shoulders at -/+74 and the bearings' seats outside them, the bores for the side gears' hubs and the cross."""
    yf1 = -L['z_back']
    yf0 = yf1 - T_FLANGE
    dC = ad.SKF_TRB['32213'][0]
    c = P.sphere(O, R_CASE)
    P.union(c, P.cyl((0, -Y_SHOULDER, 0), (0, Y_SHOULDER, 0), R_BOSS),
            P.cyl((0, yf0, 0), (0, yf1, 0), R_FLANGE), P.cyl((0, yf1, 0), (0, yf1 + 8.0, 0), R_SPIGOT))
    P.intersect(c, P.box(-R_FLANGE - 1, R_FLANGE + 1, -Y_SHOULDER, Y_SHOULDER, -R_FLANGE - 1, R_FLANGE + 1))
    for sgn in (1, -1):
        P.union(c, P.cyl((0, sgn * (Y_SHOULDER - 1.0), 0), (0, sgn * L['y_journal'], 0), dC / 2.0))
    P.cut(c, P.sphere(O, R_CAVITY))
    P.cut(c, P.cyl((0, -L['y_journal'] - 1, 0), (0, L['y_journal'] + 1, 0), R_CASE_BORE))
    for k in range(N):
        a = arm(N, k)[0]
        P.cut(c, P.cyl(_at(a, 60.0), _at(a, R_CASE + 6.0), r_pin))
    for ang in (45.0, 225.0):
        w = (math.sin(math.radians(ang)), 0.0, math.cos(math.radians(ang)))
        P.cut(c, P.cyl(_at(w, 55.0), _at(w, R_CASE + 10.0), R_WINDOW))
    for j in range(N_BOLT_C):
        a = 2.0 * math.pi * j / N_BOLT_C
        x, z = R_BOLT_C * math.cos(a), R_BOLT_C * math.sin(a)
        P.cut(c, P.cyl((x, yf0 - 1.0, z), (x, yf1 + 1.0, z), 6.5))
    return c


def _crown(L, tooth, gc):
    (zt, rt), (zh, rh) = L['c_toe'], L['c_heel']
    zb, zf = L['z_back'], L['z_front']
    c = _frustum((0, -zt, 0), rt, (0, -zh, 0), rh)                               # below the root cone
    P.union(c, P.cyl((0, -zh, 0), (0, -zb, 0), rh))                              # behind the heel
    P.union(c, P.cyl((0, -zf, 0), (0, -zb, 0), L['R_ring']))                     # the inner ring for the bolts
    _add_teeth(c, tooth, sb.CROWN_FRAME, sb.CROWN_FRAME, 0.0, gc['z'])
    P.cut(c, P.cyl((0, -zf + 1.0, 0), (0, -zb - 1.0, 0), R_SPIGOT))              # the bore, 170.4
    for j in range(N_BOLT_C):
        a = 2.0 * math.pi * j / N_BOLT_C
        x, z = R_BOLT_C * math.cos(a), R_BOLT_C * math.sin(a)
        P.cut(c, P.cyl((x, -zf + 1.0, z), (x, -zb - 1.0, z), 6.0))              # tapped M12, drawn at 12
    return c


def _washer(axis, half_deg, r_hole):
    """A spherical washer between the gear's back (74) and the cavity (76), within half_deg of the axis."""
    w = P.cut(P.sphere(O, R_CAVITY), P.sphere(O, R_BACK))
    t = math.tan(math.radians(half_deg))
    P.intersect(w, _frustum(_at(axis, 1.0), t, _at(axis, 90.0), 90.0 * t))
    P.cut(w, P.cyl(_at(axis, 30.0), _at(axis, 90.0), r_hole))
    return w


def _gear_blank(toe, heel, axis):
    """Below the root cone from the toe out to the spherical back of radius 74."""
    (zt, rt), (zh, rh) = toe, heel
    tg = rh / zh
    g = _frustum(_at(axis, zt), zt * tg, _at(axis, R_CAVITY), R_CAVITY * tg)
    return P.intersect(g, P.sphere(O, R_BACK))


def _differential(comp, D, brg):
    L = D['layout']
    N = D['N']
    r_pin = D['diff']['d_pin'] / 2.0                   # the cross's arms, axle_design.differential's pin
    gc, gs, gq = D['gc'], D['gs'], D['gq']
    out = dict(spiders=[], sides={})
    out['case'] = _part(comp, 'Differential case', _case(L, N, r_pin), IRON)
    occ = P.new_occ(comp, 'Crown wheel z47')
    tooth = _loft_tooth(occ.component, _sections(gc, sb.CROWN_FRAME, math.pi / gc['z'], N_SECTIONS))
    _fill(occ, _crown(L, tooth, gc), STEEL)
    yf0 = -L['z_back'] - T_FLANGE
    for j in range(N_BOLT_C):
        a = 2.0 * math.pi * j / N_BOLT_C
        p = (R_BOLT_C * math.cos(a), yf0, R_BOLT_C * math.sin(a))
        _part(comp, 'Crown wheel bolt M12x35 %d' % (j + 1), P.cap_screw(12.0, L_CBOLT, p, (0.0, 1.0, 0.0)), STEEL)
    for key in ('C1', 'C2'):
        code, cone, cup = brg[key]
        _part(comp, 'Bearing %s %s cone' % (key, code), cone, STEEL)
    cross = None
    for k in range(N):
        a = arm(N, k)[0]
        c = P.cyl(_at(a, -r_pin), _at(a, R_ARM_END), r_pin)
        cross = c if cross is None else P.union(cross, c)
    _part(comp, 'Cross', cross, STEEL)
    q_wash = math.degrees(L['q_root']) - 2.5
    s_wash = math.degrees(L['s_root']) - 3.0
    for k in range(N):
        _part(comp, 'Spider washer %d' % (k + 1), _washer(arm(N, k)[0], q_wash, r_pin + R_SPIDER_PLAY), STEEL)
    for side, frame in (('L', FRAME_L), ('R', FRAME_R)):
        _part(comp, 'Side gear washer ' + side, _washer(frame[2], s_wash, R_HUB + 0.3), STEEL)
    # the side gears: one tooth lofted for L, turned onto both
    occ_L = P.new_occ(comp, 'Side gear L')
    occ_L.isGroundToParent = False
    tooth_s = _loft_tooth(occ_L.component, _sections(gs, FRAME_L, 0.0, 2))
    for side, frame, phase in (('L', FRAME_L, 0.0), ('R', FRAME_R, math.pi / Z_S)):
        o = occ_L if side == 'L' else P.new_occ(comp, 'Side gear R')
        o.isGroundToParent = False
        ez = frame[2]
        g = _gear_blank(L['s_toe'], L['s_heel'], ez)
        P.union(g, P.cyl(_at(ez, 50.0), _at(ez, 100.0), R_HUB))
        _add_teeth(g, tooth_s, FRAME_L, frame, phase, Z_S)
        P.cut(g, P.spline_hole(_at(ez, L['s_toe'][0] - 1.0), _at(ez, 101.0), *SPL_S))
        _fill(o, g, STEEL)
        out['sides'][side] = o
    # the spiders: one tooth lofted for spider 1, turned onto all
    F1 = spider_frame(N, 0)
    tooth_q = None
    for k in range(N):
        o = P.new_occ(comp, 'Spider %d' % (k + 1))
        o.isGroundToParent = False
        if tooth_q is None:
            tooth_q = _loft_tooth(o.component, _sections(gq, F1, 0.0, 2))
        Fk = spider_frame(N, k)
        g = _gear_blank(L['q_toe'], L['q_heel'], Fk[2])
        _add_teeth(g, tooth_q, F1, Fk, spider_phase(N, k), Z_Q)
        P.cut(g, P.cyl(_at(Fk[2], L['q_toe'][0] - 1.0), _at(Fk[2], R_CAVITY + 4.0), r_pin + R_SPIDER_PLAY))
        _fill(o, g, STEEL)
        out['spiders'].append((o, arm(N, k)[0]))
    return out


# =============================================================================
# Joints
# =============================================================================
def _joint(root, name, moving, fixed, direction):
    """An as-built revolute joint on a sketch point at the moving part's own origin, the moving part first."""
    comp = moving.component
    sk = comp.sketches.add(comp.xYConstructionPlane)
    sk.name = name
    pt = sk.sketchPoints.add(P3(0.0, 0.0, 0.0))
    sk.isVisible = False
    geo = adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(moving))
    ji = root.asBuiltJoints.createInput(moving, fixed, geo)
    ji.setAsRevoluteJointMotion(direction)
    j = root.asBuiltJoints.add(ji)
    j.name = name
    return j


def _joints(root, car, pin, dif, parts):
    JD = adsk.fusion.JointDirections
    jp = _joint(root, 'Pinion turns', pin, car, JD.XAxisJointDirection)
    jc = _joint(root, 'Case turns', dif, car, JD.YAxisJointDirection)
    mli = root.motionLinks.createInput(jp, jc)
    mli.valueOne = adsk.core.ValueInput.createByReal(math.radians(float(Z_C)))   # 47 turns of the pinion
    mli.valueTwo = adsk.core.ValueInput.createByReal(math.radians(float(Z_P)))   # to 12 of the case
    mli.isReversed = False
    ml = root.motionLinks.add(mli)
    try:
        ml.name = 'Pinion to case %d:%d' % (Z_C, Z_P)
    except Exception:
        pass
    case_px = parts['case'].createForAssemblyContext(dif)
    for side in ('L', 'R'):
        _joint(root, 'Side gear %s turns in the case' % side, parts['sides'][side].createForAssemblyContext(dif),
               case_px, JD.YAxisJointDirection)
    for k, (o, a) in enumerate(parts['spiders']):
        d = JD.XAxisJointDirection if abs(a[0]) > 0.5 else JD.ZAxisJointDirection
        _joint(root, 'Spider %d turns in the case' % (k + 1), o.createForAssemblyContext(dif), case_px, d)


# =============================================================================
# The parts list and the message
# =============================================================================
_SUFFIX = re.compile(r'^(.*\S) (?:[LR]|\d+)$')


def _kind(name):
    mt = _SUFFIX.match(name)
    return mt.group(1) if mt else name


def parts_list(root):
    """{kind: count} of the parts (occurrences holding a body) under root; 'Spider 3' counts as a 'Spider'."""
    out = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count:
            k = _kind(o.component.name)
            out[k] = out.get(k, 0) + 1
    return out


def _message(D, root):
    g, u, pk, cr, x = D['g'], D['u'], D['peak'], D['cruise'], D['diff']
    pb, cb = D['pb'], D['cb']
    pl = parts_list(root)
    lines = [
        'Rear axle carrier, %s' % VERSION,
        'Final drive %d:%d, ratio %.4f. Spiral bevel, module %.2f mm at the heel, face %.1f mm (a third of the cone '
        'distance is %.1f), %.0f deg spiral, %.0f deg normal pressure angle.'
        % (Z_P, Z_C, float(Z_C) / Z_P, D['m'], D['F'], g['Ae'] / 3.0, BETA, ALPHA),
        'Crown wheel %.1f mm outside diameter: ground clearance %.1f mm (%.0f mm at least).'
        % (g['da2'], D['clearance'], CLEAR_MIN),
        'Pinion at the design torque, %.2f N m for %.0f turns: bending factor %.3f, contact %.3f.'
        % (u['T1_peak'], u['n_peak'], pk['SF1'], pk['SH1']),
        'Pinion at full power, %.2f N m at %.0f rpm for %.2f million turns: bending factor %.3f, contact %.3f.'
        % (u['T1_cruise'], u['n1_cruise'], u['n_cruise'] / 1e6, cr['SF1'], cr['SH1']),
        "Pinion bearings SKF %s (A, head) and %s (B, tail) back to back: pressure points %.1f mm apart, A's %.1f mm "
        'behind the mean point; L10 at full power %.0f and %.0f million turns.'
        % (pb['A'], pb['B'], pb['s'], pb['a'], pb['L_A'] / 1e6, pb['L_B'] / 1e6),
        "Case bearings SKF %s face to face on shoulders at y = -/+%.0f: pressure points %.1f mm apart, the crown "
        "wheel's mean point %.2f mm from C1; L10 %.0f and %.0f million turns of the case."
        % (cb['C1'], Y_SHOULDER, cb['L'], cb['c'], cb['L_C1'] / 1e6, cb['L_C2'] / 1e6),
        'Differential: %d spiders z%d on a cross, two side gears z%d, straight bevel, module %.2f mm: bending factors '
        'side gear %.3f, spider %.3f at the design torque.'
        % (D['N'], Z_Q, Z_S, D['m_d'], x['SF_side'], x['SF_spider']),
        'Parts, %d in %d kinds: %s.' % (sum(pl.values()), len(pl),
                                        ', '.join('%s %d' % (k, pl[k]) for k in sorted(pl))),
    ]
    return '\n'.join(lines)


# =============================================================================
# Building
# =============================================================================
_doc = None             # the design run() opened before its dialog (no new document inside a command's events)


def _new_design():
    global _doc
    doc = _doc if (_doc is not None and _doc.isValid) else None
    _doc = None
    if doc is None:
        doc = adsk.core.Application.get().documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(doc.products.itemByProductType('DesignProductType'))
    if des.designType != adsk.fusion.DesignTypes.ParametricDesignType:     # base features need the history
        des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    return des


def build(m=6.5, F=52.0, m_d=5.5, N=4):
    """Builds the carrier in a new design and returns the message; raises Refused (a ValueError) saying why for a
    design the hard constraints refuse."""
    D = design(m, F, m_d, N)
    if D['refusals']:
        raise Refused('; '.join(D['refusals']))
    des = _new_design()
    root = des.rootComponent
    L = D['layout']
    brg = _bearings(L)
    car = P.new_occ(root, 'Carrier')
    car.isGrounded = True
    pin = P.new_occ(root, 'Pinion assembly')
    dif = P.new_occ(root, 'Differential assembly')
    _carrier(car.component, D, brg)
    _pinion(pin.component, D, brg)
    parts = _differential(dif.component, D, brg)
    _joints(root, car, pin, dif, parts)
    try:
        adsk.core.Application.get().activeViewport.fit()
    except Exception:
        pass
    return _message(D, root)


# =============================================================================
# The dialog
# =============================================================================
CMD_ID = 'mdfcRearAxleCarrierPromptTest'
_handlers = []
_keep_alive = False


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = dict(m=round(ins.itemById('m').value * 10.0, 9), F=round(ins.itemById('F').value * 10.0, 9),
                        m_d=round(ins.itemById('m_d').value * 10.0, 9), N=int(ins.itemById('N').value))
            ui.messageBox(build(**vals))
        except Refused as e:
            ui.messageBox('RearAxleCarrier refused the design: %s.\n%s' % (e, VERSION))
        except Exception:
            ui.messageBox('RearAxleCarrier failed (%s):\n%s' % (VERSION, traceback.format_exc()))


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        global _keep_alive
        if _keep_alive:
            _keep_alive = False
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        ins = cmd.commandInputs
        ins.addValueInput('m', 'Final-drive module at the heel m', 'mm', adsk.core.ValueInput.createByString('6.5 mm'))
        ins.addValueInput('F', 'Final-drive face width F', 'mm', adsk.core.ValueInput.createByString('52 mm'))
        ins.addValueInput('m_d', 'Differential module m_d', 'mm', adsk.core.ValueInput.createByString('5.5 mm'))
        ins.addIntegerSpinnerCommandInput('N', 'Spiders N', 1, 8, 1, 4)
        ex = _Execute()
        cmd.execute.add(ex)
        _handlers.append(ex)
        de = _Destroy()
        cmd.destroy.add(de)
        _handlers.append(de)


def run(context):
    global _doc, _keep_alive
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        _doc = app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Rear axle carrier',
                                                       'The carrier of the rear axle, Chapter 29')
        h = _Created()
        cd.commandCreated.add(h)
        _handlers.append(h)
        cd.execute()
        if context is not None:                 # run by Fusion: keep the script alive for the dialog's events
            adsk.autoTerminate(False)
            _keep_alive = True
    except Exception:
        ui.messageBox('RearAxleCarrier could not start (%s):\n%s' % (VERSION, traceback.format_exc()))
