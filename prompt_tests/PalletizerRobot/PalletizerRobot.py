# =============================================================================
# PalletizerRobot.py  -  Chapter 43 of "Mechanical Design in Autodesk Fusion with Claude": the whole palletizer.
# PalletizerDrives.py (Chapter 42) is loaded and its functions build the base, the slewing ring, the turret, the
# drives, the balancer and the crank, J1 now on a Nabtesco RV-700N; this script adds the links: the lower arm's box up
# to the elbow, the upper arm with its yoke, its front fork and its rear lever on a torque tube, the parallel rod, the
# levelling loop's two rods and the elbow's triangle, and the wrist housing with J4's RV-80N, its motor and the tool
# flange, every pin on its SKF bearing. Written from projects/palletizer/PalletizerRobot/PROMPT.txt (a prompt test).
#
#   INPUTS        the dialog: the pose q1, q2, q3 (deg), by default the cycle's low point
#   DERIVED       palletizer_design (the layout, the working range, the kinematics), palletizer_drives (GAS, the
#                 block), palletizer_links (Y, STANDOFFS, SECTIONS, the bearings, Chapter 43's reducers)
#   HARD RULES    build() and set_pose() refuse a pose outside the working range (the joints' limits and psi)
#   CONVENTIONS   J1 along Z, the arm in XZ facing +X, J2 and J3 about A's line along Y; every group built at q = 0
#                 and placed by set_pose(); no joints: the parallelograms close by their geometry
#   ACCEPTANCE    the message; every pin's parts on one axis; the flange where the kinematics put it; no
#                 interference; the parallel rod clear of J3's reducer; the refusal; the groups weighed (the check)
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
import companion_paths as CP        # noqa: E402  every lib/ folder on sys.path; CP.script(name) finds a script
import palletizer_design as pd      # noqa: E402
import palletizer_drives as pdr     # noqa: E402
import palletizer_links as pl       # noqa: E402
import mdfc_parts as mp             # noqa: E402
for _m in (pd, pdr, pl, mp):
    importlib.reload(_m)


def _load(name):
    """A script of the companion loaded as a module: its functions are called, nothing of it is copied."""
    spec = importlib.util.spec_from_file_location('mdfc_' + name, CP.script(name))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


PD = _load('PalletizerDrives')      # Chapter 42: the base, the ring, the turret, the drives, the balancer, the crank

VERSION = 'PalletizerRobot 1.0, prompt test (2026-11-11)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

# ---------- derived: the layout, the layers, the sections, the bearings (never typed) ----------
LAY = pd.LAYOUT
AX, AZ = LAY['A']                   # the shoulder A (mm)
HOME = pd.pts(0.0, 0.0)             # every pivot at q = 0, where each group is built: (x, z) mm
Y = pl.Y                            # each part's layer across the arm's plane (mm)
STANDOFFS = pl.STANDOFFS            # the counterweight's standoffs on the crank: (along it from A, across it)
SEC = pl.SECTIONS
LOWER = tuple(SEC['lower'][k] * 1000.0 for k in ('d', 'w', 't'))       # 220 x 300 x 12, d in the arm's plane
UPPER = tuple(SEC['upper'][k] * 1000.0 for k in ('d', 'w', 't'))       # 200 x 260 x 10
TUBE_B = (SEC['tube_B']['Do'] * 1000.0, SEC['tube_B']['t'] * 1000.0)   # the torque tube 300 x 12
LEVER = (SEC['lever']['d'] * 1000.0, SEC['lever']['w'] * 1000.0)       # the rear lever 300 deep, 60 across
ROD_P = (SEC['rod_p']['d'] * 1000.0, SEC['rod_p']['w'] * 1000.0)       # the parallel rod 40 deep, 70 across
ROD_L = (SEC['rod_l']['Do'] * 1000.0, SEC['rod_l']['t'] * 1000.0)      # the levelling rods 60 x 5
WEB = (Y['crank'] - LEVER[1] / 2, Y['crank'] + LEVER[1] / 2)           # 610 to 670: the crank's web, the rear lever
L_ROD_P = (Y['rod_p'] - ROD_P[1] / 2, Y['rod_p'] + ROD_P[1] / 2)       # 675 to 745, on the web's outer face
L_ROD_L = (Y['rod_l'] - ROD_L[0] / 2, Y['rod_l'] + ROD_L[0] / 2)       # -310 to -250
L_TRI = (-245.0, -215.0)            # the triangle and the wrist's plate up to H (the prompt)
TUBE_Y0 = 200.0                     # the torque tube round the elbow's axis from y = 200 to the lever
CW_M, CW_R = pl.CW['m'], pl.CW['r'] * 1000.0                           # Chapter 42's counterweight, 600 behind A
CW_A, CW_H = (v * 1000.0 for v in pdr.CW_BLOCK)                        # the block 460 x 460
CW_T = pdr.cw_thickness(CW_M) * 1000.0                                 # 60.2 thick for 100 kg of steel
L_BLOCK = (Y['block'] - CW_T / 2, Y['block'] + CW_T / 2)              # about 750 to 810, on its standoffs
TRB = dict(pl.TRB['32213'], B=31.0, C=27.0)    # SKF 32213: d, D, T from the library; B and C its product page
SRB = pl.SRB                        # SKF 22207 E and 22213 E
DGBB = pl.DGBB                      # SKF 61856 MA and 61836
CHOICE = pl.CHOICE43                # J1 an RV-700N (203.52), J2 and J3 RV-500N (192.75), J4 an RV-80N (101)

# ---------- PalletizerDrives.py's parts that this chapter's parts fit (read from its functions) ----------
CHEEK3 = (520.0, 600.0)             # the J3 cheek across the plane, its hole r 120 round A
HOLE3 = 120.0
CHEEK2_IN = -520.0                  # the J2 cheek's inner face
WEB_END, WEB_HALF = 830.0, 150.0    # the crank's web: 830 behind A, 150 either side of the crank's line
ROOT_BOX_END = AZ + 1100.0          # the lower arm's root box ends 1100 above A

# ---------- this script's choices (sizes the prompt leaves open) ----------
PIN = TRB['d']                      # 65: the pins at B and W (32213), C and D (22213 E, the same bore)
PIN_S = SRB['22207 E']['d']         # 35: the pins at E, F, G, H and the triangle's journal at B
CHEEK_B = (160.0, 200.0)            # the yoke's cheeks at |y| 160 to 200 round the 32213 at 180, the lower arm inside
CHEEK_W = (130.0, 170.0)            # the front fork's cheeks at |y| 130 to 170 round the 32213 at 150
R_CHEEK_B, R_CHEEK_W = 150.0, 100.0  # their bosses round B (the torque tube's size) and W
S_YOKE, T_YOKE = 340.0, 40.0        # the yoke's base 340 ahead of B: the lower arm's box clears it down to psi 140
S_FORK, T_FORK = 140.0, 40.0        # the front fork's base 140 behind W, clear of the wrist's hub
R_EYE_P = 80.0                      # the parallel rod's eyes round the 22213 E (120 outside)
R_EYE_L = 50.0                      # the levelling rods' eyes and the triangle's bosses round the 22207 E (72 outside)
EYE_IN = 40.0                       # the levelling tubes end 40 from their pins, inside the eyes
R_TRI_B, W_TRI = 60.0, 80.0         # the triangle's boss at B and its webs
R_STANDOFF = 25.0                   # the counterweight's standoffs, 50 across
PIN_B_END = L_ROD_L[1] + 2.0        # the elbow's pin stops 2 mm short of the levelling rods' layer
STUB_OUT = 2.0                      # the stub pins stand 2 mm out of their eyes
R_WHUB, Y_WHUB = 90.0, CHEEK_W[0] - 5.0   # the wrist's hub round W between the front fork's cheeks
X_PLATE = 230.0                     # the plate up to H leaves the housing 230 ahead of the wrist pin (the prompt)
RV700 = dict(R=190.0, R_fl=215.0, ri=150.0, L=170.0, out=140.0, out_i=100.0, out_fl=170.0)   # J1, as drawn
RV80 = dict(R=95.0, ri=65.0, L=100.0, R_fl=120.0, out=80.0, hub=60.0)                         # J4, as drawn
R_FL, T_FL = 125.0, 25.0            # the tool flange, 250 across, its face where the kinematics put it
R_TOWER = 120.0                     # the housing's tower round J4's axis
MOTOR4 = (65.0, 200.0)              # J4's servo motor, 130 across, 200 long

GROUPS = ('Turret', 'Lower arm', 'Crank', 'Parallel rod', 'Levelling rod 1', 'Triangle', 'Upper arm',
          'Levelling rod 2', 'Wrist')


# ---------- refusing a pose ----------
def refuse(q1, q2, q3, tol=1e-6):
    """A pose outside the working range: the joints' limits and psi = 90 - q2 - q3 between its limits."""
    L = pd.LIMITS
    q1max = pd.brief()['q1max']
    bad = []
    if abs(q1) > q1max + tol:
        bad.append('q1 beyond +-%g deg' % q1max)
    for name, v in (('q2', q2), ('q3', q3)):
        if not L[name][0] - tol <= v <= L[name][1] + tol:
            bad.append('%s outside %g to %g deg' % (name, L[name][0], L[name][1]))
    psi = pd.psi(q2, q3)
    if not L['psi'][0] - tol <= psi <= L['psi'][1] + tol:
        bad.append('psi = 90 - q2 - q3 = %g deg outside %g to %g deg' % (psi, L['psi'][0], L['psi'][1]))
    if bad:
        raise ValueError('The pose q1 = %g, q2 = %g, q3 = %g deg is outside the working range: %s.'
                         % (q1, q2, q3, '; '.join(bad)))


# ---------- bodies (world mm, every group at q = 0) ----------
def obox(p0, p1, w, y0, y1):
    """A bar in the arm's plane from p0 to p1 ((x, z) mm), w wide in the plane, from y0 to y1 across it."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dz)
    c = P3((p0[0] + p1[0]) / 2 * CM, (y0 + y1) / 2 * CM, (p0[1] + p1[1]) / 2 * CM)
    return mp.tbm().createBox(adsk.core.OrientedBoundingBox3D.create(c, V3(dx / L, 0, dz / L), V3(0, 1, 0), L * CM,
                                                                     (y1 - y0) * CM, w * CM))


_count = {}


def nm(kind):
    """kind 1, kind 2, ...: the parts list counts them together."""
    _count[kind] = _count.get(kind, 0) + 1
    return '%s %d' % (kind, _count[kind])


_PD_PART, _PD_RV = PD.part, PD.rv


def part(parent, name, tb, material='Steel'):
    """PalletizerDrives.py's part(): a component of its own in the group, the body in it, grounded to the group."""
    return _PD_PART(parent, name, tb, material)


DROP = ('J2 support bearing', 'J3 support bearing')           # drawn here as SKF 61856 MA and 61836
RENAME = {'RV-500N case J1': 'RV-700N case J1', 'RV-500N output J1': 'RV-700N output J1'}


def _drives_part(parent, name, tb, material='Steel'):
    """part() while PalletizerDrives.py builds: its two plain support rings left out, J1's reducer renamed."""
    if name in DROP:
        return None
    return _PD_PART(parent, RENAME.get(name, name), tb, material)


def _rv700(parent, name, axis, start, sign):
    """J1's RV-700N in place of PalletizerDrives.rv()'s RV-500N, in the same stack: the case on the plate at
    z = 300, the output flange's face at 470 under the disc coupling; 380 across (assumed)."""
    if axis != 'z':
        return _PD_RV(parent, name, axis, start, sign)
    g = RV700

    def seg(s0, s1, ro, ri):
        a, b = start + sign * s0, start + sign * s1
        return PD.ztube(min(a, b), max(a, b), ro, ri)
    case = seg(0.0, g['L'] - 10.0, g['R'], g['ri'])
    mp.union(case, seg(0.0, 20.0, g['R_fl'], g['ri']))
    rotor = seg(20.0, g['L'] - 10.0 - 0.01, g['out'], g['out_i'])
    mp.union(rotor, seg(g['L'] - 10.0, g['L'], g['out_fl'], 40.0))
    return case, rotor


class _In:
    """Stands for the turret occurrence PalletizerDrives.py's functions build inside: given the root component,
    they make the lower arm, the crank and the balancer's parts top-level groups."""
    def __init__(self, comp):
        self.component = comp


# ---------- bearings ----------
def dgbb(name, yc, at, inner, outer, rolling):
    """An SKF deep groove ball bearing on a y axis through at = (x, z), its middle at yc."""
    b = DGBB[name]
    i_, o_, balls, _ = mp.bearing_bodies(b['d'], b['D'], b['B'], yc - b['B'] / 2.0, at[0], at[1])
    part(inner, nm(name + ' inner ring'), i_)
    part(outer, nm(name + ' outer ring'), o_)
    part(rolling, nm(name + ' balls'), balls)


def srb(name, yc, at, inner, outer, rolling):
    """An SKF spherical roller bearing as drawn: the rings and two rows of rollers between them, on a y axis."""
    b = SRB[name]
    d, D, B = b['d'], b['D'], b['B']
    x, z = at
    dm, Dw = (d + D) / 2.0, 0.25 * (D - d)
    part(inner, nm(name + ' inner ring'), PD.ytube(x, z, yc - B / 2, yc + B / 2, (dm - Dw) / 2 - 0.05, d / 2))
    part(outer, nm(name + ' outer ring'), PD.ytube(x, z, yc - B / 2, yc + B / 2, D / 2, (dm + Dw) / 2 + 0.05))
    n = int(math.floor(math.pi * dm / (1.25 * Dw)))
    Lr = B / 2 - 2.0
    rollers = None
    for row, s in enumerate((-1.0, 1.0)):
        yr = yc + s * B / 4
        for k in range(n):
            a = 2 * math.pi * (k + 0.5 * row) / n
            px, pz = x + dm / 2 * math.cos(a), z + dm / 2 * math.sin(a)
            c = mp.cyl((px, yr - Lr / 2, pz), (px, yr + Lr / 2, pz), Dw / 2)
            rollers = c if rollers is None else mp.union(rollers, c)
    part(rolling, nm(name + ' rollers'), rollers)


def trb_pair(ycs, at, inner, outer):
    """An SKF 32213 pair: each cone (with its rollers) on the pin, its back face outboard; each cup in a cheek."""
    g = TRB
    for yc in ycs:
        s = 1.0 if yc > 0 else -1.0
        cone, cup, _ = mp.taper_bodies(g['d'], g['D'], g['T'], g['B'], g['C'], (at[0], yc + s * g['T'] / 2, at[1]),
                                       (0.0, -s, 0.0))
        part(inner, nm('32213 cone'), cone)
        part(outer, nm('32213 cup'), cup)


# ---------- the groups ----------
def shoulder(G):
    """At A: SKF 61856 MA carries the lower arm's hub on the J3 side, in a housing on the J3 cheek's inner face; SKF
    61836 carries the crank's shaft in the J3 cheek, in a sleeve. Both replace Chapter 42's plain rings."""
    tur, la, cr = G['Turret'], G['Lower arm'], G['Crank']
    A = HOME['A']
    b = DGBB['61856 MA']
    part(tur, 'J2 support housing', PD.ytube(AX, AZ, Y['sup2'] - 40.0, CHEEK3[0], b['D'] / 2 + 10.0, b['D'] / 2))
    dgbb('61856 MA', Y['sup2'], A, la, tur, la)
    b = DGBB['61836']
    part(tur, 'J3 support sleeve', PD.ytube(AX, AZ, CHEEK3[0], CHEEK3[1], HOLE3, b['D'] / 2))
    dgbb('61836', Y['sup3'], A, cr, tur, cr)


def lower_arm(G):
    """The box from the root box up to the elbow, closed by a hub round the elbow's pin; the pin, stepped down for
    the triangle's 22207 E; the cones of the elbow's 32213 pair; the triangle's bearing."""
    la, up, tri = G['Lower arm'], G['Upper arm'], G['Triangle']
    Bx, Bz = HOME['B']
    d, w, t = LOWER
    bx = mp.box(Bx - d / 2, Bx + d / 2, -w / 2, w / 2, ROOT_BOX_END, Bz)
    mp.cut(bx, mp.box(Bx - d / 2 + t, Bx + d / 2 - t, -w / 2 + t, w / 2 - t, ROOT_BOX_END - 1.0, Bz + 1.0))
    mp.union(bx, PD.ycyl(Bx, Bz, -w / 2, w / 2, d / 2))
    mp.cut(bx, PD.ycyl(Bx, Bz, -w / 2 - 1.0, w / 2 + 1.0, PIN / 2))
    part(la, 'Lower arm box', bx)
    part(la, 'Elbow pin', mp.stepped_shaft([(PIN_B_END, -CHEEK_B[1], PIN_S), (-CHEEK_B[1], CHEEK_B[1], PIN)], Bx, Bz))
    trb_pair(Y['B'], HOME['B'], la, up)
    srb('22207 E', sum(L_TRI) / 2, HOME['B'], la, tri, tri)


def crank(G):
    """On Chapter 42's crank (built without its block): a pad under the web where two standoffs stand outside its
    150, the four standoffs where the parallel rod never swings, the block out at its layer, the pin C."""
    cr = G['Crank']

    def at(s, n):                   # the crank's frame at q3 = 0: s behind A along it, n across it (+ the rod's side)
        return AX - s, AZ + n
    part(cr, 'Crank web pad', mp.box(AX - WEB_END, AX - 500.0, WEB[0], WEB[1], AZ - 210.0, AZ - WEB_HALF))
    for k, (s, n) in enumerate(STANDOFFS):
        x, z = at(s, n)
        part(cr, 'Counterweight standoff %d' % (k + 1), PD.ycyl(x, z, WEB[1], L_BLOCK[0], R_STANDOFF))
    xc, zc = at(CW_R, 0.0)
    part(cr, 'Counterweight block', mp.box(xc - CW_A / 2, xc + CW_A / 2, L_BLOCK[0], L_BLOCK[1], zc - CW_H / 2,
                                           zc + CW_H / 2))
    C = HOME['C']
    part(cr, 'Pin C', PD.ycyl(C[0], C[1], WEB[1], L_ROD_P[1] + STUB_OUT, PIN / 2))


def parallel_rod(G):
    """The flat bar 40 x 70 from C to D on the web's and the lever's outer faces, an eye for a 22213 E at each end."""
    rod, cr, up = G['Parallel rod'], G['Crank'], G['Upper arm']
    C, D = HOME['C'], HOME['D']
    y0, y1 = L_ROD_P
    body = mp.box(C[0] - ROD_P[0] / 2, C[0] + ROD_P[0] / 2, y0, y1, C[1], D[1])
    for p in (C, D):
        mp.union(body, PD.ycyl(p[0], p[1], y0, y1, R_EYE_P))
    for p in (C, D):
        mp.cut(body, PD.ycyl(p[0], p[1], y0 - 1.0, y1 + 1.0, SRB['22213 E']['D'] / 2))
    part(rod, 'Parallel rod body', body)
    srb('22213 E', Y['rod_p'], C, cr, rod, rod)
    srb('22213 E', Y['rod_p'], D, up, rod, rod)


def upper_arm(G):
    """The yoke's two cheeks round the lower arm's hub and its base; the box 200 x 260 x 10; the front fork's base
    and cheeks; the cups of both 32213 pairs; the torque tube round the elbow's axis and the rear lever on it; the
    pin D."""
    up = G['Upper arm']
    (Bx, Bz), (Wx, Wz), (Dx, Dz) = HOME['B'], HOME['W'], HOME['D']
    d, w, t = UPPER
    for k, s in enumerate((-1.0, 1.0)):
        y0, y1 = sorted((s * CHEEK_B[0], s * CHEEK_B[1]))
        ch = PD.ycyl(Bx, Bz, y0, y1, R_CHEEK_B)
        mp.union(ch, mp.box(Bx, Bx + S_YOKE, y0, y1, Bz - d / 2, Bz + d / 2))
        mp.cut(ch, PD.ycyl(Bx, Bz, y0 - 1.0, y1 + 1.0, TRB['D'] / 2))
        part(up, 'Yoke cheek %d' % (k + 1), ch)
    part(up, 'Yoke base', mp.box(Bx + S_YOKE, Bx + S_YOKE + T_YOKE, -CHEEK_B[1], CHEEK_B[1], Bz - d / 2, Bz + d / 2))
    x0, x1 = Bx + S_YOKE + T_YOKE, Wx - S_FORK - T_FORK
    bx = mp.box(x0, x1, -w / 2, w / 2, Bz - d / 2, Bz + d / 2)
    mp.cut(bx, mp.box(x0 - 1.0, x1 + 1.0, -w / 2 + t, w / 2 - t, Bz - d / 2 + t, Bz + d / 2 - t))
    part(up, 'Upper arm box', bx)
    part(up, 'Front fork base', mp.box(x1, Wx - S_FORK, -CHEEK_W[1], CHEEK_W[1], Wz - d / 2, Wz + d / 2))
    for k, s in enumerate((-1.0, 1.0)):
        y0, y1 = sorted((s * CHEEK_W[0], s * CHEEK_W[1]))
        ch = PD.ycyl(Wx, Wz, y0, y1, R_CHEEK_W)
        mp.union(ch, mp.box(Wx - S_FORK, Wx, y0, y1, Wz - d / 2, Wz + d / 2))
        mp.cut(ch, PD.ycyl(Wx, Wz, y0 - 1.0, y1 + 1.0, TRB['D'] / 2))
        part(up, 'Front fork cheek %d' % (k + 1), ch)
    Do, tt = TUBE_B
    part(up, 'Torque tube', PD.ytube(Bx, Bz, TUBE_Y0, WEB[0], Do / 2, Do / 2 - tt))
    lv = PD.ycyl(Bx, Bz, WEB[0], WEB[1], LEVER[0] / 2)
    mp.union(lv, mp.box(Dx, Bx, WEB[0], WEB[1], Bz - LEVER[0] / 2, Bz + LEVER[0] / 2))
    mp.union(lv, PD.ycyl(Dx, Dz, WEB[0], WEB[1], LEVER[0] / 2))
    part(up, 'Rear lever', lv)
    part(up, 'Pin D', PD.ycyl(Dx, Dz, WEB[1], L_ROD_P[1] + STUB_OUT, PIN / 2))


def levelling(G):
    """The loop on the J2 side: the pivot E on a bracket from the J2 cheek; the triangle at the elbow with its pins F
    and G; the two rods, tubes 60 x 5 with an eye for a 22207 E at each end."""
    tur, tri, wr = G['Turret'], G['Triangle'], G['Wrist']
    r1, r2 = G['Levelling rod 1'], G['Levelling rod 2']
    E, F, Gp, H, B = HOME['E'], HOME['F'], HOME['G'], HOME['H'], HOME['B']
    y0, y1 = L_ROD_L
    br = PD.ycyl(E[0], E[1], CHEEK2_IN, y0 - 5.0, R_EYE_L + 5.0)
    mp.union(br, mp.box(E[0] - 55.0, E[0] + 55.0, CHEEK2_IN, y0 - 5.0, E[1] - 90.0, E[1]))
    part(tur, 'Levelling pivot bracket', br)
    part(tur, 'Pin E', PD.ycyl(E[0], E[1], y0 - 5.0, y1 + STUB_OUT, PIN_S / 2))
    t0, t1 = L_TRI
    plate = PD.ycyl(B[0], B[1], t0, t1, R_TRI_B)
    for p in (F, Gp):
        mp.union(plate, PD.ycyl(p[0], p[1], t0, t1, R_EYE_L))
    for a, b in ((B, F), (B, Gp), (F, Gp)):
        mp.union(plate, obox(a, b, W_TRI, t0, t1))
    mp.cut(plate, PD.ycyl(B[0], B[1], t0 - 1.0, t1 + 1.0, SRB['22207 E']['D'] / 2))
    part(tri, 'Triangle plate', plate)
    for k, p in (('F', F), ('G', Gp)):
        part(tri, 'Pin ' + k, PD.ycyl(p[0], p[1], y0 - STUB_OUT, t0, PIN_S / 2))
    for occ, (p, q), (ip, iq), label in ((r1, (E, F), (tur, tri), 'Levelling rod 1 body'),
                                         (r2, (Gp, H), (tri, wr), 'Levelling rod 2 body')):
        Do, t = ROD_L
        body = PD.ytube(p[0], p[1], y0, y1, R_EYE_L, SRB['22207 E']['D'] / 2)
        mp.union(body, PD.ytube(q[0], q[1], y0, y1, R_EYE_L, SRB['22207 E']['D'] / 2))
        L = math.hypot(q[0] - p[0], q[1] - p[1])
        e = ((q[0] - p[0]) / L, (q[1] - p[1]) / L)
        a = (p[0] + e[0] * EYE_IN, Y['rod_l'], p[1] + e[1] * EYE_IN)
        b = (q[0] - e[0] * EYE_IN, Y['rod_l'], q[1] - e[1] * EYE_IN)
        mp.union(body, mp.tube(a, b, Do / 2, Do / 2 - t))
        part(occ, label, body)
        srb('22207 E', Y['rod_l'], p, ip, occ, occ)
        srb('22207 E', Y['rod_l'], q, iq, occ, occ)


def wrist(G):
    """The wrist housing on its pin between the front fork's cheeks: the hub, a web forward to the tower round J4's
    axis, a bracket out to the plate that goes up to H (leaving the housing 230 ahead of the pin, outside the
    cheeks' circle); J4's RV-80N in the tower, its motor on top, the tool flange under it."""
    wr, up = G['Wrist'], G['Upper arm']
    Wx, Wz = HOME['W']
    Jx = HOME['J4'][0]
    zf = HOME['fl'][1]                                  # the flange's face, 380 below the wrist pin
    part(wr, 'Wrist pin', PD.ycyl(Wx, Wz, -CHEEK_W[1], CHEEK_W[1], PIN / 2))
    trb_pair(Y['W'], HOME['W'], wr, up)
    z_out = zf + T_FL                                   # the reducer's output flange on the tool flange
    z_case = z_out + 10.0                               # its case from here, its flange under the tower
    tz0, tz1 = z_case + 10.0, Wz                        # the tower
    h = PD.ycyl(Wx, Wz, -Y_WHUB, Y_WHUB, R_WHUB)
    mp.union(h, mp.box(Wx + 60.0, Wx + 180.0, -100.0, 100.0, Wz - 150.0, Wz - 10.0))
    mp.union(h, PD.zcyl(tz0, tz1, R_TOWER, Jx, 0.0))
    mp.union(h, mp.box(Wx + X_PLATE, Wx + X_PLATE + 100.0, L_TRI[1], -100.0, Wz - 120.0, Wz - 60.0))
    mp.cut(h, PD.ycyl(Wx, Wz, -Y_WHUB - 5.0, Y_WHUB + 5.0, PIN / 2))
    mp.cut(h, PD.zcyl(tz0 - 1.0, tz1 + 1.0, RV80['R'], Jx, 0.0))
    part(wr, 'Wrist housing', h)
    y0, y1 = L_TRI
    Hx, Hz = HOME['H']
    pt = mp.box(Wx + X_PLATE, Wx + X_PLATE + 100.0, y0, y1, Wz - 120.0, Wz - 60.0)
    mp.union(pt, obox((Wx + X_PLATE + 50.0, Wz - 90.0), (Hx, Hz), 80.0, y0, y1))
    mp.union(pt, PD.ycyl(Hx, Hz, y0, y1, R_EYE_L))
    part(wr, 'Wrist plate', pt)
    part(wr, 'Pin H', PD.ycyl(Hx, Hz, L_ROD_L[0] - STUB_OUT, y0, PIN_S / 2))
    part(wr, 'Tool flange', PD.zcyl(zf, z_out, R_FL, Jx, 0.0))
    out = PD.zcyl(z_out, z_case, RV80['out'], Jx, 0.0)
    mp.union(out, PD.zcyl(z_case, z_case + RV80['L'] - 10.0, RV80['hub'], Jx, 0.0))
    part(wr, 'RV-80N output J4', out)
    cs = PD.ztube(z_case, z_case + RV80['L'], RV80['R'], RV80['ri'], Jx, 0.0)
    mp.union(cs, PD.ztube(z_case, tz0, RV80['R_fl'], RV80['ri'], Jx, 0.0))
    part(wr, 'RV-80N case J4', cs)
    part(wr, 'J4 motor adaptor', PD.ztube(tz1, tz1 + 20.0, R_TOWER, 30.0, Jx, 0.0))
    part(wr, 'Servo motor J4', PD.zcyl(tz1 + 20.0, tz1 + 20.0 + MOTOR4[1], MOTOR4[0], Jx, 0.0), 'Aluminum')


def build_groups(root):
    """Every group at q = 0: PalletizerDrives.py's base, turret (J1 on the RV-700N), lower arm's root, crank and
    balancer, then this chapter's parts."""
    _count.clear()
    PD.part = _drives_part
    try:
        PD.rv = _rv700
        try:
            base, j1_rotor = PD.build_base(root)
        finally:
            PD.rv = _PD_RV
        tur, rotor2, rotor3 = PD.build_turret(root, j1_rotor)
        la = PD.build_lower_arm(_In(root), rotor2)
        cr = PD.build_crank(_In(root), rotor3, 0.0, CW_R)       # its block is added below, on standoffs
        cyls = PD.build_balancer(_In(root))
    finally:
        PD.part = _PD_PART
    G = {'Base': base, 'Turret': tur, 'Lower arm': la, 'Crank': cr}
    for name in ('Upper arm', 'Parallel rod', 'Levelling rod 1', 'Levelling rod 2', 'Triangle', 'Wrist'):
        G[name] = mp.new_occ(root, name)
    G['balancer'] = cyls
    shoulder(G)
    lower_arm(G)
    crank(G)
    upper_arm(G)
    parallel_rod(G)
    levelling(G)
    wrist(G)
    return G


# ---------- the pose ----------
def _turn_y(deg, about):
    """A turn about the line along Y through about = (x, z) mm (right-handed: +q2 tilts the lower arm forward)."""
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.radians(deg), V3(0, 1, 0), P3(about[0] * CM, 0, about[1] * CM))
    return m


def _then_move(m, frm, to):
    """m followed by the translation carrying the point frm to the point to ((x, z) mm)."""
    t = adsk.core.Matrix3D.create()
    t.translation = V3((to[0] - frm[0]) * CM, 0, (to[1] - frm[1]) * CM)
    m.transformBy(t)
    return m


def set_pose(G, q1, q2, q3):
    """Every group placed for the pose (deg): the lower arm turned by q2 about A; the crank by q3 about A and the
    upper arm by q3 about the elbow; the parallel rod by q2 about its pin C; the levelling rods by q2 about E and by
    q3 about G; the triangle and the wrist moved with the elbow and the wrist pin; the balancer's barrels and rods
    as PalletizerDrives.py places them; then everything but the base turned by q1 about Z."""
    refuse(q1, q2, q3)
    P, H0 = pd.pts(q2, q3), HOME
    tur = adsk.core.Matrix3D.create()
    tur.setToRotation(math.radians(q1), V3(0, 0, 1), P3(0, 0, 0))
    place = {
        'Turret': adsk.core.Matrix3D.create(),
        'Lower arm': _turn_y(q2, H0['A']),
        'Crank': _turn_y(-q3, H0['A']),
        'Upper arm': _then_move(_turn_y(-q3, H0['B']), H0['B'], P['B']),
        'Parallel rod': _then_move(_turn_y(q2, H0['C']), H0['C'], P['C']),
        'Levelling rod 1': _turn_y(q2, H0['E']),
        'Levelling rod 2': _then_move(_turn_y(-q3, H0['G']), H0['G'], P['G']),
        'Triangle': _then_move(adsk.core.Matrix3D.create(), H0['B'], P['B']),
        'Wrist': _then_move(adsk.core.Matrix3D.create(), H0['W'], P['W']),
    }
    for name, m in place.items():
        m.transformBy(tur)
        G[name].transform2 = m
    PD.set_balancer(G['balancer'], q2, G['Turret'])
    return P


parts_list = PD.parts_list


# ---------- the message ----------
def weights(G):
    """Each moving group's mass (kg) as Fusion measures it."""
    out = []
    for name in GROUPS:
        kids = [o for o in G[name].childOccurrences if o.component.bRepBodies.count]
        out.append((name.lower(), sum(o.physicalProperties.mass for o in kids)))
    out.append(('balancer', sum(o.physicalProperties.mass for b, r in G['balancer'] for o in (b, r))))
    return out


def message(G, root, q1, q2, q3):
    P = pd.pts(q2, q3)
    c1, s1 = math.cos(math.radians(q1)), math.sin(math.radians(q1))
    fl = (c1 * P['fl'][0], s1 * P['fl'][0], P['fl'][1])
    Fq = [pdr.gas_force(q) for q in range(int(pd.LIMITS['q2'][0]), int(pd.LIMITS['q2'][1]) + 1)]
    gd = dict(Fmin=min(Fq), Fmax=max(Fq))       # gas_design()'s range of the pair's push, without its cycle run
    try:
        wt = '; '.join('%s %.1f' % (n, m) for n, m in weights(G))
    except Exception:
        wt = 'not measured'
    pl_ = parts_list(root)
    secs = ('lower arm box %.0f x %.0f x %.0f; upper arm box %.0f x %.0f x %.0f; torque tube %.0f x %.0f; rear lever '
            '%.0f x %.0f; parallel rod flat bar %.0f x %.0f; levelling rods tubes %.0f x %.0f'
            % (LOWER + UPPER + TUBE_B + LEVER + ROD_P + ROD_L))
    return ('Palletizer robot built: PalletizerDrives.py\'s base, slewing ring, turret, drives, balancer and crank '
            'with the links.\n'
            'Pose: q1 %.3f, q2 %.3f, q3 %.3f deg, psi %.3f deg; the flange\'s face at (%.1f, %.1f, %.1f) mm.\n'
            'Reducers: J1 Nabtesco %s, ratio %g, on the SKF slewing ring RKS.23 0841; J2 %s, ratio %g, with the gas '
            'balancer (2 cylinders, %d mm bore, %.1f to %.1f kN); J3 %s, ratio %g, with %.0f kg on the crank %.0f mm '
            'behind A; J4 %s, ratio %g.\n'
            'Sections (mm): %s.\n'
            'Bearings (SKF): 61856 MA for the lower arm on the J3 side (y %.0f); 61836 for the crank in the J3 cheek '
            '(y %.0f); 32213 pairs in the upper arm\'s cheeks at the elbow (y %.0f, %.0f) and at the wrist (y %.0f, '
            '%.0f); 22213 E in the parallel rod\'s eyes; 22207 E in the levelling rods\' eyes and under the '
            'triangle.\n'
            'Layers (y, mm): crank web and rear lever %.0f to %.0f; parallel rod %.0f to %.0f; block %.1f to %.1f; '
            'levelling rods %.0f to %.0f; triangle and wrist plate %.0f to %.0f.\n'
            'Moving groups (kg): %s.\n\n'
            'Parts: %d components; %s.\n\n[%s]'
            % (q1, q2, q3, pd.psi(q2, q3), fl[0], fl[1], fl[2],
               CHOICE['rv1'], CHOICE['i1'], CHOICE['rv2'], CHOICE['i2'], round(pdr.GAS['bore'] * 1000),
               gd['Fmin'] / 1e3, gd['Fmax'] / 1e3, CHOICE['rv3'], CHOICE['i3'], CW_M, CW_R, CHOICE['rv4'],
               CHOICE['i4'], secs, Y['sup2'], Y['sup3'], Y['B'][0], Y['B'][1], Y['W'][0], Y['W'][1],
               WEB[0], WEB[1], L_ROD_P[0], L_ROD_P[1], L_BLOCK[0], L_BLOCK[1], L_ROD_L[0], L_ROD_L[1], L_TRI[0],
               L_TRI[1], wt, sum(pl_.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl_.items())), VERSION))


_last = {}
LO = pd.waypoints()['lo']           # the cycle's low point, the dialog's default pose


def build(q1=0.0, q2=LO['q2'], q3=LO['q3']):
    q1, q2, q3 = float(q1), float(q2), float(q3)
    refuse(q1, q2, q3)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    des.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    root = des.rootComponent
    G = build_groups(root)
    for name, col in (('Base', 'Grey'), ('Turret', 'Blue'), ('Lower arm', 'Yellow'), ('Crank', 'Grey'),
                      ('Upper arm', 'Yellow'), ('Parallel rod', 'Grey'), ('Levelling rod 1', 'Grey'),
                      ('Levelling rod 2', 'Grey'), ('Triangle', 'Blue'), ('Wrist', 'Blue')):
        mp.colour(des, G[name], 'Paint - Enamel Glossy (%s)' % col)
    set_pose(G, q1, q2, q3)
    _last.clear()
    _last.update(groups=G, pose=(q1, q2, q3), root=root)
    app.activeViewport.fit()
    return message(G, root, q1, q2, q3)


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('q1', 'J1, q1 (deg)', 0.0), ('q2', 'J2, q2 (deg)', round(LO['q2'], 3)),
          ('q3', 'J3, q3 (deg)', round(LO['q3'], 3))]


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
        cid = 'PalletizerRobotPromptTestCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Palletizer robot', 'The whole palletizer of Chapter 43')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PalletizerRobot could not start:\n' + traceback.format_exc())
