# =============================================================================
# PalletizerRobot.py  -  Chapter 43 of "Mechanical Design in Autodesk Fusion with Claude": the whole palletizer.
# Chapter 42's base, turret, drives, balancer and crank (J1 now on an RV-700N for its stiffness), and this chapter's
# links: the lower arm's box up to the elbow, the upper arm with its yoke, its front fork and its rear lever on a
# torque tube, the parallel rod on the crank's outer face, the levelling loop's two rods and the elbow's triangle,
# the wrist housing with J4's RV-80N and the tool flange; every pin on its bearing (SKF 32213 pairs at the elbow and
# the wrist, 22213 E in the parallel rod's eyes, 22207 E in the levelling rods' eyes and under the triangle).
# The sizes are lib/palletizer_links.py's. Written from PROMPT.txt beside it; checked by
# checks/palletizer_robot_check.py.
#
#   INPUTS        the dialog: the pose to show, q1 to q3 (deg)
#   DERIVED       palletizer_design (the layout), palletizer_drives (the balancer), palletizer_links (the sections,
#                 the layers across the arm's plane, the bearings)
#   HARD RULES    set_pose() refuses a pose outside the working range (the joints' limits and psi)
#   CONVENTIONS   J1 along Z, the arm in XZ facing +X; each group built at q = 0 and placed by set_pose() from the
#                 joints' angles: a parallelogram's loop closes by its geometry, which Fusion's joints do not do
#   ACCEPTANCE    the message; the check: the pins meet, no interference over the range, the flange where the
#                 kinematics put it, the groups weighed
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
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import palletizer_design as pd      # noqa: E402
import palletizer_drives as pdr     # noqa: E402
import palletizer_links as pl       # noqa: E402
import mdfc_parts as mp             # noqa: E402
importlib.reload(pd)
importlib.reload(pdr)
importlib.reload(pl)
importlib.reload(mp)

VERSION = 'PalletizerRobot 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

LAY = pd.LAYOUT
AX, AZ = LAY['A']
L2, L3, R = LAY['L2'], LAY['L3'], LAY['r']
ZR = pdr.Z_RING * 1000.0
RING = dict(D=948.0, D2=842.5, d=734.0, d2=839.5, dm=841.0, H=56.0, h=46.0, ball=20.0)   # RKS.23 0841
RV500 = (162.5, 170.0)      # the RV-500N as drawn: 325 mm across (product page), 170 mm long (assumed)
RV700 = (197.5, 200.0)      # the RV-700N: 395 mm across (product page), 200 mm long (assumed)
RV80 = (94.5, 100.0)        # the RV-80N: 189 mm across (product page), 100 mm long (assumed)
Y = pl.Y
Y_CRANK = Y['crank']
GAS = pdr.GAS
A_MM, B_MM = GAS['a'] * 1000.0, GAS['b'] * 1000.0
Y_CYL, ROD_LEN, BARREL = 60.0, 560.0, (30.0, 480.0)
S = pl.SECTIONS
# the pivots at q = 0 (mm, the arm's plane)
HOME = {k: v for k, v in pd.pts(0.0, 0.0).items()}
YR = (Y['rod_p'] - 35.0, Y['rod_p'] + 35.0)          # the parallel rod's layer, 675 to 745
YL = (Y['rod_l'] - 30.0, Y['rod_l'] + 30.0)          # the levelling rods' layer, -310 to -250
YT = (-245.0, -215.0)                                 # the triangle's and the wrist's H plate
TRB = pl.TRB['32213']
SRB = pl.SRB


def mm(v):
    return v * 1000.0


# ---------- bodies (world mm) ----------
def ycyl(x, z, y0, y1, r):
    return mp.cyl((x, y0, z), (x, y1, z), r)


def ytube(x, z, y0, y1, ro, ri):
    return mp.tube((x, y0, z), (x, y1, z), ro, ri)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def ztube(z0, z1, ro, ri, x=0.0, y=0.0):
    return mp.tube((x, y, z0), (x, y, z1), ro, ri)


def bar(p0, p1, y0, y1, h):
    """A bar in the arm's plane from p0 to p1 (x, z), h deep across its length, between y0 and y1."""
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dz)
    c = P3((p0[0] + p1[0]) / 2 * CM, (y0 + y1) / 2 * CM, (p0[1] + p1[1]) / 2 * CM)
    ob = adsk.core.OrientedBoundingBox3D.create(c, V3(dx / L, 0, dz / L), V3(0, 1, 0), L * CM, (y1 - y0) * CM, h * CM)
    return mp.tbm().createBox(ob)


def torus(c, axis, R_, r):
    t = mp.tbm().createTorus(P3(0, 0, 0), V3(*axis), R_ * CM, r * CM)
    m = adsk.core.Matrix3D.create()
    m.translation = V3(*[v * CM for v in c])
    mp.tbm().transform(t, m)
    return t


def group(root, name):
    o = mp.new_occ(root, name)
    o.isGroundToParent = False
    return o


def part(g, name, tb, material='Steel'):
    o = mp.new_occ(g.component, name)
    mp.add_body(o, tb, name, material)
    return o


def ring_y(g, name, x, z, yc, width, ro, ri, material='Steel'):
    return part(g, name, ytube(x, z, yc - width / 2, yc + width / 2, ro, ri), material)


def rv_y(x, z, start, sign, size):
    """An RV reducer along y: the case with its flange at `start`, the output toward `sign`. Returns case, output."""
    Rr, Lr = size
    seg = lambda s0, s1, ro, ri: ytube(x, z, *sorted((start + sign * s0, start + sign * s1)), ro, ri)
    case = seg(0.0, Lr - 10.0, Rr, Rr - 27.5)
    mp.union(case, seg(0.0, 20.0, Rr + 27.5, Rr - 27.5))
    out = seg(Lr - 10.0, Lr, Rr - 12.5, 40.0)
    mp.union(out, seg(20.0, Lr - 10.0 - 0.01, Rr - 37.5, Rr - 72.5))
    return case, out


# ---------- the groups at q = 0 ----------
def build_base(root):
    g = group(root, 'Base')
    g.isGrounded = True
    part(g, 'Floor plate', mp.box(-550, 550, -550, 550, 0, 40))
    part(g, 'Column', ztube(40, 470, 330, 290))
    part(g, 'Base flange', ztube(470, ZR, 500, 260))
    dia = zcyl(260, 270, 289.99)
    mp.cut(dia, zcyl(250, 280, 100))
    part(g, 'J1 reducer plate', dia)
    o = ztube(ZR, ZR + RING['h'], 445.0, RING['D2'] / 2)
    mp.union(o, ztube(ZR, ZR + 12.0, RING['D'] / 2, 445.0))
    mp.cut(o, torus((0, 0, ZR + 28.0), (0, 0, 1), RING['dm'] / 2, RING['ball'] / 2 + 0.4))
    part(g, 'Slewing ring outer ring', o)
    part(g, 'Slewing ring balls', torus((0, 0, ZR + 28.0), (0, 0, 1), RING['dm'] / 2, RING['ball'] / 2))
    # J1's RV-700N: its case on the plate, its output up to the coupling at 470
    Rr, Lr = RV700
    case = ztube(270, 270 + Lr - 10.0, Rr, Rr - 27.5)
    mp.union(case, ztube(270, 290, Rr + 27.5 if Rr + 27.5 < 289 else 288.9, Rr - 27.5))
    part(g, 'RV-700N case J1', case)
    part(g, 'J1 stop', mp.box(-565, -500, -40, 40, 470, 560))
    part(g, 'Servo motor J1', zcyl(60, 260, 95), 'Aluminum')
    out = ztube(270 + Lr - 10.0, 270 + Lr, Rr - 12.5, 40.0)
    mp.union(out, ztube(290, 270 + Lr - 10.0 - 0.01, Rr - 37.5, Rr - 72.5))
    return g, out


def build_turret(root, j1_out):
    g = group(root, 'Turret')
    i_ = ztube(ZR + 10.0, ZR + RING['H'], RING['d2'] / 2, 395.0)
    mp.union(i_, ztube(ZR + RING['H'] - 12.0, ZR + RING['H'], 395.0, RING['d'] / 2))
    mp.cut(i_, torus((0, 0, ZR + 28.0), (0, 0, 1), RING['dm'] / 2, RING['ball'] / 2 + 0.4))
    part(g, 'Slewing ring inner ring', i_)
    z0 = ZR + RING['H']
    plate = mp.box(-480, 560, -600, 600, z0, z0 + 60)
    mp.cut(plate, zcyl(z0 - 1, z0 + 61, 100))
    mp.cut(plate, mp.box(220, 420, -130, 130, z0 + 30, z0 + 61))
    part(g, 'Turret plate', plate)
    part(g, 'J1 stop dog', mp.box(515, 560, -25, 25, z0 - 48, z0))
    part(g, 'RV-700N output J1', j1_out)
    part(g, 'Coupling disc 1', zcyl(470, 485, 140))
    part(g, 'Coupling spacer', ztube(485, z0 - 15, 60, 45))
    part(g, 'Coupling disc 2', zcyl(z0 - 15, z0, 140))
    top = z0 + 60
    ch = mp.box(-200, 520, -600, -520, top, 1180)
    mp.cut(ch, ycyl(AX, AZ, -610, -510, 165))
    part(g, 'Cheek J2 side', ch)
    ch = mp.box(-200, 520, 520, 600, top, 1180)
    mp.cut(ch, ycyl(AX, AZ, 510, 610, 120))
    part(g, 'Cheek J3 side', ch)
    case, out2 = rv_y(AX, AZ, -620.0, 1, RV500)
    part(g, 'RV-500N case J2', case)
    part(g, 'Servo motor J2', ycyl(AX, AZ, -850, -620, 95), 'Aluminum')
    # the lower arm's support: an SKF 61856 MA in a housing on the J3 cheek's inner face; the crank's: a 61836
    # in the J3 cheek
    b = pl.DGBB['61856 MA']
    part(g, 'J2 support housing', ytube(AX, AZ, Y['sup2'] - b['B'] / 2 - 8.0, 520.0, b['D'] / 2 + 30.0, b['D'] / 2))
    ring_y(g, 'Bearing 61856 MA lower arm', AX, AZ, Y['sup2'], b['B'], b['D'] / 2, b['d'] / 2)
    b = pl.DGBB['61836']
    part(g, 'J3 support housing', ytube(AX, AZ, 520.0, 600.0, 120.0, b['D'] / 2))
    ring_y(g, 'Bearing 61836 crank', AX, AZ, Y['sup3'], b['B'], b['D'] / 2, b['d'] / 2)
    part(g, 'J3 bracket', mp.box(150, 640, 840, 870, 820, 1180))
    for k, zp in enumerate((850.0, 1150.0)):
        part(g, 'J3 bracket post %d' % (k + 1), mp.cyl((480, 600, zp), (480, 840, zp), 25))
    case, out3 = rv_y(AX, AZ, 840.0, -1, RV500)
    part(g, 'RV-500N case J3', case)
    part(g, 'Servo motor J3', ycyl(AX, AZ, 870, 1100, 95), 'Aluminum')
    zp = AZ - A_MM
    for k, (y0, y1) in enumerate(((-125, -100), (-15, 15), (100, 125))):
        lug = mp.box(AX - 30, AX + 30, y0, y1, z0 + 30, zp + 30)
        mp.cut(lug, ycyl(AX, zp, y0 - 1, y1 + 1, 15.5))
        part(g, 'Balancer lug %d' % (k + 1), lug)
    part(g, 'Balancer pin P', ycyl(AX, zp, -125, 125, 15))
    for k, (y0, y1) in enumerate(((-300, -20), (20, 300))):
        bt = ytube(-330, top + 55, y0, y1, 55, 50)
        mp.union(bt, ycyl(-330, top + 55, y0, y0 + 5, 50.01), ycyl(-330, top + 55, y1 - 5, y1, 50.01))
        part(g, 'Gas bottle %d' % (k + 1), bt)
    # the levelling loop's fixed pivot E: a bracket from the J2 cheek's inner face, its pin out to the rod's eye
    E = HOME['E']
    part(g, 'E bracket', mp.box(E[0] - 45, E[0] + 45, -520, YL[0] - 5.0, E[1] - 45, E[1] + 45))
    part(g, 'Pin E', ycyl(E[0], E[1], YL[0] - 5.0, YL[1] - 5.0, SRB['22207 E']['d'] / 2))
    return g, out2, out3


def build_lower_arm(root, out2):
    """q2 = 0: the fork's plates and the hub's bosses (Chapter 42), the box up to the elbow's hub, the elbow pin."""
    g = group(root, 'Lower arm')
    part(g, 'RV-500N output J2', out2)
    for sy, name in ((-1, 'Fork plate 1'), (1, 'Fork plate 2')):
        y0, y1 = sorted((sy * 150.0, sy * 190.0))
        pl_ = mp.box(AX - 110, AX + 110, y0, y1, AZ, AZ + 700)
        mp.union(pl_, ycyl(AX, AZ, y0, y1, 160))
        part(g, name, pl_)
    part(g, 'Hub boss J2 side', ytube(AX, AZ, -450, -190, 140, 60))
    part(g, 'Hub boss J3 side', ytube(AX, AZ, 190, 500, 140, 60))
    part(g, 'Balancer cross pin Q', ycyl(AX, AZ + B_MM, -150, 150, 15))
    s = S['lower']
    d, w, t = mm(s['d']), mm(s['w']), mm(s['t'])
    Bz = AZ + L2
    bx = mp.box(AX - d / 2, AX + d / 2, -w / 2, w / 2, AZ + 650, Bz - 80)
    mp.cut(bx, mp.box(AX - d / 2 + t, AX + d / 2 - t, -w / 2 + t, w / 2 - t, AZ + 640, Bz - 70))
    mp.cut(bx, ycyl(AX, Bz, -w, w, 110.0))
    part(g, 'Lower arm box', bx)
    part(g, 'Elbow hub', ytube(AX, Bz, -150, 150, 110.0, TRB['d'] / 2))
    pin = ycyl(AX, Bz, -200.0, 200.0, TRB['d'] / 2)
    mp.union(pin, ycyl(AX, Bz, YT[0] - 10.0, -200.0, SRB['22207 E']['d'] / 2))
    part(g, 'Elbow pin', pin)
    return g


def build_crank(root, out3, cw=100.0, r_cw=600.0):
    """q3 = 0, pointing back (-X): its shaft in the 61836, its web, J3's output, the pin C on the web's outer face
    for the parallel rod's eye, and the counterweight on four standoffs outboard of the rod's layer."""
    g = group(root, 'Crank')
    y0, y1 = Y_CRANK - 30.0, Y_CRANK + 30.0
    part(g, 'Crank shaft', ycyl(AX, AZ, 505, y0, pl.DGBB['61836']['d'] / 2))
    web = ycyl(AX, AZ, y0, y1, 150)
    mp.union(web, mp.box(AX - 830, AX, y0, y1, AZ - 150, AZ + 150))
    mp.union(web, ycyl(AX - R, AZ, y0, y1, 75))
    part(g, 'Crank web', web)
    part(g, 'RV-500N output J3', out3)
    part(g, 'Pin C', ycyl(AX - R, AZ, y1, YR[1], SRB['22213 E']['d'] / 2))
    a, h = (v * 1000 for v in pl.CW['block'])
    tk = pdr.cw_thickness(cw) * 1000.0
    xc = AX - r_cw
    yb = Y['block'] - 30.0
    part(g, 'Counterweight block', mp.box(xc - a / 2, xc + a / 2, yb, yb + tk, AZ - h / 2, AZ + h / 2))
    for k, (al, pe) in enumerate(STANDOFFS):
        part(g, 'Counterweight standoff %d' % (k + 1), ycyl(AX - al, AZ + pe, y1, yb, 25.0))
    return g


STANDOFFS = pl.STANDOFFS      # the counterweight's standoffs, where the parallel rod never swings


def build_upper_arm(root):
    """q3 = 0, along +X from the elbow: the yoke's cheeks round the elbow with a 32213 in each, the box, the front
    fork with the wrist's 32213 pair, and the rear lever: a torque tube round the elbow's axis out to the lever's
    plate in the crank's layer, the pin D for the parallel rod."""
    g = group(root, 'Upper arm')
    B, W, Dp = HOME['B'], HOME['W'], HOME['D']
    for k, sy in enumerate((-1, 1)):
        y0, y1 = sorted((sy * 160.0, sy * 200.0))
        ch = mp.box(B[0], B[0] + 400, y0, y1, B[1] - 100, B[1] + 100)
        mp.union(ch, ycyl(B[0], B[1], y0, y1, 100))
        mp.cut(ch, ycyl(B[0], B[1], y0 - 1, y1 + 1, TRB['D'] / 2))
        part(g, 'Upper arm cheek %d' % (k + 1), ch)
        ring_y(g, 'Bearing 32213 elbow %d' % (k + 1), B[0], B[1], sy * 180.0, TRB['T'], TRB['D'] / 2, TRB['d'] / 2)
    part(g, 'Yoke bulkhead', mp.box(B[0] + 400, B[0] + 420, -200, 200, B[1] - 100, B[1] + 100))
    s = S['upper']
    d, w, t = mm(s['d']), mm(s['w']), mm(s['t'])
    bx = mp.box(B[0] + 420, W[0] - 120, -w / 2, w / 2, B[1] - d / 2, B[1] + d / 2)
    mp.cut(bx, mp.box(B[0] + 410, W[0] - 110, -w / 2 + t, w / 2 - t, B[1] - d / 2 + t, B[1] + d / 2 - t))
    part(g, 'Upper arm box', bx)
    for k, sy in enumerate((-1, 1)):
        y0, y1 = sorted((sy * 130.0, sy * 170.0))
        fk = mp.box(W[0] - 120, W[0], y0, y1, W[1] - 100, W[1] + 100)
        mp.union(fk, ycyl(W[0], W[1], y0, y1, 90))
        mp.cut(fk, ycyl(W[0], W[1], y0 - 1, y1 + 1, TRB['D'] / 2))
        part(g, 'Front fork cheek %d' % (k + 1), fk)
        ring_y(g, 'Bearing 32213 wrist %d' % (k + 1), W[0], W[1], sy * 150.0, TRB['T'], TRB['D'] / 2, TRB['d'] / 2)
    tb = S['tube_B']
    part(g, 'Torque tube', ytube(B[0], B[1], 200.0, Y['lever'] - 30.0, mm(tb['Do']) / 2, mm(tb['Do']) / 2 - mm(tb['t'])))
    lv = mp.box(Dp[0], B[0], Y['lever'] - 30.0, Y['lever'] + 30.0, B[1] - 150, B[1] + 150)
    mp.union(lv, ycyl(B[0], B[1], Y['lever'] - 30.0, Y['lever'] + 30.0, 150))
    mp.union(lv, ycyl(Dp[0], Dp[1], Y['lever'] - 30.0, Y['lever'] + 30.0, 75))
    part(g, 'Rear lever', lv)
    part(g, 'Pin D', ycyl(Dp[0], Dp[1], Y['lever'] + 30.0, YR[1], SRB['22213 E']['d'] / 2))
    return g


def rod_eye(g, name, c, y0, y1, br, ro):
    part(g, name + ' eye', ytube(c[0], c[1], y0, y1, ro, br['D'] / 2))
    yc = (y0 + y1) / 2
    ring_y(g, 'Bearing %s %s' % (br['name'], name), c[0], c[1], yc, br['B'], br['D'] / 2, br['d'] / 2)


def build_rod(root, name, p0, p1, sec, ylay, brname, ro, flat=False):
    """A rod between the pivots p0 and p1 at q = 0, its eyes with their spherical roller bearings."""
    g = group(root, name)
    br = dict(SRB[brname], name=brname)
    y0, y1 = ylay
    dx, dz = p1[0] - p0[0], p1[1] - p0[1]
    L = math.hypot(dx, dz)
    e = (dx / L, dz / L)
    a = (p0[0] + e[0] * ro, p0[1] + e[1] * ro)
    b = (p1[0] - e[0] * ro, p1[1] - e[1] * ro)
    if flat:
        body = bar(a, b, y0, y1, mm(sec['d']))
    else:
        r = mm(sec['Do']) / 2
        yc = (y0 + y1) / 2
        body = mp.tube((a[0], yc, a[1]), (b[0], yc, b[1]), r, r - mm(sec['t']))
    part(g, name + ' body', body)
    rod_eye(g, name + ' end 1', p0, y0 + (y1 - y0 - 40.0) / 2, y1 - (y1 - y0 - 40.0) / 2, br, ro) if not flat else \
        rod_eye(g, name + ' end 1', p0, y0, y1, br, ro)
    rod_eye(g, name + ' end 2', p1, y0 + (y1 - y0 - 40.0) / 2, y1 - (y1 - y0 - 40.0) / 2, br, ro) if not flat else \
        rod_eye(g, name + ' end 2', p1, y0, y1, br, ro)
    return g


def build_triangle(root):
    """The elbow's triangle at q = 0: a plate on its 22207 E on the elbow pin, the pins F and G out to the rods."""
    g = group(root, 'Triangle')
    B, F, Gp = HOME['B'], HOME['F'], HOME['G']
    br = SRB['22207 E']
    body = ycyl(B[0], B[1], YT[0], YT[1], 60)
    for p in (F, Gp):
        mp.union(body, ycyl(p[0], p[1], YT[0], YT[1], 50), bar(B, p, YT[0], YT[1], 80))
    mp.union(body, bar(F, Gp, YT[0], YT[1], 60))
    mp.cut(body, ycyl(B[0], B[1], YT[0] - 1, YT[1] + 1, br['D'] / 2))
    part(g, 'Triangle plate', body)
    ring_y(g, 'Bearing 22207 E triangle', B[0], B[1], (YT[0] + YT[1]) / 2, br['B'], br['D'] / 2, br['d'] / 2)
    for nm, p in (('Pin F', F), ('Pin G', Gp)):
        part(g, nm, ycyl(p[0], p[1], YL[0] - 10.0, YT[0], br['d'] / 2))
    return g


def build_wrist(root):
    """The wrist housing at q = 0: its hub between the front fork's bearings, the housing, J4's RV-80N with its
    axis 250 ahead of the wrist pin, its motor, the tool flange, the plate up to the pin H."""
    g = group(root, 'Wrist')
    W, H = HOME['W'], HOME['H']
    x4, zf = W[0] + LAY['e4'], W[1] - LAY['h4']
    part(g, 'Wrist hub', ytube(W[0], W[1], -130.0, 130.0, 75.0, TRB['d'] / 2))
    part(g, 'Wrist pin', ycyl(W[0], W[1], -170.0, 170.0, TRB['d'] / 2))
    Rr, Lr = RV80
    zc = zf + 10.0 + Lr            # the case's top
    hs = mp.box(W[0] + 40.0, x4 + Rr + 15.0, -110.0, 110.0, zc, W[1] + 60.0)
    mp.cut(hs, mp.box(W[0] + 55.0, x4 + Rr, -95.0, 95.0, zc + 15.0, W[1] + 45.0))     # a box of 15 mm walls
    mp.cut(hs, zcyl(zc - 1.0, zc + 16.0, 60.0, x4))                                   # J4's input through its floor
    mp.union(hs, mp.box(W[0], W[0] + 60.0, -130.0, 130.0, W[1] - 75.0, W[1] + 60.0))
    mp.cut(hs, ycyl(W[0], W[1], -131.0, 131.0, 75.0))
    part(g, 'Wrist housing', hs)
    case = ztube(zf + 10.0, zc, Rr, 40.0, x4)
    part(g, 'RV-80N case J4', case)
    part(g, 'RV-80N output J4', ztube(zf, zf + 10.0, Rr - 12.5, 30.0, x4))
    part(g, 'Servo motor J4', zcyl(W[1] + 60.0, W[1] + 260.0, 65.0, x4), 'Aluminum')
    part(g, 'Tool flange', zcyl(zf - 30.0, zf, 125.0, x4))
    # the plate up to H leaves the housing 230 mm ahead of the wrist pin, outside the circle the front fork's cheeks
    # sweep round the pin as the upper arm turns (their corner 156 mm out)
    foot = (W[0] + 230.0, W[1])
    hp = bar(foot, H, YT[0], YT[1], 100.0)
    mp.union(hp, ycyl(H[0], H[1], YT[0], YT[1], 50))
    mp.union(hp, mp.box(foot[0] - 30.0, foot[0] + 30.0, YT[0], -110.0, W[1] - 50.0, W[1] + 50.0))
    part(g, 'H plate', hp)
    part(g, 'Pin H', ycyl(H[0], H[1], YL[0] - 10.0, YT[0], SRB['22207 E']['d'] / 2))
    return g


def build_balancer(root):
    zp, zq = AZ - A_MM, AZ + B_MM
    L0 = zq - zp
    out = []
    for k, ys in enumerate((-1, 1)):
        yc = ys * Y_CYL
        bar_ = group(root, 'Balancer barrel %d' % (k + 1))
        b = ycyl(AX, zp, yc - 30, yc + 30, 25)
        mp.union(b, zcyl(zp + 20, zp + BARREL[0] + 10, 20, AX, yc))
        mp.union(b, ztube(zp + BARREL[0], zp + BARREL[1], 37.5, 31.5, AX, yc))
        mp.union(b, zcyl(zp + BARREL[0], zp + BARREL[0] + 10, 31.6, AX, yc))
        mp.union(b, ztube(zp + BARREL[1] - 10, zp + BARREL[1], 37.5, 16.5, AX, yc))
        mp.cut(b, ycyl(AX, zp, yc - 31, yc + 31, 15.5))
        mp.add_body(bar_, b, 'Balancer barrel', 'Steel')
        rod = group(root, 'Balancer rod %d' % (k + 1))
        sp = L0 - ROD_LEN
        r = ycyl(AX, zq, yc - 30, yc + 30, 25)
        mp.union(r, zcyl(zp + sp, zq - 20, 16, AX, yc))
        mp.union(r, zcyl(zp + sp - 20, zp + sp, 31.0, AX, yc))
        mp.cut(r, ycyl(AX, zq, yc - 31, yc + 31, 15.5))
        mp.add_body(rod, r, 'Balancer rod', 'Steel')
        out.append((bar_, rod))
    return out


# ---------- the pose ----------
def place(occ, q1, turn=0.0, about=(0.0, 0.0), move=(0.0, 0.0)):
    """The group turned about the y axis through `about` (x, z) by `turn` (rad, right-handed about +Y), moved by
    `move` (x, z), then the whole turned by q1 about J1 (rad)."""
    m = adsk.core.Matrix3D.create()
    if turn:
        m.setToRotation(turn, V3(0, 1, 0), P3(about[0] * CM, 0, about[1] * CM))
    t = adsk.core.Matrix3D.create()
    t.translation = V3(move[0] * CM, 0, move[1] * CM)
    m.transformBy(t)
    r = adsk.core.Matrix3D.create()
    r.setToRotation(q1, V3(0, 0, 1), P3(0, 0, 0))
    m.transformBy(r)
    occ.transform2 = m


def set_pose(G_, q1=0.0, q2=0.0, q3=0.0):
    """Every group where the joints' angles (deg) put it: the lower arm turned by q2 about the shoulder, the crank
    and the upper arm by -q3 (q3 turns them the other way about +Y), the parallel rod with q2 about its pin C, the
    levelling rods with q2 and q3, the triangle and the wrist moved with the elbow and the wrist pin."""
    if not pd.in_range(q2, q3):
        raise ValueError('The pose q2 = %.1f, q3 = %.1f deg is outside the working range (psi = %.1f).'
                         % (q2, q3, pd.psi(q2, q3)))
    p = pd.pts(q2, q3)
    a1, a2, a3 = math.radians(q1), math.radians(q2), math.radians(q3)
    d = lambda k: (p[k][0] - HOME[k][0], p[k][1] - HOME[k][1])
    place(G_['Turret'], a1)
    place(G_['Lower arm'], a1, a2, HOME['A'])
    place(G_['Crank'], a1, -a3, HOME['A'])
    place(G_['Upper arm'], a1, -a3, HOME['B'], d('B'))
    place(G_['Parallel rod'], a1, a2, HOME['C'], d('C'))
    place(G_['Levelling rod 1'], a1, a2, HOME['E'])
    place(G_['Levelling rod 2'], a1, -a3, HOME['G'], d('G'))
    place(G_['Triangle'], a1, 0.0, (0, 0), d('B'))
    place(G_['Wrist'], a1, 0.0, (0, 0), d('W'))
    # the balancer: each barrel turned about the turret's pin toward the cross pin, each rod slid along it
    zp = AZ - A_MM
    Qx, Qz = AX + B_MM * math.sin(a2), AZ + B_MM * math.cos(a2)
    Lb = math.hypot(Qx - AX, Qz - zp)
    th = math.atan2(Qx - AX, Qz - zp)
    L0 = (AZ + B_MM) - zp
    for bar_, rod in G_['balancer']:
        for occ, slide in ((bar_, 0.0), (rod, Lb - L0)):
            place(occ, a1, th, (AX, zp), (math.sin(th) * slide, math.cos(th) * slide))
    adsk.doEvents()
    return p


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0:
            continue
        wds = o.component.name.split()
        if wds[-1].isdigit():
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


_last = {}
FIELDS = [('q1', 'J1, q1 (deg)', 0.0), ('q2', 'J2, q2 (deg)', round(pd.waypoints()['lo']['q2'], 3)),
          ('q3', 'J3, q3 (deg)', round(pd.waypoints()['lo']['q3'], 3))]


def build(q1=0.0, q2=None, q3=None):
    q2 = FIELDS[1][2] if q2 is None else q2
    q3 = FIELDS[2][2] if q3 is None else q3
    if not pd.in_range(q2, q3):
        raise ValueError('The pose q2 = %.1f, q3 = %.1f deg is outside the working range (psi = %.1f).'
                         % (q2, q3, pd.psi(q2, q3)))
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    base, j1_out = build_base(root)
    tur, out2, out3 = build_turret(root, j1_out)
    G_ = {'Base': base, 'Turret': tur}
    G_['Lower arm'] = build_lower_arm(root, out2)
    G_['Crank'] = build_crank(root, out3)
    G_['Upper arm'] = build_upper_arm(root)
    G_['Parallel rod'] = build_rod(root, 'Parallel rod', HOME['C'], HOME['D'], S['rod_p'], YR, '22213 E', 75.0, flat=True)
    G_['Levelling rod 1'] = build_rod(root, 'Levelling rod 1', HOME['E'], HOME['F'], S['rod_l'], YL, '22207 E', 50.0)
    G_['Levelling rod 2'] = build_rod(root, 'Levelling rod 2', HOME['G'], HOME['H'], S['rod_l'], YL, '22207 E', 50.0)
    G_['Triangle'] = build_triangle(root)
    G_['Wrist'] = build_wrist(root)
    G_['balancer'] = build_balancer(root)
    colours = {'Base': 'Paint - Enamel Glossy (Grey)', 'Turret': 'Paint - Enamel Glossy (Blue)',
               'Lower arm': 'Paint - Enamel Glossy (Yellow)', 'Upper arm': 'Paint - Enamel Glossy (Yellow)',
               'Crank': 'Paint - Enamel Glossy (Grey)', 'Wrist': 'Paint - Enamel Glossy (Blue)'}
    for k, c in colours.items():
        mp.colour(des, G_[k], c)
    set_pose(G_, q1, q2, q3)
    _last.update(groups=G_)
    app.activeViewport.fit()
    n = parts_list(root)
    return ('Palletizer built at q1 %.1f, q2 %.2f, q3 %.2f deg.\nJ1 RV-700N (for J1\'s stiffness with the arms); '
            'J2 and J3 RV-500N; J4 RV-80N, ratio %d.\nArms: lower %s, upper %s (box, mm); parallel rod %s; '
            'levelling rods %s.\nBearings: 32213 pairs at the elbow and the wrist, 22213 E in the parallel rod\'s eyes, '
            '22207 E in the levelling rods\' eyes and under the triangle.\n\nParts: %d components; %s.\n\n[%s]'
            % (q1, q2, q3, pl.CHOICE43['i4'], '%g x %g x %g' % tuple(mm(S['lower'][k]) for k in 'dwt'),
               '%g x %g x %g' % tuple(mm(S['upper'][k]) for k in 'dwt'),
               'flat %g x %g' % (mm(S['rod_p']['d']), mm(S['rod_p']['w'])),
               'tube %g x %g' % (mm(S['rod_l']['Do']), mm(S['rod_l']['t'])), sum(n.values()),
               ', '.join('%d %s' % (v, k) for k, v in sorted(n.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []


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
        cid = 'PalletizerRobotCmd'
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
