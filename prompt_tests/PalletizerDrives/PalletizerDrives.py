# =============================================================================
# PalletizerDrives.py  -  Chapter 42 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's drives.
# The base with SKF's slewing ring RKS.23 0841 on its flange; the turret on the ring's inner ring, turned by a
# Nabtesco RV-500N on J1's axis through a disc coupling; the shoulder's two cheeks, J2's RV-500N and motor on one,
# on the other the crank with its counterweight, turned by J3's RV-500N on an outboard bracket; the root of the
# lower arm, a fork, with the gas balancer's two cylinders in its gap. The arms themselves are
# projects/palletizer/PalletizerRobot/PalletizerRobot.py's. Written from PROMPT.txt alone (a prompt test).
#
#   INPUTS        the dialog: the counterweight added to the crank (kg), its centre's distance behind the shoulder (mm)
#   DERIVED       palletizer_design (the shoulder A, J3's range), palletizer_drives (the ring's plane, the balancer's
#                 a, b and bore, the block's size and thickness, the ring chosen, the reducers' lives, the forces)
#   HARD RULES    build() refuses a counterweight over 200 kg, and one whose block comes within 20 mm of the floor
#                 with the crank turned down to q3 = 60 deg
#   CONVENTIONS   mm in the world, J1 along Z, the arm in XZ facing +X; every part built in place at q = 0; as-built
#                 revolute joints J1 (Z), J2 and J3 (the line through A along Y); set_balancer() places the barrels
#                 and rods for a J2 pose through their proxies from the root, the turret's turn included
#   ACCEPTANCE    the message; 48 components in 40 kinds; no interference at rest, over J1's cycle and over J2's
#                 and J3's ranges; the balancer's pins' distance; the two refusals
# =============================================================================
import importlib
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
import companion_paths as CP        # noqa: E402,F401  every lib/ folder on sys.path
import palletizer_design as pd      # noqa: E402
import palletizer_drives as pdr     # noqa: E402
import mdfc_parts as mp             # noqa: E402
importlib.reload(pd)
importlib.reload(pdr)
importlib.reload(mp)

VERSION = 'PalletizerDrives prompt test 1.0 (2026-11-12)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

# ---------- derived (palletizer_design, palletizer_drives) ----------
AX, AZ = pd.LAYOUT['A']                     # the shoulder A: 300, 1000
ZR = pdr.Z_RING * 1000.0                    # the ring's plane: 520
GAS = pdr.GAS
A_MM, B_MM = GAS['a'] * 1000.0, GAS['b'] * 1000.0      # 350 below the shoulder, 550 up the arm
BORE, ROD = GAS['bore'] * 1000.0, GAS['rod'] * 1000.0  # 63, 32
CW_A, CW_H = (v * 1000.0 for v in pdr.CW_BLOCK)        # the block: 460 along the crank, 460 high
Q3_DOWN = pd.LIMITS['q3'][1]                            # 60 deg: the crank turned furthest down
R0841 = next(r for r in pdr.RINGS if r['name'] == 'RKS.23 0841')

# ---------- typed here (the prompt) ----------
CW_MAX = 200.0                  # kg; more and J1's RV-500N falls short of its rated life
FLOOR_GAP = 20.0                # mm; the block at q3 = 60 deg stays this far above the floor (z = 0)
Y_CRANK = 640.0                 # the crank's plane
WEB_T = 60.0                    # the crank's web (palletizer_drives.with_counterweight)
RING = dict(bore=842.5, h_out=46.0, d_out=839.5, h_in=56.0, ball=20.0, z_ball=28.0, clear=0.4)
RV500 = (325.0, 170.0)          # the RV-500N drawn 325 across (product page), 170 long (assumed)
Y_CYL = 60.0                    # the cylinders at y = +60 and -60
LUGS = ((-125.0, -100.0), (-15.0, 15.0), (100.0, 125.0))

# ---------- chosen here (the prompt gives none of these; see the report) ----------
N_BALLS = 120                   # the ring's balls, 2 mm apart on dm
PLATE = (-480.0, 560.0, -600.0, 600.0)  # the turret plate's outline (x0, x1, y0, y1)
CHEEK_X, CHEEK_TOP = (-200.0, 520.0), 1180.0
CHEEK_Y = (520.0, 600.0)        # each cheek between |y| = 520 and 600
PIN_R = 15.0                    # the balancer's pins P and Q, 30 mm
ROD_LEN, BARREL = 560.0, (30.0, 480.0)    # the rod's eye to its piston's face; the barrel from P (mm along its axis)
BRG_J2 = (280.0, 350.0, 33.0)   # the lower arm's support on the J3 side, SKF 61856 MA (d, D, B)
BRG_J3 = (180.0, 225.0, 22.0)   # the crank's shaft in the J3 cheek, SKF 61836 (d, D, B)

Z0 = ZR + RING['h_in']          # the turret plate's underside, on the inner ring: 576
Z1 = Z0 + 60.0                  # its top: 636
ZP, ZQ = AZ - A_MM, AZ + B_MM   # the turret's pin P (650) and the fork's cross pin Q (1550) at q2 = 0


# ---------- solids as data: world mm, built by solid() ----------
def box(x0, x1, y0, y1, z0, z1):
    return ('box', x0, x1, y0, y1, z0, z1)


def cyl(p0, p1, r):
    return ('cyl', tuple(p0), tuple(p1), r)


def tube(p0, p1, ro, ri):
    return ('tube', tuple(p0), tuple(p1), ro, ri)


def ycyl(x, z, y0, y1, r):
    return cyl((x, y0, z), (x, y1, z), r)


def ytube(x, z, y0, y1, ro, ri):
    return tube((x, y0, z), (x, y1, z), ro, ri)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return cyl((x, y, z0), (x, y, z1), r)


def ztube(z0, z1, ro, ri, x=0.0, y=0.0):
    return tube((x, y, z0), (x, y, z1), ro, ri)


def torus(c, R, r):
    """A torus about a vertical axis through c."""
    return ('torus', tuple(c), R, r)


def balls(c, R, r, n):
    """n balls of radius r on a circle of radius R about a vertical axis through c."""
    return ('balls', tuple(c), R, r, n)


def U(*a):
    return ('union',) + a


def C(*a):
    return ('cut',) + a


def solid(s):
    """The temporary body of a solid given as data."""
    k = s[0]
    if k == 'box':
        return mp.box(*s[1:])
    if k == 'cyl':
        return mp.cyl(*s[1:])
    if k == 'tube':
        return mp.tube(*s[1:])
    if k == 'sphere':
        return mp.sphere(*s[1:])
    if k == 'torus':                     # createTorus ignores its centre: make it at the origin and move it
        c, R, r = s[1:]
        t = mp.tbm().createTorus(P3(0, 0, 0), V3(0, 0, 1), R * CM, r * CM)
        m = adsk.core.Matrix3D.create()
        m.translation = V3(c[0] * CM, c[1] * CM, c[2] * CM)
        mp.tbm().transform(t, m)
        return t
    if k == 'balls':
        c, R, r, n = s[1:]
        b = None
        for i in range(n):
            a = 2 * math.pi * i / n
            sp = mp.sphere((c[0] + R * math.cos(a), c[1] + R * math.sin(a), c[2]), r)
            b = sp if b is None else mp.union(b, sp)
        return b
    if k == 'union':
        return mp.union(solid(s[1]), *[solid(x) for x in s[2:]])
    if k == 'cut':
        return mp.cut(solid(s[1]), *[solid(x) for x in s[2:]])
    raise ValueError(k)


# ---------- the parts at q = 0 ----------
def along(axis, a, b, ro, ri=0.0):
    """A cylinder (ri = 0) or a tube on J1's axis ('z', through the origin) or the shoulder's ('y', through A)."""
    a, b = sorted((a, b))
    p0, p1 = ((0.0, 0.0, a), (0.0, 0.0, b)) if axis == 'z' else ((AX, a, AZ), (AX, b, AZ))
    return tube(p0, p1, ro, ri) if ri > 0 else cyl(p0, p1, ro)


def rv(axis, start, sign):
    """An RV-500N, 325 across and 170 long: the case from `start` toward `sign`, a cover over its first 15 mm (the
    motor's face), the output's disc at the far end and its hub inside the case. Returns (case, output)."""
    Rr, Lr = RV500[0] / 2.0, RV500[1]

    def seg(s0, s1, ro, ri):
        return along(axis, start + sign * s0, start + sign * s1, ro, ri)
    case = U(seg(0.0, Lr - 10.0, Rr, Rr - 27.5), seg(0.0, 15.0, Rr, 40.0))
    out = U(seg(Lr - 10.0, Lr, Rr - 12.5, 40.0), seg(20.0, Lr - 9.5, Rr - 37.5, Rr - 72.5))
    return case, out


def block_low(r_cw):
    """The block's lowest corner (z, mm) with the crank turned down to q3 = 60 deg: the block 460 along the crank,
    its centre r_cw behind A on the crank's line, 460 across it."""
    t = math.radians(Q3_DOWN)
    return AZ - (r_cw + CW_A / 2.0) * math.sin(t) - CW_H / 2.0 * math.cos(t)


def cw_thick(cw):
    return pdr.cw_thickness(cw) * 1000.0


def parts(cw=100.0, r_cw=600.0):
    """Every part at q = 0 in world mm: {'Base': [(name, solid, material)], 'Turret': [...], 'Lower arm': [...],
    'Crank': [...], 'balancer': [((barrel name, solid), (rod name, solid)), ...]}."""
    out = {'Base': [], 'Turret': [], 'Lower arm': [], 'Crank': [], 'balancer': []}

    def add(g, name, s, mat='Steel'):
        out[g].append((name, s, mat))

    # the base: the floor plate, the column, the flange; J1's plate, reducer case and motor inside the column
    add('Base', 'Floor plate', box(-550, 550, -550, 550, 0, 40))
    add('Base', 'Column', ztube(40, 470, 330, 290))
    add('Base', 'Base flange', ztube(470, ZR, 500, 260))
    add('Base', 'J1 reducer plate', C(zcyl(260, 270, 290), zcyl(259, 271, 40)))
    case1, out1 = rv('z', 270.0, 1)
    add('Base', 'RV-500N case J1', case1)
    add('Base', 'Servo motor J1', zcyl(60, 260, 95), 'Aluminum')
    add('Base', 'J1 stop', C(box(-565, -478, -40, 40, 470, 560), zcyl(469, ZR, 500)))
    # the ring (SKF pp. 66-67): the outer ring on the flange, the inner ring under the turret, one row of balls
    zb = ZR + RING['z_ball']
    groove = torus((0, 0, zb), R0841['dm'] / 2, RING['ball'] / 2 + RING['clear'])
    add('Base', 'Slewing ring outer ring', C(ztube(ZR, ZR + RING['h_out'], R0841['D'] / 2, RING['bore'] / 2), groove))
    add('Base', 'Slewing ring balls', balls((0, 0, zb), R0841['dm'] / 2, RING['ball'] / 2, N_BALLS))

    # the turret
    add('Turret', 'Slewing ring inner ring',
        C(U(ztube(ZR + 10, Z0, RING['d_out'] / 2, 395), ztube(Z0 - 12, Z0, 395.5, R0841['d'] / 2)), groove))
    add('Turret', 'Turret plate', C(box(PLATE[0], PLATE[1], PLATE[2], PLATE[3], Z0, Z1), zcyl(Z0 - 1, Z1 + 1, 100),
                                    box(AX - 100, AX + 100, -130, 130, Z1 - 30, Z1 + 1)))
    add('Turret', 'J1 stop dog', box(515, 560, -25, 25, Z0 - 48, Z0))
    add('Turret', 'RV-500N output J1', out1)
    z_out = 270.0 + RV500[1]
    add('Turret', 'Coupling disc 1', zcyl(z_out, z_out + 15, 140))
    add('Turret', 'Coupling spacer', ztube(z_out + 15, Z0 - 15, 60, 45))
    add('Turret', 'Coupling disc 2', zcyl(Z0 - 15, Z0, 140))
    yi, yo = CHEEK_Y
    add('Turret', 'Cheek J2 side', C(box(CHEEK_X[0], CHEEK_X[1], -yo, -yi, Z1, CHEEK_TOP),
                                     ycyl(AX, AZ, -yo - 1, -yi + 1, RV500[0] / 2)))
    d2, D2, B2 = BRG_J2
    d3, D3, B3 = BRG_J3
    y_brg2 = yi - 5.0                   # the J2 support bearing's outer face, against the housing's lip
    add('Turret', 'Cheek J3 side', C(U(box(CHEEK_X[0], CHEEK_X[1], yi, yo, Z1, CHEEK_TOP),
                                       ytube(AX, AZ, y_brg2 - 40.0, y_brg2 + 0.5, D2 / 2 + 25, D2 / 2),
                                       ytube(AX, AZ, y_brg2, yi + 0.5, D2 / 2 + 25, D2 / 2 - 15)),
                                     ycyl(AX, AZ, yi - 1, yo + 1, D3 / 2)))
    case2, out2 = rv('y', -yo - 20.0, 1)
    add('Turret', 'RV-500N case J2', case2)
    add('Turret', 'Servo motor J2', ycyl(AX, AZ, -yo - 250.0, -yo - 20.0, 95), 'Aluminum')
    add('Turret', 'J2 support bearing', ytube(AX, AZ, y_brg2 - B2, y_brg2, D2 / 2, d2 / 2))
    add('Turret', 'J3 support bearing', ytube(AX, AZ, yo - B3, yo, D3 / 2, d3 / 2))
    y_br = 840.0                        # the J3 bracket's inner face
    add('Turret', 'J3 bracket', C(box(150, 640, y_br, y_br + 30, 820, CHEEK_TOP), ycyl(AX, AZ, y_br - 1, y_br + 31, 40)))
    for k, zp in enumerate((850.0, 1150.0)):
        add('Turret', 'J3 bracket post %d' % (k + 1), cyl((480, yo, zp), (480, y_br, zp), 25))
    case3, out3 = rv('y', y_br, -1)
    add('Turret', 'RV-500N case J3', case3)
    add('Turret', 'Servo motor J3', ycyl(AX, AZ, y_br + 30, y_br + 260, 95), 'Aluminum')
    for k, (y0, y1) in enumerate(LUGS):
        add('Turret', 'Balancer lug %d' % (k + 1), C(box(AX - 30, AX + 30, y0, y1, Z1 - 30, ZP + 30),
                                                      ycyl(AX, ZP, y0 - 1, y1 + 1, PIN_R)))
    add('Turret', 'Balancer pin P', ycyl(AX, ZP, LUGS[0][0], LUGS[-1][1], PIN_R))
    # the gas bottles: each holds what of the cylinder's gas column V0 = (V0/A) A its barrel does not at q2 = 0
    area = math.pi * BORE ** 2 / 4
    in_barrel = area * ((A_MM + B_MM - ROD_LEN - 20.0) - (BARREL[0] + 10.0))
    length = (GAS['V0A'] * 1000.0 * area - in_barrel) / (math.pi * 50.0 ** 2) + 10.0
    zb_ = Z1 + 55.0
    for k, s in enumerate((-1, 1)):
        y0, y1 = sorted((s * 20.0, s * (20.0 + length)))
        add('Turret', 'Gas bottle %d' % (k + 1), U(ytube(-330, zb_, y0, y1, 55, 50), ycyl(-330, zb_, y0, y0 + 5, 50.5),
                                                    ycyl(-330, zb_, y1 - 5, y1, 50.5)))

    # the lower arm's root at q2 = 0: J2's output, the fork's plates and hub bosses, the cross pin Q, the root box
    add('Lower arm', 'RV-500N output J2', out2)
    for k, s in enumerate((-1, 1)):
        y0, y1 = sorted((s * 150.0, s * 190.0))
        add('Lower arm', 'Fork plate %d' % (k + 1), C(U(box(AX - 110, AX + 110, y0, y1, AZ, AZ + 700),
                                                        ycyl(AX, AZ, y0, y1, 160)),
                                                      ycyl(AX, ZQ, y0 - 1, y1 + 1, PIN_R)))
    add('Lower arm', 'Hub boss J2 side', ytube(AX, AZ, -yo - 20.0 + RV500[1], -190, d2 / 2, 60))
    add('Lower arm', 'Hub boss J3 side', ytube(AX, AZ, 190, y_brg2, d2 / 2, 60))
    add('Lower arm', 'Balancer cross pin Q', ycyl(AX, ZQ, -190, 190, PIN_R))
    add('Lower arm', 'Lower arm root box', C(box(AX - 110, AX + 110, -190, 190, AZ + 700, AZ + 900),
                                             box(AX - 95, AX + 95, -175, 175, AZ + 715, AZ + 901)))

    # the crank at q3 = 0, pointing back (-X): its shaft in the J3 cheek, its web in the plane y = 640, J3's output on
    # the web's outer face, and the block on that face
    yw0, yw1 = Y_CRANK - WEB_T / 2, Y_CRANK + WEB_T / 2
    reach = max(pd.LAYOUT['r'] + 75.0, r_cw + CW_A / 2)
    add('Crank', 'Crank shaft', ycyl(AX, AZ, yi, yw0, d3 / 2))
    add('Crank', 'Crank web', U(ycyl(AX, AZ, yw0, yw1, 150), box(AX - reach, AX, yw0, yw1, AZ - 150, AZ + 150)))
    add('Crank', 'RV-500N output J3', out3)
    xc = AX - r_cw
    add('Crank', 'Counterweight block', box(xc - CW_A / 2, xc + CW_A / 2, yw1, yw1 + cw_thick(cw), AZ - CW_H / 2,
                                            AZ + CW_H / 2))

    # the balancer at q2 = 0: each barrel from its eye on P, each rod from its piston to its eye on Q
    sp = A_MM + B_MM - ROD_LEN
    for k, s in enumerate((-1, 1)):
        yc = s * Y_CYL
        b = U(ycyl(AX, ZP, yc - 30, yc + 30, 25),
              zcyl(ZP + 20, ZP + BARREL[0] + 10, 20, AX, yc),
              ztube(ZP + BARREL[0], ZP + BARREL[1], 37.5, BORE / 2, AX, yc),
              zcyl(ZP + BARREL[0], ZP + BARREL[0] + 10, BORE / 2 + 0.1, AX, yc),
              ztube(ZP + BARREL[1] - 10, ZP + BARREL[1], 37.5, ROD / 2 + 0.5, AX, yc))
        b = C(b, ycyl(AX, ZP, yc - 31, yc + 31, PIN_R + 0.5))
        r = U(ycyl(AX, ZQ, yc - 30, yc + 30, 25),
              zcyl(ZP + sp - 0.5, ZQ - 20, ROD / 2, AX, yc),
              zcyl(ZP + sp - 20, ZP + sp, BORE / 2 - 0.5, AX, yc))
        r = C(r, ycyl(AX, ZQ, yc - 31, yc + 31, PIN_R + 0.5))
        out['balancer'].append((('Balancer barrel %d' % (k + 1), b), ('Balancer rod %d' % (k + 1), r)))
    return out


# ---------- the balancer's pose ----------
def balancer_pose(q2):
    """For J2 at q2 (deg): the cylinders' turn about P from the vertical (rad, right-handed about +Y), the pins'
    distance L (mm) and the rods' slide out of their barrels from q2 = 0 (mm)."""
    a2 = math.radians(q2)
    qx, qz = AX + B_MM * math.sin(a2), AZ + B_MM * math.cos(a2)
    L = math.hypot(qx - AX, qz - ZP)
    return math.atan2(qx - AX, qz - ZP), L, L - (A_MM + B_MM)


def set_balancer(cyls, q2, tur=None):
    """Each barrel turned about the turret's pin P toward the cross pin Q, each rod turned with it and slid along
    it, then the whole turned with the turret: set on their proxies from the root, in world coordinates. Returns
    the pins' distance (mm)."""
    tur = tur if tur is not None else _last['occ']['tur']
    th, L, slide = balancer_pose(q2)
    T = tur.transform2.copy()
    for bar_, rod in cyls:
        for occ, s in ((bar_, 0.0), (rod, slide)):
            m = adsk.core.Matrix3D.create()
            m.setToRotation(th, V3(0, 1, 0), P3(AX * CM, 0, ZP * CM))
            t = adsk.core.Matrix3D.create()
            t.translation = V3(math.sin(th) * s * CM, 0, math.cos(th) * s * CM)
            m.transformBy(t)
            m.transformBy(T)
            occ.transform2 = m
    return L


# ---------- the build ----------
def refusal(cw, r_cw):
    if cw <= 0:
        return 'Give the counterweight a mass above 0 kg (%g kg given).' % cw
    if cw > CW_MAX:
        return ('A counterweight of %g kg is over the %g kg limit: with it J1\'s RV-500N falls short of its rated '
                'life (palletizer_drives.rv_life).' % (cw, CW_MAX))
    z = block_low(r_cw)
    if z < FLOOR_GAP:
        return ('A counterweight %g mm behind the shoulder comes within %g mm of the floor: with the crank turned '
                'down to q3 = %g deg its block\'s lowest corner would be at z = %.0f mm, the floor at 0.'
                % (r_cw, FLOOR_GAP, Q3_DOWN, z))
    return None


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


def _part(comp, name, s, mat):
    o = mp.new_occ(comp, name)
    mp.add_body(o, solid(s), name, mat)
    return o


def _joint_point(comp, p):
    """A sketch point at p (mm, the component's own frame, which is the world's at q = 0), on a plane parallel to
    the component's XY plane through it, so that the sketch's axes are the component's (a sketch on XZ maps its
    coordinates unexpectedly). If the offset plane is refused, the point goes off the XY plane's sketch."""
    target = P3(p[0] * CM, p[1] * CM, p[2] * CM)
    plane = comp.xYConstructionPlane
    if abs(p[2]) > 1e-9:
        try:
            ci = comp.constructionPlanes.createInput()
            ci.setByOffset(comp.xYConstructionPlane, VI(p[2] * CM))
            plane = comp.constructionPlanes.add(ci)
        except Exception:
            plane = comp.xYConstructionPlane
    sk = comp.sketches.add(plane)
    sk.name = 'Joint point'
    pt = sk.sketchPoints.add(sk.modelToSketchSpace(target))
    if pt.worldGeometry.distanceTo(target) > 1e-5:
        raise RuntimeError('the joint point for %s landed at %s, not at %s mm' % (comp.name, pt.worldGeometry.asArray(), p))
    return pt


def _turn(occ, axis):
    """The occurrence's turn about +Z or +Y (rad), read from its transform."""
    t = occ.transform2
    if axis == 'z':
        return math.atan2(t.getCell(1, 0), t.getCell(0, 0))
    return math.atan2(t.getCell(0, 2), t.getCell(0, 0))


def _sign(joint, occ, axis):
    """+1 when a positive joint value turns the part right-handed about the world's axis, -1 the other way."""
    s = 1
    try:
        joint.jointMotion.rotationValue = 0.2
        adsk.doEvents()
        a = _turn(occ, axis)
        if abs(a) > 1e-3:
            s = 1 if a > 0 else -1
    finally:
        joint.jointMotion.rotationValue = 0.0
        adsk.doEvents()
    return s


_last = {}


def _fmt(x):
    return '{:,.0f}'.format(x).replace(',', ' ')


def build(cw=100.0, r_cw=600.0):
    cw, r_cw = float(cw), float(r_cw)
    why = refusal(cw, r_cw)
    if why:
        raise ValueError(why)
    # Chapter 42's numbers with this counterweight (palletizer_drives.design: the cycle with the full model)
    d = pdr.design(c=dict(pdr.CHOICE, cw=cw, r_cw=r_cw / 1000.0))
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    root = des.rootComponent
    P_ = parts(cw, r_cw)

    base = mp.new_occ(root, 'Base')
    base.isGrounded = True
    tur = mp.new_occ(root, 'Turret')
    tur.isGrounded = False
    occs = {}
    for name, s, mat in P_['Base']:
        occs[name] = _part(base.component, name, s, mat)
    adsk.doEvents()
    for name, s, mat in P_['Turret']:
        occs[name] = _part(tur.component, name, s, mat)
    adsk.doEvents()
    groups = {}
    for g in ('Lower arm', 'Crank'):
        o = mp.new_occ(tur.component, g)
        o.isGroundToParent = False              # it moves on its joint
        for name, s, mat in P_[g]:
            occs[name] = _part(o.component, name, s, mat)
        groups[g] = o
        adsk.doEvents()
    cyls = []
    for (nb, sb), (nr, sr) in P_['balancer']:
        pair = []
        for name, s in ((nb, sb), (nr, sr)):
            o = mp.new_occ(tur.component, name)
            mp.add_body(o, solid(s), name.rsplit(' ', 1)[0], 'Steel')
            o.isGroundToParent = False          # placed by set_balancer()
            pair.append(o.createForAssemblyContext(tur))
        cyls.append(tuple(pair))
    for o, c in ((base, 'Paint - Enamel Glossy (Grey)'), (tur, 'Paint - Enamel Glossy (Blue)'),
                 (groups['Lower arm'], 'Paint - Enamel Glossy (Yellow)'), (groups['Crank'], 'Paint - Enamel Glossy (Grey)')):
        mp.colour(des, o, c)

    # the joints: as built, each on a sketch point of the moving group on its axis, the moving group first
    la, cr = groups['Lower arm'], groups['Crank']
    la_p, cr_p = la.createForAssemblyContext(tur), cr.createForAssemblyContext(tur)
    ch2_p = occs['Cheek J2 side'].createForAssemblyContext(tur)
    ch3_p = occs['Cheek J3 side'].createForAssemblyContext(tur)
    sp1 = _joint_point(tur.component, (0.0, 0.0, 0.0)).createForAssemblyContext(tur)
    sp2 = _joint_point(la.component, (AX, 0.0, AZ)).createForAssemblyContext(la_p)
    sp3 = _joint_point(cr.component, (AX, 0.0, AZ)).createForAssemblyContext(cr_p)
    J = adsk.fusion.JointDirections
    joints = {}
    for k, name, mov, fix, pt, dirn in ((1, 'J1', tur, base, sp1, J.ZAxisJointDirection),
                                         (2, 'J2', la_p, ch2_p, sp2, J.YAxisJointDirection),
                                         (3, 'J3', cr_p, ch3_p, sp3, J.YAxisJointDirection)):
        ji = root.asBuiltJoints.createInput(mov, fix, adsk.fusion.JointGeometry.createByPoint(pt))
        ji.setAsRevoluteJointMotion(dirn)
        j = root.asBuiltJoints.add(ji)
        j.name = name
        joints[k] = j
    adsk.doEvents()
    signs = {1: _sign(joints[1], tur, 'z'), 2: _sign(joints[2], la_p, 'y'), 3: _sign(joints[3], cr_p, 'y')}
    _last.clear()
    _last.update(joints={k: (joints[k], signs[k]) for k in (1, 2, 3)}, cyls=cyls,
                 occ={'base': base, 'tur': tur, 'la': la_p, 'crank': cr_p}, design=d)
    try:
        app.activeViewport.fit()
    except Exception:
        pass

    n = parts_list(root)
    red, gas = d['red'], d['gas']
    ring = d['j1']['chosen']['ring']['name']
    lines = ['The palletizer\'s drives of Chapter 42, built at rest (q = 0).',
             'Slewing ring: SKF %s (dm %g mm, one row of %d balls of %g mm)%s.'
             % (ring, R0841['dm'], N_BALLS, RING['ball'],
                '' if ring == R0841['name'] else '; the drawing shows the RKS.23 0841, whose sizes are typed here')]
    for j in (1, 2, 3):
        r = red[j]
        lines.append('J%d %s: %s h of the %s h it needs.' % (j, r['name'], _fmt(r['Lh']), _fmt(r['Lh_need'])))
    lines.append('Balancer: two cylinders of %g mm bore; the pair pushes %.1f to %.1f kN over J2\'s range, '
                 '%.2f to %.2f MPa in each.' % (BORE, gas['Fmin'] / 1e3, gas['Fmax'] / 1e3, gas['p0'] / 1e6,
                                                gas['pmax'] / 1e6))
    lines.append('Counterweight: %g kg at %g mm behind the shoulder, a steel block %g x %g x %.1f mm on the crank\'s '
                 'outer face; its lowest corner %.0f mm above the floor at q3 = %g deg.'
                 % (cw, r_cw, CW_A, CW_H, cw_thick(cw), block_low(r_cw), Q3_DOWN))
    lines.append('Parts: %d components in %d kinds: %s.' % (sum(n.values()), len(n),
                                                            ', '.join('%d %s' % (v, k) for k, v in sorted(n.items()))))
    lines.append('[%s]' % VERSION)
    return '\n'.join(lines)


# ---------- the command dialog ----------
FIELDS = [('cw', 'Counterweight added to the crank (kg)', 100.0),
          ('r_cw', 'Its centre behind the shoulder (mm)', 600.0)]
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
        # the new design opens here: Fusion refuses a new document inside a command's events
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        des = adsk.fusion.Design.cast(app.activeProduct)
        des.designType = adsk.fusion.DesignTypes.ParametricDesignType
        cid = 'PalletizerDrivesPromptTestCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Palletizer drives', 'The palletizer\'s drives of Chapter 42')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PalletizerDrives could not start:\n' + traceback.format_exc())
