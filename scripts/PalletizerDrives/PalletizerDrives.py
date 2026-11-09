# =============================================================================
# PalletizerDrives.py  -  Chapter 42 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's drives.
# The base with an SKF slewing ring RKS.23 0841 on its flange; the turret on the ring's inner ring, turned by a
# Nabtesco RV-500N on the axis through a disc coupling; the shoulder's cheeks with J2's RV-500N and motor on one side
# and the crank with its counterweight, turned by J3's RV-500N on an outboard bracket, on the other; the lower arm's
# root, a fork, with the gas balancer's two cylinders in its gap. The sizing is lib/palletizer_drives.py. Written
# from PROMPT.txt beside it; checked by checks/palletizer_drives_check.py.
#
#   INPUTS        the dialog: the counterweight's mass and its distance behind the shoulder
#   DERIVED       palletizer_drives: the ring, the reducers, the balancer (GAS), the block (CW_BLOCK)
#   HARD RULES    build() refuses a counterweight over 200 kg (CW_MAX) or one whose block would near the floor at
#                 q3 = 60 deg
#   CONVENTIONS   J1 along Z, the arm in XZ facing +X, J2 and J3 about the line through A along Y; subassemblies
#                 Base (grounded), Turret, Lower arm, Crank, and the balancer's barrels and rods
#   ACCEPTANCE    the message; no interference at the poses; the balancer's length against its formula
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
import palletizer_design as pd      # noqa: E402
import palletizer_drives as pdr     # noqa: E402
import mdfc_parts as mp             # noqa: E402
importlib.reload(pd)
importlib.reload(pdr)
importlib.reload(mp)

VERSION = 'PalletizerDrives 1.0 (2026-11-02)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
VI = adsk.core.ValueInput.createByReal

AX, AZ = pd.LAYOUT['A']                 # the shoulder (mm)
ZR = pdr.Z_RING * 1000.0                # the ring's plane (mm)
RING = dict(D=948.0, D2=842.5, d=734.0, d2=839.5, dm=841.0, H=56.0, h=46.0, ball=20.0)   # RKS.23 0841 (pp. 66-67)
RV_R, RV_L = 162.5, 170.0               # the RV-500N as drawn: 325 mm across (product page), 170 mm long (assumed)
Y_CRANK = 640.0                          # the crank's plane, outboard of the J3 cheek and the base's flange
G = pdr.GAS
A_MM, B_MM = G['a'] * 1000.0, G['b'] * 1000.0
Y_CYL = 60.0                             # the balancer's cylinders at y = +60 and -60
ROD_LEN = 560.0                          # from the rod's eye to the piston's face
BARREL = (30.0, 480.0)                   # the barrel's tube from 30 to 480 mm along its axis from the turret's pin
CW_MAX = 200.0                           # above this J1's reducer falls short of its life (Table 42.4)


def design(cw=100.0, r_cw=600.0):
    return dict(cw=float(cw), r_cw=float(r_cw), gas=pdr.gas_design(), choice=pdr.CHOICE)


def block_low(r_cw):
    """The block's lowest corner (mm above the floor) with the crank turned down to q3 = 60 deg: the crank then
    points 240 deg from +X, and the square block's corner hangs (a sin 60 + h cos 60)/2 below its centre."""
    a, h = pdr.CW_BLOCK
    return AZ - r_cw * math.sin(math.radians(60)) - (a * 1000 * math.sin(math.radians(60)) + h * 1000 * math.cos(math.radians(60))) / 2


def refuse(o):
    if o['cw'] > CW_MAX:
        raise ValueError('A counterweight of %.0f kg is over %.0f kg: J1\'s RV-500N then falls short of its life.' % (o['cw'], CW_MAX))
    z = block_low(o['r_cw'])
    if z < 20.0:
        raise ValueError('With the crank turned down to 60 deg the block would reach %.0f mm above the floor: it must stay 20 mm clear.' % z)


# ---------- bodies (world mm) ----------
def ycyl(x, z, y0, y1, r):
    return mp.cyl((x, y0, z), (x, y1, z), r)


def ytube(x, z, y0, y1, ro, ri):
    return mp.tube((x, y0, z), (x, y1, z), ro, ri)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def ztube(z0, z1, ro, ri, x=0.0, y=0.0):
    return mp.tube((x, y, z0), (x, y, z1), ro, ri)


def torus(c, axis, R, r):
    """A torus about `axis` through c. The temporary-body manager makes it at the origin whatever centre it is
    given, so it is made there and moved."""
    t = mp.tbm().createTorus(P3(0, 0, 0), V3(*axis), R * CM, r * CM)
    m = adsk.core.Matrix3D.create()
    m.translation = V3(*[v * CM for v in c])
    mp.tbm().transform(t, m)
    return t


def part(parent, name, tb, material='Steel'):
    o = mp.new_occ(parent.component, name)
    mp.add_body(o, tb, name, material)
    return o


def rv(parent, name, axis, start, sign):
    """An RV-500N as drawn: the case (stator) with its flange at `start`, the output flange and hub (rotor) at the
    other end. axis 'z' or 'y'; sign the direction from the case's flange to the output."""
    def seg(s0, s1, ro, ri=0.0):
        a, b = start + sign * s0, start + sign * s1
        lo, hi = min(a, b), max(a, b)
        if axis == 'z':
            return ztube(lo, hi, ro, ri) if ri else zcyl(lo, hi, ro)
        x, z = parent['x'], parent['z']
        return ytube(x, z, lo, hi, ro, ri) if ri else ycyl(x, z, lo, hi, ro)
    case = seg(0.0, RV_L - 10.0, RV_R, 135.0)
    mp.union(case, seg(0.0, 20.0, 190.0, 135.0))
    rotor = seg(20.0, RV_L - 10.0 - 0.01, 125.0, 90.0)
    mp.union(rotor, seg(RV_L - 10.0, RV_L, 150.0, 40.0))
    return case, rotor


def build_base(root):
    base = mp.new_occ(root, 'Base')
    base.isGrounded = True
    c = base
    part(c, 'Floor plate', mp.box(-550, 550, -550, 550, 0, 40))
    part(c, 'Column', ztube(40, 470, 330, 290))
    part(c, 'Base flange', ztube(470, ZR, 500, 260))
    dia = zcyl(290, 300, 289.99)
    mp.cut(dia, zcyl(280, 310, 100))
    part(c, 'J1 reducer plate', dia)
    # the ring's outer ring: its body round the balls and its flange on the base, a groove for the balls
    o = ztube(ZR, ZR + RING['h'], 445.0, RING['D2'] / 2)
    mp.union(o, ztube(ZR, ZR + 12.0, RING['D'] / 2, 445.0))
    mp.cut(o, torus((0, 0, ZR + 28.0), (0, 0, 1), RING['dm'] / 2, RING['ball'] / 2 + 0.4))
    part(c, 'Slewing ring outer ring', o)
    part(c, 'Slewing ring balls', torus((0, 0, ZR + 28.0), (0, 0, 1), RING['dm'] / 2, RING['ball'] / 2))
    # J1's reducer: its case on the plate, its output up; the motor under the plate
    case, rotor = rv({'x': 0, 'z': 0}, 'J1', 'z', 300.0, 1)
    part(c, 'RV-500N case J1', case)
    blk = mp.box(-565, -500, -40, 40, 470, 560)
    part(c, 'J1 stop', blk)
    part(c, 'Servo motor J1', zcyl(60, 290, 95), 'Aluminum')
    return base, rotor


def build_turret(root, j1_rotor):
    tur = mp.new_occ(root, 'Turret')
    tur.isGroundToParent = False
    c = tur
    # the ring's inner ring, with its groove
    i_ = ztube(ZR + 10.0, ZR + RING['H'], RING['d2'] / 2, 395.0)
    mp.union(i_, ztube(ZR + RING['H'] - 12.0, ZR + RING['H'], 395.0, RING['d'] / 2))
    mp.cut(i_, torus((0, 0, ZR + 28.0), (0, 0, 1), RING['dm'] / 2, RING['ball'] / 2 + 0.4))
    part(c, 'Slewing ring inner ring', i_)
    z0 = ZR + RING['H']                     # 576: the turret's plate on the inner ring
    plate = mp.box(-480, 560, -600, 600, z0, z0 + 60)
    mp.cut(plate, zcyl(z0 - 1, z0 + 61, 100))
    mp.cut(plate, mp.box(220, 420, -130, 130, z0 + 30, z0 + 61))      # the pocket under the balancer's pin
    part(c, 'Turret plate', plate)
    part(c, 'J1 stop dog', mp.box(515, 560, -25, 25, z0 - 48, z0))
    # J1's rotor and the disc coupling
    part(c, 'RV-500N output J1', j1_rotor)
    part(c, 'Coupling disc 1', zcyl(470, 485, 140))
    part(c, 'Coupling spacer', ztube(485, z0 - 15, 60, 45))
    part(c, 'Coupling disc 2', zcyl(z0 - 15, z0, 140))
    # the cheeks
    top = z0 + 60
    ch = mp.box(-200, 520, -600, -520, top, 1180)
    mp.cut(ch, ycyl(AX, AZ, -610, -510, 165))
    part(c, 'Cheek J2 side', ch)
    ch = mp.box(-200, 520, 520, 600, top, 1180)
    mp.cut(ch, ycyl(AX, AZ, 510, 610, 120))
    part(c, 'Cheek J3 side', ch)
    # J2's reducer: case flange on the cheek's outer face, the output inward; the motor outboard
    case, rotor2 = rv({'x': AX, 'z': AZ}, 'J2', 'y', -620.0, 1)
    part(c, 'RV-500N case J2', case)
    part(c, 'Servo motor J2', ycyl(AX, AZ, -850, -620, 95), 'Aluminum')
    # the lower arm's support on the J3 side, and the crank's bearing in the J3 cheek
    part(c, 'J2 support bearing', ytube(AX, AZ, 440, 520, 185, 140.5))
    part(c, 'J3 support bearing', ytube(AX, AZ, 520, 600, 119.5, 90.5))
    # J3's outboard bracket on two posts in front of the shoulder, the reducer's case on it, the motor outside
    part(c, 'J3 bracket', mp.box(150, 640, 840, 870, 820, 1180))
    for k, zp in enumerate((850.0, 1150.0)):
        part(c, 'J3 bracket post %d' % (k + 1), mp.cyl((480, 600, zp), (480, 840, zp), 25))
    case, rotor3 = rv({'x': AX, 'z': AZ}, 'J3', 'y', 840.0, -1)
    part(c, 'RV-500N case J3', case)
    part(c, 'Servo motor J3', ycyl(AX, AZ, 870, 1100, 95), 'Aluminum')
    # the balancer's pin on three lugs in the plate's pocket, and the two gas bottles
    zp = AZ - A_MM
    for k, (y0, y1) in enumerate(((-125, -100), (-15, 15), (100, 125))):
        lug = mp.box(AX - 30, AX + 30, y0, y1, z0 + 30, zp + 30)
        mp.cut(lug, ycyl(AX, zp, y0 - 1, y1 + 1, 15.5))
        part(c, 'Balancer lug %d' % (k + 1), lug)
    part(c, 'Balancer pin P', ycyl(AX, zp, -125, 125, 15))
    for k, (y0, y1) in enumerate(((-300, -20), (20, 300))):
        bt = ytube(-330, top + 55, y0, y1, 55, 50)
        mp.union(bt, ycyl(-330, top + 55, y0, y0 + 5, 50.01), ycyl(-330, top + 55, y1 - 5, y1, 50.01))
        part(c, 'Gas bottle %d' % (k + 1), bt)
    return tur, rotor2, rotor3


def build_lower_arm(tur, rotor2):
    """The lower arm's root at q2 = 0: the fork's two plates, the hub's bosses, the cross pin, the root of the box."""
    la = mp.new_occ(tur.component, 'Lower arm')
    la.isGroundToParent = False
    c = la
    part(c, 'RV-500N output J2', rotor2)
    for sy, name in ((-1, 'Fork plate 1'), (1, 'Fork plate 2')):
        y0, y1 = sorted((sy * 150.0, sy * 190.0))
        pl = mp.box(AX - 110, AX + 110, y0, y1, AZ, AZ + 700)
        mp.union(pl, ycyl(AX, AZ, y0, y1, 160))
        part(c, name, pl)
    part(c, 'Hub boss J2 side', ytube(AX, AZ, -450, -190, 140, 60))
    part(c, 'Hub boss J3 side', ytube(AX, AZ, 190, 500, 140, 60))
    part(c, 'Balancer cross pin Q', ycyl(AX, AZ + B_MM, -150, 150, 15))
    bx = mp.box(AX - 110, AX + 110, -150, 150, AZ + 650, AZ + 1100)
    mp.cut(bx, mp.box(AX - 98, AX + 98, -138, 138, AZ + 640, AZ + 1110))
    part(c, 'Lower arm root box', bx)
    return la


def build_crank(tur, rotor3, cw, r_cw):
    """The crank at q3 = 0 (pointing back, -X): its shaft through the J3 cheek, its web, its pin boss for the
    parallel rod at r = 450 mm, J3's output flange, and the counterweight block on its outer face."""
    cr = mp.new_occ(tur.component, 'Crank')
    cr.isGroundToParent = False
    c = cr
    y0, y1 = Y_CRANK - 30.0, Y_CRANK + 30.0
    part(c, 'Crank shaft', ycyl(AX, AZ, 505, y0, 90))
    web = ycyl(AX, AZ, y0, y1, 150)
    mp.union(web, mp.box(AX - 830, AX, y0, y1, AZ - 150, AZ + 150))
    mp.union(web, ycyl(AX - pd.LAYOUT['r'], AZ, y0, y1, 45))
    part(c, 'Crank web', web)
    part(c, 'RV-500N output J3', rotor3)
    if cw > 0:
        a, h = (v * 1000 for v in pdr.CW_BLOCK)
        tk = pdr.cw_thickness(cw) * 1000.0               # the block's thickness for its mass (mm)
        xc = AX - r_cw
        part(c, 'Counterweight block', mp.box(xc - a / 2, xc + a / 2, y1 + 5, y1 + 5 + tk, AZ - h / 2, AZ + h / 2))
    return cr


def build_balancer(tur):
    """The two cylinders at q2 = 0 (upright): each barrel from the turret's pin, with its eye; each rod with its
    piston and its eye on the fork's cross pin."""
    zp = AZ - A_MM
    zq = AZ + B_MM
    L0 = zq - zp
    out = []
    for k, ys in enumerate((-1, 1)):
        yc = ys * Y_CYL
        bar = mp.new_occ(tur.component, 'Balancer barrel %d' % (k + 1))
        bar.isGroundToParent = False
        b = ycyl(AX, zp, yc - 30, yc + 30, 25)
        mp.union(b, zcyl(zp + 20, zp + BARREL[0] + 10, 20, AX, yc))
        mp.union(b, ztube(zp + BARREL[0], zp + BARREL[1], 37.5, 31.5, AX, yc))
        mp.union(b, zcyl(zp + BARREL[0], zp + BARREL[0] + 10, 31.6, AX, yc))
        top = ztube(zp + BARREL[1] - 10, zp + BARREL[1], 37.5, 16.5, AX, yc)
        mp.union(b, top)
        mp.cut(b, ycyl(AX, zp, yc - 31, yc + 31, 15.5))
        mp.add_body(bar, b, 'Balancer barrel', 'Steel')
        rod = mp.new_occ(tur.component, 'Balancer rod %d' % (k + 1))
        rod.isGroundToParent = False
        sp = L0 - ROD_LEN                               # the piston's face from the turret's pin
        r = ycyl(AX, zq, yc - 30, yc + 30, 25)
        mp.union(r, zcyl(zp + sp, zq - 20, 16, AX, yc))
        mp.union(r, zcyl(zp + sp - 20, zp + sp, 31.0, AX, yc))
        mp.cut(r, ycyl(AX, zq, yc - 31, yc + 31, 15.5))
        mp.add_body(rod, r, 'Balancer rod', 'Steel')
        out.append((bar, rod))
    return out


# ---------- joints ----------
def rotation(occ):
    t = mp.world_matrix(occ)
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def revolute(root, moving, fixed, point, axis, name):
    """An As-built revolute joint through `point` whose turn is about the world axis 'z' or 'y'; the joint's own
    direction is tried until a test turn shows that axis."""
    Jd = adsk.fusion.JointDirections
    k = {'x': 0, 'y': 1, 'z': 2}[axis]
    for direction in (Jd.ZAxisJointDirection, Jd.XAxisJointDirection, Jd.YAxisJointDirection):
        sk = moving.component.sketches.add(moving.component.xYConstructionPlane)
        sk.isVisible = False
        pt = sk.sketchPoints.add(P3(*[v * CM for v in point])).createForAssemblyContext(moving)
        inp = root.asBuiltJoints.createInput(moving, fixed, adsk.fusion.JointGeometry.createByPoint(pt))
        inp.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(inp)
        r0 = rotation(moving)
        j.jointMotion.rotationValue = 0.05
        r1 = rotation(moving)
        j.jointMotion.rotationValue = 0.0
        D = [[sum(r1[a][m] * r0[c][m] for m in range(3)) for c in range(3)] for a in range(3)]
        wv = (D[2][1] - D[1][2], D[0][2] - D[2][0], D[1][0] - D[0][1])
        if abs(wv[k]) > 0.02 and all(abs(wv[m]) < 0.02 for m in range(3) if m != k):
            j.name = name
            return j, (1 if wv[k] > 0 else -1)
        j.deleteMe()
    raise RuntimeError('no joint direction turns %s about %s' % (moving.name, axis))


def set_balancer(cyls, q2_deg, tur=None):
    """Put the barrels and rods where the arm at q2 puts them: the barrel turned about the turret's pin P toward the
    cross pin Q, the rod along the same line with its eye on Q. A part that is neither grounded nor on a joint stays
    where it is when its parent turns on a joint, so the turret's own turn (tur) is added. Returns the pins'
    distance (mm)."""
    zp = AZ - A_MM
    q = math.radians(q2_deg)
    Qx, Qz = AX + B_MM * math.sin(q), AZ + B_MM * math.cos(q)
    L = math.hypot(Qx - AX, Qz - zp)
    th = math.atan2(Qx - AX, Qz - zp)               # the barrel's tilt from upright, about +Y
    L0 = (AZ + B_MM) - zp
    for bar, rod in cyls:
        for occ, slide in ((bar, 0.0), (rod, L - L0)):
            m = adsk.core.Matrix3D.create()
            m.setToRotation(th, V3(0, 1, 0), P3(AX * CM, 0, zp * CM))
            t = adsk.core.Matrix3D.create()
            t.translation = V3(math.sin(th) * slide * CM, 0, math.cos(th) * slide * CM)
            m.transformBy(t)
            if tur is not None:
                m.transformBy(tur.transform2)
            occ.transform2 = m
    return L


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


def build(cw=100.0, r_cw=600.0):
    o = design(cw, r_cw)
    refuse(o)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design_ = adsk.fusion.Design.cast(app.activeProduct)
    design_.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = design_.rootComponent
    base, j1_rotor = build_base(root)
    tur, rotor2, rotor3 = build_turret(root, j1_rotor)
    la = build_lower_arm(tur, rotor2)
    cr = build_crank(tur, rotor3, o['cw'], o['r_cw'])
    cyls = build_balancer(tur)
    # the lower arm and the crank sit inside the turret: a joint in the root takes them in the turret's context
    la_p, cr_p = la.createForAssemblyContext(tur), cr.createForAssemblyContext(tur)
    cyls_p = [(b.createForAssemblyContext(tur), r.createForAssemblyContext(tur)) for b, r in cyls]
    j1, s1 = revolute(root, tur, base, (0, 0, ZR), 'z', 'J1')
    j2, s2 = revolute(root, la_p, tur, (AX, 0, AZ), 'y', 'J2')
    j3, s3 = revolute(root, cr_p, tur, (AX, 0, AZ), 'y', 'J3')
    _last.update(joints={1: (j1, s1), 2: (j2, s2), 3: (j3, s3)}, cyls=cyls_p,
                 occ=dict(base=base, tur=tur, la=la_p, cr=cr_p))
    for oc, col in ((base, 'Paint - Enamel Glossy (Grey)'), (tur, 'Paint - Enamel Glossy (Blue)'),
                    (la, 'Paint - Enamel Glossy (Yellow)'), (cr, 'Paint - Enamel Glossy (Grey)')):
        mp.colour(design_, oc, col)
    app.activeViewport.fit()
    gd, ch = o['gas'], o['choice']
    pl = parts_list(root)
    return ('Palletizer drives built.\nJ1: slewing ring %s; RV-500N, ratio %g.\nJ2: RV-500N with the gas balancer, two '
            'cylinders of %d mm bore, together %.1f to %.1f kN, %.2f to %.2f MPa.\nJ3: RV-500N; counterweight %.0f kg '
            'at %.0f mm.\nLives (h, the reducers, against the hours each joint moves): J1 12 497 of 11 667, J2 120 984 '
            'of 10 667, J3 13 074 of 10 667 (Table 42.4).\n\nParts: %d components; %s.\n\n[%s]'
            % ('RKS.23 0841', ch['i1'], round(G['bore'] * 1000), gd['Fmin'] / 1e3, gd['Fmax'] / 1e3, gd['p0'] / 1e6,
               gd['pmax'] / 1e6, o['cw'], o['r_cw'], sum(pl.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(pl.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('cw', 'Counterweight added to the crank (kg)', 100.0), ('r_cw', 'Its distance behind the shoulder (mm)', 600.0)]


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
        cid = 'PalletizerDrivesCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Palletizer drives', 'The drives of Chapter 42')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('PalletizerDrives could not start:\n' + traceback.format_exc())
