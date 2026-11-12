# =============================================================================
# BagGripper.py  -  Chapter 47 of "Mechanical Design in Autodesk Fusion with Claude": the palletizer's bag gripper.
# The adapter to ISO 9409-1-160-6-M10 of the parallel-jaw gripper (ClampGripper.py, loaded and called) on an
# aluminium box beam with an end block in each end; two end plates with a wall each; two jaws, each a steel box tube
# on stub shafts in 6206 bearings with eight tines, turned by a DSBC-80-125 with a clamping unit on an SNCB swivel
# flange through a lever on the long stub shaft; a press plate on two DSBC-32-50; the bag and the station's rollers.
# Every size is lib/bag_gripper.py's (LAY, DSBC, BRG, CYL, cylinder_geometry(), jaw_moment()).
#
#   INPUTS        the dialog: the bag's mass (kg), its width (mm), the supply pressure (bar)
#   HARD RULES    build() refuses a bag narrower than 280 or wider than 520 mm, a supply under 6 bar, and a bag the
#                 cylinder or its clamping unit does not hold 1.25 times in the worst brake stop
#   CONVENTIONS   z = 0 the mounting face, z up, x along the bag's 800; groups Adapter and Frame (grounded), Jaw A
#                 (+y) and B (-y, A turned half a turn about z), Cylinder A and B, Rod A and B, Press, Bag, Station;
#                 set_state() opens the jaws and moves the press plate by the groups' transforms; no joints
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
import mdfc_parts as mp       # noqa: E402
import bag_gripper as bg      # noqa: E402


def _load(name, script):
    spec = importlib.util.spec_from_file_location(name, CP.script(script))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


CGS = _load('mdfc_clampgripper', 'ClampGripper')        # Chapter 45's script: its adapter is used as it is
importlib.reload(mp)
importlib.reload(bg)

VERSION = 'BagGripper 1.0 (prompt test, 2026-11-11)'
CM = 0.1
V3 = adsk.core.Vector3D.create
P3 = adsk.core.Point3D.create

L = bg.LAY
CYL = bg.CYL
D80 = bg.DSBC[CYL['bore']]
PR = L['press']
D32 = bg.DSBC[PR['bore']]
GEO = bg.cylinder_geometry()
YH, ZH = L['hinge']
PIN_Y, PIN_Z = GEO['pin']                    # the +y jaw's cylinder: its rear pin, closed
XC = L['cyl_x']
PIN_R = D80['EK'] / 2.0 - 0.05               # the pins, 0.05 mm under the bores of EK
M8_PLATE = ((45.0, -44.0), (-45.0, -44.0), (45.0, -88.0), (-45.0, -88.0))     # (y, z) of the end plates' M8
TG32 = 32.5                                  # the DSBC-32's cover screws on a square of 32.5 (ISO 15552)
ROLLER_HALF = 300.0                          # the rollers 600 long, across the bag
N_ROLLERS = 9
W_RANGE = (280.0, 520.0)
P_MIN = 6.0


# ---------- small helpers ----------
def xcyl(y, z, x0, x1, r):
    return mp.cyl((x0, y, z), (x1, y, z), r)


def ycyl(x, z, y0, y1, r):
    return mp.cyl((x, y0, z), (x, y1, z), r)


def zcyl(z0, z1, r, x=0.0, y=0.0):
    return mp.cyl((x, y, z0), (x, y, z1), r)


def _half_turn():
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, V3(0, 0, 1), P3(0, 0, 0))
    return m


def group(root, name, grounded=False):
    o = mp.new_occ(root, name)
    o.isGroundToParent = False
    if grounded:
        o.isGrounded = True
    return o


def part(g, name, tb, material='Steel', turn=False):
    """A part in the group g; turn: the body turned half a turn about z first (the -y side)."""
    if turn:
        mp.tbm().transform(tb, _half_turn())
    o = mp.new_occ(g.component, name)
    mp.add_body(o, tb, name, material)
    return o


def dsbc(bore, stroke, clamping):
    """A DSBC drawn from the catalogue's lengths: the body ZJ - WH + stroke from its rear face to its front face, two
    covers, a bore stroke + piston + 1 mm long; the clamping unit ZW - ZJ in front; the rod's end ZJ (or ZW) + stroke
    from the rear face when retracted. The piston 0.5 mm off each cover at the stroke's ends."""
    d = bg.DSBC[bore]
    tp = 30.0 if bore >= 50 else 20.0
    Lb = d['ZJ'] - d['WH'] + stroke
    c = (d['ZJ'] - d['WH'] - tp - 1.0) / 2.0
    Lk = (d['ZW'] - d['ZJ']) if clamping else 0.0
    end_ret = (d['ZW'] if clamping else d['ZJ']) + stroke
    return dict(d=d, tp=tp, Lb=Lb, c=c, Lk=Lk, end_ret=end_ret, p_ret=c + 0.5, R=bore / 2.0, r=d['rod'] / 2.0,
                h=d['E'] / 2.0, stroke=stroke)


C80 = dsbc(CYL['bore'], CYL['stroke'], True)
C32 = dsbc(PR['bore'], PR['stroke'], False)
FL = D80['FL']
assert abs(FL + C80['end_ret'] + CYL['stroke'] + L['rod_end'] - GEO['L_ext']) < 1e-6     # the lib's pin to pin
Z_BEAM_BOT = L['beam'][1]
E_BAG = Z_BEAM_BOT - C32['end_ret'] - (L['bag_top'] + PR['plate'][2])    # the press's stroke with the plate on the bag


def bearing_6206(x0, y, z):
    """A 6206 on an x axis, its faces at x0 and x0 + B: rings and balls in one part (Volume 1's proportions)."""
    D, B = bg.BRG[int(L['stub_d'])][0], bg.BRG[int(L['stub_d'])][1]
    d = L['stub_d']
    g = mp.bearing_geometry(d, D)
    b = mp.tube((x0, y, z), (x0 + B, y, z), g['in_od'] / 2, d / 2)
    mp.union(b, mp.tube((x0, y, z), (x0 + B, y, z), D / 2, g['out_id'] / 2))
    for k in range(g['Z']):
        a = 2 * math.pi * k / g['Z']
        mp.union(b, mp.sphere((x0 + B / 2, y + g['dm'] / 2 * math.cos(a), z + g['dm'] / 2 * math.sin(a)), g['Dw'] / 2))
    return b


# ---------- the frame ----------
def frame(root):
    g = group(root, 'Frame', True)
    zt, zb = L['beam']
    bx, by, t = L['beam_x'], L['beam_y'], L['beam_t']
    b = mp.box(-bx, bx, -by, by, zb, zt)
    mp.cut(b, mp.box(-bx - 1, bx + 1, -by + t, by - t, zb + t, zt - t))
    for x, y in CGS.M8:
        mp.cut(b, zcyl(zt - t - 1, zt + 1, 4.5, x, y))                  # the adapter's M8 through the top
        mp.cut(b, zcyl(zb - 1, zb + t + 1, 10.0, x, y))                 # a key's way in through the bottom
    for xp in PR['x']:
        for sx in (1, -1):
            for sy in (1, -1):
                mp.cut(b, zcyl(zb - 1, zb + t + 1, 3.3, xp + sx * TG32 / 2, sy * TG32 / 2))   # the press cylinders' M6
    part(g, 'Beam', b, 'Aluminum')
    for k, (x, y) in enumerate(CGS.M8):
        part(g, 'Cap screw M8x20 %d' % (k + 1), mp.cap_screw(8.0, 20.0, (x, y, zt - t), (0, 0, 1)))
    n8 = 0
    for k, s in enumerate((1, -1)):
        # the end block, tapped for the end plate's four M8 (drawn at 8)
        xa, xb = sorted((s * (bx - L['block']), s * bx))
        blk = mp.box(xa, xb, -by + t, by - t, zb + t, zt - t)
        for y, z in M8_PLATE:
            mp.cut(blk, xcyl(y, z, s * (bx - 21.0), s * (bx + 1.0), 4.0))
        part(g, 'End block %d' % (k + 1), blk, 'Aluminum')
        part(g, 'End plate %d' % (k + 1), end_plate(s), 'Aluminum')
        for y, z in M8_PLATE:
            n8 += 1
            part(g, 'Cap screw M8x30 %d' % n8, mp.cap_screw(8.0, 30.0, (s * L['plate_x'][1], y, z), (-s, 0, 0)))
    nb = 0
    for s in (1, -1):
        for yh in (YH, -YH):
            nb += 1
            x0 = L['bearing_x'][0] if s > 0 else -L['bearing_x'][1]
            part(g, 'Bearing 6206 %d' % nb, bearing_6206(x0, yh, ZH))
    for k, side in enumerate(('A', 'B')):
        turn = side == 'B'
        part(g, 'Bracket %d' % (k + 1), bracket(), 'Steel', turn)
        part(g, 'Pin 16 %d' % (k + 1), xcyl(PIN_Y, PIN_Z, XC - 30.0, XC + 31.0, PIN_R), 'Steel', turn)
    n6 = 0
    for k, xp in enumerate(PR['x']):
        part(g, 'Cylinder DSBC-32-50 %d' % (k + 1), press_cylinder(xp), 'Aluminum')
        for sx in (1, -1):
            for sy in (1, -1):
                n6 += 1
                part(g, 'Cap screw M6x16 %d' % n6,
                     mp.cap_screw(6.0, 16.0, (xp + sx * TG32 / 2, sy * TG32 / 2, zb + t), (0, 0, -1)))
    return g


def end_plate(s):
    """The end plate at x = s x (410 to 425): 720 wide down to -245, the wall 400 wide down to -380, a boss for each
    jaw's bearing on its outer face, the stub's hole, the bearing's seat and four M8 clearance holes."""
    x0, x1 = L['plate_x']
    xa, xb = sorted((s * x0, s * x1))
    zt, zp = L['plate_z']
    p = mp.box(xa, xb, -L['plate_y'], L['plate_y'], zp, zt)
    mp.union(p, mp.box(xa, xb, -L['wall_y'], L['wall_y'], L['wall_z'], zp + 1.0))
    b0, b1 = L['boss']
    D = bg.BRG[int(L['stub_d'])][0]
    for yh in (YH, -YH):
        mp.union(p, xcyl(yh, ZH, s * (b0 - 1.0), s * b1, L['boss_r']))
    for yh in (YH, -YH):
        mp.cut(p, xcyl(yh, ZH, s * (x0 - 1.0), s * L['bearing_x'][0], L['stub_d'] / 2 + 5.0))
        mp.cut(p, xcyl(yh, ZH, s * L['bearing_x'][0], s * (b1 + 1.0), D / 2))
    for y, z in M8_PLATE:
        mp.cut(p, xcyl(y, z, s * (x0 - 1.0), s * (x1 + 1.0), 4.5))
    return p


def bracket():
    """The +y jaw's cylinder bracket on the +x end plate: a block from the plate to the inner cheek, an outer cheek
    and a yoke behind the pin joining them; the pin's bore EK through both cheeks."""
    x_in, x_out = XC - 16.0, XC + 16.0                  # the swivel flange's lug is 30 wide, 1 mm clear each side
    y0, y1 = PIN_Y - 35.0, PIN_Y + 15.0
    z0, z1 = PIN_Z - 25.0, PIN_Z + 25.0
    b = mp.box(L['plate_x'][1], x_in, y0, y1, z0, z1)
    mp.union(b, mp.box(x_out, x_out + 15.0, y0, y1, z0, z1))
    mp.union(b, mp.box(L['plate_x'][1], x_out + 15.0, y0, PIN_Y - 23.5, z0, z1))
    mp.cut(b, xcyl(PIN_Y, PIN_Z, x_in - 15.0, x_out + 16.0, D80['EK'] / 2.0))
    return b


def press_cylinder(x):
    """A DSBC-32-50 hanging under the beam, its rear face on the beam's underside, its rod downward."""
    c = C32
    z0 = Z_BEAM_BOT
    b = mp.box(x - c['h'], x + c['h'], -c['h'], c['h'], z0 - c['Lb'], z0)
    mp.cut(b, zcyl(z0 - c['Lb'] + c['c'], z0 - c['c'], c['R'], x, 0.0))
    mp.cut(b, zcyl(z0 - c['Lb'] - 1.0, z0 - c['Lb'] + c['c'] + 1.0, c['r'] + 0.5, x, 0.0))
    for sx in (1, -1):
        for sy in (1, -1):
            mp.cut(b, zcyl(z0 - 12.0, z0 + 1.0, 3.0, x + sx * TG32 / 2, sy * TG32 / 2))     # tapped M6 (drawn at 6)
    return b


# ---------- the jaws, their cylinders and rods (built for the +y side, the -y side turned) ----------
def jaw(root, side):
    g = group(root, 'Jaw ' + side)
    turn = side == 'B'
    h, t, X = L['tube'] / 2.0, L['tube_t'], L['tube_x']
    tb = mp.box(-X, X, YH - h, YH + h, ZH - h, ZH + h)
    mp.cut(tb, mp.box(-X - 1, X + 1, YH - h + t, YH + h - t, ZH - h + t, ZH + h - t))
    part(g, 'Box tube 60x4 ' + side, tb, 'Steel', turn)
    rs = L['stub_d'] / 2.0

    def plug(x0, x1):
        return mp.box(x0, x1, YH - h + t, YH + h - t, ZH - h + t, ZH + h - t)
    lx0, lx1 = L['lever_x']
    part(g, 'Long stub shaft ' + side, mp.union(plug(X - 8.0, X), xcyl(YH, ZH, X - 8.0, lx1, rs)), 'Steel', turn)
    part(g, 'Short stub shaft ' + side,
         mp.union(plug(-X, -X + 8.0), xcyl(YH, ZH, -(L['bearing_x'][1] + 2.0), -X + 8.0, rs)), 'Steel', turn)
    r = L['lever']
    lev = xcyl(YH, ZH, lx0, lx1, 25.0)
    mp.union(lev, mp.box(lx0, lx1, YH - 20.0, YH + 20.0, ZH, ZH + r), xcyl(YH, ZH + r, lx0, lx1, 20.0),
             xcyl(YH, ZH + r, lx1 - 1.0, XC + 10.0, PIN_R))          # the pin through the rod end's eye, 20 thick
    mp.cut(lev, xcyl(YH, ZH, lx0 - 1.0, lx1 + 1.0, rs))
    part(g, 'Lever ' + side, lev, 'Steel', turn)
    y0, y1 = L['leg_y']
    zf0, zf1 = L['foot_z']
    tt = L['tine_t'] / 2.0
    for k, x in enumerate(L['tine_x']):
        tn = mp.box(x - tt, x + tt, y0, y1, zf0, L['leg_top'])
        mp.union(tn, mp.box(x - tt, x + tt, L['tip_y'], y1, zf0, zf1))
        part(g, 'Tine %d' % (k + 1 + (len(L['tine_x']) if turn else 0)), tn, 'Steel', turn)
    return g


def jaw_cylinder(root, side):
    """The DSBC-80-125 with its clamping unit on an SNCB swivel flange, closed at full stroke: its axis along +y at
    x = cyl_x, z of the rear pin, from the rear pin to the lever's pin."""
    g = group(root, 'Cylinder ' + side)
    turn = side == 'B'
    c = C80
    hh = c['h']
    s0 = PIN_Y + FL                                     # the cylinder's rear face
    lug = mp.box(XC - 15.0, XC + 15.0, PIN_Y, s0 - 9.0, PIN_Z - 22.0, PIN_Z + 22.0)
    mp.union(lug, xcyl(PIN_Y, PIN_Z, XC - 15.0, XC + 15.0, 22.0),
             mp.box(XC - hh, XC + hh, s0 - 10.0, s0, PIN_Z - hh, PIN_Z + hh))
    mp.cut(lug, xcyl(PIN_Y, PIN_Z, XC - 16.0, XC + 16.0, D80['EK'] / 2.0))
    part(g, 'Swivel flange SNCB-80 ' + side, lug, 'Aluminum', turn)
    b = mp.box(XC - hh, XC + hh, s0, s0 + c['Lb'], PIN_Z - hh, PIN_Z + hh)
    mp.cut(b, ycyl(XC, PIN_Z, s0 + c['c'], s0 + c['Lb'] - c['c'], c['R']))
    mp.cut(b, ycyl(XC, PIN_Z, s0 + c['Lb'] - c['c'] - 1.0, s0 + c['Lb'] + 1.0, c['r'] + 0.5))
    part(g, 'Cylinder DSBC-80-125 ' + side, b, 'Aluminum', turn)
    k = mp.box(XC - hh, XC + hh, s0 + c['Lb'], s0 + c['Lb'] + c['Lk'], PIN_Z - hh, PIN_Z + hh)
    mp.cut(k, ycyl(XC, PIN_Z, s0 + c['Lb'] - 1.0, s0 + c['Lb'] + c['Lk'] + 1.0, c['r'] + 0.5))
    part(g, 'Clamping unit ' + side, k, 'Aluminum', turn)
    return g


def jaw_rod(root, side):
    """The piston and rod at full stroke and the rod end: a block 20 x 25 from the rod's end to the eye, the eye 40
    across and 20 thick on the lever's pin."""
    g = group(root, 'Rod ' + side)
    turn = side == 'B'
    c = C80
    s0 = PIN_Y + FL
    sp = s0 + c['p_ret'] + c['stroke']
    s_end = s0 + c['end_ret'] + c['stroke']
    pr = ycyl(XC, PIN_Z, sp, sp + c['tp'], c['R'] - 0.5)
    mp.union(pr, ycyl(XC, PIN_Z, sp + c['tp'] / 2, s_end, c['r']))
    part(g, 'Piston rod ' + side, pr, 'Steel', turn)
    y_pin = PIN_Y + GEO['L_ext']
    re = mp.box(XC - 10.0, XC + 10.0, s_end, y_pin, PIN_Z - c['r'], PIN_Z + c['r'])
    mp.union(re, xcyl(y_pin, PIN_Z, XC - 10.0, XC + 10.0, 20.0))
    mp.cut(re, xcyl(y_pin, PIN_Z, XC - 11.0, XC + 11.0, D80['EK'] / 2.0))
    part(g, 'Rod end ' + side, re, 'Steel', turn)
    return g


# ---------- the press, the bag and the station ----------
def press(root):
    g = group(root, 'Press')
    px, py, pt = PR['plate']
    zt = L['bag_top'] + pt
    part(g, 'Press plate', mp.box(-px / 2, px / 2, -py / 2, py / 2, L['bag_top'], zt), 'Aluminum')
    c = C32
    for k, x in enumerate(PR['x']):
        zp = Z_BEAM_BOT - c['p_ret'] - E_BAG                 # the piston's top
        r = zcyl(zp - c['tp'], zp, c['R'] - 0.5, x, 0.0)
        mp.union(r, zcyl(zt, zp - c['tp'] / 2, c['r'], x, 0.0))
        part(g, 'Piston rod DSBC-32 %d' % (k + 1), r, 'Steel')
    return g


def bag(root, w):
    g = group(root, 'Bag')
    part(g, 'Bag body', mp.box(-bg.BAG['l'] / 2, bg.BAG['l'] / 2, -w / 2, w / 2, L['bag_bottom'], L['bag_top']),
         'Paper')
    return g


def station(root):
    g = group(root, 'Station')
    rr = bg.PICK['roller_d'] / 2
    for k in range(N_ROLLERS):
        x = (k - (N_ROLLERS - 1) / 2.0) * bg.PICK['pitch']
        part(g, 'Roller %d' % (k + 1), ycyl(x, L['bag_bottom'] - rr, -ROLLER_HALF, ROLLER_HALF, rr), 'Steel')
    return g


# ---------- the states ----------
def _rot_x(a, y0, z0):
    c, s = math.cos(a), math.sin(a)
    R = [[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]]
    return R, [0.0, y0 - (c * y0 - s * z0), z0 - (s * y0 + c * z0)]


def _turned(R, t):
    """The same motion for the -y side: conjugated by the half turn about z."""
    Z = (-1.0, -1.0, 1.0)
    return [[Z[i] * R[i][j] * Z[j] for j in range(3)] for i in range(3)], [Z[i] * t[i] for i in range(3)]


def _place(occ, R, t):
    m = adsk.core.Matrix3D.create()
    for i in range(3):
        for j in range(3):
            m.setCell(i, j, R[i][j])
        m.setCell(i, 3, t[i] * CM)
    occ.transform2 = m


def lengths(phi):
    """The +y jaw opened by phi (deg): the lever's tip, the cylinder's angle and its pin-to-pin length."""
    ph = math.radians(phi)
    r = L['lever']
    tip = (YH - r * math.sin(ph), ZH + r * math.cos(ph))
    Lp = math.hypot(tip[0] - PIN_Y, tip[1] - PIN_Z)
    th = math.atan2(tip[1] - PIN_Z, tip[0] - PIN_Y)
    return tip, th, Lp


def set_state(G_, phi=0.0, e=None):
    """Both jaws opened by phi (deg) about their hinges, each cylinder turned about its rear pin onto its lever and
    its rod run out to the lever's pin; the press plate at the extension e (mm, 0 retracted; None: on the bag)."""
    if e is None:
        e = E_BAG
    ph = math.radians(phi)
    tip, th, Lp = lengths(phi)
    RA, tA = _rot_x(ph, YH, ZH)
    RC, tC = _rot_x(th, PIN_Y, PIN_Z)
    d = Lp - GEO['L_ext']
    tR = [tC[0], tC[1] + d * math.cos(th), tC[2] + d * math.sin(th)]
    for side, turn in (('A', False), ('B', True)):
        for name, R, t in (('Jaw', RA, tA), ('Cylinder', RC, tC), ('Rod', RC, tR)):
            RR, tt = _turned(R, t) if turn else (R, t)
            _place(G_['%s %s' % (name, side)], RR, tt)
    I3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    _place(G_['Press'], I3, [0.0, 0.0, -(e - E_BAG)])
    adsk.doEvents()
    return dict(phi=phi, e=e, L=Lp, theta=math.degrees(th), tip=tip)


# ---------- the rules ----------
def holding(m, w):
    """The jaw's opening moment in every brake stop at this payload (the changer, the gripper as weighed and the bag),
    the worst, and the factors of the cylinder's advancing force and its clamping unit's hold on the lever."""
    b = dict(bg.BAG)
    b['m'], b['w'] = m, w
    m_grip = bg.design()['m_gripper']
    pl = bg.cg.M_CHANGER + m_grip + m
    es = bg.tc.estop_loads(decel=bg.stop_decels(pl))
    rows = []
    for c in es['cases']:
        a = c['a']
        av, ah = a[2] + bg.G, math.hypot(a[0], a[1])
        rows.append(dict(case=c, a_v=av, a_h=ah, jm=bg.jaw_moment(max(av, 0.0), ah, bag=b)))
    wst = max(rows, key=lambda r: r['jm']['M'])
    M = wst['jm']['M']
    arm = L['lever'] / 1000.0
    return dict(m_grip=m_grip, payload=pl, worst=wst, M=M, S_adv=D80['F_adv'] * arm / M, S_hold=D80['F_hold'] * arm / M,
                n=len(rows))


def refuse(m, w, p_bar):
    if w < W_RANGE[0] or w > W_RANGE[1]:
        raise ValueError('A bag %.0f mm wide: the tines take bags from %.0f to %.0f mm wide.' % (w, W_RANGE[0], W_RANGE[1]))
    if p_bar < P_MIN:
        raise ValueError('A supply of %.1f bar: the cylinders and their forces are sized at 6 bar; the gripper needs at '
                         'least 6 bar.' % p_bar)
    h = holding(m, w)
    if min(h['S_adv'], h['S_hold']) < bg.S_STOP:
        c = h['worst']['case']
        raise ValueError('A bag of %.0f kg: in the worst stop (%s, J%s %s) the jaw\'s opening moment is %.1f N m; the '
                         'cylinder holds it %.2f times and its clamping unit %.2f times, under the %.2f needed.'
                         % (m, c['pose'], c['joint'], '+' if c['sense'] > 0 else '-', h['M'], h['S_adv'], h['S_hold'],
                            bg.S_STOP))
    return h


def parts_list(root):
    count = {}
    for o in root.allOccurrences:
        if o.component.bRepBodies.count == 0 or o.fullPathName.startswith('Bag') or o.fullPathName.startswith('Station'):
            continue
        nm = o.component.name
        if nm.endswith(')') and ' (' in nm:
            nm = nm[:nm.rindex(' (')]
        wds = nm.split()
        if wds[-1].isdigit() or wds[-1] in ('A', 'B'):
            wds = wds[:-1]
        n = ' '.join(wds)
        count[n] = count.get(n, 0) + 1
    return count


_last = {}


def build(m=50.0, w=500.0, p_bar=6.0):
    h = refuse(m, w, p_bar)
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    des = adsk.fusion.Design.cast(app.activeProduct)
    des.designType = adsk.fusion.DesignTypes.ParametricDesignType
    root = des.rootComponent
    G_ = {'Adapter': CGS.adapter(root), 'Frame': frame(root)}
    for s in ('A', 'B'):
        G_['Jaw ' + s] = jaw(root, s)
        G_['Cylinder ' + s] = jaw_cylinder(root, s)
        G_['Rod ' + s] = jaw_rod(root, s)
    G_['Press'] = press(root)
    G_['Bag'] = bag(root, w)
    G_['Station'] = station(root)
    for k, col in (('Adapter', 'Paint - Enamel Glossy (Blue)'), ('Frame', 'Paint - Enamel Glossy (Grey)'),
                   ('Jaw A', 'Paint - Enamel Glossy (Yellow)'), ('Jaw B', 'Paint - Enamel Glossy (Yellow)'),
                   ('Press', 'Paint - Enamel Glossy (Red)')):
        mp.colour(des, G_[k], col)
    st = set_state(G_, 0.0)
    _last.update(groups=G_, m=m, w=w, p_bar=p_bar, holding=h)
    app.activeViewport.fit()
    n = parts_list(root)
    c = h['worst']['case']
    jm = h['worst']['jm']
    _, _, L_open = lengths(L['open'])
    return ('Bag gripper built, closed on a bag of %.0f kg, %.0f x %.0f x %.0f mm, on the station\'s %d rollers %.0f mm '
            'across at %.0f mm.\nSupply %.1f bar. Payload in the stops %.1f kg (changer %.1f, gripper %.1f, bag %.0f); '
            '%d stops worked.\nWorst stop for the jaws: %s, J%s %s, a_v %.2f and a_h %.2f m/s2: the jaw\'s opening '
            'moment %.1f N m (the bag\'s weight on the tines %.1f, its sideways push on the legs %.1f, the tines\' own '
            '%.1f).\nDSBC-80-125 on a lever of %.0f mm: it holds that %.2f times (%.0f N at 6 bar), its clamping unit '
            '%.2f times (%.0f N); %.2f needed.\nCylinders %.2f mm pin to pin closed, %.2f mm opened %.0f deg; the press '
            'plate on two DSBC-32-50, on the bag at %.1f mm of stroke.\n\nParts: %d components; %s.\n\n[%s]'
            % (m, bg.BAG['l'], w, bg.BAG['h'], N_ROLLERS, bg.PICK['roller_d'], bg.PICK['pitch'], p_bar, h['payload'],
               bg.cg.M_CHANGER, h['m_grip'], m, h['n'], c['pose'], c['joint'], '+' if c['sense'] > 0 else '-',
               h['worst']['a_v'], h['worst']['a_h'], h['M'], jm['Mv'], jm['Mh'], jm['Mt'], L['lever'], h['S_adv'],
               D80['F_adv'], h['S_hold'], D80['F_hold'], bg.S_STOP, st['L'], L_open, L['open'], E_BAG,
               sum(n.values()), ', '.join('%d %s' % (v, k) for k, v in sorted(n.items())), VERSION))


# ---------- the command dialog ----------
_handlers = []
FIELDS = [('m', 'The bag\'s mass (kg)', 50.0), ('w', 'The bag\'s width (mm)', 500.0),
          ('p_bar', 'The supply pressure (bar)', 6.0)]


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            vals = {k: ins.itemById(k).value for k, _, _ in FIELDS}
            ui.messageBox(build(**vals), VERSION)
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
        cid = 'BagGripperCmd'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Bag gripper', 'The palletizer\'s bag gripper of Chapter 47')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('BagGripper could not start:\n' + traceback.format_exc())
