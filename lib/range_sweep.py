"""Chapter 56: interference and motion over the whole range. The states each detailed machine is stepped through,
from its own limits: the wrist's pitch to its hard stop and its roll round a turn, the flap over its deflection, the
axle's pinion round a turn straight and in a turn, the robot round its working range's outline, the grippers, the
changer and the compliance unit over their strokes. The sweep itself runs in Fusion (scripts/RangeSweep).

Sources: lib/wrist_design.py (the pitch's software limit 120 deg, the switch 121, the hard stop's contact 128),
lib/flap_design.py (delta_max 40 deg), lib/palletizer_design.py (the working range's polygon), lib/bag_gripper.py
(the jaws' opening), lib/compliance_unit.py and lib/vacuum_gripper.py (the strokes), Chapter 45's check (the clamp's
jaws from 200 to 650 mm); for the bag gripper's two ends of travel, Festo SE & Co. KG (2025), *Shock absorber
DYSC*, pp. 4, 6, 7 and 8 (Web pages/Festo_DYSC_shock_absorbers_2026-11-03.md), and Chapter 47's jaw (lib/bag_gripper.py).
"""
import math

import bag_gripper as BG
import compliance_unit as CU
import flap_design as FD
import palletizer_design as PD
import vacuum_gripper as VG
import wrist_design as WD

GAP = 10.0          # mm: pairs whose boxes come within this are measured
STEADY = 0.05       # mm: a pair whose distance changes by less is a fixed gap or a designed contact, not reported


def wrist():
    """Pitch from the hard stop's contact on one side to the other's at roll 0, every 8 deg, with the software limit
    and the switch; roll round a turn every 30 deg at the pitch's two limits and level."""
    c, sw, soft = WD.STOP40['contact'], WD.STOP40['switch'], WD.STOP40['soft']
    ps = sorted(set([-c, -sw, -soft] + [float(p) for p in range(-120, 121, 8)] + [soft, sw, c]))
    out = [('pitch', (p, 0.0)) for p in ps]
    out += [('roll', (p, float(r))) for p in (-soft, 0.0, soft) for r in range(0, 360, 30)]
    return out


def flap():
    return [('deflection', float(d)) for d in range(0, int(FD.brief()['delta_max']) + 1, 2)]


def axle():
    """The pinion round a turn every 15 deg, straight; then the same with the differential's spider turned 2.5 deg in
    the case, a turn's differential motion."""
    return [('straight', (float(a), 0.0)) for a in range(0, 360, 15)] + \
           [('turning', (float(a), 2.5)) for a in range(0, 360, 30)]


def robot():
    """The working range's outline: each corner of Chapter 41's polygon and three points along each edge, and the
    full reach."""
    poly = PD.range_polygon()
    out = []
    for i in range(len(poly)):
        (a, b), (c, d) = poly[i], poly[(i + 1) % len(poly)]
        for k in range(4):
            t = k / 4.0
            out.append(('outline', (round(a + (c - a) * t, 3), round(b + (d - b) * t, 3))))
    ws = PD.workspace()
    out.append(('reach', (round(ws['reach_q2'], 3), round(ws['reach_q3'], 3))))
    return out


def bag():
    """The jaws from closed to the open stop, 45 deg, every 3 deg."""
    o = BG.LAY['open']
    return [('jaws', float(p)) for p in sorted(set(list(range(0, int(o) + 1, 3)) + [o]))]


def open_end():
    """The angle at which the jaw cylinder reaches its retracted length, the jaw's end of travel with no stop: the
    lever's pin on its circle about the hinge, the cylinder from its rear pin, by bisection."""
    g = BG.cylinder_geometry()
    yh, zh = BG.LAY['hinge']
    r = BG.LAY['lever']
    py, pz = g['pin']
    L = lambda ph: math.hypot(yh - r * math.sin(math.radians(ph)) - py, zh + r * math.cos(math.radians(ph)) - pz)
    lo, hi = BG.LAY['open'], 90.0
    for _ in range(60):
        mid = (lo + hi) / 2.0
        lo, hi = (mid, hi) if L(mid) > g['L_ret'] else (lo, mid)
    return dict(phi=(lo + hi) / 2.0, L_ret=g['L_ret'])


def bag_free():
    """The jaws as Chapter 47 left them, with nothing to stop them open: from closed to the cylinder's retracted end,
    every 3 deg."""
    e = round(open_end()['phi'], 1)
    return [('jaws', float(p)) for p in list(range(0, int(e) + 1, 3)) + [e]]


def clamp():
    return [('jaws', float(j)) for j in range(200, 651, 25)]


def vacuum():
    s = VG.LAY['stroke']
    return [('cups', float(z)) for z in range(0, int(s) + 1, 5)]


def changer():
    return [('lock', (True, 0.0)), ('lock', (False, 0.0)), ('lock', (False, 5.0)), ('lock', (False, 10.0))]


def unit():
    s = CU.LAY['stroke']
    return [('float', float(z)) for z in range(0, int(s) + 1, 3)]


STATES = dict(wrist=wrist, flap=flap, axle=axle, robot=robot, bag0=bag_free, bag=bag, clamp=clamp, vacuum=vacuum,
              changer=changer, unit=unit)


# ---- the bag gripper's two ends of travel ---------------------------------------------------------------------------
# Festo's DYSC (2025, p. 4 the data, p. 6 the chart of impact speed against mass for the 20 and the 25, p. 7 the
# dimensions, p. 8 the part numbers): W per stroke and per hour, the stop force, the load range, the chart's highest
# force curve and the least speed that curve allows anywhere in the load range (read off p. 6 at its right end).
DYSC = {
    12: dict(type='DYSC-12-12-Y1F', part=548014, s=12.0, W=10.0, W_h=36000.0, F_stop=1000.0, m=45.0, kg=0.081,
             D1=16.0, P=1.0, D2=11.0, D3=14.5, L1=82.5, L2=69.5, B1=5.0, AF=19.0, F_chart=500.0, v_chart=None),
    16: dict(type='DYSC-16-18-Y1F', part=553593, s=18.0, W=25.0, W_h=50000.0, F_stop=2000.0, m=70.0, kg=0.210,
             D1=22.0, P=1.5, D2=15.0, D3=19.6, L1=110.0, L2=93.0, B1=6.0, AF=27.0, F_chart=800.0, v_chart=None),
    20: dict(type='DYSC-20-18-Y1F', part=2479149, s=18.0, W=38.0, W_h=80000.0, F_stop=3000.0, m=100.0, kg=0.370,
             D1=26.0, P=1.5, D2=18.8, D3=23.8, L1=122.0, L2=105.0, B1=8.0, AF=32.0, F_chart=1200.0, v_chart=0.6),
    25: dict(type='DYSC-25-25-Y1F', part=2480234, s=25.0, W=100.0, W_h=140000.0, F_stop=4000.0, m=160.0, kg=0.575,
             D1=30.0, P=1.5, D2=22.8, D3=27.8, L1=165.0, L2=137.0, B1=10.0, AF=36.0, F_chart=2500.0, v_chart=0.75)}
V_MAX = 2.0                      # m/s, the lower of the two printed (p. 4)
CYCLES_H = 600.0                 # Chapter 41's 6 s cycle: each jaw closes once and opens once a cycle
# the lever carried on past its pin as a striker for the closing stop; the opening stop on the lever's bar
STRIKER = dict(r=(150.0, 200.0), x=(480.0, 495.0), contact=190.0)
AXIS_X = 485.0
OPEN_CONTACT = 120.0             # mm from the hinge, on the bar's leading face at the open stop
BRACKET = dict(foot=12.0, bolt=8.0, At=36.6, Sp=580.0, mu=0.2, washer=(8.4, 16.0, 1.6), nut_m=6.8, head=(13.0, 8.0))


def _end(name, I, omega, v_rod, m_mov, T, r_a, sizes):
    """One end of a jaw's travel against Festo's data: the jaw arrives turning at omega with the piston and rod at
    v_rod, the cylinder still pushing with torque T, the absorber at r_a from the hinge. Festo's terms: W1 kinetic,
    W2 the propelling force over the stroke, W3 their sum per stroke, W4 an hour; the effective mass J/r^2 and the
    impact speed at the absorber."""
    W1 = 0.5 * I * omega ** 2 + 0.5 * m_mov * v_rod ** 2
    F = T / (r_a / 1000.0)
    v = omega * r_a / 1000.0
    m_e = 2.0 * W1 / v ** 2
    rows = []
    for k in sorted(sizes):
        d = DYSC[k]
        W2 = F * d['s'] / 1000.0
        W3 = W1 + W2
        why = []
        if W3 > d['W']:
            why.append('%.1f J a stroke over its %.0f' % (W3, d['W']))
        if W3 * CYCLES_H > d['W_h']:
            why.append('%.0f J an hour over its %.0f' % (W3 * CYCLES_H, d['W_h']))
        if F > d['F_stop']:
            why.append('%.0f N over its stop force %.0f' % (F, d['F_stop']))
        if F > d['F_chart']:
            why.append('%.0f N above its chart\'s highest curve, %.0f' % (F, d['F_chart']))
        if m_e > d['m']:
            why.append('%.1f kg over its load range %.0f' % (m_e, d['m']))
        if v > V_MAX or (d['v_chart'] and v > d['v_chart']):
            why.append('%.2f m/s over what its chart allows' % v)
        rows.append(dict(size=k, type=d['type'], W2=W2, W3=W3, W4=W3 * CYCLES_H, ok=not why, why=why,
                         use_W=W3 / d['W'], use_F=F / d['F_chart']))
    pick = next(r for r in rows if r['ok'])
    return dict(name=name, I=I, omega=omega, v_rod=v_rod, T=T, r_a=r_a, F=F, v=v, m_e=m_e, W1=W1, rows=rows, pick=pick)


def jaw_stops():
    """The bag gripper's jaws stopped at both ends by a DYSC each, so that the cylinders' own cushioning (1.8 J,
    Chapter 47) no longer limits their speed: closed, the striker on the lever at 190 mm against a DYSC on the
    +y side, the cylinder pushing it with its full advancing force on the 150 mm arm; open at 45 deg, the bar's
    leading face at 120 mm against one along the bar's motion, the cylinder pulling with its retracting force on its
    arm there. Each stroke of the jaws in Chapter 47's 0.25 s, so the rod at the average speed 0.416 m/s; the jaw's
    inertia from Fusion with the striker added."""
    d = BG.design()
    imp = d['impact']
    g = BG.cylinder_geometry()
    L = BG.LAY
    cyl = BG.DSBC[BG.CYL['bore']]
    r0, r1 = STRIKER['r']
    w = 40.0
    t = STRIKER['x'][1] - STRIKER['x'][0]
    m_s = 7850e-9 * (r1 - r0) * w * t
    rc = (r0 + r1) / 2.0 / 1000.0
    I_s = m_s * (rc ** 2 + ((r1 - r0) ** 2 + w ** 2) / 12e6)
    I_jaw = imp['I'] + I_s
    v = imp['v_avg']
    arm_c, arm_o = g['arm_closed'], g['arm_open']
    closed = _end('closed', I_jaw, v / arm_c, v, imp['m_mov'], cyl['F_adv'] * arm_c, STRIKER['contact'], DYSC)
    opened = _end('open', I_jaw, v / arm_o, v, imp['m_mov'], cyl['F_ret'] * arm_o, OPEN_CONTACT, DYSC)
    # no stop: the jaw runs on to the cylinder's retracted end, where its arm is small; at the same rod speed the jaw
    # turns fast and the cylinder's cushioning meets the jaw's inertia over the arm squared
    e = open_end()
    ph = math.radians(e['phi'])
    yh, zh = L['hinge']
    r = L['lever']
    py, pz = g['pin']
    tip = (yh - r * math.sin(ph), zh + r * math.cos(ph))
    Lc = math.hypot(tip[0] - py, tip[1] - pz)
    u = ((tip[0] - py) / Lc, (tip[1] - pz) / Lc)
    lev = (-math.sin(ph), math.cos(ph))
    arm_e = r * abs(u[0] * lev[1] - u[1] * lev[0]) / 1000.0
    m_rod = imp['I'] / arm_e ** 2 + imp['m_mov']
    E_end = 0.5 * imp['I'] * (v / arm_e) ** 2 + 0.5 * imp['m_mov'] * v ** 2
    free = dict(phi=e['phi'], arm=arm_e, m_rod=m_rod, E=E_end, E_cush=BG.DSBC[80]['E_imp'], ratio=E_end / BG.DSBC[80]['E_imp'])
    rate = 3600.0 / 6.0
    return dict(closed=closed, open=opened, free=free, I_jaw=I_jaw, I_striker=I_s, m_striker=m_s, v=v,
                rate_before=imp['rate'], rate=rate, t=g['used'] / 1000.0 / v)


def stop_layout():
    """Where the stops sit, for jaw A at the +x end (jaw B's are the same turned half a turn about z), in mm: the
    absorbers' axes at x = 485, clear of the cylinder's rod (from 497.5) and its rod end (from 500); the closing one
    along +y from its contact on the striker, the opening one along the bar's motion at 45 deg from its contact on
    the leading face; each drawn in its end position, its front face at the contact. Each bracket: a foot 12 thick on
    the end plate's outer face, two M8 through the plate, a lug round the absorber's thread; the closing bracket's lip
    over the plate's +y end, which takes the stop's push."""
    L = BG.LAY
    yh, zh = L['hinge']
    face = L['plate_x'][1]
    ax = AXIS_X
    ft = BRACKET['foot']
    c_face = (yh + 20.0, zh + STRIKER['contact'])
    closed = dict(face=c_face, axis=(1.0, 0.0), x=ax,
                  foot=dict(x=(face, face + ft), y=(296.0, L['plate_y']), z=(-72.0, 16.0)),
                  lip=dict(x=(L['plate_x'][0], face), y=(L['plate_y'], L['plate_y'] + 12.0), z=(-72.0, -16.0)),
                  lug=dict(x=(face + ft, 509.0), y=(335.0, L['plate_y']), z=(c_face[1] - 24.0, c_face[1] + 24.0)),
                  bolts=[(312.0, -30.0), (312.0, -58.0)], pivot=(face, L['plate_y']), nut=(325.0, 335.0))
    ph = math.radians(L['open'])
    u = (-math.sin(ph), math.cos(ph))
    n = (-math.cos(ph), -math.sin(ph))
    o_face = (yh + OPEN_CONTACT * u[0] + 20.0 * n[0], zh + OPEN_CONTACT * u[1] + 20.0 * n[1])
    P = lambda t, w=0.0: (o_face[0] + t * n[0] + w * u[0], o_face[1] + t * n[1] + w * u[1])
    c = P(24.5)                                   # the lug's middle, 12 to 37 along the axis, the lock nut 2 to 12
    opened = dict(face=o_face, axis=n, x=ax, u=u, at=P,
                  foot=dict(x=(face, face + ft), y=(c[0] - 35.0, c[0] + 35.0), z=(c[1] - 35.0, c[1] + 35.0)),
                  arm=dict(x=(face + ft, 465.0), y=(c[0] - 12.0, c[0] + 12.0), z=(c[1] - 12.0, c[1] + 12.0)),
                  lug=dict(t=(12.0, 37.0), r=24.0), centre=c,
                  bolts=[P(24.5, 25.0), P(24.5, -25.0)], nut=(2.0, 12.0))
    return dict(closed=closed, open=opened)


def bracket_bolts():
    """Each bracket's two ISO 4762 M8 through the end plate with a washer and an ISO 4032 nut inside. The closing
    bracket's lip takes the stop's push on the plate's end face; the push at the absorber's axis and the lip's
    reaction make a couple about the lip's corner that the two bolts hold in tension (the bracket tipping about the
    corner, Shigley's bolt-group rule). The opening bracket's bolts lie square to the push through the lug's middle,
    so the push loads them alike: friction against the preload holds it, and they hold the tipping about the foot's
    far corner. The length by L > grip + nut, the next commercial."""
    import standard_parts as S
    js = jaw_stops()
    lay = stop_layout()
    B = BRACKET
    face = BG.LAY['plate_x'][1]
    Fi = 0.75 * B['At'] * B['Sp']
    grip = B['foot'] + (BG.LAY['plate_x'][1] - BG.LAY['plate_x'][0]) + B['washer'][2]
    need = grip + B['nut_m']
    L = S.shortest(S.commercial('DIN_EN_ISO_4762')['8'], need)
    c, o = lay['closed'], lay['open']
    # closing: couple of the push at the axis and the lip's reaction at the plate's mid-thickness, about the corner
    F = js['closed']['F']
    lip_mid = (BG.LAY['plate_x'][0] + face) / 2.0
    arm_c = (c['x'] - face) + (face - lip_mid)
    d_c = [c['pivot'][1] - y for y, _ in c['bolts']]
    T_c = F * arm_c * max(d_c) / sum(x * x for x in d_c)
    p_lip = F / ((face - BG.LAY['plate_x'][0]) * (c['lip']['z'][1] - c['lip']['z'][0]))
    closed = dict(F=F, arm=arm_c, d=d_c, F_bolt=T_c, n_sep=Fi / T_c, p_lip=p_lip)
    # opening: the push along the axis; tipping about the foot's corner farthest along the push
    F = js['open']['F']
    h = o['x'] - face
    cx, cz = o['centre']
    n = o['axis']
    reach = 35.0 * (abs(n[0]) + abs(n[1]))                   # the far corner's distance along n from the middle
    d_o = [reach + ((cx - y) * n[0] + (cz - z) * n[1]) for y, z in o['bolts']]
    T_o = F * h * max(d_o) / sum(x * x for x in d_o)
    slip = B['mu'] * (2 * Fi - 2 * T_o) / F
    opened = dict(F=F, arm=h, d=d_o, F_bolt=T_o, n_sep=Fi / T_o, slip=slip)
    return dict(Fi=Fi, grip=grip, need=need, L=L, closed=closed, open=opened)




SY_1045_HR = 310.0               # MPa, Budynas & Nisbett (2015), Table A-20, p. 1048: 1045 hot rolled


def bracket_strength():
    """Each bracket's own section where it leaves its foot, at the foot's face 437 mm out: the push at the absorber's
    axis, 48 mm out from there, bends it. The closing bracket's lug, 25 along the push and 48 across; the opening
    bracket's arm, 24 square, bent about a diagonal, the push lying along the other (W = a^3 / (6 sqrt 2)). Against
    1045 hot rolled, the bar above 76 mm Chapter 51 takes for a part this size."""
    js = jaw_stops()
    lay = stop_layout()
    c, o = lay['closed'], lay['open']
    root = c['lug']['x'][0]
    out = {}
    h = c['x'] - root
    b = c['lug']['z'][1] - c['lug']['z'][0]
    t = c['lug']['y'][1] - c['lug']['y'][0]
    M = js['closed']['F'] * h
    W = b * t ** 2 / 6.0
    out['closed'] = dict(h=h, b=b, t=t, M=M / 1000.0, W=W, sigma=M / W, n=SY_1045_HR / (M / W))
    a_ = o['arm']['y'][1] - o['arm']['y'][0]
    M = js['open']['F'] * (o['x'] - root)
    W = a_ ** 3 / (6.0 * math.sqrt(2.0))
    out['open'] = dict(h=o['x'] - root, a=a_, M=M / 1000.0, W=W, sigma=M / W, n=SY_1045_HR / (M / W))
    return out

# Chapter 49's rule: a designation new to the machines takes the next free serial; a part that changes everywhere it
# is used keeps its number
NEW_NUMBERS = [  # (number, designation, quantity, where)
    ('S-0120', 'Festo DYSC-25-25-Y1F, part no. 2480234', 4, "the jaws' stops, two to a jaw"),
    ('S-0121', 'ISO 4762 M8 x 40 - 8.8 - zinc-plated', 8, 'the stop brackets, two each'),
    ('S-0122', 'ISO 7089 8 - 200 HV - zinc-plated', 8, "under the brackets' nuts"),
    ('S-0123', 'ISO 4032 M8 - 8 - zinc-plated', 8, "the brackets' nuts"),
    ('P13-017', 'stop bracket, closing: foot, lip and lug, steel 1045', 2, 'one at each end plate'),
    ('P13-018', 'stop bracket, opening: foot, arm and lug, steel 1045', 2, 'one at each end plate'),
]
CHANGED_NUMBERS = [  # (number, as it was, as it is now)
    ('P13-006', "lever, 150 from the hinge to the rod end's pin", "the same, carried on to 200 mm, 15 thick, as the closing stop's striker"),
    ('P13-005', 'end plate', "the same, with four holes 9 across for the stop brackets' bolts"),
]
