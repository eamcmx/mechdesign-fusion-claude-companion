# -*- coding: utf-8 -*-
"""bag_gripper.py: Chapter 47 of Volume 2, the palletizer's gripper for bags, and the services every gripper takes
across the tool changer and the robot's joints.

A bag neither takes a squeeze nor seals, so it is carried in a basket: two jaws of tines swing in under it from its
long sides, the tines pass between the rollers of the pick-up station, plates at the ends keep it in place along its
length, and a plate on two cylinders presses it down onto the tines. Each jaw is a steel box tube on stub shafts in
deep-groove ball bearings, turned by a cylinder with a clamping unit on a lever at one end. The bag's weight and its
sideways inertia in the brake stops try to open the jaws; the cylinder, and with the air gone its clamping unit, must
hold them. Then the services: the air each gripper uses, the supply hose and the changer's four passages, the
electrical coupling, and how the dress pack crosses the joints.
"""
import math

import clamp_gripper as cg
import tool_changer as tc
import vacuum_gripper as vg

G = 9.81
pi = math.pi
E_ST, G_ST = 207e9, 79.3e9                 # Pa, steel (Budynas & Nisbett, 2015, Table A-5, p. 1015)
RHO_ST, RHO_AL = 7850.0, 2700.0
S_CYCLE, S_STOP = 2.0, 1.25                # as Chapter 45: the factors on the cycle and in a brake stop

# the task (assumed, the customer's to confirm): one bag of 50 kg, 800 x 500 x 160 mm, from a roller station
BAG = dict(l=800.0, w=500.0, h=160.0, m=50.0)
PICK = dict(roller_d=60.0, pitch=100.0)    # the pick-up station's rollers, across the bag (axes along y), assumed

# Festo DSBC to ISO 15552 (Festo AG & Co. KG, 2012): bore: rod, forces at 6 bar advancing and retracting (p. 7),
# cushioning length (p. 6), impact energy (p. 7), clamping unit's static holding force (p. 7), the square E, ZJ,
# the clamping unit's ZW (p. 14), the swivel flange SNCB's pin EK and XC (p. 25), weights (p. 8)
DSBC = {
    32: dict(rod=12, F_adv=483, F_ret=415, cush=20, E_imp=0.4, F_hold=600, E=45.0, ZJ=119.1, ZW=164.1, EK=10, XC=141.1, m0=465, m10=27, FL=22.0, WH=26.0),
    40: dict(rod=16, F_adv=754, F_ret=633, cush=20, E_imp=0.7, F_hold=1000, E=54.0, ZJ=133.9, ZW=186.9, EK=12, XC=158.9, m0=740, m10=37, FL=25.0, WH=28.7),
    50: dict(rod=20, F_adv=1178, F_ret=990, cush=22, E_imp=1.0, F_hold=1400, E=64.0, ZJ=141.8, ZW=208.8, EK=12, XC=168.8, m0=1190, m10=56, FL=27.0, WH=35.6),
    63: dict(rod=20, F_adv=1870, F_ret=1682, cush=22, E_imp=1.3, F_hold=2000, E=75.0, ZJ=157.1, ZW=233.1, EK=16, XC=189.1, m0=1740, m10=62, FL=32.0, WH=35.9),
    80: dict(rod=25, F_adv=3016, F_ret=2721, cush=32, E_imp=1.8, F_hold=5000, E=93.0, ZJ=173.6, ZW=268.6, EK=16, XC=209.6, m0=2660, m10=92, FL=36.0, WH=45.4),
}
# 02-series deep-groove ball bearings (Budynas & Nisbett, 2015, Table 11-2, p. 573): bore: OD, width, C10, C0 (kN)
BRG = {25: (52.0, 15.0, 14.0, 6.95), 30: (62.0, 16.0, 19.5, 10.0), 35: (72.0, 17.0, 25.5, 13.7), 40: (80.0, 18.0, 30.7, 16.6)}

# the layout (mm; z = 0 the gripper's mounting face, z up; x along the bag's 800, y across its 500)
LAY = dict(adapter=(0.0, -16.0), beam=(-16.0, -116.0), beam_x=410.0, beam_y=80.0, beam_t=6.0, block=30.0,
           plate_x=(410.0, 425.0), plate_y=360.0, plate_z=(-16.0, -245.0), wall_y=200.0, wall_z=-380.0,
           hinge=(300.0, -200.0), tube=60.0, tube_t=4.0, tube_x=405.0, stub_d=30.0,
           bearing_x=(427.0, 443.0), boss=(425.0, 445.0), boss_r=45.0,
           tine_x=(-350.0, -250.0, -150.0, -50.0, 50.0, 150.0, 250.0, 350.0), tine_t=10.0,
           leg_y=(270.0, 295.0), leg_top=-230.0, foot_z=(-505.0, -480.0), tip_y=40.0,
           bag_top=-320.0, open=45.0,
           lever=150.0, lever_t=20.0, lever_x=(480.0, 500.0), cyl_x=510.0, rod_end=40.0,
           press=dict(bore=32, stroke=50, x=(-250.0, 250.0), plate=(700.0, 400.0, 10.0), ext=24.0))
LAY['bag_bottom'] = LAY['bag_top'] - BAG['h']
CYL = dict(bore=80, stroke=125)
TUBE = dict(b=60.0, t=4.0)


def stop_decels(payload):
    return vg.stop_decels(payload)


def tine_geometry(lay=LAY):
    """One tine of the +y jaw in the y-z plane: the leg from the tube's underside down the bag's side, the foot under
    the bag to its tip; its area, its centre and the tip's offsets from the hinge."""
    yh, zh = lay['hinge']
    y0, y1 = lay['leg_y']
    zf0, zf1 = lay['foot_z']
    leg = ((y0, y1), (zf1, lay['leg_top']))
    foot = ((lay['tip_y'], y1), (zf0, zf1))
    parts = []
    for (ya, yb), (za, zb) in (leg, foot):
        A = (yb - ya) * (zb - za)
        parts.append((A, (ya + yb) / 2, (za + zb) / 2))
    A = sum(p[0] for p in parts)
    yc = sum(p[0] * p[1] for p in parts) / A
    zc = sum(p[0] * p[2] for p in parts) / A
    tip = (lay['tip_y'] - yh, zf0 - zh)
    return dict(A=A, yc=yc, zc=zc, m=A * lay['tine_t'] * RHO_ST * 1e-9, tip=tip, R=math.hypot(*tip),
                dy=yh - yc, dz=zh - zc)


def sweep(lay=LAY, n=91):
    """The tine's tip and the leg's inner foot corner as the jaw opens by phi (deg), turning the +y jaw about x so
    that its tip moves out (+y) and, at first, down; the least opening that clears the bag's side by 20 mm, the
    lowest point any part of the tine reaches."""
    yh, zh = lay['hinge']
    t = tine_geometry(lay)
    pts = {'tip': t['tip'], 'corner': (lay['leg_y'][0] - yh, lay['foot_z'][0] - zh),
           'heel': (lay['leg_y'][1] - yh, lay['foot_z'][0] - zh)}
    rows = []
    clear = None
    for k in range(n):
        ph = math.radians(k * 60.0 / (n - 1))
        c, s = math.cos(ph), math.sin(ph)
        P = {key: (yh + y * c - z * s, zh + y * s + z * c) for key, (y, z) in pts.items()}
        rows.append((math.degrees(ph), P))
        if clear is None and P['tip'][0] >= BAG['w'] / 2 + 20.0:
            clear = math.degrees(ph)
    lowest = min(P[k][1] for _, P in rows for k in P)
    lo, hi = 0.0, 60.0                                         # the clearing angle to 1e-6 deg
    tip = t['tip']
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        ph = math.radians(mid)
        lo, hi = (mid, hi) if yh + tip[0] * math.cos(ph) - tip[1] * math.sin(ph) < BAG['w'] / 2 + 20.0 else (lo, mid)
    clear = hi
    ph = math.radians(lay['open'])
    c, s = math.cos(ph), math.sin(ph)
    tip_open = (yh + t['tip'][0] * c - t['tip'][1] * s, zh + t['tip'][0] * s + t['tip'][1] * c)
    return dict(rows=rows, clear=clear, lowest=lowest, below_rollers=lay['bag_bottom'] - lowest, tip_open=tip_open)


def jaw_moment(a_v, a_h, lay=LAY, bag=BAG):
    """The moment that opens one jaw: half the bag's weight with the vertical acceleration on the tines' feet (its
    centre midway between the tips and the bag's side), the whole bag's sideways inertia on the jaw's legs at the
    bag's mid-height, and the tines' own, at their centre; a_v with g, both taken the way that opens."""
    yh, zh = lay['hinge']
    t = tine_geometry(lay)
    n = len(lay['tine_x'])
    arm_v = yh - (lay['tip_y'] + bag['w'] / 2) / 2
    arm_h = zh - (lay['bag_top'] - bag['h'] / 2)
    Mv = bag['m'] / 2 * a_v * arm_v / 1000.0
    Mh = bag['m'] * a_h * arm_h / 1000.0
    Mt = n * t['m'] * (a_v * t['dy'] + a_h * t['dz']) / 1000.0
    return dict(Mv=Mv, Mh=Mh, Mt=Mt, M=Mv + Mh + Mt, arm_v=arm_v, arm_h=arm_h)


def accelerations(payload):
    """Every brake stop at this payload and the cycle's worst for the jaws: the opening moment for each case; and the
    least vertical acceleration, with which the bag would lift off the tines."""
    es = tc.estop_loads(decel=stop_decels(payload))
    cases = []
    for c in es['cases']:
        a = c['a']
        av, ah = a[2] + G, math.hypot(a[0], a[1])
        cases.append(dict(pose=c['pose'], joint=c['joint'], sense=c['sense'], a_v=av, a_h=ah,
                          M=jaw_moment(max(av, 0.0), ah)['M']))
    cyc = tc.cycle_loads(n=600)
    cy = [dict(a_v=r['a'][2] + G, a_h=math.hypot(r['a'][0], r['a'][1])) for r in cyc]
    for r in cy:
        r['M'] = jaw_moment(r['a_v'], r['a_h'])['M']
    return dict(stop=max(cases, key=lambda r: r['M']), cases=cases, cycle=max(cy, key=lambda r: r['M']),
                lift=min(cases, key=lambda r: r['a_v']), lift_cycle=min(r['a_v'] for r in cy))


def cylinder_geometry(cyl=CYL, lay=LAY):
    """The jaw's lever (vertical, closed) and its cylinder (horizontal, closed): the cylinder pushes the lever's end
    outward to close. Pin to pin with the swivel flange: XC + the clamping unit's length + the stroke (+ the stroke
    again extended) + the rod end; closed at full stroke. The rear pin, the open length, the stroke used and the
    torque arm through the travel."""
    d = DSBC[cyl['bore']]
    yh, zh = lay['hinge']
    r = lay['lever']
    L_ret = d['XC'] + (d['ZW'] - d['ZJ']) + cyl['stroke'] + lay['rod_end']
    L_ext = L_ret + cyl['stroke']
    tip_c = (yh, zh + r)
    pin = (yh - L_ext, zh + r)
    rows = []
    for k in range(0, 61):
        ph = math.radians(k)
        tip = (yh - r * math.sin(ph), zh + r * math.cos(ph))
        L = math.hypot(tip[0] - pin[0], tip[1] - pin[1])
        u = ((tip[0] - pin[0]) / L, (tip[1] - pin[1]) / L)
        lev = (-math.sin(ph), math.cos(ph))
        arm = r * abs(u[0] * lev[1] - u[1] * lev[0]) / 1000.0         # the lever arm of the cylinder's force, m
        rows.append(dict(phi=k, L=L, arm=arm))
    op = [x for x in rows if x['phi'] == round(lay['open'])][0]
    reach = max(x['phi'] for x in rows if x['L'] >= L_ret)
    return dict(L_ret=L_ret, L_ext=L_ext, pin=pin, tip_closed=tip_c, rows=rows, L_open=op['L'],
                used=L_ext - op['L'], arm_closed=rows[0]['arm'], arm_open=op['arm'], max_open=reach)


def choose_cylinder(M_cycle, M_stop, cyl=CYL, lay=LAY):
    """The forces the jaw needs at the lever, closed (the cylinder square to the lever), against the cylinder's
    advancing force at 6 bar and its clamping unit's holding force, for each bore; the factors."""
    r = lay['lever'] / 1000.0
    out = {}
    for b, d in DSBC.items():
        out[b] = dict(F_cycle=M_cycle / r, F_stop=M_stop / r, S_cycle=d['F_adv'] * r / M_cycle,
                      S_stop=d['F_adv'] * r / M_stop, S_hold=d['F_hold'] * r / M_stop)
    ok = [b for b in sorted(out) if out[b]['S_cycle'] >= S_CYCLE and out[b]['S_stop'] >= S_STOP
          and out[b]['S_hold'] >= S_STOP]
    return dict(rows=out, smallest=ok[0] if ok else None)


MOVING = {32: (110, 9), 40: (205, 16), 50: (365, 25), 63: (430, 25), 80: (810, 39)}   # g at 0 mm, g per 10 mm (p. 8)


def rod_end_mass(lay=LAY):
    """The rod end: a block 20 x 25 from the rod's end face to the eye, and the eye, 40 across and 20 thick, with
    its bore for the pin, steel."""
    ek = DSBC[CYL['bore']]['EK']
    rod = DSBC[CYL['bore']]['rod']
    V = 20.0 * rod * lay['rod_end'] + 20.0 * pi * (20.0 ** 2 - (ek / 2) ** 2) - 20.0 * rod * 20.0 / 2
    return RHO_ST * V * 1e-9


def jaw_inertia(lay=LAY, tb=TUBE):
    """A jaw's moment of inertia about its hinge, from its parts' sizes: the box tube, the two caps and stub shafts,
    eight tines (each a leg and a foot), the lever with its hub, bar, tip and pin; the overlaps counted twice."""
    yh, zh = lay['hinge']
    rho = RHO_ST
    b, t = tb['b'], tb['t']
    Lt = 2 * lay['tube_x']
    rows = []
    mo, mi = rho * b * b * Lt * 1e-9, rho * (b - 2 * t) ** 2 * Lt * 1e-9
    rows.append(('tube', mo - mi, (mo * 2 * b * b - mi * 2 * (b - 2 * t) ** 2) / 12 * 1e-6))
    for Ls in ((lay['lever_x'][1] - lay['tube_x'] + 8.0), (lay['bearing_x'][1] + 2.0 - lay['tube_x'] + 8.0)):
        m = rho * pi * (lay['stub_d'] / 2) ** 2 * Ls * 1e-9
        mc = rho * (b - 2 * t) ** 2 * 8.0 * 1e-9
        rows.append(('stub', m + mc, (m * (lay['stub_d'] / 2) ** 2 / 2 + mc * 2 * (b - 2 * t) ** 2 / 12) * 1e-6))
    y0, y1 = lay['leg_y']
    zf0, zf1 = lay['foot_z']
    for (ya, yb), (za, zb) in (((y0, y1), (zf1, lay['leg_top'])), ((lay['tip_y'], y1), (zf0, zf1))):
        a, h = yb - ya, zb - za
        m = rho * a * h * lay['tine_t'] * 1e-9 * len(lay['tine_x'])
        d2 = ((ya + yb) / 2 - yh) ** 2 + ((za + zb) / 2 - zh) ** 2
        rows.append(('tines', m, m * ((a * a + h * h) / 12 + d2) * 1e-6))
    tl = lay['lever_x'][1] - lay['lever_x'][0]
    r = lay['lever']
    m = rho * pi * (25.0 ** 2 - (lay['stub_d'] / 2) ** 2) * tl * 1e-9
    rows.append(('hub', m, m * (25.0 ** 2 + (lay['stub_d'] / 2) ** 2) / 2 * 1e-6))
    m = rho * 40.0 * r * tl * 1e-9
    rows.append(('bar', m, m * ((40.0 ** 2 + r * r) / 12 + (r / 2) ** 2) * 1e-6))
    m = rho * pi * 20.0 ** 2 * tl * 1e-9
    rows.append(('tip', m, m * (20.0 ** 2 / 2 + r * r) * 1e-6))
    ek = DSBC[CYL['bore']]['EK']
    m = rho * pi * (ek / 2) ** 2 * 20.0 * 1e-9
    rows.append(('pin', m, m * ((ek / 2) ** 2 / 2 + r * r) * 1e-6))
    return dict(rows=rows, m=sum(x[1] for x in rows), I=sum(x[2] for x in rows))


def tube_section(tb=TUBE):
    b, t = tb['b'], tb['t']
    I = (b ** 4 - (b - 2 * t) ** 4) / 12.0
    Am = (b - t) ** 2
    J = 4 * Am ** 2 * t / (4 * (b - t))
    return dict(I=I, J=J, Am=Am, A=b * b - (b - 2 * t) ** 2)


def design(m_gripper=None):
    gw = None
    try:
        import json
        import os
        gw = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'bag_gripper_weighed.json')))
    except (OSError, ValueError):
        pass
    mg = m_gripper or (gw['total']['m'] if gw else 120.0)
    out = dict(m_gripper=mg, weighed=gw)
    pl = cg.M_CHANGER + mg + BAG['m']
    out['payload'] = pl
    out['decel'] = stop_decels(pl)
    acc = accelerations(pl)
    out['acc'] = acc
    t = tine_geometry()
    out['tine'] = t
    out['sweep'] = sweep()
    out['jaw_stop'] = jaw_moment(acc['stop']['a_v'], acc['stop']['a_h'])
    out['jaw_cycle'] = jaw_moment(acc['cycle']['a_v'], acc['cycle']['a_h'])
    out['cyl_geom'] = cylinder_geometry()
    out['choice'] = choose_cylinder(out['jaw_cycle']['M'], out['jaw_stop']['M'])
    # the tines: the foot a cantilever from the leg under its share of the bag; the leg a cantilever from the tube
    # under the foot's load and its share of the bag's sideways push at the bag's mid-height
    n = len(LAY['tine_x'])
    a_v, a_h = acc['stop']['a_v'], acc['stop']['a_h']
    Fv = BAG['m'] / (2 * n) * a_v
    span = (BAG['w'] / 2 - LAY['tip_y'])
    a_foot = (LAY['leg_y'][0] - BAG['w'] / 2) + span / 2
    tt, hf = LAY['tine_t'], LAY['foot_z'][1] - LAY['foot_z'][0]
    wl = LAY['leg_y'][1] - LAY['leg_y'][0]
    W_foot = tt * hf ** 2 / 6.0
    M_foot = Fv * a_foot / 1000.0
    Fh = BAG['m'] / n * a_h
    h_leg = LAY['leg_top'] - (LAY['bag_top'] - BAG['h'] / 2)
    M_leg = Fh * h_leg / 1000.0
    W_leg = tt * wl ** 2 / 6.0
    out['tines'] = dict(n=n, Fv=Fv, a_foot=a_foot, M_foot=M_foot, W_foot=W_foot, s_foot=M_foot * 1e3 / W_foot,
                        Fh=Fh, h_leg=h_leg, M_leg=M_leg, W_leg=W_leg, s_leg=M_leg * 1e3 / W_leg,
                        Sy=390.0)          # 1020 CD (Budynas & Nisbett, 2015, Table A-20, p. 1048)
    # the box tube as the jaw's torque tube: the cylinder's end to the far tine; its bending between the bearings
    ts = tube_section()
    T = out['jaw_stop']['M']
    L_tw = (LAY['tube_x'] + LAY['tine_x'][-1]) / 1000.0
    th = T * L_tw / (G_ST * ts['J'] * 1e-12)
    tau = T * 1000.0 / (2 * ts['Am'] * TUBE['t'])
    span_b = 2 * (LAY['bearing_x'][0] + LAY['bearing_x'][1]) / 2 / 1000.0
    F_t = Fv + t['m'] * a_v
    # the tines' loads on the tube, symmetric: reactions and the moment at the middle
    xs = [x / 1000.0 for x in LAY['tine_x']]
    R = n * F_t / 2
    Mb = R * span_b / 2 - sum(F_t * (0.0 - x) for x in xs if x < 0)
    out['tube'] = dict(sec=ts, T=T, L=L_tw, twist=th, tip_lag=th * t['R'], tau=tau, F_t=F_t, R=R, M=Mb,
                       s_bend=Mb * 1e3 * TUBE['b'] / 2 / ts['I'], span=span_b,
                       m=RHO_ST * ts['A'] * 2 * LAY['tube_x'] * 1e-9)
    # the bearings at the driven end: the tube's reaction and the cylinder's force on the lever, beyond the bearing
    cyl = DSBC[CYL['bore']]
    F_c = out['jaw_stop']['M'] / (LAY['lever'] / 1000.0)
    xb = (LAY['bearing_x'][0] + LAY['bearing_x'][1]) / 2
    a_l = (LAY['lever_x'][0] + LAY['lever_x'][1]) / 2 - xb
    P_brg = math.hypot(R, F_c)          # taken together at the driven end's bearing, conservatively
    C0 = BRG[int(LAY['stub_d'])][3] * 1000.0
    out['bearing'] = dict(R=R, F_c=F_c, P=P_brg, C0=C0, s0=C0 / P_brg, size=int(LAY['stub_d']), a_l=a_l,
                          M_stub=F_c * a_l / 1000.0,
                          s_stub=F_c * a_l / 1000.0 * 32 / (pi * (LAY['stub_d'] / 1000.0) ** 3) / 1e6)
    # the stub shaft at the lever: the cylinder's force bends it over the lever's distance from the bearing and the
    # jaw's moment twists it; von Mises, against 1045 CD (Table A-20, p. 1048: Sy 530 MPa)
    d_s = LAY['stub_d'] / 1000.0
    tau_s = 16 * T / (pi * d_s ** 3) / 1e6
    out['bearing']['tau_stub'] = tau_s
    out['bearing']['vm_stub'] = math.sqrt(out['bearing']['s_stub'] ** 2 + 3 * tau_s ** 2)
    out['bearing']['n_stub'] = 530.0 / out['bearing']['vm_stub']
    # the cylinder's end energy: the jaw turning at the end of its stroke, the rod's speed over the lever, and the
    # piston, rod and rod end moving with it (Festo's moving load at 0 mm and per 10 mm, p. 8, and the rod end)
    out['jaw_I'] = gw['jaw_I'] if gw and 'jaw_I' in gw else jaw_inertia()['I']
    r_l = LAY['lever'] / 1000.0
    m_mov = (MOVING[CYL['bore']][0] + MOVING[CYL['bore']][1] * CYL['stroke'] / 10.0) / 1000.0 + rod_end_mass()
    I_eff = out['jaw_I'] + m_mov * r_l ** 2
    w_max = math.sqrt(2 * cyl['E_imp'] / I_eff)
    v_max = w_max * r_l
    used = out['cyl_geom']['used'] / 1000.0
    t_close = used / v_max
    Tc = 6.0                                                   # s, Chapter 41's cycle
    out['impact'] = dict(I=out['jaw_I'], I_calc=jaw_inertia()['I'], m_mov=m_mov, I_eff=I_eff, E=cyl['E_imp'],
                         w=w_max, v=v_max, v_avg=used / vg.T_GRIP, t=t_close, over=t_close - vg.T_GRIP,
                         rate=3600.0 / (Tc + 2 * (t_close - vg.T_GRIP)))
    # the press plate: what keeps the bag on the tines when the stop accelerates the load down faster than g
    pr = DSBC[LAY['press']['bore']]
    lift = max(0.0, -acc['lift']['a_v'])
    F_need = S_STOP * BAG['m'] * lift
    px, py, _ = LAY['press']['plate']
    F_press = 2 * pr['F_adv']
    out['press'] = dict(lift=lift, F_need=F_need, F=F_press, S=F_press / (BAG['m'] * lift) if lift else None,
                        p_bag=F_press / (px * py) * 1e3, lift_cycle=acc['lift_cycle'])
    out['air'] = air(out)
    return out


def air(d, cyl=CYL, lay=LAY, p=6.0, cph=600.0):
    """Free air a cycle: each jaw cylinder fills its cap side to close and its rod side to open over the stroke it
    uses, the press cylinders over theirs; the volume times the absolute pressure over the atmosphere's."""
    c = DSBC[cyl['bore']]
    used = d['cyl_geom']['used']
    A_cap = pi * cyl['bore'] ** 2 / 4
    A_rod = A_cap - pi * c['rod'] ** 2 / 4
    V_j = 2 * (A_cap + A_rod) * used * 1e-6                 # litre, both jaws, close and open
    pr = DSBC[lay['press']['bore']]
    Ap = pi * lay['press']['bore'] ** 2 / 4
    Apr = Ap - pi * pr['rod'] ** 2 / 4
    sp = lay['press']['stroke']
    V_p = 2 * (Ap + Apr) * sp * 1e-6
    k = (p + 1.013) / 1.013
    free = (V_j + V_p) * k
    close = 2 * A_cap * used * 1e-6 * k
    return dict(V_jaws=V_j, V_press=V_p, ratio=k, free=free, per_hour=free * cph / 1000.0, close=close,
                q_close=close / vg.T_GRIP)


def envelope(d=None):
    """Below the robot plate: the changer's groups and the gripper as Fusion weighs them, the bag at its centre (its
    top 320 mm under the gripper's face), against Chapter 41's envelope; then Chapter 44's interface worked again
    with it, as in Chapters 45 and 46."""
    import palletizer_design as P
    d = d or design()
    gw = d['weighed']
    face = -tc.GEOM['z_tool'] / 1000.0
    rows = [(k, v['m'], -v['z'] / 1000.0) for k, v in (cg.CW or {}).items() if k != 'note']
    rows.append(('gripper', gw['total']['m'], face - gw['total']['z'] / 1000.0))
    z_b = face + (-LAY['bag_top'] + BAG['h'] / 2) / 1000.0
    rows.append(('bag', BAG['m'], z_b))
    M = sum(r[1] for r in rows)
    mz = sum(r[1] * r[2] for r in rows)
    I_b = BAG['m'] * ((BAG['l'] / 1000.0) ** 2 + (BAG['w'] / 1000.0) ** 2) / 12
    I = I_b + gw['total']['izz']
    b = dict(P.brief())
    b['payload'] = M
    b['drop'] = mz / M * 1000.0
    b['layer'] = (math.sqrt(12 * I / M) * 1000.0, 0.0)
    dec = stop_decels(M)
    c = tc.CHANGER
    cyc_i = tc.cycle_loads(depth=c['depth'], b=b)
    es_i = tc.estop_loads(depth=c['depth'], b=b, decel=dec)
    need_c = max(tc.face_clamp_needed(x['Fa'], x['M'], c['face_Ri'], c['face_Ro'])[0] for x in cyc_i)
    need_e = max(tc.face_clamp_needed(cs['Fa'], cs['M'], c['face_Ri'], c['face_Ro'])[0] for cs in es_i['cases'])
    Fc = tc.design()['lock']['Fc']
    es0 = tc.estop_loads(depth=0.0, b=b, decel=dec)
    return dict(rows=rows, m=M, mz=mz, z=mz / M, I=I, I_bag=I_b, decel=dec, n_open=Fc / need_c, n_open_stop=Fc / need_e,
                M_stop=es0['worst']['M']['M'], T_stop=es0['worst']['T']['T'])


def dress_pack():
    """How the bundle crosses each joint: twisted along the joint's axis, within igus's +-180 deg per 1 m of cable
    (igus GmbH, n.d.-a), so a joint that turns +-q needs q/180 m of free bundle along its axis; J4 at the robot's
    +-300 deg and at the cell's +-135 deg. The supply hose's run: up the turret, along both arms, down the wrist, the
    J4 loop, and the fittings as equivalent length (assumed)."""
    import palletizer_design as P
    b = P.brief()
    lay = P.LAYOUT
    rng = {'J1': 2 * b['q1max'], 'J2': 127.0, 'J3': 135.0, 'J4 robot': 2 * b['q4max'], 'J4 cell': 2 * J4_CELL}
    need = {k: v / 2 / TWIST for k, v in rng.items()}
    run = lay['A'][1] / 1000.0 + lay['L2'] / 1000.0 + lay['L3'] / 1000.0 + lay['h4'] / 1000.0 + need['J4 cell']
    return dict(range=rng, need=need, run=run, l=run + FITTINGS, fittings=FITTINGS)


J4_CELL = 135.0       # deg either way: a load's orientations repeat every 180 deg, so +-90 reaches all, +-135 spare
TWIST = 180.0         # deg per 1 m of cable (igus GmbH, n.d.-a)
FITTINGS = 1.5        # m of equivalent length for the couplings and bends (assumed; Atlas Copco Airpower NV, 2019, p. 93)


def coupling(r_frame=140.0, changes_a_year=5000.0):
    """The changer's electrical coupling: a HARTING docking frame for three modules, float mount 09 14 010 1701
    (+-2 mm floating, +-4 mm lock-in; 500 matings, 10 000 with HMC components), with a Han E (6 contacts, 16 A,
    500 V) for the clamp gripper's motor, a Han DD (12 contacts, 10 A, 250 V) for 24 V, the bus and the encoder, and
    a PE module for the motor's earth, leading. A docking frame has no PE contact of its own (HARTING's selection
    guide, pp. 2-3, 6-7), hence three places, not two. Corrected 4 November 2026 from a frame for two modules and
    10 000 matings for any module: the modules' standard versions last > 500 matings, their HMC versions > 10 000
    (guide p. 2). The changer's own location error at the frame's radius against the float; the matings of the HMC
    versions against the tool changes (assumed), and of the standard versions for comparison."""
    err = 0.0145 + 0.18e-3 * r_frame
    alloc = dict(HanE=['motor U', 'motor V', 'motor W', 'brake +', 'brake -', 'spare'],
                 HanDD=['24 V logic', '0 V logic', '24 V load', '0 V load', 'bus TD+', 'bus TD-', 'bus RD+', 'bus RD-',
                        'encoder +', 'encoder -', 'spare', 'spare'],
                 PE=['motor PE, leading'])
    return dict(frame='10 B', modules=3, err=err, float=2.0, lockin=4.0, ratio=2.0 / err, cycles=10000.0,
                cycles_std=500.0, changes=changes_a_year, years=10000.0 / changes_a_year,
                weeks_std=500.0 / changes_a_year * 52.0, alloc=alloc)


def services(d=None, dv=None):
    """The air the three grippers take through the changer: the vacuum gripper's twelve ejectors while they suck, the
    bag gripper's closing at the speed its cushioning allows; the pressure lost in the supply hose and the changer's
    passages (Atlas Copco Airpower NV, 2019, p. 92: dp = 450 qc^1.85 l/(d^5 p), bar, l/s, mm, m, bar(a))."""
    d = d or design()
    dv = dv or vg.design()
    q_vac = dv['air']['q'] / 60.0                         # l/s free air, twelve ejectors at 4 bar
    q_bag = d['air']['close'] / d['impact']['t']
    q_bag_025 = d['air']['q_close']
    q = max(q_vac, q_bag)
    dpk = dress_pack()
    out = dict(q_vac=q_vac, q_bag=q_bag, q_bag_025=q_bag_025, q=q, l=dpk['l'], dress=dpk)

    def dp(qq, l, dd, pa):
        return 450.0 * qq ** 1.85 * l / (dd ** 5 * pa)
    out['dp'] = dp
    pa = 6.0 + 1.013
    out['hose'] = {dd: dp(q, dpk['l'], dd, pa) for dd in HOSE['d']}
    out['passages'] = {n: dp(q / n, PASSAGE['l'], PASSAGE['d'], pa) for n in (1, 2, 4)}
    out['check_example'] = dp(140.0, 8 * 1.3 + 23.0, 80.0, 8.0)        # the manual's own example, p. 125: 0.0054
    out['per_hour'] = dict(vacuum=dv['air']['per_hour'], bag=d['air']['free'] * d['impact']['rate'] / 1000.0,
                           clamp=0.0)
    return out


# the supply hose's inner diameters tried and the changer's passages
HOSE = dict(d=(8.0, 11.0, 13.0, 16.0))
PASSAGE = dict(l=0.112, d=10.0)                     # m, the changer from the robot plate to the tool plate's face


if __name__ == '__main__':
    d = design()
    print('payload %.2f decel' % d['payload'], {k: round(v, 3) for k, v in d['decel'].items()})
    a = d['acc']
    print('stop', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in a['stop'].items()})
    print('cycle', {k: round(v, 3) for k, v in a['cycle'].items()})
    print('lift', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in a['lift'].items()}, 'cycle min', round(a['lift_cycle'], 3))
    print('tine', {k: (round(v, 4) if isinstance(v, float) else v) for k, v in d['tine'].items()})
    s = d['sweep']
    print('sweep clear %.2f lowest %.1f below rollers %.1f tip open' % (s['clear'], s['lowest'], s['below_rollers']), s['tip_open'])
    print('jaw stop', {k: round(v, 3) for k, v in d['jaw_stop'].items()})
    print('jaw cycle', {k: round(v, 3) for k, v in d['jaw_cycle'].items()})
    cgm = d['cyl_geom']
    print('cyl', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in cgm.items() if k != 'rows'})
    for b, r in d['choice']['rows'].items():
        print('  bore', b, {k: round(v, 3) for k, v in r.items()})
    print('  smallest', d['choice']['smallest'])
    print('tines', {k: round(v, 3) for k, v in d['tines'].items()})
    print('tube', {k: (round(v, 5) if isinstance(v, float) else '') for k, v in d['tube'].items()})
    print('bearing', {k: round(v, 3) for k, v in d['bearing'].items()})
    print('press', d['press'])
    print('air', d['air'])
    sv = services(d)
    print('services q', sv['q_vac'], sv['q_bag'], 'hose', sv['hose'], 'passages', sv['passages'], 'example', sv['check_example'])
