# -*- coding: utf-8 -*-
"""vacuum_gripper.py: Chapter 46 of Volume 2, the palletizer's vacuum gripper for light cartons and slip sheets.

A plate under the tool changer carries a grid of bellows suction cups, a zone of cups over each carton of a layer,
each zone on its own compact ejectors with the air-saving function, so that the gripper takes one carton or a
whole layer of six and the slip sheet between layers. The cups' holding force against the palletizer's cycle and
brake stops (Chapter 44's models, the stops worked for each payload as in Chapter 45) by Schmalz's load cases sets
how heavy a carton may be; the cups' volume and the ejectors' evacuation times set how many ejectors a zone needs to
grip in the cycle's 0.25 s; level compensators let the cups meet cartons of uneven height.
"""
import math

import clamp_gripper as cg
import tool_changer as tc

G = 9.81
pi = math.pi

# the task (assumed, the customer's to confirm): light cartons 400 x 400 x 300 mm, a layer of 3 x 2 on a EUR
# pallet; a slip sheet 1200 x 800 between layers
CARTON = dict(l=0.400, w=0.400, h=0.300)
LAYER = dict(nx=3, ny=2)
SHEET = dict(l=1.200, w=0.800, m=1.2)       # kg, a corrugated slip sheet, assumed
S = 2.0                                      # Schmalz: at least 2.0 for porous workpieces such as carton board
P_U = 0.6                                    # bar, the vacuum the cups are rated at and the system is set to hold

# the cup: Schmalz SPB1 80 ED-65 SC065 (art. 10.01.06.03071): theoretical suction force at -0.6 bar, the lateral
# force, the volume, the stroke and sizes
CUP = dict(name='SPB1 80 ED-65', F=166.0, F_pull=200.0, F_lat=59.0, V=98.358e-3, Z=23.5, D1=82.7, Ds=81.4,
           Dmax=88.0, H=45.9, H2=15.3, d1=29.3, d2=56.0, m=0.0401)
# the ejector: Schmalz SCPSi 2-14 (two stages), from the SCPS/SCPSi family data sheet: suction rate, air consumption
# at 4 bar, the evacuation time per litre to 600 mbar, weight
EJECTOR = dict(name='SCPSi 2-14', q_max=71.5, q_air=82.0, t600=0.65, t500=0.35, m=0.195, B=18.0, H=99.0, L=93.0)
EJECTOR15 = dict(name='SCPSi 15', q_max=65.5, q_air=98.0, t600=0.9)
T_GRIP = 0.25                                # s, the grip's time in the cycle (Chapter 41)


def holding(n, m, acc):
    """Schmalz's load cases I and II together for a carton of m kg on n cups: the cups pull it up against its
    weight and the vertical acceleration and hold it sideways by their lateral force; the two shares of the cups'
    capacity added, times S, must not exceed 1."""
    a_v = acc['stop_v']
    a_h = acc['stop_h']
    u_stop = S * m * (a_v / (n * CUP['F']) + a_h / (n * CUP['F_lat']))
    u_cyc = S * m * (acc['cycle_v'] / (n * CUP['F']) + acc['cycle_h'] / (n * CUP['F_lat']))
    return dict(u_stop=u_stop, u_cycle=u_cyc, u=max(u_stop, u_cyc))


def stop_decels(payload):
    """The brakes' decelerations for `payload`: the brake's torque over the joint's inertia, and the inertia is
    linear in the payload's mass, so 1/decel is linear in it; worked exactly from Chapter 45's two stops at 150 and
    450 kg."""
    a, b = cg.stop_decels(150.0), cg.stop_decels(450.0)
    t = (payload - 150.0) / 300.0
    return {j: 1.0 / (1.0 / a[j] + t * (1.0 / b[j] - 1.0 / a[j])) for j in (1, 2, 3)}


_CYC = []


def accelerations(payload):
    """The payload's vertical (with g) and horizontal accelerations on the cycle and in the worst stop for this
    payload: the stop is the case with the largest use of the cups, so both parts of each case are kept."""
    if not _CYC:
        _CYC.extend(tc.cycle_loads(n=600))
    cyc = _CYC
    es = tc.estop_loads(decel=stop_decels(payload))
    use = lambda a: (a[2] + G) / CUP['F'] + math.hypot(a[0], a[1]) / CUP['F_lat']
    wc = max(cyc, key=lambda r: use(r['a']))
    ws = max(es['cases'], key=lambda c: use(c['a']))
    return dict(cycle_v=wc['a'][2] + G, cycle_h=math.hypot(wc['a'][0], wc['a'][1]),
                stop_v=ws['a'][2] + G, stop_h=math.hypot(ws['a'][0], ws['a'][1]), stop_case=ws)


def carton_limit(n, m_gripper, m_changer=cg.M_CHANGER, layer=LAYER, tol=1e-5):
    """The heaviest carton n cups hold through the cycle and the worst stop, a full layer under the gripper (the
    heavier the cartons, the heavier the payload and the gentler the stop: found by bisection)."""
    lo, hi = 0.5, 40.0
    k = layer['nx'] * layer['ny']
    for _ in range(60):
        m = 0.5 * (lo + hi)
        acc = accelerations(m_changer + m_gripper + k * m)
        if holding(n, m, acc)['u'] <= 1.0:
            lo = m
        else:
            hi = m
        if hi - lo < tol:
            break
    return lo


def carton_limit_k(n, m_gripper, k, m_changer=cg.M_CHANGER, tol=1e-5):
    """As carton_limit, with k cartons under the gripper (one zone to six)."""
    lo, hi = 0.5, 40.0
    for _ in range(60):
        m = 0.5 * (lo + hi)
        acc = accelerations(m_changer + m_gripper + k * m)
        if holding(n, m, acc)['u'] <= 1.0:
            lo = m
        else:
            hi = m
        if hi - lo < tol:
            break
    return lo


# the layout (mm; z = 0 the gripper's mounting face, z up; x along the pallet's 1200, y across its 800)
N_CUP = 6                                     # cups per carton, 3 along x by 2 across
M_CARTON = 6.5                                # kg, the rated carton: one carton alone, stopped hardest, holds 6.99
LAY = dict(adapter=(0.0, -16.0), beam=(-16.0, -116.0), beam_y=70.0, beam_w=60.0, beam_t=5.0, beam_len=1240.0,
           plate=(-116.0, -126.0), plate_l=1240.0, plate_w=840.0, zone_x=(-400.0, 0.0, 400.0), zone_y=(-200.0, 200.0),
           cup_dx=(-130.0, 0.0, 130.0), cup_dy=(-70.0, 70.0), sleeve=(-116.0, -66.0), sleeve_r=(15.0, 8.1),
           plunger_r=(8.0, 6.0), collar=(-66.0, -60.0), collar_r=12.0, seat=-180.0, seat_t=4.0, seat_r=12.0,
           plunger_low=-204.0, stroke=25.0, manifold=(-116.0, -76.0), manifold_xy=(150.0, 60.0))
LAY['cup_top'] = LAY['plunger_low']
LAY['cup_lip'] = LAY['plunger_low'] - CUP['H']
# the plate held to each beam's bottom wall by eight M6 x 14 cap screws on the beam's middle, from inside the beam
# through holes in its top wall, clear of the adapter's 200 mm; tapped 9 mm into the plate
LAY['plate_screw'] = dict(d=6.0, L=14.0, x=(-560.0, -420.0, -280.0, -140.0, 140.0, 280.0, 420.0, 560.0),
                          access_r=6.5, clear_r=3.3, At=20.1)
RHO_AL, RHO_ST = 2700.0, 7850.0              # kg/m3
E_AL = 71.7e9                                # Pa, aluminium (Budynas & Nisbett, 2015, Table A-5, p. 1015)
# the compensator's spring: music wire A228 (Shigley, Tables 10-4 to 10-6), squared and ground ends
SPRING = dict(d=1.5, D=20.0, F1=10.0, F2=25.0, A=2211.0, m=0.145, G=81.7e3, ssy=0.45)


def spring(s=SPRING, lay=LAY):
    """The level compensator's spring between the plate's underside and the plunger's seat: installed length L1 at
    the preload F1 with the plunger hanging, L1 - stroke at F2 when a carton pushes the cup up the whole stroke;
    k from the two, the active coils from k = d^4 G/(8 D^3 N) (Eq. 10-9), the stress tau = K_B 8 F D/(pi d^3)
    (Eqs. 10-5, 10-7), the strength Sut = A/d^m (Eq. 10-14) and Ssy = 0.45 Sut (Table 10-6)."""
    L1 = lay['plate'][1] - (lay['seat'] + lay['seat_t'])
    L2 = L1 - lay['stroke']
    k = (s['F2'] - s['F1']) / lay['stroke']
    Na = s['d'] ** 4 * s['G'] / (8 * s['D'] ** 3 * k)
    C = s['D'] / s['d']
    KB = (4 * C + 2) / (4 * C - 3)
    tau = KB * 8 * s['F2'] * s['D'] / (pi * s['d'] ** 3)
    Sut = s['A'] / s['d'] ** s['m']
    Ssy = s['ssy'] * Sut
    Nt = Na + 2
    Ls = Nt * s['d']
    L0 = L1 + s['F1'] / k
    F_s = k * (L0 - Ls)
    tau_s = KB * 8 * F_s * s['D'] / (pi * s['d'] ** 3)
    p = (L0 - 2 * s['d']) / Na
    return dict(L1=L1, L2=L2, k=k, Na=Na, Nt=Nt, C=C, KB=KB, tau=tau, Sut=Sut, Ssy=Ssy, n=Ssy / tau, Ls=Ls, L0=L0,
                F_solid=F_s, tau_solid=tau_s, n_solid=Ssy / tau_s, pitch=p, xi=(L2 - Ls) / Ls, ID=s['D'] - s['d'],
                OD=s['D'] + s['d'])


def design(m_gripper=None):
    gw = None
    try:
        import json
        import os
        gw = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vacuum_gripper_weighed.json')))
    except (OSError, ValueError):
        pass
    mg = m_gripper or (gw['total']['m'] if gw else 70.0)
    out = dict(m_gripper=mg, weighed=gw)
    out['limits'] = {n: carton_limit(n, mg) for n in (4, 6, 9)}
    out['limits_k'] = {k: carton_limit_k(N_CUP, mg, k) for k in (1, 2, 3, 4, 5, 6)}
    k = LAYER['nx'] * LAYER['ny']
    pl = cg.M_CHANGER + mg + k * M_CARTON
    acc = accelerations(pl)
    out['payload'] = pl
    out['acc'] = acc
    out['decel'] = stop_decels(pl)
    out['hold'] = holding(N_CUP, M_CARTON, acc)
    pl1 = cg.M_CHANGER + mg + M_CARTON
    acc1 = accelerations(pl1)
    out['payload1'] = pl1
    out['acc1'] = acc1
    out['hold1'] = holding(N_CUP, M_CARTON, acc1)
    # the slip sheet on all 36 cups, at the lower vacuum a porous sheet may give
    acc_s = accelerations(cg.M_CHANGER + mg + SHEET['m'])          # the sheet's own payload, the hardest stop
    u_s = holding(N_CUP * k, SHEET['m'], acc_s)['u']
    out['sheet'] = dict(payload=cg.M_CHANGER + mg + SHEET['m'], u=u_s, u_02=u_s * P_U / 0.2)
    # evacuating a zone's cups to 600 mbar with e ejectors (the cups' volume; the hoses' added)
    V_hose = pi * 0.3 ** 2 * 50.0 * 1e-3 * N_CUP     # litre: 6 mm hoses, 500 mm each
    V = N_CUP * CUP['V'] + V_hose
    out['evac'] = {e: V * EJECTOR['t600'] / e for e in (1, 2, 3)}
    out['evac15'] = V * EJECTOR15['t600'] / 2
    out['V_zone'] = V
    out['V_hose'] = V_hose
    n_ej = 2 * k
    out['air'] = dict(n=n_ej, q=n_ej * EJECTOR['q_air'], per_cycle=n_ej * EJECTOR['q_air'] * 3.0 / 60.0,
                      per_hour=n_ej * EJECTOR['q_air'] * 3.0 / 60.0 * 600 / 1000.0,
                      evac_only=n_ej * EJECTOR['q_air'] * out['evac'][2] / 60.0)
    out['spring'] = spring()
    # the plunger as a cantilever below its sleeve carrying a cup's share of the sideways load in the stop
    lay = LAY
    Fl = S * M_CARTON * acc['stop_h'] / N_CUP
    arm_ = (lay['plate'][1] - lay['plunger_low']) / 1000.0
    ro, ri = lay['plunger_r']
    I = pi / 64 * ((2 * ro) ** 4 - (2 * ri) ** 4)
    out['plunger'] = dict(F=Fl, arm=arm_, M=Fl * arm_, I=I, stress=Fl * arm_ * ro / 1000.0 / (I * 1e-12))
    # the plunger in its guide, the sleeve's bore and the plate's, from the sleeve's top to the plate's underside:
    # the two reactions; the tilt the bore's clearance allows and how far it moves the lip
    Lg = lay['sleeve'][1] - lay['plate'][1]
    a_ = arm_ * 1000.0
    c_ = 2 * (lay['sleeve_r'][1] - ro)
    lip = lay['plate'][1] - lay['cup_lip']
    out['plunger'].update(Lg=Lg, R_low=Fl * (a_ + Lg) / Lg, R_up=Fl * a_ / Lg, p=Fl * (a_ + Lg) / Lg / (2 * ro * 10.0),
                          clear=c_, tilt=c_ / Lg, lip=lip, shift=c_ / Lg * (Lg / 2 + lip))
    out['cases'] = cases(pl)
    out['j3'] = j3_stop(out['decel'][3])
    out['shares'] = shares(N_CUP, M_CARTON, acc)
    out['by_count'] = by_count(mg, out['limits_k'])
    out['push'] = dict(F=N_CUP * SPRING['F2'], F_mid=N_CUP * (SPRING['F1'] + 0.5 * LAY['stroke'] * out['spring']['k']),
                       cartons=N_CUP * SPRING['F2'] / (M_CARTON * G))
    out['frame'] = frame(mg, gw, acc)
    return out


def cases(payload):
    """Every brake stop at the payload: the pose, the joint, the sense, the vertical acceleration with g, the
    horizontal, and a carton's use of one cup per kg at S = 1, (a_v/F_s + a_h/F_l)."""
    es = tc.estop_loads(decel=stop_decels(payload))
    rows = []
    for c in es['cases']:
        a = c['a']
        av, ah = a[2] + G, math.hypot(a[0], a[1])
        rows.append(dict(pose=c['pose'], joint=c['joint'], sense=c['sense'], a_v=av, a_h=ah,
                         per=av / CUP['F'] + ah / CUP['F_lat']))
    return rows


def j3_stop(alpha):
    """J3's brake stop worked by hand: the upper arm L3 from the elbow to the wrist, the parallelograms keeping the
    load level, so the load moves with the wrist, at alpha L3 across the upper arm; its parts at the cycle's low and
    high points."""
    import palletizer_design as P
    L3 = P.LAYOUT['L3'] / 1000.0
    a = alpha * L3
    w = P.waypoints()
    out = dict(alpha=alpha, L3=L3, a=a)
    for k in ('lo', 'hi'):
        q3 = math.radians(w[k]['q3'])
        av = a * math.cos(q3) + G
        ah = a * abs(math.sin(q3))
        out[k] = dict(q3=w[k]['q3'], a_v=av, a_h=ah, per=av / CUP['F'] + ah / CUP['F_lat'])
    return out


def shares(n, m, acc):
    """Equation 46.1's two shares in the stop, their sum and, for comparison, the root of their squares; the
    friction coefficient the cup's lateral force stands for, F_l / F_s."""
    sv = S * m * acc['stop_v'] / (n * CUP['F'])
    sh = S * m * acc['stop_h'] / (n * CUP['F_lat'])
    return dict(pull=sv, side=sh, sum=sv + sh, quad=math.hypot(sv, sh), mu=CUP['F_lat'] / CUP['F'])


def by_count(mg, limits_k, m=None):
    """For one carton under the gripper to six: the payload with cartons of m kg, J3's stop, the cups' use and the
    heaviest carton."""
    m = m or M_CARTON
    rows = []
    for k in range(1, 7):
        pl = cg.M_CHANGER + mg + k * m
        acc = accelerations(pl)
        rows.append(dict(k=k, payload=pl, j3=stop_decels(pl)[3], u=holding(N_CUP, m, acc)['u'],
                         limit=limits_k[k]))
    return rows


def _cantilever(P, L, w, EI, x):
    """The deflection at x of a cantilever of length L carrying point loads P [(F, a)] and a uniform load w
    (Budynas & Nisbett, 2015, Table A-9, cases 2 and 3, pp. 1021-1022), in N and m."""
    d = 0.0
    for F, a in P:
        d += F * a * a * (3 * x - a) / (6 * EI) if a <= x else F * x * x * (3 * a - x) / (6 * EI)
    return d + w * x * x * (6 * L * L - 4 * L * x + x * x) / (24 * EI)


def frame(mg, gw, acc, lay=LAY):
    """The plate and the beams: the plate's overhang beyond each beam as a cantilever 1240 mm wide carrying its
    side's cups, compensators, manifolds and ejectors, and their cartons in the stop; each beam as a cantilever from
    the adapter's screws carrying a quarter of what hangs below the beams; the M6 that hold the plate to a beam, the
    plate's middle taken to carry nothing across (its half pivoting on the beam's inner edge). Masses from the
    drawing's sizes; their total against Fusion's weighing."""
    t = lay['plate'][0] - lay['plate'][1]
    b = lay['plate_l']
    y_in = lay['beam_y'] - lay['beam_w'] / 2
    y_out = lay['beam_y'] + lay['beam_w'] / 2
    y_edge = lay['plate_w'] / 2
    Lo = (y_edge - y_out) / 1000.0
    # the parts' masses
    bt = lay['beam_t']
    hb = lay['beam'][0] - lay['beam'][1]
    A_b = lay['beam_w'] * hb - (lay['beam_w'] - 2 * bt) * (hb - 2 * bt)
    m_beams = 2 * RHO_AL * A_b * lay['beam_len'] * 1e-9
    ro, ri = lay['sleeve_r']
    m_sleeve = RHO_AL * pi * (ro * ro - ri * ri) * (lay['sleeve'][1] - lay['sleeve'][0]) * 1e-9
    sp = spring()
    m_spring = RHO_ST * pi * SPRING['D'] * sp['Nt'] * pi * SPRING['d'] ** 2 / 4 * 1e-9
    mx, my = lay['manifold_xy']
    m_man = RHO_AL * mx * my * (lay['manifold'][1] - lay['manifold'][0]) * 1e-9
    n_cups = N_CUP * LAYER['nx'] * LAYER['ny']
    holes = n_cups * pi * ri * ri * t + 4 * pi * 10.0 ** 2 * t
    m_plate = RHO_AL * (b * lay['plate_w'] * t - holes) * 1e-9
    m_zone = gw['Zone 1']['m'] if gw else 1.14
    m_frame_calc = m_beams + m_plate + n_cups * (m_sleeve + m_spring) + 6 * (m_man + 2 * EJECTOR['m'])
    m_hang = gw['Frame']['m'] - m_beams + 6 * m_zone + 6 * M_CARTON

    def overhang(a_v, cartons):
        # a side's three zones: per cup point the carton's sixth, the zone's sixth, a sleeve and a spring
        per_cup = (M_CARTON / N_CUP if cartons else 0.0) + m_zone / N_CUP + m_sleeve + m_spring
        offs = [(zy + dy) - y_out for zy in (lay['zone_y'][1],) for dy in lay['cup_dy']]
        P = [(3 * 3 * per_cup * a_v, o / 1000.0) for o in offs]                        # 9 cups at each offset
        P.append((3 * (m_man + 2 * EJECTOR['m']) * a_v, (lay['zone_y'][1] - y_out) / 1000.0))
        w = RHO_AL * b * t * 1e-9 * 1000.0 * a_v                                       # N/m over the overhang
        M = sum(F * a for F, a in P) + w * Lo * Lo / 2
        I = b * t ** 3 / 12.0
        EI = E_AL * I * 1e-12
        d_out = _cantilever(P, Lo, w, EI, max(offs) / 1000.0)
        d_in = _cantilever(P, Lo, w, EI, min(offs) / 1000.0)
        d_tip = _cantilever(P, Lo, w, EI, Lo)
        return dict(P=P, w=w, M=M, stress=6 * M / (b / 1000.0 * (t / 1000.0) ** 2), d_out=d_out * 1000.0,
                    d_in=d_in * 1000.0, d_rows=(d_out - d_in) * 1000.0, d_tip=d_tip * 1000.0, I=I)

    hang = overhang(G, False)
    stop = overhang(acc['stop_v'], True)
    # each beam's half from the adapter's M8 (x = 55) to its end, a quarter of what hangs below the beams and its
    # own half, spread along its length
    x0 = 55.0
    Lb = (lay['beam_len'] / 2 - x0) / 1000.0
    w_b = (m_hang / 2 + m_beams / 2) * acc['stop_v'] / (lay['beam_len'] / 1000.0)
    Ib = (lay['beam_w'] * hb ** 3 - (lay['beam_w'] - 2 * bt) * (hb - 2 * bt) ** 3) / 12.0
    EIb = E_AL * Ib * 1e-12
    Mb = w_b * Lb * Lb / 2
    beam = dict(x0=x0, L=Lb, w=w_b, I=Ib, M=Mb, stress=Mb * hb / 2 / 1000.0 / (Ib * 1e-12),
                d_tip=w_b * Lb ** 4 / (8 * EIb) * 1000.0)
    # the M6: a half plate, everything on it, hanging on its beam's eight screws at the beam's middle and pivoting
    # on the beam's inner edge (the plate's middle carrying no moment across)
    a_v = acc['stop_v']
    W = [(F, a + (y_out - y_in) / 1000.0) for F, a in stop['P']]     # the overhang's loads, their arms from y_in
    w = stop['w']
    m_plate_half = RHO_AL * b * t * y_edge * 1e-9
    Wt = sum(F for F, _ in stop['P']) + m_plate_half * a_v
    Mo = sum(F * a for F, a in W) + w * (y_edge - y_in) ** 2 / 2 / 1e6 - w * y_in ** 2 / 2 / 1e6
    lever = (lay['beam_y'] - y_in) / 1000.0
    ns = len(lay['plate_screw']['x'])
    T = Mo / lever
    Rc = T - Wt
    screws = dict(n=ns, Wt=Wt, Mo=Mo, lever=lever, T=T, per=T / ns, Rc=Rc, stress=T / ns / lay['plate_screw']['At'],
                  shear=m_hang * acc['stop_h'] / (2 * ns))
    # the beam's twist: the half plate's couple Rc (y_beam - y_in) on it, half to each side of the adapter
    Am = (lay['beam_w'] - bt) * (hb - bt)
    Tq = Rc * lever / 2
    beam['torque'] = Tq
    beam['Am'] = Am
    beam['tau'] = Tq * 1000.0 / (2 * Am * bt)
    # the adapter's four M8 on a 110 mm square carry everything below the beams' top: its weight and the stop's
    # pull, and the sideways force at its centre's depth as a moment; the screws share the moment by their
    # distances from an axis through the pattern's middle, the worst axis a diagonal
    z_if = lay['adapter'][1]
    z_ct = lay['cup_lip'] - CARTON['h'] * 1000.0 / 2
    rows = [(gw['Frame']['m'], gw['Frame']['z']), (6 * m_zone, gw['Zone 1']['z'] if gw else -167.5),
            (6 * M_CARTON, z_ct)]
    mb = sum(r[0] for r in rows)
    zb_ = sum(r[0] * r[1] for r in rows) / mb
    Fv = mb * acc['stop_v']
    Fh = mb * acc['stop_h']
    Mi = Fh * (z_if - zb_) / 1000.0
    r_d = math.hypot(55.0, 55.0)
    sr2 = 2 * r_d ** 2
    Fs = Fv / 4 + Mi * 1000.0 * r_d / sr2
    m8 = dict(m=mb, z=zb_, depth=z_if - zb_, Fv=Fv, Fh=Fh, M=Mi, r=r_d, sr2=sr2, F_moment=Mi * 1000.0 * r_d / sr2,
              F=Fs, stress=Fs / tc.THREAD['M8'][2], shear=Fh / 4, F_side=Fv / 4 + Mi * 1000.0 * 55.0 / (4 * 55.0 ** 2))
    return dict(m_beams=m_beams, m_plate=m_plate, m_sleeve=m_sleeve, m_spring=m_spring, m_man=m_man,
                m_frame_calc=m_frame_calc, m_hang=m_hang, Lo=Lo, hang=hang, stop=stop, beam=beam, screws=screws, m8=m8)


def envelope(d=None, n=6, m=None):
    """Below the robot plate: the changer's groups and the gripper as Fusion weighs them, n cartons of m kg at their
    centre (the cups' lips 249.9 mm under the gripper's face, the cartons 300 mm deep), against Chapter 41's envelope;
    then Chapter 44's interface worked again with it, as in Chapter 45."""
    import palletizer_design as P
    d = d or design()
    m = m if m is not None else M_CARTON
    gw = d['weighed']
    face = -tc.GEOM['z_tool'] / 1000.0
    rows = [(k, v['m'], -v['z'] / 1000.0) for k, v in (cg.CW or {}).items() if k != 'note']
    rows.append(('gripper', gw['total']['m'], face - gw['total']['z'] / 1000.0))
    z_c = face + (-LAY['cup_lip'] + CARTON['h'] * 1000 / 2) / 1000.0
    rows.append(('cartons', n * m, z_c))
    M = sum(r[1] for r in rows)
    mz = sum(r[1] * r[2] for r in rows)
    I_l = n * m * ((LAYER['nx'] * CARTON['l']) ** 2 + (LAYER['ny'] * CARTON['w']) ** 2) / 12 if n == 6 else 0.0
    I = I_l + gw['total']['izz']
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
    return dict(rows=rows, m=M, mz=mz, z=mz / M, I=I, I_layer=I_l, decel=dec, need_cycle=need_c, need_stop=need_e,
                n_open=Fc / need_c, n_open_stop=Fc / need_e, M_stop=es0['worst']['M']['M'], T_stop=es0['worst']['T']['T'])


if __name__ == '__main__':
    d = design()
    print('m_gripper', d['m_gripper'], 'limits', {k: round(v, 2) for k, v in d['limits'].items()},
          'by cartons', {k: round(v, 2) for k, v in d['limits_k'].items()})
    print('payload %.1f acc' % d['payload'], {k: round(v, 3) for k, v in d['acc'].items() if k != 'stop_case'},
          d['acc']['stop_case']['joint'], d['acc']['stop_case']['pose'])
    print('decel', d['decel'], 'hold', d['hold'], 'one carton', round(d['payload1'], 1), d['hold1'])
    print('sheet', d['sheet'])
    print('evac', d['evac'], 'V', d['V_zone'], 'hose', d['V_hose'], 'evac15', d['evac15'])
    print('air', d['air'])
    print('spring', {k: round(v, 3) for k, v in d['spring'].items()})
    print('plunger', d['plunger'])
