# -*- coding: utf-8 -*-
"""clamp_gripper.py: Chapter 45 of Volume 2, the palletizer's parallel-jaw gripper for cartons.

Two jaws on HIWIN HG linear guides under an aluminium beam squeeze a pack of cartons from two sides. One ballscrew,
right-hand thread on one half and left-hand on the other, moves the two jaws together and apart by the same amount,
so they stay centred on the gripper without a separate synchroniser. A servo motor with a spring-applied brake turns
the screw; the brake holds the squeeze when the power is off. The sizes follow from what the pack needs: the squeeze
from friction against the palletizer's accelerations (Chapter 44's cycle and brake stops), the screw and its bearings
from the squeeze (HIWIN BS-08-9-EN-2204-K), the guides from the jaws' moments (HIWIN GW-10-1-EN-1701-K), the jaws'
arms from their bending, and the whole from the payload envelope Chapter 41 set at the flange.
"""
import json
import math
import os

import tool_changer as tc

G = 9.81
pi = math.pi
E_AL, E_STEEL = 71.7e9, 207e9       # Pa (Budynas & Nisbett, 2015, Table A-5, p. 1015)

# ------------------------------------------------------------------------------------------------------------------
# The task (assumed, the customer's to confirm): cartons of drinks, 400 x 400 x 300 mm, up to 40 kg; a pack of up to
# four, two along the squeeze and two side by side, 160 kg; one to three cartons along the squeeze, one or two across
# ------------------------------------------------------------------------------------------------------------------
CARTON = dict(l=0.400, w=0.400, h=0.300, m=40.0)
PACK = dict(nx=2, ny=2)                   # the design pack
MU_PAD = 0.5                              # the pads' rubber on carton board, assumed (to be measured)
MU_CC = 0.35                              # carton board on carton board, assumed (to be measured)
P_SIDE = 40e3                             # Pa, the pressure the cartons' sides accept, assumed (the customer's)
S_CYCLE, S_STOP = 2.0, 1.25               # holding factors on the cycle and in a brake stop, chosen
CLEAR = 0.050                             # m, each jaw's clearance to the pack when open for a pick
APPROACH = 0.020                          # m, each jaw's gap to the pack after pre-setting on the way down
T_GRIP = 0.25                             # s, the grip's time in the cycle (Chapter 41)

# the screw and its nuts (HIWIN BS-08-9-EN-2204-K): peeled 32 x 10, R32-10K5-DEB right and left (Table 5.4,
# pp. 41-42; the left-hand nut from stock); 32 x 5 (R32-05K5-DEB, C 20 700, C0 48 700) kept for the comparison.
# The ball, not in the table, taken as 4.763 mm, which puts the balls' circle at 33.65 on the core of 28.7
SCREW = dict(d=32.0, lead=10.0, dk=28.7, C=30900.0, C0=72800.0, nut_m=0.68, D=50.0, D1=80.0, L=83.0, L1=14.0,
             Dw=4.763, eta1=0.90, eta2=0.80)
SCREW5 = dict(d=32.0, lead=5.0, dk=29.3, C=20700.0, C0=48700.0, nut_m=0.60, D=50.0, D1=80.0, L=53.0, L1=12.0)
# the bearing units (Tables 8.27-8.30, pp. 74-75): BK25 fixed (7205A pair), BF25 supported (6205ZZ)
BK25 = dict(d=25.0, L=42.0, B=106.0, H=80.0, h=48.0, C0=26300.0, Cdyn=20500.0, Fmax=7000.0)
BF25 = dict(d=25.0, L=30.0, B=106.0, H=80.0, h=48.0)
# the guides (HIWIN GW-10-1-EN-1701-K): HGR25 rails and HGH25CA blocks (Tables 3.4, 3.6, 3.9, pp. 30-35)
GUIDE = dict(name='HGH25CA', C=26480.0, C0=36490.0, M0X=420.0, M0Y=330.0, M0Z=330.0, H=40.0, W=48.0, L=84.0,
             B=35.0, Cb=35.0, m=0.51, WR=23.0, HR=22.0, pitch=60.0, rail_kgm=3.21)
GUIDE30 = dict(name='HGH30CA', C=38740.0, C0=52190.0, M0X=660.0, M0Y=530.0, M0Z=530.0)
GUIDE20 = dict(name='HGH20CA', C=17750.0, C0=27760.0, rail_kgm=2.21)

# the layout (mm; z = 0 the gripper's mounting face under the tool plate, z up; x the squeeze, y across)
LAY = dict(adapter=(0.0, -16.0), frame=(-16.0, -116.0), frame_len=1900.0, frame_w=200.0, walls=(10.0, 12.0, 6.0),
           rail_y=70.0, rail_len=1720.0, block_face=-156.0, carriage=(-156.0, -176.0), carriage_len=194.0,
           carriage_w=200.0, d_blocks=110.0, z_screw=-225.0, pad_t=10.0, arm_t=12.0, rib=(20.0, 60.0),
           rib_y=250.0, jaw_w=800.0, pack_top=-265.0, pad=(-275.0, -555.0), arm_bottom=-565.0, j_min=200.0,
           j_max=650.0, bearing_x=880.0, spacer_w=80.0)
LAY['z_pad'] = 0.5 * (LAY['pad'][0] + LAY['pad'][1])


HERE = os.path.dirname(os.path.abspath(__file__))


def _weighed(name, default):
    try:
        return json.load(open(os.path.join(HERE, name)))
    except (OSError, ValueError):
        return default


GW = _weighed('clamp_gripper_weighed.json', None)          # written by checks/clamp_gripper_check.py
CW = _weighed('tool_changer_weighed.json', None)           # the changer's groups as Fusion weighs them
M_GRIPPER = GW['total']['m'] if GW else 110.0              # kg; 110 assumed before Fusion weighs it
M_CHANGER = sum(v['m'] for k, v in CW.items() if k != 'note') if CW else 23.46


def pack_mass(nx, ny, c=CARTON):
    return nx * ny * c['m']


_DECEL = {}


def stop_decels(payload):
    """The brakes' decelerations of J1 to J3 with `payload` kg at the flange (Chapter 42's stops_and_brakes with the
    robot as Fusion weighed it in Chapter 43, the payload at Chapter 41's place 250 mm under the flange): a lighter
    payload is stopped harder by the same brakes."""
    import hashlib
    src = os.path.join(HERE, 'palletizer_weighed.json')
    tag = hashlib.md5(open(src, 'rb').read()).hexdigest()[:12] if os.path.exists(src) else 'none'
    key = '%.3f|%s' % (payload, tag)                      # cached on disk while the weighed robot is the same
    cache_path = os.path.join(HERE, 'clamp_gripper_decels.json')
    if not _DECEL:
        _DECEL.update(_weighed('clamp_gripper_decels.json', {}))
    if key in _DECEL:
        return {int(j): v for j, v in _DECEL[key].items()}
    if key not in _DECEL:
        import palletizer_design as P
        import palletizer_drives as D
        import palletizer_links as PL
        b = dict(P.brief())
        b['payload'] = payload
        wb = PL.weighed_bodies()
        r = PL.full_rows(1200, wb)
        with PL.bodies_in(wb):
            D._RUN.clear()
            s = D.stops_and_brakes(b, r['rows'], PL.CHOICE43)
            D._RUN.clear()
        _DECEL[key] = {j: s[j]['acc'] for j in (1, 2, 3)}
        try:
            json.dump(_DECEL, open(cache_path, 'w'), indent=1)
        except OSError:
            pass
    return {int(j): v for j, v in _DECEL[key].items()}


def accelerations(payload=None):
    """|a + g| of the payload's centre, the largest on the cycle and in the brake stops (Chapter 44's models); with
    `payload` the stops' decelerations for that payload, else Chapter 44's for the full 450 kg."""
    cyc = tc.cycle_loads(n=600)
    ac = max(math.sqrt(r['a'][0] ** 2 + r['a'][1] ** 2 + (r['a'][2] + G) ** 2) for r in cyc)
    es = tc.estop_loads(decel=stop_decels(payload) if payload else None)
    w = max(es['cases'], key=lambda c: math.sqrt(c['a'][0] ** 2 + c['a'][1] ** 2 + (c['a'][2] + G) ** 2))
    a_st = math.sqrt(w['a'][0] ** 2 + w['a'][1] ** 2 + (w['a'][2] + G) ** 2)
    ah_c = max(math.hypot(r['a'][0], r['a'][1]) for r in cyc)
    ah_s = max(math.hypot(c['a'][0], c['a'][1]) for c in es['cases'])
    return dict(cycle=ac, stop=a_st, stop_case=w, ah_cycle=ah_c, ah_stop=ah_s)


def payload(nx, ny):
    """What hangs at the flange with this pack: the changer, the gripper and the cartons (kg)."""
    return M_CHANGER + M_GRIPPER + pack_mass(nx, ny)


def squeeze_needed(nx, ny, acc=None, mu=MU_PAD, s_c=S_CYCLE, s_s=S_STOP, c=CARTON):
    """The jaws' normal force that holds the pack by the pads' friction (both jaws, 2 mu N), and the force the
    cartons inside a row need: with n cartons in a row the joint next to an outer carton carries the weight of the
    cartons between it and the row's middle, (n/2 - 1) cartons' worth, on mu_cc N_row. The stops' accelerations are
    those of the payload this pack makes."""
    acc = acc or accelerations(payload(nx, ny))
    m = pack_mass(nx, ny, c)
    need_pad = max(s_c * m * acc['cycle'], s_s * m * acc['stop']) / (2 * mu)
    inner = max(nx / 2.0 - 1.0, 0.0) * c['m']            # per row
    need_cc = max(s_c * inner * acc['cycle'], s_s * inner * acc['stop']) / MU_CC * ny if inner > 0 else 0.0
    area = ny * c['w'] * (LAY['pad'][0] - LAY['pad'][1]) / 1000.0
    return dict(m=m, pad=need_pad, cc=need_cc, need=max(need_pad, need_cc), area=area, N_side=P_SIDE * area, acc=acc,
                governs='stop' if s_s * acc['stop'] > s_c * acc['cycle'] else 'cycle')


N_MAX = 5000.0      # N, the squeeze the gripper is built for, chosen above the design pack's need
N_SET = 4700.0      # N, the design pack's setting, rounded up from its need


def envelope(nx=PACK['nx'], ny=PACK['ny']):
    """What hangs below the robot plate's face, against Chapter 41's envelope (450 kg, its centre 250 mm under the
    face): the changer's groups and the gripper as Fusion weighs them, the pack at its pads' centre. Depths in m below
    the robot plate's face; the gripper's face is the tool plate's underside, 112 mm down."""
    face = -tc.GEOM['z_tool'] / 1000.0
    rows = [(k, v['m'], -v['z'] / 1000.0) for k, v in (CW or {}).items() if k != 'note']
    gz = GW['total']['z'] if GW else -200.0
    rows.append(('gripper', M_GRIPPER, face - gz / 1000.0))
    m_p = pack_mass(nx, ny)
    rows.append(('pack', m_p, face - LAY['z_pad'] / 1000.0))
    m = sum(r[1] for r in rows)
    mz = sum(r[1] * r[2] for r in rows)
    c = CARTON
    I_pack = m_p * ((nx * c['l']) ** 2 + (ny * c['w']) ** 2) / 12
    I_grip = GW['total']['izz'] if GW and 'izz' in GW['total'] else None
    return dict(rows=rows, m=m, mz=mz, z=mz / m, I_pack=I_pack, I_grip=I_grip,
                I=I_pack + (I_grip or 0.0), m_env=450.0, mz_env=450.0 * 0.250, I_env=78.0)


def interface_again(nx=PACK['nx'], ny=PACK['ny']):
    """Chapter 44's interface worked again with this gripper and pack instead of the envelope: the payload's mass at
    its own centre, its inertia about J4, the stops' decelerations for that payload; the coupling face's factors and
    the robot plate's screws against Chapter 44's."""
    import palletizer_design as P
    ev = envelope(nx, ny)
    b = dict(P.brief())
    b['payload'] = ev['m']
    b['drop'] = ev['z'] * 1000.0
    b['layer'] = (math.sqrt(12 * ev['I'] / ev['m']) * 1000.0, 0.0)          # so that hanging() gives this inertia
    dec = stop_decels(ev['m'])
    cyc = tc.cycle_loads(depth=0.0, b=b)
    cyc_i = tc.cycle_loads(depth=tc.CHANGER['depth'], b=b)
    es = tc.estop_loads(depth=0.0, b=b, decel=dec)
    es_i = tc.estop_loads(depth=tc.CHANGER['depth'], b=b, decel=dec)
    c = tc.CHANGER
    need_c = max(tc.face_clamp_needed(x['Fa'], x['M'], c['face_Ri'], c['face_Ro'])[0] for x in cyc_i)
    need_e = max(tc.face_clamp_needed(cs['Fa'], cs['M'], c['face_Ri'], c['face_Ro'])[0] for cs in es_i['cases'])
    Fc = tc.design()['lock']['Fc']
    bj = tc.bolt_joint(160, c['h_flange'], c['cls'], cyc, es)
    peaks = {k: max(x[k] for x in cyc) for k in ('Fa', 'V', 'M', 'T')}
    worst = {k: es['worst'][k][k] for k in ('Fa', 'V', 'M', 'T')}
    return dict(env=ev, decel=dec, peaks=peaks, worst=worst, need_cycle=need_c, need_stop=need_e,
                n_open=Fc / need_c, n_open_stop=Fc / need_e, bolts=bj)


def design():
    out = dict(acc_env=accelerations())
    # the squeeze for each pack the gripper takes, its stops' accelerations those of its own payload
    packs = {}
    for nx in (1, 2, 3):
        for ny in (1, 2):
            if pack_mass(nx, ny) > pack_mass(PACK['nx'], PACK['ny']):
                continue
            s = squeeze_needed(nx, ny)
            s['payload'] = payload(nx, ny)
            s['set'] = min(max(s['need'], 0.0), s['N_side'], N_MAX)
            s['ok'] = s['need'] <= s['set'] + 1e-9
            s['p'] = s['set'] / s['area']
            s['S_cycle'] = 2 * MU_PAD * s['set'] / (s['m'] * s['acc']['cycle'])
            s['S_stop'] = 2 * MU_PAD * s['set'] / (s['m'] * s['acc']['stop'])
            packs[(nx, ny)] = s
    out['packs'] = packs
    dp = packs[(PACK['nx'], PACK['ny'])]
    acc = dp['acc']
    out['acc'] = acc
    out['N_set'] = N_SET
    out['design'] = dict(m=dp['m'], need=dp['need'], p=N_SET / dp['area'],
                         S_cycle=2 * MU_PAD * N_SET / (dp['m'] * acc['cycle']),
                         S_stop=2 * MU_PAD * N_SET / (dp['m'] * acc['stop']))
    # the screw: the two nuts pull on it from either side, so between them it is in tension N; the torque to make
    # the squeeze counts both nuts (Ta = F P/(2000 pi eta1) each, HIWIN F 3.15, p. 28), the torque the brake holds
    # (Tc = F P eta2/(2000 pi), F 3.16)
    sc = SCREW
    T_make = 2 * N_MAX * sc['lead'] / (2000 * pi * sc['eta1'])
    T_hold = 2 * N_MAX * sc['lead'] * sc['eta2'] / (2000 * pi)
    A_core = pi / 4 * sc['dk'] ** 2
    v_close = APPROACH / (T_GRIP * 2.0 / 3.0)               # two thirds of the grip's time to close the gap
    n_close = v_close * 1000 / sc['lead'] * 60
    ls = 2 * LAY['bearing_x']
    nk = 2.71e8 * 0.692 * sc['dk'] / ls ** 2                # fixed and supported (HIWIN F 3.27, fn = 0.692, p. 29)
    out['screw'] = dict(T_make=T_make, T_hold=T_hold, stress=N_MAX / A_core, S0=sc['C0'] / N_MAX,
                        v_close=v_close, n_close=n_close, DN_close=sc['d'] * n_close, ls=ls, nk=nk,
                        nk_max=0.8 * nk, v_max=0.8 * nk * sc['lead'] / 60000.0, DN_max=sc['d'] * 0.8 * nk)
    sc5 = SCREW5
    out['screw5'] = dict(T_make=2 * N_MAX * sc5['lead'] / (2000 * pi * sc['eta1']),
                         n_close=v_close * 1000 / sc5['lead'] * 60,
                         nk_max=0.8 * 2.71e8 * 0.692 * sc5['dk'] / ls ** 2)
    # the motor and its brake, bought to this specification: at the screw, and at the motor behind a 3:1 gearhead
    # (its efficiency 0.95 counted driving, none counted holding)
    gh, eta_gh = 3.0, 0.95
    out['motor'] = dict(T_peak=1.25 * T_make, n_max=0.8 * nk, brake=1.5 * T_hold, i=gh,
                        T_peak_m=1.25 * T_make / (gh * eta_gh), n_max_m=0.8 * nk * gh, brake_m=1.5 * T_hold / gh,
                        n_close_m=n_close * gh)
    out['beam'] = beam(N_MAX, dp['m'], GW['Jaw A']['m'] if GW else 20.0, acc)
    # the squeeze set by the motor's torque: the torque for N_SET at the nominal efficiency, and the squeeze that
    # torque makes at HIWIN's 0.85 and 0.95 (F 3.15); the torque to hold before the brake opens (F 3.16)
    T_set = 2 * N_SET * sc['lead'] / (2000 * pi * sc['eta1'])
    out['force'] = dict(T_set=T_set, T_set_m=T_set / (gh * eta_gh), N_lo=N_SET * 0.85 / sc['eta1'],
                        N_hi=N_SET * 0.95 / sc['eta1'], T_release=2 * N_SET * sc['lead'] * sc['eta2'] / (2000 * pi),
                        T_release_m=2 * N_SET * sc['lead'] * sc['eta2'] / (2000 * pi) / gh,
                        preset=(CLEAR - APPROACH) / out['screw']['v_max'])
    # the pneumatic alternative: two ISO 15552 cylinders of 125 mm, Festo DSBC, pulling to close at the supply's least
    # 5.5 bar; their weight at a 500 mm stroke from the data sheet (6768 g at no stroke, 247 g per 10 mm)
    out['pneumatic'] = dict(F=6881.0 * 5.5 / 6.0, m=6.768 + 50 * 0.247, m2=2 * (6.768 + 50 * 0.247),
                            yaw=N_SET * 0.080)
    # the guides: per jaw four blocks, two on each rail, d apart along x and 2 rail_y across
    m_jaw = GW['Jaw A']['m'] if GW else 20.0
    out['m_jaw'] = m_jaw
    out['guides'] = guides(N_MAX, dp['m'], m_jaw, acc)
    # the jaws' arms: a plate with two ribs, the squeeze's couple N (z_screw - z_pad) between nut and pad
    out['arm'] = arm(N_MAX)
    out['life'] = travel_life(m_jaw)
    out['arm_flat'] = arm(N_MAX, dict(LAY, rib=(0.0, 0.0)))                # the same plate without its ribs
    # the common-mode push in a stop: the pack and the jaws pushed along x, carried by the fixed bearing
    F_cm = (dp['m'] + 2 * m_jaw) * acc['ah_stop']
    out['fixed_bearing'] = dict(F=F_cm, n=BK25['Fmax'] / F_cm)
    out['approach_time'] = APPROACH / v_close
    return out


E_Y = 0.050         # m, the pack's centre off the jaws' middle across, as far as Chapter 41 lets the payload's go


def guides(N, m_pack, m_jaw, acc, g=GUIDE, e_y=E_Y):
    """Each jaw: the squeeze's couple N (z_screw - z_pad), the vertical load (half the pack and the jaw at |a+g|) at
    the pad's face, 107 mm inboard of the blocks' centre and e_y off the jaws' middle across, and the sideways load
    across the plane at the pad's centre; the blocks' loads by HIWIN's Table 2.3 (p. 13): P = W/4 +- M_y/(2d) +-
    M_x/(2c)."""
    lay = LAY
    lever = (lay['z_screw'] - lay['z_pad']) / 1000.0
    Fv = (m_pack / 2 + m_jaw) * acc['stop']
    x_off = (lay['pad_t'] + lay['carriage_len'] / 2) / 1000.0
    My = N * lever + Fv * x_off
    Fy = (m_pack / 2 + m_jaw) * acc['ah_stop']
    zc = (lay['block_face'] - lay['z_pad']) / 1000.0
    Mx = Fy * zc + Fv * e_y
    d, c = lay['d_blocks'] / 1000.0, 2 * lay['rail_y'] / 1000.0
    P = Fv / 4 + My / (2 * d) + Mx / (2 * c)
    return dict(lever=lever, Fv=Fv, My=My, Fy=Fy, Mx=Mx, Mx_side=Fy * zc, Mx_off=Fv * e_y, P=P, fSL=g['C0'] / P,
                fSL20=GUIDE20['C0'] / P, couple=N * lever)


def travel_life(m_jaw, g=GUIDE, sc=None, cph=600.0, life_h=20000.0, fw=1.5):
    """What the guides and the screw travel in the palletizer's life: each jaw 30 mm in on the way down, 20 mm to
    the pack, 20 mm back and 30 mm out, every cycle; the blocks carry the jaw's own weight as they move (HIWIN F 2.2
    with fw, p. 11), the nut the guides' friction only."""
    sc = sc or SCREW
    per = 2 * ((CLEAR - APPROACH) + APPROACH) * 1000.0        # mm per cycle
    cycles = cph * life_h
    km = per * cycles / 1e6
    P = m_jaw * G / 4
    L = (g['C'] / (fw * P)) ** 3 * 50.0
    revs = per / sc['lead'] * cycles
    return dict(per=per, cycles=cycles, km=km, P=P, L=L, n=L / km, revs=revs)


def beam(N, m_pack, m_jaw, acc, lay=LAY, j=400.0):
    """The beam held at its middle by the adapter: each half a cantilever carrying its jaw's vertical load at the
    blocks' centre and, between the jaws, the squeeze's couple N (z_screw - z_pad) the blocks hand to it, taken as
    adding at the middle; the box's section from its walls."""
    b, h = lay['frame_w'], lay['frame'][0] - lay['frame'][1]
    t1, t2, ts = lay['walls']
    parts = [(b * t1, h - t1 / 2, b * t1 ** 3 / 12), (b * t2, t2 / 2, b * t2 ** 3 / 12),
             (2 * ts * (h - t1 - t2), t2 + (h - t1 - t2) / 2, 2 * ts * (h - t1 - t2) ** 3 / 12)]
    A = sum(p[0] for p in parts)
    zc = sum(p[0] * p[1] for p in parts) / A
    I = sum(p[2] + p[0] * (p[1] - zc) ** 2 for p in parts)
    c = max(zc, h - zc)
    Fv = (m_pack / 2 + m_jaw) * acc['stop']
    xb = (j + lay['pad_t'] + lay['carriage_len'] / 2) / 1000.0
    C = N * (lay['z_screw'] - lay['z_pad']) / 1000.0
    M = Fv * xb + C
    EI = E_AL * I * 1e-12
    defl = Fv * xb ** 3 / (3 * EI) + C * xb ** 2 / (2 * EI)
    return dict(A=A, zc=zc, I=I, c=c, Fv=Fv, xb=xb, C=C, M=M, stress=M * c / 1000.0 / (I * 1e-12), defl=defl)


def arm(N, lay=LAY):
    """The arm as a beam hanging from the carriage: a plate jaw_w x arm_t with two ribs on its outer face; the moment
    N b between the nut and the pad, its stress at the rib's tip and the pad's deflection."""
    w, t = lay['jaw_w'], lay['arm_t']
    rb, rh = lay['rib']
    A1, A2 = w * t, 2 * rb * rh
    zc = (A1 * t / 2 + A2 * (t + rh / 2)) / (A1 + A2)
    I = w * t ** 3 / 12 + A1 * (zc - t / 2) ** 2 + 2 * rb * rh ** 3 / 12 + A2 * (t + rh / 2 - zc) ** 2   # mm4
    a = (lay['carriage'][1] - lay['z_screw']) / 1000.0
    b = (lay['z_screw'] - lay['z_pad']) / 1000.0
    EI = E_AL * I * 1e-12
    M = N * b
    defl = (M * a * a / 2 + M * a * b + N * b ** 3 / 3) / EI
    return dict(I=I, zc=zc, c=t + rh - zc, M=M, stress=M * (t + rh - zc) / 1000.0 / (I * 1e-12), defl=defl, EI=EI,
                a=a, b=b)


if __name__ == '__main__':
    d = design()
    a = d['acc']
    print('acc env stop %.3f; design pack cycle %.3f stop %.3f (J%d %s) ah %.3f / %.3f' % (d['acc_env']['stop'],
          a['cycle'], a['stop'], a['stop_case']['joint'], a['stop_case']['pose'], a['ah_cycle'], a['ah_stop']))
    for k, s in sorted(d['packs'].items()):
        print(k, 'payload %.1f m %.0f pad %.0f cc %.0f need %.0f (%s) side %.0f set %.0f p %.1f kPa S %.2f / %.2f stop a %.2f' % (
            s['payload'], s['m'], s['pad'], s['cc'], s['need'], s['governs'], s['N_side'], s['set'], s['p'] / 1e3,
            s['S_cycle'], s['S_stop'], s['acc']['stop']))
    print('design', {k: round(v, 3) for k, v in d['design'].items()})
    print('screw', {k: round(v, 3) for k, v in d['screw'].items()})
    print('screw5', {k: round(v, 3) for k, v in d['screw5'].items()})
    print('motor', {k: round(v, 3) for k, v in d['motor'].items()})
    print('guides', {k: round(v, 3) for k, v in d['guides'].items()})
    print('arm', {k: round(v, 6) for k, v in d['arm'].items()})
    print('fixed bearing', d['fixed_bearing'], 'approach', d['approach_time'], 'm_jaw', d['m_jaw'])
    ev = envelope()
    print('envelope', [(r[0], round(r[1], 2), round(r[2], 4)) for r in ev['rows']], {k: v for k, v in ev.items() if k != 'rows'})
    ia = interface_again()
    print('interface again: decel', ia['decel'], 'peaks', {k: round(v, 1) for k, v in ia['peaks'].items()},
          'worst', {k: round(v, 1) for k, v in ia['worst'].items()})
    print('  n_open %.3f stop %.3f need %.0f / %.0f' % (ia['n_open'], ia['n_open_stop'], ia['need_cycle'], ia['need_stop']))
    bj = ia['bolts']
    print('  bolts nf %.3f nL_es %.3f n0_es %.3f slip_es %.3f' % (bj['nf'], bj['nL_estop'], bj['n0_estop'], bj['slip_estop']))
