# -*- coding: utf-8 -*-
"""flap_design.py: the trailing-edge flap's actuation system of Volume 2, Chapters 32 to 34.

The brief (values marked 'assumed' are the brief's, not from a source), the air load on the flap and its hinge
moment, the linkage from the actuator to the flap's horn, and the actuator's force, stroke, speed and power. The
screw's formulas are Childs (2014), Chapter 16; the airworthiness factor of 1.5 is the brief's.

Axes of the flap's section: x aft, z up, the hinge at the origin; the flap deflects trailing edge down by delta.
"""
import math

pi = math.pi


def brief():
    return dict(
        aircraft='light twin turboprop',                   # assumed
        mtow=5700.0,                                       # kg, assumed
        flaps=2,                                           # one panel each side
        span=3.2,                                          # m, one flap panel, assumed
        chord=0.55,                                        # m, the flap's chord, assumed
        delta_max=40.0,                                    # deg, full flap, assumed
        positions=(0.0, 15.0, 40.0),                       # up, take-off, landing; assumed
        V_FE=70.0,                                         # m/s, the highest speed with any flap out, assumed
        rho=1.225,                                         # kg/m3, sea level, standard atmosphere
        CN_max=1.2,                                        # normal-force coefficient at full flap, assumed
        x_cp=0.45,                                         # the centre of pressure, fraction of chord aft of the hinge, assumed
        ult=1.5,                                           # ultimate over limit load, assumed (the airworthiness factor)
        t_deploy=12.0,                                     # s, 0 to 40 deg at the limit load, assumed
        V_bus=28.0,                                        # V DC, the aircraft's bus, assumed
        T_min=-55.0, T_max=70.0,                           # deg C, assumed
        flights=30000,                                     # the aircraft's life, assumed
        cycles_per_flight=1,                               # out and back once a flight, assumed
        asym_max=2.0,                                      # deg, the largest left-right difference allowed, assumed
        mass_max=9.0,                                      # kg, one actuator, assumed
    )


def air(b, delta):
    """The flap's air load at V_FE with the flap down delta degrees: the dynamic pressure, the normal force taken as
    rising linearly with the deflection to CN_max at full flap, its arm about the hinge and the hinge moment
    (N m, positive trailing edge up, the way the air pushes the flap)."""
    q = 0.5 * b['rho'] * b['V_FE'] ** 2
    S = b['span'] * b['chord']
    CN = b['CN_max'] * delta / b['delta_max']
    N = q * S * CN
    arm = b['x_cp'] * b['chord']
    return dict(q=q, S=S, CN=CN, N=N, arm=arm, H=N * arm)


# the linkage: a horn of radius r on the flap at the hinge, its tip at angle theta0 - delta from +x (anticlockwise),
# pushed aft by the actuator from its gimbal at A on the rear spar's fitting
LINK = dict(r=120.0, theta0=110.0, A=(-420.0, 114.0))       # mm, deg, mm; the geometry chosen in Chapter 32


def linkage(delta, r=None, theta0=None, A=None):
    r = LINK['r'] if r is None else r
    theta0 = LINK['theta0'] if theta0 is None else theta0
    A = LINK['A'] if A is None else A
    th = math.radians(theta0 - delta)
    tip = (r * math.cos(th), r * math.sin(th))
    ax, az = tip[0] - A[0], tip[1] - A[1]
    length = math.hypot(ax, az)
    u = (ax / length, az / length)                         # along the actuator, from the gimbal to the tip
    horn = (math.cos(th), math.sin(th))
    # the transmission angle: between the actuator's line and the horn
    mu = math.degrees(math.acos(abs(u[0] * horn[0] + u[1] * horn[1])))
    # the arm of the actuator's force about the hinge: |tip x u|
    arm = abs(tip[0] * u[1] - tip[1] * u[0])
    return dict(tip=tip, length=length, u=u, mu=mu, arm=arm)


def actuator(b, deltas=None, **geo):
    """The actuator over the flap's travel: the length, the force to hold the limit hinge moment (H / arm), the
    stroke, and the force and stroke at the extremes."""
    deltas = deltas or [b['delta_max'] * i / 40.0 for i in range(41)]
    rows = []
    for d in deltas:
        k = linkage(d, **geo)
        a = air(b, d)
        F = a['H'] * 1000.0 / k['arm'] if k['arm'] > 0 else float('inf')   # N, the moment in N m, the arm in mm
        rows.append(dict(delta=d, length=k['length'], mu=k['mu'], arm=k['arm'], H=a['H'], F=F))
    out = dict(rows=rows)
    out['stroke'] = rows[-1]['length'] - rows[0]['length']
    out['L_retracted'] = rows[0]['length']
    out['L_extended'] = rows[-1]['length']
    out['mu_min'] = min(r['mu'] for r in rows)
    out['F_limit'] = max(r['F'] for r in rows)
    out['F_ult'] = b['ult'] * out['F_limit']
    out['v_mean'] = out['stroke'] / b['t_deploy']          # mm/s
    out['P_out'] = out['F_limit'] * out['v_mean'] / 1000.0  # W, the largest force at the mean speed
    out['monotonic'] = all(r2['length'] > r1['length'] for r1, r2 in zip(rows, rows[1:]))
    return out


def screw_efficiency(lead, d_p, mu, alpha_deg):
    """Childs (2014), Eq. 16.33: the efficiency of a power screw driving its load, and Eq. 16.36: it locks itself
    if mu >= tan(lambda) cos(alpha)."""
    lam = math.atan(lead / (pi * d_p))
    a = math.radians(alpha_deg)
    eta = (math.cos(a) - mu * math.tan(lam)) / (math.cos(a) + mu / math.tan(lam))
    return dict(lam=math.degrees(lam), eta=eta, locks=mu >= math.tan(lam) * math.cos(a))


# ============================================================================================================
# Chapter 33: the actuator. Sources: HIWIN GmbH (2022), Ballscrews BS-08-9-EN-2204-K: F 3.3 (p. 25), F 3.8 to
# F 3.10 and the operating factor f_p (p. 26), F 3.25 to F 3.28 (p. 29), Tables 6.6 (p. 54) and 6.8 (p. 58),
# Table 8.12 (p. 67); SKF product pages for BSD 2562 CGA and SI 20 ES; Shigley (Budynas & Nisbett, 2015):
# Table 11-2 (p. 573), Table 11-5 (p. 576), Table 14-2 (p. 730), Eqs. 14-6c, 14-8 (p. 731), 14-12 to 14-14
# (p. 736), Table A-21, Table A-22 (p. 1051), Table 8-1; Childs (2014), Eq. 16.28 (p. 697).
# ============================================================================================================
STROKES = 60000                    # 30 000 flights, out and back once each
SY_4140, SUT_4140 = 1140.0, 1250.0 # 4140 Q&T at 425 C, Shigley Table A-21 (MPa)
SY_7075, SUT_7075 = 542.0, 593.0   # 7075-T6, Shigley Table A-22 (MPa)
E_STEEL, NU_STEEL = 207000.0, 0.292   # Shigley Table A-5

# HIWIN nuts: ds, P, ball, Cdyn, C0, nut length L, root diameter dk, nut D, flange D1 (None for a cylindrical nut)
HIWIN = {
    'R20-5T4-FSI': (20, 5, 3.175, 10910, 23560, 53, 17.32, 34, 57),
    'R25-5T4-FSI': (25, 5, 3.175, 12520, 30850, 51, 22.32, 40, 64),
    'R25-5T6-FSI': (25, 5, 3.175, 17730, 46270, 65, 22.32, 40, 63),
    'R25-10T4-FSI': (25, 10, 4.763, 20380, 43150, 80, 21.13, 45, 69),
    'R32-5T6-FSI': (32, 5, 3.175, 20270, 61620, 66, 29.32, 44, 74),
    'R32-10T3-FSI': (32, 10, 6.350, 25390, 53270, 72, 26.91, 51, 82),
    'R32-10T4-FSI': (32, 10, 6.350, 32520, 71020, 83, 26.91, 51, 82),
    'R40-10T4-FSI': (40, 10, 6.350, 37890, 94260, 87, 34.91, 60, 96),
    'R32-10T4-RSI': (32, 10, 6.350, 32520, 71020, 79, 26.91, 51, None),   # dk from Table 6.6, same screw
}
NUT = 'R32-10T4-RSI'
RSI_KEY = dict(K=32.0, W=6.0, H=3.5)       # Table 6.8: the nut's feather key groove
BSD = dict(d=25.0, D=62.0, B=15.0, alpha=62.0, C=28500.0, C0=71000.0, mass=0.23)   # SKF BSD 2562 CGA
WBK25DF_DRAG = 0.29                         # N m, starting torque of HIWIN's WBK25DF (same size pair), Table 8.12
ROD_END = dict(d=20.0, D_eye=54.0, B=16.0, B_eye=13.5, h=77.0, L=106.0, C=30000.0, C0=57000.0, mass=0.364,
               thread='M20x1.5')            # SKF SI 20 ES
B6201 = dict(d=12.0, D=32.0, B=10.0, C10=6890.0, C0=3100.0)    # Shigley Table 11-2, deep groove
LEWIS_Y = {12: 0.245, 13: 0.261, 14: 0.277, 15: 0.290, 16: 0.296, 17: 0.303, 18: 0.309, 19: 0.314, 20: 0.322,
           21: 0.328, 22: 0.331, 24: 0.337, 26: 0.346, 28: 0.353, 30: 0.359, 34: 0.371, 38: 0.384, 43: 0.397,
           50: 0.409, 60: 0.422, 75: 0.435, 100: 0.447}          # Shigley Table 14-2


def lewis_y(z):
    if z in LEWIS_Y:
        return LEWIS_Y[z]
    ks = sorted(LEWIS_Y)
    for a, c in zip(ks, ks[1:]):
        if a < z < c:
            return LEWIS_Y[a] + (LEWIS_Y[c] - LEWIS_Y[a]) * (z - a) / (c - a)
    raise ValueError('no Lewis factor for %d teeth' % z)


def cube_mean_force(act):
    """The force's cube mean over the stroke: at a steady speed the time in each step is its share of the stroke,
    so F 3.8 weights each step by its length."""
    rows = act['rows']
    num = den = 0.0
    for r1, r2 in zip(rows, rows[1:]):
        ds = r2['length'] - r1['length']
        F = 0.5 * (r1['F'] + r2['F'])
        num += F ** 3 * ds
        den += ds
    return (num / den) ** (1.0 / 3.0)


def nut_check(b, act, name, fp=3.0, F_cm=None):
    """One nut against the static rating (F 3.3 at the ultimate force), the life (F 3.8 to F 3.10) and the
    short-stroke rule of p. 26."""
    ds, P, Dw, C, C0, Ln, dk, D, D1 = HIWIN[name]
    F_cm = F_cm or cube_mean_force(act)
    rev = STROKES * act['stroke'] / P
    F_bm = fp * F_cm
    L = (C / F_bm) ** 3 * 1e6
    return dict(name=name, ds=ds, P=P, C=C, C0=C0, Ln=Ln, dk=dk, D=D, D1=D1, rev=rev, F_bm=F_bm, L=L,
                life=L / rev, static=C0 / act['F_ult'], short=act['stroke'] < 3 * Ln,
                fp_max=C / (F_cm * (rev / 1e6) ** (1.0 / 3.0)),
                r_out=(D1 / 2.0 if D1 else D / 2.0 + 5.0) + 1.0 + 4.0)   # radius of the housing round the nut


def screw_torque(F, P, ds, mu=0.005, alpha=45.0):
    e = screw_efficiency(P, ds, mu, alpha)
    return F * P / (2 * pi * e['eta']), e


def backdrive(F, P, ds, mu=0.005, alpha=45.0):
    """Childs (2014) Eq. 16.28 with the collar left out: the torque to lower the load; negative, the load turns
    the screw by itself."""
    a = math.radians(alpha)
    return F * ds * (mu * pi * ds - P * math.cos(a)) / (2 * (pi * ds * math.cos(a) + mu * P))


# the actuator laid out along its own axis s (mm): s = 0 at the gimbal's axis, + towards the horn; z up, x along
# the span. Chapter 33's figures and the Fusion script read these numbers.
LAY = dict(
    cover=(-8.0, -2.0), nose=(-2.0, 30.0), r_nose=50.0, r_cavity=44.0,
    lobe_x=34.0, lobe_z=-132.0, cav_x=27.0, cav_z=-117.0, lobe_s=40.0,
    washer=(7.0, 10.0), gear_hub=(10.0, 30.0), gear_face=(13.0, 27.0), key=(12.0, 28.0),
    shoulder_ring=(28.0, 30.0), r_shoulder_in=25.0,
    bearings=((30.0, 45.0), (45.0, 60.0)), ring_nut=(60.0, 66.0), r_ring=33.0,
    seat=(10.0, 60.0), neck=(60.0, 68.0), thread=(68.0, 240.0), groove=(70.0, 238.0), stop=(240.0, 248.0),
    journal_end=(240.0, 250.0),
    boss=(30.0, 66.0), r_boss=40.0, tube=(66.0, 245.0), r_tube=38.0, bore=62.0, gland=(245.0, 265.0),
    slot_top=34.5, slot_w=6.2,
    nut=(72.0, 151.0), sleeve=(68.0, 161.0), r_sleeve=30.5, rod_tube=(161.0, 282.0), r_rod=22.0, r_rod_in=19.0,
    plug=(282.0, 289.96), jam=(291.96, 301.96), stud=(289.96, 331.96),
    trunnion_x=(48.5, 80.0), r_trunnion=10.0, boss_x=(50.0, 62.0), r_tboss=16.0, bearing_x=70.0,
    gear=dict(m=1.5, z_G=54, z_I=30, z_P=18, z_S=26, F=14.0, backlash=0.4),
    z_I=-63.0, z_P=-99.0, sensor_angle=225.0,
    motor=(40.0, 216.0), r_motor=26.0, motor_mass=1.6, sensor=(40.0, 72.0), r_sensor=14.0,
    idler_hub=(7.0, 27.0), idler_pin=(-7.0, 40.0),
)


def gear_train(T_s, T_s_ult, n_s, L=None):
    """The spur stage from the motor unit's pinion through an idler to the gear on the screw: sizes, ratio, and
    each tooth's bending (Lewis, Eq. 14-8, with K_v of Eq. 14-6c for hobbed teeth) and contact stress (Eq. 14-14).
    T_s, T_s_ult in N mm at the screw; n_s the screw's speed in rev/min."""
    L = L or LAY
    g = L['gear']
    m, F = g['m'], g['F']
    r = {k: m * g['z_' + k] / 2.0 for k in ('G', 'I', 'P', 'S')}
    Wt = T_s / r['G']
    Wt_u = T_s_ult / r['G']
    V = r['G'] / 1000.0 * 2 * pi * n_s / 60.0                       # m/s, the pitch line
    Kv = (3.56 + math.sqrt(V)) / 3.56
    out = dict(r=r, ratio=g['z_G'] / g['z_P'], Wt=Wt, Wt_u=Wt_u, V=V, Kv=Kv,
               a_GI=r['G'] + r['I'], a_IP=r['I'] + r['P'], a_GS=r['G'] + r['S'])
    for k in ('G', 'I', 'P'):
        Y = lewis_y(g['z_' + k])
        out['Y_' + k] = Y
        out['sig_' + k] = Kv * Wt / (F * m * Y)
        out['sig_u_' + k] = Wt_u / (F * m * Y)                          # the ultimate is held at rest: K_v = 1
    Cp = math.sqrt(1.0 / (pi * 2 * (1 - NU_STEEL ** 2) / E_STEEL))     # Eq. 14-13, both gears steel
    phi = math.radians(20.0)
    out['Cp'] = Cp
    for a, c in (('P', 'I'), ('I', 'G')):
        r1, r2 = r[a] * math.sin(phi), r[c] * math.sin(phi)              # Eq. 14-12 with the radii, d sin(phi)/2
        out['sigC_%s%s' % (a, c)] = Cp * math.sqrt(Kv * Wt / (F * math.cos(phi)) * (1 / r1 + 1 / r2))
    # fatigue at the limit: the pinion and the gear bend one way, the idler both ways (Shigley Sec. 14-2)
    ka = 4.51 * SUT_4140 ** -0.265                                       # machined, Eq. 6-19
    Se = ka * 0.5 * SUT_4140                                             # Eq. 6-8; k_b taken as 1 (assumed)
    out['Se'] = Se
    out['nf_P'] = 1.0 / (out['sig_P'] / 2 / Se + out['sig_P'] / 2 / SUT_4140)    # Goodman, one way
    out['nf_G'] = 1.0 / (out['sig_G'] / 2 / Se + out['sig_G'] / 2 / SUT_4140)
    out['nf_I'] = Se / out['sig_I']                                               # fully reversed
    out['n_static'] = SY_4140 / max(out['sig_u_' + k] for k in ('G', 'I', 'P'))
    # the idler's bearings: the two mesh forces on a collinear idler add
    out['idler_load_u'] = 2 * Wt_u / math.cos(phi)
    out['idler_load'] = 2 * Wt / math.cos(phi)
    # the surface: through-hardened 4140 at 425 C, HB 370 (Table A-21), Sc = 2.22 HB + 200 MPa (Fig. 14-5, grade 1)
    out['Sc'] = 2.22 * 370 + 200.0
    out['nc'] = out['Sc'] / max(out['sigC_PI'], out['sigC_IG'])
    return out


def actuator_design(b=None, fp=3.0):
    """Chapter 33's sizing, in the order of the chapter."""
    b = b or brief()
    act = actuator(b, deltas=[b['delta_max'] * i / 400.0 for i in range(401)])
    F_cm = cube_mean_force(act)
    out = dict(act=act, F_cm=F_cm, fp=fp)
    out['nuts'] = [nut_check(b, act, n, fp, F_cm) for n in HIWIN]
    nut = nut_check(b, act, NUT, fp, F_cm)
    out['nut'] = nut
    ds, P = nut['ds'], nut['P']
    T_lim, e = screw_torque(act['F_limit'], P, ds)
    T_ult, _ = screw_torque(act['F_ult'], P, ds)
    out.update(eta=e['eta'], lam=e['lam'], T_lim=T_lim, T_ult=T_ult,
               T_back=backdrive(act['F_ult'], P, ds), T_back_lim=backdrive(act['F_limit'], P, ds))
    out['n_screw'] = act['v_mean'] / P * 60.0                             # rev/min
    out['turns_stroke'] = act['stroke'] / P
    # buckling (F 3.25, F 3.26) and the critical speed (F 3.27, F 3.28): fixed at the bearings, supported by the
    # nut in the housing; l_s from the bearings' middle to the nut's middle at full stroke
    L = LAY
    s_brg = 0.5 * (L['bearings'][0][0] + L['bearings'][1][1])
    s_nut = 0.5 * (L['nut'][0] + L['nut'][1]) + act['stroke']
    l_s = s_nut - s_brg
    dk = nut['dk']
    out['l_s'] = l_s
    out['F_k'] = 4.072e5 * 0.5 * dk ** 4 / l_s ** 2
    out['F_kmax'] = 0.5 * out['F_k']
    out['F_k_free'] = 4.072e5 * 0.0625 * dk ** 4 / l_s ** 2              # if the nut held nothing: fixed-free
    out['n_k'] = 2.71e8 * 0.692 * dk / l_s ** 2
    out['n_kmax'] = 0.8 * out['n_k']
    out['dn'] = ds * out['n_screw']
    # the thrust bearings, one of the back-to-back pair carrying the compression; a_f from Shigley Table 11-5
    a_f = 3.0
    out['brg_L'] = (BSD['C'] / (a_f * F_cm)) ** 3 * 1e6
    out['brg_life'] = out['brg_L'] / nut['rev']
    out['brg_static'] = BSD['C0'] / act['F_ult']
    out['shoulder_p'] = act['F_ult'] / (pi * ((BSD['D'] / 2) ** 2 - (L['r_shoulder_in']) ** 2))
    out['nose_sig'] = act['F_ult'] / (pi * (L['r_nose'] ** 2 - L['r_cavity'] ** 2))   # the nose's ring at the trunnions
    # the gears
    g = gear_train(T_lim, -out['T_back'], out['n_screw'])
    out['gears'] = g
    # the idler's two 6201 bearings, sharing the two mesh forces, over the idler's turns in a life (a_f 3.0)
    out['idler_rev'] = nut['rev'] * g['ratio'] * L['gear']['z_P'] / L['gear']['z_I']
    out['idler_L'] = (B6201['C10'] / (a_f * g['idler_load'] / 2)) ** 3 * 1e6
    out['idler_static'] = B6201['C0'] / (g['idler_load_u'] / 2)
    eta_mesh = 0.98                                                       # each spur mesh, assumed
    out['T_pin'] = (T_lim + WBK25DF_DRAG * 1000.0) / g['ratio'] / eta_mesh ** 2   # N mm at the motor unit's pinion
    out['n_pin'] = out['n_screw'] * g['ratio']
    # the motor unit, bought to this specification: gearhead i_gh, efficiency assumed
    i_gh, eta_gh, eta_m = 24.0, 0.81, 0.85
    out.update(i_gh=i_gh, eta_gh=eta_gh, eta_m=eta_m)
    out['n_motor'] = out['n_pin'] * i_gh
    out['T_motor'] = out['T_pin'] / i_gh / eta_gh                         # N mm
    out['P_motor'] = out['T_motor'] / 1000.0 * out['n_motor'] * 2 * pi / 60.0
    out['T_brake_need'] = -out['T_back'] / g['ratio'] / i_gh               # N mm, gears' friction left out
    out['T_brake'] = 2.0 * out['T_brake_need']                            # specified with a factor of 2, assumed
    out['eta_total'] = e['eta'] * eta_mesh ** 2 * eta_gh * eta_m
    out['I_one'] = act['P_out'] / out['eta_total'] / b['V_bus']
    out['I_two'] = 2 * out['I_one']
    # the rod: a pin-ended strut over the actuator's extended length with the tube's section (conservative)
    ro, ri = L['r_rod'], L['r_rod_in']
    I_t = pi * (2 * ro) ** 4 / 64 - pi * (2 * ri) ** 4 / 64
    A_t = pi * (ro ** 2 - ri ** 2)
    out['I_tube'], out['A_tube'] = I_t, A_t
    out['sig_tube'] = act['F_ult'] / A_t
    out['slender'] = act['L_extended'] / math.sqrt(I_t / A_t)
    out['slender_1'] = math.sqrt(2 * pi ** 2 * E_STEEL / SY_4140)       # Eq. 4-45, C = 1: Euler above, Johnson below
    out['P_euler'] = pi ** 2 * E_STEEL * I_t / act['L_extended'] ** 2      # Eq. 4-43
    out['P_johnson'] = A_t * (SY_4140 - (SY_4140 * out['slender'] / (2 * pi)) ** 2 / E_STEEL)   # Eq. 4-46
    out['P_cr'] = out['P_johnson'] if out['slender'] < out['slender_1'] else out['P_euler']
    out['stud_sig'] = act['F_ult'] / 272.0                               # M20 x 1.5, A_t 272 mm2 (Table 8-1)
    # the trunnions: half the ultimate force each, 10 mm out from the boss to the fitting's bearing
    V = act['F_ult'] / 2
    d = 2 * L['r_trunnion']
    arm = L['bearing_x'] - L['boss_x'][1]
    out['trunnion_V'] = V
    out['trunnion_sig'] = 32 * V * arm / (pi * d ** 3)
    out['trunnion_tau'] = 4 * V / (3 * pi * d ** 2 / 4)
    out['trunnion_n'] = SY_4140 / out['trunnion_sig']
    out['boss_p'] = V / (d * (L['boss_x'][1] - L['trunnion_x'][0]))
    # the guide key: the screw's torque held at the ultimate, at the key's radius
    out['key_F'] = -out['T_back'] / (L['r_sleeve'] + 1.75)
    out['rod_end_static'] = ROD_END['C0'] / act['F_ult']
    return out


if __name__ == '__main__':
    b = brief()
    for d in (0.0, 15.0, 40.0):
        print(d, {k: round(v, 4) for k, v in air(b, d).items()})
    act = actuator(b)
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in act.items() if k != 'rows'})
    for r in act['rows'][::10]:
        print({k: round(v, 3) for k, v in r.items()})
