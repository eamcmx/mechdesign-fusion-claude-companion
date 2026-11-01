# =============================================================================
# axle_design.py  -  the rear axle of Chapters 28 to 31 (Volume 2): a live beam axle for a light pickup truck.
# No Fusion here: the numbers that decide the axle, worked in the order of the chapters, so that the scripts and
# their checks can call them. Every value marked "assumed" in brief() is a judgement of the brief, not a measurement.
#
#   brief()          the vehicle and the brief (Chapter 28, Table "the brief")
#   tyre(b)          the tyre's free radius from its size code 265/65 R17
#   duty(b)          the axle's duty: weight on the axles, the traction limit, the engine limit, the final-drive
#                    ratio from the top speed, the start on a grade, speeds, the turn, the life in revolutions
#   load_cases(b)    the four load cases on the wheels: bump, cornering, braking, full traction
#   envelope(b)      the ground clearance and the largest crown wheel it leaves room for
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math

G = 9.81                                                  # m/s^2


def brief(m=3100.0, m_r=1850.0, L=3100.0, h=800.0, track=1560.0, tyre_w=265.0, tyre_ar=65.0, rim_in=17.0,
          r_roll=375.0, r_stat=365.0, T_e=450.0, n_Pmax=3600.0, P_max=150.0, n_e_max=4000.0, i_1=4.70, i_6=0.75,
          eta_gb=0.95, eta_fd=0.96, mu=0.9, mu_y=0.8, f_r=0.015, v_max=170.0, grade=0.30, bump=3.0,
          clearance=210.0, R_turn=5.5, life_km=300000.0, z_p=12, z_c=47):
    """The brief. Masses in kg, lengths in mm, torque in N m, power in kW, speeds in rpm or km/h.
    m, m_r      gross vehicle mass and the gross load on the rear axle (assumed)
    L, h        wheelbase and the laden centre of gravity's height (assumed)
    r_roll      the tyre's rolling radius, r_stat its static loaded radius (assumed, a few per cent under the free one)
    T_e ...     the engine: 450 N m, 150 kW at 3600 rpm, 4000 rpm at most (assumed)
    i_1, i_6    the gearbox's first and sixth gears; eta_gb, eta_fd the gearbox's and the final drive's efficiency
    mu, mu_y    tyre-road friction, driving and cornering (dry asphalt, assumed); f_r rolling resistance (assumed)
    v_max       top speed (km/h) at the engine's maximum-power speed in sixth
    grade       the steepest start, laden, as a rise per unit run
    bump        the vertical load factor of a pothole (assumed)
    clearance   the least ground clearance under the axle, laden (mm)
    R_turn      the radius of the tightest turn, at the middle of the rear axle (m, assumed)
    life_km     the life of the axle (km, assumed)
    z_p, z_c    the pinion's and the crown wheel's teeth (chosen in duty())"""
    return dict(m=m, m_r=m_r, L=L, h=h, track=track, tyre_w=tyre_w, tyre_ar=tyre_ar, rim_in=rim_in, r_roll=r_roll,
                r_stat=r_stat, T_e=T_e, n_Pmax=n_Pmax, P_max=P_max, n_e_max=n_e_max, i_1=i_1, i_6=i_6,
                eta_gb=eta_gb, eta_fd=eta_fd, mu=mu, mu_y=mu_y, f_r=f_r, v_max=v_max, grade=grade, bump=bump,
                clearance=clearance, R_turn=R_turn, life_km=life_km, z_p=z_p, z_c=z_c)


def tyre(b):
    """The tyre 265/65 R17: section width 265 mm, sidewall 65 % of it, on a rim of 17 inches."""
    t = dict(rim=b['rim_in'] * 25.4, wall=b['tyre_ar'] / 100.0 * b['tyre_w'])
    t['D'] = t['rim'] + 2 * t['wall']
    t['r0'] = t['D'] / 2
    t['roll_pct'] = 100.0 * (1 - b['r_roll'] / t['r0'])
    return t


def duty(b):
    d = {}
    W = b['m'] * G                                         # N
    d['W'] = W
    d['W_r'] = b['m_r'] * G                                # static rear axle load, N
    d['W_f'] = W - d['W_r']
    d['a'] = b['L'] * b['m_r'] / b['m']                    # front axle to the centre of gravity, mm
    d['b'] = b['L'] - d['a']                               # centre of gravity to the rear axle, mm
    r = b['r_roll'] / 1000.0
    # the traction limit: F = mu (W a/L + F h/L)  ->  F = mu W (a/L) / (1 - mu h/L)
    d['aL'], d['hL'] = d['a'] / b['L'], b['h'] / b['L']
    d['F_trac'] = b['mu'] * W * d['aL'] / (1 - b['mu'] * d['hL'])
    d['W_r_dyn'] = d['W_r'] + d['F_trac'] * d['hL']        # the rear axle's load while it pulls its hardest
    d['T_trac'] = d['F_trac'] * r                          # N m, at the crown wheel (both wheels)
    # the final drive: the top speed at the maximum-power speed in sixth
    d['n_prop_top'] = b['n_Pmax'] / b['i_6']               # rpm
    v = b['v_max'] / 3.6                                   # m/s
    d['omega_w_top'] = v / r                               # rad/s
    d['n_w_top'] = d['omega_w_top'] * 60 / (2 * math.pi)   # rpm
    d['i0_max'] = d['n_prop_top'] / d['n_w_top']           # the largest ratio that still reaches v_max
    d['i0'] = b['z_c'] / b['z_p']
    d['v_top'] = d['n_prop_top'] / d['i0'] * 2 * math.pi / 60 * r * 3.6     # km/h actually reached
    # the engine limit, first gear, full torque
    d['T_pinion'] = b['T_e'] * b['i_1'] * b['eta_gb']      # N m into the pinion
    d['T_eng'] = d['T_pinion'] * d['i0'] * b['eta_fd']     # N m at the crown wheel
    d['T_design'] = min(d['T_eng'], d['T_trac'])
    d['T_shaft'] = d['T_design'] / 2                       # each half shaft (open differential)
    d['F_avail'] = d['T_eng'] / r                          # N at the road, first gear
    # the start on the grade: weight's component down the slope and rolling resistance
    th = math.atan(b['grade'])
    d['theta'] = math.degrees(th)
    d['F_grade'] = W * (math.sin(th) + b['f_r'] * math.cos(th))
    # on the grade the rear axle carries more: (W cos th a + W sin th h)/L at rest
    d['W_r_grade'] = W * (math.cos(th) * d['aL'] + math.sin(th) * d['hL'])
    d['mu_needed'] = d['F_grade'] / d['W_r_grade']
    # speeds
    d['n_pinion_max'] = b['n_e_max'] / b['i_6']            # rpm, the propeller shaft at the engine's top speed
    d['n_case_max'] = d['n_pinion_max'] / d['i0']
    # the tightest turn: the wheels on R -/+ track/2, the case at the mean
    R, t2 = b['R_turn'], b['track'] / 2000.0
    d['ratio_turn'] = (R + t2) / (R - t2)                  # outer to inner wheel speed
    d['rel_turn'] = t2 / R                                 # each side gear's speed in the case, per unit case speed
    # life
    d['rev_wheel'] = b['life_km'] * 1000.0 / (2 * math.pi * r)
    d['rev_pinion'] = d['rev_wheel'] * d['i0']
    return d


def load_cases(b):
    """The loads on one rear wheel at the road, N: vertical V, lateral S (along the axle), longitudinal X."""
    d = duty(b)
    t = b['track'] / 1000.0
    h = b['h'] / 1000.0
    Ws = d['W_r'] / 2                                      # static, per wheel
    c = {'static': dict(V=Ws, S=0.0, X=0.0)}
    c['bump'] = dict(V=b['bump'] * Ws, S=0.0, X=0.0)
    # cornering at mu_y: the rear axle's load moves to the outer wheel by W_r mu_y h / t
    dW = d['W_r'] * b['mu_y'] * h / t
    c['corner_out'] = dict(V=Ws + dW, S=b['mu_y'] * (Ws + dW), X=0.0)
    c['corner_in'] = dict(V=Ws - dW, S=b['mu_y'] * (Ws - dW), X=0.0)
    c['dW_corner'] = dW
    # braking at mu: the load moves forward; the rear wheels brake with mu times what they still carry
    W_r_brake = (d['W_r'] - b['mu'] * d['W'] * d['hL'])
    c['brake'] = dict(V=W_r_brake / 2, S=0.0, X=b['mu'] * W_r_brake / 2)
    c['W_r_brake'] = W_r_brake
    c['trac'] = dict(V=d['W_r_dyn'] / 2, S=0.0, X=d['F_trac'] / 2)
    return c


def envelope(b, wall=8.0, gap=6.0):
    """The ground clearance leaves the housing r_stat - clearance below the axle's centre; less the housing's wall
    and the gap to the crown wheel's teeth (assumed), that is the largest crown-wheel radius."""
    e = dict(below=b['r_stat'] - b['clearance'])
    e['r_crown_max'] = e['below'] - wall - gap
    e['d_crown_max'] = 2 * e['r_crown_max']
    return e


# =============================================================================
# Chapter 29: the final drive, rated by ANSI/AGMA 2003-B97 as Childs (2014, Chapter 10) gives it in SI units
# =============================================================================
ZE = 191.0                                                # steel on steel, Childs Table 10.5 (sqrt MPa)
CARBURIZED = {1: (205.0, 1380.0), 2: (240.0, 1550.0), 3: (275.0, 1720.0)}   # Tables 10.3, 10.4: sF_lim, sH_lim, MPa
KA_TABLE = {('uniform', 'uniform'): 1.00, ('uniform', 'light'): 1.25, ('light', 'uniform'): 1.10,
            ('light', 'light'): 1.35, ('light', 'medium'): 1.60}                # Table 10.1 (part)
ZI_READ = {45: 0.1221, 50: 0.1355}                        # Fig. 10.10 at 12 pinion teeth, read by tracing the curves
YJ_PINION = 0.23                                          # Fig. 10.11: 12 teeth, mate of 45 (0.230); conservative
YJ_GEAR = 0.23                                            # Fig. 10.11: 47 teeth, mate of 12 (0.232)


def zi_12_47():
    return ZI_READ[45] + (47 - 45) / 5.0 * (ZI_READ[50] - ZI_READ[45])


def kv_agma(v_et, Qv):
    """Childs Eqs. 10.16 to 10.18; v_et the pitch-line speed at the outer pitch diameter, m/s."""
    B = 0.25 * (12 - Qv) ** 0.667
    A = 50 + 56 * (1 - B)
    return ((A + math.sqrt(200 * v_et)) / A) ** B, A, B


def z_nt(n):                                              # Childs: 2 from 1e3 to 1e4 cycles, then to 1e10
    return 2.0 if n <= 1e4 else 3.4822 * n ** -0.0602


def y_nt(n):                                              # Childs, bending: 2.7 to 1e3, to 3e6, then general conditions
    if n <= 1e3:
        return 2.7
    return 6.1514 * n ** -0.1182 if n < 3e6 else 1.6831 * n ** -0.0323


def bevel_geometry(z1=12, z2=47, m=6.5, F=52.0, beta=35.0, alpha=20.0):
    g = dict(z1=z1, z2=z2, m=m, F=F, beta=beta, alpha=alpha)
    g['d1'], g['d2'] = m * z1, m * z2                     # outer pitch diameters
    g['delta1'] = math.degrees(math.atan(z1 / z2))
    g['delta2'] = 90.0 - g['delta1']
    g['Ae'] = 0.5 * m * math.hypot(z1, z2)                # outer cone distance
    g['F_max'] = g['Ae'] / 3                              # Childs: face at most a third of the cone distance
    g['Rm'] = g['Ae'] - F / 2                             # mean cone distance
    g['rm1'] = g['d1'] / 2 * g['Rm'] / g['Ae']
    g['rm2'] = g['d2'] / 2 * g['Rm'] / g['Ae']
    # teeth: addendum m cos(beta) at the heel, dedendum 1.25 times that (the helical gear's normal proportions)
    g['ha'] = m * math.cos(math.radians(beta))
    g['hf'] = 1.25 * g['ha']
    g['da2'] = g['d2'] + 2 * g['ha'] * math.cos(math.radians(g['delta2']))
    g['da1'] = g['d1'] + 2 * g['ha'] * math.cos(math.radians(g['delta1']))
    # virtual teeth: Tredgold's back cone, then the normal section of a helix (Volume 1, Chapters 14 and 17)
    g['zv1'] = z1 / math.cos(math.radians(g['delta1']))
    g['zn1'] = g['zv1'] / math.cos(math.radians(beta)) ** 3
    return g


def rate(g, T1, n1, cycles, KA, grade=2, Qv=8, Kmb=1.1):
    """Contact and bending stresses of the pinion and the gear, Childs Eqs. 10.9, 10.10, 10.12, 10.13; Y_beta = 1
    and Z_xc = 1 (crowned teeth), Z_W = K_theta = Z_Z = Y_Z = 1, as in Childs's Example 10.2."""
    sFlim, sHlim = CARBURIZED[grade]
    r = dict(T1=T1, n1=n1, cycles=cycles, KA=KA)
    r['Wt'] = 2000 * T1 / g['d1']                         # N at the outer pitch circle
    r['v_et'] = 5.236e-5 * g['d1'] * n1                   # Eq. 10.19
    r['Kv'], r['A'], r['B'] = kv_agma(r['v_et'], Qv)
    r['v_max'] = (r['A'] + (Qv - 3)) ** 2 / 200           # Eq. 10.20
    r['KHb'] = Kmb + 5.6e-6 * g['F'] ** 2                 # Eq. 10.23
    r['Zx'] = 0.00492 * g['F'] + 0.4375                   # Eq. 10.21
    r['Yx'] = 0.4867 + 0.008399 * g['m']                  # Eq. 10.22
    r['ZI'] = zi_12_47()
    r['sH'] = ZE * math.sqrt(2000 * T1 / (g['F'] * g['d1'] ** 2 * r['ZI']) * KA * r['Kv'] * r['KHb'] * r['Zx'])
    base = 2000 * T1 / (g['F'] * g['d1']) * KA * r['Kv'] / g['m'] * r['Yx'] * r['KHb']
    r['sF1'], r['sF2'] = base / YJ_PINION, base / YJ_GEAR
    n2 = cycles * g['z1'] / g['z2']
    r['ZNT1'], r['ZNT2'] = z_nt(cycles), z_nt(n2)
    r['YNT1'], r['YNT2'] = y_nt(cycles), y_nt(n2)
    r['SH1'], r['SH2'] = sHlim * r['ZNT1'] / r['sH'], sHlim * r['ZNT2'] / r['sH']
    r['SF1'], r['SF2'] = sFlim * r['YNT1'] / r['sF1'], sFlim * r['YNT2'] / r['sF2']
    return r


def gear_duty(b, n_peak_eng=2000.0, band=(1600.0, 2400.0), starts=5000):
    """The two points of the assumed duty: the design torque for the hard starts, full power in sixth for the life."""
    d = duty(b)
    u = dict(T1_peak=d['T_pinion'], n1_peak=n_peak_eng / b['i_1'])
    # a hard start, laden: the engine crosses its torque band in first gear while the truck accelerates at full torque
    r = b['r_roll'] / 1000.0
    k = 1.0 / (b['i_1'] * d['i0'])                        # wheel turns per engine turn
    v0, v1 = [n * k * 2 * math.pi / 60 * r for n in band]
    acc = (d['F_avail'] - b['f_r'] * d['W']) / b['m']
    u['acc'] = acc
    u['v0'], u['v1'] = v0, v1
    u['s_band'] = (v1 ** 2 - v0 ** 2) / (2 * acc)
    u['turns_per_start'] = u['s_band'] / (2 * math.pi * r) * d['i0']
    u['starts'] = starts
    u['n_peak'] = starts * u['turns_per_start']
    T_e_Pmax = b['P_max'] * 1000 / (b['n_Pmax'] * 2 * math.pi / 60)
    u['T_e_Pmax'] = T_e_Pmax
    u['T1_cruise'] = T_e_Pmax * b['i_6'] * b['eta_gb']
    u['n1_cruise'] = b['n_Pmax'] / b['i_6']
    u['n_cruise'] = d['rev_pinion']
    return u


def bevel_forces(g, T1, hand_sign=+1):
    """Childs Eqs. 10.1, 10.6, 10.7 at the pinion's mean radius. hand_sign +1: the upper signs (the pinion pushed away
    from the apex, as in drive with the hand chosen so); -1: the lower signs (coast)."""
    a, b_, d1 = [math.radians(x) for x in (g['alpha'], g['beta'], g['delta1'])]
    Wt = T1 * 1000.0 / g['rm1']
    Wa = Wt / math.cos(b_) * (math.tan(a) * math.sin(d1) + hand_sign * math.sin(b_) * math.cos(d1))
    Wr = Wt / math.cos(b_) * (math.tan(a) * math.cos(d1) - hand_sign * math.sin(b_) * math.sin(d1))
    return dict(Wt=Wt, Wa=Wa, Wr=Wr)


def pinion_reactions(f, rm1, a, s):
    """Reactions at the pinion's two bearings, A (the head, at x = 0) and B (the tail, at x = s), for the forces at the
    mean point, overhung at x = -a and at radius rm1 towards the crown wheel. Radial force positive pushing the pinion
    away from the crown wheel (-z); axial force positive away from the apex (+x, towards the bearings)."""
    Fx, Fz, Fy = f['Wa'], -f['Wr'], f['Wt']
    RBz = (rm1 * Fx + a * Fz) / s
    RAz = -Fz - RBz
    RBy = a * Fy / s
    RAy = -Fy - RBy
    return dict(RA=math.hypot(RAz, RAy), RB=math.hypot(RBz, RBy), RAz=RAz, RAy=RAy, RBz=RBz, RBy=RBy, Fae=Fx)


def tapered_pair(FrA, FrB, Fae, KA_, KB_):
    """Shigley Eqs. 11-18 to 11-20: bearing A is the one the external thrust Fae squeezes."""
    FiA, FiB = 0.47 * FrA / KA_, 0.47 * FrB / KB_
    if FiA <= FiB + Fae:
        FeA, FeB = 0.4 * FrA + KA_ * (FiB + Fae), FrB
        carries = 'A'
    else:
        FeB, FeA = 0.4 * FrB + KB_ * (FiA - Fae), FrA
        carries = 'B'
    return dict(FiA=FiA, FiB=FiB, FeA=max(FeA, FrA), FeB=max(FeB, FrB), carries=carries)


# The carrier's joint to the housing (Chapters 29 to 31): a plate and a flange no lower than the bowl's bottom, which
# the ground clearance sets (Change 1: 195 mm leaves 365 - 196.93 = 168.07 mm under the axle), so eight studs on a
# circle of 370 mm spread over the top three quarters, none in the bottom quarter
JOINT_BOTTOM = -166.0                                       # mm, the plate's and the flange's flat bottom
JOINT_BOLT_R = 185.0
JOINT_BOLT_DEG = [-30.0 + k * 240.0 / 7.0 for k in range(8)]


def joint_bolts():
    """The studs' positions on the joint face, (y, z) in mm."""
    return [(JOINT_BOLT_R * math.cos(math.radians(a)), JOINT_BOLT_R * math.sin(math.radians(a))) for a in JOINT_BOLT_DEG]


# SKF single row tapered roller bearings, read on skf.com's product pages, 1 November 2026:
# d, D, T, a (side face to pressure point), C kN, C0 kN, e, Y, Y0
SKF_TRB = {'32307': (35, 80, 32.75, 20.15, 117.0, 106.0, 0.31, 1.9, 1.1),
           '32308': (40, 90, 35.25, 22.95, 143.0, 140.0, 0.35, 1.7, 0.9),
           '32309': (45, 100, 38.25, 24.75, 173.0, 170.0, 0.35, 1.7, 0.9),
           '32212': (60, 110, 29.75, 24.05, 155.0, 160.0, 0.40, 1.5, 0.8),
           '32213': (65, 120, 32.75, 26.55, 186.0, 193.0, 0.40, 1.5, 0.8)}


def life_rev(C_kN, Fe_N):
    """Shigley Eq. 11-3 with a = 10/3 for roller bearings: L10 in revolutions, C rated at 1e6 revolutions (SKF)."""
    return (C_kN * 1000.0 / Fe_N) ** (10.0 / 3.0) * 1e6


def pinion_bearings(g, u, a=46.94, s=111.0, A='32309', B='32308'):
    """The pinion's head bearing A and tail bearing B, back to back (O): the drive thrust, away from the apex, squeezes
    A; the reverse thrust squeezes B. Peak drive, peak reverse (full torque in reverse, the lower signs) and cruise."""
    out = dict(a=a, s=s, A=A, B=B)
    yA, yB = SKF_TRB[A][7], SKF_TRB[B][7]
    for name, T1, sign in (('peak', u['T1_peak'], +1), ('reverse', u['T1_peak'], -1), ('cruise', u['T1_cruise'], +1)):
        f = bevel_forces(g, T1, sign)
        r = pinion_reactions(f, g['rm1'], a, s)
        if r['Fae'] >= 0:                                 # towards the tail: squeezes the head bearing A
            t = tapered_pair(r['RA'], r['RB'], r['Fae'], yA, yB)
            FeA, FeB = t['FeA'], t['FeB']
        else:                                             # towards the head: squeezes the tail bearing B
            t = tapered_pair(r['RB'], r['RA'], -r['Fae'], yB, yA)
            FeA, FeB = t['FeB'], t['FeA']
        out[name] = dict(f, **r, FeA=FeA, FeB=FeB, FiA=t['FiA'], FiB=t['FiB'])
    out['L_A'] = life_rev(SKF_TRB[A][4], out['cruise']['FeA'])
    out['L_B'] = life_rev(SKF_TRB[B][4], out['cruise']['FeB'])
    return out


def case_reactions(Wt, Wr2, Wa2, rm2, c, L):
    """Reactions at the case's bearings C1 (z = 0, the crown wheel's side) and C2 (z = L), the crown wheel's mean point
    at z = c and radius rm2 towards the pinion. Wr2 pushes the crown wheel towards its axis; Wa2 pushes it along the
    axle away from the pinion, towards C1."""
    Fx, Fy, Fz = -Wr2, Wt, -Wa2
    R2x = (rm2 * Fz - c * Fx) / L
    R1x = -Fx - R2x
    R2y = -c * Fy / L
    R1y = -Fy - R2y
    return dict(R1=math.hypot(R1x, R1y), R2=math.hypot(R2x, R2y), R1x=R1x, R1y=R1y, R2x=R2x, R2y=R2y, Fae=-Fz)


def case_bearings(g, u, d, c=47.63, L=160.4, C1='32213', C2='32213'):
    """The case's bearings, face to face (X): the crown wheel's thrust towards C1 squeezes C1."""
    out = dict(c=c, L=L, C1=C1, C2=C2)
    for name, T1 in (('peak', u['T1_peak']), ('cruise', u['T1_cruise'])):
        f = bevel_forces(g, T1, +1)
        # on the crown wheel: the same tangential force; its radial force is the pinion's axial and its axial force
        # the pinion's radial (90-degree shafts)
        r = case_reactions(f['Wt'], f['Wa'], f['Wr'], g['rm2'], c, L)
        t = tapered_pair(r['R1'], r['R2'], r['Fae'], SKF_TRB[C1][7], SKF_TRB[C2][7])
        out[name] = dict(r, FeC1=t['FeA'], FeC2=t['FeB'], carries=t['carries'])
    out['L_C1'] = life_rev(SKF_TRB[C1][4], out['cruise']['FeC1'])
    out['L_C2'] = life_rev(SKF_TRB[C2][4], out['cruise']['FeC2'])
    out['turns_needed'] = d['rev_wheel']
    return out


YJ_STRAIGHT = {'side': 0.17, 'spider': 0.21}            # Childs Fig. 10.9: 22 teeth with a mate of 15 (0.176 on the
                                                          # 15-tooth curve), 15 teeth with a mate of 22 (between the
                                                          # 20 and 25 curves, 0.21 to 0.22); both rounded down


def differential(d, z_s=22, z_p=15, N=4, m=5.5, F=24.0, grade=2, n_load=1e4, d_pin=22.0, L_bore=26.0):
    """The open differential: N spiders on a cross, two side gears, straight bevel teeth rated in bending by Childs's
    Eq. 10.12 at the design torque (they roll on each other only in turns, so bending, not pitting, decides them)."""
    x = dict(z_s=z_s, z_p=z_p, N=N, m=m, F=F)
    x['assembly'] = 2.0 * z_s / N                         # must be a whole number for the spiders to mesh
    x['delta_s'] = math.degrees(math.atan(z_s / z_p))
    x['delta_p'] = 90.0 - x['delta_s']
    x['zv_p'] = z_p / math.cos(math.radians(x['delta_p']))
    x['d_s'], x['d_p'] = m * z_s, m * z_p
    x['A'] = 0.5 * m * math.hypot(z_s, z_p)
    x['F_max'] = x['A'] / 3
    T_side = d['T_design'] / 2.0 * 1000.0                  # N mm on each side gear
    x['T_side'] = T_side
    x['Wt'] = T_side / (N * x['d_s'] / 2.0)                # per mesh, at the outer pitch circle
    x['Yx'] = 0.4867 + 0.008399 * m
    x['KHb'] = 1.2 + 5.6e-6 * F ** 2                       # neither member straddle mounted
    base = x['Wt'] / F * 1.0 * 1.0 / m * x['Yx'] * x['KHb']   # KA = 1 (the design torque is the peak), Kv = 1 (no speed)
    x['sF_side'], x['sF_spider'] = base / YJ_STRAIGHT['side'], base / YJ_STRAIGHT['spider']
    sFlim = CARBURIZED[grade][0]
    x['YNT'] = y_nt(n_load)
    x['SF_side'] = sFlim * x['YNT'] / x['sF_side']
    x['SF_spider'] = sFlim * x['YNT'] / x['sF_spider']
    # the cross pin: each spider is pushed by both side gears at their mean radius
    x['rm_s'] = x['d_s'] / 2.0 * (x['A'] - F / 2.0) / x['A']
    x['Wt_m'] = T_side / (N * x['rm_s'])
    x['F_pin'] = 2 * x['Wt_m']
    x['tau_pin'] = x['F_pin'] / (math.pi * d_pin ** 2 / 4)
    x['p_bore'] = x['F_pin'] / (d_pin * L_bore)
    # each side gear is pushed outward against its thrust washer by every mesh
    x['Fa_side'] = N * x['Wt_m'] * math.tan(math.radians(20.0)) * math.sin(math.radians(x['delta_s']))
    x['d_pin'], x['L_bore'] = d_pin, L_bore
    return x


def crown_bolts(d, n=10, r_b=93.0, A_s=113.1, Sp=830.0):
    """The crown wheel's bolts, M12 class 10.9 (Shigley Table 8-11): the torque they would carry in shear, through
    the plain shank, if the clamped faces slipped."""
    c = dict(n=n, r_b=r_b)
    c['F_bolt'] = d['T_design'] * 1000.0 / (n * r_b)
    c['tau'] = c['F_bolt'] / A_s
    c['S_sy'] = 0.577 * Sp
    c['n_s'] = c['S_sy'] / c['tau']
    return c


# =============================================================================
# Chapter 30: the half shafts, the wheel bearings and the housing (Shigley's Chapters 6, 7, 11 and Table A-20)
# =============================================================================
TABLE_11_2 = {50: (90, 20, 1.0, 56, 82, 35.1, 19.6), 55: (100, 21, 1.5, 63, 90, 43.6, 25.0),
              60: (110, 22, 1.5, 70, 99, 47.5, 28.0), 65: (120, 23, 1.5, 74, 109, 55.9, 34.0),
              70: (125, 24, 1.5, 79, 114, 61.8, 37.5)}   # 02 deep groove: OD, width, r, dS, dH, C10 kN, C0 kN
TABLE_11_1 = [(0.014, 0.19, 2.30), (0.021, 0.21, 2.15), (0.028, 0.22, 1.99), (0.042, 0.24, 1.85),
              (0.056, 0.26, 1.71), (0.070, 0.27, 1.63), (0.084, 0.28, 1.55), (0.110, 0.30, 1.45),
              (0.17, 0.34, 1.31), (0.28, 0.38, 1.15), (0.42, 0.42, 1.04), (0.56, 0.44, 1.00)]   # Fa/C0, e, Y2


def ball_equivalent(Fr, Fa, C0):
    """Shigley Eq. 11-12 with Table 11-1, interpolated in Fa/C0; X2 = 0.56; V = 1 (the inner ring turns)."""
    q = max(Fa / C0, 0.014)
    for (q0, e0, y0), (q1, e1, y1) in zip(TABLE_11_1, TABLE_11_1[1:]):
        if q0 <= q <= q1:
            t = (q - q0) / (q1 - q0)
            e, Y = e0 + t * (e1 - e0), y0 + t * (y1 - y0)
            break
    else:
        e, Y = TABLE_11_1[-1][1], TABLE_11_1[-1][2]
    if Fa / Fr <= e:
        return Fr, e, Y, 1.0
    return 0.56 * Fr + Y * Fa, e, Y, 0.56


def wheel_end(b, b_off=65.0, y_inner=100.0, d_seat=65, d_body=48.0, d_spline=35.0, r_fillet_ratio=0.1,
              Sy=1140.0, Sut=1250.0, Kt=1.7, Kts=1.5):
    """The semi-floating wheel end, for one wheel. The wheel's middle plane at track/2 from the axle's middle; the
    bearing b_off inboard of it; the shaft's inner end held by the side gear at y_inner. Loads of load_cases() at the
    road, r_stat below the axis: the bending at the bearing is |b V - r S| on the outer wheel in a turn (the side force
    points to the turn's centre, inwards), b V + r S on the inner. The shaft is 4140 quenched and tempered at 425 C
    (Shigley Table A-21); the seat of the bearing is d_seat, the shaft's body d_body, the splines' minor diameter
    d_spline."""
    c = load_cases(b)
    t2 = b['track'] / 2.0
    y_b = t2 - b_off
    L = y_b - y_inner                                   # the shaft between its supports
    r = b['r_stat']
    w = dict(y_b=y_b, L=L, b_off=b_off)
    rows = {}
    for name in ('static', 'bump', 'corner_out', 'corner_in', 'brake', 'trac'):
        V, S, X = c[name]['V'], c[name]['S'], c[name]['X']
        # the moment of the wheel's loads about the bearing, in the vertical plane, signed: positive lifts the wheel
        # end; the side force acts at the road, r below the axis
        Mv = b_off * V - r * S if name == 'corner_out' else b_off * V + r * S
        Mh = b_off * X                                  # the force along the road, in the horizontal plane
        M = math.hypot(Mv, Mh)
        # the shaft on two supports: the inner end's reaction balances the moment about the bearing, so the bearing
        # carries the wheel's load plus (or less) that reaction
        rows[name] = dict(V=V, S=S, X=X, Mv=Mv, Mh=Mh, M=M, R_i=M / L,
                          R_b=math.hypot(V + Mv / L, X + Mh / L), Fa=S)
    w['cases'] = rows
    worst = max(rows, key=lambda k: rows[k]['M'])
    w['worst'] = worst
    # the shaft at its body, just inboard of the bearing's seat: yield under the worst bending, with the design torque
    T = duty(b)['T_shaft'] * 1000.0
    M = rows[worst]['M']
    sig = 32 * M / (math.pi * d_body ** 3)
    tau = 16 * T / (math.pi * d_body ** 3)
    w['sigma_worst'], w['tau_design'] = sig, tau
    w['vm_worst'] = math.sqrt(sig ** 2 + 3 * tau ** 2)
    w['n_yield'] = Sy / w['vm_worst']
    # fatigue: the static wheel load bends the turning shaft once a turn; the torque at full power in sixth is steady
    u = gear_duty(b)
    T_run = u['T1_cruise'] * duty(b)['i0'] * b['eta_fd'] / 2.0 * 1000.0
    Ma = rows['static']['M']
    ka = 4.51 * Sut ** -0.265                           # machined, Eq. 6-19, Table 6-2
    kb = 1.24 * d_body ** -0.107 if d_body <= 51 else 1.51 * d_body ** -0.157     # Eq. 6-20
    Se = ka * kb * 0.5 * Sut                            # Eq. 6-8 for Sut under 1400 MPa
    sa = Kt * 32 * Ma / (math.pi * d_body ** 3)
    tm = Kts * 16 * T_run / (math.pi * d_body ** 3)
    w.update(T_run=T_run, ka=ka, kb=kb, Se=Se, sa=sa, tm=tm)
    w['n_fatigue'] = 1.0 / (sa / Se + math.sqrt(3) * tm / Sut)                   # DE-Goodman, Eq. 7-7 form
    # the splines: torsion at their minor diameter at the design torque
    w['tau_spline'] = 16 * T / (math.pi * d_spline ** 3)
    w['n_spline'] = 0.577 * Sy / w['tau_spline']
    # the wheel bearing, Table 11-2: life under the static load, the turn's load against C0
    B = TABLE_11_2[d_seat]
    w['bearing'] = dict(d=d_seat, D=B[0], B=B[1], dS=B[3], dH=B[4], C10=B[5], C0=B[6])
    Rs = rows['static']['R_b']
    w['R_static'] = Rs
    w['L10_rev'] = (B[5] * 1000.0 / Rs) ** 3 * 1e6
    w['rev_needed'] = duty(b)['rev_wheel']
    co = rows['corner_out']
    Fe, e, Y, X_ = ball_equivalent(co['R_b'], co['Fa'], B[6] * 1000.0)
    w['corner_Fe'], w['corner_e'], w['corner_Y'], w['corner_X'] = Fe, e, Y, X_
    return w


def housing_tube(b, w, D=80.0, t=8.0, y_seat=575.0, Sy=270.0):
    """The banjo's tube at the spring seat: the bearing's reaction on the arm from the bearing to the seat; 1035
    hot-rolled steel (Shigley Table A-20), Sy 270 MPa."""
    d = D - 2 * t
    Z = math.pi * (D ** 4 - d ** 4) / (32 * D)
    h = dict(D=D, t=t, d=d, Z=Z, arm=w['y_b'] - y_seat)
    worst = max(w['cases'], key=lambda k: w['cases'][k]['R_b'])
    h['case'] = worst
    h['R_b'] = w['cases'][worst]['R_b']
    h['M'] = h['R_b'] * h['arm']
    h['sigma'] = h['M'] / Z
    h['n'] = Sy / h['sigma']
    c = load_cases(b)
    h['T_brake'] = c['brake']['X'] * b['r_stat']
    h['tau_brake'] = h['T_brake'] / (2 * Z)
    return h


def spline_engagement(y_tip, y_end, y_hub, T, d_minor=35.0, d_major=40.0, n=8, Sy=1140.0, share=0.5, n_min=2.0):
    """The half shaft's straight-sided splines in the side gear (Chapter 31). The side gear's splined bore runs from
    its toe out to the end of its hub, y_hub; the shaft's splines from its tip, y_tip, to y_end. Engaged: from the
    tip to the nearer of the two ends. The flanks carry T at the mean radius on the tooth's height; `share` of the
    teeth are taken to carry it (an assumption for spacing errors, not from a source), and the pressure must keep
    n_min against the shaft's yield."""
    l = min(y_end, y_hub) - y_tip
    rm = (d_minor + d_major) / 4.0
    h = (d_major - d_minor) / 2.0
    out = dict(l=l, rm=rm, h=h, share=share, n_min=n_min)
    out['l_need'] = n_min * T / (share * n * rm * h * Sy)
    out['p'] = T / (share * n * rm * h * l) if l > 0 else float('inf')
    out['n'] = Sy / out['p']
    return out


if __name__ == '__main__':
    b = brief()
    for k, v in tyre(b).items():
        print('tyre', k, round(v, 4))
    for k, v in duty(b).items():
        print('duty', k, round(v, 4))
    for k, v in load_cases(b).items():
        print('load', k, v if isinstance(v, dict) else round(v, 3))
    print('envelope', envelope(b))
    g = bevel_geometry()
    print({k: round(v, 4) for k, v in g.items()})
    u = gear_duty(b)
    print({k: round(v, 4) for k, v in u.items()})
    for name, T1, n1, N, KA in (('peak', u['T1_peak'], u['n1_peak'], u['n_peak'], 1.0),
                                ('cruise', u['T1_cruise'], u['n1_cruise'], u['n_cruise'], KA_TABLE[('light', 'light')])):
        r = rate(g, T1, n1, N, KA)
        print(name, {k: round(v, 4) for k, v in r.items()})
    for s in (+1, -1):
        f = bevel_forces(g, u['T1_peak'], s)
        print('forces', s, {k: round(v, 1) for k, v in f.items()})
