# -*- coding: utf-8 -*-
"""wrist_design.py: the pitch-roll wrist of Volume 2, Chapters 39 and 40.

A bevel differential: two side gears on the pitch axis, each driven by a toothed belt from a motor with a gearhead,
and an output gear on the roll shaft between them. The pitch carrier holds the roll shaft on two bearings and turns
on two trunnions in the frame's cheeks; each side gear runs on two bearings on its trunnion.

Axes (mm): the origin at the bevel gears' common apex, y along the pitch axis, z along the roll axis at rest (up), x
forward. Pitch phi turns the carrier about +y; roll psi turns the output on the carrier about +z; the side gears'
angles about +y are thL (the left gear, on +y, as in Chapter 31) and thR. Rolling at the mesh gives
thL = phi - psi and thR = phi + psi, so phi = (thL + thR) / 2 and psi = (thR - thL) / 2.

Values marked 'assumed' are the brief's or the designer's, not from a source. Sources: Budynas & Nisbett (2015),
Shigley's Mechanical Engineering Design, 10th ed., Chapters 13, 15 and 11; Gates Corporation (n.d.), Light Power and
Precision Drive Design Manual; SKF's product pages for the 6002-2RSH and the 61902-2RS1.
"""
import math

import axle_design as ad          # ball_equivalent: Shigley Eq. 11-12 with Table 11-1
import scara_design as sd         # MOTOR34, SF_BELT, ETA_BELT, shaft_fatigue

pi = math.pi
G = 9.81
E_STEEL = 207000.0                # MPa, Budynas & Nisbett (2015), Table A-5


def brief():
    return dict(
        m_p=3.0,                  # kg, the tool and the part together, assumed
        e_p=80.0,                 # mm, their centre beyond the tool flange's face, on the roll axis, assumed
        D_p=100.0, L_p=150.0,     # mm, taken as a solid cylinder of this size for its own inertia, assumed
        a_arm=5.0,                # m/s2, the arm's acceleration at the wrist, any direction, assumed
        pitch_max=120.0,          # deg either way, assumed
        roll_max=360.0,           # deg either way, assumed
        pitch_move=90.0, roll_move=180.0, t_move=1.0,   # deg, deg, s: the standard moves, cycloidal, together, assumed
        cycle=4.0,                # s: out, dwell, back, dwell, each a quarter, assumed
        life_h=20000.0,           # h of running, assumed
        f_min=80.0,               # Hz, the payload's lowest frequency on the wrist: four times a 20 Hz position loop, assumed
    )


def payload_inertia(b=None):
    """The payload's own moments of inertia about its centre (kg m2): across the roll axis and about it."""
    b = b or brief()
    r, L = b['D_p'] / 2000.0, b['L_p'] / 1000.0
    return b['m_p'] * (3 * r ** 2 + L ** 2) / 12.0, b['m_p'] * r ** 2 / 2.0


def cycloid(h, T, t):
    """Position, speed and acceleration of a cycloidal move of h in time T at time t (0 <= t <= T)."""
    x = 2 * pi * t / T
    return h * (t / T - math.sin(x) / (2 * pi)), h / T * (1 - math.cos(x)), 2 * pi * h / T ** 2 * math.sin(x)


def moves(b=None):
    b = b or brief()
    hp, hr, T = math.radians(b['pitch_move']), math.radians(b['roll_move']), b['t_move']
    return dict(hp=hp, hr=hr, T=T, wp=2 * hp / T, wr=2 * hr / T, ap=2 * pi * hp / T ** 2, ar=2 * pi * hr / T ** 2,
                n_in=(2 * hp / T + 2 * hr / T) * 60 / (2 * pi))


# =============================================================================
# The bevel set, by Shigley (Budynas & Nisbett, 2015), Chapters 13 and 15
# =============================================================================
MODULES = [1.0, 1.25, 1.5, 2.0, 2.5, 3.0]        # Table 13-2, preferred modules (p. 689)
BEARINGS = {   # SKF product pages, read 2 November 2026 (notes in the references folder)
    '6002-2RSH': dict(d=15.0, D=32.0, B=9.0, C=5850.0, C0=2850.0, nlim=14000.0, mass=0.0298),
    '61902-2RS1': dict(d=15.0, D=28.0, B=7.0, C=4360.0, C0=2240.0, nlim=16000.0, f0=14.3),
}
WALL = 3.0           # mm, the carrier's wall beside the roll bearing's seat (7075-T6), assumed
FLANGE_T = 3.0       # mm, the trunnion's flange, assumed
SHIM = 0.5           # mm, the least shim space in front of each side gear, assumed
CLEAR = 1.5          # mm, the carrier's faces to the gears' front faces, assumed


def bevel(m=1.5, z=40, b=11.0, alpha=20.0):
    """A miter set of straight bevel gears in Shigley's proportions (Table 13-3, p. 689): working depth 2m, clearance
    0.188m + 0.05 mm, the addendum m for a 1:1 ratio. Heel = the large end, toe = the small end; the toe's section is
    the heel's scaled about the apex, so every tooth line runs to it."""
    g = dict(m=m, z=z, b=b, alpha=alpha)
    d_ = math.radians(45.0)
    g['delta'] = 45.0
    g['d'] = m * z
    g['R'] = g['d'] / (2 * math.sin(d_))
    g['k'] = (g['R'] - b) / g['R']
    g['ha'] = m
    g['c'] = 0.188 * m + 0.05
    g['hf'] = 2 * m - g['ha'] + g['c']
    g['ht'] = 2 * m + g['c']
    s, c = math.sin(d_), math.cos(d_)
    g['heel_tip'] = (g['R'] * s + g['ha'] * c, g['R'] * c - g['ha'] * s)
    g['heel_root'] = (g['R'] * s - g['hf'] * c, g['R'] * c + g['hf'] * s)
    g['toe_root'] = (g['k'] * g['heel_root'][0], g['k'] * g['heel_root'][1])
    g['zff'] = g['toe_root'][1]                    # the front face, normal to the gear's axis, through the toe's root
    g['da'] = g['d'] + 2 * g['ha'] * c
    g['Rm'] = g['R'] - b / 2
    g['rm'] = g['Rm'] * s                          # mean pitch radius
    g['b_max'] = min(0.3 * g['R'], 10 * m)         # Table 13-3: 0.3 A0 or 10/P, whichever is smaller
    g['zv'] = z / c                                # Tredgold's virtual spur gear
    rv = g['R'] * math.tan(d_)
    a = math.radians(alpha)
    rb, ra = rv * math.cos(a), rv + g['ha']
    g['eps'] = (2 * math.sqrt(ra ** 2 - rb ** 2) - 2 * rv * math.sin(a)) / (pi * m * math.cos(a))
    g['theta_f'] = math.degrees(math.atan(g['hf'] / g['R']))
    return g


def space():
    """What the toe must leave inside: the carrier between the side gears holds the roll shaft's 6002 in a seat
    with walls of WALL, the trunnion's flange of FLANGE_T and a shim space of SHIM in front of each side gear. The
    side gear's front face must be at least zff_min from the apex."""
    D = BEARINGS['6002-2RSH']['D']
    zff_min = D / 2 + WALL + FLANGE_T + SHIM
    return dict(D=D, half=D / 2 + WALL, zff_min=zff_min)


def toe_needed(m):
    """The toe's cone distance R - b that puts the front face at zff_min, for module m (the dedendum depends on it):
    zff = (R - b) (cos d + hf sin d / R); solved for R - b by iteration with R = (R - b) / 0.75."""
    sp = space()
    hf = m + 0.188 * m + 0.05
    Ri = sp['zff_min'] / math.cos(math.radians(45))
    for _ in range(50):
        R = Ri / 0.75
        Ri = sp['zff_min'] / (math.cos(math.radians(45)) + hf * math.sin(math.radians(45)) / R)
    return Ri


def candidates(face=0.25):
    """Pitch diameters that leave the toe its room with a face of a quarter of the cone distance (the least the
    design takes, assumed): R >= Ri / (1 - face); the preferred modules and whole tooth numbers just above."""
    rows = []
    for m in MODULES:
        Ri = toe_needed(m)
        R_min = Ri / (1 - face)
        d_min = R_min * math.sqrt(2)
        z = math.ceil(d_min / m - 1e-9)
        g = bevel(m, z, 1.0)
        b_room = g['R'] - Ri
        rows.append(dict(m=m, Ri=Ri, d_min=d_min, z=z, d=m * z, R=g['R'], b_room=b_room, b_rule=min(0.3 * g['R'], 10 * m)))
    return rows


# --- Shigley Chapter 15, SI forms (pp. 783-792)
ZE = 190.0                    # sqrt(MPa), steel on steel (Eq. 15-1's elastic coefficient)
ZI_40 = 0.0695                # Fig. 15-6, 40 teeth with a mate of 40, read on a zoomed crop
YJ_40 = 0.248                 # Fig. 15-7, the same
HB = 300.0                    # through-hardened 42CrMo4, quenched and tempered, assumed
QV = 8                        # transmission accuracy number, assumed


def sH_lim(hb=HB):
    return 2.35 * hb + 162.89     # Eq. 15-22, grade 1, MPa


def sF_lim(hb=HB):
    return 0.30 * hb + 14.48      # Eq. 15-23, grade 1, MPa


def kv(d, n, Qv=QV):
    """Eqs. 15-5 to 15-8: v_et the pitch-line speed at the heel, m/s, d in mm, n in rev/min."""
    v = 5.236e-5 * d * n
    B = 0.25 * (12 - Qv) ** (2.0 / 3.0)
    A = 50 + 56 * (1 - B)
    return ((A + math.sqrt(200 * v)) / A) ** B, v, A, B


def rate(g, T, n_rel, cycles, KA=1.25, Kmb=1.10, reversed_=True, credit=True, dynamic=True):
    """The side gear's contact and bending stresses and their safety factors (Eqs. 15-1 to 15-4). T the side gear's
    torque (N m); n_rel its speed on the carrier (rev/min); cycles its teeth's load cycles. Z_NT is taken as 1: Fig.
    15-8 is drawn for case-hardened gears. Reversed loading at 70 % of the bending allowable (p. 792)."""
    r = dict(T=T)
    r['Wt'] = 2 * T / g['d']                                # kN at the heel's pitch circle (T in N m, d in mm)
    r['Kv'], r['v'], r['A'], r['B'] = kv(g['d'], n_rel) if dynamic else (1.0, 0.0, 0.0, 0.0)
    r['KA'] = KA
    r['Zx'] = 0.00492 * g['b'] + 0.4375 if g['b'] >= 12.7 else 0.5      # Eq. 15-9
    r['Yx'] = 0.4867 + 0.008339 * g['m'] if g['m'] >= 1.6 else 0.5      # Eq. 15-10
    r['KHb'] = Kmb + 5.6e-6 * g['b'] ** 2                                # Eq. 15-11
    r['Zxc'] = 1.5                                                       # Eq. 15-12, crowned teeth
    r['sH'] = ZE * math.sqrt(1000 * r['Wt'] / (g['b'] * g['d'] * ZI_40) * KA * r['Kv'] * r['KHb'] * r['Zx'] * r['Zxc'])
    r['sF'] = 1000 * r['Wt'] / g['b'] * KA * r['Kv'] / g['m'] * r['Yx'] * r['KHb'] / (1.0 * YJ_40)
    r['YNT'] = 1.683 * cycles ** -0.0323 if credit else 1.0             # Fig. 15-9 (Eq. 15-15)
    r['rev'] = 0.7 if reversed_ else 1.0
    r['sHP'] = sH_lim()
    r['sFP'] = sF_lim() * r['YNT'] * r['rev']
    r['SH'] = r['sHP'] / r['sH']
    r['SF'] = r['sFP'] / r['sF']
    return r


def mesh_forces(g, T):
    """Shigley Eq. 13-38 at the mean radius: tangential, radial (toward the gear's axis) and axial (away from the
    apex), N, for a side gear torque T (N m)."""
    Wt = T * 1000.0 / g['rm']
    a = math.radians(g['alpha'])
    return dict(Wt=Wt, Wr=Wt * math.tan(a) * math.cos(math.radians(45)), Wa=Wt * math.tan(a) * math.sin(math.radians(45)))


GEAR = dict(m=1.5, z=40, b=11.0)      # the set chosen in Chapter 39


# =============================================================================
# The layout: every axial position, from the gear set and the bearings (mm)
# =============================================================================
def layout(gear=None):
    g = bevel(**(gear or GEAR))
    sp = space()
    L = dict(g=g)
    zff = g['zff']
    # --- the pitch axis, |y| outward from the apex
    L['y_cs'] = sp['half']                       # the carrier's side face
    L['y_fl'] = (L['y_cs'], L['y_cs'] + FLANGE_T)   # the trunnion's flange
    L['S1'] = (L['y_fl'][1], zff)                # shim space and spacer S1
    L['B1'] = (zff, zff + 7.0)                   # 61902 under the teeth
    L['pulley'] = (34.0, 50.0)                   # the side gear's 36-groove teeth, 16 wide for a 15 mm belt
    L['belt'] = (34.5, 49.5)
    L['B2'] = (L['pulley'][1] - 7.0, L['pulley'][1])   # 61902 inside the pulley's hub
    L['S2'] = (L['B1'][1], L['B2'][0])
    L['cheek'] = (L['pulley'][1] + 2.0, L['pulley'][1] + 14.0)
    L['lip'] = (L['cheek'][0], L['cheek'][0] + 1.5)     # the cheek's lip that locates the 6002 inward
    L['B3'] = (L['lip'][1], L['lip'][1] + 9.0)          # the frame's 6002
    L['S3'] = (L['B2'][1], L['B3'][0])
    L['tr_end'] = L['B3'][1]                     # the trunnion's outer end; the washer and the M5 screw beyond it
    L['tr_spigot'] = 13.0                        # the trunnion's spigot from |y| 13 into the carrier
    # --- the roll axis, z
    L['z_ct'] = zff - CLEAR                      # the carrier's top face
    L['groove'] = (L['z_ct'] - 1.5 - 1.3, L['z_ct'] - 1.5)   # DIN 472-32 ring's groove, 1.3 wide, 1.5 from the top
    L['B4'] = (L['groove'][0] - 9.0, L['groove'][0])          # the fixed (upper) 6002
    L['S4'] = (L['B4'][1], zff)
    L['B5'] = (-21.0, -12.0)                     # the floating (lower) 6002
    L['z_cb'] = L['B5'][0] - 1.5                 # the carrier's bottom face
    L['collar'] = (L['B5'][1], L['B4'][0])       # the roll shaft's collar, 20 mm, between the bearings
    L['ring471'] = (L['B5'][0] - 1.1, L['B5'][0])
    L['shaft_bot'] = L['ring471'][0] - 1.4
    L['gear_back'] = 36.0                        # the output gear's back face; the hub to 40 (the side gear: the pulley)
    L['hub'] = (L['gear_back'], 40.0)
    L['flange'] = (40.0, 51.0)                   # the tool flange, 45 mm, 11 thick
    L['shaft_top'] = 44.0
    L['h_f'] = L['flange'][1]
    L['spread'] = sum(L['B4']) / 2 - sum(L['B5']) / 2
    L['a_over'] = L['h_f'] - sum(L['B4']) / 2    # the upper bearing's middle to the flange's face
    L['key'] = (26.0, 44.0)                      # DIN 6885 A 5 x 5 x 18
    # --- the motors: side by side under the carrier, each in its own belt's plane, its body across the frame
    bt = belt()
    L['C'] = bt['C']
    L['x_m'] = MOTOR_X
    L['z_m'] = -math.sqrt(bt['C'] ** 2 - MOTOR_X ** 2)
    L['gh_face'] = 22.0                          # |y| of the gearhead's face; its body and the motor's run inward
    L['bracket'] = (22.0, 26.5)
    L['m_hub'] = (27.0, 33.0)                    # the small pulley's hub (set screws), then its flanges and teeth
    L['m_flanges'] = ((33.0, 34.0), (50.0, 51.0))
    L['m_shaft'] = (L['gh_face'] - 5.0, L['gh_face'] + GEARHEAD['shaft_L'])   # from the gearhead's body, through its flange
    L['m_end'] = L['gh_face'] - GEARHEAD['L'] - 5.0 - MOTOR39['L_m'] - MOTOR39['L_br']
    L['base'] = (-144.0, -132.0)                 # the base plate's underside and top
    L['tie'] = (52.0, -60.0, 8.0)                # the tie bars: x, z, radius
    return L


# =============================================================================
# Masses of the moving groups (kg, mm, kg m2): estimates, then the model's (Chapter 39's check)
# =============================================================================
MASS_EST = dict(roll=0.45, roll_z=25.0, roll_Iz=0.00003, roll_Ix=0.0004,      # the roll shaft, output gear, flange (assumed)
                carrier=0.40, carrier_z=0.0, carrier_Iy=0.0001,              # block, trunnions, spacers, roll bearings (assumed)
                side=0.45, side_Iy=0.00016)                                  # each side gear with its pulley (assumed)
# measured on the model by checks/wrist_check.py (WristDifferential 1.0): the groups at rest, about the apex
MASS_MEAS = dict(roll=0.4565017, roll_z=28.0288096, roll_Iz=0.0001219, roll_Ix=0.0001883,
                 carrier=0.3389319, carrier_z=0.2120428, carrier_Iy=5.32e-05, side=0.3955012, side_Iy=0.0001719)


def masses(measured=None):
    """The groups' masses: the model's (MASS_MEAS) unless others are given; MASS_EST was the first estimate."""
    return dict(MASS_MEAS, **(measured or {}))


def loads(b=None, M=None):
    """The wrist's equations of motion from its kinetic energy (Chapter 39): with thL = phi + psi and thR = phi - psi,
    the side gears add 2 I_s to the inertias of both pitch and roll. The pitch torque carries the static moment of
    everything above the pitch axis at g plus the arm's acceleration, taken at its worst, all the time."""
    b = b or brief()
    M = masses(M)
    L = layout()
    Jc, Ja = payload_inertia(b)
    r_p = (L['h_f'] + b['e_p']) / 1000.0
    g_eff = G + b['a_arm']
    o = dict(r_p=r_p, g_eff=g_eff, Jc=Jc, Ja=Ja)
    o['J_P'] = b['m_p'] * r_p ** 2 + Jc + M['roll'] * (M['roll_z'] / 1000) ** 2 + M['roll_Ix'] + M['carrier_Iy']
    o['J_R'] = Ja + M['roll_Iz']
    o['J_P2'] = o['J_P'] + 2 * M['side_Iy']
    o['J_R2'] = o['J_R'] + 2 * M['side_Iy']
    o['S'] = b['m_p'] * r_p + M['roll'] * M['roll_z'] / 1000 + M['carrier'] * M['carrier_z'] / 1000   # kg m
    o['Tg'] = o['S'] * g_eff
    mv = moves(b)
    o['Tp_acc'] = o['J_P2'] * mv['ap']
    o['Tr_acc'] = o['J_R2'] * mv['ar']
    o['Tp'] = o['Tg'] + o['Tp_acc']
    o['Tr'] = o['Tr_acc']
    o['T_in'] = (o['Tp'] + o['Tr']) / 2          # the larger input's peak
    o['T_hold'] = o['Tg'] / 2                    # each input, holding
    return o


def timeline(b=None, M=None, n=4000):
    """Rows over the cycle: t, dt, phi'', psi'' (rad/s2), phi', psi' (rad/s), and the two inputs' torques (N m):
    tau_L = (tau_p - tau_r) / 2 and tau_R = (tau_p + tau_r) / 2, from the power tau_L thL' + tau_R thR'."""
    b = b or brief()
    o = loads(b, M)
    mv = moves(b)
    T, C = b['t_move'], b['cycle']
    rows = []
    for i in range(n):
        t = C * (i + 0.5) / n
        wp = wr = ap = ar = 0.0
        for t0, sg in ((0.0, 1), (C / 2, -1)):
            if t0 <= t < t0 + T:
                _, wp, ap = cycloid(mv['hp'], T, t - t0)
                _, wr, ar = cycloid(mv['hr'], T, t - t0)
                wp, ap, wr, ar = sg * wp, sg * ap, sg * wr, sg * ar
        tp = o['Tg'] + o['J_P2'] * ap
        tr = o['J_R2'] * ar
        rows.append((t, C / n, ap, ar, wp, wr, (tp - tr) / 2, (tp + tr) / 2))
    return rows


# =============================================================================
# The motors, the gearheads and the belts
# =============================================================================
MOTOR39 = dict(rated=0.16, peak=0.48, n_rated=3000.0, nmax=6000.0, Jm=sd.MOTOR34[4]['Jm'],   # a 50 W servo, assumed
               flange=40.0, L_m=60.0, L_br=25.0, brake=0.32, mass=0.45)
GEARHEAD = dict(i=15.0, eta=0.85, J_in=0.004e-4, L=45.0, D=40.0, shaft=8.0, shaft_L=25.0, mass=0.30)   # assumed
PULLEYS39 = (18, 36)                  # grooves: the gearhead's, the side gear's
BELT39 = dict(width=15.0, pitch=5.0)
I_BELT = PULLEYS39[1] / PULLEYS39[0]
LIMIT = 0.35                          # N m, the drives' torque (current) limit at the motor, assumed
MOTOR_X = 26.0                        # mm, each motor's axis either side of the middle: their 40 mm flanges 12 mm apart, assumed
HTD5M_18 = [(200, 2.9), (300, 2.6), (400, 2.5), (500, 2.3), (600, 2.2), (700, 2.1), (800, 2.1), (870, 2.0), (1000, 2.0)]
HTD5M_WIDTH = {9.0: 1.00, 15.0: 1.89, 25.0: 3.38}                 # Gates p. 43
HTD5M_LENGTH = [(435.0, 0.80), (550.0, 0.90), (840.0, 1.00)]      # p. 43, by the belt's pitch length (from 350 mm)
HTD5M_STOCK = [350, 375, 400, 425, 450, 460, 475, 480, 495, 500, 520]   # p. 40, stock lengths in this range
T8_5M15 = 19.0 * 4.4482               # N per span, Table 8 (p. 65): HTD 5M, 15 mm, general value
TMIN_5M15 = 13.5 * 4.4482             # N per span, Table 7 (p. 65): the least static tension


def pd5(z):
    return 5.0 * z / pi


def motors(b=None, M=None, i_g=None, eta_g=None):
    """Each motor's torque and speed over the cycle: its input's torque through the belt and the gearhead, the losses
    against the motor both ways, plus its rotor's, the gearhead's and the small pulley's inertia at the motor's
    acceleration (the pulley's is small and taken in the gearhead's)."""
    b = b or brief()
    i_g = i_g or GEARHEAD['i']
    eta_g = eta_g or GEARHEAD['eta']
    i = i_g * I_BELT
    eta = eta_g * sd.ETA_BELT
    Jm = MOTOR39['Jm'] + GEARHEAD['J_in']
    rows = timeline(b, M)
    pk = rms2 = 0.0
    nmax = 0.0
    tot = 0.0
    for t, dt, ap, ar, wp, wr, tL, tR in rows:
        for tin, a_in, w_in in ((tL, ap - ar, wp - wr), (tR, ap + ar, wp + wr)):
            tm = tin / (i * eta) + Jm * a_in * i
            pk = max(pk, abs(tm))
            rms2 += dt * tm ** 2
            nmax = max(nmax, abs(w_in) * i * 60 / (2 * pi))
        tot += dt
    o = loads(b, M)
    hold = o['T_hold'] / (i * eta)
    return dict(i=i, eta=eta, Jm=Jm, peak=pk, rms=math.sqrt(rms2 / (2 * tot)), nmax=nmax, hold=hold,
                brake_need=2.0 * o['T_hold'] / i,          # with the power off: the efficiency taken as 1, a factor of 2
                stall_in=LIMIT * i * eta)                  # the side gear's torque at the drive's limit


def belt(C_try=None):
    """Gates' procedure (pp. 12, 103, 104) for the 18/36 drive: the stock length, its centre distance, the arc and
    the teeth in mesh, the rating, and the static tension (p. 65)."""
    D, d_ = pd5(PULLEYS39[1]), pd5(PULLEYS39[0])

    def length(C):
        phi = math.asin((D - d_) / (2 * C))
        return 2 * C * math.cos(phi) + pi * (D + d_) / 2 + phi * (D - d_)
    stock = 350
    lo, hi = 60.0, 200.0
    for _ in range(100):
        C = (lo + hi) / 2
        lo, hi = (C, hi) if length(C) < stock else (lo, C)
    arc = 180 - 60 * (D - d_) / C
    tim = PULLEYS39[0] * arc / 360
    return dict(D=D, d=d_, C=C, stock=stock, teeth=stock // 5, arc=arc, tim=tim, L_check=length(C))


def belt_rating(T_small, n_small):
    row = next(r for r in HTD5M_18 if r[0] >= n_small)
    lf = next(f for Lmax, f in HTD5M_LENGTH if belt()['stock'] <= Lmax)
    rating = row[1] * HTD5M_WIDTH[BELT39['width']] * lf
    design = T_small * sd.SF_BELT
    d_ = pd5(PULLEYS39[0]) / 1000
    Q = T_small
    Tst1 = 1.21 * Q / d_                      # Formula 1 (p. 65), the belt speed's term negligible here
    Tst = max(Tst1, TMIN_5M15, 2.0 * T8_5M15)   # a registration drive: Table 8 times 1.5 to 2.0, the top taken
    return dict(row=row, lf=lf, rating=rating, design=design, ratio=rating / design, Te=2 * Q / d_,
                Tst1=Tst1, Tmin=TMIN_5M15, T8=T8_5M15, Tst=Tst, shaft=2 * Tst)


# =============================================================================
# Shafts and bearings
# =============================================================================
def roll_shaft(d=15.0, b=None, L=None):
    """The roll shaft as an overhanging beam on its two bearings (simple supports at their middles), loaded at the
    flange's face by the payload's force F and its moment F e; the flange and the gear's hub taken as adding no
    stiffness. Deflection at the payload's centre per newton; the frequency of the payload on that spring."""
    b = b or brief()
    L = L or layout()
    s, a, e = L['spread'], L['a_over'], b['e_p']
    I = pi * d ** 4 / 64
    EI = E_STEEL * I
    y_P = a ** 2 * (s + a) / (3 * EI)
    th_P = a * (2 * s + 3 * a) / (6 * EI)
    y_M = e * a * (2 * s + 3 * a) / (6 * EI)
    th_M = e * (s + 3 * a) / (3 * EI)
    delta = y_P + y_M + (th_P + th_M) * e
    k = 1.0 / delta                       # N/mm
    f = math.sqrt(k * 1000 / b['m_p']) / (2 * pi)
    return dict(d=d, I=I, s=s, a=a, e=e, y_P=y_P, th_P=th_P, y_M=y_M, th_M=th_M, delta=delta, k=k, f=f)


def roll_bearings(b=None, M=None):
    """Reactions on the roll shaft's bearings (N): the payload's force across the axis, m_p (g + a_arm) plus its
    share of the pitch move (r alpha and r omega2), at e beyond the flange; the two meshes' tangential forces in the
    same direction (the pitch torque's case) at the mean cone point's height; the axial load on the upper (fixed)
    bearing: both meshes' axial forces and the payload's weight along the axis."""
    b = b or brief()
    L = layout()
    o = loads(b, M)
    mv = moves(b)
    g = L['g']
    F_p = b['m_p'] * (o['g_eff'] + o['r_p'] * mv['ap'] + o['r_p'] * mv['wp'] ** 2)
    f = mesh_forces(g, o['T_in'])
    z4, z5 = sum(L['B4']) / 2, sum(L['B5']) / 2
    zp = L['h_f'] + b['e_p']
    zm = g['Rm'] * math.cos(math.radians(45))
    Fx = F_p + 2 * f['Wt']
    Mx = F_p * (zp - z5) + 2 * f['Wt'] * (zm - z5)          # moments about the lower bearing
    R4 = Mx / (z4 - z5)
    R5 = R4 - Fx
    Fa = 2 * f['Wa'] + b['m_p'] * o['g_eff']
    return dict(F_p=F_p, mesh=f, z4=z4, z5=z5, zp=zp, zm=zm, R4=R4, R5=R5, Fa=Fa)


def side_bearings(b=None, M=None):
    """One side gear's two 61902s (N): the mesh forces at the mean cone point, the belt's pull at the belt's middle
    toward the motor (taken straight down, the worst for B2 and the cheek), B2 taking the outward axial force."""
    b = b or brief()
    L = layout()
    o = loads(b, M)
    g = L['g']
    f = mesh_forces(g, o['T_in'])
    br = belt_rating(o['T_in'] / (I_BELT * sd.ETA_BELT), 0)
    y1, y2 = sum(L['B1']) / 2, sum(L['B2']) / 2
    ym = g['Rm'] * math.cos(math.radians(45))
    yb = sum(L['belt']) / 2
    # x: Wt at ym; z: -Wr at ym (toward the axis from the mesh above), -F_belt at yb
    Fx1 = f['Wt'] * (y2 - ym) / (y2 - y1)
    Fx2 = f['Wt'] - Fx1
    Fz = [-f['Wr'], -br['shaft']]
    ys = [ym, yb]
    Mz = sum(F * (y2 - y) for F, y in zip(Fz, ys))
    Fz1 = Mz / (y2 - y1)
    Fz2 = sum(Fz) - Fz1
    R1 = math.hypot(Fx1, Fz1)
    R2 = math.hypot(Fx2, Fz2)
    return dict(mesh=f, belt=br, y1=y1, y2=y2, ym=ym, yb=yb, R1=R1, R2=R2, Fa2=f['Wa'], Fz1=Fz1, Fz2=Fz2, Fx1=Fx1, Fx2=Fx2)


def frame_bearings(b=None, M=None):
    """The cheeks' 6002s (N): the pitching assembly's weight and inertia at g + a_arm shared equally, and each side's
    belt pull through its trunnion: the trunnions and the carrier as one beam between the two frame bearings."""
    b = b or brief()
    L = layout()
    M_ = masses(M)
    o = loads(b, M)
    sb = side_bearings(b, M)
    m_tot = b['m_p'] + M_['roll'] + M_['carrier'] + 2 * M_['side']
    W = m_tot * o['g_eff']
    y3 = sum(L['B3']) / 2
    F_b = sb['belt']['shaft']
    near = F_b * (y3 + sb['yb']) / (2 * y3)
    far = F_b - near
    R3 = W / 2 + near + far            # the two belts pull the same way: each bearing takes its own near share and the other's far one
    return dict(m_tot=m_tot, W=W, y3=y3, near=near, far=far, R3=R3)


def life(Fr, Fa, name, revs):
    """SKF's basic rating life in revolutions against the revolutions the brief needs; the static factor at Fr."""
    bg = BEARINGS[name]
    P = Fr if Fa == 0 else ad.ball_equivalent(Fr, Fa, bg['C0'] / 1000.0 * 1000.0)[0]
    L10 = (bg['C'] / P) ** 3 * 1e6
    return dict(P=P, L10=L10, need=revs, ratio=L10 / revs, s0=bg['C0'] / max(P, 1e-9))


def trunnion(b=None, M=None, d=15.0):
    """Bending at the trunnion's flange (|y| = y_fl[1]): the frame bearing's reaction less the side gear's bearing
    loads outboard of the section; then the endurance (shaft_fatigue) for reversed bending as the carrier pitches
    under the fixed belt pull."""
    L = layout()
    sb = side_bearings(b, M)
    fb = frame_bearings(b, M)
    y0 = L['y_fl'][1]
    Mz = fb['R3'] * (fb['y3'] - y0) - abs(sb['Fz1']) * (sb['y1'] - y0) - abs(sb['Fz2']) * (sb['y2'] - y0)
    sigma = 32 * abs(Mz) / (pi * d ** 3)
    return dict(M=Mz, sigma=sigma, fat=sd.shaft_fatigue(sigma, d))


def report():
    b = brief()
    print('moves', moves(b))
    print('space', space())
    for r in candidates():
        print('cand', {k: round(v, 3) if isinstance(v, float) else v for k, v in r.items()})
    L = layout()
    g = L['g']
    print('gear', {k: (round(v, 3) if isinstance(v, float) else v) for k, v in g.items() if not isinstance(v, tuple)})
    print('layout', {k: v for k, v in L.items() if k != 'g'})
    o = loads(b)
    print('loads', {k: round(v, 5) for k, v in o.items()})
    mo = motors(b)
    print('motors', {k: round(v, 5) for k, v in mo.items()})
    mv = moves(b)
    ra = rate(g, o['T_in'], mv['wr'] * 60 / (2 * pi), 1.8e7)
    print('duty', {k: round(v, 4) for k, v in ra.items()})
    rs = rate(g, mo['stall_in'], 0, 1e3, KA=1.0, credit=False, dynamic=False)
    print('stall', {k: round(v, 4) for k, v in rs.items()})
    print('belt', belt())
    print('belt rating', belt_rating(o['T_in'] / (I_BELT * sd.ETA_BELT), 180))
    print('roll shaft', roll_shaft())
    for d in (10.0, 12.0):
        print('roll shaft', d, round(roll_shaft(d)['f'], 1))
    print('roll bearings', roll_bearings())
    print('side bearings', side_bearings())
    print('frame bearings', frame_bearings())
    print('trunnion', trunnion())
    cyc = b['life_h'] * 3600 / b['cycle']
    rb, sb, fb = roll_bearings(), side_bearings(), frame_bearings()
    print('cycles', cyc)
    print('life B4', life(rb['R4'], rb['Fa'], '6002-2RSH', cyc))
    print('life B5', life(rb['R5'], 0.0, '6002-2RSH', cyc))
    print('life B1', life(sb['R1'], 0.0, '61902-2RS1', cyc))
    print('life B2', life(sb['R2'], sb['Fa2'], '61902-2RS1', cyc))
    print('life B3', life(fb['R3'], 0.0, '6002-2RSH', cyc / 2))


if __name__ == '__main__':
    report()
