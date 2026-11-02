# -*- coding: utf-8 -*-
"""scara_design.py: the SCARA robot of Volume 2, Chapters 35 to 38.

The brief (values marked 'assumed' are the brief's, not from a source), the arm's geometry and workspace, the
standard cycle and the joints' motions, the masses taken for a first estimate, and the joints' torques from the
two-link arm's equations of motion, derived in Chapter 35 from its kinetic energy.

Axes: the base's axis is z, up; J1 turns the first arm about it; J2 turns the second arm about a parallel axis at
the first arm's end; J3 moves the spline shaft along z at the second arm's end, J4 turns it.
"""
import math

pi = math.pi
G = 9.81


def brief():
    return dict(
        task='pick and place of small parts in assembly',       # assumed
        L1=350.0, L2=250.0,                                     # mm, the arms' lengths, assumed
        stroke=200.0,                                           # mm, J3, assumed
        q1max=132.0, q2max=150.0,                               # deg, J1 and J2 either way, assumed
        q4max=360.0,                                            # deg, J4 either way, assumed
        m_rated=2.0, m_max=5.0,                                 # kg, payload, assumed
        m_tool=0.5,                                             # kg, the gripper, assumed
        cycle=0.50,                                             # s, the standard cycle with the rated payload, assumed
        up=25.0, across=300.0,                                  # mm, the standard cycle's legs, assumed (common practice)
        t_h=0.20, t_v=0.10,                                     # s, a horizontal leg; a vertical leg, overlapping, assumed
        t_r=0.25,                                               # s, a radial move of 300 mm, assumed
        t4=0.20, q4_move=180.0, k4=30.0,                        # s, deg: J4's turn in the cycle; the part's radius of gyration, mm, assumed
        rep_xy=0.02, rep_z=0.01,                                # mm, repeatability, assumed
        life_h=20000.0,                                         # h of running, assumed
        x0=400.0,                                               # mm, the line of the cycle's horizontal leg, assumed
    )


# the masses taken for a first estimate (kg, mm), each to be checked against Fusion in Chapter 38 (assumed)
MASS = dict(
    arm1=3.5, arm1_c=175.0,          # the first arm's structure, its centre of mass from J1
    j2=3.0,                          # the J2 module at the elbow: motor, reducer, bearing, housing
    arm2=2.0, arm2_c=125.0,          # the second arm's structure, from J2
    j34_motors=2.0, j34_c=80.0,      # J3 and J4 motors on the second arm, from J2
    spline=1.5,                      # the ball screw spline's shaft and nuts at the second arm's end
)


def ik(b, x, y, elbow=1):
    """The two-link arm's inverse kinematics (mm, rad): q2 from the law of cosines, q1 from the two angles."""
    L1, L2 = b['L1'], b['L2']
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1:
        raise ValueError('out of reach')
    q2 = elbow * math.acos(c2)
    q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
    return q1, q2


def fk(b, q1, q2):
    L1, L2 = b['L1'], b['L2']
    return L1 * math.cos(q1) + L2 * math.cos(q1 + q2), L1 * math.sin(q1) + L2 * math.sin(q1 + q2)


def workspace(b):
    """The reach, the inner radius with the arm folded to the J2 limit, and the area the tool covers."""
    L1, L2 = b['L1'], b['L2']
    r_in = math.sqrt(L1 ** 2 + L2 ** 2 + 2 * L1 * L2 * math.cos(math.radians(b['q2max'])))
    r_out = L1 + L2
    # at radius r the two elbows cover arcs of 2 q1max centred on +-beta; the union of the two arcs, at most 2 pi
    q1 = math.radians(b['q1max'])
    n = 2000
    area = 0.0
    for i in range(n):
        r = r_in + (r_out - r_in) * (i + 0.5) / n
        cb = (L1 ** 2 + r ** 2 - L2 ** 2) / (2 * L1 * r)
        beta = math.acos(max(-1.0, min(1.0, cb)))
        span = min(2 * pi, 2 * q1 + 2 * beta) if 2 * beta < 2 * q1 else min(2 * pi, 4 * q1)
        area += r * span * (r_out - r_in) / n
    return dict(r_in=r_in, r_out=r_out, area=area)


def inertia(b, payload):
    """The arm's mass properties for the equations of motion, SI units: link 1 (its structure and the J2 module at
    its end), link 2 (its structure, the J3 and J4 motors) and the end mass at the second arm's tip (spline, tool,
    payload)."""
    M = MASS
    L1, L2 = b['L1'] / 1000, b['L2'] / 1000
    I1 = M['arm1'] * L1 ** 2 / 12 + M['arm1'] * (M['arm1_c'] / 1000) ** 2 + M['j2'] * L1 ** 2   # about J1
    m2 = M['arm2'] + M['j34_motors']
    c2 = (M['arm2'] * M['arm2_c'] + M['j34_motors'] * M['j34_c']) / m2 / 1000
    I2c = M['arm2'] * L2 ** 2 / 12 + M['arm2'] * (M['arm2_c'] / 1000 - c2) ** 2 + M['j34_motors'] * (M['j34_c'] / 1000 - c2) ** 2
    m3 = M['spline'] + b['m_tool'] + payload
    return dict(L1=L1, L2=L2, I1=I1, m2=m2, c2=c2, I2c=I2c, m3=m3)


def dynamics(p, q2, qd1, qd2, qdd1, qdd2):
    """The joints' torques of the two-link arm turning in a horizontal plane (no gravity in the plane), from its
    kinetic energy (Chapter 35): link 1 turns about J1 with inertia I1; link 2 of mass m2, its centre c2 from J2 and
    inertia I2c about it; the end mass m3 at L2."""
    L1, L2, m2, c2, m3 = p['L1'], p['L2'], p['m2'], p['c2'], p['m3']
    a = p['I1'] + (m2 + m3) * L1 ** 2 + p['I2c'] + m2 * c2 ** 2 + m3 * L2 ** 2
    bq = (m2 * c2 + m3 * L2) * L1
    d = p['I2c'] + m2 * c2 ** 2 + m3 * L2 ** 2
    M11 = a + 2 * bq * math.cos(q2)
    M12 = d + bq * math.cos(q2)
    M22 = d
    h = bq * math.sin(q2)
    t1 = M11 * qdd1 + M12 * qdd2 - h * (2 * qd1 * qd2 + qd2 ** 2)
    t2 = M12 * qdd1 + M22 * qdd2 + h * qd1 ** 2
    return t1, t2, dict(M11=M11, M12=M12, M22=M22, h=h)


def cycloid(dq, T, t):
    """A cycloidal move of dq in T: position, speed and acceleration at t."""
    s = t / T
    return (dq * (s - math.sin(2 * pi * s) / (2 * pi)), dq / T * (1 - math.cos(2 * pi * s)),
            2 * pi * dq / T ** 2 * math.sin(2 * pi * s))


def move(b, A, B, payload, T=None, n=200, elbow=1):
    """A point-to-point move in joint space from A to B (mm), both joints cycloidal over T: the largest speeds,
    accelerations and torques."""
    T = T or b['t_h']
    p = inertia(b, payload)
    qa, qb = ik(b, *A, elbow=elbow), ik(b, *B, elbow=elbow)
    d1, d2 = qb[0] - qa[0], qb[1] - qa[1]
    out = dict(qa=qa, qb=qb, d1=d1, d2=d2, w1=0.0, w2=0.0, a1=0.0, a2=0.0, t1=0.0, t2=0.0, P1=0.0, P2=0.0)
    for i in range(n + 1):
        t = T * i / n
        s1, w1, a1 = cycloid(d1, T, t)
        s2, w2, a2 = cycloid(d2, T, t)
        t1, t2, _ = dynamics(p, qa[1] + s2, w1, w2, a1, a2)
        out['w1'], out['w2'] = max(out['w1'], abs(w1)), max(out['w2'], abs(w2))
        out['a1'], out['a2'] = max(out['a1'], abs(a1)), max(out['a2'], abs(a2))
        out['t1'], out['t2'] = max(out['t1'], abs(t1)), max(out['t2'], abs(t2))
        out['P1'], out['P2'] = max(out['P1'], abs(t1 * w1)), max(out['P2'], abs(t2 * w2))
    out['p'] = p
    return out


def cycle(b, payload=None):
    """The standard cycle's horizontal leg, 300 mm across at x0, and a radial move of the same length, the two moves
    that load J1 and J2 hardest; and the vertical leg for J3."""
    payload = b['m_rated'] if payload is None else payload
    h = b['across'] / 2
    across = move(b, (b['x0'], -h), (b['x0'], h), payload)
    radial = move(b, (b['L1'] + b['L2'] - 30 - b['across'], 0.0), (b['L1'] + b['L2'] - 30, 0.0), payload, T=b['t_r'])
    T = b['t_v']
    v3 = 2 * b['up'] / T                                   # mm/s, cycloidal
    a3 = 2 * pi * b['up'] / T ** 2                          # mm/s2
    m_z = MASS['spline'] + b['m_tool'] + payload
    F3 = m_z * (G + a3 / 1000)                              # N, lifting with the largest acceleration
    # J4: the part and the tool turned by q4_move in t4, cycloidal; the tool taken as a disc of 40 mm radius
    I4 = payload * (b['k4'] / 1000) ** 2 + 0.5 * b['m_tool'] * 0.04 ** 2
    a4 = 2 * pi * math.radians(b['q4_move']) / b['t4'] ** 2
    w4 = 2 * math.radians(b['q4_move']) / b['t4']
    return dict(across=across, radial=radial, v3=v3, a3=a3, F3=F3, m_z=m_z, I4=I4, a4=a4, w4=w4, T4=I4 * a4)


# ============================================================================================================
# Chapter 36: the J1 and J2 modules. Harmonic Drive LLC, CSF/CSG catalogue: Table 3 (CSG ratings, p. 11), the
# wave generator's life (p. 14), the selection procedure (p. 15), Tables 28 to 30 (the housed units' dimensions and
# cross roller bearings, pp. 32-33), the output bearing's equations 11 to 19 and lists 2 to 4 (pp. 34-35), the
# housed units' efficiency (p. 42); the product pages for the torsional stiffness (K1, K2, K3, T1, T2).
# ============================================================================================================
CSG = {      # ratio 50, grease; N m, rev/min, kg m2, N m/rad, m, N, kg, mm
    17: dict(Tr=21, Trep=44, Tav=34, Tmom=91, nmax=7300, nav=3500, Iwg=0.079e-4, K1=0.81e4, T1=3.9, K2=1.1e4, T2=12,
             K3=1.3e4, dp=0.0425, R=0.0095, C=5290, C0=7550, Mc=64, Km=7.75e4, mass=0.68, A=79, B=45, O=63, T=48, U=8),
    20: dict(Tr=33, Trep=73, Tav=44, Tmom=127, nmax=6500, nav=3500, Iwg=0.193e-4, K1=1.3e4, T1=7, K2=1.8e4, T2=25,
             K3=2.3e4, dp=0.050, R=0.0095, C=5780, C0=9000, Mc=91, Km=12.8e4, mass=0.98, A=93, B=45.5, E=28, O=72, Q=58,
             S=10, T=56, U=14, b=82, nc=8, d=5.5),
    25: dict(Tr=51, Trep=127, Tav=72, Tmom=242, nmax=5600, nav=3500, Iwg=0.413e-4, K1=2.5e4, T1=14, K2=3.4e4, T2=48,
             K3=4.4e4, dp=0.062, R=0.0115, C=9600, C0=15100, Mc=156, Km=24.2e4, mass=1.5, A=107, B=52, O=86, T=67, U=14),
    32: dict(Tr=99, Trep=281, Tav=140, Tmom=497, nmax=4800, nav=3500, Iwg=1.69e-4, K1=5.4e4, T1=29, K2=7.8e4, T2=108,
             K3=9.8e4, dp=0.080, R=0.013, C=15000, C0=25000, Mc=313, Km=53.9e4, mass=3.2, A=138, B=62, E=45, O=113,
             Q=96, S=20, T=90, U=14, b=125, nc=12, d=6.6),
    40: dict(Tr=178, Trep=523, Tav=255, Tmom=892, nmax=4000, nav=3000, Iwg=4.50e-4, K1=13e4, T1=54, K2=20e4, T2=196,
             K3=23e4, dp=0.096, R=0.0145, C=21300, C0=36500, Mc=450, Km=91.0e4, mass=5.0, A=160, B=72.5, E=50.5,
             O=127, Q=109, S=24, T=110, U=14, b=144, nc=10, d=9),
}
LN_CSG, NR = 10000.0, 2000.0          # h, rev/min: the CSG's rated L10 life at its rated input speed (p. 14)
ETA_50 = 0.65                         # housed unit 17-65, ratio 50, grease, 3500 rev/min, 20 C (chart p. 42, read)
MOTOR = {1: dict(Jm=1.5e-4, nmax=5000.0, flange=130.0, L=150.0, D=110.0, mass=4.5),       # the servo motors'
         2: dict(Jm=0.5e-4, nmax=6500.0, flange=60.0, L=110.0, D=56.0, mass=1.2)}          # specification, assumed


def leg_pattern(b, A, B, payload, T, joint, rest=0.0, n=200, elbow=1):
    """The joint's output speed (rev/min), torque (N m) and acceleration (rad/s2) in small steps over a move from A
    to B and back, with `rest` s at rest after each: the load pattern of the catalogue's selection (p. 15)."""
    p = inertia(b, payload)
    qa, qb = ik(b, *A, elbow=elbow), ik(b, *B, elbow=elbow)
    d1, d2 = qb[0] - qa[0], qb[1] - qa[1]
    rows = []
    for back in (False, True):
        for i in range(n):
            t = T * (i + 0.5) / n
            s1, w1, a1 = cycloid(d1, T, t)
            s2, w2, a2 = cycloid(d2, T, t)
            if back:
                s1, w1, a1 = d1 - s1, -w1, -a1
                s2, w2, a2 = d2 - s2, -w2, -a2
            t1, t2, _ = dynamics(p, qa[1] + s2, w1, w2, a1, a2)
            w, tau, a = (w1, t1, a1) if joint == 1 else (w2, t2, a2)
            rows.append((T / n, abs(w) * 60 / (2 * pi), tau, a))
        if rest > 0:
            rows.append((rest, 0.0, 0.0, 0.0))
    return rows


def hd_select(rows, size, R=50):
    """The catalogue's selection for a load pattern: the average torque (cube mean weighted by speed and time), the
    average output and input speeds, the largest input speed, the repeated peak, and the wave generator's L10 life."""
    u = CSG[size]
    num = sum(dt * n * abs(T) ** 3 for dt, n, T, _ in rows)
    den = sum(dt * n for dt, n, T, _ in rows)
    Tav = (num / den) ** (1 / 3)
    nav = den / sum(dt for dt, n, T, _ in rows)
    ni_av = nav * R
    ni_max = max(n for dt, n, T, _ in rows) * R
    Tpk = max(abs(T) for dt, n, T, _ in rows)
    Lh = LN_CSG * (u['Tr'] / Tav) ** 3 * (NR / ni_av)
    return dict(size=size, Tav=Tav, nav=nav, ni_av=ni_av, ni_max=ni_max, Tpk=Tpk, Lh=Lh,
                ok=Tpk <= u['Trep'] and Tav <= u['Tav'] and ni_max <= u['nmax'] and ni_av <= u['nav'])


def motor(rows, size, joint, R=50, eta=ETA_50):
    """The motor's torque in each step: the joint's torque through the ratio and the efficiency, plus the motor's and
    the wave generator's inertia accelerated at R times the joint's acceleration; its peak and RMS."""
    Jm = MOTOR[joint]['Jm'] + CSG[size]['Iwg']
    Tm = [T / (R * eta) + Jm * a * R for dt, n, T, a in rows]
    tot = sum(dt for dt, n, T, a in rows)
    rms = math.sqrt(sum(dt * x ** 2 for (dt, n, T, a), x in zip(rows, Tm)) / tot)
    return dict(peak=max(abs(x) for x in Tm), rms=rms, nmax=max(n for dt, n, T, a in rows) * R,
                inertia_share=Jm * max(abs(a) for dt, n, T, a in rows) * R)


def j1_bearing(b, payload=None, Lr=0.05, fw=1.5, size=40):
    """The J1 unit's cross roller bearing (pp. 34-35): the largest moment from the arm's weight and the horizontal
    force that accelerates it, the static safety factor, and the life in oscillating motion over the standard cycle.
    Lr is the height of the arm's mass centre above the bearing's face (assumed)."""
    payload = b['m_rated'] if payload is None else payload
    u = CSG[size]
    M = MASS
    W = [(M['arm1'], M['arm1_c']), (M['j2'], b['L1']), (M['arm2'], b['L1'] + M['arm2_c']),
         (M['j34_motors'], b['L1'] + M['j34_c']), (M['spline'] + b['m_tool'] + payload, b['L1'] + b['L2'])]
    Fa = sum(m * G for m, r in W)
    Mw = sum(m * G * r / 1000 for m, r in W)
    La = Mw / Fa
    # the horizontal force: each mass's acceleration about J1 on the horizontal leg (the elbow fixed, so the arm
    # turns as one body); tangential alpha r and centripetal w^2 r, at the cycloid's largest of each
    A = (b['x0'], -b['across'] / 2)
    B = (b['x0'], b['across'] / 2)
    qa, qb = ik(b, *A), ik(b, *B)
    d1 = qb[0] - qa[0]
    T = b['t_h']
    Frmax = 0.0
    Fr_rows = []
    elbow = (b['L1'], 0.0)
    q2 = qa[1]
    def pos(r_from_elbow):
        return (b['L1'] + r_from_elbow * math.cos(q2), r_from_elbow * math.sin(q2))
    pts = [(M['arm1'], (M['arm1_c'], 0.0)), (M['j2'], elbow), (M['arm2'], pos(M['arm2_c'])),
           (M['j34_motors'], pos(M['j34_c'])), (M['spline'] + b['m_tool'] + payload, pos(b['L2']))]
    for i in range(201):
        t = T * i / 200
        _, w, a = cycloid(d1, T, t)
        fx = fy = 0.0
        for m, (x, y) in pts:
            x, y = x / 1000, y / 1000
            fx += m * (-a * y - w * w * x)
            fy += m * (a * x - w * w * y)
        F = math.hypot(fx, fy)
        Frmax = max(Frmax, F)
        Fr_rows.append((T / 200, abs(w) * 60 / (2 * pi), F))
    Mmax = Frmax * (Lr + u['R']) + Fa * La
    P0 = Frmax + 2 * Mmax / u['dp'] + 0.44 * Fa
    fs = u['C0'] / P0
    # average loads (eqs. 11, 12) over the leg, the axial load constant
    num = sum(dt * n * F ** (10 / 3) for dt, n, F in Fr_rows)
    den = sum(dt * n for dt, n, F in Fr_rows)
    Frav = (num / den) ** (3 / 10)
    Faav = Fa
    ratio = Faav / (Frav + 2 * (Frav * (Lr + u['R']) + Faav * La) / u['dp'])
    X, Y = (1.0, 0.45) if ratio <= 1.5 else (0.67, 0.67)
    Pc = X * (Frav + 2 * (Frav * (Lr + u['R']) + Faav * La) / u['dp']) + Y * Faav
    n1 = 60.0 / b['cycle']                        # round trips a minute
    phi = math.degrees(d1) / 2                    # half the oscillating angle
    Loc = 1e6 / (60 * n1) * 90 / phi * (u['C'] / (fw * Pc)) ** (10 / 3)
    return dict(Fa=Fa, Mw=Mw, La=La, Frmax=Frmax, Mmax=Mmax, Mc=u['Mc'], P0=P0, fs=fs, Frav=Frav, ratio=ratio, X=X,
                Y=Y, Pc=Pc, n1=n1, phi=phi, Loc=Loc, Lr=Lr, fw=fw)


def j2_bearing(b, payload=None, Lr=0.04):
    """The J2 unit's bearing, statically: the second arm's weight and its moment about J2, and the horizontal force
    of the radial move."""
    payload = b['m_rated'] if payload is None else payload
    u = CSG[20]
    M = MASS
    W = [(M['arm2'], M['arm2_c']), (M['j34_motors'], M['j34_c']), (M['spline'] + b['m_tool'] + payload, b['L2'])]
    Fa = sum(m * G for m, r in W)
    Mw = sum(m * G * r / 1000 for m, r in W)
    c = cycle(b, payload)
    # the largest horizontal force on the second arm: its masses at the radial move's largest acceleration about J2
    a2 = c['radial']['a2']
    Fr = sum(m * a2 * r / 1000 for m, r in W)
    Mmax = Fr * (Lr + u['R']) + Mw
    P0 = Fr + 2 * Mmax / u['dp'] + 0.44 * Fa
    return dict(Fa=Fa, Mw=Mw, Fr=Fr, Mmax=Mmax, Mc=u['Mc'], P0=P0, fs=u['C0'] / P0)


def case_bolts(size, mu=0.15, Sp=970.0, At=20.1):
    """The unit's case held to its seat by its bolts (Table 29: c holes of d on the circle b), class 12.9: the
    torque friction carries at the preload 0.75 At Sp (Shigley Eqs. 8-31, 8-32), against the momentary peak."""
    u = CSG[size]
    Fi = 0.75 * At * Sp
    T = u['nc'] * Fi * mu * u['b'] / 2000
    return dict(Fi=Fi, T=T, n=T / u['Tmom'])


def twist(size, T):
    """The strain wave gear's wind-up under an output torque T (N m), rad: the catalogue's three straight lines,
    K1 up to T1, K2 from T1 to T2, K3 beyond (the product pages' torsional stiffness)."""
    u = CSG[size]
    T = abs(T)
    if T <= u['T1']:
        return T / u['K1']
    if T <= u['T2']:
        return u['T1'] / u['K1'] + (T - u['T1']) / u['K2']
    return u['T1'] / u['K1'] + (u['T2'] - u['T1']) / u['K2'] + (T - u['T2']) / u['K3']


def joint_inertias(b=None, joint=1, R=50):
    """The arm's inertia about the joint (kg m2, the rated payload), the motor's side seen from the output (R^2 times
    the motor's rotor and the wave generator), and the joint's largest speed (rad/s): J1 on the horizontal leg,
    J2 on the radial move."""
    b = b or brief()
    p = inertia(b, b['m_rated'])
    c = cycle(b, b['m_rated'])
    if joint == 1:
        qa = ik(b, b['x0'], -b['across'] / 2)
        Ja = dynamics(p, qa[1], 0, 0, 0, 0)[2]['M11']
        w0, size = c['across']['w1'], 40
    else:
        Ja = dynamics(p, 0.0, 0, 0, 0, 0)[2]['M22']
        w0, size = c['radial']['w2'], 20
    Jr = R ** 2 * (MOTOR[joint]['Jm'] + CSG[size]['Iwg'])
    return dict(Ja=Ja, Jr=Jr, w0=w0, size=size, R=R)


def ratio_choice(b=None):
    """The ratio's two bounds for each joint: the largest the unit's input speed allows, nmax / n_out, and the one
    that accelerates pure inertia with the least motor torque, sqrt(Ja / Jm)."""
    out = {}
    for joint in (1, 2):
        j = joint_inertias(b, joint)
        size = j['size']
        n_out = j['w0'] * 60 / (2 * pi)
        Jm = MOTOR[joint]['Jm'] + CSG[size]['Iwg']
        out[joint] = dict(n_out=n_out, R_speed=CSG[size]['nmax'] / n_out, R_opt=math.sqrt(j['Ja'] / Jm),
                          motor_speed=MOTOR[joint]['nmax'] / n_out)
    return out


def stop_on_arm(joint, b=None, r=None):
    """The arm overruns into a stop on the arm at its largest speed, the drive off. A buffer much softer than the
    gear stops arm and motor together; the gear's teeth carry the torque that stops the motor's side, Jr alpha. A
    linear buffer keeps that torque under the unit's momentary limit if it gives at least Jr w0^2 / Tmom (rad); the
    collision lasts 2 theta / w0, and the catalogue's Eq. 7 counts the collisions the flexspline takes."""
    b = b or brief()
    j = joint_inertias(b, joint)
    u = CSG[j['size']]
    r = r if r is not None else (b['L1'] if joint == 1 else b['L2'])
    theta = j['Jr'] * j['w0'] ** 2 / u['Tmom']
    t = 2 * theta / j['w0']
    n_in = j['w0'] * 60 / (2 * pi) * j['R']
    N = 1.0e4 / (2 * n_in / 60 * t)
    Tb = u['Tmom'] * (j['Ja'] + j['Jr']) / j['Jr']
    return dict(E_arm=0.5 * j['Ja'] * j['w0'] ** 2, E_motor=0.5 * j['Jr'] * j['w0'] ** 2, theta=theta,
                travel=theta * r, t=t, N=N, T_buffer=Tb, F_buffer=Tb / (r / 1000), r=r, **j)


def brake_stop(joint, Tb, b=None, angle=15.0):
    """The motor's brake stops the arm from its largest speed, the drive off, losses left out: the deceleration
    R Tb / (Ja + Jr), the angle and time it takes, and the torque the gear carries to stop the arm, Ja alpha.
    The brake's bounds: at least the torque that stops the arm within `angle` degrees, at most the one that keeps
    the gear at its momentary limit."""
    b = b or brief()
    j = joint_inertias(b, joint)
    u = CSG[j['size']]
    Jt = j['Ja'] + j['Jr']
    alpha = j['R'] * Tb / Jt
    lo = j['w0'] ** 2 / (2 * math.radians(angle)) * Jt / j['R']
    hi_rep = u['Trep'] * Jt / (j['Ja'] * j['R'])
    hi_mom = u['Tmom'] * Jt / (j['Ja'] * j['R'])
    Tg = j['Ja'] * alpha
    t = j['w0'] / alpha
    n_in = j['w0'] * 60 / (2 * pi) * j['R']
    N = 1.0e4 / (2 * n_in / 60 * t) if Tg > u['Trep'] else None
    return dict(Tb=Tb, alpha=alpha, stop=math.degrees(j['w0'] ** 2 / (2 * alpha)), t=t, Tg=Tg, lo=lo, hi_rep=hi_rep,
                hi_mom=hi_mom, N=N, Trep=u['Trep'], Tmom=u['Tmom'])


BRAKE = {1: 8.0, 2: 3.5}        # the brakes' static torque at the motor (N m), chosen in Chapter 36


def modules(b=None):
    """Chapter 36's sizing, in the order of the chapter."""
    b = b or brief()
    out = {}
    A = (b['x0'], -b['across'] / 2)
    B = (b['x0'], b['across'] / 2)
    rest = (b['cycle'] - 2 * b['t_h']) / 2
    r1 = leg_pattern(b, A, B, b['m_rated'], b['t_h'], 1, rest=rest)
    out['j1'] = {s: hd_select(r1, s) for s in (25, 32, 40)}
    out['j1_motor'] = motor(r1, 40, 1)
    out['j1_motor_32'] = motor(r1, 32, 1)
    R0, R1 = (b['L1'] + b['L2'] - 30 - b['across'], 0.0), (b['L1'] + b['L2'] - 30, 0.0)
    r2 = leg_pattern(b, R0, R1, b['m_rated'], b['t_r'], 2, rest=b['t_v'] / 2)
    out['j2'] = {s: hd_select(r2, s) for s in (17, 20, 25)}
    out['j2_motor'] = motor(r2, 20, 2)
    out['j1_bearing'] = j1_bearing(b)
    out['j2_bearing'] = j2_bearing(b)
    out['j1_bolts'] = case_bolts(40, At=36.6)
    out['j2_bolts'] = case_bolts(20, At=14.2)               # M5 (Table 29: d = 5.5)
    p = inertia(b, b['m_rated'])
    qa = ik(b, *A)
    _, _, Mx = dynamics(p, qa[1], 0, 0, 0, 0)
    out['f1'] = math.sqrt(CSG[40]['K1'] / Mx['M11']) / (2 * pi)
    out['f2'] = math.sqrt(CSG[20]['K1'] / Mx['M22']) / (2 * pi)
    out['ratio'] = ratio_choice(b)
    out['twist1'] = twist(40, out['j1'][40]['Tpk'])
    out['stops'] = {k: stop_on_arm(k, b) for k in (1, 2)}
    out['brakes'] = {k: brake_stop(k, BRAKE[k], b) for k in (1, 2)}
    return out


if __name__ == '__main__':
    b = brief()
    print(workspace(b))
    for pl in (b['m_rated'], b['m_max']):
        c = cycle(b, pl)
        for k in ('across', 'radial'):
            m = c[k]
            print(pl, k, {x: round(m[x], 3) for x in ('d1', 'd2', 'w1', 'w2', 'a1', 'a2', 't1', 't2', 'P1', 'P2')})
        print(pl, 'J3', round(c['v3'], 1), round(c['a3'], 1), round(c['F3'], 2), 'J4', round(c['w4'], 2), round(c['a4'], 1), round(c['T4'], 3))
