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
    45: dict(Tr=229, Trep=650, Tav=345, Tmom=1235, nmax=3800, nav=3000, Iwg=8.68e-4, K1=18e4, T1=76, K2=29e4, T2=275,
             K3=33e4, dp=0.111, R=0.0155, C=23000, C0=42600, Mc=686, Km=141e4, mass=7.0, A=180, B=79.5, E=58,
             O=148, Q=127, S=25, T=124, U=19, b=164, nc=12, d=9),   # Table 4 (p. 12), Tables 28-30, product page
}
LN_CSG, NR = 10000.0, 2000.0          # h, rev/min: the CSG's rated L10 life at its rated input speed (p. 14)
ETA_50 = 0.65                         # housed unit 17-65, ratio 50, grease, 3500 rev/min, 20 C (chart p. 42, read)
MOTOR = {1: dict(Jm=1.5e-4, nmax=5000.0, flange=130.0, L=150.0, D=110.0, mass=4.5),       # the servo motors'
         2: dict(Jm=0.5e-4, nmax=6500.0, flange=60.0, L=110.0, D=56.0, mass=1.2)}          # specification, assumed


def leg_pattern(b, A, B, payload, T, joint, rest=0.0, n=200, elbow=1, p=None):
    """The joint's output speed (rev/min), torque (N m) and acceleration (rad/s2) in small steps over a move from A
    to B and back, with `rest` s at rest after each: the load pattern of the catalogue's selection (p. 15)."""
    p = p or inertia(b, payload)
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


BRAKE = {1: 8.0, 2: 3.5}        # the brakes' static torque at the motor (N m), chosen in Chapter 36; Chapter 38's
#                                 integration() lowers J2's to 3.4, since 3.5 would load the gear to 125.6 N m, within
#                                 1.1 % of its momentary 127 (the 3.4 below, in integration())


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


# ============================================================================================================
# Chapter 37: J3 and J4, the ball screw spline. HIWIN GmbH (2022), Ballscrews BS-08-9-EN-2204-K: the ball spline FB
# type, Tables 9.2 and 9.3 (p. 90) and the working modes, Table 9.4 (p. 91); the operating factor f_p and its short
# stroke rule, F 3.8 to F 3.10 (p. 26); DN (p. 30). Gates (2026 download), Light Power and Precision Drive Design
# Manual: the drive selection procedure (p. 12), the 3 mm stock belt lengths (p. 15), the 3MGT ratings with the
# width multipliers and length factors (p. 19), the formulas for the pitch length, the arc of contact, the teeth in
# mesh and the tensions (pp. 103-104). Shigley (Budynas & Nisbett, 2015): E of steel (Table A-5), the Marin factors
# (Eqs. 6-8, 6-19, 6-20). The layout below and every value marked so are assumed.
# ============================================================================================================
SP = {   # HIWIN FB ball screw spline: shaft d and the catalogue's inner diameter, lead; screw nut (Table 9.2): C, C0,
         # D2, A2, L2, F2, T2, E2, Z2, W2, C2 (four flange holes), W3 and m2 (six tapped holes); spline nut (Table
         # 9.3): Cs, C0s, the torsional ratings CT, C0T, the permissible static moment MA, D1, A1, L1, F1, T1, E1, Z1,
         # W, C1, W1, m1; the support bearings' Ca, Coa (N, N m, mm)
    16: dict(d=16, d_in=11, l=16, C=4700, C0=9600, D2=48, A2=64, L2=40, F2=36.0, T2=21, E2=10, Z2=6, W2=56, C2=4.5,
             W3=25, m2=4, Cs=7200, C0s=13500, CT=32.1, C0T=34.4, MA=67.6, D1=48, A1=64, L1=50, F1=36.0, T1=21, E1=10,
             Z1=6, W=56, C1=4.5, W1=30, m1=4, Ca=9300, Coa=11500),
    20: dict(d=20, d_in=14, l=20, C=6400, C0=14000, D2=56, A2=72, L2=46, F2=43.5, T2=21, E2=11, Z2=6, W2=64, C2=4.5,
             W3=31, m2=5, Cs=10400, C0s=20000, CT=57.8, C0T=63.2, MA=118.0, D1=56, A1=72, L1=63, F1=43.5, T1=21,
             E1=12, Z1=6, W=64, C1=4.5, W1=36, m1=5, Ca=9800, Coa=13300),
    25: dict(d=25, d_in=18, l=25, C=9500, C0=21900, D2=66, A2=86, L2=58, F2=52.0, T2=25, E2=13, Z2=7, W2=75, C2=5.5,
             W3=38, m2=6, Cs=15400, C0s=27500, CT=106.5, C0T=108.8, MA=210.0, D1=66, A1=86, L1=71, F1=52.0, T1=25,
             E1=13, Z1=7, W=75, C1=5.5, W1=44, m1=5, Ca=13100, Coa=22000),
    32: dict(d=32, d_in=23, l=32, C=13000, C0=29800, D2=78, A2=103, L2=72, F2=63.0, T2=25, E2=14, Z2=8, W2=89, C2=6.6,
             W3=48, m2=6, Cs=20500, C0s=34400, CT=181.5, C0T=173.1, MA=290.0, D1=78, A1=103, L1=80, F1=63.0, T1=25,
             E1=17, Z1=8, W=89, C1=6.6, W1=54, m1=6, Ca=13700, Coa=25200),
}
E_STEEL = 207000.0                  # MPa (Shigley Table A-5)
RHO_STEEL, RHO_AL = 7.85e-6, 2.70e-6   # kg/mm3
F_P = 3.0                           # HIWIN's operating factor: the short-stroke range, 3.0 to 5.0 (p. 26)
MU_BALL, ALPHA_BALL = 0.005, 45.0   # the balls' friction and contact angle, as Chapter 33 (assumed)


def screw_eta(l, d, mu=MU_BALL, alpha=ALPHA_BALL):
    """Childs (2014), Eq. 16.33, as Chapter 33: a ball screw's efficiency from its lead angle on the nominal
    diameter, the balls' friction and their contact angle."""
    lam = math.atan(l / (pi * d))
    a = math.radians(alpha)
    return (math.cos(a) - mu * math.tan(lam)) / (math.cos(a) + mu / math.tan(lam))


ETA_SCREW = screw_eta(20.0, 20.0)   # the SP20's lead on its diameter: 0.976, taken against the motor both ways
ETA_BELT = 0.98                     # a toothed belt drive's efficiency (assumed)
SF_BELT = 2.0                       # Gates' service factor, the top of its 1.5 to 2.0 (p. 12)
MOTOR34 = {3: dict(Jm=0.18e-4, nmax=6000.0, flange=60.0, L=100.0, D=56.0, mass=1.0, shaft=45.0),   # assumed
           4: dict(Jm=0.05e-4, nmax=6000.0, flange=40.0, L=90.0, D=38.0, mass=0.6, shaft=45.0)}

# the layout at the second arm's end (mm): x along the arm from J2, the shaft's axis at x = 250, y across, z up from
# the base plate's top face; the spline nut under the base plate, its pulley below; the screw nut on the top plate,
# its pulley above; the two motors inside the box, side by side, their shafts through the plates (assumed)
LAY37 = dict(x_s=250.0, H=150.0, t_base=12.0, t_top=10.0, half_w=74.0, wall=8.0, y_m=34.0,
             gap_pulley=10.0,       # the tool flange's top below the spline pulley at J3 = 0
             flange_t=10.0, flange_d=50.0, tool_c=60.0,   # the tool flange; the gripper's and payload's centre below it
             above=5.0)             # the shaft's top above the screw nut's rotor at the full stroke
BELT_W = {3: 9.0, 4: 9.0}           # mm
PULLEYS = {3: (28, 56), 4: (18, 72)}   # grooves: motor, nut
GT3_LENGTHS = [99, 111, 123, 129, 159, 165, 180, 183, 189, 201, 219, 225, 240, 243, 255, 267, 282, 291, 300, 339, 348,
               357, 360, 375, 393, 408, 414, 420, 447, 450, 474, 480, 483, 489, 501, 504, 510, 513, 537, 552, 564, 600,
               630, 684, 735, 750, 786, 840, 945, 1050, 1080, 1536, 1587, 2061]   # stock 3MR pitch lengths (p. 15)
GT3_WIDTH = {6: 1.00, 9: 1.64, 12: 2.32, 15: 3.03}                                  # p. 19
GT3_LENGTH_F = [(126, 0.70), (150, 0.75), (177, 0.80), (210, 0.85), (249, 0.90), (291, 0.95), (345, 1.00),
                (405, 1.05), (477, 1.10), (564, 1.15), (663, 1.20), (783, 1.25), (921, 1.30), (1089, 1.35),
                (1200, 1.40)]       # up to that length (mm): the factor (p. 19)
GT3_RPM = [600, 800, 1000, 1200, 1400, 1600, 1800, 2000, 2400, 2800, 3200, 3600, 4000]
GT3_NM = {   # 3MGT, 6 mm, rated torque (N m) for the small sprocket at those speeds (p. 19)
    16: [1.19, 1.13, 1.08, 1.04, 1.01, 0.98, 0.95, 0.93, 0.89, 0.86, 0.83, 0.80, 0.78],
    18: [1.41, 1.34, 1.28, 1.24, 1.20, 1.17, 1.14, 1.11, 1.07, 1.03, 1.00, 0.97, 0.94],
    20: [1.62, 1.54, 1.48, 1.43, 1.39, 1.35, 1.32, 1.29, 1.24, 1.20, 1.16, 1.13, 1.10],
    22: [1.83, 1.74, 1.67, 1.62, 1.57, 1.53, 1.50, 1.47, 1.41, 1.37, 1.33, 1.29, 1.26],
    24: [2.03, 1.94, 1.87, 1.81, 1.76, 1.72, 1.68, 1.64, 1.58, 1.53, 1.49, 1.45, 1.42],
    26: [2.24, 2.14, 2.06, 2.00, 1.94, 1.89, 1.85, 1.82, 1.75, 1.70, 1.65, 1.61, 1.57],
    28: [2.44, 2.34, 2.25, 2.18, 2.12, 2.07, 2.03, 1.99, 1.92, 1.86, 1.81, 1.76, 1.72],
    30: [2.65, 2.53, 2.44, 2.36, 2.30, 2.25, 2.20, 2.16, 2.08, 2.02, 1.96, 1.91, 1.87],
}


def pd(z, p=3.0):
    """A toothed pulley's pitch diameter (mm)."""
    return p * z / pi


def nut_layout(size=20):
    """Heights (mm) of the nuts, pulleys, belts and the shaft at J3 = 0 (shaft up), from the catalogue's lengths and
    LAY37. The spline nut's flange under the base plate, its rotor's pilot down; the screw nut's flange on the top
    plate, its pilot up."""
    s, L = SP[size], LAY37
    zb = -L['t_base']                                  # the base plate's underside
    spl = dict(flange=(zb - s['E1'], zb), body=(zb, zb + s['L1'] - s['T1']), rotor=(zb - s['T1'], zb - s['T1'] + s['L1']))
    zt = L['H'] + L['t_top']                           # the top plate's top face
    scr = dict(flange=(zt, zt + s['E2']), body=(zt - (s['L2'] - s['T2']), zt), rotor=(zt + s['T2'] - s['L2'], zt + s['T2']))
    rim = BELT_W[4] + 3.0                              # the pulley's toothed rim: the belt and 1.5 either side
    p4 = (spl['rotor'][0] - rim - 3.0, min(spl['rotor'][0] + s['Z1'], spl['flange'][0] - 5.0))   # clear of the bolt heads
    p3 = (scr['rotor'][1] - s['Z2'], scr['rotor'][1] + rim + 3.0)
    belt4 = (p4[0] + 1.5, p4[0] + 1.5 + BELT_W[4])
    belt3 = (p3[1] - 1.5 - BELT_W[3], p3[1] - 1.5)
    tool_top = p4[0] - L['gap_pulley']
    shaft_top = scr['rotor'][1] + L['above'] + 200.0   # the shaft's top at J3 = 0, so that at 200 down it still clears
    return dict(spl=spl, scr=scr, p4=p4, p3=p3, belt4=belt4, belt3=belt3, tool_top=tool_top, shaft=(tool_top, shaft_top),
                spl_c=sum(spl['rotor']) / 2, scr_c=sum(scr['rotor']) / 2)


def shaft_cantilever(size=20, payload=2.0, stroke=200.0, b=None):
    """The shaft at the full stroke as a cantilever from the spline nut's lower end: its section taken as the solid
    circle of the catalogue's inner diameter (the least it could be), its mass as the solid outer diameter (the most);
    the tool flange, the gripper and the payload at their places. Stiffness, the swing's frequency, and under the
    tool's largest horizontal acceleration the moment on the spline nut, the bending stress and the nuts' reactions."""
    b = b or brief()
    s, L = SP[size], LAY37
    lay = nut_layout(size)
    z0 = lay['spl']['rotor'][0]                        # where the shaft leaves the spline nut
    zf = lay['tool_top'] - stroke                      # the tool flange's top at the full stroke
    zc = zf - L['flange_t'] - L['tool_c']              # the gripper's and payload's centre
    Lc = z0 - zc
    I = pi * s['d_in'] ** 4 / 64
    k = 3 * E_STEEL * I / Lc ** 3                      # N/mm at the mass centre
    over = z0 - zf
    m_over = pi / 4 * s['d'] ** 2 * over * RHO_STEEL
    m_fl = pi / 4 * L['flange_d'] ** 2 * L['flange_t'] * RHO_AL
    m_t = b['m_tool'] + payload
    # the flange and the shaft's overhang brought to the mass centre with the cantilever's shape factors
    m_eff = m_t + m_fl * ((z0 - zf + L['flange_t'] / 2) / Lc) ** 2 * 1.0 + 0.236 * m_over * (over / Lc) ** 3
    f = math.sqrt(k * 1000 / m_eff) / (2 * pi)
    # the tool's largest horizontal acceleration: on the horizontal leg, its distance from J1 times the largest of
    # sqrt(qdd1^2 + qd1^4) over the cycloid
    A = (b['x0'], -b['across'] / 2)
    qa, qb = ik(b, *A), ik(b, b['x0'], b['across'] / 2)
    d1 = qb[0] - qa[0]
    acc = max(math.hypot(2 * pi * d1 / b['t_h'] ** 2 * math.sin(2 * pi * i / 4000),
                         (d1 / b['t_h'] * (1 - math.cos(2 * pi * i / 4000))) ** 2) for i in range(4001))
    a_tool = acc * math.hypot(*A) / 1000               # m/s2
    a_rad = 2 * pi * (b['across'] / 1000) / b['t_r'] ** 2
    F_t = m_t * a_tool
    F_fl = m_fl * a_tool
    F_sh = m_over * a_tool
    M_face = F_t * Lc / 1000 + F_fl * (z0 - zf + L['flange_t'] / 2) / 1000 + F_sh * over / 2000    # N m
    zc_n = lay['spl_c']
    arm = zc_n - z0                                    # the nut's middle above the shaft's exit
    M_c = M_face + (F_t + F_fl + F_sh) * arm / 1000                                                 # at the nut's middle
    sigma = M_face * 1000 / (pi * s['d_in'] ** 3 / 32)
    span = lay['scr_c'] - lay['spl_c']
    F_all = F_t + F_fl + F_sh
    R_up = M_c * 1000 / span
    R_low = F_all + R_up
    return dict(size=size, Lc=Lc, I=I, k=k, over=over, m_over=m_over, m_fl=m_fl, m_t=m_t, m_eff=m_eff, f=f,
                a_tool=a_tool, a_rad=a_rad, acc=acc, F_t=F_t, F_all=F_all, M_face=M_face, M_c=M_c, MA=s['MA'],
                sigma=sigma, span=span, R_up=R_up, R_low=R_low, r_tool=math.hypot(*A))


def shaft_fatigue(sigma, d=14.0, Sut=700.0, Kf=2.0):
    """The shaft's reversed bending against Shigley's endurance limit with the Marin factors for a ground surface and
    the size; the material's strength and the grooves' notch factor assumed (the catalogue gives neither)."""
    Se_p = 0.5 * Sut
    ka = 1.58 * Sut ** -0.085
    kb = 1.24 * d ** -0.107
    Se = ka * kb * Se_p
    return dict(Se_p=Se_p, ka=ka, kb=kb, Se=Se, Kf=Kf, n=Se / (Kf * sigma))


def j34_timeline(b=None, n=2000):
    """The standard cycle for J3 and J4 (0.50 s): four vertical legs of 25 mm, each 0.10 s, up at 0, down at 0.15,
    up at 0.25, down at 0.40; J4 turns 180 deg during each horizontal leg, 0.025 to 0.225 and 0.275 to 0.475 s, the
    other way on the way back. Rows: t, dt, J3's speed and acceleration (mm/s, mm/s2, + up), J4's (rad/s, rad/s2)."""
    b = b or brief()
    T3, T4 = b['t_v'], b['t4']
    legs = [(0.0, +1), (0.15, -1), (0.25, +1), (0.40, -1)]
    turns = [(0.025, +1), (0.275, -1)]
    q4 = math.radians(b['q4_move'])
    rows = []
    for i in range(n):
        t = b['cycle'] * (i + 0.5) / n
        v3 = a3 = w4 = a4 = 0.0
        for t0, sg in legs:
            if t0 <= t < t0 + T3:
                _, v, a = cycloid(b['up'], T3, t - t0)
                v3, a3 = sg * v, sg * a
        for t0, sg in turns:
            if t0 <= t < t0 + T4:
                _, w, a = cycloid(q4, T4, t - t0)
                w4, a4 = sg * w, sg * a
        rows.append((t, b['cycle'] / n, v3, a3, w4, a4))
    return rows


def screw_life(size=20, payload=None, b=None, fp=F_P):
    """The screw nut over the cycle: the axial force m_z (g + a3), its cube mean weighted by the nut's turns on the
    shaft (F 3.8), the operating factor, the life in turns and hours (F 3.10), the static factor at the largest
    payload, the short-stroke rule, and DN."""
    b = b or brief()
    payload = b['m_rated'] if payload is None else payload
    s = SP[size]
    m_z = MASS['spline'] + b['m_tool'] + payload
    rows = j34_timeline(b)
    num = den = 0.0
    Fmax = 0.0
    for t, dt, v3, a3, w4, a4 in rows:
        F = m_z * (G + a3 / 1000)
        Fmax = max(Fmax, abs(F))
        num += dt * abs(v3) * abs(F) ** 3
        den += dt * abs(v3)
    Fm = (num / den) ** (1 / 3)
    rev_cycle = 4 * b['up'] / s['l']
    rev_h = rev_cycle * 3600 / b['cycle']
    L = (s['C'] / (fp * Fm)) ** 3 * 1e6
    m_max = MASS['spline'] + b['m_tool'] + b['m_max']
    F_static = m_max * (G + 2 * pi * b['up'] / b['t_v'] ** 2 / 1000)
    n_rel = 2 * b['up'] / b['t_v'] / s['l'] * 60
    return dict(m_z=m_z, Fm=Fm, Fmax=Fmax, Fmin=m_z * (G - 2 * pi * b['up'] / b['t_v'] ** 2 / 1000), F_bm=fp * Fm,
                L=L, rev_cycle=rev_cycle, rev_h=rev_h, Lh=L / rev_h, F_static=F_static, S0=s['C0'] / F_static,
                short=b['up'] < 3 * s['L2'], n_rel=n_rel, DN=s['d'] * n_rel, fp=fp)


def inertias34(size=20):
    """The rotating parts' inertias (kg m2), drawn as in the Fusion script: each nut's rotor a steel tube from the
    shaft to its pilot's diameter over the nut's length; the pulleys aluminium rings from the pilot (or the motor's
    shaft) to the pitch circle less 0.76 mm, the rim's width; the shaft solid steel over its length."""
    s = SP[size]
    lay = nut_layout(size)

    def tube(r1, r2, h, rho):
        m = pi * (r2 ** 2 - r1 ** 2) * h * rho
        return m, 0.5 * m * (r1 ** 2 + r2 ** 2) * 1e-6
    out = {}
    out['rotor_scr'] = tube(s['d'] / 2 + 0.3, s['F2'] / 2, s['L2'], RHO_STEEL)
    out['rotor_spl'] = tube(s['d'] / 2 + 0.3, s['F1'] / 2, s['L1'], RHO_STEEL)
    rim = BELT_W[3] + 3.0
    for j, (zm, zn) in PULLEYS.items():
        bore_n = (s['F2'] if j == 3 else s['F1']) / 2
        out['pulley_nut%d' % j] = tube(bore_n, (pd(zn) - 0.76) / 2, rim, RHO_AL)
        out['pulley_m%d' % j] = tube(4.0, (pd(zm) - 0.76) / 2, rim, RHO_AL)
    Ls = lay['shaft'][1] - lay['shaft'][0]
    m_sh = pi / 4 * s['d'] ** 2 * Ls * RHO_STEEL
    out['shaft'] = (m_sh, 0.5 * m_sh * (s['d'] / 2) ** 2 * 1e-6)
    return out


def motors34(size=20, payload=None, b=None):
    """The J3 and J4 motors' torques over the cycle. The screw nut turns at 2 pi v3 / l on the shaft plus J4's speed;
    its torque is the screw's, F l / (2 pi eta), and its rotor's and pulley's inertia; the J3 motor turns R3 times as
    fast. The spline nut turns with the shaft: its motor accelerates the shaft, the tool, the payload, its rotor and
    pulley, and holds the screw's torque on the shaft. The belts' and the screw's losses taken against the motor
    both ways."""
    b = b or brief()
    payload = b['m_rated'] if payload is None else payload
    s = SP[size]
    J = inertias34(size)
    m_z = MASS['spline'] + b['m_tool'] + payload
    I4 = payload * (b['k4'] / 1000) ** 2 + 0.5 * b['m_tool'] * 0.04 ** 2
    R3 = PULLEYS[3][1] / PULLEYS[3][0]
    R4 = PULLEYS[4][1] / PULLEYS[4][0]
    J_n3 = J['rotor_scr'][1] + J['pulley_nut3'][1]
    J_m3 = MOTOR34[3]['Jm'] + J['pulley_m3'][1]
    J_n4 = I4 + J['shaft'][1] + J['rotor_spl'][1] + J['pulley_nut4'][1]
    J_m4 = MOTOR34[4]['Jm'] + J['pulley_m4'][1]
    l = s['l'] / 1000
    out = {3: [], 4: []}
    for sg in (+1, -1):                                 # J4's turn with J3's sense, or against it
        r3, r4 = [], []
        for t, dt, v3, a3, w4, a4 in j34_timeline(b):
            F = m_z * (G + a3 / 1000)
            T_screw = F * l / (2 * pi * ETA_SCREW)
            wn = 2 * pi * v3 / 1000 / l + sg * w4
            an = 2 * pi * a3 / 1000 / l + sg * a4
            Tn = T_screw + J_n3 * an
            Tm3 = Tn / (R3 * ETA_BELT) + J_m3 * an * R3
            T_spl = J_n4 * a4 + T_screw
            Tm4 = T_spl / (R4 * ETA_BELT) + J_m4 * a4 * R4
            r3.append((dt, abs(wn) * R3 * 60 / (2 * pi), Tm3, Tn))
            r4.append((dt, abs(w4) * R4 * 60 / (2 * pi), Tm4, T_spl))
        for j, rows in ((3, r3), (4, r4)):
            tot = sum(r[0] for r in rows)
            out[j].append(dict(sense=sg, peak=max(abs(r[2]) for r in rows), rms=math.sqrt(sum(r[0] * r[2] ** 2 for r in rows) / tot),
                               nmax=max(r[1] for r in rows), nut=max(abs(r[3]) for r in rows)))
    res = {j: dict(max(out[j], key=lambda d: d['peak'])) for j in (3, 4)}
    for j in (3, 4):
        res[j]['rms'] = max(d['rms'] for d in out[j])
        res[j]['nmax'] = max(d['nmax'] for d in out[j])
        res[j]['nut'] = max(d['nut'] for d in out[j])
    hold = m_z * G * l / (2 * pi * ETA_SCREW)
    return dict(J=J, I4=I4, J_n3=J_n3, J_m3=J_m3, J_n4=J_n4, J_m4=J_m4, R3=R3, R4=R4, m3=res[3], m4=res[4],
                hold_spl=hold, hold_m4=hold / (R4 * ETA_BELT))


def gt3_rating(z, rpm, width, length):
    """Gates' rating for the small sprocket at the next tabled speed at or above rpm, times the width multiplier and
    the length factor."""
    i = next(k for k, r in enumerate(GT3_RPM) if r >= rpm)
    lf = next(f for Lmax, f in GT3_LENGTH_F if length <= Lmax)
    return GT3_NM[z][i] * GT3_WIDTH[width] * lf, GT3_RPM[i], lf


def belt_drive(j, x_m, y_m, size=20, Tpeak=None, rpm=None):
    """One belt drive by Gates' procedure (pp. 12, 103-104): pitch diameters, the centre distance in plan, the exact
    pitch length, the stock belt and the centre distance it gives (iterated), the arc of contact and the teeth in mesh
    on the small pulley, and the rating against the peak design load, the motor's peak times the service factor."""
    zm, zn = PULLEYS[j]
    D, d_ = pd(zn), pd(zm)

    def length(CD):
        phi = math.asin((D - d_) / (2 * CD))
        return 2 * CD * math.cos(phi) + pi * (D + d_) / 2 + phi * (D - d_)
    CD0 = math.hypot(LAY37['x_s'] - x_m, y_m)
    PL0 = length(CD0)
    stock = min((Lb for Lb in GT3_LENGTHS if Lb >= PL0))
    lo, hi = CD0, CD0 + 50
    for _ in range(100):
        mid = (lo + hi) / 2
        if length(mid) < stock:
            lo = mid
        else:
            hi = mid
    CD = (lo + hi) / 2
    arc = 180 - 60 * (D - d_) / CD
    tim = int(arc * zm / 360)
    rating, rpm_row, lf = gt3_rating(zm, rpm, int(BELT_W[j]), stock)
    design = Tpeak * SF_BELT
    Te = 2 * Tpeak / (d_ / 1000)
    x_m_new = LAY37['x_s'] - math.sqrt(CD ** 2 - y_m ** 2)
    return dict(j=j, z=(zm, zn), D=D, d=d_, CD0=CD0, PL0=PL0, stock=stock, teeth=stock // 3, CD=CD, arc=arc, tim=tim,
                x_m=x_m_new, y_m=y_m,
                rating=rating, rpm_row=rpm_row, lf=lf, design=design, ratio=rating / design, Te=Te,
                TT=2.286 * Tpeak / (d_ / 1000), total=2.571 * Tpeak / (d_ / 1000), v=pi * d_ * rpm / 60000)


def brakes34(size=20, b=None, factor=2.0, Tb3=0.32):
    """The two brakes with the power off. The load, the spline, the gripper and the largest payload, pulls on the
    screw nut with F l / (2 pi), the efficiency taken as 1, and turns the shaft with the same torque through the
    spline nut: the J3 brake must hold the first, the J4 brake the second, each through its belt, with a factor of 2.
    Then the J3 brake's stop of the shaft falling at 500 mm/s with the largest payload, the other inertias brought to
    the shaft's travel."""
    b = b or brief()
    s = SP[size]
    m = MASS['spline'] + b['m_tool'] + b['m_max']
    F = m * G
    l = s['l'] / 1000
    T = F * l / (2 * pi)
    R3 = PULLEYS[3][1] / PULLEYS[3][0]
    R4 = PULLEYS[4][1] / PULLEYS[4][0]
    J = inertias34(size)
    J_side = J['rotor_scr'][1] + J['pulley_nut3'][1] + (MOTOR34[3]['Jm'] + J['pulley_m3'][1]) * R3 ** 2
    m_eq = m + J_side * (2 * pi / l) ** 2
    F_b = Tb3 * R3 * 2 * pi / l
    a = (F_b - F) / m_eq
    v = 2 * b['up'] / b['t_v'] / 1000
    return dict(F=F, T=T, Tb3_min=factor * T / R3, Tb4_min=factor * T / R4, m_eq=m_eq, J_side=J_side, Tb3=Tb3, F_b=F_b,
                a=a, stop=v ** 2 / (2 * a) * 1000, t=v / a)


def depth_rule(size=20, s_full=100.0, n_min=1.5, b=None):
    """The shaft's bending decides how deep the tool may work at the full horizontal acceleration: the fatigue
    factor at s_full and at the full stroke, and the share of the acceleration that keeps the moment at its value
    at s_full when the tool is at the bottom, with the horizontal leg's time that gives."""
    b = b or brief()
    c1 = shaft_cantilever(size, b['m_rated'], stroke=s_full, b=b)
    c2 = shaft_cantilever(size, b['m_rated'], stroke=200.0, b=b)
    n1 = shaft_fatigue(c1['sigma'], SP[size]['d_in'])['n']
    n2 = shaft_fatigue(c2['sigma'], SP[size]['d_in'])['n']
    scale = c1['M_face'] / c2['M_face']
    return dict(s_full=s_full, c1=c1, c2=c2, n1=n1, n2=n2, scale=scale, t_leg=b['t_h'] / math.sqrt(scale), n_min=n_min)


def sp25_cost(b=None):
    """What the SP25 would cost the first joint: its shaft's extra mass at the tool, 427.2 mm from J1 on the
    horizontal leg, added to M11; the first joint's torque grows in proportion, and its life with the cube."""
    b = b or brief()
    m = {}
    for k in (20, 25):
        lay = nut_layout(k)
        m[k] = pi / 4 * SP[k]['d'] ** 2 * (lay['shaft'][1] - lay['shaft'][0]) * RHO_STEEL
    r = math.hypot(b['x0'], b['across'] / 2) / 1000
    p = inertia(b, b['m_rated'])
    qa = ik(b, b['x0'], -b['across'] / 2)
    M11 = dynamics(p, qa[1], 0, 0, 0, 0)[2]['M11']
    dM = (m[25] - m[20]) * r ** 2
    life = modules(b)['j1'][40]['Lh'] / ((M11 + dM) / M11) ** 3
    return dict(m20=m[20], m25=m[25], dm=m[25] - m[20], r=r, M11=M11, dM=dM, share=dM / M11, life=life)


def remass(m_spline, m_motors, b=None):
    """Chapter 36's sizing again with the masses at the second arm's end that Fusion measured in Chapter 37: the
    ball screw spline's group at the arm's end and the J3 and J4 motors at 80 mm from J2; MASS is restored."""
    b = b or brief()
    saved = dict(MASS)
    try:
        MASS['spline'] = m_spline
        MASS['j34_motors'] = m_motors
        p = inertia(b, b['m_rated'])
        qa = ik(b, b['x0'], -b['across'] / 2)
        Mx = dynamics(p, qa[1], 0, 0, 0, 0)[2]
        o = modules(b)
        return dict(M11=Mx['M11'], M22=Mx['M22'], j1=o['j1'][40], j2=o['j2'][20], j1_motor=o['j1_motor'],
                    j2_motor=o['j2_motor'], c=cycle(b, b['m_rated']))
    finally:
        MASS.clear()
        MASS.update(saved)


def tool_joints(b=None, size=20):
    """Chapter 37's sizing, in the order of the chapter."""
    b = b or brief()
    out = dict(sizes={k: shaft_cantilever(k, b['m_rated'], b=b) for k in SP})
    out['cant'] = out['sizes'][size]
    out['cant_max'] = shaft_cantilever(size, b['m_max'], b=b)
    out['fatigue'] = shaft_fatigue(out['cant']['sigma'], SP[size]['d_in'])
    out['screw'] = screw_life(size, b=b)
    out['screw_max'] = screw_life(size, b['m_max'], b=b)
    c = cycle(b, b['m_rated'])
    out['T4'] = c['T4']
    out['mot'] = motors34(size, b=b)
    J = out['mot']
    out['spline_T'] = J['m4']['nut']
    out['belts'] = {}
    for j, ym in ((3, LAY37['y_m']), (4, -LAY37['y_m'])):
        m = out['mot']['m%d' % j]
        out['belts'][j] = belt_drive(j, 80.0, ym, size, m['peak'], m['nmax'])
    out['brakes'] = brakes34(size, b)
    out['lay'] = nut_layout(size)
    out['depth'] = depth_rule(size, b=b)
    out['sp25'] = sp25_cost(b)
    out['eta_screw'] = ETA_SCREW
    return out


# ============================================================================================================
# Chapter 38: the SCARA put together. The layout of the robot (mm, world: J1's axis at x = y = 0, z up from the
# floor) and the arms' sections; every value assumed unless the text gives a source.
# ============================================================================================================
LAY38 = dict(z_col=200.0,          # the base column's top face, where the J1 module plate sits
             col_r=110.0, col_t=10.0, foot=300.0, foot_t=15.0, top_r=125.0, top_t=12.0,
             a1_w=110.0, a1_h=70.0, a1_t=4.0, a1_back=75.0, a1_front=50.0,     # the first arm: an aluminium box
             a2_back=45.0,          # the second arm reaches 45 mm behind J2
             skin=4.0, wall2=3.0)   # the second arm's skins and walls, with bosses where bolts and nuts need more


def heights(size1=40):
    """The robot's heights (world mm) from the layout: the J1 unit's input face, the first arm, the J2 module plate,
    the J2 output flange, the second arm's base plate."""
    L = LAY38
    z1 = L['z_col'] + 12.0                       # the J1 module plate is 12 thick on the column's top
    a1_bot = z1 + CSG[size1]['B']
    a1_top = a1_bot + L['a1_h']
    z2 = a1_top + 8.0                             # the J2 module plate, 8 thick, on the arm's top
    f2 = z2 + CSG[20]['B']
    arm2 = f2 + LAY37['t_base']                   # the second arm's base plate's top face
    return dict(z1=z1, a1_bot=a1_bot, a1_top=a1_top, z2=z2, f2=f2, arm2=arm2,
                tool_top0=arm2 + nut_layout(20)['tool_top'])


def box_section(b, h, t):
    """A hollow rectangle b wide and h high, walls t: area, I about the horizontal axis (vertical bending) and the
    vertical one (horizontal bending), and the torsion constant 4 Am^2 t / perimeter (Bredt)."""
    A = b * h - (b - 2 * t) * (h - 2 * t)
    Iv = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12
    Ih = (h * b ** 3 - (h - 2 * t) * (b - 2 * t) ** 3) / 12
    Am = (b - t) * (h - t)
    J = 4 * Am ** 2 * t / (2 * ((b - t) + (h - t)))
    return dict(A=A, Iv=Iv, Ih=Ih, J=J)



def mass_model(rows, size1=40, b=None, catalogue=True):
    """The robot's mass properties from Fusion's parts (rows: link, name, kg, x, y, z mm at rest, the part's own
    inertia about a vertical axis through its centre, kg m2): each motor at its specified mass and each strain wave
    unit at the catalogue's (their drawn centres kept, their own inertia scaled with the mass), the HIWIN nuts as
    drawn; the gripper and the rated payload added at the tool. The first link about J1, the second about J2, and
    the a, b, d of Chapter 35's equations of motion."""
    b = b or brief()
    drawn = {}
    for r in rows:
        for size in (size1, 20):
            if r[1].startswith('CSG-%d' % size) or r[1].endswith(' %d' % size) and r[1].split()[0] in (
                    'Cross', 'Output', 'Flexspline', 'Wave'):
                drawn[size] = drawn.get(size, 0.0) + r[2]
    spec = {'Servo motor J1': MOTOR[1]['mass'], 'Servo motor J2': MOTOR[2]['mass'],
            'Servo motor J3': MOTOR34[3]['mass'], 'Servo motor J4': MOTOR34[4]['mass']}
    out = []
    for link, name, m, x, y, z, Iz in rows:
        f = 1.0
        if catalogue and name in spec:
            f = spec[name] / m
        elif catalogue:
            for size in (size1, 20):
                if name.startswith('CSG-%d' % size) or (name.endswith(' %d' % size) and name.split()[0] in
                                                        ('Cross', 'Output', 'Flexspline', 'Wave')):
                    f = CSG[size]['mass'] / drawn[size]
        out.append((link, name, m * f, x, y, z, Iz * f))
    L1 = b['L1'] / 1000
    m_tool = b['m_tool'] + b['m_rated']
    I_tool = b['m_rated'] * (b['k4'] / 1000) ** 2 + 0.5 * b['m_tool'] * 0.04 ** 2
    one = [r for r in out if r[0] == 1]
    two = [r for r in out if r[0] == 2]
    I1 = sum(r[6] + r[2] * (r[3] ** 2 + r[4] ** 2) * 1e-6 for r in one)
    m1 = sum(r[2] for r in one)
    c1 = sum(r[2] * r[3] for r in one) / m1
    m2 = sum(r[2] for r in two) + m_tool
    mx = sum(r[2] * (r[3] - b['L1']) for r in two) + m_tool * b['L2']
    my = sum(r[2] * r[4] for r in two)
    d = sum(r[6] + r[2] * ((r[3] - b['L1']) ** 2 + r[4] ** 2) * 1e-6 for r in two) + I_tool + m_tool * (b['L2'] / 1000) ** 2
    c2 = mx / m2 / 1000
    bq = m2 * c2 * L1
    a = I1 + m2 * L1 ** 2 + d
    base = sum(r[2] for r in out if r[0] == 0)
    return dict(I1=I1, m1=m1, c1=c1, m2=m2, c2=c2, c2y=my / m2, d=d, b=bq, a=a, m_base=base, rows=out, drawn=drawn,
                m_two_parts=sum(r[2] for r in two))


def p_from_abd(a, bq, d, m2, L1=0.35, L2=0.25):
    """The equivalent two-body arm of dynamics() for given a, b, d: everything beyond J2 as one body."""
    c2 = bq / (m2 * L1)
    return dict(L1=L1, L2=L2, I1=a - m2 * L1 ** 2 - d, m2=m2, c2=c2, I2c=d - m2 * c2 ** 2, m3=0.0)


def sizing_measured(mm, size1=40, b=None):
    """Chapter 36's selection again on the measured arm: the J1 and J2 load patterns, the units' averages and lives,
    the motors, the peak torques; the bearing's moment with the arms stretched out."""
    b = b or brief()
    p = p_from_abd(mm['a'], mm['b'], mm['d'], mm['m2'])
    A = (b['x0'], -b['across'] / 2)
    B = (b['x0'], b['across'] / 2)
    rest = (b['cycle'] - 2 * b['t_h']) / 2
    r1 = leg_pattern(b, A, B, b['m_rated'], b['t_h'], 1, rest=rest, p=p)
    R0, R1 = (b['L1'] + b['L2'] - 30 - b['across'], 0.0), (b['L1'] + b['L2'] - 30, 0.0)
    r2 = leg_pattern(b, R0, R1, b['m_rated'], b['t_r'], 2, rest=b['t_v'] / 2, p=p)
    qa = ik(b, *A)
    M11 = dynamics(p, qa[1], 0, 0, 0, 0)[2]['M11']
    j1 = hd_select(r1, size1)
    j2 = hd_select(r2, 20)
    return dict(M11=M11, M22=mm['d'], j1=j1, j2=j2, m1=motor(r1, size1, 1), m2=motor(r2, 20, 2), p=p)



W_LIM = {1: 5.0, 2: 8.5}             # rad/s: the joints' speed near the ends of their ranges (Chapter 38)


def integration(rows, b=None):
    """Chapter 38's numbers on the robot as Fusion measured it (rows), the motors and the strain wave units at their
    specified and catalogue masses: the links against Chapter 35's assumptions; J1 on the CSG-40 and the CSG-45;
    J2 on the CSG-20 and its duty; the motors; the output bearings; the stops and the brakes."""
    b = b or brief()
    mm = mass_model(rows, 45, b)
    dr = mass_model(rows, 45, b, catalogue=False)
    out = dict(mm=mm, drawn=dr)
    out['s40'] = sizing_measured(mm, 40, b)
    out['s45'] = sizing_measured(mm, 45, b)
    s45 = out['s45']
    out['j2_share'] = s45['j2']['Lh'] / b['life_h']
    # the J1 bearing (CSG-45): the weights with the arms stretched out, the horizontal force on the leg
    L1, L2 = b['L1'] / 1000, b['L2'] / 1000
    m1, c1, m2, c2 = mm['m1'], mm['c1'] / 1000, mm['m2'], mm['c2']
    Fa = (m1 + m2) * G
    Mw = (m1 * c1 + m2 * (L1 + c2)) * G
    qa = ik(b, b['x0'], -b['across'] / 2)
    sx = m1 * c1 + m2 * (L1 + c2 * math.cos(qa[1]))
    sy = m2 * c2 * math.sin(qa[1])
    first = math.hypot(sx, sy)
    acc = 113.4851
    Fr = first * acc
    u = CSG[45]
    Lr = 0.05
    Mmax = Fr * (Lr + u['R']) + Mw
    P0 = Fr + 2 * Mmax / u['dp'] + 0.44 * Fa
    out['b1'] = dict(Fa=Fa, Mw=Mw, La=Mw / Fa, first=first, Fr=Fr, Mmax=Mmax, Mc=u['Mc'], P0=P0, fs=u['C0'] / P0)
    # the J2 bearing (CSG-20)
    u2 = CSG[20]
    c = cycle(b, b['m_rated'])
    a2 = c['radial']['a2']
    Fa2 = m2 * G
    Mw2 = m2 * c2 * G
    Fr2 = m2 * c2 * a2
    M2 = Fr2 * (0.04 + u2['R']) + Mw2
    P02 = Fr2 + 2 * M2 / u2['dp'] + 0.44 * Fa2
    out['b2'] = dict(Fa=Fa2, Mw=Mw2, Fr=Fr2, Mmax=M2, Mc=u2['Mc'], P0=P02, fs=u2['C0'] / P02)
    # the case bolts of the CSG-45: twelve M8 on 164 mm
    Fi = 0.75 * 36.6 * 970
    T = 12 * Fi * 0.15 * u['b'] / 2000
    out['bolts45'] = dict(Fi=Fi, T=T, n=T / u['Tmom'])
    # the stops: the motor's side through the teeth, as Chapter 36, with the measured inertias
    w1 = c['across']['w1']
    w2 = c['radial']['w2']
    Ja1, Ja2 = s45['M11'], mm['d']
    Jr1 = 2500 * (MOTOR[1]['Jm'] + u['Iwg'])
    Jr2 = 2500 * (MOTOR[2]['Jm'] + u2['Iwg'])
    st = {}
    for k, Ja, Jr, w0, Tmom, Trep, wl, rs, R, dp, C0 in (
            (1, Ja1, Jr1, w1, u['Tmom'], u['Trep'], W_LIM[1], 0.120, u['R'], u['dp'], u['C0']),
            (2, Ja2, Jr2, w2, u2['Tmom'], u2['Trep'], W_LIM[2], 0.100, u2['R'], u2['dp'], u2['C0'])):
        full_th = Jr * w0 ** 2 / Tmom
        full_F = Tmom * (Ja + Jr) / Jr / rs
        th = Jr * wl ** 2 / Trep                      # the buffer keeps the teeth under the repeated peak
        Tb = Trep * (Ja + Jr) / Jr
        F = Tb / rs
        P0 = F + 2 * F * R / dp                       # the stop's force at the output flange's face, R above the rollers
        st[k] = dict(Ja=Ja, Jr=Jr, w0=w0, E_full=0.5 * (Ja + Jr) * w0 ** 2, full_deg=math.degrees(full_th),
                     full_F=full_F, full_fs=C0 / (full_F + 2 * full_F * R / dp), w_lim=wl,
                     E=0.5 * (Ja + Jr) * wl ** 2, theta=th, deg=math.degrees(th), rs=rs, travel=th * rs * 1000,
                     T_buffer=Tb, F=F, P0=P0, fs=C0 / P0)
    out['stops'] = st
    # the brakes: J1 within 15 deg under the repeated peak; J2 against its momentary limit
    br = {}
    for k, Ja, Jr, w0, Trep, Tmom, Tb, ang in ((1, Ja1, Jr1, w1, u['Trep'], u['Tmom'], 12.0, 15.0),
                                               (2, Ja2, Jr2, w2, u2['Trep'], u2['Tmom'], 3.4, 15.0)):
        Jt = Ja + Jr
        lo = w0 ** 2 / (2 * math.radians(ang)) * Jt / 50
        hi_rep = Trep * Jt / (Ja * 50)
        hi_mom = Tmom * Jt / (Ja * 50)
        al = 50 * Tb / Jt
        stop = math.degrees(w0 ** 2 / (2 * al))
        wl = W_LIM[k]
        br[k] = dict(Tb=Tb, lo=lo, hi_rep=hi_rep, hi_mom=hi_mom, alpha=al, stop=stop, Tg=Ja * al, w0=w0,
                     stop_lim=math.degrees(wl ** 2 / (2 * al)))
    # where the tool would meet the first arm when J2 overruns: the flange's edge (25 mm) and the arm's half-width
    br[2]['q2_hit'] = 180.0 - math.degrees(math.asin((LAY38['a1_w'] / 2 + LAY37['flange_d'] / 2) / b['L2']))
    out['brakes'] = br
    return out



GROUPS38 = [   # Chapter 35's masses (Table 35.4) against the robot's groups as measured
    ('first arm', 3.5, lambda n, l: n == 'First arm beam'),
    ('J2 module', 3.0, lambda n, l: l == 1 and n != 'First arm beam' and not n.endswith(' 45')),
    ('second arm', 2.0, lambda n, l: n == 'Second arm structure'),
    ('J3 and J4 motors', 2.0, lambda n, l: l == 2 and ('motor' in n.lower() and 'Pulley' not in n or
                                                      n.startswith('Pulley') and 'motor' in n)),
    ('ball screw spline', 1.5, lambda n, l: l == 2 and (n.startswith(('Ball screw', 'Screw nut', 'Spline nut',
                                                                     'Pulley screw', 'Flange bolt', 'Tool flange',
                                                                     'Belt')) or n.startswith('Pulley') and 'nut' in n)),
]


def groups38(mm):
    """The measured groups' masses (the motors and units at their specified and catalogue masses) against Chapter
    35's, with what is left in each link (the gear's output flange and flexspline, carried by the next link)."""
    out = []
    for name, assumed, pick in GROUPS38:
        m = sum(r[2] for r in mm['rows'] if r[0] in (1, 2) and pick(r[1], r[0]))
        out.append((name, assumed, m))
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
