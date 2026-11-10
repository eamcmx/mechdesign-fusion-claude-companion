# -*- coding: utf-8 -*-
"""palletizer_design.py: the four-axis palletizer of Volume 2, Chapters 41 to 43.

Chapter 41: the brief (values marked 'assumed' are the brief's, not from a source), the arm's linkage (the drive
parallelogram and the two levelling parallelograms), its kinematics and working range, the standard cycle and the
joints' motions, the masses taken to start, the gravity torques by virtual work, the equations of motion (a planar
2 x 2 model for J2 and J3 worked by hand in the chapter, checked here against a full four-axis model built from each
body's position and orientation), and what each joint must give.

Frame: z up from the floor, x forward in the arm's plane, y to the left (right-handed); J1 turns the turret about
z. Angles: q2 = the lower arm's forward tilt from the vertical; q3 = the upper arm's elevation above the
horizontal, up positive; q4 = the tool flange about its vertical axis. Lengths in mm unless a name says otherwise.
"""
import math

pi = math.pi
G = 9.81


def brief():
    return dict(
        task='palletizing cartons, bags or a full layer from an infeed conveyor onto pallets',   # assumed
        payload=450.0,               # kg at the flange: gripper and load, assumed
        gripper=150.0,               # kg, the gripper alone (the empty return), assumed
        drop=250.0,                  # mm, the payload's centre below the flange face, assumed
        offcentre=50.0,              # mm, the payload's centre off the J4 axis at most, assumed
        layer=(1200.0, 800.0),       # mm, the layer the payload represents (EUR pallet), assumed
        reach=3150.0,                # mm, to the J4 axis, at least, assumed
        q1max=180.0, q4max=300.0,    # deg, J1 and J4 either way, assumed
        w_max=(85.0, 85.0, 85.0, 160.0),   # deg/s, the joints' largest speeds, assumed
        R=2000.0, z_low=700.0,       # mm, the standard cycle's radius (J4 axis) and the flange at its ends, assumed
        up=400.0, across=2000.0,     # mm, the standard cycle: 400 up, 2000 across (a chord), 400 down, and back
        q4_move=90.0,                # deg, J4 turns the load by a quarter turn on each horizontal leg, assumed
        t_h=1.75, t_v=0.80,          # s, a horizontal leg; a vertical leg, assumed
        t_ov=0.30, t_dw=0.25,        # s, each vertical leg overlapping its horizontal leg; grip and release, assumed
        cph=600.0,                   # cycles an hour with the full payload, assumed
        repeat=0.1,                  # mm, repeatability, assumed
        f_min=5.0,                   # Hz, the lowest frequency of the arm on its drives, assumed
        life_h=20000.0,              # h of running at the standard cycle, assumed
        n_motor=3000.0,              # rev/min, the motors' top speed taken for the ratios, assumed
    )


# the arm's layout (mm), the October model's, kept: J2 300 mm ahead of J1 and 1000 mm up
LAYOUT = dict(
    A=(300.0, 1000.0),           # J2, the shoulder; also the J3 crank's axis
    L2=1300.0,                   # lower arm, J2 to the elbow
    L3=1400.0,                   # upper arm, the elbow to the wrist pin
    r=450.0,                     # the crank's radius = the upper arm's rear lever (the drive parallelogram)
    D1=(-390.0, 150.0),          # levelling loop 1: the turret's pivot from J2 = the triangle's pivot from the elbow
    D2=(0.0, 250.0),             # levelling loop 2: the triangle's pivot from the elbow = the wrist's from the wrist pin
    e4=250.0,                    # the J4 axis ahead of the wrist pin
    h4=380.0,                    # the flange face below the wrist pin
)

LIMITS = dict(q2=(-42.0, 85.0), q3=(-75.0, 60.0), psi=(25.0, 140.0))   # deg; psi = 90 - q2 - q3


def pts(q2, q3, lay=LAYOUT):
    """Every pivot of the arm in its plane (x, z), mm, for q2 and q3 in degrees."""
    A = lay['A']
    a2 = (math.sin(math.radians(q2)), math.cos(math.radians(q2)))      # along the lower arm, A to B
    u3 = (math.cos(math.radians(q3)), math.sin(math.radians(q3)))      # along the upper arm, B to W
    add = lambda p, v, k=1.0: (p[0] + k * v[0], p[1] + k * v[1])
    B = add(A, a2, lay['L2'])
    W = add(B, u3, lay['L3'])
    C = add(A, u3, -lay['r'])                    # crank pin
    D = add(B, u3, -lay['r'])                    # the upper arm's rear lever
    E = add(A, lay['D1'])                        # levelling rod 1 on the turret
    F = add(B, lay['D1'])                        # ... on the triangle
    Gp = add(B, lay['D2'])                       # levelling rod 2 on the triangle
    H = add(W, lay['D2'])                        # ... on the wrist housing
    J4 = (W[0] + lay['e4'], W[1])                # the J4 axis at the wrist pin's height
    fl = (W[0] + lay['e4'], W[1] - lay['h4'])    # the flange face's centre
    return dict(A=A, B=B, W=W, C=C, D=D, E=E, F=F, G=Gp, H=H, J4=J4, fl=fl)


def psi(q2, q3):
    """The angle between the arms' directions, deg: 0 with the upper arm in line with the lower, 180 folded back."""
    return 90.0 - q2 - q3


def in_range(q2, q3, tol=1e-9):
    return (LIMITS['q2'][0] - tol <= q2 <= LIMITS['q2'][1] + tol and LIMITS['q3'][0] - tol <= q3 <= LIMITS['q3'][1] + tol
            and LIMITS['psi'][0] - tol <= psi(q2, q3) <= LIMITS['psi'][1] + tol)


def ik(R, z, lay=LAYOUT):
    """The flange's centre at the J4 axis radius R and height z (mm) -> q2, q3 (deg), elbow up (the only branch the
    parallelograms allow). Steps of Chapter 41: the wrist pin; the two-arm triangle by the law of cosines."""
    A = lay['A']
    x, h = R - lay['e4'] - A[0], z + lay['h4'] - A[1]     # the wrist pin from J2
    d = math.hypot(x, h)
    c = (d * d + lay['L2'] ** 2 - lay['L3'] ** 2) / (2 * lay['L2'] * d)
    if abs(c) > 1:
        raise ValueError('out of reach')
    gam = math.degrees(math.atan2(h, x))
    phi = gam + math.degrees(math.acos(c))                 # the lower arm's elevation
    B = (A[0] + lay['L2'] * math.cos(math.radians(phi)), A[1] + lay['L2'] * math.sin(math.radians(phi)))
    q3 = math.degrees(math.atan2(h + A[1] - B[1], x + A[0] - B[0]))
    return dict(q2=90.0 - phi, q3=q3, d=d, c=c, gam=gam, phi=phi, B=B, x=x, h=h)


def fk(q2, q3, lay=LAYOUT):
    p = pts(q2, q3, lay)
    return p['fl']


def range_polygon():
    """The working range in (q2, q3) is a polygon: the joints' limits and the limits on psi."""
    (a, b), (c, d), (p0, p1) = LIMITS['q2'], LIMITS['q3'], LIMITS['psi']
    s_lo, s_hi = 90.0 - p1, 90.0 - p0            # q2 + q3 between these
    return [(a, max(c, s_lo - a)), (a, min(d, s_hi - a)), (s_hi - d, d), (b, s_hi - b), (b, c), (s_lo - c, c)]


def workspace(n=400):
    """The flange's region in the arm's plane: the polygon's edges mapped by fk. Returns the outline and the
    largest reach with the height it is reached at."""
    poly = range_polygon()
    out = []
    for i in range(len(poly)):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % len(poly)]
        for k in range(n):
            s = k / n
            out.append(fk(x0 + (x1 - x0) * s, y0 + (y1 - y0) * s))
    best = max(out, key=lambda p: p[0])
    # the largest reach along the psi = 25 deg edge, by a fine search
    s_hi = 90.0 - LIMITS['psi'][0]
    rr = max(((fk(q2, s_hi - q2)[0], q2) for q2 in [LIMITS['q2'][0] + 0.001 * k for k in range(int(1000 * (LIMITS['q2'][1] - LIMITS['q2'][0])) + 1)]
              if LIMITS['q3'][0] <= s_hi - q2 <= LIMITS['q3'][1]))
    q2r = rr[1]
    return dict(outline=out, reach=rr[0], reach_q2=q2r, reach_q3=s_hi - q2r, reach_z=fk(q2r, s_hi - q2r)[1],
                top=max(p[1] for p in out), bottom=min(p[1] for p in out), best=best)


def reach_at(z, lay=LAYOUT):
    """The largest J4-axis radius at a flange height z: on the psi = 25 deg edge unless a joint limit bites."""
    lo = None
    best = 0.0
    for k in range(0, 12701):
        q2 = LIMITS['q2'][0] + k * 0.01
        # the q3 that puts the flange at z for this q2: z = A_z + L2 cos q2 + L3 sin q3 - h4
        s = (z + lay['h4'] - lay['A'][1] - lay['L2'] * math.cos(math.radians(q2))) / lay['L3']
        if abs(s) > 1:
            continue
        q3 = math.degrees(math.asin(s))
        if in_range(q2, q3):
            best = max(best, fk(q2, q3)[0])
    return best


def flattening():
    """Where each parallelogram flattens (its four pivots in line), deg."""
    d1 = LAYOUT['D1']
    t1 = math.degrees(math.atan2(d1[0], d1[1]))    # the tilt from the vertical of the offset D1
    return dict(drive=(0.0, 180.0),              # psi: the arms in line or folded
                loop1=(t1, t1 + 180.0),          # q2: the lower arm along D1
                loop2=(-90.0, 90.0))             # q3: the upper arm along D2 (vertical)


# ------------------------------------------------------------------------------------------------------------------
# The masses taken to start: the October model's groups, as Fusion measured them (pal-v2, home pose), rounded.
# Each group moves one of seven ways: 'turret' with J1 only; 'q2@A', 'q2@E' turn with q2 about J2 or rod 1's turret
# pivot; 'q3@A', 'q3@B', 'q3@G' turn with q3 about J2 (the crank), the elbow (the upper arm) or rod 2's pivot;
# 'trB' translates with the elbow, 'trC' translates with the crank pin and turns with q2 (the parallel rod), 'trW'
# translates with the wrist pin (and turns with J4 if it is the flange or the payload).
# c: the centre along the link from its pivot, mm (negative: behind it); off: the centre's offset from the moving
# point for translating groups (x, z), mm; y: the centre's distance from the arm's plane, mm.
# Ia, It, In: kg m2 about the centre: the link's own axis, the y axis, the third axis.
# ------------------------------------------------------------------------------------------------------------------
BODIES = [
    dict(name='turret', m=967.0, move='turret', off=(-220.0, -125.0), y=-20.0, Izz=180.0),
    dict(name='lower arm', m=431.0, move='q2@A', c=535.0, y=-25.0, Ia=12.0, It=138.0, In=137.0),
    dict(name='crank and counterweight', m=233.0, move='q3@A', c=-485.0, y=495.0, Ia=4.0, It=22.0, In=22.0),
    dict(name='parallel rod', m=29.0, move='trC', c=650.0, y=465.0, Ia=0.0, It=5.5, In=5.5),
    dict(name='levelling rod 1', m=12.0, move='q2@E', c=650.0, y=-245.0, Ia=0.0, It=2.6, In=2.6),
    dict(name='triangle', m=36.0, move='trB', off=(-140.0, 125.0), y=-205.0, Izz=0.8),
    dict(name='upper arm', m=485.0, move='q3@B', c=535.0, y=40.0, Ia=18.0, It=117.0, In=128.0),
    dict(name='levelling rod 2', m=13.0, move='q3@G', c=700.0, y=-245.0, Ia=0.0, It=3.2, In=3.2),
    dict(name='wrist housing', m=156.0, move='trW', off=(135.0, -125.0), y=-20.0, Izz=4.5),
    dict(name='tool flange', m=13.0, move='trW', off=(250.0, -360.0), y=0.0, Izz=0.1, spin=True),
    dict(name='balancer', m=33.0, move='turret', off=(-440.0, 40.0), y=0.0, Izz=0.7),
]
BASE = dict(m=1360.0)          # the fixed base with the slewing ring's outer ring: carried by the floor only


def payload_body(m, b=None):
    b = b or brief()
    a, c = b['layer'][0] / 1000, b['layer'][1] / 1000
    return dict(name='payload', m=m, move='trW', off=(LAYOUT['e4'], -LAYOUT['h4'] - b['drop']), y=0.0,
                Izz=m * (a * a + c * c) / 12, spin=True)


def bodies(payload=None):
    b = brief()
    m = b['payload'] if payload is None else payload
    return BODIES + ([payload_body(m, b)] if m > 0 else [])


# ------------------------------------------------------------------------------------------------------------------
# Gravity by virtual work (Chapter 41): hold the crank and turn the lower arm, and the parallelograms move every
# group either about a fixed pivot with q2 or in translation with the elbow; hold the lower arm and turn the crank,
# and every group either turns with q3 or translates with the crank pin, the elbow's triangle staying put.
# ------------------------------------------------------------------------------------------------------------------
def gravity_sums(payload=None):
    """S2 and S3, kg m: the holding torques are T2 = g S2 sin q2 and T3 = g S3 cos q3, each of one angle only."""
    L2, L3, r = LAYOUT['L2'] / 1000, LAYOUT['L3'] / 1000, LAYOUT['r'] / 1000
    S2 = S3 = 0.0
    rows = []
    for bd in bodies(payload):
        mv, m = bd['move'], bd['m']
        s2 = s3 = 0.0
        if mv in ('q2@A', 'q2@E'):
            s2 = m * bd['c'] / 1000
        elif mv == 'trC':
            s2, s3 = m * bd['c'] / 1000, -m * r      # it turns with q2 about the crank pin, which moves with q3
        elif mv in ('trB', 'q3@B', 'q3@G', 'trW'):
            s2 = m * L2
            if mv in ('q3@B', 'q3@G'):
                s3 = m * bd['c'] / 1000
            elif mv == 'trW':
                s3 = m * L3
        elif mv == 'q3@A':
            s3 = m * bd['c'] / 1000
        S2 += s2
        S3 += s3
        rows.append((bd['name'], m, s2, s3))
    return dict(S2=S2, S3=S3, rows=rows)


def T2_grav(q2, payload=None):
    return G * gravity_sums(payload)['S2'] * math.sin(math.radians(q2))


def T3_grav(q3, payload=None):
    return G * gravity_sums(payload)['S3'] * math.cos(math.radians(q3))


# ------------------------------------------------------------------------------------------------------------------
# The planar equations of motion for J2 and J3 (Chapter 41): every group's centre moves with the elbow (L2 along
# the lower arm's normal) and with its own turning, so the kinetic energy is 1/2 M22 qd2^2 + M23 qd2 qd3 +
# 1/2 M33 qd3^2 with M22 and M33 constant and M23 = -K23 sin(q2 + q3).
# ------------------------------------------------------------------------------------------------------------------
def planar_mass(payload=None):
    L2, L3, r = LAYOUT['L2'] / 1000, LAYOUT['L3'] / 1000, LAYOUT['r'] / 1000
    M22 = M33 = K23 = 0.0
    rows = []
    for bd in bodies(payload):
        mv, m = bd['move'], bd['m']
        a22 = a33 = k = 0.0
        if mv in ('q2@A', 'q2@E'):
            c = bd['c'] / 1000
            a22 = m * c * c + bd['It']
        elif mv == 'trC':                     # the crank pin moves with q3 (r behind J2), the rod turns with q2
            c = bd['c'] / 1000
            a22, a33, k = m * c * c + bd['It'], m * r * r, -m * r * c
        elif mv == 'trB':
            a22 = m * L2 * L2
        elif mv in ('q3@B', 'q3@G'):
            c = bd['c'] / 1000
            a22, a33, k = m * L2 * L2, m * c * c + bd['It'], m * L2 * c
        elif mv == 'trW':
            a22, a33, k = m * L2 * L2, m * L3 * L3, m * L2 * L3
        elif mv == 'q3@A':
            c = bd['c'] / 1000
            a33 = m * c * c + bd['It']
        M22 += a22
        M33 += a33
        K23 += k
        rows.append((bd['name'], a22, a33, k))
    return dict(M22=M22, M33=M33, K23=K23, rows=rows)


def planar_torques(q2, q3, qd2, qd3, qdd2, qdd3, payload=None, pm=None, gs=None):
    """The torques J2 and J3 must give in the sense of increasing q2 and q3, N m (angles in deg, rates in rad/s)."""
    pm = pm or planar_mass(payload)
    gs = gs or gravity_sums(payload)
    s = math.radians(q2 + q3)
    M23 = -pm['K23'] * math.sin(s)
    k = pm['K23'] * math.cos(s)
    t2 = pm['M22'] * qdd2 + M23 * qdd3 - k * qd3 ** 2 - G * gs['S2'] * math.sin(math.radians(q2))
    t3 = pm['M33'] * qdd3 + M23 * qdd2 - k * qd2 ** 2 + G * gs['S3'] * math.cos(math.radians(q3))
    return t2, t3


# ------------------------------------------------------------------------------------------------------------------
# The full four-axis model, from each group's centre and orientation (no hand algebra): the check of the planar
# model and the source of J1's torque with its couplings.
# ------------------------------------------------------------------------------------------------------------------
def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return ((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c))


def _rot_z(t):
    c, s = math.cos(t), math.sin(t)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def _mm(A, B):
    return tuple(tuple(sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)) for i in range(3))


def _mv(A, v):
    return tuple(sum(A[i][k] * v[k] for k in range(3)) for i in range(3))


def _T(A):
    return tuple(tuple(A[j][i] for j in range(3)) for i in range(3))


def body_pose(bd, q):
    """The centre (m) and orientation of a group in the world for q = (q1, q2, q3, q4) in rad."""
    q1, q2, q3, q4 = q
    d2, d3 = math.degrees(q2), math.degrees(q3)
    P = pts(d2, d3)
    mv = bd['move']
    if mv == 'turret':
        x, z = LAYOUT['A'][0] + bd['off'][0], LAYOUT['A'][1] + bd['off'][1]
        R = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    elif mv in ('q2@A', 'q2@E', 'trC'):
        piv = {'q2@A': P['A'], 'q2@E': P['E'], 'trC': P['C']}[mv]
        x, z = piv[0] + bd['c'] * math.sin(q2), piv[1] + bd['c'] * math.cos(q2)
        R = _rot_y(q2)            # the link's own axis is z in its frame (vertical at q2 = 0)
    elif mv in ('q3@A', 'q3@B', 'q3@G'):
        piv = {'q3@A': P['A'], 'q3@B': P['B'], 'q3@G': P['G']}[mv]
        x, z = piv[0] + bd['c'] * math.cos(q3), piv[1] + bd['c'] * math.sin(q3)
        R = _rot_y(-q3)           # the link's own axis is x in its frame (horizontal at q3 = 0)
    elif mv == 'trB':
        x, z = P['B'][0] + bd['off'][0], P['B'][1] + bd['off'][1]
        R = ((1, 0, 0), (0, 1, 0), (0, 0, 1))
    elif mv == 'trW':
        x, z = P['W'][0] + bd['off'][0], P['W'][1] + bd['off'][1]
        R = _rot_z(q4) if bd.get('spin') else ((1, 0, 0), (0, 1, 0), (0, 0, 1))
        if bd.get('spin'):        # J4 is at x = W + e4: the group turns about it
            dx, dy = x - (P['W'][0] + LAYOUT['e4']), bd['y']
            cq, sq = math.cos(q4), math.sin(q4)
            x, y4 = P['W'][0] + LAYOUT['e4'] + cq * dx - sq * dy, sq * dx + cq * dy
    else:
        raise ValueError(mv)
    y = bd['y'] if not (mv == 'trW' and bd.get('spin')) else y4
    Rz = _rot_z(q1)
    c = _mv(Rz, (x / 1000, y / 1000, z / 1000))
    return c, _mm(Rz, R)


def body_inertia(bd):
    """The inertia tensor about the centre in the group's own frame (kg m2)."""
    mv = bd['move']
    if mv in ('q2@A', 'q2@E', 'trC'):          # own axis z
        return ((bd['In'], 0, 0), (0, bd['It'], 0), (0, 0, bd['Ia']))
    if mv in ('q3@A', 'q3@B', 'q3@G'):         # own axis x
        return ((bd['Ia'], 0, 0), (0, bd['It'], 0), (0, 0, bd['In']))
    Izz = bd['Izz']
    return ((Izz, 0, 0), (0, Izz, 0), (0, 0, Izz))   # translating groups: only Izz enters (they never tilt)


def _omega_cols(bd, q):
    """omega = sum of column_j * qd_j: every group turns with q1 about z; q2 groups about +y1, q3 groups about -y1;
    the flange and the payload with q4 about z."""
    q1 = q[0]
    z = (0.0, 0.0, 1.0)
    y1 = (-math.sin(q1), math.cos(q1), 0.0)
    zero = (0.0, 0.0, 0.0)
    mv = bd['move']
    c2 = y1 if mv in ('q2@A', 'q2@E', 'trC') else zero
    c3 = tuple(-v for v in y1) if mv in ('q3@A', 'q3@B', 'q3@G') else zero
    c4 = z if bd.get('spin') else zero
    return (z, c2, c3, c4)


def mass_matrix(q, payload=None, h=1e-6):
    bl = bodies(payload)
    M = [[0.0] * 4 for _ in range(4)]
    for bd in bl:
        c0, R = body_pose(bd, q)
        Jv = []
        for j in range(4):
            qp = list(q); qm = list(q)
            qp[j] += h; qm[j] -= h
            cp, cm = body_pose(bd, qp)[0], body_pose(bd, qm)[0]
            Jv.append(tuple((cp[i] - cm[i]) / (2 * h) for i in range(3)))
        Iw = _mm(_mm(R, body_inertia(bd)), _T(R))
        W = _omega_cols(bd, q)
        for i in range(4):
            for j in range(4):
                M[i][j] += bd['m'] * sum(Jv[i][k] * Jv[j][k] for k in range(3))
                M[i][j] += sum(W[i][a] * Iw[a][b] * W[j][b] for a in range(3) for b in range(3))
    return M


def potential(q, payload=None):
    return sum(bd['m'] * G * body_pose(bd, q)[0][2] for bd in bodies(payload))


def torques(q, qd, qdd, payload=None, h=1e-5):
    """tau = M qdd + c(q, qd) + dU/dq, the full four-axis model (rad, rad/s, rad/s2) -> N m."""
    M = mass_matrix(q, payload)
    dM = []
    for k in range(4):
        qp = list(q); qm = list(q)
        qp[k] += h; qm[k] -= h
        Mp, Mm = mass_matrix(qp, payload), mass_matrix(qm, payload)
        dM.append([[(Mp[i][j] - Mm[i][j]) / (2 * h) for j in range(4)] for i in range(4)])
    tau = []
    for i in range(4):
        t = sum(M[i][j] * qdd[j] for j in range(4))
        for j in range(4):
            for k in range(4):
                t += (dM[k][i][j] - 0.5 * dM[i][j][k]) * qd[j] * qd[k]
        qp = list(q); qm = list(q)
        qp[i] += h; qm[i] -= h
        t += (potential(qp, payload) - potential(qm, payload)) / (2 * h)
        tau.append(t)
    return tau


def izz(q2, q3, payload=None, q4=0.0):
    """J1's moment of inertia, kg m2 (q in deg)."""
    return mass_matrix((0.0, math.radians(q2), math.radians(q3), math.radians(q4)), payload)[0][0]


def izz_rows(q2, q3, payload=None):
    """J1's inertia group by group: m (x^2 + y^2) and the group's own part about the vertical."""
    rows = []
    for bd in bodies(payload):
        c, R = body_pose(bd, (0.0, math.radians(q2), math.radians(q3), 0.0))
        Iw = _mm(_mm(R, body_inertia(bd)), _T(R))
        rows.append((bd['name'], bd['m'], c[0] * 1000, c[1] * 1000, bd['m'] * (c[0] ** 2 + c[1] ** 2), Iw[2][2]))
    return rows


# ------------------------------------------------------------------------------------------------------------------
# The standard cycle (Chapter 41): 400 up, 2000 across, 400 down and back, every leg a cycloid in joint space; each
# vertical leg overlaps its horizontal leg by t_ov; the gripper grips and releases for t_dw at the two ends.
# ------------------------------------------------------------------------------------------------------------------
def cycloid(dq, T, t):
    if t <= 0:
        return 0.0, 0.0, 0.0
    if t >= T:
        return dq, 0.0, 0.0
    s = t / T
    return (dq * (s - math.sin(2 * pi * s) / (2 * pi)), dq / T * (1 - math.cos(2 * pi * s)),
            2 * pi * dq / T ** 2 * math.sin(2 * pi * s))


def waypoints(b=None):
    """The cycle's four points: pick low, pick high, place high, place low; q1 either side of the chord's middle."""
    b = b or brief()
    a1 = math.degrees(math.asin(b['across'] / 2 / b['R']))
    lo, hi = ik(b['R'], b['z_low']), ik(b['R'], b['z_low'] + b['up'])
    return dict(a1=a1, lo=lo, hi=hi,
                P=[(-a1, lo['q2'], lo['q3'], 0.0), (-a1, hi['q2'], hi['q3'], 0.0),
                   (a1, hi['q2'], hi['q3'], b['q4_move']), (a1, lo['q2'], lo['q3'], b['q4_move'])])


def timeline(b=None):
    """The legs on one clock: (start, duration, from, to, loaded). The cycle: dwell (grip), up, across, down, dwell
    (release), up, back, down; each vertical leg overlaps the horizontal leg next to it by t_ov."""
    b = b or brief()
    P = waypoints(b)['P']
    th, tv, to, td = b['t_h'], b['t_v'], b['t_ov'], b['t_dw']
    legs = []
    t = td
    legs.append((t, tv, P[0], P[1], True)); t += tv - to
    legs.append((t, th, P[1], P[2], True)); t += th - to
    legs.append((t, tv, P[2], P[3], True)); t += tv + td
    legs.append((t, tv, P[3], P[2], False)); t += tv - to
    legs.append((t, th, P[2], P[1], False)); t += th - to
    legs.append((t, tv, P[1], P[0], False)); t += tv
    return legs, t


def cycle_state(t, b=None, legs=None):
    """q (deg), qd, qdd (rad/s, rad/s2) and whether the payload is held, at time t of the cycle."""
    b = b or brief()
    if legs is None:
        legs, _ = timeline(b)
    q = list(legs[0][2])
    qd = [0.0] * 4
    qdd = [0.0] * 4
    for (t0, T, a, c, _) in legs:
        for j in range(4):
            dq = math.radians(c[j] - a[j])
            s, v, acc = cycloid(dq, T, t - t0)
            q[j] += math.degrees(s)
            qd[j] += v
            qdd[j] += acc
    loaded = t < legs[2][0] + legs[2][1] + b['t_dw']     # from the grip to the end of the release, both dwells
    return q, qd, qdd, loaded


def cycle_run(b=None, n=1200, full=True):
    """Every joint's torque over the cycle with the full model (full=True) or the planar one for J2 and J3: the
    peaks, the RMS values over the cycle, the largest powers, and where each peak happens."""
    b = b or brief()
    legs, Tc = timeline(b)
    rows = []
    gs = {True: gravity_sums(b['payload']), False: gravity_sums(b['gripper'])}
    pm = {True: planar_mass(b['payload']), False: planar_mass(b['gripper'])}
    for i in range(n + 1):
        t = Tc * i / n
        q, qd, qdd, loaded = cycle_state(t, b, legs)
        m = b['payload'] if loaded else b['gripper']
        if full:
            tau = torques(tuple(math.radians(v) for v in q), qd, qdd, m)
        else:
            t2, t3 = planar_torques(q[1], q[2], qd[1], qd[2], qdd[1], qdd[2], pm=pm[loaded], gs=gs[loaded])
            tau = [float('nan'), t2, t3, float('nan')]
        rows.append((t, q, qd, qdd, tau, loaded))
    out = dict(Tc=Tc, cph=3600.0 / Tc, rows=rows)
    for j in range(4):
        if not full and j in (0, 3):
            continue
        tj = [r[4][j] for r in rows]
        k = max(range(len(tj)), key=lambda i: abs(tj[i]))
        out['peak%d' % (j + 1)] = abs(tj[k])
        out['peak%d_t' % (j + 1)] = rows[k][0]
        out['rms%d' % (j + 1)] = math.sqrt(sum(v * v for v in tj[:-1]) / (len(tj) - 1))
        out['P%d' % (j + 1)] = max(abs(r[4][j] * r[2][j]) for r in rows)
        out['w%d' % (j + 1)] = max(abs(r[2][j]) for r in rows)
        out['a%d' % (j + 1)] = max(abs(r[3][j]) for r in rows)
    return out


def leg_peaks(b=None):
    """Each leg's cycloid: the joints' moves and their largest speeds and accelerations (closed form)."""
    b = b or brief()
    W = waypoints(b)
    lo, hi = W['lo'], W['hi']
    dq1 = math.radians(2 * W['a1'])
    dq2, dq3 = math.radians(hi['q2'] - lo['q2']), math.radians(hi['q3'] - lo['q3'])
    dq4 = math.radians(b['q4_move'])
    th, tv = b['t_h'], b['t_v']
    w = lambda d, T: 2 * abs(d) / T
    a = lambda d, T: 2 * pi * abs(d) / T ** 2
    return dict(dq1=dq1, dq2=dq2, dq3=dq3, dq4=dq4, w1=w(dq1, th), a1=a(dq1, th), w2=w(dq2, tv), a2=a(dq2, tv),
                w3=w(dq3, tv), a3=a(dq3, tv), w4=w(dq4, th), a4=a(dq4, th))


def cycle_time(b=None):
    b = b or brief()
    return 2 * b['t_h'] + 4 * (b['t_v'] - b['t_ov']) + 2 * b['t_dw']


# ------------------------------------------------------------------------------------------------------------------
# What else the brief asks: the ratios, the stiffness for the lowest frequency, the J1 bearing's loads, the stops'
# energies and the brakes' torques.
# ------------------------------------------------------------------------------------------------------------------
def ratios(b=None):
    b = b or brief()
    out = {}
    for j, w in enumerate(b['w_max']):
        n_out = w / 360.0 * 60.0                   # rev/min at the joint
        out[j + 1] = dict(n_out=n_out, i_max=b['n_motor'] / n_out)
    return out


def stiffness(b=None):
    """The torsional stiffness each joint needs for f_min with the arm at the cycle's high point (K = (2 pi f)^2 I)."""
    b = b or brief()
    hi = waypoints(b)['hi']
    pm = planar_mass(b['payload'])
    k = (2 * pi * b['f_min']) ** 2
    I1 = izz(hi['q2'], hi['q3'], b['payload'])
    reach = workspace()
    I1r = izz(reach['reach_q2'], reach['reach_q3'], b['payload'])
    return dict(k=k, I1=I1, I1_reach=I1r, M22=pm['M22'], M33=pm['M33'], K1=k * I1, K1_reach=k * I1r,
                K2=k * pm['M22'], K3=k * pm['M33'])


def j1_bearing(q2, q3, payload=None):
    """The turret's weight and tipping moment on the J1 bearing, the arm's groups and the payload (static): the
    axial force (N) and the moment about the bearing's centre (N m) from the centres' distance from J1's axis."""
    Fa = M = 0.0
    for bd in bodies(payload):
        c, _ = body_pose(bd, (0.0, math.radians(q2), math.radians(q3), 0.0))
        Fa += bd['m'] * G
        M += bd['m'] * G * c[0]
    return dict(Fa=Fa, M=M, mass=Fa / G)


def stops(b=None):
    """The energy each arm joint carries at its largest speed with the payload, at the worst pose for J1 (full
    reach); J2 and J3 with their planar inertias."""
    b = b or brief()
    w = [math.radians(v) for v in b['w_max']]
    st = stiffness(b)
    pm = planar_mass(b['payload'])
    return dict(E1=0.5 * st['I1_reach'] * w[0] ** 2, E2=0.5 * pm['M22'] * w[1] ** 2, E3=0.5 * pm['M33'] * w[2] ** 2)


def holding(b=None):
    """The largest gravity torque each of J2 and J3 must hold, with the full payload, over their ranges."""
    b = b or brief()
    gs = gravity_sums(b['payload'])
    q2max = max(abs(LIMITS['q2'][0]), abs(LIMITS['q2'][1]))
    T2 = G * gs['S2'] * math.sin(math.radians(q2max))
    T3 = G * gs['S3'] * 1.0                     # at q3 = 0, inside its range
    return dict(T2=T2, T3=T3, q2=q2max, S2=gs['S2'], S3=gs['S3'])


def radius_range(z, lay=LAYOUT):
    """The J4 axis's least and largest radius at a flange height z, and the pose at each. The arm's map is one to one
    inside the range (psi stays away from 0 and 180 deg), so the extremes lie where the range's edges cross the
    height: each of the hexagon's six edges is searched for its crossings, and each crossing found by bisection."""
    poly = range_polygon()
    zf = lambda q2, q3: pts(q2, q3, lay)['fl'][1] - z
    cands = []
    for i in range(len(poly)):
        (a2, a3), (b2, b3) = poly[i], poly[(i + 1) % len(poly)]
        at = lambda t: (a2 + (b2 - a2) * t, a3 + (b3 - a3) * t)
        n = 400
        prev = zf(*at(0.0))
        for k in range(1, n + 1):
            t1 = k / n
            cur = zf(*at(t1))
            if prev == 0.0:
                cands.append(at((k - 1) / n))
            if prev * cur < 0:
                lo_t, hi_t, f_lo = (k - 1) / n, t1, prev
                for _ in range(80):
                    mid = 0.5 * (lo_t + hi_t)
                    f_mid = zf(*at(mid))
                    if f_lo * f_mid <= 0:
                        hi_t = mid
                    else:
                        lo_t, f_lo = mid, f_mid
                cands.append(at(0.5 * (lo_t + hi_t)))
            prev = cur
    xs = [(pts(q2, q3, lay)['fl'][0], (q2, q3)) for q2, q3 in cands]
    lo, hi = min(xs), max(xs)
    return dict(r_min=lo[0], q_min=lo[1], r_max=hi[0], q_max=hi[1])


def binding(q2, q3, tol=0.05):
    """Which limits a pose sits on."""
    out = []
    for name, v, (a, c) in (('q2', q2, LIMITS['q2']), ('q3', q3, LIMITS['q3']), ('psi', psi(q2, q3), LIMITS['psi'])):
        if abs(v - a) < tol:
            out.append('%s = %g' % (name, a))
        if abs(v - c) < tol:
            out.append('%s = %g' % (name, c))
    return out


def split_table(splits=((1200.0, 1500.0), (1300.0, 1400.0), (1400.0, 1300.0)), b=None):
    """Three ways to split 2700 mm between the arms: the least radius at the bottom layer (flange 600 mm), the
    largest at the top (2400 mm), and J2's and J3's largest holding torques with the full payload, the arms' masses,
    centres and rods taken in proportion to their lengths, everything else as in BODIES."""
    b = b or brief()
    global BODIES
    keep = BODIES
    rows = []
    try:
        for L2, L3 in splits:
            lay = dict(LAYOUT)
            lay['L2'], lay['L3'] = L2, L3
            k2, k3 = L2 / LAYOUT['L2'], L3 / LAYOUT['L3']
            nb = []
            for bd in keep:
                bd = dict(bd)
                if bd['move'] in ('q2@A', 'q2@E', 'trC'):
                    bd['m'] *= k2
                    bd['c'] *= k2
                elif bd['move'] in ('q3@B', 'q3@G'):
                    bd['m'] *= k3
                    bd['c'] *= k3
                nb.append(bd)
            BODIES = nb
            saved = dict(LAYOUT)
            LAYOUT.update(lay)
            try:
                gs = gravity_sums(b['payload'])
                lo = radius_range(600.0, lay)
                top = radius_range(2400.0, lay)
            finally:
                LAYOUT.update(saved)
            rows.append(dict(L2=L2, L3=L3, r_min600=lo['r_min'], bind600=binding(*lo['q_min']), r_max2400=top['r_max'],
                             T2=G * gs['S2'] * math.sin(math.radians(LIMITS['q2'][1])), T3=G * gs['S3']))
    finally:
        BODIES = keep
    return rows


def stations(b=None):
    """The pallet stations the brief asks for: the pallet's 800 mm side towards the robot, from 1600 to 2400 mm
    out; its far corners and the radius each layer needs (assumed)."""
    b = b or brief()
    r0, r1, half = 1600.0, 2400.0, 600.0
    return dict(r0=r0, r1=r1, half=half, corner=math.hypot(r1, half), z_bottom=600.0, z_top=2400.0,
                at_bottom=radius_range(600.0), at_top=radius_range(2400.0))


def wrist_moment(b=None, off=0.0):
    """The moment of the wrist group's weight about the wrist pin, N m (the housing, the flange and the payload,
    each at its offset ahead of the pin; the payload's centre 'off' mm farther out)."""
    b = b or brief()
    s = 0.0
    for bd in bodies(b['payload']):
        if bd['move'] == 'trW':
            x = bd['off'][0] + (off if bd['name'] == 'payload' else 0.0)
            s += bd['m'] * x / 1000
    return G * s


def rod_forces(q2, q3, b=None):
    """The static forces in the three rods (N, tension positive away from the pin it pulls): the parallel rod from the
    moment the upper arm needs about the elbow, M_B = F r sin psi; rod 2 from the wrist group's moment about the
    wrist pin, M_W = F2 h cos q3 (h = 250 mm, the lever D2); rod 1 from the same moment carried by the triangle about
    the elbow, M_W = F1 |D1 x a2|."""
    b = b or brief()
    L3 = LAYOUT['L3'] / 1000
    MB = 0.0
    for bd in bodies(b['payload']):
        if bd['move'] == 'q3@B':
            MB += bd['m'] * bd['c'] / 1000
        elif bd['move'] == 'q3@G':
            MB += 0.5 * bd['m'] * L3                 # half of rod 2's weight reaches the wrist pin
        elif bd['move'] == 'trW':
            MB += bd['m'] * L3
    MB *= G * math.cos(math.radians(q3))
    MW = wrist_moment(b)
    r = LAYOUT['r'] / 1000
    h = LAYOUT['D2'][1] / 1000
    d1x, d1z = LAYOUT['D1'][0] / 1000, LAYOUT['D1'][1] / 1000
    a2 = (math.sin(math.radians(q2)), math.cos(math.radians(q2)))
    lever1 = abs(d1x * a2[1] - d1z * a2[0])
    return dict(MB=MB, MW=MW, F=MB / (r * math.sin(math.radians(psi(q2, q3)))),
                F2=MW / (h * math.cos(math.radians(q3))), F1=MW / lever1, lever1=lever1)


def life_counts(b=None):
    """The cycles in the life and each joint's travel in a cycle."""
    b = b or brief()
    n = b['life_h'] * 3600.0 / cycle_time(b)
    W = waypoints(b)
    lo, hi = W['lo'], W['hi']
    travel = (4 * W['a1'], 4 * abs(hi['q2'] - lo['q2']), 4 * abs(hi['q3'] - lo['q3']), 2 * b['q4_move'])   # deg a cycle
    return dict(cycles=n, travel=travel, turns=[t / 360.0 * n for t in travel],
                mean_rpm=[t / 360.0 / cycle_time(b) * 60.0 for t in travel])


def j1_loads(b=None):
    """The J1 bearing's axial load and tipping moment: at the cycle's ends and at the largest reach, with the
    payload, statically."""
    b = b or brief()
    W = waypoints(b)
    ws = workspace()
    return dict(lo=j1_bearing(W['lo']['q2'], W['lo']['q3'], b['payload']),
                hi=j1_bearing(W['hi']['q2'], W['hi']['q3'], b['payload']),
                reach=j1_bearing(ws['reach_q2'], ws['reach_q3'], b['payload']),
                empty_back=j1_bearing(LIMITS['q2'][0], 60.0 - 0.0, 0.0))


def report():
    b = brief()
    W = waypoints(b)
    print('waypoints', W['a1'], W['lo']['q2'], W['lo']['q3'], W['hi']['q2'], W['hi']['q3'])
    ws = workspace()
    print('reach %.1f at q2 %.2f q3 %.2f z %.1f; top %.1f bottom %.1f' % (ws['reach'], ws['reach_q2'], ws['reach_q3'],
                                                                        ws['reach_z'], ws['top'], ws['bottom']))
    gs = gravity_sums()
    print('S2 %.2f S3 %.2f; T2(85) %.0f T3(0) %.0f' % (gs['S2'], gs['S3'], T2_grav(85), T3_grav(0)))
    pm = planar_mass()
    print('M22 %.1f M33 %.1f K23 %.1f' % (pm['M22'], pm['M33'], pm['K23']))
    print('cycle %.2f s, %.0f/h' % (cycle_time(b), 3600 / cycle_time(b)))
    print('legs', leg_peaks(b))


if __name__ == '__main__':
    report()
