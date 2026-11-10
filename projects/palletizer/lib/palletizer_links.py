# -*- coding: utf-8 -*-
"""palletizer_links.py: Chapter 43 of Volume 2, the palletizer's links, the integration and the parts list.

The arm's forces in its plane, every pin at once: the lower arm, the crank, the upper arm, the elbow's triangle and
the wrist housing as rigid bodies, the three rods as two-force members, the balancer's push on the lower arm, and the
drives' torques at J2 and J3: fifteen equations, fifteen unknowns, solved by Gaussian elimination, statically over the
working range and with each body's d'Alembert forces along the standard cycle. Then the third dimension: every load
at its distance from the arm's plane, so that each joint's moment across the plane and each arm's lateral bending and
torsion come out of the same sums. Then the parts: the rods as columns, the arms as welded boxes (stiffness and weld
fatigue), the bearings at every pin (SKF's spherical plain bearings by SKF's own method, SKF tapered roller bearings by
their rating life), the reducers' main bearings (Nabtesco's allowable moment diagram), and J4's reducer.

The bodies are lib/palletizer_drives.py's (Chapter 42) with this chapter's layers across the arm's plane; once the
whole robot is weighed in Fusion (PalletizerRobot.py) the weighed bodies replace them (weighed_bodies()).
"""
import json
import math
import os

import palletizer_design as P
import palletizer_drives as D

G = P.G
pi = math.pi
E_STEEL = 207e9            # Pa, carbon steel (Budynas & Nisbett, 2015, Table A-5)
G_STEEL = 79.3e9           # Pa
HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------------------------------------------------------
# The layers across the arm's plane (y, mm), this chapter's choices. The crank stays where Chapter 42 put it, its web
# at 610 to 670 outboard of the J3 cheek; the parallel rod runs on the web's outer face (675 to 745), the only layer
# where nothing else turns: inboard is the cheek, and J3's reducer lies outboard of the web. The upper arm's rear lever
# mirrors the crank at the elbow, a plate at 610 to 670 on a torque tube round the elbow's axis. The counterweight
# moves out to 750 to 810 on four standoffs at the crank's far end, where the rod never swings. The levelling loop
# stays on the J2 side, its rods at -280. The bearings: J2's reducer's moment centre and its support bearing; J3's
# support bearing and its reducer's moment centre; the elbow's and the wrist's bearing pairs.
# ------------------------------------------------------------------------------------------------------------------
Y = dict(rod_p=710.0, crank=640.0, lever=640.0, block=780.0, rod_l=-280.0, tri=-280.0,
         rv2=-450.0 - 79.55, sup2=480.0, sup3=560.0, rv3=670.0 + 79.55, B=(-180.0, 180.0), W=(-150.0, 150.0))
CW = dict(m=100.0, r=0.60, block=(0.46, 0.46))           # Chapter 42's counterweight: 100 kg, 600 mm behind A
# the block's four standoffs in the crank's own frame (mm along it from A, across it: + toward the side the parallel
# rod can be, which it reaches from C at 40 deg or more to the crank's line)
STANDOFFS = ((560.0, -170.0), (800.0, -170.0), (800.0, 60.0), (650.0, 60.0))


def chapter_bodies(base=None):
    """Chapter 42's bodies (the crank out at 640 with its 100 kg block) with this chapter's layers: the parallel rod
    at 710 and the block at 780."""
    out = []
    for bd in D.with_counterweight(CW['m'], CW['r'], base=base):
        if bd['name'] == 'parallel rod':
            bd = dict(bd, y=Y['rod_p'])
        elif bd['name'] == 'counterweight':
            bd = dict(bd, y=Y['block'])
        elif bd['name'] in ('levelling rod 1', 'levelling rod 2', 'triangle'):
            bd = dict(bd, y=Y['rod_l'])
        out.append(bd)
    return out


WEIGHED_FILE = os.path.join(HERE, 'palletizer_weighed.json')


def weighed_bodies():
    """The bodies as Fusion weighed them (PalletizerRobot.py's groups, written by the check), or None."""
    if not os.path.exists(WEIGHED_FILE):
        return None
    return json.load(open(WEIGHED_FILE))['bodies']


def use_bodies(bl):
    """Context helper: put a body list into palletizer_design for the cycle's functions; returns the old list."""
    saved = P.BODIES[:]
    P.BODIES[:] = bl
    return saved


class bodies_in:
    """with bodies_in(bl): palletizer_design's functions (inertias, planar masses, the cycle) see the body list bl
    (without the payload, which they add)."""
    def __init__(self, bl=None):
        self.bl = [bd for bd in (bl or chapter_bodies()) if bd['name'] != 'payload']

    def __enter__(self):
        self.saved = use_bodies(self.bl)
        return self.bl

    def __exit__(self, *exc):
        P.BODIES[:] = self.saved
        return False


# ------------------------------------------------------------------------------------------------------------------
# The planar forces
# ------------------------------------------------------------------------------------------------------------------
def solve(Am, bv):
    """Gaussian elimination with partial pivoting."""
    n = len(bv)
    M = [row[:] + [bv[i]] for i, row in enumerate(Am)]
    for c in range(n):
        p = max(range(c, n), key=lambda r: abs(M[r][c]))
        if abs(M[p][c]) < 1e-12:
            raise ValueError('singular at column %d' % c)
        M[c], M[p] = M[p], M[c]
        for r in range(n):
            if r != c and M[r][c] != 0.0:
                f = M[r][c] / M[c][c]
                for k in range(c, n + 1):
                    M[r][k] -= f * M[c][k]
    return [M[i][n] / M[i][i] for i in range(n)]


UNKNOWNS = ['Ax', 'Az', 'M2', 'Cx', 'Cz', 'M3', 'Bux', 'Buz', 'Btx', 'Btz', 'Wx', 'Wz', 'Tp', 'T1', 'T2']
LINK = {'q2@A': 'lower', 'q3@A': 'crank', 'q3@B': 'upper', 'trB': 'triangle', 'trW': 'wrist',
        'trC': 'rod_p', 'q2@E': 'rod_1', 'q3@G': 'rod_2', 'turret': None}
ROD_ENDS = {'rod_p': (('crank', 'C'), ('upper', 'D')), 'rod_1': ((None, 'E'), ('triangle', 'F')),
            'rod_2': (('triangle', 'G'), ('wrist', 'H'))}
ROD_Y = {'rod_p': 'rod_p', 'rod_1': 'rod_l', 'rod_2': 'rod_l'}


def centre3(bd, q2, q3):
    """A body's centre (m) in the turret's frame at the pose: x and z in the arm's plane, y across it."""
    c, _ = P.body_pose(bd, (0.0, math.radians(q2), math.radians(q3), 0.0))
    return c


def pose_points(q2, q3):
    p = P.pts(q2, q3)
    return {k: (v[0] / 1000.0, v[1] / 1000.0) for k, v in p.items()}


def unit(a, c):
    d = math.hypot(c[0] - a[0], c[1] - a[1])
    return ((c[0] - a[0]) / d, (c[1] - a[1]) / d)


def link_alpha(name, qdd):
    """The link's angular acceleration in the arm's plane, counterclockwise seen with x right and z up (rad/s2):
    the lower arm's direction is 90 deg - q2, the crank's and the upper arm's q3."""
    return {'lower': -qdd[1], 'crank': qdd[2], 'upper': qdd[2]}.get(name, 0.0)


def loads(q2, q3, bodies, state=None, balancer=True):
    """Every load on every body at the pose, as (body, fx, fy, fz, x, y, z) in N and m (turret frame), and the
    in-plane couples (body, M): weights, the d'Alembert forces and couples if state = (acc, qdd) is given (acc: each
    body's centre acceleration and each pin's, turret frame), the rods' halves moved to their end pins, and the
    balancer's push on the lower arm at Q."""
    pp = pose_points(q2, q3)
    F, Mc, Zc = [], [], []
    acc, qdd = state if state else ({}, (0.0, 0.0, 0.0, 0.0))
    for i, bd in enumerate(bodies):
        link = LINK.get(bd['move'])
        if link is None:
            continue
        m = bd['m']
        if link in ROD_ENDS:
            for side, pin in ROD_ENDS[link]:
                if side is None:
                    continue
                y = Y[ROD_Y[link]] / 1000.0
                a = acc.get(pin, (0.0, 0.0, 0.0))
                F.append((side, -0.5 * m * a[0], -0.5 * m * a[1], -0.5 * m * (G + a[2]), pp[pin][0], y, pp[pin][1],
                          False))
            continue
        c = centre3(bd, q2, q3)
        a = acc.get(i, (0.0, 0.0, 0.0))
        F.append((link, -m * a[0], -m * a[1], -m * (G + a[2]), c[0], c[1], c[2], bool(bd.get('spin'))))
        if link in ('lower', 'crank', 'upper'):
            Mc.append((link, -bd['It'] * link_alpha(link, qdd)))
        if state:
            # J1's (and J4's) angular acceleration about the vertical, each body's own inertia about it
            _, R = P.body_pose(bd, (0.0, math.radians(q2), math.radians(q3), 0.0))
            Izz = P._mm(P._mm(R, P.body_inertia(bd)), P._T(R))[2][2]
            Zc.append((link, -Izz * (qdd[0] + (qdd[3] if bd.get('spin') else 0.0))))
    if balancer:
        g = D.GAS
        A = pp['A']
        q = math.radians(q2)
        Pp = (A[0], A[1] - g['a'])
        Q = (A[0] + g['b'] * math.sin(q), A[1] + g['b'] * math.cos(q))
        u = unit(Pp, Q)
        Fb = D.gas_force(q2)
        F.append(('lower', Fb * u[0], 0.0, Fb * u[1], Q[0], 0.0, Q[1], False))
    return F, Mc, Zc


def plane(q2, q3, bodies, state=None, balancer=True):
    """The fifteen unknowns at the pose (N, N m) and each pin's resultant; J2 and J3 in the sense of q2 and q3."""
    F, Mc, Zc = loads(q2, q3, bodies, state, balancer)
    pp = pose_points(q2, q3)
    A, B, W, C, Dp, E, Fp, Gp, H = (pp[k] for k in 'ABWCDEFGH')
    uCD, uEF, uGH = unit(C, Dp), unit(E, Fp), unit(Gp, H)
    n = len(UNKNOWNS)
    ix = {u: i for i, u in enumerate(UNKNOWNS)}
    rows, rhs = [], []

    def body(name, terms):
        fx, fz, mo = [0.0] * n, [0.0] * n, [0.0] * n
        for u, coef, pt in terms:
            if coef == 'M':
                mo[ix[u]] += 1.0
                continue
            fx[ix[u]] += coef[0]
            fz[ix[u]] += coef[1]
            mo[ix[u]] += pt[0] * coef[1] - pt[1] * coef[0]
        sx = sum(f[1] for f in F if f[0] == name)
        sz = sum(f[3] for f in F if f[0] == name)
        sm = sum(f[4] * f[3] - f[6] * f[1] for f in F if f[0] == name) + sum(m for k, m in Mc if k == name)
        for row, r in ((fx, -sx), (fz, -sz), (mo, -sm)):
            rows.append(row)
            rhs.append(r)
    body('lower', [('Ax', (1, 0), A), ('Az', (0, 1), A), ('M2', 'M', None), ('Bux', (-1, 0), B), ('Buz', (0, -1), B),
                   ('Btx', (-1, 0), B), ('Btz', (0, -1), B)])
    body('crank', [('Cx', (1, 0), A), ('Cz', (0, 1), A), ('M3', 'M', None), ('Tp', uCD, C)])
    body('upper', [('Bux', (1, 0), B), ('Buz', (0, 1), B), ('Tp', (-uCD[0], -uCD[1]), Dp), ('Wx', (-1, 0), W),
                   ('Wz', (0, -1), W)])
    body('triangle', [('Btx', (1, 0), B), ('Btz', (0, 1), B), ('T1', (-uEF[0], -uEF[1]), Fp), ('T2', uGH, Gp)])
    body('wrist', [('Wx', (1, 0), W), ('Wz', (0, 1), W), ('T2', (-uGH[0], -uGH[1]), H)])
    x = solve(rows, rhs)
    out = {u: x[ix[u]] for u in UNKNOWNS}
    out['J2'], out['J3'] = -out['M2'], out['M3']
    out['pins'] = dict(A=math.hypot(out['Ax'], out['Az']), A_crank=math.hypot(out['Cx'], out['Cz']),
                       B_upper=math.hypot(out['Bux'], out['Buz']), B_triangle=math.hypot(out['Btx'], out['Btz']),
                       W=math.hypot(out['Wx'], out['Wz']), C=abs(out['Tp']), D=abs(out['Tp']), E=abs(out['T1']),
                       F=abs(out['T1']), G=abs(out['T2']), H=abs(out['T2']))
    out['u'] = dict(CD=uCD, EF=uEF, GH=uGH)
    out['loads'], out['couples'], out['zcouples'] = F, Mc, Zc
    return out


# ------------------------------------------------------------------------------------------------------------------
# The third dimension: each group's loads at their distance from the arm's plane give the joints' moments across the
# plane (about x and z), which the bearing pairs carry as opposite forces.
# ------------------------------------------------------------------------------------------------------------------
def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def resultant(forces, about):
    """Sum of forces and of their moments about a point, 3D: forces [(fx, fy, fz, x, y, z)]."""
    Fs = [0.0, 0.0, 0.0]
    Ms = [0.0, 0.0, 0.0]
    for fx, fy, fz, x, y, z in forces:
        Fs[0] += fx
        Fs[1] += fy
        Fs[2] += fz
        m = cross((x - about[0], y - about[1], z - about[2]), (fx, fy, fz))
        for k in range(3):
            Ms[k] += m[k]
    return Fs, Ms


def joints3d(sol, q2, q3):
    """The reaction each joint gives its outboard group, 3D, about the joint's centre in the arm's plane (y = 0):
    W on the wrist group, B on the upper arm (with the wrist group), Bt on the triangle, A on the lower arm (with
    everything it carries), A3 on the crank. The y component of each moment is the in-plane balance: zero at the
    pins, the drive's moment at A and A3. Each is the negative of the group's loads, the rods' forces placed at their
    pins in their own layers."""
    pp = pose_points(q2, q3)
    yv = lambda k: Y[k] / 1000.0
    L = sol['loads']
    uCD, uEF, uGH = sol['u']['CD'], sol['u']['EF'], sol['u']['GH']
    Tp, T1, T2 = sol['Tp'], sol['T1'], sol['T2']
    on = lambda name: [f[1:7] for f in L if f[0] == name]
    rodD = (-Tp * uCD[0], 0.0, -Tp * uCD[1], pp['D'][0], yv('rod_p'), pp['D'][1])     # on the upper arm at D
    rodC = (Tp * uCD[0], 0.0, Tp * uCD[1], pp['C'][0], yv('rod_p'), pp['C'][1])       # on the crank at C
    rodH = (-T2 * uGH[0], 0.0, -T2 * uGH[1], pp['H'][0], yv('rod_l'), pp['H'][1])     # on the wrist at H
    rodG = (T2 * uGH[0], 0.0, T2 * uGH[1], pp['G'][0], yv('rod_l'), pp['G'][1])       # on the triangle at G
    rodF = (-T1 * uEF[0], 0.0, -T1 * uEF[1], pp['F'][0], yv('rod_l'), pp['F'][1])     # on the triangle at F
    pt = lambda k: (pp[k][0], 0.0, pp[k][1])
    neg = lambda v: [-x for x in v]
    out = {}

    def react(forces, about, names):
        """The joint's reaction on the group: minus the loads' sum and moment. A counterclockwise couple c in the
        x-z view (x right, z up) turns about -y, so it adds (0, -c, 0) to the loads' moment; a couple about the
        vertical adds (0, 0, c)."""
        Fs, Ms = resultant(forces, about)
        Ms[1] -= sum(c for name, c in sol['couples'] if name in names)
        Ms[2] += sum(c for name, c in sol['zcouples'] if name in names)
        return neg(Fs), neg(Ms)
    wrist = on('wrist') + [rodH]
    out['W'] = react(wrist, pt('W'), ('wrist',))
    upper = on('upper') + [rodD] + wrist
    out['B'] = react(upper, pt('B'), ('upper', 'wrist'))
    tri = on('triangle') + [rodG, rodF]
    out['Bt'] = react(tri, pt('B'), ('triangle',))
    lower = on('lower') + upper + tri
    out['A'] = react(lower, pt('A'), ('lower', 'upper', 'wrist', 'triangle'))
    crank = on('crank') + [rodC]
    out['A3'] = react(crank, pt('A'), ('crank',))
    return out


def split(F, M, y1, y2):
    """A joint's force F and moment M (about the plane's point, components across the plane) shared by two bearings
    on the joint's axis at y1 and y2 (m): R1 + R2 = F (x and z), y1 R1 + y2 R2 balancing M's x and z. The moment
    of a force (fx, 0, fz) at y is (y fz, ., -y fx), so y1 R1z + y2 R2z = Mx and y1 R1x + y2 R2x = -Mz."""
    Vx, Vz = -M[2], M[0]
    R2x = (Vx - y1 * F[0]) / (y2 - y1)
    R2z = (Vz - y1 * F[2]) / (y2 - y1)
    R1x, R1z = F[0] - R2x, F[2] - R2z
    return (R1x, R1z), (R2x, R2z)


# ------------------------------------------------------------------------------------------------------------------
# Along the cycle: accelerations by central differences of the centres and pins, in the turret's frame
# ------------------------------------------------------------------------------------------------------------------
def _world(bd, q):
    c, _ = P.body_pose(bd, q)
    return c


def _pin_world(q, key):
    pp = P.pts(math.degrees(q[1]), math.degrees(q[2]))[key]
    y = {'C': Y['rod_p'], 'D': Y['rod_p']}.get(key, Y['rod_l']) / 1000.0
    x, z = pp[0] / 1000.0, pp[1] / 1000.0
    c1, s1 = math.cos(q[0]), math.sin(q[0])
    return (c1 * x - s1 * y, s1 * x + c1 * y, z)


def state_at(t, bodies, b=None, legs=None, h=1e-4):
    """The pose, the payload's state and every centre's and pin's acceleration in the turret's frame at time t."""
    b = b or P.brief()
    if legs is None:
        legs, _ = P.timeline(b)
    q, qd, qdd, loaded = P.cycle_state(t, b, legs)
    qm = [math.radians(v) for v in P.cycle_state(t - h, b, legs)[0]]
    q0 = [math.radians(v) for v in q]
    qp = [math.radians(v) for v in P.cycle_state(t + h, b, legs)[0]]
    c1, s1 = math.cos(q0[0]), math.sin(q0[0])
    to_turret = lambda a: (c1 * a[0] + s1 * a[1], -s1 * a[0] + c1 * a[1], a[2])
    acc = {}
    for i, bd in enumerate(bodies):
        if LINK.get(bd['move']) is None or bd['move'] in ('trC', 'q2@E', 'q3@G'):
            continue
        cm, c0, cp = _world(bd, qm), _world(bd, q0), _world(bd, qp)
        acc[i] = to_turret(tuple((cp[k] - 2 * c0[k] + cm[k]) / (h * h) for k in range(3)))
    for key in 'CDFGH':
        cm, c0, cp = _pin_world(qm, key), _pin_world(q0, key), _pin_world(qp, key)
        acc[key] = to_turret(tuple((cp[k] - 2 * c0[k] + cm[k]) / (h * h) for k in range(3)))
    return q, qd, qdd, loaded, acc


def with_payload(bodies, m):
    """The bodies with the payload's mass m (the full payload or the gripper alone)."""
    out = [bd for bd in bodies if bd['name'] != 'payload']
    if m > 0:
        out.append(P.payload_body(m))
    return out


def cycle(n=600, bodies=None, b=None):
    """Every force in the arm at n + 1 instants of the standard cycle: the plane's fifteen unknowns and the joints' 3D
    reactions, with the d'Alembert forces of every body."""
    b = b or P.brief()
    base = [bd for bd in (bodies or chapter_bodies()) if bd['name'] != 'payload']
    legs, Tc = P.timeline(b)
    full, empty = with_payload(base, b['payload']), with_payload(base, b['gripper'])
    rows = []
    for k in range(n + 1):
        t = Tc * k / n
        bl = full if P.cycle_state(t, b, legs)[3] else empty
        q, qd, qdd, loaded, acc = state_at(t, bl, b, legs)
        s = plane(q[1], q[2], bl, (acc, qdd))
        rows.append(dict(t=t, q=q, qd=qd, qdd=qdd, loaded=loaded, sol=s, j3=joints3d(s, q[1], q[2])))
    return dict(rows=rows, Tc=Tc)


_CYC = {}


def cycle_cached(n=600):
    if n not in _CYC:
        _CYC[n] = cycle(n)
    return _CYC[n]


def corners():
    """The working range's corners (Chapter 41's polygon) and the full-reach pose, with names."""
    ws = P.workspace()
    pts = [('corner %d' % (i + 1), q2, q3) for i, (q2, q3) in enumerate(P.range_polygon())]
    pts.append(('reach', ws['reach_q2'], ws['reach_q3']))
    return pts


def envelope(bodies=None, payload=None):
    """The static pin forces at the range's corners and full reach, with the payload: the largest at each pin and
    where."""
    b = P.brief()
    bl = with_payload([bd for bd in (bodies or chapter_bodies()) if bd['name'] != 'payload'],
                      b['payload'] if payload is None else payload)
    res = []
    for name, q2, q3 in corners():
        s = plane(q2, q3, bl)
        res.append((name, q2, q3, s, joints3d(s, q2, q3)))
    best = {}
    for k in res[0][3]['pins']:
        r = max(res, key=lambda r: r[3]['pins'][k])
        best[k] = (r[3]['pins'][k], r[0])
    return dict(poses=res, best=best)


# ------------------------------------------------------------------------------------------------------------------
# The catalogues: SKF's radial spherical plain bearings, steel/steel, GE .. ES (SKF, 2013, pp. 103-106: d, D, B inner
# ring width, Cw outer ring width, mm; C and C0, kN; dk, mm; kg), SKF's tapered roller bearing 32213 and deep groove
# ball bearings 61856 MA and 61836 (product pages), and the RV-N reducers this chapter adds (Nabtesco Corporation,
# 2018, pp. 8-9, 33, 38) with their allowable moment diagrams as (moment N m, thrust N) corners.
# ------------------------------------------------------------------------------------------------------------------
GE_ES = {
    'GE 30 ES': dict(d=30, D=47, B=22, Cw=18, C=62.0, C0=310.0, dk=40.7, m=0.16),
    'GE 35 ES': dict(d=35, D=55, B=25, Cw=20, C=80.0, C0=400.0, dk=47.0, m=0.22),
    'GE 40 ES': dict(d=40, D=62, B=28, Cw=22, C=100.0, C0=500.0, dk=53.0, m=0.31),
    'GE 45 ES': dict(d=45, D=68, B=32, Cw=25, C=127.0, C0=640.0, dk=60.0, m=0.46),
    'GE 50 ES': dict(d=50, D=75, B=35, Cw=28, C=156.0, C0=780.0, dk=66.0, m=0.56),
    'GE 60 ES': dict(d=60, D=90, B=44, Cw=36, C=245.0, C0=1220.0, dk=80.0, m=1.10),
    'GE 70 ES': dict(d=70, D=105, B=49, Cw=40, C=315.0, C0=1560.0, dk=92.0, m=1.55),
    'GE 80 ES': dict(d=80, D=120, B=55, Cw=45, C=400.0, C0=2000.0, dk=105.0, m=2.30),
}
K_SS, K0_SS = 100.0, 500.0          # N/mm2, steel/steel, metric (SKF, 2013, Table 4, p. 45)
TRB = {'32213': dict(d=65, D=120, T=32.75, a=26.55, C=186.0, C0=193.0, e=0.4, Y=1.5, Y0=0.8, m=1.55)}
DGBB = {'61856 MA': dict(d=280, D=350, B=33, C=138.0, C0=200.0, m=7.34),
        '61836': dict(d=180, D=225, B=22, C=62.4, C0=81.5, m=1.72)}
RV_SMALL = {
    'RV-42N': dict(T0=412.0, TS1=1029.0, TS2=2058.0, NS0=52.0, MO1=1660.0, m=6.3, Kt=113.0, Km=840.0, a=29.0,
                   b=131.1, ratios=(41, 81, 105, 126, 141, 164.07), allow=((1490.0, 5220.0), (1660.0, 3320.0))),
    'RV-60N': dict(T0=600.0, TS1=1500.0, TS2=3000.0, NS0=44.0, MO1=2000.0, m=8.9, Kt=200.0, Km=1140.0, a=35.0,
                   b=147.0, ratios=(41, 81, 102.17, 121, 145.61, 161), allow=((1770.0, 5880.0), (2000.0, 3570.0))),
    'RV-80N': dict(T0=784.0, TS1=1960.0, TS2=3920.0, NS0=40.0, MO1=2150.0, m=9.3, Kt=212.0, Km=1190.0, a=33.8,
                   b=151.8, ratios=(41, 81, 101, 129, 141, 171), allow=((1860.0, 6530.0), (2150.0, 3720.0))),
}
RV_ALLOW = {'RV-500N': ((7110.0, 32000.0), (11000.0, 10630.0))}


def allowable_moment(name, thrust):
    """The allowable moment (N m) at a thrust (N) from the diagram's two corners: flat to the first, a straight line
    to the second, the moment's limit below it; None above the diagram's largest thrust."""
    (m1, w1), (m2, w2) = RV_SMALL[name]['allow'] if name in RV_SMALL else RV_ALLOW[name]
    if thrust > w1:
        return None
    if thrust <= w2:
        return m2
    return m1 + (w1 - thrust) / (w1 - w2) * (m2 - m1)


def rv_geom(name):
    rv = RV_SMALL.get(name) or D.RV[name]
    return rv['a'], rv['b']


# ---------- SKF's life factors (SKF, 2013, pp. 51-53), read from the catalogue's diagrams ----------
B5_LOWER = [(5.0, 1.007), (8.801, 1.99), (19.314, 5.086), (20.074, 5.308), (21.286, 5.649), (22.224, 5.886),
            (23.079, 6.105), (24.217, 6.44), (25.217, 6.763), (27.238, 7.247), (30.81, 7.965), (35.191, 8.518),
            (41.418, 8.884), (45.0, 8.948)]
FB_LOWER = [(5.0, 1.0), (5.994, 1.399), (7.139, 2.006), (8.189, 2.537), (9.078, 3.011), (9.993, 3.551),
            (10.978, 4.024), (11.917, 4.405), (12.708, 4.676), (13.558, 4.913), (14.589, 5.115), (15.924, 5.28),
            (17.684, 5.404), (20.0, 5.486)]
FH = [(1.0, 1.314), (2.303, 1.539), (3.993, 1.814), (5.04, 1.995), (7.061, 2.311), (9.852, 2.727), (11.866, 3.0),
      (14.156, 3.275), (17.705, 3.683), (20.096, 3.951), (24.133, 4.36), (28.314, 4.742), (31.639, 4.999),
      (34.33, 5.15), (38.091, 5.352), (40.0, 5.453), (50.0, 5.453)]


def _interp(tab, x, log=False):
    x = min(max(x, tab[0][0]), tab[-1][0])
    for (a, va), (c, vc) in zip(tab, tab[1:]):
        if a <= x <= c:
            if log:
                return va * (vc / va) ** (math.log(x / a) / math.log(c / a))
            return va + (vc - va) * (x - a) / (c - a)
    return tab[-1][1]


def b3(dk):
    """Diagram 11: a straight line on log scales from 1 at dk = 10 mm to 3.9 at 500 mm."""
    return 3.9 ** (math.log(dk / 10.0) / math.log(50.0))


def b4(v):
    """Diagram 12, steel/steel: a straight line on log scales from 1 at 0.002 m/s to 7 at 0.1 m/s."""
    return 7.0 ** (math.log(max(v, 0.002) / 0.002) / math.log(50.0))


def b5(beta):
    """Diagram 13, the band's lower edge for bearings under 150 mm outside (SKF's note, p. 53); beta < 5 as 5."""
    return _interp(B5_LOWER, max(beta, 5.0), log=True)


def f_beta(beta):
    return _interp(FB_LOWER, max(beta, 5.0))


def f_H(H):
    return _interp(FH, H)


def gh(p, v, dk, beta, b1=1.0, b2=1.0):
    """The basic rating life with initial lubrication, h (SKF, 2013, p. 51): p under 10 N/mm2 taken as 10."""
    p = max(p, 10.0)
    return b1 * b2 * b3(dk) * b4(v) * b5(beta) * 330.0 / (p ** 2.5 * v)


def pv_range_I(p, v):
    """SKF's diagram 5, steel/steel: range I starts at 0.002 m/s; its upper edge is read at 100 N/mm2 up to
    0.002 m/s and about 10 N/mm2 at 0.05 m/s, a straight line on log scales between them."""
    if v < 0.002 or v > 0.1:
        return False
    lim = 100.0 * 0.1 ** (math.log(v / 0.002) / math.log(25.0)) if v <= 0.05 else 10.0
    return p <= lim


# ------------------------------------------------------------------------------------------------------------------
# Each pin's duty over the cycle: the load and the turn between its two parts, leg by leg
# ------------------------------------------------------------------------------------------------------------------
TURN = {'A': lambda q: q[1], 'A3': lambda q: q[2], 'B': lambda q: q[1] + q[2], 'Bt': lambda q: q[1],
        'C': lambda q: q[1] + q[2], 'D': lambda q: q[1] + q[2], 'E': lambda q: q[1], 'F': lambda q: q[1],
        'G': lambda q: q[2], 'H': lambda q: q[2], 'W': lambda q: q[2]}


def pin_force(row, pin):
    s = row['sol']
    if pin in ('C', 'D'):
        return abs(s['Tp'])
    if pin in ('E', 'F'):
        return abs(s['T1'])
    if pin in ('G', 'H'):
        return abs(s['T2'])
    return {'A': s['pins']['A'], 'A3': s['pins']['A_crank'], 'B': s['pins']['B_upper'],
            'Bt': s['pins']['B_triangle'], 'W': s['pins']['W']}[pin]


def duty(pin, cyc=None):
    """The pin's legs (the times it turns): duration, the turn (deg), the mean load (N); and the whole cycle's."""
    cyc = cyc or cycle_cached()
    rows, Tc = cyc['rows'], cyc['Tc']
    legs, _ = P.timeline()
    out = []
    for (t0, T, a, c, _) in legs:
        sel = [r for r in rows if t0 - 1e-9 <= r['t'] <= t0 + T + 1e-9]
        turn = abs(TURN[pin](c) - TURN[pin](a))
        if turn < 1e-6:
            continue
        Fm = sum(pin_force(r, pin) for r in sel) / len(sel)
        out.append(dict(t=T, turn=turn, F=Fm, Fmax=max(pin_force(r, pin) for r in sel)))
    allF = [pin_force(r, pin) for r in rows]
    return dict(legs=out, Tc=Tc, Fmax=max(allF), Fmin=min(allF), Fmean=sum(allF[:-1]) / (len(allF) - 1))


def plain_life(name, pin, b1=1.0, N=None, cyc=None):
    """SKF's method for a GE .. ES at a pin, period by period (SKF, 2013, p. 61): each leg's p from its mean load,
    its v from its turn and duration, its life Gh_i; Gh = T / sum(t_i / Gh_i) over the cycle (the dwells and J1's
    legs slide nothing). With relubrication every N hours, GhN = Gh f_beta f_H (H = Gh / N)."""
    br = GE_ES[name]
    du = duty(pin, cyc)
    dm = br['dk'] / 1000.0
    s = 0.0
    legs = []
    for lg in du['legs']:
        p = K_SS * lg['F'] / 1000.0 / br['C']
        v = math.radians(lg['turn']) * dm / 2.0 / lg['t']
        beta = lg['turn'] / 2.0
        g = gh(p, v, br['dk'], beta, b1)
        s += lg['t'] / g
        legs.append(dict(p=p, v=v, beta=beta, Gh=g, rangeI=pv_range_I(max(p, 10.0), v)))
    Gh = du['Tc'] / s
    beta = max(lg['beta'] for lg in legs)
    out = dict(name=name, pin=pin, Gh=Gh, legs=legs, C_over_Pmax=br['C'] / (du['Fmax'] / 1000.0), beta=beta,
               fb=f_beta(beta), Fmax=du['Fmax'])
    if N:
        out.update(N=N, H=Gh / N, fH=f_H(Gh / N), GhN=Gh * f_beta(beta) * f_H(Gh / N))
    return out


# ---------- the tapered roller bearings by their rating life (Budynas & Nisbett, 2015, pp. 585-589) ----------
def trb_pair(Fr1, Fr2, Fae, K):
    """Eqs. 11-18 to 11-20: the equivalent loads of a pair, bearing 1 the one Fae squeezes (Fae >= 0)."""
    Fi1, Fi2 = 0.47 * Fr1 / K, 0.47 * Fr2 / K
    if Fi1 <= Fi2 + Fae:
        Fe1, Fe2 = 0.4 * Fr1 + K * (Fi2 + Fae), Fr2
    else:
        Fe1, Fe2 = Fr1, 0.4 * Fr2 + K * (Fi1 - Fae)
    return max(Fe1, Fr1), max(Fe2, Fr2)


def roller_life(name, Fe_series, turns_per_cycle, Tc):
    """L10 in hours: the equivalent load averaged as its power a over the cycle's instants, L10 = (C/Pm)^a million
    turns (a = 10/3 for rollers, 3 for balls), at the mean speed the pin turns."""
    br = TRB.get(name) or DGBB.get(name)
    a = 10.0 / 3.0 if name in TRB else 3.0
    Pm = (sum(f ** a for f in Fe_series) / len(Fe_series)) ** (1.0 / a)
    L10 = (br['C'] * 1000.0 / Pm) ** a * 1e6
    n_mean = turns_per_cycle / Tc            # turns a second
    return dict(Pm=Pm, L10=L10, hours=L10 / n_mean / 3600.0, n_mean=n_mean * 60.0)


def pin_turns(pin):
    """Turns a cycle at a pin: every leg's turn, there and back."""
    legs, _ = P.timeline()
    return sum(abs(TURN[pin](c) - TURN[pin](a)) for (t0, T, a, c, _) in legs) / 360.0


def joint_bearings(cyc=None, env=None):
    """Each two-bearing joint's bearing loads along the cycle and over the range's corners: the elbow B and the wrist
    W on 32213 pairs; the shoulder on J2's reducer and its support bearing; the crank on its support bearing and J3's
    reducer. The largest radial load per bearing, the equivalent loads' mean, the life in hours."""
    cyc = cyc or cycle_cached()
    env = env or envelope()
    sp = {'B': (Y['B'][0] / 1000, Y['B'][1] / 1000), 'W': (Y['W'][0] / 1000, Y['W'][1] / 1000),
          'A': (Y['rv2'] / 1000, Y['sup2'] / 1000), 'A3': (Y['sup3'] / 1000, Y['rv3'] / 1000)}
    out = {}
    for j, (y1, y2) in sp.items():
        series, stat = [], []
        for r in cyc['rows']:
            F, M = r['j3'][j]
            R1, R2 = split(F, M, y1, y2)
            series.append((math.hypot(*R1), math.hypot(*R2), abs(F[1])))
        for name, q2, q3, s, jj in env['poses']:
            F, M = jj[j]
            R1, R2 = split(F, M, y1, y2)
            stat.append((name, math.hypot(*R1), math.hypot(*R2)))
        out[j] = dict(series=series, static=stat, max1=max(x[0] for x in series), max2=max(x[1] for x in series),
                      stat1=max(stat, key=lambda x: x[1]), stat2=max(stat, key=lambda x: x[2]),
                      Fy=max(x[2] for x in series))
    K = TRB['32213']['Y']
    for j in ('B', 'W'):
        Fe = []
        for R1, R2, Fy in out[j]['series']:
            e1, e2 = trb_pair(R1, R2, Fy, K)
            e2b, e1b = trb_pair(R2, R1, Fy, K)
            Fe.append(max(e1, e1b, e2, e2b))
        out[j]['life'] = roller_life('32213', Fe, pin_turns(j), cyc['Tc'])
        out[j]['C0_margin'] = TRB['32213']['C0'] * 1000.0 / max(out[j]['stat1'][1], out[j]['stat2'][2])
    for j, name, k in (('A', '61856 MA', 1), ('A3', '61836', 0)):
        Fe = [x[k] for x in out[j]['series']]
        out[j]['support'] = name
        out[j]['life'] = roller_life(name, Fe, pin_turns(j), cyc['Tc'])
        st = out[j]['stat2'][2] if k == 1 else out[j]['stat1'][1]
        out[j]['C0_margin'] = DGBB[name]['C0'] * 1000.0 / st
    # the reducers: W1 their share at the moment centre, so l + b - a = b/2 (Nabtesco Corporation, 2018, p. 29)
    for j, k in (('A', 0), ('A3', 1)):
        a, b = rv_geom('RV-500N')
        W1c = max(x[k] for x in out[j]['series'])
        W1s = max(x[1 + k] for x in out[j]['static'])
        Fy = out[j]['Fy']
        out[j]['rv'] = dict(W1=W1c, M=W1c * b / 2 / 1000.0, W1_static=W1s, M_static=W1s * b / 2 / 1000.0,
                            thrust=Fy, allow=allowable_moment('RV-500N', Fy))
    return out


# ---------- the rolling bearings the pins get instead (SKF product pages) ----------
SRB = {'22207 E': dict(d=35, D=72, B=23, C=88.8, C0=83.0, e=0.31, m=0.438),
       '22213 E': dict(d=65, D=120, B=31, C=198.0, C0=216.0, e=0.24, m=1.52)}
PIN_BEARING = dict(C='22213 E', D='22213 E', E='22207 E', F='22207 E', G='22207 E', H='22207 E', Bt='22207 E')


def srb_life(name, pin, cyc=None, env=None):
    """A spherical roller bearing in a rod's eye or under the triangle: radial only (the rods are two-force members),
    P = Fr; L10 = (C/Pm)^(10/3) million turns with Pm the 10/3 mean over the cycle's instants, at the pin's mean
    speed; the static margin C0 over the largest load at the range's corners."""
    cyc = cyc or cycle_cached()
    env = env or envelope()
    br = SRB[name]
    F = [pin_force(r, pin) for r in cyc['rows']]
    a = 10.0 / 3.0
    Pm = (sum(f ** a for f in F) / len(F)) ** (1 / a)
    turns = pin_turns(pin)
    L10 = (br['C'] * 1000.0 / Pm) ** a * 1e6
    hours = L10 / (turns / cyc['Tc']) / 3600.0
    key = {'Bt': 'B_triangle'}.get(pin, pin)
    st = env['best'][key][0]
    return dict(name=name, pin=pin, Pm=Pm, Fmax=max(F), L10=L10, hours=hours, turns=turns,
                C0_margin=br['C0'] * 1000.0 / st, static=st, where=env['best'][key][1])


# ------------------------------------------------------------------------------------------------------------------
# The sections: welded boxes of AISI 1020 hot rolled (Sut 380, Sy 210 MPa, Table A-20; near a weld a drawn bar has
# hot-rolled properties anyway, p. 482), tubes for the rods
# ------------------------------------------------------------------------------------------------------------------
STEEL = dict(Sut=380e6, Sy=210e6, rho=7850.0)


def box(d, w, t):
    """A thin-walled box d deep in the arm's plane, w wide across it, wall t (m): area, the second moments for
    bending in the plane and across it, the median line's area and length, Bredt's torsion constant."""
    di, wi = d - 2 * t, w - 2 * t
    A = d * w - di * wi
    Iip = (w * d ** 3 - wi * di ** 3) / 12.0
    Ilat = (d * w ** 3 - di * wi ** 3) / 12.0
    Am = (d - t) * (w - t)
    Lm = 2 * ((d - t) + (w - t))
    return dict(d=d, w=w, t=t, A=A, Iip=Iip, Ilat=Ilat, Am=Am, Lm=Lm, J=4 * Am * Am * t / Lm, m_per_m=A * STEEL['rho'])


def flat(d, w):
    """A flat bar d deep in the arm's plane, w across it."""
    A = d * w
    I = w * d ** 3 / 12.0
    return dict(d=d, w=w, A=A, I=I, k=math.sqrt(I / A), m_per_m=A * STEEL['rho'])


def column(sec, L, Sy=STEEL['Sy'], E=E_STEEL, C=1.0):
    """A strut pinned at both ends (Budynas & Nisbett, 2015, Eqs. 4-43 to 4-46, pp. 196-198): Euler above
    (l/k)1 = (2 pi^2 C E/Sy)^1/2, Johnson's parabola below."""
    lk = L / sec['k']
    lk1 = math.sqrt(2 * pi * pi * C * E / Sy)
    if lk >= lk1:
        Pcr = C * pi * pi * E * sec['A'] / lk ** 2
        kind = 'Euler'
    else:
        Pcr = sec['A'] * (Sy - (Sy * lk / (2 * pi)) ** 2 / (C * E))
        kind = 'Johnson'
    return dict(lk=lk, lk1=lk1, Pcr=Pcr, kind=kind)


def tube(Do, t):
    Di = Do - 2 * t
    A = pi / 4 * (Do * Do - Di * Di)
    I = pi / 64 * (Do ** 4 - Di ** 4)
    return dict(Do=Do, t=t, A=A, I=I, k=math.sqrt(I / A), m_per_m=A * STEEL['rho'])


S_HUB, S_FORK = 0.16, 0.65    # m from the shoulder: the hub's bosses' edge, and where the fork's plates end in the box


def fork(d, t, yc, L_open=S_FORK - S_HUB):
    """The lower arm's root: two plates d deep and t thick, their middles at +-yc across the plane, tied at both
    ends (the shoulder's hub, the arm's box). Twisted, each plate is open (b t^3/3) and the pair also bends in
    opposite directions as two beams fixed at one end and guided at the other, F = 12 E I delta / L^3 each
    (Table A-9's cantilever under an end force and the end moment that keeps it straight), so the pair's stiffness is
    24 E I yc^2 / L^3 for the open length L: the equivalent GJ is that times L."""
    A = 2 * d * t
    Ip = t * d ** 3 / 12.0
    Iip = 2 * Ip
    Ilat = 2 * (d * t ** 3 / 12.0 + d * t * yc * yc)
    J_sv = 2 * d * t ** 3 / 3.0
    GJ_frame = 24 * E_STEEL * Ip * yc * yc / L_open ** 2
    return dict(d=d, w=2 * (yc + t / 2), t=t, A=A, Iip=Iip, Ilat=Ilat, J=J_sv + GJ_frame / G_STEEL, J_sv=J_sv,
                GJ_frame=GJ_frame, open=True, m_per_m=A * STEEL['rho'])


SECTIONS = dict(fork=fork(0.220, 0.040, 0.170), lower=box(0.220, 0.300, 0.012), upper=box(0.200, 0.260, 0.010), lever=box(0.300, 0.060, 0.030),
                tube_B=tube(0.300, 0.012), rod_p=flat(0.040, 0.070), rod_l=tube(0.060, 0.005))


def arm_frames(q2, q3):
    a2 = (math.sin(math.radians(q2)), 0.0, math.cos(math.radians(q2)))
    n2 = (math.cos(math.radians(q2)), 0.0, -math.sin(math.radians(q2)))
    u3 = (math.cos(math.radians(q3)), 0.0, math.sin(math.radians(q3)))
    n3 = (-math.sin(math.radians(q3)), 0.0, math.cos(math.radians(q3)))
    return dict(lower=(a2, n2), upper=(u3, n3))


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def section(sol, j3, q2, q3, arm, s):
    """The internal force and moment at a section of an arm, s (m) from its root (A for the lower arm, B for the
    upper, toward B and W), carried from the root's side to the tip's: the axial force, the in-plane and lateral
    bending moments and the torsion (N, N m), from the loads beyond the section."""
    pp = pose_points(q2, q3)
    ax, nn = arm_frames(q2, q3)[arm]
    root = pp['A'] if arm == 'lower' else pp['B']
    Sp = (root[0] + s * ax[0], 0.0, root[1] + s * ax[2])
    tip = []
    L = sol['loads']
    own = [f[1:7] for f in L if f[0] == arm]
    cpl = sum(c for name, c in sol['couples'] if name == arm)
    beyond = False
    for f in own:
        if dot((f[3] - root[0], 0.0, f[5] - root[1]), ax) > s:
            tip.append(f)
            beyond = True
    # what the next joint brings, as the group's loads at the joint: the upper arm and the triangle at B, the wrist at W
    groups = [('B', 'B'), ('Bt', 'B')] if arm == 'lower' else [('W', 'W')]
    Fs, Ms = resultant(tip, Sp)
    if beyond:
        Ms[1] -= cpl                  # the arm's own couple (counterclockwise in x-z, about -y) with its centre
    for key, pin in groups:
        Fr, Mr = j3[key]
        Fg, Mg = [-v for v in Fr], [-v for v in Mr]
        r = (pp[pin][0] - Sp[0], 0.0, pp[pin][1] - Sp[2])
        mm = cross(r, Fg)
        for k in range(3):
            Fs[k] += Fg[k]
            Ms[k] += Mg[k] + mm[k]
    M = [-v for v in Ms]
    F = [-v for v in Fs]
    return dict(N=dot(F, ax), M_ip=M[1], M_lat=dot(M, nn), T=dot(M, ax), V=F)


def box_stress(sec, ld):
    """The largest normal stress at a corner and the torsional shear, Pa: a closed box by Eq. 3-45, the open fork's
    plates by tau = T t / J."""
    sig = ld['N'] / sec['A'] + abs(ld['M_ip']) * sec['d'] / 2 / sec['Iip'] + abs(ld['M_lat']) * sec['w'] / 2 / sec['Ilat']
    if sec.get('open'):
        # the plates' own twist carries T in proportion J_sv / J; the rest bends them in opposite directions
        Tsv = abs(ld['T']) * sec['J_sv'] / sec['J']
        tau = Tsv * sec['t'] / sec['J_sv']
        Tfr = abs(ld['T']) - Tsv
        sig += Tfr / (2 * sec['w'] / 2 - sec['t']) * (S_FORK - S_HUB) / 2 * sec['d'] / 2 / (sec['Iip'] / 2)
    else:
        tau = abs(ld['T']) / (2 * sec['Am'] * sec['t'])
    return sig, tau


def marin(d_equiv, Sut=STEEL['Sut']):
    """Se for a hot-rolled part in bending (Eqs. 6-8, 6-19, 6-20; Table 6-2): ka = 57.7 Sut^-0.718 (MPa), kb =
    1.51 d^-0.157 for 51 < d <= 254 mm."""
    S = Sut / 1e6
    ka = 57.7 * S ** -0.718
    dmm = d_equiv * 1000.0
    kb = 1.51 * dmm ** -0.157 if dmm > 51 else 1.24 * dmm ** -0.107
    return dict(ka=ka, kb=kb, Se=ka * kb * 0.5 * Sut, Se_prime=0.5 * Sut)


def weld_fatigue(arm, s, Kfs=2.0, cyc=None, key=None):
    """The section at a weld over the cycle: the stresses' ranges with the weld's factor (Table 9-5, T-butt joint
    with sharp corners 2.0) on both parts, von Mises of the alternating and of the mean parts, and Goodman's factor
    n = 1/(sa/Se + sm/Sut) (Eq. 6-41)."""
    cyc = cyc or cycle_cached()
    sec = SECTIONS[key or arm]
    sig, tau = [], []
    for r in cyc['rows']:
        ld = section(r['sol'], r['j3'], r['q'][1], r['q'][2], arm, s)
        a, b = box_stress(sec, ld)
        sig.append(a)
        tau.append(b)
    sa, sm = (max(sig) - min(sig)) / 2, (max(sig) + min(sig)) / 2
    ta, tm = (max(tau) - min(tau)) / 2, (max(tau) + min(tau)) / 2
    va = math.hypot(Kfs * sa, math.sqrt(3) * Kfs * ta)
    vm = math.hypot(Kfs * sm, math.sqrt(3) * Kfs * tm)
    mr = marin(0.808 * math.sqrt(sec['d'] * sec['w']))
    n = 1.0 / (va / mr['Se'] + vm / STEEL['Sut'])
    return dict(arm=arm, s=s, sig=(min(sig), max(sig)), tau=(min(tau), max(tau)), va=va, vm=vm, Se=mr['Se'],
                marin=mr, n=n)


def static_yield(arm, s, env=None, key=None):
    """The section at the range's corners with the payload: von Mises against Sy."""
    env = env or envelope()
    sec = SECTIONS[key or arm]
    best = None
    for name, q2, q3, sol, j3 in env['poses']:
        ld = section(sol, j3, q2, q3, arm, s)
        a, b = box_stress(sec, ld)
        v = math.sqrt(a * a + 3 * b * b)
        if best is None or v > best[0]:
            best = (v, name, ld)
    return dict(vm=best[0], where=best[1], loads=best[2], n=STEEL['Sy'] / best[0])


# ------------------------------------------------------------------------------------------------------------------
# Stiffness: the arms in series with the reducers for the 5 Hz of Chapter 41
# ------------------------------------------------------------------------------------------------------------------
ARCMIN = math.radians(1.0 / 60.0)


def rv_K(name):
    rv = RV_SMALL.get(name) or D.RV[name]
    return rv['Kt'] / ARCMIN


def stiffness(q2=None, q3=None, b=None, bodies=None):
    """J2: the lower arm bending as a cantilever from the shoulder, 3EI/L, in series with the reducer. J3: the
    parallel rod's stretch on its lever arm r sin psi, the upper arm as a cantilever from the elbow, the torque tube
    in torsion (Eq. 3-46) and the lever and crank as cantilevers, in series with the reducer. Each joint's frequency
    on Chapter 41's planar inertia, at the cycle's high point."""
    b = b or P.brief()
    hi = P.waypoints(b)['hi']
    q2 = hi['q2'] if q2 is None else q2
    q3 = hi['q3'] if q3 is None else q3
    lay = P.LAYOUT
    L2, L3, r = lay['L2'] / 1000, lay['L3'] / 1000, lay['r'] / 1000
    S = SECTIONS
    E, Gs = E_STEEL, G_STEEL
    with bodies_in(bodies):
        pm = P.planar_mass(b['payload'])
    K_rv = rv_K('RV-500N')
    # a force at the elbow across the lower arm: the cantilever's tip turn per unit moment, the fork to S_FORK
    n = 400
    c2 = sum((L2 - (k + 0.5) * L2 / n) ** 2 / (E * (S['fork'] if (k + 0.5) * L2 / n < S_FORK else S['lower'])['Iip'])
             for k in range(n)) * L2 / n / L2 ** 2
    psi = math.radians(P.psi(q2, q3))
    h = r * math.sin(psi)
    rod = S['rod_p']
    c_rod = 1.0 / (E * rod['A'] / L2 * h * h)
    c_up = L3 / (3 * E * S['upper']['Iip'])
    tb = S['tube_B']
    Lt = (Y['lever'] - 30.0 - Y['B'][1]) / 1000.0             # the torque tube from the cheek to the lever plate
    J_tube = 2 * tb['I']
    c_tube = Lt / (Gs * J_tube)
    c_lev = 2 * r / (3 * E * S['lever']['Iip'])              # the lever and the crank alike
    c3 = c_rod + c_up + c_tube + c_lev
    K2 = 1 / (1 / K_rv + c2)
    K3 = 1 / (1 / K_rv + c3)
    f = lambda K, M: math.sqrt(K / M) / (2 * pi)
    return dict(K_rv=K_rv, c2=c2, K2=K2, f2=f(K2, pm['M22']), f2_rv=f(K_rv, pm['M22']), c_rod=c_rod, c_up=c_up,
                c_tube=c_tube, c_lev=c_lev, c3=c3, K3=K3, f3=f(K3, pm['M33']), f3_rv=f(K_rv, pm['M33']), h=h,
                M22=pm['M22'], M33=pm['M33'], K2_need=(2 * pi * b['f_min']) ** 2 * pm['M22'],
                K3_need=(2 * pi * b['f_min']) ** 2 * pm['M33'])


def lateral(q2=None, q3=None, b=None, n=200, bodies=None, K_drive=None, at=None):
    """J1 with the arms' lateral give: a unit force across the plane at the flange, the arms bending across and
    twisting (unit load: the integral of M^2/EI + T^2/GJ), the wrist rigid; its compliance as a turn about J1 at the
    flange's radius. Two inertias on two springs: the turret with everything rigidly on it, and the wrist group at
    the flange's radius beyond the arms; the lower of the two frequencies. at: the point (x, z) in m the force acts at
    instead of the flange's (Chapter 59's study pushes on the flange's outer face)."""
    b = b or P.brief()
    hi = P.waypoints(b)['hi']
    q2 = hi['q2'] if q2 is None else q2
    q3 = hi['q3'] if q3 is None else q3
    pp = pose_points(q2, q3)
    fl = pp['fl'] if at is None else at
    fr = arm_frames(q2, q3)
    c = 0.0
    for arm, root, Lm in (('lower', pp['A'], P.LAYOUT['L2'] / 1000), ('upper', pp['B'], P.LAYOUT['L3'] / 1000)):
        ax, nn = fr[arm]
        for k in range(n):
            s = (k + 0.5) / n * Lm
            sec = SECTIONS['fork'] if (arm == 'lower' and S_HUB <= s < S_FORK) else SECTIONS[arm]
            if arm == 'lower' and s < S_HUB:
                continue                                  # inside the hub's bosses, rigid
            Sx, Sz = root[0] + s * ax[0], root[1] + s * ax[2]
            dx, dz = fl[0] - Sx, fl[1] - Sz
            M = (-dz, 0.0, dx)
            T, Ml = dot(M, ax), dot(M, nn)
            c += (Ml * Ml / (E_STEEL * sec['Ilat']) + T * T / (G_STEEL * sec['J'])) * Lm / n
    R = fl[0]
    k_arm = R * R / c                            # N m/rad about J1 at the flange's radius
    K_drive = K_drive or 1 / (1 / rv_K('RV-500N') + 1 / 12e6)   # J1's reducer and its disc coupling (Chapter 42)
    with bodies_in(bodies) as bl:
        m_tip = sum(bd['m'] for bd in with_payload(bl, b['payload']) if bd['move'] == 'trW')
        I1 = P.izz(q2, q3, b['payload'])
    I_tip = m_tip * R * R
    I_rest = I1 - I_tip
    # two inertias: I_rest on K_drive to the ground, I_tip on k_arm to I_rest
    a = I_rest * I_tip
    bb = -(I_rest * k_arm + I_tip * (K_drive + k_arm))
    cc = K_drive * k_arm
    w2 = (-bb - math.sqrt(bb * bb - 4 * a * cc)) / (2 * a)
    return dict(c_flange=c, k_arm=k_arm, K_drive=K_drive, I1=I1, I_tip=I_tip, R=R, f1=math.sqrt(w2) / (2 * pi),
                f1_rigid=math.sqrt(K_drive / I1) / (2 * pi), m_tip=m_tip)


# ------------------------------------------------------------------------------------------------------------------
# J4: the wrist's reducer carries the flange, the gripper and the payload on its main bearing
# ------------------------------------------------------------------------------------------------------------------
def j4_loads(cyc=None, name='RV-80N'):
    """On J4's output, at its installation surface (the flange's face): the thrust along the vertical axis, the
    radial force and the moment of the flange and payload's weights and d'Alembert forces; Nabtesco's moment
    M = W1 (l + b - a) + W2 l2, here the moment about the surface's centre plus W1 (b - a)."""
    cyc = cyc or cycle_cached()
    a, b = rv_geom(name)
    out = []
    for r in cyc['rows']:
        q2, q3 = r['q'][1], r['q'][2]
        pp = pose_points(q2, q3)
        face = (pp['fl'][0], 0.0, pp['fl'][1])
        bl_spin = [f[1:7] for f in r['sol']['loads'] if f[0] == 'wrist' and f[7]]     # the flange and payload
        Fs, Ms = resultant(bl_spin, face)
        W2 = -Fs[2]
        W1 = math.hypot(Fs[0], Fs[1])
        Mf = math.hypot(Ms[0], Ms[1])
        out.append(dict(t=r['t'], W2=W2, W1=W1, M=Mf + W1 * (b - a) / 1000.0, loaded=r['loaded']))
    return out


_FULL = {}


def full_rows(n=1200, bodies=None):
    """Chapter 41's full four-axis model along the cycle with this chapter's bodies (or the weighed ones)."""
    key = (n, 'weighed' if bodies else 'chapter')
    if key not in _FULL:
        with bodies_in(bodies):
            _FULL[key] = P.cycle_run(n=n, full=True)
    return _FULL[key]


def j4_design(name='RV-80N', ratio=101, cyc=None, bodies=None):
    """J4's reducer: its main bearing against the flange and payload's thrust and moment (the allowable moment
    diagram), its life on J4's torques (Nabtesco's Tm and Nm), its peak against TS1, the ratio against the brief's."""
    b = P.brief()
    cyc = cyc or cycle_cached()
    rv = RV_SMALL[name]
    ld = j4_loads(cyc, name)
    k = max(range(len(ld)), key=lambda i: ld[i]['W2'])
    W2max, Mmax = ld[k]['W2'], max(x['M'] for x in ld)
    allow = allowable_moment(name, W2max)
    rows = full_rows(bodies=bodies)['rows']
    st = D.rv_tm(rows, 3, b)
    Lh = D.rv_life(rv['T0'], st['Tm'], st['Nm'])
    Lh_need = st['t_move'] / P.cycle_time(b) * b['life_h']
    peak = max(abs(x[4][3]) for x in rows)
    ratio_max = P.ratios(b)[4]['i_max']
    I4 = sum(bd['Izz'] for bd in with_payload(bodies or chapter_bodies(), b['payload']) if bd.get('spin'))
    return dict(name=name, ratio=ratio, W2max=W2max, W2_static=b['payload'] * G + 13.0 * G, M_max=Mmax,
                allow=allow, Tm=st['Tm'], Nm=st['Nm'], Lh=Lh, Lh_need=Lh_need, peak=peak, TS1=rv['TS1'],
                ratio_max=ratio_max, K=rv_K(name), f4=math.sqrt(rv_K(name) / I4) / (2 * pi), I4=I4,
                ok=allow is not None and Mmax <= allow and Lh >= Lh_need and peak <= rv['TS1'] and ratio <= ratio_max)


# ------------------------------------------------------------------------------------------------------------------
# The chapter's choices and the whole design
# ------------------------------------------------------------------------------------------------------------------
CHOICE43 = dict(D.CHOICE, rv1='RV-700N', i1=203.52, rv4='RV-80N', i4=101)
STATIONS = dict(lower=(0.20, S_FORK, 0.90, 1.15), upper=(0.40, 0.80, 1.25))


def lateral_need(K_drive, I1, I_tip, f=5.0):
    """The arms' lateral stiffness (N m/rad at the flange's radius) that gives J1 the frequency f on K_drive."""
    def f1(k):
        Ir = I1 - I_tip
        a = Ir * I_tip
        bb = -(Ir * k + I_tip * (K_drive + k))
        cc = K_drive * k
        return math.sqrt((-bb - math.sqrt(bb * bb - 4 * a * cc)) / (2 * a)) / (2 * pi)
    if f1(1e13) < f:
        return None
    lo, hi = 1e5, 1e13
    for _ in range(200):
        mid = math.sqrt(lo * hi)
        lo, hi = (mid, hi) if f1(mid) < f else (lo, mid)
    return hi


def design(n=600, bodies=None):
    """Every number of the chapter's design from the chapter's bodies, or from the weighed ones."""
    b = P.brief()
    cyc = cycle(n, bodies) if bodies else cycle_cached(n)
    env = envelope(bodies)
    out = dict(cycle=cyc, env=env)
    out['rods'] = {k: dict(min=min(r['sol'][k] for r in cyc['rows']), max=max(r['sol'][k] for r in cyc['rows']),
                           static_min=min(p[3][k] for p in env['poses']), static_max=max(p[3][k] for p in env['poses']))
                   for k in ('Tp', 'T1', 'T2')}
    out['plain'] = {pin: plain_life(nm, pin, 1.0, N=500, cyc=cyc)
                    for pin, nm in (('E', 'GE 35 ES'), ('G', 'GE 45 ES'), ('Bt', 'GE 50 ES'), ('C', 'GE 70 ES'))}
    out['srb'] = {pin: srb_life(nm, pin, cyc, env) for pin, nm in PIN_BEARING.items()}
    out['joints'] = joint_bearings(cyc, env)
    key = lambda arm, s: 'fork' if (arm == 'lower' and s < S_FORK) else arm
    out['fatigue'] = {(arm, s): weld_fatigue(arm, s, cyc=cyc, key=key(arm, s)) for arm, ss in STATIONS.items() for s in ss}
    out['static'] = {(arm, s): static_yield(arm, s, env, key=key(arm, s)) for arm, ss in STATIONS.items() for s in ss}
    out['columns'] = dict(rod_p=column(SECTIONS['rod_p'], P.LAYOUT['L2'] / 1000),
                          rod_l=column(SECTIONS['rod_l'], P.LAYOUT['L3'] / 1000))
    out['stiff'] = stiffness(bodies=bodies)
    lat = lateral(bodies=bodies)
    out['lateral'] = lat
    K700 = 1 / (1 / rv_K('RV-700N') + 1 / 12e6)
    out['lateral_need'] = dict(rv500=lateral_need(lat['K_drive'], lat['I1'], lat['I_tip']),
                               rv700=lateral_need(K700, lat['I1'], lat['I_tip']), K700=K700)
    Ir, It, k = lat['I1'] - lat['I_tip'], lat['I_tip'], lat['k_arm']
    a, bb, cc = Ir * It, -(Ir * k + It * (K700 + k)), K700 * k
    out['f1_rv700'] = math.sqrt((-bb - math.sqrt(bb * bb - 4 * a * cc)) / (2 * a)) / (2 * pi)
    return out


def rework(bodies=None):
    """The cycle again with the robot as Fusion weighed it: the four joints' torques (Chapter 41's full model), the
    balancer's effect on J2, every reducer's life and peak (J1 now an RV-700N, ratio 203.52), the slewing ring,
    the brakes; and the forces in the arm with the weighed bodies."""
    b = P.brief()
    wb = bodies or weighed_bodies()
    if wb is None:
        raise RuntimeError('no weighed bodies: run checks/palletizer_robot_check.py')
    c = CHOICE43
    out = dict(bodies=wb)
    r = full_rows(1200, wb)
    with bodies_in(wb):
        D._RUN.clear()
        st = P.stiffness(b)
        rows2 = D.rows_balanced(lambda q: D.gas_torque(q), r['rows'])
        out['cycle'] = r
        out['peak'] = {1: r['peak1'], 2: max(abs(x[4][1]) for x in rows2), 3: r['peak3'], 4: r['peak4']}
        out['rms'] = {1: r['rms1'], 2: D.rms(rows2, 1), 3: r['rms3'], 4: r['rms4']}
        out['red'] = {1: D.reducer_check(0, c['rv1'], r['rows'], b, st['K1']),
                      2: D.reducer_check(1, c['rv2'], rows2, b, st['K2']),
                      3: D.reducer_check(2, c['rv3'], r['rows'], b, st['K3'])}
        out['j4'] = j4_design(c['rv4'], c['i4'], cycle(240, wb), wb)
        out['ring'] = D.j1_design(b)
        out['brakes'] = D.stops_and_brakes(b, r['rows'], c)
        out['gas'] = D.gas_design()
        out['stiff_rv'] = st
        out['I1_hi'] = P.izz(P.waypoints(b)['hi']['q2'], P.waypoints(b)['hi']['q3'], b['payload'])
        D._RUN.clear()
    out['stiff'] = stiffness(bodies=wb)
    K700 = 1 / (1 / rv_K('RV-700N') + 1 / 12e6)
    out['lateral'] = lateral(bodies=wb, K_drive=K700)
    return out


if __name__ == '__main__':
    bl = with_payload(chapter_bodies(), P.brief()['payload'])
    lo = P.waypoints()['lo']
    s = plane(lo['q2'], lo['q3'], bl)
    print('low point', {k: round(s[k], 1) for k in UNKNOWNS + ['J2', 'J3']})
    j = joints3d(s, lo['q2'], lo['q3'])
    for k, (F, M) in j.items():
        print(k, [round(v, 1) for v in F], [round(v, 1) for v in M])
