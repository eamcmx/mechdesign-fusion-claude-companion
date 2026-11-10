# -*- coding: utf-8 -*-
"""palletizer_drives.py: Chapter 42 of Volume 2, the palletizer's drives.

The J1 slewing drive (a slewing ring that carries the turret and an RV reducer that only turns it), the J2 module
with its spring balancer, the J3 module with its counterweight, the motors' specification, the brakes and the stops.
Every number in the chapter comes from here; lib/palletizer_design.py (Chapter 41) gives the arm, the cycle and the
loads.

Sources (notes in "Mechanical Design References/Web pages"):
  Nabtesco RV-N catalogue RVN_E0102 (2018), pp. 9, 23-33, 37, 38; the RV-N series and RV-1500E product pages.
  SKF, Slewing bearings (2021), pp. 19-22, 66-67.
  Budynas and Nisbett (2015), Ch. 10 (springs), Ch. 6 (fatigue), Tables A-5 and A-21.
"""
import math

import palletizer_design as P

G = P.G
pi = math.pi
ARCMIN = pi / 180 / 60

# ------------------------------------------------------------------------------------------------------------------
# The reducers (Nabtesco RV-N catalogue, 2018, p. 9 ratings, p. 29 pins, p. 37 efficiency, p. 38 rigidity)
# ------------------------------------------------------------------------------------------------------------------
RV_K, RV_N0 = 6000.0, 15.0            # rated life (h) at the rated output speed (rev/min)
RV = {
    'RV-160N': dict(T0=1600.0, TS1=4000.0, TS2=8000.0, NS0=19.0, NS1=48.0, MO1=4000.0, MO2=8000.0, m=22.1, Z4=40,
                    Kt=490.0, Km=2050.0, a=35.0, b=194.0, ratios=()),
    'RV-380N': dict(T0=3724.0, TS1=9310.0, TS2=18620.0, NS0=11.5, NS1=27.0, MO1=7050.0, MO2=14100.0, m=44.0, Z4=46,
                    Kt=948.0, Km=5200.0, a=48.7, b=248.9, ratios=(75, 93, 117, 139, 162, 185)),
    'RV-500N': dict(T0=4900.0, TS1=12250.0, TS2=24500.0, NS0=11.0, NS1=25.0, MO1=11000.0, MO2=22000.0, m=57.2, Z4=52,
                    Kt=1620.0, Km=6850.0, a=56.3, b=271.7, ratios=(81, 105, 123, 144, 159, 192.75)),
    'RV-700N': dict(T0=7000.0, TS1=17500.0, TS2=35000.0, NS0=7.5, NS1=19.0, MO1=15000.0, MO2=30000.0, m=102.0, Z4=52,
                    Kt=2600.0, Km=9000.0, a=66.3, b=323.5, ratios=(105, 118, 142.44, 159, 183, 203.52)),
}
# the product pages: sizes above the catalogue's
RV1500E = dict(T0=14700.0, TS1=36750.0, TS2=73500.0, MO1=44100.0, Fa=51000.0, Fr=97265.0, m=280.0, Kt=6320.0,
               Km=19800.0, ratios=(65, 156, 236.29))
ETA_RV = 0.75                          # read on p. 37 for RV-500N at 15 rev/min and 4500 N m (69 % at 3000, 81 % at 6000)


def joint_moving(j, b=None):
    """Joint j's moving time in a cycle (s) and its travel (rev), exactly from the legs: a joint moves through every
    leg in which its angle changes."""
    b = b or P.brief()
    legs, _ = P.timeline(b)
    t = sum(T for (t0, T, a, c, _) in legs if abs(c[j] - a[j]) > 1e-9)
    rev = sum(abs(c[j] - a[j]) for (t0, T, a, c, _) in legs) / 360.0
    return t, rev


def rv_tm(rows, j, b=None):
    """Nabtesco's average load torque and speed over the moving time (p. 27): Tm = (sum t N T^(10/3) / sum t N)^(3/10),
    with t the time step, N the output speed (rev/min) and T the torque (N m) of joint j; Nm = the travel over the
    moving time, both exact from the legs."""
    dt = rows[1][0] - rows[0][0]
    sTN = sN = 0.0
    for r in rows[:-1]:
        n = abs(r[2][j]) * 60 / (2 * pi)
        T = abs(r[4][j])
        sTN += dt * n * T ** (10.0 / 3.0)
        sN += dt * n
    t_move, rev = joint_moving(j, b)
    return dict(Tm=(sTN / sN) ** 0.3, Nm=rev / t_move * 60.0, t_move=t_move)


def rv_life(T0, Tm, Nm):
    """Life in hours of operation (p. 30): Lh = K (N0/Nm) (T0/Tm)^(10/3)."""
    return RV_K * (RV_N0 / Nm) * (T0 / Tm) ** (10.0 / 3.0)


def rv_t0_needed(Tm, Nm, L_hours):
    """The rated torque that gives the life (p. 28): T0' = Tm (L/K . Nm/N0)^(3/10)."""
    return Tm * (L_hours / RV_K * Nm / RV_N0) ** 0.3


def rv_cem(TS2, Tem, Z4, Nem, tem):
    """How many emergency stops the reducer takes (p. 29): Cem = 775 (TS2/Tem)^(10/3) / (Z4 Nem/60 tem)."""
    return 775.0 * (TS2 / Tem) ** (10.0 / 3.0) / (Z4 * Nem / 60.0 * tem)


# ------------------------------------------------------------------------------------------------------------------
# The cycle once, with the full model (Chapter 41), shared by everything below
# ------------------------------------------------------------------------------------------------------------------
_RUN = {}


def run(n=1200):
    if n not in _RUN:
        _RUN[n] = P.cycle_run(n=n, full=True)
    return _RUN[n]


def moving_hours(j, b=None):
    """The hours joint j moves in the robot's life: its moving time per cycle times the cycles."""
    b = b or P.brief()
    return joint_moving(j, b)[0] / P.cycle_time(b) * b['life_h']


# ------------------------------------------------------------------------------------------------------------------
# J1: the turret's bearing loads along the cycle, with the groups' inertia forces
# ------------------------------------------------------------------------------------------------------------------
Z_RING = 0.52        # m, the slewing ring's plane above the floor (the base's height, chosen)


def _centres(q, m_pay):
    return [(bd['m'], P.body_pose(bd, q)[0]) for bd in P.bodies(m_pay)]


def j1_loads_dyn(b=None, n=1200, h=1e-3):
    """At each step of the cycle: the axial force Fa (N) and the tilting moment M (N m) on the turret's bearing,
    about its centre at height Z_RING, from the groups' weights and the forces that accelerate their centres
    (accelerations by central differences of the centres along the cycle). The groups' own rotary inertia about
    horizontal axes is left out; j1_rotary_share() measures it."""
    b = b or P.brief()
    legs, Tc = P.timeline(b)
    out = []
    for i in range(n + 1):
        t = Tc * i / n
        qs = []
        for tt in (t - h, t, t + h):
            q, _, _, loaded = P.cycle_state(tt % Tc, b, legs)
            qs.append(tuple(math.radians(v) for v in q))
        loaded = P.cycle_state(t, b, legs)[3]
        m_pay = b['payload'] if loaded else b['gripper']
        cm, c0, cp = (_centres(qq, m_pay) for qq in qs)
        Fx = Fy = Fz = Mx = My = 0.0
        for (m, a), (_, c), (_, d) in zip(cm, c0, cp):
            acc = [(d[k] - 2 * c[k] + a[k]) / h ** 2 for k in range(3)]
            f = (-m * acc[0], -m * acc[1], -m * (G + acc[2]))         # the load each group puts on the bearing
            r = (c[0], c[1], c[2] - Z_RING)
            Fx += f[0]; Fy += f[1]; Fz += f[2]
            Mx += r[1] * f[2] - r[2] * f[1]
            My += r[2] * f[0] - r[0] * f[2]
        out.append(dict(t=t, Fa=-Fz, M=math.hypot(Mx, My), Fr=math.hypot(Fx, Fy), q1=qs[1][0], loaded=loaded))
    return out


def j1_rotary_share(b=None):
    """The largest moment the arm groups' own angular accelerations about the shoulder's axis add (N m): the sum of
    each group's inertia about its own transverse axis times q2's or q3's acceleration, over the cycle."""
    r = run()
    worst = 0.0
    for row in r['rows']:
        qdd = row[3]
        s = 0.0
        for bd in P.BODIES:
            if bd['move'] in ('q2@A', 'q2@E'):
                s += bd.get('It', 0.0) * abs(qdd[1])
            elif bd['move'] in ('q3@A', 'q3@B', 'q3@G'):
                s += bd.get('It', 0.0) * abs(qdd[2])
        worst = max(worst, s)
    return worst


# the slewing rings (SKF, light series without a gear, pp. 66-67); raceway and bolting lines read from the diagram
RINGS = [
    dict(name='RKS.23 0641', dm=641.0, D=748.0, d=534.0, m=36.5, C=200e3, C0=440e3, r0=68e3, b0=40e3, bs=0.068),
    dict(name='RKS.23 0741', dm=741.0, D=848.0, d=634.0, m=43.0, C=211e3, C0=524e3, r0=88e3, b0=47e3, bs=0.116),
    dict(name='RKS.23 0841', dm=841.0, D=948.0, d=734.0, m=48.0, C=222e3, C0=580e3, r0=111e3, b0=54e3, bs=0.123),
    dict(name='RKS.23 0941', dm=941.0, D=1048.0, d=834.0, m=54.0, C=231e3, C0=664e3, r0=137e3, b0=71e3, bs=0.137),
    dict(name='RKS.23 1091', dm=1091.0, D=1198.0, d=984.0, m=62.0, C=244e3, C0=766e3, r0=181e3, b0=89e3, bs=0.176),
]
FL = 1.33              # SKF's load factor, Table 1 p. 21: the value for aerial platforms and mini excavators, chosen


def ring_static(ring, Fa, M, fl=FL):
    """SKF's check (p. 21): fL Fa and fL M under the raceway line r0 (1 - Fa/C0) and the bolting line b0 + bs Fa."""
    Far, Mtr = fl * Fa, fl * M
    race = ring['r0'] * (1 - Far / ring['C0'])
    bolt = ring['b0'] + ring['bs'] * Far
    return dict(Far=Far, Mtr=Mtr, race=race, bolt=bolt, ok=Mtr <= race and Mtr <= bolt)


def ring_life(ring, loads):
    """An estimate of the ring's rating life in turns. With the rings rigid and the balls' loads in proportion to the
    rings' approach, the most loaded ball of a ring loaded by Fa and M carries what a uniform axial load
    P = Fa + 4 M/dm would give it; P is averaged as its cube over the turret's turning, and L10 = (C/P)^3 million
    turns, as for a thrust ball bearing. SKF gives no life method (p. 19): this is the book's estimate."""
    num = den = 0.0
    dm = ring['dm'] / 1000
    for a, c in zip(loads[:-1], loads[1:]):
        dq = abs(c['q1'] - a['q1'])
        p = a['Fa'] + 4 * a['M'] / dm
        num += p ** 3 * dq
        den += dq
    Pm = (num / den) ** (1.0 / 3.0)
    return dict(Pm=Pm, L10=(ring['C'] / Pm) ** 3 * 1e6)


def j1_design(b=None):
    """J1: the loads, the RV-1500E on its own bearing against the ring, and the ring chosen."""
    b = b or P.brief()
    loads = j1_loads_dyn(b)
    Fa_max = max(r['Fa'] for r in loads)
    M_max = max(r['M'] for r in loads)
    k = max(range(len(loads)), key=lambda i: loads[i]['M'])
    reach = P.j1_loads(b)['reach']
    turns = P.life_counts(b)['turns'][0]
    rings = []
    for ring in RINGS:
        cyc = ring_static(ring, Fa_max, M_max)
        rea = ring_static(ring, reach['Fa'], reach['M'])
        life = ring_life(ring, loads)
        rings.append(dict(ring=ring, cycle=cyc, reach=rea, life=life, ok=cyc['ok'] and rea['ok'] and life['L10'] >= turns,
                          ok_margin=cyc['ok'] and rea['ok'] and life['L10'] >= CHOICE['ring_margin'] * turns))
    chosen = next(r for r in rings if r['ok_margin'])
    rv1500 = dict(reach=(FL * reach['Fa'], FL * reach['M']), allow=(RV1500E['Fa'], RV1500E['MO1']))
    return dict(loads=loads, Fa_max=Fa_max, M_max=M_max, M_max_t=loads[k]['t'], reach=reach, turns=turns,
                rings=rings, chosen=chosen, rv1500=rv1500, rotary=j1_rotary_share(b))


# ------------------------------------------------------------------------------------------------------------------
# The reducers' checks for a joint: life, peaks, speed, stiffness
# ------------------------------------------------------------------------------------------------------------------
def reducer_check(j, name, rows=None, b=None, K_need=None, extra=0.0):
    """Joint j's reducer `name` on its torques over the cycle (rows; the full model's by default)."""
    b = b or P.brief()
    rows = rows or run()['rows']
    rv = RV[name]
    st = rv_tm(rows, j, b)
    Lh_need = st['t_move'] / P.cycle_time(b) * b['life_h']
    Lh = rv_life(rv['T0'], st['Tm'], st['Nm'])
    peak = max(abs(r[4][j]) for r in rows)
    nmax = max(abs(r[2][j]) for r in rows) * 60 / (2 * pi)
    n_avg = sum(abs(r[2][j]) for r in rows[:-1]) / (len(rows) - 1) * 60 / (2 * pi)
    K = rv['Kt'] / ARCMIN
    return dict(name=name, Tm=st['Tm'], Nm=st['Nm'], t_move=st['t_move'], Lh=Lh, Lh_need=Lh_need,
                T0_need=rv_t0_needed(st['Tm'], st['Nm'], Lh_need), peak=peak, TS1=rv['TS1'], nmax=nmax,
                n_avg=n_avg, NS0=rv['NS0'], NS1=rv['NS1'], K=K, K_need=K_need,
                ok=Lh >= Lh_need and peak <= rv['TS1'] and n_avg <= rv['NS0'] and (K_need is None or K >= K_need))


# ------------------------------------------------------------------------------------------------------------------
# J2: the balancer. A steel spring first, to see what it would weigh; then the gas spring the arm gets.
# ------------------------------------------------------------------------------------------------------------------
def rms(rows, j):
    v = [r[4][j] for r in rows[:-1]]
    return math.sqrt(sum(x * x for x in v) / len(v))


def j2_hold(T_bal, payload=None):
    """The largest torque J2 must still hold over its range with the payload, gravity less the balancer:
    T_bal(q2) is the balancer's torque on the arm (N m, positive with q2)."""
    S2 = P.gravity_sums(P.brief()['payload'] if payload is None else payload)['S2']
    return max(abs(G * S2 * math.sin(math.radians(q)) + T_bal(q)) for q in range(int(P.LIMITS['q2'][0]),
                                                                                int(P.LIMITS['q2'][1]) + 1))


def balance_work(payload=None):
    """The work gravity does on the lower arm from q2 = 0 to the range's ends: what a full balancer stores."""
    S2 = P.gravity_sums(P.brief()['payload'] if payload is None else payload)['S2']
    return {q: G * S2 * (1 - math.cos(math.radians(q))) for q in P.LIMITS['q2']}


def steel_energy_density(tau, KB=1.2, Gm=79.3e9):
    """The energy a helical spring's active wire stores per unit volume at the stress tau: with
    tau = K_B 8 F D/(pi d^3) (Eq. 10-7) and y = 8 F D^3 Na/(d^4 G) (Eq. 10-8), U = F y/2 = tau^2 V/(4 K_B^2 G),
    V = (pi d^2/4)(pi D Na) the active wire's volume."""
    return tau ** 2 / (4 * KB ** 2 * Gm)


# the gas balancer: two cylinders, one each side of the lower arm, each from a pin on the turret a below the shoulder
# to a lug on the arm b above it, pushing; nitrogen on the piston's cap side, the rod's side vented. F0 is the pair's.
GAS = dict(a=0.35, b=0.55, F0=54000.0, V0A=1.0, n=1.0, bore=0.063, rod=0.032, ncyl=2)


def gas_length(q2, g=GAS):
    """The pins' distance (m): L^2 = a^2 + b^2 + 2 a b cos q2 (the pin below, the lug above the shoulder)."""
    return math.sqrt(g['a'] ** 2 + g['b'] ** 2 + 2 * g['a'] * g['b'] * math.cos(math.radians(q2)))


def gas_force(q2, g=GAS):
    """F = F0 (V0/(V0 - A (Lmax - L)))^n: F0 at the longest length (q2 = 0), the gas column V0/A long there."""
    Lmax = gas_length(0.0, g)
    return g['F0'] * (g['V0A'] / (g['V0A'] - (Lmax - gas_length(q2, g)))) ** g['n']


def gas_torque(q2, g=GAS):
    """The push's torque on the lower arm (N m, positive with q2): -F a b sin q2 / L, restoring both ways."""
    return -gas_force(q2, g) * g['a'] * g['b'] * math.sin(math.radians(q2)) / gas_length(q2, g)


def rows_balanced(T_bal, rows=None):
    """The cycle's rows with the balancer's torque taken off J2's drive (drive + gravity + balancer = inertia)."""
    rows = rows or run()['rows']
    out = []
    for (t, q, qd, qdd, tau, loaded) in rows:
        tau2 = list(tau)
        tau2[1] = tau[1] - T_bal(q[1])
        out.append((t, q, qd, qdd, tau2, loaded))
    return out


def gas_search(V0As=(0.6, 0.8, 1.0, 1.25, 1.5, 2.0, 3.0, 6.0), F0s=None, g=GAS):
    """For each gas column V0/A: the F0 that makes J2's holding torque over the range the least, with the cycle's
    RMS that results; the table the chapter prints."""
    F0s = F0s or [40000.0 + 1000.0 * i for i in range(61)]
    out = []
    for V0A in V0As:
        best = None
        for F0 in F0s:
            gg = dict(g, V0A=V0A, F0=F0)
            h = j2_hold(lambda q: gas_torque(q, gg))
            if best is None or h < best[0]:
                best = (h, F0)
        gg = dict(g, V0A=V0A, F0=best[1])
        Ls = gas_length(0, gg) - gas_length(P.LIMITS['q2'][1], gg)
        out.append(dict(V0A=V0A, ratio=V0A / Ls, F0=best[1], hold=best[0],
                        rms=rms(rows_balanced(lambda q: gas_torque(q, gg)), 1),
                        Fmax=max(gas_force(q, gg) for q in range(-42, 86))))
    return out


def gas_design(g=GAS, Sy=530e6, n_wall=3.0):
    """The balancer's numbers: lengths, stroke, the pair's forces, each cylinder's pressures in its bore and its gas
    volumes, and the cylinder's wall by Lame's equations (Budynas and Nisbett, 2015, Eq. 3-50) for a factor n_wall
    on the tube's yield strength (AISI 1045 cold drawn, Table A-20, Sy 530 MPa)."""
    qs = range(int(P.LIMITS['q2'][0]), int(P.LIMITS['q2'][1]) + 1)
    Ls = [gas_length(q, g) for q in qs]
    Fs = [gas_force(q, g) for q in qs]
    A = pi * g['bore'] ** 2 / 4
    Lmax, Lmin = max(Ls), min(Ls)
    pmax, p0 = max(Fs) / g['ncyl'] / A, g['F0'] / g['ncyl'] / A
    ri = g['bore'] / 2
    s_allow = Sy / n_wall
    ro = ri * math.sqrt((s_allow + pmax) / (s_allow - pmax))     # sigma_t at the bore = p (ro^2 + ri^2)/(ro^2 - ri^2)
    cyc = [gas_length(r[1][1], g) for r in run()['rows']]
    Fc = [gas_force(r[1][1], g) for r in run()['rows']]
    return dict(Lmax=Lmax, Lmin=Lmin, stroke=Lmax - Lmin, L_cycle=(min(cyc), max(cyc)), F_cycle=(min(Fc), max(Fc)),
                Fmin=min(Fs), Fmax=max(Fs), A=A, p0=p0, pmax=pmax, V0=g['V0A'] * A, dV=A * (Lmax - Lmin),
                ri=ri, ro_min=ro, s_allow=s_allow, hold=j2_hold(lambda q: gas_torque(q, g)),
                hold0=j2_hold(lambda q: 0.0), hold_one=j2_hold(lambda q: gas_torque(q, g) / g['ncyl']),
                energy=gas_work(0.0, P.LIMITS['q2'][1], g))


def gas_work(q_a, q_b, g=GAS, n=400):
    """The work the gas takes in as the arm turns from q_a to q_b (J): the integral of F dL, L falling."""
    w = 0.0
    for i in range(n):
        qa, qb = q_a + (q_b - q_a) * i / n, q_a + (q_b - q_a) * (i + 1) / n
        w += 0.5 * (gas_force(qa, g) + gas_force(qb, g)) * (gas_length(qa, g) - gas_length(qb, g))
    return w


def j2_rows(g=GAS):
    return rows_balanced(lambda q: gas_torque(q, g))


# ------------------------------------------------------------------------------------------------------------------
# J3: the counterweight added to the crank
# ------------------------------------------------------------------------------------------------------------------
CW_BLOCK = (0.46, 0.46)            # m: the added steel block's length along the crank and its height


def cw_thickness(dm_cw, block=CW_BLOCK):
    """The block's thickness (m) for its mass, steel of 7850 kg/m3."""
    return dm_cw / (7850.0 * block[0] * block[1])


def with_counterweight(dm_cw, r_cw=0.60, base=None, y_crank=640.0, block=CW_BLOCK):
    """The bodies (Chapter 41's, or base) with the crank moved out to y_crank (mm), clear of the J3 cheek and the
    base's flange, and a steel block of dm_cw kg bolted to its outer face, 5 mm out from the web (60 mm thick), its
    centre r_cw (m) behind the shoulder on the crank's line."""
    out = []
    for bd in (base if base is not None else P.BODIES):
        if bd['name'] == 'crank and counterweight':
            bd = dict(bd, y=y_crank)
        out.append(bd)
    if dm_cw > 0:
        a, h = block
        t = cw_thickness(dm_cw, block)
        out.append(dict(name='counterweight', m=dm_cw, move='q3@A', c=-r_cw * 1000.0, y=y_crank + 35.0 + t * 500.0,
                        Ia=dm_cw * (h * h + t * t) / 12, It=dm_cw * (a * a + h * h) / 12, In=dm_cw * (a * a + t * t) / 12))
    return out


def cw_sweep(masses=(0, 100, 200, 300, 400, 500, 600), r_cw=0.60, n=600):
    """J3's peak and RMS over the cycle, J1's inertia at reach and the turret's tipping moment at reach, for each
    added counterweight."""
    saved = P.BODIES[:]
    res = []
    try:
        for dm in masses:
            P.BODIES[:] = with_counterweight(dm, r_cw, base=saved)
            r = P.cycle_run(n=n, full=True)
            ws = P.workspace()
            I1r = P.izz(ws['reach_q2'], ws['reach_q3'], P.brief()['payload'])
            tip = P.j1_loads()['reach']
            gs = P.gravity_sums(P.brief()['payload'])
            res.append(dict(dm=dm, peak3=r['peak3'], rms3=r['rms3'], peak1=r['peak1'], rms1=r['rms1'], I1r=I1r,
                            M_reach=tip['M'], Fa=tip['Fa'], S3=gs['S3'], hold3=G * gs['S3']))
    finally:
        P.BODIES[:] = saved
    return res


def cw_clearance(r_cw=0.60, R_lump=0.20):
    """The counterweight's lowest point at q3 = 60 deg (the crank then points down and back at 60 deg below the
    horizontal), against the ring's plane: z = A_z - r sin 60 - R_lump."""
    return P.LAYOUT['A'][1] / 1000 - r_cw * math.sin(math.radians(60)) - R_lump


# ------------------------------------------------------------------------------------------------------------------
# The motors and brakes
# ------------------------------------------------------------------------------------------------------------------
def motor_spec(rows, j, ratio, Jm, eta=ETA_RV):
    """Joint j's motor through a reducer of ratio i: T_m = T/(i eta) + Jm i qdd; the RMS over the cycle, the peak,
    the top speed (rev/min)."""
    Tm = [r[4][j] / (ratio * eta) + Jm * ratio * r[3][j] for r in rows]
    rmsv = math.sqrt(sum(x * x for x in Tm[:-1]) / (len(Tm) - 1))
    n = max(abs(r[2][j]) for r in rows) * ratio * 60 / (2 * pi)
    return dict(rms=rmsv, peak=max(abs(x) for x in Tm), n=n)


def brake_bounds(J_out, J_rot_out, w0, theta, T_rep, ratio):
    """Chapter 36's two bounds on a brake T_b at the motor: stop from w0 within theta, T_b >= w0^2 (Ja + Jr)/(2 theta i),
    and keep the reducer under T_rep, T_b <= T_rep (Ja + Jr)/(Ja i); the joint's own gravity torque is added outside."""
    lo = w0 ** 2 * (J_out + J_rot_out) / (2 * theta * ratio)
    hi = T_rep * (J_out + J_rot_out) / (J_out * ratio)
    return lo, hi


# ------------------------------------------------------------------------------------------------------------------
# The design: the choices of Chapter 42 and everything they give
# ------------------------------------------------------------------------------------------------------------------
CHOICE = dict(ring_margin=1.5, rv1='RV-500N', i1=192.75, rv2='RV-500N', i2=192.75, rv3='RV-500N', i3=192.75,
              cw=100.0, r_cw=0.60, Jm=(30e-4, 25e-4, 30e-4), theta_stop=math.radians(20.0), brake=(62.0, 64.0, 60.0),
              pem=250)


def stops_and_brakes(b, rows, c=CHOICE):
    """Each joint's brake against Chapter 36's bounds, its holding factor and its emergency stops (Nabtesco p. 29):
    the brake T_b at the motor stops the joint at R T_b/(Ja + Jr) and the reducer carries Ja/(Ja + Jr) of R T_b, plus
    the gravity torque for J2 and J3, moving down: J2's what its balancer leaves; J2's brake holds the arm with one
    cylinder empty. J1 at the cycle's high point; J2 and J3 with their planar inertias; one emergency stop by the
    brakes a week over the life (pem)."""
    st = P.stiffness(b)
    pm = P.planar_mass(b['payload'])
    w0 = math.radians(b['w_max'][0])
    gd = gas_design()
    hold = {1: 0.0, 2: gd['hold'], 3: G * P.gravity_sums(b['payload'])['S3']}
    hold_static = {1: 0.0, 2: gd['hold_one'], 3: hold[3]}
    out = {}
    for j, Ja, rvn, i, Jm, Tb in ((1, st['I1'], c['rv1'], c['i1'], c['Jm'][0], c['brake'][0]),
                                  (2, pm['M22'], c['rv2'], c['i2'], c['Jm'][1], c['brake'][1]),
                                  (3, pm['M33'], c['rv3'], c['i3'], c['Jm'][2], c['brake'][2])):
        rv = RV[rvn]
        Jr = i * i * Jm
        lo, hi1 = brake_bounds(Ja, Jr, w0, c['theta_stop'], rv['TS1'], i)
        hi2 = brake_bounds(Ja, Jr, w0, c['theta_stop'], rv['TS2'], i)[1]
        acc = i * Tb / (Ja + Jr)
        T_red = Ja * acc + hold[j]
        t_em = w0 / acc
        out[j] = dict(Ja=Ja, Jr=Jr, lo=lo, hi_TS1=hi1, hi_TS2=hi2, Tb=Tb, acc=acc, angle=w0 ** 2 / (2 * acc),
                      T_red=T_red, TS1=rv['TS1'], TS2=rv['TS2'], t_em=t_em, hold=hold[j],
                      hold_static=hold_static[j], hold_factor=(Tb * i / hold_static[j]) if hold_static[j] else None,
                      Cem=rv_cem(rv['TS2'], T_red, rv['Z4'], w0 * 60 / (2 * pi), t_em), Pem=c['pem'])
    return out


def design(b=None, c=CHOICE):
    b = b or P.brief()
    saved = P.BODIES[:]
    try:
        P.BODIES[:] = with_counterweight(c['cw'], c['r_cw'], base=saved)
        r = P.cycle_run(n=1200, full=True)
        st = P.stiffness(b)
        rows2 = rows_balanced(lambda q: gas_torque(q), r['rows'])
        d = dict(cycle=r, stiff=st, gas=gas_design(), gas_table=gas_search(), j1=j1_design(b))
        d['red'] = {1: reducer_check(0, c['rv1'], r['rows'], b, st['K1']),
                    2: reducer_check(1, c['rv2'], rows2, b, st['K2']),
                    3: reducer_check(2, c['rv3'], r['rows'], b, st['K3'])}
        d['motor'] = {1: motor_spec(r['rows'], 0, c['i1'], c['Jm'][0]), 2: motor_spec(rows2, 1, c['i2'], c['Jm'][1]),
                      3: motor_spec(r['rows'], 2, c['i3'], c['Jm'][2])}
        d['rms'] = {1: r['rms1'], 2: rms(rows2, 1), 3: r['rms3']}
        d['peak'] = {1: r['peak1'], 2: max(abs(x[4][1]) for x in rows2), 3: r['peak3']}
        gs = P.gravity_sums(b['payload'])
        d['hold'] = {2: d['gas']['hold'], 3: G * gs['S3'], '2_empty_balancer': d['gas']['hold0']}
        d['brakes'] = stops_and_brakes(b, r['rows'], c)
        d['I1_reach'] = st['I1_reach']
        d['E1_reach'] = 0.5 * st['I1_reach'] * math.radians(b['w_max'][0]) ** 2
        d['loads_reach'] = P.j1_loads(b)['reach']
        d['S3'] = gs['S3']
    finally:
        P.BODIES[:] = saved
    return d


if __name__ == '__main__':
    b = P.brief()
    d1 = j1_design(b)
    print('J1 loads: Fa max %.0f N, M max %.0f N m at %.3f s; reach Fa %.0f, M %.0f; rotary %.0f N m'
          % (d1['Fa_max'], d1['M_max'], d1['M_max_t'], d1['reach']['Fa'], d1['reach']['M'], d1['rotary']))
    for r in d1['rings']:
        print('  %s: cycle %.1f/%.1f/%.1f kN m, reach %.1f/%.1f/%.1f, Pm %.1f kN, L10 %.2f M turns (need %.2f) %s'
              % (r['ring']['name'], r['cycle']['Mtr'] / 1e3, r['cycle']['race'] / 1e3, r['cycle']['bolt'] / 1e3,
                 r['reach']['Mtr'] / 1e3, r['reach']['race'] / 1e3, r['reach']['bolt'] / 1e3, r['life']['Pm'] / 1e3,
                 r['life']['L10'] / 1e6, d1['turns'] / 1e6, r['ok']))
    print('  RV-1500E: needs %.0f N, %.0f N m; allows %.0f, %.0f' % (d1['rv1500']['reach'] + d1['rv1500']['allow']))
    st = P.stiffness(b)
    for name in ('RV-380N', 'RV-500N', 'RV-700N'):
        c = reducer_check(0, name, K_need=st['K1'])
        print('J1 %s: Tm %.0f Nm %.2f move %.2f s Lh %.0f (need %.0f) T0' + "'" + ' %.0f peak %.0f/%.0f n %.2f/%.2f K %.2e/%.2e %s'
              and '', end='')
        print('J1 %s: Tm %.0f, Nm %.2f, Lh %.0f need %.0f, T0n %.0f, peak %.0f/%.0f, navg %.2f/%.1f, K %.2e/%.2e %s'
              % (name, c['Tm'], c['Nm'], c['Lh'], c['Lh_need'], c['T0_need'], c['peak'], c['TS1'], c['n_avg'], c['NS0'],
                 c['K'], c['K_need'], c['ok']))
