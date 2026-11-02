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


if __name__ == '__main__':
    b = brief()
    print(workspace(b))
    for pl in (b['m_rated'], b['m_max']):
        c = cycle(b, pl)
        for k in ('across', 'radial'):
            m = c[k]
            print(pl, k, {x: round(m[x], 3) for x in ('d1', 'd2', 'w1', 'w2', 'a1', 'a2', 't1', 't2', 'P1', 'P2')})
        print(pl, 'J3', round(c['v3'], 1), round(c['a3'], 1), round(c['F3'], 2), 'J4', round(c['w4'], 2), round(c['a4'], 1), round(c['T4'], 3))
