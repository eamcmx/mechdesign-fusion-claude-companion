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


if __name__ == '__main__':
    b = brief()
    for d in (0.0, 15.0, 40.0):
        print(d, {k: round(v, 4) for k, v in air(b, d).items()})
    act = actuator(b)
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in act.items() if k != 'rows'})
    for r in act['rows'][::10]:
        print({k: round(v, 3) for k, v in r.items()})
