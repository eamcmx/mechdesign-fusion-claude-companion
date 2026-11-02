# -*- coding: utf-8 -*-
"""tool_changer.py: Chapter 44 of Volume 2, the palletizer's tool flange and its tool changer.

The interface the palletizer hands to its tools (Chapter 43): the loads at the RV-80N's output face along the standard
cycle and in the emergency stops; the robot's plate chosen from ISO 9409-1's Table 1 by a bolted joint worked with
Shigley's method; and an automatic tool changer: a double-acting piston whose cam holds itself drives six segments
into the tool plate's ring, sized so the coupling face does not open on the cycle or in the stops. Balls are checked
first and rejected on their contact stress; then the segments' faces, the cam, the ring's hoop and the lip at both
ends of the friction's range, the release under the piston, and the air one change uses.
"""
import math

import palletizer_design as P
import palletizer_drives as D
import palletizer_links as L

G = P.G
pi = math.pi
E_STEEL, NU_STEEL = 207e9, 0.30          # Budynas & Nisbett (2015), Table A-5

# ------------------------------------------------------------------------------------------------------------------
# ISO 9409-1:1996, Table 1 (mm): d1 the pitch circle, d2 h8 the plate's outside, d3 H7 the centring recess, the thread,
# d5 H7 the pin hole, the depths t1 to t3 at least, t5 the holes' position tolerance, the number of holes
# ------------------------------------------------------------------------------------------------------------------
ISO9409 = {
    25: dict(series=1, d2=34, d3=16, d4='M4', d5=4, t1=6, t2=4, t3=4, t5=0.1, n=4),
    31.5: dict(series=2, d2=40, d3=20, d4='M5', d5=5, t1=6, t2=5, t3=4, t5=0.1, n=4),
    40: dict(series=1, d2=50, d3=25, d4='M6', d5=6, t1=6, t2=6, t3=6, t5=0.1, n=4),
    50: dict(series=2, d2=63, d3=31.5, d4='M6', d5=6, t1=6, t2=6, t3=6, t5=0.1, n=4),
    63: dict(series=1, d2=80, d3=40, d4='M6', d5=6, t1=6, t2=6, t3=6, t5=0.1, n=4),
    80: dict(series=2, d2=100, d3=50, d4='M8', d5=8, t1=6, t2=8, t3=6, t5=0.15, n=6),
    100: dict(series=1, d2=125, d3=63, d4='M8', d5=8, t1=6, t2=8, t3=6, t5=0.15, n=6),
    125: dict(series=2, d2=160, d3=80, d4='M10', d5=10, t1=8, t2=10, t3=8, t5=0.15, n=6),
    160: dict(series=1, d2=200, d3=100, d4='M10', d5=10, t1=8, t2=10, t3=8, t5=0.15, n=6),
    200: dict(series=2, d2=250, d3=125, d4='M12', d5=12, t1=8, t2=12, t3=8, t5=0.2, n=6),
    250: dict(series=1, d2=315, d3=160, d4='M12', d5=12, t1=8, t2=12, t3=8, t5=0.2, n=6),
}


def iso_holes(d1):
    """The threaded holes' angles (deg from +Xm): up to 63 mm four at 45 + 90 k, above six at 30 + 60 k; the pin
    hole on +Xm (ISO 9409-1:1996, Figure 1)."""
    n = ISO9409[d1]['n']
    return [45.0 + 90.0 * k for k in range(4)] if n == 4 else [30.0 + 60.0 * k for k in range(6)]


# the threads (Budynas & Nisbett, 2015, Table 8-1, p. 404): d, pitch, tensile stress area (mm, mm2)
THREAD = {'M8': (8.0, 1.25, 36.6), 'M10': (10.0, 1.5, 58.0), 'M12': (12.0, 1.75, 84.3)}
# property classes (Table 8-11, p. 427) and endurance strengths (Table 8-17, p. 437), MPa
CLASS = {'10.9': dict(Sp=830.0, Sut=1040.0, Sy=940.0, Se=162.0), '12.9': dict(Sp=970.0, Sut=1220.0, Sy=1100.0, Se=190.0)}
MU = 0.15            # friction between the plates and in the lock, assumed (dry or lightly oiled steel)
MU_MIN = 0.10        # the lowest it may be, for the lock's holding itself, assumed


# ------------------------------------------------------------------------------------------------------------------
# The loads at the interface: what hangs below it (mass m at depth h below the face, off the axis by e) and its
# acceleration along the cycle, from the payload's path (Chapter 41's cycle, the wrist only translating)
# ------------------------------------------------------------------------------------------------------------------
def hanging(b=None):
    b = b or P.brief()
    return dict(m=b['payload'], drop=b['drop'] / 1000.0, e=b['offcentre'] / 1000.0,
                I=b['payload'] * (b['layer'][0] ** 2 + b['layer'][1] ** 2) / 12e6)


def _face_point(q):
    """The flange face's centre in the world (m) at q (rad) and the rotation of the flange's frame (J1 + J4)."""
    p = P.pts(math.degrees(q[1]), math.degrees(q[2]))
    x, z = p['fl'][0] / 1000.0, p['fl'][1] / 1000.0
    return (math.cos(q[0]) * x, math.sin(q[0]) * x, z), q[0] + q[3]


def interface(a, alpha_z, psi, hz, b=None, depth=0.0):
    """The loads a plane `depth` below the face carries for a hanging mass with acceleration a (world, m/s2) and
    angular acceleration alpha_z about the vertical, its offset e at angle psi in the flange's frame (flange turned
    hz): axial tension, shear, tipping moment, torque (N, N m)."""
    hg = hanging(b)
    m = hg['m']
    F = (m * a[0], m * a[1], m * (a[2] + G))         # what the interface supplies to hold it, with g
    ang = hz + psi
    r = (hg['e'] * math.cos(ang), hg['e'] * math.sin(ang), -(hg['drop'] - depth))
    M = L.cross(r, F)
    return dict(Fa=F[2], V=math.hypot(F[0], F[1]), M=math.hypot(M[0], M[1]), T=abs(M[2] + hg['I'] * alpha_z))


def cycle_loads(n=600, depth=0.0, b=None, nangle=8):
    """The interface's loads along the cycle, the payload held (the gripper alone weighs less): for each instant the
    worst of nangle directions of the 50 mm offset."""
    b = b or P.brief()
    legs, Tc = P.timeline(b)
    h = 1e-4
    out = []
    for k in range(n + 1):
        t = Tc * k / n
        q, qd, qdd, loaded = P.cycle_state(t, b, legs)
        if not loaded:
            continue
        pts = [_face_point([math.radians(v) for v in P.cycle_state(t + s, b, legs)[0]]) for s in (-h, 0.0, h)]
        a = tuple((pts[2][0][i] - 2 * pts[1][0][i] + pts[0][0][i]) / h ** 2 for i in range(3))
        az = (pts[2][1] - 2 * pts[1][1] + pts[0][1]) / h ** 2
        rows = [interface(a, az, 2 * pi * j / nangle, pts[1][1], b, depth) for j in range(nangle)]
        out.append(dict(t=t, a=a, alpha=az, Fa=max(r['Fa'] for r in rows), V=max(r['V'] for r in rows),
                        M=max(r['M'] for r in rows), T=max(r['T'] for r in rows), rows=rows))
    return out


def estop_loads(depth=0.0, b=None, theta4=None):
    """The emergency stops: each joint stopped by its brake (Chapter 42's decelerations with the weighed robot; J4
    stopped from its top speed within the same 20 deg the others are), at the cycle's high and low points, the
    payload held, J1's centripetal acceleration at its top speed on the cycle added; either sense of each stop."""
    b = b or P.brief()
    acc = dict(D.CHOICE_ESTOP) if hasattr(D, 'CHOICE_ESTOP') else {1: 3.559, 2: 6.645, 3: 7.964}
    w4 = math.radians(b['w_max'][3])
    th = theta4 or D.CHOICE['theta_stop']
    acc[4] = w4 ** 2 / (2 * th)
    lp = P.leg_peaks(b)
    lay = P.LAYOUT
    cases = []
    for name, pose in (('high point', P.waypoints(b)['hi']), ('low point', P.waypoints(b)['lo'])):
        q2, q3 = math.radians(pose['q2']), math.radians(pose['q3'])
        R = (P.pts(pose['q2'], pose['q3'])['fl'][0]) / 1000.0
        cen = lp['w1'] ** 2 * R
        for j in (1, 2, 3, 4):
            for sg in (1, -1):
                if j == 1:
                    a, az = (-cen, sg * acc[1] * R, 0.0), sg * acc[1]
                elif j == 2:
                    L2 = lay['L2'] / 1000
                    a, az = (sg * acc[2] * L2 * math.cos(q2), 0.0, -sg * acc[2] * L2 * math.sin(q2)), 0.0
                elif j == 3:
                    L3 = lay['L3'] / 1000
                    a, az = (-sg * acc[3] * L3 * math.sin(q3), 0.0, sg * acc[3] * L3 * math.cos(q3)), 0.0
                else:
                    a, az = (0.0, 0.0, 0.0), sg * acc[4]
                rows = [interface(a, az, 2 * pi * k / 8, 0.0, b, depth) for k in range(8)]
                cases.append(dict(pose=name, joint=j, sense=sg, a=a, alpha=az,
                                  Fa=max(r['Fa'] for r in rows), V=max(r['V'] for r in rows),
                                  M=max(r['M'] for r in rows), T=max(r['T'] for r in rows)))
    return dict(cases=cases, acc=acc,
                worst={k: max(cases, key=lambda c: c[k]) for k in ('Fa', 'V', 'M', 'T')})


# ------------------------------------------------------------------------------------------------------------------
# The bolted joint: the changer's master on the robot's plate, n cap screws on d1 into tapped holes
# (Budynas & Nisbett, 2015, Eqs. 8-17, 8-22, 8-27, 8-29, 8-30, 8-31, 8-38; Tables 8-7, 8-15)
# ------------------------------------------------------------------------------------------------------------------
def bolt_joint(d1, h_mm, cls='10.9', loads=None, estop=None, mu=MU):
    """The joint for ISO size d1 with the master's flange h_mm thick: stiffnesses, preload, and on the cycle and in
    the stops the worst screw's load factor, separation factor and fatigue factor, and the friction's factor against
    slip under torque and shear together."""
    iso = ISO9409[d1]
    d, p, At = THREAD[iso['d4']]
    c = CLASS[cls]
    n = iso['n']
    r = d1 / 2.0 / 1000.0
    # the grip of a cap screw in a tapped hole: l' = h + d/2 for a thread engaged at least d deep (Table 8-7)
    lg = h_mm + d / 2.0
    Ad = pi * d * d / 4.0
    # the screw's thread length in the grip: a cap screw threaded 2d + 6 mm (Table 8-7, d <= 125 mm, L <= 125)
    L_total = h_mm + 1.5 * d
    LT = 2 * d + 6.0
    ld = max(L_total - LT, 0.0)
    lt = lg - ld
    kb = Ad * At * E_STEEL / 1e6 / (Ad * lt + At * ld) * 1e3          # N/m; Eq. 8-17 in mm and N/mm2 made N/m
    km = 0.5774 * pi * E_STEEL * d / 1000.0 / (2 * math.log(5 * (0.5774 * lg + 0.5 * d) / (0.5774 * lg + 2.5 * d)))
    C = kb / (kb + km)
    Fp = At * c['Sp']
    Fi = 0.75 * Fp
    out = dict(d1=d1, thread=iso['d4'], n=n, r=r, At=At, lg=lg, lt=lt, ld=ld, kb=kb, km=km, C=C, Fp=Fp, Fi=Fi, cls=cls)

    def worst_screw(Fa, M):
        return Fa / n + 2 * M / (n * r)
    if estop:
        w = estop['worst']
        P_ = max(worst_screw(cs['Fa'], cs['M']) for cs in estop['cases'])
        out['P_estop'] = P_
        out['nL_estop'] = (c['Sp'] * At - Fi) / (C * P_)
        out['n0_estop'] = Fi / (P_ * (1 - C))
        clamp = n * Fi - (1 - C) * max(cs['Fa'] for cs in estop['cases'])
        T_cap, V_cap = mu * clamp * r, mu * clamp
        out['slip_estop'] = min(1.0 / (cs['T'] / T_cap + cs['V'] / V_cap) for cs in estop['cases'])
        out['T_cap'], out['V_cap'] = T_cap, V_cap
    if loads:
        # each screw's load along the cycle, the moment's direction turning with the flange: the worst range
        Pmax = max(worst_screw(x['Fa'], x['M']) for x in loads)
        Pmin = min(x['Fa'] / n - 2 * x['M'] / (n * r) for x in loads)
        sa = C * (Pmax - Pmin) / (2 * At)
        sm = C * (Pmax + Pmin) / (2 * At) + Fi / At
        si = Fi / At
        nf = c['Se'] * (c['Sut'] - si) / (c['Sut'] * sa + c['Se'] * (sm - si))
        out.update(P_cycle=Pmax, Pmin_cycle=Pmin, sa=sa, sm=sm, si=si, nf=nf,
                   nL_cycle=(c['Sp'] * At - Fi) / (C * Pmax), n0_cycle=Fi / (Pmax * (1 - C)))
    return out


# ------------------------------------------------------------------------------------------------------------------
# The ball lock: a piston drives a cam of angle alpha (to the axis); n balls pushed out by it press on the tool
# plate's ring, a face at beta to the radial plane. With friction mu at both contacts the clamp is
# F_c = F_p / (tan(alpha + phi) tan(beta + phi)), phi = atan mu; the cam holds itself if alpha < atan mu_min.
# ------------------------------------------------------------------------------------------------------------------
def lock_clamp(Fp, alpha, beta, mu=MU):
    ph = math.atan(mu)
    return Fp / (math.tan(math.radians(alpha) + ph) * math.tan(math.radians(beta) + ph))


def lock_ball_forces(Fc, n, alpha, beta, mu=MU):
    """Each ball's normal forces at the ring (N2) and at the cam (N1) for a clamp F_c, and the ring's radial load."""
    a, bt = math.radians(alpha), math.radians(beta)
    N2 = Fc / n / (math.cos(bt) - mu * math.sin(bt))
    N1 = N2 * (math.sin(bt) + mu * math.cos(bt)) / (math.cos(a) - mu * math.sin(a))
    return dict(N1=N1, N2=N2, radial=N2 * (math.sin(bt) + mu * math.cos(bt)))


def face_clamp_needed(Fa, M, Ri, Ro):
    """The clamp that keeps an annular face from opening under a tension Fa and a moment M: the face's pressure,
    Fc/A - Fa/A - M Ro/I, stays positive at the outer edge (Budynas & Nisbett, 2015, Eqs. 3-22 and 3-24, pp. 103-104:
    the axial and the bending stress added, here on the contact's area)."""
    A = pi * (Ro * Ro - Ri * Ri)
    I = pi / 4 * (Ro ** 4 - Ri ** 4)
    return Fa + M * Ro * A / I, A, I


def hertz_ball_plane(F, d_ball, E=E_STEEL, nu=NU_STEEL):
    """A ball pressed on a plane: the contact's radius and the largest pressure (Eqs. 3-68, 3-69, p. 137, d2 = inf)."""
    k = 2 * (1 - nu * nu) / E
    a = (3 * F / 8 * k / (1 / d_ball)) ** (1.0 / 3.0)
    return a, 3 * F / (2 * pi * a * a)


def surface_endurance(HB):
    """Steel's surface endurance strength at 10^8 cycles, (S_C) = 2.76 HB - 70 MPa (Eq. 6-68, p. 337)."""
    return (2.76 * HB - 70.0) * 1e6


# the changer as designed: the robot side ISO 9409-1-160-6-M10, the tool side the same; six segments driven out by
# the piston's cam into the tool plate's ring
CHANGER = dict(iso=160, h_flange=20.0, cls='10.9', p_min=0.55e6, D_piston=0.100, spring=0.0,
               n_seg=6, seg_w=0.036, seg_h=0.016, travel=0.003, alpha=5.0, alpha_free=30.0, beta=15.0,
               face_Ri=0.090, face_Ro=0.100, depth=0.070, balls=dict(n=12, d=0.0127), HB=600.0)
# the lock's geometry (mm, the robot's face at z = 0, z up): the master's cover from 0 to -7, its body from -7 to -70
# with its spigot of radius 65 down to -95; the piston's bore -51 to -7 above the gland (-55 to -51) the rod passes;
# the piston's face at -47 locked, 20 higher free; the segments at -92 to -76, out to 68 locked, in to 65 free; the
# tool plate's bore 65.25 with the lip's 15 deg face from r 65.25 to 68; the tool plate down to -112; the two air
# ports (release under the piston, lock over it) radial, 3 mm, at 15 and 195 deg
GEOM = dict(R_spigot=65.0, R_bore=65.25, R_seg_out=68.0, R_seg_in=50.0, z_seg=(-92.0, -76.0), z_chamfer=-80.0,
            R_groove=70.0, z_face=-70.0, z_spigot=-95.0, z_tool=-112.0, z_web=(-102.0, -95.0), R_piston=50.0,
            z_bore=(-51.0, -7.0), z_gland=(-55.0, -51.0), z_piston=-47.0, z_cover=(-7.0, 0.0), piston_h=15.0,
            R_rod=20.0, stroke=10.0, ring_w=0.015, port_d=3.0, z_ports=(-49.0, -10.0), a_ports=(15.0, 195.0))
GEOM['contact_w'] = (GEOM['R_seg_out'] - GEOM['R_bore']) / math.cos(math.radians(15.0))
GEOM['R_contact'] = (GEOM['R_seg_out'] + GEOM['R_bore']) / 2


def design(c=CHANGER, b=None):
    b = b or P.brief()
    cyc = cycle_loads(depth=0.0, b=b)
    cyc_i = cycle_loads(depth=c['depth'], b=b)       # at the changer's own face, its depth below the robot's plate
    es = estop_loads(depth=0.0, b=b)
    es_i = estop_loads(depth=c['depth'], b=b)
    out = dict(cycle=cyc, cycle_i=cyc_i, estop=es, estop_i=es_i)
    out['peaks'] = {k: max(x[k] for x in cyc) for k in ('Fa', 'V', 'M', 'T')}
    out['peaks_i'] = {k: max(x[k] for x in cyc_i) for k in ('Fa', 'V', 'M', 'T')}
    out['bolts'] = {d1: bolt_joint(d1, c['h_flange'] if ISO9409[d1]['d4'] != 'M8' else 16.0, c['cls'], cyc, es)
                    for d1 in (100, 125, 160, 200)}
    # the clamp the faces need: not opening on the cycle and in the stops, at the changer's face
    need_cyc = max(face_clamp_needed(x['Fa'], x['M'], c['face_Ri'], c['face_Ro'])[0] for x in cyc_i)
    need_es = max(face_clamp_needed(cs['Fa'], cs['M'], c['face_Ri'], c['face_Ro'])[0] for cs in es_i['cases'])
    # balls first: twelve 12.7 mm balls on the ring's 30 deg face carrying 1.5 times the cycle's need
    bl = c['balls']
    N2b = lock_ball_forces(1.5 * need_cyc, bl['n'], c['alpha'], 30.0)['N2']
    ball = dict(N2=N2b, hertz=hertz_ball_plane(N2b, bl['d']), Sc=surface_endurance(c['HB']))
    # the segments
    Fp = pi / 4 * c['D_piston'] ** 2 * c['p_min']
    Fc = lock_clamp(Fp + c['spring'], c['alpha'], c['beta'])
    sf = lock_ball_forces(Fc, c['n_seg'], c['alpha'], c['beta'])          # the same wedge, per segment
    gm = GEOM
    A_face = c['seg_w'] * gm['contact_w'] / 1000          # a straight segment: its width times the face's
    A_cam = c['seg_w'] * c['seg_h']
    radial_total = c['n_seg'] * sf['radial']
    # the tool plate's ring round the groove: from the groove's wall out to the face ring, as high as the groove;
    # its lip, above the segments, sheared on the bore's circle through its height over the chamfer
    rg = dict(w=c['face_Ro'] - gm['R_groove'] / 1000, h=(gm['z_chamfer'] - gm['z_seg'][0]) / 1000,
              lip=(gm['z_face'] - gm['z_chamfer']) / 1000, D_lip=2 * gm['R_bore'] / 1000)
    out['lock'] = dict(Fp=Fp, Fs=c['spring'], Fc=Fc, need_cycle=need_cyc, need_estop=need_es, n_open=Fc / need_cyc,
                       n_open_estop=Fc / need_es, phi=math.degrees(math.atan(MU)), phi_min=math.degrees(math.atan(MU_MIN)),
                       self_locking=c['alpha'] < math.degrees(math.atan(MU_MIN)), seg=sf, ball=ball,
                       p_face=sf['N2'] / A_face, p_cam=sf['N1'] / A_cam, A_face=A_face, A_cam=A_cam,
                       radial_total=radial_total, ring_T=radial_total / (2 * pi),
                       ring_stress=radial_total / (2 * pi) / (rg['w'] * rg['h']), ring=rg,
                       lip_shear=Fc / (pi * rg['D_lip'] * rg['lip']),
                       advantage=Fc / (Fp + c['spring']), clamp_spring_only=lock_clamp(c['spring'], c['alpha'], c['beta']),
                       take_up=c['travel'] * math.tan(math.radians(c['beta'])),
                       stroke=c['travel'] / math.tan(math.radians(c['alpha_free'])))
    # the friction's low end locks harder: the same piston at mu_min, and the stresses there
    Fc_hi = lock_clamp(Fp + c['spring'], c['alpha'], c['beta'], MU_MIN)
    sf_hi = lock_ball_forces(Fc_hi, c['n_seg'], c['alpha'], c['beta'], MU_MIN)
    rad_hi = c['n_seg'] * sf_hi['radial']
    out['lock_hi'] = dict(Fc=Fc_hi, seg=sf_hi, p_face=sf_hi['N2'] / A_face, p_cam=sf_hi['N1'] / A_cam,
                          ring_stress=rad_hi / (2 * pi) / (rg['w'] * rg['h']), lip_shear=Fc_hi / (pi * rg['D_lip'] * rg['lip']))
    # the release: the cam holds itself, so air under the piston must pull it up against the segments' friction.
    # On the cam each segment pushes N1 (sin alpha - mu cos alpha) along the axis, a pull when mu > tan alpha: the
    # wedge stored at mu_min (the largest N1), let go at mu
    a = math.radians(c['alpha'])
    A_rel = pi * ((gm['R_piston'] / 1000) ** 2 - (gm['R_rod'] / 1000) ** 2)
    F_rel = A_rel * c['p_min']
    pull = {m: c['n_seg'] * sf_hi['N1'] * (m * math.cos(a) - math.sin(a)) for m in (MU, 0.20)}
    out['release'] = dict(A=A_rel, F=F_rel, pull=pull, n={m: F_rel / v for m, v in pull.items()})
    # the air one change uses: the lock chamber full (piston down, from its top to the cover) and the release chamber
    # full (piston up, from the gland to its face), each at the supply's pressure, as free air at 1 atm
    st = 2 * gm['stroke']
    V_lock = pi * gm['R_piston'] ** 2 * (gm['z_cover'][0] - (gm['z_piston'] + gm['piston_h'])) * 1e-9
    V_rel = pi * (gm['R_piston'] ** 2 - gm['R_rod'] ** 2) * (gm['z_piston'] + st - gm['z_gland'][1]) * 1e-9
    ratio = (c['p_min'] + 101325.0) / 101325.0
    out['air'] = dict(V_lock=V_lock, V_rel=V_rel, ratio=ratio, free=(V_lock + V_rel) * ratio)
    return out


if __name__ == '__main__':
    d = design()
    print('peaks at the face', {k: round(v, 1) for k, v in d['peaks'].items()})
    print('peaks at the changer face', {k: round(v, 1) for k, v in d['peaks_i'].items()})
    print('estop worst', {k: (round(v[k], 1), v['pose'], v['joint']) for k, v in d['estop']['worst'].items()})
    for d1, bj in d['bolts'].items():
        print(d1, {k: (round(v, 3) if isinstance(v, float) else v) for k, v in bj.items()})
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in d['lock'].items() if not isinstance(v, (dict, tuple))})
    print(d['lock']['seg'], d['lock']['ball'])
