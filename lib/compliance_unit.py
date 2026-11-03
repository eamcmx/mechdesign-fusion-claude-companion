# -*- coding: utf-8 -*-
"""compliance_unit.py: Chapter 48 of Volume 2, the compliance unit: a vertical float between the tool plate and a
gripper, so that the gripper gives upward when its load touches the stack and a sensor stops the robot before the arm
feels the stack.

The carriage that carries the gripper hangs on stops under the unit's body, pressed down onto them by springs, and
slides up on two HIWIN rails with two blocks each when the load below it is pushed up. First the concepts: a ball
breakaway is weighed against the preload it would need, and the float's height against the changer's margin with
each gripper. Then the float for the vacuum and bag grippers: the springs against the stops that throw the load up,
the guides against the tipping moment, the sensor and the robot's stop within the remaining travel, and the
envelope with the unit in it.
"""
import math

import clamp_gripper as cg
import vacuum_gripper as vg
import bag_gripper as bg
import tool_changer as tc
import palletizer_design as P

G = 9.81
pi = math.pi
S_STOP = 1.25
CHANGER = ('Master', 'Piston', 'Tool plate', 'Segments')

# the float's layout (mm; z = 0 the unit's top face on the tool plate, z up): a top plate with two cheeks at x = +-c/2,
# an HGR25 rail on each cheek's inner face, two HGH25CA blocks on each rail d apart; the carriage hangs on stops at the
# cheeks' feet; its underside is the gripper's mounting face, H below the unit's top
LAY = dict(top_t=20.0, plate=(260.0, 200.0), cheek_x=(105.0, 125.0), cheek_y=60.0, c=166.0, d=100.0, stroke=30.0,
           gap=2.0, carriage_t=20.0, wing_x=(50.0, 65.0), wing_y=60.0, sense=3.0,
           hangers=((25.0, 50.0), (-25.0, 50.0), (-25.0, -50.0), (25.0, -50.0)), hanger_d=12.0, head=(20.0, 8.0),
           boss=(25.0, 6.5), boss_bottom=-138.0)
# the rails' axes: the cheeks' inner faces at x = +-105 carry the rails (HR 22 high), so the rails' running faces
# stand at +-83 and the blocks' tops, H = 40 over the rail's foot, at +-65 where the carriage's wings carry them; the
# rails c = 166 apart between their running faces
GUIDE = dict(name='HGH25CA', C0=36490.0, Cdyn=26480.0, L=84.0, H=40.0, W=48.0, rail='HGR25R', WR=23.0, HR=22.0,
             fSL_min=3.0)       # HIWIN GmbH (2017), Table 3.4 p. 30, Table 3.6 p. 32, Table 2.1 p. 10 (shocks: 3 to 5)
SPRING = dict(n=4, d=3.0, D=24.0, A=2211.0, m=0.145, G=81.7e3, ssy=0.45)   # music wire A228 (Shigley Tables 10-4 to 10-6)


def height(lay=LAY, g=GUIDE):
    """The unit's height from its top to the gripper's face: the top plate, the rail's length (the blocks d apart,
    a block's length and the stroke), the carriage's plate."""
    rail = lay['d'] + g['L'] + lay['stroke'] + 2 * lay['gap']
    return dict(rail=rail, H=lay['top_t'] + rail + lay['carriage_t'])


def rows_with_unit(env, H, m_unit=12.0):
    """A gripper's envelope rows with the unit inserted under the changer: the gripper and its load H lower, the unit's
    mass at its middle."""
    rows = [(n, m, (z + H / 1000.0) if n not in CHANGER else z) for n, m, z in env['rows']]
    if H > 0:
        rows.append(('unit', m_unit, -tc.GEOM['z_tool'] / 1000.0 + H / 2000.0))
    return rows


def ring_factor(env, H, m_unit=12.0):
    """Chapter 44's ring kept closed in the worst stop with the unit H mm tall in the stack."""
    rows = rows_with_unit(env, H, m_unit)
    M = sum(r[1] for r in rows)
    mz = sum(r[1] * r[2] for r in rows)
    b = dict(P.brief())
    b['payload'] = M
    b['drop'] = mz / M * 1000.0
    b['layer'] = (math.sqrt(12 * env['I'] / M) * 1000.0, 0.0)
    c = tc.CHANGER
    es = tc.estop_loads(depth=c['depth'], b=b, decel=vg.stop_decels(M))
    need = max(tc.face_clamp_needed(cs['Fa'], cs['M'], c['face_Ri'], c['face_Ro'])[0] for cs in es['cases'])
    return dict(m=M, mz=mz, n=tc.design()['lock']['Fc'] / need)


def envelopes():
    return dict(clamp=cg.envelope(), vacuum=vg.envelope(), bag=bg.envelope())


def below(env, H, m_lower=8.0):
    """What hangs on the unit's carriage: the gripper and its load, H lower, and the carriage itself."""
    rows = [(n, m, z + H / 1000.0) for n, m, z in env['rows'] if n not in CHANGER]
    rows.append(('carriage', m_lower, -tc.GEOM['z_tool'] / 1000.0 + (H - LAY['carriage_t'] / 2) / 1000.0))
    m = sum(r[1] for r in rows)
    return dict(rows=rows, m=m, z=sum(r[1] * r[2] for r in rows) / m)


def stop_cases(env, H):
    """Every brake stop for the payload with the unit: the carriage's load's vertical acceleration (with g) and the
    horizontal, and the loads at the unit's top plane from Chapter 44's model with the mass below the carriage."""
    rows = rows_with_unit(env, H)
    M = sum(r[1] for r in rows)
    bl = below(env, H)
    b = dict(P.brief())
    b['payload'] = bl['m']
    b['drop'] = bl['z'] * 1000.0
    b['layer'] = (math.sqrt(12 * env['I'] / bl['m']) * 1000.0, 0.0)
    es = tc.estop_loads(depth=-tc.GEOM['z_tool'] / 1000.0, b=b, decel=vg.stop_decels(M))
    out = []
    for c in es['cases']:
        a = c['a']
        out.append(dict(pose=c['pose'], joint=c['joint'], sense=c['sense'], a_v=a[2] + G, a_h=math.hypot(a[0], a[1]),
                        Fa=c['Fa'], V=c['V'], M=c['M'], T=c['T']))
    return dict(cases=out, m=bl['m'], z=bl['z'], payload=M)


def breakaway(env, r=0.085, S=S_STOP):
    """A three-ball breakaway under the changer: the preload that keeps the balls seated in every stop at S, tipping
    about the line through two balls (its preload's arm r/2), with the axial pull added."""
    sc = stop_cases(env, 60.0)
    need = max(S * c['M'] / (r / 2) + c['Fa'] for c in sc['cases'])
    return dict(r=r, F=need, M=max(c['M'] for c in sc['cases']))


def largest_coil(d=12.7, C=6.0, A=1783.0, m=0.190, ssy=0.45):
    """The most a helical spring of Shigley's wires carries: hard-drawn A227 at its largest diameter, 12.7 mm (Table
    10-4, p. 517; Ssy = 0.45 Sut, Table 10-6), wound to an index of 6: F = Ssy pi d^3/(8 K_B D)."""
    Sut = A / d ** m
    KB = (4 * C + 2) / (4 * C - 3)
    F = ssy * Sut * pi * d ** 3 / (8 * KB * C * d)
    return dict(d=d, D=C * d, Sut=Sut, KB=KB, F=F)


def springs(m_below, a_lift, s=SPRING, lay=LAY):
    """The springs that keep the carriage on its stops when a stop throws the load up: their preload, at S, at least
    the load's mass times the lift; the touch-down force; the spring of Shigley's chapter 10 for it."""
    F_pre = S_STOP * m_below * max(0.0, -a_lift)
    Fi = F_pre / s['n']
    W = m_below * G
    k_tot = None
    return dict(F_pre=F_pre, F_each=Fi, W=W, touch=W + F_pre)


def choose_coil(F1, stroke, wires=(3.5, 4.0, 4.5, 5.0, 5.5, 6.0), means=(24.0, 28.0, 32.0, 36.0, 40.0),
                lengths=(60.0, 70.0, 80.0, 90.0, 100.0), n_min=1.2):
    """The lightest spring that gives F1 installed and survives the stroke: music wire (G 80.0 GPa above 3.18 mm,
    Table 10-5, p. 518; E 193 GPa), closed and ground ends between flat seats (alpha 0.5, Table 10-2), stable by
    Eq. 10-12 (p. 514), at least 15 % over its solid length at the end of the stroke, a static factor of n_min; for each
    wire and coil the fewest active turns the solid length allows, which is the stiffest spring that still fits."""
    best = None
    E, Gs = 193e3, 80.0e3
    for d in wires:
        for D in means:
            C = D / d
            if not 4.0 <= C <= 12.0:
                continue
            for L1 in lengths:
                L2 = L1 - stroke
                Na = L2 / 1.15 / d - 2                      # the most turns the overrun allows
                if Na < 3:
                    continue
                k = d ** 4 * Gs / (8 * D ** 3 * Na)
                F2 = F1 + k * stroke
                L0 = L1 + F1 / k
                stable = L0 < math.pi * D / 0.5 * math.sqrt(2 * (E - Gs) / (2 * Gs + E))
                KB = (4 * C + 2) / (4 * C - 3)
                tau = KB * 8 * F2 * D / (pi * d ** 3)
                Sut = 2211.0 / d ** 0.145
                n = 0.45 * Sut / tau
                if stable and n >= n_min:
                    m = 7850.0 * pi * D * (Na + 2) * pi * d * d / 4 * 1e-9
                    cand = dict(d=d, D=D, C=C, L1=L1, L2=L2, Na=Na, k=k, F1=F1, F2=F2, L0=L0, KB=KB, tau=tau, Sut=Sut,
                                Ssy=0.45 * Sut, n=n, Ls=(Na + 2) * d, mass=m, L0_max=math.pi * D / 0.5 * math.sqrt(2 * (E - Gs) / (2 * Gs + E)))
                    if best is None or (cand['F2'], cand['mass']) < (best['F2'], best['mass']):
                        best = cand
    return best


def coil(F1, F2, stroke, s=SPRING):
    """A compression spring from F1 installed to F2 at the end of the stroke: rate, active turns, stress, strength,
    factor, solid length and the overrun (Budynas & Nisbett, 2015, Eqs. 10-1, 10-5, 10-7, 10-9, 10-14; Tables 10-1,
    10-4 to 10-6)."""
    k = (F2 - F1) / stroke
    Na = s['d'] ** 4 * s['G'] / (8 * s['D'] ** 3 * k)
    C = s['D'] / s['d']
    KB = (4 * C + 2) / (4 * C - 3)
    tau = KB * 8 * F2 * s['D'] / (pi * s['d'] ** 3)
    Sut = s['A'] / s['d'] ** s['m']
    Ssy = s['ssy'] * Sut
    Ls = (Na + 2) * s['d']
    return dict(k=k, Na=Na, C=C, KB=KB, tau=tau, Sut=Sut, Ssy=Ssy, n=Ssy / tau, Ls=Ls)


def guides(case, lay=LAY, g=GUIDE):
    """The four blocks against a stop's loads at the unit's top plane: a shear V at the carriage, the moment M, the
    torque T; HIWIN's Table 2.3 for a force at a lever across the rails and along them. The worst block, its static
    factor."""
    c, d = lay['c'] / 1000.0, lay['d'] / 1000.0
    # the moment in the plane of the rails is carried as opposite radial forces on the blocks d apart; across it,
    # by the blocks' lateral forces d apart; the torque as lateral forces c apart; the shear shared by four
    P_M = case['M'] / (2 * d)
    P_T = case['T'] / (2 * c)
    P_V = case['V'] / 4
    P = P_M + P_T + P_V
    return dict(P_M=P_M, P_T=P_T, P_V=P_V, P=P, fSL=g['C0'] / P)


def touchdown(lay=LAY, a_stop=None, t_react=0.010, early=None):
    """Touching a stack higher than expected on the cycle's last vertical leg: the speed of the leg's cycloid where
    the stroke's remaining travel begins, and the distance the robot needs to stop from it at a_stop after the
    sensor's reaction (assumed 10 ms)."""
    b = P.brief()
    S = b['up'] / 1000.0
    T = b['t_v']
    rem = (lay['stroke'] - lay['sense']) / 1000.0
    out = []
    for early in ((early,) if early is not None else (0.010, 0.020, 0.030, 0.050, 0.100)):
        # the load meets the stack `early` metres before the leg's end: the leg's speed there
        lo, hi = 0.0, T
        for _ in range(60):
            t = 0.5 * (lo + hi)
            s = S * (t / T - math.sin(2 * pi * t / T) / (2 * pi))
            lo, hi = (t, hi) if s < S - early else (lo, t)
        v = S / T * (1 - math.cos(2 * pi * t / T))
        stop = v * t_react + v * v / (2 * a_stop)
        out.append(dict(early=early, v=v, stop=stop, ok=stop <= rem))
    return dict(rem=rem, rows=out, v_peak=2 * S / T)


def structure(g, sp, lay=LAY):
    """The housing's cheek at its root under its rail's couple, against 2024-T3 (Sy 345 MPa, Budynas & Nisbett, 2015,
    Table A-24, p. 1055); the hanger bolts in the stop, the axial pull with the springs' preload over four, on the
    shank and on an M10's stress area (Table 8-1, p. 404); the springs in fatigue if every placement used the whole
    stroke, unpeened, Zimmerli's Ssa 241 and Ssm 379 MPa (Eq. 10-28, p. 528), Ssu = 0.67 Sut (Eq. 10-30), Gerber."""
    w = g['worst']
    M_ch = w['M'] / 2
    b, t = 2 * lay['cheek_y'], lay['cheek_x'][1] - lay['cheek_x'][0]
    W = b * t * t / 6.0
    s_ch = M_ch * 1000.0 / W
    F_h = (w['Fa'] + g['springs']['F_pre']) / 4
    A_sh = pi * lay['hanger_d'] ** 2 / 4
    # the springs in fatigue
    KB, D, d = sp['KB'], sp['D'], sp['d']
    t1 = KB * 8 * sp['F1'] * D / (pi * d ** 3)
    t2 = KB * 8 * sp['F2'] * D / (pi * d ** 3)
    ta, tm = (t2 - t1) / 2, (t2 + t1) / 2
    Ssu = 0.67 * sp['Sut']
    Sse = 241.0 / (1 - (379.0 / Ssu) ** 2)
    nf = 0.5 * (Ssu / tm) ** 2 * (ta / Sse) * (-1 + math.sqrt(1 + (2 * tm * Sse / (Ssu * ta)) ** 2))
    return dict(M_cheek=M_ch, W=W, s_cheek=s_ch, n_cheek=345.0 / s_ch, F_hanger=F_h, s_shank=F_h / A_sh,
                s_thread=F_h / 58.0, t1=t1, t2=t2, ta=ta, tm=tm, Ssu=Ssu, Sse=Sse, nf=nf)


def spacing(ds=(60.0, 80.0, 100.0, 120.0, 140.0)):
    """The blocks' spacing along a rail against the unit's height, the worst block's static factor with the bag gripper,
    and the changer's ring factor with the bag and vacuum grippers."""
    envs = envelopes()
    rows = []
    for d in ds:
        lay = dict(LAY, d=d)
        H = height(lay)['H']
        sc = stop_cases(envs['bag'], H)
        w = max(sc['cases'], key=lambda c: guides(c, lay)['P'])
        g = guides(w, lay)
        rows.append(dict(d=d, H=H, P=g['P'], fSL=g['fSL'], ring_bag=ring_factor(envs['bag'], H)['n'],
                         ring_vac=ring_factor(envs['vacuum'], H)['n']))
    return rows


def design():
    envs = envelopes()
    hz = height()
    out = dict(height=hz)
    out['ring'] = {k: {H: ring_factor(e, H)['n'] for H in (0.0, 100.0, 150.0, 200.0, hz['H'])} for k, e in envs.items()}
    out['breakaway'] = {k: breakaway(e) for k, e in envs.items()}
    out['largest_coil'] = largest_coil()
    out['grippers'] = {}
    for k in ('vacuum', 'bag'):
        sc = stop_cases(envs[k], hz['H'])
        lift = min(sc['cases'], key=lambda c: c['a_v'])
        worst = max(sc['cases'], key=lambda c: guides(c)['P'])
        sp = springs(sc['m'], lift['a_v'])
        out['grippers'][k] = dict(m=sc['m'], payload=sc['payload'], lift=lift, springs=sp, worst=worst, guides=guides(worst))
    F_pre = max(v['springs']['F_pre'] for v in out['grippers'].values())
    out['F_pre'] = F_pre
    out['coil'] = choose_coil(F_pre / SPRING['n'], LAY['stroke'])
    # the robot's stop on the vertical leg: J3's brake at the low point for the heavier float payload, the wrist's
    # vertical deceleration alpha L3 cos q3 (J2's brake would add to it, so J3 alone is the slower stop)
    pl = out['grippers']['bag']['payload']
    q3 = math.radians(P.waypoints()['lo']['q3'])
    a3 = vg.stop_decels(pl)[3] * P.LAYOUT['L3'] / 1000.0 * math.cos(q3)
    out['a_stop'] = a3
    out['touchdown'] = touchdown(a_stop=a3)
    td = out['touchdown']
    lo, hi = 0.0, 0.2
    for _ in range(60):
        e = 0.5 * (lo + hi)
        lo, hi = (e, hi) if touchdown(a_stop=a3, early=e)['rows'][0]['ok'] else (lo, e)
    td['limit'] = lo
    for k, g in out['grippers'].items():
        g['structure'] = structure(g, out['coil'])
    v_lim = touchdown(a_stop=a3, early=lo)['rows'][0]['v']
    m_bag = out['grippers']['bag']['m']
    out['impact'] = dict(v=v_lim, m=m_bag, E=0.5 * m_bag * v_lim ** 2,
                         v30=touchdown(a_stop=a3, early=0.030)['rows'][0]['v'],
                         E30=0.5 * m_bag * touchdown(a_stop=a3, early=0.030)['rows'][0]['v'] ** 2)
    return out


if __name__ == '__main__':
    d = design()
    print('height', d['height'])
    for k, v in d['ring'].items():
        print('ring', k, {h: round(n, 3) for h, n in v.items()})
    for k, v in d['breakaway'].items():
        print('breakaway', k, {a: round(b, 1) for a, b in v.items()})
    for k, v in d['grippers'].items():
        print(k, 'm %.2f payload %.2f' % (v['m'], v['payload']), 'lift', v['lift']['pose'], v['lift']['joint'], round(v['lift']['a_v'], 3),
              'springs', {a: round(b, 1) for a, b in v['springs'].items()},
              'worst', v['worst']['pose'], v['worst']['joint'], {a: round(v['worst'][a], 1) for a in ('M', 'V', 'T', 'Fa')},
              'guides', {a: round(b, 2) for a, b in v['guides'].items()})
    print('F_pre', round(d['F_pre'], 1), 'coil', {a: round(b, 3) for a, b in d['coil'].items()})
    print('touchdown', d['touchdown'])
