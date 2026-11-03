"""Chapter 55: the palletizer in detail. The robot's rod eyes against the rods' forces, every rolling bearing held
tight on both rings checked against its internal clearance, the eyes and pins that locate the rings; the bag
gripper's 6206 on its shaft class and clearance, its jaws' locating and floating ends, the plug welded into the jaw's
tube; the compliance unit's O-ring gland, its tube's two zones and its gallery plugs; the vacuum gripper's plate
joint; the HIWIN guideways' preload and accuracy classes.

Sources (the pages saved in References/Web pages, 3 November 2026):
  SKF product pages 22207 E, 22213 E: dimensions, abutments, initial clearance (SKF_22207E_22213E_abutments_...)
  SKF, internal clearance: table 4 (deep groove ball bearings), table 5 (spherical roller bearings), the general
    rule for the operating clearance, Delta_r = Delta1 f1 + Delta2 f2 with diagram 1, Delta_r = 0.012 dT dm
    (SKF_internal_clearance_...)
  SKF, Seat tolerances for standard conditions, tables 1, 2 and 4; Tolerances and resultant fits, tables 4, 5, 10
    (SKF_bearing_seats_..., sections 2, 5, 6 and 7)
  Fasten.it tables of DIN 471, DIN 472, DIN 908, ISO 4762, ISO 4032, ISO 7089
  ERIKS (n.d.), O-ring gland design: dynamic surface finish (p. 133), Table 3.D-1 (p. 146)
  Hydro (2023), EN AW-6082: EN 755-2 minimum strengths, expansion
  HIWIN GmbH (2017), linear guideways: order codes (p. 27), preload (Table 3.3, p. 29), accuracy (pp. 40-42)
  Shigley (Budynas & Nisbett, 2015): Eq. 3-56 and the thick-cylinder stresses (p. 130), Table 3-3 (p. 131), Table 8-7
    (p. 418), Tables 9-1 (p. 476) and 9-2 (p. 480), the weldable steels (p. 481), Tables 9-3 to 9-6 (pp. 482-483),
    Table 6-2 and Eqs. 6-8, 6-18, 6-26, 6-54 (Example 9-6's fatigue method), Table A-20 (p. 1048)
Values marked 'assumed' are this chapter's, not a source's.
"""
import functools
import math

import materials_fits as MF

pi = math.pi

# ---- SKF's spherical roller bearings in the rods' eyes and under the triangle ---------------------------------------
SRB = {'22207 E': dict(d=35.0, D=72.0, B=23.0, d2=44.5, D1=61.8, da=42.0, Da=65.0, ra=1.0, C=88.8,
                       clear={'Normal': (30.0, 45.0), 'C3': (45.0, 60.0), 'C4': (60.0, 80.0)}),
       '22213 E': dict(d=65.0, D=120.0, B=31.0, d2=80.1, D1=106.0, da=74.0, Da=111.0, ra=1.5, C=198.0,
                       clear={'Normal': (40.0, 65.0), 'C3': (65.0, 90.0), 'C4': (90.0, 120.0)})}
# SKF's deep groove ball bearings held tight on both rings: table 4's rows for their bores
DGBB = {'6206': dict(d=30.0, D=62.0, B=16.0, C=20.3,
                     clear={'Normal': (5.0, 20.0), 'C3': (13.0, 28.0), 'C4': (23.0, 41.0), 'C5': (30.0, 53.0)}),
        '61836': dict(d=180.0, D=225.0, B=22.0, C=62.4,
                      clear={'Normal': (20.0, 61.0), 'C3': (53.0, 102.0), 'C4': (91.0, 147.0)}),
        '61856 MA': dict(d=280.0, D=350.0, B=33.0, C=138.0,
                         clear={'Normal': (35.0, 105.0), 'C3': (90.0, 170.0), 'C4': (155.0, 245.0)})}
# SKF's diagram 1, read off its grid (+-0.01): the reduction factors against d/D for a solid steel shaft and a
# thick-walled steel or cast iron housing
F1 = ((0.40, 0.735), (0.50, 0.80), (0.60, 0.85), (0.70, 0.90), (0.84, 0.96))
F2 = ((0.40, 0.85), (0.60, 0.905), (0.84, 0.965))
# SKF's resultant fits over 250 mm (tables 5 and 10), the probable interference (-) or clearance (+), um
PROBABLE_BIG = {('n6', 280.0): (-92.0, -43.0), ('K7', 350.0): (-27.0, 44.0)}
DT_RINGS = 5.0          # K, the inner ring over the outer in the slow joints and the jaws: assumed
T_PRESS, T_HALL = 20.0, (5.0, 40.0)   # deg C, assembled at 20, the hall from 5 to 40: assumed
ALPHA_AL, ALPHA_ST = 23.1e-6, 10.8e-6  # 1/K, Hydro (2023) for EN AW-6082; carbon steel, Shigley Table 3-3, p. 131
E_ST, NU_ST, E_AL, NU_AL = 207e3, 0.292, 70e3, 0.33   # MPa; Shigley Table A-5; Hydro's 70 GPa, Poisson's ratio assumed


def interp(table, x):
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x0 <= x <= x1:
            return y0 + (y1 - y0) * (x - x0) / (x1 - x0)
    raise ValueError('d/D = %.3f is outside SKF\'s diagram 1' % x)


def f_factors(d, D):
    return interp(F1, d / D), interp(F2, d / D)


def probable(cls, d):
    """SKF's probable interference (-) or clearance (+) of the bearing on a seat of that class, um."""
    if (cls, d) in PROBABLE_BIG:
        return PROBABLE_BIG[(cls, d)]
    return tuple(MF.skf_row(cls, d)['probable'])


# ---- the rods: the load's direction in each eye -------------------------------------------------------------------
RODS = {'Tp': ('parallel rod', 'C', 'D', '22213 E'), 'T1': ('levelling rod 1', 'E', 'F', '22207 E'),
        'T2': ('levelling rod 2', 'G', 'H', '22207 E')}


def _unwrap(a):
    out = [a[0]]
    for x in a[1:]:
        while x - out[-1] > 180.0:
            x -= 360.0
        while x - out[-1] < -180.0:
            x += 360.0
        out.append(x)
    return out


@functools.lru_cache(maxsize=1)
def rods():
    """Each rod's axial force over the cycle and the range's corners, and the largest angle between the load on an
    eye and the rod's axis: the axial force plus half the rod's weight and inertia at that end (Chapter 43 moves the
    halves to the pins), every instant of the cycle; the rod's mass."""
    import palletizer_links as L
    import palletizer_design as P
    cyc, env = L.cycle_cached(), L.envelope()
    bodies = L.chapter_bodies()
    mass = {b['name']: b['m'] for b in bodies}
    b = P.brief()
    legs, _ = P.timeline(b)
    out = {k: dict(name=v[0], ends=(v[1], v[2]), bearing=v[3], m=mass[v[0]], dev=0.0, at=None) for k, v in RODS.items()}
    for r in cyc['rows']:
        acc = L.state_at(r['t'], bodies, b, legs)[4]
        pp = L.pose_points(r['q'][1], r['q'][2])
        for k, o in out.items():
            p0, p1 = o['ends']
            u = L.unit(pp[p0], pp[p1])
            T = r['sol'][k]
            for pin, sgn in ((p0, 1.0), (p1, -1.0)):
                a = acc.get(pin, (0.0, 0.0, 0.0))
                wx, wz = -0.5 * o['m'] * a[0], -0.5 * o['m'] * (L.G + a[2])
                vx, vz = sgn * T * u[0] + wx, sgn * T * u[1] + wz
                ang = math.degrees(math.atan2(abs(vx * u[1] - vz * u[0]), sgn * (vx * u[0] + vz * u[1])))
                if ang > o['dev']:
                    o['dev'], o['at'] = ang, (pin, r['t'], T, math.hypot(wx, wz))
    for k, o in out.items():
        v = [r['sol'][k] for r in cyc['rows']]
        s = [p[3][k] for p in env['poses']]
        o.update(cycle=(min(v), max(v)), static=(min(s), max(s)), sign_changes=sum(1 for a, c in zip(v, v[1:]) if a * c < 0))
    return out


@functools.lru_cache(maxsize=1)
def joint_dirs():
    """The crank's 61836 and the lower arm's 61856 MA: the direction of each one's radial load over the cycle and the
    range's corners, in the turret's frame (the housing) and in its link's (the shaft), and the spans it sweeps."""
    import palletizer_links as L
    cyc, env = L.cycle_cached(), L.envelope()
    Y = L.Y
    sp = {'61856 MA': ('A', Y['rv2'] / 1000.0, Y['sup2'] / 1000.0, 1, lambda q2, q3: 90.0 - q2),
          '61836': ('A3', Y['sup3'] / 1000.0, Y['rv3'] / 1000.0, 0, lambda q2, q3: q3)}
    out = {}
    for nm, (j, y1, y2, k, th) in sp.items():
        res = {}
        for tag, items in (('cycle', [(r['j3'][j], r['q'][1], r['q'][2]) for r in cyc['rows']]),
                           ('range', [(jj[j], q2, q3) for _, q2, q3, _, jj in env['poses']])):
            fh, fl, mag = [], [], []
            for (F, M), q2, q3 in items:
                R = L.split(F, M, y1, y2)[k]
                phi = math.degrees(math.atan2(R[1], R[0]))
                fh.append(phi)
                fl.append(phi - th(q2, q3))
                mag.append(math.hypot(*R))
            fh, fl = _unwrap(fh), _unwrap(fl)
            res[tag] = dict(housing=max(fh) - min(fh), shaft=max(fl) - min(fl), R=(min(mag), max(mag)))
        out[nm] = res
    return out


# ---- every robot bearing held tight on both rings, against its clearance -------------------------------------------
def clearance(designation, table, shaft_cls, housing_cls, dT=DT_RINGS, f2=None, d2_extra=0.0):
    """SKF's operating clearance from the initial: the fit's reduction Delta1 f1 + Delta2 f2 with the maximum probable
    interference of each seat (a clearance counts zero), and 0.012 dT dm for the rings' temperatures; the least
    clearance left in each class, um. d2_extra adds diametral interference at the outer ring (an aluminium housing
    in the cold); f2 replaces SKF's factor (an aluminium housing)."""
    b = table[designation]
    d, D = b['d'], b['D']
    f1, f2s = f_factors(d, D)
    f2 = f2s if f2 is None else f2
    d1 = max(0.0, -probable(shaft_cls, d)[0])
    d2 = max(0.0, -probable(housing_cls, D)[0] + d2_extra) if housing_cls else 0.0
    fit = d1 * f1 + d2 * f2
    temp = 0.012 * dT * (d + D) / 2.0
    left = {c: lo - fit - temp for c, (lo, hi) in b['clear'].items()}
    return dict(designation=designation, d=d, D=D, shaft=shaft_cls, housing=housing_cls, d1=d1, d2=d2, f1=f1, f2=f2,
                fit=fit, temp=temp, red=fit + temp, left=left)


def least_class(c, need=0.0):
    """The first clearance class, in SKF's order, whose least operating clearance is at least `need` um."""
    for k in ('Normal', 'C3', 'C4', 'C5'):
        if k in c['left'] and c['left'][k] >= need:
            return k
    return None


def load_class(P, C):
    return 'light' if P <= 0.05 * C else ('normal' if P <= 0.1 * C else 'heavy')


def robot_bearings():
    """The robot's rolling bearings that both links hold: the rods' six eyes (H7, the load staying along the rod),
    the triangle's 22207 E, the crank's 61836 and the lower arm's 61856 MA (K7, the load going round); the shaft's
    class from the peak load's class (SKF table 2 for the roller bearings, table 1 for the ball bearings), the
    clearance with K7 as Chapter 51 had it and with the detail's housing, and the class each needs: at least a small
    operating clearance for the roller bearings, virtually zero (at least 0) for the ball bearings."""
    import palletizer_links as L
    cyc, env = L.cycle_cached(), L.envelope()
    rows = []
    for pin, nm in sorted(L.PIN_BEARING.items()):
        s = L.srb_life(nm, pin, cyc, env)
        br = SRB[nm]
        cls = load_class(s['Fmax'] / 1000.0, br['C'])
        # SKF table 2's heavy rows start over 50 mm and give n5 to 65 mm, as the normal row does: one class for both
        shaft = MF.bearing_seat('Bearing ' + nm, br['d'], br['D'], True, True,
                                load='light' if cls == 'light' else 'normal')['shaft']
        rod = pin != 'Bt'
        new = clearance(nm, SRB, shaft, 'H7' if rod else 'K7')
        old = clearance(nm, SRB, shaft, 'K7')
        rows.append(dict(pin=pin, bearing=nm, rod=rod, P_cycle=s['Fmax'] / 1000.0, P_static=s['static'] / 1000.0,
                         cls=load_class(s['Fmax'] / 1000.0, br['C']), shaft=shaft, housing='H7' if rod else 'K7',
                         old=old, new=new, needs=least_class(new, 1.0), needs_old=least_class(old, 1.0)))
    jb = L.joint_bearings(cyc, env)
    for nm, j, k, shaft in (('61836', 'A3', 0, 'm6'), ('61856 MA', 'A', 1, 'n6')):
        Pm = max(x[k] for x in jb[j]['series']) / 1000.0
        c = clearance(nm, DGBB, shaft, 'K7')
        rows.append(dict(pin=j, bearing=nm, rod=False, P_cycle=Pm, cls=load_class(Pm, DGBB[nm]['C']), shaft=shaft,
                         housing='K7', old=c, new=c, needs=least_class(c, 0.0), needs_old=least_class(c, 0.0)))
    return rows


# ---- the rods' eyes and pins in detail -----------------------------------------------------------------------------
# DIN 472 (Fasten.it): d1: (a max, b, d3, s, d2 groove, m groove width, n min edge distance); DIN 471 the same order
DIN472 = {62.0: (7.3, 5.5, 66.2, 2.0, 65.0, 2.15, 4.5), 72.0: (7.8, 6.4, 76.5, 2.5, 75.0, 2.65, 4.5),
          120.0: (11.0, 9.7, 127.0, 4.0, 124.0, 4.15, 6.0)}
DIN471 = {30.0: (5.0, 3.5, 27.9, 1.5, 28.6, 1.6, 2.1)}
EYE_LIP = {'22207 E': 64.0, '22213 E': 110.0}      # mm, the eye's shoulder: under SKF's Da max, over D1
PIN_COLLAR = {'22207 E': 43.0, '22213 E': 76.0}    # mm, the pin's collar: over SKF's da min, under d2
END_WASHER = {'22207 E': (43.0, 6.0, 9.0), '22213 E': (76.0, 8.0, 13.0)}   # D, t, hole: made
END_SCREW = {'22207 E': (8.0, 20.0), '22213 E': (12.0, 30.0)}              # ISO 4762, d x L
# each eye: its rod end, its pin, the bearing, the side the pin's free end is on (+1 = +y), the eye's span and the
# pin's root face (mm, from PalletizerRobot.py's layers: the parallel rod 675 to 745 on the crank's and the lever's
# faces at 670; the levelling rods' eyes -300 to -260, E from its bracket at -315, F, G and H from the plates at -245)
EYES = [('Parallel rod end 1', 'C', '22213 E', 1, (675.0, 745.0), 670.0),
        ('Parallel rod end 2', 'D', '22213 E', 1, (675.0, 745.0), 670.0),
        ('Levelling rod 1 end 1', 'E', '22207 E', 1, (-300.0, -260.0), -315.0),
        ('Levelling rod 1 end 2', 'F', '22207 E', -1, (-300.0, -260.0), -245.0),
        ('Levelling rod 2 end 1', 'G', '22207 E', -1, (-300.0, -260.0), -245.0),
        ('Levelling rod 2 end 2', 'H', '22207 E', -1, (-300.0, -260.0), -245.0)]


def eyes():
    """Each eye's detail along its pin, mm: the bearing's faces, the shoulder (lip) on the root side, the circlip's
    groove on the free side and the wall left beyond it, the pin's collar from its root face to the inner ring, the
    seat, the pin's end 0.5 short of the ring's face, the end washer and the screw, the screw's length by Shigley's
    L > h + 1.5 d (Table 8-7) for a tapped hole."""
    out = []
    for name, pin, nm, s, (y0, y1), root in EYES:
        br = SRB[nm]
        yc = (y0 + y1) / 2.0
        r_root, r_free = yc - s * br['B'] / 2.0, yc + s * br['B'] / 2.0       # the ring's faces
        face_root, face_free = (y0, y1) if s > 0 else (y1, y0)
        a, b, d3, cs, g_d, g_m, g_n = DIN472[br['D']]
        groove = (r_free, r_free + s * g_m)
        wall = abs(face_free - groove[1])
        Dw, tw, hole = END_WASHER[nm]
        sd, sl = END_SCREW[nm]
        washer = (r_free, r_free + s * tw)
        out.append(dict(name=name, pin=pin, bearing=nm, side=s, eye=(y0, y1), ring=(r_root, r_free),
                        lip=(face_root, r_root), lip_d=EYE_LIP[nm], groove=groove, groove_d=g_d, circlip=(cs, a, b, d3),
                        wall=wall, wall_min=g_n, collar=(root, r_root), collar_d=PIN_COLLAR[nm],
                        pin_end=r_free - s * 0.5, washer=washer, washer_d=Dw, washer_t=tw, screw=(sd, sl),
                        screw_need=tw + 1.5 * sd, lip_ok=br['D1'] < EYE_LIP[nm] <= br['Da'],
                        collar_ok=br['da'] <= PIN_COLLAR[nm] < br['d2'], washer_ok=br['da'] <= Dw < br['d2'],
                        clip_inner=br['D'] - 2.0 * a))
    return out


# ---- the bag gripper: the 6206, the jaws' ends, the plug's weld ----------------------------------------------------
def f2_aluminium(D=62.0, D2=54.06):
    """The outer ring's compression factor in a thick aluminium housing from Shigley's Eq. 3-56 (p. 130): the ring a
    steel cylinder from its recess D2 to D under the contact pressure, the housing an aluminium one of large outside
    radius. In a steel housing the same arithmetic gives D2/D, SKF's f2 within its reading."""
    R, ri = D / 2.0, D2 / 2.0
    k_ring = R / E_ST * ((R * R + ri * ri) / (R * R - ri * ri) - NU_ST)
    k_al = R / E_AL * (1.0 + NU_AL)
    k_st = R / E_ST * (1.0 + NU_ST)
    f_steel = ri / R
    return dict(steel=f_steel, aluminium=f_steel * (k_st + k_ring) / (k_al + k_ring), ratio=(k_st + k_ring) / (k_al + k_ring))


def bag_6206():
    """The 6206-2RS1 on the stub shafts: the load's class in the cycle and the brake stop, SKF's least interference
    for the rotating inner ring (Delta = 2.5 Fr d / B, Chapter 51) against j6's, k5's and m5's probable least, and the
    operating clearance at the hall's coldest and warmest for the locating end (J7, Chapter 51's class for the
    aluminium end plate) and the floating end (H7), each with SKF's f2 reduced for aluminium."""
    w = MF.worked_6206()
    fa = f2_aluminium()
    cold = (ALPHA_AL - ALPHA_ST) * (T_PRESS - T_HALL[0]) * DGBB['6206']['D'] * 1000.0     # um gained, cold
    warm = (ALPHA_AL - ALPHA_ST) * (T_HALL[1] - T_PRESS) * DGBB['6206']['D'] * 1000.0     # um lost, warm
    cases = {}
    for end, h in (('locating', 'J7'), ('floating', 'H7')):
        cases[end] = dict(cold=clearance('6206', DGBB, 'k5', h, dT=0.0, f2=fa['aluminium'], d2_extra=cold),
                          room=clearance('6206', DGBB, 'k5', h, f2=fa['aluminium']),
                          warm=clearance('6206', DGBB, 'k5', h, f2=fa['aluminium'], d2_extra=-warm))
    least = {c: min(cases[e][t]['left'][c] for e in cases for t in ('cold', 'room', 'warm')) for c in DGBB['6206']['clear']}
    most = {c: max(cases[e][t]['left'][c] for e in cases for t in ('cold', 'room', 'warm')) for c in DGBB['6206']['clear']}
    return dict(w=w, f2=fa, cold=cold, warm=warm, cases=cases, least=least, most=most,
                least_k5=-w['k5_prob'][1], need_cycle=w['need_cycle'], need_stop=w['need_stop'])


def jaw_growth():
    """How much the end plates' distance changes against the jaw's length at the hall's ends: the aluminium frame
    against the steel jaw over the span between the bearings' middles, mm (+ apart, - closer)."""
    import bag_gripper as B
    L_span = 2 * sum(B.LAY['bearing_x']) / 2.0
    return {t: (ALPHA_AL - ALPHA_ST) * (t - T_PRESS) * L_span for t in T_HALL}


BOSS = dict(lip=56.0, ext=6.0, play=1.0)     # mm: the lip under SKF's Da max 56.4, the boss lengthened, the floating
#                                              end's play each way: the detail's


def jaw_ends():
    """The jaws' ends in the end plates' bosses, mm along x (+x end): the locating end's outer ring between the lip
    and a DIN 472 62 circlip, the floating end's free 1 mm either way; the short stub's shoulder and its DIN 471 30
    circlip; the boss lengthened for the groove and its wall (n min)."""
    import bag_gripper as B
    x0, x1 = B.LAY['bearing_x']
    bo0, bo1 = B.LAY['boss']
    a, b, d3, s, g_d, g_m, g_n = DIN472[62.0]
    loc = dict(lip=(B.LAY['plate_x'][0], x0), groove=(x1, x1 + g_m), wall=bo1 + BOSS['ext'] - (x1 + g_m))
    flo = dict(lip=(B.LAY['plate_x'][0], x0 - BOSS['play']), groove=(x1 + BOSS['play'], x1 + BOSS['play'] + g_m),
               wall=bo1 + BOSS['ext'] - (x1 + BOSS['play'] + g_m))
    a1, b1, d31, s1, g1, m1, n1 = DIN471[30.0]
    short = dict(shoulder=(B.LAY['tube_x'] - 4.0, x0), groove=(x1, x1 + m1), end=x1 + m1 + n1 + 0.4, d2=g1)
    return dict(locating=loc, floating=flo, short=short, boss=(bo0, bo1 + BOSS['ext']), wall_min=g_n,
                lip_ok=BOSS['lip'] < 56.4, din472=DIN472[62.0], din471=DIN471[30.0])


# the plug's weld: a fillet all round inside the tube's end, the plug set 4 mm in; E60xx electrodes (Table 9-3)
WELD = dict(h=4.0, b=52.0, d=52.0, E60=dict(Sut=427.0, Sy=345.0))
STEEL_1020 = dict(HR=dict(Sut=380.0, Sy=210.0), CD=dict(Sut=470.0, Sy=390.0))   # Shigley Table A-20, p. 1048
STEEL_1045_CD = dict(Sut=630.0, Sy=530.0)


def plug_weld():
    """The weld between the plug and the jaw's tube, Shigley's method (Section 9-3): the weld group the tube's inside,
    52 x 52, as a line; torsion (Table 9-1, case 5: A = 1.414 h (b + d), Ju = (b + d)^3 / 6) and bending (Table 9-2,
    the box: Iu = d^2 (3 b + d) / 6) at a corner, with the shear from the bearings; in the brake stop against the
    AISC's 0.30 Sut of the E60xx weld metal and 0.40 Sy of the 1020 base metal (Table 9-4), the size against Table 9-6;
    in the cycle, repeated, Kfs = 2.7 (Table 9-5), and Gerber with the base metal's Sse as Example 9-6 takes it."""
    import bag_gripper as B
    des = B.design()
    h, b, d = WELD['h'], WELD['b'], WELD['d']
    A = 1.414 * h * (b + d)
    Ju = (b + d) ** 3 / 6.0
    J = 0.707 * h * Ju
    Iu = d * d * (3 * b + d) / 6.0
    I = 0.707 * h * Iu
    r = math.hypot(b / 2.0, d / 2.0)
    x_w = B.LAY['tube_x'] - 4.0
    x_b = sum(B.LAY['bearing_x']) / 2.0
    span = 2 * x_b

    def stresses(case, F_c, R):
        T = des[case]['M'] * 1000.0                                   # N mm
        M_stub = F_c * des['bearing']['a_l']                          # the lever's force about the bearing, N mm
        V = math.hypot(R, M_stub / span)
        M = M_stub + R * (x_b - x_w)
        tT, tV, tM = T * r / J, V / A, M * (d / 2.0) / I
        return dict(T=T / 1000.0, V=V, M=M / 1000.0, tT=tT, tV=tV, tM=tM, tau=math.hypot(tT + tV, tM))

    st = stresses('jaw_stop', des['bearing']['F_c'], des['bearing']['R'])
    ratio = des['jaw_cycle']['M'] / des['jaw_stop']['M']
    cy = stresses('jaw_cycle', des['jaw_cycle']['M'] / (B.LAY['lever'] / 1000.0), des['bearing']['R'] * ratio)
    weld_allow = 0.30 * WELD['E60']['Sut']
    base_allow = 0.40 * STEEL_1020['HR']['Sy']
    tau_base = st['tau'] * 0.707                                      # on the fusion face, the leg's area
    Sut = STEEL_1020['HR']['Sut']
    ka = 272.0 * Sut ** -0.995                                        # as forged, Table 6-2 (as Example 9-6)
    Sse = ka * 1.0 * 0.59 * 0.5 * Sut                                 # kb = 1, kc = 0.59, Se' = 0.5 Sut
    Ssu = 0.67 * Sut
    ta = tm = 2.7 * cy['tau'] / 2.0
    nf = 0.5 * (Ssu / tm) ** 2 * (ta / Sse) * (-1.0 + math.sqrt(1.0 + (2.0 * tm * Sse / (Ssu * ta)) ** 2))
    return dict(h=h, A=A, Ju=Ju, J=J, Iu=Iu, I=I, r=r, stop=st, cycle=cy, weld_allow=weld_allow, n_weld=weld_allow / st['tau'],
                base_allow=base_allow, tau_base=tau_base, n_base=base_allow / tau_base, h_min=3.2, h_max=B.TUBE['t'],
                ka=ka, Sse=Sse, Ssu=Ssu, ta=ta, tm=tm, nf=nf, x_weld=x_w)


def stub_1020():
    """The stub shaft and plug made one 1020 part so that it welds as Shigley advises (G10140 to G10230, p. 481): the
    seat's static factor in the brake stop with the hot-rolled strength the weld leaves near it (p. 482), against
    Chapter 47's 1045; the keyseat, 67 mm from the weld, keeps the drawn bar's 390 MPa, over the key stock's 370."""
    import bag_gripper as B
    vm = B.design()['bearing']['vm_stub']
    return dict(vm=vm, n_1045=STEEL_1045_CD['Sy'] / vm, n_1020=STEEL_1020['HR']['Sy'] / vm,
                key_side=STEEL_1020['CD']['Sy'], key_stock=370.0)


# ---- the compliance unit -------------------------------------------------------------------------------------------
GLAND = dict(W=2.50, W_tol=0.09, E=2.15, E_tol=0.04, S=0.10, F=3.40, R=0.2)       # ERIKS Table 3.D-1, the 2.50 row
FINISH = dict(dynamic=(0.2, 0.4), floor=0.15, groove=0.4, flanks=0.8)             # um Ra, ERIKS pp. 133, 146


def gland(rod=20.0):
    """The O-ring's groove in the receiver's bore round the 20 mm tube, to ERIKS's dynamic table: the groove's
    diameter and width, the squeeze nominal and at the tolerances' ends, the tube's g6 against the bore's H7 against
    ERIKS's diametral clearance; Volume 2's groove for comparison."""
    lo_g6, hi_g6 = MF.fit('H7', 'g6', rod)
    W, E = GLAND['W'], GLAND['E']
    sq = lambda w, e: (w - e) / w * 100.0
    return dict(groove_d=rod + 2 * E, width=GLAND['F'], squeeze=sq(W, E),
                squeeze_min=sq(W - GLAND['W_tol'], E + GLAND['E_tol']), squeeze_max=sq(W + GLAND['W_tol'], E),
                gap=(lo_g6, hi_g6), S=GLAND['S'] * 1000.0, v2=dict(groove_d=rod + 5.0, width=3.5, squeeze=sq(W, 2.5)))


DIN908 = dict(d=10.0, p=1.0, k=3.0, dk=14.0, i=8.0, s=5.0)       # Fasten.it, the M10 x 1 row
PLUG = dict(tap_drill=9.0, tapped=10.0, cb_d=15.0, cb_t=3.0, p_air=0.6)  # mm, MPa: the detail's; the air's 6 bar


def gallery_plug():
    return dict(DIN908, **PLUG, force=PLUG['p_air'] * pi / 4.0 * DIN908['d'] ** 2)


# ---- the vacuum gripper's plate joint -------------------------------------------------------------------------------
VAC = dict(wall=5.0, plate=10.0, washer=(6.4, 12.0, 1.6), nut_m=5.0, d=6.0, At=20.1, Sp=580.0, K=0.20,
           head=10.0)                                            # ISO 7089, ISO 4032, ISO 4762's head; Shigley K


def vacuum_joint():
    """The plate held to each beam by through bolts: ISO 4762 M6 from inside the beam with a washer under its head,
    through the beam's 5 mm wall and the 10 mm plate, a washer and an ISO 4032 nut under the plate. The length by
    Shigley's L > l + H (Table 8-7), the next commercial; Volume 1's preload and torque; the pressure under the
    washers on the 6082-T6 against Hydro's EN 755-2 minimum Rp0.2; the stop's load per screw (Chapter 46)."""
    import standard_parts as S
    import vacuum_gripper as V
    v = VAC
    grip = v['wall'] + v['plate'] + 2 * v['washer'][2]
    need = grip + v['nut_m']
    L = S.shortest(S.commercial('DIN_EN_ISO_4762')['6'], need)
    Fi = 0.75 * v['At'] * v['Sp']
    T = v['K'] * Fi * v['d'] / 1000.0
    Aw = pi / 4.0 * (v['washer'][1] ** 2 - v['washer'][0] ** 2)
    Ah = pi / 4.0 * (v['head'] ** 2 - v['washer'][0] ** 2)
    fr = V.design()['frame']['screws']
    return dict(grip=grip, need=need, L=L, Fi=Fi, T=T, Aw=Aw, p_washer=Fi / Aw, p_head=Fi / Ah, Rp_beam=250.0,
                Rp_plate=260.0, per=fr['per'], shear=fr['shear'], n_washer=250.0 / (Fi / Aw),
                thread_v2=V.LAY['plate_screw']['L'] - v['wall'], screws=2 * len(V.LAY['plate_screw']['x']))


# ---- HIWIN's codes --------------------------------------------------------------------------------------------------
HIWIN = dict(unit=dict(block='HGH25CAZ0C', rail='HGR25R218C', blocks=4, rails=2),
             clamp=dict(block='HGH25CAZ0C', rail='HGR25R1720C', blocks=8, rails=2),
             Z0=(0.0, 0.02), parallel_C={218: 15.0, 1720: 28.0})


def summary():
    return dict(rods=rods(), joints=joint_dirs(), bearings=robot_bearings(), eyes=eyes(), bag=bag_6206(),
                growth=jaw_growth(), ends=jaw_ends(), weld=plug_weld(), stub=stub_1020(), gland=gland(),
                plug=gallery_plug(), vacuum=vacuum_joint(), hiwin=HIWIN)


if __name__ == '__main__':
    import pprint
    pprint.pprint(summary(), width=150)


# ---- what is ordered: the numbers (Chapter 49's scheme: a new designation takes the next free serial) ---------------
NEW_NUMBERS = [  # (number, designation, assembly, quantity, where)
    ('S-0110', 'DIN 472 62 x 2', 'bag', 4, 'the bosses\' circlips'),
    ('S-0111', 'DIN 471 30 x 1.5', 'bag', 2, 'the short stubs\' circlips'),
    ('S-0112', 'SKF 22207 E/C3', 'robot', 1, 'under the triangle'),
    ('S-0113', 'DIN 472 72 x 2.5', 'robot', 4, 'the levelling rods\' eyes'),
    ('S-0114', 'DIN 472 120 x 4', 'robot', 2, 'the parallel rod\'s eyes'),
    ('S-0115', 'ISO 4762 M12 x 30 - 8.8 - zinc-plated', 'robot', 2, 'the parallel rod\'s pins\' end screws'),
    ('S-0116', 'ISO 4762 M6 x 25 - 8.8 - zinc-plated', 'vacuum', 16, 'the plate\'s through bolts'),
    ('S-0117', 'ISO 7089 6 - 200 HV', 'vacuum', 32, 'under the bolts\' heads and nuts'),
    ('S-0118', 'ISO 4032 M6 - 8', 'vacuum', 16, 'the bolts\' nuts'),
    ('P01-060', 'end washer 43 x 6, hole 9, steel 1045', 'robot', 4, 'the levelling rods\' pins'),
    ('P01-061', 'end washer 76 x 8, hole 13, steel 1045', 'robot', 2, 'the parallel rod\'s pins'),
    ('P13-016', 'jaw weldment: the tube and its two stubs, welded, the seats ground', 'bag', 2, 'each jaw'),
]
CHANGED_NUMBERS = [  # (number, as it was, as it is now)
    ('S-0006', 'SKF 22207 E', 'SKF 22207 E, Normal clearance, in the levelling rods\' four eyes only'),
    ('S-0007', 'SKF 22213 E', 'SKF 22213 E/C3'),
    ('S-0012', 'SKF 61836', 'SKF 61836/C4, its availability SKF\'s to confirm'),
    ('S-0013', 'SKF 61856 MA', 'SKF 61856 MA/C4, its availability SKF\'s to confirm'),
    ('S-0016', 'SKF 6206-2RS1', 'SKF 6206-2RS1/C4, its availability SKF\'s to confirm'),
    ('S-0036', 'ISO 4762 M8 x 20', 'also the levelling rods\' pins\' end screws, 4 more'),
    ('S-0102', 'gallery plug 8', 'DIN 908 M10 x 1, a sealing ring under its collar'),
    ('S-0103', 'O-ring 20 x 2.5', 'O-ring 20 x 2.5 in a groove 24.30 x 3.40 (ERIKS)'),
    ('S-0021', 'HIWIN HGH25CA', 'HIWIN HGH25CAZ0C'),
]
