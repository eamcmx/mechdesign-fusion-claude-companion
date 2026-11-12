# =============================================================================
# SpurReducer.py - a single-stage spur reducer sized by subsystems and built in Fusion: a split cast iron housing
# with bearing caps, lip seals, screws, dowels and plugs; an input shaft with the pinion cut on it; an output shaft
# carrying the gear, its key and a spacer. Every size comes from the sizing below (Budynas and Nisbett, Shigley's
# Mechanical Engineering Design, 10th ed., 2015); a duty the parts cannot carry is refused, saying which part fails.
#
# Run it: a dialog asks for the duty (power, input speed, application factor, bearing life, design factor); OK sizes
# the reducer, refuses it or builds it in a new design, and a message gives the numbers and the parts list.
#
# World frame: shafts along y, the split plane z = 0, the pinion's axis at x = 0, the gear's at x = 171 mm.
# Sizes in mm, N, MPa, N m; Fusion works in cm and radians (mdfc_parts converts).
# =============================================================================
import importlib
import math
import os
import re
import sys
import traceback

import adsk.core
import adsk.fusion


def _companion_common():
    """Walk up from this script's folder to the first one that holds common/companion_paths.py."""
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if os.path.isfile(os.path.join(d, 'common', 'companion_paths.py')):
            return os.path.join(d, 'common')
        up = os.path.dirname(d)
        if up == d:
            raise RuntimeError('no common/companion_paths.py above %s' % __file__)
        d = up


_COMMON = _companion_common()
if _COMMON not in sys.path:
    sys.path.insert(0, _COMMON)
import companion_paths as CP  # noqa: E402,F401  (puts every lib/ folder on sys.path)
import mdfc_parts as mp       # noqa: E402
importlib.reload(mp)

VERSION = 'SpurReducer v1.0 (prompt test, 12 Nov 2026)'
CMD_ID = 'mdfcSpurReducerPromptTest'
DEFAULTS = dict(P=4.0, n1=1440.0, KA=1.25, life=20000.0, nd=2.0)

# ---------------------------------------------------------------------------------------------------------------
# data of Budynas and Nisbett (2015) and DIN 6885
# ---------------------------------------------------------------------------------------------------------------
# Table 14-2: Lewis form factor Y, 20 deg full-depth teeth
TABLE_14_2 = [(12, 0.245), (13, 0.261), (14, 0.277), (15, 0.290), (16, 0.296), (17, 0.303), (18, 0.309),
              (19, 0.314), (20, 0.322), (21, 0.328), (22, 0.331), (24, 0.337), (26, 0.346), (28, 0.353),
              (30, 0.359), (34, 0.371), (38, 0.384), (43, 0.397), (50, 0.409), (60, 0.422), (75, 0.435),
              (100, 0.447), (150, 0.460), (300, 0.472), (400, 0.480)]

# Table 11-2: 02-series deep-groove ball bearings, bore d -> (D, B, fillet r, shoulder dS, C10 kN)
TABLE_11_2 = {20: (47.0, 14.0, 1.0, 25.0, 12.7), 25: (52.0, 15.0, 1.0, 30.0, 14.0),
              30: (62.0, 16.0, 1.0, 35.0, 19.5), 35: (72.0, 17.0, 1.0, 41.0, 25.5)}

# DIN 6885-1 parallel keys: (b, h) -> keyseat depth in the shaft t1, in the hub t2 (mm)
DIN_6885 = {(5, 5): (3.0, 2.3), (8, 7): (4.0, 3.3), (10, 8): (5.0, 3.3)}

# Table 7-1: first-iteration stress concentration, end-milled keyseat
KT_KEYSEAT, KTS_KEYSEAT = 2.14, 3.0
E_STEEL = 207e3                      # MPa, Table A-5
SLOPE_ALLOWED = 0.001                # rad at the deep-groove bearings (Table 7-2)

# the sizes chosen
M, Z1, Z2, FACE, PHI_DEG = 3.0, 19, 95, 30.0, 20.0
EFF = 0.98                           # mesh efficiency in T2
HB = 300.0                           # through-hardened steel
CP_STEEL = 191.0                     # sqrt(MPa), steel on steel (Eq. 14-14)
SUT, SY = 630.0, 530.0               # 1045 CD (Table A-20)
SPAN_IN, SPAN_OUT = 98.0, 100.0      # bearing centres, mm
SEAT_IN, SEAT_OUT = 20.0, 30.0       # bearing seats, mm
SHOULDER_IN, SHOULDER_OUT = 25.0, 40.0
TAU_ALLOWED, SIGMA_ALLOWED = 60.0, 100.0
KEYS = (('in', 'input end', 16.0, 'T1', 5, 5, 28.0),
        ('gear', 'gear seat', 35.0, 'T2', 10, 8, 42.0),
        ('out', 'output end', 25.0, 'T2', 8, 7, 52.0))
KEY_NAMES = {'in': 'Key 5x5x28 input', 'gear': 'Key 10x8x42 gear', 'out': 'Key 8x7x52 output'}


def lewis_Y(z):
    """Y of Table 14-2, straight-line interpolation between the tabled tooth numbers."""
    for (za, ya), (zb, yb) in zip(TABLE_14_2, TABLE_14_2[1:]):
        if za <= z <= zb:
            return ya + (yb - ya) * (z - za) / float(zb - za)
    raise ValueError('no Lewis form factor for %d teeth in Table 14-2' % z)


def bearing_row(seat):
    """The 02-series bearing of Table 11-2 whose bore is the seat: 6204 on 20 mm, 6206 on 30 mm."""
    d = int(round(seat))
    if d not in TABLE_11_2:
        raise ValueError('no 02-series bearing of Table 11-2 for a %g mm seat' % seat)
    D, B, r, dS, C10 = TABLE_11_2[d]
    return dict(name='62%02d' % (d // 5), d=float(d), D=D, B=B, r=r, dS=dS, C10=C10)


def gear_deflection_allowed(m):
    """Table 7-2, transverse deflection at spur gears, by diametral pitch P = 25.4 / m teeth/in (mm)."""
    P = 25.4 / m
    if P < 10.0:
        return 0.010 * 25.4
    if P < 20.0:
        return 0.005 * 25.4
    return 0.003 * 25.4


# ---------------------------------------------------------------------------------------------------------------
# the sizing
# ---------------------------------------------------------------------------------------------------------------
def sizing(P=4.0, n1=1440.0, KA=1.25, life=20000.0, nd=2.0):
    """Every number of the reducer for one duty, and the list of reasons to refuse it (empty if it carries it)."""
    for k, v in (('P', P), ('n1', n1), ('KA', KA), ('life', life), ('nd', nd)):
        if not v > 0:
            raise ValueError('Refused: %s must be positive, not %g' % (k, v))
    phi = math.radians(PHI_DEG)
    refuse = []

    # ---- gears
    w1 = n1 * math.pi / 30.0                      # rad/s
    T1 = P * 1e3 / w1                             # N m
    T2 = T1 * (Z2 / Z1) * EFF                     # N m
    d1, d2 = M * Z1, M * Z2
    Ft = KA * 2.0 * T1 * 1e3 / d1                 # N
    Fr = Ft * math.tan(phi)
    W = Ft / math.cos(phi)
    V = math.pi * d1 * n1 / 60e3                  # m/s
    Kv = (6.1 + V) / 6.1                          # Eq. 14-6b, cut or milled profile
    Y1, Y2 = lewis_Y(Z1), lewis_Y(Z2)
    s1 = Kv * Ft / (FACE * M * Y1)                # Lewis, MPa
    s2 = Kv * Ft / (FACE * M * Y2)
    r1, r2 = d1 * math.sin(phi) / 2.0, d2 * math.sin(phi) / 2.0     # radii of curvature at the pitch point
    sc = CP_STEEL * math.sqrt(Kv * Ft / (FACE * math.cos(phi)) * (1.0 / r1 + 1.0 / r2))   # Eq. 14-14, magnitude
    St = 0.533 * HB + 88.3                        # Fig. 14-2, grade 1
    Sc = 2.22 * HB + 200.0                        # Fig. 14-5, grade 1
    N1 = 60.0 * n1 * life                         # load cycles over the life
    N2 = N1 * Z1 / Z2
    YN1, YN2 = 1.6831 * N1 ** -0.0323, 1.6831 * N2 ** -0.0323      # Fig. 14-14
    ZN1, ZN2 = 2.466 * N1 ** -0.056, 2.466 * N2 ** -0.056          # Fig. 14-15
    SF1, SF2 = St * YN1 / s1, St * YN2 / s2
    SH1, SH2 = Sc * ZN1 / sc, Sc * ZN2 / sc
    for who, SF, SH in (('pinion', SF1, SH1), ('gear', SF2, SH2)):
        if SF < nd:
            refuse.append("the %s's teeth have a bending factor of %.3f, below nd = %g" % (who, SF, nd))
        if SH < 1.1:
            refuse.append("the %s's flanks have a contact factor of %.3f, below 1.1" % (who, SH))
    gears = dict(m=M, z1=Z1, z2=Z2, F=FACE, phi_deg=PHI_DEG, d1=d1, d2=d2, a=(d1 + d2) / 2.0, w1=w1, T1=T1,
                 T2=T2, Ft=Ft, Fr=Fr, W=W, V=V, Kv=Kv, Y1=Y1, Y2=Y2, s1=s1, s2=s2, r1=r1, r2=r2, sc=sc, St=St,
                 Sc=Sc, N1=N1, N2=N2, YN1=YN1, YN2=YN2, ZN1=ZN1, ZN2=ZN2, SF1=SF1, SF2=SF2, SH1=SH1, SH2=SH2)

    # ---- shafts: DE-Goodman (Eq. 7-8 solved for n), bending fully reversed, torque steady
    ka = 4.51 * SUT ** -0.265                     # machined or cold drawn (Eq. 6-19)

    def n_de_goodman(d, Ma, Tm):
        kb = 1.24 * d ** -0.107                   # Eq. 6-20
        Se = ka * kb * 0.5 * SUT
        n = math.pi * d ** 3 / 16.0 / (2.0 * KT_KEYSEAT * Ma / Se + math.sqrt(3.0) * KTS_KEYSEAT * Tm / SUT)
        return n, kb, Se

    M_gear = W * SPAN_OUT / 4.0                   # N mm, W at mid-span
    sections = {}
    for key, name, d, Ma, Tm in (('n_in', 'input end', 16.0, 0.0, KA * T1 * 1e3),
                                 ('n_gear', 'gear seat', 35.0, M_gear, KA * T2 * 1e3),
                                 ('n_out', 'output end', 25.0, 0.0, KA * T2 * 1e3)):
        n, kb, Se = n_de_goodman(d, Ma, Tm)
        sections[key] = dict(name=name, d=d, Ma=Ma, Tm=Tm, kb=kb, Se=Se, n=n)
        if n < nd:
            refuse.append('the %s has a factor of safety of %.2f, below nd = %g' % (name, n, nd))
    I_in = math.pi * SEAT_IN ** 4 / 64.0
    I_out = math.pi * SEAT_OUT ** 4 / 64.0
    theta1 = W * SPAN_IN ** 2 / (16.0 * E_STEEL * I_in)
    theta2 = W * SPAN_OUT ** 2 / (16.0 * E_STEEL * I_out)
    y1 = W * SPAN_IN ** 3 / (48.0 * E_STEEL * I_in)
    y2 = W * SPAN_OUT ** 3 / (48.0 * E_STEEL * I_out)
    y_allowed = gear_deflection_allowed(M)
    for who, th in (('input', theta1), ('output', theta2)):
        if th > SLOPE_ALLOWED:
            refuse.append('the %s shaft slopes %.5f rad at its bearings, above %g rad' % (who, th, SLOPE_ALLOWED))
    for who, y in (('pinion', y1), ('gear', y2)):
        if y > y_allowed:
            refuse.append('the %s deflects %.4f mm, more than the %.3f mm Table 7-2 allows' % (who, y, y_allowed))
    shafts = dict(Sut=SUT, Sy=SY, ka=ka, Kt=KT_KEYSEAT, Kts=KTS_KEYSEAT, M_gear=M_gear, sections=sections,
                  n_in=sections['n_in']['n'], n_gear=sections['n_gear']['n'], n_out=sections['n_out']['n'],
                  theta1=theta1, theta2=theta2, y1=y1, y2=y2, y_allowed=y_allowed, E=E_STEEL)

    # ---- bearings: W shared equally by the two bearings of each shaft
    bearings = {}
    for key, seat, n in (('in', SEAT_IN, n1), ('out', SEAT_OUT, n1 * Z1 / Z2)):
        row = bearing_row(seat)
        R = W / 2.0
        C_needed = R * (60.0 * n * life / 1e6) ** (1.0 / 3.0) / 1e3      # kN
        L10h = (row['C10'] * 1e3 / R) ** 3 * 1e6 / (60.0 * n)
        row.update(R=R, n=n, C_needed=C_needed, L10h=L10h)
        bearings[key] = row
        if L10h < life:
            refuse.append('bearing %s lasts %.0f h, short of the %.0f h asked (C needed %.2f kN, C10 %.2f kN)'
                          % (row['name'], L10h, life, C_needed, row['C10']))

    # ---- keys, DIN 6885 form A, effective length L - b
    keys = {}
    for key, where, d, which, b, h, L in KEYS:
        t1, t2 = DIN_6885[(b, h)]
        T = T1 if which == 'T1' else T2
        F = 2.0 * KA * T * 1e3 / d
        tau = F / (b * (L - b))
        sigma = F / ((h - t1) * (L - b))
        keys[key] = dict(name=KEY_NAMES[key], where=where, d=d, b=float(b), h=float(h), t1=t1, t2=t2, L=L, F=F,
                         tau=tau, sigma=sigma)
        if tau > TAU_ALLOWED:
            refuse.append('the %s key shears at %.1f MPa, above %g MPa' % (key, tau, TAU_ALLOWED))
        if sigma > SIGMA_ALLOWED:
            refuse.append('the %s key bears %.1f MPa in crushing, above %g MPa' % (key, sigma, SIGMA_ALLOWED))

    return dict(inputs=dict(P=P, n1=n1, KA=KA, life=life, nd=nd), gears=gears, shafts=shafts, bearings=bearings,
                keys=keys, refusals=refuse)


class rd:
    """The design calculations on their own: rd.design(**duty) returns the sizing of sizing()."""
    design = staticmethod(sizing)
    lewis_Y = staticmethod(lewis_Y)
    bearing_row = staticmethod(bearing_row)


# ---------------------------------------------------------------------------------------------------------------
# the layout (mm, world)
# ---------------------------------------------------------------------------------------------------------------
A = 171.0                                  # centre distance, m (z1 + z2) / 2
YW_IN, YW_OUT = 42.0, 60.0                 # the bearing walls, each side
CAP_T = 8.0
Y_CAP = YW_OUT + CAP_T                     # the caps' outer faces
BACKLASH = 0.2                             # circular backlash of the pair, mm (half taken off each gear's teeth)

X0, X1 = -57.0, 342.0                      # housing outside, x
ZB, ZT = -175.0, 172.0                     # housing bottom and top
T_END, T_FLOOR, T_ROOF = 12.0, 12.0, 12.0
FL_T, FL_Y, FL_X = 14.0, 88.0, 25.0        # split flange: thickness each half, reach in y, reach in x
CAP_GAP = 4.0                              # the flange stops this far short of each cap
FOOT_Y, FOOT_T = 95.0, 15.0
Z_DRAIN, X_FILL = -154.0, 80.0

# input shaft (x = 0), sections (y0, y1, d) in order along y; the 16 mm end through the cap at -y
RF1 = M * Z1 / 2.0 - 1.25 * M
IN_SECTIONS = [(-120.0, -72.0, 16.0), (-72.0, -42.0, SEAT_IN), (-42.0, -FACE / 2, SHOULDER_IN),
               (-FACE / 2, FACE / 2, 2 * RF1), (FACE / 2, 42.0, SHOULDER_IN), (42.0, 58.0, SEAT_IN)]
IN_KEY_Y = (-110.0, -82.0)
# output shaft (x = 171): blind cap at -y, the 25 mm end through the cap at +y
HUB_L, HUB_D, GEAR_BORE = 50.0, 65.0, 35.0
OUT_SECTIONS = [(-59.0, -42.0, SEAT_OUT), (-42.0, -HUB_L / 2, SHOULDER_OUT), (-HUB_L / 2, HUB_L / 2 - 2.0, GEAR_BORE),
                (HUB_L / 2 - 2.0, 72.0, SEAT_OUT), (72.0, 130.0, 25.0)]
GEAR_KEY_Y = (-21.0, 21.0)
OUT_KEY_Y = (75.0, 127.0)
SPACER = (HUB_L / 2, YW_IN, 40.0, SEAT_OUT)         # y0, y1, outside, bore
RF2 = M * Z2 / 2.0 - 1.25 * M
RIM_IN, WEB_T, N_HOLES, HOLE_R, HOLE_D = 129.0, 10.0, 6, 80.0, 40.0

SEALS = {'in': (20.0, 35.0, 7.0), 'out': (30.0, 47.0, 7.0)}     # d x D x b
BOLTS_X = (-55.0, 55.0, 109.0, 233.0)                           # split flange M10, both sides at y = +-74
BOLT_Y = YW_OUT + FL_T
DOWELS = ((-70.0, BOLT_Y), (355.0, -BOLT_Y))
FEET_HOLES = [(x, s * 80.0) for x in (X0 + 25.0, X1 - 25.0) for s in (1, -1)]


def cap_layout(row):
    """Outer radius, screw circle and spigot bore of a cap on a bearing row."""
    g = mp.bearing_geometry(row['d'], row['D'])
    return dict(R=row['D'] / 2.0 + 14.0, Rbc=row['D'] / 2.0 + 7.0, r_spigot=g['out_id'] / 2.0 + 0.5,
                ring_face=YW_IN + row['B'])


def cap_screws(xc, row):
    """The (x, z) of the four M6 screws of a cap, at 45 deg off the split plane."""
    c = cap_layout(row)
    return [(xc + c['Rbc'] * math.cos(math.radians(a)), c['Rbc'] * math.sin(math.radians(a))) for a in (45, 135, 225, 315)]


def flange_x_ranges(bearings):
    """Where the split flange runs along the long sides: everywhere but beside the caps."""
    ri = cap_layout(bearings['in'])['R'] + CAP_GAP
    ro = cap_layout(bearings['out'])['R'] + CAP_GAP
    return [(X0 - FL_X, -ri), (ri, A - ro), (A + ro, X1 + FL_X)]


# ---------------------------------------------------------------------------------------------------------------
# temporary bodies
# ---------------------------------------------------------------------------------------------------------------
def housing_half(lower, d):
    B = d['bearings']
    z0, z1 = (ZB, 0.0) if lower else (0.0, ZT)
    body = mp.box(X0, X1, -YW_OUT, YW_OUT, z0, z1)
    if lower:
        mp.cut(body, mp.box(X0 + T_END, X1 - T_END, -YW_IN, YW_IN, ZB + T_FLOOR, 1.0))
    else:
        mp.cut(body, mp.box(X0 + T_END, X1 - T_END, -YW_IN, YW_IN, -1.0, ZT - T_ROOF))
    fz0, fz1 = (-FL_T, 0.0) if lower else (0.0, FL_T)
    pieces = []
    for s in (1, -1):
        ya, yb = sorted((s * (YW_OUT - 5.0), s * FL_Y))
        for xa, xb in flange_x_ranges(B):
            pieces.append(mp.box(xa, xb, ya, yb, fz0, fz1))
    for xa, xb in ((X0 - FL_X, X0 + 5.0), (X1 - 5.0, X1 + FL_X)):
        pieces.append(mp.box(xa, xb, -FL_Y, FL_Y, fz0, fz1))
    if lower:
        for s in (1, -1):
            ya, yb = sorted((s * (YW_OUT - 5.0), s * FOOT_Y))
            pieces.append(mp.box(X0, X1, ya, yb, ZB, ZB + FOOT_T))
    mp.union(body, *pieces)
    tools = []
    for xc, row in ((0.0, B['in']), (A, B['out'])):
        tools.append(mp.cyl((xc, -YW_OUT - 1.0, 0.0), (xc, YW_OUT + 1.0, 0.0), row['D'] / 2.0))
        for x, z in cap_screws(xc, row):
            if (z < 0) == lower:
                for s in (1, -1):
                    tools.append(mp.cyl((x, s * (YW_OUT - 10.0), z), (x, s * (YW_OUT + 1.0), z), 3.0))
    for x in BOLTS_X:
        for s in (1, -1):
            tools.append(mp.cyl((x, s * BOLT_Y, -FL_T - 1.0), (x, s * BOLT_Y, FL_T + 1.0), 5.5))
    for x, y in DOWELS:
        tools.append(mp.cyl((x, y, -FL_T - 1.0), (x, y, FL_T + 1.0), 4.0))
    if lower:
        tools.append(mp.cyl((X1 - T_END - 1.0, 0.0, Z_DRAIN), (X1 + 1.0, 0.0, Z_DRAIN), 8.0))
        for x, y in FEET_HOLES:
            tools.append(mp.cyl((x, y, ZB - 1.0), (x, y, ZB + FOOT_T + 1.0), 7.0))
    else:
        tools.append(mp.cyl((X_FILL, 0.0, ZT - T_ROOF - 1.0), (X_FILL, 0.0, ZT + 1.0), 8.0))
    mp.cut(body, *tools)
    return body


def cap_body(xc, s, row, seal=None):
    """A cap on the wall's outer face at y = s 60, its spigot in the bore down to the outer ring's face; a through
    cap holds a lip seal (d, D, b) in a bore of its own."""
    c = cap_layout(row)
    body = mp.cyl((xc, s * YW_OUT, 0.0), (xc, s * Y_CAP, 0.0), c['R'])
    mp.union(body, mp.tube((xc, s * c['ring_face'], 0.0), (xc, s * (YW_OUT + 1.0), 0.0), row['D'] / 2.0, c['r_spigot']))
    tools = []
    if seal:
        sd, sD, sb = seal
        tools.append(mp.cyl((xc, s * (YW_OUT - 1.0), 0.0), (xc, s * (Y_CAP - sb + 0.5), 0.0), sd / 2.0 + 1.0))
        tools.append(mp.cyl((xc, s * (Y_CAP - sb), 0.0), (xc, s * (Y_CAP + 1.0), 0.0), sD / 2.0))
    for x, z in cap_screws(xc, row):
        tools.append(mp.cyl((x, s * (YW_OUT - 1.0), z), (x, s * (Y_CAP + 1.0), z), 3.3))
    mp.cut(body, *tools)
    return body


def plug_body(p, axis, d=16.0):
    """A screw plug M16 x 1.5: a head drawn round (24 across the flats) on the outside, the shank in the hole."""
    head = mp.cyl(p, [p[i] - axis[i] * 10.0 for i in range(3)], 12.0)
    return mp.union(head, mp.cyl([p[i] - axis[i] * 0.5 for i in range(3)], [p[i] + axis[i] * 11.0 for i in range(3)], d / 2.0))


def gear_blank(d):
    """The gear without its teeth: hub on the 35 mm seat, web with six holes, rim under the teeth, hub keyway."""
    t2 = d['keys']['gear']['t2']
    hub = mp.tube((A, -HUB_L / 2, 0.0), (A, HUB_L / 2, 0.0), HUB_D / 2.0, GEAR_BORE / 2.0)
    web = mp.tube((A, -WEB_T / 2, 0.0), (A, WEB_T / 2, 0.0), RIM_IN + 1.0, HUB_D / 2.0 - 0.5)
    rim = mp.tube((A, -FACE / 2, 0.0), (A, FACE / 2, 0.0), RF2, RIM_IN)
    mp.union(hub, web, rim)
    tools = []
    for k in range(N_HOLES):
        a = math.radians(30.0 + 60.0 * k)
        x, z = A + HOLE_R * math.cos(a), HOLE_R * math.sin(a)
        tools.append(mp.cyl((x, -WEB_T, z), (x, WEB_T, z), HOLE_D / 2.0))
    b = d['keys']['gear']['b']
    tools.append(mp.box(A - b / 2, A + b / 2, -HUB_L / 2 - 1.0, HUB_L / 2 + 1.0, 0.0, GEAR_BORE / 2.0 + t2))
    mp.cut(hub, *tools)
    return hub


# ---------------------------------------------------------------------------------------------------------------
# the model
# ---------------------------------------------------------------------------------------------------------------
def _new_occ(parent, name, t=(0.0, 0.0, 0.0)):
    """mp.new_occ; should Fusion refuse a component name already used, the copy is named 'name (2)', 'name (3)'."""
    before = parent.occurrences.count
    try:
        return mp.new_occ(parent, name, t)
    except Exception:
        if parent.occurrences.count == before:
            raise
        occ = parent.occurrences.item(parent.occurrences.count - 1)
        for k in range(2, 1000):
            try:
                occ.component.name = '%s (%d)' % (name, k)
                break
            except Exception:
                continue
        if parent != parent.parentDesign.rootComponent:
            occ.isGroundToParent = True
        return occ


def _part(parent, name, tb, material, t=(0.0, 0.0, 0.0)):
    """A component of the subassembly `parent`, grounded to it, holding one body given in world mm."""
    occ = _new_occ(parent, name, t)
    occ.isGroundToParent = True
    mp.add_body(occ, tb, name, material)
    return occ


def _rotation(occ):
    t = occ.transform2
    return [[t.getCell(i, j) for j in range(3)] for i in range(3)]


def _turn_about_y(R1, R0):
    """Degrees of the rotation R1 R0^T about +y (right-handed)."""
    D = [[sum(R1[a][k] * R0[b][k] for k in range(3)) for b in range(3)] for a in range(3)]
    s = (D[0][2] - D[2][0]) / 2.0
    c = (D[0][0] + D[1][1] + D[2][2] - 1.0) / 2.0
    return math.degrees(math.atan2(s, c))


def _plus_y_joint_geometry(comp, occ, x):
    """A joint origin on the axis at (x, 0, 0) and the joint direction of it that points along +y, so that a
    positive rotation is right-handed about +y."""
    dirs = adsk.fusion.JointDirections
    first = None
    for plane in (comp.xYConstructionPlane, comp.xZConstructionPlane, comp.yZConstructionPlane):
        sk = comp.sketches.add(plane)
        sk.name = 'Joint origin x = %g mm' % x
        pt = sk.sketchPoints.add(sk.modelToSketchSpace(mp.P3(x * mp.CM, 0.0, 0.0)))
        geo = adsk.fusion.JointGeometry.createByPoint(pt.createForAssemblyContext(occ))
        first = first or geo
        for direction, attr in ((dirs.YAxisJointDirection, 'thirdAxisVector'),
                                (dirs.ZAxisJointDirection, 'primaryAxisVector'),
                                (dirs.XAxisJointDirection, 'secondaryAxisVector')):
            try:
                v = getattr(geo, attr)
                if v is not None and v.y > 0.999:
                    return geo, direction
            except Exception:
                pass
    return first, dirs.YAxisJointDirection


def build_model(root, d):
    B, K = d['bearings'], d['keys']
    steel, iron = 'Steel', 'Iron, Cast'
    housing = mp.new_occ(root, 'Housing')
    inp = mp.new_occ(root, 'Input shaft assembly')
    out = mp.new_occ(root, 'Output shaft assembly')
    housing.isGrounded = True
    H, I, O = housing.component, inp.component, out.component

    # ---- housing
    _part(H, 'Housing base', housing_half(True, d), iron)
    _part(H, 'Housing cover', housing_half(False, d), iron)
    rings = {}
    for key, xc in (('in', 0.0), ('out', A)):
        row = B[key]
        for side, y0 in (('A', -YW_IN - row['B']), ('B', YW_IN)):
            inner, outer, balls, g = mp.bearing_bodies(row['d'], row['D'], row['B'], y0, x=xc)
            rings[(key, side)] = inner
            _part(H, 'Bearing %s %s outer ring' % (row['name'], side), outer, steel, (xc, 0.0, 0.0))
            _part(H, 'Bearing %s %s balls' % (row['name'], side), balls, steel, (xc, 0.0, 0.0))
    _part(H, 'Cap input through', cap_body(0.0, -1, B['in'], SEALS['in']), iron, (0.0, 0.0, 0.0))
    _part(H, 'Cap input blind', cap_body(0.0, 1, B['in']), iron, (0.0, 0.0, 0.0))
    _part(H, 'Cap output blind', cap_body(A, -1, B['out']), iron, (A, 0.0, 0.0))
    _part(H, 'Cap output through', cap_body(A, 1, B['out'], SEALS['out']), iron, (A, 0.0, 0.0))
    for key, xc, s in (('in', 0.0, -1), ('out', A, 1)):
        sd, sD, sb = SEALS[key]
        seal = mp.tube((xc, s * (Y_CAP - sb), 0.0), (xc, s * Y_CAP, 0.0), sD / 2.0, sd / 2.0)
        _part(H, 'Lip seal %gx%gx%g' % (sd, sD, sb), seal, steel, (xc, 0.0, 0.0))
    for key, xc, sides in (('in', 0.0, (-1, 1)), ('out', A, (-1, 1))):
        for s in sides:
            for x, z in cap_screws(xc, B[key]):
                _part(H, 'Cap screw M6x16', mp.cap_screw(6.0, 16.0, (x, s * Y_CAP, z), (0.0, -s, 0.0)), steel)
    for x in BOLTS_X:
        for s in (1, -1):
            _part(H, 'Screw M10x45', mp.cap_screw(10.0, 45.0, (x, s * BOLT_Y, FL_T), (0.0, 0.0, -1.0)), steel)
            _part(H, 'Nut M10', mp.nut(10.0, (x, s * BOLT_Y, -FL_T), (0.0, 0.0, -1.0)), steel)
    for x, y in DOWELS:
        _part(H, 'Dowel 8x24', mp.dowel(8.0, (x, y, -12.0), (x, y, 12.0)), steel)
    _part(H, 'Drain plug M16', plug_body((X1, 0.0, Z_DRAIN), (-1.0, 0.0, 0.0)), steel)
    _part(H, 'Filler plug M16', plug_body((X_FILL, 0.0, ZT), (0.0, 0.0, -1.0)), steel)

    # ---- input shaft assembly: the pinion cut on the shaft
    shaft = mp.stepped_shaft(IN_SECTIONS, x=0.0)
    k = K['in']
    mp.keyseat(shaft, k['d'], IN_KEY_Y[0], IN_KEY_Y[1], k['b'], k['t1'], x=0.0)
    pin = _part(I, 'Pinion shaft', shaft, steel)
    mp.spur_teeth(pin, pin.component.bRepBodies.item(0), M, Z1, PHI_DEG, BACKLASH, 0.0, -FACE / 2, FACE / 2)
    _part(I, k['name'], mp.parallel_key(k['d'], IN_KEY_Y[0], IN_KEY_Y[1], k['b'], k['h'], k['t1'], x=0.0), steel)
    for side in ('A', 'B'):
        _part(I, 'Bearing %s %s inner ring' % (B['in']['name'], side), rings[('in', side)], steel)

    # ---- output shaft assembly
    shaft = mp.stepped_shaft(OUT_SECTIONS, x=A)
    kg, ko = K['gear'], K['out']
    mp.keyseat(shaft, kg['d'], GEAR_KEY_Y[0], GEAR_KEY_Y[1], kg['b'], kg['t1'], x=A)
    mp.keyseat(shaft, ko['d'], OUT_KEY_Y[0], OUT_KEY_Y[1], ko['b'], ko['t1'], x=A)
    _part(O, 'Output shaft', shaft, steel, (A, 0.0, 0.0))
    gear = _part(O, 'Gear z95', gear_blank(d), steel, (A, 0.0, 0.0))
    # z2 odd: with a tooth at 0 the gap is at 180 deg, facing the pinion's tooth at 0
    mp.spur_teeth(gear, gear.component.bRepBodies.item(0), M, Z2, PHI_DEG, BACKLASH, 0.0, -FACE / 2, FACE / 2)
    _part(O, kg['name'], mp.parallel_key(kg['d'], GEAR_KEY_Y[0], GEAR_KEY_Y[1], kg['b'], kg['h'], kg['t1'], x=A),
          steel, (A, 0.0, 0.0))
    _part(O, ko['name'], mp.parallel_key(ko['d'], OUT_KEY_Y[0], OUT_KEY_Y[1], ko['b'], ko['h'], ko['t1'], x=A),
          steel, (A, 0.0, 0.0))
    y0, y1, Dout, bore = SPACER
    _part(O, 'Spacer', mp.tube((A, y0, 0.0), (A, y1, 0.0), Dout / 2.0, bore / 2.0), steel, (A, 0.0, 0.0))
    for side in ('A', 'B'):
        _part(O, 'Bearing %s %s inner ring' % (B['out']['name'], side), rings[('out', side)], steel, (A, 0.0, 0.0))

    # ---- motion: each shaft assembly revolute about y on the housing, linked 1800 deg to 360 deg, reversed
    joints = []
    for occ, name, x in ((inp, 'Input shaft', 0.0), (out, 'Output shaft', A)):
        geo, direction = _plus_y_joint_geometry(H, housing, x)
        ji = root.asBuiltJoints.createInput(occ, housing, geo)
        ji.setAsRevoluteJointMotion(direction)
        j = root.asBuiltJoints.add(ji)
        j.name = name
        joints.append(j)
    li = root.motionLinks.createInput(joints[0], joints[1])
    try:
        li.motionOne = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
        li.motionTwo = adsk.fusion.JointMotionTypes.RevoluteJointRotateMotionType
    except Exception:
        pass
    li.valueOne = mp.VS('1800 deg')
    li.valueTwo = mp.VS('360 deg')
    li.isReversed = True
    root.motionLinks.add(li)

    # ---- the ratio, measured on the model: turn the input 18 deg and back
    axes = []
    for j in joints:
        try:
            v = adsk.fusion.RevoluteJointMotion.cast(j.jointMotion).rotationAxisVector
            axes.append('%+.0f %+.0f %+.0f' % (v.x, v.y, v.z))
        except Exception:
            axes.append('?')
    R_in0, R_out0 = _rotation(inp), _rotation(out)
    jm = joints[0].jointMotion
    jm.rotationValue = math.radians(18.0)
    adsk.doEvents()
    t_in, t_out = _turn_about_y(_rotation(inp), R_in0), _turn_about_y(_rotation(out), R_out0)
    jm.rotationValue = 0.0
    adsk.doEvents()
    return dict(axes=axes, t_in=t_in, t_out=t_out)


def _kind(name):
    """The parts-list kind of a component: the bearing's position letter (A, B) dropped, and any copy number."""
    name = re.sub(r' \(\d+\)$', '', name)
    return re.sub(r'^(Bearing \d{4}) [AB] ', r'\1 ', name)


def parts_list(root):
    """{kind: count} of every component that holds a body, counted from Fusion's occurrences."""
    out = {}
    for occ in root.allOccurrences:
        if occ.component.bRepBodies.count:
            k = _kind(occ.component.name)
            out[k] = out.get(k, 0) + 1
    return out


def message(d, parts, motion):
    g, s, B, K, i = d['gears'], d['shafts'], d['bearings'], d['keys'], d['inputs']
    sec = s['sections']
    lines = [
        VERSION,
        'Duty: %g kW at %g rpm, KA %g, bearing life %g h, design factor %g'
        % (i['P'], i['n1'], i['KA'], i['life'], i['nd']),
        '',
        'Gears m %g, z %d/%d, face %g mm, %g deg, a = %g mm: T1 %.3f N m, T2 %.3f N m; Ft %.1f N, Fr %.1f N, W %.1f N;'
        ' V %.3f m/s, Kv %.4f' % (g['m'], g['z1'], g['z2'], g['F'], g['phi_deg'], g['a'], g['T1'], g['T2'], g['Ft'],
                                 g['Fr'], g['W'], g['V'], g['Kv']),
        '  Lewis: pinion %.1f MPa (Y %.3f), gear %.1f MPa (Y %.4f); contact %.1f MPa'
        % (g['s1'], g['Y1'], g['s2'], g['Y2'], g['sc']),
        '  300 HB: St %.1f, Sc %.0f MPa; YN %.4f / %.4f, ZN %.4f / %.4f' % (g['St'], g['Sc'], g['YN1'], g['YN2'],
                                                                         g['ZN1'], g['ZN2']),
        "  pinion: bending factor %.3f, contact factor %.3f; gear: bending factor %.3f, contact factor %.3f"
        % (g['SF1'], g['SH1'], g['SF2'], g['SH2']),
        'Shafts, 1045 CD, DE-Goodman with Kt %.2f, Kts %.1f: input end (16) %.2f, gear seat (35) %.2f, output end (25) %.2f'
        % (s['Kt'], s['Kts'], s['n_in'], s['n_gear'], s['n_out']),
        '  slopes at the bearings: input %.5f rad, output %.5f rad (%g allowed)'
        % (s['theta1'], s['theta2'], SLOPE_ALLOWED),
        '  deflection under the pinion %.4f mm, under the gear %.4f mm (Table 7-2: %.3f mm)'
        % (s['y1'], s['y2'], s['y_allowed']),
        'Bearings: %s on %g mm, R %.1f N at %g rpm, C needed %.2f of %.1f kN, L10 %.0f h; %s on %g mm, C needed %.2f'
        ' of %.1f kN, L10 %.0f h' % (B['in']['name'], B['in']['d'], B['in']['R'], B['in']['n'], B['in']['C_needed'],
                                    B['in']['C10'], B['in']['L10h'], B['out']['name'], B['out']['d'],
                                    B['out']['C_needed'], B['out']['C10'], B['out']['L10h']),
        'Keys (%g MPa shear, %g MPa crushing allowed): %s' % (TAU_ALLOWED, SIGMA_ALLOWED, '; '.join(
            '%s on %g mm: shear %.1f, crushing %.1f MPa' % (K[k]['name'], K[k]['d'], K[k]['tau'], K[k]['sigma'])
            for k in ('in', 'gear', 'out'))),
        'Motion, measured: input turned %.3f deg, output %.3f deg (joint axes %s)'
        % (motion['t_in'], motion['t_out'], ', '.join(motion['axes'])),
        '',
        'Parts list, %d components in %d kinds:' % (sum(parts.values()), len(parts)),
    ]
    lines += ['  %3d  %s' % (parts[k], k) for k in sorted(parts)]
    return '\n'.join(lines)


def build(P=4.0, n1=1440.0, KA=1.25, life=20000.0, nd=2.0):
    """Size the reducer for the duty; refuse it (ValueError) or build it in a new design and return the message."""
    d = sizing(P, n1, KA, life, nd)
    if d['refusals']:
        raise ValueError('Refused: ' + '; '.join(d['refusals']))
    app = adsk.core.Application.get()
    app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    try:
        design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
    except Exception:
        pass
    root = design.rootComponent
    motion = build_model(root, d)
    try:
        app.activeViewport.fit()
    except Exception:
        pass
    return message(d, parts_list(root), motion)


# ---------------------------------------------------------------------------------------------------------------
# the dialog
# ---------------------------------------------------------------------------------------------------------------
_handlers = []
INPUTS = (('P', 'Power P (kW)'), ('n1', 'Input speed n1 (rpm)'), ('KA', 'Application factor KA'),
          ('life', 'Bearing life L10 (h)'), ('nd', 'Design factor nd'))


class _Execute(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ins = args.command.commandInputs
            duty = {k: ins.itemById(k).value for k, _ in INPUTS}
            ui.messageBox(build(**duty), 'Spur reducer')
        except ValueError as e:
            ui.messageBox('%s\n\n%s' % (e, VERSION), 'Spur reducer')
        except Exception:
            ui.messageBox('Failed:\n%s\n%s' % (traceback.format_exc(), VERSION), 'Spur reducer')


class _Destroy(adsk.core.CommandEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        if __name__ != 'mdfc_check':          # a check that loads this file keeps running after the dialog
            adsk.terminate()


class _Created(adsk.core.CommandCreatedEventHandler):
    def __init__(self):
        super().__init__()

    def notify(self, args):
        cmd = args.command
        cmd.okButtonText = 'Build'
        ins = cmd.commandInputs
        for k, label in INPUTS:
            ins.addValueInput(k, label, '', adsk.core.ValueInput.createByReal(DEFAULTS[k]))
        on_execute, on_destroy = _Execute(), _Destroy()
        cmd.execute.add(on_execute)
        cmd.destroy.add(on_destroy)
        _handlers.extend([on_execute, on_destroy])


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        cd = ui.commandDefinitions.itemById(CMD_ID)
        if cd:
            cd.deleteMe()
        cd = ui.commandDefinitions.addButtonDefinition(CMD_ID, 'Spur reducer', 'Sizes and builds a single-stage spur reducer')
        on_created = _Created()
        cd.commandCreated.add(on_created)
        _handlers.append(on_created)
        cd.execute()
        adsk.autoTerminate(False)
    except Exception:
        if ui:
            ui.messageBox('Failed:\n%s\n%s' % (traceback.format_exc(), VERSION))


def stop(context):
    pass
