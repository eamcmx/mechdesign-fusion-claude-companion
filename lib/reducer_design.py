# =============================================================================
# reducer_design.py  -  the sizing of Chapter 27's single-stage spur reducer, along its load path, with the data of
# Budynas and Nisbett (2015), Shigley's Mechanical Engineering Design, 10th edition. No Fusion here: the numbers that
# decide the parts, worked in the order of the chapter, so that a script and a check can both call them.
#
#   brief()      the requirements
#   gears()      torques, the mesh, Lewis bending (Eq. 14-8, Kv of Eq. 14-6b, Y of Table 14-2), contact (Eq. 14-14),
#                the strengths of 300 HB steel (Figs. 14-2, 14-5) for the life's cycles (Figs. 14-14, 14-15)
#   shafts()     free-body loads, the DE-Goodman diameter (Eq. 7-8) at the critical sections, the chosen standard
#                diameters and the factor of safety they give, slopes and deflection against Table 7-2
#   bearings()   the radial loads, the rating needed for the life, the bearings of Table 11-2 chosen by the seats
#   keys()       DIN 6885 keys (Volume 1, Chapter 6) at the three hubs, their shear and crushing stresses
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import math

SUT, SY, E = 630.0, 530.0, 207000.0                  # 1045 CD, Table A-20 (MPa); steel's modulus (MPa)
CP = 191.0                                           # steel on steel, Table 14-8 (sqrt MPa)
LEWIS = [(12, 0.245), (13, 0.261), (14, 0.277), (15, 0.290), (16, 0.296), (17, 0.303), (18, 0.309), (19, 0.314),
         (20, 0.322), (21, 0.328), (22, 0.331), (24, 0.337), (26, 0.346), (28, 0.353), (30, 0.359), (34, 0.371),
         (38, 0.384), (43, 0.397), (50, 0.409), (60, 0.422), (75, 0.435), (100, 0.447), (150, 0.460), (300, 0.472),
         (400, 0.480)]                               # Table 14-2
# Table 11-2, 02 series deep-groove: bore: (OD, width, fillet r, shoulder dS, housing dH, C10 kN, C0 kN)
BRG = {15: (35, 11, 0.6, 17.5, 31, 7.80, 3.55), 17: (40, 12, 0.6, 19.5, 34, 9.56, 4.50),
       20: (47, 14, 1.0, 25, 41, 12.7, 6.20), 25: (52, 15, 1.0, 30, 47, 14.0, 6.95),
       30: (62, 16, 1.0, 35, 55, 19.5, 10.0), 35: (72, 17, 1.0, 41, 65, 25.5, 13.7),
       40: (80, 18, 1.0, 46, 72, 30.7, 16.6), 45: (85, 19, 1.0, 52, 77, 33.2, 18.6),
       50: (90, 20, 1.0, 56, 82, 35.1, 19.6)}
# DIN 6885 parallel keys: (D above, D up to, b, h, t1, t2), mm (as in Volume 1, Chapter 6)
DIN6885 = [(10, 12, 4, 4, 2.5, 1.8), (12, 17, 5, 5, 3.0, 2.3), (17, 22, 6, 6, 3.5, 2.8), (22, 30, 8, 7, 4.0, 3.3),
           (30, 38, 10, 8, 5.0, 3.3), (38, 44, 12, 8, 5.0, 3.3), (44, 50, 14, 9, 5.5, 3.8), (50, 58, 16, 10, 6.0, 4.3)]
KT_KEY, KTS_KEY = 2.14, 3.0                           # end-mill keyseat, Table 7-1
KT_SH, KTS_SH = 1.7, 1.5                              # well-rounded shoulder fillet, Table 7-1


def brief(P=4.0, n1=1440.0, i=5.0, KA=1.25, life=20000.0, nd=2.0):
    """The requirements: power (kW), input speed (rpm), ratio, application factor, L10 life (h), design factor."""
    return dict(P=P, n1=n1, i=i, KA=KA, life=life, nd=nd)


def lewis_y(z):
    for (z0, y0), (z1, y1) in zip(LEWIS, LEWIS[1:]):
        if z0 <= z <= z1:
            return y0 + (y1 - y0) * (z - z0) / (z1 - z0)
    raise ValueError('%d teeth: outside Table 14-2' % z)


def gears(b, z1=19, z2=95, m=3.0, face=30.0, alpha=20.0, eta=0.98, HB=300.0):
    a = math.radians(alpha)
    g = dict(z1=z1, z2=z2, m=m, face=face, alpha=alpha, eta=eta, ratio=z2 / z1)
    g['d1'], g['d2'] = m * z1, m * z2
    g['a'] = m * (z1 + z2) / 2
    g['n2'] = b['n1'] / g['ratio']
    g['T1'] = b['P'] * 1000.0 / (b['n1'] * 2 * math.pi / 60)                 # N m
    g['T2'] = g['T1'] * g['ratio'] * eta
    g['Ft'] = b['KA'] * 2 * g['T1'] * 1000.0 / g['d1']                       # N, the design load (KA applied)
    g['Fr'] = g['Ft'] * math.tan(a)
    g['W'] = g['Ft'] / math.cos(a)
    g['V'] = math.pi * g['d1'] / 1000.0 * b['n1'] / 60                         # m/s
    g['Kv'] = (6.1 + g['V']) / 6.1                                             # Eq. 14-6b, cut or milled
    g['Y1'], g['Y2'] = lewis_y(z1), lewis_y(z2)
    g['s1'] = g['Kv'] * g['Ft'] / (face * m * g['Y1'])                         # Eq. 14-8, MPa
    g['s2'] = g['Kv'] * g['Ft'] / (face * m * g['Y2'])
    g['r1'], g['r2'] = g['d1'] / 2 * math.sin(a), g['d2'] / 2 * math.sin(a)
    g['sc'] = CP * math.sqrt(g['Kv'] * g['Ft'] / (face * math.cos(a)) * (1 / g['r1'] + 1 / g['r2']))   # Eq. 14-14
    # strengths of through-hardened steel, grade 1 (Figs. 14-2 and 14-5), at 10^7 cycles; for the cycles of the
    # life, the lower curves of the stress-cycle factors (Figs. 14-14 and 14-15)
    g['HB'] = HB
    g['St'], g['Sc'] = 0.533 * HB + 88.3, 2.22 * HB + 200.0
    g['N1'] = 60.0 * b['n1'] * b['life']
    g['N2'] = g['N1'] / g['ratio']
    for k, N in (('1', g['N1']), ('2', g['N2'])):
        g['YN' + k] = 1.6831 * N ** -0.0323
        g['ZN' + k] = 2.466 * N ** -0.056
    g['SF1'] = g['St'] * g['YN1'] / g['s1']                                    # bending, on stress
    g['SF2'] = g['St'] * g['YN2'] / g['s2']
    g['SH1'] = g['Sc'] * g['ZN1'] / g['sc']                                    # contact, on stress
    g['SH2'] = g['Sc'] * g['ZN2'] / g['sc']
    return g


def de_goodman(nd, Ma, Tm, Kf, Kfs, d_guess=30.0):
    """The DE-Goodman diameter (Eq. 7-8) for rotating bending Ma and steady torque Tm (N mm), iterating the size
    factor kb (Eq. 6-20) on the diameter."""
    ka = 4.51 * SUT ** -0.265                                                  # Eq. 6-19, machined (Table 6-2)
    d = d_guess
    for _ in range(50):
        kb = 1.24 * d ** -0.107                                                # Eq. 6-20, 2.79 to 51 mm
        Se = ka * kb * 0.5 * SUT                                               # Eq. 6-8 and 6-18
        d_new = (16 * nd / math.pi * (2 * Kf * Ma / Se + math.sqrt(3) * Kfs * Tm / SUT)) ** (1 / 3)
        if abs(d_new - d) < 1e-9:
            break
        d = d_new
    return d, dict(ka=ka, kb=kb, Se=Se)


def factor(d, Ma, Tm, Kf, Kfs):
    """The DE-Goodman factor of safety (Eq. 7-7) of a section of diameter d."""
    ka = 4.51 * SUT ** -0.265
    Se = ka * 1.24 * d ** -0.107 * 0.5 * SUT
    return 1.0 / (16 / (math.pi * d ** 3) * (2 * Kf * Ma / Se + math.sqrt(3) * Kfs * Tm / SUT))


def shafts(b, g, span1=98.0, span2=100.0, d_in=16.0, d_seat1=20.0, d_out=25.0, d_seat2=30.0, d_gear=35.0):
    """Both shafts carry the mesh force W at mid-span between their bearings; the input torque enters at the
    coupling end, the output torque leaves at the other."""
    s = dict(span1=span1, span2=span2)
    T1d, T2d = b['KA'] * g['T1'] * 1000.0, b['KA'] * g['T2'] * 1000.0          # N mm
    s['R1'], s['R2'] = g['W'] / 2, g['W'] / 2                                  # each bearing
    s['M1'], s['M2'] = g['W'] * span1 / 4, g['W'] * span2 / 4                   # N mm, under the gears
    nd = b['nd']
    # input: the coupling end (torque, keyseat); output: the gear seat (bending and torque, keyseat) and the end
    s['dmin_in'], _ = de_goodman(nd, 0.0, T1d, KT_KEY, KTS_KEY)
    s['dmin_gear'], s['marin'] = de_goodman(nd, s['M2'], T2d, KT_KEY, KTS_KEY)
    s['dmin_out'], _ = de_goodman(nd, 0.0, T2d, KT_KEY, KTS_KEY)
    s['d'] = dict(d_in=d_in, d_seat1=d_seat1, d_out=d_out, d_seat2=d_seat2, d_gear=d_gear)
    s['n_in'] = factor(d_in, 0.0, T1d, KT_KEY, KTS_KEY)
    s['n_gear'] = factor(d_gear, s['M2'], T2d, KT_KEY, KTS_KEY)
    s['n_out'] = factor(d_out, 0.0, T2d, KT_KEY, KTS_KEY)
    # slopes at the bearings and deflection under the gear, the seat diameter all along (on the safe side)
    for k, L, d in (('1', span1, d_seat1), ('2', span2, d_seat2)):
        I = math.pi * d ** 4 / 64
        s['theta' + k] = g['W'] * L ** 2 / (16 * E * I)                        # rad
        s['y' + k] = g['W'] * L ** 3 / (48 * E * I)                             # mm
    s['P_teeth_in'] = 25.4 / g['m']                                            # diametral pitch, teeth per inch
    s['y_allow'] = 0.254 if s['P_teeth_in'] < 10 else (0.127 if s['P_teeth_in'] < 19 else 0.0762)   # Table 7-2
    return s


def bearings(b, g, s):
    out = {}
    for k, n, d in (('in', b['n1'], s['d']['d_seat1']), ('out', g['n2'], s['d']['d_seat2'])):
        R = s['R1'] if k == 'in' else s['R2']
        C_req = R * (60 * n * b['life'] / 1e6) ** (1 / 3) / 1000.0           # kN, Eq. 11-3 with a = 3
        D, B, r, dS, dH, C10, C0 = BRG[int(d)]
        L10h = (C10 * 1000.0 / R) ** 3 * 1e6 / (60 * n)
        out[k] = dict(name='62%02d' % (int(d) // 5), d=d, D=D, B=B, r=r, dS=dS, dH=dH, C10=C10, C0=C0, R=R, n=n,
                      C_req=C_req, L10h=L10h)
    return out


def key_for(D):
    for lo, hi, kb, h, t1, t2 in DIN6885:
        if lo < D <= hi:
            return kb, h, t1, t2
    raise ValueError('a shaft of %.1f mm is outside the DIN 6885 table' % D)


def keys(b, g, s, L_in=28.0, L_gear=42.0, L_out=52.0, tau_allow=60.0, sigma_allow=100.0):
    out = {}
    for k, D, T, L in (('in', s['d']['d_in'], g['T1'], L_in), ('gear', s['d']['d_gear'], g['T2'], L_gear),
                       ('out', s['d']['d_out'], g['T2'], L_out)):
        kb, h, t1, t2 = key_for(D)
        F = 2 * b['KA'] * T * 1000.0 / D
        Lw = L - kb                                                            # form A: the round ends do not bear
        out[k] = dict(D=D, b=kb, h=h, t1=t1, t2=t2, L=L, F=F, tau=F / (kb * Lw), sigma=F / ((h - t1) * Lw),
                      ok=F / (kb * Lw) <= tau_allow and F / ((h - t1) * Lw) <= sigma_allow)
    return out


def design(**kw):
    b = brief(**{k: v for k, v in kw.items() if k in ('P', 'n1', 'i', 'KA', 'life', 'nd')})
    g = gears(b)
    s = shafts(b, g)
    return dict(brief=b, gears=g, shafts=s, bearings=bearings(b, g, s), keys=keys(b, g, s))


if __name__ == '__main__':
    r = design()
    for part in ('gears', 'shafts', 'bearings', 'keys'):
        print(part)
        for k, v in r[part].items():
            print('   %-10s %s' % (k, v))
