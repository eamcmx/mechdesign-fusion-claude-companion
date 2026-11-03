"""Chapter 54: the rear axle in detail. The tapered bearings' shoulders and spacer against SKF's abutment dimensions,
the cups' bores closed to the cups, the bearings' load classes against their loads, the half shafts' splines to
ISO 14, and the case bearings' adjusters threaded.

Sources (the pages saved in References/Web pages, 3 November 2026):
  SKF product pages 32213, 32309, 32308: dimensions and abutment dimensions (SKF_32213_32309_32308_abutments_...)
  SKF, Seat tolerances for standard conditions: table 2 (shafts, radial roller bearings), table 4 (housings)
  RoyMech (n.d.), ISO straight sided spline (BS 5686:1986, ISO 14:1982): the light and medium series, the fits
  Shigley (Budynas & Nisbett, 2015): Table 8-1's footnote (d_r = d - 1.226869 p, d_p = d - 0.649519 p, p. 404),
    Eqs. 8-10 to 8-12 (p. 410), Eq. 11-18 to 11-20 (tapered pairs, as lib/axle_design.py)
Values marked 'assumed' are this chapter's, not a source's.
"""
import math
import re

import axle_design as ad

# SKF's abutment dimensions, mm: da max (shaft, the cone's back face), db min (shaft, its front face), Da min and
# Da max (housing, the cup's back face), Db min (housing, beside the cup's front face), Ca, Cb (space at the large and
# the small side faces), B and C (the cone's and the cup's widths)
SKF_ABUT = {'32213': dict(da=76.0, db=75.5, Da=(104.0, 111.5), Db=115.0, Ca=4.0, Cb=5.5, B=31.0, C=27.0, d1=90.3),
            '32309': dict(da=57.0, db=55.0, Da=(82.0, 92.0), Db=93.0, Ca=4.0, Cb=8.0, B=36.0, C=30.0, d1=71.1),
            '32308': dict(da=51.0, db=49.5, Da=(73.0, 82.5), Db=82.0, Ca=4.0, Cb=8.0, B=33.0, C=27.0, d1=62.95)}


def carrier_layout():
    """Chapter 29's layout along the pinion's axis (RearAxleCarrier.layout(), the same arithmetic), mm."""
    g = ad.bevel_geometry()
    A, B = ad.SKF_TRB['32309'], ad.SKF_TRB['32308']
    sd, cd = math.sin(math.radians(g['delta1'])), math.cos(math.radians(g['delta1']))
    heel = g['Ae'] * cd + g['hf'] * sd
    L = dict(collar=heel + 5.6)
    L['xA'] = L['collar'] + 1.0
    L['xB'] = L['xA'] + A[2] + 63.3 + B[2]
    L['cupA'] = (L['xA'] + A[2] - SKF_ABUT['32309']['C'], L['xA'] + A[2])
    L['cupB'] = (L['xB'] - B[2], L['xB'] - B[2] + SKF_ABUT['32308']['C'])
    L['spacer_v2'] = (L['xA'] + A[2], L['xB'] - B[2])
    L['coneA_front'] = L['xA'] + SKF_ABUT['32309']['B']
    L['coneB_front'] = L['xB'] - SKF_ABUT['32308']['B']
    L['spacer'] = (L['coneA_front'], L['coneB_front'])
    L['middle'] = (L['cupA'][1], L['cupB'][0])
    L['step'] = 0.5 * sum(L['middle'])
    return L


HUB_Y, C_T = 74.0, ad.SKF_TRB['32213'][2]
# Volume 2's diameters at each abutment (RearAxleCarrier.py) and the detail's
ABUTMENTS = [
    # what, bearing, kind ('da' shaft at the back face, 'db' shaft at the front face, 'Da' housing at the cup's back
    # face, 'Db' housing beside the cup's front face), Volume 2's diameter, the detail's
    ('the pinion\'s shim on cone A\'s back face', '32309', 'da', 56.0, 56.0),
    ('the spacer on cone A\'s front face', '32309', 'db', 56.0, 56.0),
    ('the nose\'s middle bore at cup A\'s back face', '32309', 'Da', 93.0, 88.0),
    ('the nose\'s bore beside cup A\'s front face', '32309', 'Db', 100.1, 100.0),
    ('the spacer on cone B\'s front face', '32308', 'db', 56.0, 56.0),
    ('the companion flange\'s hub on cone B\'s back face', '32308', 'da', 60.0, 50.0),
    ('the nose\'s middle bore at cup B\'s back face', '32308', 'Da', 93.0, 80.0),
    ('the nose\'s bore beside cup B\'s front face', '32308', 'Db', 90.1, 90.0),
    ('the case\'s shoulder on a cone C\'s back face', '32213', 'da', 80.0, 75.0),
    ('the adjuster\'s face on a cup C\'s back face', '32213', 'Da', 72.0, 106.0),
]
RELIEF = dict(flange=3.0, adjuster=6.0, case=2.0)   # mm deep, the reliefs on the flange's hub, the adjusters (over
#                                                     SKF's Cb = 5.5) and the case's ends (all assumed)


def abut_ok(kind, d, bearing, D_out=None):
    s = SKF_ABUT[bearing]
    if kind == 'da':
        return d <= s['da']
    if kind == 'db':
        return d >= s['db']
    if kind == 'Da':
        return s['Da'][0] <= d <= s['Da'][1] and (D_out is None or d < D_out)
    return d >= s['Db']


def abutments():
    out = []
    D_out = {'32309': 100.0, '32308': 90.0, '32213': 120.0}
    for what, brg, kind, v2, new in ABUTMENTS:
        s = SKF_ABUT[brg]
        band = {'da': (None, s['da']), 'db': (s['db'], None), 'Da': s['Da'], 'Db': (s['Db'], None)}[kind]
        out.append(dict(what=what, bearing=brg, kind=kind, v2=v2, new=new, band=band,
                        ok_v2=abut_ok(kind, v2, brg, D_out[brg]), ok=abut_ok(kind, new, brg, D_out[brg]),
                        none=kind == 'Da' and v2 >= D_out[brg]))
    return out


def spacer():
    """The spacer between the pinion's cones: Volume 2's from the bearings' total widths T, the detail's from the
    cones' own faces (their widths B)."""
    L = carrier_layout()
    v2, new = L['spacer_v2'], L['spacer']
    return dict(v2=v2, new=new, L_v2=v2[1] - v2[0], L=new[1] - new[0], gap_A=v2[0] - L['coneA_front'],
                gap_B=L['coneB_front'] - v2[1], TB_A=ad.SKF_TRB['32309'][2] - SKF_ABUT['32309']['B'],
                TB_B=ad.SKF_TRB['32308'][2] - SKF_ABUT['32308']['B'], layout=L)


# ---- the load classes ------------------------------------------------------------------------------------------------
ROLLER_TABLE2 = {'light': ((25, 'j6'), (60, 'k6'), (140, 'm6')),
                 'normal': ((30, 'k6'), (50, 'm5'), (65, 'n5'), (100, 'n6'), (280, 'p6')),
                 'heavy': ((50, None), (65, 'n5'), (85, 'n6'), (140, 'p6'))}     # SKF table 2 (no heavy row <= 50)


def table2(cls, d):
    for top, c in ROLLER_TABLE2[cls]:
        if d <= top:
            return c
    return None


def load_classes():
    """Each tapered bearing's equivalent load at cruise and its largest at a peak, against SKF's C, and the class
    SKF's table 2 gives the shaft seat for each; the class kept is the tighter of the two, the heavy row falling back
    to the normal one where the table has none (under 50 mm)."""
    b = ad.brief()
    g = ad.bevel_geometry()
    u = ad.gear_duty(b)
    d = ad.duty(b)
    pb = ad.pinion_bearings(g, u)
    cb = ad.case_bearings(g, u, d)
    rows = []
    order = ['k6', 'm5', 'm6', 'n5', 'n6', 'p6']
    for name, brg, cruise, peak in (('pinion head A', '32309', pb['cruise']['FeA'], max(pb['peak']['FeA'], pb['reverse']['FeA'])),
                                    ('pinion tail B', '32308', pb['cruise']['FeB'], max(pb['peak']['FeB'], pb['reverse']['FeB'])),
                                    ('case C1', '32213', cb['cruise']['FeC1'], cb['peak']['FeC1']),
                                    ('case C2', '32213', cb['cruise']['FeC2'], cb['peak']['FeC2'])):
        C = ad.SKF_TRB[brg][4] * 1000.0
        dd = ad.SKF_TRB[brg][0]
        cl = lambda P: 'light' if P <= 0.05 * C else ('normal' if P <= 0.1 * C else 'heavy')
        c_cr, c_pk = cl(cruise), cl(peak)
        s_cr, s_pk = table2(c_cr, dd) or table2('normal', dd), table2(c_pk, dd)
        s_pk = s_pk or table2('normal', dd)
        keep = max((s_cr, s_pk), key=lambda c: order.index(c))
        rows.append(dict(name=name, bearing=brg, d=dd, cruise=cruise / C, peak=peak / C, cls_cruise=c_cr, cls_peak=c_pk,
                         seat_cruise=s_cr, seat_peak=s_pk, seat=keep, ch51=table2('normal', dd)))
    return rows


# ---- the splines -------------------------------------------------------------------------------------------------------
ISO14 = {'light': {(8, 32): (36.0, 6.0), (8, 36): (40.0, 7.0)}, 'medium': {(8, 32): (38.0, 6.0), (8, 36): (42.0, 7.0)}}
FITS14 = dict(hub=dict(d='H7', D='H10', B='H9'), sliding=dict(d='f7', D='a11', B='d10'),
              close=dict(d='g7', D='a11', B='f9'), fixed=dict(d='h7', D='a11', B='h10'))
SHAFT_V2, SHAFT = (35.0, 40.0, 8, 7.0), (36.0, 40.0, 8, 7.0)
PINION = (32.0, 38.0, 8, 6.0)


def splines():
    """The half shafts' torsion at the splines' minor diameter at the design torque (Chapter 30's 16 T / pi d^3 and
    0.577 S_y), and their engagement in the side gears (axle_design.spline_engagement), with Volume 2's 8 x 35 x 40
    and ISO 14's 8 x 36 x 40; the pinion's 8 x 32 x 38 x 6 against ISO 14's medium series."""
    b = ad.brief()
    w = ad.wheel_end(b)
    T = w['tau_spline'] * math.pi * SHAFT_V2[0] ** 3 / 16.0
    tau = 16 * T / (math.pi * SHAFT[0] ** 3)
    Sy = 1140.0
    T_sh = ad.duty(b)['T_shaft'] * 1000.0
    y_hub = HUB_Y + C_T
    e2 = ad.spline_engagement(35.0, 115.0, y_hub, T_sh)
    e = ad.spline_engagement(35.0, 115.0, y_hub, T_sh, d_minor=SHAFT[0], d_major=SHAFT[1])
    return dict(T=T, tau_v2=w['tau_spline'], n_v2=w['n_spline'], tau=tau, n=0.577 * Sy / tau, eng_v2=e2, eng=e,
                light=ISO14['light'][(8, 36)], pinion_iso=ISO14['medium'][(8, 32)] == (PINION[1], PINION[3]))


# ---- the adjusters ----------------------------------------------------------------------------------------------------
ADJ = dict(d=120.0, p=2.0, L=16.0, cls=('6H', '6g'))          # M120 x 2; the pitch and the classes assumed


def adjuster():
    """The adjuster's thread, M120 x 2: Shigley's minor and pitch diameters (Table 8-1's footnote), the threads
    engaged over its 16 mm, and the stresses of Eqs. 8-10 to 8-12 with the case bearing's axial load at the peak,
    taken on one thread (the first thread carries the most); C1's axial load by Eqs. 11-18 to 11-20."""
    b = ad.brief()
    g = ad.bevel_geometry()
    u = ad.gear_duty(b)
    d_ = ad.duty(b)
    cb = ad.case_bearings(g, u, d_)
    pk = cb['peak']
    K1 = K2 = ad.SKF_TRB['32213'][7]
    Fi1, Fi2 = 0.47 * pk['R1'] / K1, 0.47 * pk['R2'] / K2
    Fa1 = max(Fi1, Fi2 + pk['Fae'])
    d, p = ADJ['d'], ADJ['p']
    dr, dp = d - 1.226869 * p, d - 0.649519 * p
    nt = ADJ['L'] / p
    out = dict(dr=dr, dp=dp, nt=nt, Fa=Fa1, Fi1=Fi1, Fi2=Fi2, Fae=pk['Fae'])
    for k, n in (('one', 1.0), ('all', nt)):
        out['sigB_' + k] = 2 * Fa1 / (math.pi * dp * n * p)
        out['sigb_' + k] = 6 * Fa1 / (math.pi * dr * n * p)
        out['tau_' + k] = 3 * Fa1 / (math.pi * dr * n * p)
    out['turn'] = p                                               # mm of axial travel a turn
    return out


if __name__ == '__main__':
    for r in abutments():
        print(r)
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in spacer().items() if k != 'layout'})
    for r in load_classes():
        print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in r.items()})
    s = splines()
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in s.items() if not k.startswith('eng')})
    print('eng v2', {k: round(v, 3) for k, v in s['eng_v2'].items()})
    print('eng', {k: round(v, 3) for k, v in s['eng'].items()})
    print({k: round(v, 3) for k, v in adjuster().items()})


# ---- the pinion's stack and its nut ---------------------------------------------------------------------------------
def pinion_stack(K=0.2, d=30.0, At=561.0):
    """The stack the pinion nut clamps: flange, cone B, spacer, cone A and shim against the pinion's collar. A thrust
    that pulls the pinion either way unloads the part of the stack the bearing does not carry, so the nut's clamp must
    exceed the larger of the two cones' axial loads at the peaks (Shigley Eqs. 11-18 to 11-20); the torque by
    T = K F d (Eq. 8-27, K = 0.2 for zinc-plated threads, Table 8-15), the stress on a coarse M30's 561 mm2 (Table 8-1)."""
    b = ad.brief()
    g = ad.bevel_geometry()
    u = ad.gear_duty(b)
    pb = ad.pinion_bearings(g, u)
    KA, KB = ad.SKF_TRB['32309'][7], ad.SKF_TRB['32308'][7]
    out = {}
    for nm in ('peak', 'reverse'):
        r = pb[nm]
        FiA, FiB, Fae = 0.47 * r['RA'] / KA, 0.47 * r['RB'] / KB, r['Fae']
        if Fae >= 0:
            out[nm] = dict(FiA=FiA, FiB=FiB, Fae=Fae, Fa=max(FiA, FiB + Fae), on='A')
        else:
            out[nm] = dict(FiA=FiA, FiB=FiB, Fae=Fae, Fa=max(FiB, FiA - Fae), on='B')
    F = max(out['peak']['Fa'], out['reverse']['Fa'])
    out.update(F=F, T=K * F * d / 1000.0, sigma=F / At, K=K)
    return out


CONE_FITS = {'m5 30-50': (-32, -9, -29, -12), 'n5 50-80': (-48, -20, -44, -24)}   # SKF table 4, um: theoretical,
#                                                                                 then probable (minus: interference)


# ---- the new faces' pressures, and the cast iron's thread --------------------------------------------------------------
CAST_IRON_20 = dict(Sut=22 * 6.894757, Suc=83 * 6.894757, Ssu=26 * 6.894757)    # ASTM No. 20, Shigley Table A-24 (kpsi
#                                                                                 to MPa); the carrier's grade not named,
#                                                                                 so the weakest of the table


def pressures():
    """The axial loads on the faces the detail made, each on its annulus: the cups' shoulders, cone B on the flange's
    relieved hub (inside the spline's 38 mm major diameter), cone C1 on the case's relieved end (outside its 65 mm
    seat), the adjuster's face on cup C1; the internal thread's shear in the cast-iron leg at its major diameter."""
    st = pinion_stack()
    a = adjuster()
    ann = lambda D, d: math.pi / 4 * (D ** 2 - d ** 2)
    rows = [('cup A on its shoulder', st['peak']['Fa'], 100.0, 88.0),
            ('cup B on its shoulder', st['reverse']['Fa'], 90.0, 80.0),
            ('cone B on the flange\'s hub', st['reverse']['Fa'], 50.0, 38.0),
            ('cone C1 on the case\'s end', a['Fa'], 75.0, 65.0),
            ('the adjuster on cup C1', a['Fa'], 120.0, 106.0)]
    out = [dict(what=w, F=F, D=D, d=d, A=ann(D, d), p=F / ann(D, d)) for w, F, D, d in rows]
    tau1 = 3 * a['Fa'] / (math.pi * ADJ['d'] * 1 * ADJ['p'])
    return dict(rows=out, tau_cast_one=tau1, tau_cast_all=tau1 / a['nt'], n_cast=CAST_IRON_20['Ssu'] / tau1,
                cast=CAST_IRON_20)


def spline_fits():
    """ISO 14's fits in micrometres from Shigley's Tables A-11 and A-12 (f: -25 at 30-40 mm, d: -40 at 6-10 mm; IT7 25
    at 30-50; IT9 36 and IT10 58 at 6-10 for the half shafts' 7 mm widths, IT9 30 and IT10 48 at 3-6 for the pinion's
    6): the minor diameter and the widths, the half shafts' sliding fit and the pinion's fixed one."""
    c = lambda hole, shaft: (hole[0] - shaft[1], hole[1] - shaft[0])
    return dict(minor_sliding=c((0, 25), (-50, -25)), width_sliding=c((0, 36), (-98, -40)),
                minor_fixed=c((0, 25), (-25, 0)), width_fixed=c((0, 30), (-48, 0)))
