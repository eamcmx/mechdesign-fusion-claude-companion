"""Chapter 60: fatigue of the notches the detail added at the bag gripper's ends, where the jaws swing twice and strike
their stops twice in every cycle, 600 cycles an hour over the brief's 20 000 hours: the stop brackets' M8 bolts, the
brackets' roots, the long stub shaft's keyseat under the cylinder's torque, and the short stub's circlip groove.

Sources (Budynas & Nisbett, 2015, Shigley's Mechanical Engineering Design, 10th ed.):
  Eq. 6-8 (p. 290): S'e = 0.5 Sut for Sut <= 1400 MPa
  Eq. 6-19 and Table 6-2 (pp. 295-296): ka = a Sut^b, machined a 4.51, b -0.265
  Eq. 6-20 (p. 296): kb = 1.24 d^-0.107 for 2.79 <= d <= 51 mm
  Eq. 6-25 (p. 297): a rectangle h x b in bending, not rotating: de = 0.808 (h b)^0.5
  Table 6-5 (p. 301): ke = 0.814 at 99 % reliability
  Eqs. 6-33 to 6-35 (p. 304): Kf = 1 + q (Kt - 1), q = 1 / (1 + sqrt(a)/sqrt(r)), Neuber's sqrt(a) for steel
  Eq. 6-41 (p. 314): modified Goodman, 1/n = sa/Se + sm/Sut
  Table 7-1 (p. 365): first-iteration Kt and Kts: shoulder fillet, sharp (r/d 0.02) 2.7, well rounded (r/d 0.1) 1.7;
    end-mill keyseat (r/d 0.02) 2.14 in bending, 3.0 in torsion; retaining-ring groove 5.0 and 3.0
  Eq. 8-17 (p. 419): kb = Ad At E / (Ad lt + At ld); Eq. 8-20 (p. 421): a frustum's stiffness
  Eqs. 8-35 to 8-38 and Table 8-17 (pp. 437-438): a preloaded bolt in fatigue; ISO 8.8 Se = 129 MPa
  Table A-5 (p. 1018): E of carbon steel 207 GPa, of aluminium alloys 71.7 GPa
  Table A-20 (p. 1048): 1045 HR Sut 570, Sy 310; 1020 CD Sut 470, Sy 390 MPa
Fasten.it after DIN EN ISO 4762: M8's thread length b 28 mm; ISO 898-1: class 8.8 at d <= 16, Rm 800 MPa.
"""
import math

KPSI = 6.894757
E_STEEL, E_AL = 207000.0, 71700.0                  # MPa
SUT_88 = 800.0                                     # MPa, ISO 898-1, class 8.8, d <= 16
SE_88 = 129.0                                      # MPa, Table 8-17, ISO 8.8 (its M16 to M36 row)
SE_98 = 140.0                                      # MPa, Table 8-17, ISO 9.8, M1.6 to M16
B_4762_M8 = 28.0                                   # mm, ISO 4762 M8's thread length b
STEEL_1045_HR = dict(Sut=570.0, Sy=310.0)
STEEL_1020_CD = dict(Sut=470.0, Sy=390.0)
KE_99 = 0.814
CYCLES_H, LIFE_H = 600.0, 20000.0
R_ROOT = 2.5                                       # mm, the brackets' root radius the drawing gives (this chapter)


def ka_machined(Sut):
    return 4.51 * Sut ** -0.265


def kb_d(d):
    return 1.24 * d ** -0.107


def sqrt_a(Sut, torsion=False):
    """Neuber's constant, sqrt(in), from Sut in MPa (Eq. 6-35a or b, kpsi inside)."""
    s = Sut / KPSI
    if torsion:
        return 0.190 - 2.51e-3 * s + 1.35e-5 * s ** 2 - 2.67e-8 * s ** 3
    return 0.246 - 3.08e-3 * s + 1.51e-5 * s ** 2 - 2.67e-8 * s ** 3


def q_notch(r_mm, Sut, torsion=False):
    return 1.0 / (1.0 + sqrt_a(Sut, torsion) / math.sqrt(r_mm / 25.4))


def cycles():
    return CYCLES_H * LIFE_H


# ---- the stop brackets' bolts -----------------------------------------------------------------------------------------
def frustum(E, t, D, d):
    """Eq. 8-20: a 30 deg frustum of thickness t, its smaller diameter D, round a hole for a bolt of diameter d."""
    return 0.5774 * math.pi * E * d / math.log((1.155 * t + D - d) * (D + d) / ((1.155 * t + D + d) * (D - d)))


def bolt_joint():
    """The bracket's M8 x 40 through the bracket's 12 mm steel foot, the 15 mm aluminium end plate and a 1.6 mm steel
    washer under the nut: the bolt's stiffness (Eq. 8-17, the thread length b of ISO 4762), the members' as four
    frusta meeting at the grip's middle, their washer faces 1.5 d (p. 421), and the joint constant C."""
    import range_sweep as RS
    import bag_gripper as BG
    B = RS.BRACKET
    d = B['bolt']
    t_plate = BG.LAY['plate_x'][1] - BG.LAY['plate_x'][0]
    l = B['foot'] + t_plate + B['washer'][2]
    L = 40.0
    ld = L - B_4762_M8
    lt = l - ld
    Ad = math.pi / 4 * d ** 2
    At = B['At']
    kb = Ad * At * E_STEEL / (Ad * lt + At * ld)
    D = 1.5 * d
    half = l / 2.0
    t30 = math.tan(math.radians(30.0))
    # from the head: steel foot, then aluminium to the middle; from the nut: steel washer, then aluminium
    parts = [('the foot, steel', E_STEEL, B['foot'], D),
             ('the plate, aluminium, head side', E_AL, half - B['foot'], D + 2 * B['foot'] * t30),
             ('the plate, aluminium, nut side', E_AL, half - B['washer'][2], D + 2 * B['washer'][2] * t30),
             ('the washer, steel', E_STEEL, B['washer'][2], D)]
    ks = [(nm, E, t, Dm, frustum(E, t, Dm, d)) for nm, E, t, Dm in parts]
    km = 1.0 / sum(1.0 / k for *_, k in ks)
    C = kb / (kb + km)
    return dict(d=d, l=l, L=L, ld=ld, lt=lt, Ad=Ad, At=At, kb=kb, km=km, C=C, frusta=ks, t_plate=t_plate, D=D)


def bolt_fatigue():
    """Each stop puts its push P on the worst bolt once a cycle, from 0 to P, over the preload (Eqs. 8-35 to 8-38):
    sa = C P / (2 At), sm = sa + Fi / At, and Goodman's factor against the bolt's fully corrected endurance strength."""
    import range_sweep as RS
    j = bolt_joint()
    bb = RS.bracket_bolts()
    Fi = bb['Fi']
    out = dict(joint=j, Fi=Fi, si=Fi / j['At'])
    for k in ('closed', 'open'):
        P = bb[k]['F_bolt']
        sa = j['C'] * P / (2 * j['At'])
        sm = sa + out['si']
        nf = SE_88 * (SUT_88 - out['si']) / (SUT_88 * sa + SE_88 * (sm - out['si']))
        out[k] = dict(P=P, sa=sa, sm=sm, nf=nf, share=j['C'] * P)
    return out


# ---- the brackets' roots ----------------------------------------------------------------------------------------------
def bracket_roots(r=R_ROOT):
    """Each bracket's section where it leaves its foot, struck once a cycle from nothing to the stop's push: repeated
    bending, sa = sm = Kf s / 2. Kt from Table 7-1's shoulder fillets as a first estimate, sharp at r/d 0.02 and well
    rounded at r/d 0.1, the drawing's root radius r giving q (Eq. 6-34); Se with the machined surface, de of the
    section (Eq. 6-25) and 99 % reliability; Goodman (Eq. 6-41)."""
    import range_sweep as RS
    st = RS.bracket_strength()
    m = STEEL_1045_HR
    Se_p = 0.5 * m['Sut']
    ka = ka_machined(m['Sut'])
    out = dict(Se_p=Se_p, ka=ka, r=r)
    for k, depth, h, b in (('closed', st['closed']['t'], st['closed']['b'], st['closed']['t']),
                           ('open', st['open']['a'], st['open']['a'], st['open']['a'])):
        de = 0.808 * math.sqrt(h * b)
        kb = kb_d(de)
        Se = ka * kb * KE_99 * Se_p
        s = st[k]['sigma']
        rows = {}
        for tag, Kt, rr in (('sharp', 2.7, 0.02 * depth), ('drawn', 1.7, r)):
            q = q_notch(rr, m['Sut'])
            Kf = 1 + q * (Kt - 1)
            sa = sm = Kf * s / 2.0
            n = 1.0 / (sa / Se + sm / m['Sut'])
            rows[tag] = dict(Kt=Kt, r=rr, q=q, Kf=Kf, sa=sa, n=n)
        out[k] = dict(sigma=s, depth=depth, de=de, kb=kb, Se=Se, **rows)
    return out


# ---- the long stub's keyseat ------------------------------------------------------------------------------------------
KEY = dict(b=8.0, h=7.0, L=25.0, t1=4.0)            # DIN 6885 A 8 x 7 on a 30 mm shaft: t1 4.0 (Fasten.it)


def keyseat():
    """The cylinder's torque on the jaw, through the lever and its key into the long stub, each way once a cycle:
    fully reversed torsion on the 30 mm stub, tau = 16 T / (pi d^3), Kts 3.0 for an end-mill keyseat (Table 7-1),
    r/d 0.02, q in torsion (Eq. 6-35b); Sse with kc 0.59 and 99 % reliability; and the bending the lever's force
    adds at the keyseat's inner end, 10 mm from its middle."""
    import bag_gripper as BG
    des = BG.design()
    m = STEEL_1020_CD
    d = BG.LAY['stub_d']
    T = des['jaw_cycle']['M'] * 1000.0
    tau = 16 * T / (math.pi * d ** 3)
    r = 0.02 * d
    qs = q_notch(r, m['Sut'], torsion=True)
    Kfs = 1 + qs * (3.0 - 1)
    ta = Kfs * tau
    ka, kb = ka_machined(m['Sut']), kb_d(d)
    Sse = ka * kb * 0.59 * KE_99 * 0.5 * m['Sut']
    F = T / BG.LAY['lever']
    lev = BG.LAY['lever_x']
    M = F * ((lev[0] + lev[1]) / 2.0 - lev[0])
    sb = 32 * M / (math.pi * d ** 3)
    return dict(d=d, T=T / 1000.0, tau=tau, r=r, qs=qs, Kfs=Kfs, ta=ta, ka=ka, kb=kb, Sse=Sse, nf=Sse / ta, F=F, M=M / 1000.0,
                sb=sb, T_stop=des['jaw_stop']['M'], tau_stop=16 * des['jaw_stop']['M'] * 1000.0 / (math.pi * d ** 3))


# ---- the short stub's groove ------------------------------------------------------------------------------------------
def groove():
    """The short stub's DIN 471 30 groove sits beyond its bearing, where nothing is carried: the jaw's load reaches the
    stub through the tube's end and leaves it through the bearing, so the moment at the groove is the circlip's own
    axial load's, nil in service; Table 7-1's Kt 5.0 multiplies no stress."""
    import palletizer_detail as PD
    import bag_gripper as BG
    e = PD.jaw_ends()['short']
    x0, x1 = BG.LAY['bearing_x']
    return dict(bearing=(x0, x1), groove=e['groove'], beyond=e['groove'][0] >= x1, d2=e['d2'], Kt=5.0)



def no_preload():
    """The closing bracket's worse bolt with no preload at all: the whole pull on the bolt, from nothing to P, sa = sm
    = P / (2 At), Goodman against the same Se and Sut; and the load factor against the joint's separation with the
    preload, n0 = Fi / (P (1 - C)) (Eq. 8-30, p. 433)."""
    b = bolt_fatigue()
    j, P = b['joint'], b['closed']['P']
    sa = sm = P / (2 * j['At'])
    nf = 1.0 / (sa / SE_88 + sm / SUT_88)
    n0 = b['Fi'] / (P * (1 - j['C']))
    return dict(sa=sa, sm=sm, nf=nf, n0=n0)


def sn_line(Sut=None, Se=None):
    """Shigley's S-N line for a steel part, Eqs. 6-10 to 6-14 (pp. 292-293): the fatigue strength fraction f at 10^3
    cycles from sigma'_F = Sut + 50 kpsi, b' = -log(sigma'_F / S'e) / log(2 Ne), f = sigma'_F / Sut (2 10^3)^b'; then
    Sf = a N^b with a = (f Sut)^2 / Se and b = -log(f Sut / Se) / 3, flat at Se beyond Ne = 10^6. For the opening
    bracket's arm unless given; with its drawn root's stresses as the fully reversed stress on the same Goodman line,
    sar = sa / (1 - sm / Sut)."""
    r = bracket_roots()
    m = STEEL_1045_HR
    Sut = Sut or m['Sut']
    Se = Se or r['open']['Se']
    s = Sut / KPSI
    sF = s + 50.0
    Se_p = 0.5 * s
    bp = -math.log10(sF / Se_p) / math.log10(2 * 1e6)
    f = sF / s * (2 * 1e3) ** bp
    a = (f * Sut) ** 2 / Se
    b = -math.log10(f * Sut / Se) / 3.0
    sa = r['open']['drawn']['sa']
    sar = sa / (1 - sa / Sut)
    return dict(Sut=Sut, Se=Se, sF=sF * KPSI, bp=bp, f=f, a=a, b=b, S1e3=f * Sut, sar=sar, N=cycles())



K_ZINC = 0.2              # the torque coefficient for zinc-plated threads, Table 8-15 (p. 430), as Chapter 54 took it


def tightening():
    """The torque that gives the stop bolts their preload, T = K Fi d (Eq. 8-27, p. 430)."""
    import range_sweep as RS
    Fi = RS.bracket_bolts()['Fi']
    d = RS.BRACKET['bolt']
    return dict(Fi=Fi, d=d, K=K_ZINC, T=K_ZINC * Fi * d / 1000.0)



AL_SF_MIN = 55.0          # MPa: the lowest fatigue strength of the wrought alloys in Table A-24(b) (p. 1055), 3003-H12,
#                           at 5 10^8 cycles; the table has no row for 6061, and aluminium has no endurance limit


def plate_under():
    """The aluminium end plate under the closing bracket's worse bolt's washer: the preload's bearing stress on the
    washer's face, and how much of it each strike takes away, the members' share of the pull, (1 - C) P."""
    import range_sweep as RS
    b = bolt_fatigue()
    j = b['joint']
    dw, Dw, tw = RS.BRACKET['washer']
    A = math.pi / 4 * (Dw ** 2 - dw ** 2)
    P = b['closed']['P']
    relief = (1 - j['C']) * P
    s0 = b['Fi'] / A
    ds = relief / A
    return dict(A=A, s0=s0, relief=relief, ds=ds, sa=ds / 2, sm=-(s0 - ds / 2), n=AL_SF_MIN / (ds / 2))



def at_max_stop():
    """The spots the stops load, at the DYSC-25's largest stop force (Festo: 4000 N) instead of the design's pushes:
    the stresses scale with the push, so the roots' factors (sa = sm) fall in proportion and the bolts' by Eq. 8-38."""
    import range_sweep as RS
    Fmax = RS.DYSC[25]['F_stop']
    js = RS.jaw_stops()
    r = bracket_roots()
    b = bolt_fatigue()
    out = dict(F=Fmax)
    for k in ('closed', 'open'):
        f = Fmax / js[k]['F']
        out[k] = dict(push=js[k]['F'], scale=f, n_root=r[k]['drawn']['n'] / f)
    j = b['joint']
    P = b['closed']['P'] * Fmax / js['closed']['F']
    sa = j['C'] * P / (2 * j['At'])
    out['bolt'] = dict(P=P, sa=sa, nf=SE_88 * (SUT_88 - b['si']) / (SUT_88 * sa + SE_88 * sa))
    return out


def summary():
    return dict(cycles=cycles(), bolts=bolt_fatigue(), roots=bracket_roots(), keyseat=keyseat(), groove=groove(),
                no_preload=no_preload(), sn=sn_line())


if __name__ == '__main__':
    s = summary()
    j = s['bolts']['joint']
    print('cycles %.3g' % s['cycles'])
    print('bolt joint', {k: round(v, 3) for k, v in j.items() if isinstance(v, float)}, [(n, round(k)) for n, _, _, _, k in j['frusta']])
    for k in ('closed', 'open'):
        print('bolt', k, {a: round(b, 3) for a, b in s['bolts'][k].items()})
    print('si %.2f' % s['bolts']['si'])
    r = s['roots']
    for k in ('closed', 'open'):
        print('root', k, {a: (round(b, 3) if isinstance(b, float) else {c: round(e, 3) for c, e in b.items()}) for a, b in r[k].items()})
    print('keyseat', {k: round(v, 3) for k, v in s['keyseat'].items()})
    print('groove', s['groove'])
