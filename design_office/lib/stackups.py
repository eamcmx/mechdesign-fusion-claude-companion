"""Chapter 61: tolerance stack-ups. A gap or a length that several parts make together, each part's dimension with its
tolerance: the worst case, adding every tolerance (Shigley's absolute system, Example 1-7, p. 30), the statistical
estimate with the tolerances as three standard deviations, root-sum-square (p. 31), and a Monte Carlo run of the same
chain (p. 31's three steps), each dimension drawn from a normal distribution with its tolerance band as six standard
deviations, centred on the band's middle. The stacks are the ones Part 9 and Chapter 56 handed on.

Sources: Budynas & Nisbett (2015), Section 1-14, pp. 29-31; SKF (n.d.-j) table 1 for tapered roller bearings
(Web pages/SKF_tapered_roller_tolerances_2026-11-03.md); Coolen (2021) after ISO 492 for the radial ball bearings
(Web pages/Coolen_ISO492_Normal_radial_tolerances_2026-11-03.md); AmesWeb after ISO 2768-1 for dimensions drawn without
a tolerance of their own (Web pages/AmesWeb_ISO_2768-1_linear_2026-11-03.md).
"""
import math
import os
import random

# ISO 2768-1, the linear dimensions' permissible deviations, mm, by class: (over, incl, deviation)
ISO2768 = {'f': ((0.5, 3, 0.05), (3, 6, 0.05), (6, 30, 0.1), (30, 120, 0.15), (120, 400, 0.2), (400, 1000, 0.3),
                 (1000, 2000, 0.5)),
           'm': ((0.5, 3, 0.1), (3, 6, 0.1), (6, 30, 0.2), (30, 120, 0.3), (120, 400, 0.5), (400, 1000, 0.8),
                 (1000, 2000, 1.2), (2000, 4000, 2.0)),
           'c': ((0.5, 3, 0.2), (3, 6, 0.3), (6, 30, 0.5), (30, 120, 0.8), (120, 400, 1.2), (400, 1000, 2.0),
                 (1000, 2000, 3.0), (2000, 4000, 4.0))}


def iso2768(size, cls='m'):
    for lo, hi, t in ISO2768[cls]:
        if lo <= size <= hi if lo == 0.5 else lo < size <= hi:
            return t
    raise ValueError('no ISO 2768 range for %g mm' % size)


class Stack:
    """A chain of dimensions that adds to one result: each link (name, nominal, upper, lower, sign), deviations in mm
    from the nominal, sign +1 if the link lengthens the result, -1 if it shortens it."""

    def __init__(self, name, links):
        self.name = name
        self.links = links

    def nominal(self):
        return sum(s * n for _, n, _, _, s in self.links)

    def limits(self):
        """The worst case: each link at the end of its band that moves the result most, both ways."""
        hi = sum(s * (n + (u if s > 0 else l)) for _, n, u, l, s in self.links)
        lo = sum(s * (n + (l if s > 0 else u)) for _, n, u, l, s in self.links)
        return lo, hi

    def mean(self):
        """The result with every link at its band's middle."""
        return sum(s * (n + (u + l) / 2.0) for _, n, u, l, s in self.links)

    def rss(self):
        """The statistical estimate: each band 6 sigma; the result's sigma the root of the sum of squares; its 3-sigma
        limits about the mean."""
        sig = math.sqrt(sum(((u - l) / 6.0) ** 2 for _, _, u, l, _ in self.links))
        m = self.mean()
        return dict(sigma=sig, lo=m - 3 * sig, hi=m + 3 * sig)

    def monte_carlo(self, n=200000, seed=56):
        """Shigley's three steps: draw each link, add them, repeat; the result's least, largest, 0.135 and 99.865
        percentiles (3 sigma either side for a normal result)."""
        rnd = random.Random(seed)
        ws = []
        for _ in range(n):
            ws.append(sum(s * (nom + rnd.gauss((u + l) / 2.0, (u - l) / 6.0)) for _, nom, u, l, s in self.links))
        ws.sort()
        q = lambda p: ws[min(n - 1, max(0, int(round(p * (n - 1)))))]
        return dict(n=n, min=ws[0], max=ws[-1], p_lo=q(0.00135), p_hi=q(0.99865), mean=sum(ws) / n)

    def table(self):
        return [dict(name=nm, nominal=n, upper=u, lower=l, sign=s, band=u - l) for nm, n, u, l, s in self.links]


# ---- the pinion's spacer (Chapter 54) ---------------------------------------------------------------------------------
# SKF table 1 (Normal), um: d 30-50 (the 32309's 45 and the 32308's 40): Bs 0/-120, Ts +200/0
SKF_TAPERED = {(30, 50): dict(Bs=(0.0, -0.120), Ts=(0.200, 0.0)), (50, 80): dict(Bs=(0.0, -0.150), Ts=(0.200, 0.0))}
NOSE_TOL = 0.02          # mm, the nose's two cup shoulders drawn to +-0.02 from each other (this chapter's decision)


def _tapered(d):
    for (lo, hi), v in SKF_TAPERED.items():
        if lo < d <= hi:
            return v
    raise ValueError(d)


def pinion_spacer():
    """The length the spacer must have between the two cones' front faces for the bearings to sit with no end play:
    the distance between the cups' back faces in the nose, S, and for each bearing its abutment width T less its cone
    width B. Each bearing's T and B carry SKF's Normal tolerances, the nose's S this chapter's +-0.02."""
    import axle_detail as A
    import axle_design as D
    sp = A.spacer()
    lay = sp['layout']
    S = lay['cupB'][0] - lay['cupA'][1]
    links = [("S, the cups' back faces in the nose", S, NOSE_TOL, -NOSE_TOL, +1)]
    for tag, name in (('A', '32309'), ('B', '32308')):
        d, T = D.SKF_TRB[name][0], D.SKF_TRB[name][2]
        tol = _tapered(d)
        links.append(('T of bearing %s, %s' % (tag, name), T, tol['Ts'][0], tol['Ts'][1], +1))
        links.append(("B of bearing %s's cone" % tag, A.SKF_ABUT[name]['B'], tol['Bs'][0], tol['Bs'][1], -1))
    st = Stack('the pinion\'s spacer, cone face to cone face', links)
    return dict(stack=st, nominal=st.nominal(), L=sp['L'], limits=st.limits(), rss=st.rss(), mc=st.monte_carlo())


# ---- a ring held between a shoulder and a circlip (Chapter 55) -----------------------------------------------------
# ISO 286-1 IT13, um, for the grooves' width H13 (Wikipedia after ISO 286-1:2010, Table 1; its IT11 agrees with Shigley)
IT13 = ((0, 3, 0.140), (3, 6, 0.180), (6, 10, 0.220))
# the circlips' thickness tolerance (Springmasters, DIN 472 and DIN 471 tables), mm: (thickness, upper, lower)
CIRCLIP_T = {('472', 62): (2.0, 0.0, -0.07), ('472', 72): (2.5, 0.0, -0.07), ('472', 120): (4.0, 0.0, -0.10),
             ('471', 30): (1.5, 0.0, -0.06)}
# the rings' width, Normal class (Coolen after ISO 492; SKF table 1 for the tapered): d over-incl -> (upper, lower)
RADIAL_BS = ((18, 30, 0.0, -0.120), (30, 50, 0.0, -0.120), (50, 80, 0.0, -0.150))
SHOULDER_GROOVE = 0.05   # mm, a bore's shoulder to its groove's inner flank, drawn +-0.05 (this chapter's decision)


def it13(w):
    for lo, hi, t in IT13:
        if lo < w <= hi:
            return t
    raise ValueError(w)


def radial_bs(d):
    for lo, hi, u, l in RADIAL_BS:
        if lo < d <= hi:
            return u, l
    raise ValueError(d)


def held_ring(name, B, d, D_sg, W, ring):
    """The axial play of a ring between a shoulder and a circlip in its groove: the shoulder to the groove's inner
    flank D_sg and the groove's width W less the ring's width B and the circlip's thickness. The circlip bears on the
    groove's far flank."""
    t, tu, tl = CIRCLIP_T[ring]
    bu, bl = radial_bs(d)
    links = [('the shoulder to the groove', D_sg, SHOULDER_GROOVE, -SHOULDER_GROOVE, +1),
             ('the groove\'s width, H13', W, it13(W), 0.0, +1),
             ('the bearing\'s width', B, bu, bl, -1),
             ('the circlip\'s thickness', t, tu, tl, -1)]
    st = Stack(name, links)
    return dict(stack=st, nominal=st.nominal(), limits=st.limits(), rss=st.rss(), mc=st.monte_carlo())


def eyes():
    """The robot's rod eyes, the outer ring of each 22213 E or 22207 E between the eye's lip and its DIN 472."""
    import palletizer_detail as X
    out = {}
    for e in X.eyes():
        key = e['bearing']
        if key in out:
            continue
        B = e['ring'][1] - e['ring'][0]
        d = X.SRB[key]['d']
        W = e['groove'][1] - e['groove'][0]
        D_sg = e['groove'][0] - e['lip'][1]
        ring = ('472', {124.0: 120, 75.0: 72}[e['groove_d']])
        out[key] = held_ring('the %s in its eye' % key, B, d, D_sg, W, ring)
    return out


def bag_locating():
    """The bag gripper's locating 6206: its outer ring between the boss's lip and the DIN 472 62 x 2."""
    import palletizer_detail as X
    import bag_gripper as BG
    je = X.jaw_ends()
    e = je['locating']
    b0, b1 = BG.LAY['bearing_x']
    return held_ring('the locating 6206 in its boss', b1 - b0, 30.0, e['groove'][0] - b0, e['groove'][1] - e['groove'][0],
                     ('472', 62))


# ---- the bag gripper's floating end (Chapter 55) ---------------------------------------------------------------------
BEAM_TOL = 0.05          # mm, the beam's length between its machined ends (this chapter's decision; 0.10 fails)
LIP_TOL = 0.05           # mm, a boss's lip and groove from the end plate's inner face
JAW_TOL = 0.05           # mm, the jaw's two stub shoulders apart, ground after welding (idem)


def bag_floating(jaw_tol=None, beam_tol=None):
    """The floating 6206's room each way in its boss, from the locating boss's lip: toward its lip, the jaw's span
    between the two inner rings' inboard faces less the frame's (the beam's length and the two lips' depths), less the
    jaw's shift in the locating ring's play; toward its circlip, the frame's distance to the circlip's face less the jaw's
    span and the bearing. The jaw's shift in the locating ring's play, at its worst; at 20 C, then at the hall's ends with
    Chapter 55's growth of the aluminium frame against the steel jaw."""
    import bag_gripper as BG
    import palletizer_detail as X
    jt = JAW_TOL if jaw_tol is None else jaw_tol
    bt = BEAM_TOL if beam_tol is None else beam_tol
    L = BG.LAY
    je = X.jaw_ends()
    beam = 2 * L['beam_x']
    face = L['plate_x'][0]
    lip_loc = je['locating']['lip'][1] - face
    lip_flt = je['floating']['lip'][1] - face
    c_flt = je['floating']['groove'][0] - face
    W = je['floating']['groove'][1] - je['floating']['groove'][0]
    b0, b1 = L['bearing_x']
    J = 2 * b0
    B = b1 - b0
    t, tu, tl = CIRCLIP_T[('472', 62)]
    bu, bl = radial_bs(30.0)
    p_max = bag_locating()['limits'][1]
    g_in = Stack('toward the lip', [
        ("the jaw's shoulders apart", J, jt, -jt, +1),
        ("the beam's length", beam, bt, -bt, -1),
        ("the locating lip's depth", lip_loc, LIP_TOL, -LIP_TOL, -1),
        ("the floating lip's depth", lip_flt, LIP_TOL, -LIP_TOL, -1)])
    g_out = Stack('toward the circlip', [
        ("the beam's length", beam, bt, -bt, +1),
        ("the locating lip's depth", lip_loc, LIP_TOL, -LIP_TOL, +1),
        ("the floating groove's position", c_flt, LIP_TOL, -LIP_TOL, +1),
        ("the groove's width, H13", W, it13(W), 0.0, +1),
        ("the circlip's thickness", t, tu, tl, -1),
        ("the jaw's shoulders apart", J, jt, -jt, -1),
        ("the bearing's width", B, bu, bl, -1)])
    gr = X.jaw_growth()
    hot, cold = max(gr.values()), min(gr.values())
    # the jaw's place in its locating play is not a tolerance: a push along the jaw can hold it at either end, so it
    # is taken at its worst in both estimates: toward the lip the whole play, toward the circlip none of it
    out = {}
    for key, st, therm, shift in (('in', g_in, hot, p_max), ('out', g_out, -cold, 0.0)):
        lo, hi = st.limits()
        r = st.rss()
        out[key] = dict(stack=st, nominal=st.nominal(), limits=(lo, hi), rss=r, mc=st.monte_carlo(), shift=shift,
                        thermal=therm, worst=lo - shift - therm, rss_lo=r['lo'] - shift - therm)
    return dict(p_max=p_max, growth=gr, **out)


# ---- the wrist's play at the tool (Chapter 52) -------------------------------------------------------------------------
def wrist_play(n=200000, seed=52):
    """The play at the payload's centre when the load reverses, from Chapter 52's three sources: the gear unit's
    backlash, halved by the belt; the belt's; the bevel set's. Each is a maker's or a design's largest value, so a
    wrist has anything from none to that, taken as evenly likely: the worst case adds the three largest; the Monte Carlo
    draws each from 0 to its largest and gives the share of wrists over a play."""
    import wrist_detail as WD
    pl = WD.numbers()[1]['play']
    rows = [(nm, at) for nm, _, at in pl['rows']]
    rnd = random.Random(seed)
    ws = sorted(sum(rnd.uniform(0.0, at) for _, at in rows) for _ in range(n))
    q = lambda p_: ws[min(n - 1, int(round(p_ * (n - 1))))]
    mean = sum(at for _, at in rows) / 2.0
    sigma = math.sqrt(sum(at ** 2 / 12.0 for _, at in rows))
    return dict(rows=rows, worst=pl['at_tool'], r_p=pl['r_p'], mean=mean, sigma=sigma, p99=q(0.99), p9973=q(0.9973), p99865=q(0.99865),
                mc_mean=sum(ws) / n, max=ws[-1])


# ---- the flap's GE 20 ES outer rings in the 7075 eyes over the brief's temperatures (Chapter 53) ---------------------
# hole classes at 30-50 mm, um (Miki Pulley after JIS B 0401, the counterpart of ISO 286): upper, lower
HOLES_35 = {'N7': (-0.008, -0.033), 'P7': (-0.017, -0.042), 'R7': (-0.025, -0.050), 'S7': (-0.034, -0.059)}
# S7 at 30-50 from the same rule ISO 286 builds the others by (ES = -ei(s) + delta, delta = IT7 - IT6 = 9 um), the
# shaft s's 43 from Shigley Table A-12: -43 + 9 = -34, less IT7 25: -59
LOCTITE638 = dict(tau_al=17.0, hot70=0.70, gap=0.25, T_test=-75.0, Tg=76.0)   # Henkel TDS May 2024, pp. 1-2
MU_END = 0.25            # SKF: steel/steel's failure criterion, the friction coefficient 0.20 to 0.25


def ge_fits():
    """Each candidate housing class for the GE's 35 mm outer ring in 7075: its loosest fit hot, at +70 C, and its
    tightest cold, at -55 C, with the bearings' operating clearance there (Chapter 53's model: the outer ring's
    interference shrinks the sphere, the aluminium's greater contraction adds to it). A fit that holds the ring hot
    and leaves the Normal hinges a clearance cold is wanted."""
    import flap_detail as FD
    import flap_design as F
    GE = FD.GE
    dA = (FD.A_AL - FD.A_ST) * GE['D']
    b = F.brief()
    T_lo, T_hi = b['T_min'], b['T_max']
    out = {}
    for cls, (hu, hl) in HOLES_35.items():
        loose20 = -hu + GE['od'][0]                     # the ring smallest, the hole largest
        tight20 = -hl + GE['od'][1]
        loose_hot = loose20 - dA * (T_hi - 20.0)
        tight_cold = tight20 + dA * (20.0 - T_lo)
        _, shrink = FD.outer_shrink(tight_cold)
        rows = {}
        for where, shaft, cls_b in (('hinges', 'h6', 'CN'), ('fitting', 'm6', 'C3')):
            sf = FD.FITS[shaft]
            ii = sf[1] - GE['bore'][0]
            _, grow = FD.inner_growth(ii)
            rows[where] = GE[cls_b][0] - grow - shrink
        out[cls] = dict(loose20=loose20, tight20=tight20, loose_hot=loose_hot, tight_cold=tight_cold, shrink=shrink,
                        hinges=rows['hinges'], fitting=rows['fitting'], holds_hot=loose_hot > 0, clear_cold=min(rows.values()) > 0)
    return dict(T=(T_lo, T_hi), dA=dA, rows=out)


def ge_bond():
    """The N7 seat kept and the outer ring bonded with LOCTITE 638: the bond's torque on the ring's outside at +70 C,
    against the sphere's friction torque at the end of the bearing's life, at the largest load on a GE: the hinge's."""
    import flap_design as F
    import flap_detail as FD
    sd = F.system_design()
    GE = FD.GE
    C = F.GE20['C_out']
    D = GE['D']
    tau = LOCTITE638['tau_al'] * LOCTITE638['hot70']
    T_bond = tau * math.pi * D * C * D / 2.0 / 1000.0
    loads = dict(hinge=sd['R_hinge'], fitting=2 * sd['fit_bolt_load'])
    T_f = {k: MU_END * v * GE['dk'] / 2.0 / 1000.0 for k, v in loads.items()}
    gap_hot = -ge_fits()['rows']['N7']['loose_hot']
    return dict(tau=tau, area=math.pi * D * C, T_bond=T_bond, loads=loads, T_f=T_f,
                n=min(T_bond / t for t in T_f.values()), gap_hot=gap_hot, gap_ok=gap_hot < LOCTITE638['gap'])


# ---- the small gaps of Chapter 56's sweep, measured in Fusion by design_office/StackGaps ----------------------------------
# each: the gap's name, its machine, the state of its closest approach, the two parts (names start with; b_not: names
# left out though they start so), the states either side to see how fast it closes
def flap_end_stop():
    """The flap's deflection with the ball screw's nut on the screw's end stop: the thread the nut has left beyond the
    full stroke (its front at full stroke to the stop's face, Chapter 33's layout) added to the actuator's extended
    length, and the linkage solved for the deflection that gives that length."""
    import flap_design as fd
    a = fd.actuator(fd.brief())
    margin = fd.LAY['stop'][0] - (fd.LAY['nut'][1] + a['stroke'])
    target = a['L_extended'] + margin
    lo, hi = fd.brief()['delta_max'], 90.0
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if fd.linkage(mid)['length'] < target:
            lo = mid
        else:
            hi = mid
    return dict(margin=margin, delta=lo, length=target, arm=fd.linkage(fd.brief()['delta_max'])['arm'])


FLAP_STOP = round(flap_end_stop()['delta'], 2)


GAPS = [
    dict(key='unit', machine='unit', state=('float', 30.0), a='Cap screw M10x25', b='Carriage', around=[('float', 27.0)]),
    dict(key='flap', machine='flap', state=('deflection', 40.0), a='Rod end SI 20 ES', b='Horn', b_not=('Horn pin',),
         around=[('deflection', 38.0), ('deflection', 36.0), ('deflection', FLAP_STOP)]),
    dict(key='wrist', machine='wrist', state=('pitch', (24.0, 0.0)), a='Stop disc', b='Countersunk screw ISO 10642 M4x12',
         around=[('pitch', (32.0, 0.0)), ('pitch', (112.0, 0.0))]),
    dict(key='changer', machine='changer', state=('lock', (True, 0.0)), a='Cam', b='Tool plate ISO 9409-1-160-6-M10',
         around=[('lock', (False, 0.0))]),
    dict(key='axle_casting', machine='axle', state=('straight', (0.0, 0.0)), a='Crown wheel bolt M12x35', b='Carrier casting',
         around=[('straight', (15.0, 0.0)), ('turning', (0.0, 2.5))]),
    dict(key='axle_cap', machine='axle', state=('turning', (30.0, 2.5)), a='Crown wheel bolt M12x35', b='Bearing cap C1',
         around=[('straight', (30.0, 0.0))]),
]


# ---- the sweep's gaps, stacked ----------------------------------------------------------------------------------------
# ISO 4762 heads' height k, min and max (Fasten.it after DIN EN ISO 4762, 2004); ISO 10642 M4: dk theoretical (max) and
# the actual head's least, k max (Fasten.it after DIN EN ISO 10642, 2020)
K_4762 = {10: (9.64, 10.0), 12: (11.57, 12.0)}
CSK_M4 = dict(dk=8.96, dk_min=7.53, k=2.48)


def unit_recess():
    """The compliance unit's six M10 heads in the top plate's counterbores, 10.5 deep from the plate's underside. At
    the full float the carriage's wings reach the underside, so the heads clear them by the counterbore's depth less
    the head's height k; the depth drawn without a tolerance, ISO 2768-m."""
    k_lo, k_hi = K_4762[10]
    t = iso2768(10.5)
    return Stack("the M10 heads above the top plate's underside",
                 [("the counterbore's depth", 10.5, t, -t, +1), ("the head's height k", k_hi, 0.0, k_lo - k_hi, -1)])


WRIST_RINGS = 0.15       # mm, a 6002's inner ring's outer face against its outer ring's: the axial play and the rings'
#                          widths together, taken generously (this chapter's assumption, not a maker's figure)
CSK_CUT = 8.9            # mm, the cap's 90 deg countersinks at its face, drawn without a tolerance (ISO 2768-m)


def wrist_disc():
    """The stop disc's inner face, 1 mm outside the fixed cap's plate and its four countersunk M4. From the 6002's
    outer face at |y| 62.5: the disc sits on the spacer S5 (8.5 long) on the inner ring; the plate's face is the cap's
    spigot (7.5 from its end to the plate's face) on the outer ring. A countersunk head stands proud of the face by
    half the amount its diameter exceeds the countersink's (90 deg): at most the theoretical 8.96 in the smallest
    countersink, and sunk with the smallest head in the largest."""
    t = iso2768(CSK_CUT)
    proud = (CSK_M4['dk'] - (CSK_CUT - t)) / 2.0
    sunk = (CSK_M4['dk_min'] - (CSK_CUT + t)) / 2.0
    return Stack('the stop disc to the M4 heads',
                 [('the spacer S5', 8.5, iso2768(8.5), -iso2768(8.5), +1),
                  ("the inner ring's face against the outer ring's", 0.0, WRIST_RINGS, -WRIST_RINGS, +1),
                  ("the cap's spigot to its plate's face", 7.5, iso2768(7.5), -iso2768(7.5), -1),
                  ("the head's top above the plate's face", 0.0, proud, sunk, -1)])


GROUND = 0.01            # mm, the changer's four wedge faces ground (this chapter's decision)
CHANGER_HELD = 0.05      # mm, the groove's floor and the cam's length held (this chapter's decision)


def changer_wedge(face=GROUND, held=CHANGER_HELD):
    """The cam's bottom, 1.5 mm over the tool plate's groove floor at lock. The cam goes down until the segments are
    tight between its 5 deg cone and the lip's 15 deg face, so its height at lock is set by the radial room: a radial
    change dr moves the cam dr / tan 5 deg, and a change dz in a 15 deg face's height moves the segment dz / tan 15
    deg radially, so the cam dz / (tan 5 tan 15). Four faces set the room: the lip's depth below the tool plate's face
    and the segments' seats below the master's face (axial, at 15 deg), the segments' width between their faces and
    the cam's cone (radial). The gap is then the groove floor's depth less the cam's length below its lock height."""
    import tool_changer as tc
    G, C = tc.GEOM, tc.CHANGER
    t5, t15 = math.tan(math.radians(C['alpha'])), math.tan(math.radians(C['beta']))
    ax, rd = 1.0 / (t5 * t15), 1.0 / t5               # mm of the cam's height for a mm of a face
    z_bot = G['z_spigot'] + 0.5                         # the cam's bottom at lock
    floor = G['z_face'] - (G['z_spigot'] - 1.0)         # 26: the groove floor below the tool plate's face
    cam_len = G['z_seg'][1] - z_bot                     # 18.5: the cone's top to the cam's bottom
    lock = G['z_face'] - G['z_seg'][1]                  # 6: the cone's top below the faces at lock
    lip = G['z_face'] - G['z_chamfer']                  # 10: the 15 deg faces' line below the faces
    links = [("the cam's lock height: the lip's 15 deg face", 0.0, face * ax, -face * ax, +1),
             ("... the segments' seats in the master", 0.0, face * ax, -face * ax, +1),
             ("... the segments' width", 0.0, face * rd, -face * rd, +1),
             ("... the cam's cone", 0.0, face * rd, -face * rd, +1),
             ("the groove floor below the tool plate's face", floor, held, -held, +1),
             ("the cam's cone top below the faces", lock, 0.0, 0.0, -1),
             ("the cam's length below its cone's top", cam_len, held, -held, -1)]
    return dict(stack=Stack("the cam's bottom over the groove floor", links), ax=ax, rd=rd, lip=lip, t5=t5, t15=t15,
                stroke_left=G['z_piston'] - G['z_gland'][1], m2768=iso2768(lip) * ax)


# ISO 8062 casting tolerances, total, mm, by the raw casting's nominal size (Zhejiang Guanyu after ISO 8062:1994,
# Web pages/Guanyu_ISO8062_casting_tolerances_2026-11-03.md), CT6 to CT13
CT_TOTAL = {(63, 100): (0.78, 1.1, 1.6, 2.2, 3.2, 4.4, 6.0, 9.0), (100, 160): (0.88, 1.2, 1.8, 2.5, 3.6, 5.0, 7.0, 10.0),
            (160, 250): (1.0, 1.4, 2.0, 2.8, 4.0, 5.6, 8.0, 11.0)}
CW_SET = 0.10            # mm, the crown wheel's back face from the pinion's axis after the backlash is set (decision)


def ct(size, grade):
    for (lo, hi), row in CT_TOTAL.items():
        if lo < size <= hi:
            return row[grade - 6]
    raise ValueError('no CT row for %g mm' % size)


def axle_bolts(grade=None):
    """The crown wheel's M12 heads, 1 mm inside the carrier's legs and the bearing caps along the axle. From the pinion's
    axis: the crown wheel's back face at 57 (where the backlash setting puts it), the case's flange 12 and the head's
    k 12 out to the head at 81; the legs' and the caps' inner faces at 82. As cast, the face takes the casting's
    tolerance, centred; faced in the line-boring set-up, ISO 2768-m's."""
    k_lo, k_hi = K_4762[12]
    wall = 82.0
    tw = ct(wall, grade) / 2.0 if grade else iso2768(wall)
    return Stack('the M12 heads to the %s' % ('cast face, CT%d' % grade if grade else 'faced legs and caps'),
                 [("the legs' and the caps' inner faces", wall, tw, -tw, +1),
                  ("the crown wheel's back face, set", 57.0, CW_SET, -CW_SET, -1),
                  ("the case's flange", 12.0, iso2768(12.0), -iso2768(12.0), -1),
                  ("the head's height k", k_hi, 0.0, k_lo - k_hi, -1)])



FLAP_FACES = 0.2         # mm, the horn's lug faces and the rod end's eye along the gap, ISO 2768-m at 6 to 30 mm (the
#                          rod end's own tolerance taken the same: this chapter's assumption, not SKF's figure)


def flap_rod_end():
    """The flap's rod end against its horn. In service the flap stops at 40 deg; a drive that ran on would stop with the
    screw's nut on its end stop, at FLAP_STOP, where Fusion measures the gap (results/stack_gaps.json). Three links
    move it there: the end stop's place along the screw, ISO 2768-m on its 240 mm, which moves the flap by that over
    the horn's arm and the gap by the gap's change a degree between 40 deg and the stop; the horn's face and the rod
    end's eye, each at FLAP_FACES along the gap."""
    import json
    import flap_design as fd
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    rows = [g for g in json.load(open(os.path.join(here, 'results', 'stack_gaps.json'), encoding='utf-8'))
            if g['key'] == 'flap'][0]['rows']
    at = {round(r['state'][1], 2): r['d'] for r in rows}
    g40, gs = at[40.0], at[FLAP_STOP]
    slope = (g40 - gs) / (FLAP_STOP - 40.0)
    es = flap_end_stop()
    t_stop = iso2768(fd.LAY['stop'][0])
    d_deg = math.degrees(t_stop / es['arm'])
    st = Stack("the rod end to the horn at the screw's end stop",
               [("the gap at the end stop, in Fusion", gs, 0.0, 0.0, +1),
                ("the end stop's place, as the flap's angle", 0.0, d_deg * slope, -d_deg * slope, -1),
                ("the horn's face", 0.0, FLAP_FACES, -FLAP_FACES, -1),
                ("the rod end's eye", 0.0, FLAP_FACES, -FLAP_FACES, -1)])
    return dict(stack=st, g40=g40, g_stop=gs, at=at, slope=slope, t_stop=t_stop, d_deg=d_deg, end=es)


def sweep_stacks():
    """Every sweep gap's stack: its nominal, worst case, root-sum-square and Monte Carlo."""
    out = {}
    cw = changer_wedge()
    for key, st in (('unit', unit_recess()), ('flap', flap_rod_end()['stack']), ('wrist', wrist_disc()), ('changer', cw['stack']),
                    ('axle', axle_bolts()), ('axle_CT8', axle_bolts(8)), ('axle_CT9', axle_bolts(9)),
                    ('axle_CT10', axle_bolts(10))):
        out[key] = dict(name=st.name, nominal=st.nominal(), worst=st.limits(), rss=st.rss(), mc=st.monte_carlo(),
                        links=st.table())
    out['changer_factors'] = dict(ax=cw['ax'], rd=cw['rd'], m2768=cw['m2768'], stroke_left=cw['stroke_left'])
    return out
