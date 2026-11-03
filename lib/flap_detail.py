"""Chapter 53: the flap actuator in detail. The motor unit that Chapter 33 bought to a specification is set against
the nearest catalogue products (maxon's EC-i 52 and GP 52 C) and the actuator's numbers are worked again with them;
the pinion's tooth force is set against the gearhead's bearing; the spherical plain bearings' clearance against their
fits and the brief's temperatures; the hinge pins' seats are hardened; the ball screw is ordered from HIWIN.

Sources (the pages saved in References/Web pages, 3 November 2026):
  maxon, product pages 574741 (EC-i 52, 180 W, 24 V) and 223087 (GP 52 C, 26:1), and the Product Range 2026/27,
    pp. 207, 386, 479, 480 (drawings, overall lengths, brakes)
  SKF product pages 61804-2RS1 and 6001-2RSH; SKF (2013) Spherical plain bearings and rod ends, pp. 70-71, 101-104
  NBK (2015) Conversion table of hardness (after ASTM E140)
  HIWIN GmbH (2022) Ballscrews BS-08-9-EN-2204-K: pp. 22, 33, 48-49, 62
  Shigley (Budynas & Nisbett, 2015): Eqs. 3-49, 3-50 (p. 127), 3-56 (p. 130), 3-19 (p. 102), Table 3-3 (p. 131),
    Table A-5 (p. 1015), Table A-21 (p. 1050), Eqs. 14-39, 14-40 (p. 756), p. 62 (induction hardening),
    Table 8-7 (p. 418); DIN 6885-1 and DIN 471 (Fasten.it)
Values marked 'assumed' are this chapter's, not a source's.
"""
import math

import flap_design as fd

pi = math.pi

# ---- the products -------------------------------------------------------------------------------------------------
MOTOR = dict(name='EC-i 52, 180 W, High Torque, 24 V', part='574741', U_n=24.0, n0=4720.0, I0=0.716, n_n=4210.0,
             T_n=438.0, I_n=9.03, T_stall=4580.0, eta_max=0.90, R=0.0935, k_M=48.1, k_n=198.0, grad=0.386,
             J=170.0, R_th1=0.63, R_th2=4.32, tau_w=19.9, tau_m=1780.0, T_amb=(-40.0, 100.0), T_wmax=155.0,
             n_max=6000.0, mass=0.823, length=80.0, D=52.0, rear_shaft=(8.0, 16.8))      # mN m, A, rpm, gcm2, K/W, s
GEARHEAD = dict(name='GP 52 C, 2 stages, 26:1', part='223087', i=26.0, T_cont=15.0, T_int=22.5, eta=0.83,
                backlash=0.8, J=9.1, L1=65.0, mass=0.620, Fr=630.0, Fr_at=12.0, Fa=200.0, n_in=6000.0,
                T_rec=(-15.0, 80.0), T_ext=(-40.0, 100.0), shaft=(12.0, 29.5), shaft_tol=(-0.020, -0.005),
                key=(4.0, 4.0, 20.0), pilot=(32.0, 3.15), pilot_tol=(-0.02, 0.0), D=52.0, holes_d=46.0,
                holes_depth=8.0, MA=4.0)
RATIOS_2STAGE = (12.0, 15.0, 19.0, 21.0, 26.0)       # GP 52 C, two stages (49/4, 91/6, 169/9, 21, 26)
OVERALL = 145.1                                      # mm, EC-i 52 180 W with the 2-stage GP 52 C (catalogue p. 386)
AB42S = dict(T=1.4, mass=0.14, J=6.1, T_amb=(-40.0, 120.0), U=(24.0 * 0.90, 24.0 * 1.05), couple=30.0, open_=85.0)
ETA_ELEC = 0.95                                      # the drive's electronics, assumed
BRAKE_FACTOR = 2.0                                   # Chapter 33's, assumed
SPEC = dict(T_motor=428.5, n_motor=2955.0, P=132.6, i=24.0, eta_gh=0.81, brake=0.856, D=52.0, L=176.0, mass=1.6,
            T_min=-55.0, T_max=70.0)                 # Table ch33motor
FLANGE_T = 6.0                                       # the unit's flange, as Table ch33motor, mm


def numbers(i=None, eta=None):
    """Chapter 33's motor side worked again with a gearhead of ratio i and efficiency eta (the GP 52 C's by default):
    the motor's point, the speed it has at 28 V, its current, its losses and its winding's warming over the 12 s of a
    deployment, the brake it needs, and the actuator's current from the bus."""
    o = fd.actuator_design()
    b = fd.brief()
    i = i or GEARHEAD['i']
    eta = eta or GEARHEAD['eta']
    M = MOTOR
    g = o['gears']
    out = dict(i=i, eta=eta, T_pin=o['T_pin'], n_pin=o['n_pin'], ratio=g['ratio'])
    out['n_m'] = o['n_pin'] * i
    out['T_m'] = o['T_pin'] / i / eta                              # mN m (N mm)
    out['P_m'] = out['T_m'] / 1000.0 * out['n_m'] * 2 * pi / 60.0
    out['n_28'] = M['k_n'] * b['V_bus'] - M['grad'] * out['T_m']   # rpm the motor reaches at 28 V with this load
    out['n_24'] = M['k_n'] * 24.0 - M['grad'] * out['T_m']
    out['I_m'] = M['I0'] + out['T_m'] / M['k_M']                  # A, the friction's torque k_M I0 included
    out['P_cu'] = out['I_m'] ** 2 * M['R']
    out['P_fr'] = M['k_M'] * M['I0'] / 1000.0 * out['n_m'] * 2 * pi / 60.0
    out['eta_m'] = out['P_m'] / (out['P_m'] + out['P_cu'] + out['P_fr'])
    t = fd.brief()['t_deploy']
    out['dT_w'] = out['P_cu'] * (M['R_th1'] * (1 - math.exp(-t / M['tau_w'])) + M['R_th2'] * (1 - math.exp(-t / M['tau_m'])))
    out['dT_w_steady'] = out['P_cu'] * (M['R_th1'] + M['R_th2'])
    out['use_T'] = out['T_m'] / M['T_n']
    out['use_I'] = out['I_m'] / M['I_n']
    out['use_n'] = out['n_m'] / out['n_28']
    out['P_bus'] = (out['P_m'] + out['P_cu'] + out['P_fr']) / ETA_ELEC
    out['I_bus'] = out['P_bus'] / b['V_bus']
    out['eta_total'] = o['eta'] * 0.98 ** 2 * eta * out['eta_m'] * ETA_ELEC
    out['I_one'] = o['act']['P_out'] / out['eta_total'] / b['V_bus']
    out['T_out'] = o['T_pin'] / 1000.0                            # N m at the gearhead's output, the limit
    out['T_out_ult'] = -o['T_back'] / g['ratio'] / 1000.0         # held by the brake at the ultimate
    out['brake_need'] = -o['T_back'] / g['ratio'] / i / 1000.0    # N m at the motor
    out['brake'] = BRAKE_FACTOR * out['brake_need']
    out['J_m'] = M['J'] + GEARHEAD['J'] + AB42S['J']
    out['mass_unit'] = M['mass'] + GEARHEAD['mass']
    out['brake_mass'] = SPEC['mass'] - out['mass_unit']
    out['brake_len'] = SPEC['L'] - FLANGE_T - OVERALL
    # the play the gearhead's backlash lets the flap make: at the screw a third of it, the nut a lead a turn
    k = fd.linkage(b['delta_max'])
    ds = fd.HIWIN[fd.NUT][1] * GEARHEAD['backlash'] / g['ratio'] / 360.0
    out['play_nut'] = ds
    out['play_flap'] = math.degrees(ds / math.hypot(*k['tip']) / math.sin(math.radians(k['mu'])))
    return out


def ratios():
    """Each two-stage GP 52 C against the motor: its continuous torque and the speed it needs (the efficiency taken
    as the catalogue's 83 % for all)."""
    rows = []
    for i in RATIOS_2STAGE:
        n = numbers(i=i)
        rows.append(dict(i=i, T_m=n['T_m'], n_m=n['n_m'], ok=n['T_m'] <= MOTOR['T_n'] and n['n_m'] <= n['n_28']))
    return rows


# ---- the pinion on the gearhead's shaft ------------------------------------------------------------------------------
# along s (mm, the actuator's axis, as flap_design.LAY): the unit's flange 6 thick from the housing's back face at 40,
# the output shaft 29.5 from the flange's face, maxon's key 4 x 4 x 20 two from its end, the pinion's teeth on
# Chapter 33's face with a hub forward to the shaft's end, an M4 screw and a made end washer, 18 across, in the shaft's
# centre hole (an ISO 7089 washer, 9 across, would bear on the 12 mm shaft's end and not on the hub)
PIN = dict(flange=(40.0, 46.0), shaft=(40.0 - GEARHEAD['shaft'][1], 46.0), key=(12.5, 32.5), hub=(10.5, 13.0),
           r_hub=9.0, t1=2.5, t2=1.8, washer=(8.5, 10.5), screw=(4.0, 10.0), centre_hole=10.0)
END_WASHER = dict(d1=4.3, d2=18.0, h=2.0)          # made, the hub's diameter; its thickness assumed


def pinion_force():
    """The tooth force on the pinion: at the limit from the torque it gives (Chapter 33's T_P, the drag and the meshes
    included), at the ultimate from the screw's backdriving torque held by the brake, both through the pitch radius
    and the pressure angle; against the gearhead's radial rating, given 12 mm from its flange."""
    o = fd.actuator_design()
    g = fd.LAY['gear']
    rP = g['m'] * g['z_P'] / 2.0
    phi = math.radians(20.0)
    W = o['T_pin'] / rP / math.cos(phi)
    W_u = -o['T_back'] / o['gears']['ratio'] / rP / math.cos(phi)
    face = 0.5 * sum(fd.LAY['gear_face'])
    return dict(rP=rP, Wt=o['T_pin'] / rP, W=W, W_u=W_u, rating=GEARHEAD['Fr'], at=GEARHEAD['Fr_at'],
                arm=PIN['flange'][0] - face, over=W / GEARHEAD['Fr'], over_u=W_u / GEARHEAD['Fr'], rating_3=900.0,
                M=W * (PIN['flange'][0] - face) / 1000.0, M_rating=GEARHEAD['Fr'] * GEARHEAD['Fr_at'] / 1000.0)


def pinion_rim():
    """The pinion's rim under the teeth over its keyway: Shigley's backup ratio m_B = t_R / h_t (Eq. 14-39) and the
    rim-thickness factor K_B (Eq. 14-40); the bore 12 mm with DIN 6885-1's hub depth t2 = 1.8 for a 4 x 4 key."""
    g = fd.LAY['gear']
    m = g['m']
    rf = m * g['z_P'] / 2.0 - 1.25 * m
    t_R = rf - (6.0 + PIN['t2'])
    h_t = 2.25 * m
    mB = t_R / h_t
    KB = 1.6 * math.log(2.242 / mB) if mB < 1.2 else 1.0
    gt = fd.actuator_design()['gears']
    sig = gt['sig_P'] * KB
    Se, Sut = gt['Se'], fd.SUT_4140
    nf = 1.0 / (sig / 2 / Se + sig / 2 / Sut)
    return dict(rf=rf, t_R=t_R, h_t=h_t, mB=mB, KB=KB, sig_v2=gt['sig_P'], sig=sig, nf_v2=gt['nf_P'], nf=nf,
                n_static=fd.SY_4140 / (gt['sig_u_P'] * KB), n_static_v2=gt['n_static'])


def pinion_key():
    """maxon's key in the pinion: its length in the hub, the side pressure at the limit and the ultimate,
    4 T / (d h L) on the hub's side (as Chapter 40's key), and the fit and the end screw: maxon's shaft
    -0.020/-0.005 in a K7 bore (Shigley Table A-12), the M4 screw by L > h + 1.5 d (Table 8-7) through the end
    washer, its thread in the shaft with the shaft at maxon's longest and shortest (29.5 0/-1)."""
    o = fd.actuator_design()
    L_in = min(PIN['key'][1], fd.LAY['gear_face'][1]) - max(PIN['key'][0], PIN['hub'][0])
    d, h = GEARHEAD['shaft'][0], GEARHEAD['key'][1]
    T, T_u = o['T_pin'], -o['T_back'] / o['gears']['ratio']
    K7 = (-0.012, 0.006)
    lo, hi = GEARHEAD['shaft_tol']
    return dict(L_in=L_in, sig=4 * T / (d * h * L_in), sig_u=4 * T_u / (d * h * L_in),
                fit=(K7[0] - hi, K7[1] - lo), screw_min=END_WASHER['h'] + 1.5 * PIN['screw'][0],
                engage=PIN['screw'][1] - END_WASHER['h'], engage_short=PIN['screw'][1] - END_WASHER['h'] - 1.0,
                hole=PIN['centre_hole'])


# ---- the spherical plain bearings: clearance against fits and temperature ----------------------------------------
GE = dict(d=20.0, D=35.0, dk=29.0, bore=(-0.010, 0.0), od=(-0.011, 0.0), CN=(0.040, 0.082), C3=(0.082, 0.124),
          C2=(0.010, 0.040), T_open=(-50.0, 200.0), T_LS=(-55.0, 110.0), lug_r=27.5)
E_ST, NU_ST, E_AL, NU_AL = fd.E_STEEL, fd.NU_STEEL, 71700.0, 0.333      # Shigley Table A-5
A_ST, A_AL = 10.8e-6, 23.9e-6                                           # Shigley Table 3-3, 0-100 C
FITS = dict(m6=(0.008, 0.021), h6=(-0.013, 0.0), N7=(-0.033, -0.008))   # 20 mm shaft, 35 mm hole (Shigley A-12, A-13)


def p_fit(delta, R, ro, ri, Eo, nuo, Ei, nui):
    """Shigley Eq. 3-56 with diameters and the diametral interference: the contact pressure, MPa."""
    D, do, di = 2 * R, 2 * ro, 2 * ri
    return delta / (D * ((do ** 2 + D ** 2) / (do ** 2 - D ** 2) + nuo) / Eo
                    + D * ((D ** 2 + di ** 2) / (D ** 2 - di ** 2) - nui) / Ei)


def inner_growth(delta):
    """The inner ring on a solid steel pin: the pressure, and the sphere's diametral growth from the tangential stress
    at the ring's outside (Eq. 3-50 at r = r_o, where the radial stress is nil) through Hooke's law (Eq. 3-19)."""
    R, ro = GE['d'] / 2, GE['dk'] / 2
    p = p_fit(delta, R, ro, 0.0, E_ST, NU_ST, E_ST, NU_ST) if delta > 0 else 0.0
    return p, 2 * ro * (2 * p * R ** 2 / (ro ** 2 - R ** 2)) / E_ST


def outer_shrink(delta):
    """The outer ring in an aluminium eye of radius lug_r: the pressure, and the sphere's diametral shrink from the
    tangential stress at the ring's inside (Eq. 3-49 with p_i = 0 at r = r_i)."""
    R, ri = GE['D'] / 2, GE['dk'] / 2
    p = p_fit(delta, R, GE['lug_r'], ri, E_AL, NU_AL, E_ST, NU_ST) if delta > 0 else 0.0
    return p, 2 * ri * (2 * p * R ** 2 / (R ** 2 - ri ** 2)) / E_ST


def clearance(shaft='m6', T=20.0, worst=True, cls='CN'):
    """The operating clearance of a GE 20 ES (mm) with its inner ring on a steel pin of class `shaft` and its outer
    ring in N7 in 7075, assembled at 20 C and soaked at T: the aluminium's greater contraction (Eq. 3-60) adds to the
    outer ring's interference when cold and takes from it when hot. worst: the tightest fits and the smallest
    clearance; otherwise the loosest fits and the largest."""
    s = FITS[shaft]
    ii = (s[1] - GE['bore'][0]) if worst else (s[0] - GE['bore'][1])
    io = (-FITS['N7'][0] + GE['od'][1]) if worst else (-FITS['N7'][1] + GE['od'][0])
    th = (A_AL - A_ST) * GE['D'] * (20.0 - T)
    io_T = io + th
    pi_, gi = inner_growth(ii)
    po, so = outer_shrink(io_T)
    c0 = GE[cls][0] if worst else GE[cls][1]
    return dict(shaft=shaft, T=T, cls=cls, interf_in=ii, interf_out=io, thermal=th, interf_out_T=io_T, p_in=pi_,
                p_out=po, grow=gi, shrink=so, initial=c0, operating=c0 - gi - so)


def clearance_table():
    rows = []
    for where, shaft in (('spar fitting, on the trunnions', 'm6'), ('hinges, on the hinge pins', 'h6')):
        for cls in ('CN', 'C3'):
            for T in (fd.brief()['T_min'], 20.0, fd.brief()['T_max']):
                rows.append(dict(where=where, cold=clearance(shaft, T, True, cls), loose=clearance(shaft, T, False, cls)))
    return rows


# ---- the hinge pins' seats ------------------------------------------------------------------------------------------
HARD = dict(min_HRC=50.0, HRC_range=(50.0, 55.0), case=(0.8, 1.2), zone_margin=2.0, Rz=10.0,
            core_HB=370.0, core_HRC=40.0, through_HB=510.0, through_HRC=(51.0, 52.0), elong_425=13.0, elong_205=8.0,
            HB_50=481.0, HB_52=512.0)
# upper hardness, case depth and the zone's margin past the ring are assumed; core and through-hardened values from
# Shigley Table A-21 (4140 Q&T at 425 and 205 C), HRC from NBK's table after ASTM E140


def hinge_zones():
    """The seats on the pins: under each GE 20 ES's inner ring (16 wide, centred on the hinge) and under the rod end's
    ball on the horn pin (16 wide); the band hardened is the seat and 2 mm more either side (assumed), so that the
    hardness is full where the ring sits and the band's soft edges lie under the cheeks and the spacers."""
    S = fd.SYS
    w = fd.GE20['B'] / 2
    wr = fd.ROD_END['B'] / 2
    m = HARD['zone_margin']
    return dict(hinges=[(h - w, h + w) for h in S['hinges']], horn=(-wr, wr),
                band_hinges=[(h - w - m, h + w + m) for h in S['hinges']], band_horn=(-wr - m, wr + m),
                width=2 * w, band=2 * (w + m), cheek_gap=S['cheek'][0] - w, spacer_gap=S['spacer'][0] - wr)


# ---- the ball screw from HIWIN ---------------------------------------------------------------------------------------
HIWIN_T = {'T0': 0.0035, 'T1': 0.006, 'T2': 0.008, 'T3': 0.012, 'T4': 0.018, 'T5': 0.023}   # e300, mm (Table 6.1)


def screw_order():
    """The ground screw's order: its class from what the flap can feel of the lead's deviation, its lengths from
    Chapter 33's layout, its shaft end to the book's drawing (the BSD pair's seat h4, SKF), no preload."""
    L = fd.LAY
    b = fd.brief()
    k = fd.linkage(b['delta_max'])
    e = HIWIN_T['T5']
    flap = math.degrees(e / math.hypot(*k['tip']) / math.sin(math.radians(k['mu'])))
    thread = L['thread'][1] - L['thread'][0]
    total = L['journal_end'][1] - L['washer'][1]
    code = 'R32-10T4-RSI-%d-%d-%.3f' % (round(thread), round(total), e)
    return dict(cls='T5', e300=e, flap=flap, thread=thread, total=total, code=code, material='1.7225 or 1.7228',
                HRC=(58.0, 62.0), journal='25 h4', seat=L['seat'])


if __name__ == '__main__':
    n = numbers()
    for k in sorted(n):
        print('%-14s %s' % (k, round(n[k], 4) if isinstance(n[k], float) else n[k]))
    print([(r['i'], round(r['T_m'], 1), round(r['n_m']), r['ok']) for r in ratios()])
    print({k: (round(v, 3) if isinstance(v, float) else v) for k, v in pinion_force().items()})
    print({k: round(v, 4) for k, v in pinion_rim().items()})
    print({k: (tuple(round(x, 4) for x in v) if isinstance(v, tuple) else round(v, 4)) for k, v in pinion_key().items()})
    for r in clearance_table():
        c, l_ = r['cold'], r['loose']
        print('%-32s %s %6.1f C: tight %6.1f um (in %.1f, out %.1f), loose %6.1f um, out-interf %.1f..%.1f um'
              % (r['where'], c['cls'], c['T'], c['operating'] * 1000, c['grow'] * 1000, c['shrink'] * 1000,
                 l_['operating'] * 1000, l_['interf_out_T'] * 1000, c['interf_out_T'] * 1000))
    print(hinge_zones())
    print(screw_order())
