# -*- coding: utf-8 -*-
"""materials_fits.py: Chapter 51 of Volume 3, materials, fits and surface finish. Every made part of the nine
assemblies (detail_inventory.py) given its material and every joint that the Fusion census found
(checks/joint_census.py, results/census_<assembly>.json) given its fit and its roughness, by rule, with the source of
each rule.

Sources (all in the references folder):
- ISO 286 standard tolerances and shaft deviations: Budynas & Nisbett (2015), Tables A-11 and A-12, pp. 1030-1031;
  the preferred fits, Volume 1's Chapter 6 (Table 7-9, p. 389).
- Bearing seats: SKF (n.d.), Seat tolerances for standard conditions, tables 1, 2 and 4; Surface texture of bearing
  seats, table 1; Selecting fits (an oscillating load counts as a rotating one; Delta = 2.5 Fr d / B); Tolerances and
  resultant fits (saved as Web pages/SKF_bearing_seats_2026-11-03.md).
- Keyways: J.W. Winco (n.d.), DIN 6885-1 (loose fit: hub JS9, shaft N9; tight: P9, P9). Dowels: Wurth (n.d.), ISO 8734
  tolerance class m6.
- Roughness by process: Childs (2014), Table 19.4, p. 771 (after BSI, 1991): the average ranges.

    python materials_fits.py      -> the decisions, assembly by assembly
"""
import io
import json
import os

import detail_inventory as D

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(os.path.dirname(HERE), 'results')

# ---- ISO 286 (Shigley Table A-11, p. 1030): standard tolerances, um, for sizes over-incl. mm ----------------------
RANGES = ((0, 3), (3, 6), (6, 10), (10, 18), (18, 30), (30, 50), (50, 80), (80, 120), (120, 180), (180, 250),
          (250, 315), (315, 400))
IT = {6: (6, 8, 9, 11, 13, 16, 19, 22, 25, 29, 32, 36), 7: (10, 12, 15, 18, 21, 25, 30, 35, 40, 46, 52, 57),
      8: (14, 18, 22, 27, 33, 39, 46, 54, 63, 72, 81, 89), 9: (25, 30, 36, 43, 52, 62, 74, 87, 100, 115, 130, 140),
      10: (40, 48, 58, 70, 84, 100, 120, 140, 160, 185, 210, 230), 11: (60, 75, 90, 110, 130, 160, 190, 220, 250, 290,
                                                                            320, 360)}
# Shigley Table A-12, p. 1031: the shafts' fundamental deviations, um (upper for c to h, lower for k to u), over-incl.
DEV_RANGES = ((0, 3), (3, 6), (6, 10), (10, 14), (14, 18), (18, 24), (24, 30), (30, 40), (40, 50), (50, 65), (65, 80),
              (80, 100), (100, 120), (120, 140), (140, 160), (160, 180), (180, 200), (200, 225), (225, 250))
DEV = {'f': (-6, -10, -13, -16, -16, -20, -20, -25, -25, -30, -30, -36, -36, -43, -43, -43, -50, -50, -50),
       'g': (-2, -4, -5, -6, -6, -7, -7, -9, -9, -10, -10, -12, -12, -14, -14, -14, -15, -15, -15),
       'h': (0,) * 19,
       'k': (0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 4, 4, 4),
       'n': (4, 8, 10, 12, 12, 15, 15, 17, 17, 20, 20, 23, 23, 27, 27, 27, 31, 31, 31),
       'p': (6, 12, 15, 18, 18, 22, 22, 26, 26, 32, 32, 37, 37, 43, 43, 43, 50, 50, 50),
       's': (14, 19, 23, 28, 28, 35, 35, 43, 43, 53, 59, 71, 79, 92, 100, 108, 122, 130, 140)}
UPPER = 'cdfgh'


def _row(ranges, d):
    for k, (lo, hi) in enumerate(ranges):
        if lo < d <= hi or (lo == 0 and d <= hi):
            return k
    raise ValueError('no ISO range for %g mm' % d)


def it(grade, d):
    return IT[grade][_row(RANGES, d)]


def shaft(cls, d):
    """A shaft class such as 'g6' at d mm: (upper, lower) deviations in um (Shigley Tables A-11 and A-12; a grade
    or deviation Shigley does not tabulate, such as k5 or m6, from SKF's tables of the seats)."""
    letter, grade = cls[0], int(cls[1:])
    if grade not in IT or letter not in DEV:
        seat = skf_row(cls, d)['seat']
        return max(seat), min(seat)
    t = it(grade, d)
    fd = DEV[letter][_row(DEV_RANGES, d)]
    return (fd, fd - t) if letter in UPPER else (fd + t, fd)


def hole(cls, d):
    """A hole H class at d mm: (upper, lower) in um; the hole basis H starts at the nominal size."""
    if cls[0] != 'H':
        raise ValueError('only H holes come from Shigley here: %s' % cls)
    return (it(int(cls[1:]), d), 0)


def fit_known(hole_cls, shaft_cls, d):
    """fit(), or None where neither Shigley's tables (to 400 mm) nor SKF's rows give the limits."""
    try:
        return fit(hole_cls, shaft_cls, d)
    except (ValueError, IndexError, KeyError):
        return None


def fit(hole_cls, shaft_cls, d):
    """The fit of a hole on a shaft: the least and largest clearance in um (negative for interference)."""
    hu, hl = hole(hole_cls, d)
    su, sl = shaft(shaft_cls, d)
    return hl - su, hu - sl


# ---- roughness by process (Childs 2014, Table 19.4, p. 771): the average range, Ra um ------------------------------
PROCESS_RA = {'turning': (0.4, 6.3), 'milling': (0.8, 6.3), 'drilling': (1.6, 6.3), 'reaming': (0.8, 3.2),
              'grinding': (0.1, 1.6), 'honing': (0.1, 0.8), 'lapping': (0.05, 0.4), 'sand casting': (12.5, 25.0),
              'cold drawing': (0.8, 3.2)}

# ---- bearing seats (SKF, n.d.) -------------------------------------------------------------------------------------
# table 1 (radial ball bearings) and table 2 (radial roller bearings) for a rotating inner ring load, by load class
# and shaft diameter over-incl.; table 4 for housings; the Ra beside each
BALL_SHAFT = {'light': ((0, 17, 'js5', 0.4), (17, 100, 'j6', 0.8), (100, 140, 'k6', 1.6)),
              'normal': ((0, 10, 'js5', 0.4), (10, 17, 'j5', 0.4), (17, 100, 'k5', 0.8), (100, 140, 'm5', 0.8),
                         (140, 200, 'm6', 1.6), (200, 500, 'n6', 1.6))}
ROLLER_SHAFT = {'light': ((0, 25, 'j6', 0.8), (25, 60, 'k6', 0.8), (60, 140, 'm6', 0.8)),
                'normal': ((0, 30, 'k6', 0.8), (30, 50, 'm5', 0.8), (50, 65, 'n5', 0.8), (65, 100, 'n6', 0.8),
                           (100, 280, 'p6', 1.6), (280, 500, 'r6', 1.6))}
HOUSING = {'stationary outer': ('H7', 3.2), 'rotating outer': ('N7', 3.2), 'indeterminate': ('K7', 3.2)}
STATIONARY_INNER = ('h6', 1.6)                 # SKF tables 1 and 2: easy axial displacement unnecessary
SEAT_RA = ((80, {7: 1.6, 6: 0.8, 5: 0.4}), (500, {7: 1.6, 6: 1.6, 5: 0.8}), (1250, {7: 3.2, 6: 1.6, 5: 1.6}))
                                              # SKF, Surface texture of bearing seats, table 1


REFS = os.path.join(os.path.expanduser('~'), 'Documents', 'Mechanical Design References', 'Web pages')
SKF_PAGE = os.path.join(REFS, 'SKF_bearing_seats_2026-11-03.md')


def _pair(cell):
    a, b = [float(x.replace('+', '')) for x in cell.split(',')]
    return a, b


def skf_fits():
    """SKF's Tolerances and resultant fits, read from the saved page: for each class, the rows over-incl. with the
    bearing's own deviations, the seat's deviations and the probable fit (um; minus an interference)."""
    lines = io.open(SKF_PAGE, encoding='utf-8').read().split('\n')
    out = {}
    head = None
    for ln in lines:
        if ln.startswith('| d over-incl.') or ln.startswith('| D over-incl.'):
            head = [c.strip() for c in ln.strip('|').split('|')]
            continue
        if head and ln.startswith('| ') and '-' in ln.split('|')[1] and not ln.startswith('|---'):
            cells = [c.strip() for c in ln.strip('|').split('|')]
            lo, hi = [float(x) for x in cells[0].split('-')]
            own = _pair(cells[1])
            classes = head[2:-2]
            probable = cells[-1].split(';')
            for j, cls in enumerate(classes):
                if cells[2 + j] in ('-', ''):
                    continue
                pr = [float(x.replace('+', '')) for x in probable[j].split()] if j < len(probable) and probable[j].strip() else None
                out.setdefault(cls, []).append(dict(lo=lo, hi=hi, own=own, seat=_pair(cells[2 + j]), probable=pr))
        elif head and not ln.startswith('|'):
            head = None if ln.strip() else head
    return out


def skf_row(cls, d):
    for r in skf_fits()[cls]:
        if r['lo'] < d <= r['hi']:
            return r
    raise ValueError('%s at %g mm is not in the saved SKF tables' % (cls, d))


def load_case(inner_moves, outer_moves):
    """SKF's conditions of rotation from which of the two seats moves: a ring whose seat moves under a load that stays
    with the frame carries a rotating load (an oscillating one counts as rotating); both moving, the direction of the
    load is indeterminate."""
    if inner_moves and outer_moves:
        return 'indeterminate'
    return 'rotating inner' if inner_moves else ('rotating outer' if outer_moves else 'rotating inner')


def bearing_seat(designation, d, D_, inner_moves, outer_moves, load='normal'):
    """The shaft's and the housing's tolerance classes and Ra for one bearing."""
    roller = designation.startswith(('Bearing 22', 'Bearing 23', 'Bearing 32', 'Bearing 30', 'Bearing NU'))
    case = load_case(inner_moves, outer_moves)
    if case == 'rotating outer':
        s_cls, s_ra = STATIONARY_INNER
    else:
        table = (ROLLER_SHAFT if roller else BALL_SHAFT)[load]
        s_cls, s_ra = [(c, r) for lo, hi, c, r in table if lo < d <= hi or (lo == 0 and d <= hi)][0]
    h_cls, h_ra = HOUSING['stationary outer' if case == 'rotating inner' else case]
    return dict(case=case, shaft=s_cls, shaft_ra=s_ra, housing=h_cls, housing_ra=h_ra, roller=roller)


def census(key):
    return json.load(io.open(os.path.join(RESULTS, 'census_%s.json' % key), encoding='utf-8'))


# ---- materials ------------------------------------------------------------------------------------------------------
# what Volume 2's parts lists name (book/tools/partslist_materials.py -> named_materials.json), read for the alloy and
# its condition; where a list names no grade, or names nothing, the model's own material gives the default
NAMED = json.load(io.open(os.path.join(HERE, 'named_materials.json'), encoding='utf-8'))
ALLOYS = [(r'7075-T6', 'aluminium 7075-T6'), (r'6082-T6', 'aluminium 6082-T6'), (r'2024-T3', 'aluminium 2024-T3'),
          (r'42CrMo4', 'steel 42CrMo4'), (r'4140', 'steel 4140'), (r'1045|C45', 'steel 1045 (C45)'), (r'1020', 'steel 1020'),
          (r'carburi[sz]ed', 'case-hardening steel'), (r'cast iron', 'cast iron'), (r'stainless', 'stainless steel'),
          (r'polyurethane', 'polyurethane'), (r'brass', 'brass'), (r'alumin', 'aluminium'), (r'steel', 'steel')]
CONDITIONS = [(r'Q\\?&T\s*(\d+)', 'quenched and tempered at %s deg C'), (r'through-hardened to (\d+)', 'through-hardened to %s HB'),
              (r'hardened to (\d+)', 'hardened to %s HB'), (r'(\d+)~?HB', 'through-hardened to %s HB'),
              (r'carburi[sz]ed', 'carburized and hardened'), (r'cold drawn', 'cold drawn'), (r'lip face hardened', 'lip face hardened')]
DEFAULT = {'steel': ('steel 1045 (C45)', 'cold drawn, or hot rolled over 76 mm across'),   # Shigley p. 353
           'aluminium': ('aluminium 6082-T6', 'T6')}
FROM_MODEL = {'Steel': 'steel', 'Aluminum': 'aluminium', 'ABS Plastic': 'ABS', 'Rubber': 'rubber', 'Paper': 'paper'}
# Fusion's own library (Fusion Material Library, read 3 November 2026): the material each alloy is given in the model,
# for its mass. Where the library holds the alloy, that entry; where it does not, the library's entry of the same series
# (6061 for 6082, 2014 for 2024, 4142 for 4140 and 42CrMo4, 8630 for a carburizing steel), named so in Chapter 51.
# A solid polyurethane is not in the library: reported, the model's material kept.
FUSION = {'steel 1045 (C45)': 'Steel AISI 1045 225 ANLD', 'steel 1020': 'Steel AISI 1020 107 HR',
          'steel 4140': 'Steel AISI 4142 380 QT', 'steel 42CrMo4': 'Steel AISI 4142 380 QT',
          'case-hardening steel': 'Steel AISI 8630 254 NORM', 'stainless steel': 'Stainless Steel',
          'cast iron': 'Iron, Gray', 'aluminium 7075-T6': 'Aluminum 7075', 'aluminium 6082-T6': 'Aluminum 6061',
          'aluminium 2024-T3': 'Aluminum 2014-T6', 'rubber': 'Rubber', 'brass': 'Brass'}
FUSION_DENSITY = {'Steel AISI 1045 225 ANLD': 7850.0, 'Steel AISI 1020 107 HR': 7870.0, 'Steel AISI 4142 380 QT': 7850.0,
                  'Steel AISI 8630 254 NORM': 7850.0, 'Stainless Steel': 8000.0, 'Iron, Gray': 7150.0,
                  'Aluminum 7075': 2810.0, 'Aluminum 6061': 2700.0, 'Aluminum 2014-T6': 2789.27, 'Rubber': 930.0,
                  'Brass': 8490.0}                       # kg/m3, as the library gives them


def _tokens(s):
    import re
    s = re.sub(r'\bz(\d+)\b', r'\1', re.sub(r'[,:()]', ' ', s.lower()))
    return s.split()


def named(key, kind):
    """The parts list's row for a made kind: its own name or the longest one that begins with the kind's words, or
    whose words the kind's begin with; None if the list names nothing like it."""
    rows = NAMED.get(key, {})
    k = _tokens(kind)
    best, score = None, 0
    for name, r in rows.items():
        n = _tokens(name)
        m = 0
        while m < min(len(k), len(n)) and k[m] == n[m]:
            m += 1
        if m == len(k) or m == len(n):
            if m > score:
                best, score = (name, r), m
    return best


def material(key, kind, model_material):
    """(material, condition, source) of one made kind."""
    import re
    row = named(key, kind)
    text = row[1]['material'] if row else ''
    alloy = next((lab for pat, lab in ALLOYS if re.search(pat, text, re.I)), None)
    cond = None
    for pat, lab in CONDITIONS:
        m = re.search(pat, text, re.I)
        if m:
            cond = lab % m.group(1) if '%s' in lab else lab
            break
    if alloy is None:
        alloy = FROM_MODEL.get(model_material, model_material)
        src = 'the model'
    else:
        src = 'Chapter %d' % row[1]['chapter']
    if alloy in DEFAULT:
        a, c = DEFAULT[alloy]
        return a, cond or c, (src + ', the grade by default')
    return alloy, cond, src


# ---- the worked bearing: the bag gripper's 6206-2RS1 at the driven end (Chapter 50) -----------------------------------
def worked_6206():
    """The driven end's 6206: its loads in the cycle and in the brake stop against SKF's dynamic rating, SKF's load
    class for each and the shaft class it gives, the interference SKF's Delta = 2.5 Fr d / B asks for, and what j6
    and k5 give with the bearing's own bore tolerance (theoretical and probable); the housing's H7 and J7."""
    import bag_gripper as B
    import standard_parts as S
    des = B.design()
    skf = S.bearing_6206()['skf']
    C, d, Db, Bw = skf['C'], skf['d'], skf['D'], skf['B']
    stop = des['bearing']
    ratio = des['jaw_cycle']['Mv'] / des['jaw_stop']['Mv']          # the vertical loads scale with Mv
    F_c_cycle = des['jaw_cycle']['M'] / (B.LAY['lever'] / 1000.0)
    R_cycle = stop['R'] * ratio
    P_cycle = (R_cycle ** 2 + F_c_cycle ** 2) ** 0.5 / 1000.0          # kN
    P_stop = stop['P'] / 1000.0
    cls = lambda P: 'light' if P <= 0.05 * C else ('normal' if P <= 0.1 * C else 'heavy')
    need = lambda P: 2.5 * P * d / Bw
    rows = {c: skf_row(c, d) for c in ('j6', 'k5', 'm5')}
    hrows = {c: skf_row(c, Db) for c in ('H7', 'J7')}
    return dict(C=C, d=d, D=Db, B=Bw, R_cycle=R_cycle, F_c_cycle=F_c_cycle, P_cycle=P_cycle, P_stop=P_stop,
                ratio_cycle=P_cycle / C, ratio_stop=P_stop / C, class_cycle=cls(P_cycle), class_stop=cls(P_stop),
                need_cycle=need(P_cycle), need_stop=need(P_stop),
                bore=(rows['k5']['own'][1], rows['k5']['own'][0]), od=hrows['H7']['own'],
                j6=rows['j6']['seat'], k5=rows['k5']['seat'], m5=rows['m5']['seat'],
                j6_prob=rows['j6']['probable'], k5_prob=rows['k5']['probable'], m5_prob=rows['m5']['probable'],
                H7=hrows['H7']['seat'], J7=hrows['J7']['seat'], H7_prob=hrows['H7']['probable'], J7_prob=hrows['J7']['probable'])


# ---- the worked press fit: the compliance unit's tube in its aluminium boss (Chapter 48) ------------------------------
E_AL, NU_AL, E_ST, NU_ST = 71.7e3, 0.333, 207.0e3, 0.292     # MPa; Shigley Table A-5, p. 1015
SY_2024 = 345.0                                               # MPa, 2024-T3 (Shigley Table A-24, as Chapter 48)
F_PRESS = 0.12                                                # friction in a press fit, as Volume 1's Chapter 6 assumed
ALPHA_AL, ALPHA_ST = 23.9e-6, 10.8e-6                         # 1/deg C, Shigley Table 3-3, p. 131


DT_SERVICE = 20.0                                             # deg C over the pressing temperature: assumed


def press_tube(shaft_cls='p6', dT=DT_SERVICE):
    """The unit's steel tube (20 x 16) pressed into the 2024-T3 boss (32 across, 20 deep), H7 on shaft_cls: the
    interference at its least and largest, and what is left of the least after a rise of dT, the boss growing more than
    the tube (Shigley Table 3-3, p. 131); for each, the contact pressure (Eq. 3-56, p. 130, with the radial
    interference), the tangential stresses at the joint (Eqs. 3-58, 3-59) and the friction force that holds the tube;
    the force that holds it against the O-ring's drag (Chapter 48), and the press force at the largest."""
    import compliance_unit as cu
    R = cu.AIR['tube'][0] / 2.0
    ri = cu.AIR['tube'][1] / 2.0
    ro = cu.AIR['boss'][0]
    L = cu.AIR['boss'][1]
    lo, hi = fit('H7', shaft_cls, 2 * R)                    # um, negative = interference
    k = R * ((ro ** 2 + R ** 2) / (ro ** 2 - R ** 2) + NU_AL) / E_AL + R * ((R ** 2 + ri ** 2) / (R ** 2 - ri ** 2) - NU_ST) / E_ST
    loss = (ALPHA_AL - ALPHA_ST) * dT * 2 * R * 1000.0          # um of diametral interference lost
    out = {}
    for name, dia in (('least', -hi), ('largest', -lo), ('least, warm', -hi - loss)):
        p = max(dia, 0.0) / 2.0 / 1000.0 / k
        out[name] = dict(interference=dia, p=p, s_boss=p * (ro ** 2 + R ** 2) / (ro ** 2 - R ** 2),
                         s_tube=-p * (R ** 2 + ri ** 2) / (R ** 2 - ri ** 2), F=F_PRESS * p * 3.141592653589793 * 2 * R * L)
    friction = cu.AIR['friction']
    return dict(cls=shaft_cls, R=R, ri=ri, ro=ro, L=L, k=k, fit=(lo, hi), cases=out, dT=dT, loss=loss, friction=friction,
                factor=SY_2024 / out['largest']['s_boss'], hold_warm=out['least, warm']['F'] / friction,
                holds=out['least, warm']['interference'] > 0)


# ---- the joints ------------------------------------------------------------------------------------------------------
# made parts in made parts (Volume 1, Chapter 6's preferred fits): parts that move against each other slide (H7/g6);
# parts fixed together and joined by a key are located with a transition fit (H7/k6); a part pressed in to stay is an
# interference fit (H7/p6, or H7/s6 where p6 would warm loose), where Volume 2 presses it; any other pair is located
# with a clearance fit (H7/h6)
PRESSED = {('unit', 'Tube 20x16', 'Housing')}
KEYS_HUB, KEYS_SHAFT = 'JS9', 'N9'              # J.W. Winco (n.d.), DIN 6885-1, the loose fit
DOWEL, DOWEL_HOLE = 'm6', 'H7'                  # Wurth (n.d.), ISO 8734; the hole of Volume 1's locational fits
OTHER_RA = 3.2                                  # a machined surface that fits nothing: within turning's and milling's ranges


# spherical plain bearings and rod ends, steel/steel (SKF, n.d.-c, tables 1 and 2; the seats' Rz): a pin that turns
# in the bore where Volume 2 drew it clear is the clearance fit on a hardened pin; any other inner ring is located by
# interference; the outer ring's class by the housing's material
PLAIN_SHAFT = {'interference': 'm6', 'clearance': 'h6'}
PLAIN_HOUSING = {'aluminium': 'N7', 'steel': 'M7'}
PLAIN_RZ = (10.0, 16.0)                       # um, shaft and housing seats (ISO 4288)
PLAIN_HARD = 50                               # HRC, a seat the bore slides on, hardened and ground
# angular contact thrust ball bearings for screw drives (SKF, n.d.-d, tables 1 and 2)
SCREW_SEATS = ('h4', 'H5')


def bearing_type(designation):
    if designation.startswith(('Bearing GE', 'Rod end')):
        return 'plain'
    if designation.startswith('Bearing BSD'):
        return 'screw drive'
    if designation.startswith(('Bearing 22', 'Bearing 23', 'Bearing 32', 'Bearing 30', 'Bearing NU')):
        return 'roller'
    return 'ball'


def _seat_ra(grade, d):
    for lim, by in SEAT_RA:
        if d <= lim:
            return by[grade]
    return by[grade]


def _bearing_id(s):
    """One bearing's seats belong together: its occurrence (a census since the second), else its name less the ring's
    word."""
    if s.get('occ'):
        tail = s['occ'].split('+')[-1].split(':')[0]
        for piece in (' inner ring', ' outer ring', ' cone', ' cup'):
            if tail.endswith(piece) or (piece + ' ') in tail:
                return tail.replace(piece, '')           # the rings are named for their bearing, which is one
        return s['occ']
    label = s['piece'].split(' (')[0]
    for piece in (' inner ring', ' outer ring', ' cone', ' cup'):
        label = label.replace(piece, '')
    return label


def nominal(f):
    """A fit's nominal size: the bore's diameter where the two touch; where the model draws the shaft clear, the one of
    the two diameters that is a whole millimetre (a 19.98 pin in a 20 bore is 20, a 16 plunger in a 16.2 bore is 16)."""
    d = f['d']
    if f.get('gap', 0) < 0.001:
        return d
    ds = round(d - 2 * f['gap'], 3)
    return ds if ds == round(ds) else d


IDLER = ('gear', 'pulley', 'wheel', 'sheave', 'sprocket')
SPLINE_RECORDS = 6


def _idler(pin, outer, fits):
    """A gear or pulley that turns on a pin fixed in a third part: the pin is located in that part (a fit within its
    group) and the bearing's outer ring sits in the gear. The gear's loads stay in the frame the pin is fixed to, so
    the outer ring turns under a load that stays still, SKF's rotating outer ring load."""
    held = any(f['shaft'] == pin and f['bore'] != outer and f.get('shaft_top') == f.get('bore_top') for f in fits)
    return held and any(w in outer.lower() for w in IDLER)


def _family(material):
    return 'aluminium' if 'alumin' in (material or '') else 'steel'


def standing(c):
    """The top-level groups that do not move: the grounded ones and every one joined to them, directly or through
    others, by a rigid joint (a census since the third records the joints; before it, the grounded ones alone)."""
    still = {t for t, g in c.get('tops', {}).items() if g} | {None}
    rigid = [(a, b) for a, b, r in c.get('links', []) if r]
    grew = True
    while grew:
        grew = False
        for a, b in rigid:
            if (a in still) != (b in still):
                still |= {a, b}
                grew = True
    return still - {None}


def clusters(c):
    """The top-level groups joined rigidly, directly or through others, as one cluster each; the ground (None) and the
    grounded groups one cluster with them. Returns (cluster of each group, the pairs of clusters a moving joint joins,
    the clusters any moving joint touches)."""
    tops = set(c.get('tops', {})) | {None}
    parent = {t: t for t in tops}

    def root(t):
        while parent[t] != t:
            t = parent[t]
        return t
    for t, g in c.get('tops', {}).items():
        if g:
            parent[root(t)] = root(None)
    links = [(a, b, r) for a, b, r in c.get('links', []) if a in parent and b in parent]
    for a, b, r in links:
        if r:
            parent[root(a)] = root(b)
    cl = {t: root(t) for t in tops}
    moving = {frozenset((cl[a], cl[b])) for a, b, r in links if not r and cl[a] != cl[b]}
    touched = {x for pr in moving for x in pr}
    return cl, moving, touched


def moves_against(c, a, b):
    """Whether two top-level groups move against each other in the model. Where the model has joints: not if one rigid
    cluster holds both, or if no moving joint touches either; if a moving joint joins them, or touches each, they do.
    Where it has none, its script moves the groups themselves (Volume 2's set_state), so two groups move."""
    if not c.get('links'):
        return a != b                                   # no joints in the model: its script moves the groups themselves
    cl, moving, touched = clusters(c)
    if cl.get(a) == cl.get(b):
        return False
    if frozenset((cl.get(a), cl.get(b))) in moving:
        return True
    return cl.get(a) in touched and cl.get(b) in touched


def decide(key, c=None):
    """Every joint of one assembly with its fit and roughness, and every made kind with its material; from the census
    (results/census_<key>.json) or from records of the same form made on a live model (MaterialsFits.py). Each seat and
    fit record is marked with its class ('cls'), its roughness ('ra', or 'rz' for SKF's plain bearings) and its kind
    of joint ('joint') as well. A seat the model leaves open by up to 0.2 mm is decided like a touching one and listed
    under 'gapped': the drawing gives it its class, the model has to be closed."""
    c = c or census(key)
    top_of = {x['name']: x.get('top') for x in c['components']}
    still = standing(c)
    moves = lambda name: top_of.get(name) not in still
    kinds = {}
    for x in c['components']:
        if x['status'] == 'made':
            kinds.setdefault(x['kind'], x['material'])
    mats = {k: material(key, k, m) for k, m in kinds.items()}
    joints = []
    seats = [s for s in c['seats'] if s.get('face') != 'sides']
    gapped = [s for s in seats if s.get('gap', 0) >= 0.001]
    by_brg = {}
    for s in seats:
        if s['designation'].startswith(('Bearing', 'Rod end')):
            by_brg.setdefault((s['designation'], _bearing_id(s)), []).append(s)
    for (dsg, bid), ss in sorted(by_brg.items()):
        label = bid.split('+')[-1].split(':')[0]
        t = bearing_type(dsg)
        if dsg.startswith('Rod end'):
            # SKF's SI rod ends have a female thread in the shank: the face found on the rod is the thread's
            for x in ss:
                x.update(joint='thread')
            joints.append(dict(kind='thread', bearing=label, designation=dsg, part=ss[0]['made'], d=ss[0]['d'],
                               note='the rod end screwed on the %s: a thread, its fit the thread\'s' % ss[0]['made']))
            continue
        inner = [s for s in ss if s['face'] == 'bore']
        outer = [s for s in ss if s['face'] == 'outside']
        d = inner[0]['d'] if inner else None
        Dd = outer[0]['d'] if outer else None
        housing_mat = mats.get(outer[0]['made'], ('',))[0] if outer else ''
        open_ = sorted({x['made'] for x in ss if x.get('gap', 0) >= 0.001})
        if t == 'plain':
            fit_kind = 'clearance' if any(x.get('gap', 0) >= 0.001 for x in inner) else 'interference'
            s_cls, h_cls = PLAIN_SHAFT[fit_kind], PLAIN_HOUSING[_family(housing_mat)]
            for x in inner:
                x.update(cls=s_cls, ra=None, rz=PLAIN_RZ[0], joint='plain bearing seat')
            for x in outer:
                x.update(cls=h_cls, ra=None, rz=PLAIN_RZ[1], joint='plain bearing seat')
            joints.append(dict(kind='bearing', bearing=label, designation=dsg, type=t, case=fit_kind,
                               shaft=inner[0]['made'] if inner else None, d=d, shaft_cls=s_cls, shaft_rz=PLAIN_RZ[0],
                               housing=outer[0]['made'] if outer else None, D=Dd, housing_cls=h_cls,
                               housing_rz=PLAIN_RZ[1], housing_material=housing_mat, hardened=fit_kind == 'clearance',
                               open=[], decided=bool(inner and outer)))
            continue
        if t == 'screw drive':
            s_cls, h_cls = SCREW_SEATS
            s_ra, h_ra = _seat_ra(5, d) if d else None, _seat_ra(5, Dd) if Dd else None
            for x in inner:
                x.update(cls=s_cls, ra=s_ra, joint='bearing seat')
            for x in outer:
                x.update(cls=h_cls, ra=h_ra, joint='bearing seat')
            joints.append(dict(kind='bearing', bearing=label, designation=dsg, type=t, case='screw drive',
                               shaft=inner[0]['made'] if inner else 'the ball screw (bought)', d=d, shaft_cls=s_cls,
                               shaft_ra=s_ra, housing=outer[0]['made'] if outer else None, D=Dd, housing_cls=h_cls,
                               housing_ra=h_ra, housing_material=housing_mat, open=open_,
                               decided=bool(outer), on_bought=not inner))
            continue
        im = moves(inner[0]['made_name']) if inner else False
        om = moves(outer[0]['made_name']) if outer else False
        if inner and outer and top_of.get(inner[0]['made_name']) == top_of.get(outer[0]['made_name']):
            im, om = True, False                    # both seats in one rigid group: the shaft taken as the one turning
        if inner and outer and _idler(inner[0]['made'], outer[0]['made'], c['fits']):
            im, om = False, True                    # a gear turning on a pin fixed in its frame, the load in the frame
        seat = bearing_seat(dsg, d or 0, Dd or 0, im, om)
        h_cls = seat['housing']
        if 'aluminium' in housing_mat and h_cls == 'H7':
            h_cls = 'J7'                             # SKF: other housing materials need other fits; one class tighter
        for x in inner:
            x.update(cls=seat['shaft'], ra=seat['shaft_ra'], joint='bearing seat')
        for x in outer:
            x.update(cls=h_cls, ra=seat['housing_ra'], joint='bearing seat')
        joints.append(dict(kind='bearing', bearing=label, designation=dsg, type=t, case=seat['case'],
                           shaft=inner[0]['made'] if inner else None, d=d, shaft_cls=seat['shaft'],
                           shaft_ra=seat['shaft_ra'] if inner else None,
                           housing=outer[0]['made'] if outer else None, D=Dd, housing_cls=h_cls,
                           housing_ra=seat['housing_ra'] if outer else None, housing_material=housing_mat,
                           open=open_, decided=bool(inner and outer)))
    # the shafts the bearings' inner rings sit on, by part and diameter: a hub on the same seat takes its class
    # (one class on that diameter; where bearings give it two, each fit keeps its own class, a zone of its own)
    seat_cls = {}
    for j in joints:
        if j['kind'] == 'bearing' and j.get('shaft') and j.get('d'):
            seat_cls.setdefault((j['shaft'], j['d']), set()).add((j['shaft_cls'], j.get('shaft_ra'), j.get('shaft_rz')))
    on_seat = {k: list(v)[0] for k, v in seat_cls.items() if len(v) == 1}
    # dowels and pins in made parts
    for s in seats:
        if s['group'] != 'pins and keys':
            continue
        if s['designation'].startswith(('Dowel', 'Pin ', 'ISO 8734')):
            joints.append(dict(kind='dowel hole', part=s['made'], pin=s['designation'], d=s['d'], hole_cls=DOWEL_HOLE,
                               pin_cls=DOWEL, fit=None, ra=_seat_ra(7, s['d'])))
            s.update(cls=DOWEL_HOLE, ra=_seat_ra(7, s['d']), joint='dowel hole')
    # keys: the two made parts each key touches, the shaft the one that fits in the other
    keyed_parts = set()
    by_key = {}
    for s in c['seats']:
        if s.get('face') == 'sides':
            by_key.setdefault(s['occ'], []).append(s)
    shafts_in = {(f['shaft'], f['bore']) for f in c['fits']}
    for occ, ss in sorted(by_key.items()):
        names = [x['made'] for x in ss]
        shaft_ = next((a for a in names for b in names if (a, b) in shafts_in), None)
        if shaft_ is not None:
            hubs = [n for n in names if (shaft_, n) in shafts_in]
            other = None
        else:
            # one made part: the key's other member is bought (a ball screw, a ball nut), its seat the maker's
            shaft_, hubs = (names[0], []) if any(w in names[0].lower() for w in ('shaft', 'pin')) else (None, names[:1])
            other = 'bought'
        for x in ss:
            role = 'shaft' if x['made'] == shaft_ else ('hub' if x['made'] in hubs else None)
            if role:
                x.update(cls=KEYS_SHAFT if role == 'shaft' else KEYS_HUB, ra=OTHER_RA, joint='key seat')
                keyed_parts.add(x['made'])
            else:
                x.update(joint="touches the key's end")
        joints.append(dict(kind='key seat', key=ss[0]['designation'], shaft=shaft_ or 'bought', hub=hubs or ['bought'],
                           ends=[n for n in names if n != shaft_ and n not in hubs], shaft_cls=KEYS_SHAFT,
                           hub_cls=KEYS_HUB, other=other))
    # made in made. A spline's teeth meet on many short cylinders of one diameter: six or more records between one
    # pair of parts are a spline, whose fit is its standard's, not a cylinder's
    pair_of = lambda f: (f.get('shaft_occ', f['shaft_name']), f.get('bore_occ', f['bore_name']), f['d'])
    n_pair = {}
    for f in c['fits']:
        n_pair[pair_of(f)] = n_pair.get(pair_of(f), 0) + 1
    for k_, n_ in sorted(n_pair.items()):
        if n_ >= SPLINE_RECORDS:
            f = [x for x in c['fits'] if pair_of(x) == k_][0]
            joints.append(dict(kind='spline', shaft=f['shaft'], bore=f['bore'], d=f['d'], records=n_,
                               note='a spline: its fit is the spline standard\'s'))
    for f in c['fits']:
        if n_pair[pair_of(f)] >= SPLINE_RECORDS:
            f.update(joint='spline')
            continue
        pair = (key, f['shaft'], f['bore'])
        note = None
        drawn = 2000.0 * f.get('gap', 0.0)             # um across: what the model leaves between the two
        if moves_against(c, f.get('shaft_top'), f.get('bore_top')):
            h, sh, kind = 'H7', 'g6', 'sliding'
        elif fit_known('H7', 'h6', nominal(f)) and drawn > fit('H7', 'h6', nominal(f))[1]:
            # looser than any located fit can be: the model means the part to move, as the vacuum cups' plungers do
            h, sh, kind = 'H7', 'g6', 'sliding'
        elif pair in PRESSED:
            # p6 unless the joint warms out of it: a steel part in an aluminium boss loses interference with heat
            h, kind = 'H7', 'pressed'
            sh = 'p6' if press_tube('p6')['holds'] else 's6'
        elif f['shaft'] in keyed_parts and f['bore'] in keyed_parts:
            h, sh, kind = 'H7', 'k6', 'keyed'
        else:
            h, sh, kind = 'H7', 'h6', 'located'
        d = nominal(f)
        ra = _seat_ra(int(sh[1:]), d)                    # each face's Ra by its own grade (SKF's guideline values)
        rz = None
        if kind in ('keyed', 'located') and (f['shaft'], d) in on_seat:
            (sh, ra, rz), note = on_seat[(f['shaft'], d)], 'on the bearing\'s seat'
        ra_hole = _seat_ra(int(h[1:]), d)
        if f.get('gap', 0) >= 0.001:
            note = 'drawn clear by %.3f mm' % f['gap']
        joints.append(dict(kind=kind, shaft=f['shaft'], bore=f['bore'], d=d, hole_cls=h, shaft_cls=sh,
                           fit=fit_known(h, sh, d), ra=ra, rz=rz, ra_hole=ra_hole, note=note, gap=f.get('gap', 0)))
        f.update(cls='%s/%s' % (h, sh), ra=ra, rz=rz, ra_hole=ra_hole, joint=kind)
    # one diameter of one part given two classes: the drawing splits the face into zones, each with its length
    by_face = {}
    for j in joints:
        if j['kind'] == 'bearing' and j.get('shaft') and j.get('d'):
            by_face.setdefault((j['shaft'], j['d']), set()).add((j['shaft_cls'], 'bearing seat'))
        elif j['kind'] in ('sliding', 'located', 'keyed', 'pressed'):
            by_face.setdefault((j['shaft'], j['d']), set()).add((j['shaft_cls'], j['kind']))
    zones = [dict(part=k[0], d=k[1], classes=sorted(v)) for k, v in sorted(by_face.items())
             if len({c for c, _ in v}) > 1]
    return dict(key=key, materials=mats, joints=joints, gaps=c.get('gaps', []), gapped=gapped, zones=zones)


if __name__ == '__main__':
    print('g6 at 100:', shaft('g6', 100), ' H7 at 100:', hole('H7', 100), ' fit:', fit('H7', 'g6', 100))
    print('k6 at 30:', shaft('k6', 30), ' p6 at 20:', shaft('p6', 20), ' H7/p6 at 20:', fit('H7', 'p6', 20))
    print(bearing_seat('Bearing 6206', 30, 62, True, False))
