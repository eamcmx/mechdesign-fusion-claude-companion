# -*- coding: utf-8 -*-
"""standard_parts.py: Chapter 50 of Volume 3, standard parts. Every bought designation of the eight assemblies Volume 3
details (detail_inventory.py) turned into what a buyer orders: its standard or maker, its full designation, the source
of its data, and where the standard does not make it as the concept drew it, the change. The data come from sources
saved in the references' Web pages folder (makers' pages; Fasten.it's tables of the fastener standards, read cell for
cell by book/tools/norm_tables.py) and from Shigley and Volume 1 for the strengths and the formulas.

    python standard_parts.py            -> the decisions, group by group, the counts by status, the worked examples

Statuses:
  as named   the name on the parts list is already the maker's full designation
  completed  completed here: the standard, the grade or class, the finish, or the maker's variant chosen
  changed    the standard does not make it as the concept drew it (a length, a height): the nearest that is made
  specified  bought to a specification Volume 2 worked out (torque, speed, stroke); the maker's product is chosen
             with the machine's detail (Chapters 52 to 55)
  to drawing bought, but made to the designer's drawing (a ball screw's machined ends, a spring)
  open       a question the machine's own detail answers first (Chapters 52 to 55), named in the note
"""
import io
import math
import os
import re

import detail_inventory as D

REFS = os.path.join(os.path.expanduser('~'), 'Documents', 'Mechanical Design References', 'Web pages')
DAY = '2026-11-03'

# makers' pages saved in the references (file -> the designation it describes)
PAGES = {'SKF_32213_2026-11-01.md': 'Bearing 32213', 'SKF_32308_2026-11-01.md': 'Bearing 32308',
         'SKF_32309_2026-11-01.md': 'Bearing 32309', 'SKF_22207E_2026-11-02.md': 'Bearing 22207 E',
         'SKF_22213E_2026-11-02.md': 'Bearing 22213 E', 'SKF_61836_2026-11-02.md': 'Bearing 61836',
         'SKF_61856MA_2026-11-02.md': 'Bearing 61856 MA', 'SKF_6002-2RSH_2026-11-02.md': 'Bearing 6002-2RSH',
         'SKF_61902-2RS1_2026-11-02.md': 'Bearing 61902-2RS1', 'SKF_BSD2562CGA_2026-11-02.md': 'Bearing BSD 2562 CGA',
         'SKF_GE20ES_2026-11-02.md': 'Bearing GE 20 ES', 'SKF_SI20ES_2026-11-02.md': 'Rod end SI 20 ES',
         'SKF_slewing_bearings_2026-11-02.md': 'Slewing ring', 'SKF_6206-2RS1_2026-11-03.md': 'Bearing 6206'}
# designations whose data come from a table or a catalogue Volume 2 cites (the source as the book cites it)
CITED = [
    (r'^Bearing 62\d\d$', 'Budynas & Nisbett (2015), Table 11-2, p. 573'),
    (r'^RV-\d+N$', 'Nabtesco Precision Europe GmbH (n.d.)'),
    (r'^Rail HGR|^Block HGH|^Ball ?screw|^Ball nut|^Fixed bearing BK|^Supported bearing BF', 'HIWIN GmbH (2017, 2022)'),
    (r'^Cylinder DSBC', 'Festo AG & Co. KG (2012)'),
    (r'^Cup SPB|^Ejector', 'J. Schmalz GmbH (n.d.)'),
    (r'^Belt HTD', 'Gates Corporation (n.d.)'),
    (r'^Spring$', 'designed to Budynas & Nisbett (2015), Chapter 10 (Chapter 46)'),
    (r'^Spring 4 x 40$', 'designed to Budynas & Nisbett (2015), Chapter 10 (Chapter 48)'),
]
# Fasten.it's tables of the standards (book/tools/norm_tables.py)
FASTEN = {n: 'Fasten.it_%s_%s.md' % (n, DAY) for n in
          ('DIN_EN_ISO_4762', 'DIN_EN_ISO_10642', 'DIN_EN_ISO_4017', 'DIN_EN_ISO_4014', 'DIN_EN_ISO_4032',
           'DIN_EN_ISO_7089', 'DIN_EN_ISO_7093-1', 'DIN_EN_ISO_8734', 'DIN_EN_ISO_8675', 'DIN_EN_ISO_2338',
           'DIN_6885-1', 'DIN_471', 'DIN_472', 'DIN_908', 'DIN_939')}

# ---- strengths and factors (book sources) ---------------------------------------------------------------------
AT = {3: 5.03, 4: 8.78, 5: 14.2, 6: 20.1, 8: 36.6, 10: 58.0, 12: 84.3, 14: 115.0, 16: 157.0}  # mm2, Shigley Table 8-1, p. 404
SP = {'8.8': 580.0,      # MPa, ISO 898-1, d <= 16 mm, as Volume 1's Chapter 8 takes it
      '10.9': 830.0}     # MPa, Shigley Table 8-11, p. 427 (Volume 2, Chapter 44)
K_ZINC = 0.20            # Shigley Table 8-15, p. 431: zinc-plated
PRELOAD = 0.75           # Fi = 0.75 Fp for a reusable joint (Shigley Eq. 8-31, Volume 1's Chapter 8)
SY_KEY = 370.0           # MPa, 1018 CD, Shigley Table A-20, p. 1048: the key stock of Example 7-6
N_KEY = 1.5              # the design factor of Shigley's Example 7-6, pp. 386-387, a static peak load
KEY_MAX = 1.5            # a key at most 1.5 d long (Shigley p. 383)
SCREW_IN = 1.5           # a cap screw in a tapped hole: L > h + 1.5 d (Shigley Table 8-7, p. 418)
DEFAULT = dict(cls='8.8', finish='zinc-plated')   # Emmanuel's default, 3 November 2026


# ---- reading the saved sources ----------------------------------------------------------------------------------
def md_tables(fname):
    """The Markdown tables of a saved page: [(caption, [rows of cells])], the caption the '## ' line above."""
    out, cap, rows = [], '', None
    for line in io.open(os.path.join(REFS, fname), encoding='utf-8'):
        line = line.rstrip('\n')
        if line.startswith('## '):
            cap = line[3:]
        if line.startswith('|'):
            cells = [c.strip() for c in line.strip().strip('|').split('|')]
            if set(''.join(cells)) <= set('-: '):
                continue
            if rows is None:
                rows = []
                out.append((cap, rows))
            rows.append(cells)
        else:
            rows = None
    return out


def page_data(fname):
    """The two-column rows of a saved maker's page: {item: value}."""
    return {r[0]: r[1] for cap, rows in md_tables(fname) for r in rows if len(r) == 2 and r[0] != 'Item'}


def num(s):
    return float(re.search(r'-?\d+(?:\.\d+)?', s.replace(' ', '')).group(0))


def size_key(s):
    """'M 8', 'M8' -> '8'; '10' -> '10'."""
    return re.sub(r'^M\s*', '', s.strip())


def commercial(norm):
    """{size: [lengths]} from the standard's table of commercial items (a mass or an X where it is made)."""
    for cap, rows in md_tables(FASTEN[norm]):
        if 'commercial' not in cap.lower():
            continue
        sizes = [size_key(c) for c in rows[0][1:]]
        out = {}
        for r in rows[1:]:
            if not re.match(r'^\d+(\.\d+)?$', r[0]):
                continue
            for s, c in zip(sizes, r[1:]):
                if c:
                    out.setdefault(s, []).append(float(r[0]))
        if out:
            return out
    return None


def din6885_row(d):
    """DIN 6885-1's key for a shaft of d mm: (b, h, t1, t2), the row whose range holds d (over, up to and with)."""
    cap, rows = md_tables(FASTEN['DIN_6885-1'])[0]
    col = {r[0]: r[1:] for r in rows}
    for k, rng in enumerate(col['for shaft diameters d1 (over÷to)']):
        lo, hi = [float(x) for x in rng.split('÷')]
        if lo < d <= hi:
            return tuple(float(col[n][k]) for n in ('key width b', 'key heigth h', 'depth t1', 'groove depth t2'))
    raise ValueError(d)


def shortest(lengths, at_least):
    """The shortest commercial length not under at_least, or None."""
    ok = [L for L in sorted(lengths) if L >= at_least - 1e-9]
    return ok[0] if ok else None


# ---- the fasteners: each name on the parts lists, its standard and class --------------------------------------
# std, d, L (None for a nut, a washer or a stud with no length), class, and the reason the class or standard differs
# from the default. Every screw Volume 2 modelled has the cap screw's head (mdfc_parts.cap_screw), whatever its name.
F = dict
FASTENERS = {
    'Cap screw ISO 4762 M3x8': F(std='ISO 4762', d=3, L=8), 'Cap screw ISO 4762 M4x10': F(std='ISO 4762', d=4, L=10),
    'Cap screw ISO 4762 M4x12': F(std='ISO 4762', d=4, L=12), 'Cap screw ISO 4762 M5x12': F(std='ISO 4762', d=5, L=12),
    'Cap screw ISO 4762 M5x16': F(std='ISO 4762', d=5, L=16), 'Cap screw ISO 4762 M8x20': F(std='ISO 4762', d=8, L=20),
    'Countersunk screw ISO 10642 M3x8': F(std='ISO 10642', d=3, L=8),
    'Countersunk screw ISO 10642 M4x12': F(std='ISO 10642', d=4, L=12),
    'Cap screw M10x25': F(std='ISO 4762', d=10, L=25, cls='10.9', why='the ISO 9409-1 joint of the tool plate, '
                          'worked in class 10.9 (Chapter 44)'),
    'Cap screw M10x35': F(std='ISO 4762', d=10, L=35, cls='10.9', why='the changer\'s flange, class 10.9 (Chapter 44)'),
    'Cap screw M6x14': F(std='ISO 4762', d=6, L=14, h=5.0, plate=10.0,
                         why='the plate\'s screws pass the beam\'s 5 mm wall into a plate 10 mm thick'),
    'Cap screw M6x16': F(std='ISO 4762', d=6, L=16), 'Cap screw M8x20': F(std='ISO 4762', d=8, L=20),
    'Cap screw M6x20': F(std='ISO 4762', d=6, L=20, h=13.0, plate=20.0,
                         why='the compliance unit\'s rails: HIWIN\'s M6 x 20 for the HGR25R (2017, Table 3.9, p. 35), '
                             'through the rail\'s 13 mm under the head into the aluminium cheek, 20 thick'),
    'Cap screw M8x25': F(std='ISO 4762', d=8, L=25), 'Cap screw M8x25 nut': F(std='ISO 4762', d=8, L=25,
                                                                            why='the ball nuts\' flange screws'),
    'Cap screw M8x30': F(std='ISO 4762', d=8, L=30),
    'Cover screw M5x16': F(std='ISO 4762', d=5, L=16), 'Motor screw M5x12': F(std='ISO 4762', d=5, L=12),
    'End bolt M8x20': F(std='ISO 4762', d=8, L=20), 'Stop bolt M8x16': F(std='ISO 4762', d=8, L=16),
    'Bracket bolt M8x25': F(std='ISO 4762', d=8, L=25, cls='10.9', why='the hinge bracket\'s bolts, class 10.9 '
                            '(Chapter 34)'),
    'Fitting bolt M8x35': F(std='ISO 4762', d=8, L=35, cls='10.9', why='the spar fitting\'s bolts, class 10.9 '
                            '(Chapter 34)'),
    'Bracket nut M8': F(std='ISO 4032', d=8, cls='10', why='the nut of a class 10.9 bolt'),
    'Fitting nut M8': F(std='ISO 4032', d=8, cls='10', why='the nut of a class 10.9 bolt'),
    'Cover screw M8x20': F(std='ISO 4762', d=8, L=20, why='ISO 4762 (Chapter 30)'),
    'Crown wheel bolt M12x35': F(std='ISO 4762', d=12, L=35, cls='10.9', why='ISO 4762, class 10.9 (Chapter 29)'),
    'Cap bolt M12x90': F(std='ISO 4762', d=12, L=90, cls='10.9', why='the bearing caps\' screws, ISO 4762, class 10.9 '
                         '(Chapter 29)'),
    'Carrier nut M12': F(std='ISO 4032', d=12, cls='10', why='class 10 (Chapter 30)'),
    # the rear axle's and the flap actuator's specials: Chapters 53 and 54 decide them
    'Carrier stud M12': F(open='class 10.9 (Chapter 30); its length and its metal end follow the housing\'s flange '
                               '(Chapter 54)'),
    'Wheel stud M14': F(open='class 10.9 (Chapter 30); a wheel stud comes with the hub and takes the wheel nut\'s '
                             'thread (Chapter 54)'),
    'Drain plug M22': F(open='a plug\'s thread and seal follow the housing\'s boss and the oil'),
    'Filler plug M22': F(open='as the drain plug'),
    'Pinion nut M30': F(open='the pinion nut sets the pinion bearings\' preload (Chapter 54)'),
    'Washer': F(open='the pinion nut\'s washer, 27 mm across, made with the nut\'s seat'),
    'Hinge pin nut': F(open='the hinge pin\'s nut and its locking follow the pin (Chapter 53)'),
    'Horn pin nut': F(open='as the hinge pin\'s'),
    'Jam nut M20x1.5': F(open='the rod end\'s jam nut: a thin nut of fine pitch (ISO 8675) once the rod end\'s '
                              'thread is fixed (Chapter 53)'),
    'Ring nut M66x1.5': F(open='the ring nut that holds the screw\'s bearings: a lock nut to the bearings\' maker\'s '
                               'instructions (Chapter 53)'),
    'Washer M8': F(open='modelled 30 mm across and 3 mm thick, neither ISO 7089 (16 x 1.6) nor ISO 7093-1 (24 x 2): an '
                        'end washer to be made, or the end redrawn for ISO 7093-1 (Chapter 53)'),
}
STD_TABLE = {'ISO 4762': 'DIN_EN_ISO_4762', 'ISO 10642': None, 'ISO 4032': None}   # 10642: no commercial table


def fastener(name):
    """The decision for one fastener name: its full designation, status, source and note."""
    f = FASTENERS[name]
    if 'open' in f:
        return dict(full=name, status='open', source=None, note=f['open'])
    cls = f.get('cls', DEFAULT['cls'])
    fin = DEFAULT['finish']
    src = 'Fasten.it (n.d.), %s' % f['std']
    if f['std'] == 'ISO 4032':
        return dict(full='%s M%d - %s - %s' % (f['std'], f['d'], cls, fin), status='completed', source=src,
                    note=f.get('why', ''))
    full = '%s M%d x %d - %s - %s' % (f['std'], f['d'], f['L'], cls, fin)
    norm = STD_TABLE.get(f['std'])
    if norm:
        Ls = commercial(norm)[str(f['d'])]
        need = f['h'] + SCREW_IN * f['d'] if 'h' in f else f['L']
        short = 'h' in f and f['L'] <= need                  # Shigley's L > h + 1.5 d, where the joint gives h
        if f['L'] not in Ls or short:
            nxt = shortest(Ls, need)
            if 'plate' in f and nxt - f['h'] > f['plate']:
                return dict(full=full, status='open', source=src,
                            note='%d is not a commercial length; L > h + 1.5 d = %.0f needs %d, which would pass '
                                 'through the %.0f mm plate: the joint changes with the vacuum gripper\'s detail '
                                 '(Chapter 55)' % (f['L'], need, nxt, f['plate']))
            why = ('%d is not a commercial length' % f['L'] if f['L'] not in Ls else
                   'L > h + 1.5 d = %.0f: the %d leaves %.0f mm of thread, the %d leaves %.0f' %
                   (need, f['L'], f['L'] - f['h'], nxt, nxt - f['h']))
            return dict(full='%s M%d x %d - %s - %s' % (f['std'], f['d'], nxt, cls, fin), status='changed',
                        source=src, note=why)
    return dict(full=full, status='completed', source=src, note=f.get('why', ''))


# ---- the other bought parts ------------------------------------------------------------------------------------
O = dict
OTHERS = {
    # bearings: SKF's pages give the full designations; the three named only by size are completed here or left open
    'Bearing 6206': O(full='SKF 6206-2RS1', status='completed', note='sealed both sides with contact seals, greased '
                      'for life: the bag gripper works in the bags\' dust'),
    'Bearing 6201': O(status='open', note='the flap actuator\'s gear stage: sealed or open follows how the stage is '
                      'lubricated (Chapter 53)'),
    'Bearing 6213': O(status='open', note='the rear axle\'s wheel bearings: sealed or open follows the oil seals\' '
                      'places (Chapter 54)'),
    'Fixed bearing BK25': O(status='as named', note='HIWIN\'s fixed support unit for the 32 mm screw'),
    'Supported bearing BF25': O(status='as named', note='HIWIN\'s floating support unit'),
    'Slewing ring': O(full='SKF RKS.23 0841', status='completed', note='the ring Chapter 42 chose from SKF\'s light '
                      'series without a gear'),
    'Slip ring': O(status='specified', note='the wrist\'s slip ring: channels and current from Chapter 40'),
    # transmission parts
    'Ball nut R32-10K5-DEB left': O(status='as named'), 'Ball nut R32-10K5-DEB right': O(status='as named'),
    'Ball nut R32-10T4-RSI': O(status='as named'),
    'Ball screw 32x10': O(status='to drawing', note='the screw\'s ends are machined to the designer\'s drawing'),
    'Ballscrew 32x10 right and left': O(status='to drawing', note='as the flap actuator\'s screw'),
    'Belt HTD 350-5M-15': O(status='as named'),
    'Block HGH25CA': O(status='specified', note='the preload and accuracy classes in HIWIN\'s code are chosen with the '
                       'clamp gripper\'s and the compliance unit\'s detail'),
    'Rail HGR25': O(status='specified', note='the clamp gripper\'s rails, 1720 long; the accuracy class with the '
                    'blocks\''),
    'Rail HGR25 218': O(status='specified', note='the compliance unit\'s rails, 218 long; the accuracy class with the '
                        'blocks\''),
    'Coupling': O(status='specified', note='torque and bores from Chapter 45'),
    'Gearhead 3:1': O(status='specified', note='ratio 3, the motor\'s flange'),
    'RV-500N': O(full='Nabtesco RV-500N, ratio 192.75', status='completed', note='J2 and J3 (Chapter 42)'),
    'RV-700N': O(full='Nabtesco RV-700N, ratio 203.52', status='completed', note='J1 (Chapter 43)'),
    'RV-80N': O(full='Nabtesco RV-80N, ratio 101', status='completed', note='J4 (Chapter 43)'),
    # motors, cylinders and vacuum
    'Cup SPB1 80': O(full='Schmalz SPB1 80 ED-65 SC065, article 10.01.06.03071', status='completed',
                     note='the data sheet Chapter 46 used'),
    'Ejector SCPSi 2-14': O(status='specified', note='the variant (supply, signals) with the vacuum gripper\'s detail'),
    'Cylinder DSBC-32-50': O(status='specified', note='cushioning and position sensing in Festo\'s type code'),
    'Cylinder DSBC-80-125-C': O(status='specified', note='as the press cylinder, with the clamping unit'),
    'Encoder': O(status='specified'), 'Motor unit': O(status='specified'), 'Motor with brake': O(status='specified'),
    'Servo motor J1': O(status='specified'), 'Servo motor J2': O(status='specified'),
    'Servo motor J3': O(status='specified'), 'Servo motor J4': O(status='specified'),
    'Servo motor with brake and 15:1 gearhead': O(status='specified'),
    # pins and keys
    'Dowel ISO 8734 5x16': O(std=('DIN_EN_ISO_8734', '5', 16)),
    'Dowel pin 10x17': O(std=('DIN_EN_ISO_8734', '10', 17), note='8 mm in the adapter or the compliance unit\'s top '
                         'plate and 9 in the tool plate'),
    'Pin 10 robot': O(std=('DIN_EN_ISO_8734', '10', 18), note='the robot side\'s locating pin, 8 + 10 mm'),
    'Pin 12 tool': O(std=('DIN_EN_ISO_8734', '12', 24), note='the tool side\'s locating pins, 12 + 12 mm'),
    'Key DIN 6885 A 5x5x18': O(key=(5, 5, 18)), 'Key DIN 6885 A 5x5x8': O(key=(5, 5, 8)),
    'Key 8x7x16': O(key=(8, 7, 16)),
    'Nut key 6x6.5x32': O(status='open', note='DIN 6885-1 makes a key 6 wide 6 high; a key 6.5 high is made to the '
                                              'ball nut\'s keyway (Chapter 53)'),
    # seals, rings and springs
    'Retaining ring DIN 471-15': O(ring=('DIN_471', 15)), 'Retaining ring DIN 472-32': O(ring=('DIN_472', 32)),
    'Retaining ring': O(status='open', note='the rear axle\'s ring has no size on the list (Chapter 54)'),
    'Nut retaining ring': O(status='open', note='the flap actuator\'s nut ring follows the ring nut (Chapter 53)'),
    'Oil seal 74x100x10': O(status='specified', note='a radial shaft seal 74 x 100 x 10; lip and compound with the '
                            'axle\'s oil (Chapter 54)'),
    'Pinion seal 60x85x10': O(status='specified', note='a DIN 3760 lip seal (Chapter 29); lip and compound as the oil '
                              'seals\''),
    'Breather': O(status='specified', note='the axle\'s breather'),
    'Rod seal': O(status='specified', note='the flap actuator\'s rod seal, with its groove (Chapter 53)'),
    'Spring': O(status='to drawing', note='music wire, made to the spring\'s specification of Chapter 46'),
    'Spring 4 x 40': O(status='to drawing', note='music wire, made to the spring\'s specification of Chapter 48'),
    'O-ring 20x2.5': O(status='specified', note='the compliance unit\'s tube seal in the receiver: its size, compound '
                       'and groove to the seal maker\'s tables (Chapter 51)'),
    'Gallery plug 8': O(status='specified', note='the compliance unit\'s galleries, drilled 8 and plugged at the '
                        'edges: a plug that holds the air\'s 6 bar, its kind with the housing\'s detail (Chapter 55)'),
    # motors, cylinders and vacuum: the compliance unit's sensor
    'Sensor M12': O(status='specified', note='an M12 proximity sensor: its sensing distance on the aluminium wing, '
                    'assumed 4 mm in Chapter 48, and its output from the maker\'s data (Chapter 55)'),
}
AS_NAMED_PAGES = {v for v in PAGES.values()} - {'Bearing 6206', 'Slewing ring'}


def other(name):
    o = dict(OTHERS.get(name, {}))
    src = None
    by_page = {v: k for k, v in PAGES.items()}
    if name in by_page:
        src = 'saved page ' + by_page[name]
    else:
        src = next((s for pat, s in CITED if re.search(pat, name)), None)
    if 'std' in o:                                   # a dowel to ISO 8734: is its length made?
        norm, d, L = o['std']
        Ls = commercial(norm)[d]
        src = 'Fasten.it (n.d.), ISO 8734'
        if L in Ls:
            return dict(full='ISO 8734 %s x %d' % (d, L), status='completed', source=src, note=o.get('note', ''))
        nxt = shortest(Ls, L)
        return dict(full='ISO 8734 %s x %d' % (d, nxt), status='changed', source=src,
                    note='%d is not a commercial length; %s' % (L, o.get('note', '')))
    if 'key' in o:                                   # a key to DIN 6885-1: is its height and length made?
        b, h, L = o['key']
        src = 'Fasten.it (n.d.), DIN 6885-1'
        rows = {r[0]: r[1:] for r in md_tables(FASTEN['DIN_6885-1'])[0][1]}
        heights = dict(zip([float(x) for x in rows['key width b']], [float(x) for x in rows['key heigth h']]))
        Ls = commercial('DIN_6885-1')[str(int(b))]
        notes = []
        h2, L2 = h, L
        if heights.get(b) != h:
            h2 = heights[b]
            notes.append('a key %g wide is %g high' % (b, h2))
        if L not in Ls:
            L2 = shortest(Ls, L)
            notes.append('%d is not a commercial length' % L)
        full = 'DIN 6885-1 A %d x %d x %d' % (b, h2, L2)
        st = 'changed' if notes else ('as named' if name.startswith('Key DIN') else 'completed')
        return dict(full=full, status=st, source=src, note='; '.join(notes + ([o['note']] if 'note' in o else [])))
    if 'ring' in o:
        norm, d = o['ring']
        return dict(full=name.replace('Retaining ring ', '').replace('-', ' - '), status='as named',
                    source='Fasten.it (n.d.), %s' % norm.replace('_', ' '), note='')
    st = o.get('status', 'as named' if name in AS_NAMED_PAGES else 'specified')
    return dict(full=o.get('full', name), status=st, source=src, note=o.get('note', ''))


def decide(name):
    out = fastener(name) if name in FASTENERS else other(name)
    out['designation'] = name
    return out


def work_list(rows=None):
    """Every bought designation with its group, the assemblies that use it, its pieces, and the decision."""
    rows = rows or D.all_machines()
    use, pieces, group = {}, {}, {}
    for r in rows:
        for g, dd in r['groups'].items():
            for d, n in dd.items():
                use.setdefault(d, []).append(r['key'])
                pieces[d] = pieces.get(d, 0) + n
                group[d] = g
    out = []
    for d in sorted(use):
        x = decide(d)
        x.update(group=group[d], uses=use[d], pieces=pieces[d])
        out.append(x)
    return out


STATUSES = ('as named', 'completed', 'changed', 'specified', 'to drawing', 'open')


def counts(wl=None):
    wl = wl or work_list()
    return {s: sum(1 for w in wl if w['status'] == s) for s in STATUSES}


# ---- the worked examples ---------------------------------------------------------------------------------------
def bearing_6206():
    """The bag gripper's 6206: Shigley's table against SKF's page for the sealed 6206-2RS1, the static safety in the
    brake stop with each, and the shaft's shoulder against SKF's abutment."""
    import bag_gripper as B
    pg = page_data('SKF_6206-2RS1_%s.md' % DAY)
    skf = dict(d=num(pg['Bore diameter d']), D=num(pg['Outside diameter D']), B=num(pg['Width B']),
               C=num(pg['Basic dynamic load rating C']), C0=num(pg['Basic static load rating C0']),
               da_min=num(pg['Diameter of shaft abutment da min']), da_max=num(pg['Diameter of shaft abutment da max']),
               Da_max=num(pg['Diameter of housing abutment Da max']), ra_max=num(pg['Radius of shaft or housing fillet ra max']),
               d1=num(pg['Shoulder diameter d1']), D2=num(pg['Recess diameter D2']), m=num(pg['Product net weight']),
               n_lim=num(pg['Limiting speed']))
    shig = dict(D=62.0, B=16.0, C=19.5, C0=10.0, dS=35.0, dH=55.0, r=1.0)   # Table 11-2, p. 573
    br = B.design()['bearing']
    P = br['P'] / 1000.0
    shoulder = math.ceil(skf['da_min'])                      # the shaft's shoulder, a whole millimetre over da min
    return dict(skf=skf, shig=shig, P=P, s0_shig=shig['C0'] / P, s0_skf=skf['C0'] / P,
                shoulder_ch49=shig['dS'], shoulder=shoulder,
                ok49=skf['da_min'] <= shig['dS'] <= skf['da_max'], ok=skf['da_min'] <= shoulder <= skf['da_max'],
                housing_ok=shig['dH'] <= skf['Da_max'])


def tightening(d, cls='8.8'):
    """Preload and tightening torque of a zinc-plated screw (Shigley Eq. 8-27, K from Table 8-15)."""
    Fp = AT[d] * SP[cls]
    Fi = PRELOAD * Fp
    return dict(d=d, cls=cls, At=AT[d], Sp=SP[cls], Fp=Fp, Fi=Fi, K=K_ZINC, T=K_ZINC * Fi * d / 1000.0)


def lever_key():
    """The bag gripper's lever on its 30 mm stub shaft: DIN 6885-1's key, its length by Shigley's Example 7-6 at the
    brake stop's torque, the shortest commercial length, and the hub it needs against the 20 mm lever."""
    import bag_gripper as B
    des = B.design()
    T, Tc = des['jaw_stop']['M'], des['jaw_cycle']['M']      # N m
    d = B.LAY['stub_d']
    b, h, t1, t2 = din6885_row(d)
    F, Fc = 2 * T * 1000.0 / d, 2 * Tc * 1000.0 / d          # N at the shaft's surface
    Ssy = 0.577 * SY_KEY
    l_shear = F * N_KEY / (b * Ssy)
    l_crush = F * N_KEY / ((h - t1) * SY_KEY)                # the hub's side, h - t1 high (Volume 1, Chapter 5)
    l_need = max(l_shear, l_crush)
    Ls = commercial('DIN_6885-1')[str(int(b))]
    L = shortest(Ls, l_need)
    return dict(T=T, Tc=Tc, d=d, b=b, h=h, t1=t1, t2=t2, F=F, Fc=Fc, Ssy=Ssy, l_shear=l_shear, l_crush=l_crush,
                L=L, Ls=sorted(Ls), L_max=KEY_MAX * d, lever_t=B.LAY['lever_t'],
                n_shear=b * L * Ssy / F, n_crush=(h - t1) * L * SY_KEY / F,
                crush_cycle=Fc / ((h - t1) * L), shear_cycle=Fc / (b * L),
                crush_in_lever=F / ((h - t1) * B.LAY['lever_t']))


def dowel_adapter():
    """The grippers' adapter dowel, 10 x 17 in the concept (8 mm in the adapter, 9 in the tool plate): the shortest
    ISO 8734 dowel of 10 mm, and where its extra length goes."""
    import bag_gripper as B
    Ls = commercial('DIN_EN_ISO_8734')['10']
    L = shortest(Ls, 17)
    t_adapter = -B.LAY['adapter'][1]
    in_tool = 9.0
    return dict(L0=17, in_adapter0=8.0, in_tool=in_tool, L=L, Ls=sorted(Ls), in_adapter=L - in_tool,
                t_adapter=t_adapter, ok=L - in_tool < t_adapter)


def vacuum_m6():
    f = FASTENERS['Cap screw M6x14']
    Ls = commercial('DIN_EN_ISO_4762')['6']
    need = f['h'] + SCREW_IN * f['d']
    return dict(d=f['d'], L=f['L'], h=f['h'], plate=f['plate'], need=need, Ls=sorted(Ls), next=shortest(Ls, need),
                below=[x for x in sorted(Ls) if x < need][-1], engaged_below=[x for x in sorted(Ls) if x < need][-1] - f['h'])


# ---- the driven stub shaft in detail (the Fusion step of Chapter 50) -------------------------------------------
END_WASHER = dict(D=40.0, t=5.0, hole=9.0)       # made: a plain disc that clamps the hub, the spacer and the inner ring
SEAL_SET_IN = 0.5                                 # mm: the seals' faces set in from the rings' (not on SKF's page)


def stub_detail():
    """The bag gripper's driven stub shaft in detail, along the jaw's hinge (x, mm) from the tube's end: the shoulder
    at the bearing's abutment (bearing_6206), the 6206-2RS1 in the end plate's boss, a spacer to the lever's hub, the
    hub lengthened into a boss that holds the key of lever_key() with 2.5 mm at each end, the key's seats (DIN 6885-1:
    t1 in the shaft, t2 in the hub), and an end washer and screw that clamp hub, spacer and inner ring against the
    shoulder; the shaft ends 1 mm inside the hub's face so that the washer bears on the hub."""
    import bag_gripper as B
    L = B.LAY
    brg, key = bearing_6206(), lever_key()
    b, h, t1, t2 = din6885_row(L['stub_d'])
    boss = key['L'] + 5.0
    x_lev = L['lever_x']
    hub = (x_lev[1] - boss, x_lev[1])
    end = x_lev[1] - 1.0
    screw = FASTENERS['Cap screw M8x20']
    under_head = END_WASHER['t'] + (x_lev[1] - end)
    return dict(d=L['stub_d'], shoulder=brg['shoulder'], da_min=brg['skf']['da_min'], da_max=brg['skf']['da_max'],
                r_max=brg['skf']['ra_max'], x_tube=L['tube_x'], x_brg=L['bearing_x'], x_lever=x_lev, hub=hub, boss=boss,
                hub_r=25.0, key=(hub[0] + 2.5, hub[1] - 2.5), b=b, h=h, t1=t1, t2=t2, key_L=key['L'], key_need=key['l_crush'],
                spacer=(L['bearing_x'][1], hub[0]), end=end, washer=END_WASHER, screw=(screw['d'], screw['L']),
                under_head=under_head, thread=screw['L'] - under_head, thread_need=SCREW_IN * screw['d'],
                bore=(brg['skf']['d'], brg['skf']['D'], brg['skf']['B'], brg['skf']['d1'], brg['skf']['D2']),
                set_in=SEAL_SET_IN)


def detail_numbers(rows=None):
    """The part numbers of the kinds the detail adds to the bag gripper: its made parts the next free items of P13 in
    the order of their names, the key the next free serial; with their descriptions."""
    nb, sn = D.numbering(rows)
    made = [n for n, _, _, k in nb['bag'] if k == 'made']
    nxt = int(made[-1].split('-')[1]) + 1
    out = {}
    for k, name in enumerate(sorted(('End washer', 'Spacer'))):
        out[name] = ('P13-%03d' % (nxt + k), name)
    key = lever_key()
    out['Key %dx%dx%d' % (key['b'], key['h'], key['L'])] = ('S-%04d' % (len(sn) + 1),
                                                         'DIN 6885-1 A %d x %d x %d' % (key['b'], key['h'], key['L']))
    return out


if __name__ == '__main__':
    wl = work_list()
    for g in sorted({w['group'] for w in wl}):
        print('== %s' % g)
        for w in wl:
            if w['group'] == g:
                print('  %-40s %-11s %-48s %s' % (w['designation'], w['status'], w['full'], w['note'][:60]))
    print(counts(wl), len(wl))
    print(bearing_6206())
    print(tightening(8))
    print(lever_key())
    print(dowel_adapter())
    print(vacuum_m6())
