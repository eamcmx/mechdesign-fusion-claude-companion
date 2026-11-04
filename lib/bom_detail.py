# -*- coding: utf-8 -*-
"""bom_detail.py: Chapter 64 of Volume 3, the bills of materials. What each of the nine assemblies is made of after
Parts 9 and 10, line by line: the part number of Chapter 49's scheme with the numbers Chapters 50 to 56 added, the
description a shop or a buyer orders by, the quantity, the material or the maker, and the mass, Fusion's for a made
part and the maker's for a bought part wherever a saved page gives it.

The census itself is Fusion's: scripts/BillOfMaterials builds each machine through RangeSweep.py and hands every
occurrence that has a body, (name, mass kg, materials, volume cm3), to bom(). Nothing here reads the model.

Sources: lib/detail_inventory.py (Chapter 49: the names of Volume 2's parts lists and their numbers), lib/standard_parts.py
(Chapter 50: the full designations and the bag gripper's added kinds), lib/wrist_detail.py (Chapter 52), Chapter 53's
text (the flap: ch53_flap_detail.tex, the section on the numbers; no module holds them), lib/palletizer_detail.py
(Chapter 55), lib/range_sweep.py (Chapter 56), lib/materials_fits.py (Chapter 51: each made kind's material), and the
makers' pages in Documents/Mechanical Design References/Web pages (MAKER below, each with its page).
"""
import io
import os
import re

import detail_inventory as DI

# ---- what is not a part: the loads and the station the grippers' checks carry -----------------------------------------
LOADS = ('Bag', 'Roller', 'Carton', 'Pack', 'Station')

# ---- the kinds Volume 3 added or renumbered, by the start of the name Fusion gives them: (start, number, description,
# kind, chapter). kind: 'made', 'bought', 'with <number>' (a body that comes with a bought part), 'in <number>' (a part
# of a weldment). Tried before Chapter 49's names, the longest start first.
DETAIL = {
    'wrist': [('Cap screw ISO 4762 M3x10', 'S-0108', 'ISO 4762 M3 x 10 - 8.8 - zinc-plated', 'bought', 52),
              ('Key DIN 6885 A 5x5x10', 'S-0067', 'DIN 6885-1 A 5 x 5 x 10', 'bought', 52)],
    'flap': [('Cap screw ISO 4762 M4x10', 'S-0027', 'ISO 4762 M4 x 10 - 8.8 - zinc-plated', 'bought', 53),
             ('Pinion end washer', 'F01-025', 'Pinion end washer', 'made', 53),
             ('Bearing GE 20 ES fitting', 'S-0109', "SKF GE 20 ES-2LS/C3, C3's availability SKF's to confirm", 'bought', 53)],
    'bag': [('Shock absorber DYSC-25-25-Y1F', 'S-0120', 'Festo DYSC-25-25-Y1F, part no. 2480234', 'bought', 56),
            ('Lock nut DYSC-25-25-Y1F', None, 'with the DYSC-25-25-Y1F', 'with S-0120', 56),
            ('Cap screw ISO 4762 M8x40 stop', 'S-0121', 'ISO 4762 M8 x 40 - 8.8 - zinc-plated', 'bought', 56),
            ('Washer ISO 7089 8 stop', 'S-0122', 'ISO 7089 8 - 200 HV - zinc-plated', 'bought', 56),
            ('Nut ISO 4032 M8 stop', 'S-0123', 'ISO 4032 M8 - 8 - zinc-plated', 'bought', 56),
            ('Stop bracket closed', 'P13-017', 'Stop bracket, closing', 'made', 56),
            ('Stop bracket open', 'P13-018', 'Stop bracket, opening', 'made', 56),
            ('Circlip DIN 471 30x1.5', 'S-0111', 'DIN 471 30 x 1.5', 'bought', 55),
            ('Circlip DIN 472 62x2', 'S-0110', 'DIN 472 62 x 2', 'bought', 55),
            ('Key 8x7x25', 'S-0107', 'DIN 6885-1 A 8 x 7 x 25', 'bought', 50),
            ('End washer', 'P13-014', 'End washer', 'made', 50),
            ('Spacer', 'P13-015', 'Spacer', 'made', 50),
            ('Fillet weld', None, 'the jaw weldment\'s fillet welds', 'in P13-016', 55),
            ('Tube', 'P13-013', 'Tube', 'in P13-016', 55),
            ('Stub shaft long', 'P13-010', 'Stub shaft long', 'in P13-016', 55),
            ('Stub shaft short', 'P13-011', 'Stub shaft short', 'in P13-016', 55)],
    'robot': [('Bearing 22207 E triangle', 'S-0112', 'SKF 22207 E/C3', 'bought', 55),
              ('Cap screw M12x30 pin', 'S-0115', 'ISO 4762 M12 x 30 - 8.8 - zinc-plated', 'bought', 55),
              ('Cap screw M8x20 pin', 'S-0036', 'ISO 4762 M8 x 20 - 8.8 - zinc-plated', 'bought', 55),
              ('Circlip DIN 472 120x4', 'S-0114', 'DIN 472 120 x 4', 'bought', 55),
              ('Circlip DIN 472 72x2.5', 'S-0113', 'DIN 472 72 x 2.5', 'bought', 55),
              ('End washer pin C', 'P01-061', 'End washer 76 x 8, hole 13', 'made', 55),
              ('End washer pin D', 'P01-061', 'End washer 76 x 8, hole 13', 'made', 55),
              ('End washer pin', 'P01-060', 'End washer 43 x 6, hole 9', 'made', 55)],
    'vacuum': [('Cap screw M4x12 s', 'S-0119', 'ISO 4762 M4 x 12 - 8.8 - zinc-plated', 'bought', 55),
               ('Cap screw M6x25', 'S-0116', 'ISO 4762 M6 x 25 - 8.8 - zinc-plated', 'bought', 55),
               ('Washer ISO 7089 6', 'S-0117', 'ISO 7089 6 - 200 HV', 'bought', 55),
               ('Nut ISO 4032 M6', 'S-0118', 'ISO 4032 M6 - 8', 'bought', 55)],
}
# the weldments: a made part whose pieces are welded into one before it is machined (Chapter 55)
WELDMENTS = {'P13-016': dict(machine='bag', description='Jaw weldment: the tube and its two stubs, welded, the seats ground',
                             chapter=55)}

# ---- the designations later chapters gave a number that keeps it (Chapter 49's rule: a part that changes everywhere
# it is used keeps its number): number -> (description, chapter)
CHANGED = {
    'S-0092': ('Beckhoff AM8111 with holding brake, and AG2250-PLE040 15:1 with its adapter for the AM8111, two items', 52),
    'S-0094': ('Senring SNM022A-06, rotor and stator', 52),
    'S-0069': ('maxon special unit: EC-i 52 with GP 52 C 26:1, flange, shaft, bearing and brake as Chapter 53 orders', 53),
    'S-0004': ('HIWIN R32-10T4-RSI-172-240-0.023, the end to the book\'s drawing (with its nut, S-0003)', 53),
    'S-0003': ('HIWIN R32-10T4-RSI nut, without preload (with its screw, S-0004)', 53),
    'S-0019': ('SKF GE 20 ES-2LS', 53),
}


def _changed_55_56():
    """Chapters 55 and 56's changes, read from their modules."""
    import palletizer_detail as PDT
    import range_sweep as RS
    out = {}
    for num, was, now in PDT.CHANGED_NUMBERS:
        if now.startswith('also '):                      # more pieces of the same designation, not a new one
            continue
        out[num] = (now, 55)
    for num, was, now in RS.CHANGED_NUMBERS:
        out[num] = (now[0].upper() + now[1:] if not now.startswith('the same') else '%s, %s' % (was[0].upper() + was[1:], now[9:]), 56)
    return out


def _full():
    """Chapter 50's full designation of each bought number."""
    import standard_parts as SP
    nb, snum = DI.numbering()
    return {snum[w['designation']]: (w['full'], w['status']) for w in SP.work_list()}


def designations():
    """Every bought number's description as it stands after Part 10: Chapter 50's, then the later chapters'."""
    out = {n: (full, 50, status) for n, (full, status) in _full().items()}
    for num, (txt, ch) in list(CHANGED.items()) + list(_changed_55_56().items()):
        out[num] = (txt, ch, 'changed' if num in out else 'new')
    for key, rows in DETAIL.items():                    # the numbers Chapters 50 to 56 gave
        for start, num, desc, kind, ch in rows:
            if kind == 'bought' and num not in out:
                out[num] = (desc, ch, 'new')
    return out


# ---- one occurrence's line ---------------------------------------------------------------------------------------
def _base(name):
    return re.sub(r' \(\d+\)$', '', name)


def line_of(key, name, pl=None, nmap=None):
    """What one occurrence named `name` is in machine `key`: (number, description, kind, set) or None for a load.
    kind: 'made', 'bought', 'with <number>', 'in <number>'; set: the name of the bought set a body belongs to (a
    bearing's rings and balls, a reducer's case and output), the base name otherwise."""
    nm = _base(name)
    if nm.startswith(LOADS):
        return None
    for start, num, desc, kind, ch in sorted(DETAIL.get(key, ()), key=lambda r: -len(r[0])):
        if nm.startswith(start):
            return num, desc, kind, start, ''
    if pl is None:
        pl = DI.parts([m for m in DI.MACHINES if m[0] == key][0][2])[0]
        nmap = DI.number_map(key)
    kd = DI.kind_of(nm, pl)
    if kd is None:
        raise KeyError('%s: no number for %r' % (key, name))
    num, desc = nmap[kd]
    c, grp = DI.classify(kd, pl)
    if c == 'part of':
        parent = PARENT.get((key, grp))
        return None, desc, 'with ' + (parent or grp), kd, ''
    if c != 'bought':
        return num, desc, c, kd, ''
    setn = DI.set_name(kd)
    piece = ' '.join(w for w in kd.split() if w not in setn.split())     # 'balls', 'inner ring', 'case', or ''
    return num, desc, c, setn, piece


# the bought part a body of Chapter 49's PART_OF comes with, where the machine has one such part
PARENT = {('flap', 'Motor unit'): 'S-0069', ('clamp', 'Fixed bearing BK25'): 'S-0060',
          ('clamp', 'Supported bearing BF25'): 'S-0097', ('clamp', 'Servo motor'): 'S-0070',
          ('wrist', 'Servo motor'): 'S-0092',                  # the wrist's motor output: the AM8111's shaft end
          ('bag', 'Cylinder'): 'S-0050'}                       # the jaws' piston rods A and B: the DSBC-80-125-C's


def bom(key, occs):
    """The machine's bill of materials from Fusion's census, occs = [(name, mass kg, materials, volume cm3)] of every
    occurrence that has a body. Returns dict(lines, loads, unnumbered) with a line per number: dict(number,
    description, kind, qty, material or maker, the drawn mass of all its pieces, the bodies that come with it)."""
    pl = DI.parts([m for m in DI.MACHINES if m[0] == key][0][2])[0]
    nmap = DI.number_map(key)
    des = designations()
    loads, groups, withs = [], {}, []
    for name, mass, mats, vol in occs:
        ln = line_of(key, name, pl, nmap)
        if ln is None:
            loads.append((name, mass))
            continue
        num, desc, kind, setn, piece = ln
        if kind.startswith('with'):
            withs.append(dict(name=name, mass=mass, parent=kind[5:], description=desc))
            continue
        if num is None and kind.startswith('in '):              # the welds: bodies of the weldment, no number
            g = groups.setdefault(('weld', kind[3:]), dict(number=None, description=desc, kind=kind, names={}, mass=0.0,
                                                            mats=set()))
        else:
            g = groups.setdefault(num, dict(number=num, description=desc, kind=kind, names={}, mass=0.0, mats=set()))
        g['names'].setdefault(setn, []).append((name, piece))
        g['mass'] += mass
        g['mats'].update(m.strip() for m in mats.split(',') if m.strip())
    lines = []
    for k, g in groups.items():
        if g['kind'] == 'bought':
            qty = _sets(g['names'])
            d = des.get(g['number'])
            g['description'] = d[0] if d else g['description']
            g['source'] = 'Chapter %d' % d[1] if d else 'Chapter 49'
            g['status'] = d[2] if d else 'as named'
        else:
            qty = sum(len(v) for v in g['names'].values())
            d = des.get(g['number'])                       # a made part a later chapter changed (the lever, the end plate)
            if d:
                g['description'], g['source'] = d[0], 'Chapter %d' % d[1]
        g['qty'] = qty
        g['mats'] = sorted(g['mats'])
        lines.append(g)
    for w in withs:                                          # the bodies that come with a bought part
        for g in lines:
            if g['number'] == w['parent']:
                g.setdefault('with', []).append(w)
                break
        else:
            lines.append(dict(number=None, description=w['description'], kind='with ' + w['parent'], qty=1,
                              mass=w['mass'], mats=[], names={w['name']: [(w['name'], '')]}))
    for num, w in WELDMENTS.items():                         # a weldment: one line, its pieces under it
        if w['machine'] != key:
            continue
        parts = [g for g in lines if g['kind'] == 'in ' + num]
        n = min(g['qty'] for g in parts if g['number'])       # one tube to a weldment
        for g in parts:
            g['per'] = g['qty'] / n
        lines.append(dict(number=num, description=w['description'], kind='made', qty=n, mass=sum(g['mass'] for g in parts),
                          mats=sorted({m for g in parts for m in g['mats']}), names={}, source='Chapter %d' % w['chapter'],
                          parts=[g['number'] for g in parts]))
    return dict(key=key, lines=lines, loads=loads)


def top(lines):
    """The lines of the first level: a weldment's pieces are inside its line."""
    return [g for g in lines if not g['kind'].startswith('in ')]


def _sets(names):
    """The pieces of a bought kind: per set name, the bodies of one kind of piece (a bearing's inner rings), summed
    over the set names that share the number (the axle's two 32213, C1 and C2)."""
    per = {}
    for setn, nm in names.items():
        for n, piece in nm:
            per.setdefault(setn, {}).setdefault(piece, 0)
            per[setn][piece] += 1
    return sum(max(v.values()) for v in per.values())


# ---- the makers' masses: kg for one piece, with the saved page each is read from ---------------------------------
REFS = os.path.join(os.path.expanduser('~'), 'Documents', 'Mechanical Design References', 'Web pages')
WEIGHTS = 'SKF_product_net_weights_2026-11-04.md'
FIXED = {   # number: (kg, page, what the page gives)
    'S-0006': (0.438, 'SKF_22207E_2026-11-02.md', 'Product net weight 0.438 kg'),
    'S-0112': (0.438, 'SKF_22207E_2026-11-02.md', 'Product net weight 0.438 kg (the C3 bearing the same)'),
    'S-0007': (1.52, 'SKF_22213E_2026-11-02.md', 'Product net weight 1.52 kg'),
    'S-0008': (1.48, WEIGHTS, '32213: 1.48 kg'),
    'S-0009': (1.04, WEIGHTS, '32308: 1.04 kg'),
    'S-0010': (1.36, WEIGHTS, '32309: 1.36 kg'),
    'S-0011': (0.0298, 'SKF_6002-2RSH_2026-11-02.md', 'Product net weight 0.0298 kg'),
    'S-0012': (1.72, 'SKF_61836_2026-11-02.md', 'Product net weight 1.72 kg'),
    'S-0013': (7.34, 'SKF_61856MA_2026-11-02.md', 'Product net weight 7.34 kg'),
    'S-0014': (0.0159, WEIGHTS, '61902-2RS1: 0.0159 kg'),
    'S-0015': (0.0354, WEIGHTS, '6201: 0.0354 kg'),
    'S-0016': (0.2, 'SKF_6206-2RS1_2026-11-03.md', 'Product net weight 0.2 kg'),
    'S-0017': (0.99, WEIGHTS, '6213: 0.99 kg'),
    'S-0018': (0.23, 'SKF_BSD2562CGA_2026-11-02.md', 'Product net weight 0.23 kg'),
    'S-0019': (0.0589, WEIGHTS, 'GE 20 ES-2LS: 0.0589 kg'),
    'S-0109': (0.0589, WEIGHTS, 'GE 20 ES-2LS: 0.0589 kg (the C3 bearing the same)'),
    'S-0086': (0.364, 'SKF_SI20ES_2026-11-02.md', 'Product net weight 0.364 kg'),
    'S-0093': (48.0, 'SKF_slewing_bearings_2026-11-02.md', 'RKS.23 0841: the row of dm 841, mass 48.0 kg'),
    'S-0021': (0.51, 'HIWIN_linear_guideways_HG_2026-11-02.md', 'Table 3.6: HGH25CA 0.51 kg'),
    'S-0048': (0.0401, 'Schmalz_vacuum_cups_ejectors_2026-11-03.md', 'SPB1 80 ED-65: weight 40.10 g'),
    'S-0054': (0.195, 'Schmalz_vacuum_cups_ejectors_2026-11-03.md', 'SCPSi: weight 0.195 kg'),
    'S-0049': (0.465 + 5 * 0.027, 'Ch47_bag_gripper_and_services_sources_2026-11-03.md',
               'DSBC-32: 465 g at 0 mm stroke and 27 g per 10 mm, 50 mm: 600 g'),
    'S-0120': (0.575, 'Festo_DYSC_shock_absorbers_2026-11-03.md', 'DYSC-25: product weight 575 g'),
    'S-0078': (57.2, 'Nabtesco_RV-N_catalogue_CAT180410_2026-11-03.pdf', 'p. 9: RV-500N 57.2 kg'),
    'S-0079': (102.0, 'Nabtesco_RV-N_catalogue_CAT180410_2026-11-03.pdf', 'p. 9: RV-700N 102.0 kg'),
    'S-0080': (9.3, 'Nabtesco_RV-N_catalogue_CAT180410_2026-11-03.pdf', 'p. 9: RV-80N 9.3 kg'),
    'S-0092': (0.74 + 0.45, 'Beckhoff_AM8111_PLE040_2026-11-03.md', 'AM8111 with brake 0.74 kg; PLE040 two-stage 0.45 kg'),
    'S-0069': (0.823 + 0.620 + 0.157, 'maxon_ECi52_GP52C_2026-11-03.md',
               "EC-i 52 0.823 kg; GP 52 C two-stage 0.620 kg; the brake 0.157 kg, Chapter 53's budget for it"),
}
RAIL = 3.21   # kg/m, HGR25R (HIWIN, Table 3.9)
NOT_ON_PAGE = {   # number: why the drawn mass stands
    'S-0050': "Festo's page gives the basic DSBC-80, 3.81 kg at 125 mm, not the clamping unit's weight",
}


def _per1000(fname, key, caption=None):
    """kg for one piece from a Fasten.it table of mass per 1000 pieces in its last column: the row whose first cell
    is `key` (spaces ignored)."""
    import standard_parts as SP
    for cap, rows in SP.md_tables(fname):
        if caption and caption not in cap:
            continue
        for r in rows:
            if r[0].replace(' ', '') == key.replace(' ', '') and r[-1]:
                try:
                    return float(r[-1]) / 1000.0
                except ValueError:
                    continue
    return None


def _iso4762(d, L):
    """ISO 4762 M d x L: Fasten.it's Tables 2 and 3, kg per 1000 pieces by length (rows) and thread size (columns)."""
    import standard_parts as SP
    for cap, rows in SP.md_tables('Fasten.it_DIN_EN_ISO_4762_2026-11-03.md'):
        if 'Approximate mass' not in cap:
            continue
        sizes = [c.replace(' ', '') for c in rows[0]]
        if 'M%g' % d not in sizes:
            continue
        col = sizes.index('M%g' % d)
        for r in rows[1:]:
            if r[0] == '%g' % L and r[col]:
                return float(r[col]) / 1000.0
    return None


def maker_mass(number, description):
    """(kg each, page, what it gives) of a bought part, or None when no saved page gives its mass."""
    if number in FIXED:
        return FIXED[number]
    if number == 'S-0081':
        return (RAIL * 1.720, 'HIWIN_linear_guideways_HG_2026-11-02.md', 'HGR25R 3.21 kg/m, 1720 mm (Chapter 45)')
    if number == 'S-0104':
        return (RAIL * 0.218, 'HIWIN_linear_guideways_HG_2026-11-02.md', 'HGR25R 3.21 kg/m, 218 mm')
    m = re.match(r'ISO 4762 M(\d+) x (\d+)', description)
    if m:
        kg = _iso4762(int(m.group(1)), int(m.group(2)))
        if kg:
            return kg, 'Fasten.it_DIN_EN_ISO_4762_2026-11-03.md', 'M%s x %s: %.4g kg per 1000' % (m.group(1), m.group(2), kg * 1000)
        return None
    m = re.match(r'ISO 4032 M(\d+)', description)
    if m:
        kg = _per1000('Fasten.it_DIN_934_2026-11-03.md', 'M %s' % m.group(1))
        if kg:
            return kg, 'Fasten.it_DIN_934_2026-11-03.md', ('DIN 934 M%s: %.4g kg per 1000 (the nut ISO 4032 replaced, '
                                                           'as wide across the flats)' % (m.group(1), kg * 1000))
        return None
    m = re.match(r'ISO 7089 (\d+)', description)
    if m:
        kg = _per1000('Fasten.it_DIN_EN_ISO_7089_2026-11-03.md', 'M %s' % m.group(1))
        if kg:
            return kg, 'Fasten.it_DIN_EN_ISO_7089_2026-11-03.md', '%s: %.4g kg per 1000' % (m.group(1), kg * 1000)
        return None
    m = re.match(r'DIN (47[12])\s*-?\s*(\d+)', description)
    if m:
        page = 'Fasten.it_DIN_%s_2026-11-03.md' % m.group(1)
        kg = _per1000(page, m.group(2))
        if kg:
            return kg, page, 'd1 %s: %.4g kg per 1000' % (m.group(2), kg * 1000)
        return None
    m = re.match(r'DIN 908 (M\d+ x \d+)', description)
    if m:
        kg = _per1000('Fasten.it_DIN_908_2026-11-03.md', m.group(1).replace(' ', ''))
        if kg:
            return kg, 'Fasten.it_DIN_908_2026-11-03.md', '%s: %.4g kg per 1000' % (m.group(1), kg * 1000)
    return None


# ---- what the census cannot see: the tool changer's electrical coupling, specified in Volume 2's Chapter 47 and never
# drawn, added to the changer's bill by hand. HARTING's selection guide (Web pages/HARTING_Han-Modular_selection_guide_
# and_docking_frame_2026-11-04.md): the two leading PE contacts are the hinged frames' (p. 2), the docking frame has none
# and carries its earth in a PE module (pp. 3, 6-7); docking frames 6 B for 2 modules, 10 B for 3 (p. 3); standard
# modules > 500 matings, HMC > 10 000 (p. 2); the frame 09 14 006 1701 >= 10 000 only with HMC components (its data
# sheet). Each pair one line under one number, as S-0092 is (Chapter 52). No mass until the part numbers are chosen.
COUPLING = [  # (number, description, qty, status)
    ('S-0124', 'HARTING Han-Modular docking frame 10 B for 3 modules, a pair: float mount and fix mount', 1, 'specified'),
    ('S-0125', 'HARTING Han E module, HMC version, 6 contacts 16 A, male and female', 1, 'specified'),
    ('S-0126', 'HARTING Han DD module, HMC version, 12 contacts 10 A, male and female', 1, 'specified'),
    ('S-0127', "HARTING Han-Modular PE module, leading, male and female, sized to the motor's earth conductor", 1, 'open'),
]


# ---- one designation, one number: where the full designations of Chapter 50 and later made two numbers name the same
# orderable item, the lower number stands and the higher is superseded by it (Chapter 49's rule, applied to the
# designations as they now stand)
def superseded(des=None):
    """{higher number: the lower number that stands}, for the bought numbers whose descriptions are the same."""
    des = des or designations()
    first, out = {}, {}
    for num in sorted(des):
        key = re.sub(r'\s+', ' ', des[num][0].strip().lower())
        if key in first:
            out[num] = first[key]
        else:
            first[key] = num
    return out


def finish(b):
    """A machine's lines as they are ordered: the superseded numbers merged into the ones that stand, each bought
    line's mass from its maker where a page gives it, the item numbers, and the totals."""
    des = designations()
    sup = superseded(des)
    merged = {}
    for g in b['lines']:
        num = g['number']
        if g['kind'] == 'bought' and num in sup:
            g = dict(g, was=[num], number=sup[num])
            num = g['number']
        if num is not None and num in merged and g['kind'] == 'bought':
            m = merged[num]
            m['qty'] += g['qty']
            m['mass'] += g['mass']
            m['with'] = m.get('with', []) + g.get('with', [])
            m['was'] = sorted(set(m.get('was', []) + g.get('was', [])))
            for k, v in g['names'].items():
                m['names'].setdefault(k, []).extend(v)
            continue
        merged[num if num is not None else (g['kind'], g['description'])] = g
    lines = list(merged.values())
    for g in lines:
        g['drawn'] = g['mass'] + sum(w['mass'] for w in g.get('with', []))
        if g['kind'] == 'bought':
            mk = maker_mass(g['number'], g['description'])
            g['maker'] = mk
            g['note'] = NOT_ON_PAGE.get(g['number'])
            g['book'] = mk[0] * g['qty'] if mk else g['drawn']
        else:
            g['book'] = g['drawn']
    order = lambda g: (0 if g['kind'] == 'made' else 1 if g['kind'].startswith('in ') else 2 if g['kind'] == 'bought' else 3,
                       g['number'] or 'zzz')
    lines.sort(key=order)
    item = 0
    for g in lines:
        if g['kind'] in ('made', 'bought'):
            item += 1
            g['item'] = item
    tl = top(lines)
    made = [g for g in tl if g['kind'] == 'made']
    bought = [g for g in tl if g['kind'] == 'bought']
    b.update(lines=lines, made_kinds=len(made), made_pieces=sum(g['qty'] for g in made), bought_kinds=len(bought),
             bought_pieces=sum(g['qty'] for g in bought), with_maker=sum(1 for g in bought if g['maker']),
             drawn=sum(g['drawn'] for g in tl), book=sum(g['book'] for g in tl),
             bought_drawn=sum(g['drawn'] for g in bought), bought_book=sum(g['book'] for g in bought))
    return b
