# -*- coding: utf-8 -*-
"""detail_inventory.py: Chapter 49 of Volume 3, from concept to detailed design. What the four machines of Volume 2 that
Volume 3 details are made of, read from the parts lists their Fusion checks printed (companion/results): every kind
of part sorted into the parts a workshop makes, which each need a drawing, and the parts bought from a catalogue,
which each need a maker's number. Bodies that Fusion holds apart but a buyer orders as one (a bearing's rings and
balls, a reducer's case and output, a slip ring's rotor and stator, a cylinder's piston rod) are counted as the one
part bought, and a bought part used in several places is one kind with several pieces.

    python detail_inventory.py      -> the counts, machine by machine
"""
import io
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
RESULTS = os.path.join(os.path.dirname(HERE), 'results')

# the machines Volume 3 details (vol3/PLAN.md) and the checks of their final assemblies in Volume 2
MACHINES = [
    ('wrist', 'The robotic wrist', 'wrist_integration_check.txt', 40),
    ('flap', 'The flap actuation system', 'flap_system_check.txt', 34),
    ('axle', 'The rear axle', 'rear_axle_check.txt', 31),
    ('robot', 'The palletizer\'s robot', 'palletizer_robot_check.txt', 43),
    ('changer', 'The tool changer', 'tool_changer_check.txt', 44),
    ('clamp', 'The clamp gripper', 'clamp_gripper_check.txt', 45),
    ('vacuum', 'The vacuum gripper', 'vacuum_gripper_check.txt', 46),
    ('bag', 'The bag gripper', 'bag_gripper_check.txt', 47),
]

# the bodies of one bought part that the model holds apart: dropped from the name, counted once per set
PIECES = (' balls', ' inner ring', ' outer ring', ' cone', ' cup', ' rotor', ' stator')
# bodies modelled apart that come with another bought part
PART_OF = {'Piston rod': 'Cylinder', 'Angular contact bearing 7205': 'Fixed bearing BK25',
           'Deep groove bearing': 'Supported bearing BF25', 'Motor output': 'Servo motor',
           'Gearhead shaft': 'Motor unit'}
# made, although the name starts like a bought part's
MADE = ('Bearing cap', 'Bearing cap floating', 'Motor bracket', 'Slip ring bracket')
# what is bought, by a pattern in its name; the group it belongs to, the first that matches (a ball screw and its nut
# are transmission parts, tried before the fasteners' "screw" and "nut")
BOUGHT = [
    (r'^[Bb]earing |^Slewing ring|^Slip ring|^Fixed bearing BK|^Supported bearing BF|^Rod end SI', 'bearings'),
    (r'^Belt |^Ball nut|^Ball ?screw|^Rail |^Block |^Gearhead|^RV-|^Coupling$', 'transmission parts'),
    (r'screw|[Bb]olt M\d|\bnut\b|[Nn]ut M\d|stud M\d|plug M\d|^Washer( M\d|$)', 'fasteners'),
    (r'^Dowel|^Pin \d|^Key DIN|^Key \d|^Nut key', 'pins and keys'),
    (r'^Servo motor|^Motor|^Encoder$|^Sensor|^Cylinder DSBC|^Cup |^Ejector', 'motors, cylinders and vacuum'),
    (r'[Ss]eal\b|[Rr]etaining ring|^O-ring|^Breather|^Spring$', 'seals, rings and springs'),
]


def parts(fname):
    """The parts list a check printed: {name: count}, Fusion's (1), (2) copies merged."""
    s = io.open(os.path.join(RESULTS, fname), encoding='utf-8').read()
    m = re.search(r'^Parts: (\d+) components[^;]*; (.*?)\.\s*$', s, re.M)
    total = int(m.group(1))
    out = {}
    for it in re.split(r', (?=\d+ )', m.group(2)):                  # a name may hold a comma ("Roll shaft, bored")
        n, name = it.split(' ', 1)
        name = re.sub(r' \(\d+\)$', '', name)
        out[name] = out.get(name, 0) + int(n)
    assert sum(out.values()) == total, fname
    return out, total


def set_name(name):
    """The name of the bought part a body belongs to: a bearing's ring or balls, a reducer's case or output."""
    for p in PIECES:
        if name.endswith(p):
            return name[:-len(p)]
    return re.sub(r'^(RV-\d+N) (case|output)', r'\1', name)


def designation(name):
    """What a buyer orders: a bearing by its designation, a reducer by its size, anything else by its name."""
    m = re.match(r'^RV-\d+N', name)
    if m:
        return m.group(0)
    if re.match(r'^[Bb]earing ', name):
        for pat in (r'GE \d+ ES', r'BSD \d+ CGA', r'\d{4,5}(?:-2RS\w*)?(?: E| MA)?(?=\b|$)'):
            m = re.search(pat, name)
            if m:
                return 'Bearing ' + m.group(0)
    return name.split(', ')[0]                        # the same part in another place ("..., flange")


def classify(name, names=()):
    """('made', None), ('bought', group) or ('part of', the bought part) for a kind of part; a body counts as part of
    a bought part only when the machine has that part (a flap actuator's own piston rod is made)."""
    if name in PART_OF and any(n.startswith(PART_OF[name]) for n in names):
        return 'part of', PART_OF[name]
    if name in MADE:
        return 'made', None
    base = set_name(name)
    for pat, grp in BOUGHT:
        if re.search(pat, base):
            return 'bought', grp
    return 'made', None


def inventory(key):
    """One machine: its components in Fusion and its kinds; the parts to make and to buy, as kinds and as pieces."""
    k, title, fname, ch = [m for m in MACHINES if m[0] == key][0]
    pl, total = parts(fname)
    made, sets, groups, part_of = {}, {}, {}, {}
    for name, n in pl.items():
        c, grp = classify(name, pl)
        if c == 'made':
            made[name] = n
        elif c == 'part of':
            part_of[name] = n
        else:
            sn = set_name(name)
            sets[sn] = max(sets.get(sn, 0), n)            # a set of bodies is one part per set
            groups[sn] = grp
    bought, bgroup = {}, {}
    for sn, n in sets.items():
        dsg = designation(sn)
        bought[dsg] = bought.get(dsg, 0) + n
        bgroup[dsg] = groups[sn]
    by_group = {}
    for dsg, grp in bgroup.items():
        by_group.setdefault(grp, {})[dsg] = bought[dsg]
    return dict(key=k, title=title, chapter=ch, components=total, kinds=len(pl), made=made, bought=bought,
                groups=by_group, part_of=part_of, made_kinds=len(made), made_pieces=sum(made.values()),
                bought_kinds=len(bought), bought_pieces=sum(bought.values()))


def all_machines():
    return [inventory(m[0]) for m in MACHINES]


def totals(rows=None):
    rows = rows or all_machines()
    keys = ('components', 'made_kinds', 'made_pieces', 'bought_kinds', 'bought_pieces')
    return {k: sum(r[k] for r in rows) for k in keys}


# the part numbers: a made part is <machine><subassembly>-<item>, the subassembly's own number ending in 000; a bought
# part is S-<serial>, one number for one designation wherever it is used, its maker's designation in the description
SUB = {'wrist': 'W01', 'flap': 'F01', 'axle': 'A01', 'robot': 'P01', 'changer': 'P10', 'clamp': 'P11',
       'vacuum': 'P12', 'bag': 'P13'}


def numbering(rows=None):
    """Every kind of part numbered: the made parts of each assembly in the order of their names, the bought parts of
    all the assemblies once, in the order of their designations. Returns {key: [(number, name, quantity, 'made' or
    'bought')]} and the bought parts' numbers."""
    rows = rows or all_machines()
    bought = sorted({d for r in rows for d in r['bought']})
    snum = {d: 'S-%04d' % (k + 1) for k, d in enumerate(bought)}
    out = {}
    for r in rows:
        sub = SUB[r['key']]
        lst = [(sub + '-000', r['title'][4:].capitalize(), 1, 'assembly')]
        for k, name in enumerate(sorted(r['made'])):
            lst.append(('%s-%03d' % (sub, k + 1), name, r['made'][name], 'made'))
        for d in sorted(r['bought']):
            lst.append((snum[d], d, r['bought'][d], 'bought'))
        out[r['key']] = lst
    return out, snum


def number_map(key, rows=None):
    """For one assembly, each name on its parts list with its part number and description: a made part its own number
    and its name, a bought part the number and the designation of what is ordered, a body that comes with a bought
    part no number and 'with the <part>'."""
    rows = rows or all_machines()
    nb, snum = numbering(rows)
    k, title, fname, ch = [m for m in MACHINES if m[0] == key][0]
    pl, total = parts(fname)
    made = [n for n, num, q, kind in [(x[1], x[0], x[2], x[3]) for x in nb[key]] if kind == 'made']
    out = {}
    for name in pl:
        c, grp = classify(name, pl)
        if c == 'made':
            out[name] = ('%s-%03d' % (SUB[key], made.index(name) + 1), name)
        elif c == 'bought':
            d = designation(set_name(name))
            out[name] = (snum[d], d)
        else:
            out[name] = (None, 'with the ' + grp)
    return out


SCRIPTS = {'wrist': 'WristIntegration', 'flap': 'FlapSystem', 'axle': 'RearAxle', 'robot': 'PalletizerRobot',
           'changer': 'ToolChanger', 'clamp': 'ClampGripper', 'vacuum': 'VacuumGripper', 'bag': 'BagGripper'}


def kind_of(name, known):
    """The name on the parts list that a Fusion component's name stands for: Fusion's (1) dropped, then a jaw's A or
    B, then the part's own number, until a known name is reached; None for a body that is not a part (the bag, the
    station's rollers)."""
    nm = re.sub(r' \(\d+\)$', '', name)
    wds = nm.split()
    while wds:
        if ' '.join(wds) in known:
            return ' '.join(wds)
        if len(wds) > 1 and (wds[-1] in ('A', 'B') or wds[-1].isdigit()):
            wds = wds[:-1]
        else:
            return None
    return None


def shared(rows=None):
    """The bought parts used in more than one assembly: {designation: [keys]}."""
    rows = rows or all_machines()
    use = {}
    for r in rows:
        for d in r['bought']:
            use.setdefault(d, []).append(r['key'])
    return {d: ks for d, ks in use.items() if len(ks) > 1}


if __name__ == '__main__':
    rows = all_machines()
    for r in rows:
        print('%-28s ch %d: %3d components, %2d kinds; make %2d kinds (%3d pieces); buy %2d kinds (%3d pieces)'
              % (r['title'], r['chapter'], r['components'], r['kinds'], r['made_kinds'], r['made_pieces'],
                 r['bought_kinds'], r['bought_pieces']))
    print(totals(rows))
