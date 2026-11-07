"""Chapter 63: the drawings. One part of each machine, the one whose drawing Part 10 changed most, and every value its
sheet carries, worked out here so that the script, the check and the book read the same numbers:

- the opening stop bracket P13-018 (the bag gripper): Chapter 62's arm turned 45 deg, Chapter 60's root radius, its two
  bolt holes and their position;
- the tool changer's segment P10-006: Chapter 61's two ground faces as a profile tolerance, the six ground as a set;
- the rear axle's pinion spacer A01-016: Chapter 61's length ground to each axle, its faces' parallelism;
- the flap's spar fitting F01-022: Chapter 51's N7 bores for the GE 20 ES, with the envelope requirement and Rz 16;
- the wrist's stop disc W01-018: nothing beyond ISO 2768-m (Chapter 61), the drawing that needs no tolerance of its own.

Sources: Budynas & Nisbett (2015), Shigley 10th, Chapter 20 (GD&T: the profile zone centred on the true profile, p. 988;
the bonus tolerance at MMC, p. 994) and Table A-11 (IT6, IT7, p. 1030); Childs (2014), Chapter 19 (the implied square
zone of a +-0.1 hole and its 0.28 circle, pp. 772-773); SKF's seat tolerances (Table 1 of 'Tolerances for bearing
seats and abutments': run-outs by IT grade) and its plain-bearing seats (Rz, the envelope); ISO 286-1's IT5 from the 'IT Grade' page; AmesWeb's ISO 2768-1 table.
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)

import range_sweep as RS        # noqa: E402
import lugs_brackets as LB      # noqa: E402

FOLDER = ('MechDesignFusionClaude Vol3', 'Chapter 63 drawings')

# ISO 286-1, Table 1, in um: IT6 and IT7 as Shigley's Table A-11 (p. 1030), IT5 from the 'IT Grade' page (n.d.), which
# agrees with Shigley on IT6 and IT7
IT = {5: {(30, 50): 11, (50, 80): 13}, 6: {(6, 10): 9, (18, 30): 13, (30, 50): 16, (50, 80): 19},
      7: {(30, 50): 25, (50, 80): 30}, 13: {(6, 10): 220}}


def it(grade, d):
    for (a, b), v in IT[grade].items():
        if a < d <= b:
            return v / 1000.0
    raise KeyError('IT%d at %g mm is not in the table here' % (grade, d))


# ISO 2768-1, class m, from AmesWeb (n.d.), Table 1
ISO2768_M = ((0.5, 3, 0.1), (3, 6, 0.1), (6, 30, 0.2), (30, 120, 0.3), (120, 400, 0.5), (400, 1000, 0.8))


def general(d):
    for a, b, t in ISO2768_M:
        if a <= d <= b:
            return t
    raise KeyError(d)


# ---- the opening stop bracket ----------------------------------------------------------------------------------------
def bracket():
    """The opening bracket's holes: two M8 through it and the end plate, both clearance holes, so the bolt floats in
    both. A bolt of diameter F passes a hole of diameter H while its axis lies within (H - F)/2 of the hole's; two
    holes whose axes stray in opposite directions let one bolt through while their axes lie H - F apart. So each
    hole's axis may stray (H - F)/2 from its true place: a zone H - F across, at the holes' and the bolt's maximum
    material, the smallest hole and the largest bolt. A hole drawn H13 gives more when it comes out larger (Shigley's
    bonus tolerance, p. 994)."""
    o = RS.stop_layout()['open']
    B = RS.BRACKET
    cy, cz = o['centre']
    face = o['foot']['x'][1]                        # the foot's face that the arm leaves, 437 mm out
    arm = o['arm']
    corners = LB.arm(True)['corners']                # the arm as Chapter 62 turned it
    H = 9.0                                         # the hole drawn, as the end plate's (Chapter 56)
    F = B['bolt']                                   # the M8's largest shank
    t = H - F
    pitch = math.hypot(o['bolts'][0][0] - o['bolts'][1][0], o['bolts'][0][1] - o['bolts'][1][1])
    # the same holes by +-0.1 on two coordinates, as Childs's Figure 19.6: a square zone 0.2 wide, 0.28 on its diagonal
    square = 2 * 0.1
    return dict(H=H, H_max=H + it(13, H), F=F, t=t, bonus=it(13, H), t_lmc=t + it(13, H), pitch=pitch,
                square=square, diagonal=square * math.sqrt(2.0), area_gain=(math.pi / 4 * (square * math.sqrt(2)) ** 2) / square ** 2 - 1,
                r_root=2.5, corners=corners, turn=LB.TURN, foot=B['foot'], arm_side=arm['y'][1] - arm['y'][0], arm_len=arm['x'][1] - arm['x'][0],
                thread='M%d x %g' % (RS.DYSC[25]['D1'], RS.DYSC[25]['P']), lug_d=2 * o['lug']['r'],
                lug_len=o['lug']['t'][1] - o['lug']['t'][0], axis_out=o['x'] - o['foot']['x'][0])


# ---- the tool changer's segment --------------------------------------------------------------------------------------
def segment():
    """Chapter 61: the segment's two wedge faces ground to +-0.01, the six ground as one set. A profile tolerance is
    centred on the true profile (Shigley, p. 988), so +-0.01 is a zone 0.02 wide."""
    pm = 0.01
    return dict(pm=pm, profile=2 * pm, set=6, factor_15=1 / math.tan(math.radians(15)), factor_5=1 / math.tan(math.radians(5)),
                factor=1 / (math.tan(math.radians(5)) * math.tan(math.radians(15))))


# ---- the pinion's spacer ---------------------------------------------------------------------------------------------
def spacer():
    """Chapter 61: no fixed length; ground to each axle's bearings and nose, 67.8 mm nominal. Its two faces are the
    cones' abutments, and SKF asks of a shaft abutment for bearings of Normal tolerances an axial run-out of IT5
    (Table 1). The spacer's 56 mm faces: IT5 over 50 to 80 mm."""
    import axle_design as AD
    d_out = 56.0
    d_in = AD.SKF_TRB['32309'][0] + 0.1             # the bore 0.05 clear of the 45 mm seat each side, as Chapter 54 drew it
    import stackups as ST
    ps = ST.pinion_spacer()
    lo, hi = ps['limits']
    stock = math.ceil((hi + 0.1) * 10) / 10          # turned long, at least 0.1 over the longest any axle needs
    return dict(L=ps['L'], d_out=d_out, d_in=d_in, parallel=it(5, d_out), range=(lo, hi), stock=stock,
                grind=(stock - hi, stock - lo))


# ---- the flap's spar fitting -----------------------------------------------------------------------------------------
def fitting():
    """Chapter 51: the GE 20 ES's 35 mm outer rings in N7 bores in the 7075 fitting, SKF's class for a light-alloy
    housing, with the envelope requirement (ISO 14405-1) SKF writes under its tables, and a housing seat of
    Rz <= 16 um (ISO 4288); SKF's plain-bearing page asks no run-out of the seat. N7 at 35 mm is -0.008 to -0.033
    (Chapter 53). Chapter 61 keeps N7 and bonds the ring with LOCTITE 638, an assembly instruction."""
    D_ = 35.0
    return dict(D=D_, es=-0.008, ei=-0.033, Rz=16.0, envelope=True, bond='LOCTITE 638')


# ---- the wrist's stop disc -------------------------------------------------------------------------------------------
def disc():
    """Chapter 61: the disc's gaps keep 0.32 mm in the worst case with every dimension at ISO 2768-m."""
    return dict(cls='m', worst=0.32)


PARTS = [
    dict(key='bracket', machine='bag', part='Stop bracket open 1', kind='Stop bracket', number='P13-018',
         title='Stop bracket, opening', changes=[dict(kind='turn_arm', side=1), dict(kind='root_fillet', r=2.5, corners=None)]),
    dict(key='segment', machine='changer', part='Segment 1 (1)', kind='Segment', number='P10-006', title='Segment',
         apply51=True),
    dict(key='spacer', machine='axle', part='Spacer', kind='Spacer', number='A01-016', title='Spacer, pinion'),
    dict(key='fitting', machine='flap', part='Spar fitting', kind='Spar fitting', number='F01-022', title='Spar fitting'),
    dict(key='disc', machine='wrist', part='Stop disc', kind='Stop disc', number='W01-018', title='Stop disc'),
    dict(key='stop', machine='bag', part='Stop bracket open 1', kind='Stop bracket', number='P13-000',
         title='Bag gripper, the opening stop', assembly=True),     # a sheet of the bag gripper's own drawing
]
PARTS[0]['changes'][1]['corners'] = bracket()['corners']


# the bar each steel part is cut from, mm across: the bracket's 70 mm square foot needs a bar of its diagonal, 99 (as
# Chapter 56 took it, over 76); the spacer is turned from bar over its 56; the disc's lug reaches 35 from its axis
BAR = {'bracket': 99.0, 'segment': 0.0, 'spacer': 60.0, 'fitting': 0.0, 'disc': 70.0}


def fit_rows(fits):
    """The table's rows for the classes Chapter 51 left on the faces: one row for each class on each size, with how
    many faces carry it; a capital class is a hole's, a small one a shaft's (ISO 286)."""
    groups = {}
    for f in fits:
        if f['kind'] == 'cylinder':
            lab = '%s %s' % (('%g' % f['d']), 'bore' if f['value'][:1].isupper() else 'seat')
        else:
            lab = 'keyseat' if f['value'][:2] in ('JS', 'N9', 'P9') else 'face'
        groups.setdefault((lab, f['value']), 0)
        groups[(lab, f['value'])] += 1
    out = []
    for (lab, val), n in sorted(groups.items()):
        cls, _, rough = val.partition(' ')
        what = lab if not lab[0].isdigit() else 'Ø' + lab
        out.append((what if n == 1 or not lab[0].isdigit() else '%d × %s' % (n, what), '%s, %s' % (cls, rough), 'Ch 51'))
    return out


def rows(key, fits, material):
    """Every row of a sheet's requirement table: the material, the decisions of Part 10 for this part, the classes of
    Chapter 51 read off the faces, the general tolerance."""
    alloy, cond, _src = material
    if cond and cond.startswith('cold drawn, or hot rolled over 76'):     # Chapter 51's rule, decided by the bar
        cond = 'hot rolled' if BAR[key] > 76 else 'cold drawn'
    out = [('Material', alloy + (', ' + cond if cond else ''), 'Ch 51')]
    if key == 'bracket':
        b = bracket()
        out += [('Arm', "%g square, at %g° to the foot, its sides square to the lug's axis" % (b['arm_side'], b['turn']), 'Ch 62'),
                ('Root of the arm', 'R%g min, all round' % b['r_root'], 'Ch 60'),
                ('Holes', "2 × Ø%g H13 through, %g apart on the foot's diagonal" % (b['H'], b['pitch']), 'Ch 56'),
                ('Their position', 'Ø%.1f (M) to A, the face on the end plate' % b['t'], 'Ch 63'),
                ('Lug', 'Ø%g × %g, %s-6H through' % (b['lug_d'], b['lug_len'], b['thread'].replace(' x ', ' × ')), 'Ch 56')]
    elif key == 'segment':
        g = segment()
        out += [('15° face', 'profile of a surface %.2f to A' % g['profile'], 'Ch 61'),
                ('5° face', 'profile of a surface %.2f to B' % g['profile'], 'Ch 61'),
                ('Wedge faces', 'ground after hardening', 'Ch 61'),
                ('Set', 'the %d ground as one set, kept as one item' % g['set'], 'Ch 61')]
    elif key == 'spacer':
        sp = spacer()
        out += [('Length', '%.1f nominal; turned to %.1f' % (sp['L'], sp['stock']), 'Ch 61'),
                ('Grinding', 'to each axle, %.2f to %.2f' % sp['range'], 'Ch 66'),
                ('Faces', 'both ground in one set-up', 'Ch 61'),
                ('Datum A', 'the face on cone A', 'Ch 63'),
                ('Face on cone B', 'parallel to A within %.3f (IT5)' % sp['parallel'], 'Ch 63')]
    elif key == 'fitting':
        f = fitting()
        out += [('Bores', 'envelope (E), ISO 14405-1', 'Ch 51')]
    out += fit_rows(fits)
    out.append(('General', 'ISO 2768-mK', 'Ch 61'))
    return out


# ---- the opening stop put together: the assembly sheet --------------------------------------------------------------
STOP_ITEMS = [  # (number, title, designation, the part's name or the start of its pieces' names, pieces)
    ('P13-018', 'Stop bracket, opening', 'stop bracket, opening: foot, arm and lug, steel 1045', 'Stop bracket open 1', 1),
    ('S-0120', 'Shock absorber DYSC-25-25-Y1F', 'Festo DYSC-25-25-Y1F, part no. 2480234', 'Shock absorber DYSC-25-25-Y1F open 1', 1),
    ('', 'Lock nut', 'with the DYSC-25-25-Y1F, S-0120 (Chapter 49: no number of its own)', 'Lock nut DYSC-25-25-Y1F open 1', 1),
    ('S-0121', 'Cap screw M8 x 40', 'ISO 4762 M8 x 40 - 8.8 - zinc-plated', 'Cap screw ISO 4762 M8x40 stop', 2),
    ('S-0122', 'Washer 8', 'ISO 7089 8 - 200 HV - zinc-plated', 'Washer ISO 7089 8 stop', 2),
    ('S-0123', 'Nut M8', 'ISO 4032 M8 - 8 - zinc-plated', 'Nut ISO 4032 M8 stop', 2),
]


def assembly_items(spec, leaves):
    """The pieces of each item among the machine's parts, (name, centre in mm): a whole name as it is; a start of names,
    the pieces whose centres lie in the opening bracket's foot square of jaw A, off the end plate's face."""
    o = RS.stop_layout()['open']
    cy, cz = o['centre']
    out = []
    for number, title, des, name, n in STOP_ITEMS:
        hits = [nm for nm, c in leaves if nm == name] or \
               [nm for nm, c in leaves if nm.startswith(name) and c[0] > 380 and abs(c[1] - cy) < 35 and abs(c[2] - cz) < 35]
        if len(hits) != n:
            raise RuntimeError('%s: %d pieces found, %d expected' % (title, len(hits), n))
        out.append(dict(number=number, title=title, designation=des, names=sorted(hits)))
    return out


def stop_rows():
    """The fitter's table on the assembly sheet: the bolts' torque that gives Chapter 60's preload, the absorber held
    by its own lock nut, the bracket's seat."""
    import fatigue_detail as F
    t = F.tightening()
    return [('Seat', 'the foot on the end plate P13-005, its holes over the plate\'s', 'Ch 56'),
            ('S-0121', 'tighten to %.1f N m (K %.1f, zinc-plated)' % (t['T'], t['K']), 'Ch 60'),
            ('Preload', '%.0f N, 75 %% of the proof load' % t['Fi'], 'Ch 60'),
            ('S-0120', 'in the lug\'s thread, held by its lock nut', 'Ch 56')]


def summary():
    return dict(bracket=bracket(), segment=segment(), spacer=spacer(), fitting=fitting(), disc=disc())


if __name__ == '__main__':
    import json
    print(json.dumps(summary(), indent=1, default=str))
