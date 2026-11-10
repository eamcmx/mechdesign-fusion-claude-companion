# WristDifferential (Chapter 39) against the acceptance criteria of its prompt, measured on what Fusion built: the
# gear set's front face, the belt's centre distance and the payload's frequency worked again here; the groups and
# the parts counted; each joint's links (pitch alone, roll alone: every gear and motor at its equation's angle); no
# interference at rest and at eight poses; the moving groups' masses and inertias measured for the chapter; three
# refusals.
import math, sys, json, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/projects/robotic_wrist/WristDifferential/WristDifferential.py'
EXPECTED = {'Base plate': 1, 'Bearing 6002-2RSH': 4, 'Bearing 61902-2RS1': 4, 'Bearing cap fixed': 1, 'Bearing cap floating': 1, 'Belt HTD 350-5M-15': 2, 'Cap screw ISO 4762 M4x10': 8, 'Cap screw ISO 4762 M4x12': 8, 'Cap screw ISO 4762 M5x12': 6, 'Cap screw ISO 4762 M5x16': 6, 'Cap screw ISO 4762 M8x20': 4, 'Carrier block': 1, 'Countersunk screw ISO 10642 M3x8': 8, 'Dowel ISO 8734 5x16': 4, 'End washer 18x5.5x2.5': 2, 'End washer 18x6.4x2': 1, 'Key DIN 6885 A 5x5x18': 1, 'Low-head screw DIN 7984 M6x12': 1, 'Motor bracket': 2, 'Motor output': 2, 'Output bevel gear': 1, 'Retaining ring DIN 471-15': 1, 'Retaining ring DIN 472-32': 1, 'Roll shaft': 1, 'Servo motor with brake and 15:1 gearhead': 2, 'Side bevel gear with 36-groove pulley': 2, 'Side cheek': 2, 'Spacer S1': 2, 'Spacer S2': 2, 'Spacer S3': 2, 'Spacer S4': 1, 'Tie bar': 2, 'Tool flange ISO 9409-1-31.5-4-M5': 1, 'Trunnion': 2}
POSES = [(30.0, 0.0), (0.0, 45.0), (-40.0, 70.0), (75.0, -123.0), (-110.0, 200.0), (25.0, 30.0), (-60.0, 150.0), (120.0, -90.0)]


def own_numbers():
    """The front face of a miter gear of module 1.5, 40 teeth, face 11 (Shigley's proportions); the 350-5M belt's
    centre distance on 18 and 36 grooves; the payload's frequency on a 15 mm overhanging shaft."""
    m, z, b = 1.5, 40, 11.0
    R = m * z / math.sqrt(2)
    hf = m + 0.188 * m + 0.05
    zff = (R - b) / R * (R / math.sqrt(2) + hf / math.sqrt(2))
    D, d = 36 * 5 / math.pi, 18 * 5 / math.pi
    lo, hi = 60.0, 200.0
    for _ in range(200):
        C = (lo + hi) / 2
        ph = math.asin((D - d) / (2 * C))
        Lp = 2 * C * math.cos(ph) + math.pi * (D + d) / 2 + ph * (D - d)
        lo, hi = (C, hi) if Lp < 350 else (lo, C)
    return dict(zff=zff, C=C)


def group_props(o):
    pp = o.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    ok, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()
    c = pp.centerOfMass
    return dict(m=pp.mass, x=c.x * 10, y=c.y * 10, z=c.z * 10, Ixx=xx * 1e-4, Iyy=yy * 1e-4, Izz=zz * 1e-4)


def abs_turn(o, R0, axis):
    return fc.turn_deg(fc.rotation(o), R0, axis)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    own = own_numbers()
    o = ns['design']()
    fc.ok(abs(o['g']['zff'] - own['zff']) < 1e-6 and abs(o['belt']['C'] - own['C']) < 1e-6,
          'the front face %.3f mm and the belt\'s centre distance %.3f mm worked again (the module: %.3f, %.3f)'
          % (own['zff'], own['C'], o['g']['zff'], o['belt']['C']))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = sorted(x.component.name for x in root.occurrences)
    fc.ok(len(subs) == 9, 'nine groups: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds: %s' % (sum(pl.values()), len(pl), sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    last = ns['_last']
    occ = last['occ']
    pose = ns['pose']
    Y, Z = (0, 1, 0), (0, 0, 1)
    pose(0.0, 0.0)
    R0 = {k: fc.rotation(v) for k, v in occ.items()}
    # pitch alone: the carrier, both gears and the roll output turn phi; each motor 2 phi
    pose(20.0, 0.0)
    t = {k: abs_turn(occ[k], R0[k], Y) for k in ('carrier', 'inL', 'inR', 'mL', 'mR')}
    fc.ok(abs(t['carrier'] - 20) < 0.01 and abs(t['inL'] - 20) < 0.01 and abs(t['inR'] - 20) < 0.01 and abs(t['mL'] - 40) < 0.02
          and abs(t['mR'] - 40) < 0.02, 'pitch 20 deg alone: carrier %.3f, gear L %.3f, gear R %.3f, motor L %.3f, motor R %.3f'
          % (t['carrier'], t['inL'], t['inR'], t['mL'], t['mR']))
    # roll alone: the output turns psi about z; gear L -psi, gear R +psi about y; motor L -2 psi, motor R +2 psi
    pose(0.0, 30.0)
    r = abs_turn(occ['roll'], R0['roll'], Z)
    t = {k: abs_turn(occ[k], R0[k], Y) for k in ('carrier', 'inL', 'inR', 'mL', 'mR')}
    fc.ok(abs(r - 30) < 0.01 and abs(t['carrier']) < 0.01 and abs(t['inL'] + 30) < 0.01 and abs(t['inR'] - 30) < 0.01
          and abs(t['mL'] + 60) < 0.02 and abs(t['mR'] - 60) < 0.02,
          'roll 30 deg alone: output %.3f, carrier %.3f, gear L %.3f, gear R %.3f, motor L %.3f, motor R %.3f'
          % (r, t['carrier'], t['inL'], t['inR'], t['mL'], t['mR']))
    # both: thL = phi - psi, thR = phi + psi, motors twice
    pose(25.0, 30.0)
    t = {k: abs_turn(occ[k], R0[k], Y) for k in ('inL', 'inR', 'mL', 'mR')}
    fc.ok(abs(t['inL'] + 5) < 0.01 and abs(t['inR'] - 55) < 0.01 and abs(t['mL'] + 10) < 0.02 and abs(t['mR'] - 110) < 0.02,
          'pitch 25, roll 30: gear L %.3f (-5), gear R %.3f (55), motor L %.3f (-10), motor R %.3f (110)'
          % (t['inL'], t['inR'], t['mL'], t['mR']))
    pose(0.0, 0.0)
    h = fc.interference(leaves)
    fc.ok(not h, 'no interference among the %d parts at rest %s' % (len(leaves), h or ''))
    for p_, r_ in POSES:
        pose(p_, r_)
        h = fc.interference(leaves)
        fc.ok(not h, 'no interference at pitch %.0f, roll %.0f deg %s' % (p_, r_, h or ''))
    pose(0.0, 0.0)
    # the moving groups, at rest, about the world origin (on both axes)
    gp = {k: group_props(occ[k]) for k in ('roll', 'carrier', 'inL', 'inR', 'frame', 'mL')}
    meas = dict(roll=gp['roll']['m'], roll_z=gp['roll']['z'], roll_Iz=gp['roll']['Izz'],
                roll_Ix=gp['roll']['Iyy'] - gp['roll']['m'] * (gp['roll']['z'] ** 2 + gp['roll']['x'] ** 2) * 1e-6,
                carrier=gp['carrier']['m'], carrier_z=gp['carrier']['z'], carrier_Iy=gp['carrier']['Iyy'],
                side=gp['inL']['m'], side_Iy=gp['inL']['Iyy'])
    print('  MEASURED ' + json.dumps({k: round(v, 7) for k, v in meas.items()}))
    tot = sum(group_props(x)['m'] for x in root.occurrences)
    print('  the wrist in all: %.3f kg (the frame %.3f, its share of the motors in it); each motor output %.3f kg'
          % (tot, gp['frame']['m'], gp['mL']['m']))
    for x in leaves:
        if x.component.name in ('Side bevel gear with 36-groove pulley', 'Output bevel gear', 'Carrier block', 'Trunnion',
                                'Roll shaft', 'Tool flange ISO 9409-1-31.5-4-M5', 'Side cheek', 'Base plate',
                                'Servo motor with brake and 15:1 gearhead'):
            print('  %s: %.4f kg' % (x.component.name, x.physicalProperties.mass))
    fc.refused(ns, 'carrier needs', module=1.25, teeth=40, face=11.0)
    fc.refused(ns, 'Table 13-3', module=1.5, teeth=40, face=13.0)
    fc.refused(ns, '40 teeth on 40', module=2.0, teeth=30, face=11.0)
    fc.verdict()
