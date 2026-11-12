# RearAxle (Chapter 31) against the acceptance criteria of its prompt, measured on what Fusion built: the splines'
# engagement worked again here; the parts of both subsystems counted from Fusion's components; no interference among
# all of them at rest; the splines' reach into the side gears measured on the bodies; the axle driven straight (the
# pinion turned, both wheels following the case) and in a turn (a spider turned as well: the wheels' mean is the
# case's turn, their ratio the turn's 6.28/4.72), with no interference; the refusal of short splines.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/RearAxle/RearAxle.py'
EXPECTED = {'Adjuster': 2, 'Bearing A 32309 cone': 1, 'Bearing A 32309 cup': 1, 'Bearing B 32308 cone': 1,
            'Bearing B 32308 cup': 1, 'Bearing C1 32213 cone': 1, 'Bearing C1 32213 cup': 1, 'Bearing C2 32213 cone': 1,
            'Bearing C2 32213 cup': 1, 'Bearing cap': 2, 'Cap bolt M12x90': 4, 'Carrier casting': 1,
            'Companion flange': 1, 'Cross': 1, 'Crown wheel bolt M12x35': 10, 'Crown wheel z47': 1,
            'Differential case': 1, 'Pinion depth shim': 1, 'Pinion nut M30': 1, 'Pinion seal 60x95x10': 1,
            'Pinion shaft z12': 1, 'Side gear': 2, 'Side gear washer': 2, 'Spacer': 1, 'Spider': 4,
            'Spider washer': 4, 'Washer': 1,
            'Banjo housing': 1, 'Bearing 6213 balls': 2, 'Bearing 6213 inner ring': 2, 'Bearing 6213 outer ring': 2,
            'Breather': 1, 'Carrier nut M12': 8, 'Carrier stud M12': 8, 'Cover screw M8x20': 10, 'Drain plug M22': 1,
            'Filler plug M22': 1, 'Half shaft': 2, 'Oil seal 74x100x10': 2, 'Rear cover': 1, 'Retainer plate': 2,
            'Retaining ring': 2, 'Wheel stud M14': 12}
TURN = 6.28 / 4.72          # the outer wheel's path over the inner one's in the tightest turn (Chapter 28)


def own_numbers():
    """The splines in the side gear, worked again: 35 mm tip, the hub's end at 74 + 32.75 mm, the design torque."""
    T = 450 * 4.70 * 0.95 * 47 / 12 * 0.96 / 2 * 1000
    l = 74 + 32.75 - 35.0
    p = T / (0.5 * 8 * 18.75 * 2.5 * l)
    return dict(l=l, p=p, l_need=2 * T / (0.5 * 8 * 18.75 * 2.5 * 1140))


def reach(leaves, tag, s):
    """How far the half shaft's splines reach into its side gear, measured on the bodies (mm)."""
    sh = leaves['Half shaft %s' % tag].bRepBodies.item(0).boundingBox
    gr = leaves['Side gear %s' % tag].bRepBodies.item(0).boundingBox
    tip = min(s * sh.minPoint.y, s * sh.maxPoint.y) * 10
    hub_end = max(s * gr.minPoint.y, s * gr.maxPoint.y) * 10
    return hub_end - tip, tip, hub_end


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    o = own_numbers()
    sp = ns['design']()['sp']
    bad = [k for k in ('l', 'p', 'l_need') if abs(sp[k] - o[k]) > 1e-6 * abs(o[k])]
    fc.ok(not bad, 'the splines recomputed: %.2f mm engaged, %.1f mm needed, %.1f MPa on the flanks %s'
          % (o['l'], o['l_need'], o['p'], bad or ''))
    print(ns['build']())
    fc.mark()
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(len(subs) == 6, 'six subassemblies: %s' % subs)
    pl = ns['CAR'].parts_list(root)
    fc.ok(pl == EXPECTED, 'the parts, counted from Fusion: %d components, %d kinds %s'
          % (sum(pl.values()), len(pl), '' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))
    for tag, s in (('L', 1), ('R', -1)):
        l, tip, end = reach(leaves, tag, s)
        fc.ok(l >= o['l_need'], 'half shaft %s: its splines reach %.2f mm into the side gear (tip at %.2f, the hub ends '
              'at %.2f); %.1f needed' % (tag, l, tip, end, o['l_need']))
    # a joint made in the root between two parts of one subassembly is kept by that subassembly (Chapter 29)
    joints = {j.name: j for comp in fc.design().allComponents for j in comp.asBuiltJoints}
    links = sorted(m.name for comp in fc.design().allComponents for m in comp.motionLinks)
    print('  %d joints, %d motion links: %s' % (len(joints), len(links), links))
    jp, j1 = joints['Pinion turns'], joints['Spider 1 turns in the case']
    case = [x for x in root.allOccurrences if x.component.name == 'Differential case'][0]
    R0 = {k: fc.rotation(v) for k, v in (('case', case), ('L', leaves['Half shaft L']), ('R', leaves['Half shaft R']))}

    def turned():
        return {k: fc.turn_deg(fc.rotation(v), R0[k], (0, 1, 0))
                for k, v in (('case', case), ('L', leaves['Half shaft L']), ('R', leaves['Half shaft R']))}
    # straight ahead: the pinion turned 30 deg, the side gears still in the case
    jp.jointMotion.rotationValue = math.radians(30.0)
    adsk.doEvents()
    t = turned()
    fc.ok(abs(abs(t['case']) - 30 * 12 / 47) < 1e-3 and abs(t['L'] - t['case']) < 1e-3 and abs(t['R'] - t['case']) < 1e-3,
          'straight: the pinion turned 30 deg, the case %.4f deg, the half shafts %.4f and %.4f' % (t['case'], t['L'], t['R']))
    hit = fc.interference(list(leaves.values()))
    fc.ok(not hit, 'no interference driving straight %s' % (hit or ''))
    # a turn: the pinion 47 deg (the case 12), a spider 2.5 deg in the case. Fusion keeps a joint's angle to 0.1 deg,
    # so the angles are ones it holds exactly; the side gears then turn 2.5 x 15/22 deg in the case, opposite ways
    jp.jointMotion.rotationValue = math.radians(47.0)
    j1.jointMotion.rotationValue = math.radians(2.5)
    adsk.doEvents()
    t = turned()
    held = {n: math.degrees(joints[n].jointMotion.rotationValue)
            for n in ('Pinion turns', 'Case turns', 'Spider 1 turns in the case', 'Side gear L turns in the case',
                      'Side gear R turns in the case')}
    print('  the joints hold (deg): %s' % ', '.join('%s %.4f' % kv for kv in held.items()))
    half = abs(t['L'] - t['R']) / 2
    hi, lo = max(abs(t['L']), abs(t['R'])), min(abs(t['L']), abs(t['R']))
    fc.ok(abs((t['L'] + t['R']) / 2 - 12.0) < 1e-3 and abs(half - 2.5 * 15 / 22) < 1e-3,
          'in a turn: the case %.4f deg, the half shafts %.4f and %.4f; their mean %.4f, half their difference %.4f '
          '(2.5 x 15/22 = %.4f), their ratio %.4f (the tightest turn, 6.28/4.72 = %.4f)'
          % (t['case'], t['L'], t['R'], (t['L'] + t['R']) / 2, half, 2.5 * 15 / 22, hi / lo, TURN))
    hit = fc.interference(list(leaves.values()))
    fc.ok(not hit, 'no interference in the turn %s' % (hit or ''))
    j1.jointMotion.rotationValue = 0.0
    jp.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    mass = sum(x.physicalProperties.mass for x in leaves.values())
    print('  mass of the axle %.2f kg' % mass)
    for top in root.occurrences:
        own = [o for o in top.component.allOccurrences if o.component.bRepBodies.count]
        print('  %s: %d parts, %.2f kg' % (top.component.name, len(own), sum(o.physicalProperties.mass for o in own)))
    hsg = ns['HSG']
    keep = hsg.Y_TIP
    try:
        hsg.Y_TIP = 100.0
        ns['refuse'](ns['design']())
        fc.ok(False, 'splines from 100 mm refused')
    except ValueError as e:
        fc.ok('splines' in str(e), 'splines from 100 mm refused: %s' % e)
    finally:
        hsg.Y_TIP = keep
    fc.verdict()
