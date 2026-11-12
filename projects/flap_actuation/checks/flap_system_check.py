# FlapSystem (Chapter 34) against the acceptance criteria of its prompt, measured on what Fusion built: the flap's
# forces and the hinges' reaction worked again here; the parts counted from Fusion's components; with the flap at 0,
# 10, 20, 30 and 40 deg, the rod end's centre on the horn's pin and no interference; the hinges' bearings on the
# hinge line; the transmission angle measured in Fusion; the system's mass; a refusal.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/flap_actuation/FlapSystem/FlapSystem.py'
EXPECTED = {'Actuator housing': 1, 'Ball nut R32-10T4-RSI': 1, 'Ball screw 32x10': 1, 'Bearing 6201': 2, 'Bearing BSD 2562 CGA': 2, 'Bearing GE 20 ES fitting': 2, 'Bearing GE 20 ES hinge': 2, 'Bracket bolt M8x25': 4, 'Bracket nut M8': 4, 'Cover screw M5x16': 6, 'Encoder': 1, 'Encoder pinion z26': 1, 'Encoder shaft': 1, 'End bolt M8x20': 1, 'End stop': 1, 'Fitting bolt M8x35': 4, 'Fitting nut M8': 4, 'Flap panel': 1, 'Gear cover': 1, 'Gearhead shaft': 1, 'Guide key': 1, 'Hinge bracket': 2, 'Hinge fitting': 4, 'Hinge pin': 2, 'Hinge pin nut': 2, 'Hinge rib': 2, 'Horn': 1, 'Horn pin': 1, 'Horn pin nut': 1, 'Idler gear z30': 1, 'Idler pin': 1, 'Jam nut M20x1.5': 1, 'Key 8x7x16': 1, 'Lower skin': 1, 'Motor screw M5x12': 4, 'Motor unit': 1, 'Nut key 6x6.5x32': 1, 'Nut retaining ring': 1, 'Pinion z18': 1, 'Piston rod': 1, 'Rear spar web': 1, 'Ring nut M60x1.5': 1, 'Rod end SI 20 ES': 1, 'Rod seal': 1, 'Screw gear z54': 1, 'Spacer': 2, 'Spar fitting': 1, 'Stop bolt M8x16': 1, 'Trunnion': 2, 'Upper skin': 1, 'Washer M8': 2}
STRUCTURE = ('Flap panel', 'Rear spar web', 'Upper skin', 'Lower skin')
# the actuator's parts that can meet the rest of the system; its inside was checked over its stroke in Chapter 33
OUTER = ('Actuator housing', 'Gear cover', 'Cover screw', 'Trunnion', 'Motor unit', 'Motor screw', 'Encoder', 'Piston rod',
         'Rod seal', 'Jam nut', 'Rod end')     # drawn from the brief, not designed here


def own_numbers():
    """At 40 deg and the ultimate: the air's normal force, the actuator's push, the hinges' reaction."""
    q = 0.5 * 1.225 * 70.0 ** 2
    N = 1.5 * q * 3.2 * 0.55 * 1.2
    th = math.radians(70.0)
    tip = (120 * math.cos(th), 120 * math.sin(th))
    ax, az = tip[0] + 420, tip[1] - 114
    Lg = math.hypot(ax, az)
    u = (ax / Lg, az / Lg)
    arm = abs(tip[0] * u[1] - tip[1] * u[0])
    F = N * 0.2475 * 1000 / arm
    d = math.radians(40)
    R = (-(N * math.sin(d) + F * u[0]), -(N * math.cos(d) + F * u[1]))
    return dict(F=F, R_hinge=math.hypot(*R) / 2)


def centre(o, body_name=None):
    for b in o.bRepBodies:
        if body_name is None or b.name == body_name:
            bb = b.boundingBox
            return [(bb.minPoint.x + bb.maxPoint.x) * 5, (bb.minPoint.y + bb.maxPoint.y) * 5, (bb.minPoint.z + bb.maxPoint.z) * 5]
    raise RuntimeError('no body %s in %s' % (body_name, o.name))


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    own = own_numbers()
    D = ns['design']()
    s = D['sys']
    bad = [(k, a, b) for k, a, b in (('F', s['ult']['F'], own['F']), ('R_hinge', s['R_hinge'], own['R_hinge']))
           if abs(a - b) > 1e-6 * abs(b)]
    fc.ok(not bad, 'at 40 deg and the ultimate: the push %.1f N and each hinge\'s reaction %.1f N worked again %s'
          % (own['F'], own['R_hinge'], bad or ''))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['Wing', 'Actuator', 'Flap'], 'three subassemblies: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds: %s' % (sum(pl.values()), len(pl), sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = {x.fullPathName: x for x in root.allOccurrences if x.component.bRepBodies.count}
    by = {x.component.name: x for x in leaves.values()}
    near = [x for x in leaves.values() if not x.fullPathName.startswith('Actuator')
            or any(x.component.name.startswith(k) for k in OUTER)]
    print("  interference over %d parts: the system and the outside of the actuator" % len(near))
    hits, gaps, mus = [], [], []
    for d in (0.0, 10.0, 20.0, 30.0, 40.0):
        ns['pose'](d, ns['_last_joints'])
        adsk.doEvents()
        rc = centre(by['Rod end SI 20 ES'], 'Inner ring')
        pc = centre(by['Horn pin'], 'Horn pin')
        gaps.append((d, math.dist(rc[1:], pc[1:])))
        g = centre(by['Trunnion R'])
        v1 = (pc[1] - g[1], pc[2] - g[2])                          # the actuator's line, gimbal to pin
        v2 = (pc[1], pc[2])                                       # the horn, hinge to pin
        c = abs(v1[0] * v2[0] + v1[1] * v2[1]) / (math.hypot(*v1) * math.hypot(*v2))
        mus.append((d, math.degrees(math.acos(c))))
        h = fc.interference(near)
        if h:
            hits.append((d, h))
    worst = max(gp for _, gp in gaps)
    fc.ok(worst < 0.01, 'the rod end\'s centre on the horn\'s pin at 0 to 40 deg: the largest gap %.4f mm' % worst)
    print('  transmission angles measured: %s' % ', '.join('%.0f deg %.2f' % m for m in mus))
    fc.ok(abs(mus[0][1] - 69.81) < 0.05 and abs(mus[-1][1] - 70.15) < 0.05,
          'the transmission angle measured in Fusion: %.2f deg flap up, %.2f at 40 deg (69.81 and 70.15)' % (mus[0][1], mus[-1][1]))
    fc.ok(not hits, 'no interference among the %d parts at 0, 10, 20, 30 and 40 deg %s' % (len(near), hits or ''))
    ns['pose'](0.0, ns['_last_joints'])
    adsk.doEvents()
    line = [centre(by['Bearing GE 20 ES hinge %s' % t]) for t in ('L', 'R')]
    off = max(math.hypot(p[1], p[2]) for p in line)
    fc.ok(off < 0.01, 'both hinges\' bearings on the hinge line: %.4f mm off it at most, %.0f mm apart'
          % (off, abs(line[0][0] - line[1][0])))
    mass = sum(x.physicalProperties.mass for x in leaves.values()
               if x.component.name not in STRUCTURE and x.component.name != 'Motor unit')
    tot = mass + ns['fd'].LAY['motor_mass']
    print('  the system\'s mass, the flap and the wing\'s structure left out, the motor unit at 1.6 kg: %.2f kg' % tot)
    for k in ('Spar fitting', 'Horn', 'Hinge bracket L'):
        print('  %s %.3f kg' % (k, by[k].physicalProperties.mass))
    fc.refused(ns, 'strokes', fp=5.0)
    fc.verdict()
