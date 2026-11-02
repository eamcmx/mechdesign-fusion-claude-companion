# FlapActuator (Chapter 33) against the acceptance criteria of its prompt, measured on what Fusion built: the screw's
# life and torques recomputed here from the linkage; the parts counted from Fusion's components; no interference
# retracted, at a third, two thirds and the full stroke; the rod end's centre from the gimbal at both ends of the
# stroke; the actuator inside the wing's skins at 0, 20 and 40 deg; its mass; a refusal.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/FlapActuator/FlapActuator.py'
EXPECTED = {'Actuator housing': 1, 'Ball nut R32-10T4-RSI': 1, 'Ball screw 32x10': 1, 'Bearing 6201': 2,
            'Bearing BSD 2562 CGA': 2, 'Cover screw M5x16': 6, 'Encoder': 1, 'Encoder pinion z26': 1, 'Encoder shaft': 1,
            'End bolt M8x20': 1, 'End stop': 1, 'Gear cover': 1, 'Gearhead shaft': 1, 'Guide key': 1, 'Idler gear z30': 1,
            'Idler pin': 1, 'Jam nut M20x1.5': 1, 'Key 8x7x16': 1, 'Motor screw M5x12': 4, 'Motor unit': 1,
            'Nut key 6x6.5x32': 1, 'Nut retaining ring': 1, 'Pinion z18': 1, 'Piston rod': 1, 'Ring nut M66x1.5': 1,
            'Rod end SI 20 ES': 1, 'Rod seal': 1, 'Screw gear z54': 1, 'Stop bolt M8x16': 1, 'Trunnion': 2, 'Washer M8': 2}
A, R_HORN, TH0 = (-420.0, 114.0), 120.0, 110.0
UP = ((-430.0, 175.0), (-60.0, 160.0))          # the brief's skins in the wing's section (Chapter 32)
LOW = ((-430.0, -90.0), (-90.0, -55.0))


def tip(d):
    t = math.radians(TH0 - d)
    return R_HORN * math.cos(t), R_HORN * math.sin(t)


def own_numbers():
    """The force over the stroke from the hinge moment and the arm, its cube mean, the nut's life, the torques."""
    q = 0.5 * 1.225 * 70.0 ** 2
    rows = []
    for i in range(401):
        d = 40.0 * i / 400
        H = q * 3.2 * 0.55 * 1.2 * d / 40.0 * 0.45 * 0.55 * 1000.0
        tx, tz = tip(d)
        ax, az = tx - A[0], tz - A[1]
        Lg = math.hypot(ax, az)
        arm = abs(tx * az / Lg - tz * ax / Lg)
        rows.append((Lg, H / arm))
    num = sum(((F1 + F2) / 2) ** 3 * (L2 - L1) for (L1, F1), (L2, F2) in zip(rows, rows[1:]))
    stroke = rows[-1][0] - rows[0][0]
    Fcm = (num / stroke) ** (1 / 3)
    life = (32520 / (3.0 * Fcm)) ** 3 * 1e6 / (60000 * stroke / 10)
    Fl = max(F for _, F in rows)
    lam = math.atan(10 / (math.pi * 32))
    c45 = math.cos(math.radians(45))
    eta = (c45 - 0.005 * math.tan(lam)) / (c45 + 0.005 / math.tan(lam))
    Tb = 1.5 * Fl * 32 * (0.005 * math.pi * 32 - 10 * c45) / (2 * (math.pi * 32 * c45 + 0.005 * 10))
    return dict(life=life, T_lim=Fl * 10 / (2 * math.pi * eta), T_back=Tb, stroke=stroke, L0=rows[0][0], L1=rows[-1][0])


def skin(line, x):
    (x0, z0), (x1, z1) = line
    return z0 + (z1 - z0) * (x - x0) / (x1 - x0)


def face_boxes(leaves):
    """Every face's box in the actuator's frame at rest, where nothing is turned (a turned part's box is loose), with
    whether the face moves with the rod."""
    out = []
    for o in leaves:
        moves = o.assemblyContext is not None and o.assemblyContext.component.name == 'Rod'
        for body in o.bRepBodies:
            for f in body.faces:
                bb = f.boundingBox
                out.append((o.component.name, moves, [bb.minPoint.y * 10, bb.maxPoint.y * 10],
                            bb.minPoint.z * 10, bb.maxPoint.z * 10))
    return out


def envelope(boxes, d, slide):
    """The least room above and below the actuator, in the wing's section with the flap at d and the rod out by
    `slide` mm: the box corners of every face, mapped from the actuator's frame (y along it from the gimbal, z up)
    into the section. The turning parts are round about their axes, so their boxes at rest hold at every angle."""
    tx, tz = tip(d)
    Lg = math.hypot(tx - A[0], tz - A[1])
    u = ((tx - A[0]) / Lg, (tz - A[1]) / Lg)
    n = (-u[1], u[0])
    up = low = (1e9, '')
    for name, moves, ys, zlo, zhi in boxes:
        for y in ys:
            y = y + (slide if moves else 0.0)
            for z, side in ((zhi, 1), (zlo, -1)):
                px, pz = A[0] + y * u[0] + z * n[0], A[1] + y * u[1] + z * n[1]
                if side > 0:
                    up = min(up, (skin(UP, px) - pz, name))
                else:
                    low = min(low, (pz - skin(LOW, px), name))
    return up, low


def rod_end_y(leaves):
    for b in leaves['Rod end SI 20 ES'].bRepBodies:   # the occurrence's bodies: world coordinates
        if b.name == 'Inner ring':
            bb = b.boundingBox
            return (bb.minPoint.y + bb.maxPoint.y) * 5.0
    raise RuntimeError('no inner ring')


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    own = own_numbers()
    o = ns['design']()
    pairs = (('life', o['nut']['life']), ('T_lim', o['T_lim']), ('T_back', o['T_back']))
    bad = [(k, x, own[k]) for k, x in pairs if abs(x - own[k]) > 1e-4 * abs(own[k])]
    fc.ok(not bad, 'the nut\'s life %.4f, the screw\'s torque %.1f N mm and its back-driving torque %.1f N mm '
          'recomputed from the linkage %s' % (own['life'], own['T_lim'], own['T_back'], bad or ''))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['Housing', 'Screw', 'Rod', 'Idler', 'Pinion', 'Encoder pinion'], 'six subassemblies: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds: %s' % (sum(pl.values()), len(pl), sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    fc.ok(not c, 'no interference among the %d parts, retracted %s' % (len(leaves), c or ''))
    y0 = rod_end_y(leaves)
    fc.ok(abs(y0 - own['L0']) < 0.01, 'the rod end\'s centre %.3f mm from the gimbal, retracted (%.3f)' % (y0, own['L0']))
    jr = fc.joint('Rod slides')
    lim = jr.jointMotion.slideLimits
    ext = lim.maximumValue if abs(lim.maximumValue) > abs(lim.minimumValue) else lim.minimumValue
    js = fc.joint('Screw turns')
    hits = []
    room = []
    boxes = face_boxes(list(leaves.values()))
    for d in (0.0, 10.0, 20.0, 30.0, 40.0):                    # the rod at the length that holds the flap at d
        tx, tz = tip(d)
        out = math.hypot(tx - A[0], tz - A[1]) - own['L0']
        jr.jointMotion.slideValue = ext * out / own['stroke']
        adsk.doEvents()
        room.append((d,) + envelope(boxes, d, out))
        if d > 0:
            h = fc.interference(list(leaves.values()))
            if h:
                hits.append((d, h))
    y1 = rod_end_y(leaves)
    turns = js.jointMotion.rotationValue / (2 * math.pi)
    fc.ok(not hits, 'no interference with the flap at 10, 20, 30 and 40 deg %s' % (hits or ''))
    fc.ok(abs(y1 - own['L1']) < 0.01 and abs(abs(turns) - own['stroke'] / 10) < 1e-3,
          'extended: the rod end\'s centre %.3f mm from the gimbal (%.3f), the screw turned %.4f turns'
          % (y1, own['L1'], turns))
    up, low = min(r[1] for r in room), min(r[2] for r in room)
    fc.ok(up[0] > 2.0 and low[0] > 2.0, 'inside the skins with the flap at 0 to 40 deg: %.2f mm to spare above (%s), '
          '%.2f below (%s)' % (up[0], up[1], low[0], low[1]))
    ymin = min(x.boundingBox.minPoint.y * 10 for x in leaves.values())
    fc.ok(ymin > -25.0, 'nothing more than %.2f mm forward of the gimbal\'s axis: room for the fitting' % -ymin)
    jr.jointMotion.slideValue = 0.0
    adsk.doEvents()
    mass = sum(x.physicalProperties.mass for k, x in leaves.items() if k != 'Motor unit')
    tot = mass + ns['fd'].LAY['motor_mass']
    fc.ok(tot <= 9.0, 'mass %.2f kg with the motor unit at its specified %.1f kg (the brief allows 9 kg)'
          % (tot, ns['fd'].LAY['motor_mass']))
    for k in ('Actuator housing', 'Ball screw 32x10', 'Piston rod', 'Screw gear z54'):
        print('  %s %.3f kg' % (k, leaves[k].physicalProperties.mass))
    fc.refused(ns, 'strokes', fp=5.0)
    fc.verdict()
