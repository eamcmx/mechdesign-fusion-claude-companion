# PalletizerDrives (Chapter 42) against the acceptance criteria of its prompt, measured on what Fusion built: the
# parts counted from Fusion's components; no interference at rest, with J1 turned through the cycle, with J2 at six
# poses (the balancer's barrels and rods set to each) and with J3 at five; at each J2 pose the pins' distance
# measured between the turret's pin and the fork's cross pin against sqrt(a^2 + b^2 + 2 a b cos q2), and each
# piston inside its barrel; the groups' masses as drawn; two refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/scripts/PalletizerDrives/PalletizerDrives.py'
EXPECTED = {'Balancer barrel': 2, 'Balancer cross pin Q': 1, 'Balancer lug': 3, 'Balancer pin P': 1, 'Balancer rod': 2,
            'Base flange': 1, 'Cheek J2 side': 1, 'Cheek J3 side': 1, 'Column': 1, 'Counterweight block': 1,
            'Coupling disc': 2, 'Coupling spacer': 1, 'Crank shaft': 1, 'Crank web': 1, 'Floor plate': 1,
            'Fork plate': 2, 'Gas bottle': 2, 'Hub boss J2 side': 1, 'Hub boss J3 side': 1, 'J1 reducer plate': 1,
            'J1 stop': 1, 'J1 stop dog': 1, 'J2 support bearing': 1, 'J3 bracket': 1, 'J3 bracket post': 2,
            'J3 support bearing': 1, 'Lower arm root box': 1, 'RV-500N case J1': 1, 'RV-500N case J2': 1,
            'RV-500N case J3': 1, 'RV-500N output J1': 1, 'RV-500N output J2': 1, 'RV-500N output J3': 1,
            'Servo motor J1': 1, 'Servo motor J2': 1, 'Servo motor J3': 1, 'Slewing ring balls': 1,
            'Slewing ring inner ring': 1, 'Slewing ring outer ring': 1, 'Turret plate': 1}


def world_point(occ, p):
    m = occ.transform2
    q = adsk.core.Point3D.create(p[0] * 0.1, p[1] * 0.1, p[2] * 0.1)
    q.transformBy(m)
    return (q.x * 10, q.y * 10, q.z * 10)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['Base', 'Turret'], 'two top-level groups: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds' % (sum(pl.values()), len(pl)))
    fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    c = fc.interference(leaves)
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))
    L = ns['_last']
    (j1, s1), (j2, s2), (j3, s3) = L['joints'][1], L['joints'][2], L['joints'][3]
    # J1 through the cycle's 30 deg either way
    hits = []
    for deg in (-30.0, 30.0):
        j1.jointMotion.rotationValue = s1 * math.radians(deg)
        adsk.doEvents()
        ns['set_balancer'](L['cyls'], 0.0, L['occ']['tur'])
        adsk.doEvents()
        h = fc.interference(leaves)
        if h:
            hits.append((deg, h))
    j1.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    ns['set_balancer'](L['cyls'], 0.0)
    adsk.doEvents()
    fc.ok(not hits, 'J1 at -30 and +30 deg: no interference %s' % (hits or ''))
    # J2 through its range, the balancer set to each pose
    a, b = ns['A_MM'], ns['B_MM']
    AX, AZ = ns['AX'], ns['AZ']
    P = (AX, 0.0, AZ - a)
    hits, worst, travel = [], 0.0, []
    for q2 in (-42.0, 0.0, 13.0, 26.0, 60.0, 85.0):
        j2.jointMotion.rotationValue = s2 * math.radians(q2)
        adsk.doEvents()
        Lset = ns['set_balancer'](L['cyls'], q2)
        adsk.doEvents()
        Q = world_point(L['occ']['la'], (AX, 0.0, AZ + b))
        Lm = math.dist(Q, P)
        Lf = math.sqrt(a * a + b * b + 2 * a * b * math.cos(math.radians(q2)))
        worst = max(worst, abs(Lm - Lf))
        sp = Lm - ns['ROD_LEN']
        travel.append((q2, round(sp, 1)))
        h = fc.interference(leaves)
        if h:
            hits.append((q2, h))
    j2.jointMotion.rotationValue = 0.0
    ns['set_balancer'](L['cyls'], 0.0)
    adsk.doEvents()
    fc.ok(worst < 0.01, 'J2 at -42, 0, 13, 26, 60 and 85 deg: the pins\' distance measured against sqrt(a^2 + b^2 + 2 a b cos q2), worst %.4f mm' % worst)
    lo, hi = ns['BARREL'][0] + 10.0 + 20.0, ns['BARREL'][1] - 10.0
    fc.ok(all(lo <= s <= hi for _, s in travel), 'each piston inside its barrel\'s travel, %.0f to %.0f mm from the pin: %s' % (lo, hi, travel))
    fc.ok(not hits, 'J2 at its six poses, the balancer set to each: no interference %s' % (hits or ''))
    # J3 through its range
    hits = []
    for q3 in (-75.0, -51.0, 0.0, 30.0, 60.0):
        j3.jointMotion.rotationValue = -s3 * math.radians(q3)
        adsk.doEvents()
        h = fc.interference(leaves)
        if h:
            hits.append((q3, h))
    j3.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.ok(not hits, 'J3 at -75, -51, 0, 30 and 60 deg: no interference %s' % (hits or ''))
    # the masses as drawn
    def mass(pred):
        return sum(x.physicalProperties.mass for x in leaves if pred(x))
    groups = {}
    for x in leaves:
        top = x.fullPathName.split('+')[0].split(':')[0]
        sub = x.fullPathName.split('+')[1].split(':')[0] if '+' in x.fullPathName else ''
        key = sub if sub in ('Lower arm', 'Crank') else top
        groups[key] = groups.get(key, 0.0) + x.physicalProperties.mass
    print('  masses as drawn: %s' % ', '.join('%s %.1f kg' % (k, v) for k, v in sorted(groups.items())))
    ring = mass(lambda x: x.component.name.startswith('Slewing ring'))
    rvs = [mass(lambda x, j=j: x.component.name in ('RV-500N case J%d' % j, 'RV-500N output J%d' % j)) for j in (1, 2, 3)]
    print('  the slewing ring as drawn %.1f kg (SKF 48.0); each RV-500N as drawn %.1f kg (Nabtesco 57.2)' % (ring, rvs[0]))
    blk = mass(lambda x: x.component.name == 'Counterweight block')
    fc.ok(abs(blk - 100.0) < 0.5, 'the counterweight block %.2f kg' % blk)
    fc.refused(ns, 'over', cw=300.0)
    fc.refused(ns, 'floor', r_cw=1100.0)
    fc.verdict()
