# PalletizerRobot (Chapter 43) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted from Fusion's components; at the cycle's two points, J1 at -30 deg, the working range's six corners and full
# reach: every pin's two (or three) parts meet on one axis, no interference, the flange's face where the kinematics
# put it; the parallel rod's least distance from J3's reducer at full reach; the refusal of a pose outside the range;
# each moving group weighed (mass, centre, inertia), written to lib/palletizer_weighed.json for the cycle again.
import json, math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/common')
import companion_paths                   # every lib/ folder on sys.path
import fusion_check as fc
importlib.reload(fc)
import palletizer_design as pd
importlib.reload(pd)
import adsk.core

PATH = '__COMPANION__/projects/palletizer/PalletizerRobot/PalletizerRobot.py'
OUT = '__COMPANION__/projects/palletizer/lib/palletizer_weighed.json'


def world_point(occ, p):
    q = adsk.core.Point3D.create(p[0] * 0.1, p[1] * 0.1, p[2] * 0.1)
    q.transformBy(occ.transform2)
    return (q.x * 10, q.y * 10, q.z * 10)


def leaf(G, group, name):
    for o in G[group].childOccurrences:
        if o.component.name == name:
            return o
    raise RuntimeError('no %s in %s' % (name, group))


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    G = ns['_last']['groups']
    H = ns['HOME']
    Y = ns['Y']
    pl_ = ns['parts_list'](root)
    print('  parts: %d components, %d kinds' % (sum(pl_.values()), len(pl_)))
    print('  ' + ', '.join('%s %d' % (k, v) for k, v in sorted(pl_.items())))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    # the pins: each pivot's point carried by every group that meets there, at the pivot's layer
    yr, yl = Y['rod_p'], Y['rod_l']
    pins = {'C': [('Crank', yr), ('Parallel rod', yr)], 'D': [('Upper arm', yr), ('Parallel rod', yr)],
            'E': [('Turret', yl), ('Levelling rod 1', yl)], 'F': [('Triangle', yl), ('Levelling rod 1', yl)],
            'G': [('Triangle', yl), ('Levelling rod 2', yl)], 'H': [('Wrist', yl), ('Levelling rod 2', yl)],
            'B': [('Lower arm', 0.0), ('Upper arm', 0.0), ('Triangle', 0.0)], 'W': [('Upper arm', 0.0), ('Wrist', 0.0)]}
    lo, hi = pd.waypoints()['lo'], pd.waypoints()['hi']
    ws = pd.workspace()
    poses = [('low point', 0.0, lo['q2'], lo['q3']), ('low point, J1 -30', -30.0, lo['q2'], lo['q3']),
             ('high point', 0.0, hi['q2'], hi['q3'])]
    poses += [('corner %d' % (i + 1), 0.0, q2, q3) for i, (q2, q3) in enumerate(pd.range_polygon())]
    poses.append(('reach', 0.0, ws['reach_q2'], ws['reach_q3']))
    worst_pin, worst_fl, hits = 0.0, 0.0, []
    mm_ = adsk.core.Application.get().measureManager
    rod = leaf(G, 'Parallel rod', 'Parallel rod body').bRepBodies.item(0)
    rv3 = leaf(G, 'Turret', 'RV-500N case J3').bRepBodies.item(0)
    gap = {}
    for name, q1, q2, q3 in poses:
        ns['set_pose'](G, q1, q2, q3)
        for k, members in pins.items():
            pts = [world_point(G[g], (H[k][0], y, H[k][1])) for g, y in members]
            for p in pts[1:]:
                worst_pin = max(worst_pin, math.dist(p, pts[0]))
        fl = world_point(G['Wrist'], (H['fl'][0], 0.0, H['fl'][1]))
        f = pd.fk(q2, q3)
        c1, s1 = math.cos(math.radians(q1)), math.sin(math.radians(q1))
        worst_fl = max(worst_fl, math.dist(fl, (c1 * f[0], s1 * f[0], f[1])))
        h = fc.interference(leaves)
        if h:
            hits.append((name, h))
        d = mm_.measureMinimumDistance(rod, rv3).value * 10.0
        gap[name] = round(d, 1)
    ns['set_pose'](G, 0.0, lo['q2'], lo['q3'])
    fc.ok(worst_pin < 0.001, 'at %d poses every pin\'s parts meet on one axis, worst %.6f mm' % (len(poses), worst_pin))
    fc.ok(worst_fl < 0.001, 'the flange\'s face where the kinematics put it, worst %.6f mm' % worst_fl)
    fc.ok(not hits, 'no interference among the %d parts at the low and high points, J1 at -30, the six corners and '
                    'full reach %s' % (len(leaves), hits or ''))
    fc.ok(min(gap.values()) >= 5.0, 'the parallel rod clear of J3\'s reducer by at least 5 mm: %s' % gap)
    fc.refused(ns, 'outside', q2=85.0, q3=60.0)
    # the groups weighed at q = 0
    root = fc.design().rootComponent if False else root
    ns['set_pose'](G, 0.0, 0.0, 0.0)
    spin = ('RV-80N output J4', 'Tool flange')

    def props(occs):
        m = sum(o.physicalProperties.mass for o in occs)
        c = [sum(o.physicalProperties.mass * getattr(o.physicalProperties.centerOfMass, a) * 10.0 for o in occs) / m
             for a in 'xyz']
        I = [0.0] * 6
        for o in occs:
            r = o.physicalProperties.getXYZMomentsOfInertia()      # kg cm^2 about the world's origin
            for i in range(6):
                I[i] += r[i + 1]
        cm = [v / 1000.0 for v in c]                                # m
        Ixx = I[0] * 1e-4 - m * (cm[1] ** 2 + cm[2] ** 2)
        Iyy = I[1] * 1e-4 - m * (cm[0] ** 2 + cm[2] ** 2)
        Izz = I[2] * 1e-4 - m * (cm[0] ** 2 + cm[1] ** 2)
        return m, c, (Ixx, Iyy, Izz)
    # a probe of the convention: the counterweight block, a box, about its centre against its formula
    blk = leaf(G, 'Crank', 'Counterweight block')
    m, c, I = props([blk])
    a, h = 0.46, 0.46
    tk = blk.physicalProperties.mass / (7850.0 * a * h)
    fc.ok(abs(I[1] - m * (a * a + h * h) / 12.0) < 0.01 * I[1], 'the inertia about a centre: the block\'s Iyy %.3f '
          'against m (a^2 + h^2)/12 = %.3f kg m2' % (I[1], m * (a * a + h * h) / 12.0))
    out, rows = [], []
    AXz, AZz = ns['AX'], ns['AZ']

    def add(name, occs, move, **kw):
        m, c, I = props(occs)
        bd = dict(name=name, m=round(m, 3), move=move, y=round(c[1], 1))
        if move in ('turret', 'trB', 'trW'):
            ref = {'turret': (AXz, AZz), 'trB': H['B'], 'trW': H['W']}[move]
            bd.update(off=(round(c[0] - ref[0], 1), round(c[2] - ref[1], 1)), Izz=round(I[2], 3))
        elif move in ('q2@A', 'q2@E', 'trC'):
            piv = {'q2@A': H['A'], 'q2@E': H['E'], 'trC': H['C']}[move]
            bd.update(c=round(c[2] - piv[1], 1), Ia=round(I[2], 3), It=round(I[1], 3), In=round(I[0], 3))
        else:
            piv = {'q3@A': H['A'], 'q3@B': H['B'], 'q3@G': H['G']}[move]
            bd.update(c=round(c[0] - piv[0], 1), Ia=round(I[0], 3), It=round(I[1], 3), In=round(I[2], 3))
        bd.update(kw)
        out.append(bd)
        rows.append('%s %.1f kg at (%.1f, %.1f, %.1f) mm' % (name, m, c[0], c[1], c[2]))
    kids = lambda g: [o for o in G[g].childOccurrences if o.component.bRepBodies.count]
    add('turret', kids('Turret'), 'turret')
    add('lower arm', kids('Lower arm'), 'q2@A')
    add('crank and counterweight', kids('Crank'), 'q3@A')
    add('parallel rod', kids('Parallel rod'), 'trC')
    add('levelling rod 1', kids('Levelling rod 1'), 'q2@E')
    add('triangle', kids('Triangle'), 'trB')
    add('upper arm', kids('Upper arm'), 'q3@B')
    add('levelling rod 2', kids('Levelling rod 2'), 'q3@G')
    add('wrist housing', [o for o in kids('Wrist') if o.component.name not in spin], 'trW')
    add('tool flange', [o for o in kids('Wrist') if o.component.name in spin], 'trW', spin=True)
    bal = [o for b, r in G['balancer'] for o in (b, r)]
    add('balancer', bal, 'turret')
    base = sum(o.physicalProperties.mass for o in kids('Base'))
    print('  weighed: ' + '; '.join(rows) + '; base %.1f kg' % base)
    json.dump(dict(version=ns['VERSION'], base=round(base, 1), bodies=out), open(OUT, 'w'), indent=1)
    print('  written: lib/palletizer_weighed.json')
    ns['set_pose'](G, 0.0, lo['q2'], lo['q3'])
    fc.verdict()
