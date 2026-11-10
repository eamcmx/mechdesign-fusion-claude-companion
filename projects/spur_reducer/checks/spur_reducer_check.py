# SpurReducer (Chapter 27) against the acceptance criteria of its prompt, measured on what Fusion built: every number
# of the sizing recomputed here with formulas of this check's own; the parts list counted from Fusion's components;
# the interfaces measured on the bodies (bearing bores on their seats, outer rings in their bores, the parts along
# each shaft touching without play, the caps' spigots on the outer rings); no interference at rest or through a pinion
# pitch; the ratio measured; three refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/spur_reducer/SpurReducer/SpurReducer.py'
P, N1, KA, LIFE, ND = 4.0, 1440.0, 1.25, 20000.0, 2.0
Z1, Z2, M, F, ALPHA = 19, 95, 3.0, 30.0, math.radians(20.0)
EXPECTED = {'Bearing 6204 balls': 2, 'Bearing 6204 inner ring': 2, 'Bearing 6204 outer ring': 2, 'Bearing 6206 balls': 2,
            'Bearing 6206 inner ring': 2, 'Bearing 6206 outer ring': 2, 'Cap input blind': 1, 'Cap input through': 1,
            'Cap output blind': 1, 'Cap output through': 1, 'Cap screw M6x16': 16, 'Dowel 8x24': 2,
            'Drain plug M16': 1, 'Filler plug M16': 1, 'Gear z95': 1, 'Housing base': 1, 'Housing cover': 1,
            'Key 10x8x42 gear': 1, 'Key 5x5x28 input': 1, 'Key 8x7x52 output': 1, 'Lip seal 20x35x7': 1,
            'Lip seal 30x47x7': 1, 'Nut M10': 8, 'Output shaft': 1, 'Pinion shaft': 1, 'Screw M10x45': 8, 'Spacer': 1}


def own_numbers():
    """The sizing, worked again here."""
    T1 = P * 1e3 / (N1 * math.pi / 30)
    T2 = T1 * Z2 / Z1 * 0.98
    d1, d2 = M * Z1, M * Z2
    Ft = KA * T1 * 2e3 / d1
    W = Ft / math.cos(ALPHA)
    V = math.pi * d1 * N1 / 60e3
    Kv = 1 + V / 6.1
    Y2 = 0.435 + (0.447 - 0.435) * (95 - 75) / 25
    sc = 191 * math.sqrt(Kv * Ft / (F * math.cos(ALPHA)) * 2 / math.sin(ALPHA) * (1 / d1 + 1 / d2))
    ka = 4.51 * 630 ** -0.265

    def n_of(d, Ma, Tm):
        Se = ka * 1.24 * d ** -0.107 * 315
        return math.pi * d ** 3 / 16 / (2 * 2.14 * Ma / Se + math.sqrt(3) * 3.0 * Tm / 630)
    n_in = n_of(16, 0, KA * T1 * 1e3)
    n_gear = n_of(35, W * 100 / 4, KA * T2 * 1e3)
    n_out = n_of(25, 0, KA * T2 * 1e3)
    L10_in = (12.7e3 / (W / 2)) ** 3 * 1e6 / (60 * N1)
    L10_out = (19.5e3 / (W / 2)) ** 3 * 1e6 / (60 * N1 / 5)
    keys = [2 * KA * T * 1e3 / D / ((h - t1) * (L - b)) for D, T, b, h, t1, L in
            ((16, T1, 5, 5, 3.0, 28), (35, T2, 10, 8, 5.0, 42), (25, T2, 8, 7, 4.0, 52))]
    th1 = W * 98 ** 2 / (16 * 207e3 * math.pi * 20 ** 4 / 64)
    cycles = 60 * N1 * LIFE                                       # the pinion's, 300 HB grade 1, the lower curves
    SF1 = (0.533 * 300 + 88.3) * 1.6831 * cycles ** -0.0323 / (Kv * Ft / (F * M * 0.314))
    SH1 = (2.22 * 300 + 200) * 2.466 * cycles ** -0.056 / sc
    return dict(T1=T1, T2=T2, Ft=Ft, Kv=Kv, s1=Kv * Ft / (F * M * 0.314), s2=Kv * Ft / (F * M * Y2), sc=sc,
                n_in=n_in, n_gear=n_gear, n_out=n_out, L10_in=L10_in, L10_out=L10_out, keys=keys, theta1=th1,
                SF1=SF1, SH1=SH1)


def radii(occ):
    """The radii (mm) of the body's cylindrical faces."""
    out = set()
    for f in occ.bRepBodies.item(0).faces:
        if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
            out.add(round(f.geometry.radius * 10, 4))
    return out


def yspan(occ):
    bb = occ.bRepBodies.item(0).boundingBox
    return round(bb.minPoint.y * 10, 4), round(bb.maxPoint.y * 10, 4)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    d = ns['rd'].design()
    g, s, B, K = d['gears'], d['shafts'], d['bearings'], d['keys']
    o = own_numbers()
    got = dict(T1=g['T1'], T2=g['T2'], Ft=g['Ft'], Kv=g['Kv'], s1=g['s1'], s2=g['s2'], sc=g['sc'], n_in=s['n_in'],
               n_gear=s['n_gear'], n_out=s['n_out'], L10_in=B['in']['L10h'], L10_out=B['out']['L10h'],
               theta1=s['theta1'], SF1=g['SF1'], SH1=g['SH1'])
    bad = [k for k in got if abs(got[k] - o[k]) > 1e-9 * max(1.0, abs(o[k]))]
    bad += ['keys'] if any(abs(a - b) > 1e-9 for a, b in zip([K[k]['sigma'] for k in ('in', 'gear', 'out')], o['keys'])) else []
    fc.ok(not bad, 'the sizing recomputed independently %s' % (bad or ''))
    print('  T1 %.3f, T2 %.3f N m; Ft %.1f N, Kv %.4f; Lewis %.1f / %.1f MPa, contact %.1f MPa; DE-Goodman %.2f, %.2f, %.2f;'
          ' L10 %.0f h, %.0f h; keys %.1f, %.1f, %.1f MPa; the pinion\'s bending factor %.3f, contact factor %.4f'
          % (o['T1'], o['T2'], o['Ft'], o['Kv'], o['s1'], o['s2'], o['sc'], o['n_in'], o['n_gear'], o['n_out'],
             o['L10_in'], o['L10_out'], *o['keys'], o['SF1'], o['SH1']))

    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['Housing', 'Input shaft assembly', 'Output shaft assembly'], 'three subassemblies: %s' % subs)
    pl = ns['parts_list'](root)
    fc.ok(pl == EXPECTED, 'the parts, counted from Fusion: %d components, %d kinds %s'
          % (sum(pl.values()), len(pl), '' if pl == EXPECTED else (sorted(set(pl.items()) ^ set(EXPECTED.items())))))
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    mats = {n: leaves[n].bRepBodies.item(0).material.name for n in ('Pinion shaft', 'Output shaft', 'Gear z95',
                                                                     'Housing base', 'Cap input through')}
    fc.ok(mats == {'Pinion shaft': 'Steel', 'Output shaft': 'Steel', 'Gear z95': 'Steel', 'Housing base': 'Iron, Cast',
                   'Cap input through': 'Iron, Cast'}, 'materials %s' % mats)
    kinds = [type(j.jointMotion).__name__ for j in root.asBuiltJoints]
    fc.ok(kinds == ['RevoluteJointMotion'] * 2 and root.motionLinks.count == 1, 'two revolute As-built Joints, one link')
    c = fc.interference(list(leaves.values()))
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))

    # interfaces, measured on the bodies
    rin = radii(leaves['Pinion shaft'])
    rout = radii(leaves['Output shaft'])
    checks = []
    for name, shaft_r, row, shoulder in (('6204', rin, B['in'], 25.0), ('6206', rout, B['out'], 40.0)):
        ir, orr = leaves['Bearing %s A inner ring' % name], leaves['Bearing %s A outer ring' % name]
        checks.append((name + ' bore on its seat', row['d'] / 2 in radii(ir) and row['d'] / 2 in shaft_r))
        checks.append((name + ' width %.0f mm' % row['B'], abs(yspan(ir)[1] - yspan(ir)[0] - row['B']) < 1e-6))
        checks.append((name + ' outside in its bore', row['D'] / 2 in radii(orr) and row['D'] / 2 in radii(leaves['Housing base'])))
        checks.append((name + ' shoulder %.0f mm, at least dS = %.0f mm (Table 11-2)' % (shoulder, row['dS']),
                       shoulder / 2 in shaft_r and shoulder >= row['dS']))
    gear, spacer = leaves['Gear z95'], leaves['Spacer']
    ring_b = leaves['Bearing 6206 B inner ring']
    stack = [('shoulder to gear', -25.0 - yspan(gear)[0]), ('gear to spacer', yspan(spacer)[0] - yspan(gear)[1]),
             ('spacer to bearing', yspan(ring_b)[0] - yspan(spacer)[1])]
    checks.append(('the output shaft\'s stack without play: %s' % ', '.join('%s %.3f mm' % x for x in stack),
                   all(abs(v) < 1e-6 for _, v in stack)))
    for n in ('Cap input through', 'Cap input blind', 'Cap output blind', 'Cap output through'):
        cy = yspan(leaves[n])
        ring = leaves['Bearing %s %s outer ring' % ('6204' if 'input' in n else '6206', 'A' if cy[0] < 0 else 'B')]
        ry = yspan(ring)
        checks.append(('%s on its outer ring' % n, abs((cy[1] - ry[0]) if cy[0] < 0 else (ry[1] - cy[0])) < 1e-6))
    for msg, good in checks:
        fc.ok(good, msg)
    bb = gear.bRepBodies.item(0).boundingBox
    tip = (bb.maxPoint.x - bb.minPoint.x) * 10 / 2
    fc.ok(abs(tip - M * (Z2 + 2) / 2) < 0.05, 'the gear\'s tip radius %.3f mm (want %.1f)' % (tip, M * (Z2 + 2) / 2))

    # the ratio and the mesh
    ji, pin = fc.joint('Input shaft'), leaves['Pinion shaft']
    ro = root.occurrences.item(2)
    R0, Ri0 = fc.rotation(ro), fc.rotation(root.occurrences.item(1))
    ji.jointMotion.rotationValue = math.radians(18.0)
    adsk.doEvents()
    t_in = fc.turn_deg(fc.rotation(root.occurrences.item(1)), Ri0, (0, 1, 0))
    t_out = fc.turn_deg(fc.rotation(ro), R0, (0, 1, 0))
    fc.ok(abs(t_out + t_in / 5) < 1e-6, 'the input turned %.3f deg: the output %.3f deg (want %.3f)' % (t_in, t_out, -t_in / 5))
    # every part of each shaft assembly turns with it (Fusion grounds only the first to its parent by itself)
    ji.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    kids = [x for asm in (root.occurrences.item(1), root.occurrences.item(2)) for x in asm.childOccurrences]
    c0 = {x.component.name: x.physicalProperties.centerOfMass for x in kids}
    ji.jointMotion.rotationValue = math.radians(90.0)
    adsk.doEvents()
    off = []
    for x in kids:
        p0, p1 = c0[x.component.name], x.physicalProperties.centerOfMass
        ang = math.radians(90.0) if x.assemblyContext.component.name.startswith('Input') else math.radians(-18.0)
        cx = 0.0 if x.assemblyContext.component.name.startswith('Input') else 17.1                 # cm
        rx, rz = p0.x - cx, p0.z
        want = (cx + rx * math.cos(ang) + rz * math.sin(ang), p0.y, -rx * math.sin(ang) + rz * math.cos(ang))
        off.append((x.component.name, 10 * math.dist((p1.x, p1.y, p1.z), want)))
    worst = max(off, key=lambda t: t[1])
    fc.ok(worst[1] < 1e-6, 'all %d parts of the two shaft assemblies turned with them (worst %s, %.1e mm)'
          % (len(kids), worst[0], worst[1]))
    hits = []
    for k in range(11):
        ji.jointMotion.rotationValue = math.radians(1.9 * k)
        adsk.doEvents()
        h = fc.interference([pin, gear])
        if h:
            hits.append((round(1.9 * k, 1), h))
    ji.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.ok(not hits, 'no tooth collision while the pinion turns 0 to 19 deg, past one pitch of %.3f deg %s'
          % (360.0 / Z1, hits or ''))
    mass = sum(x.physicalProperties.mass for x in leaves.values())
    print('  mass of the reducer %.2f kg; the gear %.2f kg, the housing %.2f kg'
          % (mass, gear.physicalProperties.mass, leaves['Housing base'].physicalProperties.mass
             + leaves['Housing cover'].physicalProperties.mass))
    fc.refused(ns, 'the in key bears 101.4 MPa', P=4.5)
    fc.refused(ns, 'bearing 6204 lasts 99941 h', life=150000.0)
    fc.refused(ns, 'the output end has a factor of safety of 2.29', nd=3.0)
    fc.verdict()
