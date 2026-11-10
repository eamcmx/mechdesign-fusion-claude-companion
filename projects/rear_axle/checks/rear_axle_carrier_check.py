# RearAxleCarrier (Chapter 29) against the acceptance criteria of its prompt, measured on what Fusion built: the
# rating recomputed here with formulas of this check's own; the parts counted from Fusion's components; no
# interference at rest, while the pinion turns through one tooth pitch, or with the side gears turned in the case;
# the ratio measured on the bodies; every bearing's cone on its seat and its cup in its bore; four refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/RearAxleCarrier/RearAxleCarrier.py'
EXPECTED = {'Adjuster': 2, 'Bearing A 32309 cone': 1, 'Bearing A 32309 cup': 1, 'Bearing B 32308 cone': 1,
            'Bearing B 32308 cup': 1, 'Bearing C1 32213 cone': 1, 'Bearing C1 32213 cup': 1, 'Bearing C2 32213 cone': 1,
            'Bearing C2 32213 cup': 1, 'Bearing cap': 2, 'Cap bolt M12x90': 4, 'Carrier casting': 1,
            'Companion flange': 1, 'Cross': 1, 'Crown wheel bolt M12x35': 10, 'Crown wheel z47': 1,
            'Differential case': 1, 'Pinion depth shim': 1, 'Pinion nut M30': 1, 'Pinion seal 60x85x10': 1,
            'Pinion shaft z12': 1, 'Side gear': 2, 'Side gear washer': 2, 'Spacer': 1, 'Spider': 4,
            'Spider washer': 4, 'Washer': 1}


def own_numbers():
    """The rating and the pinion's bearings, worked again here from Childs (2014) and Shigley."""
    Tp = 450 * 4.70 * 0.95
    i0 = 47 / 12
    v0, v1 = [n / (4.70 * i0) * 2 * math.pi / 60 * 0.375 for n in (1600.0, 2400.0)]
    n_pk = 5000 * ((v1 ** 2 - v0 ** 2) / (2 * ((Tp * i0 * 0.96 / 0.375 - 0.015 * 3100 * 9.81) / 3100))
                   / (2 * math.pi * 0.375) * i0)
    B = 0.25 * 4 ** 0.667
    A = 50 + 56 * (1 - B)
    v = 5.236e-5 * 78 * 2000 / 4.70
    Kv = ((A + math.sqrt(200 * v)) / A) ** B
    sF = 2000 * Tp / (52 * 78) * Kv / 6.5 * (0.4867 + 0.008399 * 6.5) * (1.1 + 5.6e-6 * 52 ** 2) / 0.23
    SF1 = 240 * 6.1514 * n_pk ** -0.1182 / sF
    Am = 0.5 * 6.5 * math.hypot(12, 47) - 26
    rm1 = 39 * Am / (0.5 * 6.5 * math.hypot(12, 47))
    d1 = math.atan(12 / 47)
    b, al = math.radians(35), math.radians(20)
    Wt = Tp * 1000 / rm1
    Wa = Wt / math.cos(b) * (math.tan(al) * math.sin(d1) + math.sin(b) * math.cos(d1))
    Wr = Wt / math.cos(b) * (math.tan(al) * math.cos(d1) - math.sin(b) * math.sin(d1))
    return dict(SF1=SF1, Wt=Wt, Wa=Wa, Wr=Wr)


def all_joints():
    return [j for comp in fc.design().allComponents for j in comp.asBuiltJoints]


def find_joint(name):
    for j in all_joints():
        if j.name == name:
            return j
    raise RuntimeError('no joint named %r' % name)


def cyl_radii(o):
    out = set()
    for body in o.component.bRepBodies:
        for f in body.faces:
            if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                out.add(round(f.geometry.radius * 10, 3))
    return out


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    o = own_numbers()
    D = ns['design']()
    pk = D['peak']
    f = ns['ad'].bevel_forces(D['g'], D['u']['T1_peak'], +1)
    bad = [k for k, x in (('SF1', pk['SF1']), ('Wt', f['Wt']), ('Wa', f['Wa']), ('Wr', f['Wr'])) if abs(x - o[k]) > 1e-6 * max(1, abs(o[k]))]
    fc.ok(not bad, 'the rating and the forces recomputed independently: S_F %.4f, W_t %.1f, W_a %.1f, W_r %.1f N %s'
          % (o['SF1'], o['Wt'], o['Wa'], o['Wr'], bad or ''))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['Carrier', 'Pinion assembly', 'Differential assembly'], 'three subassemblies: %s' % subs)
    pl = ns['parts_list'](root)
    fc.ok(pl == EXPECTED, 'the parts, counted from Fusion: %d components, %d kinds %s'
          % (sum(pl.values()), len(pl), '' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    mats = {n: leaves[n].bRepBodies.item(0).material.name for n in ('Pinion shaft z12', 'Crown wheel z47',
                                                                     'Carrier casting', 'Differential case')}
    fc.ok(mats == {'Pinion shaft z12': 'Steel', 'Crown wheel z47': 'Steel', 'Carrier casting': 'Iron, Cast',
                   'Differential case': 'Iron, Cast'}, 'materials %s' % mats)
    nj = len(all_joints())
    fc.ok(nj == 8 and root.motionLinks.count == 1,
          '%d As-built Joints (pinion, case, four spiders, two side gears), %d Motion Link' % (nj, root.motionLinks.count))
    c = fc.interference(list(leaves.values()))
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))

    # the bearings on their seats and in their bores, measured on the bodies
    seats = []
    for cone, cup, d, Dd, shaft, house in (('Bearing A 32309 cone', 'Bearing A 32309 cup', 45, 100, 'Pinion shaft z12', 'Carrier casting'),
                                           ('Bearing B 32308 cone', 'Bearing B 32308 cup', 40, 90, 'Pinion shaft z12', 'Carrier casting'),
                                           ('Bearing C1 32213 cone', 'Bearing C1 32213 cup', 65, 120, 'Differential case', 'Carrier casting'),
                                           ('Bearing C2 32213 cone', 'Bearing C2 32213 cup', 65, 120, 'Differential case', 'Carrier casting')):
        good = (d / 2.0 in cyl_radii(leaves[cone]) and d / 2.0 in cyl_radii(leaves[shaft])
                and Dd / 2.0 in cyl_radii(leaves[cup]) and round(Dd / 2.0 + 0.05, 3) in cyl_radii(leaves[house]))
        seats.append(good)
        fc.ok(good, '%s on a %d mm seat, its cup %d mm in a bore of %.1f' % (cone[:-5], d, Dd, Dd + 0.1))

    # the ratio, measured on the bodies
    jp = fc.joint('Pinion turns')
    pin = root.occurrences.item(1)
    dif = root.occurrences.item(2)
    R0p, R0c = fc.rotation(pin), fc.rotation(dif)
    jp.jointMotion.rotationValue = math.radians(30.0)
    adsk.doEvents()
    tp = fc.turn_deg(fc.rotation(pin), R0p, (1, 0, 0))
    tc = fc.turn_deg(fc.rotation(dif), R0c, (0, 1, 0))
    fc.ok(abs(tc - tp * 12 / 47) < 1e-6, 'the pinion turned %.3f deg about +X: the case %.4f deg about +Y '
          '(want %.4f, the same way, 12/47 as far)' % (tp, tc, tp * 12 / 47))
    # the mesh through one tooth pitch of the pinion, 30 deg, in steps of 5
    hits = []
    for k in range(7):
        jp.jointMotion.rotationValue = math.radians(5.0 * k)
        adsk.doEvents()
        h = fc.interference([leaves['Pinion shaft z12'], leaves['Crown wheel z47']])
        if h:
            hits.append((5 * k, h))
    jp.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.ok(not hits, 'no tooth collision while the pinion turns 0 to 30 deg, one pitch %s' % (hits or ''))
    # the pinion's parts all turn with it
    kids = list(pin.childOccurrences)
    c0 = {x.component.name: x.physicalProperties.centerOfMass for x in kids}
    jp.jointMotion.rotationValue = math.radians(90.0)
    adsk.doEvents()
    off = []
    for x in kids:
        p0, p1 = c0[x.component.name], x.physicalProperties.centerOfMass
        want_a = (p0.y * math.cos(math.radians(tp * 3)) - p0.z * math.sin(math.radians(tp * 3)),
                  p0.y * math.sin(math.radians(tp * 3)) + p0.z * math.cos(math.radians(tp * 3)))
        off.append((x.component.name, 10 * math.hypot(p1.x - p0.x, math.hypot(p1.y - want_a[0], p1.z - want_a[1]))))
    jp.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    worst = max(off, key=lambda t: t[1])
    fc.ok(worst[1] < 1e-4, 'all %d parts of the pinion assembly turned with it (worst %s, %.1e mm)' % (len(kids), worst[0], worst[1]))

    # the differential: the left side gear turned theta in the case, the right -theta, each spider -(22/15) theta
    # about its arm, outwards; each joint's sense found by turning it and measuring
    def set_turn(jname, occ_px, axis, ang):
        j = find_joint(jname)
        R0 = fc.rotation(occ_px)
        j.jointMotion.rotationValue = math.radians(1.0)
        adsk.doEvents()
        s = 1 if fc.turn_deg(fc.rotation(occ_px), R0, axis) > 0 else -1
        j.jointMotion.rotationValue = math.radians(s * ang)
        adsk.doEvents()
    movers = {x.component.name: x for x in dif.childOccurrences}
    gear_hits = []
    for th in (4.0, 8.181818, 12.0):
        set_turn('Side gear L turns in the case', movers['Side gear L'], (0, 1, 0), th)
        set_turn('Side gear R turns in the case', movers['Side gear R'], (0, 1, 0), -th)
        for i, p in enumerate((0.0, 90.0, 180.0, 270.0)):
            ax = (math.sin(math.radians(p)), 0, math.cos(math.radians(p)))
            set_turn('Spider %d turns in the case' % (i + 1), movers['Spider %d' % (i + 1)], ax, -22.0 / 15.0 * th)
        h = fc.interference([movers[n] for n in movers if n.startswith(('Spider ', 'Side gear L', 'Side gear R'))
                             and 'washer' not in n])
        if h:
            gear_hits.append((th, h))
    fc.ok(not gear_hits, 'no tooth collision in the differential with the side gears turned 4, 8.18 (half a pitch) and '
          '12 deg in the case and the spiders 22/15 as far %s' % (gear_hits or ''))
    bb = leaves['Carrier casting'].bRepBodies.item(0).boundingBox
    fc.ok(abs(bb.minPoint.z * 10 + 166.0) < 0.01, 'the plate stops %.2f mm below the axle, above the bowl\'s bottom'
          % (-bb.minPoint.z * 10))
    mass = sum(x.physicalProperties.mass for x in leaves.values())
    print('  mass of the carrier assembly %.2f kg; the crown wheel %.2f kg, the casting %.2f kg'
          % (mass, leaves['Crown wheel z47'].physicalProperties.mass, leaves['Carrier casting'].physicalProperties.mass))
    fc.refused(ns, 'bending factor of 0.818', m=5.9, F=47.0)
    fc.refused(ns, 'ground clearance', m=6.75, F=54.0)
    fc.refused(ns, 'third of the cone distance', F=56.0)
    fc.refused(ns, '2 z_s / N = 14.67', N=3)
    fc.verdict()
