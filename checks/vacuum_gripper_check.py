# VacuumGripper (Chapter 46) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted; on the layer, no interference and every cup's lip on its carton; every zone pushed up the compensators'
# whole stroke with its cartons, nothing in the way but the springs, which compress; the springs measured; the
# adapter's ISO 9409-1 pattern; the groups weighed and written for the envelope; three refusals.
import json, math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/scripts/VacuumGripper/VacuumGripper.py'
OUT = '__COMPANION__/lib/vacuum_gripper_weighed.json'


def holes(body, r_mm, tol=0.01):
    out = []
    for f in body.faces:
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10 - r_mm) < tol:
            p = (round(g.origin.x * 10, 3), round(g.origin.y * 10, 3))
            if p not in out:
                out.append(p)
    return out


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    G = ns['_last']['groups']
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds' % (sum(pl.values()), len(pl)))
    print('  ' + ', '.join('%s %d' % (k, v) for k, v in sorted(pl.items())))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    cartons = [o for o in leaves if o.fullPathName.startswith('Pack')]
    gripper = [o for o in leaves if not o.fullPathName.startswith('Pack')]
    cups = [o for o in leaves if o.component.name.startswith('Cup SPB1')]
    springs = [o for o in leaves if o.component.name.startswith('Spring')]
    mm_ = adsk.core.Application.get().measureManager
    ns['set_state'](G, 0.0)
    c = fc.interference(leaves)
    fc.ok(not c, 'on the layer: no interference among the %d parts and the %d cartons %s' % (len(gripper), len(cartons), c or ''))
    gaps = [min(mm_.measureMinimumDistance(u.bRepBodies.item(0), k.bRepBodies.item(0)).value * 10 for k in cartons)
            for u in cups]
    fc.ok(len(gaps) == 36 and max(gaps) < 0.001, 'every one of the %d cups\' lips on its carton, the largest gap %.4f mm'
          % (len(gaps), max(gaps)))
    st = ns['L']['stroke']
    ns['set_state'](G, st)
    rest = [o for o in leaves if not o.component.name.startswith('Spring')]
    c = fc.interference(rest)
    fc.ok(not c, 'every zone and its cartons pushed up the whole %.0f mm stroke: nothing in the way but the springs %s'
          % (st, c or ''))
    gaps = [min(mm_.measureMinimumDistance(u.bRepBodies.item(0), k.bRepBodies.item(0)).value * 10 for k in cartons)
            for u in cups]
    fc.ok(max(gaps) < 0.001, 'pushed up, the lips still on the cartons, the largest gap %.4f mm' % max(gaps))
    ns['set_state'](G, 0.0)
    b = springs[0].bRepBodies.item(0).boundingBox
    h = (b.maxPoint.z - b.minPoint.z) * 10
    od = (b.maxPoint.x - b.minPoint.x) * 10
    s = ns['vg'].spring()
    fc.ok(len(springs) == 36 and abs(h - s['L1']) < 0.05 and abs(od - s['OD']) < 0.2,
          'the 36 springs: %.2f mm long installed, %.2f mm across, against %.1f and %.1f' % (h, od, s['L1'], s['OD']))
    m6 = [o for o in leaves if o.component.name.startswith('Cap screw M6x14')]
    zz = [(o.bRepBodies.item(0).boundingBox.minPoint.z * 10, o.bRepBodies.item(0).boundingBox.maxPoint.z * 10) for o in m6]
    plate = [o for o in leaves if o.component.name == 'Plate'][0].bRepBodies.item(0)
    beams = [o.bRepBodies.item(0) for o in leaves if o.component.name.startswith('Beam')]
    touch = [min(mm_.measureMinimumDistance(o.bRepBodies.item(0), bb).value * 10 for bb in beams) for o in m6]
    eng = [-116.0 - z0 for z0, _ in zz]
    fc.ok(len(m6) == 16 and all(abs(e - 9.0) < 0.01 for e in eng) and max(touch) < 0.001
          and all(mm_.measureMinimumDistance(o.bRepBodies.item(0), plate).value * 10 < 0.001 for o in m6),
          'the plate held by %d M6 x 14, their heads on the beams\' bottom walls, each %.2f mm into the plate'
          % (len(m6), min(eng)))
    ad = [o for o in leaves if o.component.name.startswith('Adapter plate')][0].bRepBodies.item(0)
    th = holes(ad, 5.5)
    ang = sorted(round(math.degrees(math.atan2(y, x)) % 360.0, 3) for x, y in th)
    fc.ok(ang == [30.0, 90.0, 150.0, 210.0, 270.0, 330.0] and all(abs(math.hypot(x, y) - 80.0) < 0.001 for x, y in th)
          and holes(ad, 5.0) == [(80.0, 0.0)] and holes(ad, 49.99) == [(0.0, 0.0)],
          'the adapter: six M10 holes on the 160 circle at %s deg, the pin on +Xm, the spigot on the axis' % ang)
    out = {}
    tot, mz, izz = 0.0, 0.0, 0.0
    for name in ['Adapter', 'Frame'] + ['Zone %d' % k for k in range(1, 7)]:
        pp = G[name].physicalProperties
        r = pp.getXYZMomentsOfInertia()
        out[name] = dict(m=pp.mass, z=pp.centerOfMass.z * 10, izz=r[3] / 1e4)
        tot += pp.mass
        mz += pp.mass * pp.centerOfMass.z * 10
        izz += r[3] / 1e4
    cups_m = sum(o.physicalProperties.mass for o in cups)
    out['total'] = dict(m=tot, z=mz / tot, izz=izz, cups=cups_m)
    print('  weighed: adapter %.2f kg, frame %.2f kg, a zone %.2f kg; the gripper %.2f kg, its centre at z %.1f mm, '
          '%.2f kg m2 about its axis (cups %.2f kg)' % (out['Adapter']['m'], out['Frame']['m'], out['Zone 1']['m'], tot,
                                                       mz / tot, izz, cups_m))
    try:
        open(OUT, 'w').write(json.dumps(out, indent=1))
        print('  written: lib/vacuum_gripper_weighed.json')
    except Exception as e:
        print('  not written: %s' % e)
    fc.refused(ns, 'six zones', n=7)
    fc.refused(ns, 'used', n=1, m=7.5)
    fc.refused(ns, 'worked at', p_bar=0.5)
    fc.verdict()
