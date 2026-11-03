# ComplianceUnit (Chapter 48) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted; at rest no interference and the carriage on the hangers' heads; the springs measured; raised by the
# sensing distance, the sensor 2 mm off the wing's face; raised the whole stroke, nothing in the way but the springs,
# the wings on the top plate and the blocks still on their rails; the ISO pattern; weighed and written; refusals.
import json, math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/scripts/ComplianceUnit/ComplianceUnit.py'
OUT = '__COMPANION__/lib/compliance_unit_weighed.json'


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
    L = ns['L']
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds' % (sum(pl.values()), len(pl)))
    print('  ' + ', '.join('%s %d' % (k, v) for k, v in sorted(pl.items())))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    mm_ = adsk.core.Application.get().measureManager
    dist = lambda a, b: mm_.measureMinimumDistance(a.bRepBodies.item(0), b.bRepBodies.item(0)).value * 10
    one = lambda nm: [o for o in leaves if o.component.name == nm][0]
    car = one('Carriage')
    hangers = [o for o in leaves if o.component.name.startswith('Hanger bolt')]
    springs = [o for o in leaves if o.component.name.startswith('Spring ') and not o.component.name.startswith('Spring seat')]
    ns['set_state'](G, 0.0)
    c = fc.interference(leaves)
    fc.ok(not c, 'at rest: no interference among the %d parts %s' % (len(leaves), c or ''))
    fc.ok(len(hangers) == 4 and max(dist(car, h) for h in hangers) < 0.001,
          'the carriage on the four hangers\' heads, the largest gap %.4f mm' % max(dist(car, h) for h in hangers))
    b = springs[0].bRepBodies.item(0).boundingBox
    h = (b.maxPoint.z - b.minPoint.z) * 10
    od = (b.maxPoint.x - b.minPoint.x) * 10
    sp = ns['_last']['coil']
    fc.ok(len(springs) == 4 and abs(h - sp['L1']) < 0.05 and abs(od - (sp['D'] + sp['d'])) < 0.2,
          'the 4 springs: %.2f mm long installed, %.2f mm across, against %.1f and %.1f' % (h, od, sp['L1'], sp['D'] + sp['d']))
    sensor = one('Sensor M12')
    d0 = dist(sensor, car)
    ns['set_state'](G, L['sense'])
    d1 = dist(sensor, car)
    fc.ok(abs(d0 - math.hypot(2.0, L['sense'])) < 0.01 and abs(d1 - 2.0) < 0.01,
          'the sensor %.3f mm from the carriage at rest and %.3f mm once it has risen %.0f mm' % (d0, d1, L['sense']))
    ns['set_state'](G, L['stroke'])
    rest = [o for o in leaves if o not in springs]
    c = fc.interference(rest)
    top = one('Top plate')
    blocks = [o for o in leaves if o.component.name.startswith('Block')]
    rails = [o for o in leaves if o.component.name.startswith('Rail')]
    on = all(bl.bRepBodies.item(0).boundingBox.maxPoint.z <= max(r.bRepBodies.item(0).boundingBox.maxPoint.z for r in rails) + 1e-6
             for bl in blocks)
    fc.ok(not c and dist(car, top) < 0.001 and on and len(blocks) == 4,
          'raised the whole %.0f mm: nothing in the way but the springs %s; the wings on the top plate (%.4f mm); the '
          'four blocks still on their rails' % (L['stroke'], c or '', dist(car, top)))
    ns['set_state'](G, 0.0)
    th = holes(top.bRepBodies.item(0), 5.5)
    ang = sorted(round(math.degrees(math.atan2(y, x)) % 360.0, 3) for x, y in th)
    fc.ok(ang == [30.0, 90.0, 150.0, 210.0, 270.0, 330.0] and all(abs(math.hypot(x, y) - 80.0) < 0.001 for x, y in th),
          'the top plate: six M10 holes on the 160 circle at %s deg' % ang)
    out = {}
    for name in ('Body', 'Carriage'):
        pp = G[name].physicalProperties
        out[name] = dict(m=pp.mass, z=pp.centerOfMass.z * 10)
    out['total'] = dict(m=out['Body']['m'] + out['Carriage']['m'])
    print('  weighed: the body %.2f kg, its centre at z %.1f mm; the carriage %.2f kg, its centre at z %.1f mm; the unit %.2f kg'
          % (out['Body']['m'], out['Body']['z'], out['Carriage']['m'], out['Carriage']['z'], out['total']['m']))
    try:
        open(OUT, 'w').write(json.dumps(out, indent=1))
        print('  written: lib/compliance_unit_weighed.json')
    except Exception as e:
        print('  not written: %s' % e)
    fc.refused(ns, 'ring would open', gripper='clamp')
    fc.refused(ns, 'rails give', gripper='bag', stroke=40.0)
    fc.refused(ns, 'preload', gripper='bag', F_pre=500.0)
    fc.verdict()
