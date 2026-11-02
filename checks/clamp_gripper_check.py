# ClampGripper (Chapter 45) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted; gripping the design pack, no interference and both pads on the pack (their least distance); open, the
# pack clear by the jaws' 50 mm; over the jaws' whole range, without the pack, nothing in the way; the screw's two
# hands found by probing the groove; the adapter's ISO 9409-1 pattern measured; the groups weighed and written for
# the envelope; four refusals.
import json, math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/scripts/ClampGripper/ClampGripper.py'
OUT = '__COMPANION__/lib/clamp_gripper_weighed.json'


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
    mm_ = adsk.core.Application.get().measureManager
    pads = [o for o in leaves if o.component.name.startswith('Pad')]
    j0 = ns['_last']['j_grip']

    def gap_to_pack(pad):
        return min(mm_.measureMinimumDistance(pad.bRepBodies.item(0), c.bRepBodies.item(0)).value * 10 for c in cartons)
    # gripping
    ns['set_state'](G, j0)
    c = fc.interference(leaves)
    fc.ok(not c, 'gripping: no interference among the %d parts and the %d cartons %s' % (len(gripper), len(cartons), c or ''))
    g = [gap_to_pack(p) for p in pads]
    fc.ok(len(g) == 2 and max(g) < 0.001, 'gripping: both pads on the pack, least distances %.4f and %.4f mm' % tuple(g))
    # open for the pick
    ns['set_state'](G, j0 + 50.0)
    c = fc.interference(leaves)
    g = [gap_to_pack(p) for p in pads]
    fc.ok(not c and all(abs(x - 50.0) < 0.001 for x in g), 'open: no interference, the pads %.3f and %.3f mm off the pack'
          % tuple(g))
    # the jaws' whole range, without the pack
    hits = []
    for j in range(200, 651, 25):
        ns['set_state'](G, float(j))
        c = fc.interference(gripper)
        if c:
            hits.append((j, c))
    fc.ok(not hits, 'the jaws from 200 to 650 mm in 25 mm steps: nothing in the way %s' % (hits or ''))
    ns['set_state'](G, j0)
    # the screw's hands: at x = 10 k + 2.5 the two hands' groove angles are half a turn apart; a point 15 mm off the
    # axis at the groove's angle is in the groove (outside the body), at the other hand's angle in the land (inside)
    sc_occ = [o for o in leaves if o.component.name.startswith('Ballscrew')][0]
    halves = {b.name: b for b in sc_occ.bRepBodies}
    P, zs = ns['SC']['lead'], ns['ZS']
    OUT_, IN_ = adsk.fusion.PointContainment.PointOutsidePointContainment, adsk.fusion.PointContainment.PointInsidePointContainment

    def probe(x, th):
        scr = halves['Right-hand half' if x > 0 else 'Left-hand half']
        return scr.pointContainment(adsk.core.Point3D.create(x * 0.1, 15.0 * math.cos(th) * 0.1, (zs + 15.0 * math.sin(th)) * 0.1))
    right = left = True
    xs = [k * 10.0 + 2.5 for k in range(25, 76, 7)]
    for x in xs:
        # a right-hand groove turns +2 pi x / P about +x, a left-hand one -2 pi x / P
        right = right and probe(x, 2 * math.pi * x / P) == OUT_ and probe(x, -2 * math.pi * x / P) == IN_
        left = left and probe(-x, 2 * math.pi * x / P) == OUT_ and probe(-x, -2 * math.pi * x / P) == IN_
    fc.ok(right and left, 'the screw: right-hand groove on +x, left-hand on -x, probed at %d places each side' % len(xs))
    # the adapter's ISO 9409-1-160-6-M10 pattern
    ad = [o for o in leaves if o.component.name.startswith('Adapter plate')][0].bRepBodies.item(0)
    th = holes(ad, 5.5)
    ang = sorted(round(math.degrees(math.atan2(y, x)) % 360.0, 3) for x, y in th)
    pin = holes(ad, 5.0)
    sp = holes(ad, 49.99)
    fc.ok(ang == [30.0, 90.0, 150.0, 210.0, 270.0, 330.0] and all(abs(math.hypot(x, y) - 80.0) < 0.001 for x, y in th)
          and pin == [(80.0, 0.0)] and sp == [(0.0, 0.0)],
          'the adapter: six M10 holes on the 160 circle at %s deg, the 10 mm pin on +Xm, the 100 g6 spigot on the axis' % ang)
    # weighed: each group, and the gripper's centre below its face
    out = {}
    tot, mz, izz = 0.0, 0.0, 0.0
    for name in ('Adapter', 'Frame', 'Screw', 'Jaw A', 'Jaw B'):
        pp = G[name].physicalProperties
        cz = pp.centerOfMass.z * 10
        r = pp.getXYZMomentsOfInertia()                 # kg cm2 about the world's axes: z is the gripper's (J4's)
        out[name] = dict(m=pp.mass, z=cz, izz=r[3] / 1e4)
        tot += pp.mass
        mz += pp.mass * cz
        izz += r[3] / 1e4
    pads_m = sum(o.physicalProperties.mass for o in pads)
    out['total'] = dict(m=tot, z=mz / tot, pads=pads_m, izz=izz)
    print('  weighed: ' + ', '.join('%s %.2f kg at z %.1f' % (k, v['m'], v['z']) for k, v in out.items() if k != 'total'))
    print('  gripper %.2f kg, its centre at z %.1f mm, %.2f kg m2 about its axis (pads %.2f kg)' % (tot, mz / tot, izz, pads_m))
    try:
        open(OUT, 'w').write(json.dumps(out, indent=1))
        print('  written: lib/clamp_gripper_weighed.json')
    except Exception as e:
        print('  not written: %s' % e)
    fc.refused(ns, 'three along', nx=3, ny=2, N=4700.0)
    fc.refused(ns, 'built for', N=5500.0)
    fc.refused(ns, 'needs', N=4500.0)
    fc.refused(ns, 'kPa', nx=3, ny=1, N=4700.0)
    fc.verdict()
