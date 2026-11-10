# BagGripper (Chapter 47) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted; closed on the bag, no interference and every tine's foot and the press plate on the bag; the jaws opened
# in steps to 45 deg with the press up, nothing in the way, the tines between the station's rollers; the cylinders'
# pin-to-pin lengths; the adapter's ISO 9409-1 pattern; the groups weighed, the jaw's inertia about its hinge, all
# written for the chapter; three refusals.
import json, math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/projects/palletizer/BagGripper/BagGripper.py'
OUT = '__COMPANION__/projects/palletizer/lib/bag_gripper_weighed.json'


def holes(body, r_mm, tol=0.01):
    out = []
    for f in body.faces:
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10 - r_mm) < tol:
            p = (round(g.origin.x * 10, 3), round(g.origin.y * 10, 3), round(g.origin.z * 10, 3))
            if p not in out:
                out.append(p)
    return out


def axis_point(body, r_mm, x_mm):
    """The (y, z) where a cylindrical face of radius r_mm crosses the plane x = x_mm (its axis along x)."""
    for f in body.faces:
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(g.radius * 10 - r_mm) < 0.01 and abs(abs(g.axis.x) - 1) < 1e-6:
            t = (x_mm / 10 - g.origin.x) / g.axis.x
            return (g.origin.y * 10 + t * g.axis.y * 10, g.origin.z * 10 + t * g.axis.z * 10)
    return None


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    G = ns['_last']['groups']
    L = ns['L']
    bg = ns['bg']
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds' % (sum(pl.values()), len(pl)))
    print('  ' + ', '.join('%s %d' % (k, v) for k, v in sorted(pl.items())))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    bag = [o for o in leaves if o.fullPathName.startswith('Bag')][0]
    rollers = [o for o in leaves if o.fullPathName.startswith('Station')]
    tines = [o for o in leaves if o.component.name.startswith('Tine')]
    mm_ = adsk.core.Application.get().measureManager
    dist = lambda a, b: mm_.measureMinimumDistance(a.bRepBodies.item(0), b.bRepBodies.item(0)).value * 10
    ns['set_state'](G, 0.0)
    c = fc.interference(leaves)
    fc.ok(not c, 'closed on the bag: no interference among the %d parts, the bag and the %d rollers %s'
          % (len(leaves) - 1 - len(rollers), len(rollers), c or ''))
    feet = [dist(t, bag) for t in tines]
    plate = [o for o in leaves if o.component.name == 'Press plate'][0]
    walls = [o for o in leaves if o.component.name.startswith('End plate')]
    fc.ok(len(feet) == 16 and max(feet) < 0.001 and dist(plate, bag) < 0.001,
          'every one of the %d tines and the press plate on the bag, the largest gap %.4f mm' % (len(feet), max(feet)))
    fc.ok(all(abs(dist(w, bag) - 10.0) < 0.01 for w in walls), 'the end plates %.2f and %.2f mm off the bag\'s ends'
          % tuple(dist(w, bag) for w in walls))
    gaps = []
    for ph in range(5, int(L['open']) + 1, 5):
        s = ns['set_state'](G, float(ph), e=0.0)
        c = fc.interference(leaves)
        d_bag = min(dist(t, bag) for t in tines)
        d_rol = min(dist(t, r) for t in tines for r in rollers)
        low = min(t.bRepBodies.item(0).boundingBox.minPoint.z * 10 for t in tines)
        gaps.append((ph, round(d_bag, 2), round(d_rol, 2), round(low, 1), round(s['L'], 2)))
        if c:
            fc.ok(False, 'opened %d deg: interference %s' % (ph, c))
    fc.ok(all(g[1] > 0.0 and g[2] > 0.0 for g in gaps),
          'opened 5 to %d deg in steps of 5, the press up: nothing in the way; the tines\' least distances to the bag '
          'and to the rollers, their lowest point and the cylinder\'s pin to pin: %s' % (L['open'], gaps))
    cg = bg.cylinder_geometry()
    fc.ok(abs(gaps[-1][4] - cg['L_open']) < 0.01, 'open: the cylinders %.2f mm pin to pin, against %.2f; %.2f of the '
          'stroke used' % (gaps[-1][4], cg['L_open'], cg['L_ext'] - gaps[-1][4]))
    for ph in (0.0, L['open']):
        ns['set_state'](G, ph, e=0.0 if ph else None)
        for nm, s in (('A', 1), ('B', -1)):
            rod_end = [o for o in leaves if o.component.name == 'Rod end %s' % nm][0].bRepBodies.item(0)
            brk = [o for o in leaves if o.component.name == 'Bracket %d' % (1 if s > 0 else 2)][0].bRepBodies.item(0)
            xc = s * L['cyl_x']
            p1 = axis_point(rod_end, 8.0, xc)
            p0 = axis_point(brk, 8.0, xc)
            Lm = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            want = cg['L_ext'] if ph == 0.0 else cg['L_open']
            fc.ok(abs(Lm - want) < 0.01, 'cylinder %s at %.0f deg: %.2f mm between its pins, against %.2f' % (nm, ph, Lm, want))
    ns['set_state'](G, 0.0)
    ad = [o for o in leaves if o.component.name.startswith('Adapter plate')][0].bRepBodies.item(0)
    th = [(x, y) for x, y, z in holes(ad, 5.5)]
    ang = sorted(round(math.degrees(math.atan2(y, x)) % 360.0, 3) for x, y in th)
    fc.ok(ang == [30.0, 90.0, 150.0, 210.0, 270.0, 330.0] and all(abs(math.hypot(x, y) - 80.0) < 0.001 for x, y in th),
          'the adapter: six M10 holes on the 160 circle at %s deg' % ang)
    out = {}
    tot, mz, izz = 0.0, 0.0, 0.0
    for name in ('Adapter', 'Frame', 'Jaw A', 'Jaw B', 'Cylinder A', 'Cylinder B', 'Rod A', 'Rod B', 'Press'):
        pp = G[name].physicalProperties
        r = pp.getXYZMomentsOfInertia()
        c_ = pp.centerOfMass
        out[name] = dict(m=pp.mass, x=c_.x * 10, y=c_.y * 10, z=c_.z * 10, ixx=r[1] / 1e4, izz=r[3] / 1e4)
        tot += pp.mass
        mz += pp.mass * c_.z * 10
        izz += r[3] / 1e4
    ja = out['Jaw A']
    yh, zh = L['hinge']
    I_hinge = ja['ixx'] - ja['m'] * ((ja['y'] / 1000) ** 2 + (ja['z'] / 1000) ** 2) + ja['m'] * (((ja['y'] - yh) / 1000) ** 2 + ((ja['z'] - zh) / 1000) ** 2)
    out['total'] = dict(m=tot, z=mz / tot, izz=izz)
    out['jaw_I'] = I_hinge
    print('  weighed: adapter %.2f kg, frame %.2f kg, each jaw %.2f kg, each cylinder %.2f + rod %.2f kg, press %.2f kg; '
          'the gripper %.2f kg, its centre at z %.1f mm, %.2f kg m2 about its axis; a jaw %.4f kg m2 about its hinge'
          % (out['Adapter']['m'], out['Frame']['m'], ja['m'], out['Cylinder A']['m'], out['Rod A']['m'],
             out['Press']['m'], tot, mz / tot, izz, I_hinge))
    try:
        open(OUT, 'w').write(json.dumps(out, indent=1))
        print('  written: lib/bag_gripper_weighed.json')
    except Exception as e:
        print('  not written: %s' % e)
    fc.refused(ns, 'worst stop', m=80.0)
    fc.refused(ns, 'mm wide', w=600.0)
    fc.refused(ns, '6 bar', p_bar=5.0)
    fc.verdict()
