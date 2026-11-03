# AxleDetail (Chapter 54) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the cups' bores closed and the census's seats touching; the nose's middle bore stepped; the spacer from cone to cone;
# the flange's hub, the case's shoulders and the adjusters inside SKF's bands; the splines' minor diameter; the faces'
# classes; no interference at rest, driving straight and in a turn; the refusal.
import sys, importlib, math
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import axle_detail as XD
importlib.reload(XD)
import adsk.core

PATH = '__COMPANION__/scripts/AxleDetail/AxleDetail.py'


def radii(o):
    return sorted({round(f.geometry.radius * 10, 3) for b in o.bRepBodies for f in b.faces
                   if isinstance(f.geometry, adsk.core.Cylinder)})


def xspan(o):
    b = o.boundingBox
    return round(b.minPoint.x * 10, 3), round(b.maxPoint.x * 10, 3)


def attr(f):
    a = f.attributes.itemByName('mdfc', 'fit')
    return a.value if a else None


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    D = ns['DIA']
    L = XD.carrier_layout()
    cast = leaves['Carrier casting']
    rc = radii(cast)
    fc.ok(50.0 in rc and 45.0 in rc and 60.0 in rc and not {50.05, 45.05, 60.05} & set(rc),
          'the cups\' bores 100, 90 and 120 across, none 0.1 over: the casting\'s radii %s'
          % [r for r in rc if 40 <= r <= 61])
    caps = [v for k, v in leaves.items() if k.startswith('Bearing cap')]
    fc.ok(len(caps) == 2 and all(60.0 in radii(c) and 60.05 not in radii(c) for c in caps), 'the caps\' bores 120')
    mf = ns['_last']['mf']
    cups = [x for x in mf['seats'] if 'cup' in x['bought'] and x['face'] == 'outside']
    pairs = sorted({(x['bought'], x['made_name']) for x in cups})
    fc.ok(len(pairs) == 6 and all(x['gap'] < 1e-6 and x.get('cls') == 'H7' for x in cups),
          'the census\'s cup seats touching, H7, on six pairs of cup and bore (%d records, the threads\' faces beside '
          'the cups among them): %s' % (len(cups), pairs))
    fc.ok(D['middle_A'] / 2 in rc and D['middle_B'] / 2 in rc and 46.5 not in rc,
          'the nose\'s middle bore %.0f at cup A and %.0f at cup B, none 93' % (D['middle_A'], D['middle_B']))
    sp, ca, cb = leaves['Spacer'], leaves['Bearing A 32309 cone'], leaves['Bearing B 32308 cone']
    s0, s1 = xspan(sp)
    a0, a1 = xspan(ca)
    b0, b1 = xspan(cb)
    fc.ok(abs(s0 - a1) < 0.05 and abs(s1 - b0) < 0.05 and abs((s1 - s0) - 67.8) < 0.05,
          'the spacer from %.3f to %.3f mm, %.2f long, on cone A\'s face at %.3f and cone B\'s at %.3f' % (s0, s1, s1 - s0, a1, b0))
    fl = radii(leaves['Companion flange'])
    case = radii(leaves['Differential case'])
    adj = [radii(v) for k, v in leaves.items() if k.startswith('Adjuster')]
    fc.ok(D['hub_relief'] / 2 in fl and D['case_shoulder'] / 2 in case
          and len(adj) == 2 and all(D['adjuster_relief'] / 2 in r and 36.0 in r for r in adj),
          'the flange\'s hub %.0f at cone B, the case\'s ends relieved to %.0f, the adjusters %.0f at the cups and 72 beyond'
          % (D['hub_relief'], D['case_shoulder'], D['adjuster_relief']))
    hs = [radii(v) for k, v in leaves.items() if k.startswith('Half shaft')]
    sg = [radii(v) for k, v in leaves.items() if k.startswith('Side gear') and 'washer' not in k]
    mins = sorted({x for r in hs + sg for x in r if 17.0 < x < 19.0})
    fc.ok(len(hs) == 2 and len(sg) == 2 and mins == [18.01, 18.1],
          'the half shafts\' and the side gears\' splines 36 across their minor diameter (drawn %s, the shafts 0.01 '
          'and the hubs 0.1 over, as Chapters 29 and 30 draw them)' % mins)
    # the faces given by hand
    ch, new = ns['_last']['changed'], ns['_last']['new']
    vals = {}
    for f in ch:
        vals[attr(f)] = vals.get(attr(f), 0) + 1
    fc.ok(vals.get('M120 x 2 6g', 0) == 2 and vals.get('M120 x 2 6H', 0) == 4,
          'the adjusters\' threads 6g on 2 faces and 6H on 4 in the casting and the caps: %s'
          % {k: v for k, v in vals.items() if k and k.startswith('M120')})
    sp_vals = sorted(k for k in vals if k and not k.startswith('M120'))
    fc.ok(set(sp_vals) == {'H7 Ra 1.6', 'H10 Ra 3.2', 'H9 Ra 1.6', 'f7 Ra 1.6', 'a11 Ra 3.2', 'd10 Ra 1.6',
                           'h7 Ra 1.6', 'h10 Ra 1.6'},
          'the splines\' classes, ISO 14: %s' % ', '.join('%s %d' % (k, vals[k]) for k in sp_vals))
    carried = sum(1 for o in leaves.values() for b in o.bRepBodies for f in b.faces if attr(f))
    fc.ok(carried == sum(mf['coloured'].values()) + len(new),
          'every other decided face as Chapter 51 decides it: %d faces carry a class, %d by the rules and %d more by '
          'hand' % (carried, sum(mf['coloured'].values()), len(new)))
    # no interference at rest, driving straight and in a turn
    c = fc.interference(list(leaves.values()))
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))
    joints = {j.name: j for comp in fc.design().allComponents for j in comp.asBuiltJoints}
    jp, j1 = joints['Pinion turns'], joints['Spider 1 turns in the case']
    jp.jointMotion.rotationValue = math.radians(30.0)
    adsk.doEvents()
    hit = fc.interference(list(leaves.values()))
    jp.jointMotion.rotationValue = math.radians(47.0)
    j1.jointMotion.rotationValue = math.radians(2.5)
    adsk.doEvents()
    hit2 = fc.interference(list(leaves.values()))
    j1.jointMotion.rotationValue = 0.0
    jp.jointMotion.rotationValue = 0.0
    adsk.doEvents()
    fc.ok(not hit and not hit2, 'no interference driving straight nor in a turn %s %s' % (hit or '', hit2 or ''))
    fc.refused(ns, 'falls short', pitch=1.0)
    fc.verdict()
