# AxleHousing (Chapter 30) against the acceptance criteria of its prompt, measured on what Fusion built: the wheel
# end's sizing recomputed here; the parts counted from Fusion's components; no interference at rest or with the half
# shafts turned; each wheel bearing's inner ring on the shaft's seat and its outer ring in the tube's bore; each
# wheel end able to go together (the bearing and its retaining ring pressed on from the shaft's inner end); the
# housing's lowest point; three refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/rear_axle/AxleHousing/AxleHousing.py'
EXPECTED = {'Banjo housing': 1, 'Bearing 6213 balls': 2, 'Bearing 6213 inner ring': 2, 'Bearing 6213 outer ring': 2,
            'Breather': 1, 'Carrier nut M12': 8, 'Carrier stud M12': 8, 'Cover screw M8x20': 10, 'Drain plug M22': 1,
            'Filler plug M22': 1, 'Half shaft': 2, 'Oil seal 74x100x10': 2, 'Rear cover': 1, 'Retainer plate': 2,
            'Retaining ring': 2, 'Wheel stud M14': 12}


def own_numbers():
    """The outer wheel in the hardest turn and the pothole, worked again here."""
    W_r = 1850 * 9.81
    Ws = W_r / 2
    dW = W_r * 0.8 * 0.8 / 1.56
    V, S = Ws + dW, 0.8 * (Ws + dW)
    M = abs(65 * V - 365 * S)
    T = 450 * 4.70 * 0.95 * 47 / 12 * 0.96 / 2 * 1000
    d = 48.0
    vm = math.sqrt((32 * M / (math.pi * d ** 3)) ** 2 + 3 * (16 * T / (math.pi * d ** 3)) ** 2)
    Rb_pot = 3 * Ws * (1 + 65 / 615)
    sig_tube = Rb_pot * 140 / (math.pi * (80 ** 4 - 64 ** 4) / (32 * 80))
    return dict(n_yield=1140 / vm, sigma=sig_tube)


def cyl_radii(o):
    out = set()
    for body in o.component.bRepBodies:
        for f in body.faces:
            if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                out.add(round(f.geometry.radius * 10, 3))
    return out


def span(entity, s):
    """An entity's extent along the axle, outwards from the middle, in mm."""
    bb = entity.boundingBox
    a, b = s * bb.minPoint.y * 10, s * bb.maxPoint.y * 10
    return min(a, b), max(a, b)


def assembly_order(leaves, tag, s):
    """The bearing's inner ring and the retaining ring are pressed onto the half shaft from its inner end, the flange
    being part of the shaft: everything on the shaft inboard of the bearing's outboard face must pass through their
    bores, and a shoulder of at least d_S/2 must begin at that face for the bearing to be pressed against."""
    ring_in, ring_out = span(leaves['Bearing 6213 inner ring %s' % tag].bRepBodies.item(0), s)
    keep_in, keep_out = span(leaves['Retaining ring %s' % tag].bRepBodies.item(0), s)
    inboard, shoulder = 0.0, 0.0
    for f in leaves['Half shaft %s' % tag].bRepBodies.item(0).faces:
        if f.geometry.surfaceType != adsk.core.SurfaceTypes.CylinderSurfaceType:
            continue
        r = f.geometry.radius * 10
        lo, hi = span(f, s)
        if hi <= ring_out + 0.01:
            inboard = max(inboard, r)
        elif abs(lo - ring_out) < 0.05:
            shoulder = max(shoulder, r)
    return dict(inboard=inboard, shoulder=shoulder, ring_inboard=keep_out <= ring_in + 0.01)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    o = own_numbers()
    Dz = ns['design']()
    bad = [k for k, x in (('n_yield', Dz['w']['n_yield']), ('sigma', Dz['h']['sigma'])) if abs(x - o[k]) > 1e-6 * abs(o[k])]
    fc.ok(not bad, 'the shaft in the hardest turn and the tube in the pothole recomputed: %.4f, %.3f MPa %s'
          % (o['n_yield'], o['sigma'], bad or ''))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = [x.component.name for x in root.occurrences]
    fc.ok(subs == ['Housing', 'Wheel end L', 'Wheel end R'], 'three subassemblies: %s' % subs)
    pl = ns['parts_list'](root)
    fc.ok(pl == EXPECTED, 'the parts, counted from Fusion: %d components, %d kinds %s'
          % (sum(pl.values()), len(pl), '' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = {x.component.name: x for x in root.allOccurrences if x.component.bRepBodies.count}
    c = fc.interference(list(leaves.values()))
    fc.ok(not c, 'no interference among the %d parts at rest %s' % (len(leaves), c or ''))
    for tag in ('L', 'R'):
        good = (32.5 in cyl_radii(leaves['Bearing 6213 inner ring %s' % tag]) and 32.5 in cyl_radii(leaves['Half shaft %s' % tag])
                and 60.0 in cyl_radii(leaves['Bearing 6213 outer ring %s' % tag]) and 60.0 in cyl_radii(leaves['Banjo housing']))
        fc.ok(good, 'wheel bearing %s: inner ring on the 65 mm seat, outer ring in the 120 mm bore' % tag)
    for tag, s in (('L', 1), ('R', -1)):
        fit = assembly_order(leaves, tag, s)
        fc.ok(fit['inboard'] <= 32.5 + 1e-3 and fit['ring_inboard'] and fit['shoulder'] >= 37.0 - 1e-3,
              'wheel end %s goes together: the shaft inboard of the bearing at most %.2f mm in radius against the 32.5 mm '
              'bore, the retaining ring inboard of the bearing %s, a shoulder of %.2f mm radius at its outboard face'
              % (tag, fit['inboard'], fit['ring_inboard'], fit['shoulder']))
    bb = leaves['Banjo housing'].bRepBodies.item(0).boundingBox
    low = -bb.minPoint.z * 10
    fc.ok(abs(low - 168.07) < 0.01, 'the housing reaches %.2f mm below the axle: %.2f mm above the road' % (low, 365 - low))
    hits = []
    for tag in ('L', 'R'):
        j = fc.joint('Half shaft %s turns' % tag)
        for a in (30.0, 75.0):
            j.jointMotion.rotationValue = math.radians(a)
            adsk.doEvents()
            h = fc.interference(list(leaves.values()))
            if h:
                hits.append((tag, a, h))
        j.jointMotion.rotationValue = 0.0
        adsk.doEvents()
    fc.ok(not hits, 'no interference with each half shaft turned 30 and 75 deg %s' % (hits or ''))
    mass = sum(x.physicalProperties.mass for x in leaves.values())
    print('  mass %.2f kg; the banjo %.2f kg, a half shaft %.2f kg' % (mass, leaves['Banjo housing'].physicalProperties.mass,
                                                                       leaves['Half shaft L'].physicalProperties.mass))
    fc.refused(ns, 'against yield', d_body=38.0)
    fc.refused(ns, 'million turns', d_seat=60.0)
    fc.refused(ns, 'in the pothole', D=70.0, t=5.0)
    fc.verdict()
