# WristIntegration (Chapter 40) against the acceptance criteria of its prompt, measured on what Fusion built: the
# brakes' stopping angle worked again here; the parts counted; the links still exact; no interference at rest, at
# Chapter 39's eight poses and over the pitch range to 127 deg either way with the roll turned; at 129 deg the stop
# disc's lug inside a buffer pad and nothing else touching; the flange's interface measured; the masses and the
# wrist's centre of mass; two refusals.
import math, sys, json, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/projects/robotic_wrist/WristIntegration/WristIntegration.py'
EXPECTED = {'Base plate': 1, 'Bearing 6002-2RSH': 4, 'Bearing 61902-2RS1': 4, 'Bearing cap floating': 1, 'Belt HTD 350-5M-15': 2, 'Buffer pad': 2, 'Cap screw ISO 4762 M3x8': 1, 'Cap screw ISO 4762 M4x10': 8, 'Cap screw ISO 4762 M4x12': 4, 'Cap screw ISO 4762 M5x12': 6, 'Cap screw ISO 4762 M5x16': 6, 'Cap screw ISO 4762 M8x20': 4, 'Carrier block': 1, 'Countersunk screw ISO 10642 M3x8': 8, 'Countersunk screw ISO 10642 M4x12': 4, 'Dowel ISO 8734 5x16': 4, 'End washer 18x5.5x2.5': 2, 'Key DIN 6885 A 5x5x18': 1, 'Key DIN 6885 A 5x5x8': 1, 'Motor bracket': 2, 'Motor output': 2, 'Output bevel gear': 1, 'Retaining ring DIN 471-15': 1, 'Retaining ring DIN 471-15, flange': 1, 'Retaining ring DIN 472-32': 1, 'Roll shaft, bored': 1, 'Servo motor with brake and 15:1 gearhead': 2, 'Side bevel gear with 36-groove pulley': 2, 'Side cheek': 2, 'Slip ring bracket': 1, 'Slip ring rotor': 1, 'Slip ring stator': 1, 'Spacer S1': 2, 'Spacer S2': 2, 'Spacer S3': 2, 'Spacer S4': 1, 'Spacer S5': 1, 'Stop cap': 1, 'Stop disc': 1, 'Tie bar': 2, 'Tool flange ISO 9409-1-31.5-4-M5': 1, 'Trunnion': 1, 'Trunnion with stop end': 1}
POSES = [(30.0, 0.0), (0.0, 45.0), (-40.0, 70.0), (75.0, -123.0), (-110.0, 200.0), (25.0, 30.0), (-60.0, 150.0),
         (120.0, -90.0), (-120.0, 300.0), (127.0, 30.0), (-127.0, -200.0)]


def own_numbers():
    """The brakes' stop: the switch at 121 deg, 20 ms at pi rad/s, then both 0.32 N m brakes through 30 against
    gravity's 6.011 N m, the pitching group's and the motors' inertia 0.05993 + 2 * 5.4e-6 * 900."""
    J = 0.05993 + 2 * 5.4e-6 * 900
    KE = 0.5 * J * math.pi ** 2
    return 121.0 + math.degrees(math.pi * 0.020) + math.degrees(KE / (2 * 0.32 * 30 - 6.011))


def props(o):
    pp = o.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    c = pp.centerOfMass
    return pp.mass, (c.x * 10, c.y * 10, c.z * 10)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    o = ns['design']()
    own = own_numbers()
    fc.ok(abs(o['stop']['stop_at'] - own) < 0.01, 'the brakes stop the carrier at %.2f deg, worked again %.2f' % (o['stop']['stop_at'], own))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    pl = {}
    for k, v in ns['parts_list'](root).items():          # Fusion names a second component of the same name 'name (1)'
        k = k.rsplit(' (', 1)[0] if k.endswith(')') and k.rsplit(' (', 1)[-1][:-1].isdigit() else k
        pl[k] = pl.get(k, 0) + v
    print('  parts: %d components, %d kinds: %s' % (sum(pl.values()), len(pl), sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    last = ns['_last']
    occ = last['occ']
    pose = ns['pose']
    Y = (0, 1, 0)
    pose(0.0, 0.0)
    R0 = {k: fc.rotation(v) for k, v in occ.items()}
    pose(20.0, 0.0)
    t = {k: fc.turn_deg(fc.rotation(occ[k]), R0[k], Y) for k in ('carrier', 'mL', 'mR')}
    fc.ok(abs(t['carrier'] - 20) < 0.01 and abs(t['mL'] - 40) < 0.02 and abs(t['mR'] - 40) < 0.02,
          'the links still hold: pitch 20, motors %.3f and %.3f' % (t['mL'], t['mR']))
    pose(0.0, 0.0)
    h = fc.interference(leaves)
    fc.ok(not h, 'no interference among the %d parts at rest %s' % (len(leaves), h or ''))
    for p_, r_ in POSES:
        pose(p_, r_)
        h = fc.interference(leaves)
        fc.ok(not h, 'no interference at pitch %.0f, roll %.0f deg %s' % (p_, r_, h or ''))
    for p_ in (129.0, -129.0):
        pose(p_, 0.0)
        h = fc.interference(leaves)
        names = set(tuple(sorted(x[:2])) for x in h)
        fc.ok(names == {('pad', 'stop disc')}, 'at pitch %.0f the lug is in a pad and nothing else touches: %s' % (p_, h))
    pose(0.0, 0.0)
    # the flange's interface, measured on its faces
    fl = [x for x in leaves if x.component.name == 'Tool flange ISO 9409-1-31.5-4-M5'][0]
    cyl = sorted(set(round(f.geometry.radius * 20, 2) for b in fl.bRepBodies for f in b.faces
                     if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType))
    print('  the flange\'s cylindrical faces, diameters mm: %s' % cyl)
    fc.ok(20.0 in cyl and 5.0 in cyl and 45.0 in cyl, 'the flange has its 20 mm recess, its 5 mm pin hole and its 45 mm outside')
    m_all = 0.0
    S = [0.0, 0.0, 0.0]
    for x in root.occurrences:
        m, c = props(x)
        m_all += m
        for i in range(3):
            S[i] += m * c[i]
    cg = [v / m_all for v in S]
    mr, cr = props(occ['roll'])
    mc, cc = props(occ['carrier'])
    pp = occ['roll'].getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    ok_, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()
    ppc = occ['carrier'].getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
    ok_, cxx, cyy, czz, cxy, cyz, cxz = ppc.getXYZMomentsOfInertia()
    mot = [x for x in leaves if x.component.name == 'Servo motor with brake and 15:1 gearhead']
    m_mot = sum(x.physicalProperties.mass for x in mot)
    meas = dict(total=m_all, cg_x=cg[0], cg_y=cg[1], cg_z=cg[2], roll=mr, roll_z=cr[2], roll_Iz=zz * 1e-4,
                roll_Ix=yy * 1e-4 - mr * (cr[2] ** 2 + cr[0] ** 2) * 1e-6, carrier=mc, carrier_z=cc[2], carrier_Iy=cyy * 1e-4,
                motors_drawn=m_mot)
    print('  MEASURED ' + json.dumps({k: round(v, 7) for k, v in meas.items()}))
    fc.refused(ns, 'brakes', contact=125.0)
    fc.refused(ns, 'too short', contact=155.0)
    fc.verdict()
