# ScaraToolJoints (Chapter 37) against the acceptance criteria of its prompt, measured on what Fusion built: the
# screw's cube-mean force and the shaft's swing at the full stroke worked again here; the parts counted from
# Fusion's components; no interference at three poses of J3 and J4; each nut and motor turned as the belts and the
# screw's lead say; the shaft still in the screw nut at the full stroke; the belts' centre distances measured on the
# pulleys' faces; the masses against Chapter 35's; two refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/scara_robot/ScaraToolJoints/ScaraToolJoints.py'
EXPECTED = {'Ball screw spline shaft SP20': 1, 'Base plate': 1, 'Belt 3MR-474-09 J3': 1, 'Belt 3MR-489-09 J4': 1, 'Flange bolt M4': 8, 'Motor bolt M4': 4, 'Motor bolt M5': 4, 'Motor shaft J3': 1, 'Motor shaft J4': 1, 'Pulley 18T motor J4': 1, 'Pulley 28T motor J3': 1, 'Pulley 56T nut J3': 1, 'Pulley 72T nut J4': 1, 'Pulley screw M5': 12, 'Screw nut SP20 flange': 1, 'Screw nut SP20 rotor': 1, 'Servo motor J3': 1, 'Servo motor J4': 1, 'Side wall left': 1, 'Side wall right': 1, 'Spline nut SP20 flange': 1, 'Spline nut SP20 rotor': 1, 'Tool flange': 1, 'Top plate': 1}


def own_numbers():
    """The screw nut's cube-mean force over the cycle's four vertical legs (25 mm cycloids in 0.10 s, 4.0 kg) and
    the SP20 shaft's swing at the full stroke (a cantilever of 295 mm, the 14 mm section, 2.5 kg at its end with
    the flange and the shaft's own share)."""
    m, g, h, T = 4.0, 9.81, 25.0, 0.10
    num = den = 0.0
    n = 1000
    for sg in (1, -1, 1, -1):
        for k in range(n):
            s = (k + 0.5) / n
            v = h / T * (1 - math.cos(2 * math.pi * s))
            a = 2 * math.pi * h / T ** 2 * math.sin(2 * math.pi * s) / 1000
            F = abs(m * (g + sg * a))
            num += v * F ** 3
            den += v
    Fm = (num / den) ** (1 / 3)
    I = math.pi * 14 ** 4 / 64
    k = 3 * 207000 * I / 295.0 ** 3
    m_fl = math.pi / 4 * 50 ** 2 * 10 * 2.70e-6
    over = 225.0
    m_over = math.pi / 4 * 20 ** 2 * over * 7.85e-6
    m_eff = 2.5 + m_fl * ((over + 5.0) / 295.0) ** 2 + 0.236 * m_over * (over / 295.0) ** 3
    f = math.sqrt(k * 1000 / m_eff) / (2 * math.pi)
    return dict(Fm=Fm, f=f)


def axis_of(occ):
    """The axis (x, y) of the largest cylindrical face of an occurrence's bodies, in world mm."""
    best = None
    for body in occ.bRepBodies:
        for f in body.faces:
            if f.geometry.surfaceType == adsk.core.SurfaceTypes.CylinderSurfaceType:
                r = f.geometry.radius
                if best is None or r > best[0]:
                    best = (r, f.geometry.origin)
    return best[1].x * 10, best[1].y * 10


def top_z(occ):
    return max(f.boundingBox.maxPoint.z * 10 for body in occ.bRepBodies for f in body.faces
               if f.geometry.surfaceType == adsk.core.SurfaceTypes.PlaneSurfaceType)


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    own = own_numbers()
    o = ns['design']()
    sd = ns['sd']
    ok = abs(o['screw']['Fm'] - own['Fm']) < 0.01 * own['Fm'] and abs(o['cant']['f'] - own['f']) < 0.02 * own['f']
    fc.ok(ok, 'the screw\'s cube mean %.2f N and the shaft\'s swing %.2f Hz worked again (the module: %.2f, %.2f)'
          % (own['Fm'], own['f'], o['screw']['Fm'], o['cant']['f']))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    subs = sorted(x.component.name for x in root.occurrences)
    fc.ok(len(subs) == 7, 'seven groups: %s' % subs)
    pl = ns['parts_list'](root)
    print('  parts: %d components, %d kinds: %s' % (sum(pl.values()), len(pl), sorted(pl.items())))
    if EXPECTED is not None:
        fc.ok(pl == EXPECTED, 'the parts as listed %s' % ('' if pl == EXPECTED else sorted(set(pl.items()) ^ set(EXPECTED.items()))))
    leaves = [x for x in root.allOccurrences if x.component.bRepBodies.count]
    by = {x.component.name: x for x in leaves}
    pose = ns['pose']
    J = ns['_last']['joints']
    hits = []
    for j3, j4 in ((0.0, 0.0), (100.0, 90.0), (200.0, 180.0)):
        pose(j3, j4)
        h = fc.interference(leaves)
        fc.ok(not h, 'no interference among the %d parts at J3 %.0f mm, J4 %.0f deg %s' % (len(leaves), j3, j4, h or ''))
    # the links and the screw: J4 alone, then J3 alone
    deg = lambda key: math.degrees(J[key][0].jointMotion.rotationValue) * J[key][1]
    same = lambda a, b, tol: abs(((a - b + 180.0) % 360.0) - 180.0) < tol    # Fusion keeps a joint's angle within a turn
    pose(0.0, 90.0)
    fc.ok(abs(deg('spline') - 90.0) < 0.2 and abs(deg('m4') - 360.0) < 0.5 and abs(deg('screw') - 90.0) < 0.2 and abs(deg('m3') - 180.0) < 0.5,
          'J4 90 deg alone: the spline nut %.1f, its motor %.1f, the screw nut %.1f, its motor %.1f deg' % (deg('spline'), deg('m4'), deg('screw'), deg('m3')))
    z0 = top_z(by['Ball screw spline shaft SP20'])
    pose(7.0, 0.0)
    dz = z0 - top_z(by['Ball screw spline shaft SP20'])
    fc.ok(abs(dz - 7.0) < 0.001 and same(deg('spline'), 0.0, 0.2) and same(deg('screw'), -126.0, 0.2) and same(deg('m3'), -252.0, 0.5),
          'J3 7 mm alone: the shaft down %.3f mm, the spline nut %.1f, the screw nut %.1f (-126), its motor %.1f (-252) deg, within a turn'
          % (dz, deg('spline'), deg('screw'), deg('m3')))
    pose(200.0, 0.0)
    shaft_top = top_z(by['Ball screw spline shaft SP20'])
    rotor_top = top_z(by['Screw nut SP20 rotor'])
    fc.ok(shaft_top >= rotor_top + 5.0 - 0.01, 'at the full stroke the shaft\'s top %.2f mm, the screw nut\'s rotor\'s top %.2f mm' % (shaft_top, rotor_top))
    pose(0.0, 0.0)
    for j, nut, mot in ((3, 'Pulley 56T nut J3', 'Pulley 28T motor J3'), (4, 'Pulley 72T nut J4', 'Pulley 18T motor J4')):
        a, b_ = axis_of(by[nut]), axis_of(by[mot])
        cd = math.hypot(a[0] - b_[0], a[1] - b_[1])
        fc.ok(abs(cd - o['belts'][j]['CD']) < 0.01, 'J%d belt: centre distance %.3f mm measured, %.3f for the 3MR-%d' % (j, cd, o['belts'][j]['CD'], o['belts'][j]['stock']))
    spline_group = sum(x.physicalProperties.mass for x in leaves if x.fullPathName.split('+')[0].split(':')[0] in ('Tool shaft', 'Screw side', 'Spline side')
                       or 'nut SP' in x.component.name or x.component.name.startswith('Flange bolt'))
    motors = sum(x.physicalProperties.mass for x in leaves if x.fullPathName.split('+')[0].split(':')[0] in ('J3 motor side', 'J4 motor side'))
    print('  the shaft, the nuts, their pulleys, screws and bolts and the tool flange: %.3f kg (Chapter 35: 1.5 kg)' % spline_group)
    print('  the motors at their specified %.1f and %.1f kg, their shafts and pulleys: %.3f kg (Chapter 35: 2.0 kg)'
          % (sd.MOTOR34[3]['mass'], sd.MOTOR34[4]['mass'], motors + sd.MOTOR34[3]['mass'] + sd.MOTOR34[4]['mass']))
    for x in leaves:
        if x.component.name.startswith('Belt'):
            print('  %s: %.4f kg' % (x.component.name, x.physicalProperties.mass))
    fc.refused(ns, 'falls short', size=16.0)
    fc.refused(ns, 'falls short', size=25.0)
    fc.verdict()
