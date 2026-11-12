# HarmonicDrive (Chapter 19) against the acceptance criteria of its prompt, measured on what Fusion built: the tips
# at the long and short axes and the circular spline's tips measured on the bodies; the drive turned through one mesh
# period (360 / z_c deg) in 6 steps and to 90 deg, with no interference at any (7 rebuilds: each takes the script
# about two minutes, and the whole check must stay under half an hour).
import math, sys, importlib, time
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core

PATH = '__COMPANION__/projects/harmonic_drive/HarmonicDrive/HarmonicDrive.py'


def edge_mm(body, ang_deg, z, r_in, r_out):
    """Along the ray at ang_deg (height z), the radius where the body's material ends between r_in and r_out,
    found by halving the interval with point-containment tests to 1e-6 mm."""
    c, s = math.cos(math.radians(ang_deg)), math.sin(math.radians(ang_deg))
    inside = lambda r: body.pointContainment(adsk.core.Point3D.create(r * c * 0.1, r * s * 0.1, z * 0.1)) == 0
    a, b = r_in, r_out
    want = inside(a)
    for _ in range(40):
        m = (a + b) / 2
        if inside(m) == want:
            a = m
        else:
            b = m
    return (a + b) / 2


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    k = ns['harmonic']()
    want = dict(z_c=202, ratio=-100.0, r_f=30.0, r_c=30.3, w0=0.3)
    bad = [x for x in want if abs(k[x] - want[x]) > 1e-9]
    fc.ok(not bad, 'z_c %d, ratio %.0f, r_f %.3f, r_c %.3f, w0 %.3f mm %s' % (k['z_c'], k['ratio'], k['r_f'], k['r_c'],
                                                                         k['w0'], bad or ''))
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    names = sorted(o.component.name for o in root.occurrences)
    fc.ok(names == ['CircularSpline', 'Flexspline', 'WaveGenerator'], 'three components: %s' % names)
    fc.ok(all(o.component.bRepBodies.count == 1 for o in root.occurrences), 'one body each')
    fc.ok(root.joints.count == 0 and root.asBuiltJoints.count == 2 and root.motionLinks.count == 0,
          'two revolute As-built Joints, no Motion Link')
    fl = fc.occ('Flexspline').component.bRepBodies.item(0)
    cs = fc.occ('CircularSpline').component.bRepBodies.item(0)
    z = k['b'] / 2
    long_ = edge_mm(fl, 0.0, z, 29.6, 31.0)
    short = edge_mm(fl, 90.0, z, 29.2, 31.0)
    tips = edge_mm(cs, 180.0 / k['z_c'], z, 29.0, 31.0)
    fc.ok(abs(long_ - 30.525) < 5e-4, 'flexspline tip on the long axis %.4f mm from the axis' % long_)
    fc.ok(abs(short - 29.925) < 5e-4, 'flexspline tip on the short axis %.4f mm from the axis' % short)
    fc.ok(abs(tips - 30.075) < 5e-4, 'circular spline tips %.4f mm from the axis' % tips)
    c = fc.interference()
    fc.ok(not c, 'no interference at phi = 0 %s' % (c or ''))
    per = 360.0 / k['z_c']
    angles = [round(per * i / 6, 4) for i in range(1, 7)] + [90]
    hits, rhos = [], []
    pair = [fc.occ('Flexspline'), fc.occ('CircularSpline')]        # the meshing pair, at every angle
    for a in angles:
        t0 = time.time()
        rhos.append((a, ns['turn_to'](a)))
        adsk.doEvents()
        t1 = time.time()
        hit = fc.interference() if a == 90 else fc.interference(pair)          # all three parts at 90
        if hit:
            hits.append((a, hit))
        print('  phi %8.4f deg: rebuilt in %.0f s, interference in %.0f s' % (a, t1 - t0, time.time() - t1))
    fc.ok(abs(rhos[-1][1] + 0.9) < 1e-9, 'the flexspline turned %.4f deg at phi = 90' % rhos[-1][1])
    fc.ok(not hits, 'no interference at any of the %d angles %s' % (len(angles), hits or ''))
    fc.refused(ns, 'difference of 2', dz=1)
    fc.refused(ns, 'from 60 to 320', z_f=40)
    fc.verdict()
