# ToolChanger (Chapter 44) against the acceptance criteria of its prompt, measured on what Fusion built: the parts
# counted; no interference locked and free; locked, the tool plate lowered 0.5 mm runs into the segments (the lock
# holds); free, it comes off through 45 mm with nothing in the way; the segments' faces on the lip's face when locked
# (their least distance) and clear of the bore when free; the robot plate's ISO 9409-1 pattern measured on its
# cylindrical faces; the groups weighed; two refusals.
import math, sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
import fusion_check as fc
importlib.reload(fc)
import adsk.core, adsk.fusion

PATH = '__COMPANION__/scripts/ToolChanger/ToolChanger.py'


def holes(body, r_mm, tol=0.01):
    """The axes (x, y) in mm of the body's cylindrical faces of radius r_mm."""
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
    seg_names = ['Segment %d' % (k + 1) for k in range(6)]
    tool = [o for o in leaves if o.fullPathName.startswith('Tool plate')]
    segs = [o for o in leaves if o.fullPathName.split(':')[0] in seg_names]
    # locked
    ns['set_state'](G, True)
    c = fc.interference(leaves)
    fc.ok(not c, 'locked: no interference among the %d parts %s' % (len(leaves), c or ''))
    mm_ = adsk.core.Application.get().measureManager
    tb = tool[0].bRepBodies.item(0)
    gap = min(mm_.measureMinimumDistance(s.bRepBodies.item(0), tb).value * 10 for s in segs)
    fc.ok(gap < 0.001, 'locked: every segment\'s 15 deg face on the lip\'s face, least distance %.4f mm' % gap)
    ns['set_state'](G, True, 0.5)
    c = fc.interference(segs + tool)
    vol = sum(v for a, b, v in c)
    fc.ok(len(c) == 6 and vol > 0, 'locked, the tool plate lowered 0.5 mm runs into all six segments (%d clashes, '
          '%.1f mm3): the lock holds it' % (len(c), vol))
    # free
    ns['set_state'](G, False)
    c = fc.interference(leaves)
    fc.ok(not c, 'free: no interference %s' % (c or ''))
    gapf = min(mm_.measureMinimumDistance(s.bRepBodies.item(0), tb).value * 10 for s in segs)
    fc.ok(gapf > 0.2, 'free: the segments clear of the tool plate by %.3f mm' % gapf)
    hits = []
    for dz in (5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0):
        ns['set_state'](G, False, dz)
        c = fc.interference(leaves)
        if c:
            hits.append((dz, c))
    fc.ok(not hits, 'free: the tool plate comes off through 45 mm in 5 mm steps with nothing in the way %s' % (hits or ''))
    # the piston's rod in the gland and the two air ports on their sides of the piston, locked and free, measured on
    # the bodies' boxes in the assembly
    GM = ns['GM']
    pis = [o for o in leaves if o.component.name.startswith('Piston with rod')][0].bRepBodies.item(0)
    zg0, zg1 = GM['z_gland']
    z_rel, z_lock = GM['z_ports']
    rows = []
    for locked in (True, False):
        ns['set_state'](G, locked)
        bb = pis.boundingBox
        z_end, z_top = bb.minPoint.z * 10, bb.maxPoint.z * 10
        z_face = z_top - GM['piston_h']
        rows.append((z_end, z_face, z_top))
    good = all(z_end < zg0 and z_face > z_rel + GM['port_d'] / 2 and z_top < z_lock - GM['port_d'] / 2
               for z_end, z_face, z_top in rows)
    fc.ok(good, 'the rod through the gland (%.0f to %.0f) locked and free, its end at %.1f and %.1f; the release port '
          '(%.0f) under the piston\'s face (%.1f, %.1f), the lock port (%.0f) over its top (%.1f, %.1f)'
          % (zg0, zg1, rows[0][0], rows[1][0], z_rel, rows[0][1], rows[1][1], z_lock, rows[0][2], rows[1][2]))
    ns['set_state'](G, True)
    # the robot plate's ISO 9409-1-160-6-M10 pattern, measured
    rp = [o for o in leaves if o.fullPathName.startswith('Robot plate')][0].bRepBodies.item(0)
    th = holes(rp, 5.0)
    ang = sorted(round(math.degrees(math.atan2(y, x)) % 360.0, 3) for x, y in th)
    rad = [math.hypot(x, y) for x, y in th]
    rec = holes(rp, 50.0)
    fc.ok(len(th) == 7 and all(abs(r - 80.0) < 0.001 for r in rad) and ang == [0.0, 30.0, 90.0, 150.0, 210.0, 270.0, 330.0]
          and rec == [(0.0, 0.0)],
          'the robot plate: six M10 holes and the 10 H7 pin hole on the 160 circle at %s deg, the 100 H7 recess on the '
          'axis' % ang)
    # weighed
    def mass(pred):
        return sum(o.physicalProperties.mass for o in leaves if pred(o))
    top = lambda o: o.fullPathName.split('+')[0].split(':')[0]
    m_master = mass(lambda o: top(o) in ('Master', 'Piston') or top(o) in seg_names)
    m_tool = mass(lambda o: top(o) == 'Tool plate')
    m_robot = mass(lambda o: top(o) == 'Robot plate')
    print('  weighed: robot plate %.2f kg, master with piston and segments %.2f kg, tool plate %.2f kg' % (m_robot, m_master, m_tool))
    fc.refused(ns, 'friction', alpha=6.0)
    fc.refused(ns, 'under', p_bar=1.5)
    fc.verdict()
