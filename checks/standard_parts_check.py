# StandardParts (Chapter 50) against the acceptance criteria of its prompt, measured on what Fusion holds after the run:
# the bag gripper built, numbered and detailed; every bought part described by its full designation and the detail's
# new kinds numbered; the shoulder measured inside SKF's band; the bearing's faces at SKF's figures; the key inside the
# hub's boss; the stack from the shoulder to the end washer closed and the washer clear of the shaft's end; the end
# screw's thread; the dowel at its made length; no interference closed and with the jaws open; refusals.
import sys, importlib
sys.path.insert(0, '__COMPANION__/tools')
sys.path.insert(0, '__COMPANION__/lib')
import fusion_check as fc
importlib.reload(fc)
import detail_inventory as D
import standard_parts as S
importlib.reload(D)
importlib.reload(S)
import adsk.core

PATH = '__COMPANION__/scripts/StandardParts/StandardParts.py'


def cylinders(o):
    """The x-axis cylindrical faces of an occurrence's body, in world mm: [(diameter, x_min, x_max)]."""
    out = []
    for f in o.bRepBodies.item(0).faces:
        g = f.geometry
        if isinstance(g, adsk.core.Cylinder) and abs(abs(g.axis.x) - 1.0) < 1e-6:
            b = f.boundingBox
            out.append((round(g.radius * 20, 3), round(b.minPoint.x * 10, 3), round(b.maxPoint.x * 10, 3)))
    return out


def run(_context):
    ns = fc.load(PATH)
    print(ns['VERSION'])
    fc.run_dialog(ns)
    root = fc.design().rootComponent
    DT, DW = S.stub_detail(), S.dowel_adapter()
    leaves = [o for o in root.allOccurrences if o.component.bRepBodies.count]
    parts = [o for o in leaves if not (o.fullPathName.startswith('Bag') or o.fullPathName.startswith('Station'))]
    one = lambda nm: [o for o in leaves if o.component.name.split(' (')[0] == nm][0]
    mm_ = adsk.core.Application.get().measureManager
    dist = lambda a, b: mm_.measureMinimumDistance(a.bRepBodies.item(0), b.bRepBodies.item(0)).value * 10
    bb = lambda o: o.bRepBodies.item(0).boundingBox
    new = S.detail_numbers()
    print('  parts: %d, the bag and the station\'s %d rollers apart' % (len(parts), len(leaves) - len(parts) - 1))
    fc.ok(len(parts) == 74 + 2 * 4, 'the parts: %d, the gripper\'s 74 and four more on each jaw\'s driven end' % len(parts))
    # the descriptions
    known = D.number_map('bag')
    bad, n_s, seen = [], 0, {}
    for o in parts:
        c = o.component
        if not c.partNumber.startswith('S-'):
            continue
        n_s += 1
        kind = D.kind_of(c.name, new)
        want = new[kind][1] if kind else S.decide(known[D.kind_of(c.name, known)][1])['full']
        seen[c.partNumber] = c.description
        if c.description != want:
            bad.append((c.name, c.description, want))
    fc.ok(not bad, 'every bought component described by its full designation: %d components, %d numbers; wrong %s'
          % (n_s, len(seen), bad[:3]))
    print('  ' + '; '.join('%s %s' % (k, seen[k]) for k in sorted(seen)))
    nums = {}
    for o in parts:
        k = D.kind_of(o.component.name, new)
        if k:
            nums.setdefault(k, set()).add((o.component.partNumber, o.component.description))
    fc.ok(all(nums.get(k) == {v} for k, v in new.items()) and len(ns['_last']['added']) == 2 * len(new),
          'the detail\'s new kinds numbered on both jaws: %s' % ', '.join('%s %s' % new[k] for k in sorted(new)))
    # the shoulder and the bearing, on jaw A's driven end
    sh = one('Stub shaft long A')
    shoulder = [d for d, x0, x1 in cylinders(sh) if abs(x0 - DT['x_tube']) < 0.02 and abs(x1 - DT['x_brg'][0]) < 0.02]
    fc.ok(len(shoulder) == 1 and DT['da_min'] <= shoulder[0] <= DT['da_max'],
          'the shoulder %s mm across, inside SKF\'s %.1f to %.1f' % (shoulder, DT['da_min'], DT['da_max']))
    brg = [o for o in leaves if o.component.name.startswith('Bearing 6206')]
    db = sorted({d for d, _, _ in cylinders(brg[0])})
    d, Dd, B, d1, D2 = DT['bore']
    fc.ok(len(brg) == 4 and db == sorted({d, Dd, d1, D2}) and all(abs((bb(o).maxPoint.x - bb(o).minPoint.x) * 10 - B) < 1e-6
                                                                    for o in brg),
          'the four 6206-2RS1: faces at %s mm across, %.0f wide' % (db, B))
    # the key in the hub's boss
    key = one('Key %dx%dx%d A' % (DT['b'], DT['h'], DT['key_L']))
    lever = one('Lever A')
    hub = [(x0, x1) for dd, x0, x1 in cylinders(lever) if abs(dd - 2 * DT['hub_r']) < 0.01]
    kx = (round(bb(key).minPoint.x * 10, 3), round(bb(key).maxPoint.x * 10, 3))
    fc.ok(len(hub) >= 1 and hub[0][0] + 2.5 - 1e-6 <= kx[0] and kx[1] <= hub[0][1] - 2.5 + 1e-6
          and abs(kx[1] - kx[0] - DT['key_L']) < 1e-6,
          'the key %s inside the hub\'s boss %s, %.0f long' % (kx, hub[0] if hub else '-', kx[1] - kx[0]))
    # the stack: the shoulder, the inner ring, the spacer, the hub, the end washer
    b1 = [o for o in brg if abs(bb(o).minPoint.x * 10 - DT['x_brg'][0]) < 1e-6 and bb(o).minPoint.y > 0][0]
    sp, ew, sc = one('Spacer A'), one('End washer A'), one('Cap screw M%dx%d A' % DT['screw'])
    gaps = [dist(sh, b1), dist(b1, sp), dist(sp, lever), dist(lever, ew)]
    fc.ok(max(gaps) < 0.001 and abs(dist(ew, sh) - (DT['x_lever'][1] - DT['end'])) < 0.001,
          'the stack closed from the shoulder to the end washer, the largest gap %.4f mm; the washer %.3f mm clear of the '
          'shaft\'s end' % (max(gaps), dist(ew, sh)))
    tip = bb(sc).minPoint.x * 10
    fc.ok(abs(DT['end'] - tip - DT['thread']) < 1e-6 and DT['thread'] >= DT['thread_need'],
          'the end screw\'s thread in the shaft %.1f mm, against 1.5 d = %.1f' % (DT['end'] - tip, DT['thread_need']))
    dw = one('Dowel pin 10x17')
    z0, z1 = bb(dw).minPoint.z * 10, bb(dw).maxPoint.z * 10
    fc.ok(abs(z1 - z0 - DW['L']) < 1e-6 and abs(-z0 - DW['in_adapter']) < 1e-6 and DW['ok'],
          'the dowel %.1f mm long, %.1f in the adapter (%.0f thick), %.1f in the tool plate' % (z1 - z0, -z0, DW['t_adapter'], z1))
    # no interference, closed and opened
    G = ns['_last']['groups']
    g = ns['_last']['g']
    g.set_state(G, 0.0)
    c = fc.interference(leaves)
    fc.ok(not c, 'closed on the bag: no interference among the %d components %s' % (len(leaves), c or ''))
    g.set_state(G, 45.0, e=0.0)
    c = fc.interference(leaves)
    fc.ok(not c, 'the jaws open 45 deg, the press up: no interference %s' % (c or ''))
    g.set_state(G, 0.0)
    fc.refused(ns, 'SKF', shoulder=35.0)
    fc.refused(ns, 'crushed', key_len=22.0)
    fc.refused(ns, 'makes keys', key_len=26.0)
    fc.verdict()
