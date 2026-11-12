# =============================================================================
# RangeSweep.py  -  Chapter 56 of "Mechanical Design in Autodesk Fusion with Claude": interference and motion over the
# whole range. It builds one of the nine machines as Part 9 detailed it, each with its own script (WristDetail,
# FlapDetail, AxleDetail, PalletizerDetail for the robot, the bag gripper, the compliance unit and the vacuum gripper,
# ClampGripper and ToolChanger), steps it through the states lib/range_sweep.py gives, and at every state looks for
# interference among all its parts and measures the gap between every two parts that move against each other and
# come within 10 mm. It reports the states with interference, the contacts made in motion, and the closest approaches.
# The bag gripper is swept twice: as Part 9 left it (bag0), its jaws free to the cylinders' retracted end, and with
# this chapter's stops (bag): the levers carried on as strikers, a Festo DYSC at each end of each jaw's travel on a
# bracket bolted to the end plate.
# Written from PROMPT.txt beside it; checked by checks/range_sweep_check.py.
#
#   INPUTS        the dialog: the machine (wrist by default)
#   DERIVED       range_sweep: STATES, GAP, STEADY
#   HARD RULES    nothing of the machines' scripts copied; the states only from the machines' own limits
#   CONVENTIONS   each machine's own new design document; lengths in mm, the API's cm inside
#   ACCEPTANCE    the message; the check: each machine's sweep, its interference, its closest approaches
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import math
import os
import sys
import time
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
PROJECT = os.path.dirname(HERE)                         # the project's folder: what this script writes goes in its results/
ROOT = PROJECT                                          # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script


def _load(name):
    spec = importlib.util.spec_from_file_location('mdfc_rs56_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


import range_sweep as RS      # noqa: E402
importlib.reload(RS)

VERSION = 'RangeSweep 1.2 (2026-11-12)'
MACHINES = ('wrist', 'flap', 'axle', 'robot', 'bag0', 'bag', 'clamp', 'vacuum', 'changer', 'unit')
EXCLUDE = dict(unit=('Spring',), clamp=('Carton',), vacuum=('Carton', 'Pack', 'Spring'), bag=('Bag', 'Roller'), bag0=('Bag', 'Roller'))
_last = {}
_mods = {}                     # the machine's own script as loaded, for a later chapter's use


# ---------- each machine: build it, and a function that puts it in a state ----------
def _joint(name):
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    for comp in des.allComponents:
        for j in list(comp.asBuiltJoints) + list(comp.joints):
            if j.name == name:
                return j
    raise RuntimeError('no joint named %r' % name)


def machine(key):
    if key == 'wrist':
        m = _load('WristDetail')
        msg = m.build()
        return msg, lambda s: m.pose(*s[1])
    if key == 'flap':
        m = _load('FlapDetail')
        msg = m.build()
        return msg, lambda s: m.pose(s[1])
    if key == 'axle':
        m = _load('AxleDetail')
        msg = m.build()
        jp, js = _joint('Pinion turns'), _joint('Spider 1 turns in the case')

        def axle_set(s):
            a, sp = s[1]
            js.jointMotion.rotationValue = math.radians(sp)
            jp.jointMotion.rotationValue = math.radians(a)
        return msg, axle_set
    if key in ('robot', 'bag0', 'bag', 'unit', 'vacuum'):
        m = _load('PalletizerDetail')
        if key == 'bag':
            _stops_before(m)
        msg = m.build('bag' if key == 'bag0' else key)
        mod = m._last['mod']
        _mods[key] = mod
        if key == 'bag':
            msg = _stops_after(m, mod) + '\n' + msg
        if key == 'robot':
            return msg, lambda s: mod.set_pose(mod._last['groups'], 0.0, s[1][0], s[1][1])
        if key in ('bag0', 'bag'):
            return msg, lambda s: mod._last['g'].set_state(mod._last['groups'], phi=s[1])
        return msg, lambda s: mod.set_state(mod._last['groups'], lift=s[1])
    if key == 'clamp':
        m = _load('ClampGripper')
        msg = m.build()
        return msg, lambda s: m.set_state(m._last['groups'], s[1])
    if key == 'changer':
        m = _load('ToolChanger')
        msg = m.build()
        return msg, lambda s: m.set_state(m._last['groups'], s[1][0], s[1][1])
    raise ValueError('No machine called %s: Chapter 56 sweeps %s.' % (key, ', '.join(MACHINES)))


# ---------- the bag gripper's stops (lib/range_sweep.py: STRIKER, DYSC, stop_layout, bracket_bolts) ----------
def _stops_before(m):
    """Before the build: each lever carried on past its pin as the closing stop's striker, and the end plates
    drilled for the brackets' bolts, through PalletizerDetail's own patch of BagGripper."""
    plain = m.patch_bag
    mp = m.mp
    lay = RS.stop_layout()
    x0, x1 = RS.STRIKER['x']
    r0, r1 = RS.STRIKER['r']

    def patch_bag(g, weld):
        g = plain(g, weld)
        pt, ep = g.part, g.end_plate

        def part(gr, name, tb, material='Steel'):
            # the levers as StandardParts draws them (its own lever replaces the jaw's), carried on here, as each is
            # made: jaw B's is jaw A's turned half a turn
            if name in ('Lever A', 'Lever B'):
                st = mp.box(x0, x1, g.YH - 20.0, g.YH + 20.0, g.ZH + r0, g.ZH + r1)
                mp.union(tb, st if name == 'Lever A' else g.turned(st))
            return pt(gr, name, tb, material)

        def end_plate():
            b = ep()
            px0, px1 = g.L['plate_x']
            for y, z in lay['closed']['bolts'] + lay['open']['bolts']:
                mp.cut(b, mp.cyl((px0 - 1.0, y, z), (px1 + 1.0, y, z), 4.5))
            return b
        g.part, g.end_plate = part, end_plate
        return g
    m.patch_bag = patch_bag


def _absorber(d, face, n, x, mp):
    """A DYSC in its end position, drawn to Festo's p. 7: the body's front face at `face` (y, z), along n; the
    thread D1 for L2, the rear D3 to L1; the rod pushed in; the lock nut across its flats as a ring."""
    P = lambda t: (x, face[0] + n[0] * t, face[1] + n[1] * t)
    b = mp.cyl(P(0.0), P(d['L2']), d['D1'] / 2.0)
    mp.union(b, mp.cyl(P(d['L2'] - 0.01), P(d['L1']), d['D3'] / 2.0))
    return b


def _stops_after(m, mod):
    """After the build: for each end plate, the closing and the opening bracket, their DYSC with its lock nut, and
    the brackets' ISO 4762 M8 with an ISO 7089 washer and an ISO 4032 nut inside the plate. Jaw B's turned half a
    turn about z."""
    mp = m.mp
    g = mod._last['g']
    G_ = mod._last['groups']
    frame = G_['Frame']
    lay = RS.stop_layout()
    d = RS.DYSC[RS.jaw_stops()['closed']['pick']['size']]
    bb = RS.bracket_bolts()
    B = RS.BRACKET
    face = g.L['plate_x'][1]
    px0 = g.L['plate_x'][0]
    import materials_fits as MFL
    steel = m.MF.library_material(MFL.FUSION['steel 1045 (C45)'])
    c, o = lay['closed'], lay['open']
    box = lambda q: mp.box(q['x'][0], q['x'][1], q['y'][0], q['y'][1], q['z'][0], q['z'][1])

    def bracket_closed():
        b = box(c['foot'])
        mp.union(b, box(c['lip']))
        mp.union(b, box(c['lug']))
        mp.cut(b, mp.cyl((c['x'], c['lug']['y'][0] - 1.0, c['face'][1]), (c['x'], c['lug']['y'][1] + 1.0, c['face'][1]), d['D1'] / 2.0))
        for y, z in c['bolts']:
            mp.cut(b, mp.cyl((face - 1.0, y, z), (face + B['foot'] + 1.0, y, z), 4.5))
        return b

    def bracket_open():
        b = box(o['foot'])
        mp.union(b, box(o['arm']))
        P = o['at']
        t0, t1 = o['lug']['t']
        p0, p1 = P(t0), P(t1)
        mp.union(b, mp.cyl((o['x'], p0[0], p0[1]), (o['x'], p1[0], p1[1]), o['lug']['r']))
        q0, q1 = P(t0 - 1.0), P(t1 + 1.0)
        mp.cut(b, mp.cyl((o['x'], q0[0], q0[1]), (o['x'], q1[0], q1[1]), d['D1'] / 2.0))
        for y, z in o['bolts']:
            mp.cut(b, mp.cyl((face - 1.0, y, z), (face + B['foot'] + 1.0, y, z), 4.5))
        return b

    def lock_nut(face_, n, t):
        P = lambda u: (c['x'], face_[0] + n[0] * u, face_[1] + n[1] * u)
        return mp.tube(P(t[0]), P(t[1]), d['AF'] / 2.0, d['D1'] / 2.0 + 0.001)

    sets = [('closed', bracket_closed(), _absorber(d, c['face'], c['axis'], c['x'], mp), lock_nut(c['face'], c['axis'], (c['nut'][0] - c['face'][0], c['nut'][1] - c['face'][0])), c['bolts']),
            ('open', bracket_open(), _absorber(d, o['face'], o['axis'], o['x'], mp), lock_nut(o['face'], o['axis'], o['nut']), o['bolts'])]
    k = 0
    made = []
    for side, turn in ((1, False), (2, True)):
        for name, br, ab, nt, bolts in sets:
            T = (lambda tb: g.turned(tb)) if turn else (lambda tb: mp.tbm().copy(tb))
            ob = g.part(frame, 'Stop bracket %s %d' % (name, side), T(br), 'Steel')
            for bd in ob.component.bRepBodies:
                bd.material = steel
            made.append(ob)
            g.part(frame, 'Shock absorber %s %s %d' % (d['type'], name, side), T(ab), 'Steel')
            g.part(frame, 'Lock nut %s %s %d' % (d['type'], name, side), T(nt), 'Steel')
            for y, z in bolts:
                k += 1
                sc = mp.cap_screw(B['bolt'], bb['L'], (face + B['foot'], y, z), (-1.0, 0.0, 0.0))
                ws = mp.tube((px0, y, z), (px0 - B['washer'][2], y, z), B['washer'][1] / 2.0, B['washer'][0] / 2.0)
                nu = mp.nut(B['bolt'], (px0 - B['washer'][2], y, z), (-1.0, 0.0, 0.0), B['nut_m'])
                g.part(frame, 'Cap screw ISO 4762 M8x%d stop %d' % (bb['L'], k), T(sc), 'Steel')
                g.part(frame, 'Washer ISO 7089 8 stop %d' % k, T(ws), 'Steel')
                g.part(frame, 'Nut ISO 4032 M8 stop %d' % k, T(nu), 'Steel')
    root = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct).rootComponent
    desc = {}
    m.describe(root, 'Shock absorber', 'Festo %s, part no. %d' % (d['type'], d['part']), desc)
    m.describe(root, 'Lock nut', 'Festo %s, its lock nut M%dx%.1f, %d across flats' % (d['type'], d['D1'], d['P'], d['AF']), desc)
    m.describe(root, 'Stop bracket', 'steel 1045 (C45), bolted to the end plate', desc)
    m.describe(root, 'Cap screw ISO 4762 M8x%d stop' % bb['L'], 'ISO 4762 M8 x %d - 8.8 - zinc-plated' % bb['L'], desc)
    m.describe(root, 'Washer ISO 7089 8 stop', 'ISO 7089 8 - 200 HV - zinc-plated', desc)
    m.describe(root, 'Nut ISO 4032 M8 stop', 'ISO 4032 M8 - 8 - zinc-plated', desc)
    return ('Jaw stops: the levers carried on to %.0f mm as strikers; four %s on four brackets, %d M8 x %d through '
            'the end plates.' % (RS.STRIKER['r'][1], d['type'], k, bb['L']))


# ---------- the sweep ----------
def _mat(o):
    a = o.transform2.asArray()
    return [list(a[0:4]), list(a[4:8]), list(a[8:12]), list(a[12:16])]


def _mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _inv(T):
    R = [[T[j][i] for j in range(3)] for i in range(3)]
    t = [-(R[i][0] * T[0][3] + R[i][1] * T[1][3] + R[i][2] * T[2][3]) for i in range(3)]
    return [R[0] + [t[0]], R[1] + [t[1]], R[2] + [t[2]], [0.0, 0.0, 0.0, 1.0]]


def _sig(deltas):
    return tuple(round(v, 4) for D in deltas for row in D[:3] for v in row)


def _gap(a, b):
    g = 0.0
    for ax in 'xyz':
        g = max(g, getattr(a.minPoint, ax) - getattr(b.maxPoint, ax), getattr(b.minPoint, ax) - getattr(a.maxPoint, ax))
    return g * 10.0


def _interference(occs):
    des = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    col = adsk.core.ObjectCollection.create()
    for o in occs:
        col.add(o)
    inp = des.createInterferenceInput(col)
    inp.areCoincidentFacesIncluded = False
    res = des.analyzeInterference(inp)
    return [(r.entityOne.name, r.entityTwo.name, round(r.interferenceBody.volume * 1000, 3)) for r in res] if res else []


def sweep(key, states, setter):
    app = adsk.core.Application.get()
    des = adsk.fusion.Design.cast(app.activeProduct)
    ex = EXCLUDE.get(key, ())
    leaves = [o for o in des.rootComponent.allOccurrences if o.component.bRepBodies.count
              and not o.component.name.startswith(ex) and not any(p.startswith(ex) for p in o.fullPathName.split('+'))]
    # the parts that move together: their transforms at a few states, against the first
    sample = sorted({0, len(states) // 3, 2 * len(states) // 3, len(states) - 1})
    snaps = []
    for i in sample:
        setter(states[i])
        adsk.doEvents()
        snaps.append([_mat(o) for o in leaves])
    group = {}
    for k, o in enumerate(leaves):
        T0i = _inv(snaps[0][k])
        group[k] = _sig([_mul(s[k], T0i) for s in snaps[1:]])
    gid = {g: n for n, g in enumerate(sorted(set(group.values())))}
    mm = app.measureManager
    dist = lambda i, j: min(mm.measureMinimumDistance(ba, bb).value * 10.0
                            for ba in leaves[i].bRepBodies for bb in leaves[j].bRepBodies)
    # the candidates: pairs from different groups within GAP at a sample state, measured there; a pair whose gap is
    # the same at every sample (a bearing in its seat, a part turning round its own axis) is not measured again
    t0 = time.time()
    seen = {}
    for i_s in sample:
        setter(states[i_s])
        adsk.doEvents()
        bbs = [o.boundingBox for o in leaves]
        for i in range(len(leaves)):
            for j in range(i + 1, len(leaves)):
                if group[i] != group[j] and _gap(bbs[i], bbs[j]) <= RS.GAP:
                    seen.setdefault((i, j), []).append(dist(i, j))
    keep = [k for k, v in seen.items() if len(v) < len(sample) or max(v) - min(v) >= RS.STEADY]
    pairs, hits = {}, []
    for s in states:
        setter(s)
        adsk.doEvents()
        h = _interference(leaves)
        if h:
            hits.append((s, h[:4]))
        bbs = [o.boundingBox for o in leaves]
        for i, j in keep:
            if _gap(bbs[i], bbs[j]) > RS.GAP:
                continue
            pairs.setdefault((i, j), []).append((dist(i, j), s))          # by the parts, not their names: two bearings
    n = len(states)                                                       # of one kind are two parts
    rows = []
    for (i, j), v in pairs.items():
        ds = [d for d, _ in v]
        hi = max(ds) if len(v) == n else RS.GAP
        lo, at = min(v, key=lambda x: x[0])
        if hi - lo < RS.STEADY:
            continue
        rows.append(dict(pair=(leaves[i].component.name, leaves[j].component.name), min=lo, at=at, max=hi, seen=len(v),
                         paths=(leaves[i].fullPathName, leaves[j].fullPathName)))
    rows.sort(key=lambda r: r['min'])
    setter(states[0])
    adsk.doEvents()
    n_all = len(leaves) * (len(leaves) - 1) // 2
    n_cross = sum(1 for i in range(len(leaves)) for j in range(i + 1, len(leaves)) if group[i] != group[j])
    return dict(key=key, states=n, parts=len(leaves), groups=len(gid), hits=hits, rows=rows,
                contacts=[r for r in rows if r['min'] < 0.01], seconds=time.time() - t0,
                pairs=dict(all=n_all, cross=n_cross, near=len(seen), kept=len(keep), measured=len(pairs)))


def build(key='wrist'):
    if key not in MACHINES:
        raise ValueError('No machine called %s: Chapter 56 sweeps %s.' % (key, ', '.join(MACHINES)))
    states = RS.STATES[key]()
    msg, setter = machine(key)
    r = sweep(key, states, setter)
    _last.clear()
    _last.update(r)
    near = ', '.join('%s / %s %.2f mm at %s' % (x['pair'][0], x['pair'][1], x['min'], x['at'][1]) for x in r['rows'][:3])
    return ('Range sweep of the %s: %d states, %d parts in %d groups that move apart; %s; %d contacts made in motion; '
            'closest: %s.\n%s\n\n[%s]' % (key, r['states'], r['parts'], r['groups'],
                                         'no interference' if not r['hits'] else 'interference at %d states' % len(r['hits']),
                                         len(r['contacts']), near or 'none', msg.split('\n')[0], VERSION))


# ---------- the command dialog ----------
_handlers = []


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            key = args.command.commandInputs.itemById('key').selectedItem.name
            ui.messageBox(build(key))
        except ValueError as e:
            ui.messageBox(str(e))
        except Exception:
            ui.messageBox('RangeSweep failed:\n%s' % traceback.format_exc())


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)
        dd = cmd.commandInputs.addDropDownCommandInput('key', 'Machine', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in MACHINES:
            dd.listItems.add(k, k == 'wrist')


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        cid = 'mdfcRangeSweep'
        cdef = ui.commandDefinitions.itemById(cid)
        if cdef:
            cdef.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Range sweep', 'Chapter 56: interference over the whole range')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('RangeSweep failed:\n%s' % traceback.format_exc())
