# =============================================================================
# RangeSweep.py  -  Chapter 56 of "Mechanical Design in Autodesk Fusion with Claude": interference and motion over
# the whole range. It builds one of the detailed machines with its own detail script, loaded, not copied, steps it
# through its range of motion (design_office/lib/range_sweep.py), and at every state asks Fusion for interference
# among all its parts and measures how close the parts that move against each other come. For the bag gripper it
# first adds its stops: a striker on each lever, the end plates drilled, and at each end plate a closing and an
# opening bracket with a Festo DYSC in each.
# Written from PROMPT.txt (design_office/RangeSweep); checked by design_office/checks/range_sweep_check.py.
#
#   INPUTS        the dialog: the machine (wrist by default)
#   DERIVED       range_sweep: STATES, GAP, STEADY, STRIKER, AXIS_X, DYSC, jaw_stops(), stop_layout(),
#                 bracket_bolts(), BRACKET
#   HARD RULES    each machine built and moved by its own script; build() refuses a machine that is not one of ten
#   CONVENTIONS   lengths in mm, the API's cm inside; pairs keyed by the parts, not their names
#   ACCEPTANCE    the message; the check: no interference at any state, the stops' contacts, the JSON of results
#
#   Emmanuel Merchan, TSI Engineering Faculty - AI Augmented Education
# =============================================================================
import importlib
import importlib.util
import inspect
import json
import math
import os
import sys
import time
import traceback

import adsk.core
import adsk.fusion

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = HERE                                             # the companion's folder: the first one up that holds common/
while not os.path.isfile(os.path.join(ROOT, 'common', 'companion_paths.py')) and os.path.dirname(ROOT) != ROOT:
    ROOT = os.path.dirname(ROOT)
if os.path.join(ROOT, 'common') not in sys.path:
    sys.path.insert(0, os.path.join(ROOT, 'common'))
import companion_paths as CP                            # every lib/ folder on sys.path; CP.script(name) finds a script
import mdfc_parts as mp                                 # noqa: E402
import range_sweep as RS                                # noqa: E402
importlib.reload(mp)
importlib.reload(RS)

VERSION = 'RangeSweep 1.0 (2026-11-12)'
CM = 0.1
P3 = adsk.core.Point3D.create
V3 = adsk.core.Vector3D.create
EVENT = 'mdfcRangeSweepBuild'
ZERO = 0.001                    # mm: a measured gap this small is a contact
HOLE_D = 9.0                    # mm: the end plates drilled 9 across for the brackets' M8 (the prompt)

ORDER = ['wrist', 'flap', 'axle', 'robot', 'bag0', 'bag', 'clamp', 'vacuum', 'changer', 'unit']
LABEL = dict(wrist='robotic wrist', flap='flap actuation system', axle='rear axle', robot='palletizer robot',
             bag0='bag gripper without its stops', bag='bag gripper with its stops', clamp='clamp gripper',
             vacuum='vacuum gripper', changer='tool changer', unit='compliance unit')
# the parts that are not the machine, left out by the start of their names (the prompt names what, not the words)
EXCLUDE = dict(clamp=('Carton',), vacuum=('Carton', 'Pack', 'Spring'), bag0=('Bag', 'Roller'), bag=('Bag', 'Roller'),
               unit=('Spring',))
# the names a mover may give the state's values, used only when the mover has all of them as parameters
VALUE_NAMES = dict(wrist=('pitch', 'roll'), robot=('q2', 'q3'), changer=('locked', 'tool_down'))

_last = {}
_handlers = []


def design():
    return adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)


def _load(name):
    """One of the companion's scripts, loaded from its own file as a fresh module."""
    spec = importlib.util.spec_from_file_location('mdfc_rs56_' + name, CP.script(name))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---------- temporary bodies, world millimetres (own box: PalletizerDetail patches mp.box during the bag's build) ----
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def _obox(c, d1, d2, l1, l2, l3):
    """A box centred at c, l1 along d1, l2 along d2 and l3 along d1 x d2."""
    ob = adsk.core.OrientedBoundingBox3D.create(P3(*[v * CM for v in c]), V3(*d1), V3(*d2), l1 * CM, l2 * CM, l3 * CM)
    return _tbm().createBox(ob)


def _abox(b):
    """An axis-aligned box from a dict of x, y and z ranges."""
    (x0, x1), (y0, y1), (z0, z1) = sorted(b['x']), sorted(b['y']), sorted(b['z'])
    return _obox(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (1, 0, 0), (0, 1, 0), x1 - x0, y1 - y0, z1 - z0)


def _along(p, e, t):
    return tuple(p[i] + e[i] * t for i in range(3))


def _turn_matrix():
    m = adsk.core.Matrix3D.create()
    m.setToRotation(math.pi, V3(0, 0, 1), P3(0, 0, 0))
    return m


def _turn(tb):
    """Half a turn about z: the -x end's parts from the +x end's."""
    _tbm().transform(tb, _turn_matrix())
    return tb


def _end_sign(tb):
    bb = tb.boundingBox
    return 1 if bb.minPoint.x + bb.maxPoint.x >= 0 else -1


# ---------- the bag gripper's stops ----------
def _pin(tb, hinge, r_lever):
    """The lever's pin seen in its body: the smallest cylinder along x whose axis lies r_lever from the hinge."""
    best = None
    for f in tb.faces:
        g = f.geometry
        if not isinstance(g, adsk.core.Cylinder) or abs(abs(g.axis.x) - 1.0) > 1e-6:
            continue
        y, z, r = g.origin.y * 10.0, g.origin.z * 10.0, g.radius * 10.0
        if abs(math.hypot(y - hinge[0], z - hinge[1]) - r_lever) > 1.0:
            continue
        if best is None or r < best[2]:
            best = (y, z, r)
    return best


def _striker(tb, name, made):
    """Jaw A's lever (the +x end; jaw B's turned half a turn about z) carried on past its pin to STRIKER's outer
    radius, STRIKER's thickness, as wide as the bar (twice the contact face's offset from the hinge in stop_layout)."""
    L = RS.BG.LAY
    s = _end_sign(tb)
    if s < 0:
        _turn(tb)
    bb = tb.boundingBox
    reach = max(bb.maxPoint.y - bb.minPoint.y, bb.maxPoint.z - bb.minPoint.z) * 10.0
    if reach < 0.6 * L['lever']:                         # a small part whose name starts with Lever: not the lever
        if s < 0:
            _turn(tb)
        return tb
    yh, zh = L['hinge']
    pin = _pin(tb, (yh, zh), L['lever'])
    phi = 0.0 if pin is None else round(math.degrees(math.atan2(-(pin[0] - yh), pin[1] - zh)), 6)
    u = (-math.sin(math.radians(phi)), math.cos(math.radians(phi)))
    r0, r1 = RS.STRIKER['r']
    x0, x1 = RS.STRIKER['x']
    w = 2.0 * (RS.stop_layout()['closed']['face'][0] - yh)
    rc = (r0 + r1) / 2.0
    bar = _obox(((x0 + x1) / 2.0, yh + rc * u[0], zh + rc * u[1]), (0.0, u[0], u[1]), (1.0, 0.0, 0.0),
                r1 - r0, x1 - x0, w)
    if pin is not None:                                  # the pin's hole kept clear through the new bar
        mp.cut(bar, mp.cyl((x0 - 1.0, pin[0], pin[1]), (x1 + 1.0, pin[0], pin[1]), pin[2]))
    mp.union(tb, bar)
    if s < 0:
        _turn(tb)
    made['levers'].append(dict(name=name, end=s, phi=phi, pin=pin))
    return tb


def _bolt_points():
    lay = RS.stop_layout()
    return [('closing', p) for p in lay['closed']['bolts']] + [('opening', p) for p in lay['open']['bolts']]


def _drill_plate(tb, name, made):
    """The end plate drilled 9 across at the brackets' four bolts (the -x end's turned)."""
    L = RS.BG.LAY
    s = _end_sign(tb)
    if s < 0:
        _turn(tb)
    x0, x1 = L['plate_x']
    for _, (y, z) in _bolt_points():
        mp.cut(tb, mp.cyl((x0 - 1.0, y, z), (x1 + 1.0, y, z), HOLE_D / 2.0))
    if s < 0:
        _turn(tb)
    made['plates'].append(dict(name=name, end=s, holes=len(_bolt_points())))
    return tb


def _wrap_bag(g, made):
    """BagGripper as PalletizerDetail has patched it, its parts changed once more on their way in: the levers carry
    the strikers and the end plates the brackets' holes."""
    plain = g.part

    def part(gr, name, tb, material='Steel'):
        if name.startswith('Lever'):
            tb = _striker(tb, name, made)
        elif name.startswith('End plate'):
            tb = _drill_plate(tb, name, made)
        return plain(gr, name, tb, material)
    g.part = part
    return g


def _absorber(face, ax, d):
    """A DYSC with its rod pushed in: from the face along the axis the thread D1 for L2, then the body D3 to L1."""
    b = mp.cyl(face, _along(face, ax, d['L2']), d['D1'] / 2.0)
    mp.union(b, mp.cyl(_along(face, ax, d['L2'] - 0.01), _along(face, ax, d['L1']), d['D3'] / 2.0))
    return b


def _lock_nut(face, ax, t0, t1, d):
    """The DYSC's lock nut, drawn round across its flats, on the thread from t0 to t1 along the axis."""
    return mp.tube(_along(face, ax, t0), _along(face, ax, t1), d['AF'] / 2.0, d['D1'] / 2.0 + 0.001)


def _cap_screw(p, ax, L):
    """ISO 4762 M8: the head BRACKET gives (13 across, 8 high) behind p, the shank L along ax."""
    dk, k = RS.BRACKET['head']
    d = RS.BRACKET['bolt']
    head = mp.cyl(p, _along(p, ax, -k), dk / 2.0)
    mp.cut(head, mp.cyl(_along(p, ax, -0.5 * k), _along(p, ax, -(k + 1.0)), 0.43 * d))
    return mp.union(head, mp.cyl(_along(p, ax, -0.001), _along(p, ax, L), d / 2.0))


def _stop_parts(k):
    """The stops at one end plate, in jaw A's frame (+x end): (name, body, material, description). k names the end."""
    lay = RS.stop_layout()
    js = RS.jaw_stops()
    bolts = RS.bracket_bolts()
    B = RS.BRACKET
    L = RS.BG.LAY
    px0 = L['plate_x'][0]
    Lb = int(round(bolts['L']))
    wd, wD, wt = B['washer']
    desc = {num: text for num, text, _, _ in RS.NEW_NUMBERS}
    out = []
    for end, side in (('closed', 'closing'), ('open', 'opening')):
        c = lay[end]
        d = RS.DYSC[js[end]['pick']['size']]
        ax = (0.0, c['axis'][0], c['axis'][1])
        face = (c['x'], c['face'][0], c['face'][1])
        out.append(('Shock absorber %s %s %d' % (d['type'], end, k), _absorber(face, ax, d), 'Steel',
                    'Festo %s, part no. %d' % (d['type'], d['part'])))
        if end == 'closed':
            t0, t1 = [(y - c['face'][0]) / c['axis'][0] for y in c['nut']]
        else:
            t0, t1 = c['nut']
        out.append(('Lock nut %s %s %d' % (d['type'], end, k), _lock_nut(face, ax, min(t0, t1), max(t0, t1), d),
                    'Steel', 'Lock nut M%g x %g of the %s, %g across the flats, %g thick'
                    % (d['D1'], d['P'], d['type'], d['AF'], d['B1'])))
        foot = c['foot']
        br = _abox(foot)
        if end == 'closed':
            lip = dict(c['lip'])
            lip['x'] = (min(lip['x']), max(max(lip['x']), max(foot['x'])))   # the lip joined to the foot over its corner
            mp.union(br, _abox(lip), _abox(c['lug']))
        else:
            mp.union(br, _abox(c['arm']))
            mp.union(br, mp.cyl(_along(face, ax, c['lug']['t'][0]), _along(face, ax, c['lug']['t'][1]), c['lug']['r']))
        mp.cut(br, mp.cyl(_along(face, ax, -1.0), _along(face, ax, d['L1'] + 1.0), d['D1'] / 2.0 + 0.001))
        fx0, fx1 = sorted(foot['x'])
        for (y, z) in c['bolts']:
            mp.cut(br, mp.cyl((fx0 - 1.0, y, z), (fx1 + 1.0, y, z), HOLE_D / 2.0))
        num = 'P13-017' if end == 'closed' else 'P13-018'
        out.append(('Stop bracket %s %d' % (side, k), br, '1045',
                    desc.get(num, 'stop bracket, %s, steel 1045' % side)))
        for j, (y, z) in enumerate(c['bolts']):
            tag = '%s %d %s' % (side, k, 'ab'[j])
            out.append(('Cap screw ISO 4762 M8x%d stop %s' % (Lb, tag), _cap_screw((fx1, y, z), (-1.0, 0.0, 0.0), Lb),
                        'Steel', 'ISO 4762 M8 x %d - 8.8 - zinc-plated' % Lb))
            out.append(('Washer ISO 7089 8 stop %s' % tag, mp.tube((px0, y, z), (px0 - wt, y, z), wD / 2.0, wd / 2.0),
                        'Steel', desc.get('S-0122', 'ISO 7089 8 - 200 HV')))
            out.append(('Nut ISO 4032 M8 stop %s' % tag, mp.nut(B['bolt'], (px0 - wt, y, z), (-1.0, 0.0, 0.0), m=B['nut_m']),
                        'Steel', desc.get('S-0123', 'ISO 4032 M8 - 8')))
    return out


def _material(word):
    """The Fusion library's material whose name holds `word`, hot rolled first; None if it has none."""
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    hits = []
    for i in range(lib.materials.count):
        m = lib.materials.item(i)
        if word in m.name:
            hits.append(m)
    hits.sort(key=lambda m: (0 if (' HR' in m.name or 'Hot' in m.name) else 1, m.name))
    return hits[0] if hits else None


def _add_stops(des):
    """After the build: the stops into the frame, the component that holds the end plates, at each end plate."""
    root = des.rootComponent
    plates = [o for o in root.allOccurrences if o.component.name.startswith('End plate')]
    if not plates:
        raise RuntimeError('No end plate in the bag gripper: the stops have nowhere to go.')
    frame = plates[0].assemblyContext
    comp = frame.component if frame else root
    inv = frame.transform2.copy() if frame else adsk.core.Matrix3D.create()
    inv.invert()
    try:
        if des.snapshots.hasPendingSnapshot:             # the parts' places kept before the new features
            des.snapshots.add()
    except Exception:
        pass
    steel45 = _material('1045')
    lib = adsk.core.Application.get().materialLibraries.itemByName('Fusion Material Library')
    try:
        steel = lib.materials.itemByName('Steel')
    except Exception:
        steel = None
    names = []
    for k, turned in ((1, False), (2, True)):
        for name, tb, mat, text in _stop_parts(k):
            if turned:
                _turn(tb)
            _tbm().transform(tb, inv)
            o = mp.new_occ(comp, name)
            body = mp.add_body(o, tb, name)
            m = steel45 if mat == '1045' else steel
            if m:
                body.material = m
            o.component.description = text
            names.append(name)
    return dict(names=names, frame=comp.name, steel=steel45.name if steel45 else 'Steel (no 1045 in the library)')


# ---------- moving each machine with its own script ----------
_NONE = object()


def _context(fn):
    """What a mover may want besides the state: its module's groups and joints, the design."""
    g = getattr(fn, '__globals__', {}) or {}
    last = g.get('_last') if isinstance(g.get('_last'), dict) else {}
    groups = last.get('groups') or last.get('G') or last.get('G_') or g.get('G_')
    if groups is None:
        groups = next((v for k, v in last.items() if 'group' in k.lower()), None)
    joints = g.get('_last_joints') or last.get('joints') or last.get('J')
    des = design()
    return dict(groups=groups, joints=joints, design=des, root=des.rootComponent)


_SKIP = object()


def _ctx_value(p, ctx):
    """The context a parameter names; _NONE if it names a state value; _SKIP if it names context the script does not
    keep but has a default."""
    name = p.name
    for key, test in (('groups', name in ('G_', 'G', 'GR', 'g_', 'groups', 'grps', 'gs', 'occs', 'grp')),
                      ('joints', 'joint' in name.lower() or name in ('J', 'J_', 'JS', 'js', 'jts')),
                      ('design', name in ('des', 'design')), ('root', name in ('root', 'rootComp', 'root_comp'))):
        if not test:
            continue
        if ctx[key] is not None:
            return ctx[key]
        if p.default is not p.empty:
            return _SKIP
        raise RuntimeError('the mover wants the %s (%s) but its script keeps none' % (key, name))
    return _NONE


def _call(fn, value, names):
    """Call the machine's own pose / set_state with one state: the groups or joints it keeps passed where it asks
    for them, the state's values in order (or by name when it names them all)."""
    vals = list(value) if isinstance(value, (tuple, list)) else [value]
    try:
        params = list(inspect.signature(fn).parameters.values())
    except (TypeError, ValueError):
        return fn(*vals)
    ctx = _context(fn)
    pnames = [p.name for p in params]
    if names and len(names) == len(vals) and all(n in pnames for n in names):
        kw = dict(zip(names, vals))
        for p in params:
            if p.name in kw or p.kind in (p.VAR_POSITIONAL, p.VAR_KEYWORD):
                continue
            c = _ctx_value(p, ctx)
            if c is not _NONE and c is not _SKIP:
                kw[p.name] = c
        return fn(**kw)
    args, rest = [], list(vals)
    for p in params:
        if p.kind == p.VAR_POSITIONAL:
            args.extend(rest)
            rest = []
            break
        if p.kind in (p.KEYWORD_ONLY, p.VAR_KEYWORD):
            break
        c = _ctx_value(p, ctx)
        if c is _SKIP:
            break
        if c is not _NONE:
            args.append(c)
        elif rest:
            args.append(rest.pop(0))
        elif p.default is p.empty:
            raise TypeError('%s%s wants more than the state %r' % (fn.__name__, inspect.signature(fn), value))
        else:
            break
    if rest:
        raise TypeError('%s%s takes fewer values than the state %r' % (fn.__name__, inspect.signature(fn), value))
    return fn(*args)


def _mover(fn, machine):
    def move(value):
        _call(fn, value, VALUE_NAMES.get(machine))
        adsk.doEvents()
    return move, '%s.%s' % (getattr(fn, '__module__', '?'), fn.__name__)


def _mover_of(mods, machine):
    for m in mods:
        for nm in ('set_state', 'set_pose', 'pose'):
            f = getattr(m, nm, None) if m is not None else None
            if callable(f):
                return _mover(f, machine)
    raise RuntimeError('The %s\'s script has no set_state, set_pose or pose to move it.' % LABEL[machine])


def _all_joints(des):
    out, seen = [], set()
    for comp in des.allComponents:
        for coll in (comp.asBuiltJoints, comp.joints):
            for j in coll:
                try:
                    key = j.entityToken
                except Exception:
                    key = id(j)
                if key not in seen:
                    seen.add(key)
                    out.append(j)
    return out


def _occ_names(j):
    out = []
    for a in ('occurrenceOne', 'occurrenceTwo'):
        try:
            o = getattr(j, a)
            if o:
                out.append(o.component.name)
        except Exception:
            pass
    return out


def _axle_mover(des):
    """The rear axle's two drives among its own as-built joints: the pinion's (its shaft named Pinion shaft) and the
    differential's spider in the case."""
    rev = [j for j in _all_joints(des) if j.jointMotion.jointType == adsk.fusion.JointTypes.RevoluteJointType]
    text = lambda j: (j.name + ' ' + ' '.join(_occ_names(j))).lower()
    spider = sorted([j for j in rev if 'spider' in text(j)], key=lambda j: j.name)
    if not spider:
        spider = sorted([j for j in rev if 'differential pinion' in text(j) or 'planet' in text(j)], key=lambda j: j.name)
    rest = [j for j in rev if j not in spider]
    drive = [j for j in rest if any(n.startswith('Pinion shaft') for n in _occ_names(j))] or \
            [j for j in rest if 'pinion' in j.name.lower()]
    if not drive or not spider:
        raise RuntimeError('The rear axle\'s two drives not found among its revolute joints: %s'
                           % '; '.join('%s (%s)' % (j.name, ', '.join(_occ_names(j))) for j in rev))
    dj, sj = drive[0], spider[0]

    def move(value):
        a, s = value
        dj.jointMotion.rotationValue = math.radians(a)          # the outer drive first, then the inner
        sj.jointMotion.rotationValue = math.radians(s)
        adsk.doEvents()
    return move, 'the joints %s and %s' % (dj.name, sj.name)


# ---------- building each machine ----------
def _build_bag(P, stops):
    made = dict(levers=[], plates=[], module=None)
    plain = P.patch_bag

    def patch(g, weld):
        g = plain(g, weld)
        made['module'] = g
        return _wrap_bag(g, made) if stops else g
    P.patch_bag = patch
    try:
        msg = P.build('bag')
    finally:
        P.patch_bag = plain
    if made['module'] is None:
        raise RuntimeError('PalletizerDetail built the bag gripper without its patch of BagGripper.')
    extra = None
    if stops:
        lev = [x for x in made['levers']]
        if len(lev) != 2 or sorted(x['end'] for x in lev) != [-1, 1] or len(made['plates']) != 2:
            raise RuntimeError('The strikers and the drilling did not find two levers and two end plates: levers %s, '
                               'plates %s' % ([(x['name'], x['end']) for x in lev], [x['name'] for x in made['plates']]))
        extra = _add_stops(design())
        extra.update(levers=lev, plates=made['plates'])
    move, how = _mover_of([made['module'], P._last.get('mod')], 'bag' if stops else 'bag0')
    return msg, move, how, extra


def _build(machine):
    if machine == 'wrist':
        W = _load('WristDetail')
        return (W.build(),) + _mover(W.pose, machine) + (None,)
    if machine == 'flap':
        F = _load('FlapDetail')
        return (F.build(),) + _mover(F.pose, machine) + (None,)
    if machine == 'axle':
        A = _load('AxleDetail')
        msg = A.build()
        return (msg,) + _axle_mover(design()) + (None,)
    if machine in ('robot', 'vacuum', 'unit'):
        P = _load('PalletizerDetail')
        msg = P.build(machine)
        return (msg,) + _mover_of([P._last.get('mod')], machine) + (None,)
    if machine in ('bag0', 'bag'):
        return _build_bag(_load('PalletizerDetail'), machine == 'bag')
    if machine == 'clamp':
        C = _load('ClampGripper')
        return (C.build(),) + _mover(C.set_state, machine) + (None,)
    if machine == 'changer':
        T = _load('ToolChanger')
        return (T.build(),) + _mover(T.set_state, machine) + (None,)
    raise ValueError('No machine called %s.' % machine)


# ---------- the parts, their groups and their gaps ----------
def _left_out(o, starts):
    p = o
    while p:
        if p.component.name.startswith(starts):
            return True
        p = p.assemblyContext
    return False


def _parts(des, machine):
    starts = EXCLUDE.get(machine, ())
    out, out_names = [], []
    for o in des.rootComponent.allOccurrences:
        if o.bRepBodies.count == 0:
            continue
        if starts and _left_out(o, starts):
            out_names.append(o.component.name)
            continue
        out.append(o)
    return out, out_names


def _matrix(o):
    t = o.transform2
    return [[t.getCell(i, j) for j in range(4)] for i in range(3)]


def _motion(T0, T1):
    """The rigid motion from T0 to T1, T1 T0^-1: its rotation (9 numbers) and its translation in mm."""
    R0 = [r[:3] for r in T0]
    R1 = [r[:3] for r in T1]
    t0 = [r[3] for r in T0]
    t1 = [r[3] for r in T1]
    R = [[sum(R1[i][k] * R0[j][k] for k in range(3)) for j in range(3)] for i in range(3)]
    t = [(t1[i] - sum(R[i][k] * t0[k] for k in range(3))) * 10.0 for i in range(3)]
    return [R[i][j] for i in range(3) for j in range(3)], t


def _alike(a, b):
    for (Ra, ta), (Rb, tb) in zip(a, b):
        if max(abs(x - y) for x, y in zip(Ra, Rb)) > 1e-5 or max(abs(x - y) for x, y in zip(ta, tb)) > 0.01:
            return False
    return True


def _groups(mats):
    """Parts whose transforms change alike against the first state are one rigid group."""
    sigs = [[_motion(m[0], mk) for mk in m[1:]] for m in mats]
    reps, gid = [], []
    for s in sigs:
        for k, r in enumerate(reps):
            if _alike(s, r):
                gid.append(k)
                break
        else:
            reps.append(s)
            gid.append(len(reps) - 1)
    return gid, len(reps)


def _box(o):
    lo, hi = [1e30] * 3, [-1e30] * 3
    for i in range(o.bRepBodies.count):
        bb = o.bRepBodies.item(i).boundingBox
        for a, k in (('x', 0), ('y', 1), ('z', 2)):
            lo[k] = min(lo[k], getattr(bb.minPoint, a) * 10.0)
            hi[k] = max(hi[k], getattr(bb.maxPoint, a) * 10.0)
    return lo + hi


def _box_gap(a, b):
    d = [max(a[k] - b[k + 3], b[k] - a[k + 3], 0.0) for k in range(3)]
    return math.sqrt(sum(x * x for x in d))


_measure = dict(mode='occ')


def _distance(a, b):
    """The least distance between two parts (mm): Fusion's minimum distance, between the occurrences or, if Fusion
    takes no occurrences, between their bodies."""
    mm = adsk.core.Application.get().measureManager
    if _measure['mode'] == 'occ':
        try:
            return mm.measureMinimumDistance(a, b).value * 10.0
        except Exception:
            _measure['mode'] = 'body'
    d = 1e30
    for i in range(a.bRepBodies.count):
        for j in range(b.bRepBodies.count):
            d = min(d, mm.measureMinimumDistance(a.bRepBodies.item(i), b.bRepBodies.item(j)).value * 10.0)
    return d


def _interference(des, parts):
    """Fusion's interference among all the parts, touching faces not counted: [(name, name, mm^3)]."""
    def run(items):
        col = adsk.core.ObjectCollection.create()
        for x in items:
            col.add(x)
        inp = des.createInterferenceInput(col)
        inp.areCoincidentFacesIncluded = False
        return des.analyzeInterference(inp)
    try:
        res = run(parts)
    except Exception:
        res = run([o.bRepBodies.item(i) for o in parts for i in range(o.bRepBodies.count)])
    if not res:
        return []
    return [(r.entityOne.name, r.entityTwo.name, round(r.interferenceBody.volume * 1000.0, 3)) for r in res]


def _four(n):
    return sorted(set(int(round(k * (n - 1) / 3.0)) for k in range(4))) if n > 1 else [0]


def _state_text(s):
    kind, v = s
    if isinstance(v, (tuple, list)):
        v = '(%s)' % ', '.join('%g' % x if isinstance(x, float) else str(x) for x in v)
    elif isinstance(v, float):
        v = '%g' % v
    return '%s %s' % (kind, v)


def _sweep(machine, move, states, t0):
    des = design()
    parts, left = _parts(des, machine)
    n = len(parts)
    names = [o.component.name for o in parts]
    four = _four(len(states))
    # the groups: the machine in four of its states, each part's transform against the first
    mats = [[] for _ in parts]
    for i in four:
        move(states[i][1])
        for k, o in enumerate(parts):
            mats[k].append(_matrix(o))
    gid, ng = _groups(mats)
    if ng < 2:
        raise RuntimeError('The %s did not move: in four of its states every part kept its place against the first.'
                           % LABEL[machine])
    cross = [(a, b) for a in range(n) for b in range(a + 1, n) if gid[a] != gid[b]]
    # before the sweep: the pairs near at the four states, and those near at all four with a steady gap dropped
    near, seen4 = {}, {}
    for i in four:
        move(states[i][1])
        bx = [_box(o) for o in parts]
        for p in cross:
            if _box_gap(bx[p[0]], bx[p[1]]) <= RS.GAP:
                near[p] = near.get(p, 0) + 1
                seen4.setdefault(p, []).append(_distance(parts[p[0]], parts[p[1]]))
        adsk.doEvents()
    steady = set(p for p, c in near.items() if c == len(four) and max(seen4[p]) - min(seen4[p]) < RS.STEADY)
    kept = [p for p in cross if p not in steady]
    # the sweep
    hits, rec = [], {}
    for s in states:
        move(s[1])
        h = _interference(des, parts)
        if h:
            hits.append((s, [list(x) for x in h]))
        bx = [_box(o) for o in parts]
        for p in kept:
            if _box_gap(bx[p[0]], bx[p[1]]) <= RS.GAP:
                d = _distance(parts[p[0]], parts[p[1]])
                r = rec.get(p)
                if r is None:
                    rec[p] = dict(min=d, at=s, max=d, seen=1)
                else:
                    if d < r['min']:
                        r['min'], r['at'] = d, s
                    r['max'] = max(r['max'], d)
                    r['seen'] += 1
        adsk.doEvents()
    move(states[0][1])                                   # back to the first state
    rows = []
    for p, r in rec.items():
        top = r['max'] if r['seen'] == len(states) else max(r['max'], RS.GAP)   # unmeasured: boxes beyond GAP
        if top - r['min'] >= RS.STEADY:
            rows.append(dict(pair=(names[p[0]], names[p[1]]), parts=p, min=r['min'], at=r['at'], max=r['max'],
                             seen=r['seen'], left=r['seen'] < len(states)))
    rows.sort(key=lambda x: x['min'])
    pairs = dict([('all', n * (n - 1) // 2), ('across groups', len(cross)), ('near at the four states', len(near)),
                  ('kept', len(kept)), ('near in the sweep', len(rec)), ('changing', len(rows)),
                  ('contacts', sum(1 for x in rows if x['min'] < ZERO))])
    return dict(states=len(states), parts=n, groups=ng, hits=hits, rows=rows, pairs=pairs, left_out=left,
                four=[states[i] for i in four], seconds=time.time() - t0)


# ---------- the results ----------
def _keep(machine, r):
    """This machine's contacts made in motion, closest approaches and pair counts, in results/range_sweep.json."""
    path = os.path.join(CP.ROOT, 'design_office', 'results', 'range_sweep.json')
    row = lambda x: dict(a=x['pair'][0], b=x['pair'][1], min=round(x['min'], 3), at=x['at'], max=round(x['max'], 3),
                         seen=x['seen'])
    entry = dict(states=r['states'], parts=r['parts'], groups=r['groups'],
                 hits=[[str(s), h] for s, h in r['hits']],
                 contacts=[row(x) for x in r['rows'] if x['min'] < ZERO],
                 closest=[row(x) for x in r['rows'] if x['min'] >= ZERO][:3],
                 pairs=r['pairs'], rows=[row(x) for x in r['rows']], seconds=round(r['seconds'], 1), version=VERSION)
    try:
        data = {}
        if os.path.isfile(path):
            try:
                with open(path, encoding='utf-8') as f:
                    data = json.load(f)
            except ValueError:
                data = {}
        if not isinstance(data, dict):
            data = {}
        data[machine] = entry
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=1)
        return path
    except Exception as e:
        return 'not kept (%s)' % e


def build(machine='wrist'):
    machine = str(machine).strip()
    if machine not in ORDER:
        raise ValueError('No machine called %s: the range sweep knows %s.' % (machine, ', '.join(ORDER)))
    t0 = time.time()
    msg, move, how, extra = _build(machine)
    states = RS.STATES[machine]()
    r = _sweep(machine, move, states, t0)
    first = msg.split('\n')[0]
    con = [x for x in r['rows'] if x['min'] < ZERO]
    close = [x for x in r['rows'] if x['min'] >= ZERO][:3]
    kept_at = _keep(machine, r)
    _last.clear()
    _last.update(r, machine=machine, first=first, moved_by=how, stops=extra, kept=kept_at,
                 contacts=len(con), closest=close)
    lines = ['Range sweep of the %s: %d states, %d parts in %d groups, moved by %s.'
             % (LABEL[machine], r['states'], r['parts'], r['groups'], how),
             'Interference at %d of the %d states%s; %d contacts made in motion%s.'
             % (len(r['hits']), r['states'],
                (' (first at %s: %s)' % (_state_text(r['hits'][0][0]), r['hits'][0][1][:2])) if r['hits'] else '',
                len(con), (': ' + '; '.join('%s / %s at %s' % (x['pair'][0], x['pair'][1], _state_text(x['at']))
                                            for x in con[:6])) if con else ''),
             'Closest approaches: %s.' % ('; '.join('%s / %s %.3f mm at %s' % (x['pair'][0], x['pair'][1], x['min'],
                                                                              _state_text(x['at'])) for x in close)
                                         or 'none within %.0f mm that changes' % RS.GAP),
             'Pairs: %s (GAP %.0f mm, STEADY %.2f mm); %d parts left out.'
             % (', '.join('%s %d' % kv for kv in r['pairs'].items()), RS.GAP, RS.STEADY, len(r['left_out']))]
    if extra:
        lines.append('Stops: strikers on %s; %s drilled %.0f across; %d parts in %s (brackets in %s).'
                     % (', '.join('%s (%s, built at %g deg)' % (x['name'], 'its pin %.1f across kept clear'
                                                                % (2 * x['pin'][2]) if x['pin'] else 'no pin seen',
                                                                x['phi']) for x in extra['levers']),
                        ', '.join(x['name'] for x in extra['plates']), HOLE_D, len(extra['names']), extra['frame'],
                        extra['steel']))
    lines += ['The machine: ' + first, 'Results: %s' % kept_at, '', '[%s]' % VERSION]
    return '\n'.join(lines)


# ---------- the command dialog ----------
class _Build(adsk.core.CustomEventHandler):
    """The build, fired from the dialog's execute: Fusion makes no new document inside a command's events."""
    def notify(self, args):
        ui = adsk.core.Application.get().userInterface
        try:
            ui.messageBox(build(args.additionalInfo), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('RangeSweep failed:\n' + traceback.format_exc(), VERSION)


class _Execute(adsk.core.CommandEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        ui = app.userInterface
        try:
            machine = args.command.commandInputs.itemById('machine').selectedItem.name
            if not app.fireCustomEvent(EVENT, machine):
                ui.messageBox(build(machine), VERSION)
        except ValueError as e:
            ui.messageBox(str(e), VERSION)
        except Exception:
            ui.messageBox('RangeSweep failed:\n' + traceback.format_exc(), VERSION)


class _Created(adsk.core.CommandCreatedEventHandler):
    def notify(self, args):
        cmd = args.command
        dd = cmd.commandInputs.addDropDownCommandInput('machine', 'Machine', adsk.core.DropDownStyles.TextListDropDownStyle)
        for k in ORDER:
            dd.listItems.add(k, k == 'wrist')
        h = _Execute()
        cmd.execute.add(h)
        _handlers.append(h)


def run(_context):
    app = adsk.core.Application.get()
    ui = app.userInterface
    try:
        try:
            app.unregisterCustomEvent(EVENT)
        except Exception:
            pass
        ev = app.registerCustomEvent(EVENT)
        hb = _Build()
        ev.add(hb)
        _handlers.append(hb)
        cid = 'mdfcRangeSweep'
        old = ui.commandDefinitions.itemById(cid)
        if old:
            old.deleteMe()
        cdef = ui.commandDefinitions.addButtonDefinition(cid, 'Range sweep', 'Chapter 56: interference and motion '
                                                         'over the whole range')
        h = _Created()
        cdef.commandCreated.add(h)
        _handlers.append(h)
        cdef.execute()
        adsk.autoTerminate(False)
    except Exception:
        ui.messageBox('RangeSweep could not start:\n' + traceback.format_exc())


def stop(_context):
    try:
        adsk.core.Application.get().unregisterCustomEvent(EVENT)
    except Exception:
        pass
